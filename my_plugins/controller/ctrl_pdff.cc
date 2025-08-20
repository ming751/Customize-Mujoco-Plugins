#include "ctrl_pdff.h"

#include <cctype>
#include <cstring>
#include <string>
#include <optional>
#include <algorithm>
#include <iostream>


namespace mujoco::plugin::ctrl {
namespace {

// 读取 instance 的字符串属性
std::optional<std::string> ReadStringAttr(const mjModel* m, int instance,
                                          const char* key) {
  const char* v = mj_getPluginConfig(m, instance, key);
  if (!v || !v[0]) return std::nullopt;
  return std::string(v);
}

// 读取 instance 的 double 属性
std::optional<double> ReadDoubleAttr(const mjModel* m, int instance,
                                     const char* key) {
  const char* v = mj_getPluginConfig(m, instance, key);
  if (!v || !v[0]) return std::nullopt;
  return std::strtod(v, nullptr);
}

// 小写后缀判断
bool EndsWithLower(const std::string& name, const std::string& suffix) {
  if (suffix.size() > name.size()) return false;
  auto it = name.end() - suffix.size();
  for (size_t i = 0; i < suffix.size(); ++i) {
    char a = std::tolower(*(it + i));
    char b = std::tolower(suffix[i]);
    if (a != b) return false;
  }
  return true;
}

// 获取 actuator 名字
std::string ActName(const mjModel* m, int act_id) {
  const char* n = mj_id2name(m, mjOBJ_ACTUATOR, act_id);
  return n ? std::string(n) : std::string();
}

}  // namespace

// 工厂：从模型与实例配置创建 PdFf 控制对象
// 约定：同一个插件实例 instance 下面必须有 3 个 actuator，名字后缀分别为：
//   - *_qref 或 :qref   （期望位置）
//   - *_qdref 或 :qdref （期望速度）
//   - *_tau  或 :tau    （前馈力矩输入；默认也作为输出力矩目标通道）
// 可选 config：
//   - kp、kd：PD 参数
//   - target：显式指定输出写入的 actuator 名（否则优先 *_tau，其次选第一个同实例 actuator）
std::unique_ptr<PdFf> PdFf::Create(const mjModel* m, int instance) {
  PdFfConfig cfg;
  cfg.kp = ReadDoubleAttr(m, instance, "kp").value_or(0.0);
  cfg.kd = ReadDoubleAttr(m, instance, "kd").value_or(0.0);
  cfg.target_actuator_name = ReadStringAttr(m, instance, "target");

  // 收集绑定到本 instance 的 actuators
  int id_qref = -1, id_qdref = -1, id_tau = -1, id_target = -1;

  for (int i = 0; i < m->nu; ++i) {
    if (m->actuator_plugin[i] != instance) continue;
    const std::string name = ActName(m, i);  //通过actuator_plugin[i]获取actuator_name
    if (EndsWithLower(name, "_qref") || EndsWithLower(name, ":qref")) {
      id_qref = i;
    } else if (EndsWithLower(name, "_qdref") || EndsWithLower(name, ":qdref")) {
      id_qdref = i;
    } else if (EndsWithLower(name, "_tau") || EndsWithLower(name, ":tau")) {
      id_tau = i;
    }
  }

  // 选择输出目标
  if (cfg.target_actuator_name.has_value()) {
    for (int i = 0; i < m->nu; ++i) {
      if (m->actuator_plugin[i] != instance) continue;
      if (ActName(m, i) == *cfg.target_actuator_name) {
        id_target = i;
        break;
      }
    }
  } else if (id_tau >= 0) {
    id_target = id_tau;
  } else {
    // 若没有显式 tau 通道，就选第一个绑定的 actuator 作为目标
    for (int i = 0; i < m->nu; ++i) {
      if (m->actuator_plugin[i] == instance) {
        id_target = i;
        break;
      }
    }
  }

  if (id_target < 0) {
    mju_warning("pdff: target actuator not found.");
    return nullptr;
  }
  if (id_qref < 0 || id_qdref < 0 || id_tau < 0) {
    mju_warning("pdff: need three actuators with suffixes {_qref,_qdref,_tau}.");
    return nullptr;
  }

  int joint_id = -1;
  int dof_adr = -1;
  int dof_num = 0;

  int trntype = m->actuator_trntype[id_target];
  if (trntype == mjTRN_JOINT || trntype == mjTRN_JOINTINPARENT) {
    joint_id = m->actuator_trnid[2*id_target + 0];  // 绑定的关节 id
    int jtype = m->jnt_type[joint_id];
    dof_adr = m->jnt_dofadr[joint_id];
    dof_num = (jtype == mjJNT_HINGE || jtype == mjJNT_SLIDE) ? 1
            : (jtype == mjJNT_BALL ? 3
            : (jtype == mjJNT_FREE ? 6 : 0));
    // 现在可用 d->qfrc_actuator[dof_adr .. dof_adr + dof_num - 1]
}

  return std::unique_ptr<PdFf>(
      new PdFf(cfg, id_qref, id_qdref, id_tau, id_target));
}

PdFf::PdFf(PdFfConfig config,
           int id_qref, int id_qdref, int id_tau, int id_target)
  : config_(std::move(config)),
    id_qref_(id_qref), id_qdref_(id_qdref), id_tau_(id_tau),
    id_target_(id_target) {}

// compute 回调：在仿真步（插件阶段）被 MuJoCo 调用
// 数据流：
//   输入：d->ctrl[三路] = {期望位置, 期望速度, 前馈力矩}
//   反馈：d->actuator_length/velocity[目标通道] = 执行器空间的位姿/速度（由引擎计算）
//   输出：d->actuator_force[目标通道] += kp*(err) + kd*(err_dot) + tau_ff

void PdFf::Compute(const mjModel* m, mjData* d, int instance) {
  // 取三路输入
  const mjtNum q_ref  = d->ctrl[id_qref_];
  const mjtNum qd_ref = d->ctrl[id_qdref_];
  const mjtNum tau_ff = d->ctrl[id_tau_];

  // 目标执行器的反馈（位姿/速度在“执行器空间”的量）
  const mjtNum q_meas  = d->actuator_length[id_target_];
  const mjtNum qd_meas = d->actuator_velocity[id_target_];

  // PD + 前馈
  const mjtNum err     = q_ref  - q_meas;
  const mjtNum err_dot = qd_ref - qd_meas;
  const mjtNum tau     = config_.kp * err + config_.kd * err_dot + tau_ff;

  // 写入目标执行器的力（叠加）
  d->actuator_force[id_target_] += tau;

  // 其他两个输入执行器避免产生额外力
  d->actuator_force[id_qref_] = 0;
  d->actuator_force[id_qdref_] = 0;

  std::cout << "id_target_: " << id_target_ << std::endl;
  std::cout << "tau_ff: " << tau_ff << std::endl;
  std::cout << "d->actuator_force[id_target_]: " << d->actuator_force[id_target_] << std::endl;

  // int j = mj_name2id(m, mjOBJ_JOINT, "hip");
  // int dof = m->jnt_dofadr[j];
  std::cout << "qfrc_actuator at hip = " << d->qfrc_actuator[dof_adr] << std::endl;
}

// 注册插件：向 MuJoCo 声明本插件的能力、属性与回调
void PdFf::RegisterPlugin() {
  mjpPlugin p;
  mjp_defaultPlugin(&p); // 清零并填入默认值，防止未初始化字段

  // 唯一名称与能力：ACTUATOR 表示它在执行器通道上工作
  p.name = "mujoco.ctrl.pdff";
  p.capabilityflags |= mjPLUGIN_ACTUATOR;

  // 可在 XML <config> 中使用的属性列表
  static const char* kAttrs[] = {"kp","kd","target"};
  p.nattribute = 3;
  p.attributes = kAttrs;

  // 无内部“状态向量”（与 d->plugin_state 相关）
  p.nstate = +[](const mjModel*, int){ return 0; };

  // init：为每个实例创建 PdFf 对象，并把指针放到 d->plugin_data[instance]
  p.init = +[](const mjModel* m, mjData* d, int instance){
    auto obj = PdFf::Create(m, instance);
    if (!obj) return -1;
    d->plugin_data[instance] = reinterpret_cast<uintptr_t>(obj.release());
    return 0;
  };

  // 无内部状态，提供空 reset 以满足接口
  p.reset = +[](const mjModel*, mjtNum*, void*, int){};

  // destroy：释放在 init 阶段分配的对象内存
  p.destroy = +[](mjData* d, int instance){
    delete reinterpret_cast<PdFf*>(d->plugin_data[instance]);
    d->plugin_data[instance] = 0;
  };

  // compute：仿真步中被调用的主回调；将插件数据指针还原为 PdFf* 并调用成员函数
  p.compute = +[](const mjModel* m, mjData* d, int instance, int){
    auto* obj = reinterpret_cast<PdFf*>(d->plugin_data[instance]);
    obj->Compute(m, d, instance);
  };

  //完成注册插件
  mjp_registerPlugin(&p);
}

}  // namespace mujoco::plugin::ctrl