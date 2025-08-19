#include "ctrl_pdff.h"

#include <cctype>
#include <cstring>
#include <string>
#include <optional>
#include <algorithm>

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

std::unique_ptr<PdFf> PdFf::Create(const mjModel* m, int instance) {
  PdFfConfig cfg;
  cfg.kp = ReadDoubleAttr(m, instance, "kp").value_or(0.0);
  cfg.kd = ReadDoubleAttr(m, instance, "kd").value_or(0.0);
  cfg.target_actuator_name = ReadStringAttr(m, instance, "target");

  // 收集绑定到本 instance 的 actuators
  int id_qref = -1, id_qdref = -1, id_tau = -1, id_target = -1;

  for (int i = 0; i < m->nu; ++i) {
    if (m->actuator_plugin[i] != instance) continue;
    const std::string name = ActName(m, i);
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

  return std::unique_ptr<PdFf>(
      new PdFf(cfg, id_qref, id_qdref, id_tau, id_target));
}

PdFf::PdFf(PdFfConfig config,
           int id_qref, int id_qdref, int id_tau, int id_target)
  : config_(std::move(config)),
    id_qref_(id_qref), id_qdref_(id_qdref), id_tau_(id_tau),
    id_target_(id_target) {}

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
}

void PdFf::RegisterPlugin() {
  mjpPlugin p;
  mjp_defaultPlugin(&p);

  p.name = "mujoco.ctrl.pdff";
  p.capabilityflags |= mjPLUGIN_ACTUATOR;

  static const char* kAttrs[] = {"kp","kd","target"};
  p.nattribute = 3;
  p.attributes = kAttrs;

  p.nstate = +[](const mjModel*, int){ return 0; };

  p.init = +[](const mjModel* m, mjData* d, int instance){
    auto obj = PdFf::Create(m, instance);
    if (!obj) return -1;
    d->plugin_data[instance] = reinterpret_cast<uintptr_t>(obj.release());
    return 0;
  };

  // 无内部状态，提供空 reset 以满足接口
  p.reset = +[](const mjModel*, mjtNum*, void*, int){};

  p.destroy = +[](mjData* d, int instance){
    delete reinterpret_cast<PdFf*>(d->plugin_data[instance]);
    d->plugin_data[instance] = 0;
  };

  p.compute = +[](const mjModel* m, mjData* d, int instance, int){
    auto* obj = reinterpret_cast<PdFf*>(d->plugin_data[instance]);
    obj->Compute(m, d, instance);
  };

  mjp_registerPlugin(&p);
}

}  // namespace mujoco::plugin::ctrl