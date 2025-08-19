// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -----------------------------------------------------------------------------
//
//  文件：spring_damper.cc
//  作者：自定义
//  说明：
//    这是一个 MuJoCo PASSIVE 插件示例，实现了最经典的“弹簧-阻尼器”模型。
//    插件通过在两个刚体之间施加方向相反且大小相等的力来模拟弹簧拉力，
//    同时叠加速度相关的阻尼力。可在 XML 中通过 <plugin> 标签进行参数化。
//
//    支持的 XML 属性：
//      • stiffness   : 弹簧刚度系数 k (单位 N/m)
//      • damping     : 阻尼系数 d (单位 N·s/m)
//      • restlength  : 弹簧自然长度 (负值表示使用模型初始距离)
//      • body1 / body2: 参与作用的两个刚体名称
//
// -----------------------------------------------------------------------------

#include "spring_damper.h"

#include <cstdint>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>

namespace mujoco::plugin::passive {
namespace {  // 匿名命名空间，限制辅助函数作用域

// ----------------------------- 工具函数 ------------------------------------
// 说明：以下两个 Read* 函数封装了读取 XML 属性并转换为所需类型的流程，
//       返回 std::optional 以便统一判断属性是否存在。

// 从插件配置读取 double 类型，可缺省
std::optional<mjtNum> ReadOptionalDoubleAttr(const mjModel* m, int instance,
                                             const char* attr) {
  const char* value = mj_getPluginConfig(m, instance, attr);
  if (value == nullptr || value[0] == '\0') {
    return std::nullopt;  // 未提供该属性
  }
  return std::strtod(value, nullptr);
}

// 从插件配置读取 string 类型，可缺省
std::optional<std::string> ReadStringAttr(const mjModel* m, int instance,
                                          const char* attr) {
  const char* value = mj_getPluginConfig(m, instance, attr);
  if (value == nullptr || value[0] == '\0') {
    return std::nullopt;  // 未提供
  }
  return std::string(value);
}

}  // namespace

// --------------------------- SpringConfig -----------------------------------
// 从 MuJoCo 模型 (mjModel) 解析 XML 中的插件实例配置，若失败返回空值
std::optional<SpringConfig> SpringConfig::FromModel(const mjModel* m,
                                                   int instance) {
  SpringConfig config;

  // 1) 读取数值参数
  config.stiffness   = ReadOptionalDoubleAttr(m, instance, "stiffness").value_or(100.0);
  config.damping     = ReadOptionalDoubleAttr(m, instance, "damping").value_or(10.0);
  config.rest_length = ReadOptionalDoubleAttr(m, instance, "restlength").value_or(-1.0);

  // 2) 读取两刚体名称
  auto body1_name = ReadStringAttr(m, instance, "body1");
  auto body2_name = ReadStringAttr(m, instance, "body2");

  if (!body1_name || !body2_name) {
    mju_warning("Spring plugin requires 'body1' and 'body2' attributes.");
    return std::nullopt;
  }

  // 3) 用名称查询刚体 ID
  config.body1_id = mj_name2id(m, mjOBJ_BODY, body1_name->c_str());
  config.body2_id = mj_name2id(m, mjOBJ_BODY, body2_name->c_str());

  if (config.body1_id < 0 || config.body2_id < 0) {
    mju_warning("Could not find bodies for spring plugin.");
    return std::nullopt;
  }

  return config;
}

// ------------------------------ Spring --------------------------------------
// 工厂：创建插件实例（返回 unique_ptr 由 MuJoCo 托管）
std::unique_ptr<Spring> Spring::Create(const mjModel* m, int instance) {
  auto config = SpringConfig::FromModel(m, instance);
  if (!config) {
    return nullptr;  // 配置解析失败
  }
  return std::unique_ptr<Spring>(new Spring(*config));
}

// 构造函数 – 仅保存配置
Spring::Spring(SpringConfig config) : config_(std::move(config)) {}

// Compute 回调 – 在每个仿真子步调用 (mj_step 等函数内部)
// 参数：
//   m        – 只读模型数据
//   d        – 时变数据，可写
//   instance – 当前插件实例索引
void Spring::Compute(const mjModel* m, mjData* d, int /*instance*/) {
  // ---------- 1) 获取两刚体质心位置（世界坐标系） ----------
  mjtNum* pos1 = d->xpos + 3 * config_.body1_id;  // x,y,z 地址
  mjtNum* pos2 = d->xpos + 3 * config_.body2_id;

  // ---------- 2) 计算两点之间的向量与距离 ----------
  mjtNum vec[3];
  mju_sub3(vec, pos2, pos1);           // vec = pos2 - pos1
  mjtNum distance = mju_normalize3(vec);  // 单位化 vec，同时返回长度

  // ---------- 3) 确定自然长度 ----------
  double rest_length = config_.rest_length;
  if (rest_length < 0) {
    // 若未显式设置 restlength，则将模型初始距离视为自然长度。
    const mjtNum* init_pos1 = m->body_pos + 3 * config_.body1_id;
    const mjtNum* init_pos2 = m->body_pos + 3 * config_.body2_id;
    mjtNum init_vec[3];
    mju_sub3(init_vec, init_pos2, init_pos1);
    rest_length = mju_norm3(init_vec);
  }

  // ---------- 4) 计算弹簧拉力 (Hooke) + 阻尼 ----------
  // Hooke: F = k * (Δx)
  mjtNum force_magnitude = config_.stiffness * (distance - rest_length);

  // 阻尼:  F_damp = d * (v_rel · dir)
  mjtNum vel1[3], vel2[3];  // 两刚体质心线速度
  mju_zero3(vel1);
  mju_zero3(vel2);
  mj_objectVelocity(m, d, mjOBJ_BODY, config_.body1_id, vel1, 0 /*local*/);
  mj_objectVelocity(m, d, mjOBJ_BODY, config_.body2_id, vel2, 0);

  mjtNum vel_rel[3];
  mju_sub3(vel_rel, vel2, vel1);                // 相对速度
  mjtNum vel_along_spring = mju_dot3(vel_rel, vec);  // 投影到弹簧方向
  force_magnitude += config_.damping * vel_along_spring;

  // ---------- 5) 合成力向量并施加 ----------
  mjtNum force_vec[3];
  mju_scl3(force_vec, vec, force_magnitude);  // F = dir * |F|

  // xfrc_applied: [fx fy fz  mx my mz] – 施加在刚体质心上的外力/外矩
  mju_addTo(d->xfrc_applied + 6 * config_.body1_id, force_vec, 3);  // 加到 body1
  mju_subFrom(d->xfrc_applied + 6 * config_.body2_id, force_vec, 3);  // 反向加到 body2
}

// --------------------------- 插件注册 ---------------------------------------
void Spring::RegisterPlugin() {
  mjpPlugin plugin;          // 核心描述结构体
  mjp_defaultPlugin(&plugin);  // 填入默认值，避免忘记初始化字段

  // 基本信息
  plugin.name = "mujoco.passive.spring";  // 唯一标识
  plugin.capabilityflags |= mjPLUGIN_PASSIVE;  // 指示 compute 回调在仿真循环被调用

  // 声明可在 XML 使用的属性列表
  static const char* attributes[] = {"stiffness", "damping", "restlength",
                                     "body1", "body2"};
  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;

  // --- 以下设置各回调 ---
  // nstate：插件需要的内部状态大小（字节），此处用 lambda 返回 0
  plugin.nstate = +[](const mjModel*, int) { return 0; };

  // init：创建并保存插件实例
  plugin.init = +[](const mjModel* m, mjData* d, int instance) {
    auto spring = Spring::Create(m, instance);
    if (!spring) return -1;  // 创建失败让 MuJoCo 停止加载本实例
    d->plugin_data[instance] = reinterpret_cast<uintptr_t>(spring.release());
    return 0;
  };

  // destroy：释放内存
  plugin.destroy = +[](mjData* d, int instance) {
    delete reinterpret_cast<Spring*>(d->plugin_data[instance]);
    d->plugin_data[instance] = 0;
  };

  // compute：每步调用
  plugin.compute = +[](const mjModel* m, mjData* d, int instance, int /*stage*/) {
    reinterpret_cast<Spring*>(d->plugin_data[instance])->Compute(m, d, instance);
  };

  // 最终注册
  mjp_registerPlugin(&plugin);
}

}  // namespace mujoco::plugin::passive
