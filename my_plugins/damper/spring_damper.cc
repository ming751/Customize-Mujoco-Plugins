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
namespace {

// Helper function to read a double attribute from the plugin configuration
std::optional<mjtNum> ReadOptionalDoubleAttr(const mjModel* m, int instance,
                                             const char* attr) {
  const char* value = mj_getPluginConfig(m, instance, attr);
  if (value == nullptr || value[0] == '\0') {
    return std::nullopt;
  }
  return std::strtod(value, nullptr);
}

// Helper function to read a string attribute from the plugin configuration
std::optional<std::string> ReadStringAttr(const mjModel* m, int instance,
                                          const char* attr) {
  const char* value = mj_getPluginConfig(m, instance, attr);
  if (value == nullptr || value[0] == '\0') {
    return std::nullopt;
  }
  return std::string(value);
}

}  // namespace

// Factory function to create a SpringConfig from the MuJoCo model
std::optional<SpringConfig> SpringConfig::FromModel(const mjModel* m, int instance) {
  SpringConfig config;

  // Read stiffness, damping, and rest_length from XML attributes
  config.stiffness = ReadOptionalDoubleAttr(m, instance, "stiffness").value_or(100.0);
  config.damping = ReadOptionalDoubleAttr(m, instance, "damping").value_or(10.0);
  config.rest_length = ReadOptionalDoubleAttr(m, instance, "restlength").value_or(-1.0);

  // Read body names from XML attributes
  auto body1_name = ReadStringAttr(m, instance, "body1");
  auto body2_name = ReadStringAttr(m, instance, "body2");

  if (!body1_name || !body2_name) {
    mju_warning("Spring plugin requires 'body1' and 'body2' attributes.");
    return std::nullopt;
  }

  // Find body IDs from their names
  config.body1_id = mj_name2id(m, mjOBJ_BODY, body1_name->c_str());
  config.body2_id = mj_name2id(m, mjOBJ_BODY, body2_name->c_str());

  if (config.body1_id < 0 || config.body2_id < 0) {
    mju_warning("Could not find bodies for spring plugin.");
    return std::nullopt;
  }

  return config;
}

// Creates an instance of the Spring plugin.
std::unique_ptr<Spring> Spring::Create(const mjModel* m, int instance) {
  auto config = SpringConfig::FromModel(m, instance);
  if (!config) {
    return nullptr;
  }
  return std::unique_ptr<Spring>(new Spring(*config));
}

// Constructor
Spring::Spring(SpringConfig config) : config_(std::move(config)) {}

// Computes the spring force and applies it to the bodies.
void Spring::Compute(const mjModel* m, mjData* d, int instance) {
  // Get body positions
  mjtNum* pos1 = d->xpos + 3 * config_.body1_id;
  mjtNum* pos2 = d->xpos + 3 * config_.body2_id;

  // Calculate vector and distance between bodies
  mjtNum vec[3];
  mju_sub3(vec, pos2, pos1);
  mjtNum distance = mju_normalize3(vec);

  // Determine rest length if not specified
  double rest_length = config_.rest_length;
  if (rest_length < 0) {
    // If rest_length is negative, use the initial distance as the rest length.
    // This requires storing the initial distance, a more advanced approach.
    // For simplicity, we'll calculate it on the fly, assuming it doesn't change.
    // A better way is to compute and store it in the plugin's state during init.
    mjtNum* init_pos1 = m->body_pos + 3 * config_.body1_id;
    mjtNum* init_pos2 = m->body_pos + 3 * config_.body2_id;
    mjtNum init_vec[3];
    mju_sub3(init_vec, init_pos2, init_pos1);
    rest_length = mju_norm3(init_vec);
  }

  // Calculate spring force (Hooke's Law)
  mjtNum force_magnitude = config_.stiffness * (distance - rest_length);

  // Calculate damping force
  mjtNum vel1[3], vel2[3];
  mju_zero3(vel1);
  mju_zero3(vel2);
  mj_objectVelocity(m, d, mjOBJ_BODY, config_.body1_id, vel1, 0);
  mj_objectVelocity(m, d, mjOBJ_BODY, config_.body2_id, vel2, 0);

  mjtNum vel_rel[3];
  mju_sub3(vel_rel, vel2, vel1);
  mjtNum vel_along_spring = mju_dot3(vel_rel, vec);
  force_magnitude += config_.damping * vel_along_spring;

  // Calculate force vector
  mjtNum force_vec[3];
  mju_scl3(force_vec, vec, force_magnitude);

  // Apply forces to bodies (equal and opposite)
  // Note: xfrc_applied is in global coordinates
  mju_addTo(d->xfrc_applied + 6 * config_.body1_id, force_vec, 3);
  mju_subFrom(d->xfrc_applied + 6 * config_.body2_id, force_vec, 3);
}

// Registers the plugin with MuJoCo.
void Spring::RegisterPlugin() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mujoco.passive.spring";
  plugin.capabilityflags |= mjPLUGIN_PASSIVE;

  const char* attributes[] = {"stiffness", "damping", "restlength",
                              "body1", "body2"};
  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;

  // 必须！返回 0 也行，但指针不能为 NULL
  plugin.nstate = +[](const mjModel*, int){ return 0; };

  plugin.init = +[](const mjModel* m, mjData* d, int instance){
    auto spring = Spring::Create(m, instance);
    if(!spring) return -1;
    d->plugin_data[instance] =
        reinterpret_cast<uintptr_t>(spring.release());
    return 0;
  };
  plugin.destroy = +[](mjData* d, int instance){
    delete reinterpret_cast<Spring*>(d->plugin_data[instance]);
    d->plugin_data[instance] = 0;
  };
  plugin.compute = +[](const mjModel* m, mjData* d,
                       int instance, int){
    reinterpret_cast<Spring*>(d->plugin_data[instance])
        ->Compute(m, d, instance);
  };

  mjp_registerPlugin(&plugin);
}

}  // namespace mujoco::plugin::passive