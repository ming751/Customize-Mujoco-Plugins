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

#ifndef MUJOCO_PLUGIN_ACTUATOR_SPRING_H_
#define MUJOCO_PLUGIN_ACTUATOR_SPRING_H_

#include <memory>
#include <optional>
#include <vector>

#include <mujoco/mujoco.h>

namespace mujoco::plugin::passive {

// Configuration for the spring plugin
struct SpringConfig {
  double stiffness = 100.0;     // Spring stiffness (k)
  double damping = 10.0;        // Damping coefficient (d)
  double rest_length = -1.0;    // Rest length of the spring. If negative, it's computed from initial positions.
  int body1_id = -1;            // ID of the first body
  int body2_id = -1;            // ID of the second body

  // Factory function to create a SpringConfig from the MuJoCo model
  static std::optional<SpringConfig> FromModel(const mjModel* m, int instance);
};

// The Spring plugin class
class Spring {
 public:
  // Creates an instance of the Spring plugin.
  static std::unique_ptr<Spring> Create(const mjModel* m, int instance);

  // Computes the spring force and applies it to the bodies.
  void Compute(const mjModel* m, mjData* d, int instance);

  // Registers the plugin with MuJoCo.
  static void RegisterPlugin();

 private:
  // Constructor is private to enforce creation via the factory method.
  Spring(SpringConfig config);

  SpringConfig config_;
};

}  // namespace mujoco::plugin::passive

#endif  // MUJOCO_PLUGIN_ACTUATOR_SPRING_H_
