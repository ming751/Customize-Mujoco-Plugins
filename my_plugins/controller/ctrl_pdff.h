#ifndef MUJOCO_PLUGIN_CTRL_PDFF_H_
#define MUJOCO_PLUGIN_CTRL_PDFF_H_

#include <memory>
#include <optional>
#include <vector>
#include <string>
#include <mujoco/mujoco.h>

namespace mujoco::plugin::ctrl {

struct PdFfConfig {
  double kp = 0.0;
  double kd = 0.0;
  // 可选：若不指定，则优先用名字后缀匹配 "_tau"
  std::optional<std::string> target_actuator_name;
};

class PdFf {
 public:
  static std::unique_ptr<PdFf> Create(const mjModel* m, int instance);

  void Compute(const mjModel* m, mjData* d, int instance);

  static void RegisterPlugin();

 private:
  explicit PdFf(PdFfConfig config,
                int id_qref, int id_qdref, int id_tau, int id_target);

  PdFfConfig config_;
  int id_qref_   = -1;
  int id_qdref_  = -1;
  int id_tau_    = -1;
  int id_target_ = -1;  // 输出力写入的 actuator
};

}  // namespace mujoco::plugin::ctrl

#endif  // MUJOCO_PLUGIN_CTRL_PDFF_H_