#ifndef MUJOCO_PLUGIN_INSPECTOR_H_
#define MUJOCO_PLUGIN_INSPECTOR_H_

#include <memory>
#include <optional>
#include <string>
#include <mujoco/mujoco.h>

namespace mujoco::plugin::inspector {

// 配置：mode=print(默认)/file，file=输出路径，rate=Hz（默认 10Hz）
struct InspectorConfig {
  std::optional<std::string> mode;
  std::optional<std::string> file;
  double rate_hz = 10.0;
};

class Inspector {
 public:
  static std::unique_ptr<Inspector> Create(const mjModel* m, int instance);

  void Compute(const mjModel* m, mjData* d, int instance);

  static void RegisterPlugin();

 private:
  explicit Inspector(InspectorConfig config, void* file_handle);

  void EmitHeaderOnce(const mjModel* m);
  void EmitJoints(const mjModel* m, const mjData* d);
  void EmitSensors(const mjModel* m, const mjData* d);
  void EmitLine(const std::string& line);

  InspectorConfig config_;
  void* file_ = nullptr;     // FILE*，仅当 mode==file 时使用
  bool header_emitted_ = false;
  double last_emit_time_ = -1.0;
};

}  // namespace mujoco::plugin::inspector

#endif  // MUJOCO_PLUGIN_INSPECTOR_H_

