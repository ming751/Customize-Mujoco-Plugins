#include <mujoco/mjplugin.h>
#include "inspector.h"

namespace mujoco::plugin::inspector {
mjPLUGIN_LIB_INIT { Inspector::RegisterPlugin(); }
}  // namespace mujoco::plugin::inspector

