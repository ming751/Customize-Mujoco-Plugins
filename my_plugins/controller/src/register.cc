#include <mujoco/mjplugin.h>
#include "ctrl_pdff.h"

namespace mujoco::plugin::ctrl {
mjPLUGIN_LIB_INIT { PdFf::RegisterPlugin(); }
}  // namespace mujoco::plugin::ctrl