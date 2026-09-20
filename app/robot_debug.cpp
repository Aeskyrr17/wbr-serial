#include "robot_debug.hpp"

namespace robot
{
robot_debug_t robot_debug __asm__("robot_debug"){};
robot_tuning_t robot_tuning __asm__("robot_tuning"){};
} // namespace robot
