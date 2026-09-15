#pragma once

#include "command.hpp"
#include "msg.hpp"
#include "robot_types.hpp"

namespace robot::topics
{
extern msg::channel<command_frame> command;
extern msg::channel<robot_state> state;
extern msg::channel<command::referee_summary> referee;
bool init() noexcept;
} // namespace robot::topics
