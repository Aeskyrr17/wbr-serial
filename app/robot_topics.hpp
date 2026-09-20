#pragma once

#include "command.hpp"
#include "msg.hpp"
#include "robot_types.hpp"

namespace robot::topics
{
extern msg::channel<command::motion_request> ps2_request;
extern msg::channel<command_frame> command;
extern msg::channel<robot_state> state;
bool init() noexcept;
} // namespace robot::topics
