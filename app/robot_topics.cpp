#include "robot_topics.hpp"

namespace robot::topics
{
msg::channel<command::motion_request> ps2_request{};
msg::channel<command_frame> command{};
msg::channel<robot_state> state{};

bool init() noexcept
{
    return msg::init(ps2_request) == types::status::ok &&
           msg::init(command) == types::status::ok &&
           msg::init(state) == types::status::ok;
}
} // namespace robot::topics
