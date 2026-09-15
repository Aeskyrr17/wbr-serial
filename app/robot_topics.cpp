#include "robot_topics.hpp"

namespace robot::topics
{
msg::channel<command_frame> command{};
msg::channel<robot_state> state{};
msg::channel<command::referee_summary> referee{};

bool init() noexcept
{
    return msg::init(command) == types::status::ok &&
           msg::init(state) == types::status::ok &&
           msg::init(referee) == types::status::ok;
}
} // namespace robot::topics
