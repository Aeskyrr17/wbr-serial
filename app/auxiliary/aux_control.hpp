#pragma once

#include "djimotors.hpp"
#include "pid.hpp"
#include "robot_types.hpp"

namespace robot::aux
{

struct output
{
    std::int16_t yaw_current = 0;
    std::int16_t trigger_current = 0;
    bool yaw_ready = false;
};

class controller
{
public:
    output step(const aux_command& command, const robot_state& chassis,
                const motors::feedback& yaw, const motors::feedback& trigger) noexcept;

private:
    control::pid trigger_pid_{100.0f, 0.0f, 0.0f, 10000.0f, 0.0f};
    bool yaw_ready_ = false;
    bool first_init_tried_ = false;
};

} // namespace robot::aux
