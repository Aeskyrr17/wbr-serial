#include "aux_control.hpp"

#include "chassis_model.hpp"
#include "robot_debug.hpp"

#include <algorithm>
#include <cmath>

namespace robot::aux
{

output controller::step(const aux_command& command, const robot_state& chassis,
                        const motors::feedback& yaw, const motors::feedback& trigger) noexcept
{
    trigger_pid_.tune(robot_tuning.trigger.kp, robot_tuning.trigger.ki,
                      robot_tuning.trigger.kd);
    trigger_pid_.max_out = robot_tuning.trigger.max_out;
    trigger_pid_.max_iout = robot_tuning.trigger.max_iout;
    output out{};
    if (!command.move)
    {
        yaw_ready_ = false;
        first_init_tried_ = false;
        trigger_pid_.clear();
    }
    else if (!chassis.recovered)
    {
        if (!first_init_tried_)
        {
            out.yaw_current = static_cast<std::int16_t>(
                (yaw.position - chassis::model::yaw_offset_front > 0.0f ? -1 : 1) * 8000);
            if (std::fabs(yaw.position - chassis::model::yaw_offset_front) < 0.2f)
                first_init_tried_ = true;
        }
    }
    else if (!yaw_ready_ && !command.gimbal_only)
    {
        out.yaw_current = static_cast<std::int16_t>(
            (yaw.position - chassis::model::yaw_offset_front > 0.0f ? -1 : 1) * 8000);
        if (std::fabs(yaw.position - chassis::model::yaw_offset_front) < 0.2f)
            yaw_ready_ = true;
    }
    else
    {
        const std::int32_t yaw_current = static_cast<std::int32_t>(command.yaw_current) +
                                         (command.spin ? 5000 : 0);
        out.yaw_current = static_cast<std::int16_t>(
            std::clamp(yaw_current, static_cast<std::int32_t>(-25000),
                       static_cast<std::int32_t>(25000)));
        // PnX M2006 feedback and the B2 target are both reducer-output rad/s.
        trigger_pid_.ref = command.trigger_output_velocity;
        trigger_pid_.fdb = trigger.velocity;
        trigger_pid_.update();
        out.trigger_current = static_cast<std::int16_t>(std::clamp(
            trigger_pid_.result, -10000.0f, 10000.0f));
    }
    out.yaw_ready = yaw_ready_;

    robot_debug.aux.yaw_ref_current = out.yaw_current;
    robot_debug.aux.yaw_fdb_pos = yaw.position;
    robot_debug.aux.yaw_fdb_vel = yaw.velocity;
    robot_debug.aux.trigger_ref_vel = command.trigger_output_velocity;
    robot_debug.aux.trigger_fdb_vel = trigger.velocity;
    robot_debug.aux.trigger_current = out.trigger_current;
    robot_debug.aux.yaw_ready = yaw_ready_;
    return out;
}

} // namespace robot::aux
