#include "command.hpp"
#include "chassis_model.hpp"
#include "robot_debug.hpp"

#include <algorithm>
#include <cmath>

namespace robot::command
{
command_frame mapper::step(const motion_request& request, const robot_state& state,
                           std::uint32_t now) noexcept
{
    command_frame out{};
    out.sequence = ++sequence_;
    out.received_tick = request.received_tick;
    const bool online = request.online && now - request.received_tick <= mapping::offline_ticks;
    auto& cmd = out.chassis;
    cmd.move = online && request.move;
    if (!cmd.move)
        len_ref_ = chassis::model::Lmin;
    else if (state.state_executed == chassis_state::normal)
        len_ref_ += request.len_rate * mapping::dt;
    len_ref_ = std::clamp(len_ref_, chassis::model::Lmin, chassis::model::Lmax);
    cmd.len = len_ref_;

    if (!cmd.move)
    {
        v_ramp_.set_value(0.0f);
        yaw_ramp_.set_value(0.0f);
        hold_x_ = hold_yaw_ = false;
    }
    if (cmd.move && state.v_cmd_enabled)
    {
        cmd.v = v_ramp_.update(std::clamp(request.v, -mapping::max_v, mapping::max_v));
        cmd.dyaw = yaw_ramp_.update(std::clamp(request.dyaw, -mapping::max_dyaw, mapping::max_dyaw));
    }
    else
    {
        v_ramp_.set_value(0.0f);
        yaw_ramp_.set_value(0.0f);
    }

    if (!cmd.move)
    {
        cmd.x = state.x;
        cmd.yaw = state.yaw;
    }
    else
    {
        if (std::fabs(cmd.v) < 0.005f && std::fabs(state.v) < 0.3f)
        {
            if (!hold_x_) 
            { 
                held_x_ = state.x; 
                hold_x_ = true; 
            }
            cmd.x = held_x_;
        }
        else
        {
            hold_x_ = false;
            cmd.x = state.x + cmd.v * mapping::dt;
        }
        if (std::fabs(cmd.dyaw) < 0.002f)
        {
            if (!hold_yaw_) { held_yaw_ = state.yaw; hold_yaw_ = true; }
            cmd.yaw = held_yaw_;
        }
        else
        {
            hold_yaw_ = false;
            cmd.yaw = state.yaw + cmd.dyaw * mapping::dt;
        }
    }
    robot_debug.command.request_move = request.move;
    robot_debug.command.command = cmd;
    robot_debug.command.source = source_;
    robot_debug.command.link_online = online;
    robot_debug.command.link_age_ticks = now - request.received_tick;
    return out;
}
}
