#include "cmd.hpp"
#include "model/chassis_model.hpp"
#include "robot_debug.hpp"

#include <algorithm>
#include <cmath>

namespace chassis::cmd
{
void mapper::reset_position(float x) noexcept
{
    held_x_ = x;
    hold_x_ = true;
    stopping_x_ = false;
    stop_direction_ = 0;
    stop_confirm_count_ = 0;
}

void mapper::update_position(chassis_cmd& cmd, const chassis_state& state) noexcept
{
    if (std::fabs(cmd.v) >= mapping::stationary_vel)
    {
        stopping_x_ = false;
        hold_x_ = false;
        stop_confirm_count_ = 0;
        cmd.x = state.x + cmd.v * mapping::dt;
        return;
    }

    cmd.v = 0.0f;
    if (!stopping_x_ && !hold_x_)
    {
        stopping_x_ = true;
        held_x_ = state.x;
        stop_direction_ = state.v >= 0.0f ? 1 : -1;
        stop_confirm_count_ = 0;
    }

    if (stopping_x_)
    {
        // Follow the stopping distance, but do not move the target back on rebound.
        if ((stop_direction_ > 0 && state.x > held_x_) ||
            (stop_direction_ < 0 && state.x < held_x_))
            held_x_ = state.x;

        const bool nearly_stopped = std::fabs(state.v) < mapping::stationary_vel_error;
        stop_confirm_count_ = nearly_stopped ? stop_confirm_count_ + 1U : 0U;
        if (stop_confirm_count_ >= mapping::stop_confirm_ticks)
        {
            stopping_x_ = false;
            hold_x_ = true;
            stop_confirm_count_ = 0;
        }
    }
    cmd.x = held_x_;
}

cmd_frame mapper::step(const motion_req& request, const chassis_state& state,
                           std::uint32_t now) noexcept
{
    cmd_frame frame{};
    frame.sequence = ++sequence_;
    frame.received_tick = request.received_tick;
    const bool online = request.online && now - request.received_tick <= mapping::offline_ticks;
    auto& cmd = frame.chassis_cmd;
    cmd.move = online && request.move;
    const bool ready = state.fsm_executed == chassis_fsm::normal ||
                       state.fsm_executed == chassis_fsm::spin;
    cmd.spin = cmd.move && ready && state.v_cmd_enabled && request.spin;
    const bool spin_active = cmd.move && (cmd.spin || state.fsm_executed == chassis_fsm::spin);
    if (!cmd.move)
        len_ref_ = chassis::model::Lmin;
    else if (ready)
        len_ref_ += request.len_rate * mapping::dt;
    len_ref_ = std::clamp(len_ref_, chassis::model::Lmin, chassis::model::Lmax);
    cmd.len = len_ref_;

    if (!cmd.move)
    {
        v_ramp_.set_value(0.0f);
        yaw_ramp_.set_value(0.0f);
        hold_yaw_ = false;
    }
    if (spin_active)
    {
        v_ramp_.set_value(0.0f);
        cmd.v = 0.0f;
        cmd.dyaw = yaw_ramp_.update(cmd.spin ? mapping::spin_rate : 0.0f);
    }
    else if (cmd.move && state.v_cmd_enabled)
    {
        cmd.v = v_ramp_.update(std::clamp(request.v, -mapping::max_v, mapping::max_v));
        cmd.dyaw = yaw_ramp_.update(std::clamp(request.dyaw, -mapping::max_dyaw, mapping::max_dyaw));
    }
    else
    {
        v_ramp_.set_value(0.0f);
        yaw_ramp_.set_value(0.0f);
    }

    if (spin_active != prev_spin_)
    {
        reset_position(state.x);
        hold_yaw_ = false;
    }
    prev_spin_ = spin_active;
    if (!cmd.move)
    {
        reset_position(state.x);
        cmd.x = held_x_;
        cmd.yaw = state.yaw;
    }
    else
    {
        if (spin_active)
        {
            cmd.x = held_x_;
        }
        else if (!state.v_cmd_enabled)
        {
            reset_position(state.x);
            cmd.x = held_x_;
        }
        else
        {
            update_position(cmd, state);
        }
        if (spin_active)
        {
            hold_yaw_ = false;
            cmd.yaw = state.yaw; // Legacy spin controls yaw rate, not yaw position.
        }
        else if (std::fabs(cmd.dyaw) < 0.002f)
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
    dbg.cmd.stopping_x = stopping_x_;
    dbg.cmd.len_rate = request.len_rate;
    dbg.cmd.holding_x = hold_x_;
    dbg.cmd.stop_confirm_count = stop_confirm_count_;
    dbg.cmd.request_move = request.move;
    dbg.cmd.cmd = cmd;
    dbg.cmd.link_online = online;
    dbg.cmd.link_age_ticks = now - request.received_tick;
    return frame;
}
}
