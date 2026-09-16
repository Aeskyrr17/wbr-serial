#include "command.hpp"

#include "chassis_model.hpp"
#include "robot_debug.hpp"

#include <algorithm>
#include <cmath>

namespace robot::command
{
namespace
{
float wrap(float value) noexcept { return std::remainder(value, 2.0f * chassis::model::pi); }
}

command_frame mapper::step(const io::gimbal_command& raw, const io::tof_sample& tof,
                           const referee_summary& referee, const robot_state& state,
                           std::uint32_t now) noexcept
{
    command_frame result{};
    result.received_tick = raw.received_tick;
    result.sequence = ++published_sequence_;
    const std::uint32_t link_age = now - raw.received_tick;
    const bool online = raw.sequence != 0U && link_age <= 100U;
    const bool aux_enabled = online && raw.move;
    const bool chassis_disabled = !aux_enabled || raw.gimbal_only;
    const io::gimbal_wants new_wants{
        raw.wants.jump - handled_wants_.jump,
        raw.wants.turn - handled_wants_.turn,
        raw.wants.stair - handled_wants_.stair,
        raw.wants.fly - handled_wants_.fly,
    };
    handled_wants_ = raw.wants;
    const bool accept_wants = !chassis_disabled &&
                              state.state_executed == chassis_state::normal;
    if (!accept_wants)
    {
        jump_armed_ = false;
        jump_wait_count_ = 0U;
    }

    result.aux.move = online && raw.move;
    result.aux.spin = raw.spin;
    result.aux.gimbal_only = raw.gimbal_only;
    result.aux.yaw_current = raw.yaw_current;
    result.aux.trigger_output_velocity = aux_enabled ? raw.trigger_speed : 0.0f;

    float len = raw.len_level <= 0 ? chassis::model::Lmin :
                (raw.len_level == 1 ? chassis::model::Lmid : chassis::model::Lmax);
    auto& cmd = result.chassis;
    cmd.len = len;
    // Recovery must remain reachable before yaw initialization is complete.
    cmd.move = !chassis_disabled && (!state.recovered || state.yaw_ready);
    const float relative_angle = wrap(state.yaw_fdb - chassis::model::yaw_offset_front);
    if (!cmd.move || !state.recovered)
    {
        cmd.x = state.x;
        cmd.yaw = state.yaw;
        cmd.v = 0.0f;
        vx_ramp_.set_value(0.0f); vy_ramp_.set_value(0.0f);
    }
    else if (raw.spin)
    {
        cmd.spin = true;
        if (!prev_spin_) spin_phase_ = 0.0f;
        spin_phase_ += chassis::model::spin_len_omega * 0.001f;
        cmd.len = std::clamp(chassis::model::Lmid + chassis::model::spin_len_amp *
                             std::sin(spin_phase_), chassis::model::Lmin, chassis::model::Lmax);
        cmd.dyaw = yaw_ramp_.update(10.0f);
        cmd.v = 0.0f;
    }
    else
    {
        spin_phase_ = 0.0f;
        if (accept_wants)
        {
            events_.stair += new_wants.stair;
            events_.fly += new_wants.fly;
            if (new_wants.jump != 0U)
            {
                if ((new_wants.jump & 1U) != 0U) jump_armed_ = !jump_armed_;
                jump_wait_count_ = 0U;
            }
        }

        if (jump_armed_)
        {
            ++jump_wait_count_;
            if (tof.valid && tof.distance_cm > 5.0f && tof.distance_cm <= 60.0f)
            { ++events_.jump; jump_armed_ = false; }
            else if (jump_wait_count_ > 10000U) jump_armed_ = false;
        }

        if (yaw_ramp_.value() > 1.0f)
        {
            cmd.x = state.x;
            cmd.v = 0.0f;
            cmd.dyaw = yaw_ramp_.update(0.0f);
        }
        else if (jump_armed_)
        {
            cmd.dyaw = -relative_angle * 4.0f;
            cmd.v = std::fabs(raw.vx * 0.1f) > 0.005f
                        ? vx_ramp_.update(raw.vx * 0.2f)
                        : 0.0f;
        }
        else if (std::fabs(raw.vx * 0.1f) > 0.005f)
        {
            cmd.dyaw = -relative_angle * 4.0f;
            cmd.v = vx_ramp_.update(raw.vx * 0.1f * (state.flying ? 2.5f : 2.0f));
        }
        else if (std::fabs(raw.vy * 0.1f) > 0.005f)
        {
            cmd.dyaw = -wrap(relative_angle + 0.5f * chassis::model::pi) * 4.0f;
            cmd.v = vy_ramp_.update(raw.vy * 0.18f);
        }
        else
        {
            cmd.dyaw = -relative_angle * 4.0f;
            cmd.v = 0.0f;
        }
    }
    prev_spin_ = raw.spin;
    cmd.events = events_;

    float power_limit = referee.chassis_power_limit;
    float power_buffer = referee.power_buffer;
    if (power_limit > 210.0f || power_limit < 40.0f) power_limit = 45.0f;
    if (power_buffer > 200.0f || power_buffer < 0.0f) power_buffer = 60.0f;
    if (referee.remained_energy == 0U) power_limit *= 0.33f;
    else if ((referee.remained_energy & 0x10U) == 0U) power_limit *= 0.75f;
    power_limit = std::max(power_limit, 20.0f);
    const float buffer_coeff = std::clamp(power_buffer / 60.0f, 0.01f, 1.0f);
    power_limit = power_limit - 7.0f + buffer_coeff * 5.5f;
    if (!state.flying && !jump_armed_) cmd.v *= buffer_coeff;
    if (state.len > 0.17f)
    {
        const float velocity_step = 0.006f - 0.03f * (state.len - 0.17f);
        vx_ramp_.set_step(velocity_step);
        vy_ramp_.set_step(velocity_step);
    }
    else
    {
        vx_ramp_.set_step(0.006f);
        vy_ramp_.set_step(0.006f);
    }
    if (state.len > 0.17f) cmd.v *= 0.17f / state.len;
    if (!state.v_cmd_enabled)
    {
        cmd.v = 0.0f;
        vx_ramp_.set_value(0.0f);
        vy_ramp_.set_value(0.0f);
    }

    if (std::fabs(cmd.v) < 0.005f && std::fabs(cmd.v - state.v) < 0.3f)
    {
        if (!hold_x_) { held_x_ = state.x; hold_x_ = true; }
        cmd.x = held_x_;
    }
    else { hold_x_ = false; cmd.x = state.x + cmd.v * 0.001f; }
    if (std::fabs(cmd.dyaw) < 0.002f)
    {
        if (!hold_yaw_) { held_yaw_ = state.yaw; hold_yaw_ = true; }
        cmd.yaw = held_yaw_;
    }
    else
    {
        hold_yaw_ = false;
        cmd.yaw = state.yaw + (cmd.spin ? 0.0f : cmd.dyaw * 0.001f);
    }

    robot_debug.command.command = cmd;
    robot_debug.command.tof_distance_cm = tof.distance_cm;
    robot_debug.command.power_limit = power_limit;
    robot_debug.command.power_buffer = power_buffer;
    robot_debug.command.link_age_ticks = link_age;
    robot_debug.command.link_online = online;
    robot_debug.command.jump_armed = jump_armed_;
    robot_debug.io.command_sequence = raw.sequence;
    robot_debug.io.want_jump_count = raw.wants.jump;
    robot_debug.io.want_turn_count = raw.wants.turn;
    robot_debug.io.want_stair_count = raw.wants.stair;
    robot_debug.io.want_fly_count = raw.wants.fly;
    robot_debug.io.tof_sequence = tof.sequence;
    robot_debug.io.tof_valid = tof.valid;

    (void)io::send_chassis_status(state.yaw_ready, referee.robot_id > 9U,
                                  referee.robot_level, referee.heat_limit,
                                  referee.heat_now, 0);
    return result;
}

} // namespace robot::command
