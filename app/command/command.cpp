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
    const bool disabled = !online || !raw.move || raw.gimbal_only;
    result.aux.move = online && raw.move;
    result.aux.gimbal_only = raw.gimbal_only;
    result.aux.yaw_current = raw.yaw_current;
    result.aux.trigger_rotor_velocity = disabled ? 0.0f : raw.trigger_speed * 36.0f;

    float len = raw.len_level <= 0 ? chassis::model::Lmin :
                (raw.len_level == 1 ? chassis::model::Lmid : chassis::model::Lmax);
    auto& cmd = result.chassis;
    cmd.len = len;
    // Recovery must remain reachable before yaw initialization is complete.
    cmd.move = !disabled && (!state.recovered || state.yaw_ready);
    const float relative_angle = wrap(state.yaw_feedback - chassis::model::yaw_offset_front);
    if (!cmd.move || !state.recovered)
    {
        cmd.x = state.x;
        cmd.yaw = state.yaw_target;
        cmd.v = 0.0f;
        vx_ramp_.set_value(0.0f); vy_ramp_.set_value(0.0f);
    }
    else if (raw.spin)
    {
        cmd.spin = true;
        if (!previous_spin_) spin_phase_ = 0.0f;
        spin_phase_ += chassis::model::spin_len_omega * 0.001f;
        cmd.len = std::clamp(chassis::model::Lmid + chassis::model::spin_len_amp *
                             std::sin(spin_phase_), chassis::model::Lmin, chassis::model::Lmax);
        cmd.dyaw = yaw_ramp_.update(10.0f);
        cmd.v = 0.0f;
    }
    else
    {
        spin_phase_ = 0.0f;
        cmd.dyaw = yaw_ramp_.value() > 1.0f ? yaw_ramp_.update(0.0f) : -relative_angle * 4.0f;
        if (std::fabs(raw.vx * 0.1f) > 0.005f)
            cmd.v = vx_ramp_.update(raw.vx * 0.1f * (state.flying ? 2.5f : 2.0f));
        else if (std::fabs(raw.vy * 0.1f) > 0.005f)
        {
            cmd.dyaw = -wrap(relative_angle + 0.5f * chassis::model::pi) * 4.0f;
            cmd.v = vy_ramp_.update(raw.vy * 0.18f);
        }
        else cmd.v = 0.0f;

        if (raw.stair && !previous_stair_) ++events_.stair;
        if (raw.fly && !previous_fly_) ++events_.fly;
        if (raw.jump && !previous_jump_) { jump_armed_ = !jump_armed_; jump_wait_count_ = 0; }
        if (jump_armed_)
        {
            ++jump_wait_count_;
            if (tof.valid && tof.distance_cm > 5.0f && tof.distance_cm <= 60.0f)
            { ++events_.jump; jump_armed_ = false; }
            else if (jump_wait_count_ > 10000U) jump_armed_ = false;
        }
    }
    previous_jump_ = raw.jump; previous_stair_ = raw.stair;
    previous_fly_ = raw.fly; previous_spin_ = raw.spin;
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
    if (state.len > 0.17f) cmd.v *= 0.17f / state.len;

    if (std::fabs(cmd.v) < 0.005f && std::fabs(cmd.v - state.v) < 0.3f)
    {
        if (!hold_x_) { held_x_ = state.x; hold_x_ = true; }
        cmd.x = held_x_;
    }
    else { hold_x_ = false; cmd.x = state.x + cmd.v * 0.001f; }
    if (std::fabs(cmd.dyaw) < 0.002f)
    {
        if (!hold_yaw_) { held_yaw_ = state.yaw_target; hold_yaw_ = true; }
        cmd.yaw = held_yaw_;
    }
    else { hold_yaw_ = false; cmd.yaw = state.yaw_target + (cmd.spin ? 0.0f : cmd.dyaw * 0.001f); }

    robot_debug.command.command = cmd;
    robot_debug.command.tof_distance_cm = tof.distance_cm;
    robot_debug.command.power_limit = power_limit;
    robot_debug.command.power_buffer = power_buffer;
    robot_debug.command.link_age_ticks = link_age;
    robot_debug.command.link_online = online;
    robot_debug.io.command_sequence = raw.sequence;
    robot_debug.io.tof_sequence = tof.sequence;
    robot_debug.io.tof_valid = tof.valid;

    io::supercap_feedback cap{};
    (void)io::read_supercap(cap);
    const std::uint8_t cap_state = !referee.chassis_power_enabled ? 0U :
        ((cap.sequence != 0U && cap.state == 0U && cap.voltage > 0.0f) ? 2U : 1U);
    (void)io::send_supercap(cap_state, 0U, power_limit, 0U, referee.remained_energy);
    (void)io::send_chassis_status(state.yaw_ready, referee.robot_id > 9U,
                                  referee.robot_level, referee.heat_limit,
                                  referee.heat_now, static_cast<std::int16_t>(cmd.dyaw));
    return result;
}

} // namespace robot::command
