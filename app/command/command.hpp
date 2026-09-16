#pragma once

#include "robot_types.hpp"
#include "slope.hpp"
#include "robot_io.hpp"

namespace robot::command
{

struct referee_summary
{
    std::uint8_t robot_id = 0;
    std::uint8_t robot_level = 0;
    std::uint16_t heat_limit = 0;
    std::uint16_t heat_now = 0;
    float chassis_power_limit = 45.0f;
    float power_buffer = 60.0f;
    std::uint8_t remained_energy = 0;
    std::uint8_t hurt_armor_id = 0;
    std::uint8_t hurt_reason = 0;
    bool chassis_power_enabled = false;
};

class mapper
{
public:
    command_frame step(const io::gimbal_command& raw, const io::tof_sample& tof,
                       const referee_summary& referee, const robot_state& state,
                       std::uint32_t now) noexcept;

private:
    control::slope yaw_ramp_{0.0f, 0.02f};
    control::slope vx_ramp_{0.0f, 0.006f};
    control::slope vy_ramp_{0.0f, 0.006f};
    action_events events_{};
    io::gimbal_wants handled_wants_{};
    float held_x_ = 0.0f;
    float held_yaw_ = 0.0f;
    float spin_phase_ = 0.0f;
    std::uint32_t jump_wait_count_ = 0;
    std::uint32_t published_sequence_ = 0;
    bool hold_x_ = false;
    bool hold_yaw_ = false;
    bool jump_armed_ = false;
    bool prev_spin_ = false;
};

} // namespace robot::command
