#pragma once

#include "robot_types.hpp"

#include <cstdint>

namespace robot
{

struct pid_tuning
{
    volatile float kp;
    volatile float ki;
    volatile float kd;
    volatile float max_out;
    volatile float max_iout;
};

struct robot_tuning_t
{
    // Deliberately false in this migration build. Hardware validation must opt in.
    volatile bool motor_output_enabled = false;
    pid_tuning len_startup{5000.0f, 0.0f, -1500.0f, 125.0f, 0.0f};
    pid_tuning len_run{2000.0f, 0.0f, -400.0f, 125.0f, 0.0f};
    pid_tuning roll{0.5f, 0.0f, 0.0f, 0.05f, 0.005f};
    pid_tuning trigger{3600.0f, 0.0f, 0.0f, 10000.0f, 0.0f};
    pid_tuning phi_recover{20.0f, 0.0f, 10.0f, 40.0f, 0.005f};
    pid_tuning phi_flatten_fast{10.0f, 0.0f, 5.0f, 40.0f, 0.005f};
    pid_tuning phi_flatten_slow{5.0f, 0.0f, 10.0f, 40.0f, 0.005f};
    pid_tuning phi_neutral{30.0f, 0.0f, 10.0f, 40.0f, 0.005f};
    pid_tuning phi_stair{10.0f, 0.0f, 5.0f, 40.0f, 0.005f};
};

struct chassis_debug
{
    robot_state state{};
    leg_state left{};
    leg_state right{};
    chassis_output output{};
    float l_phi_ref = 0.0f;
    float r_phi_ref = 0.0f;
    float obs[10]{};
    float ref[10]{};
    float roll = 0.0f;
    float droll = 0.0f;
    float normal_force = 0.0f;
    std::uint8_t lqr_mode = 0;
};

struct command_debug
{
    chassis_command command{};
    float tof_distance_cm = 0.0f;
    float power_limit = 0.0f;
    float power_buffer = 0.0f;
    std::uint32_t link_age_ticks = 0;
    bool link_online = false;
    bool jump_armed = false;
};

struct aux_debug
{
    float yaw_ref_current = 0.0f;
    float yaw_fdb_pos = 0.0f;
    float yaw_fdb_vel = 0.0f;
    float trigger_ref_vel = 0.0f;
    float trigger_fdb_vel = 0.0f;
    float trigger_current = 0.0f;
    bool yaw_ready = false;
};

struct io_debug
{
    std::uint32_t command_sequence = 0;
    std::uint32_t want_jump_count = 0;
    std::uint32_t want_turn_count = 0;
    std::uint32_t want_stair_count = 0;
    std::uint32_t want_fly_count = 0;
    std::uint32_t tof_sequence = 0;
    std::uint32_t supercap_sequence = 0;
    bool tof_valid = false;
};

struct runtime_debug
{
    std::uint32_t period_us = 0;
    std::uint32_t exec_us = 0;
    std::uint32_t max_exec_us = 0;
    std::uint32_t overrun_count = 0;
};

struct robot_debug_t
{
    chassis_debug chassis{};
    command_debug command{};
    aux_debug aux{};
    io_debug io{};
    runtime_debug runtime{};
};

extern "C" robot_debug_t robot_debug;
extern "C" robot_tuning_t robot_tuning;

} // namespace robot
