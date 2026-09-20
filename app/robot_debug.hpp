#pragma once

#include "robot_types.hpp"
#include "pid.hpp"

#include <cstdint>

namespace motors { struct dm_motor_debug; struct dm_bus_debug; }

namespace robot
{

struct robot_tuning_t
{
    // Deliberately false in this migration build. Hardware validation must opt in.
    volatile bool motor_output_enabled = true;
    volatile float alpha_trim = 0.02f;
    control::pid_params len_startup{5000.0f, 0.0f, -1500.0f, 125.0f, 0.0f};
    control::pid_params len_run{2000.0f, 0.0f, -400.0f, 125.0f, 0.0f};
    control::pid_params roll{0.5f, 0.0f, 0.0f, 0.05f, 0.005f};
    control::pid_params phi_flatten_fast{10.0f, 0.0f, 5.0f, 40.0f, 0.005f};
    control::pid_params phi_flatten_slow{5.0f, 0.0f, 10.0f, 40.0f, 0.005f};
    control::pid_params phi_neutral{30.0f, 0.0f, 10.0f, 40.0f, 0.005f};
};

// Snapshot only when this controller ran in the current cycle.
// result includes the PID's own limit, before feedforward and spring compensation.
struct pid_debug
{
    float ref = 0.0f;
    float fdb = 0.0f;
    float result = 0.0f;
    bool active = false;
};

struct control_debug
{
    pid_debug l_len{};
    pid_debug r_len{};
    pid_debug l_phi{};
    pid_debug r_phi{};
    pid_debug roll{};
    float lqr_raw[4]{}; // Before LQR limits: Twl, Twr, Tpl, Tpr (Nm).
    float lqr_output[4]{}; // After LQR limits, before application overrides.
    bool lqr_active = false;
};

struct joint_torque_debug
{
    float l_joint1 = 0.0f;
    float l_joint4 = 0.0f;
    float r_joint1 = 0.0f;
    float r_joint4 = 0.0f;
};

struct chassis_debug
{
    robot_state state{};
    leg_state left{};
    leg_state right{};
    chassis_output output{}; // Combined forces and final limited motor torques.
    control_debug control{};
    joint_torque_debug joint_raw{}; // Motor coordinates, before joint limits.
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
    input_source source = input_source::can;
    std::uint32_t link_age_ticks = 0;
    bool link_online = false;
    std::uint16_t ps2_buttons = 0; // Callback: normalized held-button mask.
    bool ps2_move = false; // Callback mapping result.
    bool request_move = false; // Request actually consumed by mapper.
};

struct runtime_debug
{
    std::uint32_t period_us = 0;
    std::uint32_t exec_us = 0;
    std::uint32_t max_exec_us = 0;
    std::uint32_t overrun_count = 0;
};

// Live read-only views: continue updating while control waits for motor enable.
struct motor_link_debug
{
    const ::motors::dm_bus_debug* joint_bus = nullptr;
    const ::motors::dm_motor_debug* l_joint1 = nullptr;
    const ::motors::dm_motor_debug* l_joint4 = nullptr;
    const ::motors::dm_motor_debug* r_joint1 = nullptr;
    const ::motors::dm_motor_debug* r_joint4 = nullptr;
    bool registered = false;
    bool starting = false;
    types::status start_status = types::status::not_initialized;
};

struct robot_debug_t
{
    chassis_debug chassis{};
    command_debug command{};
    runtime_debug runtime{};
    motor_link_debug motors{};
};

// Keep the short ELF names for probe tooling while retaining the C++ type and
// namespace information needed by source-level debuggers.
extern robot_debug_t robot_debug __asm__("robot_debug");
extern robot_tuning_t robot_tuning __asm__("robot_tuning");

} // namespace robot
