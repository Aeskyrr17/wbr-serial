#pragma once

#include "robot_types.hpp"
#include "pid.hpp"
#include "model/chassis_model.hpp"

#include <cstdint>



namespace chassis
{

struct pid_param { volatile float kp, ki, kd; };

struct robot_tuning_t
{
    // Deliberately false in this migration build. Hardware validation must opt in.
    volatile bool motor_output_enabled = false;
    volatile float alpha_trim = 0.03f;
    volatile float spin_alpha_trim = -0.03f; // Legacy SPIN equilibrium offset.
    pid_param len_startup{5000.0f, 0.0f, -100.0f};
    pid_param len_run{2000.0f, 0.0f, -400.0f};
    pid_param roll{0.5f, 0.0f, 0.0f};
    pid_param phi_recover{20.0f, 0.0f, 10.0f};
    pid_param phi_flatten_fast{10.0f, 0.0f, 5.0f};
    pid_param phi_flatten_slow{5.0f, 0.0f, 10.0f};
    pid_param phi_neutral{30.0f, 0.0f, 10.0f};
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

struct leg_lqr_debug
{
    float obs[4]{}; // roll, droll, projected hip midpoint H, dH.
    float ref[4]{};
    float support_force[2]{}; // Total support before spring subtraction (N); RELAX preview only.
    bool active = false; // Snapshot: true only when NORMAL ran this cycle.
};

struct control_debug
{
    leg_lqr_debug leg_lqr{};
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
    volatile std::uint32_t sample_seq = 0; // Probe: accept equal even sequence before/after reading.
    std::uint32_t sample_tick = 0; // Both leg snapshots belong to this solve cycle.
    float spring_force = chassis::model::spring_force;
    chassis_state state{};
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
    bool stopping_x = false;
    bool holding_x = false;
    unsigned stop_confirm_count = 0;
    chassis_cmd cmd{};
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

struct robot_debug_t
{
    chassis_debug chassis{};
    command_debug cmd{};
    runtime_debug runtime{};
};

inline robot_debug_t dbg{};
inline robot_tuning_t tuning{};

} // namespace chassis
