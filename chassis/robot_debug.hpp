#pragma once

#include "robot_types.hpp"
#include "tinympc/leg_mpc.hpp"

#include <cstdint>

namespace chassis
{

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
    float support_force[2]{}; // Total support before spring subtraction (N).
    bool active = false; // True when leg LQR computed support this cycle.
};

struct control_debug
{
    leg_lqr_debug leg_lqr{};
    tinympc::mpc_diag leg_mpc{};
    pid_debug l_len{};
    pid_debug r_len{};
    pid_debug l_phi{};
    pid_debug r_phi{};
    pid_debug roll{};
    float lqr_raw[4]{}; // Before LQR limits: l_tw, r_tw, l_tp, r_tp (Nm).
    float lqr_output[4]{}; // After LQR limits, before application overrides.
    bool lqr_active = false;
};

struct joint_torque_debug
{
    float l_tj1 = 0.0f;
    float l_tj4 = 0.0f;
    float r_tj1 = 0.0f;
    float r_tj4 = 0.0f;
};

struct contact_debug
{
    std::uint32_t off_count = 0; // Consecutive NORMAL cycles below the force threshold.
    std::uint32_t touch_count = 0; // Consecutive AIRBORNE cycles matching touchdown criteria.
    float dlen_peak = 0.0f; // Peak leg extension rate in this AIRBORNE episode, m/s.
    bool air_seen = false;
    bool touchdown = false; // Contact confirmed for the current AIRBORNE episode.
};

struct chassis_debug
{
    volatile std::uint32_t sample_seq = 0; // Probe: accept equal even sequence before/after reading.
    std::uint32_t sample_tick = 0; // Both leg snapshots belong to this solve cycle.
    float len_cmd = 0.0f; // Length command consumed by the control thread, m.
    float len_target = 0.0f; // Clamped length target, m.
    float len_ref = 0.0f; // Ramp output used by the controller, m.
    contact_debug l_contact{};
    contact_debug r_contact{};
    chassis_state state{};
    leg_state left{};
    leg_state right{};
    virtual_force force{}; // Active virtual-leg targets consumed by VMC.
    motor_cmd motor{}; // Limited actuator targets, before app wheel-current conversion.
    control_debug control{};
    joint_torque_debug joint_raw{}; // Motor coordinates, before joint limits.
    lqr_state obs{};
    lqr_state ref{};
    float roll = 0.0f;
    float droll = 0.0f;
    std::uint8_t lqr_mode = 0;
};

struct command_debug
{
    float len_rate = 0.0f; // PS2 leg-length request, m/s.
    bool stopping_x = false;
    bool holding_x = false;
    unsigned stop_confirm_count = 0;
    chassis_cmd cmd{};
    std::uint32_t link_age_ticks = 0;
    bool link_online = false;
    bool request_move = false; // Request actually consumed by mapper.
};

struct runtime_debug
{
    std::uint8_t joints_enabled = 0; // Bit order: L1, L4, R1, R4; feedback-confirmed.
    std::uint8_t joint_error[4]{}; // Raw DM status, in the same order.
    bool motors_online = false; // All six motors have current feedback.
    bool imu_valid = false; // Online, running, and received within 100 ms.
    bool command_fresh = false; // Command thread published within its timeout.
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

} // namespace chassis
