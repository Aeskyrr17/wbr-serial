#pragma once

#include "motor.hpp"

#include <cstdint>

namespace robot
{

enum class chassis_state : std::uint8_t
{
    relax = 0,
    flatten,
    neutral,
    normal,
};

enum class lqr_mode : std::uint8_t { low = 0, high, spin };

enum class input_source : std::uint8_t { can, ps2 };
struct chassis_command
{
    float x = 0.0f;
    float v = 0.0f;
    float len = 0.16f;
    float yaw = 0.0f;
    float dyaw = 0.0f;
    bool move = false;
};

struct command_frame
{
    chassis_command chassis{};
    std::uint32_t received_tick = 0;
    std::uint32_t sequence = 0;
};

// One control-cycle copy of the complete Device feedback. No live Device references.
struct motor_feedback_snapshot
{
    ::motors::feedback l_joint1{};
    ::motors::feedback l_joint4{};
    ::motors::feedback r_joint1{};
    ::motors::feedback r_joint4{};
    ::motors::feedback l_wheel{};
    ::motors::feedback r_wheel{};
    bool wheels_online = false;
};

// Virtual leg geometry, not individual motor angles. Angles: rad; length: m.
// alpha = phi - pi/2 + body pitch; d* are rates in SI units.
// fs is passive spring support; leg_force/hip_torque come from joint feedback.
// normal_force is the legacy estimated wheel-ground support, not a force sensor.
struct leg_state
{
    float phi1 = 0.0f; // VMC input after motor-coordinate conversion (rad).
    float phi4 = 0.0f;
    float total_phi = 0.0f;
    float spring_len = 0.0f; // Spring attachment-point distance (m).
    bool valid = false; // Current geometry solve; other solved fields retain last valid sample on failure.
    float phi = 0.0f;
    float dphi = 0.0f;
    float alpha = 0.0f;
    float dalpha = 0.0f;
    float alpha_eq = 0.0f;
    float len = 0.0f;
    float dlen = 0.0f;
    float fs = 0.0f;
    float leg_force = 0.0f;
    float hip_torque = 0.0f;
    float normal_force = 0.0f;
    bool flat = false;
    bool neutral = false;
};

// Tw: wheel torque; Tp: equivalent hip actuator torque (Nm, tau_l); F: axial leg force (N).
// l/r_joint*: motor-coordinate torque after VMC and right-side mirroring.
struct chassis_output
{
    float Twl = 0.0f;
    float Twr = 0.0f;
    float Tpl = 0.0f;
    float Tpr = 0.0f;
    float Fl = 0.0f;
    float Fr = 0.0f;
    float l_joint1 = 0.0f;
    float l_joint4 = 0.0f;
    float r_joint1 = 0.0f;
    float r_joint4 = 0.0f;
};

struct robot_state
{
    float x = 0.0f;
    float v = 0.0f;
    float len = 0.0f;
    float lalpha = 0.0f;
    float ralpha = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    // Legacy normal: allows command.v, including wheel assistance in FLATTEN/NEUTRAL.
    bool v_cmd_enabled = false;
    bool recovered = false;
    chassis_state state_executed = chassis_state::relax;
    chassis_state next_state = chassis_state::relax;
};

} // namespace robot
