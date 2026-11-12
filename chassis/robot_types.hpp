#pragma once

#include "model/vmc.hpp"

#include "motor.hpp"

#include <array>
#include <cstdint>

namespace chassis
{

// Operating mode; contact and other measurements are data, not FSM values.
enum class chassis_fsm : std::uint8_t
{
    relax = 0,
    flatten,
    neutral,
    normal,
    spin,
    recover,
    airborne,
};


enum class lqr_mode : std::uint8_t { low = 0, high, spin };

struct lqr_state
{
    float x = 0.0f;
    float v = 0.0f;
    float yaw = 0.0f;
    float dyaw = 0.0f;
    float l_alpha = 0.0f;
    float l_dalpha = 0.0f;
    float r_alpha = 0.0f;
    float r_dalpha = 0.0f;
    float pitch = 0.0f;
    float dpitch = 0.0f;
    [[nodiscard]] std::array<float, 10> array() const noexcept
    {
        return {x, v, yaw, dyaw, l_alpha, l_dalpha, r_alpha, r_dalpha, pitch, dpitch};
    }
};

struct chassis_cmd
{
    float x = 0.0f;
    float v = 0.0f;
    float len = 0.16f;
    float yaw = 0.0f;
    float dyaw = 0.0f;
    bool move = false;
    bool spin = false;
    bool jump = false;
    bool go_stair = false;
};

struct cmd_frame
{
    chassis::chassis_cmd chassis_cmd{};
    std::uint32_t received_tick = 0;
    std::uint32_t sequence = 0;
};

// One control-cycle copy of the complete Device feedback. No live Device references.
struct motor_fdb
{
    ::motor::feedback_t l_j1{}; //左腿右关节
    ::motor::feedback_t l_j4{}; //左腿左关节
    ::motor::feedback_t r_j1{}; //右腿右关节
    ::motor::feedback_t r_j4{}; //右腿左关节
    ::motor::feedback_t l_w{};
    ::motor::feedback_t r_w{};
    bool motors_online = false;
};

// Virtual leg geometry, not individual motor angles. Angles: rad; length: m.
// alpha = phi - pi/2 + body pitch
// fs is passive spring support; leg_force/hip_torque come from joint feedback.
// normal_force is the legacy estimated wheel-ground support, not a force sensor.
struct leg_phi_state
{
    bool flat = false;
    bool neutral = false;
};
struct leg_state
{
    spring_result spring{};
    chassis::vmc_error geometry_error = chassis::vmc_error::none;
    float phi1 = 0.0f;
    float phi4 = 0.0f;
    float total_phi = 0.0f;
    bool valid = false; // Geometry AND spring valid; spring fields/len/phi are current-cycle values.
    float phi = 0.0f;
    float dphi = 0.0f;
    float alpha = 0.0f;
    float dalpha = 0.0f;
    float alpha_eq = 0.0f;
    float len = 0.0f;
    float dlen = 0.0f;
    float leg_force = 0.0f;     //< Axial leg force (N)
    float hip_torque = 0.0f;    //< Hip joint torque (Nm)
    float normal_force = 0.0f;
    bool offground = false; // NORMAL/AIRBORNE 中按力阈值判断，其余状态为 false。
    bool flat = false;
    bool neutral = false;
    leg_phi_state phi_state{};
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

enum class offground_state : std::uint8_t
{
    on_ground = 0,
    left_off_ground,
    right_off_ground,
    both_off_ground
};
// Latest chassis measurements and control status for command mapping/debugging.
struct chassis_state
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
    chassis_fsm fsm_executed = chassis_fsm::relax; // Mode used for this output.
    chassis_fsm fsm_next = chassis_fsm::relax; // Mode selected for the next step.
    float normal_force = 0.0f;
    offground_state offground = offground_state::on_ground;
};

} // namespace chassis
