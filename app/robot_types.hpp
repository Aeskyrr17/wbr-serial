#pragma once

#include <cstdint>

namespace robot
{

enum class chassis_state : std::uint8_t
{
    relax = 0,
    recover,
    flatten,
    neutral,
    normal,
    offground,
    spin,
    gostair,
    jump,
};

enum class jump_stage : std::uint8_t { dont = 0, extending, inair };
enum class lqr_mode : std::uint8_t { low = 0, high, spin };

struct action_events
{
    std::uint32_t jump = 0;
    std::uint32_t stair = 0;
    std::uint32_t fly = 0;
};

struct chassis_command
{
    float x = 0.0f;
    float v = 0.0f;
    float len = 0.16f;
    float yaw = 0.0f;
    float dyaw = 0.0f;
    bool move = false;
    bool spin = false;
    action_events events{};
};

struct aux_command
{
    std::int16_t yaw_current = 0;
    float trigger_rotor_velocity = 0.0f;
    bool move = false;
    bool gimbal_only = false;
};

struct command_frame
{
    chassis_command chassis{};
    aux_command aux{};
    std::uint32_t received_tick = 0;
    std::uint32_t sequence = 0;
};

struct leg_input
{
    float q1 = 0.0f;
    float q4 = 0.0f;
    float dq1 = 0.0f;
    float dq4 = 0.0f;
    float tq1 = 0.0f;
    float tq4 = 0.0f;
};

struct leg_state
{
    float phi = 0.0f;
    float dphi = 0.0f;
    float alpha = 0.0f;
    float dalpha = 0.0f;
    float alpha_eq = 0.0f;
    float len = 0.0f;
    float dlen = 0.0f;
    float fs = 0.0f;
    float force = 0.0f;
    float pitch_torque = 0.0f;
    float normal_force = 0.0f;
    bool flat = false;
    bool neutral = false;
};

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
    float yaw_feedback = 0.0f;
    float yaw_target = 0.0f;
    bool normal = false;
    bool recovered = false;
    bool flying = false;
    bool stairing = false;
    bool spinning = false;
    bool yaw_ready = false;
    chassis_state state_executed = chassis_state::relax;
    chassis_state next_state = chassis_state::relax;
    jump_stage jump = jump_stage::dont;
};

} // namespace robot
