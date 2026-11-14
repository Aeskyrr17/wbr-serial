#pragma once

#include <cstdint>

namespace chassis::model
{

inline constexpr float pi = 3.14159265358979323846f;
inline constexpr float gravity = 9.80665f;
inline constexpr float control_dt = 0.001f; // Nominal control period, s.
inline constexpr bool spring_test = false; // Temporary left-leg spring cancellation test.
inline constexpr float L1 = 0.220f;
inline constexpr float L2 = 0.260f;
inline constexpr float Lmin = 0.16f;
inline constexpr float Lmid = 0.25f;
inline constexpr float Lmax = 0.35f;
inline constexpr float Lfly = 0.25f;
inline constexpr float Lswitch = 0.25f;
inline constexpr float wheel_radius = 0.06f;
inline constexpr float wheel_track = 0.43f; //轮距
inline constexpr float wheel_mass = 0.21f;
inline constexpr float body_mass = 14.0f;
// inline constexpr float wheel_current_scale = 2598.21f;
inline constexpr float wheel_current_scale = 1400.00f;
inline constexpr float max_wheel_torque = 10.0f; // 不是官方算出来的，稍微给大一点了
inline constexpr float max_joint_torque = 40.0f;
// DM joint protocol ranges and feedback conversion; separate from actuator limits.
inline constexpr float joint_position_limit = 62.8319f; // rad
inline constexpr float joint_velocity_limit = 45.0f; // rad/s
inline constexpr float joint_torque_limit = 54.0f; // Nm
inline constexpr float joint_velocity_scale = 1.0f;
inline constexpr bool joint_wrap_position = false;
inline constexpr float spring_force = 450.0f;
inline constexpr float spring_d1 = 0.03f;
inline constexpr float spring_d2 = 0.05f;
inline constexpr float spring_angle = 0.2164f;
inline constexpr float spin_len_amp = 0.08f;
// Legacy value: 2.94395 rad/s, about 2.134 s per period.
inline constexpr float spin_len_omega = 2.94395f;
inline constexpr float gravity_feedforward = 67.865f;
inline constexpr float air_leg_force = 100.0f; // Active axial force for an airborne leg, N.
inline constexpr float offground_bound = 20.0f; // Whole-chassis offground force threshold, N.
inline constexpr float offground_single_bound = offground_bound * 0.5f; // Per leg, N.
inline constexpr std::uint32_t offground_confirm_count = 40U; // Consecutive NORMAL cycles. //! 20ms站起来之后容易进离地，暂时没有锁死
inline constexpr float touchdown_compress_rate = -0.05f; // m/s
inline constexpr float touchdown_len_margin = 0.03f; // m below Lmax
inline constexpr float touchdown_peak_rate = 0.10f; // m/s
inline constexpr float touchdown_rate_drop = 0.15f; // m/s
inline constexpr std::uint32_t touchdown_confirm_count = 10U; // Consecutive AIRBORNE cycles.
inline constexpr float offground_lowerbound = 0.0f; //整车（双腿）离地检测上界
inline constexpr float offground_upperbound = 0.0f; //整车（双腿）离地检测下界
//! TODO: 离地检测阈值
inline constexpr float alpha_eq[3] = {0.280918f, -1.101757f, 1.232768f}; 
//三个对应腿长下的alpha角eq偏置

} // namespace chassis::model
