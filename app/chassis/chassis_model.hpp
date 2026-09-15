#pragma once

namespace robot::chassis::model
{

inline constexpr float pi = 3.14159265358979323846f;
inline constexpr float gravity = 9.80665f;
inline constexpr float L1 = 0.220f;
inline constexpr float L2 = 0.260f;
inline constexpr float Lmin = 0.16f;
inline constexpr float Lmid = 0.25f;
inline constexpr float Lmax = 0.35f;
inline constexpr float Lfly = 0.25f;
inline constexpr float Lswitch = 0.25f;
inline constexpr float wheel_radius = 0.06f;
inline constexpr float wheel_track = 0.43f;
inline constexpr float wheel_mass = 0.21f;
inline constexpr float body_mass = 14.0f;
inline constexpr float wheel_current_per_nm = 1400.0f;
inline constexpr float max_wheel_torque = 10.0f;
inline constexpr float max_joint_torque = 40.0f;
inline constexpr float spring_force = 250.0f;
inline constexpr float spring_d1 = 0.03f;
inline constexpr float spring_d2 = 0.05f;
inline constexpr float spring_angle = 0.2164f;
inline constexpr float spin_len_amp = 0.08f;
// Legacy value: 2.94395 rad/s, about 2.134 s per period.
inline constexpr float spin_len_omega = 2.94395f;
inline constexpr float gravity_feedforward = 67.865f;
inline constexpr float alpha_eq[3] = {0.280918f, -1.101757f, 1.232768f};
inline constexpr float yaw_offset_front = -2.54257321f;
inline constexpr float yaw_offset_rear = 0.599019443f;

} // namespace robot::chassis::model
