#pragma once

namespace chassis::model
{
inline constexpr float pi = 3.14159265358979323846f;
inline constexpr float gravity = 9.80665f;

// Link geometry, wheel dimensions and masses, in m and kg.
inline constexpr float L1 = 0.220f;
inline constexpr float L2 = 0.260f;
inline constexpr float wheel_radius = 0.06f;
inline constexpr float wheel_track = 0.43f;
inline constexpr float wheel_mass = 0.21f;
inline constexpr float body_mass = 14.0f;

// Constant-force spring model and installation geometry.
inline constexpr float spring_force = 450.0f;
inline constexpr float spring_d1 = 0.03f;
inline constexpr float spring_d2 = 0.05f;
inline constexpr float spring_angle = 0.2164f;

// Equilibrium alpha = a0 + a1 * length + a2 * length^2, rad (length in m).
inline constexpr float alpha_eq[3] = {0.280918f, -1.101757f, 1.232768f};
} // namespace chassis::model
