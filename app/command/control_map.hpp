#pragma once

namespace robot::command::mapping
{
// Body-frame commands: forward m/s and yaw rate rad/s. No gimbal following.
inline constexpr float max_v = 0.5f;
inline constexpr float max_dyaw = 1.5f;
inline constexpr float len_rate = 0.05f; // Hold D-pad: 5 cm/s.
inline constexpr float can_v_scale = 0.2f;
inline constexpr float can_dyaw_scale = 0.1f;
inline constexpr float deadzone = 0.05f;
inline constexpr float dt = 0.001f;
inline constexpr float v_step = 0.006f;
inline constexpr float dyaw_step = 0.02f;
// Position hold: confirm standstill after following the stopping direction.
inline constexpr float stationary_vel = 0.005f; // Command deadband, m/s.
inline constexpr float stationary_vel_error = 0.03f; // Feedback threshold, m/s.
inline constexpr unsigned stop_confirm_ticks = 50U; // Command cycles, nominally 50 ms.
inline constexpr unsigned offline_ticks = 100U;
}
