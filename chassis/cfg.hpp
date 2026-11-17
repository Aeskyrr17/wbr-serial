#pragma once

#include <cstdint>

namespace chassis::cfg
{
// Application timing and temporary test selection.
inline constexpr float control_dt = 0.001f; // Nominal control period, s.
inline constexpr bool spring_test = false; // Temporary left-leg spring cancellation test.

// Software leg-length bounds and balance gain selection, m.
inline constexpr float Lmin = 0.16f;
inline constexpr float Lmid = 0.25f;
inline constexpr float Lmax = 0.35f;
inline constexpr float Lfly = 0.25f;
inline constexpr float Lswitch = 0.25f;

// Actuator output limits, separate from the motor protocol ranges.
inline constexpr float max_wheel_torque = 10.0f; // Nm; application limit, not a motor rating.
inline constexpr float max_joint_torque = 40.0f; // Nm

inline constexpr float spin_len_amp = 0.08f; // m
inline constexpr float spin_len_omega = 2.94395f; // rad/s, about 2.134 s per period.
inline constexpr float gravity_feedforward = 67.865f; // Per-leg force for length PID, N.
inline constexpr float air_leg_force = 100.0f; // Active axial force for an airborne leg, N.

// Contact thresholds and consecutive-cycle confirmation counts.
inline constexpr float offground_bound = 20.0f; // Whole-chassis force threshold, N.
inline constexpr float offground_single_bound = offground_bound * 0.5f; // Per leg, N.
inline constexpr double offground_extend_rate = 0.05; // m/s; preserves the existing comparison.
inline constexpr std::uint32_t offground_confirm_count = 40U; // NORMAL cycles.
inline constexpr float touchdown_compress_rate = -0.05f; // m/s
inline constexpr float touchdown_len_margin = 0.03f; // m below Lmax
inline constexpr float touchdown_peak_rate = 0.10f; // m/s
inline constexpr float touchdown_rate_drop = 0.15f; // m/s
inline constexpr std::uint32_t touchdown_confirm_count = 10U; // AIRBORNE cycles.
inline constexpr float offground_lowerbound = 0.0f; // Currently unused.
inline constexpr float offground_upperbound = 0.0f; // Currently unused.
} // namespace chassis::cfg
