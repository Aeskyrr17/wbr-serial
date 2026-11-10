#pragma once

// Generated explicitly by tools/leg_lqr/generate.py. Ordinary builds do not regenerate.
namespace chassis::leg_lqr_model
{
inline constexpr float sample_time = 0.001f;
inline constexpr float mass = 17.66f;
inline constexpr float half_track = 0.215f;
inline constexpr float roll_inertia = 0.64f;
inline constexpr float vertical_damping = 8.0f;
inline constexpr float equilibrium_force = 86.5927195f;
// Rows: left/right total support force (N). Columns: roll, droll, H, dH.
// Negative feedback is already included: F = Mg/2 + gain * (obs - ref).
inline constexpr float gain[2][4] = {
    {-1005.85401f, -92.1899959f, -1716.21614f, -326.017602f},
    {1079.37193f, 97.8932081f, -1773.40092f, -335.359574f},
};
} // namespace chassis::leg_lqr_model
