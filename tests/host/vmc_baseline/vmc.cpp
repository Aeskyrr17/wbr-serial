#include "vmc.hpp"

#include "chassis_model.hpp"
#include "model/kinematics.hpp"

#include <cmath>

namespace robot::chassis
{

bool vmc::resolve(float phi1, float phi4) noexcept
{
    using namespace model;
    static_cast<spring_result&>(state_) = {};
    state_.len = state_.phi = std::numeric_limits<float>::quiet_NaN();
    if (!std::isfinite(phi1) || !std::isfinite(phi4))
        return false;
    const float s1 = std::sin(phi1), c1 = std::cos(phi1);
    const float s4 = std::sin(phi4), c4 = std::cos(phi4);
    const auto geometry = wbr_generated::solve_geometry(phi1, phi4, L1, L2);
    if (!geometry.valid)
        return false;
    const float u2 = geometry.u2;
    const float bx = L1 * c1, by = L1 * s1;
    const float cx = bx + L2 * std::cos(u2);
    const float cy = by + L2 * std::sin(u2);
    const float dx = L1 * c4, dy = L1 * s4;
    const float u3 = pi + std::atan2(dy - cy, dx - cx);
    const float len = std::sqrt(cx * cx + cy * cy);
    if (len < 1.0e-6f)
        return false;
    const float phi = std::atan2(cy, cx);
    const float s32 = std::sin(u3 - u2);
    const float s12 = std::sin(phi1 - u2);
    const float s34 = std::sin(u3 - phi4);
    if (std::fabs(s32) < 1.0e-5f || std::fabs(s12) < 1.0e-5f ||
        std::fabs(s34) < 1.0e-5f)
        return false;

    const float c03 = std::cos(phi - u3), c02 = std::cos(phi - u2);
    const float s03 = std::sin(phi - u3), s02 = std::sin(phi - u2);
    state_.len = len;
    state_.phi = phi;
    state_.J = {L1 * s03 * s12 / s32, L1 * s02 * s34 / s32,
                L1 * c03 * s12 / (s32 * len), L1 * c02 * s34 / (s32 * len)};
    state_.JT = {state_.J[0], state_.J[2], state_.J[1], state_.J[3]};
    state_.JT_inv = {-c02 / (s12 * L1), c03 / (s34 * L1),
                     s02 * len / (s12 * L1), -s03 * len / (s34 * L1)};

    // The legacy +sqrt closure has BQ clockwise from OB on this assembly.
    if (std::sin(u2 - phi1) >= 0.0f)
    {
        state_.error = spring_error::branch;
        return false;
    }
    static_cast<spring_result&>(state_) = solve_spring(len);
    return state_.spring_valid;
}

std::array<float, 2> vmc::velocity(float dphi1, float dphi4) const noexcept
{
    return {state_.J[0] * dphi1 + state_.J[1] * dphi4,
            state_.J[2] * dphi1 + state_.J[3] * dphi4};
}

std::array<float, 2> vmc::joint_torque(float leg_force, float hip_torque) const noexcept
{
    return {state_.JT[0] * leg_force + state_.JT[1] * hip_torque,
            state_.JT[2] * leg_force + state_.JT[3] * hip_torque};
}

std::array<float, 2> vmc::generalized_force(float tq1, float tq4) const noexcept
{
    return {state_.JT_inv[0] * tq1 + state_.JT_inv[1] * tq4,
            state_.JT_inv[2] * tq1 + state_.JT_inv[3] * tq4};
}

} // namespace robot::chassis
