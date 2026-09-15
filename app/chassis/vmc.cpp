#include "vmc.hpp"

#include "chassis_model.hpp"
#include "wbr/kinematics.hpp"

#include <cmath>

namespace robot::chassis
{

bool vmc::resolve(float phi1, float phi4) noexcept
{
    using namespace model;
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

    const float p_angle = phi1 + spring_angle - 0.5f * pi;
    const float px = spring_d1 * std::cos(p_angle);
    const float py = spring_d1 * std::sin(p_angle);
    const float qx = L1 * c1 + spring_d2 * std::cos(u2);
    const float qy = L1 * s1 + spring_d2 * std::sin(u2);
    const float sx = qx - px, sy = qy - py;
    const float spring_len = std::sqrt(sx * sx + sy * sy);
    if (spring_len < 1.0e-5f || std::fabs(std::sin(u2 - u3)) < 0.05f)
    {
        state_.fs = 0.0f;
        return true;
    }
    const float fsx = spring_force * sx / spring_len;
    const float fsy = spring_force * sy / spring_len;
    const float inv = 1.0f / (L2 * std::sin(u2 - u3));
    const float du2_d1 = L1 * std::sin(u3 - phi1) * inv;
    const float du2_d4 = L1 * std::sin(phi4 - u3) * inv;
    const float dqx1 = -L1 * s1 - spring_d2 * std::sin(u2) * du2_d1;
    const float dqy1 = L1 * c1 + spring_d2 * std::cos(u2) * du2_d1;
    const float dqx4 = -spring_d2 * std::sin(u2) * du2_d4;
    const float dqy4 = spring_d2 * std::cos(u2) * du2_d4;
    const float tq1 = fsx * (dqx1 + spring_d1 * std::sin(p_angle)) +
                      fsy * (dqy1 - spring_d1 * std::cos(p_angle));
    const float tq4 = fsx * dqx4 + fsy * dqy4;
    state_.fs = state_.JT_inv[0] * tq1 + state_.JT_inv[1] * tq4;
    return true;
}

std::array<float, 2> vmc::velocity(float q1, float q4) const noexcept
{
    return {state_.J[0] * q1 + state_.J[1] * q4,
            state_.J[2] * q1 + state_.J[3] * q4};
}

std::array<float, 2> vmc::joint_torque(float force, float torque) const noexcept
{
    return {state_.JT[0] * force + state_.JT[1] * torque,
            state_.JT[2] * force + state_.JT[3] * torque};
}

std::array<float, 2> vmc::generalized_force(float tq1, float tq4) const noexcept
{
    return {state_.JT_inv[0] * tq1 + state_.JT_inv[1] * tq4,
            state_.JT_inv[2] * tq1 + state_.JT_inv[3] * tq4};
}

} // namespace robot::chassis
