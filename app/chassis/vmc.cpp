#include "vmc.hpp"

#include "chassis_model.hpp"

#include <cmath>

namespace robot::chassis
{

bool vmc::resolve(float phi1, float phi4) noexcept
{
    using namespace model;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    state_ = {};
    state_.len = state_.phi = nan;
    state_.J.fill(nan);
    state_.JT.fill(nan);
    state_.JT_inv.fill(nan);
    if (!std::isfinite(phi1) || !std::isfinite(phi4))
        return false;
    if (!std::isfinite(L1) || !std::isfinite(L2) || L1 <= 0.0f || L2 <= 0.0f)
    {
        state_.geometry_error = vmc_error::dimensions;
        return false;
    }

    // Choose the same oriented closure as the old +sqrt coordinate solution.
    // Adjust the relative angle BEFORE averaging: independent 2*pi turns must
    // not flip the assembly. Coincident active links are rejected below.
    const float q1 = std::remainder(phi1, 2.0f * pi);
    const float q4 = std::remainder(phi4, 2.0f * pi);
    float delta = std::fmod(q1 - q4, 2.0f * pi);
    if (delta < 0.0f)
        delta += 2.0f * pi;
    const float half_delta = 0.5f * delta;
    const float phi = std::remainder(q4 + half_delta, 2.0f * pi);
    const float sn = std::sin(half_delta);
    const float cs = std::cos(half_delta);
    const float h_sq = L2 * L2 - L1 * L1 * sn * sn;

    // Length thresholds are metres; k is metres per radian.
    constexpr float min_h = 1.0e-6f;
    constexpr float min_len = 1.0e-6f;
    constexpr float min_k = 1.0e-6f;
    if (!std::isfinite(h_sq) || h_sq <= min_h * min_h)
    {
        state_.geometry_error = vmc_error::singular;
        return false;
    }
    const float h = std::sqrt(h_sq);
    const float len = L1 * cs + h;
    const float k = -len * L1 * sn / (2.0f * h);
    if (len <= min_len || std::fabs(k) <= min_k)
    {
        state_.geometry_error = vmc_error::singular;
        return false;
    }

    // Preserve the old solver's domain, including its atan2 half-angle chart
    // exclusion |A+C|<1e-6 m^2; no u2/u3 or wheel coordinates are needed.
    const float db = 2.0f * L1 * sn;
    const float a_plus_c = 2.0f * L2 * db * std::sin(phi) + db * db;
    const float sin32 = db * h / (L2 * L2);
    const float sin12_abs = len * sn / L2;
    if (std::fabs(a_plus_c) < 1.0e-6f || sin32 < 1.0e-5f || sin12_abs < 1.0e-5f)
    {
        state_.geometry_error = vmc_error::legacy_domain;
        return false;
    }

    state_.len = len;
    state_.phi = phi;
    state_.J = {k, -k, 0.5f, 0.5f};
    state_.JT = {k, 0.5f, -k, 0.5f};
    state_.JT_inv = {0.5f / k, -0.5f / k, 1.0f, 1.0f};
    if (!std::isfinite(len) || !std::isfinite(phi) || !std::isfinite(k) ||
        !std::isfinite(state_.JT_inv[0]))
        return false;

    static_cast<spring_result&>(state_) = solve_spring(len);
    state_.valid = state_.spring_valid;
    state_.geometry_error = state_.valid ? vmc_error::none : vmc_error::spring;
    return state_.valid;
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
