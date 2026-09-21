#pragma once

#include "chassis_model.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace robot::chassis
{
enum class spring_error { none, leg_geometry, dimensions, domain, branch, singular, nonfinite };

struct spring_result
{
    float spring_len = std::numeric_limits<float>::quiet_NaN();
    float dls_dlen = std::numeric_limits<float>::quiet_NaN();
    float fs = std::numeric_limits<float>::quiet_NaN();
    bool spring_valid = false;
    spring_error error = spring_error::leg_geometry;
};

inline spring_result solve_spring(float len) noexcept
{
    using namespace model;
    spring_result out;
    constexpr float domain_tol = 1.0e-6f;
    constexpr float min_sin_theta = 1.0e-3f;
    if (!std::isfinite(L1) || !std::isfinite(L2) || !std::isfinite(spring_d1) ||
        !std::isfinite(spring_d2) || !std::isfinite(spring_angle))
    { out.error = spring_error::nonfinite; return out; }
    const float d3_sq = L1 * L1 + spring_d1 * spring_d1 -
                        2.0f * L1 * spring_d1 * std::sin(spring_angle);
    if (!(L1 > 0 && L2 > 0 && spring_d1 > 0 && spring_d2 > 0 && d3_sq > 0))
    { out.error = spring_error::dimensions; return out; }
    // OP=d1, BQ=d2, BP=d3; gamma is the legacy mounting angle.
    static const float d3 = std::sqrt(d3_sq);
    static const float cos_alpha = (d3_sq + L1 * L1 - spring_d1 * spring_d1) / (2 * d3 * L1);
    const float cos_theta = (L1 * L1 + L2 * L2 - len * len) / (2 * L1 * L2);
    if (!std::isfinite(len) || !std::isfinite(cos_alpha) ||
        !std::isfinite(cos_theta) || !std::isfinite(spring_force))
    { out.error = spring_error::nonfinite; return out; }
    if (len <= 0 || std::fabs(cos_alpha) > 1 + domain_tol || std::fabs(cos_theta) > 1 + domain_tol)
    { out.error = spring_error::domain; return out; }
    static const float alpha_s = std::acos(std::clamp(cos_alpha, -1.0f, 1.0f));
    const float theta = std::acos(std::clamp(cos_theta, -1.0f, 1.0f));
    const float sin_theta = std::sin(theta);
    if (sin_theta <= min_sin_theta)
    { out.error = spring_error::singular; return out; }
    const float beta_s = theta - alpha_s;
    const float ls_sq = spring_d2 * spring_d2 + d3_sq - 2 * spring_d2 * d3 * std::cos(beta_s);
    if (!std::isfinite(ls_sq) || ls_sq <= 1.0e-12f)
    { out.error = spring_error::singular; return out; }
    out.spring_len = std::sqrt(ls_sq);
    out.dls_dlen = spring_d2 * d3 * len * std::sin(beta_s) /
                  (L1 * L2 * out.spring_len * sin_theta);
    out.fs = spring_force * out.dls_dlen;
    out.spring_valid = std::isfinite(out.spring_len) && std::isfinite(out.dls_dlen) && std::isfinite(out.fs);
    out.error = out.spring_valid ? spring_error::none : spring_error::nonfinite;
    return out;
}
}
