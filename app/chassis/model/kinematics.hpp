#pragma once

#include <cmath>

namespace wbr_generated
{
struct geometry_result { float u2; bool valid; };
inline geometry_result solve_geometry(float phi1, float phi4, float L1, float L2) noexcept
{
    const float x = L1 * (std::cos(phi4) - std::cos(phi1));
    const float y = L1 * (std::sin(phi4) - std::sin(phi1));
    const float a = 2.0f * L2 * x;
    const float b = 2.0f * L2 * y;
    const float c = x * x + y * y;
    const float disc = a * a + b * b - c * c;
    if (disc <= 0.0f || std::fabs(a + c) < 1.0e-6f) return {0.0f, false};
    return {2.0f * std::atan2(b + std::sqrt(disc), a + c), true};
}
} // namespace wbr_generated
