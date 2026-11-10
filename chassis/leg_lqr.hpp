#pragma once

#include "attitude.hpp"
#include "robot_types.hpp"
#include "model/chassis_model.hpp"
#include "model/leg_lqr_coeffs.hpp"


#include <array>
#include <cmath>

namespace chassis
{
class leg_lqr
{
public:
    // State order: roll (rad), droll (rad/s), H (m), dH (m/s).
    static std::array<float, 4> observe(const ::imu::attitude::state& imu,
                                      const leg_state& left, const leg_state& right) noexcept;
    static std::array<float, 4> reference(float length, float length_rate, float alpha_trim) noexcept;
    // Total support, including gravity and passive spring; left/right, N.
    std::array<float, 2> update(const std::array<float, 4>& obs,
                               const std::array<float, 4>& ref) const noexcept;
};
inline std::array<float, 4> leg_lqr::observe(const ::imu::attitude::state& imu,
                                    const leg_state& left, const leg_state& right) noexcept
{
    // Flat ground, both wheels contacting: H is the midpoint height of the hips.
    const float projection = 0.5f * (left.len * std::cos(left.alpha) +
                                      right.len * std::cos(right.alpha));
    const float projection_rate = 0.5f * (
        left.dlen * std::cos(left.alpha) - left.len * std::sin(left.alpha) * left.dalpha +
        right.dlen * std::cos(right.alpha) - right.len * std::sin(right.alpha) * right.dalpha);
    const float height = model::wheel_radius + std::cos(imu.roll) * projection;
    const float height_rate = std::cos(imu.roll) * projection_rate -
                              std::sin(imu.roll) * imu.gyro.x * projection;
    return {imu.roll, imu.gyro.x, height, height_rate};
}

inline std::array<float, 4> leg_lqr::reference(float length, float length_rate, float alpha_trim) noexcept
{
    // Preserve the existing length command: convert at the balance LQR's nominal leg angle.
    const float alpha = model::alpha_eq[0] + model::alpha_eq[1] * length +
                        model::alpha_eq[2] * length * length + alpha_trim;
    const float dalpha_dlength = model::alpha_eq[1] + 2.0f * model::alpha_eq[2] * length;
    const float height = model::wheel_radius + length * std::cos(alpha);
    const float height_rate = (std::cos(alpha) - length * std::sin(alpha) * dalpha_dlength) * length_rate;
    return {0.0f, 0.0f, height, height_rate};
}

inline std::array<float, 2> leg_lqr::update(const std::array<float, 4>& obs,
                                   const std::array<float, 4>& ref) const noexcept
{
    std::array<float, 2> output{leg_lqr_model::equilibrium_force, leg_lqr_model::equilibrium_force};
    for (unsigned i = 0; i < output.size(); ++i)
        for (unsigned j = 0; j < obs.size(); ++j)
            output[i] += leg_lqr_model::gain[i][j] * (obs[j] - ref[j]);
    return output;
}
} // namespace chassis
