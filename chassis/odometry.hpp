#pragma once

#include "attitude.hpp"
#include "cfg.hpp"

#include <array>
#include <cmath>

namespace chassis
{

class odometry
{
public:
    void update(const ::imu::attitude::state& imu, float wheel_velocity) noexcept;
    void reset() noexcept;
    [[nodiscard]] float x() const noexcept { return state_[0]; }
    [[nodiscard]] float v() const noexcept { return state_[1]; }
    [[nodiscard]] float az() const noexcept { return az_; }

private:
    inline static constexpr float dt = cfg::control_dt;
    inline static constexpr float F[9] = {1.0f, dt, 0.5e-6f, 0.0f, 1.0f, dt, 0.0f, 0.0f, 1.0f};
    // Compatibility baseline: Q[0,0] intentionally preserves legacy dt^3/20.
    inline static constexpr float Q[9] = {2.5e-10f, 6.25e-13f, 8.333333e-10f,
                                          6.25e-13f, 1.666667e-9f, 2.5e-6f,
                                          8.333333e-10f, 2.5e-6f, 0.005f};

    static void multiply3(const float a[9], const float b[9], float result[9]) noexcept
    {
        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
            {
                result[r * 3 + c] = a[r * 3] * b[c] + a[r * 3 + 1] * b[3 + c] +
                                    a[r * 3 + 2] * b[6 + c];
            }
        }
    }

    std::array<float, 3> state_{};
    std::array<float, 9> covariance_{10.0f, 0.0f, 0.0f,
                                     0.0f, 10.0f, 0.0f,
                                     0.0f, 0.0f, 10.0f};
    float az_ = 0.0f;
};

inline void odometry::update(const ::imu::attitude::state& imu, float wheel_velocity) noexcept
{
    const auto& q = imu.quaternion;
    const float w = q.w, x = q.x, y = q.y, z = q.z;
    const float r00 = 1.0f - 2.0f * (y * y + z * z);
    const float r01 = 2.0f * (x * y - z * w);
    const float r02 = 2.0f * (x * z + y * w);
    const float r10 = 2.0f * (x * y + z * w);
    const float r11 = 1.0f - 2.0f * (x * x + z * z);
    const float r12 = 2.0f * (y * z - x * w);
    const float r20 = 2.0f * (x * z - y * w);
    const float r21 = 2.0f * (y * z + x * w);
    const float r22 = 1.0f - 2.0f * (x * x + y * y);
    const float awx = r00 * imu.accel.x + r01 * imu.accel.y + r02 * imu.accel.z;
    const float awy = r10 * imu.accel.x + r11 * imu.accel.y + r12 * imu.accel.z;
    az_ = r20 * imu.accel.x + r21 * imu.accel.y + r22 * imu.accel.z;
    const float ax = awx * std::cos(imu.total_yaw) + awy * std::sin(imu.total_yaw);

    float predicted[3] = {F[0] * state_[0] + F[1] * state_[1] + F[2] * state_[2],
                          state_[1] + dt * state_[2], state_[2]};
    float fp[9]{}, ft[9]{}, predicted_p[9]{};
    multiply3(F, covariance_.data(), fp);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            ft[r * 3 + c] = F[c * 3 + r];
    multiply3(fp, ft, predicted_p);
    for (int i = 0; i < 9; ++i) predicted_p[i] += Q[i];

    // H selects velocity and acceleration. Invert the 2x2 innovation covariance.
    const float s00 = predicted_p[4] + 0.1f;
    const float s01 = predicted_p[5];
    const float s10 = predicted_p[7];
    const float s11 = predicted_p[8] + 50.0f;
    const float determinant = s00 * s11 - s01 * s10;
    if (std::fabs(determinant) < 1.0e-12f) return;
    const float inv00 = s11 / determinant, inv01 = -s01 / determinant;
    const float inv10 = -s10 / determinant, inv11 = s00 / determinant;
    float gain[6]{};
    for (int r = 0; r < 3; ++r)
    {
        gain[r * 2] = predicted_p[r * 3 + 1] * inv00 + predicted_p[r * 3 + 2] * inv10;
        gain[r * 2 + 1] = predicted_p[r * 3 + 1] * inv01 + predicted_p[r * 3 + 2] * inv11;
    }
    const float ev = wheel_velocity - predicted[1], ea = ax - predicted[2];
    for (int r = 0; r < 3; ++r)
        state_[r] = predicted[r] + gain[r * 2] * ev + gain[r * 2 + 1] * ea;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            covariance_[r * 3 + c] = predicted_p[r * 3 + c] -
                gain[r * 2] * predicted_p[3 + c] - gain[r * 2 + 1] * predicted_p[6 + c];
}

inline void odometry::reset() noexcept
{
    state_ = {};
    covariance_ = {10.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f, 0.0f, 10.0f};
    az_ = 0.0f;
}

} // namespace chassis
