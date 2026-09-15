#pragma once

#include <array>

namespace robot::chassis
{

class odometry
{
public:
    void update(const float quaternion[4], const float accel[3],
                float wheel_velocity, float yaw) noexcept;
    void reset() noexcept;
    [[nodiscard]] float x() const noexcept { return state_[0]; }
    [[nodiscard]] float v() const noexcept { return state_[1]; }
    [[nodiscard]] float az() const noexcept { return az_; }

private:
    std::array<float, 3> state_{};
    std::array<float, 9> covariance_{10.0f, 0.0f, 0.0f,
                                     0.0f, 10.0f, 0.0f,
                                     0.0f, 0.0f, 10.0f};
    float az_ = 0.0f;
};

} // namespace robot::chassis
