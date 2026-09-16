#pragma once

#include "imu.hpp"
#include "robot_types.hpp"
#include "vmc.hpp"

namespace robot::chassis
{

class pendulum
{
public:
    explicit pendulum(bool reverse) : reverse_(reverse) {}

    bool update(const ::motors::feedback& joint1, const ::motors::feedback& joint4,
                const ::imu::state& imu, float az) noexcept;
    std::array<float, 2> joint_torque(float force, float pitch_torque) const noexcept;
    void reset_tracking() noexcept;

    [[nodiscard]] const leg_state& state() const noexcept { return state_; }
    [[nodiscard]] float total_phi() const noexcept { return total_phi_; }

private:
    vmc vmc_{};
    leg_state state_{};
    float prev_dlen_ = 0.0f;
    float prev_dalpha_ = 0.0f;
    float total_phi_ = 0.0f;
    float prev_phi_ = 0.0f;
    bool phi_initialized_ = false;
    bool reverse_ = false;
};

} // namespace robot::chassis
