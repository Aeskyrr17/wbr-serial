#pragma once

#include "lqr.hpp"
#include "odometry.hpp"
#include "pendulum.hpp"
#include "pid.hpp"
#include "robot_debug.hpp"
#include "robot_types.hpp"
#include "slope.hpp"

#include <array>

namespace robot::chassis
{

struct input
{
    chassis_command cmd{};
    leg_input left{};
    leg_input right{};
    float quaternion[4]{1.0f, 0.0f, 0.0f, 0.0f};
    float accel[3]{};
    float pitch = 0.0f;
    float roll = 0.0f;
    float yaw = 0.0f;
    float dpitch = 0.0f;
    float droll = 0.0f;
    float dyaw = 0.0f;
    float l_wheel_velocity = 0.0f;
    float r_wheel_velocity = 0.0f;
    bool imu_valid = false;
    bool wheels_online = false;
};

class controller
{
public:
    controller();
    chassis_output step(const input& in, robot_state& published) noexcept;

private:
    float len_control(control::pid& pid, const leg_state& leg, float ref) noexcept;
    float phi_control(control::pid& pid, control::slope& ramp, const leg_state& leg,
                      float total_phi, float target, float step, bool positive,
                      bool& initialized) noexcept;
    void apply_tuning() noexcept;
    void reset_phi() noexcept;

    pendulum left_{false};
    pendulum right_{true};
    odometry odom_{};
    lqr lqr_{};
    control::pid l_len_{5000.0f, 0.0f, -1500.0f, 125.0f, 0.0f, control::pid_mode::delta};
    control::pid r_len_{5000.0f, 0.0f, -1500.0f, 125.0f, 0.0f, control::pid_mode::delta};
    control::pid roll_{0.5f, 0.0f, 0.0f, 0.05f, 0.005f};
    control::pid l_phi_{20.0f, 0.0f, 10.0f, 40.0f, 0.005f};
    control::pid r_phi_{20.0f, 0.0f, 10.0f, 40.0f, 0.005f};
    control::slope len_ramp_{0.16f, 0.0002f};
    control::slope l_phi_ramp_{};
    control::slope r_phi_ramp_{};
    chassis_state state_ = chassis_state::relax;
    jump_stage jump_ = jump_stage::dont;
    action_events consumed_events_{};
    float target_len_ = 0.16f;
    float maintain_yaw_ = 0.0f;
    std::uint32_t airprotect_count_ = 0;
    std::uint32_t landing_count_ = 0;
    std::uint32_t jumpair_count_ = 0;
    bool flying_ = false;
    bool stairing_ = false;
    bool l_phi_initialized_ = false;
    bool r_phi_initialized_ = false;
    bool running_gains_ = false;
};

} // namespace robot::chassis
