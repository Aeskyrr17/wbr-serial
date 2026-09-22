#pragma once

#include "imu.hpp"
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

class controller
{
public:
    controller();
    chassis_output step(const ::imu::state& imu, const motor_feedback_snapshot& fdb,
                        const chassis_command& cmd, std::uint32_t now,
                        robot_state& published) noexcept;

private:
    // Borrowed only during step(). Each state can use the full sampled input.
    struct state_context
    {
        const ::imu::state& imu;
        const motor_feedback_snapshot& fdb;
        const chassis_command& cmd;
        const leg_state& left;
        const leg_state& right;
        const std::array<float, 10>& obs;
        bool valid;
    };

    // The continuous-angle target must survive until the next phi reset.
    struct phi_control_state
    {
        control::pid pid{20.0f, 0.0f, 10.0f, 40.0f, 0.005f};
        control::slope ramp{};
        float target = 0.0f;
        bool initialized = false;
    };

    struct state_cycle
    {
        chassis_state next = chassis_state::relax;
        lqr_mode mode = lqr_mode::low;
        std::array<float, 10> ref{};
        chassis_output out{};
    };

    float len_control(control::pid& pid, const leg_state& leg, float ref) noexcept;
    float update_phi_control(phi_control_state& ctrl, const leg_state& leg,
                             float total_phi, float target, float step,
                             bool positive) noexcept;
    void apply_tuning() noexcept;
    void reset_phi_control() noexcept;
    void relax_left() noexcept;
    void relax_right() noexcept;
    void relax_legs() noexcept;
    void run_state(chassis_state executed, const state_context& in,
                   state_cycle& cycle) noexcept;
    void run_relax(const state_context& in, state_cycle& cycle) noexcept;
    void run_flatten(const state_context& in, state_cycle& cycle) noexcept;
    void run_neutral(const state_context& in, state_cycle& cycle) noexcept;
    void run_normal(const state_context& in, state_cycle& cycle) noexcept;
    void run_spin(const state_context& in, state_cycle& cycle) noexcept;
    void update_balance(const state_context& in, state_cycle& cycle,
                        lqr_mode mode, float alpha_trim) noexcept;

    pendulum left_{false};
    pendulum right_{true};
    odometry odom_{};
    lqr lqr_{};
    control::pid l_len_{5000.0f, 0.0f, -1500.0f, 125.0f, 0.0f, control::pid_mode::delta};
    control::pid r_len_{5000.0f, 0.0f, -1500.0f, 125.0f, 0.0f, control::pid_mode::delta};
    control::pid roll_{0.5f, 0.0f, 0.0f, 0.05f, 0.005f};
    phi_control_state l_phi_{};
    phi_control_state r_phi_{};
    control::slope len_ramp_{0.16f, 0.0002f};
    chassis_state state_ = chassis_state::relax;
    float target_len_ = 0.16f;
    std::uint32_t settle_count_ = 0;
    bool recovered_ = false;
    bool v_cmd_enabled_ = false;
};

} // namespace robot::chassis
