#pragma once

#include "attitude.hpp"
#include "lqr.hpp"
#include "leg_lqr.hpp"
#include "odometry.hpp"
#include "model/pendulum.hpp"
#include "pid.hpp"
#include "robot_debug.hpp"
#include "robot_types.hpp"
#include "ramp.hpp"

#include <array>

namespace chassis
{

class controller
{
public:
    controller();
    chassis_output step(const ::imu::attitude::state& imu, const motor_fdb& fdb,
                        const chassis_cmd& cmd, std::uint32_t now,
                        chassis_state& published) noexcept;

private:
    // Borrowed only during step(). Each state can use the full sampled input.
    struct inputs
    {
        const ::imu::attitude::state& imu;
        const motor_fdb& fdb;
        const chassis_cmd& cmd;
        const leg_state& left;
        const leg_state& right;
        const std::array<float, 10>& obs;
        bool valid;
    };

    // The continuous-angle target must survive until the next phi reset.
    struct phi_pid_control
    {
        phi_pid_control() noexcept;
        control::pid pid;
        math::ramp ramp{};
        float target = 0.0f;
        bool initialized = false;
    };

    struct step_result
    {
        chassis_fsm next = chassis_fsm::relax;
        lqr_mode mode = lqr_mode::low;
        std::array<float, 10> ref{};
        chassis_output output{};
    };

    float len_control(control::pid& pid, const leg_state& leg, float ref) noexcept;
    float update_phi_control(phi_pid_control& ctrl, const leg_state& leg,
                             float total_phi, float target, float step,
                             bool positive) noexcept;
    void apply_tuning() noexcept;
    void reset_phi_control() noexcept;
    void relax_left() noexcept;
    void relax_right() noexcept;
    void run_state(chassis_fsm executed, const inputs& in,
                   step_result& result) noexcept;
    void run_relax(const inputs& in, step_result& result) noexcept;
    void run_recover(const inputs& in, step_result& result) noexcept;
    void run_flatten(const inputs& in, step_result& result) noexcept;
    void run_neutral(const inputs& in, step_result& result) noexcept;
    void run_normal(const inputs& in, step_result& result) noexcept;
    void run_spin(const inputs& in, step_result& result) noexcept;
    void update_leg_pid(const inputs& in, step_result& result) noexcept;
    void update_balance(const inputs& in, step_result& result,
                        lqr_mode mode, float alpha_trim) noexcept;

    pendulum left_{false};
    pendulum right_{true};
    odometry odom_{};
    lqr lqr_{};
    leg_lqr leg_lqr_{};
    control::pid l_len_;
    control::pid r_len_;
    control::pid roll_;
    phi_pid_control l_phi_{};
    phi_pid_control r_phi_{};
    math::ramp len_ramp_{0.16f, 0.0002f};
    chassis_fsm fsm_ = chassis_fsm::relax;
    float target_len_ = 0.16f;
    std::uint32_t settle_count_ = 0;
    bool recovered_ = false;
    bool v_cmd_enabled_ = false;
};

} // namespace chassis
