#pragma once

#include "attitude.hpp"
#include "lqr.hpp"
#include "leg_lqr.hpp"
#include "model/chassis_model.hpp"
#include "odometry.hpp"
#include "model/pendulum.hpp"
#include "pid.hpp"
#include "robot_debug.hpp"
#include "robot_types.hpp"
#include "ramp.hpp"
#include "timer.hpp"

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
        const lqr_state& obs;
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

    struct step_out
    {
        chassis_fsm fsm_next = chassis_fsm::relax;
        bool recovered = false;
        bool v_cmd_enabled = false;
        lqr_mode mode = lqr_mode::low;
        lqr_state ref{};
        chassis_output output{};
        control_debug debug{};
    };

    struct contact_state
    {
        timing::timer off_timer{model::offground_confirm_count};
        timing::timer touch_timer{model::touchdown_confirm_count};
        float dlen_peak = 0.0f;
        bool air_seen = false;
        bool touchdown = false;
    };

    float len_control(control::pid& pid, const leg_state& leg, float ref,
                      pid_debug& debug) noexcept;
    float update_phi_control(phi_pid_control& ctrl, const leg_state& leg,
                             float total_phi, float target, float step,
                             bool positive, pid_debug& debug) noexcept;
    void apply_tuning() noexcept;
    void reset_phi_control() noexcept;
    void relax_left() noexcept;
    void relax_right() noexcept;
    void run_state(chassis_fsm executed, const inputs& in,
                   step_out& out) noexcept;
    void run_relax(const inputs& in, step_out& out) noexcept;
    void run_recover(const inputs& in, step_out& out) noexcept;
    void run_flatten(const inputs& in, step_out& out) noexcept;
    void run_neutral(const inputs& in, step_out& out) noexcept;
    void run_normal(const inputs& in, step_out& out) noexcept;
    void run_spin(const inputs& in, step_out& out) noexcept;
    void run_airborne(const inputs& in, step_out& out) noexcept;
    void update_leg_pid(const inputs& in, step_out& out) noexcept;
    void update_leg_lqr(const inputs& in, step_out& out) noexcept;
    void update_balance(const inputs& in, step_out& out,
                        lqr_mode mode, float alpha_trim) noexcept;
    offground_state is_offground(const leg_state& left, const leg_state& right) const noexcept;

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
    contact_state l_contact_{}; //TODO: 有修改的空间，这个用于记录腿的接触状态
    contact_state r_contact_{};
};

/**
 * @brief 双腿离地状态判断
 * 
 * @param left 左腿状态
 * @param right 右腿状态
 */
inline offground_state controller::is_offground(const leg_state& left, const leg_state& right) const noexcept
{
    if (left.offground && right.offground)
        return offground_state::both_off_ground;
    if (left.offground)
        return offground_state::left_off_ground;
    if (right.offground)
        return offground_state::right_off_ground;
    return offground_state::on_ground;
}


} // namespace chassis
