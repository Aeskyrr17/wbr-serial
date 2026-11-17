#pragma once

#include "attitude.hpp"
#include "cfg.hpp"
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
#include "tuning.hpp"

#include <array>

namespace chassis
{

class controller
{
public:
    controller();
    void step(const ::imu::attitude::state& imu, const motor_fdb& fdb,
              const chassis_cmd& cmd, std::uint32_t now,
              motor_cmd& output, chassis_state& state) noexcept;

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
        bool v_cmd_enabled = false;
        lqr_mode mode = lqr_mode::low;
        lqr_state ref{};
        virtual_force force{};
        motor_cmd motor{};
        control_debug debug{};
    };

    struct contact_state
    {
        timing::timer off_timer{cfg::offground_confirm_count};
        timing::timer touch_timer{cfg::touchdown_confirm_count};
        float dlen_peak = 0.0f;
        bool air_seen = false; //< 是否出现过离地状态。air_seen为true时才会判定touchdown
        bool touchdown = false;//< 是否触地
    };

    float len_pid_control(control::pid& pid, const leg_state& leg, float ref,
                      pid_debug& debug) noexcept;
    float update_phi_control(phi_pid_control& ctrl, const leg_state& leg,
                             float total_phi, float target, float step,
                             bool positive, pid_debug& debug) noexcept;
    void apply_pid_tuning() noexcept;
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
    math::ramp len_ramp_{cfg::Lmin, 0.0002f};
    chassis_fsm fsm_ = chassis_fsm::relax;
    float target_len_ = cfg::Lmin;
    std::uint32_t settle_count_ = 0;
    contact_state l_contact_{};         //<记录左腿的接触状态
    contact_state r_contact_{};         //<记录右腿的接触状态
};

} // namespace chassis
