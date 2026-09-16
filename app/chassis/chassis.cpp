#include "chassis.hpp"

#include "chassis_model.hpp"

#include <algorithm>
#include <cmath>

namespace robot::chassis
{
namespace
{
void tune_pid(control::pid& pid, const pid_tuning& tuning) noexcept
{
    pid.tune(tuning.kp, tuning.ki, tuning.kd);
    pid.max_out = tuning.max_out;
    pid.max_iout = tuning.max_iout;
}
}

controller::controller() = default;

void controller::apply_tuning() noexcept
{
    const auto& len = running_gains_ ? robot_tuning.len_run : robot_tuning.len_startup;
    tune_pid(l_len_, len);
    tune_pid(r_len_, len);
    tune_pid(roll_, robot_tuning.roll);
}

float controller::len_control(control::pid& pid, const leg_state& leg, float ref) noexcept
{
    pid.ref = ref;
    pid.fdb = leg.len;
    pid.update(leg.dlen);
    return pid.result;
}

float controller::update_phi_control(phi_control_state& ctrl, const leg_state& leg,
                                     float total_phi, float target, float step,
                                     bool positive) noexcept
{
    if (!ctrl.initialized)
    {
        ctrl.ramp.set_value(total_phi);
        ctrl.ramp.set_step(step);
        while (target - total_phi > model::pi) target -= 2.0f * model::pi;
        while (target - total_phi < -model::pi) target += 2.0f * model::pi;
        if (positive && target < total_phi) target += 2.0f * model::pi;
        if (!positive && target > total_phi) target -= 2.0f * model::pi;
        ctrl.target = target;
        ctrl.initialized = true;
    }

    // Preserve the chosen turn direction; do not revert to the wrapped target.
    ctrl.pid.ref = ctrl.ramp.update(ctrl.target);
    ctrl.pid.fdb = total_phi;
    ctrl.pid.update(leg.dphi); // POSITION compatibility: velocity is intentionally unused.
    return ctrl.pid.result;
}

void controller::reset_phi_control() noexcept
{
    l_phi_.initialized = false;
    r_phi_.initialized = false;
}

void controller::relax_left() noexcept
{
    l_len_.clear();
    l_phi_.pid.clear();
    l_phi_.initialized = false;
    left_.reset_tracking();
}

void controller::relax_right() noexcept
{
    r_len_.clear();
    r_phi_.pid.clear();
    r_phi_.initialized = false;
    right_.reset_tracking();
}

void controller::relax_legs() noexcept
{
    relax_left();
    relax_right();
}

void controller::run_relax(const state_context& in, state_cycle& cycle) noexcept
{
    const auto& left = in.left;

    relax_legs();
    odom_.reset();
    target_len_ = model::Lmin;
    len_ramp_.set_value(left.len);
    recovered_ = in.imu.accel_z >= 0.0f;
    v_cmd_enabled_ = false;
    if (in.valid && in.cmd.move)
        cycle.next = in.imu.accel_z < 0.0f ? chassis_state::recover : chassis_state::flatten;
}

void controller::run_recover(const state_context& in, state_cycle& cycle) noexcept
{
    const auto& left = in.left;
    const auto& right = in.right;

    odom_.reset();
    tune_pid(l_phi_.pid, robot_tuning.phi_recover);
    tune_pid(r_phi_.pid, robot_tuning.phi_recover);
    if (in.imu.accel_z > 5.0f && std::fabs(in.imu.pitch) < 0.35f && std::fabs(in.imu.gyro_p) < 1.0f)
    {
        recovered_ = true;
        relax_legs();
        cycle.next = chassis_state::flatten;
    }
    else if (in.imu.pitch > 0.0f)
    {
        cycle.out.Tpl = update_phi_control(
            l_phi_, left, left_.total_phi(), 0.0f,
            0.005f, true);
        cycle.out.Tpr = update_phi_control(
            r_phi_, right, right_.total_phi(), 0.0f,
            0.005f, true);
        if (std::fabs(left.phi) < 0.1f && std::fabs(right.phi) < 0.1f)
        {
            cycle.out.Tpl = -35.0f;
            cycle.out.Tpr = 35.0f;
        }
    }
    else
    {
        cycle.out.Tpl = update_phi_control(
            l_phi_, left, left_.total_phi(), model::pi,
            0.005f, false);
        cycle.out.Tpr = update_phi_control(
            r_phi_, right, right_.total_phi(), model::pi,
            0.005f, false);
        if (std::fabs(left.phi - model::pi) < 0.1f &&
            std::fabs(right.phi - model::pi) < 0.1f)
        {
            cycle.out.Tpl = 35.0f;
            cycle.out.Tpr = -35.0f;
        }
    }
}

void controller::run_flatten(const state_context& in, state_cycle& cycle) noexcept
{
    const auto& left = in.left;
    const auto& right = in.right;

    recovered_ = true;
    v_cmd_enabled_ = true;
    if ((left.phi >= 0.7f && left.phi <= 1.4f) &&
        (right.phi >= 0.7f && right.phi <= 1.4f))
    {
        cycle.next = chassis_state::neutral;
        return;
    }

    if (left.flat)
    {
        relax_left();
    }
    else
    {
        tune_pid(l_phi_.pid, left.phi >= model::pi * 0.4f ? robot_tuning.phi_flatten_fast
                                                       : robot_tuning.phi_flatten_slow);
        cycle.out.Tpl = update_phi_control(
            l_phi_, left, left_.total_phi(), model::pi,
            left.phi >= model::pi * 0.4f ? 0.01f : 0.002f,
            left.phi >= model::pi * 0.4f);
    }
    if (right.flat)
    {
        relax_right();
    }
    else
    {
        tune_pid(r_phi_.pid, right.phi >= model::pi * 0.4f ? robot_tuning.phi_flatten_fast
                                                        : robot_tuning.phi_flatten_slow);
        cycle.out.Tpr = update_phi_control(
            r_phi_, right, right_.total_phi(), model::pi,
            right.phi >= model::pi * 0.4f ? 0.01f : 0.002f,
            right.phi >= model::pi * 0.4f);
    }
    cycle.out.Twl = cycle.out.Twr = in.cmd.v > 0.0f ? 1.5f : 0.0f;
    if (left.flat && right.flat)
    {
        odom_.reset();
        cycle.next = chassis_state::neutral;
    }
}

void controller::run_neutral(const state_context& in, state_cycle& cycle) noexcept
{
    const auto& left = in.left;
    const auto& right = in.right;

    odom_.reset();
    tune_pid(l_phi_.pid, robot_tuning.phi_neutral);
    tune_pid(r_phi_.pid, robot_tuning.phi_neutral);
    cycle.out.Fl = len_control(l_len_, left, left.len + (model::Lmin - left.len) * 0.6f) -
                    left.fs;
    cycle.out.Fr = len_control(r_len_, right, right.len + (model::Lmin - right.len) * 0.6f) -
                    right.fs;
    cycle.out.Tpl = update_phi_control(l_phi_, left, left_.total_phi(),
                                0.5f * model::pi, 0.003f,
                                left.phi < model::pi * 0.45f);
    cycle.out.Tpr = update_phi_control(r_phi_, right, right_.total_phi(),
                                0.5f * model::pi, 0.003f,
                                right.phi < model::pi * 0.45f);
    // Deterministic fix: the deadband explicitly commands zero wheel torque.
    if (in.cmd.v > 0.005f)
        cycle.out.Twl = cycle.out.Twr = 1.5f;
    else if (in.cmd.v < -0.005f)
        cycle.out.Twl = cycle.out.Twr = -1.5f;
    else
        cycle.out.Twl = cycle.out.Twr = 0.0f;

    if (left.neutral && right.neutral)
    {
        reset_phi_control();
        target_len_ = model::Lmin;
        len_ramp_.set_value(left.len);
        cycle.next = chassis_state::normal;
    }
}

void controller::run_normal(const state_context& in, state_cycle& cycle) noexcept
{
    const auto& left = in.left;
    const auto& right = in.right;
    const auto& obs = in.obs;

    cycle.ref = {in.cmd.x, in.cmd.v, in.cmd.yaw, in.cmd.dyaw,
                 left.alpha_eq + 0.03f, 0.0f, right.alpha_eq + 0.03f, 0.0f, 0.0f, 0.0f};
    cycle.mode = (left.len + right.len) * 0.5f > model::Lswitch
                     ? lqr_mode::high
                     : lqr_mode::low;
    const auto torque = lqr_.update(cycle.mode, left.len, right.len, obs, cycle.ref);
    cycle.out.Twl = torque[0];
    cycle.out.Twr = torque[1];
    cycle.out.Tpl = torque[2];
    cycle.out.Tpr = torque[3];
    roll_.ref = 0.0f;
    roll_.fdb = in.imu.roll;
    roll_.update(in.imu.gyro_r);

    if (in.cmd.events.stair != consumed_events_.stair)
    {
        ++consumed_events_.stair;
        stairing_ = !stairing_;
    }
    if (in.cmd.events.fly != consumed_events_.fly)
    {
        ++consumed_events_.fly;
        flying_ = !flying_;
    }
    if (stairing_)
    {
        target_len_ = model::Lmax;
        if (left.alpha - left.alpha_eq > 0.2f && right.alpha - right.alpha_eq > 0.2f)
        {
            reset_phi_control();
            cycle.next = chassis_state::gostair;
            stairing_ = false;
            airprotect_count_ = 0;
        }
    }
    if (in.cmd.events.jump != consumed_events_.jump)
    {
        ++consumed_events_.jump;
        cycle.next = chassis_state::jump;
        jump_ = jump_stage::extending;
        jumpair_count_ = airprotect_count_ = 0;
        maintain_yaw_ = in.imu.total_yaw;
    }
    if (in.cmd.spin) cycle.next = chassis_state::spin;
    if (!stairing_)
        target_len_ = flying_ ? model::Lfly : std::clamp(in.cmd.len, model::Lmin, model::Lmax);
    const float cmd_len = len_ramp_.update(target_len_);
    cycle.out.Fl = len_control(l_len_, left, cmd_len + roll_.result) +
                    model::gravity_feedforward - left.fs;
    cycle.out.Fr = len_control(r_len_, right, cmd_len - roll_.result) +
                    model::gravity_feedforward - right.fs;
    ++airprotect_count_;
    v_cmd_enabled_ = airprotect_count_ >= 500U;
    if (v_cmd_enabled_) len_ramp_.set_step(0.00027f);
    if (left.normal_force + right.normal_force < 0.0f && left.dlen > 0.05f &&
        right.dlen > 0.05f && airprotect_count_ > 500U)
        cycle.next = chassis_state::offground;
    // Legacy final assignment priority: fall protection overrides all earlier transitions.
    if (left.alpha >= 0.8f || right.alpha >= 0.8f)
    {
        reset_phi_control();
        cycle.next = chassis_state::relax;
    }
    running_gains_ = true;
}

void controller::run_offground(const state_context& in, state_cycle& cycle) noexcept
{
    const auto& left = in.left;
    const auto& right = in.right;
    const auto& obs = in.obs;

    cycle.ref = {obs[0], obs[1], obs[2], obs[3], in.imu.pitch, 0.0f,
                 in.imu.pitch, 0.0f, obs[8], obs[9]};
    const auto torque = lqr_.update(lqr_mode::low, left.len, right.len, obs, cycle.ref);
    cycle.out.Tpl = torque[2] * 0.4f;
    cycle.out.Tpr = torque[3] * 0.4f;
    ++landing_count_;
    const bool landing = flying_
                             ? (left.dlen < -0.05f && right.dlen < -0.05f &&
                                landing_count_ > 100U)
                             : (left.dlen < 0.0f && right.dlen < 0.0f);
    if (left.alpha >= 0.8f || right.alpha >= 0.8f)
    {
        reset_phi_control();
        cycle.next = chassis_state::relax;
        return;
    }
    if (landing)
    {
        landing_count_ = airprotect_count_ = 0;
        cycle.next = chassis_state::normal;
        target_len_ = model::Lmin;
        len_ramp_.set_value(left.len);
        len_ramp_.set_step(flying_ ? 0.0006f : 0.0009f);
        flying_ = false;
    }
}

void controller::run_spin(const state_context& in, state_cycle& cycle) noexcept
{
    const auto& left = in.left;
    const auto& right = in.right;
    const auto& obs = in.obs;

    cycle.ref = {in.cmd.x, in.cmd.v, in.cmd.yaw, in.cmd.dyaw,
                 left.alpha_eq - 0.03f, 0.0f, right.alpha_eq - 0.03f, 0.0f, 0.0f, 0.0f};
    cycle.mode = lqr_mode::spin;
    const auto torque = lqr_.update(cycle.mode, left.len, right.len, obs, cycle.ref);
    cycle.out.Twl = torque[0];
    cycle.out.Twr = torque[1];
    cycle.out.Tpl = torque[2];
    cycle.out.Tpr = torque[3];
    roll_.ref = 0.0f;
    roll_.fdb = in.imu.roll;
    roll_.update(in.imu.gyro_r);
    len_ramp_.set_step(model::spin_len_amp * model::spin_len_omega * 0.001f * 1.2f);
    const float cmd_len = len_ramp_.update(in.cmd.len);
    cycle.out.Fl = len_control(l_len_, left, cmd_len + roll_.result) +
                    model::gravity_feedforward - left.fs;
    cycle.out.Fr = len_control(r_len_, right, cmd_len - roll_.result) +
                    model::gravity_feedforward - right.fs;
    if (!in.cmd.spin) cycle.next = chassis_state::normal;
}

void controller::run_gostair(const state_context& in, state_cycle& cycle) noexcept
{
    const auto& left = in.left;
    const auto& right = in.right;

    tune_pid(l_phi_.pid, robot_tuning.phi_stair);
    tune_pid(r_phi_.pid, robot_tuning.phi_stair);
    cycle.out.Tpl = update_phi_control(l_phi_, left, left_.total_phi(), model::pi,
                                0.01f, true);
    cycle.out.Tpr = update_phi_control(r_phi_, right, right_.total_phi(), model::pi,
                                0.01f, true);
    if (left.flat && right.flat)
    {
        reset_phi_control();
        cycle.next = chassis_state::neutral;
    }
}

void controller::run_jump(const state_context& in, state_cycle& cycle) noexcept
{
    const auto& left = in.left;
    const auto& right = in.right;
    const auto& obs = in.obs;

    if (jump_ == jump_stage::dont)
    {
        cycle.next = chassis_state::normal;
    }
    else if (jump_ == jump_stage::extending)
    {
        cycle.ref = {0.0f, 0.0f, maintain_yaw_, 0.0f, left.alpha_eq, 0.0f,
                     right.alpha_eq, 0.0f, 0.0f, 0.0f};
        const auto torque = lqr_.update(lqr_mode::low, left.len, right.len, obs, cycle.ref);
        cycle.out.Twl = torque[0];
        cycle.out.Twr = torque[1];
        cycle.out.Tpl = torque[2];
        cycle.out.Tpr = torque[3];
        cycle.out.Fl = cycle.out.Fr = 700.0f;
        if (left.len >= 0.3f && right.len >= 0.3f)
        {
            jumpair_count_ = 0;
            jump_ = jump_stage::inair;
        }
    }
    else
    {
        odom_.reset();
        ++jumpair_count_;
        cycle.ref = {obs[0], obs[1], obs[2], obs[3], left.alpha_eq, 0.0f,
                     right.alpha_eq, 0.0f, 0.0f, 0.0f};
        const auto torque = lqr_.update(lqr_mode::low, left.len, right.len, obs, cycle.ref);
        cycle.out.Tpl = torque[2] * 0.4f;
        cycle.out.Tpr = torque[3] * 0.4f;
        cycle.out.Fl = len_control(l_len_, left, model::Lmin) - left.fs;
        cycle.out.Fr = len_control(r_len_, right, model::Lmin) - right.fs;
        if (jumpair_count_ > 500U)
        {
            target_len_ = model::Lmin;
            len_ramp_.set_value(left.len);
            jump_ = jump_stage::dont;
            cycle.next = chassis_state::normal;
        }
    }
}

void controller::run_state(chassis_state executed, const state_context& in,
                           state_cycle& cycle) noexcept
{
    switch (executed)
    {
    case chassis_state::relax:
        run_relax(in, cycle);
        break;
    case chassis_state::recover:
        run_recover(in, cycle);
        break;
    case chassis_state::flatten:
        run_flatten(in, cycle);
        break;
    case chassis_state::neutral:
        run_neutral(in, cycle);
        break;
    case chassis_state::normal:
        run_normal(in, cycle);
        break;
    case chassis_state::offground:
        run_offground(in, cycle);
        break;
    case chassis_state::spin:
        run_spin(in, cycle);
        break;
    case chassis_state::gostair:
        run_gostair(in, cycle);
        break;
    case chassis_state::jump:
        run_jump(in, cycle);
        break;
    }
}

chassis_output controller::step(const ::imu::state& imu, const motor_feedback_snapshot& fdb,
                                const chassis_command& cmd, std::uint32_t now,
                                robot_state& published) noexcept
{
    apply_tuning();
    const float wheel_velocity = (-fdb.l_wheel.velocity + fdb.r_wheel.velocity) *
                                 0.5f * model::wheel_radius;
    odom_.update(imu, wheel_velocity);
    const bool left_valid = left_.update(fdb.l_joint1, fdb.l_joint4, imu, odom_.az());
    const bool right_valid = right_.update(fdb.r_joint1, fdb.r_joint4, imu, odom_.az());
    const auto& left = left_.state();
    const auto& right = right_.state();

    const std::array<float, 10> obs{
        odom_.x(), odom_.v(), imu.total_yaw, imu.gyro_y, left.alpha,
        left.dalpha, right.alpha, right.dalpha, imu.pitch, imu.gyro_p};
    const bool imu_valid = imu.online && imu.sequence != 0U && now - imu.received_tick <= 100U;
    const bool valid = imu_valid && fdb.wheels_online && left_valid && right_valid;
    const state_context in{imu, fdb, cmd, left, right, obs, valid};
    if (!valid || !in.cmd.move) state_ = chassis_state::relax;
    if (valid && in.cmd.move && in.imu.accel_z < 0.0f && state_ != chassis_state::recover)
    {
        if (++flipover_count_ > 1000U)
        {
            reset_phi_control();
            state_ = chassis_state::relax;
            flipover_count_ = 0U;
        }
    }
    else
    {
        flipover_count_ = 0U;
    }

    const chassis_state executed = state_;
    // Legacy TaskPendulum only sampled action pulses in NORMAL. Requests that
    // arrive while another state is executing are intentionally discarded,
    // rather than being replayed after recovery/flatten/jump completes.
    if (executed != chassis_state::normal)
        consumed_events_ = in.cmd.events;

    state_cycle cycle{};
    cycle.next = state_;
    run_state(executed, in, cycle);

    auto& out = cycle.out;
    if (executed != chassis_state::relax)
    {
        const auto lt = left_.joint_torque(cycle.out.Fl, cycle.out.Tpl);
        const auto rt = right_.joint_torque(cycle.out.Fr, cycle.out.Tpr);
        out.l_joint1 = std::clamp(lt[0], -model::max_joint_torque, model::max_joint_torque);
        out.l_joint4 = std::clamp(lt[1], -model::max_joint_torque, model::max_joint_torque);
        out.r_joint1 = std::clamp(rt[0], -model::max_joint_torque, model::max_joint_torque);
        out.r_joint4 = std::clamp(rt[1], -model::max_joint_torque, model::max_joint_torque);
    }
    out.Twl = std::clamp(out.Twl, -model::max_wheel_torque, model::max_wheel_torque);
    out.Twr = std::clamp(out.Twr, -model::max_wheel_torque, model::max_wheel_torque);
    state_ = cycle.next;

    published.x = odom_.x();
    published.v = odom_.v();
    published.len = 0.5f * (left.len + right.len);
    published.lalpha = left.alpha;
    published.ralpha = right.alpha;
    published.pitch = in.imu.pitch;
    published.yaw = in.imu.total_yaw;
    published.flying = flying_;
    published.stairing = stairing_;
    published.spinning = cycle.next == chassis_state::spin;
    published.v_cmd_enabled = v_cmd_enabled_;
    published.recovered = recovered_;
    published.state_executed = executed;
    published.next_state = cycle.next;
    published.jump = jump_;

    robot_debug.chassis.state = published;
    robot_debug.chassis.left = left;
    robot_debug.chassis.right = right;
    robot_debug.chassis.output = out;
    std::copy(obs.begin(), obs.end(), robot_debug.chassis.obs);
    std::copy(cycle.ref.begin(), cycle.ref.end(), robot_debug.chassis.ref);
    robot_debug.chassis.l_phi_ref = l_phi_.pid.ref;
    robot_debug.chassis.r_phi_ref = r_phi_.pid.ref;
    robot_debug.chassis.roll = in.imu.roll;
    robot_debug.chassis.droll = in.imu.gyro_r;
    robot_debug.chassis.normal_force = left.normal_force + right.normal_force;
    robot_debug.chassis.lqr_mode = static_cast<std::uint8_t>(cycle.mode);
    return out;
}

} // namespace robot::chassis
