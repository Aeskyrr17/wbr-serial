#include "chassis.hpp"

#include "chassis_model.hpp"

#include <algorithm>
#include <cmath>

namespace robot::chassis
{
controller::controller() = default;

void controller::apply_tuning() noexcept
{
    const auto& len = (state_ == chassis_state::normal) ? robot_tuning.len_run : robot_tuning.len_startup;
    l_len_.tune(len);
    r_len_.tune(len);
    roll_.tune(robot_tuning.roll);
}

float controller::len_control(control::pid& pid, const leg_state& leg, float ref) noexcept
{
    pid.ref = ref;
    pid.fdb = leg.len;
    pid.update(leg.dlen);
    auto& debug = (&pid == &l_len_) ? robot_debug.chassis.control.l_len
                                   : robot_debug.chassis.control.r_len;
    debug = {pid.ref, pid.fdb, pid.result, true};
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
    auto& debug = (&ctrl == &l_phi_) ? robot_debug.chassis.control.l_phi
                                    : robot_debug.chassis.control.r_phi;
    debug = {ctrl.pid.ref, ctrl.pid.fdb, ctrl.pid.result, true};
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
    settle_count_ = 0;
    if (in.valid && in.cmd.move && recovered_)
        cycle.next = chassis_state::flatten;
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
        reset_phi_control();
        cycle.next = chassis_state::neutral;
        return;
    }

    if (left.flat)
    {
        relax_left();
    }
    else
    {
        l_phi_.pid.tune(left.phi >= model::pi * 0.4f ? robot_tuning.phi_flatten_fast
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
        r_phi_.pid.tune(right.phi >= model::pi * 0.4f ? robot_tuning.phi_flatten_fast
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
        reset_phi_control();
        cycle.next = chassis_state::neutral;
    }
}

void controller::run_neutral(const state_context& in, state_cycle& cycle) noexcept
{
    const auto& left = in.left;
    const auto& right = in.right;

    odom_.reset();
    l_phi_.pid.tune(robot_tuning.phi_neutral);
    r_phi_.pid.tune(robot_tuning.phi_neutral);
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
                 left.alpha_eq + robot_tuning.alpha_trim, 0.0f,
                 right.alpha_eq + robot_tuning.alpha_trim, 0.0f, 0.0f, 0.0f};
    cycle.mode = (left.len + right.len) * 0.5f > model::Lswitch
                     ? lqr_mode::high
                     : lqr_mode::low;
    const auto torque = lqr_.update(cycle.mode, left.len, right.len, obs, cycle.ref);
    auto& debug = robot_debug.chassis.control;
    std::copy(torque.begin(), torque.end(), debug.lqr_output);
    const auto& raw = lqr_.raw_output();
    std::copy(raw.begin(), raw.end(), debug.lqr_raw);
    debug.lqr_active = true;
    cycle.out.Twl = torque[0];
    cycle.out.Twr = torque[1];
    cycle.out.Tpl = torque[2];
    cycle.out.Tpr = torque[3];
    roll_.ref = 0.0f;
    roll_.fdb = in.imu.roll;
    roll_.update(in.imu.gyro_r);
    debug.roll = {roll_.ref, roll_.fdb, roll_.result, true};

    target_len_ = std::clamp(in.cmd.len, model::Lmin, model::Lmax);
    const float cmd_len = len_ramp_.update(target_len_);
    cycle.out.Fl = len_control(l_len_, left, cmd_len + roll_.result) +
                    model::gravity_feedforward - left.fs;
    cycle.out.Fr = len_control(r_len_, right, cmd_len - roll_.result) +
                    model::gravity_feedforward - right.fs;
    ++settle_count_;
    v_cmd_enabled_ = settle_count_ >= 500U;
    if (v_cmd_enabled_) len_ramp_.set_step(0.00027f);
    // Legacy final assignment priority: fall protection overrides all earlier transitions.
    if (left.alpha >= 0.8f || right.alpha >= 0.8f)
    {
        reset_phi_control();
        cycle.next = chassis_state::relax;
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
    case chassis_state::flatten:
        run_flatten(in, cycle);
        break;
    case chassis_state::neutral:
        run_neutral(in, cycle);
        break;
    case chassis_state::normal:
        run_normal(in, cycle);
        break;
    default:
        cycle.next = chassis_state::relax;
        break;
    }
}

chassis_output controller::step(const ::imu::state& imu, const motor_feedback_snapshot& fdb,
                                const chassis_command& cmd, std::uint32_t now,
                                robot_state& published) noexcept
{
    const float wheel_velocity = (-fdb.l_wheel.velocity + fdb.r_wheel.velocity) * 0.5f * model::wheel_radius;
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
    if (!valid || !cmd.move)
    {
        state_ = chassis_state::relax;
        roll_.clear();
        len_ramp_.set_step(0.0002f);
    }
    apply_tuning();
    const chassis_state executed = state_;
    state_cycle cycle{};
    cycle.next = state_;
    robot_debug.chassis.control = {};
    robot_debug.chassis.joint_raw = {};
    run_state(executed, in, cycle);

    auto& out = cycle.out;
    //! 暂时清空轮力矩
    // cycle.out.Twl = 0.0f;
    // cycle.out.Twr = 0.0f;
    if (executed != chassis_state::relax)
    {
        const auto lt = left_.joint_torque(cycle.out.Fl, cycle.out.Tpl);
        const auto rt = right_.joint_torque(cycle.out.Fr, cycle.out.Tpr);
        robot_debug.chassis.joint_raw = {lt[0], lt[1], rt[0], rt[1]};
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
    published.v_cmd_enabled = v_cmd_enabled_;
    published.recovered = recovered_;
    published.state_executed = executed;
    published.next_state = cycle.next;

    robot_debug.chassis.state = published;
    ++robot_debug.chassis.sample_seq;
    robot_debug.chassis.sample_tick = now;
    robot_debug.chassis.left = left;
    robot_debug.chassis.right = right;
    ++robot_debug.chassis.sample_seq;
    robot_debug.chassis.output = out;
    std::copy(obs.begin(), obs.end(), robot_debug.chassis.obs);
    std::copy(cycle.ref.begin(), cycle.ref.end(), robot_debug.chassis.ref);
    robot_debug.chassis.l_phi_ref = robot_debug.chassis.control.l_phi.ref;
    robot_debug.chassis.r_phi_ref = robot_debug.chassis.control.r_phi.ref;
    robot_debug.chassis.roll = in.imu.roll;
    robot_debug.chassis.droll = in.imu.gyro_r;
    robot_debug.chassis.normal_force = left.normal_force + right.normal_force;
    robot_debug.chassis.lqr_mode = static_cast<std::uint8_t>(cycle.mode);
    return out;
}

} // namespace robot::chassis
