#include "chassis.hpp"

#include "model/chassis_model.hpp"
#include "model/leg_lqr_coeffs.hpp"

#include <algorithm>
#include <cmath>

namespace chassis
{
controller::phi_pid_control::phi_pid_control() noexcept
    : pid(tuning.phi_flatten_slow.kp, tuning.phi_flatten_slow.ki,
          tuning.phi_flatten_slow.kd, 40.0f, 0.005f)
{
}

controller::controller()
    : l_len_(tuning.len_startup.kp, tuning.len_startup.ki, tuning.len_startup.kd,
             125.0f, 0.0f, control::pid_mode::delta),
      r_len_(tuning.len_startup.kp, tuning.len_startup.ki, tuning.len_startup.kd,
             125.0f, 0.0f, control::pid_mode::delta),
      roll_(tuning.roll.kp, tuning.roll.ki, tuning.roll.kd, 0.05f, 0.005f)
{
}

void controller::apply_tuning() noexcept
{
    const auto& len = (fsm_ == chassis_fsm::normal || fsm_ == chassis_fsm::spin) ? tuning.len_run : tuning.len_startup;
    l_len_.tune(len.kp, len.ki, len.kd);
    r_len_.tune(len.kp, len.ki, len.kd);
    roll_.tune(tuning.roll.kp, tuning.roll.ki, tuning.roll.kd);
}

float controller::len_control(control::pid& pid, const leg_state& leg, float ref) noexcept
{
    pid.ref = ref;
    pid.fdb = leg.len;
    pid.update(leg.dlen);
    auto& debug = (&pid == &l_len_) ? dbg.chassis.control.l_len
                                   : dbg.chassis.control.r_len;
    debug = {pid.ref, pid.fdb, pid.result, true};
    return pid.result;
}

float controller::update_phi_control(phi_pid_control& ctrl, const leg_state& leg,
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
    auto& debug = (&ctrl == &l_phi_) ? dbg.chassis.control.l_phi
                                    : dbg.chassis.control.r_phi;
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

void controller::run_relax(const inputs& in, step_result& result) noexcept
{
    const auto& left = in.left;

    relax_left();
    relax_right();
    odom_.reset();
    target_len_ = model::Lmin;
    len_ramp_.set_value(left.len);
    recovered_ = in.imu.accel.z >= 0.0f;
    v_cmd_enabled_ = false;
    settle_count_ = 0;

    // RELAX preview only: update watch values without assigning any actuator output.
    const auto leg_obs = leg_lqr::observe(in.imu, left, in.right);
    const auto leg_ref = leg_lqr::reference(
        std::clamp(in.cmd.len, model::Lmin, model::Lmax), 0.0f, tuning.alpha_trim);
    const auto support = leg_lqr_.update(leg_obs, leg_ref);
    auto& leg_debug = dbg.chassis.control.leg_lqr;
    std::copy(leg_obs.begin(), leg_obs.end(), leg_debug.obs);
    std::copy(leg_ref.begin(), leg_ref.end(), leg_debug.ref);
    std::copy(support.begin(), support.end(), leg_debug.support_force);
    leg_debug.active = false;

    if (in.valid && in.cmd.move)
    {
        result.next = recovered_ ? chassis_fsm::flatten : chassis_fsm::recover;
    }
}

void controller::run_recover(const inputs& in, step_result& result) noexcept
{
    odom_.reset();
    if (in.imu.accel.z > 5.0f && std::fabs(in.imu.pitch) < 0.35f &&
        std::fabs(in.imu.gyro.y) < 1.0f)
    {
        relax_left();
        relax_right();
        recovered_ = true;
        result.next = chassis_fsm::flatten;
        return;
    }

    const auto& param = tuning.phi_recover;
    l_phi_.pid.tune(param.kp, param.ki, param.kd);
    r_phi_.pid.tune(param.kp, param.ki, param.kd);
    const bool positive = in.imu.pitch > 0.0f;
    const float target = positive ? 0.0f : model::pi;
    result.output.Tpl = update_phi_control(l_phi_, in.left, left_.total_phi(), target, 0.005f,
                                           positive);
    result.output.Tpr = update_phi_control(r_phi_, in.right, right_.total_phi(), target, 0.005f,
                                           positive);
    if (std::fabs(in.left.phi - target) < 0.1f &&
        std::fabs(in.right.phi - target) < 0.1f)
    {
        result.output.Tpl = positive ? -35.0f : 35.0f;
        result.output.Tpr = positive ? 35.0f : -35.0f;
    }
}

void controller::run_flatten(const inputs& in, step_result& result) noexcept
{
    const auto& left = in.left;
    const auto& right = in.right;

    recovered_ = true;
    v_cmd_enabled_ = true;
    if ((left.phi >= 0.7f && left.phi <= 1.4f) &&
        (right.phi >= 0.7f && right.phi <= 1.4f))
    {
        reset_phi_control();
        result.next = chassis_fsm::neutral;
        return;
    }

    if (left.flat)
    {
        relax_left();
    }
    else
    {
        const auto& param = left.phi >= model::pi * 0.4f ? tuning.phi_flatten_fast
                                                        : tuning.phi_flatten_slow;
        l_phi_.pid.tune(param.kp, param.ki, param.kd);
        result.output.Tpl = update_phi_control(
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
        const auto& param = right.phi >= model::pi * 0.4f ? tuning.phi_flatten_fast
                                                         : tuning.phi_flatten_slow;
        r_phi_.pid.tune(param.kp, param.ki, param.kd);
        result.output.Tpr = update_phi_control(
            r_phi_, right, right_.total_phi(), model::pi,
            right.phi >= model::pi * 0.4f ? 0.01f : 0.002f,
            right.phi >= model::pi * 0.4f);
    }

    result.output.Twl = result.output.Twr = in.cmd.v > 0.0f ? 1.5f : 0.0f;
    if (left.flat && right.flat)
    {
        odom_.reset();
        reset_phi_control();
        result.next = chassis_fsm::neutral;
    }
}

void controller::run_neutral(const inputs& in, step_result& result) noexcept
{
    const auto& left = in.left;
    const auto& right = in.right;

    odom_.reset();
    const auto& param = tuning.phi_neutral;
    l_phi_.pid.tune(param.kp, param.ki, param.kd);
    r_phi_.pid.tune(param.kp, param.ki, param.kd);
    result.output.Fl = len_control(l_len_, left, left.len + (model::Lmin - left.len) * 0.6f) -
                    left.spring.fs;
    result.output.Fr = len_control(r_len_, right, right.len + (model::Lmin - right.len) * 0.6f) -
                    right.spring.fs;
    result.output.Tpl = update_phi_control(l_phi_, left, left_.total_phi(),
                                0.5f * model::pi, 0.003f,
                                left.phi < model::pi * 0.45f);
    result.output.Tpr = update_phi_control(r_phi_, right, right_.total_phi(),
                                0.5f * model::pi, 0.003f,
                                right.phi < model::pi * 0.45f);
    // Deterministic fix: the deadband explicitly commands zero wheel torque.
    if (in.cmd.v > 0.005f)
        result.output.Twl = result.output.Twr = 1.5f;
    else if (in.cmd.v < -0.005f)
        result.output.Twl = result.output.Twr = -1.5f;
    else
        result.output.Twl = result.output.Twr = 0.0f;

    if (left.neutral && right.neutral)
    {
        reset_phi_control();
        target_len_ = model::Lmin;
        len_ramp_.set_value(left.len);
        result.next = chassis_fsm::normal;
    }
}

void controller::update_balance(const inputs& in, step_result& result,
                                lqr_mode mode, float alpha_trim) noexcept
{
    const auto& left = in.left;
    const auto& right = in.right;
    const auto& obs = in.obs;

    result.ref = {in.cmd.x, in.cmd.v, in.cmd.yaw, in.cmd.dyaw,
                 left.alpha_eq + alpha_trim, 0.0f,
                 right.alpha_eq + alpha_trim, 0.0f, 0.0f, 0.0f};
    result.mode = mode;
    const auto torque = lqr_.update(result.mode, left.len, right.len, obs, result.ref);
    auto& debug = dbg.chassis.control;
    std::copy(torque.begin(), torque.end(), debug.lqr_output);
    const auto& raw = lqr_.raw_output();
    std::copy(raw.begin(), raw.end(), debug.lqr_raw);
    debug.lqr_active = true;
    result.output.Twl = torque[0];
    result.output.Twr = torque[1];
    result.output.Tpl = torque[2];
    result.output.Tpr = torque[3];
}

void controller::update_leg_pid(const inputs& in, step_result& result) noexcept
{
    const auto& left = in.left;
    const auto& right = in.right;
    auto& debug = dbg.chassis.control;
    roll_.ref = 0.0f;
    roll_.fdb = in.imu.roll;
    roll_.update(in.imu.gyro.x);
    debug.roll = {roll_.ref, roll_.fdb, roll_.result, true};

    target_len_ = std::clamp(in.cmd.len, model::Lmin, model::Lmax);
    const float cmd_len = len_ramp_.update(target_len_);
    result.output.Fl = len_control(l_len_, left, cmd_len + roll_.result) +
                    model::gravity_feedforward - left.spring.fs;
    result.output.Fr = len_control(r_len_, right, cmd_len - roll_.result) +
                    model::gravity_feedforward - right.spring.fs;
}

void controller::run_normal(const inputs& in, step_result& result) noexcept
{
    const auto& left = in.left;
    const auto& right = in.right;
    const auto mode = (left.len + right.len) * 0.5f > model::Lswitch ? lqr_mode::high : lqr_mode::low;
    update_balance(in, result, mode, tuning.alpha_trim);

    target_len_ = std::clamp(in.cmd.len, model::Lmin, model::Lmax);
    const float previous_len = len_ramp_.value();
    const float cmd_len = len_ramp_.update(target_len_);
    const float cmd_dlen = (cmd_len - previous_len) / leg_lqr_model::sample_time;
    const auto leg_obs = leg_lqr::observe(in.imu, left, right);
    const auto leg_ref = leg_lqr::reference(cmd_len, cmd_dlen, tuning.alpha_trim);
    const auto support = leg_lqr_.update(leg_obs, leg_ref);
    // The model includes roll projection; remove only the sagittal leg projection here.
    // VMC consumes active axial force. Passive spring support is subtracted once.
    result.output.Fl = support[0] / std::cos(left.alpha) - left.spring.fs;
    result.output.Fr = support[1] / std::cos(right.alpha) - right.spring.fs;
    auto& leg_debug = dbg.chassis.control.leg_lqr;
    std::copy(leg_obs.begin(), leg_obs.end(), leg_debug.obs);
    std::copy(leg_ref.begin(), leg_ref.end(), leg_debug.ref);
    std::copy(support.begin(), support.end(), leg_debug.support_force);
    leg_debug.active = true;

    ++settle_count_;
    v_cmd_enabled_ = settle_count_ >= 500U;
    if (v_cmd_enabled_) len_ramp_.set_step(0.00027f);
    if (in.cmd.spin && v_cmd_enabled_)
        result.next = chassis_fsm::spin;
    // Legacy final assignment priority: fall protection overrides all earlier transitions.
    if (left.alpha >= 0.8f || right.alpha >= 0.8f)
    {
        reset_phi_control();
        result.next = chassis_fsm::relax;
    }
}

void controller::run_spin(const inputs& in, step_result& result) noexcept
{
    update_balance(in, result, lqr_mode::spin, tuning.spin_alpha_trim);
    update_leg_pid(in, result);
    v_cmd_enabled_ = true;
    if (!in.cmd.spin && std::fabs(in.cmd.dyaw) < 0.002f)
        result.next = chassis_fsm::normal;
    if (in.left.alpha >= 0.8f || in.right.alpha >= 0.8f)
    {
        reset_phi_control();
        result.next = chassis_fsm::relax;
    }
}

void controller::run_state(chassis_fsm executed, const inputs& in,
                           step_result& result) noexcept
{
    switch (executed)
    {
    case chassis_fsm::relax:
        run_relax(in, result);
        break;
    case chassis_fsm::recover:
        run_recover(in, result);
        break;
    case chassis_fsm::flatten:
        run_flatten(in, result);
        break;
    case chassis_fsm::neutral:
        run_neutral(in, result);
        break;
    case chassis_fsm::normal:
        run_normal(in, result);
        break;
    case chassis_fsm::spin:
        run_spin(in, result);
        break;
    default:
        result.next = chassis_fsm::relax;
        break;
    }
}

chassis_output controller::step(const ::imu::attitude::state& imu, const motor_fdb& fdb,
                                const chassis_cmd& cmd, std::uint32_t now,
                                chassis_state& published) noexcept
{
    const float wheel_velocity = (-fdb.l_w.velocity + fdb.r_w.velocity) * 0.5f * model::wheel_radius;
    odom_.update(imu, wheel_velocity);
    const bool left_valid = left_.update(fdb.l_j1, fdb.l_j4, imu, odom_.az());
    const bool right_valid = right_.update(fdb.r_j1, fdb.r_j4, imu, odom_.az());
    const auto& left = left_.state();
    const auto& right = right_.state();

    const std::array<float, 10> obs{
        odom_.x(), odom_.v(), imu.total_yaw, imu.gyro.z, left.alpha,
        left.dalpha, right.alpha, right.dalpha, imu.pitch, imu.gyro.y};
    const bool imu_valid = imu.online && imu.sequence != 0U && now - imu.received_ms <= 100U;
    const bool valid = imu_valid && fdb.motors_online && left_valid && right_valid;
    const inputs in{imu, fdb, cmd, left, right, obs, valid};
    if (!valid || !cmd.move)
    {
        fsm_ = chassis_fsm::relax;
        roll_.clear();
        len_ramp_.set_step(0.0002f);
    }
    apply_tuning();
    const chassis_fsm executed = fsm_;
    step_result result{};
    result.next = fsm_;
    dbg.chassis.control = {};
    dbg.chassis.joint_raw = {};
    run_state(executed, in, result);

    auto& motor_output = result.output;
    //! 暂时清空轮力矩
    // result.output.Twl = 0.0f;
    // result.output.Twr = 0.0f;
    if (executed != chassis_fsm::relax)
    {
        const auto lt = left_.joint_torque(result.output.Fl, result.output.Tpl);
        const auto rt = right_.joint_torque(result.output.Fr, result.output.Tpr);
        dbg.chassis.joint_raw = {lt[0], lt[1], rt[0], rt[1]};
        motor_output.l_joint1 = std::clamp(lt[0], -model::max_joint_torque, model::max_joint_torque);
        motor_output.l_joint4 = std::clamp(lt[1], -model::max_joint_torque, model::max_joint_torque);
        motor_output.r_joint1 = std::clamp(rt[0], -model::max_joint_torque, model::max_joint_torque);
        motor_output.r_joint4 = std::clamp(rt[1], -model::max_joint_torque, model::max_joint_torque);
    }
    motor_output.Twl = std::clamp(motor_output.Twl, -model::max_wheel_torque, model::max_wheel_torque);
    motor_output.Twr = std::clamp(motor_output.Twr, -model::max_wheel_torque, model::max_wheel_torque);
    fsm_ = result.next;

    published.x = odom_.x();
    published.v = odom_.v();
    published.len = 0.5f * (left.len + right.len);
    published.lalpha = left.alpha;
    published.ralpha = right.alpha;
    published.pitch = in.imu.pitch;
    published.yaw = in.imu.total_yaw;
    published.v_cmd_enabled = v_cmd_enabled_;
    published.recovered = recovered_;
    published.fsm_executed = executed;
    published.fsm_next = result.next;

    dbg.chassis.state = published;
    ++dbg.chassis.sample_seq;
    dbg.chassis.sample_tick = now;
    dbg.chassis.left = left;
    dbg.chassis.right = right;
    ++dbg.chassis.sample_seq;
    dbg.chassis.output = motor_output;
    std::copy(obs.begin(), obs.end(), dbg.chassis.obs);
    std::copy(result.ref.begin(), result.ref.end(), dbg.chassis.ref);
    dbg.chassis.l_phi_ref = dbg.chassis.control.l_phi.ref;
    dbg.chassis.r_phi_ref = dbg.chassis.control.r_phi.ref;
    dbg.chassis.roll = in.imu.roll;
    dbg.chassis.droll = in.imu.gyro.x;
    dbg.chassis.normal_force = left.normal_force + right.normal_force;
    dbg.chassis.lqr_mode = static_cast<std::uint8_t>(result.mode);
    return motor_output;
}

} // namespace chassis
