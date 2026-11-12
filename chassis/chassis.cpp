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

float controller::len_control(control::pid& pid, const leg_state& leg, float ref,
                              pid_debug& debug) noexcept
{
    pid.ref = ref;
    pid.fdb = leg.len;
    pid.update(leg.dlen);
    debug = {pid.ref, pid.fdb, pid.result, true};
    return pid.result;
}

float controller::update_phi_control(phi_pid_control& ctrl, const leg_state& leg,
                                     float total_phi, float target, float step,
                                     bool positive, pid_debug& debug) noexcept
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

void controller::run_relax(const inputs& in, step_out& out) noexcept
{
    const auto& left = in.left;

    relax_left();
    relax_right();
    odom_.reset();
    target_len_ = model::Lmin;
    len_ramp_.set_value(left.len);
    out.recovered = in.imu.accel.z >= 0.0f;
    settle_count_ = 0;

    if (in.valid && in.cmd.move)
    {
        out.fsm_next = out.recovered ? chassis_fsm::flatten : chassis_fsm::recover;
    }

    // RELAX preview only: update watch values without assigning any actuator output.
    const auto leg_obs = leg_lqr::observe(in.imu, left, in.right);
    const auto leg_ref = leg_lqr::reference(
        std::clamp(in.cmd.len, model::Lmin, model::Lmax), 0.0f, tuning.alpha_trim);
    const auto support = leg_lqr_.update(leg_obs, leg_ref);
    auto& leg_debug = out.debug.leg_lqr;
    std::copy(leg_obs.begin(), leg_obs.end(), leg_debug.obs);
    std::copy(leg_ref.begin(), leg_ref.end(), leg_debug.ref);
    std::copy(support.begin(), support.end(), leg_debug.support_force);
    leg_debug.active = false;
}

void controller::run_recover(const inputs& in, step_out& out) noexcept
{
    odom_.reset();
    if (in.imu.accel.z > 5.0f && std::fabs(in.imu.pitch) < 0.35f &&
        std::fabs(in.imu.gyro.y) < 1.0f)
    {
        relax_left();
        relax_right();
        out.recovered = true;
        out.fsm_next = chassis_fsm::flatten;
        return;
    }

    const auto& param = tuning.phi_recover;
    l_phi_.pid.tune(param.kp, param.ki, param.kd);
    r_phi_.pid.tune(param.kp, param.ki, param.kd);
    const bool positive = in.imu.pitch > 0.0f;
    const float target = positive ? 0.0f : model::pi;
    out.output.Tpl = update_phi_control(l_phi_, in.left, left_.total_phi(), target, 0.005f,
                                           positive, out.debug.l_phi);
    out.output.Tpr = update_phi_control(r_phi_, in.right, right_.total_phi(), target, 0.005f,
                                           positive, out.debug.r_phi);
    if (std::fabs(in.left.phi - target) < 0.1f &&
        std::fabs(in.right.phi - target) < 0.1f)
    {
        out.output.Tpl = positive ? -35.0f : 35.0f;
        out.output.Tpr = positive ? 35.0f : -35.0f;
    }
}

void controller::run_flatten(const inputs& in, step_out& out) noexcept
{
    const auto& left = in.left;
    const auto& right = in.right;

    out.recovered = true;
    out.v_cmd_enabled = true;
    if ((left.phi >= 0.7f && left.phi <= 1.4f) &&
        (right.phi >= 0.7f && right.phi <= 1.4f))
    {
        reset_phi_control();
        out.fsm_next = chassis_fsm::neutral;
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
        out.output.Tpl = update_phi_control(
            l_phi_, left, left_.total_phi(), model::pi,
            left.phi >= model::pi * 0.4f ? 0.01f : 0.002f,
            left.phi >= model::pi * 0.4f, out.debug.l_phi);
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
        out.output.Tpr = update_phi_control(
            r_phi_, right, right_.total_phi(), model::pi,
            right.phi >= model::pi * 0.4f ? 0.01f : 0.002f,
            right.phi >= model::pi * 0.4f, out.debug.r_phi);
    }

    out.output.Twl = out.output.Twr = in.cmd.v > 0.0f ? 1.5f : 0.0f;
    if (left.flat && right.flat)
    {
        odom_.reset();
        reset_phi_control();
        out.fsm_next = chassis_fsm::neutral;
    }
}

void controller::run_neutral(const inputs& in, step_out& out) noexcept
{
    const auto& left = in.left;
    const auto& right = in.right;

    out.recovered = true;
    out.v_cmd_enabled = true;
    odom_.reset();
    const auto& param = tuning.phi_neutral;
    l_phi_.pid.tune(param.kp, param.ki, param.kd);
    r_phi_.pid.tune(param.kp, param.ki, param.kd);
    out.output.Fl = len_control(l_len_, left, left.len + (model::Lmin - left.len) * 0.6f,
                                   out.debug.l_len) -
                    left.spring.fs;
    out.output.Fr = len_control(r_len_, right, right.len + (model::Lmin - right.len) * 0.6f,
                                   out.debug.r_len) -
                    right.spring.fs;
    out.output.Tpl = update_phi_control(l_phi_, left, left_.total_phi(),
                                0.5f * model::pi, 0.003f,
                                left.phi < model::pi * 0.45f, out.debug.l_phi);
    out.output.Tpr = update_phi_control(r_phi_, right, right_.total_phi(),
                                0.5f * model::pi, 0.003f,
                                right.phi < model::pi * 0.45f, out.debug.r_phi);
    // Deterministic fix: the deadband explicitly commands zero wheel torque.
    if (in.cmd.v > 0.005f)
        out.output.Twl = out.output.Twr = 1.5f;
    else if (in.cmd.v < -0.005f)
        out.output.Twl = out.output.Twr = -1.5f;
    else
        out.output.Twl = out.output.Twr = 0.0f;

    if (left.neutral && right.neutral)
    {
        reset_phi_control();
        target_len_ = model::Lmin;
        len_ramp_.set_value(left.len);
        out.fsm_next = chassis_fsm::normal;
    }
}

void controller::update_balance(const inputs& in, step_out& out,
                                lqr_mode mode, float alpha_trim) noexcept
{
    const auto& left = in.left;
    const auto& right = in.right;
    const auto& obs = in.obs;

    out.ref.x = in.cmd.x;
    out.ref.v = in.cmd.v;
    out.ref.yaw = in.cmd.yaw;
    out.ref.dyaw = in.cmd.dyaw;
    out.ref.l_alpha = left.alpha_eq + alpha_trim;
    out.ref.l_dalpha = 0.0f;
    out.ref.r_alpha = right.alpha_eq + alpha_trim;
    out.ref.r_dalpha = 0.0f;
    out.ref.pitch = 0.0f;
    out.ref.dpitch = 0.0f;
    out.mode = mode;
    const auto torque = lqr_.update(out.mode, left.len, right.len, obs, out.ref);
    out.output.Twl = torque[0];
    out.output.Twr = torque[1];
    out.output.Tpl = torque[2];
    out.output.Tpr = torque[3];

    auto& debug = out.debug;
    std::copy(torque.begin(), torque.end(), debug.lqr_output);
    const auto& raw = lqr_.raw_output();
    std::copy(raw.begin(), raw.end(), debug.lqr_raw);
    debug.lqr_active = true;
}

void controller::update_leg_pid(const inputs& in, step_out& out) noexcept
{
    const auto& left = in.left;
    const auto& right = in.right;
    roll_.ref = 0.0f;
    roll_.fdb = in.imu.roll;
    roll_.update(in.imu.gyro.x);

    target_len_ = std::clamp(in.cmd.len, model::Lmin, model::Lmax);
    const float cmd_len = len_ramp_.update(target_len_);
    out.output.Fl = len_control(l_len_, left, cmd_len + roll_.result, out.debug.l_len) +
                    model::gravity_feedforward - left.spring.fs;
    out.output.Fr = len_control(r_len_, right, cmd_len - roll_.result, out.debug.r_len) +
                    model::gravity_feedforward - right.spring.fs;

    out.debug.roll = {roll_.ref, roll_.fdb, roll_.result, true};
}

void controller::update_leg_lqr(const inputs& in, step_out& out) noexcept
{
    const auto& left = in.left;
    const auto& right = in.right;
    target_len_ = std::clamp(in.cmd.len, model::Lmin, model::Lmax);
    const float previous_len = len_ramp_.value();
    const float cmd_len = len_ramp_.update(target_len_);
    const float cmd_dlen = (cmd_len - previous_len) / leg_lqr_model::sample_time;
    const auto leg_obs = leg_lqr::observe(in.imu, left, right);
    const auto leg_ref = leg_lqr::reference(cmd_len, cmd_dlen, tuning.alpha_trim);
    const auto support = leg_lqr_.update(leg_obs, leg_ref);
    // The model includes roll projection; remove only the sagittal leg projection here.
    // VMC consumes active axial force. Passive spring support is subtracted once.
    out.output.Fl = support[0] / std::cos(left.alpha) - left.spring.fs;
    out.output.Fr = support[1] / std::cos(right.alpha) - right.spring.fs;

    auto& leg_debug = out.debug.leg_lqr;
    std::copy(leg_obs.begin(), leg_obs.end(), leg_debug.obs);
    std::copy(leg_ref.begin(), leg_ref.end(), leg_debug.ref);
    std::copy(support.begin(), support.end(), leg_debug.support_force);
    leg_debug.active = true;
}

void controller::run_normal(const inputs& in, step_out& out) noexcept
{
    const auto& left = in.left;
    const auto& right = in.right;
    out.recovered = true;
    const auto mode = (left.len + right.len) * 0.5f > model::Lswitch ? lqr_mode::high : lqr_mode::low;
    update_balance(in, out, mode, tuning.alpha_trim);
    update_leg_lqr(in, out);
    ++settle_count_;
    out.v_cmd_enabled = settle_count_ >= 500U;
    if (out.v_cmd_enabled) len_ramp_.set_step(0.00027f);
    if (in.cmd.spin && out.v_cmd_enabled)
        out.fsm_next = chassis_fsm::spin; //! normal进airborne的离地
    const bool left_airborne = l_contact_.off_timer.update(left.offground && left.dlen > 0.05);
    const bool right_airborne = r_contact_.off_timer.update(right.offground && right.dlen > 0.05);
    if (left_airborne || right_airborne)
    {
        out.v_cmd_enabled = false;
        out.fsm_next = chassis_fsm::airborne;
    }
    // Legacy final assignment priority: fall protection overrides all earlier transitions.
    if (left.alpha >= 0.8f || right.alpha >= 0.8f)
    {
        reset_phi_control();
        out.fsm_next = chassis_fsm::relax;
    }
}

void controller::run_spin(const inputs& in, step_out& out) noexcept
{
    out.recovered = true;
    update_balance(in, out, lqr_mode::spin, tuning.spin_alpha_trim);
    update_leg_pid(in, out);
    out.v_cmd_enabled = true;
    if (!in.cmd.spin && std::fabs(in.cmd.dyaw) < 0.002f)
        out.fsm_next = chassis_fsm::normal;
    if (in.left.alpha >= 0.8f || in.right.alpha >= 0.8f)
    {
        reset_phi_control();
        out.fsm_next = chassis_fsm::relax;
    }
}

/**
 * @brief 被动离地状态
 */
void controller::run_airborne(const inputs& in, step_out& out) noexcept
{
    out.recovered = true;
    if (!l_contact_.air_seen && in.left.offground)
    {
        l_contact_.air_seen = true;
        l_contact_.dlen_peak = 0.0f;
        l_contact_.touch_timer.reset();
        l_contact_.touchdown = false;
    }
    if (!r_contact_.air_seen && in.right.offground)
    {
        r_contact_.air_seen = true;
        r_contact_.dlen_peak = 0.0f;
        r_contact_.touch_timer.reset();
        r_contact_.touchdown = false;
    }
    if (!l_contact_.touchdown)
    {
        const bool finite = std::isfinite(in.left.len) && std::isfinite(in.left.dlen);
        if (finite)
        {
            l_contact_.dlen_peak = std::max(l_contact_.dlen_peak, in.left.dlen);
        }
        const bool contact = finite && (in.left.dlen < model::touchdown_compress_rate ||
            (in.left.len < model::Lmax - model::touchdown_len_margin &&
             l_contact_.dlen_peak > model::touchdown_peak_rate &&
             l_contact_.dlen_peak - in.left.dlen > model::touchdown_rate_drop));
        if (l_contact_.touch_timer.update(contact))
        {
            l_contact_.touchdown = true;
        }
    }
    if (!r_contact_.touchdown)
    {
        const bool finite = std::isfinite(in.right.len) && std::isfinite(in.right.dlen);
        if (finite)
        {
            r_contact_.dlen_peak = std::max(r_contact_.dlen_peak, in.right.dlen);
        }
        const bool contact = finite && (in.right.dlen < model::touchdown_compress_rate ||
            (in.right.len < model::Lmax - model::touchdown_len_margin &&
             r_contact_.dlen_peak > model::touchdown_peak_rate &&
             r_contact_.dlen_peak - in.right.dlen > model::touchdown_rate_drop));
        if (r_contact_.touch_timer.update(contact))
        {
            r_contact_.touchdown = true;
        }
    }

    out.ref.x = in.obs.x;
    out.ref.v = in.obs.v;
    out.ref.yaw = in.obs.yaw;
    out.ref.dyaw = in.obs.dyaw;
    out.ref.l_alpha = in.imu.pitch; //为了保持机身的垂直，不跟踪绝对的alpha = 0
    out.ref.l_dalpha = 0.0f;
    out.ref.r_alpha = in.imu.pitch;
    out.ref.r_dalpha = 0.0f;
    out.ref.pitch = in.obs.pitch;
    out.ref.dpitch = in.obs.dpitch;
    out.mode = lqr_mode::low;
    const auto torque = lqr_.update(out.mode, in.left.len, in.right.len, in.obs, out.ref);
    out.output.Tpl = torque[2]; //TODO:为了让空中给的摆角不要那么剧烈，降低了力矩，可以继续调
    out.output.Tpr = torque[3]; //TODO:为了让空中给的摆角不要那么剧烈，降低了力矩，可以继续调
    out.output.Twl = 0.0f;
    out.output.Twr = 0.0f;

    if (l_contact_.touchdown || r_contact_.touchdown)
    {
        update_leg_lqr(in, out);
    }
    if (!l_contact_.touchdown)
    {
        out.output.Fl = model::air_leg_force;
    }
    if (!r_contact_.touchdown)
    {
        out.output.Fr = model::air_leg_force;
    }

    if (in.left.alpha >= 0.8f || in.right.alpha >= 0.8f) //空中摆角太大失衡
    {
        reset_phi_control();
        out.fsm_next = chassis_fsm::relax;
    }
    else if (l_contact_.touchdown && r_contact_.touchdown)
    {
        settle_count_ = 0;
        target_len_ = model::Lmin;
        len_ramp_.set_value(in.left.len);
        len_ramp_.set_step(0.0009f);
        out.fsm_next = chassis_fsm::normal;
    }

    auto& debug = out.debug;
    std::copy(torque.begin(), torque.end(), debug.lqr_output);
    const auto& raw = lqr_.raw_output();
    std::copy(raw.begin(), raw.end(), debug.lqr_raw);
    debug.lqr_active = true;
}

void controller::run_state(chassis_fsm executed, const inputs& in,
                           step_out& out) noexcept
{
    switch (executed)
    {
    case chassis_fsm::relax:
        run_relax(in, out);
        break;
    case chassis_fsm::recover:
        run_recover(in, out);
        break;
    case chassis_fsm::flatten:
        run_flatten(in, out);
        break;
    case chassis_fsm::neutral:
        run_neutral(in, out);
        break;
    case chassis_fsm::normal:
        run_normal(in, out);
        break;
    case chassis_fsm::spin:
        run_spin(in, out);
        break;
    case chassis_fsm::airborne:
        run_airborne(in, out);
        break;
    default:
        out.fsm_next = chassis_fsm::relax;
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

    lqr_state obs{};
    obs.x = odom_.x();
    obs.v = odom_.v();
    obs.yaw = imu.total_yaw;
    obs.dyaw = imu.gyro.z;
    obs.l_alpha = left.alpha;
    obs.l_dalpha = left.dalpha;
    obs.r_alpha = right.alpha;
    obs.r_dalpha = right.dalpha;
    obs.pitch = imu.pitch;
    obs.dpitch = imu.gyro.y;
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
    const bool detect_offground = executed == chassis_fsm::normal ||
                                  executed == chassis_fsm::airborne;
    left_.detect_offground(detect_offground);
    right_.detect_offground(detect_offground);
    if (executed != chassis_fsm::airborne)
    {
        l_contact_.dlen_peak = 0.0f;
        r_contact_.dlen_peak = 0.0f;
        l_contact_.touch_timer.reset();
        r_contact_.touch_timer.reset();
        l_contact_.air_seen = false;
        r_contact_.air_seen = false;
        l_contact_.touchdown = false;
        r_contact_.touchdown = false;
    }
    step_out out{};
    out.fsm_next = fsm_;
    run_state(executed, in, out);
    if (executed != chassis_fsm::normal || out.fsm_next != chassis_fsm::normal)
    {
        l_contact_.off_timer.reset();
        r_contact_.off_timer.reset();
    }
    if (executed != chassis_fsm::airborne && out.fsm_next == chassis_fsm::airborne)
    {
        // A leg that remained supported need not produce a new landing impulse.
        l_contact_.air_seen = left.offground;
        r_contact_.air_seen = right.offground;
        l_contact_.touchdown = !left.offground && std::isfinite(left.normal_force) &&
                               std::isfinite(left.len) && std::isfinite(left.dlen);
        r_contact_.touchdown = !right.offground && std::isfinite(right.normal_force) &&
                               std::isfinite(right.len) && std::isfinite(right.dlen);
    }

    auto& motor_output = out.output;
    joint_torque_debug joint_raw{};
    if (executed != chassis_fsm::relax)
    {
        const auto lt = left_.joint_torque(out.output.Fl, out.output.Tpl);
        const auto rt = right_.joint_torque(out.output.Fr, out.output.Tpr);
        joint_raw = {lt[0], lt[1], rt[0], rt[1]};
        motor_output.l_joint1 = std::clamp(lt[0], -model::max_joint_torque, model::max_joint_torque);
        motor_output.l_joint4 = std::clamp(lt[1], -model::max_joint_torque, model::max_joint_torque);
        motor_output.r_joint1 = std::clamp(rt[0], -model::max_joint_torque, model::max_joint_torque);
        motor_output.r_joint4 = std::clamp(rt[1], -model::max_joint_torque, model::max_joint_torque);
    }
    motor_output.Twl = std::clamp(motor_output.Twl, -model::max_wheel_torque, model::max_wheel_torque);
    motor_output.Twr = std::clamp(motor_output.Twr, -model::max_wheel_torque, model::max_wheel_torque);
    fsm_ = out.fsm_next;

    published.x = odom_.x();
    published.v = odom_.v();
    published.len = 0.5f * (left.len + right.len);
    published.lalpha = left.alpha;
    published.ralpha = right.alpha;
    published.pitch = in.imu.pitch;
    published.yaw = in.imu.total_yaw;
    published.v_cmd_enabled = out.v_cmd_enabled;
    published.recovered = out.recovered;
    published.fsm_executed = executed;
    published.fsm_next = out.fsm_next;
    published.normal_force = left.normal_force + right.normal_force;
    published.offground = is_offground(left, right);

    ++dbg.chassis.sample_seq;
    dbg.chassis.sample_tick = now;
    dbg.chassis.len_cmd = cmd.len;
    dbg.chassis.len_target = target_len_;
    dbg.chassis.len_ref = len_ramp_.value();
    dbg.chassis.l_contact = {l_contact_.off_timer.count(), l_contact_.touch_timer.count(),
                             l_contact_.dlen_peak, l_contact_.air_seen, l_contact_.touchdown};
    dbg.chassis.r_contact = {r_contact_.off_timer.count(), r_contact_.touch_timer.count(),
                             r_contact_.dlen_peak, r_contact_.air_seen, r_contact_.touchdown};
    dbg.chassis.state = published;
    dbg.chassis.left = left;
    dbg.chassis.right = right;
    ++dbg.chassis.sample_seq;
    dbg.chassis.output = motor_output;
    dbg.chassis.joint_raw = joint_raw;
    dbg.chassis.control = out.debug;
    dbg.chassis.obs = obs;
    dbg.chassis.ref = out.ref;
    dbg.chassis.roll = in.imu.roll;
    dbg.chassis.droll = in.imu.gyro.x;
    dbg.chassis.lqr_mode = static_cast<std::uint8_t>(out.mode);
    return motor_output;
}

} // namespace chassis
