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

/**
 * @brief 双腿pid的tuning赋值。
 */
void controller::apply_pid_tuning() noexcept
{
    const auto& len = (fsm_ == chassis_fsm::normal || fsm_ == chassis_fsm::spin) ? tuning.len_run : tuning.len_startup;
    l_len_.tune(len.kp, len.ki, len.kd);
    r_len_.tune(len.kp, len.ki, len.kd);
    roll_.tune(tuning.roll.kp, tuning.roll.ki, tuning.roll.kd);
}

/**
 * @brief 腿长的pid控制
 */
float controller::len_pid_control(control::pid& pid, const leg_state& leg, float ref) noexcept
{
    pid.ref = ref;
    pid.fdb = leg.len;
    pid.update(leg.dlen);
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
    out.motor.l_tw = torque[0];
    out.motor.r_tw = torque[1];
    out.force.l_tp = torque[2];
    out.force.r_tp = torque[3];

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

    target_len_ = std::clamp(in.cmd.len, cfg::Lmin, cfg::Lmax);
    const float cmd_len = len_ramp_.update(target_len_);
    out.force.l_f = len_pid_control(l_len_, left, cmd_len + roll_.result) +
                    cfg::gravity_feedforward - left.spring.fs;
    out.force.r_f = len_pid_control(r_len_, right, cmd_len - roll_.result) +
                    cfg::gravity_feedforward - right.spring.fs;

    auto& debug = out.debug;
    debug.l_len = {l_len_.ref, l_len_.fdb, l_len_.result, true};
    debug.r_len = {r_len_.ref, r_len_.fdb, r_len_.result, true};
    debug.roll = {roll_.ref, roll_.fdb, roll_.result, true};
}

void controller::update_leg_lqr(const inputs& in, step_out& out) noexcept
{
    const auto& left = in.left;
    const auto& right = in.right;
    target_len_ = std::clamp(in.cmd.len, cfg::Lmin, cfg::Lmax);
    const float previous_len = len_ramp_.value();
    const float cmd_len = len_ramp_.update(target_len_);
    const float cmd_dlen = (cmd_len - previous_len) / leg_lqr_model::sample_time;
    const auto leg_obs = leg_lqr::observe(in.imu, left, right);
    const auto leg_ref = leg_lqr::reference(cmd_len, cmd_dlen, tuning.alpha_trim);
    const auto support = leg_lqr_.update(leg_obs, leg_ref);
    // The model includes roll projection; remove only the sagittal leg projection here.
    // VMC consumes active axial force. Passive spring support is subtracted once.
    out.force.l_f = support[0] / std::cos(left.alpha) - left.spring.fs;
    out.force.r_f = support[1] / std::cos(right.alpha) - right.spring.fs;

    auto& leg_debug = out.debug.leg_lqr;
    std::copy(leg_obs.begin(), leg_obs.end(), leg_debug.obs);
    std::copy(leg_ref.begin(), leg_ref.end(), leg_debug.ref);
    std::copy(support.begin(), support.end(), leg_debug.support_force);
    leg_debug.active = true;
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
    case chassis_fsm::spring_test: //! 气弹簧测试用
        out.force.l_f = -in.left.spring.fs;
        break;
    default:
        out.fsm_next = chassis_fsm::relax;
        break;
    }
}

offground_state controller::is_offground(const leg_state& left, const leg_state& right) const noexcept
{
    if (left.offground && right.offground)
    {
        return offground_state::both_off_ground;
    }
    if (left.offground)
    {
        return offground_state::left_off_ground;
    }
    if (right.offground)
    {
        return offground_state::right_off_ground;
    }
    return offground_state::on_ground;
}

/**
 * @brief
 *
 * @param imu
 * @param fdb
 * @param cmd
 * @param now
 * @param output 电机输出
 * @param state  底盘状态
 */
void controller::step(const ::imu::attitude::state& imu, const motor_fdb& fdb,
                      const chassis_cmd& cmd, std::uint32_t now,
                      motor_cmd& output, chassis_state& state) noexcept
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
    if (!valid || !cmd.move) //imu或电机反馈异常，或者没有移动指令，relax
    {
        fsm_ = chassis_fsm::relax;
        roll_.clear();
        len_ramp_.set_step(0.0002f);
    }
    else if (cfg::spring_test)
    {
        fsm_ = chassis_fsm::spring_test;
    }
    apply_pid_tuning(); // pid腿长的tuning

    const chassis_fsm executed = fsm_;
    //TODO：检测各种状态，后续如果多起来考虑封装
    left_.detect_offground(executed == chassis_fsm::normal || executed == chassis_fsm::airborne);
    right_.detect_offground(executed == chassis_fsm::normal || executed == chassis_fsm::airborne);
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

    output = out.motor;
    joint_torque_debug joint_raw{};
    if (executed != chassis_fsm::relax)
    {
        const auto lt = left_.joint_torque(out.force.l_f, out.force.l_tp);
        const auto rt = right_.joint_torque(out.force.r_f, out.force.r_tp);
        joint_raw = {lt[0], lt[1], rt[0], rt[1]};
        output.l_tj1 = std::clamp(lt[0], -cfg::max_joint_torque, cfg::max_joint_torque);
        output.l_tj4 = std::clamp(lt[1], -cfg::max_joint_torque, cfg::max_joint_torque);
        output.r_tj1 = std::clamp(rt[0], -cfg::max_joint_torque, cfg::max_joint_torque);
        output.r_tj4 = std::clamp(rt[1], -cfg::max_joint_torque, cfg::max_joint_torque);
    }
    output.l_tw = std::clamp(output.l_tw, -cfg::max_wheel_torque, cfg::max_wheel_torque);
    output.r_tw = std::clamp(output.r_tw, -cfg::max_wheel_torque, cfg::max_wheel_torque);
    fsm_ = out.fsm_next;

    state.x = odom_.x();
    state.v = odom_.v();
    state.len = 0.5f * (left.len + right.len);
    state.lalpha = left.alpha;
    state.ralpha = right.alpha;
    state.pitch = in.imu.pitch;
    state.yaw = in.imu.total_yaw;
    state.v_cmd_enabled = out.v_cmd_enabled;
    state.fsm_executed = executed;
    state.fsm_next = out.fsm_next;
    state.normal_force = left.normal_force + right.normal_force;
    state.offground = is_offground(left, right);

    ++dbg.chassis.sample_seq;
    dbg.chassis.sample_tick = now;
    dbg.chassis.len_cmd = cmd.len;
    dbg.chassis.len_target = target_len_;
    dbg.chassis.len_ref = len_ramp_.value();
    dbg.chassis.l_contact = {l_contact_.off_timer.count(), l_contact_.touch_timer.count(),
                             l_contact_.dlen_peak, l_contact_.air_seen, l_contact_.touchdown};
    dbg.chassis.r_contact = {r_contact_.off_timer.count(), r_contact_.touch_timer.count(),
                             r_contact_.dlen_peak, r_contact_.air_seen, r_contact_.touchdown};
    dbg.chassis.state = state;
    dbg.chassis.left = left;
    dbg.chassis.right = right;
    ++dbg.chassis.sample_seq;
    dbg.chassis.force = out.force;
    dbg.chassis.motor = output;
    dbg.chassis.joint_raw = joint_raw;
    dbg.chassis.control = out.debug;
    dbg.chassis.obs = obs;
    dbg.chassis.ref = out.ref;
    dbg.chassis.roll = in.imu.roll;
    dbg.chassis.droll = in.imu.gyro.x;
    dbg.chassis.lqr_mode = static_cast<std::uint8_t>(out.mode);
}

} // namespace chassis
