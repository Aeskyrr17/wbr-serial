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

float controller::phi_control(control::pid& pid, control::slope& ramp,
                              const leg_state& leg, float total_phi, float target,
                              float step, bool positive, bool& initialized) noexcept
{
    if (!initialized)
    {
        ramp.set_value(total_phi);
        ramp.set_step(step);
        while (target - total_phi > model::pi) target -= 2.0f * model::pi;
        while (target - total_phi < -model::pi) target += 2.0f * model::pi;
        if (positive && target < total_phi) target += 2.0f * model::pi;
        if (!positive && target > total_phi) target -= 2.0f * model::pi;
        initialized = true;
    }
    pid.ref = ramp.update(target);
    pid.fdb = total_phi;
    pid.update(leg.dphi); // POSITION compatibility: velocity is intentionally unused.
    return pid.result;
}

void controller::reset_phi() noexcept
{
    l_phi_initialized_ = false;
    r_phi_initialized_ = false;
    left_.reset_tracking();
    right_.reset_tracking();
}

chassis_output controller::step(const input& in, robot_state& published) noexcept
{
    apply_tuning();
    const float wheel_velocity = (-in.l_wheel_velocity + in.r_wheel_velocity) *
                                 0.5f * model::wheel_radius;
    odom_.update(in.quaternion, in.accel, wheel_velocity, in.yaw);
    const bool legs_valid = left_.update(in.left, in.pitch, in.dpitch, odom_.az()) &&
                            right_.update(in.right, in.pitch, in.dpitch, odom_.az());
    const auto& left = left_.state();
    const auto& right = right_.state();

    std::array<float, 10> obs{odom_.x(), odom_.v(), in.yaw, in.dyaw,
                              left.alpha, left.dalpha, right.alpha, right.dalpha,
                              in.pitch, in.dpitch};
    std::array<float, 10> ref{};
    chassis_output out{};
    const bool valid = in.imu_valid && in.wheels_online && legs_valid;
    if (!valid || !in.cmd.move) state_ = chassis_state::relax;
    const chassis_state executed = state_;
    chassis_state next = state_;
    lqr_mode mode = lqr_mode::low;

    float l_force = 0.0f, r_force = 0.0f, l_pitch = 0.0f, r_pitch = 0.0f;
    switch (executed)
    {
    case chassis_state::relax:
        l_len_.clear(); r_len_.clear(); l_phi_.clear(); r_phi_.clear(); roll_.clear();
        odom_.reset();
        target_len_ = model::Lmin;
        len_ramp_.set_value(left.len);
        reset_phi();
        if (valid && in.cmd.move)
            next = in.accel[2] < 0.0f ? chassis_state::recover : chassis_state::flatten;
        break;

    case chassis_state::recover:
        odom_.reset();
        tune_pid(l_phi_, robot_tuning.phi_recover);
        tune_pid(r_phi_, robot_tuning.phi_recover);
        if (in.accel[2] > 5.0f && std::fabs(in.pitch) < 0.35f && std::fabs(in.dpitch) < 1.0f)
        {
            reset_phi();
            next = chassis_state::flatten;
        }
        else if (in.pitch > 0.0f)
        {
            l_pitch = phi_control(l_phi_, l_phi_ramp_, left, left_.total_phi(), 0.0f,
                                  0.005f, true, l_phi_initialized_);
            r_pitch = phi_control(r_phi_, r_phi_ramp_, right, right_.total_phi(), 0.0f,
                                  0.005f, true, r_phi_initialized_);
            if (std::fabs(left.phi) < 0.1f && std::fabs(right.phi) < 0.1f)
            { l_pitch = -35.0f; r_pitch = 35.0f; }
        }
        else
        {
            l_pitch = phi_control(l_phi_, l_phi_ramp_, left, left_.total_phi(), model::pi,
                                  0.005f, false, l_phi_initialized_);
            r_pitch = phi_control(r_phi_, r_phi_ramp_, right, right_.total_phi(), model::pi,
                                  0.005f, false, r_phi_initialized_);
            if (std::fabs(left.phi - model::pi) < 0.1f &&
                std::fabs(right.phi - model::pi) < 0.1f)
            { l_pitch = 35.0f; r_pitch = -35.0f; }
        }
        break;

    case chassis_state::flatten:
        if ((left.phi >= 0.7f && left.phi <= 1.4f) &&
            (right.phi >= 0.7f && right.phi <= 1.4f))
        {
            reset_phi(); next = chassis_state::neutral; break;
        }
        if (!left.flat)
        {
            tune_pid(l_phi_, left.phi >= model::pi * 0.4f ? robot_tuning.phi_flatten_fast
                                                          : robot_tuning.phi_flatten_slow);
            l_pitch = phi_control(l_phi_, l_phi_ramp_, left, left_.total_phi(), model::pi,
                                  left.phi >= model::pi * 0.4f ? 0.01f : 0.002f,
                                  left.phi >= model::pi * 0.4f, l_phi_initialized_);
        }
        if (!right.flat)
        {
            tune_pid(r_phi_, right.phi >= model::pi * 0.4f ? robot_tuning.phi_flatten_fast
                                                            : robot_tuning.phi_flatten_slow);
            r_pitch = phi_control(r_phi_, r_phi_ramp_, right, right_.total_phi(), model::pi,
                                  right.phi >= model::pi * 0.4f ? 0.01f : 0.002f,
                                  right.phi >= model::pi * 0.4f, r_phi_initialized_);
        }
        out.Twl = out.Twr = in.cmd.v > 0.0f ? 1.5f : 0.0f;
        if (left.flat && right.flat) { odom_.reset(); reset_phi(); next = chassis_state::neutral; }
        break;

    case chassis_state::neutral:
        odom_.reset();
        tune_pid(l_phi_, robot_tuning.phi_neutral);
        tune_pid(r_phi_, robot_tuning.phi_neutral);
        l_force = len_control(l_len_, left, left.len + (model::Lmin - left.len) * 0.6f) - left.fs;
        r_force = len_control(r_len_, right, right.len + (model::Lmin - right.len) * 0.6f) - right.fs;
        l_pitch = phi_control(l_phi_, l_phi_ramp_, left, left_.total_phi(), 0.5f * model::pi,
                              0.003f, left.phi < model::pi * 0.45f, l_phi_initialized_);
        r_pitch = phi_control(r_phi_, r_phi_ramp_, right, right_.total_phi(), 0.5f * model::pi,
                              0.003f, right.phi < model::pi * 0.45f, r_phi_initialized_);
        // Deterministic fix: the deadband explicitly commands zero wheel torque.
        if (in.cmd.v > 0.005f) out.Twl = out.Twr = 1.5f;
        else if (in.cmd.v < -0.005f) out.Twl = out.Twr = -1.5f;
        else out.Twl = out.Twr = 0.0f;
        if (left.neutral && right.neutral)
        {
            reset_phi(); target_len_ = model::Lmin; len_ramp_.set_value(left.len);
            next = chassis_state::normal;
        }
        break;

    case chassis_state::normal:
    {
        ref = {in.cmd.x, in.cmd.v, in.cmd.yaw, in.cmd.dyaw,
               left.alpha_eq + 0.03f, 0.0f, right.alpha_eq + 0.03f, 0.0f, 0.0f, 0.0f};
        mode = (left.len + right.len) * 0.5f > model::Lswitch ? lqr_mode::high : lqr_mode::low;
        const auto torque = lqr_.update(mode, left.len, right.len, obs, ref);
        out.Twl = torque[0]; out.Twr = torque[1]; l_pitch = torque[2]; r_pitch = torque[3];
        roll_.ref = 0.0f; roll_.fdb = in.roll; roll_.update(in.droll);

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
            { reset_phi(); next = chassis_state::gostair; stairing_ = false; airprotect_count_ = 0; }
        }
        if (in.cmd.events.jump != consumed_events_.jump)
        {
            ++consumed_events_.jump;
            next = chassis_state::jump; jump_ = jump_stage::extending;
            jumpair_count_ = airprotect_count_ = 0; maintain_yaw_ = in.yaw;
        }
        if (in.cmd.spin) next = chassis_state::spin;
        if (!stairing_) target_len_ = flying_ ? model::Lfly :
            std::clamp(in.cmd.len, model::Lmin, model::Lmax);
        const float cmd_len = len_ramp_.update(target_len_);
        l_force = len_control(l_len_, left, cmd_len + roll_.result) +
                  model::gravity_feedforward - left.fs;
        r_force = len_control(r_len_, right, cmd_len - roll_.result) +
                  model::gravity_feedforward - right.fs;
        ++airprotect_count_;
        if (airprotect_count_ >= 500U) len_ramp_.set_step(0.00027f);
        if (left.normal_force + right.normal_force < 0.0f && left.dlen > 0.05f &&
            right.dlen > 0.05f && airprotect_count_ > 500U) next = chassis_state::offground;
        // Legacy final assignment priority: fall protection overrides all earlier transitions.
        if (left.alpha >= 0.8f || right.alpha >= 0.8f) { reset_phi(); next = chassis_state::relax; }
        running_gains_ = true;
        break;
    }

    case chassis_state::offground:
    {
        ref = {obs[0], obs[1], obs[2], obs[3], in.pitch, 0.0f, in.pitch, 0.0f,
               obs[8], obs[9]};
        const auto torque = lqr_.update(lqr_mode::low, left.len, right.len, obs, ref);
        l_pitch = torque[2] * 0.4f; r_pitch = torque[3] * 0.4f;
        ++landing_count_;
        const bool landing = flying_ ? (left.dlen < -0.05f && right.dlen < -0.05f && landing_count_ > 100U)
                                     : (left.dlen < 0.0f && right.dlen < 0.0f);
        if (landing)
        {
            landing_count_ = airprotect_count_ = 0; next = chassis_state::normal;
            target_len_ = model::Lmin; len_ramp_.set_value(left.len);
            len_ramp_.set_step(flying_ ? 0.0006f : 0.0009f); flying_ = false;
        }
        if (left.alpha >= 0.8f || right.alpha >= 0.8f) { reset_phi(); next = chassis_state::relax; }
        break;
    }

    case chassis_state::spin:
    {
        ref = {in.cmd.x, in.cmd.v, in.cmd.yaw, in.cmd.dyaw,
               left.alpha_eq - 0.03f, 0.0f, right.alpha_eq - 0.03f, 0.0f, 0.0f, 0.0f};
        mode = lqr_mode::spin;
        const auto torque = lqr_.update(mode, left.len, right.len, obs, ref);
        out.Twl = torque[0]; out.Twr = torque[1]; l_pitch = torque[2]; r_pitch = torque[3];
        roll_.ref = 0.0f; roll_.fdb = in.roll; roll_.update(in.droll);
        len_ramp_.set_step(model::spin_len_amp * model::spin_len_omega * 0.001f * 1.2f);
        const float cmd_len = len_ramp_.update(in.cmd.len);
        l_force = len_control(l_len_, left, cmd_len + roll_.result) + model::gravity_feedforward - left.fs;
        r_force = len_control(r_len_, right, cmd_len - roll_.result) + model::gravity_feedforward - right.fs;
        if (!in.cmd.spin) next = chassis_state::normal;
        break;
    }

    case chassis_state::gostair:
        tune_pid(l_phi_, robot_tuning.phi_stair); tune_pid(r_phi_, robot_tuning.phi_stair);
        l_pitch = phi_control(l_phi_, l_phi_ramp_, left, left_.total_phi(), model::pi,
                              0.01f, true, l_phi_initialized_);
        r_pitch = phi_control(r_phi_, r_phi_ramp_, right, right_.total_phi(), model::pi,
                              0.01f, true, r_phi_initialized_);
        if (left.flat && right.flat) { reset_phi(); next = chassis_state::neutral; }
        break;

    case chassis_state::jump:
        if (jump_ == jump_stage::dont) next = chassis_state::normal;
        else if (jump_ == jump_stage::extending)
        {
            ref = {0.0f, 0.0f, maintain_yaw_, 0.0f, left.alpha_eq, 0.0f,
                   right.alpha_eq, 0.0f, 0.0f, 0.0f};
            const auto torque = lqr_.update(lqr_mode::low, left.len, right.len, obs, ref);
            out.Twl = torque[0]; out.Twr = torque[1]; l_pitch = torque[2]; r_pitch = torque[3];
            l_force = r_force = 700.0f;
            if (left.len >= 0.3f && right.len >= 0.3f) { jumpair_count_ = 0; jump_ = jump_stage::inair; }
        }
        else
        {
            odom_.reset(); ++jumpair_count_;
            ref = {obs[0], obs[1], obs[2], obs[3], left.alpha_eq, 0.0f,
                   right.alpha_eq, 0.0f, 0.0f, 0.0f};
            const auto torque = lqr_.update(lqr_mode::low, left.len, right.len, obs, ref);
            l_pitch = torque[2] * 0.4f; r_pitch = torque[3] * 0.4f;
            l_force = len_control(l_len_, left, model::Lmin) - left.fs;
            r_force = len_control(r_len_, right, model::Lmin) - right.fs;
            if (jumpair_count_ > 500U)
            { target_len_ = model::Lmin; len_ramp_.set_value(left.len); jump_ = jump_stage::dont; next = chassis_state::normal; }
        }
        break;
    }

    out.Fl = l_force; out.Fr = r_force; out.Tpl = l_pitch; out.Tpr = r_pitch;
    if (executed != chassis_state::relax)
    {
        const auto lt = left_.joint_torque(l_force, l_pitch);
        const auto rt = right_.joint_torque(r_force, r_pitch);
        out.l_joint1 = std::clamp(lt[0], -model::max_joint_torque, model::max_joint_torque);
        out.l_joint4 = std::clamp(lt[1], -model::max_joint_torque, model::max_joint_torque);
        out.r_joint1 = std::clamp(rt[0], -model::max_joint_torque, model::max_joint_torque);
        out.r_joint4 = std::clamp(rt[1], -model::max_joint_torque, model::max_joint_torque);
    }
    out.Twl = std::clamp(out.Twl, -model::max_wheel_torque, model::max_wheel_torque);
    out.Twr = std::clamp(out.Twr, -model::max_wheel_torque, model::max_wheel_torque);
    state_ = next;

    published.x = odom_.x(); published.v = odom_.v();
    published.len = 0.5f * (left.len + right.len);
    published.lalpha = left.alpha; published.ralpha = right.alpha;
    published.pitch = in.pitch; published.flying = flying_; published.stairing = stairing_;
    published.spinning = next == chassis_state::spin;
    published.normal = executed == chassis_state::normal && airprotect_count_ >= 500U;
    published.recovered = in.accel[2] >= 0.0f;
    published.state_executed = executed; published.next_state = next; published.jump = jump_;

    robot_debug.chassis.state = published;
    robot_debug.chassis.left = left; robot_debug.chassis.right = right;
    robot_debug.chassis.output = out;
    std::copy(obs.begin(), obs.end(), robot_debug.chassis.obs);
    std::copy(ref.begin(), ref.end(), robot_debug.chassis.ref);
    robot_debug.chassis.roll = in.roll; robot_debug.chassis.droll = in.droll;
    robot_debug.chassis.normal_force = left.normal_force + right.normal_force;
    robot_debug.chassis.lqr_mode = static_cast<std::uint8_t>(mode);
    return out;
}

} // namespace robot::chassis
