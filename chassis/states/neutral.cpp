#include "chassis.hpp"

namespace chassis
{
void controller::run_neutral(const inputs& in, step_out& out) noexcept
{
    const auto& left = in.left;
    const auto& right = in.right;

    out.v_cmd_enabled = true;
    odom_.reset();
    const auto& param = tuning.phi_neutral;
    l_phi_.pid.tune(param.kp, param.ki, param.kd);
    r_phi_.pid.tune(param.kp, param.ki, param.kd);
    out.force.l_f = len_pid_control(l_len_, left, left.len + (cfg::Lmin - left.len) * 0.6f) -
                    left.spring.fs;
    out.force.r_f = len_pid_control(r_len_, right, right.len + (cfg::Lmin - right.len) * 0.6f) -
                    right.spring.fs;
    out.force.l_tp = update_phi_control(l_phi_, left, left_.total_phi(),
                                0.5f * model::pi, 0.003f,
                                left.phi < model::pi * 0.45f);
    out.force.r_tp = update_phi_control(r_phi_, right, right_.total_phi(),
                                0.5f * model::pi, 0.003f,
                                right.phi < model::pi * 0.45f);
    // Deterministic fix: the deadband explicitly commands zero wheel torque.
    if (in.cmd.v > 0.005f)
        out.motor.l_tw = out.motor.r_tw = 1.5f;
    else if (in.cmd.v < -0.005f)
        out.motor.l_tw = out.motor.r_tw = -1.5f;
    else
        out.motor.l_tw = out.motor.r_tw = 0.0f;

    if (left.neutral && right.neutral)
    {
        reset_phi_control();
        target_len_ = cfg::Lmin;
        len_ramp_.set_value(left.len);
        out.fsm_next = chassis_fsm::normal;
    }

    auto& debug = out.debug;
    debug.l_len = {l_len_.ref, l_len_.fdb, l_len_.result, true};
    debug.r_len = {r_len_.ref, r_len_.fdb, r_len_.result, true};
    debug.l_phi = {l_phi_.pid.ref, l_phi_.pid.fdb, l_phi_.pid.result, true};
    debug.r_phi = {r_phi_.pid.ref, r_phi_.pid.fdb, r_phi_.pid.result, true};
}

} // namespace chassis
