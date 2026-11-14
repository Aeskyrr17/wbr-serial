#include "chassis.hpp"

namespace chassis
{
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

} // namespace chassis
