#include "chassis.hpp"

namespace chassis
{
void controller::run_flatten(const inputs& in, step_out& out) noexcept
{
    const auto& left = in.left;
    const auto& right = in.right;

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
        out.force.l_tp = update_phi_control(
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
        out.force.r_tp = update_phi_control(
            r_phi_, right, right_.total_phi(), model::pi,
            right.phi >= model::pi * 0.4f ? 0.01f : 0.002f,
            right.phi >= model::pi * 0.4f);
    }

    out.motor.l_tw = out.motor.r_tw = in.cmd.v > 0.0f ? 1.5f : 0.0f;
    if (left.flat && right.flat)
    {
        odom_.reset();
        reset_phi_control();
        out.fsm_next = chassis_fsm::neutral;
    }

    auto& debug = out.debug;
    if (!left.flat)
    {
        debug.l_phi = {l_phi_.pid.ref, l_phi_.pid.fdb, l_phi_.pid.result, true};
    }
    if (!right.flat)
    {
        debug.r_phi = {r_phi_.pid.ref, r_phi_.pid.fdb, r_phi_.pid.result, true};
    }
}

} // namespace chassis
