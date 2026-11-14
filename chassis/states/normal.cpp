#include "chassis.hpp"

namespace chassis
{
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

} // namespace chassis
