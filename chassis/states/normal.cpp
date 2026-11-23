#include "cfg.hpp"
#include "chassis.hpp"

namespace chassis
{
void controller::run_normal(const inputs& in, step_out& out) noexcept
{
    const auto& left = in.left;
    const auto& right = in.right;
    const auto mode = (left.len + right.len) * 0.5f > cfg::Lswitch ? lqr_mode::high : lqr_mode::low;
    update_balance(in, out, mode, tuning.alpha_trim);
    update_leg_lqr(in, out);
    
    ++settle_count_;
    out.v_cmd_enabled = settle_count_ >= 500U;// 进入normal后500ms后才允许速度控制
    if (out.v_cmd_enabled) 
    {
        len_ramp_.set_step(0.00027f);
    }
    if (in.cmd.spin && out.v_cmd_enabled)
    {
        out.fsm_next = chassis_fsm::spin;
    }
     //! normal进airborne的离地
    const bool off_ready = settle_count_ >= cfg::landing_lockout_count; //在这段时间内忽略离地（由于从airborne回到normal后settlecount会被清零）
    const bool left_airborne =
        l_contact_.off_timer.update(off_ready && left.offground && left.dlen > cfg::offground_extend_rate);
    const bool right_airborne =
        r_contact_.off_timer.update(off_ready && right.offground && right.dlen > cfg::offground_extend_rate);
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
