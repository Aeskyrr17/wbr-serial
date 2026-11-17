#include "chassis.hpp"

#include <cmath>

namespace chassis
{
void controller::run_spin(const inputs& in, step_out& out) noexcept
{
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

} // namespace chassis
