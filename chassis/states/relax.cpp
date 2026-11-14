#include "chassis.hpp"

#include <algorithm>

namespace chassis
{
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

} // namespace chassis
