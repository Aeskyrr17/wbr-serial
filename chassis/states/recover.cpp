#include "chassis.hpp"

#include <cmath>

namespace chassis
{
void controller::run_recover(const inputs& in, step_out& out) noexcept
{
    odom_.reset();
    if (in.imu.accel.z > 5.0f && std::fabs(in.imu.pitch) < 0.35f &&
        std::fabs(in.imu.gyro.y) < 1.0f)
    {
        relax_left();
        relax_right();
        out.fsm_next = chassis_fsm::flatten;
        return;
    }

    const auto& param = tuning.phi_recover;
    l_phi_.pid.tune(param.kp, param.ki, param.kd);
    r_phi_.pid.tune(param.kp, param.ki, param.kd);
    const bool positive = in.imu.pitch > 0.0f;
    const float target = positive ? 0.0f : model::pi;
    out.force.l_tp = update_phi_control(l_phi_, in.left, left_.total_phi(), target, 0.005f,
                                           positive, out.debug.l_phi);
    out.force.r_tp = update_phi_control(r_phi_, in.right, right_.total_phi(), target, 0.005f,
                                           positive, out.debug.r_phi);
    if (std::fabs(in.left.phi - target) < 0.1f &&
        std::fabs(in.right.phi - target) < 0.1f)
    {
        out.force.l_tp = positive ? -35.0f : 35.0f;
        out.force.r_tp = positive ? 35.0f : -35.0f;
    }
}

} // namespace chassis
