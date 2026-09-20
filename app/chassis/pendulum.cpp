#include "pendulum.hpp"

#include "chassis_model.hpp"

#include <cmath>

namespace robot::chassis
{
namespace
{
float wrap(float value) noexcept
{
    return std::remainder(value, 2.0f * model::pi);
}
}

bool pendulum::update(const ::motors::feedback& joint1, const ::motors::feedback& joint4,
                      const ::imu::state& imu, float az) noexcept
{
    // Protocol -> joint velocity scale exactly once; position/torque are unchanged.
    const float dq1 = joint1.velocity * model::joint_velocity_feedback_scale;
    const float dq4 = joint4.velocity * model::joint_velocity_feedback_scale;
    const float phi1 = reverse_ ? model::pi - joint1.position : model::pi + joint1.position;
    const float phi4 = reverse_ ? -joint4.position : joint4.position;
    const float dphi1 = reverse_ ? -dq1 : dq1;
    const float dphi4 = reverse_ ? -dq4 : dq4;
    const float tq1 = reverse_ ? -joint1.torque : joint1.torque;
    const float tq4 = reverse_ ? -joint4.torque : joint4.torque;
    state_.phi1 = phi1;
    state_.phi4 = phi4;
    state_.valid = vmc_.resolve(phi1, phi4);
    if (!state_.valid)
        return false;

    const auto velocity = vmc_.velocity(dphi1, dphi4);
    const auto load = vmc_.generalized_force(tq1, tq4); // [F_L, tau_l]
    const auto& solved = vmc_.state();
    state_.len = solved.len;
    state_.dlen = velocity[0];
    state_.phi = solved.phi;
    state_.dphi = velocity[1];
    state_.fs = solved.fs;
    state_.spring_len = solved.spring_len;
    state_.leg_force = load[0];
    state_.hip_torque = load[1];

    if (!phi_initialized_)
    {
        total_phi_ = prev_phi_ = state_.phi;
        phi_initialized_ = true;
    }
    else
    {
        total_phi_ += wrap(state_.phi - prev_phi_);
        prev_phi_ = state_.phi;
    }

    state_.total_phi = total_phi_;
    state_.alpha = wrap(state_.phi - 0.5f * model::pi + imu.pitch);
    state_.dalpha = state_.dphi + imu.gyro_p;
    state_.alpha_eq = model::alpha_eq[0] + model::alpha_eq[1] * state_.len +
                      model::alpha_eq[2] * state_.len * state_.len;
    const float ca = std::cos(state_.alpha), sa = std::sin(state_.alpha);
    const float vertical_force = (state_.leg_force + state_.fs) * ca;
    //! TODO: 确定是否要除以dt
    const float ddlen = state_.dlen - prev_dlen_;
    const float ddalpha = state_.dalpha - prev_dalpha_;
    const float zw = (az - model::gravity) - ddlen * ca +
                     2.0f * state_.dlen * state_.dalpha * sa +
                     state_.len * ddalpha * sa +
                     state_.len * state_.dalpha * state_.dalpha * ca;
    state_.normal_force = vertical_force + model::wheel_mass * model::gravity +
                          model::wheel_mass * zw;
    state_.neutral = std::fabs(state_.alpha) < 0.5f;
    state_.flat = state_.phi >= 2.5f && state_.phi <= 3.1f;
    prev_dlen_ = state_.dlen;
    prev_dalpha_ = state_.dalpha;
    return true;
}

std::array<float, 2> pendulum::joint_torque(float leg_force, float hip_torque) const noexcept
{
    auto output = vmc_.joint_torque(leg_force, hip_torque);
    if (reverse_)
    {
        output[0] = -output[0];
        output[1] = -output[1];
    }
    return output;
}

void pendulum::reset_tracking() noexcept
{
    phi_initialized_ = false;
}

} // namespace robot::chassis
