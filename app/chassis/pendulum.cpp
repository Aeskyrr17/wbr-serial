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

bool pendulum::update(const leg_input& input, float pitch, float dpitch, float az) noexcept
{
    const float phi1 = reverse_ ? model::pi - input.q1 : model::pi + input.q1;
    const float phi4 = reverse_ ? -input.q4 : input.q4;
    const float dphi1 = reverse_ ? -input.dq1 : input.dq1;
    const float dphi4 = reverse_ ? -input.dq4 : input.dq4;
    const float tq1 = reverse_ ? -input.tq1 : input.tq1;
    const float tq4 = reverse_ ? -input.tq4 : input.tq4;
    if (!vmc_.resolve(phi1, phi4))
        return false;

    const auto velocity = vmc_.velocity(dphi1, dphi4);
    const auto force = vmc_.generalized_force(tq1, tq4);
    const auto& solved = vmc_.state();
    state_.len = solved.len;
    state_.dlen = velocity[0];
    state_.phi = solved.phi;
    state_.dphi = velocity[1];
    state_.fs = solved.fs;
    state_.force = force[0];
    state_.pitch_torque = force[1];

    if (!phi_initialized_)
    {
        total_phi_ = last_phi_ = state_.phi;
        phi_initialized_ = true;
    }
    else
    {
        total_phi_ += wrap(state_.phi - last_phi_);
        last_phi_ = state_.phi;
    }

    state_.alpha = wrap(state_.phi - 0.5f * model::pi + pitch);
    state_.dalpha = state_.dphi + dpitch;
    state_.alpha_eq = model::alpha_eq[0] + model::alpha_eq[1] * state_.len +
                      model::alpha_eq[2] * state_.len * state_.len;
    const float ca = std::cos(state_.alpha), sa = std::sin(state_.alpha);
    const float vertical_force = (state_.force + state_.fs) * ca;
    // Keep the legacy call-count differences: changing this requires retuning N thresholds.
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

std::array<float, 2> pendulum::joint_torque(float force, float pitch_torque) const noexcept
{
    auto output = vmc_.joint_torque(force, pitch_torque);
    if (reverse_)
    {
        output[0] = -output[0];
        output[1] = -output[1];
    }
    return output;
}

void pendulum::reset_tracking() noexcept
{
    prev_dlen_ = prev_dalpha_ = 0.0f;
    phi_initialized_ = false;
}

} // namespace robot::chassis
