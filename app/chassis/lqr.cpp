#include "lqr.hpp"

#include "chassis_model.hpp"
#include "model/lqr_coeffs.hpp"

#include <algorithm>
#include <cmath>

namespace robot::chassis
{

std::array<float, 4> lqr::update(lqr_mode mode, float llen, float rlen,
                                 const std::array<float, 10>& obs,
                                 const std::array<float, 10>& ref) noexcept
{
    llen = std::round(std::clamp(llen, model::Lmin, model::Lmax) * 100.0f) / 100.0f;
    rlen = std::round(std::clamp(rlen, model::Lmin, model::Lmax) * 100.0f) / 100.0f;
    const float (*coeff)[6] = LQRKLowcoeffs;
    if (mode == lqr_mode::high) coeff = LQRKHighcoeffs;
    else if (mode == lqr_mode::spin) coeff = LQRKSpincoeffs;

    const float basis[6] = {1.0f, llen, rlen, llen * llen, llen * rlen, rlen * rlen};
    for (std::size_t row = 0; row < gain_.size(); ++row)
    {
        float value = 0.0f;
        for (std::size_t column = 0; column < 6; ++column)
            value += coeff[row][column] * basis[column];
        gain_[row] = value;
    }

    std::array<float, 4> output{};
    for (std::size_t row = 0; row < output.size(); ++row)
        for (std::size_t column = 0; column < obs.size(); ++column)
            output[row] += gain_[row * obs.size() + column] * (obs[column] - ref[column]);
    raw_output_ = output;
    output[0] = std::clamp(output[0], -model::max_wheel_torque, model::max_wheel_torque);
    output[1] = std::clamp(output[1], -model::max_wheel_torque, model::max_wheel_torque);
    output[2] = std::clamp(output[2], -model::max_joint_torque, model::max_joint_torque);
    output[3] = std::clamp(output[3], -model::max_joint_torque, model::max_joint_torque);
    return output;
}

} // namespace robot::chassis
