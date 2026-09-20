#pragma once

#include "robot_types.hpp"

#include <array>

namespace robot::chassis
{

class lqr
{
public:
    std::array<float, 4> update(lqr_mode mode, float llen, float rlen,
                                const std::array<float, 10>& obs,
                                const std::array<float, 10>& ref) noexcept;
    [[nodiscard]] const std::array<float, 40>& gain() const noexcept { return gain_; }

    [[nodiscard]] const std::array<float, 4>& raw_output() const noexcept { return raw_output_; }

private:
    std::array<float, 40> gain_{};
    std::array<float, 4> raw_output_{};
};

} // namespace robot::chassis
