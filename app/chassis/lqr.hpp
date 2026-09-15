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

private:
    std::array<float, 40> gain_{};
};

} // namespace robot::chassis
