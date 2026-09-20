#pragma once

#include <array>

namespace robot::chassis
{

struct vmc_state
{
    float len = 0.0f;
    float phi = 0.0f;
    float fs = 0.0f;
    float spring_len = 0.0f;
    std::array<float, 4> J{};
    std::array<float, 4> JT{};
    std::array<float, 4> JT_inv{};
};

class vmc
{
public:
    bool resolve(float phi1, float phi4) noexcept;
    std::array<float, 2> velocity(float dphi1, float dphi4) const noexcept;
    std::array<float, 2> joint_torque(float leg_force, float hip_torque) const noexcept;
    // Generalized load [F_L (N), tau_l (Nm)] conjugate to [len, phi].
    std::array<float, 2> generalized_force(float tq1, float tq4) const noexcept;
    [[nodiscard]] const vmc_state& state() const noexcept { return state_; }

private:
    vmc_state state_{};
};

} // namespace robot::chassis
