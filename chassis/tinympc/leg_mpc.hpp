#pragma once

#include "generated/leg_mpc_coeffs.hpp"
#include "solver/tiny_mpc.h"

#include <array>
#include <cstdint>

namespace chassis::tinympc
{

enum class solve_status : int
{
    idle = 0,
    solved = 1,
    iteration_limit = 11,
    deadline = 12,
    invalid = 13
};

struct mpc_diag
{
    solve_status status = solve_status::idle;
    bool updated = false;
    bool fallback = false; // The last update used LQR support.
    int iterations = 0;
    float primal = 0.0f;
    float dual = 0.0f;
    std::uint32_t solve_count = 0;
    std::uint32_t fallback_count = 0;
    std::uint32_t missed_periods = 0;
    std::uint32_t last_us = 0;
    std::uint32_t max_us = 0;
};

class leg_mpc
{
public:
    // Seed with the previous commanded total support (N) when entering MPC.
    void reset(const std::array<float, 2>& seed) noexcept;

    // obs/ref: roll, droll, H, dH, with both wheels in contact.
    // Returns left/right total support (N) before axial projection and spring compensation.
    // Call serially with monotonic microseconds. clock supplies the same time base for the budget.
    std::array<float, 2> update(const std::array<float, 4>& obs, const std::array<float, 4>& ref,
        std::uint64_t now_us, tiny_clock clock = nullptr) noexcept;

    const mpc_diag& diagnostics() const noexcept
    {
        return diag_;
    }

private:
    tiny_workspace work_{};
    mpc_diag diag_{};
    std::array<float, 2> held_{model::equilibrium_force, model::equilibrium_force};
    std::uint64_t next_us_ = 0;
    bool entered_ = false;
};

} // namespace chassis::tinympc
