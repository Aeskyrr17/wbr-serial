#include "leg_mpc.hpp"

#include "leg_lqr.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace chassis::tinympc
{
namespace
{

constexpr std::uint64_t period_us = model::sample_us;
constexpr std::uint32_t budget_us = 500;
constexpr int max_iterations = 200;
constexpr float force_scale = model::scale[4];
constexpr float force_tolerance = 0.02f; // N, tolerance for accepting a solver result.

static_assert(period_us > 0);
static_assert(force_scale > 0.0f && force_scale == model::scale[5]);
static_assert(model::min_force <= model::equilibrium_force &&
              model::equilibrium_force <= model::max_force);
static_assert(model::max_delta > 0.0f);

constexpr tiny_cache cache{
    model::a, model::b, model::q, model::r, model::pinf, model::k, model::inv, model::ambkt,
    model::rho};

} // namespace

void leg_mpc::reset(const std::array<float, 2>& seed) noexcept
{
    entered_ = false;
    diag_.status = solve_status::idle;
    diag_.updated = false;
    diag_.fallback = false;
    diag_.iterations = 0;
    diag_.primal = 0.0f;
    diag_.dual = 0.0f;
    for (unsigned i = 0; i < held_.size(); ++i)
    {
        held_[i] = std::isfinite(seed[i])
                       ? std::clamp(seed[i], model::min_force, model::max_force)
                       : model::equilibrium_force;
    }
}

std::array<float, 2> leg_mpc::update(
    const std::array<float, 4>& obs, const std::array<float, 4>& ref,
    std::uint64_t now_us, tiny_clock clock) noexcept
{
    diag_.updated = false;
    if (entered_ && now_us < next_us_)
    {
        return held_;
    }

    if (entered_)
    {
        const auto missed = (now_us - next_us_) / period_us;
        diag_.missed_periods += static_cast<std::uint32_t>(missed);
        next_us_ += (missed + 1U) * period_us;
    }
    else
    {
        next_us_ = now_us + period_us;
        entered_ = true;
    }

    const auto start = clock ? clock() : now_us;
    const auto finite = [](float value) { return std::isfinite(value); };
    const bool valid = std::all_of(obs.begin(), obs.end(), finite) &&
                       std::all_of(ref.begin(), ref.end(), finite);
    auto status = solve_status::invalid;
    auto support = held_;
    // Start each horizon with clean primal/slack/dual data.
    std::memset(&work_, 0, sizeof(work_));
    if (valid)
    {
        work_.x[0][0] = obs[0] / model::scale[0];
        work_.x[0][1] = obs[1] / model::scale[1];
        work_.x[0][2] = (obs[2] - ref[2]) / model::scale[2];
        work_.x[0][3] = obs[3] / model::scale[3];
        for (unsigned i = 0; i < held_.size(); ++i)
        {
            work_.x[0][4 + i] = (held_[i] - model::equilibrium_force) / force_scale;
        }
        for (unsigned t = 0; t < TINY_KNOTS; ++t)
        {
            work_.ref[t][0] = ref[0] / model::scale[0];
            work_.ref[t][1] = ref[1] / model::scale[1];
            work_.ref[t][3] = ref[3] / model::scale[3];
            std::copy_n(work_.x[0], TINY_NX, work_.v[t]);
        }
        work_.force_min = (model::min_force - model::equilibrium_force) / force_scale;
        work_.force_max = (model::max_force - model::equilibrium_force) / force_scale;
        work_.delta_max = model::max_delta / force_scale;
        status = static_cast<solve_status>(
            tiny_mpc_solve(&work_, &cache, max_iterations, clock, start + budget_us));
        if (clock && clock() - start >= budget_us)
        {
            status = solve_status::deadline;
        }

        if (status == solve_status::solved)
        {
            for (unsigned i = 0; i < support.size(); ++i)
            {
                support[i] += work_.z[0][i] * force_scale;
                if (!std::isfinite(support[i]) || support[i] < model::min_force - force_tolerance ||
                    support[i] > model::max_force + force_tolerance)
                {
                    status = solve_status::invalid;
                }
            }
        }
    }

    const bool fallback = status != solve_status::solved && valid;
    if (fallback)
    {
        support = leg_lqr{}.update(obs, ref);
    }
    for (unsigned i = 0; i < held_.size(); ++i)
    {
        if (std::isfinite(support[i]))
        {
            const float lo = std::max(model::min_force, held_[i] - model::max_delta);
            const float hi = std::min(model::max_force, held_[i] + model::max_delta);
            held_[i] = std::clamp(support[i], lo, hi);
        }
    }

    diag_.updated = true;
    diag_.fallback = fallback;
    diag_.status = status;
    diag_.iterations = work_.iterations;
    diag_.primal = work_.primal;
    diag_.dual = work_.dual;
    ++diag_.solve_count;
    if (status != solve_status::solved)
    {
        ++diag_.fallback_count;
    }
    diag_.last_us = clock ? static_cast<std::uint32_t>(clock() - start) : 0U;
    diag_.max_us = std::max(diag_.max_us, diag_.last_us);
    return held_;
}

} // namespace chassis::tinympc
