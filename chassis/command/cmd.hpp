#pragma once

#include "robot_types.hpp"
#include "ps2.hpp"
#include "ramp.hpp"
#include "control_map.hpp"

namespace chassis::cmd
{
// PS2 intent before ramps and reference generation.
struct motion_req
{
    float v = 0.0f;
    float dyaw = 0.0f;
    float len_rate = 0.0f; // Requested leg length change, m/s.
    bool move = false;
    bool spin = false;
    bool jump = false;
    bool online = false;
    std::uint32_t received_tick = 0;
};


motion_req map_ps2(const remotes::ps2::sample& rc, bool online, std::uint32_t now) noexcept;

class mapper
{
public:
    cmd_frame step(const motion_req& request, const chassis_state& state,
                       std::uint32_t now) noexcept;
private:
    void reset_position(float x) noexcept;
    void update_position(chassis_cmd& cmd, const chassis_state& state) noexcept;

    math::ramp v_ramp_{0.0f, mapping::v_step};
    math::ramp yaw_ramp_{0.0f, mapping::dyaw_step};
    std::uint32_t sequence_ = 0;
    bool prev_spin_ = false;
    bool hold_x_ = false;
    bool stopping_x_ = false;
    int stop_direction_ = 0;
    unsigned stop_confirm_count_ = 0;
    bool hold_yaw_ = false;
    float len_ref_ = 0.0f; // Clamped to the model range on every step.
    float held_x_ = 0.0f;
    float held_yaw_ = 0.0f;
};
}
