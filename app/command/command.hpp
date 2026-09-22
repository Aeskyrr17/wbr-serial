#pragma once

#include "robot_types.hpp"
#include "robot_io.hpp"
#include "remoter.hpp"
#include "slope.hpp"
#include "control_map.hpp"

namespace robot::command
{
// Source-independent operator intent, before ramps and reference generation.
struct motion_request
{
    float v = 0.0f;
    float dyaw = 0.0f;
    float len_rate = 0.0f; // Requested leg length change, m/s.
    bool move = false;
    bool spin = false;
    bool online = false;
    input_source source = input_source::can;
    std::uint32_t received_tick = 0;
};

motion_request map_can(const io::command& raw, std::uint32_t now) noexcept;

motion_request map_ps2(const remoter::state& rc, std::uint32_t now) noexcept;

class mapper
{
public:
    command_frame step(const motion_request& request, const robot_state& state,
                       std::uint32_t now) noexcept;
private:
    void reset_position(float x) noexcept;
    void update_position(chassis_command& cmd, const robot_state& state) noexcept;

    control::slope v_ramp_{0.0f, mapping::v_step};
    control::slope yaw_ramp_{0.0f, mapping::dyaw_step};
    input_source source_ = input_source::can;
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
