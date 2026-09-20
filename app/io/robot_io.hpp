#pragma once
#include "usertypes.hpp"
#include <cstdint>

namespace robot::io
{
// CAN3 B2 payload. vx = body forward input, vy = body yaw-rate input.
struct command
{
    bool move = false;
    bool gimbal_only = false;
    std::int8_t vx = 0;
    std::int8_t vy = 0;
    std::uint32_t sequence = 0;
    std::uint32_t received_tick = 0;
};
types::status init() noexcept;
bool read_command(command& output) noexcept;
types::status send_chassis_status() noexcept;
}
