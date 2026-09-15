#pragma once

#include "robot_types.hpp"
#include "usertypes.hpp"

#include <cstdint>

namespace robot::io
{

struct gimbal_command
{
    bool move = false;
    bool jump = false;
    bool spin = false;
    bool turn = false;
    bool stair = false;
    bool fly = false;
    bool gimbal_only = false;
    std::int16_t yaw_current = 0;
    std::int8_t vx = 0;
    std::int8_t vy = 0;
    std::int8_t len_level = 0;
    std::int8_t trigger_speed = 0;
    std::uint32_t sequence = 0;
    std::uint32_t received_tick = 0;
};

struct gimbal_ui
{
    bool reset = false;
    bool fire = false;
    bool bullet_frequency = false;
    bool aim_rune = false;
    std::uint8_t aim_target = 0;
    std::uint8_t aim_x = 0;
    std::uint8_t aim_y = 0;
};

struct tof_sample
{
    float distance_cm = 0.0f;
    float temperature_c = 0.0f;
    std::uint16_t strength = 0;
    std::uint32_t sequence = 0;
    std::uint32_t received_tick = 0;
    bool valid = false;
};

struct supercap_feedback
{
    std::uint8_t state = 0;
    float voltage = 0.0f;
    float input_power = 0.0f;
    float cap_power = 0.0f;
    std::int16_t remained_energy = 0;
    std::uint32_t sequence = 0;
};

types::status init() noexcept;
bool read_command(gimbal_command& output) noexcept;
bool read_ui(gimbal_ui& output) noexcept;
bool read_tof(tof_sample& output) noexcept;
bool read_supercap(supercap_feedback& output) noexcept;
types::status send_chassis_status(bool initialized, bool red, std::uint8_t level,
                                  std::uint16_t heat_limit, std::uint16_t heat_now,
                                  std::int16_t yaw_velocity) noexcept;
types::status send_supercap(std::uint8_t state, std::uint8_t fly_extra,
                            float power_limit, std::uint8_t raw_power_limit,
                            std::uint8_t remained_energy) noexcept;

} // namespace robot::io
