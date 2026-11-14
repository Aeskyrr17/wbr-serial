#pragma once

// H723 resource selection for the migrated chassis. This declares resources;
// it does not register motors, start buses, or enable actuator output.
#include "bsp_can.hpp"
#include "dm8009p.hpp"
#include "m3508.hpp"
#include "user_config.hpp"

#include <cstdint>

namespace chassis::cfg
{
inline constexpr bsp::can::fd_config joint_bus{
    bsp::can::handle::fdcan1, bsp::can::id_type::standard, true, true, 13, 13};
inline constexpr bsp::can::config wheel_bus{
    bsp::can::handle::fdcan2, bsp::can::id_type::standard};

inline constexpr std::uint16_t l_joint1_cmd_id = 0x02;
inline constexpr std::uint16_t l_joint1_fdb_id = 0x06;
inline constexpr std::uint16_t l_joint4_cmd_id = 0x01;
inline constexpr std::uint16_t l_joint4_fdb_id = 0x05;
inline constexpr std::uint16_t r_joint1_cmd_id = 0x04;
inline constexpr std::uint16_t r_joint1_fdb_id = 0x08;
inline constexpr std::uint16_t r_joint4_cmd_id = 0x03;
inline constexpr std::uint16_t r_joint4_fdb_id = 0x07;
inline constexpr std::uint16_t l_wheel = 0x201;
inline constexpr std::uint16_t r_wheel = 0x202;
inline constexpr auto wheel_ratio = motor::dji::m3508::gear_ratio::xroll_gearbox;
inline constexpr auto joint_mode = motor::dm::firmware_mode::mit;

// BMI088 wiring and PS2 SPI/CS are supplied by the template's board and
// user_config, not duplicated here. PS2 and IMU require explicit startup.
inline constexpr auto ps2_spi = user_config::ps2_spi;
inline constexpr auto ps2_cs = user_config::ps2_cs;
} // namespace chassis::cfg
