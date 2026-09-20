#include "robot_io.hpp"

#include "bsp_can.hpp"
#include "config.hpp"
#include "tx_api.h"

namespace robot::io
{
namespace
{
// Written in CAN IRQ context; read_command copies it with interrupts masked.
command latest_command{};
bool command_received = false;

void can_rx(bsp::can::bus bus, const bsp::can::rx_frame& frame) noexcept
{
    if (bus != app::can::gimbal_link || frame.len != 8U) return;
    if (frame.id == 0xB2U)
    {
        command value{};
        const std::uint8_t flags = frame.data[0];
        value.move = (flags & 0x01U) != 0U;
        value.gimbal_only = (flags & 0x40U) != 0U;
        value.vx = static_cast<std::int8_t>(frame.data[3]);
        value.vy = static_cast<std::int8_t>(frame.data[4]);
        value.sequence = latest_command.sequence + 1U;
        value.received_tick = static_cast<std::uint32_t>(tx_time_get());
        latest_command = value;
        command_received = true;
    }
}

} // namespace

types::status init() noexcept
{
    auto status = bsp::can::register_rx_callback(app::can::gimbal_link,
        bsp::can::rx_callback::bind<&can_rx>());
    if (status != types::status::ok) return status;
    status = bsp::can::init(app::can::gimbal_link);
    if (status != types::status::ok) return status;
    return status;
}

bool read_command(command& output) noexcept
{
    const UINT prev_interrupts = tx_interrupt_control(TX_INT_DISABLE);
    const bool received = command_received;
    if (received)
    {
        output = latest_command;
    }
    tx_interrupt_control(prev_interrupts);
    return received;
}

types::status send_chassis_status() noexcept
{
    const std::uint8_t frame[8]{};
    return bsp::can::transmit(app::can::gimbal_link, app::can::gimbal_link_tx_header, frame, 8U);
}

} // namespace robot::io
