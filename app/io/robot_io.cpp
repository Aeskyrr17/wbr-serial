#include "robot_io.hpp"

#include "bsp_can.hpp"
#include "bsp_dma.hpp"
#include "bsp_usart.hpp"
#include "config.hpp"
#include "memory.h"

#include <array>
#include <cstring>

namespace robot::io
{
namespace
{
template <typename T>
struct mailbox
{
    T value{};
    bool valid = false;
};

mailbox<gimbal_command> command_box{};
mailbox<tof_sample> tof_box{};
mailbox<supercap_feedback> supercap_box{};
bsp::dma::buffer<64> tof_dma BSP_DMA_BUFFER{};
std::array<std::uint8_t, 9> tof_parse{};
std::uint8_t tof_parse_size = 0;

template <typename T>
void store(mailbox<T>& box, const T& value) noexcept
{
    box.value = value;
    box.valid = true;
}

template <typename T>
bool load(const mailbox<T>& box, T& value) noexcept
{
    const UINT previous = tx_interrupt_control(TX_INT_DISABLE);
    const bool valid = box.valid;
    if (valid)
        value = box.value;
    tx_interrupt_control(previous);
    return valid;
}

std::uint16_t read_u16(const std::uint8_t* data) noexcept
{
    return static_cast<std::uint16_t>(data[0]) |
           static_cast<std::uint16_t>(data[1] << 8U);
}

void can_rx(bsp::can::bus bus, const bsp::can::rx_frame& frame) noexcept
{
    if (bus != app::can::gimbal_link || frame.len != 8U) return;
    if (frame.id == 0xB2U)
    {
        gimbal_command value{};
        const std::uint8_t flags = frame.data[0];
        value.move = (flags & 0x01U) != 0U;
        value.spin = (flags & 0x04U) != 0U;
        value.gimbal_only = (flags & 0x40U) != 0U;
        value.wants = command_box.value.wants;
        if ((flags & 0x02U) != 0U) ++value.wants.jump;
        if ((flags & 0x08U) != 0U) ++value.wants.turn;
        if ((flags & 0x10U) != 0U) ++value.wants.stair;
        if ((flags & 0x20U) != 0U) ++value.wants.fly;
        value.yaw_current = static_cast<std::int16_t>(read_u16(frame.data + 1));
        value.vx = static_cast<std::int8_t>(frame.data[3]);
        value.vy = static_cast<std::int8_t>(frame.data[4]);
        value.len_level = static_cast<std::int8_t>(frame.data[5]);
        value.trigger_speed = static_cast<std::int8_t>(frame.data[6]);
        value.sequence = command_box.value.sequence + 1U;
        value.received_tick = static_cast<std::uint32_t>(tx_time_get());
        store(command_box, value);
    }
    else if (frame.id == 0xB4U)
    {
        supercap_feedback value{};
        value.state = frame.data[0];
        value.voltage = frame.data[1] * 0.2f;
        value.input_power = static_cast<std::int16_t>(read_u16(frame.data + 2)) * 0.01f;
        value.cap_power = static_cast<std::int16_t>(read_u16(frame.data + 4)) * 0.01f;
        value.remained_energy = static_cast<std::int16_t>(read_u16(frame.data + 6));
        value.sequence = supercap_box.value.sequence + 1U;
        store(supercap_box, value);
    }
}

void push_tof_byte(std::uint8_t byte) noexcept
{
    if (tof_parse_size == 0U && byte != 0x59U) return;
    if (tof_parse_size == 1U && byte != 0x59U)
    {
        tof_parse_size = byte == 0x59U ? 1U : 0U;
        return;
    }
    tof_parse[tof_parse_size++] = byte;
    if (tof_parse_size != tof_parse.size()) return;
    std::uint8_t checksum = 0;
    for (std::size_t i = 0; i < 8U; ++i) checksum = static_cast<std::uint8_t>(checksum + tof_parse[i]);
    const bool valid = checksum == tof_parse[8];
    if (valid)
    {
        tof_sample value{};
        value.distance_cm = static_cast<float>(read_u16(tof_parse.data() + 2));
        value.strength = read_u16(tof_parse.data() + 4);
        value.temperature_c = read_u16(tof_parse.data() + 6) / 8.0f - 256.0f;
        value.sequence = tof_box.value.sequence + 1U;
        value.received_tick = static_cast<std::uint32_t>(tx_time_get());
        value.valid = true;
        store(tof_box, value);
    }
    tof_parse_size = !valid && byte == 0x59U ? 1U : 0U;
    if (tof_parse_size == 1U) tof_parse[0] = 0x59U;
}

void tof_rx(bsp::usart::port port, const bsp::usart::rx_frame& frame) noexcept
{
    if (port != app::uart::tof) return;
    for (std::size_t i = 0; i < frame.len; ++i) push_tof_byte(frame.data[i]);
}

}

types::status init() noexcept
{
    auto status = bsp::can::register_rx_callback(app::can::gimbal_link,
        bsp::can::rx_callback::bind<&can_rx>());
    if (status != types::status::ok) return status;
    status = bsp::can::init(app::can::gimbal_link);
    if (status != types::status::ok) return status;
    status = bsp::usart::init(app::uart::tof, bsp::usart::mode::dma);
    if (status != types::status::ok) return status;
    return bsp::usart::start_rx_to_idle(app::uart::tof, tof_dma.view(),
                                       bsp::usart::rx_callback::bind<&tof_rx>());
}

bool read_command(gimbal_command& output) noexcept { return load(command_box, output); }
bool read_tof(tof_sample& output) noexcept { return load(tof_box, output); }
bool read_supercap(supercap_feedback& output) noexcept { return load(supercap_box, output); }

types::status send_chassis_status(bool initialized, bool red, std::uint8_t level,
                                  std::uint16_t heat_limit, std::uint16_t heat_now,
                                  std::int16_t yaw_velocity) noexcept
{
    std::uint8_t frame[8]{};
    // Explicit little-endian layout matching the legacy packed GCC structure.
    std::uint64_t bits = (initialized ? 1ULL : 0ULL) | (red ? 2ULL : 0ULL) |
                         ((level & 0x0FU) << 2U) |
                         ((static_cast<std::uint64_t>(heat_limit) & 0x1FFU) << 6U) |
                         ((static_cast<std::uint64_t>(heat_now) & 0x1FFU) << 15U) |
                         (static_cast<std::uint64_t>(static_cast<std::uint16_t>(yaw_velocity)) << 24U);
    for (unsigned i = 0; i < 8U; ++i) frame[i] = static_cast<std::uint8_t>(bits >> (i * 8U));
    return bsp::can::transmit(app::can::gimbal_link, app::can::gimbal_link_tx_header, frame, 8U);
}

types::status send_supercap(std::uint8_t state, std::uint8_t fly_extra, float power_limit,
                            std::uint8_t raw_power_limit, std::uint8_t remained_energy) noexcept
{
    std::uint8_t frame[8]{};
    frame[0] = state;
    frame[1] = fly_extra;
    std::memcpy(frame + 2, &power_limit, sizeof(power_limit));
    frame[6] = raw_power_limit;
    frame[7] = remained_energy;
    return bsp::can::transmit(app::can::supercap, 0xC5U, frame, 8U);
}

} // namespace robot::io
