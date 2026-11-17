#include "attitude.hpp"
#include "bmi088.hpp"
#include "bsp_can.hpp"
#include "bsp_dwt.hpp"
#include "bsp_spi.hpp"
#include "main.h"
#include "chassis.hpp"
#include "command/cmd.hpp"
#include "dm_manager.hpp"
#include "dji_manager.hpp"
#include "peripherals.hpp"
#include "ps2.hpp"
#include "robot_debug.hpp"
#include "tx_api.h"
#include "tuning.hpp"

#include <cstdint>

namespace chassis
{
volatile UINT start_status = TX_SUCCESS;
namespace
{
TX_THREAD control_thread{};
TX_THREAD command_thread{};
alignas(8) std::uint8_t control_stack[8192]{};
alignas(8) std::uint8_t command_stack[4096]{};
msg::topic<cmd_frame> command_topic{};
msg::topic<chassis_state> state_topic{};
msg::subscriber<cmd_frame> command_sub{};
msg::subscriber<chassis_state> state_sub{};
motor::dm::config_t joint_params{
    cfg::joint_position_limit, cfg::joint_velocity_limit, cfg::joint_torque_limit,
    cfg::joint_velocity_scale, cfg::joint_wrap_position};
motor::dm::dm8009p l_joint1{joint_params, cfg::joint_mode};
motor::dm::dm8009p l_joint4{joint_params, cfg::joint_mode};
motor::dm::dm8009p r_joint1{joint_params, cfg::joint_mode};
motor::dm::dm8009p r_joint4{joint_params, cfg::joint_mode};
motor::dji::m3508 l_wheel{cfg::wheel_ratio};
motor::dji::m3508 r_wheel{cfg::wheel_ratio};
motor::dm::manager joints{};
motor::dji::manager wheels{};
std::uint8_t ps2_tx[remotes::ps2::frame_size]{};
std::uint8_t ps2_rx[remotes::ps2::frame_size]{};
remotes::ps2::spi ps2{cfg::ps2_spi, cfg::ps2_cs, ps2_tx, ps2_rx};

bool init_motors() noexcept
{
    const auto bus = cfg::joint_bus.instance;
    if (!joints.register_motor(l_joint1, bus, cfg::l_joint1_cmd_id, cfg::l_joint1_fdb_id) ||
        !joints.register_motor(l_joint4, bus, cfg::l_joint4_cmd_id, cfg::l_joint4_fdb_id) ||
        !joints.register_motor(r_joint1, bus, cfg::r_joint1_cmd_id, cfg::r_joint1_fdb_id) ||
        !joints.register_motor(r_joint4, bus, cfg::r_joint4_cmd_id, cfg::r_joint4_fdb_id) ||
        !wheels.register_motor(l_wheel, cfg::wheel_bus.instance, cfg::l_wheel) ||
        !wheels.register_motor(r_wheel, cfg::wheel_bus.instance, cfg::r_wheel))
    {
        return false;
    }
    l_joint1.set_fd(true);
    l_joint4.set_fd(true);
    r_joint1.set_fd(true);
    r_joint4.set_fd(true);
    return bsp::can::init(cfg::joint_bus) == pnxtypes::status::ok &&
           bsp::can::init(cfg::wheel_bus) == pnxtypes::status::ok;
}

std::uint8_t update_feedback() noexcept
{
    std::uint8_t updated = 0;
    updated |= joints.update_feedback(l_joint1) ? 0x01U : 0U;
    updated |= joints.update_feedback(l_joint4) ? 0x02U : 0U;
    updated |= joints.update_feedback(r_joint1) ? 0x04U : 0U;
    updated |= joints.update_feedback(r_joint4) ? 0x08U : 0U;
    wheels.update_feedback(l_wheel);
    wheels.update_feedback(r_wheel);
    return updated;
}

bool stop_motors() noexcept
{
    const auto disable = motor::dm::special_command::disable;
    const bool l1_stopped = joints.special(l_joint1, disable);
    const bool l4_stopped = joints.special(l_joint4, disable);
    const bool r1_stopped = joints.special(r_joint1, disable);
    const bool r4_stopped = joints.special(r_joint4, disable);
    l_wheel.cmd.current = 0.0f;
    r_wheel.cmd.current = 0.0f;
    wheels.update();
    wheels.transmit(static_cast<std::uint32_t>(bsp::dwt::timeline_us()));
    const std::uint8_t zero[8]{};
    const bool wheels_stopped =
        bsp::can::transmit(cfg::wheel_bus.instance, 0x200, zero, 8) == pnxtypes::status::ok;
    return l1_stopped && l4_stopped && r1_stopped && r4_stopped && wheels_stopped;
}

bool joints_ready() noexcept
{
    return l_joint1.motor_state == motor::state::online && l_joint1.error_code() == 0x01U &&
           l_joint4.motor_state == motor::state::online && l_joint4.error_code() == 0x01U &&
           r_joint1.motor_state == motor::state::online && r_joint1.error_code() == 0x01U &&
           r_joint4.motor_state == motor::state::online && r_joint4.error_code() == 0x01U;
}

bool enable_joint(motor::dm::motor& joint, std::uint8_t bit, std::uint8_t& confirmed) noexcept
{
    constexpr ULONG timeout_ticks = TX_TIMER_TICKS_PER_SECOND;
    constexpr ULONG retry_ticks = (TX_TIMER_TICKS_PER_SECOND + 99U) / 100U;
    (void)joints.update_feedback(joint); // Discard the previous cached feedback.
    const ULONG started = tx_time_get();
    ULONG last_send = started - retry_ticks;
    bool sent = false;
    while (tuning.motor_output_enabled && tx_time_get() - started < timeout_ticks)
    {
        const ULONG now = tx_time_get();
        if (now - last_send >= retry_ticks)
        {
            last_send = now;
            sent = joints.special(joint, motor::dm::special_command::enable) || sent;
        }
        const auto updated = update_feedback();
        joints.update();
        wheels.update();
        joints.transmit(static_cast<std::uint32_t>(bsp::dwt::timeline_us()));
        const std::uint8_t zero[8]{};
        (void)bsp::can::transmit(cfg::wheel_bus.instance, 0x200, zero, 8);
        if (sent && (updated & bit) != 0U && joint.error_code() == 0x01U)
        {
            confirmed |= bit;
            return tuning.motor_output_enabled;
        }
        tx_thread_sleep(1U);
    }
    return false;
}

bool enable_joints(std::uint8_t& confirmed) noexcept
{
    confirmed = 0;
    l_joint1.motor_mode = motor::mode::mit;
    l_joint1.cmd.torque = 0.0f;
    l_joint4.motor_mode = motor::mode::mit;
    l_joint4.cmd.torque = 0.0f;
    r_joint1.motor_mode = motor::mode::mit;
    r_joint1.cmd.torque = 0.0f;
    r_joint4.motor_mode = motor::mode::mit;
    r_joint4.cmd.torque = 0.0f;

    return enable_joint(l_joint1, 0x01U, confirmed) &&
           enable_joint(l_joint4, 0x02U, confirmed) &&
           enable_joint(r_joint1, 0x04U, confirmed) &&
           enable_joint(r_joint4, 0x08U, confirmed) && joints_ready();
}

void run_control(ULONG) noexcept
{
    using pnxtypes::status;
    if (bsp::dwt::init() != status::ok || imu::start_bmi088() != status::ok || !init_motors())
    {
        start_status = TX_NOT_DONE;
        return;
    }
    auto imu_sub = imu::attitude::output.subscribe();
    if (!imu_sub.valid())
    {
        start_status = TX_NOT_DONE;
        return;
    }
    controller control{};
    chassis_state state{};
    cmd_frame frame{};
    imu::attitude::state imu{};
    bool enabled = false;
    bool stopped = false;
    std::uint8_t confirmed = 0;
    std::uint64_t last_start = bsp::dwt::timeline_us();
    for (;;)
    {
        const auto start_us = bsp::dwt::timeline_us();
        const auto period_us = static_cast<std::uint32_t>(start_us - last_start);
        last_start = start_us;
        const std::uint32_t now = HAL_GetTick();
        (void)imu_sub.read(imu);
        (void)command_sub.read(frame);
        auto command = frame.chassis_cmd;
        const bool command_fresh =
            frame.sequence != 0U && now - frame.received_tick <= cmd::mapping::offline_ticks;
        if (!command_fresh)
        {
            command = {};
        }
        (void)update_feedback();
        motor_fdb fdb{};
        fdb.l_j1 = l_joint1.fdb;
        fdb.l_j4 = l_joint4.fdb;
        fdb.r_j1 = r_joint1.fdb;
        fdb.r_j4 = r_joint4.fdb;
        fdb.l_w = l_wheel.fdb;
        fdb.r_w = r_wheel.fdb;
        fdb.motors_online = l_wheel.motor_state == motor::state::online &&
            r_wheel.motor_state == motor::state::online &&
            l_joint1.motor_state == motor::state::online && l_joint4.motor_state == motor::state::online &&
            r_joint1.motor_state == motor::state::online && r_joint4.motor_state == motor::state::online;
        const bool motors_online = fdb.motors_online;
        const bool output_requested = tuning.motor_output_enabled;
        if (enabled && (!output_requested || !joints_ready()))
        {
            enabled = false;
            stopped = false;
        }
        if (!enabled && !stopped)
        {
            confirmed = 0;
            stopped = stop_motors();
        }
        fdb.motors_online = fdb.motors_online && enabled;
        auto imu_input = imu;
        imu_input.online = imu.online && imu.stage == imu::attitude::phase::running;
        const bool imu_valid = imu_input.online && imu_input.sequence != 0U &&
                               now - imu_input.received_ms <= 100U;
        // Match the previous AHRS x-offset compensation at the application boundary.
        imu_input.accel.x += 0.2f * imu.gyro.z * imu.gyro.z;
        motor_cmd output{};
        control.step(imu_input, fdb, command, now, output, state);
        const bool send_output = enabled;
        if (send_output)
        {
            l_joint1.motor_mode = l_joint4.motor_mode = motor::mode::mit;
            r_joint1.motor_mode = r_joint4.motor_mode = motor::mode::mit;
            l_joint1.cmd.torque = output.l_tj1;
            l_joint4.cmd.torque = output.l_tj4;
            r_joint1.cmd.torque = output.r_tj1;
            r_joint4.cmd.torque = output.r_tj4;
            l_wheel.motor_mode = r_wheel.motor_mode = motor::mode::current;
            l_wheel.cmd.current = static_cast<std::int16_t>(-output.l_tw * cfg::wheel_current_scale);
            r_wheel.cmd.current = static_cast<std::int16_t>(output.r_tw * cfg::wheel_current_scale);
        }
        joints.update();
        wheels.update();
        if (send_output)
        {
            const auto now_us = static_cast<std::uint32_t>(bsp::dwt::timeline_us());
            joints.transmit(now_us);
            wheels.transmit(now_us);
        }
        (void)state_topic.publish(state);

        if (output_requested && !enabled && stopped)
        {
            enabled = enable_joints(confirmed);
            stopped = false;
        }

        dbg.runtime.period_us = period_us;
        dbg.runtime.command_fresh = command_fresh;
        dbg.runtime.motors_online = motors_online;
        dbg.runtime.joints_enabled = confirmed;
        dbg.runtime.joint_error[0] = l_joint1.error_code();
        dbg.runtime.joint_error[1] = l_joint4.error_code();
        dbg.runtime.joint_error[2] = r_joint1.error_code();
        dbg.runtime.joint_error[3] = r_joint4.error_code();
        dbg.runtime.imu_valid = imu_valid;
        const auto exec_us = static_cast<std::uint32_t>(bsp::dwt::timeline_us() - start_us);
        dbg.runtime.exec_us = exec_us;
        if (exec_us > dbg.runtime.max_exec_us)
        {
            dbg.runtime.max_exec_us = exec_us;
        }
        if (exec_us > 1000U)
        {
            ++dbg.runtime.overrun_count;
        }
        tx_thread_sleep(1U);
    }
}

void run_command(ULONG) noexcept
{
    using pnxtypes::status;
    if (ps2.init() != status::ok)
    {
        start_status = TX_NOT_DONE;
        return;
    }

    cmd::mapper mapper{};
    chassis_state state{};
    remotes::ps2::sample sample{};
    bool received = false;
    bool pending = false;
    std::uint32_t ps2_tick = 0;
    std::uint32_t start_tick = 0;
    std::uint32_t last_poll = 0;
    for (;;)
    {
        const std::uint32_t now = HAL_GetTick();
        (void)state_sub.read(state);
        if (pending)
        {
            remotes::ps2::frame raw{};
            const auto result = ps2.read(raw);
            if (result == status::ok)
            {
                pending = false;
                if (remotes::ps2::decode(raw, sample) == status::ok)
                {
                    received = true;
                    ps2_tick = now;
                }
            }
            else if (result != status::busy || now - start_tick >= 20U)
            {
                if (ps2.cancel() == status::ok)
                {
                    pending = false;
                }
            }
        }
        if (!pending && now - last_poll >= 10U)
        {
            last_poll = now;
            if (ps2.start() == status::ok)
            {
                pending = true;
                start_tick = now;
            }
        }
        const bool online = received && now - ps2_tick <= 120U;
        const auto request = cmd::map_ps2(sample, online, now);
        const auto frame = mapper.step(request, state, now);
        (void)command_topic.publish(frame);
        tx_thread_sleep(1U);
    }
}
} // namespace
} // namespace chassis

extern "C" void app_start()
{
    using pnxtypes::status;
    if (chassis::command_topic.init() != status::ok || chassis::state_topic.init() != status::ok)
    {
        chassis::start_status = TX_NOT_DONE;
        return;
    }
    chassis::command_sub = chassis::command_topic.subscribe();
    chassis::state_sub = chassis::state_topic.subscribe();
    if (!chassis::command_sub.valid() || !chassis::state_sub.valid())
    {
        chassis::start_status = TX_NOT_DONE;
        return;
    }
    chassis::start_status = tx_thread_create(&chassis::control_thread, const_cast<CHAR*>("chassis"),
        chassis::run_control, 0, chassis::control_stack, sizeof(chassis::control_stack),
        6, 6, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (chassis::start_status != TX_SUCCESS)
    {
        return;
    }
    chassis::start_status = tx_thread_create(&chassis::command_thread, const_cast<CHAR*>("chassis_cmd"),
        chassis::run_command, 0, chassis::command_stack, sizeof(chassis::command_stack),
        7, 7, TX_NO_TIME_SLICE, TX_AUTO_START);
}
