#include "attitude.hpp"
#include "bmi088.hpp"
#include "bsp_can.hpp"
#include "bsp_dwt.hpp"
#include "bsp_spi.hpp"
#include "chassis.hpp"
#include "command/cmd.hpp"
#include "dm_manager.hpp"
#include "dji_manager.hpp"
#include "peripherals.hpp"
#include "ps2.hpp"
#include "robot_debug.hpp"
#include "stm32h7xx_hal.h"
#include "tx_api.h"

#include <cstdint>

namespace chassis
{
volatile UINT start_status = TX_SUCCESS;
namespace
{
TX_THREAD thread{};
alignas(8) std::uint8_t stack[8192]{};
motor::dm::config_t joint_params{62.8319f, 45.0f, 54.0f, 1.0f, false};
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
    if (!joints.register_motor(l_joint1, bus, cfg::l_joint1.control_id, cfg::l_joint1.feedback_id) ||
        !joints.register_motor(l_joint4, bus, cfg::l_joint4.control_id, cfg::l_joint4.feedback_id) ||
        !joints.register_motor(r_joint1, bus, cfg::r_joint1.control_id, cfg::r_joint1.feedback_id) ||
        !joints.register_motor(r_joint4, bus, cfg::r_joint4.control_id, cfg::r_joint4.feedback_id) ||
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

void feedback(motor_fdb& fdb) noexcept
{
    joints.update_feedback(l_joint1);
    joints.update_feedback(l_joint4);
    joints.update_feedback(r_joint1);
    joints.update_feedback(r_joint4);
    wheels.update_feedback(l_wheel);
    wheels.update_feedback(r_wheel);
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
}

void run(ULONG) noexcept
{
    using pnxtypes::status;
    if (bsp::dwt::init() != status::ok || imu::start_bmi088() != status::ok ||
        !init_motors() || ps2.init() != status::ok)
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
    cmd::mapper mapper{};
    chassis_state state{};
    imu::attitude::state imu{};
    remotes::ps2::sample sample{};
    bool received = false;
    bool pending = false;
    bool enabled = false;
    std::uint32_t ps2_tick = 0;
    std::uint32_t start_tick = 0;
    std::uint32_t last_poll = 0;
    for (;;)
    {
        const std::uint32_t now = HAL_GetTick();
        (void)imu_sub.read(imu);
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
        motor_fdb fdb{};
        feedback(fdb);
        auto imu_input = imu;
        imu_input.online = imu.online && imu.stage == imu::attitude::phase::running;
        // Match the previous AHRS x-offset compensation at the application boundary.
        imu_input.accel.x += 0.2f * imu.gyro.z * imu.gyro.z;
        const auto output = control.step(imu_input, fdb, frame.chassis_cmd, now, state);
        if (tuning.motor_output_enabled && !enabled)
        {
            const auto enable = motor::dm::special_command::enable;
            const bool l1_ok = joints.special(l_joint1, enable);
            const bool l4_ok = joints.special(l_joint4, enable);
            const bool r1_ok = joints.special(r_joint1, enable);
            const bool r4_ok = joints.special(r_joint4, enable);
            enabled = l1_ok && l4_ok && r1_ok && r4_ok;
            if (!enabled)
            {
                const auto disable = motor::dm::special_command::disable;
                (void)joints.special(l_joint1, disable);
                (void)joints.special(l_joint4, disable);
                (void)joints.special(r_joint1, disable);
                (void)joints.special(r_joint4, disable);
                tuning.motor_output_enabled = false;
            }
        }
        if (enabled && !tuning.motor_output_enabled)
        {
            const auto disable = motor::dm::special_command::disable;
            const bool l1_ok = joints.special(l_joint1, disable);
            const bool l4_ok = joints.special(l_joint4, disable);
            const bool r1_ok = joints.special(r_joint1, disable);
            const bool r4_ok = joints.special(r_joint4, disable);
            const std::uint8_t zero[8]{};
            const bool wheels_ok = bsp::can::transmit(cfg::wheel_bus.instance, 0x200, zero, 8) == status::ok;
            enabled = !(l1_ok && l4_ok && r1_ok && r4_ok && wheels_ok);
        }
        if (enabled && tuning.motor_output_enabled)
        {
            l_joint1.motor_mode = l_joint4.motor_mode = r_joint1.motor_mode = r_joint4.motor_mode = motor::mode::mit;
            l_joint1.cmd.torque = output.l_joint1;
            l_joint4.cmd.torque = output.l_joint4;
            r_joint1.cmd.torque = output.r_joint1;
            r_joint4.cmd.torque = output.r_joint4;
            l_wheel.motor_mode = r_wheel.motor_mode = motor::mode::current;
            l_wheel.cmd.current = static_cast<std::int16_t>(-output.Twl * model::wheel_current_scale);
            r_wheel.cmd.current = static_cast<std::int16_t>(output.Twr * model::wheel_current_scale);
            joints.update();
            wheels.update();
            const auto now_us = static_cast<std::uint32_t>(bsp::dwt::timeline_us());
            joints.transmit(now_us);
            wheels.transmit(now_us);
        }
        else
        {
            joints.update();
            wheels.update();
        }
        tx_thread_sleep(1U);
    }
}
} // namespace
} // namespace chassis

extern "C" void app_start()
{
    chassis::start_status = tx_thread_create(&chassis::thread, const_cast<CHAR*>("chassis"), chassis::run, 0,
        chassis::stack, sizeof(chassis::stack), 6, 6, TX_NO_TIME_SLICE, TX_AUTO_START);
}
