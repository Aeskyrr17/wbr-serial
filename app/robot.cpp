#include "robot.hpp"

#include "ahrs.hpp"
#include "bsp_dwt.hpp"
#include "chassis.hpp"
#include "chassis_model.hpp"
#include "command.hpp"
#include "motorservice.hpp"
#include "motortraits.hpp"
#include "remoter.hpp"
#include "robot_config.hpp"
#include "robot_debug.hpp"
#include "robot_topics.hpp"
#include "tx_api.h"

#include <cstdint>

namespace robot::application
{
namespace
{
TX_THREAD control_thread{};
TX_THREAD command_thread{};
alignas(8) std::uint8_t control_stack[6144]{};
alignas(8) std::uint8_t command_stack[4096]{};


using l_joint1_motor_type =
    robot::devices::motors::model_type_t<robot::motors::l_joint1_model>;
using l_joint4_motor_type =
    robot::devices::motors::model_type_t<robot::motors::l_joint4_model>;
using r_joint1_motor_type =
    robot::devices::motors::model_type_t<robot::motors::r_joint1_model>;
using r_joint4_motor_type =
    robot::devices::motors::model_type_t<robot::motors::r_joint4_model>;
using l_wheel_motor_type =
    robot::devices::motors::model_type_t<robot::motors::l_wheel_model>;
using r_wheel_motor_type =
    robot::devices::motors::model_type_t<robot::motors::r_wheel_model>;

l_joint1_motor_type l_joint1{robot::motors::l_joint1};
l_joint4_motor_type l_joint4{robot::motors::l_joint4};
r_joint1_motor_type r_joint1{robot::motors::r_joint1};
r_joint4_motor_type r_joint4{robot::motors::r_joint4};
l_wheel_motor_type l_wheel{robot::motors::l_wheel};
r_wheel_motor_type r_wheel{robot::motors::r_wheel};
::motors::motor_service motor_service{};

msg::subscriber ps2_sub{};
msg::subscriber imu_sub{};
msg::subscriber command_sub{};
msg::subscriber state_sub{};
bool started = false;

void ps2_update(const remoter::state& rc) noexcept
{
    const auto request = command::map_ps2(rc, static_cast<std::uint32_t>(tx_time_get()));
    robot_debug.command.ps2_buttons = rc.ps2_buttons;
    robot_debug.command.ps2_move = request.move;
    (void)msg::publish(topics::ps2_request, request, {false, false});
}

bool register_motors() noexcept
{
    return motor_service.register_motor(l_joint1) && motor_service.register_motor(l_joint4) &&
           motor_service.register_motor(r_joint1) && motor_service.register_motor(r_joint4) &&
           motor_service.register_motor(l_wheel) && motor_service.register_motor(r_wheel);
}

motor_feedback_snapshot copy_feedback() noexcept
{
    motor_feedback_snapshot fdb{};
    const UINT prev_interrupts = tx_interrupt_control(TX_INT_DISABLE);
    fdb.l_joint1 = l_joint1.get_feedback();
    fdb.l_joint4 = l_joint4.get_feedback();
    fdb.r_joint1 = r_joint1.get_feedback();
    fdb.r_joint4 = r_joint4.get_feedback();
    fdb.l_wheel = l_wheel.get_feedback();
    fdb.r_wheel = r_wheel.get_feedback();
    fdb.wheels_online = l_wheel.status() == ::motors::state::online &&
                        r_wheel.status() == ::motors::state::online;
    tx_interrupt_control(prev_interrupts);
    return fdb;
}

void control_entry(ULONG) noexcept
{
    chassis::controller chassis_control{};
    ::imu::state imu{};
    command_frame command{};
    robot_state state{};
    std::uint32_t loop_count = 0;
    std::uint64_t last_start = bsp::dwt::timeline_us();
    bool motors_started = false;
    auto& motor_debug = robot_debug.motors;
    motor_debug.l_joint1 = &l_joint1;
    motor_debug.l_joint4 = &l_joint4;
    motor_debug.r_joint1 = &r_joint1;
    motor_debug.r_joint4 = &r_joint4;
    motor_debug.l_wheel = &l_wheel;
    motor_debug.r_wheel = &r_wheel;
    motor_debug.registered = register_motors();
    if (!motor_debug.registered) for (;;) tx_thread_sleep(100U);

    for (;;)
    {
        const auto start_us = bsp::dwt::timeline_us();
        robot_debug.runtime.period_us = static_cast<std::uint32_t>(start_us - last_start);
        last_start = start_us;
        ::imu::state next_imu{};
        if (msg::read(imu_sub, next_imu) == types::status::ok) imu = next_imu;
        command_frame next_command{};
        if (msg::read(command_sub, next_command) == types::status::ok) command = next_command;

        if (++loop_count % 3U == 0U) motor_service.alive_check();
        const auto fdb = copy_feedback();
        const auto now = static_cast<std::uint32_t>(tx_time_get());
        const auto output = chassis_control.step(imu, fdb, command.chassis, now, state);

        // This migration build starts with output disabled. No enable/current/MIT
        // frame is sent until a hardware-validation session explicitly opts in.
        if (robot_tuning.motor_output_enabled)
        {
            if (!motors_started)
            {
                motor_debug.starting = true;
                motor_debug.start_status = motor_service.start();
                motors_started = motor_debug.start_status == types::status::ok;
                motor_debug.starting = false;
            }
            if (motors_started)
            {
                // RELAX is enabled zero-MIT, not the DM protocol disable frame.
                l_joint1.set_mit(0.0f, 0.0f, 0.0f, 0.0f, output.l_joint1);
                l_joint4.set_mit(0.0f, 0.0f, 0.0f, 0.0f, output.l_joint4);
                r_joint1.set_mit(0.0f, 0.0f, 0.0f, 0.0f, output.r_joint1);
                r_joint4.set_mit(0.0f, 0.0f, 0.0f, 0.0f, output.r_joint4);
                l_wheel.set_current(static_cast<std::int16_t>(-output.Twl * chassis::model::wheel_current_scale));
                r_wheel.set_current(static_cast<std::int16_t>(output.Twr * chassis::model::wheel_current_scale));
                motor_service.send_control();
            }
        }
        else if (motors_started)
        {
            l_joint1.set_mit(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            l_joint4.set_mit(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            r_joint1.set_mit(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            r_joint4.set_mit(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            l_wheel.set_current(0); r_wheel.set_current(0);
            motor_service.send_control();
            motor_service.stop();
            motors_started = false;
        }
        (void)msg::publish(topics::state, state);

        const auto exec = static_cast<std::uint32_t>(bsp::dwt::timeline_us() - start_us);
        robot_debug.runtime.exec_us = exec;
        if (exec > robot_debug.runtime.max_exec_us) robot_debug.runtime.max_exec_us = exec;
        if (exec > 1000U) ++robot_debug.runtime.overrun_count;
        tx_thread_sleep(1U);
    }
}

void command_entry(ULONG) noexcept
{
    command::mapper mapper{};
    command::motion_request ps2_request{};
    ps2_request.source = input_source::ps2;
    robot_state state{};
    for (;;)
    {
        command::motion_request next_ps2{};
        if (msg::read(ps2_sub, next_ps2) == types::status::ok) ps2_request = next_ps2;
        robot_state next_state{};
        if (msg::read(state_sub, next_state) == types::status::ok) state = next_state;
        const auto now = static_cast<std::uint32_t>(tx_time_get());
        const auto frame = mapper.step(ps2_request, state, now);
        (void)msg::publish(topics::command, frame);
        tx_thread_sleep(1U);
    }
}

}

types::status start() noexcept
{
    if (started) return types::status::ok;
    if (!bsp::dwt::initialized()) (void)bsp::dwt::init();
    if (!topics::init()) return types::status::error;
    if (ahrs::service::instance().init() != types::status::ok) return types::status::error;
    ps2_sub = msg::subscribe(topics::ps2_request);
    if constexpr (::config::feature::enable_ps2)
    {
        remoter::config rc_cfg{};
        rc_cfg.on_update_callback = remoter::update_callback::bind<&ps2_update>();
        if (remoter::service::instance().init(rc_cfg) != types::status::ok)
            return types::status::error;
    }
    imu_sub = msg::subscribe(ahrs::service::instance().output());
    command_sub = msg::subscribe(topics::command);
    state_sub = msg::subscribe(topics::state);
    if (!ps2_sub.valid() || !imu_sub.valid() || !command_sub.valid() || !state_sub.valid())
        return types::status::error;
    if (tx_thread_create(&control_thread, const_cast<CHAR*>("robot_control"), control_entry, 0U,
                         control_stack, sizeof(control_stack), 6U, 6U, TX_NO_TIME_SLICE,
                         TX_AUTO_START) != TX_SUCCESS)
        return types::status::error;
    if (tx_thread_create(&command_thread, const_cast<CHAR*>("robot_command"), command_entry, 0U,
                         command_stack, sizeof(command_stack), 7U, 7U, TX_NO_TIME_SLICE,
                         TX_AUTO_START) != TX_SUCCESS)
        return types::status::error;
    started = true;
    return types::status::ok;
}

} // namespace robot::application
