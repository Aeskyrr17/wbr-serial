#include "robot.hpp"

#include "ahrs.hpp"
#include "aux_control.hpp"
#include "bsp_dwt.hpp"
#include "chassis.hpp"
#include "chassis_model.hpp"
#include "command.hpp"
#include "motorservice.hpp"
#include "motortraits.hpp"
#include "referee.hpp"
#include "robot_config.hpp"
#include "robot_debug.hpp"
#include "robot_io.hpp"
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
using trigger_motor_type =
    robot::devices::motors::model_type_t<robot::motors::trigger_model>;
using yaw_motor_type =
    robot::devices::motors::model_type_t<robot::motors::yaw_model>;

l_joint1_motor_type l_joint1{robot::motors::l_joint1};
l_joint4_motor_type l_joint4{robot::motors::l_joint4};
r_joint1_motor_type r_joint1{robot::motors::r_joint1};
r_joint4_motor_type r_joint4{robot::motors::r_joint4};
l_wheel_motor_type l_wheel{robot::motors::l_wheel};
r_wheel_motor_type r_wheel{robot::motors::r_wheel};
trigger_motor_type trigger{robot::motors::trigger};
yaw_motor_type yaw{robot::motors::yaw};
::motors::motor_service motor_service{};

msg::subscriber imu_sub{};
msg::subscriber command_sub{};
msg::subscriber state_sub{};
msg::subscriber command_referee_sub{};
command::referee_summary referee_data{};
bool started = false;

void referee_update(const referee::packet_store& packets,
    const referee::update_info& update) noexcept
{
    if (!update.got_valid_frame || !update.is_known_packet) return;
    switch (update.command_id)
    {
    case RefereeID::GameRobotStatus:
        referee_data.robot_id = packets.game_robot_status.robot_id;
        referee_data.robot_level = packets.game_robot_status.robot_level;
        referee_data.heat_limit = packets.game_robot_status.shooter_barrel_heat_limit;
        referee_data.chassis_power_limit = packets.game_robot_status.chassis_power_limit;
        referee_data.chassis_power_enabled =
            packets.game_robot_status.power_management_chassis_output != 0U;
        break;
    case RefereeID::PowerHeatData:
        referee_data.power_buffer = packets.power_heat_data.chassis_power_buffer;
        referee_data.heat_now = packets.power_heat_data.shoot_id1_17mm_cooling_heat;
        break;
    case RefereeID::Buff:
        referee_data.remained_energy = packets.buff.remained_energy;
        break;
    case RefereeID::RobotHurt:
        referee_data.hurt_armor_id = packets.robot_hurt.armor_id;
        referee_data.hurt_reason = packets.robot_hurt.HP_deduction_reason;
        break;
    default:
        break;
    }
    (void)msg::publish(topics::referee, referee_data, {false, false});
}

bool register_motors() noexcept
{
    return motor_service.register_motor(l_joint1) && motor_service.register_motor(l_joint4) &&
           motor_service.register_motor(r_joint1) && motor_service.register_motor(r_joint4) &&
           motor_service.register_motor(l_wheel) && motor_service.register_motor(r_wheel) &&
           motor_service.register_motor(trigger) && motor_service.register_motor(yaw);
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
    fdb.trigger = trigger.get_feedback();
    fdb.yaw = yaw.get_feedback();
    fdb.wheels_online = l_wheel.status() == ::motors::state::online &&
                        r_wheel.status() == ::motors::state::online;
    tx_interrupt_control(prev_interrupts);
    return fdb;
}

void control_entry(ULONG) noexcept
{
    chassis::controller chassis_control{};
    aux::controller aux_control{};
    ::imu::state imu{};
    command_frame command{};
    robot_state state{};
    std::uint32_t loop_count = 0;
    std::uint64_t last_start = bsp::dwt::timeline_us();
    bool motors_started = false;
    if (!register_motors()) for (;;) tx_thread_sleep(100U);

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
        const auto aux_out = aux_control.step(command.aux, state, fdb.yaw, fdb.trigger);
        state.yaw_fdb = fdb.yaw.position;
        state.yaw_ref = command.chassis.yaw;
        state.yaw_ready = aux_out.yaw_ready;

        // This migration build starts with output disabled. No enable/current/MIT
        // frame is sent until a hardware-validation session explicitly opts in.
        if (robot_tuning.motor_output_enabled)
        {
            if (!motors_started)
                motors_started = motor_service.start() == types::status::ok;
            if (motors_started)
            {
                // RELAX is enabled zero-MIT, not the DM protocol disable frame.
                l_joint1.set_mit(0.0f, 0.0f, 0.0f, 0.0f, output.l_joint1);
                l_joint4.set_mit(0.0f, 0.0f, 0.0f, 0.0f, output.l_joint4);
                r_joint1.set_mit(0.0f, 0.0f, 0.0f, 0.0f, output.r_joint1);
                r_joint4.set_mit(0.0f, 0.0f, 0.0f, 0.0f, output.r_joint4);
                l_wheel.set_current(static_cast<std::int16_t>(-output.Twl * chassis::model::wheel_current_scale));
                r_wheel.set_current(static_cast<std::int16_t>(output.Twr * chassis::model::wheel_current_scale));
                yaw.set_current(aux_out.yaw_current);
                trigger.set_current(aux_out.trigger_current);
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
            yaw.set_current(0); trigger.set_current(0);
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
    io::gimbal_command raw{};
    io::tof_sample tof{};
    robot_state state{};
    command::referee_summary referee{};
    if (io::init() != types::status::ok) for (;;) tx_thread_sleep(100U);
    for (;;)
    {
        io::gimbal_command next_raw{};
        if (io::read_command(next_raw)) raw = next_raw;
        io::tof_sample next_tof{};
        if (io::read_tof(next_tof)) tof = next_tof;
        robot_state next_state{};
        if (msg::read(state_sub, next_state) == types::status::ok) state = next_state;
        command::referee_summary next_referee{};
        if (msg::read(command_referee_sub, next_referee) == types::status::ok)
            referee = next_referee;
        const auto frame = mapper.step(raw, tof, referee, state,
                                       static_cast<std::uint32_t>(tx_time_get()));
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
    referee::config referee_cfg{};
    referee_cfg.on_update_callback = referee::update_callback::bind<&referee_update>();
    if (referee::service::instance().init(referee_cfg) != types::status::ok)
        return types::status::error;
    imu_sub = msg::subscribe(ahrs::service::instance().output());
    command_sub = msg::subscribe(topics::command);
    state_sub = msg::subscribe(topics::state);
    command_referee_sub = msg::subscribe(topics::referee);
    if (!imu_sub.valid() || !command_sub.valid() || !state_sub.valid() ||
        !command_referee_sub.valid())
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
