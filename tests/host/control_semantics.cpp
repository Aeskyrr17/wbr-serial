#include "chassis.hpp"
#include "chassis_model.hpp"
#include "command.hpp"
#include "robot_debug.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

// Only the external CAN send is substituted. Control, PID, VMC, odometry,
// generated coefficients and public Device payload headers are production code.
namespace robot::io
{
types::status send_chassis_status(bool, bool, std::uint8_t, std::uint16_t,
                                  std::uint16_t, std::int16_t) noexcept
{
    return types::status::ok;
}
}

namespace
{
using namespace robot;
constexpr float pi = chassis::model::pi;

void check(bool condition, const char* message)
{
    if (!condition)
    {
        std::fprintf(stderr, "FAIL: %s\n", message);
        std::exit(1);
    }
}

bool near(float a, float b, float tolerance = 1.0e-4f)
{
    return std::fabs(a - b) < tolerance;
}

motor_feedback_snapshot leg_pose(float phi = 1.575f)
{
    motor_feedback_snapshot fdb{};
    const float rotation = phi - 1.575f;
    fdb.l_joint1.position = 2.6f + rotation - pi;
    fdb.l_joint4.position = 0.55f + rotation;
    fdb.r_joint1.position = -fdb.l_joint1.position;
    fdb.r_joint4.position = -fdb.l_joint4.position;
    fdb.wheels_online = true;
    return fdb;
}

::imu::state sample_imu()
{
    ::imu::state imu{};
    imu.online = true;
    imu.sequence = 1U;
    imu.accel_z = chassis::model::gravity;
    return imu;
}

void check_phi_direction(float pitch, float direction)
{
    robot_tuning = robot_tuning_t{};
    chassis::controller ctrl;
    robot_state state{};
    chassis_command cmd{};
    cmd.move = true;
    auto imu = sample_imu();
    imu.accel_z = -chassis::model::gravity;
    imu.pitch = pitch;
    const auto fdb = leg_pose();
    ctrl.step(imu, fdb, cmd, 0U, state); // RELAX -> RECOVER
    check(state.next_state == chassis_state::recover, "enter RECOVER");
    ctrl.step(imu, fdb, cmd, 0U, state);
    float prev_ref = robot_debug.chassis.l_phi_ref;
    for (int i = 0; i < 8; ++i)
    {
        ctrl.step(imu, fdb, cmd, 0U, state);
        const float ref = robot_debug.chassis.l_phi_ref;
        check(near(ref - prev_ref, direction * 0.005f),
              "phi ramp must retain selected direction across cycles");
        check(near(ref, robot_debug.chassis.r_phi_ref), "mirrored phi references");
        prev_ref = ref;
    }

    robot_tuning.phi_recover.kp = 0.0f;
    robot_tuning.phi_recover.kd = 0.0f;
    const auto zero = ctrl.step(imu, fdb, cmd, 0U, state);
    check(near(zero.Tpl, 0.0f), "RAM tuning affects next phi cycle");
    robot_tuning.phi_recover.kp = 20.0f;
    const auto tuned = ctrl.step(imu, fdb, cmd, 0U, state);
    check(std::fabs(tuned.Tpl) > 0.1f, "RAM tuning is not overwritten by defaults");
}

void check_leg_semantics()
{
    auto fdb = leg_pose();
    fdb.l_joint1.velocity = 9.0f;
    fdb.l_joint4.velocity = -4.5f;
    fdb.r_joint1.velocity = -9.0f;
    fdb.r_joint4.velocity = 4.5f;
    fdb.l_joint1.torque = 2.0f;
    fdb.l_joint4.torque = -1.0f;
    fdb.r_joint1.torque = -2.0f;
    fdb.r_joint4.torque = 1.0f;
    auto imu = sample_imu();
    imu.pitch = 0.2f;
    imu.gyro_p = 0.3f;
    chassis::pendulum left{false}, right{true};
    check(left.update(fdb.l_joint1, fdb.l_joint4, imu, imu.accel_z), "left geometry");
    check(right.update(fdb.r_joint1, fdb.r_joint4, imu, imu.accel_z), "right geometry");
    const auto& l = left.state();
    const auto& r = right.state();
    check(near(l.len, r.len) && near(l.phi, r.phi), "mirrored joint positions");
    check(near(l.dlen, r.dlen) && near(l.dphi, r.dphi), "mirrored joint velocities");
    check(near(l.force, r.force) && near(l.pitch_torque, r.pitch_torque),
          "mirrored joint torque feedback");
    chassis::vmc geometry;
    check(geometry.resolve(2.6f, 0.55f), "reference geometry");
    const auto velocity = geometry.velocity(1.0f, -0.5f);
    check(near(l.dlen, velocity[0]) && near(l.dphi, velocity[1]),
          "legacy 1/9 velocity scale occurs exactly once");
    check(near(l.alpha, l.phi - pi / 2.0f + imu.pitch), "alpha is world leg tilt");
    check(near(l.dalpha, l.dphi + imu.gyro_p), "dalpha includes body pitch rate");
    const auto lt = left.joint_torque(50.0f, 3.0f);
    const auto rt = right.joint_torque(50.0f, 3.0f);
    check(near(lt[0], -rt[0]) && near(lt[1], -rt[1]), "mirrored VMC outputs");
}

void check_state_semantics()
{
    robot_tuning = robot_tuning_t{};
    chassis::controller ctrl;
    robot_state state{};
    chassis_command cmd{};
    cmd.move = true;
    auto imu = sample_imu();
    imu.total_yaw = 8.0f;
    imu.yaw = 8.0f - 2.0f * pi;
    auto fdb = leg_pose(1.1f);
    ctrl.step(imu, fdb, cmd, 0U, state);
    ctrl.step(imu, fdb, cmd, 0U, state);
    check(state.state_executed == chassis_state::flatten && state.v_cmd_enabled,
          "legacy normal flag enables velocity in FLATTEN");
    check(state.next_state == chassis_state::neutral, "enter NEUTRAL");
    fdb = leg_pose();
    ++cmd.events.jump;
    ctrl.step(imu, fdb, cmd, 0U, state);
    check(state.next_state == chassis_state::normal, "enter NORMAL");
    ctrl.step(imu, fdb, cmd, 0U, state);
    check(state.next_state == chassis_state::normal, "NEUTRAL jump event is discarded");
    check(!state.v_cmd_enabled, "NORMAL startup still gates velocity");
    check(near(robot_debug.chassis.obs[2], imu.total_yaw), "LQR uses continuous yaw");
    ++cmd.events.jump;
    ctrl.step(imu, fdb, cmd, 0U, state);
    check(state.next_state == chassis_state::jump, "NORMAL jump event is consumed");

    fdb.l_joint1.position = -pi;
    fdb.l_joint4.position = 0.0f; // singular left pose
    const auto rotated = leg_pose(1.3f);
    fdb.r_joint1 = rotated.r_joint1;
    fdb.r_joint4 = rotated.r_joint4;
    ctrl.step(imu, fdb, cmd, 0U, state);
    check(state.state_executed == chassis_state::relax, "invalid geometry relaxes");
    check(near(robot_debug.chassis.right.phi, 1.3f), "right sample updates if left fails");
}

void check_jump_request_semantics()
{
    command::mapper mapper;
    io::gimbal_command raw{};
    raw.move = true;
    raw.sequence = 1U;
    io::tof_sample tof{};
    command::referee_summary referee{};
    robot_state state{};
    state.recovered = state.yaw_ready = state.v_cmd_enabled = true;
    state.state_executed = chassis_state::neutral;
    raw.wants.jump = 1U;
    mapper.step(raw, tof, referee, state, 0U);
    check(!robot_debug.command.jump_armed, "NEUTRAL want cannot arm a future jump");
    state.state_executed = chassis_state::normal;
    tof.valid = true;
    tof.distance_cm = 20.0f;
    auto frame = mapper.step(raw, tof, referee, state, 0U);
    check(frame.chassis.events.jump == 0U, "discarded want is not replayed");
    tof.valid = false;
    ++raw.wants.jump;
    mapper.step(raw, tof, referee, state, 0U);
    check(robot_debug.command.jump_armed, "new want arms TOF waiting");
    mapper.step(raw, tof, referee, state, 0U);
    check(robot_debug.command.jump_armed, "same counter is not a repeated request");
    tof.valid = true;
    frame = mapper.step(raw, tof, referee, state, 0U);
    check(frame.chassis.events.jump == 1U && !robot_debug.command.jump_armed,
          "TOF produces the actual jump event");
    ++raw.wants.jump; // consecutive wants, without sampling an intervening zero bit
    frame = mapper.step(raw, tof, referee, state, 0U);
    check(frame.chassis.events.jump == 2U, "no second edge detector loses consecutive wants");
}
}

int main()
{
    check_phi_direction(0.5f, 1.0f);
    check_phi_direction(-0.5f, -1.0f);
    check_leg_semantics();
    check_state_semantics();
    check_jump_request_semantics();
    std::puts("PASS: phi direction/tuning, leg units/mirroring, state gates, jump requests");
}
