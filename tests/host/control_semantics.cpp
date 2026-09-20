#include "chassis.hpp"
#include "chassis_model.hpp"
#include "command.hpp"
#include "robot_debug.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace
{
using namespace robot;
constexpr float pi = chassis::model::pi;

void check(bool condition, const char* message)
{
    if (!condition) { std::fprintf(stderr, "FAIL: %s\n", message); std::exit(1); }
}
bool near(float a, float b, float tolerance = 1.0e-4f)
{
    return std::fabs(a - b) < tolerance;
}
motor_feedback_snapshot leg_pose(float phi = 1.575f, float spread = 1.025f)
{
    motor_feedback_snapshot fdb{};
    fdb.l_joint1.position = phi + spread - pi;
    fdb.l_joint4.position = phi - spread;
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
    imu.total_yaw = 8.0f;
    imu.yaw = 8.0f - 2.0f * pi;
    return imu;
}
void enter_normal(chassis::controller& ctrl, robot_state& state, chassis_command& cmd)
{
    cmd.move = true;
    const auto imu = sample_imu();
    auto fdb = leg_pose(1.1f);
    ctrl.step(imu, fdb, cmd, 0, state);
    check(state.next_state == chassis_state::flatten, "RELAX -> FLATTEN");
    ctrl.step(imu, fdb, cmd, 0, state);
    check(state.next_state == chassis_state::neutral, "FLATTEN -> NEUTRAL");
    check(state.v_cmd_enabled, "FLATTEN retains wheel-assist enable");
    fdb = leg_pose();
    ctrl.step(imu, fdb, cmd, 0, state);
    check(state.next_state == chassis_state::normal, "NEUTRAL -> NORMAL");
    ctrl.step(imu, fdb, cmd, 0, state);
    check(!state.v_cmd_enabled, "NORMAL settle gates velocity");
    check(near(robot_debug.chassis.obs[2], imu.total_yaw), "LQR uses continuous yaw");
}

void pid_tuning_test()
{
    control::pid pid{0.0f, 1.0f, 2.0f, 100.0f, 100.0f};
    pid.ref = 1.0f;
    pid.update();
    check(near(pid.result, 3.0f), "PID initial integral and derivative");
    control::pid_params params{4.0f, 2.0f, 2.0f, 50.0f, 40.0f};
    pid.tune(params);
    pid.update();
    check(near(pid.result, 7.0f), "parameter tuning preserves integral and error history");
    check(near(pid.max_out, 50.0f) && near(pid.max_iout, 40.0f), "parameter tuning updates limits");
    params.kp = 6.0f;
    pid.tune(params);
    pid.update();
    check(near(pid.result, 11.0f), "live parameter change applies without resetting PID");
    check(pid.mode == control::pid_mode::position, "parameter tuning preserves PID mode");
}

void mapping_test()
{
    io::command raw{};
    raw.move = true;
    raw.sequence = 1;
    raw.vx = 1;
    raw.vy = -2;
    auto request = command::map_can(raw, 0);
    check(near(request.v, 0.2f) && near(request.dyaw, -0.2f), "CAN vx/vy are simultaneous body commands");
    command::mapper mapper;
    robot_state state{};
    state.v_cmd_enabled = true;
    state.yaw = 8;
    state.x = 2;
    auto frame = mapper.step(request, state, 0);
    check(frame.chassis.move, "body drive accepts motion");
    check(frame.chassis.v > 0 && frame.chassis.dyaw < 0, "both axes reach command");
    check(near(frame.chassis.yaw, state.yaw + frame.chassis.dyaw * 0.001f), "yaw ref uses body IMU");
    request.v = request.dyaw = 0;
    for (int i=0; i<100; ++i) frame=mapper.step(request,state,0);
    const auto held=frame.chassis;
    state.yaw += 0.2f;
    state.x += 0.1f;
    frame=mapper.step(request,state,0);
    check(near(frame.chassis.yaw,held.yaw) && near(frame.chassis.x,held.x), "release holds x/yaw");
    request.source=input_source::ps2;
    frame=mapper.step(request,state,0);
    check(!frame.chassis.move && near(frame.chassis.v,0), "source switch clears motion for one cycle");
    frame=mapper.step(request,state,101);
    check(!frame.chassis.move, "stale input stops motion");

    remoter::state rc{};
    rc.offline=false;
    rc.active_source=remoter::source::ps2;
    const auto key=[](remoter::ps2_button b){return static_cast<std::uint16_t>(b);};
    rc.ps2_buttons=key(remoter::ps2_button::l2);
    rc.left_y=0.6f;
    rc.right_x=-0.4f;
    request=command::map_ps2(rc,0);
    check(request.move && near(request.v,0.3f) && near(request.dyaw,0.6f), "PS2 body mapping");
    rc.active_source=remoter::source::ps2_uart;
    rc.ps2_uart_buttons=key(remoter::ps2_button::l2);
    request=command::map_ps2(rc,0);
    check(!request.move && !request.online, "native PS2 map rejects UART receiver");
    rc.active_source=remoter::source::ps2;
    rc.ps2_buttons=0;
    check(!command::map_ps2(rc,0).move, "L2 release disables body motion");
    rc.offline=true;
    request=command::map_ps2(rc,0);
    check(!request.move && !request.online, "PS2 offline stops");
}

motor_feedback_snapshot moving_leg(float dlen)
{
    auto fdb=leg_pose();
    chassis::vmc vmc;
    check(vmc.resolve(2.6f,0.55f),"geometry for flight stimulus");
    const auto torque=vmc.joint_torque(-1000.0f,0);
    fdb.l_joint1.torque=torque[0]; fdb.l_joint4.torque=torque[1];
    fdb.r_joint1.torque=-torque[0]; fdb.r_joint4.torque=-torque[1];
    fdb.l_joint1.velocity=9.0f*dlen/vmc.state().J[0];
    fdb.r_joint1.velocity=-fdb.l_joint1.velocity;
    return fdb;
}

void leg_test()
{
    auto fdb=moving_leg(0.1f);
    auto imu=sample_imu();
    imu.pitch=0.2f; imu.gyro_p=0.3f;
    chassis::pendulum l{false},r{true};
    check(l.update(fdb.l_joint1,fdb.l_joint4,imu,imu.accel_z),"left leg solves");
    check(r.update(fdb.r_joint1,fdb.r_joint4,imu,imu.accel_z),"right leg solves");
    check(near(l.state().dlen,0.1f) && near(r.state().dlen,0.1f),"DM protocol joint velocity 1/9 exactly once and mirrored");
    check(near(l.state().leg_force,r.state().leg_force),"mirrored force feedback");
    check(near(l.state().alpha,l.state().phi-pi/2+imu.pitch),"world leg angle");
    check(near(l.state().dalpha,l.state().dphi+imu.gyro_p),"body pitch rate included");
}
}

int main()
{
    pid_tuning_test();
    mapping_test();
    leg_test();
    chassis::controller ctrl;
    robot_state state{};
    chassis_command cmd{};
    enter_normal(ctrl, state, cmd);
    const auto imu = sample_imu();
    for (int i = 0; i < 501; ++i) ctrl.step(imu, leg_pose(), cmd, 0, state);
    check(state.next_state == chassis_state::normal && state.v_cmd_enabled, "normal settles and accepts motion");
    cmd.move = false;
    const auto out = ctrl.step(imu, leg_pose(), cmd, 0, state);
    check(state.state_executed == chassis_state::relax && !state.v_cmd_enabled, "disable returns to RELAX immediately");
    check(near(out.Fl, 0) && near(out.Twl, 0) && near(out.l_joint1, 0), "disable clears actuator outputs");
    cmd.move = true;
    ctrl.step(imu, leg_pose(1.1f), cmd, 101, state);
    check(state.next_state == chassis_state::relax, "stale IMU prevents restart");
    std::puts("PASS: body maps, normal states, disable/restart, leg semantics");
}
