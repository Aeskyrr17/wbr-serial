#include "chassis.hpp"
#include "command.hpp"
#include "robot_debug.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
using namespace robot;
void check(bool b,const char* s){if(!b){std::fprintf(stderr,"FAIL %s\n",s);std::exit(1);}}
bool near(float a,float b){return std::fabs(a-b)<1e-4f;}
motor_feedback_snapshot pose(float phi){motor_feedback_snapshot f{};float d=1.5f;
f.l_joint1.position=phi+d-chassis::model::pi;f.l_joint4.position=phi-d;
f.r_joint1.position=-f.l_joint1.position;f.r_joint4.position=-f.l_joint4.position;f.wheels_online=true;return f;}
int main(){
 remoter::state rc{};rc.offline=false;rc.active_source=remoter::source::ps2;
 const auto l1=static_cast<unsigned>(remoter::ps2_button::l1),l2=static_cast<unsigned>(remoter::ps2_button::l2),circle=static_cast<unsigned>(remoter::ps2_button::circle);
 auto map=[&](unsigned keys){rc.ps2_buttons=keys;return command::map_ps2(rc,0);};
 map(0);check(!map(circle).spin,"circle cannot enable move");map(0);map(l1|l2);map(0);
 check(map(circle).spin,"circle enters spin request");for(int i=0;i<20;++i)check(map(circle).spin,"held circle does not retoggle");
 map(0);check(!map(circle).spin,"second press exits");map(0);auto request=map(circle);
 command::mapper mapper;robot_state state{};state.state_executed=chassis_state::flatten;state.v_cmd_enabled=true;
 check(!mapper.step(request,state,0).chassis.spin,"startup gates spin");
 state.state_executed=chassis_state::normal;state.x=2;state.yaw=8;
 auto cmd=mapper.step(request,state,0).chassis;check(cmd.spin&&cmd.v==0&&near(cmd.yaw,8),"spin reference entry");
 for(int i=0;i<600;++i){state.state_executed=chassis_state::spin;state.x+=.001f;state.yaw+=.01f;cmd=mapper.step(request,state,0).chassis;}
 check(near(cmd.dyaw,command::mapping::spin_rate)&&near(cmd.x,2)&&near(cmd.yaw,state.yaw),"constant rate, fixed position, current yaw");
 request.len_rate=.05f;float before=cmd.len;cmd=mapper.step(request,state,0).chassis;check(cmd.len>before,"manual length works in spin");
 request.spin=false;float prev=cmd.dyaw;cmd=mapper.step(request,state,0).chassis;
 check(!cmd.spin&&cmd.dyaw>0&&cmd.dyaw<prev&&cmd.v==0,"exit decelerates and suppresses translation");
 for(int i=0;i<600;++i)cmd=mapper.step(request,state,0).chassis;
 check(cmd.dyaw==0,"exit reaches zero yaw rate");state.state_executed=chassis_state::normal;cmd=mapper.step(request,state,0).chassis;
 check(near(cmd.x,state.x)&&near(cmd.yaw,state.yaw),"exit resets holds to current pose");
 rc.offline=true;check(!map(circle).spin&&!map(circle).move,"offline clears spin and move");
 chassis::controller ctrl;chassis_command c{};robot_state actual{};::imu::state imu{};
 imu.online=true;imu.sequence=1;imu.accel_z=chassis::model::gravity;imu.quaternion[0]=1;
 c.move=true;c.len=.16f;ctrl.step(imu,pose(1.1f),c,0,actual);ctrl.step(imu,pose(1.1f),c,0,actual);
 ctrl.step(imu,pose(chassis::model::pi/2),c,0,actual);
 for(int i=0;i<501;++i)ctrl.step(imu,pose(chassis::model::pi/2),c,0,actual);
 check(actual.state_executed==chassis_state::normal&&actual.v_cmd_enabled,"normal ready");
 c.spin=true;c.dyaw=1;ctrl.step(imu,pose(chassis::model::pi/2),c,0,actual);check(actual.next_state==chassis_state::spin,"normal -> spin");
 ctrl.step(imu,pose(chassis::model::pi/2),c,0,actual);
 check(robot_debug.chassis.lqr_mode==static_cast<unsigned>(lqr_mode::spin),"spin gains selected");
 check(near(robot_debug.chassis.ref[4],robot_debug.chassis.left.alpha_eq+robot_tuning.spin_alpha_trim),"spin alpha offset");
 c.spin=false;ctrl.step(imu,pose(chassis::model::pi/2),c,0,actual);check(actual.next_state==chassis_state::spin,"remain spin while slowing");
 c.dyaw=0;ctrl.step(imu,pose(chassis::model::pi/2),c,0,actual);check(actual.next_state==chassis_state::normal,"spin -> normal at zero reference");
 c.spin=true;ctrl.step(imu,pose(chassis::model::pi/2),c,0,actual);ctrl.step(imu,pose(chassis::model::pi/2),c,0,actual);
 c.move=false;auto out=ctrl.step(imu,pose(chassis::model::pi/2),c,0,actual);check(actual.state_executed==chassis_state::relax&&out.Twl==0&&out.l_joint1==0,"disable spin immediately relaxes");
 std::puts("PASS: circle toggle, startup gate, spin refs, exit ramp, holds, length, offline, SPIN gains/transitions");
}
