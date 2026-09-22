#include "command.hpp"
#include "robot_debug.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
using namespace robot;
void check(bool ok,const char* msg){if(!ok){std::fprintf(stderr,"FAIL: %s\n",msg);std::exit(1);}}
bool near(float a,float b){return std::fabs(a-b)<1e-5f;}
int main(){
 for(float dir:{1.0f,-1.0f}){
 command::mapper mapper;command::motion_request req{};robot_state state{};
 req.online=req.move=true;req.v=dir*.3f;state.v_cmd_enabled=true;state.state_executed=chassis_state::normal;
 unsigned tick=0;
 auto step=[&](){req.received_tick=tick;return mapper.step(req,state,tick++).chassis;};
 chassis_command cmd;
 for(int i=0;i<60;++i)cmd=step();
 check(near(cmd.v,req.v),"velocity ramp reaches command");
 check(near(cmd.x,state.x+cmd.v*command::mapping::dt),"moving target unchanged");
 req.v=0;state.v=dir*.2f;
 for(int i=0;i<60;++i){state.x+=dir*.0002f;cmd=step();}
 check(robot_debug.command.stopping_x&&!robot_debug.command.holding_x,"coasting is not locked");
 state.x+=dir*.01f;cmd=step();float furthest=cmd.x;
 state.x-=dir*.004f;state.v=-dir*.02f;cmd=step();
 check(near(cmd.x,furthest),"target does not follow rebound");
 for(unsigned i=0;i<command::mapping::stop_confirm_ticks-2;++i)cmd=step();
 check(robot_debug.command.stopping_x,"must confirm consecutive low-speed cycles");
 state.v=dir*.04f;cmd=step();check(robot_debug.command.stop_confirm_count==0,"speed excursion resets confirmation");
 state.v=0;
 for(unsigned i=0;i<command::mapping::stop_confirm_ticks;++i)cmd=step();
 check(robot_debug.command.holding_x&&!robot_debug.command.stopping_x,"standstill locks position");
 state.x+=dir*.02f;cmd=step();check(near(cmd.x,furthest),"locked position survives displacement");
 req.v=-dir*.2f;cmd=step();check(!robot_debug.command.holding_x&&!robot_debug.command.stopping_x,"new input releases lock");
 req.online=false;state.x=2;cmd=step();check(!cmd.move&&cmd.v==0&&near(cmd.x,2),"offline resets target immediately");
 req.online=true;req.v=0;state.x=3;state.v_cmd_enabled=false;cmd=step();check(near(cmd.x,3),"startup does not keep stale target");
 state.x=4;cmd=step();check(near(cmd.x,4),"startup follows odometry reset");
 state.v_cmd_enabled=true;cmd=step();check(near(cmd.x,4),"restart holds current position");
 }
 std::puts("PASS: forward/reverse stop following, rebound, confirmation reset, hold, re-command, offline, startup");
}
