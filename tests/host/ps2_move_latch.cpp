#include "command.hpp"
#include <cstdio>
#include <cstdlib>
using namespace robot;
int main(){
 remoter::state rc{};rc.offline=false;rc.active_source=remoter::source::ps2;
 const auto l1=static_cast<unsigned>(remoter::ps2_button::l1);
 const auto l2=static_cast<unsigned>(remoter::ps2_button::l2);
 auto check=[&](unsigned keys,bool expected){rc.ps2_buttons=keys;if(command::map_ps2(rc,0).move!=expected)std::abort();};
 check(0,false);check(l1,false);check(l1|l2,true);
 for(int i=0;i<100;++i)check(l1|l2,true);
 check(l2,true);check(0,true);check(l1,false);check(l1|l2,false);
 check(0,false);check(l1|l2,true);check(0,true);check(l2,false);
 check(0,false);check(l1|l2,true);rc.offline=true;check(l1|l2,false);
 rc.offline=false;check(l1|l2,false);check(0,false);check(l1|l2,true);
 rc.offline=true;check(0,false);
 std::puts("PASS: chord entry, held/released latch, either-key exit, offline/reconnect");
}
