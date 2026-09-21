#include "vmc.hpp"
#include <cstdio>
#include <cstdlib>
#include <limits>
using namespace robot::chassis;
void check(bool ok) { if (!ok) std::abort(); }
int main() {
    for (float bad : {-1.0f, 0.0f, 0.49f, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()})
        check(!solve_spring(bad).spring_valid);
    check(!solve_spring(0.48f).spring_valid);
    check(solve_spring(0.045f).fs < 0); // Signed beta below alpha_s must survive.
    vmc solver;
    check(!solver.resolve(std::numeric_limits<float>::quiet_NaN(), 0));
    check(!solver.state().spring_valid);
    std::puts("time,len,phi,spring_len,dls_dlen,fs,spring_valid,phi1,phi4");
    unsigned tick=0;
    for (float phi : {0.8f, 1.5707963268f, 2.7f}) {
        for (int i=0;i<=1900;++i) {
            const float len=.16f+i*.0001f;
            const float a=std::acos((model::L1*model::L1+len*len-model::L2*model::L2)/(2*model::L1*len));
            const float p1=phi+a, p4=phi-a;
            check(solver.resolve(p1,p4));
            const auto& s=solver.state();
            check(s.spring_valid && std::fabs(s.len-len)<1e-5f);
            std::printf("%u,%.9g,%.9g,%.9g,%.9g,%.9g,1,%.9g,%.9g\n",tick++,s.len,s.phi,s.spring_len,s.dls_dlen,s.fs,p1,p4);
        }
    }
    // Invalid input after valid samples must not reuse a good spring result.
    check(!solver.resolve(std::numeric_limits<float>::quiet_NaN(),0.0f));
    check(!solver.state().spring_valid);
    check(std::isnan(solver.state().fs));
}
