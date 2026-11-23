/* Derived from TinyMPC admm.cpp, commit 023f36bd5b27267e1a96ed47a774d120b5707f13.
 * C port of backward_pass_grad, forward_pass,
 * update_slack, update_dual, update_linear_cost and termination_condition.
 * No Eigen, allocation, output, adaptive rho, SOC or general linear constraints.
 * The prediction horizon is generated along with the model. See MIT LICENSE. */
#include "tiny_mpc.h"
#include <math.h>

static float clip(float x, float lo, float hi) { return fminf(hi, fmaxf(lo,x)); }
int tiny_mpc_solve(tiny_workspace *w, const tiny_cache *c,
                   int max_iter, tiny_clock clock, uint64_t deadline)
{
    w->iterations=0; w->status=11;
    for (int iter=0; iter<max_iter; ++iter) {
        if (clock && clock() >= deadline) { w->status=12; return 12; }
        /* Use the terminal cost Hessian Pinf-rho*I for the reference gradient. */
        const int last = TINY_KNOTS - 1;
        for (int j=0;j<TINY_NX;++j) {
            float s=0;
            for(int h=0;h<TINY_NX;++h) s -= c->pinf[j][h]*w->ref[last][h];
            w->p[last][j]=s+c->rho*w->ref[last][j]-c->rho*(w->v[last][j]-w->g[last][j]);
        }
        for (int t=TINY_STEPS-1;t>=0;--t) {
            float rr[TINY_NU], tmp[TINY_NU];
            for(int j=0;j<TINY_NU;++j) {
                rr[j]=-c->rho*(w->z[t][j]-w->y[t][j]);
                tmp[j]=rr[j];
                for(int h=0;h<TINY_NX;++h) tmp[j]+=c->b[h][j]*w->p[t+1][h];
            }
            for(int j=0;j<TINY_NU;++j) {
                w->d[t][j]=0;
                for(int h=0;h<TINY_NU;++h) w->d[t][j]+=c->inv[j][h]*tmp[h];
            }
            for(int j=0;j<TINY_NX;++j) {
                float s=-c->q[j]*w->ref[t][j]-c->rho*(w->v[t][j]-w->g[t][j]);
                for(int h=0;h<TINY_NX;++h) s+=c->ambkt[j][h]*w->p[t+1][h];
                for(int h=0;h<TINY_NU;++h) s-=c->k[h][j]*rr[h];
                w->p[t][j]=s;
            }
        }
        for(int t=0;t<TINY_STEPS;++t) {
            for(int j=0;j<TINY_NU;++j) {
                float s=-w->d[t][j];
                for(int h=0;h<TINY_NX;++h) s-=c->k[j][h]*w->x[t][h];
                w->u[t][j]=s;
            }
            for(int j=0;j<TINY_NX;++j) {
                float s=0;
                for(int h=0;h<TINY_NX;++h) s+=c->a[j][h]*w->x[t][h];
                for(int h=0;h<TINY_NU;++h) s+=c->b[j][h]*w->u[t][h];
                w->x[t+1][j]=s;
            }
        }
        float pri=0, dua=0;
        for(int t=0;t<TINY_KNOTS;++t) for(int j=0;j<TINY_NX;++j) {
            float next=w->x[t][j]+w->g[t][j];
            if(!isfinite(next)) { w->status=13; return 13; }
            if(j>=TINY_NX-TINY_NU) next=clip(next,w->force_min,w->force_max);
            pri=fmaxf(pri,fabsf(w->x[t][j]-next));
            dua=fmaxf(dua,c->rho*fabsf(w->v[t][j]-next));
            w->g[t][j]+=w->x[t][j]-next;
            w->v[t][j]=next;
        }
        for(int t=0;t<TINY_STEPS;++t) for(int j=0;j<TINY_NU;++j) {
            float next=w->u[t][j]+w->y[t][j];
            if(!isfinite(next)) { w->status=13; return 13; }
            next=clip(next,-w->delta_max,w->delta_max);
            pri=fmaxf(pri,fabsf(w->u[t][j]-next));
            dua=fmaxf(dua,c->rho*fabsf(w->z[t][j]-next));
            w->y[t][j]+=w->u[t][j]-next;
            w->z[t][j]=next;
        }
        w->iterations=iter+1; w->primal=pri; w->dual=dua;
        if(pri<1.e-4f && dua<1.e-3f) { w->status=1; return 1; }
    }
    return 11;
}
