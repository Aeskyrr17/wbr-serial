/* C adaptation of TinyMPC's box-constrained ADMM core. MIT; see LICENSE. */
#ifndef TINY_MPC_H
#define TINY_MPC_H
#include "../generated/horizon.h"
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
enum { TINY_NX=6, TINY_NU=2 };
typedef uint64_t (*tiny_clock)(void);
typedef struct {
    const float (*a)[TINY_NX]; const float (*b)[TINY_NU]; const float *q; const float *r;
    const float (*pinf)[TINY_NX]; const float (*k)[TINY_NX]; const float (*inv)[TINY_NU];
    const float (*ambkt)[TINY_NX]; float rho;
} tiny_cache;
typedef struct {
    float x[TINY_KNOTS][TINY_NX], u[TINY_STEPS][TINY_NU];
    float v[TINY_KNOTS][TINY_NX], z[TINY_STEPS][TINY_NU];
    float g[TINY_KNOTS][TINY_NX], y[TINY_STEPS][TINY_NU];
    float p[TINY_KNOTS][TINY_NX], d[TINY_STEPS][TINY_NU];
    float ref[TINY_KNOTS][TINY_NX];
    float force_min, force_max, delta_max;
    float primal, dual;
    int iterations, status;
} tiny_workspace;
/* status: 1 solved, 11 iteration limit, 12 deadline, 13 nonfinite.
 * Deadline polled each iteration; cannot preempt a single iteration or IRQ.
 * This API is a local adapter, not a stock TinyMPC API. */
int tiny_mpc_solve(tiny_workspace*, const tiny_cache*,
                   int max_iter, tiny_clock clock, uint64_t deadline);
#ifdef __cplusplus
}
#endif
#endif
