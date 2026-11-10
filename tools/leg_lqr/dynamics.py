"""User-specified nonlinear dynamics, analytic Jacobians and discrete LQR."""

import numpy as np
from scipy.linalg import expm, solve_discrete_are

from model_params import PHYSICAL, Q_DIAG, R_DIAG, SAMPLE_TIME


def dynamics(x, u, p=PHYSICAL):
    roll, droll, height, dheight = x
    left, right = u
    return np.array([
        droll,
        (left - right) * p.half_track / p.roll_inertia,
        dheight,
        ((left + right) * np.cos(roll) - p.vertical_damping * dheight) / p.mass - p.gravity,
    ])


def jacobians(x, u, p=PHYSICAL):
    """df/dx and df/du; no numerical differencing in gain generation."""
    a = np.zeros((4, 4))
    a[0, 1] = a[2, 3] = 1.0
    a[3, 0] = -(u[0] + u[1]) * np.sin(x[0]) / p.mass
    a[3, 3] = -p.vertical_damping / p.mass
    b = np.zeros((4, 2))
    b[1] = (p.half_track / p.roll_inertia, -p.half_track / p.roll_inertia)
    b[3] = np.cos(x[0]) / p.mass
    return a, b


def state_space(height=0.25, p=PHYSICAL):
    xeq = np.array([0.0, 0.0, height, 0.0])
    ueq = np.full(2, p.mass * p.gravity / 2.0)
    a, b = jacobians(xeq, ueq, p)
    return a, b, xeq, ueq


def design():
    a, b, xeq, ueq = state_space()
    augmented = np.zeros((6, 6))
    augmented[:4, :4], augmented[:4, 4:] = a, b
    transition = expm(augmented * SAMPLE_TIME)  # Exact zero-order hold.
    ad, bd = transition[:4, :4], transition[:4, 4:]
    q, r = np.diag(Q_DIAG), np.diag(R_DIAG)
    p = solve_discrete_are(ad, bd, q, r)
    # Save negative feedback, matching the original firmware's gain convention.
    k = -np.linalg.solve(r + bd.T @ p @ bd, bd.T @ p @ ad)
    residual = ad.T @ p @ ad - p + q + ad.T @ p @ bd @ k
    controllability = np.hstack([np.linalg.matrix_power(a, i) @ b for i in range(4)])
    poles = np.linalg.eigvals(ad + bd @ k)
    radius = float(np.max(np.abs(poles)))
    # Check the rounded float32 coefficients actually used by the firmware too.
    saved_radius = float(np.max(np.abs(np.linalg.eigvals(ad + bd @ k.astype(np.float32)))))
    relative_residual = float(np.linalg.norm(residual, np.inf) / max(1.0, np.linalg.norm(p, np.inf)))
    if np.linalg.matrix_rank(controllability) != 4 or max(radius, saved_radius) >= 1.0 or relative_residual > 1e-8:
        raise RuntimeError("Leg LQR controllability, stability or Riccati residual check failed")
    return a, b, ad, bd, k, ueq, {
        "controllability_rank": 4,
        "closed_loop_poles": [[float(v.real), float(v.imag)] for v in poles],
        "spectral_radius": radius,
        "float32_spectral_radius": saved_radius,
        "relative_dare_residual": relative_residual,
    }
