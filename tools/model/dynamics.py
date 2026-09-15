"""Numerical form of the five linear equations in legacy model.m."""

import numpy as np
from scipy.linalg import solve_continuous_are

from model_params import PHYSICAL, leg_data


def state_space(left: float, right: float) -> tuple[np.ndarray, np.ndarray]:
    p = PHYSICAL
    lwl, lbl, ill = leg_data(left)
    lwr, lbr, ilr = leg_data(right)
    rw, rl, lc = p.wheel_radius, p.half_track, p.body_com_offset
    mw, ml, mb, iw, ib, iz, g = (
        p.wheel_mass, p.leg_mass, p.body_mass, p.wheel_inertia,
        p.body_inertia, p.yaw_inertia, p.gravity,
    )
    common = mw * rw * rw + iw + ml * rw * rw + mb * rw * rw / 2.0
    yaw_wheel = iz * rw / (2.0 * rl) + iw * rl / rw
    mass = np.array([
        [iw*left/rw + mw*rw*left + ml*rw*lbl, 0, ml*lwl*lbl-ill, 0, 0],
        [0, iw*right/rw + mw*rw*right + ml*rw*lbr, 0, ml*lwr*lbr-ilr, 0],
        [-common, -common, -(ml*rw*lwl + mb*rw*left/2), -(ml*rw*lwr + mb*rw*right/2), 0],
        [mw*rw*lc + iw*lc/rw + ml*rw*lc, mw*rw*lc + iw*lc/rw + ml*rw*lc,
         ml*lwl*lc, ml*lwr*lc, -ib],
        [yaw_wheel, -yaw_wheel, iz*left/(2*rl), -iz*right/(2*rl), 0],
    ], dtype=float)
    stiffness = np.zeros((5, 3))
    stiffness[0, 0] = (ml*lwl + mb*left/2) * g
    stiffness[1, 1] = (ml*lwr + mb*right/2) * g
    stiffness[3, 2] = mb * g * lc
    inputs = np.array([
        [-(1+left/rw), 0, 1, 0],
        [0, -(1+right/rw), 0, 1],
        [1, 1, 0, 0],
        [-lc/rw, -lc/rw, -1, -1],
        [-rl/rw, rl/rw, 0, 0],
    ], dtype=float)
    ja = np.linalg.solve(mass, -stiffness)
    jb = np.linalg.solve(mass, -inputs)
    a = np.zeros((10, 10)); b = np.zeros((10, 4))
    for i in range(5): a[2*i, 2*i+1] = 1.0
    for source, column in enumerate((4, 6, 8)):
        a[1, column] = rw * (ja[0, source] + ja[1, source]) / 2
        a[3, column] = (rw*(-ja[0, source]+ja[1, source]) - left*ja[2, source] + right*ja[3, source])/(2*rl)
        for row in range(2, 5): a[2*row+1, column] = ja[row, source]
    for column in range(4):
        b[1, column] = rw * (jb[0, column] + jb[1, column]) / 2
        b[3, column] = (rw*(-jb[0, column]+jb[1, column]) - left*jb[2, column] + right*jb[3, column])/(2*rl)
        for row in range(2, 5): b[2*row+1, column] = jb[row, column]
    return a, b


def lqr_gain(a: np.ndarray, b: np.ndarray, qdiag, rdiag) -> tuple[np.ndarray, float]:
    q, r = np.diag(qdiag), np.diag(rdiag)
    p = solve_continuous_are(a, b, q, r)
    saved = -np.linalg.solve(r, b.T @ p)
    residual = a.T @ p + p @ a - p @ b @ np.linalg.solve(r, b.T @ p) + q
    return saved, float(np.linalg.norm(residual, ord=np.inf))
