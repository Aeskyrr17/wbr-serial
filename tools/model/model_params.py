"""Reviewed model inputs from wbr_2026/User/Matlab/model.m and fit.m."""

from dataclasses import dataclass


@dataclass(frozen=True)
class Physical:
    wheel_radius: float = 0.0752
    half_track: float = 0.422 / 2.0
    body_com_offset: float = 0.126
    wheel_mass: float = 0.21
    leg_mass: float = 1.62
    body_mass: float = 13.85
    wheel_inertia: float = 0.00038332661
    body_inertia: float = 0.462592306
    yaw_inertia: float = 0.314373184
    gravity: float = 9.78


PHYSICAL = Physical()
TRAIN_LENGTHS = tuple(round(0.10 + 0.01 * i, 2) for i in range(28))
RUNTIME_LENGTHS = (0.16, 0.35)
ALPHA_EQ_LEGACY = (0.280918, -1.101757, 1.232768)

# x, v, yaw, dyaw, alpha_l, dalpha_l, alpha_r, dalpha_r, pitch, dpitch
# Twl, Twr, Tpl, Tpr

QR = {
    "low": (
        (300, 20, 300, 100, 20000, 600, 20000, 600, 150000, 1500),
        (20, 20, 1, 1),
    ),
    "high": (
        (200, 80, 300, 100, 15000, 1500, 15000, 1500, 120000, 2500),
        (20, 20, 1, 1),
    ),
    "spin": (
        (300, 100, 300, 100, 25000, 800, 25000, 800, 220000, 2500),
        (20, 20, 1, 1),
    ),
}


def leg_data(length: float) -> tuple[float, float, float]:
    """Linear table represented by legacy fit.m rows 0.10..0.40."""
    delta = length - 0.10
    return 0.00320 + 0.782 * delta, 0.09680 + 0.218 * delta, 0.110 + 0.4 * delta
