"""Leg height / roll LQR parameters. Q order: roll, droll, H, dH."""

from dataclasses import dataclass
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]


def firmware_value(name: str) -> float:
    text = (ROOT / "chassis/model/chassis_model.hpp").read_text(encoding="utf-8")
    return float(re.search(rf"\b{name}\s*=\s*([0-9.eE+-]+)f?\s*;", text)[1])


def leg_mass() -> float:
    text = (ROOT / "tools/model/model_params.py").read_text(encoding="utf-8")
    return float(re.search(r"leg_mass:\s*float\s*=\s*([0-9.eE+-]+)", text)[1])


@dataclass(frozen=True)
class Physical:
    body_mass: float = firmware_value("body_mass")
    leg_mass: float = leg_mass()
    wheel_mass: float = firmware_value("wheel_mass")
    half_track: float = firmware_value("wheel_track") / 2.0
    gravity: float = firmware_value("gravity")
    roll_inertia: float = 0.64  # SDUT reference, not measured on this robot.
    vertical_damping: float = 8.0  # N s/m; SDUT reference.

    @property
    def mass(self) -> float:
        return self.body_mass + 2.0 * self.leg_mass + 2.0 * self.wheel_mass


PHYSICAL = Physical()
SAMPLE_TIME = 0.001  # Current control thread: 1 ms, not the reference MPC's 10 ms.
# Initial weights, not a hardware tuning result. R order: left/right support force.
# Approximate per-leg height stiffness 2000 N/m, roll stiffness 1000 N/rad.
# (roll, droll, H, dH)
# Q_DIAG = (2_000_000.0, 20_000.0, 8_000_000.0, 200_000.0)
Q_DIAG = (800_000.0, 150.0, 800_000.0, 100.0)
R_DIAG = (1.0, 1.0)
