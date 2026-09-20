"""Generate the firmware LQR/kinematics headers and a traceable report."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

import numpy as np

from dynamics import lqr_gain, state_space
from kinematics import render_header
from model_params import ALPHA_EQ_LEGACY, PHYSICAL, QR, RUNTIME_LENGTHS, TRAIN_LENGTHS

ROOT = Path(__file__).resolve().parents[2]
LEGACY = ROOT / "app/chassis/legacy_lqr_coeffs.hpp"
ARRAY_NAMES = {"low": "LQRKLowcoeffs", "high": "LQRKHighcoeffs", "spin": "LQRKSpincoeffs"}


def write_changed(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.read_text(encoding="utf-8") == content:
        return
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(content, encoding="utf-8", newline="\n")
    temporary.replace(path)


def legacy_coefficients() -> dict[str, np.ndarray]:
    text = LEGACY.read_text(encoding="utf-8")
    result = {}
    for mode, name in ARRAY_NAMES.items():
        match = re.search(rf"{name}\[40\]\[6\]\s*=\s*\{{(.*?)\n\}};", text, re.S)
        if not match:
            raise RuntimeError(f"missing {name} in {LEGACY}")
        rows = re.findall(r"\{\s*([^{}]+?)\s*\}", match.group(1))
        values = [[float(item) for item in row.split(",") if item.strip()] for row in rows]
        array = np.asarray(values, dtype=float)
        if array.shape != (40, 6):
            raise RuntimeError(f"{name} has shape {array.shape}, expected (40, 6)")
        result[mode] = array
    return result


def regenerate_coefficients() -> tuple[dict[str, np.ndarray], dict]:
    samples = [(left, right) for left in TRAIN_LENGTHS for right in TRAIN_LENGTHS]
    basis = np.asarray([[1, l, r, l*l, l*r, r*r] for l, r in samples])
    result, checks = {}, {}
    for mode, (qdiag, rdiag) in QR.items():
        gains, care = [], []
        for left, right in samples:
            a, b = state_space(left, right)
            gain, residual = lqr_gain(a, b, qdiag, rdiag)
            gains.append(gain.reshape(40)); care.append(residual)
        gains = np.asarray(gains)
        coeff = np.linalg.lstsq(basis, gains, rcond=None)[0].T
        fit = basis @ coeff.T
        max_error = float(np.max(np.abs(fit - gains)))
        max_real = -np.inf
        max_discrete_radius = 0.0
        for left in np.linspace(TRAIN_LENGTHS[0], TRAIN_LENGTHS[-1], 15):
            for right in np.linspace(TRAIN_LENGTHS[0], TRAIN_LENGTHS[-1], 15):
                a, b = state_space(float(left), float(right))
                v = np.array([1, left, right, left*left, left*right, right*right])
                saved = (coeff @ v).reshape(4, 10)
                eigenvalues = np.linalg.eigvals(a + b @ saved)
                max_real = max(max_real, float(np.max(np.real(eigenvalues))))
                max_discrete_radius = max(
                    max_discrete_radius, float(np.max(np.abs(np.exp(eigenvalues * 0.001)))))
        if (not np.isfinite(max_error) or max_error > 3.0 or max(care) > 1.0e-5 or
                max_real >= 0.0 or max_discrete_radius >= 1.0):
            raise RuntimeError(f"{mode} regenerated model failed stability/error checks")
        result[mode] = coeff
        checks[mode] = {"max_fit_abs": max_error, "max_care_inf": max(care),
                        "dense_max_closed_loop_real": max_real,
                        "dense_1ms_max_spectral_radius": max_discrete_radius}
    return result, checks


def cpp_float(value: float) -> str:
    text = f"{value:.9g}"
    if "." not in text and "e" not in text.lower():
        text += ".0"
    return text + "f"


def render_coefficients(coefficients: dict[str, np.ndarray]) -> str:
    lines = ["#pragma once", "", "namespace robot::chassis", "{"]
    for mode in ("low", "high", "spin"):
        lines.append(f"inline constexpr float {ARRAY_NAMES[mode]}[40][6] = {{")
        for row in coefficients[mode]:
            lines.append("    {" + ", ".join(cpp_float(value) for value in row) + "},")
        lines += ["};", ""]
    lines += ["} // namespace robot::chassis", ""]
    return "\n".join(lines)


def generate(mode: str, output_dir: Path) -> None:
    if mode == "legacy":
        coefficients, checks = legacy_coefficients(), {"legacy_snapshot": "exact decimal import"}
    else:
        coefficients, checks = regenerate_coefficients()
        legacy = legacy_coefficients()
        checks["max_coefficient_delta_from_legacy"] = {
            name: float(np.max(np.abs(coefficients[name] - legacy[name]))) for name in coefficients
        }
    header = render_coefficients(coefficients)
    source_hash = hashlib.sha256(LEGACY.read_bytes()).hexdigest()
    report = {
        "mode": mode,
        "basis": ["1", "L", "R", "L^2", "L*R", "R^2"],
        "shape": [4, 10, 6],
        "sample_order": "outer left, inner right (right changes fastest)",
        "training_range_m": [TRAIN_LENGTHS[0], TRAIN_LENGTHS[-1]],
        "runtime_range_m": list(RUNTIME_LENGTHS),
        "firmware_compatibility": {"wheel_radius_m": 0.06, "wheel_track_m": 0.43,
                                    "body_mass_kg": 14.0},
        "matlab_model": PHYSICAL.__dict__,
        "alpha_eq": {"coefficients": ALPHA_EQ_LEGACY, "source": "legacy config inheritance"},
        "qr": QR,
        "checks": checks,
        "legacy_source_sha256": source_hash,
        "versions": {"python": sys.version.split()[0], "numpy": np.__version__},
    }
    write_changed(output_dir / "lqr_coeffs.hpp", header)
    write_changed(output_dir / "kinematics.hpp", render_header())
    write_changed(output_dir / "model-report.json", json.dumps(report, indent=2) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("legacy", "regenerate"), default="regenerate")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--preset", default="h723-debug")
    parser.add_argument("--build", action="store_true")
    args = parser.parse_args()
    output = args.output_dir or ROOT / "app" / "chassis" / "model"
    generate(args.mode, output)
    if args.build:
        subprocess.run(["cmake", "--preset", args.preset], cwd=ROOT, check=True)
        subprocess.run(["cmake", "--build", "--preset", args.preset], cwd=ROOT, check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
