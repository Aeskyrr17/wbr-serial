"""Generate only the leg height/roll gain header and report; never build firmware."""

import argparse
from dataclasses import asdict
import json
from pathlib import Path

from dynamics import design
from model_params import PHYSICAL, Q_DIAG, R_DIAG, ROOT, SAMPLE_TIME


def cpp_float(value):
    text = f"{value:.9g}"
    if "." not in text and "e" not in text.lower():
        text += ".0"
    return text + "f"


def render(k, ueq):
    rows = "\n".join("    {" + ", ".join(cpp_float(v) for v in row) + "}," for row in k)
    return f'''#pragma once

// Generated explicitly by tools/leg_lqr/generate.py. Ordinary builds do not regenerate.
namespace chassis::leg_lqr_model
{{
inline constexpr float sample_time = {cpp_float(SAMPLE_TIME)};
inline constexpr float mass = {cpp_float(PHYSICAL.mass)};
inline constexpr float half_track = {cpp_float(PHYSICAL.half_track)};
inline constexpr float roll_inertia = {cpp_float(PHYSICAL.roll_inertia)};
inline constexpr float vertical_damping = {cpp_float(PHYSICAL.vertical_damping)};
inline constexpr float equilibrium_force = {cpp_float(ueq[0])};
// Rows: left/right total support force (N). Columns: roll, droll, H, dH.
// Negative feedback is already included: F = Mg/2 + gain * (obs - ref).
inline constexpr float gain[2][4] = {{
{rows}
}};
}} // namespace chassis::leg_lqr_model
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "chassis/model")
    parser.add_argument("--check", action="store_true", help="Verify saved output without modifying it")
    args = parser.parse_args()
    a, b, ad, bd, k, ueq, checks = design()
    report = {
        "state_order": ["roll", "droll", "H", "dH"],
        "input_order": ["F_N_left", "F_N_right"],
        "positive_roll": "left high, right low; IMU +X forward, +Z up",
        "force_semantics": "total support in roll plane before cos(roll); sagittal projection applied before VMC",
        "physical": dict(asdict(PHYSICAL), total_mass=PHYSICAL.mass),
        "parameter_sources": {
            "body_mass,wheel_mass,wheel_track,gravity": "chassis/model/chassis_model.hpp",
            "single_leg_mass": "tools/model/model_params.py",
            "roll_inertia,vertical_damping": "SDUT reference 0.64 kg m^2 and 8 N s/m; uncalibrated",
        },
        "sample_time": SAMPLE_TIME,
        "Q_diag": Q_DIAG, "R_diag": R_DIAG,
        "A": a.tolist(), "B": b.tolist(), "Ad": ad.tolist(), "Bd": bd.tolist(),
        "K_negative_feedback": k.tolist(), "equilibrium_input": ueq.tolist(),
        "checks": checks,
    }
    outputs = {
        "leg_lqr_coeffs.hpp": render(k, ueq),
        "leg-lqr-report.json": json.dumps(report, ensure_ascii=False, indent=2) + "\n",
    }
    for name, content in outputs.items():
        path = args.output_dir / name
        if args.check:
            if not path.exists() or path.read_text(encoding="utf-8") != content:
                raise SystemExit(f"Stale output: {path}; run generate.py")
        elif not path.exists() or path.read_text(encoding="utf-8") != content:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content, encoding="utf-8", newline="\n")
    print(f"M={PHYSICAL.mass:.5g} kg, R={PHYSICAL.half_track:.5g} m, "
          f"rank=4, float32 closed-loop radius={checks['float32_spectral_radius']:.9f}")


if __name__ == "__main__":
    main()
