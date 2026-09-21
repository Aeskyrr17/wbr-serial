# Fs-vali: analytic spring validation

## Geometry

Existing coordinates: O=(0,0), B=L1*[cos(phi1),sin(phi1)],
P=d1*[cos(phi1+gamma-pi/2),sin(phi1+gamma-pi/2)], Q=B+d2*[cos(u2),sin(u2)].
Thus OP=d1=0.03 m, BQ=d2=0.05 m, gamma=0.2164 rad, L1=0.22 m, L2=0.26 m.
BP=d3=sqrt(L1^2+d1^2-2*L1*d1*sin(gamma))=0.215559184885 m.
alpha_s=0.136349036154 rad is NOT spring_angle or chassis alpha.
Clockwise BQ from OB, sin(u2-phi1)<0, gives beta_s=theta-alpha_s.
The opposite orientation is rejected, never absolute-valued.
No separate spring drawing was found in this checkout. Mapping is verified against
existing coordinates and the user-specified point definitions, not physical measurements.
Fixed d3/alpha_s are cached; source constants remain in chassis_model.hpp, Fg=250 N.

## Scope and validity

Only spring calculation replaced. J/JT/JT_inv, velocity, torque/feedback mappings,
PID/LQR and state machine remain unchanged. Neutral/Normal retain their sole -fs.
Motor output defaults false. Signed dls_dlen and fs are preserved.
Checks: finite positive dimensions; finite inputs/results; acos tolerance 1e-6;
sin(theta)>1e-3; spring_len squared>1e-12 m^2. Clearly invalid domains fail.
Every solve resets spring results to NaN/invalid, with spring_error enum:
none, leg_geometry, dimensions, domain, branch, singular, nonfinite.
Failure returns false and propagates through pendulum to existing Relax path.

## Capture and reproduce

robot_debug.chassis.sample_tick is RTOS ticks. Read sample_seq before/after both
left/right snapshots and accept only equal even values, or halt CPU for capture.
Each leg contains len,phi,spring_len,dls_dlen,fs,spring_valid,error.
robot_debug.chassis.spring_force records Fg. Export one CSV per leg with:
time,len,phi,spring_len,dls_dlen,fs,spring_valid (boolean as 0/1).
Optional phi1/phi4 enable old coordinate length comparison.

Compile tests/host/spring_validation.cpp and app/chassis/vmc.cpp with C++17 and
-Iapp/chassis. Run executable, redirect stdout to build/spring_samples.csv.
Then:

    python tools/model/validate_spring.py build/spring_samples.csv --force 250 --out build/spring-validation

Also accepts measured CSV. Rejects invalid/nonfinite samples, reversal/stops,
small length differences, uneven spacing, angle changes and singular vicinity.
Strides 5/10/20 check spacing sensitivity. Output forces.csv, forces.svg, report.txt.
The validation coordinate constants are stated above; update if dimensions change.
SVG horizontal range is the intended 0.16-0.35 m sweep. No finite differences in firmware.

## Executed results

Production C++ test: 5703 samples, phi=0.8,pi/2,2.7 rad, len=0.16..0.35 m.
Old coordinate vs new spring length max error 3.224821e-8 m.
Central-difference half-spacings 0.5/1/2 mm: max force errors
0.006539 / 0.003458 / 0.001938 N, accepted 5673/5643/5583 samples.
Negative fs near len=0.045 m preserved. Invalid length, NaN, infinity,
full-extension singularity and stale-result-reset assertions pass.
No hardware or measured-log validation performed.

H723 full build blocked: app/robot.cpp uses dmmotorhandler::debug and
DM8009P::debug absent from current pnx_devices. Changed control units compile.
Unrelated Device code was not modified; no firmware flashed.

## Direct old/new force comparison

Ran tools/model/compare_spring_force.py against pre-change coordinate VMC from
commit cf53ed19615d6e6a738d2ab6b1fa02b7664a75ed and current production VMC.
Both compiled as native C++ float using the same model constants/Fg=250 N.
Input is the identical 5703-pose CSV above, not independently regenerated poses.
Legacy guard hits are explicitly tagged; formula-valid means resolve success,
no spring protection branch, finite fs. Guard-zero values are excluded.
All 5703 points were formula-valid in both; validity mismatches=0, old guard hits=0.
Maximum absolute force difference=4.19616699e-5 N at sample 4231:
phi1=4.02217007 rad, phi4=1.37783027 rad, len=0.202899948 m,
phi=2.70000029 rad, fs_old=46.095211 N, fs_new=46.0951691 N.
Artifacts: build/spring-force-comparison/{all,normal,validity_mismatch,old_protected}.csv
and report.json. Empty mismatch/protection tables retain headers.
This is limited to the stated pose sweep, not proof of identical validity at
singularities or other assembly branches. Host math library used, not on-MCU CMSIS.

Current build status: H723 Debug now compiles and links after removing application references to the uncommitted Device debug API. robot_debug.motors exposes const pointers to the existing four dmmotor objects and two wheel motor base objects. Watch ->fdb, ->alive, ->motor_state and ->cmd; DM also exposes ->raw, ->enablefailed and ->last_tx_id. registered/starting/start_status remain application-owned. No joint_bus/debug TX-error counters remain; alive counts parsed feedback, not every CAN frame. Device files unchanged by this replacement. Output defaults false; no firmware flashed. This supersedes earlier missing-debug build-block notes.
