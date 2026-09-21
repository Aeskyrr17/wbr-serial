# Direct VMC validation

## Scope and branch

Production change: app/chassis/vmc.cpp/.hpp; geometry failure is copied by
pendulum.cpp into robot_types.hpp leg debug. Spring implementation is unchanged.
No PID/LQR, state machine, sign conversion, feedback scaling, limits or -fs changes.
Motor output remains false; no flash or commit.

The baseline in tests/host/vmc_baseline is an exact snapshot of the previously
modified production working tree (coordinate VMC + verified analytic spring).
README hashes document provenance. Test compilation only renames its namespace.
The old legacy-coordinate spring routine is NOT the baseline for this change.

For the symmetric zero-base-spacing mechanism, set delta=(phi1-phi4) mod 2pi
in [0,2pi), eta=delta/2, phi=wrap(phi4+eta). Relative-angle correction precedes
averaging; normalization of each angle alone would not suffice. sin(eta)>0
selects C on the left of the directed B-to-D chord, exactly the old +sqrt closure.
With current L2>L1, len=L1*cos(eta)+h stays positive. delta=0 is ambiguous and
singular: reject instead of choosing a previous-frame branch. This rule works
at initialization and for either joint shifted by an integer turn. Only output
phi wraps; the existing pendulum total_phi tracking remains unchanged.
No u2/u3 or endpoint coordinates remain in the production path.

## Protection

Finite inputs, dimensions and all output matrix entries; positive dimensions.
h>1e-6 m; len>1e-6 m; abs(k)>1e-6 m/rad. Nonpositive/near-zero radicand fails,
not clamped into a divisor. k stays signed.
Retain old dimensionless |sin32| and |sin12|/|sin34| threshold 1e-5 via algebraic
identities, and old half-angle chart |A+C|>=1e-6 m^2. This explicitly avoids
expanding validity into the old coordinate chart exclusion. Validity may still
differ at floating-point threshold boundaries outside tested samples.
All matrices and len/phi reset to NaN before each solve; spring results likewise
reset invalid. vmc_state.valid and geometry_error report status. A failed solve
never exposes last cycle as valid. Existing Relax propagation is unchanged.

## Reproduction and coverage

python tests/host/run_vmc_direct.py --cxx <zig.exe> --zig

Requires the prior build/spring_samples.csv (5703 exact saved input poses).
Outputs build/vmc-direct/all.csv, validity_mismatch.csv and report.json.
Additional motion grid: len=0.045..0.474 m in 0.00195 m steps, 14 phi values from
-3.15..3.15 rad, including flatten/neutral orientations and +/-pi.
Five individual/opposing integer-turn variants per motion pose.
Boundary grid includes coincident links, near full extension/retraction;
chart grid targets A+C=0; invalid inputs include NaN/infinity.
Velocity test qdot=(0.7,-0.4) rad/s; force-to-torque Q=(83 N,-2.3 Nm);
torque-to-force tau=(3.1,-1.7) Nm. Do not read these interface-error numbers as
bounds for all possible input amplitudes.

## Results by group

### original5703

Samples 5703; both valid 5703; validity differences 0.

| Quantity | Unit | Max absolute difference | Sample | phi1 / phi4 (rad) | len (m) / phi (rad) |
|---|---|---:|---:|---|---|
| len | m | 8.94069672e-08 | 906 | 1.96427369 / -0.364273608 | 0.25059998 / 0.800000012 |
| phi | rad | 4.76837158e-07 | 3833 | 4.16747475 / 1.23252559 | 0.16309993 / 2.70000029 |
| J0 | m/rad | 7.4505806e-08 | 2039 | 2.9971633 / 0.144429326 | 0.173799977 / 1.57079637 |
| J1 | m/rad | 5.96046448e-08 | 1977 | 3.02075005 / 0.120842814 | 0.167599991 / 1.57079637 |
| J2 | dimensionless | 3.87430191e-07 | 3833 | 4.16747475 / 1.23252559 | 0.16309993 / 2.70000029 |
| J3 | dimensionless | 3.27825546e-07 | 3945 | 4.12449074 / 1.27550948 | 0.17429997 / 2.70000029 |
| JT_inv0 | 1/m | 2.62260437e-06 | 3859 | 4.15730762 / 1.24269259 | 0.165699989 / 2.70000029 |
| JT_inv1 | 1/m | 3.09944153e-06 | 3910 | 4.13770437 / 1.26229584 | 0.170799941 / 2.70000029 |
| JT_inv2 | dimensionless | 2.98023224e-07 | 155 | 2.22000217 / -0.620002091 | 0.175500005 / 0.800000012 |
| JT_inv3 | dimensionless | 7.15255737e-07 | 3866 | 4.15459108 / 1.24540913 | 0.166399956 / 2.70000029 |
| velocity_len | m/s | 7.4505806e-08 | 2328 | 2.89365387 / 0.247938752 | 0.202700004 / 1.57079637 |
| velocity_phi | rad/s | 3.42726707e-07 | 3833 | 4.16747475 / 1.23252559 | 0.16309993 / 2.70000029 |
| torque1 | Nm | 6.67572021e-06 | 2039 | 2.9971633 / 0.144429326 | 0.173799977 / 1.57079637 |
| torque4 | Nm | 5.7220459e-06 | 3333 | 2.56499887 / 0.576593816 | 0.303200036 / 1.57079625 |
| force_len | N | 7.62939453e-06 | 24 | 2.27023292 / -0.670232832 | 0.162399992 / 0.800000012 |
| force_phi | Nm | 1.78813934e-06 | 3809 | 4.17697096 / 1.22302926 | 0.160699949 / 2.70000029 |
| spring_len | m | 2.98023224e-08 | 815 | 1.99375129 / -0.393751204 | 0.241500005 / 0.800000012 |
| dls_dlen | m/m | 1.1920929e-07 | 914 | 1.96168923 / -0.36168927 | 0.251399994 / 0.800000012 |
| fs | N | 2.67028809e-05 | 914 | 1.96168923 / -0.36168927 | 0.251399994 / 0.800000012 |

### motion

Samples 3094; both valid 3094; validity differences 0.

| Quantity | Unit | Max absolute difference | Sample | phi1 / phi4 (rad) | len (m) / phi (rad) |
|---|---|---:|---:|---|---|
| len | m | 4.38094139e-05 | 12393 | 1.32932833 / -2.92932833 | 0.0645000041 / -0.800000191 |
| phi | rad | 0.000659286976 | 12393 | 1.32932833 / -2.92932833 | 0.0645000041 / -0.800000191 |
| J0 | m/rad | 1.45696104e-05 | 12393 | 1.32932833 / -2.92932833 | 0.0645000041 / -0.800000191 |
| J1 | m/rad | 1.45770609e-05 | 12393 | 1.32932833 / -2.92932833 | 0.0645000041 / -0.800000191 |
| J2 | dimensionless | 0.000546127558 | 12393 | 1.32932833 / -2.92932833 | 0.0645000041 / -0.800000191 |
| J3 | dimensionless | 0.000545501709 | 12393 | 1.32932833 / -2.92932833 | 0.0645000041 / -0.800000191 |
| JT_inv0 | 1/m | 0.0230369568 | 12393 | 1.32932833 / -2.92932833 | 0.0645000041 / -0.800000191 |
| JT_inv1 | 1/m | 0.00983142853 | 12393 | 1.32932833 / -2.92932833 | 0.0645000041 / -0.800000191 |
| JT_inv2 | dimensionless | 1.66893005e-06 | 9627 | -2.80180412 / -3.45819588 | 0.458399981 / -3.13000011 |
| JT_inv3 | dimensionless | 7.51018524e-06 | 11013 | 0.966971426 / -4.10697143 | 0.0469499975 / -1.57000017 |
| velocity_len | m/s | 1.60299242e-05 | 12393 | 1.32932833 / -2.92932833 | 0.0645000041 / -0.800000191 |
| velocity_phi | rad/s | 0.000600486994 | 12393 | 1.32932833 / -2.92932833 | 0.0645000041 / -0.800000191 |
| torque1 | Nm | 0.00246548653 | 12393 | 1.32932833 / -2.92932833 | 0.0645000041 / -0.800000191 |
| torque4 | Nm | 0.00246453285 | 12393 | 1.32932833 / -2.92932833 | 0.0645000041 / -0.800000191 |
| force_len | N | 0.0547027588 | 12393 | 1.32932833 / -2.92932833 | 0.0645000041 / -0.800000191 |
| force_phi | Nm | 1.39474869e-05 | 9687 | -0.163028574 / -5.23697143 | 0.0469499975 / -2.70000005 |
| spring_len | m | 1.16229057e-06 | 12393 | 1.32932833 / -2.92932833 | 0.0645000041 / -0.800000191 |
| dls_dlen | m/m | 7.05830753e-05 | 12393 | 1.32932833 / -2.92932833 | 0.0645000041 / -0.800000191 |
| fs | N | 0.0176458359 | 12393 | 1.32932833 / -2.92932833 | 0.0645000041 / -0.800000191 |

### turns

Samples 15470; both valid 15470; validity differences 0.

| Quantity | Unit | Max absolute difference | Sample | phi1 / phi4 (rad) | len (m) / phi (rad) |
|---|---|---:|---:|---|---|
| len | m | 4.97922301e-05 | 12397 | 1.32932833 / -9.21251364 | 0.0645000041 / -0.800000191 |
| phi | rad | 0.000749409199 | 12397 | 1.32932833 / -9.21251364 | 0.0645000041 / -0.800000191 |
| J0 | m/rad | 1.65477395e-05 | 12397 | 1.32932833 / -9.21251364 | 0.0645000041 / -0.800000191 |
| J1 | m/rad | 1.65551901e-05 | 12397 | 1.32932833 / -9.21251364 | 0.0645000041 / -0.800000191 |
| J2 | dimensionless | 0.000620305538 | 12397 | 1.32932833 / -9.21251364 | 0.0645000041 / -0.800000191 |
| J3 | dimensionless | 0.000620305538 | 12397 | 1.32932833 / -9.21251364 | 0.0645000041 / -0.800000191 |
| JT_inv0 | 1/m | 0.0261793137 | 12397 | 1.32932833 / -9.21251364 | 0.0645000041 / -0.800000191 |
| JT_inv1 | 1/m | 0.0111780167 | 12397 | 1.32932833 / -9.21251364 | 0.0645000041 / -0.800000191 |
| JT_inv2 | dimensionless | 4.529953e-06 | 11015 | -5.31621388 / -4.10697143 | 0.0469499975 / -1.57000017 |
| JT_inv3 | dimensionless | 9.05990601e-06 | 11008 | 7.33398368 / -4.19079837 | 0.0449999981 / -1.57000017 |
| velocity_len | m/s | 1.82054937e-05 | 12397 | 1.32932833 / -9.21251364 | 0.0645000041 / -0.800000191 |
| velocity_phi | rad/s | 0.000682339072 | 12397 | 1.32932833 / -9.21251364 | 0.0645000041 / -0.800000191 |
| torque1 | Nm | 0.00280022621 | 12397 | 1.32932833 / -9.21251364 | 0.0645000041 / -0.800000191 |
| torque4 | Nm | 0.00280070305 | 12397 | 1.32932833 / -9.21251364 | 0.0645000041 / -0.800000191 |
| force_len | N | 0.0621566772 | 12397 | 1.32932833 / -9.21251364 | 0.0645000041 / -0.800000191 |
| force_phi | Nm | 1.8119812e-05 | 12341 | -4.54621388 / -3.33697143 | 0.0469499864 / -0.799999714 |
| spring_len | m | 1.31130219e-06 | 12397 | 1.32932833 / -9.21251364 | 0.0645000041 / -0.800000191 |
| dls_dlen | m/m | 8.02241266e-05 | 12397 | 1.32932833 / -9.21251364 | 0.0645000041 / -0.800000191 |
| fs | N | 0.0200562477 | 12397 | 1.32932833 / -9.21251364 | 0.0645000041 / -0.800000191 |

### boundary

Samples 60; both valid 7; validity differences 0.

| Quantity | Unit | Max absolute difference | Sample | phi1 / phi4 (rad) | len (m) / phi (rad) |
|---|---|---:|---:|---|---|
| len | m | 2.98023224e-08 | 24300 | 1.005 / 0.995 | 0.479994923 / 1 |
| phi | rad | 1.56760216e-05 | 24293 | -0.9995 / -1.0005 | 0.47999993 / -1 |
| J0 | m/rad | 1.12340786e-07 | 24297 | -3.13659265 / -3.14659265 | 0.479994923 / 3.14159274 |
| J1 | m/rad | 9.29139787e-08 | 24295 | 1.0005 / 0.9995 | 0.47999993 / 1 |
| J2 | dimensionless | 0.015681535 | 24293 | -0.9995 / -1.0005 | 0.47999993 / -1 |
| J3 | dimensionless | 0.0157519579 | 24293 | -0.9995 / -1.0005 | 0.47999993 / -1 |
| JT_inv0 | 1/m | 157.98291 | 24293 | -0.9995 / -1.0005 | 0.47999993 / -1 |
| JT_inv1 | 1/m | 159.36084 | 24295 | 1.0005 / 0.9995 | 0.47999993 / 1 |
| JT_inv2 | dimensionless | 6.83069229e-05 | 24295 | 1.0005 / 0.9995 | 0.47999993 / 1 |
| JT_inv3 | dimensionless | 0.000101685524 | 24293 | -0.9995 / -1.0005 | 0.47999993 / -1 |
| velocity_len | m/s | 1.11787813e-07 | 24295 | 1.0005 / 0.9995 | 0.47999993 / 1 |
| velocity_phi | rad/s | 0.0172778517 | 24293 | -0.9995 / -1.0005 | 0.47999993 / -1 |
| torque1 | Nm | 0.0360730886 | 24293 | -0.9995 / -1.0005 | 0.47999993 / -1 |
| torque4 | Nm | 0.0362346172 | 24293 | -0.9995 / -1.0005 | 0.47999993 / -1 |
| force_len | N | 231.742188 | 24293 | -0.9995 / -1.0005 | 0.47999993 / -1 |
| force_phi | Nm | 0.000324964523 | 24295 | 1.0005 / 0.9995 | 0.47999993 / 1 |
| spring_len | m | 1.1920929e-07 | 24300 | 1.005 / 0.995 | 0.479994923 / 1 |
| dls_dlen | m/m | 0.0106258392 | 24300 | 1.005 / 0.995 | 0.479994923 / 1 |
| fs | N | 2.65649414 | 24300 | 1.005 / 0.995 | 0.479994923 / 1 |

### chart

Samples 35; both valid 16; validity differences 0.

| Quantity | Unit | Max absolute difference | Sample | phi1 / phi4 (rad) | len (m) / phi (rad) |
|---|---|---:|---:|---|---|
| len | m | 0.00134187937 | 24339 | 0.147737911 / -1.45226209 | 0.359899223 / -0.652262092 |
| phi | rad | 0.0221494436 | 24353 | 1.44662401 / -2.95337599 | 0.060167674 / -0.753376007 |
| J0 | m/rad | 0.000411426648 | 24353 | 1.44662401 / -2.95337599 | 0.060167674 / -0.753376007 |
| J1 | m/rad | 0.000411499292 | 24353 | 1.44662401 / -2.95337599 | 0.060167674 / -0.753376007 |
| J2 | dimensionless | 0.0185120106 | 24353 | 1.44662401 / -2.95337599 | 0.060167674 / -0.753376007 |
| J3 | dimensionless | 0.0185106397 | 24353 | 1.44662401 / -2.95337599 | 0.060167674 / -0.753376007 |
| JT_inv0 | 1/m | 2.11456299 | 24355 | 2.69604935 / -3.10395065 | 0.0410057306 / -0.203950644 |
| JT_inv1 | 1/m | 2.01474762 | 24355 | 2.69604935 / -3.10395065 | 0.0410057306 / -0.203950644 |
| JT_inv2 | dimensionless | 1.54972076e-06 | 24327 | 0.0309931884 / -0.369006812 | 0.471914619 / -0.16900681 |
| JT_inv3 | dimensionless | 4.17232513e-06 | 24361 | 2.69624935 / -3.10375065 | 0.0410057306 / -0.20375061 |
| velocity_len | m/s | 0.000452596694 | 24353 | 1.44662401 / -2.95337599 | 0.060167674 / -0.753376007 |
| velocity_phi | rad/s | 0.0203626603 | 24353 | 1.44662401 / -2.95337599 | 0.060167674 / -0.753376007 |
| torque1 | Nm | 0.0767261982 | 24353 | 1.44662401 / -2.95337599 | 0.060167674 / -0.753376007 |
| torque4 | Nm | 0.07672894 | 24353 | 1.44662401 / -2.95337599 | 0.060167674 / -0.753376007 |
| force_len | N | 3.13006592 | 24355 | 2.69604935 / -3.10395065 | 0.0410057306 / -0.203950644 |
| force_phi | Nm | 1.06096268e-05 | 24361 | 2.69624935 / -3.10375065 | 0.0410057306 / -0.20375061 |
| spring_len | m | 0.000416338444 | 24339 | 0.147737911 / -1.45226209 | 0.359899223 / -0.652262092 |
| dls_dlen | m/m | 0.00278390944 | 24355 | 2.69604935 / -3.10395065 | 0.0410057306 / -0.203950644 |
| fs | N | 0.695976257 | 24355 | 2.69604935 / -3.10395065 | 0.0410057306 / -0.203950644 |

### cross_pi

Samples 5; both valid 5; validity differences 0.

| Quantity | Unit | Max absolute difference | Sample | phi1 / phi4 (rad) | len (m) / phi (rad) |
|---|---|---:|---:|---|---|
| len | m | 2.98023224e-08 | 24365 | 4.54259265 / 1.74259265 | 0.180913582 / -3.14059305 |
| phi | rad | 4.76837158e-07 | 24366 | 4.54359265 / 1.74359265 | 0.180913582 / -3.13959312 |
| J0 | m/rad | 4.47034836e-08 | 24362 | 4.53959265 / 1.73959265 | 0.180913538 / 3.13959265 |
| J1 | m/rad | 2.98023224e-08 | 24362 | 4.53959265 / 1.73959265 | 0.180913538 / 3.13959265 |
| J2 | dimensionless | 2.68220901e-07 | 24365 | 4.54259265 / 1.74259265 | 0.180913582 / -3.14059305 |
| J3 | dimensionless | 1.1920929e-07 | 24365 | 4.54259265 / 1.74259265 | 0.180913582 / -3.14059305 |
| JT_inv0 | 1/m | 7.15255737e-07 | 24364 | 4.54159265 / 1.74159265 | 0.180913568 / 3.1415925 |
| JT_inv1 | 1/m | 2.14576721e-06 | 24365 | 4.54259265 / 1.74259265 | 0.180913582 / -3.14059305 |
| JT_inv2 | dimensionless | 1.1920929e-07 | 24366 | 4.54359265 / 1.74359265 | 0.180913582 / -3.13959312 |
| JT_inv3 | dimensionless | 3.57627869e-07 | 24362 | 4.53959265 / 1.73959265 | 0.180913538 / 3.13959265 |
| velocity_len | m/s | 2.98023224e-08 | 24362 | 4.53959265 / 1.73959265 | 0.180913538 / 3.13959265 |
| velocity_phi | rad/s | 2.38418579e-07 | 24365 | 4.54259265 / 1.74259265 | 0.180913582 / -3.14059305 |
| torque1 | Nm | 3.81469727e-06 | 24362 | 4.53959265 / 1.73959265 | 0.180913538 / 3.13959265 |
| torque4 | Nm | 2.86102295e-06 | 24362 | 4.53959265 / 1.73959265 | 0.180913538 / 3.13959265 |
| force_len | N | 1.90734863e-06 | 24365 | 4.54259265 / 1.74259265 | 0.180913582 / -3.14059305 |
| force_phi | Nm | 5.96046448e-07 | 24362 | 4.53959265 / 1.73959265 | 0.180913538 / 3.13959265 |
| spring_len | m | 1.49011612e-08 | 24364 | 4.54159265 / 1.74159265 | 0.180913568 / 3.1415925 |
| dls_dlen | m/m | 4.47034836e-08 | 24365 | 4.54259265 / 1.74259265 | 0.180913582 / -3.14059305 |
| fs | N | 1.14440918e-05 | 24365 | 4.54259265 / 1.74259265 | 0.180913582 / -3.14059305 |

### invalid

Samples 3; both valid 0; validity differences 0.


## Independent checks

Center differences evaluate actual new production resolve() on both sides, exclude
invalid stencils and wrap angular differences. The non-singular motion grid cannot
cross the coincident-link assembly boundary at these step sizes. Report errors
separately for length/angle Jacobian rows. Shrinking epsilon eventually amplifies
float roundoff; no runtime differentiation was introduced.

| Check | Maximum error | Sample | phi1 / phi4 |
|---|---:|---:|---|
| diff_.002_0_len | 3.66121531e-05 | 5257 | 3.68665624 / 1.71334386 |
| diff_.002_0_phi | 8.29696655e-05 | 4291 | 4.00167322 / 1.39832699 |
| diff_.002_1_len | 3.25590372e-05 | 20985 | 4.22790457 / 2.03209543 |
| diff_.002_1_phi | 8.29696655e-05 | 3803 | 4.1793623 / 1.2206378 |
| diff_.001_0_len | 3.97115946e-05 | 10461 | -1.69042859 / -3.70957141 |
| diff_.001_0_phi | 0.000274688005 | 11073 | 0.53093958 / -3.67093958 |
| diff_.001_1_len | 4.8071146e-05 | 5039 | 3.75760365 / 1.64239633 |
| diff_.001_1_phi | 0.000274688005 | 11043 | 0.696254317 / -3.83625432 |
| diff_.0005_0_len | 0.000127524137 | 4916 | 3.79726076 / 1.60273933 |
| diff_.0005_0_phi | 0.000274688005 | 4782 | 3.8404007 / 1.5595994 |
| diff_.0005_1_len | 0.000127524137 | 4916 | 3.79726076 / 1.60273933 |
| diff_.0005_1_phi | 0.000274688005 | 4780 | 3.84104514 / 1.55895495 |
| roundtrip_force | 7.62939453e-06 | 2 | 2.27896309 / -0.678963125 |
| roundtrip_torque | 7.15255737e-07 | 0 | 2.27976155 / -0.679761469 |
| power | 9.53674316e-07 | 2 | 2.27896309 / -0.678963125 |
| identity | 5.96046448e-08 | 30 | 2.2678678 / -0.667867839 |

Units: derivative length row m/rad, angle row rad/rad; roundtrip_force N,
roundtrip_torque Nm, power W, identity dimensionless.
Integer-turn invariance: zero failures (2e-6 m/rad tolerances).
Across +/-pi, unwrapped phi increases by 0.001 rad per sample without a half-turn
or whole-turn jump. Every sample additionally tests a failed solve after a valid
one; spring validity and mapping outputs do not retain old values.

## Interpretation and limits

Original 5703 working poses meet explicit acceptance bounds. Expanded short-leg
poses near the old coordinate chart and near-straight singularities show larger
float differences and are NOT claimed equivalent to the same tight tolerances.
In particular near full extension, inverse-force differences exceed 200 N for
the stated input torque. This is an ill-conditioned feedback mapping, not a
reason to mix those values into normal-range statistics or retune the spring.
Double-precision coordinate-vs-direct spot checks are in double_diagnostic.json;
they support floating-point cancellation as the cause at the selected extreme
poses, but do not replace the actual production-float comparison.
No validity differences found in these sampled sets; the mismatch CSV is header-only.
No physical validation performed.

H723 full build attempted: fails at pre-existing app/robot.cpp references to
missing Device debug members. All modified control translation units compile.
Independent production VMC executable passes its assertions; full image is NOT
reported as passing. Current pnx_devices/pnx_libs local edits were preserved.

Current build status: H723 Debug now compiles and links after removing application references to the uncommitted Device debug API. robot_debug.motors exposes const pointers to the existing four dmmotor objects and two wheel motor base objects. Watch ->fdb, ->alive, ->motor_state and ->cmd; DM also exposes ->raw, ->enablefailed and ->last_tx_id. registered/starting/start_status remain application-owned. No joint_bus/debug TX-error counters remain; alive counts parsed feedback, not every CAN frame. Device files unchanged by this replacement. Output defaults false; no firmware flashed. This supersedes earlier missing-debug build-block notes.
