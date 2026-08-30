# Historical mecanum motion-accuracy experiments

> This document preserves measurements from the retired passive-roller
> architecture and the experiments that led to the current implementation. The
> active repository now has one contact path: four single-cylinder wheel
> collisions with direction-dependent friction. Runtime selection and the old
> experimental tuning interfaces no longer exist.
>
> The canonical implementation was subsequently validated manually through
> simulation, mecanum driving, odometry/EKF, SLAM mapping, map saving, saved-map
> loading, and AMCL localization. Phase 1 is closed. Autonomous Nav2 goal
> navigation is not claimed.

## Scope and immutable baseline

This document records the physical-contact calibration performed after the
measurement-only diagnostic commit `e715832`. The earlier Phase 2 implementation
at `e514a54` and Phase 1 baseline at `ae5f8bb` remain unchanged in history.
Gazebo ground truth is used only by the evaluation tools; it does not feed the
controller, odometry, EKF, TF, SLAM, or AMCL.

The baseline established that requested wheel rates, wheel ordering, signs, and
steady-state tracking were correct. Nevertheless, lateral and several diagonal
motions had 11--21% ground-truth chassis error. That is diagnostic Case B:
correct wheel actuation with inaccurate roller/ground contact motion.

## Canonical rotational calibration

After the anisotropic single-cylinder contact implementation became canonical,
fresh clockwise and counter-clockwise low-speed rotation runs exposed a
symmetric rotational scale error. With the literal wheel-center projection of
`0.142 m`, ground truth rotated `1.1340956` times as far as raw wheel odometry.
Steady-state wheel-rate error was below `1.9e-9` normalized, while unintended
translation stayed below `0.6 um`, excluding wheel order, signs, tracking, and
contact asymmetry. Filtered odometry already followed the IMU and ground truth,
so the EKF was not the source.

The directional cylinder approximation therefore uses its measured effective
rotational projection, `0.142 / 1.1340956 = 0.12521 m`, in the controller's
inverse and forward kinematics. No friction, radius, gain, EKF, SLAM, AMCL, or
TF parameter changed.

Verification, one repetition per direction:

| Direction | ground truth / raw odometry yaw | Residual scale error |
| --- | ---: | ---: |
| Counter-clockwise (`diagnostic_low_rotate_positive`) | `0.99848` | `0.152%` |
| Clockwise (`diagnostic_low_rotate_negative`) | `1.00088` | `0.088%` |

The ratio was `1.1340956` before calibration, an `11.824%` mismatch. It is now
within `0.16%` of unity in both directions, and the symmetry across directions
is what rules out a sign or handedness artifact rather than the magnitude alone.
Run-to-run spread at one repetition is comparable to the residual itself, so
treat these as "no detectable scale error" rather than as a precision figure.

Translation is unaffected, as expected: the projection enters only the
rotational term, and the wheel radius is unchanged. Measured in the same
campaign, all within threshold:

| Profile | Diagnostic error | Endpoint error |
| --- | ---: | ---: |
| `diagnostic_low_forward` | `0.455%` | `1.14 mm` |
| `diagnostic_low_left` | `0.629%` | `1.57 mm` |
| `diagnostic_low_forward_right` | `0.335%` | `0.84 mm` |

Both rotation profiles still classify as **case D**, not `within_threshold`:
full-run wheel RMSE exceeds the 10% threshold because error concentrates in the
command-start transient, while steady-state tracking and chassis motion are
accurate. This is the same transient behavior recorded for rotations under the
earlier roller model and is not a rotational defect.

### The calibrated projection is model-coupled, not geometry

`0.12521 m` is not a property of the robot. The wheel centers really are at
`x=+/-0.075` and `y=+/-0.067`, giving a literal projection of `0.142 m`. The
calibrated value compensates for the directional-contact approximation, so it is
only valid for that contact model. Re-measure it if `mu`, `mu2`, `slip1`, the
wheel cylinder geometry, or the wheel positions change.

**On hardware the calibrated value would be wrong.** It corrects a simulation
artifact that a physical base does not have. Hardware commissioning should start
from the geometric `0.142 m` and derive its own effective value from measured
rotation, exactly as this section did for the simulation.

Generated campaign data remains ignored under `phase1_results/final_fix/`.
The measurements below remain useful engineering evidence, but their passive
roller configuration and comparison controls are no longer active source.

## Geometry and symmetry audit

The audit verified all four wheel positions, the `0.03074443 m` effective
radius, ten explicit rollers per wheel, 45-degree axes, mecanum X handedness,
wheel-specific measured phases and mirrored axial offsets, preserved `0.12 kg`
wheel-assembly mass, positive inertias, and the four controller joint names and
order. The driven hubs have no collision, so all ground contact goes through
the 40 passive roller links. No kinematic, ordering, axis, handedness, or mass
defect was found.

The original straight cylinders did not represent the visibly tapered CAD
roller envelope. Depending on roller phase, the support radius changed by
about `0.56 mm`; the settled chassis also showed small roll and pitch, consistent
with phase-dependent support and load transfer. This non-circular contact
envelope was the dominant physical limitation.

## Controlled experiments

Each candidate used an identical spawn, isolated ROS domain and Gazebo
partition, fresh simulator process, and the low-speed left, right,
forward-right, and backward-left subset. A candidate had to complete with
populated ground-truth samples; `ros2 launch` exit status alone was not treated
as proof of a valid campaign.

| Parameter family | Candidates | Decision |
| --- | --- | --- |
| Roller phase | measured phases, all-zero, handed pairs | Keep measured phases; alternatives produced 28--43% aggregate error. |
| Joint damping | `0`, `0.00025`, `0.0005`, `0.001`, `0.002` | Use `0`; independently reduced focused mean error from 15.87% to 14.00%. |
| Joint friction | `0`, `0.0000125`, `0.000025`, `0.00005`, `0.0001` | Use `0`; higher passive bearing resistance worsened contact motion. |
| Contact friction | `0.4`, `0.6`, `0.8`, `1.0`, `1.2` | **Superseded — this comparison was invalid.** The friction values never reached the physics engine, so all five candidates were the same run. See [Contact parameters never reached the engine](#contact-parameters-never-reached-the-engine). |
| Physics step | `1.0`, `0.5`, `0.25 ms` | Keep `1.0 ms`; smaller steps were slower and increased focused mean error to 16.78% and 20.74%. |
| Controller limiting | installed Jazzy controller inspected | No acceleration/deceleration limiter exists in this controller version, so no unsupported parameter was added. |
| Collision shape | cylinder, dynamic mesh attempts, primitive barrel | Use the primitive barrel; DART did not produce usable dynamic contact from the attempted triangle/convex mesh path. |

Zero damping plus zero joint friction with the old cylinders improved focused
mean error to 13.40%, but did not solve Case B. The accepted primitive barrel
reduced it to 2.59%, with all four focused primitives below 3.78%.

## Historical passive-roller model

The final model retains 40 explicit passive roller links and continuous joints.
Each roller uses nine overlapping sphere collisions along its local axis to
approximate the measured tapered envelope:

| Axial station (m) | Sphere radius (m) |
| ---: | ---: |
| `0` | `0.00569443` |
| `±0.0030` | `0.00558` |
| `±0.0058` | `0.00526` |
| `±0.0082` | `0.00480` |
| `±0.0108` | `0.00405` |

This is a supported primitive approximation, not a fake mecanum plugin, direct
chassis velocity, planar joint, teleportation, or ground-truth correction. It
preserves the nominal external dimensions, roller count, joints, inertia, CAD
visuals, and physical mecanum mechanism.

| Parameter | Before | After |
| --- | ---: | ---: |
| Roller collision | straight cylinder | nine-sphere tapered barrel |
| Passive joint damping | `0.001` | `0.0` |
| Passive joint friction | `0.00005` | `0.0` |
| Isotropic contact `mu1=mu2` | `0.8` | `0.8` |
| Physics maximum step | `0.001 s` | `0.001 s` |
| Wheel phases and axial offsets | measured wheel-specific values | unchanged |
| Controller wheel radius / projection sum | `0.03074443 / 0.142 m` | unchanged |

At the time of this campaign, launch and generation tools exposed calibration
values for reproducing rejected candidates. Those controls and the comparison
geometry have since been retired.

## Ground-truth results

The nominal full campaign completed all ten primitives. The percentage is the
conservative diagnostic error: the larger normalized translation/yaw chassis
error. Endpoint error is the ideal-command versus Gazebo position error.

| Primitive | Baseline error | Calibrated nominal error | Endpoint error | Yaw drift |
| --- | ---: | ---: | ---: | ---: |
| Forward 1 m | 3.57% | 1.33% | 13.4 mm | `+0.0081 rad/m` |
| Backward 1 m | 8.42% | 0.72% | 4.3 mm | `-0.0072 rad/m` |
| Left strafe 1 m | 13.75% | 4.66% | 32.5 mm | `-0.0475 rad/m` |
| Right strafe 1 m | 18.66% | 4.86% | 31.6 mm | `+0.0494 rad/m` |
| Positive 90 degrees | 1.24% | 0.84% | 5.0 mm | `+0.139 deg` angle error |
| Negative 90 degrees | 1.00% | 0.97% | 4.9 mm | `-0.138 deg` angle error |
| Forward-left diagonal | 6.63% | 1.80% | 18.2 mm | `+0.0018 rad/m` |
| Forward-right diagonal | 12.24% | 2.73% | 27.4 mm | `+0.0290 rad/m` |
| Backward-left diagonal | 18.74% | 3.37% | 16.3 mm | `-0.0356 rad/m` |
| Backward-right diagonal | 11.19% | 3.68% | 37.2 mm | `+0.0013 rad/m` |

Mean calibrated nominal diagnostic error is 2.50%; the maximum is 4.86%.
Low-speed translation and diagonal tests range from 0.74% to 2.64% and all
complete within the physical threshold. The two short low-speed rotations are
Case D only because a 60--64 ms command-start transient raises full-run wheel
RMSE to about 12.1%; their steady-state wheel RMSE is below `1e-9` normalized
and chassis error is only 0.18--0.23%. This is not the original Case B defect.

Steady-state wheel tracking remains effectively exact in every nominal run
(maximum normalized steady RMSE below `8.3e-10`). Full-run nominal wheel RMSE
is 6.98--9.76%, reflecting bounded command transitions rather than sustained
tracking error.

## Long accumulation and closed path

| Profile | Ground-truth distance | Diagnostic error | Endpoint error | Cross drift | Yaw drift/m |
| --- | ---: | ---: | ---: | ---: | ---: |
| Forward | 5.006 m | 1.99% | 100.4 mm | 93.4 mm | `+0.00805` |
| Left strafe | 5.038 m | 4.75% | 241.0 mm | -227.1 mm | `-0.01909` |
| Forward-right diagonal | 5.007 m | 8.09% | 408.7 mm | -402.2 mm | `-0.03289` |

All three long runs complete below the 10% Case B threshold with approximately
1000 samples each. The remaining error still accumulates with distance,
especially for one diagonal; the simulation is substantially improved, not
perfect.

The corrected 1 m-per-side square run travels `4.030 m`, closes physically
within `14.4 mm` and `0.00282 rad`, and has `8.94 mm` raw-odometry endpoint
error. Multi-segment profiles now report explicit loop-closure metrics and mark
the constant-command A/B/C/D classifier not applicable; the earlier apparent
one-metre square diagnostic was a reporting error, not physical motion.

## Odometry, EKF, TF, mapping, and performance regression

Raw odometry remains independent of ground truth and agrees with the calibrated
physical motion to 3.6--36.0 mm position error across the nominal primitives.
The full simulator-backed repository test exercises the unchanged EKF and
validates its filtered output. In fused mode the controller has
`enable_odom_tf=false`, the EKF has `publish_tf=true`, and robot state publisher
owns the link and sensor transforms.

Headless `navigation_basic` mapping completed a bounded mixed mecanum sequence
covering forward/reverse, both strafes, diagonals, and both rotation directions.
SLAM Toolbox and both controllers were active; SLAM was the sole `/map`
publisher; the observed tree was
`map -> odom -> base_footprint -> base_link -> sensors`; and a fresh
`86 x 159` map at `0.05 m/pixel` saved successfully. The same corridor-safe
mixed sequence completed in `navigation_narrow` without a SLAM, controller, or
EKF process failure. A final narrow map-save query and saved-map AMCL rerun were
not executed in this session because the external execution allowance was
exhausted; they are therefore not claimed as completed evidence here.

The physics timestep is unchanged. The accepted contact approximation raises
one roller from one collision to nine (360 primitive roller collisions total).
Measured real-time factor, including campaign orchestration overhead, averaged
about `0.88` for the nominal campaign and `0.82--0.89` for the 5 m runs on the
development machine.

## Contact parameters never reached the engine

The friction row in the table above compared five identical simulations.

`gz sdf -p` does not implement the Gazebo-Classic `<gazebo reference>` friction
vocabulary. The `mu1`, `mu2`, `kp`, and `kd` authored on all 40 roller links
were therefore discarded during URDF-to-SDF conversion, and the generated model
that Gazebo actually spawned contained:

| In the generated SDF | Before the fix | After the fix |
| --- | ---: | ---: |
| Roller collisions | 360 | 360 |
| ...carrying a `<surface>` | **0** | **360** |
| `<mu>` / `<mu2>` / `<kp>` / `<kd>` anywhere | **0** | 360 each |

Confirmed for both `barrel` and `cylinder`, so it was not an artifact of the
nine-sphere collision naming. Every roller contact ran on the sdformat default
`mu` of `1.0`, never the configured `0.8`. `generate_sim_sdf.py` now injects a
real SDF `<surface>` into every roller collision and refuses to emit a model
whose rollers lack one, so the failure cannot recur silently.

### Corrected friction sweep

Re-run on the focused low-speed subset with the parameter genuinely applied,
one repetition per profile, isolated `ROS_DOMAIN_ID` and `GZ_PARTITION` per
candidate, each verified to have produced four completed runs:

| `roller_contact_mu` | Focused mean diagnostic error | Maximum |
| ---: | ---: | ---: |
| `0.4` | 3.09% | 3.76% |
| `0.6` | 2.99% | 3.73% |
| `0.8` | 2.92% | 3.63% |
| `1.0` | 2.89% | 3.72% |
| `1.2` | 2.88% | 3.69% |

The response is monotonic across all five candidates, which is the evidence
that the parameter is live; a random ordering of five values would occur about
1.7% of the time. Response saturates above `1.0`, so `1.0` is now the default.
It is also the value the simulation ran at unintentionally, which keeps the
earlier recorded results comparable with later ones.

Friction is nonetheless a weak lever here: `0.21` percentage points across a
threefold range. It is not the dominant term in the residual error.

## Parasitic strafe yaw is roller-phase asymmetry

The residual strafe signature is a yaw drift of equal magnitude and opposite
sign in the two directions. Two candidate mechanisms were tested.

**Support-radius ripple — rejected as the driver.** The ten-roller ring makes
the effective support radius vary between `29.518` and `30.744 mm`, a
`1.226 mm` peak-to-peak ripple, `3.99%` of the radius, while the controller
assumes a constant `30.744 mm`. The ripple is real and worth recording, but
modelling its contribution to yaw gives about `-0.0002 rad/m` against a
measured `-0.035 rad/m` — roughly 170 times too small to be the cause.

**Inter-wheel phase asymmetry — confirmed.** The four wheels carry different
measured phases spanning `0.197` to `0.480 rad`, which is 45% of one
`0.628 rad` roller pitch, so their contact events never cancel. Running all
four wheels at one shared phase, with every other property unchanged:

| Case | Left strafe yaw | Right strafe yaw | Error (left / right) |
| --- | ---: | ---: | ---: |
| Measured phases (shipped) | `-0.03575 rad/m` | `+0.03893 rad/m` | 3.48% / 3.81% |
| Uniform phase `0.25` | `+0.00007 rad/m` | `-0.00020 rad/m` | 2.93% / 2.58% |

A roughly 180-fold reduction. Phase asymmetry, not contact quality, produces
the parasitic strafe yaw.

**The measured phases are kept.** They describe the physical wheels, and a real
robot whose rollers sit at fixed relative phases would show the same bias for a
given strafe. What the simulation reports is one sample: on hardware the phase
relationship at the start of any run is effectively arbitrary, so the bias
varies between runs and averages toward zero, while the simulation fixes one
draw and reports it deterministically. Treat the strafe yaw as a roller-phase
artifact of a single configuration, not as a contact defect, and do not retune
friction, damping, or physics step trying to remove it.

## Early rejected attempt: anisotropic cylinder wheel contact

A single cylinder per wheel with a handed `fdir1` friction cone at 45 degrees
was implemented as an opt-in fourth of the contact cost (4 contacts instead of
360). It generated valid SDF and passed static checks, but did not drive.

With traction high across the roller axis the chassis moved backwards; with the
corrected assignment — high along the roller axis, low across it, since a
roller rolls perpendicular to its own axis and resists sliding along it — the
direction became correct but the base oscillated in place, reaching at most
12% of a `0.25 m` strafe target with along-track displacement alternating
between `+0.031` and `-0.028 m`. That is contact instability rather than a
tuning offset, so no `mu`/`mu2` sweep was pursued and the change was reverted
rather than shipped in a non-working state.

This result was later traced to contact-delivery details rather than a failure of
the approximation itself. The working implementation injects the surface after
URDF-to-SDF conversion, writes the literal `gz:expressed_in` attribute expected
by DART, and references the surviving `base_footprint` SDF link. With the
validated handed direction mapping, that implementation became the canonical
contact path. The failed attempt remains documented here to show why those
seemingly small generation details are mandatory.

## Validation commands

```bash
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose

ros2 launch mobile_base_bringup odometry_evaluation.launch.py \
  test_profile:=all repetitions:=1 \
  evaluation_mode:=raw_only localization:=false \
  output_dir:=phase1_results/calibrated_nominal

ros2 launch mobile_base_bringup odometry_evaluation.launch.py \
  profile_sequence:=long_forward_5m,long_strafe_left_5m,long_diagonal_forward_right_5m \
  repetitions:=1 evaluation_mode:=raw_only localization:=false \
  output_dir:=phase1_results/calibrated_long
```

The complete visual mapping, map-save, and AMCL procedure for both maintained
worlds is in the main README.
