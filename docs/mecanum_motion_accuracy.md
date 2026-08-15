# Mecanum motion-accuracy diagnosis

## Scope and baseline

This is a measurement-only diagnosis of the simulated mecanum drivetrain. The
preserved Phase 2 baseline is commit
`e514a54de05430699203a05f46d845adb576021f`. No wheel radius, rotational
geometry, roller/contact, controller, physics, EKF, SLAM, or AMCL parameter was
changed during the campaign.

The tests ran each primitive in a fresh Gazebo process and an isolated ROS
domain from the same spawn pose. Runtime evidence is retained locally under:

- `phase1_results/mecanum_low_speed_isolated/`
- `phase1_results/mecanum_low_speed_rotation_fixed/`
- `phase1_results/mecanum_nominal_isolated/`

Those generated results remain ignored runtime artifacts. The standard
campaign launcher correctly refused to run after its mandatory current-boot
AMDGPU safety trigger was detected, so the measurements did not weaken or
bypass that safety gate. Isolation was instead enforced with a fresh ROS
domain, Gazebo partition, and simulator process for every primitive.

## Verification chain

The requested body twist was converted to expected FL, FR, RR, and RL wheel
rates using the Jazzy `mecanum_drive_controller` convention. Those rates were
compared with `/joint_states` and controller-state feedback. Gazebo pose was
used only for evaluation of chassis motion and was never connected to control,
odometry, the EKF, mapping, localization, or TF.

Static description tests verify the wheel joint names and positions, positive
joint axes, alternating X-pattern roller handedness, roller axis geometry, and
front/rear and left/right construction. Runtime results verify the wheel order,
sign combinations, and positive rotation convention: every steady-state wheel
rate matched its expected magnitude and sign to numerical precision.

## Results

The percentages below normalize endpoint error by commanded translation or
rotation. Wheel transient error is full-run velocity RMSE normalized by peak
expected wheel rate; steady wheel error uses the second half of the samples.

| Primitive | Low-speed chassis error | Nominal chassis error | Low / nominal steady wheel error | Classification |
| --- | ---: | ---: | ---: | --- |
| Forward | 3.20% | 3.57% | <0.00001% / <0.00001% | within threshold |
| Backward | 5.34% | 8.42% | <0.00001% / <0.00001% | within threshold |
| Left strafe | 13.45% | 13.75% | <0.00001% / <0.00001% | Case B |
| Right strafe | 17.28% | 18.66% | <0.00001% / <0.00001% | Case B |
| Forward-left | 7.75% | 6.63% | <0.00001% / <0.00001% | within threshold |
| Forward-right | 12.08% | 12.24% | <0.00001% / <0.00001% | Case B |
| Backward-left | 20.68% | 18.74% | <0.00001% / <0.00001% | Case B |
| Backward-right | 11.80% | 11.19% | <0.00001% / <0.00001% | Case B |
| Positive rotation | 0.68% | 1.24% | <0.00001% / <0.00001% | transient D / within threshold |
| Negative rotation | 0.76% | 1.00% | <0.00001% / <0.00001% | transient D / within threshold |

Additional directional evidence:

- Nominal lateral yaw drift was `-0.150 rad/m` left and `+0.199 rad/m`
  right; low-speed values were `-0.148 rad/m` and `+0.187 rad/m`.
- Nominal translation drift during rotation was `0.00315 m/rad` positive and
  `0.00422 m/rad` negative. Low-speed values were `0.00413 m/rad` and
  `0.00587 m/rad`.
- Low-speed full-run wheel RMSE was about 8.5--12.1%, while nominal RMSE was
  about 6.8--9.8%. Rise and settling were about one sample interval and the
  second-half wheel error was effectively zero. The short low-speed rotations
  therefore receive Case D from the conservative 10% transient threshold, but
  their chassis error stays below 1%.
- Lateral and diagonal normalized errors remain similar at low and nominal
  velocity. Absolute error grows with travel distance, while this dataset does
  not show a dominant velocity-dependent or sustained acceleration-dependent
  wheel-tracking error.
- Directional asymmetry, especially left versus right strafe and opposing
  diagonals, is consistent with roller transition/contact behavior rather than
  a uniform wheel-radius scale error.

## Root cause and calibration decision

The dominant result is **Case B**: the wheels do what the controller requests,
but the Gazebo chassis does not realize the ideal lateral and diagonal motion.
The raw wheel odometry error follows the physical chassis discrepancy, while
forward motion and rotation remain comparatively accurate. This rules against
Case A as the dominant cause and provides no evidence for Case C or a uniform
effective-radius correction.

The most likely error source is anisotropic roller/ground contact and slip,
including direction-dependent roller transitions and simulator contact
fidelity. Case D is visible only as a bounded command-start transient in the
short rotation tests and is not the source of the accumulated mapping
deformation.

No calibration change is made. A controlled contact-physics experiment would
be the next stage, changing only one parameter family at a time and accepting
a candidate only if it reduces lateral and diagonal error without degrading
forward or rotational motion. The clean Phase 2 checkpoint and these baseline
measurements must remain the comparison reference.
