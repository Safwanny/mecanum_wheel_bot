# Mobile Base

Modular ROS 2 Jazzy model and Gazebo simulation for a four-wheel mecanum platform.

## Packages

| Package | Responsibility |
| --- | --- |
| `mobile_base_description` | Xacro model, SI-unit geometry, inertial data, and RViz configuration |
| `mobile_base_gazebo` | Gazebo Harmonic world and simulator-specific assets |
| `mobile_base_bringup` | ros2_control configuration and top-level launch files |
| `mobile_base_localization` | Planar EKF, SLAM Toolbox mapping, Nav2 map serving, and holonomic AMCL |
| `mobile_base_evaluation` | Evaluation-only, identity-selected Gazebo ground truth |
| `mobile_base_tools` | Odometry-path visualization and repeatable motion checks |

The packages are independent of the existing arm stack.

## Project status

**Status: Phase 1 and the Phase 2 mapping/localization stack are implemented.**
Phase 1 delivered the mecanum simulation and control
stack, raw wheel odometry, fused wheel/IMU odometry, explicit TF ownership,
timestamp and TF contract validation, repeatable motion evaluation, and an
isolated per-repetition campaign lifecycle with bounded cleanup and resumable,
non-overwriting results.

The completed raw and fused campaigns support these engineering conclusions:

- raw and fused odometry validation are complete across the motion profiles;
- fusion substantially improves yaw accuracy and cross-axis drift;
- endpoint position performance is broadly similar overall;
- square-path closure improves with fusion;
- startup outliers remain in the primary results rather than being removed or
  replaced; and
- no controller, EKF, geometry, trajectory, sensor-rate, or evaluation-threshold
  tuning was required to close Phase 1.

Phase 2 adds SLAM Toolbox mapping and saved-map AMCL localization. The physical
roller contact model has also been calibrated against Gazebo ground truth: the
original 11--21% lateral/diagonal Case B errors are now 0.72--4.86% across the
nominal primitive campaign. Full results, rejected experiments, long-run
limitations, and reproduction commands are in
[`docs/mecanum_motion_accuracy.md`](docs/mecanum_motion_accuracy.md). Full Nav2
planning and autonomous navigation remain later work.

## Phase 2: mapping and saved-map localization

Phase 2 provides two deliberately separate operating modes. They are mutually
exclusive: stop one mode before starting the other.

```text
Mapping Mode:
  /scan + odom -> base_footprint -> slam_toolbox -> /map + map -> odom

Localization Mode:
  saved YAML/PGM -> map_server -> /map
  /scan + odom -> base_footprint -> AMCL -> /amcl_pose + map -> odom
```

TF ownership is fixed in both modes:

| Transform | Mapping Mode owner | Localization Mode owner |
| --- | --- | --- |
| `map -> odom` | `slam_toolbox` only | AMCL only |
| `odom -> base_footprint` | Phase 1 `ekf_filter_node` | Phase 1 `ekf_filter_node` |
| `base_footprint -> base_link -> sensors` | `robot_state_publisher` | `robot_state_publisher` |

Never co-launch `mobile_base_bringup mapping.launch.py` and
`mobile_base_bringup localization.launch.py`. Doing so would create competing
`map -> odom` publishers. Neither mode changes the validated controller,
mecanum kinematics, EKF tuning, URDF geometry, or local odometry topics.

### Supported worlds and saved-map names

The world passed to the launch file and the saved-map basename must describe
the same environment. Keep every YAML beside its referenced image file.

| Gazebo world | Intended use | Saved-map pair |
| --- | --- | --- |
| `navigation_basic` | Primary Phase 2 room and obstacle test | `maps/mobile_base/navigation_basic.{yaml,pgm}` |
| `navigation_narrow` | Narrow corridors, turns, and close-wall scan matching | `maps/mobile_base/navigation_narrow.{yaml,pgm}` |
| `empty` | Motion and controller smoke tests; too little structure for useful localization | No maintained Phase 2 map |
| `sensor_test` | Camera, LiDAR, and IMU validation | No maintained Phase 2 map |

Generated maps and pose graphs live under `$HOME/ros2_ws/maps/mobile_base/`.
That runtime directory is ignored by Git. Saving a map with an existing
basename replaces that local YAML/PGM pair.

```text
ros2_ws/maps/mobile_base/
├── navigation_basic.yaml
├── navigation_basic.pgm
├── navigation_narrow.yaml      # after mapping navigation_narrow
└── navigation_narrow.pgm       # after mapping navigation_narrow
```

### One-time setup

Install dependencies and build from a clean shell:

```bash
cd "$HOME/ros2_ws"
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/mobile_base --ignore-src -r -y
colcon build --symlink-install --packages-up-to \
  mobile_base_bringup mobile_base_localization mobile_base_tools
source "$HOME/ros2_ws/install/setup.bash"
```

Open a fresh terminal after rebuilding. In every terminal below, source the
Jazzy underlay first and this workspace overlay second. Do not use a setup file
from another workspace.

### End-to-end visual test: `navigation_basic`

The following procedure starts with no running simulation, creates and saves a
map, reloads it, initializes AMCL, and visually verifies localization. Keep the
terminal numbering: commands in different terminals run concurrently.

Before Terminal 1, stop any previous mapping, localization, teleop, Gazebo, or
RViz launch with `Ctrl-C` in the terminal that owns it. Wait for its windows to
close. The following read-only check should produce no old Phase 2 processes:

```bash
pgrep -af 'mapping.launch.py|localization.launch.py|gz sim|slam_toolbox|amcl'
```

#### 1. Start Mapping Mode (Terminal 1)

```bash
cd "$HOME/ros2_ws"
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

ros2 launch mobile_base_bringup mapping.launch.py \
  world:=navigation_basic \
  rviz:=true \
  gui:=true \
  render_engine:=ogre2
```

Leave Terminal 1 running. Exactly one Gazebo window and the blue Mapping Mode
RViz window should open. RViz uses fixed frame `map` and should show the robot,
TF, LaserScan, filtered odometry/trajectory, and a live occupancy map. The map
may be small until the robot moves.

#### 2. Drive while mapping (Terminal 2)

```bash
cd "$HOME/ros2_ws"
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=true \
  -p frame_id:=base_link \
  -p speed:=0.3 \
  -p turn:=0.5 \
  -p use_sim_time:=true \
  -r cmd_vel:=/mobile_base_controller/reference
```

Keep Terminal 2 focused. `i` and `,` drive forward and backward; `j` and `l`
rotate; uppercase `J` and `L` strafe; uppercase `U`, `O`, `M`, and `>` drive
diagonally. Press `k`, Space, or any unmapped key to stop. Avoid `q`, `z`, `w`,
`x`, `e`, and `c` unless deliberately changing the velocity limits.

Drive slowly around every obstacle, rotate to observe all wall directions, and
return to a previously mapped area so SLAM Toolbox can close loops. A useful
visual result has crisp single walls rather than duplicated or smeared walls,
scan points on obstacle boundaries, and no large unexplored holes in reachable
areas. Keyboard teleoperation provides no collision avoidance.

Optional live checks from a third terminal are:

```bash
cd "$HOME/ros2_ws"
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

ros2 lifecycle get /slam_toolbox
ros2 topic hz /scan
ros2 topic hz /odometry/filtered
ros2 topic hz /map
ros2 run tf2_ros tf2_echo map base_footprint
```

`/slam_toolbox` must be `active [3]`; `/amcl` must not exist in Mapping Mode.

#### 3. Stop the robot and save the map (Terminal 3)

Press `k` in Terminal 2, but keep both Mapping Mode and Gazebo running. Then run:

```bash
cd "$HOME/ros2_ws"
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

mkdir -p "$HOME/ros2_ws/maps/mobile_base"

ros2 run nav2_map_server map_saver_cli \
  -t /map \
  -f "$HOME/ros2_ws/src/mobile_base/maps/mobile_base/navigation_basic" \
  --ros-args \
  -p map_subscribe_transient_local:=true \
  -p save_map_timeout:=10.0
```

Do not stop Terminal 1 before this command prints `Map saved successfully`.
Warnings that unspecified occupied/free thresholds use defaults are normal.
Verify both files before leaving Mapping Mode:

```bash
ls -lh "$HOME/ros2_ws/maps/mobile_base/navigation_basic.yaml" \
  "$HOME/ros2_ws/maps/mobile_base/navigation_basic.pgm"
sed -n '1,20p' \
  "$HOME/ros2_ws/maps/mobile_base/navigation_basic.yaml"
```

The YAML `image:` entry should name `navigation_basic.pgm`. Optionally preserve
the SLAM pose graph for continued mapping:

```bash
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '${HOME}/ros2_ws/maps/mobile_base/navigation_basic.posegraph'}"
```

#### 4. Stop Mapping Mode

After the YAML and PGM exist, press `Ctrl-C` in Terminal 2 and then Terminal 1.
Wait for Gazebo and RViz to close. Do not start Localization Mode while
`/slam_toolbox` or an old Gazebo server is still running.

#### 5. Reload the map in Localization Mode (Terminal 4)

```bash
cd "$HOME/ros2_ws"
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

ros2 launch mobile_base_bringup localization.launch.py \
  world:=navigation_basic \
  map:="$HOME/ros2_ws/maps/mobile_base/navigation_basic.yaml" \
  rviz:=true \
  gui:=true \
  render_engine:=ogre2
```

Leave Terminal 4 running. Gazebo and the green Localization Mode RViz window
should open with the saved map already visible. Until AMCL receives an initial
pose, RViz may report missing `map -> odom`, drop `odom` or `lidar_link`
messages, and print `Please set the initial pose`; this is the expected waiting
state.

#### 6. Initialize AMCL in RViz

In RViz, select **2D Pose Estimate** from the toolbar or Tools panel. Click the
robot's approximate position on the saved map, drag the arrow in its forward
direction, and release. The simulation respawns at the original world pose, so
the original mapping start position is the best first estimate.

Within a few seconds, the robot model and scan should appear in the map frame,
the scan should align with saved walls, the AMCL particles should contract
around the robot, and the initial-pose warnings should stop. If the scan is
offset or rotated, set the initial pose again more accurately.

#### 7. Drive while localizing (Terminal 5)

```bash
cd "$HOME/ros2_ws"
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=true \
  -p frame_id:=base_link \
  -p speed:=0.3 \
  -p turn:=0.5 \
  -p use_sim_time:=true \
  -r cmd_vel:=/mobile_base_controller/reference
```

Exercise forward/reverse motion, rotation, both strafes, and diagonals. A
successful visual test keeps the robot and scan aligned with the saved map,
keeps the particle cloud centered near the robot, and reduces covariance after
motion provides useful scan observations.

#### 8. Verify AMCL from the CLI (Terminal 6)

```bash
cd "$HOME/ros2_ws"
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 topic echo /amcl_pose --once
ros2 topic echo /particle_cloud --once
ros2 run tf2_ros tf2_echo map base_footprint
```

Both lifecycle nodes must report `active [3]`. `/amcl`, `/map_server`,
`/amcl_pose`, and `/particle_cloud` must exist; `/slam_toolbox` must not exist.

#### 9. Stop Localization Mode

Press `k` and then `Ctrl-C` in Terminal 5. Press `Ctrl-C` in Terminal 4 and wait
for Gazebo and RViz to close.

### Repeat the complete test for `navigation_narrow`

Use the same terminal order and teleop command. Change only the world and map
basename in the mapping, save, and localization commands:

```bash
# Mapping Mode
ros2 launch mobile_base_bringup mapping.launch.py \
  world:=navigation_narrow \
  rviz:=true gui:=true render_engine:=ogre2

# Save while Mapping Mode is still running
ros2 run nav2_map_server map_saver_cli \
  -t /map \
  -f "$HOME/ros2_ws/maps/mobile_base/navigation_narrow" \
  --ros-args \
  -p map_subscribe_transient_local:=true \
  -p save_map_timeout:=10.0

# After stopping Mapping Mode, start Localization Mode
ros2 launch mobile_base_bringup localization.launch.py \
  world:=navigation_narrow \
  map:="$HOME/ros2_ws/maps/mobile_base/navigation_narrow.yaml" \
  rviz:=true gui:=true render_engine:=ogre2
```

The narrow-world visual test should pay particular attention to parallel wall
alignment, duplicated corridor edges, scan matching during turns, lateral
motion near walls, and particle convergence after emerging from a corridor.

### Hardware-only mapping and localization

For an already-running hardware/base stack that supplies `/scan`,
`/odometry/filtered`, and the local TF chain, launch only the sensor-agnostic
mapping subsystem with wall time:

```bash
ros2 launch mobile_base_localization mapping.launch.py \
  use_sim_time:=false
```

After saving a hardware map, stop mapping and launch only map serving and AMCL:

```bash
ros2 launch mobile_base_localization amcl.launch.py \
  use_sim_time:=false \
  map:=/absolute/path/to/the_saved_map.yaml
```

Hardware commissioning requires measured sensor ranges, TF timing, motion
noise, and safe velocity limits; the simulation defaults are not hardware
acceptance criteria.

### Phase 2 visual troubleshooting

| Symptom | Cause and action |
| --- | --- |
| World resolves under `$HOME/install` instead of `$HOME/ros2_ws/install` | The shell contains a stale overlay. Open a fresh terminal and source `/opt/ros/jazzy/setup.bash`, then `$HOME/ros2_ws/install/setup.bash`. |
| Two Gazebo or RViz windows appear | More than one launch is alive or the workspace was not rebuilt after a launch-file change. Stop every old launch, rebuild, source the overlay, and start one mode once. |
| `Detected jump back in time` repeats | Multiple Gazebo servers are publishing the same Gazebo world clock. Stop all old simulator sessions before restarting. |
| Gazebo is visible but the robot does not move | Mapping and localization do not drive automatically. Run stamped keyboard teleop and keep its terminal focused. |
| Map is smeared or walls are duplicated | Motion was too fast, scan matching was poor, or multiple clocks existed. Remap at about `0.3 m/s`, rotate slowly, and revisit known areas. |
| `map_saver_cli` reports `Failed to spin map subscription` | Mapping Mode was stopped too early or `/map` is unavailable. Keep Terminal 1 running, confirm `ros2 topic echo /map --once`, and retry the documented durable save command. |
| Localization says `Please set the initial pose` | This is expected before initialization. Use RViz **2D Pose Estimate** and align the arrow with the robot heading. |
| Scan and map do not align after initialization | The initial position or yaw is wrong, or the wrong map/world pair was loaded. Set the pose again and verify matching basenames. |
| Robot collides with obstacles | Phase 2 has no planner, collision avoidance, velocity smoother, or command mux. The teleoperator must stop and steer safely. |

### Phase 2 validation coverage

Mapping validation covers live map updates, all holonomic directions, loop
closure, YAML/PGM saving, and map reload. Localization validation covers map
reload, RViz initial-pose setting, particle convergence, deliberately offset
pose recovery, holonomic motion, covariance behavior, and restart/reload.

### Quantitative localization evaluation

Gazebo ground truth remains evaluation-only. For a quantitative localization
run, start the existing identity-based selector for the chosen world and record
it beside AMCL and TF:

```bash
ros2 run mobile_base_evaluation ground_truth_selector --ros-args \
  -p world:=navigation_basic \
  -p gz_topic:=/world/navigation_basic/dynamic_pose/info \
  -p robot_entity:=mobile_base \
  -p output_topic:=/mobile_base/evaluation/ground_truth
ros2 bag record /mobile_base/evaluation/ground_truth /amcl_pose /tf /tf_static
```

Only compare poses after aligning the saved map and Gazebo world origins.
Measure planar position error, yaw error, RMSE, maximum position error, and
convergence time from the recording; no acceptance thresholds are asserted
until measurements establish a baseline.

### Phase 2 tuning and limitations

Defaults match the repository's 10 Hz, 0.10-4.0 m simulated LaserScan, 50 Hz
filtered odometry, `base_footprint` frame, small robot dimensions, and
conservative 0.05 m map resolution. Hardware commissioning must remeasure and
tune laser min/max range, scan and TF timing, SLAM travel/update and loop
closure thresholds, AMCL `alpha1`-`alpha5` (especially lateral `alpha5`),
particle counts, update thresholds, and transform tolerance.

Generated maps and pose graphs are runtime artifacts and are not committed.
Continued mapping from a serialized graph is optional and not automatically
launched. The simulated GPU LiDAR requires the wrappers' default Ogre2 sensor
renderer; an Ogre1 validation run pinned all 720 beams to the 0.10 m minimum
and cannot produce a usable map. Phase 2 does not add planners, controller
servers, behavior trees, goal execution, obstacle avoidance, command
arbitration, velocity smoothing, or any other autonomous-navigation component.

## Units and CAD source

URDF and controller values use SI units: metres, kilograms, seconds, and radians.
The original CAD file remains at `src/3D_Builds/ROS2_transfer.stl` and is not loaded
at runtime. The base model uses one parametric body link. The mecanum wheels use four
position-specific STL files for visual appearance and tapered, sphere-primitive
barrels on the passive roller links for collision.

The wheel meshes live in `src/mobile_base/meshes/wheels/` and are installed through
the description package. Gazebo receives runtime-generated `file://` mesh URIs from
the installed `mobile_base_description` share directory.

The four STL files were exported in assembly coordinates, so `base.xacro` gives every
wheel an explicit position-specific visual rotation and translation. Each rigid
transform flips the exported mounting face inward toward the chassis and keeps the
mesh axle centre coincident with the driven-wheel link origin. The flip is composed
about a mesh-local transverse axis aligned with that wheel's measured roller phase;
this preserves the visual roller phase set and mecanum X-pattern. These visual
transforms are independent of physical roller handedness, joints, collisions,
controller configuration, and odometry.

## URDF organization

The main assembly file is
`mobile_base_description/urdf/mobile_base.urdf.xacro`. It should stay readable at the
robot level: arguments, includes, base assembly, camera, ros2_control, and optional
Gazebo plugin/sensor instantiation.

| File | Responsibility |
| --- | --- |
| `properties.xacro` | Shared robot and sensor dimensions, masses, poses, rates, ranges, and noise |
| `materials.xacro` | Named visual materials |
| `inertials.xacro` | Reusable inertial macros |
| `chassis.xacro` | `base_footprint`, `base_link`, and body geometry |
| `macros/mecanum_rollers.xacro` | Recursive physical roller links, passive joints, collisions, and contact parameters |
| `macros/mecanum_wheels.xacro` | Driven hub links, existing wheel visuals, wheel joints, and roller-ring invocation |
| `base.xacro` | Chassis plus four mecanum wheel assembly |
| `camera.xacro` | Front RGB camera link, optical frame, fixed joints, geometry, and inertia |
| `lidar.xacro` / `imu.xacro` | Fixed physical sensor links, joints, geometry, and inertia |
| `mobile_base.ros2_control.xacro` | ros2_control hardware and command/state interfaces |
| `mobile_base.gazebo.xacro` | Gazebo Harmonic ros2_control plugin wrapper |
| `camera.gazebo.xacro` | Gazebo Harmonic camera sensor configuration |
| `lidar.gazebo.xacro` / `imu.gazebo.xacro` | Gazebo sensor, topic, rate, and noise configuration |

## Frames and hierarchy

The model follows REP-103: `+X` forward, `+Y` left, and `+Z` up. Wheelbase is the
front-to-rear wheel-center spacing. Wheel separation is the left-to-right track width.

```text
odom
`-- base_footprint
    `-- base_link
    |-- front_left_wheel_link
    |   `-- front_left_roller_0_link ... front_left_roller_9_link
    |-- front_right_wheel_link
    |   `-- front_right_roller_0_link ... front_right_roller_9_link
    |-- rear_right_wheel_link
    |   `-- rear_right_roller_0_link ... rear_right_roller_9_link
    |-- rear_left_wheel_link
    |   `-- rear_left_roller_0_link ... rear_left_roller_9_link
    |-- lidar_link
    |-- imu_link
    `-- camera_link
        `-- camera_optical_frame
```

Each wheel link owns its internal roller links through the wheel macro. The top-level
assembly intentionally does not expose those roller links.

## Dependencies

Install dependencies after sourcing ROS 2 Jazzy:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

The Gazebo launch requires `ros_gz_sim`, `ros_gz_bridge`, and `gz_ros2_control`.

## Build and inspect

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select \
  mobile_base_description mobile_base_gazebo mobile_base_localization \
  mobile_base_evaluation mobile_base_tools mobile_base_bringup
source install/setup.bash
xacro src/mobile_base/mobile_base_description/urdf/mobile_base.urdf.xacro > /tmp/mobile_base.urdf
check_urdf /tmp/mobile_base.urdf
ros2 launch mobile_base_description display.launch.py
```

This command is only for inspecting the URDF without Gazebo. It uses a headless
joint-state publisher by default so every movable link remains visible. To show the
optional joint sliders, add `use_joint_state_gui:=true` and keep that GUI open.

## Simulate

```bash
ros2 launch mobile_base_bringup simulation.launch.py
```

`world` accepts an installed world name (with or without `.sdf`) or an absolute
SDF path. The launch also exposes `use_sim_time`, `gui`, `rviz`,
`start_controller`, `localization`, `render_engine`, `roller_collision_model`,
`velocity_smoother`, `x`, `y`, `z`, and `yaw`. `roller_collision_model` selects
the motion method and is forwarded by `mapping.launch.py`,
`localization.launch.py`, and `odometry_evaluation.launch.py`.
Calibration experiments additionally expose roller damping, joint friction,
contact friction, collision model, the four roller phases, and physics maximum
step size; normal launches use the validated defaults.
Phase 1 intentionally supports one un-namespaced robot. A misleading partial
`namespace` argument was removed rather than implying multi-robot support.

Wheel appearance comes from the full position-specific mecanum wheel meshes. How
wheel-ground contact is simulated depends on the selected contact model, below.

## Wheel contact models

Two motion methods are available. They are selected with `roller_collision_model`
on any launch file in this README and differ *only* in how wheel-ground contact is
simulated. The controller, kinematics, wheel joint names, odometry, EKF, TF,
sensors, and topics are identical in both.

| | `barrel` (default) | `husarion_cylinder` |
| --- | --- | --- |
| Mechanism | 40 explicit passive roller links | one cylinder per driven hub with handed anisotropic friction |
| Ground contacts | 360 sphere collisions | 4 cylinder collisions |
| Diagnostic error | 2.92% | **0.47%** |
| Worst profile | 3.79% | **0.55%** |
| Strafe yaw drift | `+-0.035 rad/m` | **`+-0.00044 rad/m`** |
| Cross-axis drift | 0.75-2.41 mm | **0.00-0.03 mm** |
| Real-time factor | 0.60-0.80 | **1.00** |
| Coast after last command | 0.21-2.99 mm | **0.00-0.77 mm** |
| Root-cause classification | case B | **within_threshold** |

Figures are one repetition of the four low-speed diagnostic profiles at `0.10 m/s`,
so they rank the models rather than establishing tolerances. Method and rejected
alternatives are in [`docs/mecanum_motion_accuracy.md`](docs/mecanum_motion_accuracy.md).

`barrel` remains the default because it models the physical mechanism: ten measured
rollers per wheel, driven hubs with no collision, so contact cannot bypass the
rollers. `husarion_cylinder` replaces that with a direction-dependent friction cone
following the shipping Husarion ROSbot XL description; it is faster and currently
more accurate, but it is an approximation rather than the mechanism. A third value,
`cylinder`, keeps the straight-cylinder roller geometry for comparison with the
pre-calibration model.

`husarion_cylinder` emits no roller links, so `/joint_states` carries four wheel
joints instead of 44.

### Selecting a model

Append `roller_collision_model:=husarion_cylinder` to any launch. Every command in
this README works with either value.

```bash
# Plain simulation
ros2 launch mobile_base_bringup simulation.launch.py \
  roller_collision_model:=husarion_cylinder

# Mapping (SLAM Toolbox)
ros2 launch mobile_base_bringup mapping.launch.py \
  world:=navigation_basic roller_collision_model:=husarion_cylinder

# Saved-map localization (AMCL)
ros2 launch mobile_base_bringup localization.launch.py \
  world:=navigation_basic roller_collision_model:=husarion_cylinder \
  map:="$HOME/ros2_ws/src/mobile_base/maps/mobile_base/navigation_basic.yaml"

# Odometry evaluation campaign
ros2 launch mobile_base_bringup odometry_evaluation.launch.py \
  roller_collision_model:=husarion_cylinder \
  test_profile:=all repetitions:=1 evaluation_mode:=raw_only localization:=false \
  output_dir:=phase1_results/husarion_nominal
```

Omit the argument, or pass `roller_collision_model:=barrel`, for the roller model:

```bash
ros2 launch mobile_base_bringup mapping.launch.py world:=navigation_basic
```

To compare the two on identical profiles, give each run its own ROS domain and
Gazebo partition and confirm Gazebo has exited in between. `ros2 launch` returns 0
even when a campaign produced nothing, so check the result count rather than the
exit status:

```bash
for MODEL in barrel husarion_cylinder; do
  ROS_DOMAIN_ID=41 GZ_PARTITION="cmp_$MODEL" \
  ros2 launch mobile_base_bringup odometry_evaluation.launch.py \
    roller_collision_model:="$MODEL" \
    profile_sequence:=diagnostic_low_left,diagnostic_low_right,diagnostic_low_forward_right,diagnostic_low_backward_left \
    repetitions:=1 evaluation_mode:=raw_only localization:=false \
    output_dir:="phase1_results/cmp_$MODEL"
  ls "phase1_results/cmp_$MODEL"/run_results/*.json | wc -l   # expect 4
done
```

The `husarion_cylinder` contact parameters are exposed for experiments:
`wheel_contact_mu` (default `0.8`, along the roller axis), `wheel_contact_mu2`
(`0.2`, across it), and `wheel_contact_slip1` (`0.0`). Upstream uses `slip1: 0.035`,
which suits their heavier base; here it cut strafe completion from 95.7% to 67.7%,
so it is disabled by default.

### Which model to use

Use `husarion_cylinder` for mapping, navigation, and anything where real-time factor
or stopping behaviour matters. Use `barrel` when the question is about the physical
roller mechanism itself, or to reproduce earlier recorded results. Both branches of
this repository build and test cleanly, and the generated `barrel` model is
byte-identical to before `husarion_cylinder` was added.

## Explicit passive roller model

Each visual wheel mesh contains ten measured rollers. The physical model matches that
geometry with a `0.02505 m` roller-centre radius, `0.00569443 m` roller radius,
`0.02580 m` roller length, 36-degree spacing, and 45-degree roller inclination. The
effective contact radius remains the controller wheel radius:

```text
0.02505 + 0.00569443 = 0.03074443 m
```

The measured position-specific roller configuration is:

| Wheel | Handedness | Zero-angle phase (rad) | Local wheel-Y offset (m) |
| --- | ---: | ---: | ---: |
| front-left | -1 | 0.22193969 | -0.00059729 |
| front-right | +1 | 0.48030419 | +0.00059729 |
| rear-right | -1 | 0.19668582 | +0.00059729 |
| rear-left | +1 | 0.24790784 | -0.00059729 |

For roller angle `theta`, the wheel axle is `a=(0,1,0)` and the increasing-angle
tangent is `t=(-sin(theta),0,cos(theta))`. The roller axis is
`cos(pi/4)*a + handedness*sin(pi/4)*t`. The roller child frame maps local Z onto this
axis, allowing every passive joint to use a normalized local `axis="0 0 1"` while its
collision envelope and inertia use the same frame.

The original `0.12 kg` wheel assembly mass is preserved rather than duplicated:
the collision-free driven hub is `0.084 kg`, and each of its ten rollers is
`0.0036 kg`. Calibrated passive joint damping and friction are both `0.0`.
Each roller uses isotropic contact friction `mu1=mu2=0.8`, contact stiffness
`100000`, and contact damping `10`. There is no wheel-level `fdir1` or
anisotropic contact approximation.

The collision envelope follows the measured tapered roller shape using nine
overlapping spheres at axial stations `0`, `±3.0`, `±5.8`, `±8.2`, and
`±10.8 mm`, with radii decreasing from `5.69443` to `4.05 mm`. This retains the
40 passive roller joints while avoiding unsupported dynamic triangle-mesh
contact in DART. Set `roller_collision_model:=cylinder` only to reproduce the
pre-calibration comparison model.

Only the four driven wheel joints have velocity command interfaces. All 44 movable
joints—the four wheels and 40 rollers—have position and velocity state interfaces.
Inspect them while the simulation is running:

```bash
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states --once
ros2 topic echo /joint_states --field name --once
```

The STL wheel remains one visual attached to the hub link. Consequently, its rendered
rollers do not visibly spin independently even though the 40 collision-only roller
links rotate and report state. The corrected inner/outer visual face orientation does
not change those physical contacts. The explicit model adds 40 links, 40 joints, 40
passive roller bodies, 360 primitive collision shapes, and 80 state interfaces.
It is more CPU-intensive than the old one-cylinder-per-roller model or a
one-body contact approximation; measured calibrated campaign real-time factor
is about 0.82--0.89 on the development machine.

The recursive roller organization was informed by
[`DaiGuard/fuji_mecanum`](https://github.com/DaiGuard/fuji_mecanum), an MIT-licensed
structural reference. Its ROS 1 controller, Python 2 node, transmissions, meshes,
dimensions, and Gazebo Classic configuration were not ported.

The simulation launch already starts its own robot-state publisher and RViz. Do not
run `display.launch.py` at the same time, because duplicate description and TF
publishers can make the model appear incomplete.

If a launch reports a missing `libgz-transport` library, stop that failed launch with
`Ctrl-C` before retrying. The bringup launch reconstructs the Gazebo vendor environment
automatically, but an older failed launch can leave conflicting ROS nodes behind.

The simulation launch starts Gazebo, publishes `robot_description`, spawns the robot,
loads `joint_state_broadcaster`, activates `mobile_base_controller`, starts the
odometry-to-path utility, and opens RViz. Simulation RViz uses `odom` as its fixed
frame. Retained Odometry arrows and covariance visuals (the old red fan and yellow
bands) were unsuitable for trajectory display, so Odometry now shows only the current
pose and `rviz_default_plugins/Path` draws `/mobile_base/trajectory`
(`nav_msgs/msg/Path`) as one line.

The path node samples `/mobile_base_controller/odometry` after 0.02 m translation or
0.05 rad yaw change, keeps at most 1000 poses, and publishes reliable transient-local
data for late RViz subscribers. Reset it with
`ros2 service call /mobile_base/trajectory/reset std_srvs/srv/Empty {}`. Useful checks:

```bash
ros2 topic type /mobile_base/trajectory
ros2 topic echo /mobile_base/trajectory --once
ros2 topic info -v /mobile_base/trajectory
```

The static URDF display launch still uses the base-fixed RViz configuration.

## Camera

The fixed RGB camera is centered on the front (`+X`) face of `base_link`. In Gazebo it
publishes:

- Image: `/camera/image_raw`
- Camera info: `/camera/camera_info`

Quick checks:

```bash
ros2 topic hz /camera/image_raw
ros2 topic echo --once /camera/camera_info
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

## Simulated LiDAR and IMU

The robot carries a fixed planar GPU LiDAR and a fixed six-axis IMU. Their
dimensions, mass, mounting poses, rates, range, and Gaussian noise values are
centralized in `properties.xacro`. Sensor links remain part of the normal URDF;
their Gazebo sensor elements are enabled only by `use_gazebo:=true`.

| ROS interface | Type | Frame | Nominal rate | Main configuration |
| --- | --- | --- | ---: | --- |
| `/scan` | `sensor_msgs/msg/LaserScan` | `lidar_link` | 10 Hz | 720 samples, 360 degrees, 0.10-4.0 m, 0.01 m stddev |
| `/imu/data` | `sensor_msgs/msg/Imu` | `imu_link` | 50 Hz | 3-axis angular velocity and linear acceleration noise |

Both frames are fixed children of `base_link`. Gazebo publishes native
`gz.msgs.LaserScan` and `gz.msgs.IMU`; the launch bridges them one-way into ROS.
The world supplies the Sensors system with Ogre2 and the IMU system. No
The sensor definitions themselves do not add localization, SLAM, Nav2, or
sensor fusion; the top-level simulation starts the Phase 1 EKF by default.

Four self-contained worlds are installed; none downloads external models:

```bash
ros2 launch mobile_base_bringup simulation.launch.py world:=empty
ros2 launch mobile_base_bringup simulation.launch.py world:=sensor_test
ros2 launch mobile_base_bringup simulation.launch.py world:=navigation_basic
ros2 launch mobile_base_bringup simulation.launch.py world:=navigation_narrow
```

`sensor_test` places a wall at `x=3 m`, a box to the left, and a cylinder to the
right for recognizable scan returns. `navigation_basic` is a room with walls,
routes, boxes, and a column. `navigation_narrow` contains a 1 m corridor,
doorway, L-shaped turn, and dead end. An absolute custom file works too:

```bash
ros2 launch mobile_base_bringup simulation.launch.py \
  world:=/absolute/path/to/custom_world.sdf
```

RViz uses `odom` as its fixed frame and shows the robot, TF frames and axes,
`/scan`, current odometry, trajectory path, and camera image. The IMU pose is
represented by the `imu_link` TF axes; raw angular velocity and acceleration are
checked from the message:

```bash
ros2 topic hz /scan
ros2 topic echo /scan --once
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
ros2 run tf2_ros tf2_echo base_link lidar_link
ros2 run tf2_ros tf2_echo base_link imu_link
```

Expected stationary IMU behavior is a near-identity orientation, near-zero
angular velocity, and approximately `+9.81 m/s^2` along its Z axis, with small
configured noise. During positive yaw rotation, `angular_velocity.z` should be
positive. Scan ranges may contain `inf` where no surface lies inside the
configured maximum range; finite returns must fall between `range_min` and
`range_max`.

If sensor topics are absent, first confirm simulation time is advancing and that
the selected SDF contains both `gz-sim-sensors-system` and
`gz-sim-imu-system`. If Ogre2 cannot initialize on a headless host, select the
bounded Ogre software path and Mesa llvmpipe explicitly:

```bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch mobile_base_bringup simulation.launch.py \
  gui:=false rviz:=false render_engine:=ogre
```

If TF is absent, ensure only this launch owns `robot_state_publisher` and that
`/joint_states` is active. To inspect bridge types and QoS:

```bash
ros2 topic info -v /scan
ros2 topic info -v /imu/data
ros2 topic echo /clock --once
```

The sensor and world contract tests run with:

```bash
colcon test --packages-select \
  mobile_base_description mobile_base_gazebo mobile_base_bringup
colcon test-result --verbose
```

## Phase 1 state estimation

The Phase 1 local-state architecture is:

```text
Gazebo model state ──> identity selector ──> evaluation only

/mobile_base_controller/odometry (wheel twist)
                         +
/imu/data (yaw rate)
                         |
                         v
             robot_localization EKF
                         |
                         v
              /odometry/filtered
              odom -> base_footprint
```

Gazebo ground truth never feeds the controller, EKF, TF, SLAM, or navigation.
`mobile_base_evaluation/ground_truth_selector` subscribes directly to Gazebo
Transport and selects the configured model by name before publishing one
`PoseStamped`. Entity ordering is therefore irrelevant.

In the default `localization:=true` mode, `ekf_filter_node` is the sole
publisher of `odom -> base_footprint`; the mecanum controller still publishes
raw odometry but has `enable_odom_tf:=false`. `robot_state_publisher` owns
`base_footprint -> base_link` and all link/sensor transforms. In
`localization:=false` diagnostic mode, the EKF is absent and the controller
owns `odom -> base_footprint`. Both authorities are never enabled together.

The EKF uses planar mode and fuses wheel-odometry body-frame `vx`, `vy`, and
yaw rate with IMU yaw rate. It deliberately excludes wheel pose, IMU
orientation, linear acceleration, Z, roll, pitch, and Gazebo ground truth.
The controller covariance is constant and direction-independent, with lower
confidence assigned to mecanum lateral velocity than forward velocity.
Covariance expresses uncertainty; it does not change physical wheel slip.

Simulation with EKF:

```bash
ros2 launch mobile_base_bringup simulation.launch.py \
  world:=sensor_test localization:=true rviz:=true
```

Raw-controller diagnostic mode:

```bash
ros2 launch mobile_base_bringup simulation.launch.py \
  world:=empty localization:=false rviz:=true
```

## Phase 1 runtime contracts

With a fused simulation running, validate the configured IMU (50 Hz), LiDAR
(10 Hz), controller odometry (100 Hz), and filtered odometry (50 Hz) contracts:

```bash
ros2 run mobile_base_tools timestamp_validator \
  --mode fused --duration 10 \
  --output phase1_results/diagnostics/timestamp_fused.json \
  --ros-args -p use_sim_time:=true
ros2 run mobile_base_tools tf_validator \
  --mode fused --duration 10 \
  --output phase1_results/diagnostics/tf_fused.json \
  --ros-args -p use_sim_time:=true
```

Use `--mode raw` for `localization:=false`. Timestamp thresholds live in
`mobile_base_tools/config/timestamp_contracts.yaml`; they enforce sample count,
nonzero finite monotonic stamps, duplicate policy, minimum rate, maximum gap,
message age, and finite payload values. Both validators write JSON, print a
short PASS/FAIL summary, return nonzero on failure, and use a wall-time bound so
a paused simulation clock cannot hang validation.

The TF validator checks the complete Phase 1 frame tree, required sensor and
wheel connections, advancing dynamic transforms, sane values and stamps, no
loops or multiple parents, no `map -> odom`, no evaluation-only frames, and the
single-owner rule. In fused mode the EKF must publish TF while the controller's
`enable_odom_tf` is false; raw mode requires the inverse.

## Raw and filtered odometry evaluation

`mobile_base_tools` compares Gazebo ground truth, raw controller odometry, and
`/odometry/filtered`. It samples the latest fresh pose from all three sources,
records source timestamps, and rejects excessive skew or stale data.

The simulated LiDAR maximum range is `4.0 m`. Its `/scan` topic, `lidar_link`
frame, 720 samples, 360-degree field of view, 10 Hz update rate, minimum range,
resolution, and noise remain unchanged.

Build and source the workspace:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash
```

Run one forward test once:

```bash
ros2 launch mobile_base_bringup odometry_evaluation.launch.py \
  test_profile:=forward_1m repetitions:=1 \
  output_dir:=phase1_results/smoke_test
```

Run all 11 profiles with the default five repetitions:

```bash
ros2 launch mobile_base_bringup odometry_evaluation.launch.py \
  test_profile:=all repetitions:=0 \
  output_dir:=phase1_results/fused_comparison
```

Preserve a controller-only baseline separately:

```bash
ros2 launch mobile_base_bringup odometry_evaluation.launch.py \
  test_profile:=all repetitions:=0 \
  evaluation_mode:=raw_only localization:=false \
  output_dir:=phase1_results/raw_baseline
```

For the formal raw and fused fresh-process campaign, use a new empty output
directory:

```bash
ros2 run mobile_base_tools evaluation_campaign \
  --modes raw_only,raw_and_filtered --repetitions 5 \
  --output-dir phase1_results/formal_campaign
```

Every campaign repetition launches in its own process session. The lifecycle
tracks descendants, applies bounded `SIGINT -> SIGTERM -> SIGKILL` cleanup,
reaps children, requires zero remaining campaign processes, and waits for the
ROS graph to settle before another repetition starts. The campaign manifest is
updated atomically after every profile launch. An interrupted campaign is
marked `interrupted`; `--resume` skips completed cases while retaining every
prior attempt and launch log. Without `--resume`, the tool refuses to write
into a non-empty directory.

Use an explicit writable output directory for retained evidence; the fallback
is `/tmp/mobile_base_phase1`, never a hardcoded workspace source path. The
launch defaults to the flat `empty` world, `evaluation_mode:=raw_and_filtered`,
`localization:=true`, `gui:=false`, and `rviz:=false`. For a preserved raw-only
baseline, use `evaluation_mode:=raw_only localization:=false`. Other arguments
are `world`, `config_file`, `robot_entity`, `shutdown_on_complete`, `gui`, and
`rviz`. Profiles and safe velocities are defined in
`mobile_base_tools/config/odometry_tests.yaml`; `repetitions:=0` uses the
five-repetition YAML value.

Complete evaluation output is generated inside the ignored
`phase1_results/` directory. Campaign manifests, summaries, trajectories,
and plots remain local evidence and are intentionally excluded from source
commits. Copy or back up that directory separately when evidence must be
retained across machines.

Formal Phase 1 runtime validation is complete: all raw and fused profile runs
were collected with the isolated lifecycle, and the retained interrupted
attempt and startup outliers remain part of the evidence. The generated
campaign data stays local under `phase1_results/`; the conclusions relevant to
the maintained software baseline are summarized in "Phase 1 status" above.

Each repetition publishes zero velocity, verifies measured velocity is below
threshold, resets the named Gazebo model through `/world/empty/set_pose`,
verifies the identity-selected pose reaches configurable position/yaw
tolerances, settles, and records fresh relative initial poses. Single and
multi-segment runs have explicit `profile_timeout`; square segments additionally
have `segment_timeout`. All failure paths send repeated zero commands.

The selected output directory contains:

```text
summary.json
summary.csv
runs.csv
trajectories/<profile>_run_<number>.csv
plots/<profile>_run_<number>.png
plots/position_error_comparison.png
plots/yaw_error_comparison.png
plots/cross_axis_drift_comparison.png
plots/repeatability_spread.png
plots/final_displacement_comparison.png
plots/square_path_closure.png
plots/raw_vs_filtered_position_error.png
plots/raw_vs_filtered_yaw_error.png
plots/raw_vs_filtered_path_length_error.png
plots/filtered_repeatability_spread.png
plots/motion_end_vs_settled_overshoot.png
plots/per_profile_improvement.png
```

Position error is the Euclidean difference between final odometry and Gazebo
displacements. Yaw error compares normalized heading changes.
Cross-axis drift is unintended lateral motion during forward/backward tests,
unintended longitudinal motion during strafing, or cross-track error during
diagonal motion. Path-length error compares accumulated odometry and Gazebo
path lengths. Mean, median, minimum, maximum, and standard deviation quantify
repeatability across successful repetitions; failed runs remain listed.
Motion-end poses are captured when the nonzero command stops, while
settled-final poses are captured after repeated zero commands and settling.
Improvement values are raw error minus filtered error, so positive is better.
The report labels filtered position performance as better, similar, or worse.

Gazebo model pose is evaluation-only. Never use it as robot odometry or as
input to localization, sensor fusion, SLAM, navigation, or TF publication.

## Measurement-only mecanum motion diagnostics

The odometry evaluator also records the complete drivetrain chain without
changing any physical or controller parameter. The preserved Phase 2 baseline
for this campaign is commit
`e514a54de05430699203a05f46d845adb576021f`.

Each trajectory CSV records the requested body twist, the body reference
accepted by `mecanum_drive_controller`, expected FL/FR/RR/RL wheel rates,
controller-reported and `/joint_states` wheel rates, wheel positions, raw and
filtered odometry twist, IMU yaw rate, pose-derived Gazebo body velocity, and
simulation timestamps. Per-run JSON adds wheel RMSE, steady-state error, peak
error, rise time, overshoot, settling time, ideal-versus-Gazebo chassis
metrics, odometry-versus-Gazebo metrics, and the evidence classification.
Gazebo data remains evaluation-only and is never connected to control, EKF,
SLAM, AMCL, navigation, or TF.

First run all ten short, low-speed primitives from isolated fresh simulations:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run mobile_base_tools evaluation_campaign \
  --profiles diagnostic_low_forward,diagnostic_low_backward,diagnostic_low_left,diagnostic_low_right,diagnostic_low_rotate_positive,diagnostic_low_rotate_negative,diagnostic_low_forward_left,diagnostic_low_forward_right,diagnostic_low_backward_left,diagnostic_low_backward_right \
  --modes raw_and_filtered --repetitions 1 \
  --output-dir phase1_results/mecanum_low_speed_baseline
```

Then run the ten nominal primitives, also with one fresh simulation each:

```bash
ros2 run mobile_base_tools evaluation_campaign \
  --profiles forward_1m,backward_1m,strafe_left_1m,strafe_right_1m,rotate_positive_90deg,rotate_negative_90deg,diagonal_forward_left,diagonal_forward_right,diagonal_backward_left,diagonal_backward_right \
  --modes raw_and_filtered --repetitions 1 \
  --output-dir phase1_results/mecanum_nominal_baseline
```

Inspect `campaign_summary.json`, then each isolated repetition's
`summary.json`, `run_results/*.json`, `trajectories/*.csv`, plots, and
`launch.log`. Root-cause labels use a documented 10 percent threshold:

- Case A: expected wheel rates and measured wheel rates diverge.
- Case B: wheel tracking is good but Gazebo chassis motion diverges.
- Case C: Gazebo chassis motion is good but wheel odometry diverges.
- Case D: wheel error is concentrated in acceleration/deceleration transients.
- `within_threshold`: none of those stages exceeds the threshold.

The baseline findings and calibration decision are recorded in
[`docs/mecanum_motion_accuracy.md`](docs/mecanum_motion_accuracy.md).

Acceleration/deceleration behavior remains visible in the time series and the
rise/overshoot/settling metrics. Do not tune radius, rotational geometry,
roller/contact parameters, acceleration limits, or physics settings until the
primitive evidence identifies a dominant case.

Troubleshooting:

- No ground-truth pose: confirm the `empty` world is running and
  `ground_truth_selector` reports that it selected `mobile_base` from
  `/world/empty/dynamic_pose/info`.
- Wrong robot entity: keep `robot_entity:=mobile_base`, matching the simulation
  spawn name, and use the deterministic world with one dynamic model.
- Controller inactive: check
  `ros2 control list_controllers`; `mobile_base_controller` must be active.
- Robot does not stop: inspect competing publishers on
  `/mobile_base_controller/reference`; the evaluator aborts if measured twist
  remains above its stop threshold.
- Output directory not writable: choose an absolute writable `output_dir`.
- Plots unavailable: install the `python3-matplotlib` rosdep and use the
  non-interactive backend configured by the evaluator.
- Stale simulation time: verify `ros2 topic echo /clock --once`; all evaluation
  nodes use simulation time.
- Timeout before completing motion: check for collisions, inactive control, or
  stale Gazebo pose data before increasing a YAML timeout.
- Reset service unavailable: verify `/world/empty/set_pose` is present and has
  type `ros_gz_interfaces/srv/SetEntityPose`.
- Missing samples in a report: inspect zero, stale, non-finite, or
  nonmonotonic timestamps in the evaluator log.

The EKF does not correct the physical trajectory or eliminate wheel slip. IMU
yaw rate principally improves angular-state observability and orientation
consistency; planar position may still drift because no absolute XY position
sensor is fused. Phase 1 provides no autonomous navigation. Phase 2 adds only
SLAM Toolbox mapping and saved-map localization; autonomous navigation remains
out of scope.

## Teleoperation

The Jazzy `mecanum_drive_controller` accepts stamped velocity commands on
`/mobile_base_controller/reference`.

Install runtime dependencies with rosdep:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

Run keyboard teleop in a separate terminal because it needs direct keyboard focus:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=true \
  -p frame_id:=base_link \
  -p speed:=0.15 \
  -p turn:=0.5 \
  -p use_sim_time:=true \
  -r cmd_vel:=/mobile_base_controller/reference
```

Use `i`/`,` for forward/reverse. Lowercase `j`/`l` rotate the robot in place.
Do not use lowercase `u`/`o`/`m`/`.` for mecanum diagonals; those keys intentionally
combine forward/backward motion with rotation.

For holonomic movement, hold Shift:

| Key | Motion | Expected powered diagonal pair |
| --- | --- | --- |
| `J` | Strafe left | all wheels |
| `L` | Strafe right | all wheels |
| `U` | Forward-left | front-right and rear-left |
| `O` | Forward-right | front-left and rear-right |
| `M` | Backward-left | front-left and rear-right, opposite direction |
| `>` | Backward-right | front-right and rear-left, opposite direction |

Any unmapped key sends a stop command; `Ctrl-C` exits the teleop node. For hardware,
keep the same stamped command path but set `use_sim_time:=false`.

`teleop_twist_keyboard` publishes one message per keypress from a blocking read.
There is no key-release event, so letting go of a key does not stop the robot:
the last command stands until the controller's `reference_timeout` of `0.5 s`
expires, which is about `7.5 cm` of further travel at the `0.15 m/s` default.
Press an unmapped key such as `k` to stop deliberately rather than releasing.

An opt-in `nav2_velocity_smoother` stage exists to ramp commands instead of
stepping them (`velocity_smoother:=true`, then publish to `/mobile_base/cmd_vel`
rather than the controller reference). It is **off by default and unverified**:
acceleration limiting works, but deceleration behaviour on command cessation is an
open question recorded in `mobile_base_bringup/config/velocity_smoother.yaml`.
It also does not shorten the `0.5 s` above.

The teleop `frame_id` is `base_link` because the controller command is a body-frame
twist. Odometry TF is published as `odom -> base_footprint`, while the URDF keeps the
fixed `base_footprint -> base_link` transform.

Forward:

```bash
ros2 topic pub /mobile_base_controller/reference geometry_msgs/msg/TwistStamped "{header: {frame_id: base_link}, twist: {linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {z: 0.0}}}"
```

Strafe:

```bash
ros2 topic pub /mobile_base_controller/reference geometry_msgs/msg/TwistStamped "{header: {frame_id: base_link}, twist: {linear: {x: 0.0, y: 0.2, z: 0.0}, angular: {z: 0.0}}}"
```

Rotate:

```bash
ros2 topic pub /mobile_base_controller/reference geometry_msgs/msg/TwistStamped "{header: {frame_id: base_link}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.5}}}"
```

The controller timeout remains `reference_timeout: 0.5`, so the robot stops if fresh
commands stop arriving. The installed Jazzy `mecanum_drive_controller` parameter
schema does not expose body velocity or acceleration limiter fields for `linear.x`,
`linear.y`, or `angular.z`; conservative teleop defaults are therefore documented
above, and hardware motion limits still need calibration before physical testing.

No Gazebo velocity bridge, mux, smoother, or external trajectory server is included in
this patch. Future Nav2, SLAM, localization, joystick, or autonomy sources should feed
a mux/safety layer first, then publish to `/mobile_base_controller/reference`. Future
localization owns `map -> odom`; the base controller owns only `odom -> base_footprint`.

## Open-loop motion test

With the simulation running, execute the repeatable mecanum visual check in another
terminal:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run mobile_base_tools mecanum_motion_test --ros-args -p use_sim_time:=true
```

It publishes stamped body-frame commands to `/mobile_base_controller/reference` and
runs a 1 m square, a four-direction diagonal diamond, then +90/-180/+90 degree
rotations. Defaults are `speed:=0.15`, `side_length:=1.0`,
`angular_speed:=0.35`, `publish_rate:=20.0`, `settle_duration:=3.0`,
`pause_duration:=2.0`, `final_stop_duration:=5.0`, and `repeat:=false`. Override them
with normal ROS parameters. `Ctrl-C` sends an emergency zero command.

This is timed open-loop motion without odometry or sensor feedback, so geometric
drift is expected. The Phase 1 EKF runs independently and estimates the motion;
it does not feed back into this command generator.

For physical validation, use low commands and compare the actual Gazebo model pose
before and after each command; controller odometry alone is not proof of motion:

```bash
gz topic -l
timeout 4s ros2 topic pub -r 20 \
  /mobile_base_controller/reference geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.08}}}"
timeout 4s ros2 topic pub -r 20 \
  /mobile_base_controller/reference geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {y: 0.06}}}"
timeout 4s ros2 topic pub -r 20 \
  /mobile_base_controller/reference geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {angular: {z: 0.25}}}"
ros2 topic pub --once /mobile_base_controller/reference \
  geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {}}"
```

Check forward, reverse, both strafes, both rotations, and all four diagonal
combinations. Confirm the corresponding Gazebo pose changes, the driven-wheel sign
pattern, and passive roller velocity in `/joint_states`.

## Stable interfaces

- Command: `/mobile_base_controller/reference` (`geometry_msgs/msg/TwistStamped`)
- Odometry: `/mobile_base_controller/odometry`
- Joint states: `/joint_states`
- TF: `odom -> base_footprint -> base_link`

Wheel joint names are coupled to `mobile_base_bringup/config/controllers.yaml` and
`mobile_base.ros2_control.xacro`. Do not rename these without updating both files and
retesting Gazebo movement:

- `front_left_wheel_joint`
- `front_right_wheel_joint`
- `rear_right_wheel_joint`
- `rear_left_wheel_joint`

The explicit primitive roller collisions are suitable for controller and integration
development. Detailed roller mesh collision remains intentionally out of scope unless
measured cylinder-contact limitations justify its performance cost.
Accurate mecanum ground interaction eventually requires modeled rollers or calibrated
anisotropic contact parameters for the selected Gazebo physics engine.

## Continuous integration

`.github/workflows/ci.yaml` builds all six packages on ROS 2 Jazzy and runs the
deterministic unit, lint, Xacro/URDF, configuration, Python compilation, and
whitespace checks. Gazebo launch tests and the formal campaign stay out of
normal pull-request CI; they remain explicit runtime validation on a stable
host.
