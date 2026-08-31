# ARCHITECTURE — `mobile_base`

This repository targets ROS 2 Jazzy and Gazebo Harmonic. It models one
un-namespaced four-wheel mecanum robot with simulation, state estimation,
mapping, saved-map localization, autonomous navigation, and measurement-only
evaluation tooling. The robot drives itself: set an initial pose, give a goal,
and it navigates there avoiding obstacles. Every command reaches the base
through a single arbiter behind a latching emergency stop.

## Package map

| Package | Responsibility |
| --- | --- |
| `mobile_base_description` | Xacro/URDF, wheel visuals and collision geometry, sensors, ros2_control declaration |
| `mobile_base_gazebo` | Gazebo worlds |
| `mobile_base_bringup` | Top-level simulation, mapping, localization, and evaluation launches; generated SDF |
| `mobile_base_localization` | EKF, SLAM Toolbox, map server, and AMCL |
| `mobile_base_navigation` | Costmaps, planner, controller, behavior tree, command arbitration and e-stop |
| `mobile_base_interfaces` | Action and message definitions for the mobile base |
| `mobile_base_evaluation` | Evaluation-only Gazebo ground-truth selection |
| `mobile_base_tools` | Motion profiles, trajectory recording, diagnostics, reports, and process isolation |

## Runtime data flow

```text
TwistStamped command
        |
        v
/mobile_base_controller/reference
        |
        v
mecanum_drive_controller
        +--> four wheel velocity commands --> gz_ros2_control --> Gazebo
        +--> /mobile_base_controller/odometry
        +--> /mobile_base_controller/controller_state

/mobile_base_controller/odometry + /imu/data
        |
        v
robot_localization EKF
        +--> /odometry/filtered
        +--> odom -> base_footprint
```

Navigation feeds the command topic above through one arbiter. Sources are
limited, then gated for collisions, then arbitrated; `twist_mux` is the only
node permitted to publish the controller reference.

```text
/goal_pose (RViz 2D Goal Pose)
        |
        v
bt_navigator  --NavigateToPose-->  planner_server --> /plan
        |                          controller_server
        |                                  |
        v                          /cmd_vel_nav
behavior_server (wait, spin, backup) ------+
                                           |
                                           v
                                  velocity_smoother
                                           |
                                   /cmd_vel_smoothed
                                           |
                                           v
                                  collision_monitor
                                           |
                                    /autonomy_cmd
                                           |
teleop /cmd_vel_teleop --------------------+
estop_gate /safety/estop_active (lock) ----+
                                           |
                                           v
                                      twist_mux
                                           |
                                           v
                          /mobile_base_controller/reference
```

`bt_navigator` subscribes to `/goal_pose` itself, so RViz's built-in **2D Goal
Pose** tool executes a goal with no bridge node in between. Note that
`nav2_rviz_plugins`' "Nav2 Goal" tool is *not* interchangeable: it publishes
nothing and only drives the Nav2 panel, so a click does nothing without it.

The collision monitor sits **before** the mux, so it gates autonomy without
gating the teleop a human needs to drive out of a corner. The e-stop covers
what it no longer does: a `twist_mux` lock at priority 255 masks every input
below it, teleop included. The lock is a deadman - `estop_gate` publishes a
`Bool` heartbeat, and a lock whose timeout elapses counts as engaged, so
killing the gate stops the robot.

The four driven joints, in controller order, are:

1. `front_left_wheel_joint`
2. `front_right_wheel_joint`
3. `rear_right_wheel_joint`
4. `rear_left_wheel_joint`

The controller consumes `geometry_msgs/msg/TwistStamped` on
`/mobile_base_controller/reference`. That interface, wheel order, radius,
projection sum, signs, odometry semantics, limits, and covariance settings are
stable robot contracts.

The wheel centers are at `x=+/-0.075 m` and `y=+/-0.067 m`. The literal
center projection is therefore `0.142 m`, but the canonical directional-contact
approximation has a measured effective rotational projection of `0.12521 m`.
The controller uses that calibrated value in both inverse kinematics and wheel
odometry, so odometry and commanded motion stay consistent with each other. The
wheel radius remains the measured `0.03074443 m`; translational kinematics do
not depend on the rotational projection.

`0.12521 m` is coupled to the contact model rather than to the geometry, and is
the one "stable contract" here that is not a physical measurement. Re-measure it
whenever `mu`, `mu2`, `slip1`, wheel cylinder geometry, or wheel positions
change. It is also simulation-only: on hardware it would inject an 11.8%
rotational error, so a physical base starts from the geometric `0.142 m` and
derives its own value.

## Canonical mecanum contact

There is one supported simulation contact implementation:

```text
position-specific wheel visual mesh
        +
driven wheel joint
        +
single cylinder collision
        +
direction-dependent friction surface
        +
handed fdir1 expressed in base_footprint
```

The visual mesh retains the actual mecanum appearance. Physics uses one
collision cylinder on each driven wheel; no passive roller links or joints are
generated. Wheel mass remains `0.12 kg`, radius `0.03074443 m`, width
`0.03360543 m`, joint damping `0.001`, and joint friction `0.001`.

The friction direction forms an X pattern:

| Wheel | `fdir1` |
| --- | --- |
| front-left | `1 -1 0` |
| front-right | `1 1 0` |
| rear-right | `1 -1 0` |
| rear-left | `1 1 0` |

Primary friction is `0.8`, secondary friction is `0.2`, and both slip values
are zero. These are implementation constants, not launch arguments.

## Generated-SDF pipeline

The contact surface cannot live reliably in Xacro because `gz sdf -p` drops
the relevant URDF extension tags. The simulation therefore uses this pipeline:

```text
mobile_base.urdf.xacro
        |
        v
xacro expansion
        |
        v
gz sdf -p
        |
        v
generate_sim_sdf.py
        +-- resolve four installed wheel visual meshes
        +-- locate exactly four wheel collisions
        +-- inject anisotropic friction surfaces
        +-- validate joints, cylinders, values, directions, and frame
        +-- reject passive roller bodies
        |
        v
/tmp/mobile_base_sim_<pid>.sdf
        |
        v
Gazebo create
```

`base_footprint` is deliberately the `fdir1` reference. Fixed-joint reduction
leaves it as an SDF link, while `base_link` may become a frame. Expressing the
direction in the surviving chassis frame prevents it from spinning with the
wheel collision.

## TF ownership

The required tree is:

```text
map -> odom -> base_footprint -> base_link -> sensors/wheels
```

| Transform | Owner |
| --- | --- |
| `map -> odom` in mapping | SLAM Toolbox |
| `map -> odom` in saved-map localization | AMCL |
| `odom -> base_footprint` with localization enabled | EKF |
| `odom -> base_footprint` in raw simulation | mecanum controller |
| `base_footprint -> base_link -> sensors/wheels` | robot_state_publisher |

Mapping and saved-map localization are mutually exclusive. AMCL uses
`nav2_amcl::OmniMotionModel`, which describes holonomic localization motion and
is independent of Gazebo contact physics.

The costmaps consume this tree rather than owning any part of it, and both set
`robot_base_frame: base_footprint`. Nav2 defaults that parameter to `base_link`,
which this repository does not use for the odometry chain; leaving the default
silently breaks every costmap transform. The global costmap runs in `map` and
the local costmap in `odom`, so the rolling window is not disturbed by AMCL
corrections.

## Launch composition

- `simulation.launch.py` generates SDF, starts Gazebo, spawns the robot,
  activates joint-state and mecanum controllers, starts optional EKF and RViz,
  and bridges clock/camera/scan/IMU topics.
- `mapping.launch.py` includes simulation with EKF and adds SLAM Toolbox.
- `localization.launch.py` includes simulation with EKF and adds map server and
  AMCL.
- `odometry_evaluation.launch.py` includes canonical simulation and adds
  ground-truth/reset interfaces plus the evaluation runner.
- `planning.launch.py` includes `localization.launch.py` and adds the costmaps,
  planner, and goal bridge. It forces `velocity_smoother:=false`, because the
  smoother is the one stage that remaps onto the controller reference topic.
- `planning_harness.launch.py` runs the same costmaps and planner headlessly
  against static transforms, with no Gazebo, for fast tuning and future CI.

No launch exposes a mecanum contact-model choice. The normal entry points are:

```bash
ros2 launch mobile_base_bringup simulation.launch.py
ros2 launch mobile_base_bringup mapping.launch.py world:=navigation_basic
ros2 launch mobile_base_bringup localization.launch.py \
  world:=navigation_basic map:=/absolute/path/to/map.yaml
ros2 launch mobile_base_navigation planning.launch.py \
  world:=navigation_basic map:=/absolute/path/to/map.yaml
```

Two lifecycle settings in `planning.launch.py` deviate from the `bond_timeout:
4.0` / `autostart: true` pattern used elsewhere, both for reasons observed in
simulation. `bond_timeout` is `0.0`: under sim time the standalone costmap node
activates normally but is never reached by bond, and the manager then reports a
spurious bringup failure. An `autostart` argument exists because costmap
activation races AMCL, which cannot publish `map -> odom` until an initial pose
is set; with `autostart:=false` the nodes are brought up by hand through
`/lifecycle_manager_navigation/manage_nodes` once the pose is set.

## Algorithms and why each was chosen

Every choice below was made against this robot's measurements, not inherited
from a template. The recurring theme is that Nav2's defaults are written for a
differential-drive robot several times this size, and several of them fail
*silently* here — the configuration reads correctly and the robot quietly does
the wrong thing.

### State estimation

| Stage | Algorithm | Why |
| --- | --- | --- |
| Wheel odometry | `mecanum_drive_controller` inverse kinematics | ros2_control stock. Uses a calibrated rotational projection of `0.12521 m`, not the geometric `0.142 m` — see "Canonical mecanum contact" |
| Sensor fusion | `robot_localization` EKF, planar mode | Fuses wheel odometry with the IMU. Lateral mecanum velocity is given deliberately lower confidence than forward, because it is the least accurate axis |
| Mapping | `slam_toolbox`, synchronous, Ceres solver | Pose-graph SLAM with loop closure. Tuned conservatively — see below |
| Localization | `nav2_amcl` with `OmniMotionModel` | Particle filter. The **omni** motion model is required: the differential model cannot represent lateral motion, so a strafing mecanum robot would be modelled as impossible |

**SLAM loop-closure tuning is deliberately strict.** The 4.0 m LiDAR cannot see
across this 10 x 8 m room, so its centre is a feature-poor dead zone where the
matcher falls back on odometry, and the four long walls look alike. The stock
thresholds admitted false closures that rotated the entire pose graph. The fine
and coarse response floors are raised, the matching chain lengthened, and
closures that disagree with odometry penalised. The trade is accepted knowingly:
some genuine closures are now rejected too, so expect slightly more residual
drift instead of a wrong snap. A drifted map is usable; a rotated one is not.

### Planning

`nav2_smac_planner::SmacPlanner2D` — an A* variant on the costmap grid, chosen
for a holonomic base because it does not impose a kinematic model the robot does
not have. `allow_unknown: false`, so it refuses to route through unmapped space.

Costmap sizing is derived from the robot rather than copied:

| Parameter | Value | Derivation |
| --- | --- | --- |
| `robot_radius` | 0.14 | Circumscribed radius is `sqrt(0.108² + 0.0838²) = 0.1367` from `properties.xacro`, rounded up |
| `inflation_radius` | 0.30 | Exceeds the footprint for a usable gradient, and leaves a zero-cost band in the 0.925 m gap at the interior wall's north end |
| `resolution` | 0.05 | Matches the saved map exactly; a mismatch causes resampling artifacts |
| `obstacle_max_range` | 3.5 | Inside the 4.0 m LiDAR maximum, so max-range returns never mark obstacles |

**Inflation does not decide whether a route fits.** Only the inscribed band
within `robot_radius` blocks a global plan; inflation shapes cost. Measured in
`navigation_basic`, whose interior wall leaves a 0.925 m gap: the planner routes
through that gap at `inflation_radius` 0.30, at 0.55, and at Nav2's defaults.
Only raising `robot_radius` above half the gap closes it — at 0.50 the plan
detours 10.59 m instead of 4.48 m. Raising `cost_scaling_factor` makes cost fall
off *faster*, so the robot plans **closer** to walls; it does not enlarge the
cleared region.

### Control

Two mutually exclusive local controllers. Only one runs at a time, because both
publish `cmd_vel_nav` and two publishers on one input means the mux forwards
whichever arrived last.

**Discrete motion model (default).** A state machine — `ALIGN`, `TRANSLATE`,
`ORIENT` — that commands exactly one motion primitive at a time: a pure rotation
in place, or a pure translation along one of eight body-frame directions
(forward, backward, both strafes, four true diagonals with `|vx| == |vy|`). The
primitive chosen is whichever needs the least rotation from the robot's current
heading, which is what makes the diagonals used rather than decorative. Heading
drift is corrected by returning to a rotate-only phase, never by blending a
correction into the translation. Pose comes from a TF lookup in the goal frame,
not from odometry — see "TF ownership" for why that distinction matters.

**MPPI (`controller:=nav2`).** Model Predictive Path Integral: samples a batch
of candidate trajectories forward, scores them with weighted critics, and takes
the weighted average. Configured with:

- `motion_model: "Omni"` — the stock `DiffDrive` never samples lateral motion,
  so a mecanum base would silently behave like a differential one.
- `min_y_velocity_threshold: 0.001` — stock is `0.5`, which zeroes any lateral
  command below 0.5 m/s. This robot's entire validated envelope is 0.10 m/s, so
  the stock value discards *every* strafe. Husarion's ROSbot XL mecanum config
  independently uses 0.001.
- `PreferForwardCritic` deliberately absent — it penalises the lateral and
  reverse motion a mecanum base exists to provide.
- `batch_size: 1000` (stock 2000), measured to hold RTF at 0.999 alongside
  Gazebo.

### Safety

`twist_mux` is the single arbiter: the only node permitted to publish the
controller reference. Priorities put teleop (100) above autonomy (10), so a
human on the keyboard always wins.

The emergency stop is a `twist_mux` **lock** at priority 255, which masks every
input below it including teleop. The lock is a **deadman**: `twist_mux` treats a
lock whose timeout has elapsed as engaged, so `estop_gate` publishes a `Bool`
heartbeat and killing that node stops the robot on its own. The stop latches;
reset restores permission, not motion.

`twist_mux` never republishes and never emits a zero — it simply stops
publishing — so the final stop always comes from the controller's own
`reference_timeout: 0.5`. Measured at 0.121 m/s, confirmed from `/joint_states`
rather than from a command topic: **0.065 m** when the e-stop service is called,
**0.120 m** when `estop_gate` is killed outright.

The collision monitor sits *before* the mux, so it gates autonomy without gating
the teleop a human needs to escape a corner. Its creep zone uses `limit` rather
than `slowdown`: `limit` clamps to an absolute ceiling, where `slowdown` only
scales whatever was commanded, so a fast command still passes through fast.

## Inspecting a running robot

Everything below assumes a stack is already running. Start with stage 5 of the
README quick start.

### The TF tree

```bash
ros2 run tf2_tools view_frames
```

Listens for 5 s, then writes a timestamped PDF and `.gv` into the working
directory - `frames_2026-08-31_08.09.06.pdf`, not `frames.pdf`. It also prints
every frame with its parent, broadcaster and publish rate, which is often
enough without opening the file.

The expected tree:

```text
map ──(AMCL)──> odom ──(EKF)──> base_footprint ──> base_link
                                                     ├──> lidar_link
                                                     ├──> imu_link
                                                     ├──> camera_link ──> camera_optical_frame
                                                     ├──> front_left_wheel_link
                                                     ├──> front_right_wheel_link
                                                     ├──> rear_right_wheel_link
                                                     └──> rear_left_wheel_link
```

Wheel links are continuous joints driven by `joint_state_broadcaster`;
everything else below `base_link` is fixed. `camera_optical_frame` exists
because REP 103 optical frames differ in axis convention from the body frame.

One live transform:

```bash
ros2 run tf2_ros tf2_echo map base_footprint      # robot pose in the map
ros2 run tf2_ros tf2_echo map odom                # AMCL's accumulated correction
```

`map -> odom` is the single most useful diagnostic when a robot goes to the
wrong place: it is the difference between the two frames, and anything
comparing a map-frame goal to an odom-frame pose is wrong by exactly this
amount.

**`map` is absent until an initial pose is set.** AMCL publishes `map -> odom`
only after it has been localized, so before that the tree is rooted at `odom`
and every map-frame lookup fails. A `view_frames` capture with no `map` frame
usually means the 2D Pose Estimate step was skipped, not that anything is
broken.

Typical publish rates in that capture: `odom -> base_footprint` at 50 Hz from
the EKF, the wheel joints at 20 Hz from `joint_state_broadcaster`, and the
fixed sensor transforms as static (reported as a nominal 10000 Hz).

Who is publishing transforms — two broadcasters fighting over one edge is a
classic silent fault:

```bash
ros2 topic info /tf -v | grep -A2 "Publisher count"
```

### Nodes, topics, services, actions

```bash
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```

Everything one node offers and consumes:

```bash
ros2 node info /staged_pose_controller
```

Who is on a topic, and with what QoS — QoS mismatch is a common cause of
"I published but nobody receives":

```bash
ros2 topic info /mobile_base_controller/reference -v
ros2 topic hz /scan
ros2 topic echo /amcl_pose --once
```

### Lifecycle state

Most Nav2 nodes are managed. A node that is `inactive [2]` is running but doing
nothing, which looks identical to a node that is broken:

```bash
ros2 lifecycle nodes
for n in planner_server velocity_smoother collision_monitor; do echo "$n: $(ros2 lifecycle get /$n)"; done
```

### Parameters actually in force

The YAML in `src/` is not necessarily what the process loaded — an older
`install/` copy or a launch-time override wins silently:

```bash
ros2 param dump /controller_server
ros2 param get /controller_server odom_topic
```

### The command chain, end to end

Trace a command from the controller to the wheels. Each stage's output is the
next stage's input:

```bash
for t in /cmd_vel_nav /cmd_vel_smoothed /autonomy_cmd /mobile_base_controller/reference; do echo -n "$t: "; ros2 topic type $t; done
```

The single-arbiter invariant, checked live — more than one publisher here is a
failure, not a curiosity:

```bash
ros2 topic info /mobile_base_controller/reference -v | grep -i "publisher count"
```

And the last link, which no command topic can prove: whether the wheels
actually responded.

```bash
ros2 topic echo /joint_states --once
```

### Robot model and geometry

```bash
xacro mobile_base_description/urdf/mobile_base.urdf.xacro > /tmp/mobile_base.urdf && check_urdf /tmp/mobile_base.urdf
ros2 topic echo /robot_description --once
ros2 topic echo /global_costmap/published_footprint --once
```

### Recording for later

```bash
ros2 bag record /tf /tf_static /scan /odometry/filtered /amcl_pose /mobile_base_controller/reference /plan
```

Bags default to MCAP on Jazzy.

## Validation boundaries

Static description tests enforce four wheel links/joints, dimensions, poses,
visual meshes, inertials, ros2_control interfaces, and absence of passive
roller bodies. Generated-SDF tests enforce exactly four contact surfaces,
canonical `mu`, `mu2`, slip and `fdir1`, handed direction signs, resolvable
friction frame, and clear failure on missing or duplicate collisions.

Localization tests preserve frame ownership and the AMCL omni model. Controller
tests preserve joint ordering and the stamped reference interface.

Navigation tests recompute the circumscribed radius from `properties.xacro` and
assert the costmap footprint covers it, so geometry and costmap cannot drift
apart. They also pin the costmap frames, keep sensor ranges inside the LiDAR
maximum, require the `::` plugin separator that Jazzy needs, and enforce the
single-arbiter invariant: exactly one node publishes
`/mobile_base_controller/reference`, and it is `twist_mux`. They also pin the
holonomic traps Nav2's diff-drive defaults would silently reintroduce, and
require the e-stop lock to outrank every command source with a deadman timeout. Ground-truth
motion diagnostics remain measurement-only and assume the canonical contact
implementation.

## Known boundaries

- The repository supports one robot without namespace isolation.
- Ogre2 is required for usable simulated LiDAR; Ogre1 is retained only for
  scan-independent software-rendered checks.
- The optional velocity smoother remains disabled pending validation of stale
  command behavior.
- Gazebo runtime tests need isolated ROS domains/partitions and serial cleanup.
- Autonomous navigation is present and verified in simulation only. Hardware
  drivers remain outside the architecture, and no claim here is hardware
  evidence: the stop path was measured against simulated wheel feedback.
- Measured stop distances at 0.121 m/s: 0.065 m when the e-stop service is
  called, 0.120 m when `estop_gate` is killed outright (the deadman path costs
  up to one extra lock timeout). `twist_mux` never republishes or emits zero -
  it simply stops publishing - so the final stop comes from the controller's
  own `reference_timeout: 0.5`.
- Nav2 defaults `odom_topic` to `/odom` in both `controller_server` and
  `bt_navigator`. This system has no `/odom`; the EKF publishes
  `/odometry/filtered`. A dead odometry subscription is silent - MPPI simply
  believes the robot never moves and produces degenerate control.
- Costmap inflation is not lethal, so `inflation_radius` does not decide whether
  a plan fits through a gap - only `robot_radius` can close one, by making the
  gap inscribed cost end to end. Inflation governs path cost, which matters once
  a local controller follows the gradient. Measured in `navigation_basic`: the
  0.925 m north gap stays passable at both the tuned 0.30 m and Nav2's 0.55 m
  default, and closes only above a 0.4625 m robot radius.
- The standalone `nav2_costmap_2d` node reads a flat parameter namespace, unlike
  costmaps embedded in a server, which use a nested one. The nested form fails
  silently by falling back to `base_link`.
- Saved maps under `/maps/` are gitignored runtime artifacts, so a fresh clone
  must run mapping before any launch that requires a map.
- Generated `/tmp/mobile_base_*` files are process-keyed and are not currently
  removed by launch shutdown.

Earlier contact experiments and their measurements are historical evidence in
`docs/mecanum_motion_accuracy.md`; they are not supported runtime paths.

## Where to start

| Task | Start here |
| --- | --- |
| Change robot geometry | `mobile_base_description/urdf/properties.xacro`, then controller/evaluator constants and `test_mecanum_wheels.py` |
| Change contact physics | `mobile_base_bringup/scripts/generate_sim_sdf.py`, then `test_mecanum_wheel_contact.py` |
| Add a sensor | description Xacro + Gazebo Xacro + bridge + timestamp contract + sensor test |
| Change TF ownership | simulation spawners + EKF config + TF validator tests |
| Add a motion profile | `mobile_base_tools/config/odometry_tests.yaml` |
| Change costmap or planner tuning | `mobile_base_navigation/config/`, then `test_navigation_config.py`; iterate with `planning_harness.launch.py` |
| Change how the robot drives | `mobile_base_navigation/config/controller.yaml`, then `test_navigation_config.py` |
| Add a command source | `config/twist_mux.yaml`; feed the mux, never the controller reference directly |
