# ARCHITECTURE — `mobile_base`

This repository targets ROS 2 Jazzy and Gazebo Harmonic. It models one
un-namespaced four-wheel mecanum robot with simulation, state estimation,
mapping, saved-map localization, global path planning, and measurement-only
evaluation tooling. Nothing in the repository drives the robot autonomously:
planning is preview-only and no component publishes motion commands.

## Package map

| Package | Responsibility |
| --- | --- |
| `mobile_base_description` | Xacro/URDF, wheel visuals and collision geometry, sensors, ros2_control declaration |
| `mobile_base_gazebo` | Gazebo worlds |
| `mobile_base_bringup` | Top-level simulation, mapping, localization, and evaluation launches; generated SDF |
| `mobile_base_localization` | EKF, SLAM Toolbox, map server, and AMCL |
| `mobile_base_navigation` | Costmaps, global planner, and RViz goal-to-plan preview |
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

Planning is a separate, unconnected branch. It reads the map, the scan and TF,
and produces a path for display only; there is no edge from it back to the
command topic above.

```text
/goal_pose (RViz goal tool)
        |
        v
goal_to_plan  --ComputePathToPose-->  planner_server
                                          +--> /plan
                                          +--> /global_costmap/costmap
                                          +--> /global_costmap/published_footprint

/scan + /map + TF --> global_costmap (map frame)
                  --> local_costmap  (odom frame, rolling)
```

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
no-motion property: no controller, behavior-tree, behavior-server, mux or
collision-monitor component, and the string `/mobile_base_controller/reference`
in no file in the package. Ground-truth
motion diagnostics remain measurement-only and assume the canonical contact
implementation.

## Known boundaries

- The repository supports one robot without namespace isolation.
- Ogre2 is required for usable simulated LiDAR; Ogre1 is retained only for
  scan-independent software-rendered checks.
- The optional velocity smoother remains disabled pending validation of stale
  command behavior.
- Gazebo runtime tests need isolated ROS domains/partitions and serial cleanup.
- Global path planning is present; goal execution, local control, recoveries,
  command arbitration, and hardware drivers are outside the current
  architecture. Nothing can drive the robot.
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
| Add goal execution or autonomous motion | `mobile_base_navigation`, adding a controller server and behavior tree only behind command arbitration and safety validation |
