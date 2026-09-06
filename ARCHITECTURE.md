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

## Body geometry

The body is a dual-deck plate frame, not a solid block. `base_link` carries one
visual per printed part - two 10 mm deck plates and four corner standoffs -
plus a single box collision spanning the whole body. Splitting the visuals
while keeping one collision is deliberate: the planner must never see the open
electronics bay between the decks as passable geometry.

The vertical stack hangs off the wheel radius. The motors stand on the lower
deck and drive the wheels directly, so the deck's top face is pinned exactly
one motor shaft offset below the wheel axle:

| Surface | Height above ground |
| --- | --- |
| Lower deck underside (ground clearance) | 0.0095 m |
| Lower deck top, motors stand here | 0.0195 m |
| Motor tops | 0.0419 m |
| Upper deck underside, IMU hangs here | 0.0595 m |
| Upper deck top, LiDAR stands here | 0.0695 m |
| LiDAR scan plane | 0.0915 m |
| Overall height | 0.0995 m |

The camera is mounted on the front face of the upper deck, centred on the
plate's thickness. The LiDAR is a square dark base carrying a cylindrical
rotating head, and its collision is one box bounding both.

Ground clearance is therefore not a free parameter: it is
`wheel_radius - motor_shaft_offset - lower_deck_thickness`, and a thicker lower
deck or a taller motor eats directly into it.

Each motor lies flat on a large gearbox face with its output shaft horizontal,
running outboard along the wheel axis and into the wheel hub. The mesh carries
its shaft along the mesh Z axis, so every instance rolls 90 degrees about X to
lay that axis across the robot, and front and rear differ by a 180 degree yaw
so the round motor can always points inboard. Left and right are mirror images,
which no rotation can express, so the two sides rest on opposite faces.

`base_link` itself is pinned to 0.040372215 m, the height the motion model was
identified at. The body grew upward around that frame rather than moving it, so
every existing TF offset, controller gain and test expectation still holds.

Each motor is its own link, parented to `base_link` at the same origin as the
wheel it drives - the link frame is the output shaft. The wheel joints stay
parented directly to `base_link`, so the drive chain and kinematics are
untouched. Motors carry negligible inertia: the Gazebo URDF reduction lumps
fixed-joint children into the parent, and `chassis_mass` is already the
whole-body mass of the validated model. They also carry no collision, both
because the body box already covers them and because an extra collision would
break the one-surface-per-wheel contract the SDF generator enforces.

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
| Mapping | `slam_toolbox`, synchronous, Ceres solver | Pose-graph SLAM with loop closure. Wide search, strict acceptance — see below |
| Localization | `nav2_amcl` with `OmniMotionModel` | Particle filter. The **omni** motion model is required: the differential model cannot represent lateral motion, so a strafing mecanum robot would be modelled as impossible |

**SLAM tuning trades a wide loop search against strict acceptance.** The 4.0 m
LiDAR cannot see across either world, so room centres are feature-poor dead
zones where the matcher falls back on odometry, and long parallel walls look
alike.

Two failures have to be balanced against each other:

- *Search too narrow and loop closure never fires.* `loop_search_maximum_distance`
  is the radius around the current pose in which candidate closures are looked
  for. The stock 3.0 m was sized for `navigation_basic`; when `my_world` was
  20 x 20 m, a circuit accumulated drift far beyond that radius before
  returning, the search never reached the earlier pose, the graph never
  snapped, and the map translated and superimposed on itself. It is now
  **6.0 m** against a 10 x 10 m world — over half the room, but deliberately
  not all of it.
- *Search too wide and a false closure folds the map onto itself.* An apartment
  of similar rectangular rooms is exactly the repetitive geometry that provokes
  one, and a false closure is unrecoverable where drift is merely untidy. A
  search radius approaching the map size makes every similar room a candidate.
  The coarse and fine response floors are held at 0.55 / 0.70 and the minimum
  chain at 20.

The trade is accepted knowingly: some genuine closures are still rejected, so
expect residual drift rather than a wrong snap. A drifted map is usable; a
folded one is not.

**Geometry sets the ceiling, not tuning.** Sampling a world on a grid and
ray-casting the 4.0 m LiDAR at each free pose gives the fraction of the floor
that sees only *one* wall axis. Two parallel walls fix lateral position and
heading but leave position *along* the corridor unobservable, so the matcher
accepts whatever odometry says and drift there is uncorrected by construction.

| `my_world` | Floor seeing only one wall axis |
| --- | --- |
| at 20 x 20 m | **21.6%** — the 20 m hall was unmappable in its long axis |
| at 10 x 10 m | **0%** |

Shrinking the world to 10 x 10 m removed the problem outright: no pose in the
apartment is now underconstrained. No amount of SLAM tuning fixes an
unobservable direction — only geometry does.

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

`nav2_mppi_controller::MPPIController` — Model Predictive Path Integral. It
samples a batch of candidate trajectories forward from the current state, scores
each with weighted critics, and takes the weighted average as the command. It
was chosen over DWB because it handles holonomic motion through its motion model
rather than through hand-tuned per-axis samplers, and needs far less
critic-by-critic tuning.

Four settings matter more than the rest, and three of them are corrections to
diff-drive defaults that fail silently on a mecanum base:

- `motion_model: "Omni"` — stock is `DiffDrive`, which never samples lateral
  motion, so the mecanum base would silently behave like a differential one.
  The installed library reports the valid options as `DiffDrive`, `Omni` and
  `Ackermann`.
- `min_y_velocity_threshold: 0.001` — stock is `0.5`, which zeroes any lateral
  command below 0.5 m/s. This robot's entire validated envelope is 0.10 m/s, so
  the stock value discards *every* strafe. Husarion's ROSbot XL mecanum config
  independently uses 0.001.
- `odom_topic: /odometry/filtered` — stock is `/odom`, which has no publisher
  here. MPPI feeds measured velocity back into its optimiser, so a dead
  subscription makes it believe the robot is permanently stationary. Measured
  symptom: commands pinned near 0.02 m/s against a 0.15 limit, and the robot
  driving the wrong way, with nothing logged.
- `PreferForwardCritic` deliberately absent — it penalises the lateral and
  reverse motion a mecanum base exists to provide.

`batch_size: 1000` (stock 2000) was measured to hold RTF at 0.999 alongside
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

Recovery behaviours are `wait`, `spin` and `backup`, in that order — the tree
tries costmap clearing and waiting before any commanded motion. `spin` is
enabled on measurement rather than assumption: a full rotation in place changed
the closest observed obstacle distance by 0.01 m, because this robot's footprint
is circular and rotating sweeps nothing beyond the radius it already occupies.

## Motion profiles

Two planner/controller pairs ship, selected by one launch argument.

```bash
# primitive (default) - turn to face each leg, then drive it
ros2 launch mobile_base_navigation planning.launch.py map:="$HOME/ros2_ws/src/mobile_base/maps/navigation_basic.yaml"

# holonomic - MPPI, blended motion, the profile every measurement was taken against
ros2 launch mobile_base_navigation planning.launch.py motion_profile:=holonomic map:="$HOME/ros2_ws/src/mobile_base/maps/navigation_basic.yaml"
```

Everything earlier in this document describes the **holonomic** pair, which is
still the only one with measured accuracy behind it. The **primitive** pair is
now the default because its motion is legible: the robot faces where it is
about to go, so its heading tells you its intent.

`motion_profile` selects a pair, not a stack. Both profiles use the same
costmaps, the same goal and progress checkers, the same `odom_topic`, the same
behaviour tree and the same command chain below `controller_server`:

```text
motion_profile:=holonomic                  motion_profile:=primitive (default)
        |                                          |
        v                                          v
  planner.yaml                              planner.yaml
                                          + planner_lattice.yaml   (overlay)
        |                                          |
        v                                          v
  GridBased =                               GridBased =
  nav2_smac_planner::SmacPlanner2D          mobile_base_navigation::LatticePlanner
        |                                          |
  controller.yaml                           controller.yaml
                                          + controller_primitive.yaml (overlay)
        |                                          |
        v                                          v
  FollowPath =                              FollowPath =
  nav2_mppi_controller::MPPIController      mobile_base_navigation::PrimitiveController
        |                                          |
        +--------------------+---------------------+
                             |
                             v
                       /cmd_vel_nav  --> velocity_smoother --> collision_monitor
                                     --> twist_mux --> /mobile_base_controller/reference
```

The overlays are **parameter files loaded second**, naming only the leaves that
change; later files win leaf by leaf. That is why the costmap block, the goal
checker and the `min_*_velocity_threshold` corrections exist in exactly one
place and cannot drift between profiles. It is also why the plugin **ids** stay
`GridBased` and `FollowPath` even though the profile is named after them: the
behaviour tree runs a `PlannerSelector` defaulting to `GridBased` and a
`ControllerSelector` defaulting to `FollowPath`, so swapping the plugin *type*
under a stable id is what makes the alternate profile a drop-in.

### `mobile_base_navigation::LatticePlanner`

A* over `(x, y, heading)`, where heading is one of eight directions 45 degrees
apart. Two edge types:

- **TRANSLATE** — one step along the current heading. A diagonal is its own
  edge type with length `step * sqrt(2)`, not a forward edge composed with a
  strafe, because the robot executes it as a single primitive and the search
  has to cost it as one.
- **ROTATE** — plus or minus one heading, cost `turning_cost_weight` times the
  angular distance. Restricting rotation to adjacent headings keeps the
  branching factor at three; because the cost is linear in angle, composing two
  45-degree edges costs exactly what one 90-degree edge would.

`step_size_m` is quantised to a whole number of costmap cells. This is not
tidiness: a diagonal edge of an arbitrary length lands between cell centres and
the lattice walks off its own grid after a few expansions.

Edge validity is the robot's disc swept along the segment, computed as a
**dilation of the lethal set done once per search** rather than a disc test per
candidate edge — the naive form is tens of millions of lookups on a room-sized
map. Only `LETHAL_OBSTACLE` is dilated, never `INSCRIBED_INFLATED_OBSTACLE`:
the inflation layer has already marked the inscribed band using the same
`robot_radius`, so dilating that too would apply the footprint twice and close
gaps the robot fits through. `allow_unknown: false` is preserved, so unknown
space is as impassable as a wall.

The radius comes from `getCircumscribedRadius()` rather than being restated.
Worth knowing, because it does not read back as the 0.14 in `planner.yaml`:
Nav2 builds a 16-gon from `robot_radius` then pads it by `footprint_padding`
(0.01) per coordinate, so a 45-degree vertex moves from `(0.099, 0.099)` to
`(0.109, 0.109)` and the logged radius is **0.154 m**. That is the padded
footprint the rest of Nav2 collision-checks with, so matching it keeps the
planner and the costmap agreeing.

Consecutive same-heading edges are merged before the path is returned. A 3 m
straight run at a 0.05 m step is 60 edges and comes back as **two poses**.
Every pose orientation carries the **travel direction** of the segment leaving
it — not a commanded body yaw — except the last, which carries the requested
goal yaw for the goal checker to compare against.

A measured example, planning from the map origin to `(2.5, 3.4)` in
`navigation_basic`:

| # | Position | Segment leaving it |
| --- | --- | --- |
| 0 | (0.005, −0.004) | 45.000 deg, 1.344 m |
| 1 | (0.955, 0.946) | 90.000 deg, 2.100 m |
| 2 | (0.955, 3.046) | 45.000 deg, 0.495 m |
| 3 | (1.305, 3.396) | 0.000 deg, 1.150 m |
| 4 | (2.455, 3.396) | — |

Four segments, every one an exact multiple of 45 degrees, threading the 0.925 m
gap at the interior wall's north end.

### `mobile_base_navigation::PrimitiveController`

A state machine, not an optimiser:

```text
ALIGN_TO_SEGMENT_HEADING --> EXECUTE_SEGMENT --> (next segment) ... --> FINAL_ORIENT
         ^                        |
         +---- yaw drift ---------+
```

`ALIGN` and `FINAL_ORIENT` command angular velocity only. `EXECUTE_SEGMENT`
commands linear velocity only, always along one of the eight **body-frame**
directions, so a diagonal has `|vx| == |vy|` exactly and nothing in between is
reachable.

`align_to_segment` (default **true**) decides what `ALIGN` aims at, and it is
the single most visible setting in the profile:

- **`true` — orient, then move.** `ALIGN` drives the body yaw onto the
  segment's own direction, so every `EXECUTE_SEGMENT` is a pure `FORWARD`
  along the way the robot is pointing. The lattice's diagonals are still used;
  the robot turns to face along them and drives forward. This is the default
  because the heading always shows intent.
- **`false` — crab.** `ALIGN` snaps to the *nearest* of the eight headings, at
  most 22.5 degrees, once. Because the plan's segment directions are themselves
  45-degree-snapped, every segment is then exactly a body-frame primitive and
  the base strafes or moves diagonally **without turning at all**. Far fewer
  rotations, and the reason a holonomic base exists — but the robot crabs
  sideways and its heading tells you nothing.

Regulation is a PID on heading error and a separate PID on **along-track**
distance, with both integrators and both derivative histories reset on every
phase transition. Two traps are worth naming:

- Along-track distance, not straight-line distance to the waypoint.
  Straight-line distance never goes negative, so a robot that overshoots would
  be driven back and forth across the waypoint forever. The projection goes
  negative and ends the segment.
- An integrator carried across a phase change is winding up against an error it
  was never regulating. The align integral would dump itself into the first
  translation tick — precisely the blended command this profile exists to make
  impossible.

`realign_yaw_rad` (0.15) is deliberately wider than `yaw_tolerance_rad` (0.05).
Equal values chatter: `EXECUTE` leaves the moment the estimate crosses the
line, `ALIGN` hands straight back, and the robot alternates between rotating
and translating without progressing.

The plan is planned in `map` and executed against a pose in the local costmap's
`odom` frame, so it is re-transformed **every tick** — `map -> odom` moves
whenever AMCL corrects. Only the geometry is refreshed; the phase and segment
index persist, or the machine would restart 20 times a second and never leave
the first segment. The body-frame direction is snapped afterwards, which
absorbs up to 22.5 degrees of `map -> odom` rotation before a segment could be
misclassified.

Default limits are **0.20 m/s and 0.60 rad/s**, doubled from the validated
0.10 / 0.30 by request. Both profiles were doubled; MPPI now runs 0.30 / 0.30 /
1.20. Nothing about that is characterised, and the measured stop distances
recorded earlier in this document were taken at 0.12 m/s and no longer describe
the current configuration. The velocity smoother's `[0.5, 0.5, 2.0]` ceiling is
the only remaining bound, and the static tests assert both controllers stay
under it rather than asserting a number.

### The degenerate-replan bug, and why `min_segment_length_m` exists

`bt_navigator` replans at 1 Hz. A replan issued while the robot is already
sitting on its goal returns a path a few millimetres long, whose direction is
rounding noise rather than intent. Under `align_to_segment`, the robot turned
to face that noise, settled, and was handed a fresh one a second later — which
looked exactly like random spinning at the moment it should have been settling
the goal heading. `min_segment_length_m` (0.05) drops sub-threshold waypoints
when a plan loads, so such a plan has **no segments at all** and the machine
goes straight to `FINAL_ORIENT` instead of fighting it.

### What is verified, and what is not

Measured in simulation across three goals with a commanded final heading,
sampling `/cmd_vel_nav`, at the doubled speeds and with orient-then-move:

| | Result |
| --- | --- |
| Goals reached | 3 / 3 |
| Blended commands | **0** |
| Final position error | 0.024 - 0.096 m |
| Final heading error | **4.9 / 5.3 / 4.8 deg** |
| Peak commanded speed | 0.200 m/s |

Heading error was **21.7 / 23.1 / 24.4 deg** before one fix, and that spread is
the signature of a tolerance cap rather than a control problem: all three sit
just inside 28.6 deg, which is the `yaw_goal_tolerance: 0.50` inherited from
the MPPI config. `controller_server` stops calling a controller the moment its
goal checker is satisfied, so `FINAL_ORIENT` was being cut off mid-rotation
every time. The checker, not the controller, was the limit. The overlay
tightens it to 0.10 rad for this profile only; `controller.yaml` keeps 0.50 for
MPPI, which has no final-rotation phase and re-approaches forever if the
tolerance is tight.

Position error sits well inside `xy_goal_tolerance` here, because orienting
before moving means the robot arrives along the segment rather than being
carried past it by a blended approach.

What is **not** established: this profile has no accuracy campaign behind it,
no stopping-distance measurement of its own, and no hardware exposure. It is
the default because its motion is legible, not because it is better
characterised - `holonomic` remains the profile every other number in this
document was measured against, and the doubled speeds put both profiles outside
what was characterised at all. One further caveat, since it is easy to
misread a wheel-side trace: the velocity smoother still ramps *across* a phase
transition, so a twist sampled at `/mobile_base_controller/reference` during a
switch can briefly carry both terms even though the controller never commanded
both. The invariant this profile guarantees is on the controller's own output.

## Deferred navigation startup

The navigation lifecycle manager runs with `autostart: false`, and a small
`nav_autostart` node calls `STARTUP` once `/amcl_pose` arrives.

The reason is that a costmap brought up before an initial pose is set is worse
than one that is absent: it exists, it reports `active`, and it is built on
whatever `map -> odom` happened to be there, so the map appears smeared against
a pose nobody supplied. `/amcl_pose` is the right signal precisely because AMCL
publishes it *only after receiving an initial pose* — it means "a human has
said where the robot is", not merely "a node is running".

```text
launch --> planner_server, controller_server, ... all unconfigured
                    |
   RViz "2D Pose Estimate" --> AMCL --> /amcl_pose
                    |
              nav_autostart --> manage_nodes STARTUP
                    |
             costmaps, planner, controller --> active
```

Override with `autostart:=true` to bring everything up immediately, or
`autostart_on_localization:=false` to do it by hand:

```bash
ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"
```

## Inspecting a running robot

The camera has its own window and its own command, deliberately separate from
every stack launch — it is a viewer, and starting a second simulation to look
at the first one is not what anybody wants:

```bash
ros2 launch mobile_base_bringup camera_view.launch.py
```

It attaches to `/camera/image_raw` on whatever is already running.
`use_sim_time` defaults to true because the images carry simulation timestamps
and a viewer on wall time treats every one of them as far in the past.

Everything below assumes a stack is already running. Start with stage 5 of the
README quick start.

### The TF tree

```bash
ros2 run tf2_tools view_frames
```

Listens for 5 s, then writes a timestamped PDF and `.gv` into the working
directory — `frames_2026-08-31_08.09.06.pdf`, not `frames.pdf`. It also prints
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
the EKF, the wheel joints at 20 Hz from `joint_state_broadcaster`, and the fixed
sensor transforms as static (reported as a nominal 10000 Hz).

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
ros2 node info /controller_server
```

Who is on a topic, and with what QoS — a QoS mismatch is a common cause of
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
for n in controller_server planner_server behavior_server bt_navigator velocity_smoother collision_monitor; do echo "$n: $(ros2 lifecycle get /$n)"; done
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

And the last link, which no command topic can prove: whether the wheels actually
responded.

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

Static description tests enforce four wheel links/joints, four motor links on
their wheel origins, the dual-deck stacking contract and its bounding
collision, dimensions, poses,
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
