# Mobile Base

Modular ROS 2 Jazzy model and Gazebo simulation for a four-wheel mecanum platform.

## Packages

| Package | Responsibility |
| --- | --- |
| `mobile_base_description` | Xacro model, SI-unit geometry, inertial data, and RViz configuration |
| `mobile_base_gazebo` | Gazebo Harmonic world and simulator-specific assets |
| `mobile_base_bringup` | ros2_control configuration and top-level launch files |
| `mobile_base_localization` | Planar EKF, SLAM Toolbox mapping, Nav2 map serving, and holonomic AMCL |
| `mobile_base_navigation` | Costmaps, planning, control, behavior tree, command arbitration and e-stop |
| `mobile_base_tools` | ToF floor classification, odometry-path visualization and repeatable motion checks |

The packages are independent of the existing arm stack.

## Project status

**Status: Phases 1, 2 and 3 are closed. The robot navigates autonomously in
simulation: set an initial pose, give it a goal, and it drives there avoiding
obstacles, repeatedly, behind a latching emergency stop.**
Phase 1 delivered the mecanum simulation and control stack, raw wheel odometry,
fused wheel/IMU odometry, explicit TF ownership,
timestamp and TF contract validation.

The raw and fused odometry campaigns that closed Phase 1 concluded that fusion
substantially improves yaw accuracy and cross-axis drift, that endpoint position
performance is broadly similar either way, and that square-path closure improves
with fusion. **The campaign tooling and its result sets have since been removed
from the repository**, so those numbers are no longer reproducible here; the
findings and the calibration decision they justify are kept in
[`docs/mecanum_motion_accuracy.md`](docs/mecanum_motion_accuracy.md) as a
historical record.

The canonical single-cylinder, direction-dependent wheel contact implementation
has been manually validated through simulation startup, controller operation,
mecanum motion, odometry/EKF, SLAM mapping, occupancy-map save, saved-map loading,
and AMCL localization. Phase 2 adds SLAM Toolbox mapping and saved-map AMCL
localization. Phase 3
adds the full Nav2 stack: costmaps and a global planner (3a), the command chain
built but disconnected (3b), and autonomous driving behind arbitration and an
e-stop (3c). Everything is verified in simulation only; hardware commissioning
is later work.

### Two motion profiles

**`primitive` is the default.** The robot turns to face each leg of the path,
then drives it — one motion at a time, never blending rotation with
translation. Paths are eight-heading polylines, so straight runs and 45-degree
diagonals only.

```bash
ros2 launch mobile_base_navigation planning.launch.py map:="$HOME/ros2_ws/src/mobile_base/maps/navigation_basic.yaml"
```

**`holonomic`** is the original SmacPlanner2D + MPPI stack, which blends
translation and rotation freely. It is the profile every measurement in this
README was taken against.

```bash
ros2 launch mobile_base_navigation planning.launch.py motion_profile:=holonomic map:="$HOME/ros2_ws/src/mobile_base/maps/navigation_basic.yaml"
```

Both are described in
[`ARCHITECTURE.md`](ARCHITECTURE.md#motion-profiles). Two things to know before
relying on either:

- **Speeds were doubled** — `primitive` runs 0.20 m/s / 0.60 rad/s, `holonomic`
  0.30 m/s / 1.20 rad/s. That is 2-4x anything characterised, and the stopping
  distances recorded below were measured at 0.12 m/s. They no longer describe
  the current configuration.
- **Navigation waits for an initial pose.** Nothing activates until you set
  **2D Pose Estimate** in RViz; a costmap built before then is drawn against a
  pose you never supplied. Pass `autostart:=true` to skip the wait.

### Camera view

The camera has its own window and its own command. Start any stack first, then
in another terminal:

```bash
ros2 launch mobile_base_bringup camera_view.launch.py
```

It attaches to whatever is already running rather than starting a robot of its
own. Add `image_topic:=/your/topic` to point it elsewhere, or
`use_sim_time:=false` on hardware.


## Where the rest of the documentation lives

Three documents, split by the question you are asking.

| You want to | Read |
| --- | --- |
| Know what this is and why it works the way it does | this file |
| Run it, drive it, map it, look at a sensor | [docs/GUIDE.md](docs/GUIDE.md) |
| Know a frame, a topic, a measurement or a derivation | [ARCHITECTURE.md](ARCHITECTURE.md) |

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
| `robot_radius` | 0.16 | Circumscribed radius is the end-cap corner, `sqrt(0.130² + 0.0838²) = 0.1547` from `properties.xacro`, rounded up |
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

## Continuous integration

`.github/workflows/ci.yaml` builds all seven packages on ROS 2 Jazzy and runs the
deterministic unit, lint, Xacro/URDF, configuration, Python compilation, and
whitespace checks. Gazebo launch tests and the formal campaign stay out of
normal pull-request CI; they remain explicit runtime validation on a stable
host.
