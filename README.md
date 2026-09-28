# Mobile Base — an autonomous mecanum robot in ROS 2

A four-wheel mecanum platform that maps a space, localizes in it, and drives
itself to a goal while avoiding obstacles — built and verified end to end in
ROS 2 Jazzy and Gazebo Harmonic.

The interesting problem here is not the navigation stack; that is off-the-shelf
Nav2. It is everything underneath it: a holonomic drive whose wheel contact has
to be modelled rather than assumed, a body whose every dimension is derived from
one measured wheel radius, and a sensor suite chosen to cover a blind spot the
LiDAR structurally cannot see.

| | |
| --- | --- |
| **Drive** | 4 × mecanum, holonomic — moves any direction without turning first |
| **Footprint** | 260 × 168 mm, 99.5 mm tall, 9.5 mm ground clearance |
| **Mass** | 1.80 kg |
| **Compute** | ROS 2 Jazzy, Nav2, Gazebo Harmonic |
| **Sensing** | 360° LiDAR · 8-sensor ToF ring · IMU · camera |
| **Verification** | 281 automated tests |
| **Status** | Autonomous navigation working in simulation |

---

## The robot

### Chassis

Two 3D-printed PLA decks, 10 mm thick, separated by a 40 mm electronics bay. In
plan the plates form a cross: a central spine with a full-width bumper cap at
each end and a wing filling the space between each side's wheels, with the four
bumper corners chamfered at 45°.

Nothing in that outline is a free parameter. Every bound derives from the wheel
geometry, so moving a wheel moves the deck with it, and ground clearance is
`wheel_radius − motor_shaft_offset − deck_thickness` rather than a number
someone chose. The end caps reach past the wheels deliberately: an impact lands
on deck, never on a wheel.

The chamfer cannot be expressed with URDF primitives, so the plate is a
**generated mesh** — a script emits the STL from the same properties the
collision boxes are built from, and a test regenerates it and byte-compares.
Editing the body geometry and forgetting the mesh fails the suite instead of
shipping a plate that no longer matches itself.

### Drivetrain

Four TT gearmotors driving mecanum wheels directly, two TB6612FNG drivers, a 2S
LiPo sitting low and central to keep the centre of mass down, and an ESP32-S3
at the rear with its USB-C facing aft so it can be flashed without disassembly.

Mecanum wheels are the reason this project is not a differential-drive tutorial.
Each wheel's rollers sit at 45°, so the contact patch behaves differently along
and across the roller axis. Modelling that **direction-dependent friction** is
what makes simulated motion match the real kinematics; treating the wheels as
isotropic cylinders produces a robot that slides when it should grip.

---

## Sensing: three layers, one blind spot

The sensor suite exists to answer one question the LiDAR alone cannot.

### The 82 mm problem

The LiDAR scan plane sits at **91.5 mm**. Ground clearance is **9.5 mm**.
Anything between those two heights physically stops the robot and is completely
invisible to it:

```
   99.5 mm ─── top of robot
   91.5 mm ═══ LiDAR scan plane ── the only thing the costmap used to know
            
            ▒ 82 mm of obstacles that stop the robot
            ▒ and do not exist in its world model
            
    9.5 mm ─── ground clearance
       0 mm ▀▀▀ floor
```

A shoe. A cable. A door threshold. The wheels stall, and stalled wheels feed
corrupt odometry into the state estimator — so one shoe can cost the robot its
localization.

### The layers

| Layer | Sensor | Range | Job |
| --- | --- | --- | --- |
| **Mapping** | 360° planar LiDAR | 4.0 m | SLAM, localization, long-range planning |
| **Near field** | 8 × VL53L7CX ToF | 0.4 / 1.0 / 2.0 m | Low obstacles / obstacles / walls, all round |
| **Attitude** | IMU | — | Yaw fusion for odometry |
| **Inspection** | RGB camera | — | Human situational awareness |

The **ToF ring** is eight multizone time-of-flight sensors on the lower deck —
one at the centre of each body face and one on each bumper chamfer. Each reports
an 8 × 8 grid of ranges over a 60° field. Eight fans at 45° spacing cover the
full circle with 15° of overlap at every seam, which matters for a holonomic
base: it can strafe or move diagonally at any moment, so there is no "forward"
to point a sensor at.

Each sensor can be switched off from a panel, and RViz draws what the ring
sees as translucent walls, objects and one floor region around the robot.

They sit at 28.5 mm and are aimed partly at the floor on purpose. That is a
deliberate trade with a hard consequence — see *Findings* below.

---

## Software stack

Standard components, chosen deliberately. What follows is the reasoning
behind each, including where a stock default was wrong for a holonomic base.

### Why each algorithm was chosen

Every choice below was made against this robot's measurements, not inherited
from a template. The recurring theme is that Nav2's defaults are written for a
differential-drive robot several times this size, and several of them fail
*silently* here — the configuration reads correctly and the robot quietly does
the wrong thing.

#### State estimation

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

#### Planning

`nav2_smac_planner::SmacPlanner2D` — an A* variant on the costmap grid, chosen
for a holonomic base because it does not impose a kinematic model the robot does
not have. `allow_unknown: false`, so it refuses to route through unmapped space.

Costmap sizing is derived from the robot rather than copied:

| Parameter | Value | Derivation |
| --- | --- | --- |
| `robot_radius` | 0.155 | Circumscribed radius is the end-cap corner, `sqrt(0.130² + 0.0838²) = 0.1547` from `properties.xacro`: the smallest circle safe at every heading. `footprint_padding` 0.01 on top |
| `inflation_radius` | 0.22 | Just past the padded footprint (0.165) for a gradient, with `cost_scaling_factor` 8 so cost falls off within centimetres; leaves a 0.485 m zero-cost band in the 0.925 m gap |
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

#### Control

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

#### Safety

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


---

## What is being tested, and how

The project's working assumption is that **a number nobody has measured is a
guess**. Correctness is established three ways.

### Derivation over declaration

Geometry is computed from measured primitives, not typed in. The deck outline
comes from the wheel envelopes; the Nav2 footprint radius comes from the deck
corner; the mass budget holds at 1.80 kg by construction because the deck plates
absorb the balance, so adding a component cannot silently break the total. Tests
assert the *relationships*, so a change that violates one fails the build rather
than drifting quietly.

### 281 automated tests

| Kind | Covers |
| --- | --- |
| Description | URDF structure, link placement, collision envelopes, mass budget, generated mesh freshness |
| Configuration | Nav2 parameters against sensor limits, single-arbiter command chain, frame conventions |
| Unit | Sensor classification, geometry, odometry accumulation, timestamp contracts |
| Launch | Gazebo brought up headless, real topic contracts asserted on live messages |

### Measured, not assumed

Sensor behaviour is verified against a running simulator and the numbers are
recorded. Where a prediction and a measurement disagree, both go in the docs.

---

## Findings

The parts of this project that were not obvious going in.

### Mounting height is a permanent trade

The ToF apertures sit at 28.5 mm. Their downward rows reach the floor within
435 mm, and past **1.083 m every remaining ray has climbed above the robot's own
99.5 mm roof**. Nothing beyond that distance could obstruct it. Low mounting
buys near-field floor visibility and costs far-field low-obstacle range, and no
amount of software changes it — the ring is a near-field sensor permanently.

### One range for everything was wrong

Different rows see different things, so each kind gets its own marking range.
Low obstacles are caught only by the downward rows, which meet the floor by
0.435 m, so they stop at **0.4 m**. Taller obstacles are seen by the +3.75°
row until it clears the roof at 1.083 m, so **1.0 m**. A wall fills a whole
column of upward zones at one distance, which proves it stands on the floor
however far away it is, so walls are marked to **2.0 m**. A single 0.8 m limit
was too far for the first and too short for the last.

Tilting the sensors so one row is exactly level looks attractive and works in
simulation, where zones are thin rays. Real zones are 7.5° cones, so a level
row's lower half meets the floor from 0.435 m anyway, and the tilt costs half
the low-obstacle range. The sensors stay level.

### One trigonometric term decided whether it worked

A ray's predicted floor distance moves by `cot(elevation)` for every unit of
error in the estimated floor plane: **2.0 for the steepest row, 15.3 for the
shallowest**. Given a uniform threshold, that one shallow row produced 28 false
obstacles per second on open floor while every other row was clean. Scaling the
margin by cotangent removed all of them without blunting the steep rows.

### Gravity is the wrong reference for a floor

The obvious way to correct for tilt is an IMU. It is also wrong: on a slope the
robot and the surface tilt *together*, so in the robot's own frame the floor has
not moved. Subtracting gravity-referenced pitch would paint every ramp as a
drop-off. The floor is fitted from the sensors themselves instead — measured
within 0.2 mm and 0.07° on level ground.

### A rendering backend can corrupt physics data silently

Gazebo's legacy Ogre1 backend does not fail loudly on range sensors. Cloud
shapes and timestamps stay valid while the ranges are simply wrong — the bottom
sensor row read 89 mm where geometry demanded 64 mm. Every automated shape check
passed. Only comparing against predicted geometry caught it.

### Memory scarcity looks like a logic bug

Orphaned Gazebo servers run under the process name `ruby`, so name-based cleanup
never matched them. Three accumulated at ~640 MB each until the controller
manager began timing out and launch tests failed for reasons that looked nothing
like the cause. There is now a tool that matches on full command lines.

### A near-field sensor must not feed a global costmap

The global costmap lives in the map frame and never rolls, so every transient
mark persists for the entire run. Feeding it a 0.8 m sensor accumulated the
union of every mark ever made until the robot was enclosed by its own history.
It marks the local costmap only.

---

## Project status

**Autonomous navigation works in simulation.** Set an initial pose, give it a
goal, and it drives there avoiding obstacles, repeatedly, behind a latching
emergency stop.

| Phase | Delivered |
| --- | --- |
| **1** | Mecanum simulation and control, wheel odometry, IMU fusion, TF ownership, timestamp contracts |
| **2** | SLAM Toolbox mapping, map serving, AMCL localization |
| **3** | Nav2 costmaps, global planning, command arbitration, e-stop, autonomous driving |
| **4** | ToF perimeter: eight-sensor ring, floor/obstacle classification by class and range, scene shapes, per-sensor power switches, local costmap integration |
| **5** | ToF-only navigation: holonomic DistBug with adaptive standoff, deadlock guards, direction-aware speed levels, a control panel with safety and localisation readouts |
| **6** | Unknown environments: live SLAM (no saved map), laser odometry and gyro-bias correction in the EKF, Nav2 planning through unexplored space, ToF speed governor, DistBug fallback, frontier exploration |

The robot turns in place to face its path, then drives holonomically with
the camera forward. Details in
[ARCHITECTURE.md](ARCHITECTURE.md#heading-turn-to-face-then-drive-holonomically).

### Verified in simulation only

Everything above is simulated. No hardware has been commissioned, and the gap is
not merely untested — it is known to be unfavourable. Gazebo returns a perfect
reflection from every surface, while a real time-of-flight sensor on dark carpet
at a shallow angle returns nothing at all. The ToF results in particular are an
upper bound.

## Goals

- **Next** — a voxel layer (STVL) so low obstacles are remembered after they
  leave the ToF ring's view, e-stop control on the panel, and a radar presence
  sensor for a "person nearby" speed level.
- **Near term** — commission the ToF ring on hardware, per-sensor diagnostics,
  a calibration procedure, and an MCU-side reflex stop independent of the ROS
  graph.
- **Then** — temporal accumulation so obstacles are remembered while in the
  blind spot, and wall-relative alignment for docking, which multizone sensors
  support well and a holonomic base can act on directly.
- **Not pursued** — cliff detection from this geometry. It was built, measured
  and removed; the reasoning is in [ARCHITECTURE.md](ARCHITECTURE.md).

---

### Navigating somewhere it has never been

Measured in `my_world`, starting with no map at all: from the hallway into an
unseen bedroom through a 0.70 m door in 31 s, and across the whole flat to the
far bedroom in 94 s, both by Nav2 alone. SLAM put the robot 3.9 cm and 5.4 cm
from Gazebo's ground truth at the end of those runs.

Two things had to be fixed to get there, both measured on the costmaps. A live
map only covers what has been seen, so a goal in an unseen room was off a
map-sized global costmap and refused outright; the global costmap is now a
window larger than the building. And the LiDAR smears a door jamb seen at a
grazing angle about 15 cm into the opening; a steeper inflation fall-off keeps
the doorway's centre line cheap without allowing contact.

## Documentation

| You want to | Read |
| --- | --- |
| Understand the project | this file |
| Run, drive, map or inspect it | [docs/GUIDE.md](docs/GUIDE.md) |
| Look up a frame, topic, measurement or derivation | [ARCHITECTURE.md](ARCHITECTURE.md) |

## Third-party sources

Laser odometry (rf2o) and frontier exploration (explore_lite) are built from
source, pinned in `mobile_base.repos`:

```bash
vcs import ~/ros2_ws/src/external < ~/ros2_ws/src/mobile_base/mobile_base.repos
cd ~/ros2_ws && colcon build --base-paths src/external --packages-select rf2o_laser_odometry explore_lite_msgs explore_lite
```

Both are optional: without them everything builds and runs, the EKF simply
has one input fewer, and exploration is unavailable. The voxel layer comes
from apt: `sudo apt install ros-jazzy-spatio-temporal-voxel-layer`.

## Packages

| Package | Responsibility |
| --- | --- |
| `mobile_base_description` | Xacro model, SI geometry, inertials, RViz configuration |
| `mobile_base_gazebo` | Gazebo Harmonic worlds |
| `mobile_base_bringup` | ros2_control configuration and top-level launch |
| `mobile_base_localization` | EKF, SLAM Toolbox, map serving, AMCL |
| `mobile_base_navigation` | Costmaps, planning, control, behavior tree, arbitration, e-stop |
| `mobile_base_tools` | ToF classification, visualization, motion checks, cleanup |

## Continuous integration

`.github/workflows/ci.yaml` builds all seven packages on ROS 2 Jazzy and runs the
deterministic unit, lint, Xacro/URDF, configuration, Python compilation, and
whitespace checks. Gazebo launch tests and the formal campaign stay out of
normal pull-request CI; they remain explicit runtime validation on a stable
host.
