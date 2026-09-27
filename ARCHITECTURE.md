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
visual per printed part - two 10 mm deck plates, four corner standoffs, and the
battery and boards that ride on them - plus a box collision per deck limb.
Splitting the visuals while keeping collision closed is deliberate: the planner
must never see the open electronics bay between the decks as passable geometry.

Each deck is the full body rectangle with the four wheel wells cut out of it
and the four bumper corners chamfered at 45 degrees from the wheel line:

| Feature | Extent |
| --- | --- |
| Overall body | 0.260 m x 0.1676 m |
| Bumper edge, after the chamfer | 0.1271 m wide |
| Central spine | 0.2195 m x 0.100 m |
| End caps, fore and aft | 0.0203 m deep, full body width |
| Side wings, one per side | 0.0805 m x 0.0338 m |

The end caps run the full body width 20 mm in front of and behind the wheels,
so nothing strikes a wheel head on without hitting deck first. The wings fill
the space between each side's two wheels. Both reach out to the wheel outer
faces, and every limb stands `wing_clearance_x` clear of the wheels, leaving
four open wheel wells. Every bound derives from the wheel geometry, so moving a
wheel moves the deck outline with it.

### The plate is a generated mesh

The chamfer is why. URDF's primitives are box, cylinder, sphere and mesh, so a
45 degree face cannot be expressed as boxes the way the rest of the outline
can. Both decks are identical, so one mesh serves both:

- `mobile_base_description/scripts/generate_deck_mesh.py` emits
  `meshes/decks/deck_plate.stl`, reading its dimensions back through xacro
  rather than restating them, so the mesh is built from exactly what the URDF
  is built from.
- The `verify_deck_mesh` test regenerates and compares, so editing the body
  geometry and forgetting the mesh fails the suite rather than shipping a
  plate that no longer matches its own properties.
- The surface is watertight: the tiling is split at every join so no T-junction
  is left where a long edge meets a short one.

Collision stays boxes - one per limb, five in all. Boxes are cheaper and
steadier in contact than a mesh, and they keep square bumper corners where the
plate is chamfered, so collision over-claims two small triangles at each end.
Erring outwards is the safe direction for a footprint. A single box over the
whole envelope, by contrast, would reach the wheel outer faces along the entire
body length and swallow the wheels.

The collision boxes' circumscribed radius is the square cap corner at
0.1547 m, so Nav2's `robot_radius` is 0.16 m in `planner.yaml` and `local_costmap.yaml`. Raising
`chassis_length` pushes that corner further out and those two values must move
with it.

The vertical stack hangs off the wheel radius. The motors stand on the lower
deck and drive the wheels directly, so the deck's top face is pinned exactly
one motor shaft offset below the wheel axle:

| Surface | Height above ground |
| --- | --- |
| Lower deck underside (ground clearance) | 0.0095 m |
| Lower deck top, motors stand here | 0.0195 m |
| ToF apertures | 0.0285 m |
| Motor driver tops | 0.0312 m |
| Battery top | 0.0445 m |
| Motor tops | 0.0419 m |
| Upper deck underside, IMU hangs here | 0.0595 m |
| Upper deck top, LiDAR and MCU stand here | 0.0695 m |
| LiDAR scan plane | 0.0915 m |
| Overall height | 0.0995 m |

The camera is mounted on the front face of the upper deck, centred on the
plate's thickness. The LiDAR is a square dark base carrying a cylindrical
rotating head, and its collision is one box bounding both.

Eight VL53L7CX Time-of-Flight sensors stand on the lower deck, on the Pololu
#3418 carrier at its quoted 13 x 18 x 3 mm - one at the centre of each body
face, and one on each of the four bumper chamfers.
Each link origin is the optical aperture rather than the centre of the board,
the same convention `lidar_link` follows: the aperture is where rays leave, so
it is what every consumer needs. The board hangs inboard behind it, standing
upright, and clears the upper deck underside by 22 mm.

The LiDAR sees one horizontal slice at 0.0915 m. These sit at 0.0285 m and see
what that slice cannot - the near field, and the floor. Each reports an 8 x 8
grid of zones over a 60 x 60 degree field, so the bottom rows land on the floor
just ahead of the body while the top rows watch for obstacles.

The side apertures sit exactly at `wheel_outer_y` and must stay there. The wing
outer face is coplanar with the wheel outer faces, so every ray in the fan keeps
a positive outward component and grazes past the wheel. Recessing the aperture
even a board thickness inboard puts the outer zones of the fan into the wheel
instead, which is why the sensor test asserts that position as an equality.

The four cardinal sensors alone would cover 240 degrees of 360 and leave 30
degree wedges on the diagonals - which is exactly where a mecanum base travels
when it strafes at 45 degrees. The four corner sensors close those wedges. Eight
fans of 60 degrees spaced 45 degrees apart cover the full circle with 15 degrees
of overlap at every seam, verified in simulation against the published
transforms rather than assumed from the nominal geometry.

The corners sit at the midpoint of the 45 degree bumper chamfer, whose outward
normal is the diagonal itself, so each looks along a strafe direction and sits
on the part of the body that reaches an obstacle first in that motion. The
chamfer is a visual-mesh feature - the deck collision boxes keep square corners
and over-claim those triangles - which costs nothing here because `gpu_lidar`
renders the visual scene rather than the collision geometry, so the surface
these apertures sit on is the surface Gazebo traces against.

The 15 degree overlaps mean adjacent fans illuminate the same volume. In
simulation that is free; on hardware the eight 940 nm emitters would need
ranging in alternating banks to keep them from reading each other.

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
untouched. Motors carry no collision, both because the body box already covers
them and because an extra collision would break the one-surface-per-wheel
contract the SDF generator enforces. Their inertia sits at the body centroid
rather than at the link origin, which is the output shaft nearly 30 mm away.

## Mounted components

The battery and the boards ride on the decks as fixed-joint children of
`base_link`. Each link origin is the part's own centroid, so its inertial sits
at the link origin, the Gazebo reduction lumps it in with the right
parallel-axis term, and every component frame is where its mass actually is.

| Part | Deck | Centre (x, y) | Footprint | Mass |
| --- | --- | --- | --- | --- |
| 2S LiPo | lower | (0, 0) | 0.138 x 0.047 m | 0.250 kg |
| TB6612FNG driver, two | lower | (0, +/-0.045) | 0.020 x 0.0204 m | 0.003 kg |
| ESP32-S3 devkit | upper, on top | (-0.085, 0) | 0.0605 x 0.0252 m | 0.012 kg |

The battery is the single heaviest part and sits low and central, pulling the
centre of mass down rather than sideways. The drivers flank it in the mid-band
where no motor reaches. The MCU sits at the rear of the upper deck with its
USB-C facing aft, so it can be flashed without taking the robot apart.

The lower deck keeps two bays deliberately empty, claimed by properties so
later geometry cannot creep into them: the front bay for the buck converters
and the rear bay for the main power switch and the battery voltage divider.

The board meshes are authored neither centred nor axis-aligned to the robot, so
each instance carries a visual pose that seats the mesh's own bounding-box
centre on the link origin - the same per-instance convention the wheel and
motor macros use. Inertia comes from the measured bounding box rather than the
mesh: a board is close enough to a uniform slab.

### Mass budget

`total_body_mass` is `base_link`'s mass after the Gazebo reduction has lumped in
every fixed-joint child. Components carry their real masses and the two deck
plates absorb the balance, so the total is held by construction rather than by
a constant that goes stale whenever a part is added. The plates are the right
place for it: they also stand in for the fasteners, wiring, and the reserved
power components.

| Item | Mass |
| --- | --- |
| Battery | 0.250 kg |
| Four TT gearmotors | 0.120 kg |
| Two TB6612FNG drivers | 0.006 kg |
| ESP32-S3 devkit | 0.012 kg |
| LiDAR, IMU, camera | 0.190 kg |
| Eight VL53L7CX carriers | 0.004 kg |
| Two deck plates, the balance | 1.218 kg |
| **`total_body_mass`** | **1.800 kg** |

The wheels hang off revolute joints, so they are not lumped and are not counted
here. The resulting centre of mass sits 0.0415 m above ground on the
centreline, 1.3 mm lower than before the battery was modelled.

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

## Units and CAD source

URDF and controller values use SI units: metres, kilograms, seconds, and radians.
The original CAD file remains at `src/3D_Builds/ROS2_transfer.stl` and is not loaded
at runtime. The base model uses one parametric body link. The mecanum wheels use four
position-specific STL files for visual appearance and one cylinder collision per
driven wheel for contact.

The wheel meshes live in `src/mobile_base/meshes/wheels/` and are installed through
the description package. Gazebo receives runtime-generated `file://` mesh URIs from
the installed `mobile_base_description` share directory.

The four STL files were exported in assembly coordinates, so `base.xacro` gives every
wheel an explicit position-specific visual rotation and translation. Each rigid
transform flips the exported mounting face inward toward the chassis and keeps the
mesh axle centre coincident with the driven-wheel link origin. The flip is composed
about a mesh-local transverse axis aligned with that wheel's measured roller phase;
this preserves the visual roller phase set and mecanum X-pattern. These visual
transforms are independent of collision geometry, controller configuration, and
odometry.

## URDF organization

The main assembly file is
`mobile_base_description/urdf/mobile_base.urdf.xacro`. It should stay readable at the
robot level: arguments, includes, base assembly, camera, ros2_control, and optional
Gazebo plugin/sensor instantiation.

| File | Responsibility |
| --- | --- |
| `properties.xacro` | Shared robot and sensor dimensions, masses, poses, rates, ranges, and noise |
| `scripts/generate_deck_mesh.py` | Generates the chamfered deck plate mesh from `properties.xacro`; `--check` guards against drift |
| `materials.xacro` | Named visual materials |
| `inertials.xacro` | Reusable inertial macros |
| `chassis.xacro` | `base_footprint`, `base_link`, the deck plate mesh visuals, and the bounding collision limbs |
| `macros/mecanum_wheels.xacro` | Driven wheel links, visual meshes, cylinder collisions, inertials, and joints |
| `macros/motors.xacro` | TT gearmotor links, visual mesh, and fixed joints |
| `macros/components.xacro` | Deck-mounted battery and board links, visuals, inertials, and fixed joints |
| `base.xacro` | Chassis plus the four wheel and four motor assemblies, the battery, the two motor drivers, and the MCU |
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
    |-- front_right_wheel_link
    |-- rear_right_wheel_link
    |-- rear_left_wheel_link
    |-- lidar_link
    |-- imu_link
    `-- camera_link
        `-- camera_optical_frame
```

The wheel meshes retain visible mecanum rollers, but the simulation creates no
passive roller links or joints.

## Dependencies

Install dependencies after sourcing ROS 2 Jazzy:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

The Gazebo launch requires `ros_gz_sim`, `ros_gz_bridge`, and `gz_ros2_control`.

## Mecanum wheel contact

Gazebo models each driven wheel with one cylinder collision. After Xacro is
converted with `gz sdf -p`, `generate_sim_sdf.py` injects the canonical
anisotropic surface into each of the four collisions:

- `mu=0.8` along the handed `fdir1` direction;
- `mu2=0.2` across that direction;
- zero `slip1` and `slip2`; and
- `fdir1` expressed in `base_footprint`, so it does not rotate with the wheel.

The direction signs form the mecanum X pattern: front-left and rear-right use
`1 -1 0`; front-right and rear-left use `1 1 0`. The generator validates
all four wheel links, joints, cylinder dimensions, surfaces, and direction
frames before writing the SDF. It also rejects passive roller bodies.

This single-cylinder approximation preserves the visual wheel meshes and normal
controller interfaces while reducing wheel contact topology to four collisions.
There is no runtime contact-model selector. The validated constants are internal
to the generator rather than public launch arguments.

The wheel centers have a literal half-wheelbase plus half-track projection of
`0.142 m`. Directional cylinder contact produces an effective rotational
projection of `0.12521 m`, measured symmetrically from clockwise and
counter-clockwise ground-truth runs. The mecanum controller uses the effective
value for rotational inverse kinematics and wheel odometry. Wheel radius remains
`0.03074443 m`, so the calibration does not alter forward, lateral, or diagonal
kinematics.

That effective value belongs to the contact model, not to the robot. Re-measure
it if the wheel contact parameters or geometry change, and do not carry it to
hardware: it corrects a simulation artifact a physical base does not have, so
hardware commissioning starts from the geometric `0.142 m`. Method and measured
residuals are in
[`docs/mecanum_motion_accuracy.md`](docs/mecanum_motion_accuracy.md).

Standard workflows are therefore:

```bash
ros2 launch mobile_base_bringup simulation.launch.py
ros2 launch mobile_base_bringup mapping.launch.py world:=navigation_basic
ros2 launch mobile_base_bringup localization.launch.py \
  world:=navigation_basic \
  map:=/home/safwan/ros2_ws/src/mobile_base/maps/navigation_basic.yaml
```

The visual roller geometry remains useful for appearance and for interpreting
the friction direction, but it does not create links, joints, collisions, or
additional ros2_control state interfaces.

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

Two self-contained worlds are installed; neither downloads external models:

```bash
ros2 launch mobile_base_bringup simulation.launch.py world:=navigation_basic
ros2 launch mobile_base_bringup simulation.launch.py world:=my_world
```

`navigation_basic` is a 10 x 8 m room with walls, routes, boxes and a column,
and is the default. `my_world` is a 10 x 10 m sealed apartment: six rooms off a
central hall, an L-shaped kitchen and living room, and doorways of 1.20, 1.00,
0.90, 0.80, 0.70 and 0.60 m. Every point on its floor is within LiDAR range of
walls on both axes, which is what makes it mappable. An absolute custom file
works too:

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
  world:=my_world localization:=true rviz:=true
```

Raw-controller diagnostic mode:

```bash
ros2 launch mobile_base_bringup simulation.launch.py \
  world:=navigation_basic localization:=false rviz:=true
```

## Phase 1 runtime contracts

With a fused simulation running, validate the configured IMU (50 Hz), LiDAR
(10 Hz), controller odometry (100 Hz), and filtered odometry (50 Hz) contracts:

```bash
ros2 run mobile_base_tools timestamp_validator \
  --mode fused --duration 10 \
  --output diagnostics/timestamp_fused.json \
  --ros-args -p use_sim_time:=true
ros2 run mobile_base_tools tf_validator \
  --mode fused --duration 10 \
  --output diagnostics/tf_fused.json \
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

## Stable interfaces

- Command: `/mobile_base_controller/reference` (`geometry_msgs/msg/TwistStamped`)
- Odometry: `/mobile_base_controller/odometry`
- Joint states: `/joint_states`
- TF: `odom -> base_footprint -> base_link`
- Navigation goal in: `/goal_pose` (`geometry_msgs/msg/PoseStamped`)
- Planned path out: `/plan` (`nav_msgs/msg/Path`)
- Teleop into the mux: `/cmd_vel_teleop` (`geometry_msgs/msg/TwistStamped`)
- E-stop lock: `/safety/estop_active` (`std_msgs/msg/Bool`)

`twist_mux` is the **only** node permitted to publish the command topic. Every
source feeds the mux instead; publishing to the controller reference directly
bypasses both arbitration and the emergency stop. That single-arbiter rule is a
tested property, not a convention.

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

## The ToF perimeter

Eight VL53L7CX carriers on the lower deck, apertures at 28.5 mm, each reporting
an 8x8 zone grid over 60 x 60 degrees. Mounting and coverage are under *Body
geometry*; this section is what the geometry means for perception.

### Why the ring is a near-field sensor, permanently

The four downward rows terminate on the floor by 435 mm. Past that the lowest
ray still in flight is the one at +3.75 degrees, climbing 65.5 mm per metre, and
it clears the robot's own 99.5 mm roof at **1.083 m**. Beyond that distance
nothing the ring can see could obstruct this robot, and anything tall enough to
matter also crosses the LiDAR plane where it is resolved fifteen times finer.
Marking ranges follow from it (see *Marking by class*). It is a mounting-height consequence,
not a tuning choice: 28.5 mm buys near-field floor visibility and costs
far-field low-obstacle range, and the two are in direct tension.

### Floor sampling and why row 3 is treated differently

| Row | Pitch | Floor intercept | cot(elevation) |
| --- | --- | --- | --- |
| 0 | -26.25 deg | 57.9 mm | 2.0 |
| 1 | -18.75 deg | 84.1 mm | 2.9 |
| 2 | -11.25 deg | 143.5 mm | 5.0 |
| 3 | -3.75 deg | 435.5 mm | **15.3** |

Rows 0-2 fit the floor plane. Row 3 is excluded from the fit and heavily
derated in the classifier, and the cotangent column is why: a ray's predicted
floor distance moves by `cot(elevation)` for every unit of error in the fitted
plane, so row 3 is seven times more sensitive to plane error than row 0.

### Classification

The floor is **fitted from the sensors** every frame, pooling the steep near
rows of all eight. Measured on a level floor the fit lands within 0.2 mm of
zero at 0.07 degrees of tilt. It is not taken from the IMU: on a slope the robot
and the surface tilt together, so in the robot's own frame the floor has not
moved, and a gravity-referenced attitude would report a slope that is not there.
The EKF could not supply it either - it runs in `two_d_mode`, which forces pitch
and roll to zero.

Two tests then decide each zone, and both are needed:

- **Height above the fitted plane.** Above `obstacle_min_height` is an obstacle;
  above the robot's roof is clearance it drives under.
- **Range shortfall**, with a margin scaling as `expected * cot(elevation)`. A
  ray that was going to meet the floor and stopped short hit something standing
  in between. This is the only test that catches a low obstacle's vertical face,
  because a downward ray can never report a height above the aperture it left.

The cotangent scaling is the difference between the ring working and not.
Measured on open floor with a flat threshold, row 3 alone produced **28 false
obstacles per frame**, every one of them 0.5-0.9 m out and within 4 mm of the
ground, while rows 0-2 were clean. Scaling the margin by cot removed all of
them without touching the steep rows' sensitivity.

### Marking by class

`select_marks` in `tof_floor_model.py` gives each kind of return its own range,
measured horizontally from the sensor:

| Kind | How it is recognised | Marked to |
| --- | --- | --- |
| Low obstacle | Range shortfall only, at most `obstacle_min_height` above the floor | 0.4 m |
| Obstacle | Above `obstacle_min_height`, below the roof | 1.0 m |
| Wall | A column of upward zones at one distance, starting from the lowest upward row | 2.0 m, any height |

The wall rule is also what keeps overhangs out: a table top returns only in the
upper rows while the lowest upward row sees past it, so it is never marked.
Everything the classifier still publishes above the roof is wall, which is why
the costmap's ToF source has no low height cut (`max_obstacle_height: 1.2`).

The classifier also publishes `/tof/obstacle_zones`, a `UInt8MultiArray` mask
with one row per face, so displays can colour exactly the zones it marked.

### Stability

A zone must read as an obstacle for `confirm_frames` consecutive frames before
it is published, and is held for `hold_frames` after it stops. Consecutive
frames originally agreed on only 38 percent of their marks; a costmap fed that
churn accumulates the union of every transient mark. Positions are additionally
low-pass filtered, because 10 mm of range noise moves a mark about a centimetre
per frame even when the zone is steadily looking at the same wall.

### Sensor power

The Gazebo bridge publishes the LiDAR and every ToF sensor under
`/sensors/raw`. `sensor_power` relays the enabled ones, still serialized, onto
`/scan` and `/tof/<face>/points`, so nothing downstream knows switches exist. A
switched-off sensor goes silent as unpowered hardware would, and one empty
message is published so RViz and the classifier drop its last frame. Switches
are the boolean parameters `enabled.<sensor>`; state and measured rates go out
on `/sensors/status` as diagnostics. `sensor_panel` is the window that drives
them. Gazebo still renders a switched-off sensor, so switching off does not
speed the simulation up.

### Scene shapes and colours

`tof_scene_shapes` turns `/tof/obstacles` and `/tof/floor` into `/tof/shapes`:
clusters are split at corners into straight runs, long thin runs become wall
slabs, the rest object boxes, and collinear wall pieces split at a sensor seam
are joined again. The floor is one region, the furthest confirmed floor per
7.5° bearing bin, smoothed over time. Shapes are tracked across frames like
zones are.

`tof_palette.py` holds every ToF colour. Colour carries meaning, never sensor
identity: slate for raw returns and rays, green for floor, and warning bands
for obstacles by distance (red < 0.25 m, amber < 0.5 m, blue beyond). A ray
turns orange only when the classifier marked its zone.

A known weakness: a wall seen at a grazing angle is sampled in columns up to
0.8 m apart along its length, and those can be drawn as separate tall pieces.

### Costmap coupling

The ring marks the **local costmap only**. The global costmap lives in the map
frame and never rolls, so a near-field sensor feeding it accumulates marks for
the whole run and the robot ends up enclosed by its own history.

### What was removed, and why

**Cliff detection.** The design was sound - a missing return from a ray aimed at
reachable floor means the floor is gone - but the geometry does not support it
here. The rows with the reach to see an edge early are the shallow ones, and
those are exactly the rows whose predicted floor distance is least trustworthy.
The result marked open floor as often as it marked edges, and a costmap treating
those as lethal trapped the robot. Reinstating it needs dedicated downward
sensors at near-normal incidence, not shallow rays from a 28.5 mm aperture.

### Limits worth stating

- **Simulation flatters this badly.** Gazebo returns a perfect reflection from
  every surface. A real VL53L7CX on dark carpet at a shallow angle returns
  nothing at all. Nothing in simulation can validate behaviour on real
  materials.
- **Detection range is short and deliberately so.** The shallow row's margin is
  wide enough that it will not mark a low obstacle until the robot is close.
  That is the price of not marking open floor, and it was paid knowingly.

## ToF-only navigation

`bug_navigation.launch.py` runs the simulation with the LiDAR switched off
plus two nodes; no map, SLAM or Nav2.

```
/goal_pose, /odometry/filtered, /tof/obstacles
        -> bug_navigator --/governor/cmd_vel--> speed_governor
        --/mobile_base/cmd_vel--> velocity_smoother -> mobile_base_controller
```

The logic is ROS-free in `bug_model.py` and tested on a kinematic robot.

**Navigator.** Heading is held; it only translates. Motion to goal until the
body's swept corridor is blocked within `hit_distance`; then boundary following
by a vector field - tangent to the nearest point of the followed boundary plus
a pull toward a standoff. Only points within 100° of the last contact
direction count, which keeps it on one boundary round a wall end. The standoff
is `standoff` (0.30 m) with the far side open and half the available space as
it narrows, never below `min_standoff` (0.05 m). DistBug leaves when
`d(x, goal) - F <= d_min - step` or the goal is in view, F being the free range
toward the goal down a swath as wide as the standoff, capped at the 1.0 m
obstacle marking range - an empty corridor means nothing seen, not nothing
there. Bug2's m-line rule is kept behind `algorithm: bug2`.

**Governor.** Caps translation by level for the nearest wall and the nearest
other obstacle in the swept corridor, and for the gap beside the body. Walls
are told apart by the classifier's wall rule: anything above the roof is wall,
so a point sharing a column with one is wall too. Levels satisfy
`2 (v t + v^2 / 2a) <= d - 0.05` at the near edge of each band (t = 0.35 s,
a = 1.0 m/s^2). No ToF data for 0.5 s stops the robot. The STOP latch
(`/speed_governor/stop`) holds its output at zero; `/speed_governor/status`
reports level, cause and the nearest obstacle for the panel.

## Localization inputs

The EKF (`mobile_base_localization/config/ekf.yaml`) fuses three velocity
sources and no absolute pose; absolute correction comes from AMCL or SLAM.

| Input | Source | Fused |
| --- | --- | --- |
| `odom0` | wheel odometry `/mobile_base_controller/odometry` | vx, vy, yaw rate |
| `odom1` | `/laser_odometry` (rf2o via `laser_odometry.py`) | vx, vy, yaw rate |
| `imu0` | `/imu/data_unbiased` (`imu_bias.py`) | yaw rate |

- **rf2o** matches consecutive scans. Its ROS 2 node fixes the sideways
  velocity at zero, which would tell the EKF a mecanum base never strafes, so
  `laser_odometry` differentiates rf2o's pose into body velocities, attaches
  covariances (rf2o leaves them zero, i.e. "perfect") and drops matches faster
  than the base can move. Off unless `laser_odometry:=true`, because rf2o is a
  source build (`mobile_base.repos`).
- **imu_bias** learns the gyro offset whenever the wheels report the base
  still for a second, and subtracts it. Heading is the error the wheels cannot
  correct, since mecanum rollers slip in rotation.
- **`dynamic_process_noise_covariance: true`** scales process noise with
  velocity. With it false the pose covariance grew without bound at rest,
  passing a metre in about 20 s; now it grows only while moving.

Acceleration and the IMU's absolute orientation are deliberately not fused:
integrated twice, accelerometer bias becomes metres of error within seconds,
and a 6-axis IMU has no absolute heading.

## Unknown-environment navigation

`explore_navigation.launch.py` wraps `planning.launch.py` with
`localization:=slam` and `speed_governor:=true`:

```
slam_toolbox (mapping, live)   --map -> odom-->  Nav2 on the growing /map
controller / behaviours -> /cmd_vel_ungoverned -> speed_governor
    -> cmd_vel_nav -> velocity_smoother -> collision_monitor -> twist_mux
nav_supervisor: Nav2 aborted -> goal (map -> odom) to DistBug
                DistBug round the blockage -> goal back to Nav2
explore_lite (explore:=true): frontier goals until the map is complete
```

- **`localization:=slam`** includes the bringup mapping stack instead of
  map_server + AMCL, starts the navigation servers at once (no initial pose to
  wait for) and loads `planner_unknown.yaml` last on the planner.
- **`planner_unknown.yaml`**: `allow_unknown: true`; the global costmap becomes a
  20 m rolling window, because a static costmap is sized to the map and a live
  map covers only what has been seen; inflation `cost_scaling_factor: 8` so a
  doorway narrowed by LiDAR jamb smear keeps a cheap centre line.
- **The governor** sits before the smoother, so Nav2's commands are capped by
  the ToF along the direction of travel exactly like DistBug's, and the
  smoother still ramps whatever it allows.
- **The supervisor** watches `/navigate_to_pose/_action/status` alongside
  bt_navigator (which takes `/goal_pose` itself); a lingering status of the
  goal that already aborted is ignored, so a hand-back cannot re-trigger it.
  DistBug listens on `/bug_navigator/goal` in this mode and drives into
  `/cmd_vel_ungoverned`.
- The e-stop latches at startup as in every Nav2 launch.

### DistBug deadlock guards

`bug_model.Bug2.step` wraps the algorithm: a stall watchdog (commanding motion,
< 5 cm in 5 s) backs off 0.2 m and retries; leaving must beat the best
distance to goal of the whole trip by a step, so leave/re-hit cannot cycle;
returning to a hit point flips the follow side once before UNREACHABLE; a
direction reversing four times within two seconds is held; three recoveries
end in STUCK. Each is covered by a kinematic test.

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
- Ogre2 is required for usable simulated LiDAR and ToF; Ogre1 is retained only
  for scan-independent software-rendered checks. The ToF case is worse than the
  LiDAR's, because Ogre1 fails quietly rather than obviously: the clouds keep
  their shape and their timestamps, so topic-level checks pass, but the ranges
  are wrong. Measured against a flat floor under Ogre1 the bottom zone row read
  89 mm where geometry demands 64 mm, and the top row pinned to the near limit
  against nothing at all. Under Ogre2 every row lands within a millimetre of
  prediction. Any range read off an Ogre1 run is not evidence.
- ToF visualisation is timing-sensitive in RViz. The clouds are produced by
  Gazebo and the `odom` transform by the EKF behind it, so 89 percent of clouds
  arrive stamped ahead of the newest transform, by 9.5 ms on average. RViz drops
  what it cannot transform at the stamp, so a cloud display with zero decay time
  blinks. The ray markers publish with a zero stamp, which asks for the latest
  transform instead, and the cloud displays carry a 0.2 s decay time to cover
  the dropped frames the way the LiDAR's 0.25 s already does.
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
| Change what the ToF ring marks | `tof_floor_model.py` thresholds + `test_tof_floor_model.py` + the local costmap source |
| Test perception behaviour | `world:=my_world`, see [docs/GUIDE.md](docs/GUIDE.md) |
| Change TF ownership | simulation spawners + EKF config + TF validator tests |
| Add a motion profile | `mobile_base_tools/config/odometry_tests.yaml` |
| Change costmap or planner tuning | `mobile_base_navigation/config/`, then `test_navigation_config.py`; iterate with `planning_harness.launch.py` |
| Change how the robot drives | `mobile_base_navigation/config/controller.yaml`, then `test_navigation_config.py` |
| Add a command source | `config/twist_mux.yaml`; feed the mux, never the controller reference directly |
