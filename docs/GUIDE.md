# GUIDE — running, driving and inspecting the robot

Every command lives here. [README.md](../README.md) explains what the robot is
and why it works the way it does; [ARCHITECTURE.md](../ARCHITECTURE.md) is the
reference for frames, topics, measurements and derivations.

## Two rules before anything else

**Always render with `ogre2`.** The legacy Ogre1 backend does not fail loudly on
range sensors — cloud shapes and timestamps stay valid while the numbers are
wrong. Any range read off an Ogre1 run is not evidence.

**Always stop cleanly when you are done.** Leftover Gazebo servers hold memory
until unrelated things start failing:

```bash
ros2 run mobile_base_tools stop_simulation
```

## Contents

| Part | What it covers |
| --- | --- |
| [0](#part-0--stopping-cleanly-and-running-the-tests) | Stopping cleanly, running the tests |
| [1](#part-1--build-and-first-run) | Build, launch a simulation |
| [2](#part-2--drive-map-localize-navigate) | The full workflow, start to finish |
| [3](#part-3--mapping-and-saved-map-localization-in-depth) | Mapping and AMCL in depth |
| [4](#part-4--costmaps-and-path-planning-in-depth) | Costmaps and planning in depth |
| [5](#part-5--driving-it-by-hand) | Teleoperation |
| [6](#part-6--repeatable-motion-checks) | Open-loop motion accuracy checks |
| [7](#part-7--the-tof-perimeter) | The ToF ring: running it, proving it works |
| [8](#part-8--inspecting-a-running-robot) | Introspecting frames, topics, parameters |
| [9](#part-9--tof-only-navigation) | Driving to a goal on the ToF ring alone: DistBug, speed levels, the control panel |
| [10](#part-10--navigating-an-unknown-environment) | Live SLAM: goals in places never mapped, exploration, both RViz views |

---

## The whole thing in one pass

If you only want to see it work, this is the shortest complete path from a fresh
clone to the robot driving itself. Each step is expanded later in the guide.

```bash
# 1. Build once.
cd ~/ros2_ws && colcon build && source install/setup.bash

# 2. Map a world. Drive it around with teleop until the map looks complete.
ros2 launch mobile_base_bringup mapping.launch.py world:=navigation_basic render_engine:=ogre2

# 3. In a second terminal, save the map, then stop the first terminal.
ros2 run nav2_map_server map_saver_cli \
  -f "$HOME/ros2_ws/src/mobile_base/maps/navigation_basic" \
  --ros-args -p save_map_timeout:=10000.0

# 4. Navigate. Set "2D Pose Estimate" in RViz, then "2D Goal Pose".
ros2 launch mobile_base_navigation planning.launch.py \
  world:=navigation_basic \
  map:="$HOME/ros2_ws/src/mobile_base/maps/navigation_basic.yaml" \
  render_engine:=ogre2

# 5. Always finish here.
ros2 run mobile_base_tools stop_simulation
```

Two things that catch people out: **navigation does nothing until you set an
initial pose** (or pass `autostart:=true`), and the **e-stop is engaged at
startup** by design — Part 4 covers releasing it.

---

## Part 0 — Stopping cleanly, and running the tests

Run this after every session, and any time the machine feels slow:

```bash
ros2 run mobile_base_tools stop_simulation
```

Add `--dry-run` to see what it would kill without killing it.

**Do not try to do this with `pkill` on a name.** Three separate traps make that
unreliable, and each one has cost real debugging time here:

- The Gazebo server's process name is **`ruby`**. Only its command line says
  `gz sim`, so `pkill gz` never matches it. Three of these once accumulated at
  ~640 MB each and quietly starved the controller manager until launch tests
  began failing with no obvious cause.
- `robot_state_publisher` is started with a generated parameter file, so its
  command line contains **no package name at all** - only
  `/tmp/launch_params_<random>`. No package-shaped pattern will find it.
- Gazebo does not always honour SIGINT. The launch logs say so outright:
  *"failed to terminate 5 seconds after receiving SIGINT"*.

A launch started with `setsid` or `nohup` also outlives the terminal that
started it, so closing the window leaves the whole stack running.

Leftovers are not harmless. They hold memory, and once memory is tight the
controller manager times out waiting for `robot_description` and the launch
tests fail for reasons that look nothing like the actual cause.

### Running the full test suite

On this machine, use a sequential executor. In parallel the Gazebo launch test
competes for memory with the other packages and fails spuriously:

```bash
colcon test --executor sequential && colcon test-result --verbose
```

---

## Part 1 — Build and first run

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select \
  mobile_base_description mobile_base_gazebo mobile_base_localization \
  mobile_base_tools mobile_base_bringup
source install/setup.bash
xacro src/mobile_base/mobile_base_description/urdf/mobile_base.urdf.xacro > /tmp/mobile_base.urdf
check_urdf /tmp/mobile_base.urdf
ros2 launch mobile_base_description display.launch.py
```

This command is only for inspecting the URDF without Gazebo. It uses a headless
joint-state publisher by default so every movable link remains visible. To show the
optional joint sliders, add `use_joint_state_gui:=true` and keep that GUI open.

### Launching a simulation

```bash
ros2 launch mobile_base_bringup simulation.launch.py
```

`world` accepts an installed world name (with or without `.sdf`) or an absolute
SDF path. The launch also exposes `use_sim_time`, `gui`, `rviz`,
`start_controller`, `localization`, `render_engine`, `velocity_smoother`,
`physics_max_step_size`, `x`, `y`, `z`, and `yaw`. Mecanum contact is canonical
and has no runtime model selector or contact-tuning launch arguments.
Phase 1 intentionally supports one un-namespaced robot. A misleading partial
`namespace` argument was removed rather than implying multi-robot support.

Wheel appearance comes from the full position-specific mecanum wheel meshes.
Wheel-ground contact always uses the canonical model described below.

---

## Part 2 — Drive, map, localize, navigate

The complete path, in order, with every command. Each stage builds on the one
before it. The repository ships no map, so you must run stages 2 and 3 once
before navigation will work at all.

Build and source once per shell:

```bash
cd "$HOME/ros2_ws" && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install && source install/setup.bash
```

### Stage 1 — drive it by hand

No map, no localization, no navigation. Just the robot, so you can drive it by
hand and confirm the base works.

```bash
ros2 launch mobile_base_bringup simulation.launch.py world:=my_world
```

In a second terminal, drive it. Teleop needs its own terminal because it reads
the keyboard directly:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p frame_id:=base_link -p speed:=0.15 -p turn:=0.5 -p use_sim_time:=true -r cmd_vel:=/mobile_base_controller/reference
```

Use `i` and `,` for forward and reverse, `j` and `l` to rotate in place. Press
`k` to stop — releasing a key does **not** stop the robot, because
`teleop_twist_keyboard` sends one message per keypress and there is no
key-release event. The robot coasts until the controller's `reference_timeout`
of 0.5 s expires, about 7.5 cm at the default speed.

Do not use `u`, `o`, `m` or `.` for mecanum diagonals — those keys combine
translation with rotation. To strafe, publish directly:

```bash
ros2 topic pub -r 20 /mobile_base_controller/reference geometry_msgs/msg/TwistStamped "{header: {frame_id: base_link}, twist: {linear: {y: 0.1}}}"
```

Two worlds ship. `my_world` is a 10 x 10 m sealed apartment: six rooms off a
central hall, doorways from 1.20 m down to 0.60 m. `navigation_basic` is a
10 x 8 m room with an interior wall and obstacles, and it is the one with a
maintained saved map.

### Stage 2 — map the world

Stop stage 1 first. Mapping and localization are mutually exclusive by design.

```bash
ros2 launch mobile_base_bringup mapping.launch.py world:=navigation_basic
```

RViz opens showing the live map. Drive with the same teleop command as stage 1,
in a second terminal. How you drive determines whether the map is usable:

- **Hug the perimeter**, within 2–2.5 m of a wall. The LiDAR reaches 4.0 m and
  the room is 10 x 8 m, so the middle is a dead zone where the scan matcher has
  almost nothing to match against.
- **Go slowly**, especially in rotation. Above roughly 0.3 m/s the map smears.
- **Prefer forward and rotate over strafing.** Lateral odometry is this base's
  least accurate axis and the scan matcher uses it as its prior.
- **Close small loops often** rather than one big loop at the end.

### Stage 3 — save the map

With mapping still running, in a third terminal:

```bash
ros2 run nav2_map_server map_saver_cli -f "$HOME/ros2_ws/src/mobile_base/maps/navigation_basic" --ros-args -p save_map_timeout:=10000.0
```

That writes `navigation_basic.pgm` and `navigation_basic.yaml`. Confirm the
YAML says `resolution: 0.050` — the costmaps are pinned to 0.05 m and a
mismatch causes resampling artifacts. Then stop mapping.

`maps/` is gitignored, so the map is local to your machine and a fresh clone
must repeat stages 2 and 3.

### Stage 4 — localize against the saved map

```bash
ros2 launch mobile_base_bringup localization.launch.py world:=navigation_basic map:="$HOME/ros2_ws/src/mobile_base/maps/navigation_basic.yaml"
```

Click **2D Pose Estimate** in RViz, click where the robot actually is, and drag
in the direction it faces. The particle cloud appears. Drive with teleop and
watch it tighten — AMCL only converges when the robot moves.

Check it from the CLI:

```bash
ros2 topic echo /amcl_pose --once
```

The `covariance` diagonal shrinking is convergence. It starts near the 0.25 you
seeded and drops to roughly 0.01 once the robot has driven a little.

### Stage 5 — navigate autonomously

```bash
ros2 launch mobile_base_navigation planning.launch.py world:=navigation_basic map:="$HOME/ros2_ws/src/mobile_base/maps/navigation_basic.yaml"
```

Three things, in this order:

**1. Set the initial pose.** Click **2D Pose Estimate** as in stage 4. Nothing
in the navigation stack activates before you do — every server sits
`unconfigured` and the `nav_autostart` node brings them up the moment AMCL
publishes `/amcl_pose`. There is no rush and no timeout to beat; a costmap
brought up any earlier would be drawn against a pose you never supplied.

Confirm it worked — both must say `active [3]`:

```bash
ros2 lifecycle get /planner_server
```

If you would rather everything came up at launch, pass `autostart:=true`. To
bring it up by hand instead, launch with `autostart_on_localization:=false` and
call:

```bash
ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"
```

**2. Clear the emergency stop.** It is engaged at startup on purpose, so a
human decides when the scene is ready. The robot will not move until you do
this:

```bash
ros2 service call /estop_gate/reset std_srvs/srv/Trigger
```

**3. Give it a goal.** Click **2D Goal Pose** in RViz. On the default
`primitive` profile the robot **turns to face the first leg, then drives it**,
repeating that per leg and settling the goal heading at the end — so expect
visible stop-turn-go rather than a smooth curve. Click again for the next goal,
as often as you like — the pose it reaches is simply where the next goal starts
from.

Drag the goal arrow to set the final heading; the robot rotates onto it after
arriving. To watch which primitive is running:

```bash
ros2 topic echo /cmd_vel_nav --field twist
```

Goals can also be sent from the CLI:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

To stop it at any time:

```bash
ros2 service call /estop_gate/engage std_srvs/srv/Trigger
```

The stop latches. New goals do nothing until you reset, and reset restores
permission without resuming the interrupted goal.

### If the robot will not move

In order of likelihood:

```bash
# 1. Is the e-stop engaged? (data: true means engaged)
ros2 topic echo /safety/estop_active --once

# 2. Did every server activate? All must say "active [3]".
for n in controller_server planner_server behavior_server bt_navigator velocity_smoother collision_monitor; do echo "$n: $(ros2 lifecycle get /$n)"; done

# 3. Is exactly one publisher driving the wheels?
ros2 topic info /mobile_base_controller/reference -v | grep -i "publisher count"

# 4. Is anything reaching the wheels at all?
ros2 topic hz /mobile_base_controller/reference
```

---

## Part 3 — Mapping and saved-map localization in depth

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
| `my_world` | 10 x 10 m sealed apartment: six rooms, an L-shaped kitchen and living room, doorways 1.20 m down to 0.60 m | Map it yourself; none is maintained |

Generated maps and pose graphs live under `$HOME/ros2_ws/maps/mobile_base/`.
That runtime directory is ignored by Git. Saving a map with an existing
basename replaces that local YAML/PGM pair.

```text
ros2_ws/maps/mobile_base/
├── navigation_basic.yaml
├── navigation_basic.pgm
├── my_world.yaml               # after mapping my_world
└── my_world.pgm                # after mapping my_world
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

### Phase 2 tuning and limitations

Defaults match the repository's 10 Hz, 0.10-4.0 m simulated LaserScan, 50 Hz
filtered odometry, `base_footprint` frame, small robot dimensions, and
conservative 0.05 m map resolution. Hardware commissioning must remeasure and
tune laser min/max range, scan and TF timing, SLAM travel/update and loop
closure thresholds, AMCL `alpha1`-`alpha5` (especially lateral `alpha5`),
particle counts, update thresholds, and transform tolerance.

Generated maps and pose graphs are runtime artifacts and are not committed; a
fresh clone must map before any launch that requires a saved map.
Continued mapping from a serialized graph is optional and not automatically
launched. The simulated GPU LiDAR requires the wrappers' default Ogre2 sensor
renderer; an Ogre1 validation run pinned all 720 beams to the 0.10 m minimum
and cannot produce a usable map. Phase 2 does not add planners, controller
servers, behavior trees, goal execution, obstacle avoidance, command
arbitration, velocity smoothing, or any other autonomous-navigation component.

---

## Part 4 — Costmaps and path planning in depth

Phase 3a adds Nav2 costmaps and a global planner on top of saved-map
localization. It plans and displays routes; it cannot move the robot. There is
no controller server, behavior tree, behavior server, command mux, or collision
monitor, and nothing publishes to `/mobile_base_controller/reference`.

### Drive it

```bash
ros2 launch mobile_base_navigation planning.launch.py \
  world:=navigation_basic map:="$HOME/ros2_ws/src/mobile_base/maps/navigation_basic.yaml"
```

**The e-stop is engaged at startup.** That is deliberate: a stop is latched, and
a human decides when the scene is ready. Clear it before the robot will move:

```bash
ros2 service call /estop_gate/reset std_srvs/srv/Trigger
```

Then set **2D Pose Estimate** once, and click **2D Goal Pose** wherever you want
the robot to go. It drives there, avoiding obstacles, and stops. The pose it
reaches is simply where the next goal starts from — click again, as often as you
like, with no re-initialisation. A measured 5-goal sequence succeeded 5/5.

To stop it at any time:

```bash
ros2 service call /estop_gate/engage std_srvs/srv/Trigger
```

The stop latches. Reset restores *permission*, not motion: the robot stays put
until a fresh goal arrives, and never resumes the interrupted one. Killing
`estop_gate` outright also stops the robot — the lock is a deadman, and silence
engages it.

Set the initial pose with **2D Pose Estimate** promptly: the costmaps cannot
activate until AMCL publishes `map -> odom`, and the lifecycle manager gives up
after about a minute. If bringup aborts, set the pose and then start the nodes
by hand rather than relaunching:

```bash
ros2 service call /lifecycle_manager_navigation/manage_nodes \
  nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"
```

Then click **2D Goal Pose**. Each click plans from the robot's current pose and
draws the route on `/plan`:

```text
[goal_to_plan]: Planning to (2.50, 3.40) in map
[goal_to_plan]: Path found: 93 poses, 5.03 m. Shown on /plan.
```

Failures are reported with their meaning - `208` no valid path, `205` start
occupied, `204` goal outside the map - and the node keeps serving clicks. Goal
orientation is ignored, because `SmacPlanner2D` runs with
`use_final_approach_orientation: false`; only the clicked position matters.

Goals can also be sent from the CLI, which is how the planner is tested without
RViz:

```bash
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose "{goal: {header: {frame_id: map}, pose: {position: {x: 2.5, y: 3.4, z: 0.0}, orientation: {w: 1.0}}}, use_start: false}"
```

### Headless tuning harness

Costmap and planner tuning iterates faster without Gazebo. The harness runs the
same configuration against static transforms, so a plan takes seconds:

```bash
ros2 launch mobile_base_navigation planning_harness.launch.py \
  map:="$HOME/ros2_ws/src/mobile_base/maps/navigation_basic.yaml" rviz:=true
```

Confirm the stack is active rather than merely launched, and that the footprint
is the robot's rather than Nav2's default:

```bash
ros2 lifecycle get /planner_server
ros2 topic echo /global_costmap/published_footprint --once
```

Expect `active [3]` and a polygon near +/-0.15 m - the 0.14 m radius plus Nav2's
default 0.01 m footprint padding. Points near +/-0.22 mean the Nav2 default
radius is still in force.

### Sizing, and why it is derived rather than inherited

Nav2's defaults are sized for a robot three to four times larger than this one.
Every value below comes from the robot or the sensor:

| Parameter | Value | Source |
| --- | --- | --- |
| `robot_radius` | `0.14` | 0.1367 m circumscribed radius from `properties.xacro`, rounded up |
| `inflation_radius` | `0.30` | Exceeds the footprint, and leaves a zero-cost band in the 0.925 m north gap |
| `resolution` | `0.05` | Matches the saved map exactly; mismatches cause resampling artifacts |
| `obstacle_max_range` | `3.5` | Inside the 4.0 m LiDAR maximum, so max-range returns never mark |
| `raytrace_max_range` | `3.8` | Beyond marking range, so free space clears properly |
| `robot_base_frame` | `base_footprint` | Nav2 defaults to `base_link`, which this repository does not use |

A circular footprint suits a holonomic base, which has no preferred heading to
model. The local costmap repeats the footprint and inflation so tuning
transfers, but runs in `odom` with `rolling_window: true`; setting it to `map`
is a common error that makes it fight AMCL corrections. Both costmaps use
`ObstacleLayer` rather than `VoxelLayer`: there is one planar LiDAR and no 3D
sensor to populate voxels.

Plugin strings use the `::` separator that Jazzy requires. Read the installed
manifest when adding one - `nav2_smac_planner::SmacPlanner2D` was confirmed
against `/opt/ros/jazzy/share/nav2_smac_planner/smac_plugin_2d.xml` - rather
than copying a `/`-style string from an older configuration.

### What inflation does and does not do

```text
cost(d) = INSCRIBED_INFLATED_OBSTACLE * e^(-cost_scaling_factor * (d - robot_radius))
```

Raising `cost_scaling_factor` makes cost fall off *faster*, so paths run
**closer** to walls; it does not enlarge the cleared region. Set
`inflation_radius` for reach first, then shape the gradient.

More importantly, **inflation is not lethal**. Only the inscribed band within
`robot_radius` blocks a global plan, so `inflation_radius` does not decide
whether a route fits - it decides what that route costs, which is what a local
controller will follow once one exists. Measured in `navigation_basic`, whose
interior wall leaves a 0.925 m gap at its north end:

| Configuration | Result |
| --- | --- |
| `robot_radius` 0.14, `inflation_radius` 0.30 | 4.48 m through the gap, gap centre cost 0 |
| `robot_radius` 0.14, `inflation_radius` 0.55 | 4.57 m through the gap, gap centre cost 39-61 |
| Nav2 defaults, 0.22 and 0.55 | 4.56 m, still through the gap |
| `robot_radius` 0.50 | 10.59 m detour south, gap inscribed end to end |

So the gap is closed by footprint, not by inflation, and Nav2's defaults are not
tight enough to fail this map. Changing `inflation_radius` through `ros2 param
set` does not take effect on the published costmap; set it at launch instead.

### Phase 3 validation coverage

Static tests recompute the circumscribed radius from `properties.xacro` and
assert the costmap footprint covers it, so geometry and costmap cannot drift
apart. They pin both costmaps' frames, keep sensor ranges inside the LiDAR
maximum, require the `::` plugin separator, and enforce the **single-arbiter
invariant**: exactly one node publishes `/mobile_base_controller/reference`,
and it is `twist_mux`.

They also pin every holonomic trap, because Nav2's diff-drive defaults fail
*silently* on a mecanum base — the configuration looks right and the robot just
behaves like a differential one:

| Stock default | What it does here | Set to |
| --- | --- | ---: |
| `min_y_velocity_threshold: 0.5` | zeroes every lateral command this robot can produce | `0.001` |
| `motion_model: "DiffDrive"` | MPPI never samples lateral motion | `"Omni"` |
| `robot_base_frame: base_link` | breaks every costmap and server transform | `base_footprint` |
| `odom_topic: /odom` | no such topic here; MPPI believes the robot never moves | `/odometry/filtered` |
| smoother `max_velocity: [_, 0.0, _]` | clamps lateral velocity to zero after MPPI produced it | non-zero `y` |

The last one is worth dwelling on. `controller_server` and `bt_navigator` each
have their own `odom_topic`, and a dead odometry subscription produces no error
at all — the node stays active. The measured symptom was commands pinned near
0.02 m/s against a 0.15 limit, and the robot driving the wrong way.

Runtime verification, all in simulation:

```bash
ros2 topic info /mobile_base_controller/reference -v | grep -i "publisher count"
```

Must report exactly **1**. More than one publisher means something bypassed both
the mux and the e-stop.

### Measured stop path

Numbers, not estimates, all at 0.121 m/s and confirmed from `/joint_states`
rather than from a command topic:

| Trigger | Distance travelled | Wheels at zero |
| --- | ---: | ---: |
| `estop_gate/engage` service | 0.065 m | 0.523 s |
| `estop_gate` killed (deadman) | 0.120 m | 0.909 s |

The deadman path costs roughly one extra lock timeout, which is the design
working as intended. `twist_mux` never republishes and never emits a zero — it
simply stops publishing — so the final stop always comes from the controller's
own `reference_timeout: 0.5`. A zero on a command topic is not a stopped robot;
what makes these numbers evidence is the wheel feedback, and even then only for
simulated wheels.

Rotation-in-place clearance was measured before enabling `Spin`: a full turn
changed the closest observed obstacle distance by **0.01 m**. The footprint is
circular, so rotating sweeps nothing beyond the radius it already occupies —
which is why the stock warning about surprise rotations, written for
rectangular and legged bases, does not apply here.

### Phase 3 tuning and limitations

Goal error can exceed `xy_goal_tolerance`. The planner has its own `tolerance`
of 0.125 m for goals it cannot reach exactly, and the checker adds 0.15 m on
top, so a goal placed against an obstacle can settle up to ~0.275 m out. A
5-goal sequence measured 0.119–0.262 m.

`yaw_goal_tolerance` is deliberately loose at 0.50 rad. These goals are
position-to-position, and a tight yaw tolerance is the known cause of a robot
that re-approaches its goal forever without settling. Husarion go further still
on mecanum and set 6.3, ignoring final heading entirely.

The bringup velocity smoother stays disabled: this stack runs its own inside the
navigation chain, and two smoothers would both remap onto the controller
reference and break the single-arbiter rule.
The harness needs no Gazebo and is a candidate for CI once planning regressions
are worth asserting.

**Speeds are no longer the ones these numbers were measured at.** Both profiles
were doubled by request: `primitive` runs 0.20 m/s / 0.60 rad/s and `holonomic`
0.30 m/s / 1.20 rad/s, against a characterised envelope of 0.10 m/s /
0.30 rad/s. The stopping distances below (0.065 m on the e-stop service,
0.120 m on the deadman) were taken at 0.12 m/s and scale roughly with speed, so
treat them as a lower bound now. The velocity smoother's `[0.5, 0.5, 2.0]`
ceiling is the only remaining hard bound; the static tests assert both
controllers stay under it. Recovery rotation was doubled too, which is the
least comfortable part — recoveries run close to obstacles, and a spin that
misjudges clearance now does so twice as fast.

On the `primitive` profile the goal-error picture differs: the lattice ends its
path at a cell centre within its own `tolerance` of 0.05 m, but the goal checker
still terminates the run at 0.15 m, so measured error clusters near 0.15 m
rather than near 0.05 m. That is a property of the checker, not of the planner.

The `primitive` profile has no accuracy campaign, no stopping-distance
measurement of its own, and no hardware exposure. It is the default because its
motion is legible, not because it is better characterised — `holonomic` remains
the profile every number in this README was measured against.

---

## Part 5 — Driving it by hand

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

---

## Part 6 — Repeatable motion checks

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

---

## Part 7 — The ToF perimeter

Eight VL53L7CX sensors ring the lower deck, each reporting an 8x8 grid of zones
over a 60 degree field. They exist to see the band the LiDAR cannot: the scan
plane sits at 91.5 mm and ground clearance is 9.5 mm, so anything between those
two heights stops the robot while being invisible to it. A shoe. A cable. A
doorsill.

**Obstacle detection only.** Cliff detection was removed: on the geometry this
robot has, the shallow rows could not tell a drop from an error in the fitted
floor reliably enough to act on, and a costmap fed those mistakes trapped the
robot behind marks it had invented.

### Run it

```bash
ros2 launch mobile_base_bringup simulation.launch.py world:=my_world render_engine:=ogre2
```

`render_engine:=ogre2` is not optional. Ogre1 corrupts range data quietly -
cloud shapes and timestamps stay valid while the numbers are wrong - so anything
measured on an Ogre1 run is not evidence.

| Argument | Default | Effect |
| --- | --- | --- |
| `tof_classifier` | `true` | Publishes the cloud the local costmap consumes |
| `tof_markers` | `true` | Draws every ray in RViz. Visualisation only; turn it off when not looking |
| `tof_shapes` | `true` | Draws walls, objects and the floor region (needs the classifier) |
| `sensor_panel` | `true` | Opens the sensor power window |
| `gui` | `true` | The Gazebo window. It slows the sim to about 27 % of real time on an integrated GPU; `gui:=false` keeps it near 95 % and RViz shows everything anyway |

To start next to a wall, add a spawn pose, e.g. `world:=navigation_basic
x:=1.05 y:=1.0 gui:=false`.

### What to look at

| Topic | Contents |
| --- | --- |
| `/tof/<face>/points` | Raw 8x8 grid per sensor, eight of them |
| `/tof/obstacles` | Classified obstacles. The only ToF topic the costmap uses |
| `/tof/floor` | Recognised floor. Wired nowhere, on purpose |
| `/tof/rays` | Every ray, plus a marker where a classified obstacle was hit |
| `/tof/shapes` | Walls, objects and the floor region |
| `/tof/obstacle_zones` | Which zones of each sensor are marked obstacles |
| `/sensors/status` | Per-sensor on/off and measured rate |

### Reading the display

Everything ToF sits in the **ToF ring** folder in RViz, and colour always means
the same thing whichever sensor drew it:

| Colour | Meaning |
| --- | --- |
| Faint slate | Raw returns and rays. Background only |
| Orange ray and dot | A zone the classifier marked as an obstacle |
| Green region | Floor confirmed around the robot |
| Red / amber / blue shape | Wall or object under 0.25 m / under 0.5 m / further |
| Red outline, "OFF" | A sensor switched off |

Labels such as `WALL 0.42 m | h>=76 mm` give the distance and the height the
ring actually saw, which is a lower bound.

### Marking ranges

| Kind | Marked to |
| --- | --- |
| Low obstacle (caught by the floor shortfall test) | 0.4 m |
| Obstacle, 25 mm up to the roof | 1.0 m |
| Wall (a column of zones at one distance) | 2.0 m |

Low-obstacle detection has not been proven yet.

### Switching sensors off

The **Sensor power** window shows the robot front-up with every field of view
to scale and a switch on each sensor. The same switches from a terminal:

```bash
ros2 param set /sensor_power enabled.front false
ros2 param set /sensor_power enabled.lidar false   # SLAM and Nav2 stop updating
ros2 topic echo /sensors/status
```

A switched-off sensor goes silent and its data disappears from RViz, the
classifier and the costmap. Gazebo still renders it, so the sim does not get
faster.

### Proving it works

Park in a corridor in `my_world` and compare the two sensors:

```bash
ros2 topic echo /tof/obstacles --field width      # count of marked points
ros2 topic echo /scan --field ranges | head       # what the LiDAR sees
```

Measured in a corridor with walls 0.65 m either side, the ring reports about 23
points per frame and every one of them corresponds to a wall the LiDAR also
sees. Drive into open floor and the count should fall to near zero: **marks on
empty floor are the failure mode to watch for**, and the thresholds are tuned so
that the steep rows keep millimetre sensitivity while the shallow row has to
miss its prediction by a wide margin before it is believed.

To see it catch something the LiDAR cannot, put a low object in front of the
robot in `navigation_basic` - anything between 30 and 90 mm tall. It should
appear in `/tof/obstacles` while `/scan` shows nothing at that bearing.

### Seeing it in the costmap

The ToF ring marks the **local** costmap only. The global costmap is in the map
frame and never rolls, so a near-field sensor feeding it accumulates every
transient mark for the whole run until the robot is enclosed by its own history.

Costmaps only exist once navigation is running, which needs a saved map. Map
`navigation_basic` as described under *Phase 2* above, then bring up navigation
and enable the **Local costmap** display in RViz. Approaching a low obstacle
should raise lethal cells where the LiDAR shows nothing.

---

## Part 8 — Inspecting a running robot

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
                                                     ├──> tof_front_link
                                                     ├──> tof_rear_link
                                                     ├──> tof_left_link
                                                     ├──> tof_right_link
                                                     ├──> tof_front_left_link
                                                     ├──> tof_front_right_link
                                                     ├──> tof_rear_left_link
                                                     ├──> tof_rear_right_link
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

---

---

## Part 9 — ToF-only navigation

Drive to a goal with **no map, no SLAM, no Nav2 and the LiDAR switched off**:
odometry for position, the ToF ring for everything else. The navigator is a
holonomic DistBug; every command it sends passes through a speed governor that
caps it by what the ring sees along the direction of travel.

### Run it

```bash
ros2 launch mobile_base_bringup bug_navigation.launch.py world:=navigation_basic
```

Then give a goal with **2D Goal Pose** in RViz, or from a terminal:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: odom}, pose: {position: {x: 3.0, y: 1.0}, orientation: {w: 1.0}}}"
```

The launch file turns the velocity smoother on (the governor's speeds assume
its braking limits) and the Gazebo window off. Add `gui:=true` for the window;
the sim then runs at about a quarter of real time.

| Goal from the origin in `navigation_basic` | What it exercises |
| --- | --- |
| `3.0, 1.0` | Meet the interior wall, follow it, round its end, leave early |
| `2.5, -3.0` | Past the south-east box |
| `-3.3, 3.0` | Behind the north-west box |
| `1.5, 1.0` | Inside the interior wall - should end UNREACHABLE |

### What the robot does

- **Heading is held.** It never turns; it slides in whatever direction it needs.
- **Going to the goal** it moves straight at it. When the corridor its body
  would sweep is blocked within 0.35 m, it starts following that boundary.
- **Following** it keeps **0.30 m** from the boundary when the far side is open
  and takes the middle of the space as it narrows, down to **0.05 m**, so it
  squeezes through gaps it fits and keeps its distance where there is room.
  It follows only the boundary it met, so a wall end is rounded rather than
  swapped for the next wall.
- **Leaving (DistBug)**: as soon as a clear run toward the goal, down a swath
  0.30 m either side of the body, would end closer to the goal than anywhere it
  has been since meeting the obstacle - or the goal is in clear view.
- **Unreachable**: back where it met the obstacle without having left it -
  after first trying once the other way round.
- **Deadlocks**: stalled (commanding motion, under 5 cm in 5 s) it backs off
  0.2 m and retries (panel: RECOVERING); after three tries it stops as STUCK
  rather than loop.

`algorithm: bug2` in `mobile_base_tools/config/bug_navigation.yaml` switches to
Bug2's m-line leaving rule for comparison.

### Speed levels

Each level's speed stops the robot, with a factor-2 margin, inside its band:
`2 (v t + v^2 / 2a) <= d - 0.05` with t = 0.35 s and a = 1.0 m/s^2. Distances
are from the body, **along the direction of travel only** - a wall beside the
robot while it slides along it does not slow it.

| Level | Trigger | Speed |
| --- | --- | --- |
| CRUISE | nothing within 1.0 m ahead, walls beyond 0.65 m | 0.50 m/s |
| CAUTION | obstacle 0.4-1.0 m ahead, or wall 0.4-0.65 m | 0.33 m/s |
| SLOW | anything 0.25-0.4 m ahead, or under 15 cm beside the body | 0.20 m/s |
| CRAWL | 0.12-0.25 m ahead, or under 6 cm beside | 0.10 m/s |
| STOP | under 0.12 m ahead, no ToF data for 0.5 s, or the STOP button | 0 |

Levels drop at once and climb one step per 0.5 s, so a start from rest takes
about two seconds to reach cruise. The speeds assume 1.0 m/s^2 of braking;
measure it on hardware and retune in `bug_navigation.yaml`.

### The control panel

| Row | Readouts |
| --- | --- |
| Motion | Speed, body velocity, heading, odometry position |
| Safety | Speed level and cap, what limits it, nearest obstacle distance and bearing (0° front, + left), navigator state |
| Localisation | EKF 1σ position and heading uncertainty, wheel-only vs fused pose gap, distance to goal |

**STOP** latches every autonomous command at zero until **RELEASE**. It stops
what passes through the governor; it is not a hardware e-stop. The same from a
terminal:

```bash
ros2 service call /speed_governor/stop std_srvs/srv/SetBool "{data: true}"
```

A growing *wheel vs fused* gap means the wheels slipped or the IMU is
correcting them; without a map, odometry is the only position there is.

### Limits

- Odometry drifts, and nothing corrects it: long runs end centimetres off.
- The ring sees low obstacles to 0.4 m only; behind the robot's current
  direction it relies on the fans overlapping.
- A bug algorithm has no memory of places: in a maze it can take long detours.

---

## Part 10 — Navigating an unknown environment

No saved map: SLAM builds one as the robot drives, Nav2 plans on it -
including through space not yet seen - and DistBug takes over if Nav2 gives
up. Every sensor except the camera is used.

### Once: third-party sources

```bash
vcs import ~/ros2_ws/src/external < ~/ros2_ws/src/mobile_base/mobile_base.repos
```
```bash
cd ~/ros2_ws && colcon build --base-paths src/external --packages-select rf2o_laser_odometry explore_lite_msgs explore_lite
```
```bash
sudo apt install ros-jazzy-spatio-temporal-voxel-layer
```

### Step 1 - build and launch (terminal 1)

```bash
cd ~/ros2_ws && colcon build && source install/setup.bash
```
```bash
ros2 launch mobile_base_navigation explore_navigation.launch.py world:=my_world
```

Opens the **map RViz** (live SLAM map, planned path, costmaps) and the
**control panel**. Wait until the map shows the robot with the first walls
round it, 30-60 s.

### Step 2 - the simulation RViz (terminal 2, optional)

The close-up that follows the robot: ToF rays, obstacle hits, wall shapes,
floor region. Use the full path; a relative one opens an empty RViz when run
from another folder.

```bash
source ~/ros2_ws/install/setup.bash && rviz2 -d $(ros2 pkg prefix mobile_base_description)/share/mobile_base_description/rviz/mobile_base_sim.rviz --ros-args -p use_sim_time:=true -r __node:=sim_rviz
```

### Step 3 - release the e-stop (terminal 3)

Latched at every start; nothing moves until it is released.

```bash
source ~/ros2_ws/install/setup.bash && ros2 service call /estop_gate/reset std_srvs/srv/Trigger
```

### Step 4 - give a goal

In the **map RViz**: **2D Goal Pose**, click where to go, drag for the final
heading. Unexplored grey is fine. Or from terminal 3:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: -3.5, y: 2.5}, orientation: {w: 1.0}}}"
```

| x, y in `my_world` | Where |
| --- | --- |
| -3.5, 2.5 | North-west bedroom, 0.70 m door |
| 3.5, -3.8 | Master bedroom, across the flat via the gap in the south wall |
| 4.0, 4.0 | Bathroom |
| -4.0, -3.0 | South-west room |

### Step 5 - watch

| Where | What |
| --- | --- |
| Map RViz | Path appears; walls fill in; the path re-plans as they do |
| Sim RViz | Rays and shapes react to walls and doorways |
| Panel | Speed, level and its cause, navigator mode, distance to goal, pose uncertainty |

Panel **STOP / RELEASE** freezes and resumes autonomous driving. If the mode
turns to **bug fallback**, Nav2 found no way and DistBug is working round the
blockage; it hands back once past it.

```bash
ros2 topic echo /nav_supervisor/mode
```

### Exploring by itself

```bash
ros2 launch mobile_base_navigation explore_navigation.launch.py world:=my_world explore:=true
```

Then steps 2-3. Frontiers show in the map RViz; it drives to each until none
is left, then returns to the start.

### Stopping

Ctrl-C in terminals 1 and 2, then:

```bash
ros2 run mobile_base_tools stop_simulation
```

### Measured

| Run in `my_world`, from no map | Time | Navigator | SLAM error vs ground truth |
| --- | --- | --- | --- |
| Hallway to north-west bedroom | 31 s | Nav2 | 3.9 cm |
| Hallway to master bedroom | 94 s | Nav2 | 5.4 cm |

### Limits

- The e-stop must be released by hand after every launch.
- rf2o needs structure: in a long featureless corridor scan matching degrades
  and its jumps are rejected, leaving wheels and gyro.
- Doorways narrower than about 0.5 m after LiDAR jamb smear are refused by the
  planner; DistBug, which uses the ToF instead, may still get through.

