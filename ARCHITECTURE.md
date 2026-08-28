# ARCHITECTURE — `mobile_base`

Orientation document for future work on this repository. It records the package
map, runtime graph, data flows, conventions actually used here, and open issues.
`README.md` stays the user-facing operating manual; `docs/mecanum_motion_accuracy.md`
stays the calibration record. This file is the engineering map.

**Target distro: ROS 2 Jazzy** (pinned by `.github/workflows/ci.yaml`, `ros:jazzy`
container, Ubuntu 24.04; `/opt/ros/jazzy` installed locally). Gazebo pairing is
**Harmonic** via `ros_gz_*` + `gz_ros2_control`. Note that an interactive shell in
this workspace may have no `ROS_DISTRO` sourced — source
`/opt/ros/jazzy/setup.bash` before build/run, and never assume a newer distro's API.

---

## 1. Package map

Standalone colcon repo cloned into `~/ros2_ws/src/mobile_base`; each subdirectory is
one ament package. Independent of the arm stack in the same workspace.

| Package | Build type | Role |
| --- | --- | --- |
| `mobile_base_description` | `ament_cmake` (no compiled code) | Xacro/URDF model, RViz configs, wheel meshes; installs `../meshes` |
| `mobile_base_gazebo` | `ament_cmake` (no compiled code) | Gazebo Harmonic worlds: `empty`, `sensor_test`, `navigation_basic`, `navigation_narrow` |
| `mobile_base_bringup` | `ament_cmake` | `controllers.yaml`, top-level launch composition, `generate_sim_sdf.py`, launch tests |
| `mobile_base_localization` | `ament_cmake` | EKF / slam_toolbox / map_server+AMCL configs and launch, RViz views |
| `mobile_base_evaluation` | `ament_cmake` (**C++**) | `ground_truth_selector` node + library; the only compiled package |
| `mobile_base_tools` | `ament_python` | Runtime utilities and the whole evaluation/diagnostics toolchain |

**Dependency direction** (no cycles):

```
description ─┐
gazebo ──────┼──> bringup ──> (runtime) localization, evaluation, tools
localization ┤
evaluation ──┤
tools ───────┘
```

Only `mobile_base_bringup` depends on the others, and only as `exec_depend`.
Cross-package coupling in *tests* is done by passing sibling source dirs as argv
(e.g. `test_teleop_trajectory_config.py <bringup> <description> <gazebo>`), which
keeps build-time dependencies out of the graph.

### Custom CMake worth knowing

- Every `ament_cmake` package uses `find_package(Python3 ... Interpreter)` and wires
  its checks in as **plain `add_test` invocations of standalone Python scripts** with
  source paths as arguments — not `ament_add_pytest_test`. The same scripts are
  runnable by hand (CI does exactly that for `test_teleop_trajectory_config.py`).
- `mobile_base_description` installs `${CMAKE_CURRENT_SOURCE_DIR}/../meshes` — meshes
  live at the **repo root**, not inside the package.
- `mobile_base_bringup` registers two `add_launch_test`s, each pinned to its own
  `ROS_DOMAIN_ID` (31, 32) and `GZ_PARTITION`, with `RUN_SERIAL TRUE`. These are
  excluded from CI by `--ctest-args -E`.
- `mobile_base_evaluation` links Gazebo directly: `gz-msgs10`, `gz-transport13`, with
  `-Wall -Wextra -Wpedantic`, and splits pure selection logic into a library so it can
  be gtest-ed without a simulator.

---

## 2. Runtime graph

### 2.1 Nodes

| Node | Pkg / origin | Publishes | Subscribes | Services / clients | Notes |
| --- | --- | --- | --- | --- | --- |
| `robot_state_publisher` | upstream | `/tf`, `/tf_static`, `/robot_description` | `/joint_states` | — | Owns `base_footprint -> base_link` and all link/sensor TF |
| `controller_manager` (`gz_ros2_control` plugin, in-process with Gazebo) | upstream | — | — | controller lifecycle services | `update_rate: 100` Hz |
| `joint_state_broadcaster` | upstream controller | `/joint_states`, `/dynamic_joint_states` | — | — | Spawned first; gate for everything else |
| `mobile_base_controller` | `mecanum_drive_controller/MecanumDriveController` | `/mobile_base_controller/odometry`, `/mobile_base_controller/controller_state`, TF (conditional) | `/mobile_base_controller/reference` (`TwistStamped`) | — | `reference_timeout: 0.5` s |
| `ekf_filter_node` | `robot_localization` | `/odometry/filtered` (50 Hz), `odom -> base_footprint` TF | `/mobile_base_controller/odometry`, `/imu/data` | — | `two_d_mode`, `world_frame: odom` |
| `parameter_bridge` | `ros_gz_bridge` | `/clock`, `/scan`, `/imu/data`, `/camera/image_raw`, `/camera/camera_info` | — | — | GZ→ROS only (`[` direction) |
| `ground_truth_selector` | `mobile_base_evaluation` (**C++**) | `/mobile_base/evaluation/ground_truth` (`PoseStamped`, **SensorDataQoS**) | Gazebo Transport `/world/<world>/dynamic_pose/info` (not ROS) | — | Evaluation only; selects entity **by name**, rejects duplicates/non-finite poses |
| `odom_to_path` (×2: `raw_odometry_to_path`, `filtered_odometry_to_path`) | `mobile_base_tools` | `/mobile_base/trajectory`, `/mobile_base/filtered_trajectory` (**RELIABLE + TRANSIENT_LOCAL, depth 1**) | odom topic (depth 10) | offers `.../reset` (`std_srvs/Empty`) | Bounded 1000-pt deque with distance/yaw thresholds |
| `odometry_test_runner` | `mobile_base_tools` | `/mobile_base_controller/reference` | ground truth + raw odom + filtered odom + controller state + `/joint_states` + `/imu/data` | clients: `/world/<w>/set_pose`, `/controller_manager/list_controllers` | ~60 declared parameters; the main evaluation engine (1.5 kLOC) |
| `mecanum_motion_test` | `mobile_base_tools` | `/mobile_base_controller/reference` | — | — | Open-loop square/diamond/rotation sequence |
| `timestamp_validator` / `tf_validator` | `mobile_base_tools` | — | contract topics / `/tf` + `/tf_static` | `tf_validator` calls `GetParameters` on the controller | Bounded, write JSON, exit nonzero on failure |
| `slam_toolbox` (async) | upstream | `/map`, `map -> odom` | `/scan`, TF | map-save services | Mapping mode only |
| `map_server` + `amcl` + `lifecycle_manager_localization` | Nav2 | `/map` (latched), `/amcl_pose`, `map -> odom` | `/scan`, `/initialpose`, TF | lifecycle bond | Localization mode only; `autostart: true`, `bond_timeout: 4.0` |

**Lifecycle nodes**: only the Nav2 pair (`map_server`, `amcl`), managed by
`nav2_lifecycle_manager`. Everything written in this repo is a plain node. No
composition / component containers, no custom executors or callback groups, no
declared real-time constraints.

### 2.2 Data flow

```
                    Gazebo Harmonic (gz_ros2_control plugin in-process)
                              │
   gz transport dynamic_pose ─┴──> ground_truth_selector ──> /mobile_base/evaluation/ground_truth
                                                                    (EVALUATION ONLY — dead-ends
                                                                     in odometry_test_runner)
   gz sensors ──> parameter_bridge ──> /clock  /scan  /imu/data  /camera/*

   /mobile_base_controller/reference (TwistStamped)
             │
             v
   mobile_base_controller ──> /mobile_base_controller/odometry (100 Hz) ──┐
                          └─> /joint_states (via joint_state_broadcaster) │
                                                                          v
                          /imu/data (50 Hz, yaw rate only) ──────> ekf_filter_node
                                                                          │
                                                       /odometry/filtered (50 Hz)
                                                       odom -> base_footprint TF
                                                                          │
   /scan ──> slam_toolbox | amcl ──> map -> odom ──────────────────────────┘
```

### 2.3 TF ownership — the single most load-bearing invariant

Exactly one publisher per edge, switched by the `localization` launch argument:

| Mode | `odom -> base_footprint` | `map -> odom` | How enforced |
| --- | --- | --- | --- |
| `localization:=true` (default) | `ekf_filter_node` (`publish_tf: true`) | slam_toolbox **or** AMCL, never both | Controller spawned with `-p enable_odom_tf:=false` |
| `localization:=false` (raw diagnostic) | `mobile_base_controller` | none | Controller spawned with `enable_odom_tf:=true`; EKF not launched |

Two spawner `Node` actions exist for exactly this reason
(`base_controller_spawner_localized` / `_raw`), selected by a `PythonExpression`
condition, both remapping `mobile_base_controller/tf_odometry:=/tf`. `tf_validator`
asserts the rule at runtime: no loops, no multi-parent edges, no `map -> odom` in
Phase 1 modes, no evaluation-only frames, and controller `enable_odom_tf` read back
over the parameter service must match the mode.

Frame tree: `map -> odom -> base_footprint -> base_link -> {4 wheels ×10 rollers,
lidar_link, imu_link, camera_link -> camera_optical_frame}`.

### 2.4 QoS

Mostly default (`depth 10`). The deliberate departures:

| Endpoint | Profile | Why |
| --- | --- | --- |
| `odom_to_path` path output | RELIABLE, **TRANSIENT_LOCAL**, depth 1 | Late-joining RViz gets the trajectory |
| `tf_validator` `/tf` | RELIABLE, VOLATILE, depth 100 | Lossless topology observation |
| `tf_validator` `/tf_static` | RELIABLE, **TRANSIENT_LOCAL**, depth 100 | Static TF is latched |
| `ground_truth_selector` output | `rclcpp::SensorDataQoS()` (BEST_EFFORT) | High-rate pose stream |
| `odometry_test_runner` ground truth / `/joint_states` / `/imu/data` | `qos_profile_sensor_data` | Must match the BEST_EFFORT publishers above |
| Odometry / command topics | default depth 10 | RELIABLE both ends |

No DEADLINE or LIFESPAN is used anywhere; staleness is instead enforced *in
application code* (`data_is_stale`, `max_source_skew`, `data_timeout`,
`clock_stall_timeout`) and by the timestamp contracts YAML.

### 2.5 Launch composition

```
simulation.launch.py            ← the one real launch file (589 lines)
├── _gazebo_environment()       ← rewrites LD_LIBRARY_PATH/PATH/GZ_CONFIG_PATH,
│                                 and strips a workspace mecanum_drive_controller
│                                 overlay out of AMENT/CMAKE_PREFIX_PATH
├── OpaqueFunction _resolve_world  ← parses the world SDF, rewrites max_step_size
│                                    and render_engine, writes /tmp/mobile_base_world_<pid>.sdf
├── gz_sim.launch.py (gui / headless variants)
├── robot_state_publisher (xacro Command with all roller calibration args)
├── parameter_bridge (clock + 4 sensor topics)
└── generate_sim_sdf.py ─OnProcessExit→ ros_gz_sim create
      └─OnProcessExit→ joint_state_broadcaster spawner
            └─OnProcessExit→ [mobile_base_controller spawner (localized|raw),
                              localization.launch.py (EKF), odom_to_path ×2, RViz]

odometry_evaluation.launch.py → simulation.launch.py + reset bridge
                                 + ground_truth_selector + odometry_test_runner
                                 + Shutdown on evaluator exit
bringup/mapping.launch.py      → simulation (rviz=false, localization=true)
                                 + localization/mapping.launch.py (slam_toolbox async)
bringup/localization.launch.py → simulation (rviz=false, localization=true)
                                 + localization/amcl.launch.py (map_server+amcl+lifecycle)
description/display.launch.py  → RSP + joint_state_publisher(+GUI) + RViz, no Gazebo
```

Startup is a strict **event-handler chain**, not timers: each stage's failure emits
`Shutdown` with a reason rather than continuing degraded.

**Nav2 usage is deliberately partial**: `nav2_map_server`, `nav2_amcl`,
`nav2_lifecycle_manager`, `nav2_rviz_plugins` only. No planner, controller server,
behavior tree, recoveries, velocity smoother, or command arbitration exists — see §5.

---

## 3. Physical model and calibration (why the URDF looks the way it does)

- 4 driven hubs with **no collision geometry**; all ground contact goes through
  **40 explicit passive roller links** (10 per wheel, continuous joints, 45°).
- Each roller's collision is **nine overlapping spheres** approximating the measured
  tapered CAD envelope (360 primitive collisions total). Stations/radii are locked by
  `test_explicit_mecanum_rollers.py`; `roller_collision_model:=cylinder` is kept as a
  valid comparison mode.
- Calibrated defaults, threaded identically through `simulation.launch.py`,
  `odometry_evaluation.launch.py`, `generate_sim_sdf.py`, the xacro args **and**
  `odometry_test_runner` parameters: damping `0.0`, joint friction `0.0`, contact
  `mu=0.8`, physics step `0.001 s`, per-wheel measured roller phases
  (`0.22193969 / 0.48030419 / 0.19668582 / 0.24790784`).
- `wheels_radius = 0.03074443`, `sum_of_robot_center_projection_on_X_Y_axis = 0.142`
  appear in `controllers.yaml`, `properties.xacro`, and as runner parameters/launch
  defaults. **Changing one means changing all of them.**
- `generate_sim_sdf.py` exists because Gazebo needs `file://` mesh URIs and validated
  explicit rollers: it runs xacro → `gz sdf`, rewrites the four wheel mesh URIs to
  absolute installed paths, and validates roller link/joint names against regexes
  before the model is spawned.

---

## 4. Conventions actually used in this repo

**Style**

- Apache-2.0 header (`# Copyright 2026 Safwan`) on every source file; module and
  function docstrings everywhere, one-line imperative style. flake8 + pep257 via
  `ament_lint_auto` — 79-col lines, single quotes, trailing commas.
- SI units everywhere; REP-103 (`+X` fwd, `+Y` left, `+Z` up) stated in comments.
- Comments explain **why**, and frequently record a rejected alternative or a
  measurement (e.g. "Half wheelbase (0.075) plus half track width (0.067)").

**ROS patterns**

- Every parameter is declared from a `defaults` dict, then read back through
  typed validator helpers (`_string`, `_positive`, `_nonnegative`, `_finite`) that
  raise on bad values. No `ParameterDescriptor` ranges are used — validation is
  explicit and code-side instead.
- **Pure-function core, thin node shell.** `pose_math`, `mecanum_diagnostics`,
  `odometry_evaluator`, `motion_profiles`, `report_writer`, `plot_writer`,
  `process_lifecycle` contain no rclpy at all and are unit-tested directly; only
  `*_runner`/`*_validator`/`odom_to_path` subclass `Node`. The C++ package mirrors
  this (`ground_truth_selector_lib` vs `_node.cpp`).
- Launch files use `FindPackageShare`/`PathJoinSubstitution`, never hardcoded paths;
  `OpaqueFunction` is used wherever a value must be validated before nodes exist, and
  it **raises `RuntimeError`** on bad input rather than defaulting.
- `SetLaunchConfiguration` guards (`simulation_rviz`, `phase2_rviz`) exist because
  included launch files share one context — do not remove them.
- `main()` in rclpy nodes wraps spin in try/except/finally and re-checks `rclpy.ok()`
  before shutdown (campaign runs kill process groups).
- Evidence policy: generated results (`phase1_results/`, `odometry_results/`,
  `maps/`, bags) are **gitignored**, and `test_phase1_results_policy.py` enforces
  that they stay ignored while source config stays tracked. Reports are JSON + CSV,
  written atomically, non-overwriting and resumable.

**Testing** (all deterministic tests run in CI; the two Gazebo launch tests do not)

| Package | Tests |
| --- | --- |
| `description` | xacro generates; explicit-roller geometry lock (585 lines: counts, phases, sphere stations, handedness, masses); sensor description contract |
| `gazebo` | installed worlds well-formed |
| `bringup` | teleop/trajectory static config; `test_sensor_topics.py` (launch test, headless Gazebo, LiDAR+IMU); `test_odometry_evaluation_runtime.py` (launch test, short forward run produces a report) |
| `localization` | EKF config contract; Phase 2 frame/mode/sensor/launch contract |
| `evaluation` | gtest on `select_named_pose` (missing, duplicate, non-finite) |
| `tools` | 10 pytest modules — profiles, diagnostics, evaluation math (534 lines), tf/timestamp validators, odom_to_path (pure + ROS), process lifecycle, results policy |

CI additionally runs `compileall`, `xacro` + `check_urdf`, and `git diff --check`.

---

## 5. Sharp edges, debt, and open issues

No `TODO`/`FIXME` markers exist anywhere in the repo — the debt is structural and
documented in prose instead. What to watch:

1. **`simulation.launch.py` rewrites the process environment.** `_gazebo_environment()`
   prepends ~17 vendor lib/bin/config paths and *removes* a workspace-overlay
   `mecanum_drive_controller` prefix from `AMENT_PREFIX_PATH`/`CMAKE_PREFIX_PATH`.
   This is a real provenance hazard: which controller binary runs depends on where
   it is installed. Verify with `ros2 pkg prefix mecanum_drive_controller` before
   blaming controller behavior.
2. **`/tmp` files keyed only by PID.** `/tmp/mobile_base_world_<pid>_<engine>.sdf` and
   `/tmp/mobile_base_sim_<pid>.sdf` are never cleaned up and can collide across
   concurrent runs. Default `output_dir` is also `/tmp/mobile_base_phase1`.
3. **Calibration constants are duplicated in five places** (xacro properties, xacro
   args, `controllers.yaml`, launch defaults ×2, runner parameter defaults). There is
   no single source of truth; the static tests catch some but not all divergence.
   The same duplication bites for *validation*: `roller_collision_model` is checked
   against a hardcoded list in three separate places (`simulation.launch.py`
   `_resolve_world`, `generate_sim_sdf.py` argparse choices, `odometry_test_runner.py`),
   so adding a contact model means editing all three or it fails at launch.
4. **`odometry_test_runner.py` is 1493 lines in one class** with ~60 parameters — by
   far the largest and least decomposed file. Its pure math already lives elsewhere;
   the orchestration (state machine, watchdogs, reset, reporting) has not been split.
5. **Residual physical error is real and distance-dependent.** Focused low-speed
   primitives sit at ~2.9% diagnostic error; 5 m runs reach 8.09% (forward-right
   diagonal, 408 mm endpoint error). Lateral/diagonal motion is the weak axis. Do not
   treat sim odometry as ground truth for long traverses. **The parasitic strafe yaw
   (±0.035–0.039 rad/m) is a roller-phase artifact, not a contact defect** — running
   all four wheels at one shared phase collapses it ~180×. The measured phases are
   kept deliberately; see `docs/mecanum_motion_accuracy.md`. Do not retune friction,
   damping, or physics step trying to remove it. Contact friction is a weak lever:
   0.21 percentage points across a 3× range.
6. **Ogre2 is mandatory for a usable LiDAR.** `render_engine:=ogre` pins all 720 beams
   to the 0.10 m minimum and cannot map — yet `odometry_evaluation.launch.py` defaults
   to `render_engine:=ogre`. That is fine for odometry (no scan needed) and a trap for
   anything scan-based.
7. **Gazebo launch tests are excluded from CI** and require serial execution with
   dedicated `ROS_DOMAIN_ID`/`GZ_PARTITION`. Coverage of the assembled system is
   therefore manual/host-dependent. RTF is ~0.82–0.89.
8. **No safety layer.** There is no command arbitration, velocity smoothing, e-stop,
   watchdog on `/mobile_base_controller/reference`, or deceleration limiting (the
   installed Jazzy `mecanum_drive_controller` has no limiter — this was checked, see
   `docs/mecanum_motion_accuracy.md`). The only protection is the controller's
   `reference_timeout: 0.5 s`. Two nodes (`mecanum_motion_test`,
   `odometry_test_runner`) can publish commands to the same topic simultaneously
   with nothing preventing it.
9. **Nav2 is half-integrated.** map_server + AMCL only. Adding planning means adding
   costmaps, a controller server, BT navigator, and — per the point above — a command
   arbiter, before any autonomous motion is enabled.
10. **Hardware has never been in the loop.** Every result in this repo is simulation
    (verification level L2–L3). AMCL alphas, laser ranges, SLAM thresholds, and TF
    timing are all sim-tuned and are explicitly flagged in the README as requiring
    remeasurement on hardware. `mock_components/GenericSystem` is the non-Gazebo
    hardware interface — there is no real driver.

---

## 6. Where to start for common tasks

| Task | Start here |
| --- | --- |
| Change robot geometry | `description/urdf/properties.xacro` → then `controllers.yaml`, runner defaults, and `test_explicit_mecanum_rollers.py` |
| Add a sensor | `urdf/<s>.xacro` + `<s>.gazebo.xacro`, bridge args in `simulation.launch.py`, contract in `timestamp_contracts.yaml`, assert in `test_sensor_description.py` |
| Change contact physics | `generate_sim_sdf.py` `inject_roller_surfaces` — **not** the xacro. `<gazebo reference>` friction tags are dropped by `gz sdf -p` and do nothing |
| Change TF ownership | `simulation.launch.py` spawner pair + `ekf.yaml` `publish_tf` + `tf_validator.evaluate_contract` |
| Add a motion profile | `tools/config/odometry_tests.yaml` (validated by `motion_profiles.py`) |
| Add navigation | new package; do **not** extend `mobile_base_localization` (its mutual-exclusion contract is tested) |
| Debug "robot doesn't move" | controller spawned? `ros2 control list_controllers` → is anything publishing `reference`? → roller contact model (`docs/mecanum_motion_accuracy.md`) |
