#!/usr/bin/env python3
# Copyright 2026 Safwan
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Validate the static navigation costmap, planner and command-chain contracts."""

from math import hypot
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml

# Width of the gap between the interior wall's north end (y = 3.0) and the
# north wall's inner face (y = 3.925) in the navigation_basic world. This is
# the short route the phase exists to keep open.
NORTH_GAP = 0.925

# Simulated planar LiDAR maximum range.
LIDAR_MAX_RANGE = 4.0

# The ToF ring's own ceiling, and it is geometric rather than optical. The
# apertures sit 28.5 mm up and the downward rows all terminate on the floor by
# 435 mm, so beyond 1.083 m the lowest ray still in flight has climbed past the
# robot's own 99.5 mm roof. Nothing marked further out could obstruct it, and
# the LiDAR already resolves that range 15x finer.
TOF_MAX_RANGE = 1.083
# That limit is for obstacles. A wall - a column of zones returning at one
# distance down to the lowest upward row - stands on the floor however far off,
# so the classifier marks walls out to 2 m, inside the sensor's 3.5 m.
TOF_WALL_RANGE = 2.0
TOF_SENSOR_RANGE = 3.5
TOF_SOURCES = ('tof_obstacles',)

# Fastest the robot may travel and still stop on an obstacle first seen by the
# second floor row, which lands 143.5 mm ahead: 0.1 s of latency plus v^2/2a at
# 1.0 m/s^2 must fit inside that.
TOF_SAFE_LINEAR = 0.44

# The chain is connected, so the safety property is no longer "nobody names
# this topic" but the single-arbiter invariant: exactly one node publishes to
# the controller's reference interface, and it is the mux.
COMMAND_TOPIC = '/mobile_base_controller/reference'
ARBITER = 'twist_mux'

# The only odometry topic this system actually publishes for Nav2's
# consumers. The EKF owns it; /odom does not exist here.
ODOM_TOPIC = '/odometry/filtered'

# Everything was characterised at 0.10 m/s and 0.30 rad/s. Controller limits
# may exceed that, but not without bound.
VALIDATED_LINEAR = 0.10
VALIDATED_ANGULAR = 0.30
# Raised from 2.5 when the drive speeds were deliberately doubled. Both
# controllers now run at 3x the characterised linear speed and 4x the angular,
# so this bound no longer says "close to what was measured" - it says "still
# inside the velocity smoother's ceiling, which is the only thing left holding
# them". The smoother's [0.5, 0.5, 2.0] is asserted directly below.
ENVELOPE_MULTIPLE = 5.0

# The robot is 0.216 m long; a goal tolerance larger than that would count the
# goal as reached with the robot a body-length away.
ROBOT_LENGTH = 0.216

# Top of the upper deck. Anything above it is clearance the robot drives under.
ROBOT_HEIGHT = 0.0995

# Measured worst-case stopping distance: the deadman path, e-stop
# engaged at 0.121 m/s, wheels at zero after 0.909 s, 0.12 m travelled.
# Zones that must clear an obstacle are sized from this.
STOP_DISTANCE = 0.12


def _read(path):
    return Path(path).read_text(encoding='utf-8')


def _parameters(path, *keys):
    document = yaml.safe_load(_read(path))
    for key in keys:
        document = document[key]
    return document['ros__parameters']


def _xacro_properties(path):
    root = ET.parse(path).getroot()
    namespace = '{http://www.ros.org/wiki/xacro}'
    return {
        element.get('name'): element.get('value')
        for element in root.findall(f'{namespace}property')
    }


def _plugin_strings(path):
    return [
        line.split('plugin:', 1)[1].strip()
        for line in _read(path).splitlines()
        if line.strip().startswith('plugin:')
    ]


def main():
    navigation = Path(sys.argv[1])
    description = Path(sys.argv[2])

    planner_yaml = navigation / 'config' / 'planner.yaml'
    local_yaml = navigation / 'config' / 'local_costmap.yaml'
    controller_yaml = navigation / 'config' / 'controller.yaml'
    behavior_yaml = navigation / 'config' / 'behavior.yaml'
    bt_yaml = navigation / 'config' / 'bt_navigator.yaml'
    mux_yaml = navigation / 'config' / 'twist_mux.yaml'
    monitor_yaml = navigation / 'config' / 'collision_monitor.yaml'

    controller = _parameters(controller_yaml, 'controller_server')
    behavior = _parameters(behavior_yaml, 'behavior_server')
    bt_navigator = _parameters(bt_yaml, 'bt_navigator')
    mux = _parameters(mux_yaml, 'twist_mux')
    monitor = _parameters(monitor_yaml, 'collision_monitor')

    planner = _parameters(planner_yaml, 'planner_server')
    global_costmap = _parameters(planner_yaml, 'global_costmap',
                                 'global_costmap')
    local_costmap = _parameters(local_yaml, 'local_costmap',
                                'local_costmap')

    # Footprint must cover the geometry the description actually declares, so
    # the two cannot drift apart.
    properties = _xacro_properties(
        description / 'urdf' / 'properties.xacro'
    )
    wheelbase = float(properties['wheelbase'])
    wheel_separation = float(properties['wheel_separation'])
    wheel_width = float(properties['wheel_width'])
    wheel_radius = float(properties['wheel_radius'])
    chassis_length = float(properties['chassis_length'])
    chassis_width = float(properties['chassis_width'])
    wing_clearance_x = float(properties['wing_clearance_x'])

    # The body is not a filled rectangle: each deck is that rectangle with the
    # four wheel wells cut out. Measure the real limbs and take the furthest,
    # so the footprint is neither charged for material the robot does not have
    # nor blind to a corner that it does.
    wheel_outer_y = (wheel_separation + wheel_width) / 2.0
    wing_half_length = wheelbase / 2.0 - wheel_radius - wing_clearance_x
    circumscribed = max(
        # End cap corner, at the full body width. This is the furthest point
        # on the body now that the caps run the whole width.
        hypot(chassis_length / 2.0, wheel_outer_y),
        # Spine corner, at the spine width.
        hypot(wheelbase / 2.0 + wheel_radius + wing_clearance_x,
              chassis_width / 2.0),
        # Wing corner, at the wheel outer face.
        hypot(wing_half_length, wheel_outer_y),
        # The wheels themselves.
        hypot(wheelbase / 2.0 + wheel_radius, wheel_outer_y),
    )
    assert global_costmap['robot_radius'] >= circumscribed
    assert local_costmap['robot_radius'] == global_costmap['robot_radius']

    # The footprint is what can actually close the north gap: a radius of at
    # least half the gap makes it inscribed cost end to end, and the planner
    # detours the long way south. Measured at radius 0.50: 10.59 m south
    # instead of 4.48 m through the gap.
    for costmap in (global_costmap, local_costmap):
        assert 2.0 * costmap['robot_radius'] < NORTH_GAP

    # Inflation must reach past the footprint for a usable gradient, and must
    # leave a zero-cost band in the north gap so the short route stays cheap.
    # Inflation is not lethal, so this does not decide whether a plan fits -
    # it decides what the plan costs, which is what a controller will follow.
    for costmap in (global_costmap, local_costmap):
        inflation = costmap['inflation_layer']['inflation_radius']
        assert inflation > costmap['robot_radius']
        assert 2.0 * inflation < NORTH_GAP
        assert costmap['inflation_layer']['cost_scaling_factor'] > 0.0
    assert (
        local_costmap['inflation_layer']['inflation_radius']
        == global_costmap['inflation_layer']['inflation_radius']
    )

    # Costmap resolution must match the saved map exactly.
    assert global_costmap['resolution'] == 0.05
    assert local_costmap['resolution'] == 0.05

    # Sensor ranges stay inside each sensor's own maximum, and raytracing
    # reaches slightly past marking so free space clears properly.
    # The ToF ring marks the local costmap only. The global costmap does not
    # roll, so a near-field sensor feeding it accumulates every transient mark
    # for the whole run until the robot is enclosed by its own history.
    assert global_costmap['obstacle_layer'][
        'observation_sources'].split() == ['scan']
    assert local_costmap['obstacle_layer'][
        'observation_sources'].split() == ['scan'] + list(TOF_SOURCES)

    for costmap in (global_costmap, local_costmap):
        layer = costmap['obstacle_layer']
        # Space separated, not a YAML list. collision_monitor.yaml uses a list
        # for the same key and the two forms are not interchangeable.
        sources = layer['observation_sources'].split()
        for name in sources:
            source = layer[name]
            assert source['raytrace_max_range'] > source['obstacle_max_range']
            assert source['obstacle_min_range'] == 0.0

        scan = layer['scan']
        assert scan['topic'] == '/scan'
        assert scan['data_type'] == 'LaserScan'
        assert scan['obstacle_max_range'] < LIDAR_MAX_RANGE
        assert scan['raytrace_max_range'] < LIDAR_MAX_RANGE

        # The ToF sources carry pre-classified points: the floor is filtered
        # out upstream, so these must never be asked to do it with a height
        # cut, and they must stay inside the geometric ceiling.
        for name in [n for n in sources if n in TOF_SOURCES]:
            source = layer[name]
            assert source['topic'] == f'/{name.replace("_", "/", 1)}'
            assert source['data_type'] == 'PointCloud2'
            assert source['marking'] is True
            assert source['obstacle_max_range'] <= TOF_WALL_RANGE
            assert source['raytrace_max_range'] < TOF_SENSOR_RANGE
            # Overhangs above the roof are dropped by the classifier; what it
            # still publishes above the roof is confirmed wall, which past
            # ~1 m is only ever seen above it. So the cut here must let walls
            # through: up to where the top row meets a wall at the 2 m wall
            # range (28.5 mm + 2 m * tan 30 deg, about 1.18 m).
            assert source['max_obstacle_height'] > ROBOT_HEIGHT
            assert source['max_obstacle_height'] >= 1.2

    # Frames. Nav2 defaults robot_base_frame to base_link, which this repo
    # does not use for the odometry chain; the local costmap rolls in odom.
    assert global_costmap['global_frame'] == 'map'
    assert global_costmap['robot_base_frame'] == 'base_footprint'
    assert local_costmap['global_frame'] == 'odom'
    assert local_costmap['robot_base_frame'] == 'base_footprint'
    assert local_costmap['rolling_window'] is True

    # The latched map is missed without a transient-local subscription.
    assert global_costmap['static_layer']['map_subscribe_transient_local'
                                          ] is True

    # Layer choices: no VoxelLayer, since there is no 3D sensor to fill it.
    assert global_costmap['plugins'] == [
        'static_layer', 'obstacle_layer', 'inflation_layer'
    ]
    assert local_costmap['plugins'] == ['obstacle_layer', 'inflation_layer']

    # Jazzy requires the '::' plugin separator; '/' strings fail at load.
    assert planner['planner_plugins'] == ['GridBased']
    assert planner['GridBased']['plugin'] == 'nav2_smac_planner::SmacPlanner2D'
    assert planner['GridBased']['allow_unknown'] is False
    for config in (planner_yaml, local_yaml, controller_yaml,
                   behavior_yaml, bt_yaml):
        plugins = _plugin_strings(config)
        assert plugins
        for plugin in plugins:
            assert '::' in plugin
            assert '/' not in plugin

    harness_launch = _read(
        navigation / 'launch' / 'planning_harness.launch.py'
    )
    planning_launch = _read(navigation / 'launch' / 'planning.launch.py')

    # The harness stays planning-only: it serves the map and fakes the frame
    # chain. The local costmap belongs to controller_server now, and the
    # command chain needs a robot, so neither appears here.
    assert "package='nav2_map_server'" in harness_launch
    assert "package='nav2_planner'" in harness_launch
    assert "package='tf2_ros'" in harness_launch
    assert "'node_names': ['map_server', 'planner_server']" in harness_launch
    assert "package='nav2_costmap_2d'" not in harness_launch
    assert "package='nav2_controller'" not in harness_launch
    assert "DeclareLaunchArgument(\n            'map'," in harness_launch

    # The full-simulation launch adds planning to the Phase 2 stack, which
    # already owns map_server, and never enables the velocity smoother.
    assert 'localization.launch.py' in planning_launch
    assert "package='nav2_map_server'" not in planning_launch
    for server in (
        'controller_server', 'planner_server', 'behavior_server',
        'bt_navigator', 'velocity_smoother', 'collision_monitor',
    ):
        assert f"'{server}'," in planning_launch
    assert "'velocity_smoother': 'false'" in planning_launch
    assert "'rviz': 'false'" in planning_launch

    # The goal bridge turns RViz clicks into planning requests only. It is
    # the one piece that touches the goal topic, so it carries the risk.
    bridge = _read(navigation / 'scripts' / 'goal_to_plan.py')
    assert 'ComputePathToPose' in bridge
    assert 'compute_path_to_pose' in bridge
    assert 'use_start' in bridge
    assert 'NavigateToPose' not in bridge
    assert 'navigate_to_pose' not in bridge
    assert 'cmd_vel' not in bridge
    # The bridge stays in the harness, where planning-only preview is the
    # point. It is gone from the full stack: bt_navigator subscribes to
    # /goal_pose itself, so keeping the bridge would fire a redundant
    # ComputePathToPose on every click.
    assert "executable='goal_to_plan'" in harness_launch
    assert "executable='goal_to_plan'" not in planning_launch

    # --- Holonomic traps. Nav2's stock defaults are diff-drive values that
    # fail silently on a mecanum base, so each one is pinned here.

    # Stock is 0.5, which would zero every lateral command this robot can
    # produce: its whole validated envelope is 0.10 m/s.
    assert controller['min_y_velocity_threshold'] <= 0.01
    assert controller['min_x_velocity_threshold'] <= 0.01

    follow_path = controller['FollowPath']
    assert follow_path['plugin'] == \
        'nav2_rotation_shim_controller::RotationShimController'
    assert follow_path['primary_controller'] == \
        'nav2_mppi_controller::MPPIController'
    # Stock is "DiffDrive", which never samples lateral motion. The installed
    # library reports the valid options as DiffDrive, Omni and Ackermann.
    assert follow_path['motion_model'] == 'Omni'
    # A zero lateral sampling spread disables strafing regardless of the model.
    assert follow_path['vy_std'] > 0.0
    assert follow_path['vy_max'] > 0.0

    # Speeds may exceed the characterised envelope, but not without bound.
    assert follow_path['vx_max'] <= VALIDATED_LINEAR * ENVELOPE_MULTIPLE
    assert follow_path['vy_max'] <= VALIDATED_LINEAR * ENVELOPE_MULTIPLE
    assert follow_path['wz_max'] <= VALIDATED_ANGULAR * ENVELOPE_MULTIPLE

    # Fast enough to outrun the ToF ring and it is decoration.
    assert follow_path['vx_max'] <= TOF_SAFE_LINEAR
    assert follow_path['vy_max'] <= TOF_SAFE_LINEAR

    # PreferForwardCritic penalises the lateral and reverse motion a mecanum
    # base exists to use.
    assert 'PreferForwardCritic' not in follow_path['critics']

    # A goal tolerance larger than the robot would count the goal as reached a
    # body-length away; smaller than one control tick cannot converge.
    goal_checker = controller['general_goal_checker']
    tick = follow_path['vx_max'] / controller['controller_frequency']
    assert tick <= goal_checker['xy_goal_tolerance'] <= ROBOT_LENGTH

    # Frames. Nav2 defaults these to base_link in both servers. The
    # controller's own costmap frame is checked with the costmaps above.
    assert behavior['robot_base_frame'] == 'base_footprint'
    assert bt_navigator['robot_base_frame'] == 'base_footprint'
    # Nav2 defaults every one of these to /odom, which has no publisher in
    # this system. A dead odometry subscription is silent: the node stays
    # active and simply believes the robot never moves. controller_server is
    # the one that bites, because MPPI feeds measured velocity back into its
    # optimiser.
    assert bt_navigator['odom_topic'] == ODOM_TOPIC
    assert controller['odom_topic'] == ODOM_TOPIC

    behavior_tree_name = next(
        path.name
        for path in sorted((navigation / 'behavior_trees').iterdir())
        if path.suffix == '.xml'
    )

    # --- Recoveries. Motion recoveries are permitted from Phase 3c, but only
    # inside the characterised envelope, and the tree must still try the
    # actuation-free options first.
    assert 'wait' in behavior['behavior_plugins']
    assert behavior['max_rotational_vel'] <= (
        VALIDATED_ANGULAR * ENVELOPE_MULTIPLE)

    bt_xml = navigation / 'behavior_trees' / behavior_tree_name
    tree = _read(bt_xml)
    assert 'ClearEntireCostmap' in tree
    assert '<Wait ' in tree
    assert bt_xml.name in planning_launch
    # Costmap clearing must be attempted before any commanded motion.
    if '<Spin ' in tree:
        assert tree.index('ClearingActions') < tree.index('<Spin ')
        assert 'spin' in behavior['behavior_plugins']
    if '<BackUp ' in tree:
        assert tree.index('ClearingActions') < tree.index('<BackUp ')
        assert 'backup' in behavior['behavior_plugins']

    # --- Arbitration. Teleop outranks autonomy so the keyboard always wins.
    assert mux['topics']['teleop']['priority'] > (
        mux['topics']['navigation']['priority'])

    # With both controllers doubled past anything characterised, the velocity
    # smoother is the last thing bounding them. Assert the relationship, not a
    # number: a controller commanding past the smoother's ceiling is asking
    # for a limit that silently never arrives.
    smoother = _parameters(
        navigation.parent / 'mobile_base_bringup' / 'config' /
        'velocity_smoother.yaml', 'velocity_smoother')
    smoother_x, smoother_y, smoother_theta = smoother['max_velocity']
    assert follow_path['vx_max'] <= smoother_x
    assert follow_path['vy_max'] <= smoother_y
    assert follow_path['wz_max'] <= smoother_theta
    # The lateral term must stay non-zero or strafing is silently forbidden.
    assert smoother_y > 0.0

    # nav2_util shares one enable_stamped_cmd_vel per node, so every hop must
    # agree. The controller's reference interface is TwistStamped.
    assert mux['use_stamped'] is True
    assert monitor['enable_stamped_cmd_vel'] is True
    assert "'enable_stamped_cmd_vel': True" in planning_launch

    # The collision monitor is last so it gates teleop too, and slows rather
    # than hard-stopping so an operator can still escape a corner.
    actions = {
        name: monitor[name]['action_type'] for name in monitor['polygons']
    }
    # A limiting zone, not a hard stop: the monitor gates autonomy and a bare
    # stop zone near an obstacle would strand the robot. "limit" clamps to an
    # absolute ceiling, where "slowdown" only scales the incoming command.
    assert 'limit' in actions.values()
    creep = next(monitor[name] for name in monitor['polygons']
                 if monitor[name]['action_type'] == 'limit')
    # The zone must cover the footprint plus the measured stopping distance.
    assert creep['radius'] >= local_costmap['robot_radius'] + STOP_DISTANCE
    assert creep['linear_limit'] < controller['FollowPath']['vx_max']
    # Height filters exist only on the PointCloud source, so copying them onto
    # a scan source would imply a filter that is never applied.
    assert 'min_height' not in monitor['scan']
    assert 'max_height' not in monitor['scan']

    # --- Single arbiter. Exactly one node may publish the driver-facing
    # command, and it must be the mux. Anything else reaching that topic is a
    # bypass of both arbitration and the e-stop.
    # A bypass would have to be wired somewhere executable, so the sweep
    # covers launch files and node sources. YAML may name the topic in
    # comments; documenting the contract is not a bypass of it.
    needle = COMMAND_TOPIC.encode('utf-8')
    naming = sorted(
        path.name
        for directory in ('launch', 'scripts')
        for path in (navigation / directory).rglob('*')
        if path.is_file()
        and '__pycache__' not in path.parts
        and needle in path.read_bytes()
    )
    # Only the launch file wires it, and only once.
    assert naming == ['planning.launch.py'], naming
    assert planning_launch.count(COMMAND_TOPIC) == 1
    arbiter_remap = (
        f"remappings=[\n                ('cmd_vel_out', '{COMMAND_TOPIC}'),")
    assert arbiter_remap in planning_launch
    assert f"package='{ARBITER}'" in planning_launch

    # --- Chain topology, asserted by name so a reorder cannot pass silently.
    # controller/behavior -> [speed governor] -> cmd_vel_nav -> smoother ->
    # cmd_vel_smoothed -> collision monitor -> autonomy_cmd -> mux ->
    # controller reference.
    assert monitor['cmd_vel_in_topic'] == 'cmd_vel_smoothed'
    assert monitor['cmd_vel_out_topic'] == 'autonomy_cmd'
    assert mux['topics']['navigation']['topic'] == 'autonomy_cmd'
    # The smoother always reads cmd_vel_nav.
    assert planning_launch.count("('cmd_vel', 'cmd_vel_nav')") == 1
    # Controller and behaviours publish to nav_cmd_topic: cmd_vel_nav
    # directly, or the governor's input when speed_governor:=true...
    assert planning_launch.count(
        "('cmd_vel', LaunchConfiguration('nav_cmd_topic'))") == 2
    assert "'/cmd_vel_ungoverned' if '" in planning_launch
    assert "' == 'true' else 'cmd_vel_nav'" in planning_launch
    # ...and the governor closes the gap into cmd_vel_nav.
    assert ("{'cmd_in': '/cmd_vel_ungoverned', 'cmd_out': '/cmd_vel_nav'}"
            in planning_launch)

    # --- The e-stop lock outranks every command source, teleop included.
    lock = mux['locks']['estop']
    assert lock['priority'] > max(
        entry['priority'] for entry in mux['topics'].values())
    # A lock with timeout > 0 is locked until a heartbeat says otherwise, so
    # this is what makes a dead gate node stop the robot. Zero would disable
    # the deadman entirely.
    assert lock['timeout'] > 0.0

    estop = _parameters(navigation / 'config' / 'estop.yaml', 'estop_gate')
    assert estop['lock_topic'] == lock['topic']
    # The heartbeat must be comfortably inside the lock timeout or the lock
    # expires between beats and the robot stutters.
    assert estop['publish_frequency'] >= 4.0 / lock['timeout']
    gate = _read(navigation / 'scripts' / 'estop_gate.py')
    assert 'Trigger' in gate and '~/reset' in gate and '~/engage' in gate
    assert "executable='estop_gate'" in planning_launch

    # --- Deferred startup. The costmaps must not come up before AMCL has a
    # pose from a human: an "active" global costmap built on an unset pose
    # shows the map smeared against whatever transform happened to be there.
    assert "DeclareLaunchArgument('autostart', default_value='false')" in (
        planning_launch)
    assert "executable='nav_autostart'" in planning_launch
    gate = _read(navigation / 'scripts' / 'nav_autostart.py')
    # /initialpose, the message RViz's 2D Pose Estimate publishes, and NOT
    # /amcl_pose or the map -> odom transform. Measured: with the robot
    # stationary AMCL publishes neither, because its filter updates on motion
    # and nothing can move until navigation is up - the very thing being
    # waited for. /initialpose has no such circularity.
    assert '/initialpose' in gate
    assert "'/amcl_pose'" not in gate
    assert 'ManageLifecycleNodes' in gate
    assert 'STARTUP' in gate
    # The gate may trigger the lifecycle manager and nothing else. A gate that
    # could publish a command would be a second arbiter.
    assert 'cmd_vel' not in gate
    assert COMMAND_TOPIC not in gate

    package = ET.parse(navigation / 'package.xml').getroot()
    # <depend> covers build and exec together, which is what the plugin
    # dependencies use; the runtime-only Nav2 servers stay <exec_depend>.
    dependencies = {
        element.text
        for tag in ('exec_depend', 'depend')
        for element in package.findall(tag)
    }
    assert {
        'nav2_behaviors',
        'nav2_bt_navigator',
        'nav2_collision_monitor',
        'nav2_controller',
        'nav2_costmap_2d',
        'nav2_lifecycle_manager',
        'nav2_map_server',
        'nav2_planner',
        'nav2_smac_planner',
        'nav2_velocity_smoother',
        'twist_mux',
    } <= dependencies

    navigation_rviz = _read(navigation / 'rviz' / 'navigation.rviz')
    assert 'Fixed Frame: map' in navigation_rviz
    assert '/global_costmap/costmap' in navigation_rviz
    assert '/global_costmap/published_footprint' in navigation_rviz
    assert '/plan' in navigation_rviz
    assert 'Color Scheme: costmap' in navigation_rviz
    # The goal tool is a pure publisher, so clicking a goal cannot drive the
    # robot. The Nav2 panel is the part that talks to bt_navigator, and it
    # must stay out along with nav2_rviz_plugins' own goal tool.
    assert 'rviz_default_plugins/SetGoal' in navigation_rviz
    assert '/goal_pose' in navigation_rviz
    assert 'nav2_rviz_plugins/GoalTool' not in navigation_rviz
    assert 'nav2_rviz_plugins/Navigation 2' not in navigation_rviz


if __name__ == '__main__':
    main()
