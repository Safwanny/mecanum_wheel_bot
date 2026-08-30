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

"""Validate the static Phase 3a costmap, planner, and no-motion contracts."""

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

# Nothing in this package may command the robot.
COMMAND_TOPIC = '/mobile_base_controller/reference'
FORBIDDEN_COMPONENTS = (
    'controller_server',
    'bt_navigator',
    'behavior_server',
    'twist_mux',
    'collision_monitor',
)


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

    planner = _parameters(planner_yaml, 'planner_server')
    global_costmap = _parameters(planner_yaml, 'global_costmap',
                                 'global_costmap')
    local_costmap = _parameters(local_yaml, 'local_costmap')

    # Footprint must cover the geometry the description actually declares, so
    # the two cannot drift apart.
    properties = _xacro_properties(
        description / 'urdf' / 'properties.xacro'
    )
    wheelbase = float(properties['wheelbase'])
    wheel_separation = float(properties['wheel_separation'])
    wheel_width = float(properties['wheel_width'])
    chassis_length = float(properties['chassis_length'])
    half_length = max(chassis_length, wheelbase) / 2.0
    half_width = (wheel_separation + wheel_width) / 2.0
    circumscribed = hypot(half_length, half_width)
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

    # Sensor ranges stay inside the LiDAR's own maximum, and raytracing
    # reaches slightly past marking so free space clears properly.
    for costmap in (global_costmap, local_costmap):
        scan = costmap['obstacle_layer']['scan']
        assert scan['topic'] == '/scan'
        assert scan['data_type'] == 'LaserScan'
        assert scan['obstacle_max_range'] < LIDAR_MAX_RANGE
        assert scan['raytrace_max_range'] < LIDAR_MAX_RANGE
        assert scan['raytrace_max_range'] > scan['obstacle_max_range']

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
    for config in (planner_yaml, local_yaml):
        plugins = _plugin_strings(config)
        assert plugins
        for plugin in plugins:
            assert '::' in plugin
            assert '/' not in plugin

    harness_launch = _read(
        navigation / 'launch' / 'planning_harness.launch.py'
    )
    planning_launch = _read(navigation / 'launch' / 'planning.launch.py')

    # The harness stands alone: it serves the map and fakes the frame chain.
    assert "package='nav2_map_server'" in harness_launch
    assert "package='nav2_planner'" in harness_launch
    assert "package='nav2_costmap_2d'" in harness_launch
    assert "package='tf2_ros'" in harness_launch
    assert "'map_server', 'planner_server', 'local_costmap'," in harness_launch
    assert "DeclareLaunchArgument(\n            'map'," in harness_launch

    # The full-simulation launch adds planning to the Phase 2 stack, which
    # already owns map_server, and never enables the velocity smoother.
    assert 'localization.launch.py' in planning_launch
    assert "package='nav2_map_server'" not in planning_launch
    assert "'node_names': ['planner_server', 'local_costmap']" in (
        planning_launch
    )
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
    for launch in (harness_launch, planning_launch):
        assert "executable='goal_to_plan'" in launch

    # Phase 3a must be structurally incapable of moving the robot. The
    # bridge is excluded here and checked above instead: this is a blunt
    # substring test, and the node's docstring legitimately explains what it
    # avoids. Its NavigateToPose assertions cover the behaviour that matters.
    for source in (
        _read(planner_yaml),
        _read(local_yaml),
        harness_launch,
        planning_launch,
    ):
        for component in FORBIDDEN_COMPONENTS:
            assert component not in source
    tests = navigation / 'test'
    for path in sorted(navigation.rglob('*')):
        # This file names the topic in order to forbid it.
        if path.is_file() and tests not in path.parents:
            assert COMMAND_TOPIC not in _read(path)

    package = ET.parse(navigation / 'package.xml').getroot()
    dependencies = {element.text for element in package.findall('exec_depend')}
    assert {
        'nav2_costmap_2d',
        'nav2_lifecycle_manager',
        'nav2_map_server',
        'nav2_planner',
        'nav2_smac_planner',
    } <= dependencies
    assert 'nav2_controller' not in dependencies
    assert 'nav2_bt_navigator' not in dependencies
    assert 'nav2_behaviors' not in dependencies

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
