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

"""Launch the simulated robot with costmaps and the global planner."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Add planning to the Phase 2 localization stack. Nothing drives."""
    share = FindPackageShare('mobile_base_navigation')
    localization_launch = PathJoinSubstitution([
        FindPackageShare('mobile_base_bringup'),
        'launch',
        'localization.launch.py',
    ])
    use_sim_time = {'use_sim_time': True}
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            description='Path to a saved Nav2 occupancy-map YAML file.',
        ),
        DeclareLaunchArgument('world', default_value='navigation_basic'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('render_engine', default_value='ogre2'),
        # The costmaps cannot activate until AMCL publishes map -> odom, and
        # AMCL waits for an initial pose. Set autostart:=false to bring the
        # planning nodes up by hand once the particle cloud has converged:
        #   ros2 service call /lifecycle_manager_navigation/manage_nodes \
        #     nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('goal_bridge', default_value='true'),
        DeclareLaunchArgument(
            'planner_config',
            default_value=PathJoinSubstitution([
                share, 'config', 'planner.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'local_costmap_config',
            default_value=PathJoinSubstitution([
                share, 'config', 'local_costmap.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                share, 'rviz', 'navigation.rviz',
            ]),
        ),
        # Preserve the wrapper's value before the included stack receives
        # rviz=false. Include launch arguments share the launch context.
        SetLaunchConfiguration(
            'phase3_rviz', LaunchConfiguration('rviz')
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'world': LaunchConfiguration('world'),
                'gui': LaunchConfiguration('gui'),
                'rviz': 'false',
                'render_engine': LaunchConfiguration('render_engine'),
                # Phase 3a must not be able to command motion, and the
                # smoother remaps onto the controller reference topic.
                'velocity_smoother': 'false',
            }.items(),
        ),
        # map_server is owned by the included localization stack, so it is
        # neither started nor managed here.
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[
                LaunchConfiguration('planner_config'),
                use_sim_time,
            ],
            output='screen',
        ),
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='local_costmap',
            parameters=[
                LaunchConfiguration('local_costmap_config'),
                use_sim_time,
            ],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[{
                'use_sim_time': True,
                'autostart': ParameterValue(
                    LaunchConfiguration('autostart'), value_type=bool),
                'node_names': ['planner_server', 'local_costmap'],
                # Bond heartbeats are unreliable under sim time: the
                # standalone costmap node activates normally but is never
                # reached by bond, and the manager then reports a spurious
                # bringup failure. 0.0 disables the bond check, which is what
                # Nav2 itself does in simulation. The harness, which runs on
                # wall time, keeps the repo's usual 4.0 s.
                'bond_timeout': 0.0,
            }],
            output='screen',
        ),
        # Turns RViz goal clicks into planning requests. It calls only
        # ComputePathToPose, so nothing here can drive the robot.
        Node(
            package='mobile_base_navigation',
            executable='goal_to_plan',
            name='goal_to_plan',
            parameters=[use_sim_time],
            condition=IfCondition(LaunchConfiguration('goal_bridge')),
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='navigation_rviz',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[use_sim_time],
            condition=IfCondition(LaunchConfiguration('phase3_rviz')),
            output='screen',
        ),
    ])
