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

"""Launch costmaps and the global planner headlessly, without Gazebo."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _planning_nodes(context):
    requested = LaunchConfiguration('map').perform(context)
    map_path = Path(requested).expanduser().resolve()
    if map_path.suffix.lower() != '.yaml' or not map_path.is_file():
        raise RuntimeError(
            f'map must name an existing Nav2 map YAML file: {map_path}'
        )

    use_sim_time = LaunchConfiguration('use_sim_time')
    planner_config = LaunchConfiguration('planner_config')
    local_costmap_config = LaunchConfiguration('local_costmap_config')
    rviz_config = LaunchConfiguration('rviz_config')

    return [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{
                'yaml_filename': str(map_path),
                'frame_id': 'map',
                'topic_name': 'map',
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[
                planner_config,
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
        ),
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='local_costmap',
            parameters=[
                local_costmap_config,
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
        ),
        # The harness has no robot, so static transforms stand in for the
        # AMCL and EKF links and give the costmaps a complete frame chain.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='harness_map_to_odom',
            arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='harness_odom_to_base_footprint',
            arguments=[
                '--frame-id', 'odom', '--child-frame-id', 'base_footprint',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    'map_server', 'planner_server', 'local_costmap',
                ],
                'bond_timeout': 4.0,
            }],
            output='screen',
        ),
        # Turns RViz goal clicks into planning requests. It calls only
        # ComputePathToPose, so nothing here can drive the robot.
        Node(
            package='mobile_base_navigation',
            executable='goal_to_plan',
            name='goal_to_plan',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('goal_bridge')),
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='navigation_rviz',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
    ]


def generate_launch_description():
    """Start map_server, both costmaps, and planner_server. Nothing drives."""
    share = FindPackageShare('mobile_base_navigation')
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            description='Path to a saved Nav2 occupancy-map YAML file.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
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
        OpaqueFunction(function=_planning_nodes),
    ])
