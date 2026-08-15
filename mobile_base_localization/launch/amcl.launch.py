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

"""Launch saved-map serving and holonomic AMCL localization."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _localization_nodes(context):
    requested = LaunchConfiguration('map').perform(context)
    map_path = Path(requested).expanduser().resolve()
    if map_path.suffix.lower() != '.yaml' or not map_path.is_file():
        raise RuntimeError(
            f'map must name an existing Nav2 map YAML file: {map_path}'
        )

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_server_config = LaunchConfiguration('map_server_config')
    amcl_config = LaunchConfiguration('amcl_config')
    rviz_config = LaunchConfiguration('rviz_config')

    return [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[
                map_server_config,
                {
                    'yaml_filename': str(map_path),
                    'use_sim_time': use_sim_time,
                },
            ],
            output='screen',
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[
                amcl_config,
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
                'bond_timeout': 4.0,
            }],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='localization_rviz',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
    ]


def generate_launch_description():
    """Start map_server and AMCL, but never slam_toolbox."""
    share = FindPackageShare('mobile_base_localization')
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            description='Path to a saved Nav2 occupancy-map YAML file.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'map_server_config',
            default_value=PathJoinSubstitution([
                share, 'config', 'map_server.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'amcl_config',
            default_value=PathJoinSubstitution([
                share, 'config', 'amcl.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                share, 'rviz', 'localization.rviz',
            ]),
        ),
        OpaqueFunction(function=_localization_nodes),
    ])
