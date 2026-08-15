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

"""Launch the complete simulated robot in mutually exclusive mapping mode."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Compose Phase 1 simulation/EKF with slam_toolbox and mapping RViz."""
    simulation_launch = PathJoinSubstitution([
        FindPackageShare('mobile_base_bringup'),
        'launch',
        'simulation.launch.py',
    ])
    mapping_launch = PathJoinSubstitution([
        FindPackageShare('mobile_base_localization'),
        'launch',
        'mapping.launch.py',
    ])
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='navigation_basic'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('render_engine', default_value='ogre2'),
        # Preserve the wrapper's value before simulation.launch.py receives
        # rviz=false. Include launch arguments share the launch context.
        SetLaunchConfiguration(
            'phase2_rviz', LaunchConfiguration('rviz')
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulation_launch),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'use_sim_time': 'true',
                'gui': LaunchConfiguration('gui'),
                'rviz': 'false',
                'render_engine': LaunchConfiguration('render_engine'),
                'localization': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapping_launch),
            launch_arguments={
                'use_sim_time': 'true',
                'rviz': LaunchConfiguration('phase2_rviz'),
            }.items(),
        ),
    ])
