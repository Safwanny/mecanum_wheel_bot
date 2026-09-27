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

# ToF-only navigation: the simulation with the LiDAR off, Bug2 and the
# governor. No map, no SLAM, no Nav2. Give a goal with RViz's "2D Goal Pose";
# the robot translates to it on odometry, following boundaries with the ToF
# ring, at the speed the governor allows along its direction of travel.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('mobile_base_tools'), 'config', 'bug_navigation.yaml'])
    sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    arguments = [
        ('world', 'navigation_basic'), ('x', '0.0'), ('y', '0.0'),
        ('yaw', '0.0'), ('gui', 'false'), ('lidar', 'false'),
        # The governor's levels assume the smoother's braking limits, and it
        # is what listens on /mobile_base/cmd_vel.
        ('velocity_smoother', 'true'),
        ('use_sim_time', 'true'),
    ]
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default)
          for name, default in arguments],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('mobile_base_bringup'), 'launch',
                'simulation.launch.py'])),
            launch_arguments={
                name: LaunchConfiguration(name) for name, _ in arguments
            }.items(),
        ),
        Node(
            package='mobile_base_tools',
            executable='speed_governor',
            name='speed_governor',
            parameters=[config, sim_time],
            output='screen',
        ),
        Node(
            package='mobile_base_tools',
            executable='bug_navigator',
            name='bug_navigator',
            parameters=[config, sim_time],
            output='screen',
        ),
    ])
