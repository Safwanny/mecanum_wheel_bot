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

# Navigate an unknown environment: live SLAM, no saved map, every sensor.
#
#   localization - wheels + gyro (bias-corrected) + rf2o laser odometry in
#                  the EKF; slam_toolbox mapping live owns map -> odom.
#   planning     - Nav2 on the growing map, planning through unknown space.
#   safety       - ToF classifier into the local costmap; the speed governor
#                  caps every command by what the ring sees ahead.
#   fallback     - when Nav2 aborts, nav_supervisor hands the goal to DistBug
#                  and back once it has cleared the blockage.
#   exploration  - explore:=true maps the whole place by itself.
#
# Needs the sources in mobile_base.repos built (rf2o, explore_lite).

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('mobile_base_navigation')
    tools_config = PathJoinSubstitution([
        FindPackageShare('mobile_base_tools'), 'config',
        'bug_navigation.yaml'])
    sim_time = {'use_sim_time': True}
    arguments = [
        ('world', 'navigation_basic'), ('x', '0.0'), ('y', '0.0'),
        ('yaw', '0.0'), ('gui', 'false'), ('rviz', 'true'),
        ('motion_profile', 'holonomic'),
        ('explore', 'false'),
        # Consumed by the included simulation launch through the shared
        # launch context.
        ('laser_odometry', 'true'),
        ('sensor_panel', 'true'),
    ]
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default)
          for name, default in arguments],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                share, 'launch', 'planning.launch.py'])),
            launch_arguments={
                'localization': 'slam',
                'speed_governor': 'true',
                'world': LaunchConfiguration('world'),
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'yaw': LaunchConfiguration('yaw'),
                'gui': LaunchConfiguration('gui'),
                'rviz': LaunchConfiguration('rviz'),
                'motion_profile': LaunchConfiguration('motion_profile'),
                # The live SLAM map first, costmaps faint or off.
                'rviz_config': PathJoinSubstitution(
                    [share, 'rviz', 'explore.rviz']),
            }.items(),
        ),
        # DistBug as the fallback: it listens on its own goal topic, so RViz
        # goals go to Nav2, and drives into the same governed command path.
        Node(
            package='mobile_base_tools',
            executable='bug_navigator',
            name='bug_navigator',
            parameters=[tools_config, sim_time, {
                'goal_topic': '/bug_navigator/goal',
                'cmd_out': '/cmd_vel_ungoverned',
            }],
            output='screen',
        ),
        # Where SLAM corrected the pose, and how far it is from the truth.
        Node(
            package='mobile_base_tools',
            executable='localization_monitor',
            name='localization_monitor',
            parameters=[sim_time],
            output='screen',
        ),
        Node(
            package='mobile_base_tools',
            executable='nav_supervisor',
            name='nav_supervisor',
            parameters=[sim_time],
            output='screen',
        ),
        Node(
            package='explore_lite',
            executable='explore',
            name='explore_node',
            parameters=[
                PathJoinSubstitution([share, 'config', 'explore.yaml']),
                sim_time,
            ],
            condition=IfCondition(LaunchConfiguration('explore')),
            output='screen',
        ),
    ])
