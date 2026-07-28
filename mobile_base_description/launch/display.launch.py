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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model = PathJoinSubstitution(
        [FindPackageShare('mobile_base_description'), 'urdf', 'mobile_base.urdf.xacro']
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('mobile_base_description'), 'rviz', 'mobile_base.rviz']
    )
    robot_description = ParameterValue(
        Command(['xacro ', model, ' prefix:=', LaunchConfiguration('prefix')]),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'prefix',
                default_value='',
                description='Optional prefix applied to every robot frame.',
            ),
            DeclareLaunchArgument(
                'use_joint_state_gui',
                default_value='false',
                description='Use sliders for movable joints instead of a headless publisher.',
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[
                    {
                        'robot_description': robot_description,
                        'publish_frequency': 30.0,
                    }
                ],
                output='screen',
            ),
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                parameters=[
                    {
                        'rate': 30,
                        'publish_default_positions': True,
                    }
                ],
                condition=IfCondition(LaunchConfiguration('use_joint_state_gui')),
                output='screen',
            ),
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                parameters=[
                    {
                        'rate': 30,
                        'publish_default_positions': True,
                    }
                ],
                condition=UnlessCondition(LaunchConfiguration('use_joint_state_gui')),
                output='screen',
            ),
            TimerAction(
                period=1.0,
                actions=[
                    Node(
                        package='rviz2',
                        executable='rviz2',
                        arguments=['-d', rviz_config],
                        output='screen',
                    )
                ],
            ),
        ]
    )
