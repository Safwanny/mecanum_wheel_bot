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

"""Launch deterministic raw wheel-odometry characterization."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _evaluation_actions(context):
    world = LaunchConfiguration('world').perform(context)
    pose_topic = f'/world/{world}/dynamic_pose/info'
    set_pose_service = f'/world/{world}/set_pose'
    ground_truth_topic = '/mobile_base/evaluation/ground_truth'

    reset_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odometry_evaluation_bridge',
        arguments=[
            (
                set_pose_service
                + '@ros_gz_interfaces/srv/SetEntityPose'
            ),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    ground_truth_selector = Node(
        package='mobile_base_evaluation',
        executable='ground_truth_selector',
        parameters=[{
            'use_sim_time': True,
            'world': world,
            'gz_topic': pose_topic,
            'robot_entity': LaunchConfiguration('robot_entity'),
            'output_topic': ground_truth_topic,
        }],
        output='screen',
    )
    evaluator = Node(
        package='mobile_base_tools',
        executable='odometry_test_runner',
        parameters=[{
            'use_sim_time': True,
            'config_file': LaunchConfiguration('config_file'),
            'output_dir': LaunchConfiguration('output_dir'),
            'test_profile': LaunchConfiguration('test_profile'),
            'repetitions': LaunchConfiguration('repetitions'),
            'evaluation_mode': LaunchConfiguration('evaluation_mode'),
            'world': world,
            'robot_entity': LaunchConfiguration('robot_entity'),
            'ground_truth_topic': ground_truth_topic,
            'set_pose_service': set_pose_service,
        }],
        output='screen',
    )
    shutdown = RegisterEventHandler(
        OnProcessExit(
            target_action=evaluator,
            on_exit=[
                EmitEvent(
                    event=Shutdown(reason='odometry evaluation finished'),
                    condition=IfCondition(
                        LaunchConfiguration('shutdown_on_complete')
                    ),
                )
            ],
        )
    )
    return [reset_bridge, ground_truth_selector, evaluator, shutdown]


def generate_launch_description():
    """Compose the standard simulation with evaluation-only interfaces."""
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('mobile_base_bringup')
            + '/launch/simulation.launch.py'
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
            'use_sim_time': 'true',
            'localization': LaunchConfiguration('localization'),
        }.items(),
    )
    default_config = (
        get_package_share_directory('mobile_base_tools')
        + '/config/odometry_tests.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty'),
        DeclareLaunchArgument('test_profile', default_value='all'),
        DeclareLaunchArgument(
            'evaluation_mode', default_value='raw_and_filtered'),
        DeclareLaunchArgument('localization', default_value='true'),
        DeclareLaunchArgument('repetitions', default_value='0'),
        DeclareLaunchArgument(
            'output_dir', default_value='/tmp/mobile_base_phase1'
        ),
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('robot_entity', default_value='mobile_base'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('shutdown_on_complete', default_value='true'),
        simulation,
        OpaqueFunction(function=_evaluation_actions),
    ])
