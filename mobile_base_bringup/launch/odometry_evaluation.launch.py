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
            'profile_sequence': LaunchConfiguration('profile_sequence'),
            'repetitions': LaunchConfiguration('repetitions'),
            'repetition_offset': LaunchConfiguration('repetition_offset'),
            'evaluation_mode': LaunchConfiguration('evaluation_mode'),
            'localization': LaunchConfiguration('localization'),
            'profile_order': LaunchConfiguration('profile_order'),
            'random_seed': LaunchConfiguration('random_seed'),
            'diagnostic_verbose': LaunchConfiguration('diagnostic_verbose'),
            'wall_watchdog_factor': LaunchConfiguration(
                'wall_watchdog_factor'),
            'minimum_wall_watchdog': LaunchConfiguration(
                'minimum_wall_watchdog'),
            'clock_stall_timeout': LaunchConfiguration(
                'clock_stall_timeout'),
            'motion_start_timeout': LaunchConfiguration(
                'motion_start_timeout'),
            'wheels_radius': LaunchConfiguration('wheels_radius'),
            'center_projection_sum': LaunchConfiguration(
                'center_projection_sum'),
            'roller_joint_damping': LaunchConfiguration(
                'roller_joint_damping'),
            'roller_joint_friction': LaunchConfiguration(
                'roller_joint_friction'),
            'roller_contact_mu': LaunchConfiguration('roller_contact_mu'),
            'roller_collision_model': LaunchConfiguration(
                'roller_collision_model'),
            'wheel_contact_mu': LaunchConfiguration('wheel_contact_mu'),
            'wheel_contact_mu2': LaunchConfiguration('wheel_contact_mu2'),
            'wheel_contact_slip1': LaunchConfiguration('wheel_contact_slip1'),
            'physics_max_step_size': LaunchConfiguration(
                'physics_max_step_size'),
            'front_left_roller_phase': LaunchConfiguration(
                'front_left_roller_phase'),
            'front_right_roller_phase': LaunchConfiguration(
                'front_right_roller_phase'),
            'rear_right_roller_phase': LaunchConfiguration(
                'rear_right_roller_phase'),
            'rear_left_roller_phase': LaunchConfiguration(
                'rear_left_roller_phase'),
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
            'render_engine': LaunchConfiguration('render_engine'),
            'physics_max_step_size': LaunchConfiguration(
                'physics_max_step_size'),
            'roller_joint_damping': LaunchConfiguration(
                'roller_joint_damping'),
            'roller_joint_friction': LaunchConfiguration(
                'roller_joint_friction'),
            'roller_contact_mu': LaunchConfiguration('roller_contact_mu'),
            'roller_collision_model': LaunchConfiguration(
                'roller_collision_model'),
            'wheel_contact_mu': LaunchConfiguration('wheel_contact_mu'),
            'wheel_contact_mu2': LaunchConfiguration('wheel_contact_mu2'),
            'wheel_contact_slip1': LaunchConfiguration('wheel_contact_slip1'),
            'front_left_roller_phase': LaunchConfiguration(
                'front_left_roller_phase'),
            'front_right_roller_phase': LaunchConfiguration(
                'front_right_roller_phase'),
            'rear_right_roller_phase': LaunchConfiguration(
                'rear_right_roller_phase'),
            'rear_left_roller_phase': LaunchConfiguration(
                'rear_left_roller_phase'),
        }.items(),
    )
    default_config = (
        get_package_share_directory('mobile_base_tools')
        + '/config/odometry_tests.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty'),
        DeclareLaunchArgument('test_profile', default_value='all'),
        DeclareLaunchArgument('profile_sequence', default_value=''),
        DeclareLaunchArgument(
            'evaluation_mode', default_value='raw_and_filtered'),
        DeclareLaunchArgument('localization', default_value='true'),
        DeclareLaunchArgument('repetitions', default_value='0'),
        DeclareLaunchArgument('repetition_offset', default_value='0'),
        DeclareLaunchArgument('profile_order', default_value='configured'),
        DeclareLaunchArgument('random_seed', default_value='0'),
        DeclareLaunchArgument('diagnostic_verbose', default_value='false'),
        DeclareLaunchArgument('wall_watchdog_factor', default_value='3.0'),
        DeclareLaunchArgument('minimum_wall_watchdog', default_value='30.0'),
        DeclareLaunchArgument('clock_stall_timeout', default_value='5.0'),
        DeclareLaunchArgument('motion_start_timeout', default_value='2.0'),
        DeclareLaunchArgument('wheels_radius', default_value='0.03074443'),
        DeclareLaunchArgument('center_projection_sum', default_value='0.142'),
        DeclareLaunchArgument(
            'output_dir', default_value='/tmp/mobile_base_phase1'
        ),
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('robot_entity', default_value='mobile_base'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('render_engine', default_value='ogre'),
        DeclareLaunchArgument(
            'physics_max_step_size', default_value='0.001'),
        DeclareLaunchArgument('roller_joint_damping', default_value='0.0'),
        DeclareLaunchArgument(
            'roller_joint_friction', default_value='0.0'),
        DeclareLaunchArgument('roller_contact_mu', default_value='1.0'),
        DeclareLaunchArgument(
            'roller_collision_model', default_value='barrel'),
        DeclareLaunchArgument('wheel_contact_mu', default_value='0.8'),
        DeclareLaunchArgument('wheel_contact_mu2', default_value='0.2'),
        DeclareLaunchArgument('wheel_contact_slip1', default_value='0.0'),
        DeclareLaunchArgument(
            'front_left_roller_phase', default_value='0.22193969'),
        DeclareLaunchArgument(
            'front_right_roller_phase', default_value='0.48030419'),
        DeclareLaunchArgument(
            'rear_right_roller_phase', default_value='0.19668582'),
        DeclareLaunchArgument(
            'rear_left_roller_phase', default_value='0.24790784'),
        DeclareLaunchArgument('shutdown_on_complete', default_value='true'),
        simulation,
        OpaqueFunction(function=_evaluation_actions),
    ])
