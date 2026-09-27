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

"""Launch the planar robot_localization EKF and the sources it fuses."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Start the EKF, the gyro bias estimator and optional laser odometry.

    Laser odometry needs rf2o_laser_odometry, built from mobile_base.repos,
    so it is off unless asked for; the EKF simply has one input fewer.
    """
    default_config = PathJoinSubstitution([
        FindPackageShare('mobile_base_localization'), 'config', 'ekf.yaml',
    ])
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('ekf_config_file', default_value=default_config),
        DeclareLaunchArgument('laser_odometry', default_value='false'),
        # The EKF reads the gyro through this: bias learned at every stop.
        Node(
            package='mobile_base_tools',
            executable='imu_bias',
            name='imu_bias',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
        # rf2o scan matching -> body twist with covariances -> EKF odom1.
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'laser_scan_topic': '/scan',
                'odom_topic': '/odom_rf2o',
                # The EKF owns odom -> base_footprint; rf2o only reports.
                'publish_tf': False,
                'base_frame_id': 'base_footprint',
                'odom_frame_id': 'laser_odom',
                'init_pose_from_topic': '',
                'freq': 10.0,
            }],
            # rf2o logs every scan at INFO and warns between scans; that
            # buried every other node's output in the launch log.
            ros_arguments=['--log-level', 'error'],
            condition=IfCondition(LaunchConfiguration('laser_odometry')),
        ),
        Node(
            package='mobile_base_tools',
            executable='laser_odometry',
            name='laser_odometry',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            condition=IfCondition(LaunchConfiguration('laser_odometry')),
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                LaunchConfiguration('ekf_config_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[('odometry/filtered', '/odometry/filtered')],
        ),
    ])
