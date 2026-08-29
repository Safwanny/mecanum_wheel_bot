#!/usr/bin/env python3
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

"""Verify semantic launch/config fields for teleop and odom visualization."""

from pathlib import Path
import sys
import xml.etree.ElementTree as ET


def read(path):
    return Path(path).read_text(encoding='utf-8')


def main():
    bringup = Path(sys.argv[1])
    description = Path(sys.argv[2])
    gazebo = Path(sys.argv[3])

    controllers = read(bringup / 'config' / 'controllers.yaml')
    assert 'base_frame_id: base_footprint' in controllers
    assert 'odom_frame_id: odom' in controllers
    assert 'reference_timeout: 0.5' in controllers

    launch = read(bringup / 'launch' / 'simulation.launch.py')
    assert 'mobile_base_sim.rviz' in launch
    assert "'simulation_rviz', LaunchConfiguration('rviz')" in launch
    assert "IfCondition(LaunchConfiguration('simulation_rviz'))" in launch
    assert 'mobile_base_controller/tf_odometry:=/tf' in launch
    assert repr(('tf', '/tf')) in launch
    assert repr(('tf_static', '/tf_static')) in launch
    assert "'/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'" in launch
    assert "'/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'" in launch
    assert "'/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU'" in launch
    assert "DeclareLaunchArgument(\n                'use_sim_time'" in launch
    assert "default_value='empty'" in launch
    assert 'requested_path.is_absolute()' in launch
    assert "get_package_share_directory('mobile_base_gazebo')" in launch
    assert "package='mobile_base_tools'" in launch
    assert "executable='odom_to_path'" in launch
    assert "default_value='true'" in launch
    assert 'enable_odom_tf:=false' in launch
    assert 'enable_odom_tf:=true' in launch
    assert 'mobile_base_localization' in launch
    assert "'physics_max_step_size'" in launch
    assert 'max_step.text = max_step_text' in launch
    assert "DeclareLaunchArgument('namespace'" not in launch
    assert 'PushRosNamespace' not in launch

    evaluation_launch = read(
        bringup / 'launch' / 'odometry_evaluation.launch.py'
    )
    assert "package='mobile_base_evaluation'" in evaluation_launch
    assert "executable='ground_truth_selector'" in evaluation_launch
    assert 'geometry_msgs/msg/PoseArray' not in evaluation_launch
    assert 'ros_gz_interfaces/srv/SetEntityPose' in evaluation_launch
    assert "executable='odometry_test_runner'" in evaluation_launch
    assert "default_value='empty'" in evaluation_launch
    assert "default_value='false'" in evaluation_launch
    assert "'physics_max_step_size'" in evaluation_launch
    assert "'/tf'" not in evaluation_launch

    package = ET.parse(bringup / 'package.xml').getroot()
    exec_depends = [elem.text for elem in package.findall('exec_depend')]
    assert 'teleop_twist_keyboard' in exec_depends
    assert 'mobile_base_tools' in exec_depends
    assert 'mobile_base_evaluation' in exec_depends
    assert 'mobile_base_localization' in exec_depends
    assert 'sensor_msgs' in exec_depends

    installed_worlds = {
        path.stem for path in (gazebo / 'worlds').glob('*.sdf')
    }
    assert {
        'empty',
        'sensor_test',
        'navigation_basic',
        'navigation_narrow',
    } <= installed_worlds

    rviz = read(description / 'rviz' / 'mobile_base_sim.rviz')
    assert 'Fixed Frame: odom' in rviz
    assert 'Class: rviz_default_plugins/Odometry' in rviz
    assert 'Class: rviz_default_plugins/Image' in rviz
    assert 'Class: rviz_default_plugins/LaserScan' in rviz
    assert '/scan' in rviz
    assert 'Show Axes: true' in rviz
    assert 'Show Names: true' in rviz
    assert 'Topic:' in rviz and '/mobile_base_controller/odometry' in rviz
    assert '/camera/image_raw' in rviz
    assert 'Keep: 1' in rviz
    assert 'Covariance:\n        Value: false' in rviz
    assert 'Class: rviz_default_plugins/Path' in rviz
    assert '/mobile_base/trajectory' in rviz
    assert '/odometry/filtered' in rviz
    assert '/mobile_base/filtered_trajectory' in rviz
    assert 'MecanumMotionPanel' not in rviz
    assert 'mobile_base_rviz_plugins' not in rviz


if __name__ == '__main__':
    main()
