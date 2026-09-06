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

"""
Launch Gazebo headlessly and verify the ROS sensor topic contract.

Runs in my_world since sensor_test was retired. Nothing here asserts
world geometry - the checks are on frame ids, message shape, finiteness
and covariances. The one world-dependent requirement is that the LiDAR
returns something finite, and the robot spawns in the hall with walls
1.5 m away against a 4.0 m range, so that holds with margin.
"""

import math
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, LaserScan
from tf2_msgs.msg import TFMessage


def generate_test_description():
    launch_file = (
        get_package_share_directory('mobile_base_bringup')
        + '/launch/simulation.launch.py'
    )
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            'world': 'my_world',
            'gui': 'false',
            'rviz': 'false',
            'use_sim_time': 'true',
            'render_engine': 'ogre',
        }.items(),
    )
    return LaunchDescription(
        [
            simulation,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestSensorTopics(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('mobile_base_sensor_contract_test')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_lidar_and_imu_messages(self):
        received = {
            'scan': None, 'imu': None, 'camera': None, 'filtered': None,
        }
        odom_tf_received = False

        def tf_callback(message):
            nonlocal odom_tf_received
            if any(
                transform.header.frame_id == 'odom'
                and transform.child_frame_id == 'base_footprint'
                for transform in message.transforms
            ):
                odom_tf_received = True

        subscriptions = [
            self.node.create_subscription(
                LaserScan,
                '/scan',
                lambda message: received.__setitem__('scan', message),
                qos_profile_sensor_data,
            ),
            self.node.create_subscription(
                Imu,
                '/imu/data',
                lambda message: received.__setitem__('imu', message),
                qos_profile_sensor_data,
            ),
            self.node.create_subscription(
                Image,
                '/camera/image_raw',
                lambda message: received.__setitem__('camera', message),
                qos_profile_sensor_data,
            ),
            self.node.create_subscription(
                Odometry,
                '/odometry/filtered',
                lambda message: received.__setitem__('filtered', message),
                10,
            ),
            self.node.create_subscription(TFMessage, '/tf', tf_callback, 10),
        ]
        deadline = time.monotonic() + 90.0
        while time.monotonic() < deadline and (
                None in received.values() or not odom_tf_received):
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.assertIsNotNone(received['scan'], 'no /scan message received')
        self.assertIsNotNone(received['imu'], 'no /imu/data message received')
        self.assertIsNotNone(
            received['camera'], 'no /camera/image_raw message received')
        self.assertIsNotNone(
            received['filtered'], 'no /odometry/filtered message received')
        self.assertTrue(odom_tf_received)
        tf_publishers = {
            endpoint.node_name
            for endpoint in self.node.get_publishers_info_by_topic('/tf')
        }
        self.assertIn('ekf_filter_node', tf_publishers)

        parameter_client = self.node.create_client(
            GetParameters,
            '/mobile_base_controller/get_parameters',
        )
        self.assertTrue(parameter_client.wait_for_service(timeout_sec=5.0))
        request = GetParameters.Request()
        request.names = ['enable_odom_tf']
        future = parameter_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertTrue(future.done())
        self.assertFalse(future.result().values[0].bool_value)

        scan = received['scan']
        self.assertEqual(scan.header.frame_id, 'lidar_link')
        self.assertGreater(scan.header.stamp.sec + scan.header.stamp.nanosec, 0)
        self.assertGreater(len(scan.ranges), 100)
        self.assertEqual(len(scan.ranges), len(scan.intensities))
        self.assertLess(scan.angle_min, scan.angle_max)
        self.assertGreater(scan.angle_increment, 0.0)
        self.assertGreater(scan.range_min, 0.0)
        self.assertAlmostEqual(scan.range_max, 4.0, delta=0.01)
        finite_ranges = [value for value in scan.ranges if math.isfinite(value)]
        self.assertTrue(finite_ranges)
        self.assertTrue(all(scan.range_min <= value <= scan.range_max
                            for value in finite_ranges))

        imu = received['imu']
        self.assertEqual(imu.header.frame_id, 'imu_link')
        self.assertGreater(imu.header.stamp.sec + imu.header.stamp.nanosec, 0)
        values = [
            imu.orientation.x,
            imu.orientation.y,
            imu.orientation.z,
            imu.orientation.w,
            imu.angular_velocity.x,
            imu.angular_velocity.y,
            imu.angular_velocity.z,
            imu.linear_acceleration.x,
            imu.linear_acceleration.y,
            imu.linear_acceleration.z,
        ]
        self.assertTrue(all(math.isfinite(value) for value in values))
        self.assertAlmostEqual(
            sum(value * value for value in (
                imu.orientation.x,
                imu.orientation.y,
                imu.orientation.z,
                imu.orientation.w,
            )),
            1.0,
            delta=0.05,
        )
        for covariance in (
            imu.orientation_covariance,
            imu.angular_velocity_covariance,
            imu.linear_acceleration_covariance,
        ):
            self.assertEqual(len(covariance), 9)
            self.assertTrue(all(math.isfinite(value) for value in covariance))
        self.assertTrue(
            all(imu.angular_velocity_covariance[index] > 0.0
                for index in (0, 4, 8))
        )

        filtered = received['filtered']
        self.assertEqual(filtered.header.frame_id, 'odom')
        self.assertEqual(filtered.child_frame_id, 'base_footprint')
        self.assertTrue(all(
            math.isfinite(value) for value in filtered.pose.covariance
        ))
        self.assertTrue(any(
            filtered.pose.covariance[index] > 0.0
            for index in (0, 7, 35)
        ))
        self.assertTrue(
            all(imu.linear_acceleration_covariance[index] > 0.0
                for index in (0, 4, 8))
        )

        for subscription in subscriptions:
            self.node.destroy_subscription(subscription)
