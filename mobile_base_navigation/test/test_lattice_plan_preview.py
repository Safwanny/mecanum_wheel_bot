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
Check that the lattice motion constraints actually reach the planned path.

This is the regression check the feature exists for. The unit tests prove the
search core snaps its own output; this one proves the constraint survives
plugin loading, parameter overlay, lifecycle activation and the
ComputePathToPose action - the layers where a plugin silently falls back to a
default and the robot then follows a path it cannot execute.

It runs against the saved map through planning_harness.launch.py rather than
against Gazebo. The harness serves the same navigation_basic occupancy grid
the simulated robot localizes against, and a global planner reads the static
layer, so a simulator would add several minutes of startup and a live obstacle
layer without changing what is being asserted.
"""

import math
from pathlib import Path
import time
import unittest

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import launch_testing

from lifecycle_msgs.srv import GetState

from nav2_msgs.action import ComputePathToPose

import rclpy
from rclpy.action import ActionClient

# The 45 degree lattice. Every segment direction and every pose orientation
# except the goal's must land on one of these.
QUARTER_PI = math.pi / 4.0

# Poses closer together than this are a rotation in place, not a segment, so
# atan2 between them is meaningless. Half a costmap cell.
MIN_SEGMENT_LENGTH = 0.025

# Floating point only; the planner emits exact multiples by construction.
SNAP_TOLERANCE = 1e-6


def _map_path():
    """
    Locate the saved map in the source tree.

    maps/ is a loose repo-root directory that no package installs, so it
    cannot be found through the ament index. launch_test runs this file from
    its source location, which makes __file__ the reliable anchor.
    """
    return (
        Path(__file__).resolve().parents[2] / 'maps' / 'navigation_basic.yaml'
    )


def generate_test_description():
    launch_file = (
        get_package_share_directory('mobile_base_navigation')
        + '/launch/planning_harness.launch.py'
    )
    harness = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            'map': str(_map_path()),
            'rviz': 'false',
            # The action is called directly, so the click bridge would only
            # add a second in-flight goal racing this one.
            'goal_bridge': 'false',
            'motion_profile': 'primitive',
        }.items(),
    )
    return LaunchDescription([
        harness,
        launch_testing.actions.ReadyToTest(),
    ])


class TestLatticePlanPreview(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('lattice_plan_preview_test')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _await_active(self, node_name, timeout=60.0):
        """Wait for a lifecycle node to reach active, not merely to exist."""
        client = self.node.create_client(GetState, f'/{node_name}/get_state')
        self.assertTrue(
            client.wait_for_service(timeout_sec=timeout),
            f'{node_name} never offered get_state',
        )
        deadline = time.time() + timeout
        while time.time() < deadline:
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
            if future.done() and future.result() is not None:
                if future.result().current_state.label == 'active':
                    return
            self._spin(1.0)
        self.fail(f'{node_name} did not reach active within {timeout} s')

    def _plan(self, goal_x, goal_y, timeout=30.0):
        client = ActionClient(
            self.node, ComputePathToPose, '/compute_path_to_pose')
        self.assertTrue(
            client.wait_for_server(timeout_sec=timeout),
            'compute_path_to_pose action server never appeared',
        )
        goal = ComputePathToPose.Goal()
        goal.goal = PoseStamped()
        goal.goal.header.frame_id = 'map'
        goal.goal.pose.position.x = goal_x
        goal.goal.pose.position.y = goal_y
        goal.goal.pose.orientation.w = 1.0
        # The harness's static transforms put base_footprint at the map
        # origin, so resolving the start from TF is what exercises the same
        # path an RViz goal click takes.
        goal.use_start = False

        send = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=timeout)
        handle = send.result()
        self.assertIsNotNone(handle, 'the planner never accepted the goal')
        self.assertTrue(handle.accepted, 'the planner rejected the goal')

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self.node, result_future, timeout_sec=timeout)
        self.assertIsNotNone(result_future.result(), 'planning never returned')
        result = result_future.result().result
        self.assertEqual(
            result.error_code, 0,
            f'planning failed with error_code {result.error_code}')
        return result.path

    def test_plan_is_axis_snapped_end_to_end(self):
        self._await_active('map_server')
        self._await_active('planner_server')

        # North-east of the interior wall, the same goal the README's
        # tight-gap test uses, so the route is a real one through the 0.925 m
        # north gap rather than a straight line across open floor.
        path = self._plan(2.5, 3.4)
        self.assertGreaterEqual(
            len(path.poses), 2, 'the planner returned a degenerate path')

        # Every segment direction must be a multiple of 45 degrees. This is
        # the constraint: a path with a 30 degree leg is one the primitive
        # controller cannot execute as a single motion.
        segments = 0
        for index in range(len(path.poses) - 1):
            start = path.poses[index].pose.position
            end = path.poses[index + 1].pose.position
            length = math.hypot(end.x - start.x, end.y - start.y)
            if length < MIN_SEGMENT_LENGTH:
                continue
            segments += 1
            angle = math.atan2(end.y - start.y, end.x - start.x)
            residual = abs(math.remainder(angle, QUARTER_PI))
            self.assertLess(
                residual, SNAP_TOLERANCE,
                f'segment {index} leaves the lattice at '
                f'{math.degrees(angle):.4f} deg',
            )
        self.assertGreater(segments, 0, 'the path contained no segments')

        # Pose orientations carry the travel direction and must snap too. The
        # final pose is the documented exception: it carries the requested
        # goal yaw, which the goal checker compares against and which has no
        # reason to sit on the lattice.
        for index, pose in enumerate(path.poses[:-1]):
            yaw = 2.0 * math.atan2(
                pose.pose.orientation.z, pose.pose.orientation.w)
            residual = abs(math.remainder(yaw, QUARTER_PI))
            self.assertLess(
                residual, SNAP_TOLERANCE,
                f'pose {index} orientation is {math.degrees(yaw):.4f} deg',
            )

        # Merging must have happened. A 4.5 m route at a 0.05 m step is about
        # 90 edges; if every edge became a pose, the merge is not running and
        # the controller would stop at each one.
        self.assertLess(
            len(path.poses), 40,
            f'{len(path.poses)} poses suggests straight runs were not merged',
        )

    def test_unreachable_goal_fails_rather_than_inventing_a_path(self):
        self._await_active('planner_server')
        # Well outside the 10 x 8 m room.
        client = ActionClient(
            self.node, ComputePathToPose, '/compute_path_to_pose')
        self.assertTrue(client.wait_for_server(timeout_sec=30.0))
        goal = ComputePathToPose.Goal()
        goal.goal = PoseStamped()
        goal.goal.header.frame_id = 'map'
        goal.goal.pose.position.x = 40.0
        goal.goal.pose.position.y = 40.0
        goal.goal.pose.orientation.w = 1.0
        goal.use_start = False

        send = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=30.0)
        handle = send.result()
        self.assertIsNotNone(handle)
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self.node, result_future, timeout_sec=30.0)
        self.assertIsNotNone(result_future.result())
        result = result_future.result().result
        # A non-zero error code and an empty path. Returning a short path with
        # error_code 0 would be the dangerous failure: the robot would drive
        # confidently to the wrong place.
        self.assertNotEqual(result.error_code, 0)
        self.assertEqual(len(result.path.poses), 0)
