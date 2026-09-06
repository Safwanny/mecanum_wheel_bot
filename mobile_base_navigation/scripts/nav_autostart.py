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
Hold the navigation stack down until the robot knows where it is.

The costmaps cannot produce anything meaningful before AMCL publishes
map -> odom, and AMCL publishes nothing until an initial pose is set. Bringing
the planning nodes up regardless means a global costmap that exists, is
"active", and is built on a pose the operator has not supplied yet - it shows
the map smeared against whatever transform happened to be there.

So the lifecycle manager starts with autostart false and this node calls
STARTUP once the operator has said where the robot is.

The signal is /initialpose - the message RViz's "2D Pose Estimate" publishes -
and not /amcl_pose or the map -> odom transform, both of which look like the
more principled choice and both of which deadlock here. Measured: with the
robot stationary, AMCL publishes neither. Its filter updates on motion
(update_min_d), and nothing can move until navigation is up, which is the thing
being waited for. /initialpose has no such circularity: it is a human action,
it happens exactly once, and it is the event the operator is being asked for.

startup_delay covers the gap between that message and AMCL having acted on it,
so the costmaps do not ask for map -> odom a moment before it exists.
"""

from geometry_msgs.msg import PoseWithCovarianceStamped

from nav2_msgs.srv import ManageLifecycleNodes

import rclpy
from rclpy.node import Node


class NavAutostart(Node):
    """Trigger the navigation lifecycle manager once AMCL has localized."""

    def __init__(self):
        super().__init__('nav_autostart')
        self.declare_parameter('manager', 'lifecycle_manager_navigation')
        self.declare_parameter('pose_topic', '/initialpose')
        self.declare_parameter('startup_delay', 3.0)
        manager = self.get_parameter('manager').value
        pose_topic = self.get_parameter('pose_topic').value
        self._delay = float(self.get_parameter('startup_delay').value)

        self._started = False
        self._timer = None
        self._client = self.create_client(
            ManageLifecycleNodes, f'/{manager}/manage_nodes')
        self.create_subscription(
            PoseWithCovarianceStamped, pose_topic, self._on_pose, 1)
        self.get_logger().info(
            f'navigation is held down until {pose_topic} arrives. '
            'Set the initial pose in RViz (2D Pose Estimate) to bring the '
            'costmaps, planner and controller up.'
        )

    def _on_pose(self, _message):
        if self._started:
            return
        self._started = True
        self.get_logger().info(
            f'initial pose received; starting navigation in {self._delay:.1f} s'
        )
        # One-shot: let AMCL act on the pose before the costmaps ask for
        # map -> odom, then cancel so this never fires twice.
        self._timer = self.create_timer(self._delay, self._start)

    def _start(self):
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
        if not self._client.wait_for_service(timeout_sec=30.0):
            # Do not retry forever: a missing manager is a launch error, and a
            # node quietly re-arming would hide it.
            self.get_logger().error(
                'lifecycle manager never offered manage_nodes; bring the '
                'stack up by hand with a STARTUP (command: 0) call'
            )
            return
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        future = self._client.call_async(request)
        future.add_done_callback(self._on_started)

    def _on_started(self, future):
        try:
            success = future.result().success
        except Exception as error:  # noqa: BLE001 - report, never crash the gate
            self.get_logger().error(f'STARTUP call failed: {error}')
            return
        if success:
            self.get_logger().info('localized: navigation is up, send a goal')
        else:
            self.get_logger().error(
                'the lifecycle manager refused STARTUP; check which server '
                'failed to activate in its log'
            )


def main():
    """Run the gate until shutdown."""
    rclpy.init()
    node = NavAutostart()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
