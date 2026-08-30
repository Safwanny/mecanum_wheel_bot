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
Plan a path to each RViz goal click, from the robot's current pose.

RViz's goal tool only publishes a PoseStamped; it drives nothing on its own.
This node turns those clicks into ComputePathToPose requests so a path can be
previewed without a bt_navigator, which is what would make the robot move.
The planner publishes the result on /plan for RViz to draw.

The goal orientation is ignored: GridBased runs SmacPlanner2D with
use_final_approach_orientation false, so only the clicked position matters.
"""

from math import hypot

from geometry_msgs.msg import PoseStamped

from nav2_msgs.action import ComputePathToPose

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

# nav2_msgs/action/ComputePathToPose error codes worth naming in a log line.
ERROR_MEANINGS = {
    204: 'goal outside the map',
    205: 'start occupied - the robot pose is inside inflated space',
    206: 'goal occupied',
    207: 'start outside the map',
    208: 'no valid path - inflation too aggressive or goal unreachable',
}


class GoalToPlan(Node):
    """Request a plan from the current robot pose for every goal click."""

    def __init__(self):
        super().__init__('goal_to_plan')
        self._client = ActionClient(
            self, ComputePathToPose, 'compute_path_to_pose'
        )
        self._in_flight = False
        self.create_subscription(
            PoseStamped, '/goal_pose', self._on_goal, 1
        )
        self.get_logger().info(
            'Ready. Use the RViz goal tool; paths appear on /plan.'
        )

    def _on_goal(self, message):
        """Send one planning request, dropping clicks while one is open."""
        if self._in_flight:
            self.get_logger().warn('Still planning; ignoring this click.')
            return
        if not self._client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                'compute_path_to_pose is unavailable. Is planner_server '
                'active? Check: ros2 lifecycle get /planner_server'
            )
            return

        goal = ComputePathToPose.Goal()
        goal.goal = message
        # An empty start plus use_start false makes planner_server resolve
        # the start from the current transform, which is the whole point.
        goal.use_start = False
        self._in_flight = True
        self.get_logger().info(
            'Planning to '
            f'({message.pose.position.x:.2f}, {message.pose.position.y:.2f}) '
            f'in {message.header.frame_id}'
        )
        self._client.send_goal_async(goal).add_done_callback(self._on_accepted)

    def _on_accepted(self, future):
        """Chain to the result, or release the guard if it was rejected."""
        handle = future.result()
        if not handle.accepted:
            self._in_flight = False
            self.get_logger().error('Planning request was rejected.')
            return
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        """Report what the planner produced."""
        self._in_flight = False
        result = future.result().result
        if result.error_code != 0:
            meaning = ERROR_MEANINGS.get(result.error_code, 'see nav2_msgs')
            self.get_logger().error(
                f'No path: error_code {result.error_code} ({meaning})'
            )
            return

        poses = result.path.poses
        length = sum(
            hypot(
                poses[i].pose.position.x - poses[i - 1].pose.position.x,
                poses[i].pose.position.y - poses[i - 1].pose.position.y,
            )
            for i in range(1, len(poses))
        )
        self.get_logger().info(
            f'Path found: {len(poses)} poses, {length:.2f} m. Shown on /plan.'
        )


def main(args=None):
    """Run the goal-to-plan bridge node."""
    rclpy.init(args=args)
    node = GoalToPlan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, RuntimeError):
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == '__main__':
    main()
