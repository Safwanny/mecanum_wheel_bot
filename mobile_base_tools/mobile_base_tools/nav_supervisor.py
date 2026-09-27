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

"""Fall back from Nav2 to DistBug when planning fails, and hand back after.

Nav2 plans on the live SLAM map and is the primary navigator: bt_navigator
takes goals from RViz's ``/goal_pose`` directly. This node watches alongside.
It remembers the latest goal and follows the NavigateToPose action status;
when Nav2 **aborts** - no path through what it has mapped, or its recoveries
exhausted - the goal goes to the bug navigator, which needs no map at all.

DistBug runs in ``odom``, so the goal is transformed out of ``map`` at the
hand-off. Once DistBug has worked round the blockage and is heading for the
goal again, Nav2 gets the goal back: with the map grown by the detour it can
usually plan now. If DistBug arrives on its own, fine; if it ends STUCK or
UNREACHABLE, the supervisor stops trying and says so.

``/nav_supervisor/mode`` reports ``idle``, ``nav2``, ``bug fallback`` or
``failed`` for the panel.
"""

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Empty, String
import tf2_geometry_msgs  # noqa: F401  registers PoseStamped transforms
import tf2_ros

IDLE, NAV2, FALLBACK, FAILED = 'idle', 'nav2', 'bug fallback', 'failed'


class NavSupervisor(Node):
    """Nav2 first; DistBug when Nav2 gives up; Nav2 again after the detour."""

    def __init__(self):
        super().__init__('nav_supervisor')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('max_fallbacks', 3)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.max_fallbacks = self.get_parameter('max_fallbacks').value

        self.goal = None          # latest goal, as given (map frame)
        self.mode = IDLE
        self.fallbacks = 0
        self.bug_state = None
        self.followed = False     # DistBug has been round a boundary
        # The Nav2 goal that last aborted. Its status lingers as the newest
        # until the next goal is accepted, and must not trigger a second
        # fallback after the goal has been handed back.
        self.aborted = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav2 = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.bug_goal = self.create_publisher(
            PoseStamped, '/bug_navigator/goal', 10)
        self.bug_cancel = self.create_publisher(
            Empty, '/bug_navigator/cancel', 10)
        self.mode_publisher = self.create_publisher(
            String, '/nav_supervisor/mode', 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status',
            self.on_status, 10)
        self.create_subscription(
            String, '/bug_navigator/state', self.on_bug, 10)
        self.create_timer(0.5, self.publish_mode)

    def set_mode(self, mode):
        if mode != self.mode:
            self.get_logger().info('mode: ' + mode)
        self.mode = mode
        self.publish_mode()

    def publish_mode(self):
        self.mode_publisher.publish(String(data=self.mode))

    def on_goal(self, message):
        """A new goal from RViz: bt_navigator is already on it."""
        self.goal = message
        self.fallbacks = 0
        if self.mode == FALLBACK:
            self.bug_cancel.publish(Empty())
        self.set_mode(NAV2)

    def on_status(self, message):
        """Follow the newest Nav2 goal; fall back when it aborts."""
        if not message.status_list or self.goal is None:
            return
        newest = message.status_list[-1]
        goal_id = bytes(newest.goal_info.goal_id.uuid)
        if self.mode != NAV2 or goal_id == self.aborted:
            return
        if newest.status == GoalStatus.STATUS_SUCCEEDED:
            self.set_mode(IDLE)
        elif newest.status == GoalStatus.STATUS_ABORTED:
            self.aborted = goal_id
            self.fall_back()

    def fall_back(self):
        if self.fallbacks >= self.max_fallbacks:
            self.get_logger().error('Nav2 and DistBug both failed; giving up')
            self.set_mode(FAILED)
            return
        # Transform with the latest map -> odom, not the one at click time:
        # the buffer may no longer hold that moment, and the latest is what
        # DistBug's odometry will be compared against anyway.
        latest = PoseStamped()
        latest.header.frame_id = self.goal.header.frame_id or 'map'
        latest.pose = self.goal.pose
        try:
            goal = self.tf_buffer.transform(
                latest, self.odom_frame, timeout=Duration(seconds=0.5))
        except tf2_ros.TransformException as error:
            self.get_logger().error('cannot hand goal to DistBug: {}'.format(
                error))
            self.set_mode(FAILED)
            return
        self.fallbacks += 1
        self.followed = False
        self.bug_goal.publish(goal)
        self.get_logger().warn('Nav2 aborted; DistBug takes the goal '
                               '({} of {})'.format(self.fallbacks,
                                                   self.max_fallbacks))
        self.set_mode(FALLBACK)

    def on_bug(self, message):
        """Hand back to Nav2 once DistBug has cleared the blockage."""
        state, self.bug_state = message.data, message.data
        if self.mode != FALLBACK:
            return
        if state == 'follow_boundary':
            self.followed = True
        elif state == 'go_to_goal' and self.followed:
            # Round the obstacle and heading for the goal: the map now holds
            # what blocked Nav2, so let it plan again.
            self.bug_cancel.publish(Empty())
            self.send_to_nav2()
        elif state == 'arrived':
            self.set_mode(IDLE)
        elif state in ('stuck', 'unreachable'):
            self.get_logger().error('DistBug ended ' + state)
            self.set_mode(FAILED)

    def send_to_nav2(self):
        if not self.nav2.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Nav2 is not available')
            self.set_mode(FAILED)
            return
        request = NavigateToPose.Goal()
        request.pose = self.goal
        request.pose.header.stamp = Time().to_msg()
        self.nav2.send_goal_async(request)
        self.set_mode(NAV2)


def main(args=None):
    """Run the navigation supervisor."""
    rclpy.init(args=args)
    node = NavSupervisor()
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
