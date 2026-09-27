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

"""Drive to a goal with the ToF ring alone: holonomic DistBug (or Bug2).

Goal from RViz's "2D Goal Pose" (``/goal_pose``, in ``odom``), pose from the
EKF (``/odometry/filtered``), obstacles from ``/tof/obstacles``. No map, no
LiDAR. The robot holds the heading it had when the goal arrived and only
translates; the logic is in ``bug_model.Bug2``.

Commands go to the speed governor, never straight to the wheels, so every
velocity this node asks for is capped by what the ring sees along it.
"""

import math

from geometry_msgs.msg import Point, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from mobile_base_tools import tof_palette as palette
from mobile_base_tools.bug_model import (
    ARRIVED, DRIVING, STUCK, UNREACHABLE, Bug2)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class BugNavigator(Node):
    """Run Bug2 at a fixed rate and publish its velocity for the governor."""

    def __init__(self):
        super().__init__('bug_navigator')
        value = self.declare_parameter
        value('cmd_out', '/governor/cmd_vel')
        value('rate', 20.0)
        value('speed', 0.5)
        value('standoff', 0.30)
        value('min_standoff', 0.05)
        value('hit_distance', 0.35)
        value('goal_tolerance', 0.10)
        value('line_tolerance', 0.05)
        value('side', 'left')
        value('heading_gain', 1.5)
        value('algorithm', 'distbug')
        value('sense_range', 1.0)
        get = self.get_parameter
        self.bug = Bug2(
            speed=get('speed').value, standoff=get('standoff').value,
            min_standoff=get('min_standoff').value,
            hit_distance=get('hit_distance').value,
            goal_tolerance=get('goal_tolerance').value,
            line_tolerance=get('line_tolerance').value,
            side=get('side').value, algorithm=get('algorithm').value,
            dt=1.0 / get('rate').value,
            sense_range=get('sense_range').value)
        self.heading_gain = get('heading_gain').value

        self.pose = None
        self.pending = None
        self.heading = None
        self.points = []
        self.reported = None

        self.publisher = self.create_publisher(
            TwistStamped, get('cmd_out').value, 10)
        self.state_publisher = self.create_publisher(
            String, '/bug_navigator/state', 10)
        self.marker_publisher = self.create_publisher(
            MarkerArray, '/bug_navigator/markers', 10)
        self.create_subscription(
            Odometry, '/odometry/filtered', self.on_odometry, 10)
        self.create_subscription(
            PointCloud2, '/tof/obstacles', self.on_cloud, 5)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.create_timer(1.0 / get('rate').value, self.tick)

    def on_odometry(self, message):
        pose = message.pose.pose
        self.pose = (pose.position.x, pose.position.y,
                     yaw_of(pose.orientation))
        if self.pending is not None:
            goal, self.pending = self.pending, None
            self.start_run(goal)

    def on_cloud(self, message):
        self.points = [
            (float(x), float(y), float(z))
            for x, y, z in point_cloud2.read_points(
                message, field_names=('x', 'y', 'z'), skip_nans=True)
        ]

    def on_goal(self, message):
        if message.header.frame_id not in ('odom', ''):
            self.get_logger().warn(
                'goal in {!r}, treated as odom: there is no map here'.format(
                    message.header.frame_id))
        goal = (message.pose.position.x, message.pose.position.y)
        if self.pose is None:
            # The EKF starts after the controllers; hold the goal until it does.
            self.get_logger().info('goal held until odometry arrives')
            self.pending = goal
            return
        self.start_run(goal)

    def start_run(self, goal):
        self.heading = self.pose[2]
        self.bug.set_goal(self.pose, goal)
        self.get_logger().info('goal ({:.2f}, {:.2f})'.format(*goal))

    def tick(self):
        if self.pose is None:
            return
        was = self.bug.state
        vx, vy = self.bug.step(self.pose, self.points)
        # Silent unless driving, so teleop on the same input is not fought;
        # one zero goes out on the cycle a run ends, to stop the wheels.
        if self.bug.state not in DRIVING and was not in DRIVING:
            return
        spin = 0.0
        if self.heading is not None and (vx or vy):
            spin = self.heading_gain * wrap(self.heading - self.pose[2])
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'base_footprint'
        command.twist.linear.x = vx
        command.twist.linear.y = vy
        command.twist.angular.z = spin
        self.publisher.publish(command)

        state = self.bug.state
        self.state_publisher.publish(String(data=state))
        if state != self.reported:
            self.reported = state
            log = self.get_logger().warn if state in (
                UNREACHABLE, STUCK) else self.get_logger().info
            log('state: ' + state)
        if self.bug.goal is not None:
            self.marker_publisher.publish(self.markers())
        if state == ARRIVED:
            self.heading = None

    def markers(self):
        """The m-line, the goal and the hit point, in odom."""
        def marker(kind, marker_id):
            item = Marker()
            item.header.frame_id = 'odom'
            item.ns = 'bug'
            item.id = marker_id
            item.type = kind
            item.action = Marker.ADD
            item.pose.orientation.w = 1.0
            return item

        start, goal = self.bug.start, self.bug.goal
        line = marker(Marker.LINE_STRIP, 0)
        line.scale.x = 0.01
        line.color = palette.rgba(palette.SLATE, 0.6)
        line.points = [Point(x=start[0], y=start[1], z=0.005),
                       Point(x=goal[0], y=goal[1], z=0.005)]
        target = marker(Marker.CYLINDER, 1)
        target.pose.position = Point(x=goal[0], y=goal[1], z=0.01)
        target.scale.x = target.scale.y = 2.0 * self.bug.goal_tolerance
        target.scale.z = 0.02
        target.color = palette.rgba(palette.FLOOR, 0.7)
        hit = marker(Marker.SPHERE, 2)
        if self.bug.hit is not None:
            hit.pose.position = Point(x=self.bug.hit[0], y=self.bug.hit[1],
                                      z=0.02)
            hit.scale.x = hit.scale.y = hit.scale.z = 0.05
            hit.color = palette.rgba(palette.ALERT, 0.9)
        else:
            hit.action = Marker.DELETE
        # The state is shown on the control panel, not as text in the scene.
        return MarkerArray(markers=[line, target, hit])


def main(args=None):
    """Run the Bug2 navigator."""
    rclpy.init(args=args)
    node = BugNavigator()
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
