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

"""Turn rf2o's laser pose into body velocities the EKF can fuse.

rf2o matches consecutive LiDAR scans and estimates the full planar motion,
but its ROS 2 node publishes the twist as forward speed and turn rate with
the sideways component fixed at zero. Fused as-is it would tell the EKF this
mecanum base never strafes. Its *pose* is complete, so this node
differentiates that pose into body-frame vx, vy and yaw rate, and publishes
them with covariances - rf2o leaves those at zero, which the EKF would read as
perfect.

A scan match can fail outright: in a featureless corridor or with too few
returns it jumps. Velocities beyond what the base can do are dropped, not
fused, because one bad jump into the filter costs more than a missed update.
"""

import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def body_velocity(previous, current, dt):
    """Body-frame (vx, vy, wz) between two poses (x, y, yaw), dt apart.

    The displacement is expressed in the frame of the mean heading over the
    step, which is exact for a constant twist to first order.
    """
    dx, dy = current[0] - previous[0], current[1] - previous[1]
    dyaw = math.atan2(math.sin(current[2] - previous[2]),
                      math.cos(current[2] - previous[2]))
    heading = previous[2] + dyaw / 2.0
    cos_h, sin_h = math.cos(heading), math.sin(heading)
    return ((dx * cos_h + dy * sin_h) / dt,
            (-dx * sin_h + dy * cos_h) / dt,
            dyaw / dt)


class LaserOdometry(Node):
    """Republish rf2o as a fusable twist on ``/laser_odometry``."""

    def __init__(self):
        super().__init__('laser_odometry')
        value = self.declare_parameter
        value('input', '/odom_rf2o')
        value('output', '/laser_odometry')
        value('base_frame', 'base_footprint')
        # 1-sigma of each velocity, from scan-match quality in simulation;
        # retune on hardware from a stationary recording.
        value('linear_sigma', 0.03)
        value('angular_sigma', 0.05)
        # Above these the base cannot move: the match has failed.
        value('max_linear', 1.0)
        value('max_angular', 3.0)
        get = self.get_parameter
        self.linear_var = get('linear_sigma').value ** 2
        self.angular_var = get('angular_sigma').value ** 2
        self.max_linear = get('max_linear').value
        self.max_angular = get('max_angular').value
        self.base_frame = get('base_frame').value
        self.previous = None
        self.rejected = 0
        self.publisher = self.create_publisher(
            Odometry, get('output').value, 10)
        self.create_subscription(
            Odometry, get('input').value, self.on_pose, 10)

    def on_pose(self, message):
        stamp = message.header.stamp
        now = stamp.sec + stamp.nanosec * 1e-9
        pose = message.pose.pose
        current = (pose.position.x, pose.position.y, yaw_of(pose.orientation))
        previous, self.previous = self.previous, (now, current)
        if previous is None or now <= previous[0]:
            return
        vx, vy, wz = body_velocity(previous[1], current, now - previous[0])
        if math.hypot(vx, vy) > self.max_linear or abs(wz) > self.max_angular:
            self.rejected += 1
            self.get_logger().warn(
                'scan match jump rejected ({} so far)'.format(self.rejected),
                throttle_duration_sec=5.0)
            return
        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = message.header.frame_id
        out.child_frame_id = self.base_frame
        out.twist.twist.linear.x = vx
        out.twist.twist.linear.y = vy
        out.twist.twist.angular.z = wz
        covariance = [0.0] * 36
        covariance[0] = covariance[7] = self.linear_var
        covariance[35] = self.angular_var
        # Unused axes large, so nothing reads them as measured.
        covariance[14] = covariance[21] = covariance[28] = 1e3
        out.twist.covariance = covariance
        out.pose.covariance = [0.0] * 36
        out.pose.covariance[0] = -1.0  # pose is not provided
        self.publisher.publish(out)


def main(args=None):
    """Run the laser odometry adapter."""
    rclpy.init(args=args)
    node = LaserOdometry()
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
