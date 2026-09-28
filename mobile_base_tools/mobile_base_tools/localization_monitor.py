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

"""Show where SLAM corrected the pose, and how far the pose is from the truth.

The odom frame drifts by design; slam_toolbox fixes it by moving map -> odom.
Each such move is a localisation event. This node samples TF, and when
map -> odom steps it drops a dot where the robot was: amber for an ordinary
scan-match nudge, red for a jump large enough to be a loop closure, sized by
the jump.

It also publishes two paths in the map frame - the pose navigation actually
uses, and the Gazebo ground truth - and a JSON status for the control panel
with the true error, the last correction and SLAM's own uncertainty.

Ground truth is world-frame. The map origin is where the robot started, so the
truth is re-expressed relative to its first sample. It is never put on TF.
"""

from collections import deque
import json
import math

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def relative_to(origin, pose):
    """Express planar pose (x, y, yaw) in the frame of origin."""
    dx, dy = pose[0] - origin[0], pose[1] - origin[1]
    c, s = math.cos(origin[2]), math.sin(origin[2])
    return (c * dx + s * dy, -s * dx + c * dy, wrap(pose[2] - origin[2]))


def correction(previous, current, min_step=0.02, min_turn=math.radians(0.5),
               loop_step=0.15):
    """Classify a map -> odom change: None, 'match' or 'closure'.

    Returns (kind, step_m, turn_rad).
    """
    step = math.hypot(current[0] - previous[0], current[1] - previous[1])
    turn = abs(wrap(current[2] - previous[2]))
    if step < min_step and turn < min_turn:
        return None, step, turn
    return ('closure' if step >= loop_step else 'match'), step, turn


class LocalizationMonitor(Node):

    def __init__(self):
        super().__init__('localization_monitor')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('truth_topic', '/ground_truth/odom')
        self.declare_parameter('rate', 10.0)
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_path_pub = self.create_publisher(
            Path, '/mobile_base/map_trajectory', latched)
        self.truth_path_pub = self.create_publisher(
            Path, '/mobile_base/true_trajectory', latched)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/localization/corrections', latched)
        self.status_pub = self.create_publisher(
            String, '/localization/status', 10)
        self.create_subscription(
            Odometry, self.get_parameter('truth_topic').value,
            self.on_truth, 20)
        self.create_subscription(
            PoseWithCovarianceStamped, '/pose', self.on_slam_pose, 10)

        self.truth_origin = None
        self.truth = deque(maxlen=100)  # (stamp_ns, x, y, yaw) in map frame
        self.map_path = Path()
        self.truth_path = Path()
        self.map_path.header.frame_id = self.map_frame
        self.truth_path.header.frame_id = self.map_frame
        self.markers = MarkerArray()
        self.last_offset = None
        self.last_fix = None  # (time_s, kind, step, turn)
        self.fix_times = deque()
        self.slam_sigma = None
        self.errors = deque(maxlen=600)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.tick)

    def on_truth(self, message):
        p = message.pose.pose
        pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        if self.truth_origin is None:
            self.truth_origin = pose
        x, y, yaw = relative_to(self.truth_origin, pose)
        stamp = Time.from_msg(message.header.stamp).nanoseconds
        self.truth.append((stamp, x, y, yaw))
        if self.moved(self.truth_path, x, y):
            self.truth_path.poses.append(self.stamped(message.header.stamp, x, y, yaw))
            self.truth_path.header.stamp = message.header.stamp
            self.truth_path_pub.publish(self.truth_path)

    def on_slam_pose(self, message):
        c = message.pose.covariance
        self.slam_sigma = (math.sqrt(max(c[0], c[7], 0.0)),
                           math.sqrt(max(c[35], 0.0)))

    def lookup(self, parent, child):
        try:
            t = self.buffer.lookup_transform(
                parent, child, Time(), timeout=Duration(seconds=0.0))
        except tf2_ros.TransformException:
            return None, None
        r = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                yaw_of(r)), t.header.stamp

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        offset, _ = self.lookup(self.map_frame, self.odom_frame)
        pose, stamp = self.lookup(self.map_frame, self.base_frame)
        if offset is None or pose is None:
            return
        if self.last_offset is not None:
            kind, step, turn = correction(self.last_offset, offset)
            if kind:
                self.last_fix = (now, kind, step, turn)
                self.fix_times.append(now)
                self.add_marker(pose, kind, step, stamp)
        self.last_offset = offset
        while self.fix_times and now - self.fix_times[0] > 60.0:
            self.fix_times.popleft()

        if self.moved(self.map_path, pose[0], pose[1]):
            self.map_path.poses.append(self.stamped(stamp, *pose))
            self.map_path.header.stamp = stamp
            self.map_path_pub.publish(self.map_path)

        status = {'fixes_per_min': len(self.fix_times)}
        truth = self.nearest_truth(Time.from_msg(stamp).nanoseconds)
        if truth is not None:
            error = math.hypot(pose[0] - truth[1], pose[1] - truth[2])
            self.errors.append(error)
            status['error_m'] = error
            status['error_deg'] = math.degrees(wrap(pose[2] - truth[3]))
            status['error_max_m'] = max(self.errors)
            status['error_mean_m'] = sum(self.errors) / len(self.errors)
        if self.last_fix is not None:
            t, kind, step, turn = self.last_fix
            status['fix'] = {'age_s': now - t, 'kind': kind, 'step_m': step,
                             'turn_deg': math.degrees(turn)}
        if self.slam_sigma is not None:
            status['sigma_m'], sigma_yaw = self.slam_sigma
            status['sigma_deg'] = math.degrees(sigma_yaw)
        self.status_pub.publish(String(data=json.dumps(status)))

    def nearest_truth(self, stamp_ns):
        """Truth sample closest in time to stamp_ns, if within 50 ms."""
        if not self.truth:
            return None
        best = min(self.truth, key=lambda s: abs(s[0] - stamp_ns))
        return best if abs(best[0] - stamp_ns) < 50_000_000 else None

    def add_marker(self, pose, kind, step, stamp):
        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = stamp
        m.ns = 'corrections'
        m.id = len(self.markers.markers)
        m.type = Marker.SPHERE
        m.pose.position.x, m.pose.position.y = pose[0], pose[1]
        m.pose.orientation.w = 1.0
        size = min(0.04 + 0.5 * step, 0.25)
        m.scale.x = m.scale.y = m.scale.z = size
        if kind == 'closure':
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.2, 0.2, 0.8
        else:
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.7, 0.1, 0.45
        # ponytail: unbounded marker list; cap it if runs go past ~1 h
        self.markers.markers.append(m)
        self.marker_pub.publish(self.markers)

    @staticmethod
    def moved(path, x, y, min_distance=0.02):
        if not path.poses:
            return True
        last = path.poses[-1].pose.position
        return math.hypot(x - last.x, y - last.y) >= min_distance

    def stamped(self, stamp, x, y, yaw):
        s = PoseStamped()
        s.header.frame_id = self.map_frame
        s.header.stamp = stamp
        s.pose.position.x, s.pose.position.y = x, y
        s.pose.orientation.z = math.sin(yaw / 2.0)
        s.pose.orientation.w = math.cos(yaw / 2.0)
        return s


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
