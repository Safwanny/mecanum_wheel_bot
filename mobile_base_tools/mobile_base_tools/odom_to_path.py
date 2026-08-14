#!/usr/bin/env python3
"""Publish a bounded, sampled Path from controller odometry."""

from collections import deque
from dataclasses import dataclass
import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Empty


def wrapped_angle(angle):
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class Sample:
    """Minimal state used to decide whether a pose is retained."""

    stamp_ns: int
    frame_id: str
    x: float
    y: float
    yaw: float
    pose: object


class TrajectoryAccumulator:
    """Maintain a bounded trajectory using distance and yaw thresholds."""

    def __init__(self, max_points=1000, min_distance=0.02, min_yaw=0.05):
        self.samples = deque(maxlen=max_points)
        self.min_distance = min_distance
        self.min_yaw = min_yaw

    def add(self, sample):
        """Accept a sample when it changes frame/time or crosses a threshold."""
        if self.samples:
            last = self.samples[-1]
            if sample.frame_id != last.frame_id or sample.stamp_ns < last.stamp_ns:
                self.reset()
            else:
                distance = math.hypot(sample.x - last.x, sample.y - last.y)
                yaw_change = abs(wrapped_angle(sample.yaw - last.yaw))
                if distance < self.min_distance and yaw_change < self.min_yaw:
                    return False
        self.samples.append(sample)
        return True

    def reset(self):
        """Clear all retained samples."""
        self.samples.clear()


class OdomToPath(Node):
    """Convert odometry messages into a latched, bounded path."""

    def __init__(self):
        super().__init__('odom_to_path')
        self.declare_parameter('odom_topic', 'mobile_base_controller/odometry')
        self.declare_parameter('path_topic', 'mobile_base/trajectory')
        self.declare_parameter(
            'reset_service', 'mobile_base/trajectory/reset')
        self.declare_parameter('max_points', 1000)
        self.declare_parameter('min_distance', 0.02)
        self.declare_parameter('min_yaw', 0.05)
        self.declare_parameter('publish_rate', 10.0)

        self.accumulator = TrajectoryAccumulator(
            self.get_parameter('max_points').value,
            self.get_parameter('min_distance').value,
            self.get_parameter('min_yaw').value,
        )
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            Path, self.get_parameter('path_topic').value, qos)
        self.subscription = self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self.odom_callback, 10)
        self.reset_service = self.create_service(
            Empty, self.get_parameter('reset_service').value,
            self.reset_callback)

    @staticmethod
    def yaw_from_odometry(message):
        """Extract planar yaw from an odometry quaternion."""
        q = message.pose.pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def odom_callback(self, message):
        """Sample odometry and publish when a new pose is accepted."""
        stamp = message.header.stamp
        pose = message.pose.pose
        sample = Sample(
            stamp.sec * 1_000_000_000 + stamp.nanosec,
            message.header.frame_id,
            pose.position.x,
            pose.position.y,
            self.yaw_from_odometry(message),
            pose,
        )
        if self.accumulator.add(sample):
            self.publish_path()

    def publish_path(self):
        """Publish the current path using accepted odometry timestamps."""
        path = Path()
        if self.accumulator.samples:
            newest = self.accumulator.samples[-1]
            path.header.frame_id = newest.frame_id
            path.header.stamp.sec = newest.stamp_ns // 1_000_000_000
            path.header.stamp.nanosec = newest.stamp_ns % 1_000_000_000
            for sample in self.accumulator.samples:
                stamped = PoseStamped()
                stamped.header.frame_id = sample.frame_id
                stamped.header.stamp.sec = sample.stamp_ns // 1_000_000_000
                stamped.header.stamp.nanosec = sample.stamp_ns % 1_000_000_000
                stamped.pose = sample.pose
                path.poses.append(stamped)
        self.publisher.publish(path)

    def reset_callback(self, request, response):
        """Clear the path and publish the empty result."""
        del request
        self.accumulator.reset()
        self.publish_path()
        return response


def main(args=None):
    """Run the odometry-to-path node."""
    rclpy.init(args=args)
    node = OdomToPath()
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
