#!/usr/bin/env python3
"""Run a repeatable open-loop mecanum motion sequence."""

from dataclasses import dataclass
import math

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node


@dataclass(frozen=True)
class Segment:
    """One constant body-frame velocity command and its duration."""

    name: str
    linear_x: float
    linear_y: float
    angular_z: float
    duration: float


def validate_parameters(**parameters):
    """Reject values that would produce invalid timing or unsafe execution."""
    for name, value in parameters.items():
        if value <= 0.0:
            raise ValueError(f'{name} must be greater than zero')


def build_sequence(
        speed=0.15, side_length=1.0, angular_speed=0.35,
        settle_duration=3.0, pause_duration=2.0,
        final_stop_duration=5.0):
    """Build the closed square, diamond, and rotation sequence."""
    validate_parameters(
        speed=speed,
        side_length=side_length,
        angular_speed=angular_speed,
        settle_duration=settle_duration,
        pause_duration=pause_duration,
        final_stop_duration=final_stop_duration,
    )
    straight_duration = side_length / speed
    diagonal = speed / math.sqrt(2.0)
    quarter_turn = (math.pi / 2.0) / angular_speed
    half_turn = math.pi / angular_speed
    stop = (0.0, 0.0, 0.0)
    return [
        Segment('settle', *stop, settle_duration),
        Segment('square forward', speed, 0.0, 0.0, straight_duration),
        Segment('square left', 0.0, speed, 0.0, straight_duration),
        Segment('square backward', -speed, 0.0, 0.0, straight_duration),
        Segment('square right', 0.0, -speed, 0.0, straight_duration),
        Segment('square stop', *stop, pause_duration),
        Segment('diamond forward-left', diagonal, diagonal, 0.0,
                straight_duration),
        Segment('diamond backward-left', -diagonal, diagonal, 0.0,
                straight_duration),
        Segment('diamond backward-right', -diagonal, -diagonal, 0.0,
                straight_duration),
        Segment('diamond forward-right', diagonal, -diagonal, 0.0,
                straight_duration),
        Segment('diamond stop', *stop, pause_duration),
        Segment('rotate +90', 0.0, 0.0, angular_speed, quarter_turn),
        Segment('rotate -180', 0.0, 0.0, -angular_speed, half_turn),
        Segment('rotate +90 return', 0.0, 0.0, angular_speed, quarter_turn),
        Segment('final stop', *stop, final_stop_duration),
    ]


class MecanumMotionTest(Node):
    """Publish the configured sequence using the ROS clock."""

    def __init__(self):
        super().__init__('mecanum_motion_test')
        defaults = {
            'speed': 0.15,
            'side_length': 1.0,
            'angular_speed': 0.35,
            'publish_rate': 20.0,
            'settle_duration': 3.0,
            'pause_duration': 2.0,
            'final_stop_duration': 5.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.declare_parameter('repeat', False)
        self.declare_parameter('command_topic', 'mobile_base_controller/reference')
        self.declare_parameter('frame_id', 'base_link')

        values = {
            name: float(self.get_parameter(name).value) for name in defaults
        }
        validate_parameters(**values)
        self.sequence = build_sequence(
            values['speed'], values['side_length'], values['angular_speed'],
            values['settle_duration'], values['pause_duration'],
            values['final_stop_duration'])
        self.repeat = bool(self.get_parameter('repeat').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        if not self.frame_id:
            raise ValueError('frame_id must not be empty')
        self.publisher = self.create_publisher(
            TwistStamped, self.get_parameter('command_topic').value, 10)
        self.index = 0
        self.segment_start = self.get_clock().now()
        self.finished = False
        self.get_logger().info(self._segment_description())
        self.timer = self.create_timer(
            1.0 / values['publish_rate'], self.timer_callback)

    def _segment_description(self):
        segment = self.sequence[self.index]
        return (
            f'Segment {self.index + 1}/{len(self.sequence)}: {segment.name} '
            f'(x={segment.linear_x:.3f}, y={segment.linear_y:.3f}, '
            f'z={segment.angular_z:.3f}, {segment.duration:.2f}s)')

    def publish(self, segment=None):
        """Publish one stamped command, defaulting to zero velocity."""
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        if segment is not None:
            message.twist.linear.x = segment.linear_x
            message.twist.linear.y = segment.linear_y
            message.twist.angular.z = segment.angular_z
        self.publisher.publish(message)

    def timer_callback(self):
        """Advance timed segments and terminate after the final stop."""
        now = self.get_clock().now()
        elapsed = (now - self.segment_start).nanoseconds / 1e9
        segment = self.sequence[self.index]
        if elapsed >= segment.duration:
            self.index += 1
            if self.index >= len(self.sequence):
                if self.repeat:
                    self.index = 0
                else:
                    self.publish()
                    self.finished = True
                    self.timer.cancel()
                    self.get_logger().info('Motion sequence complete; stopped.')
                    return
            self.segment_start = now
            segment = self.sequence[self.index]
            self.get_logger().info(self._segment_description())
        self.publish(segment)


def main(args=None):
    """Run the open-loop motion sequence node."""
    rclpy.init(args=args)
    node = None
    try:
        node = MecanumMotionTest()
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ValueError) as error:
        if node is not None:
            node.get_logger().warning(f'Stopping motion test: {error}')
    finally:
        if node is not None and rclpy.ok():
            node.publish()
            rclpy.spin_once(node, timeout_sec=0.1)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
