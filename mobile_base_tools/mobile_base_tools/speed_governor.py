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

"""Cap a velocity command by what the ToF ring sees along its direction.

Takes a TwistStamped on ``cmd_in``, looks down the corridor the body would
sweep moving that way, picks a level from ``bug_model.SpeedLevels`` for the
nearest wall and the nearest other obstacle, and republishes the command with
its translation scaled to that level's speed. Direction is kept; only speed
changes. Rotation passes through.

A squeeze - something within a few centimetres beside the body, as in a
narrow gap - slows it too, even with the corridor ahead clear.

Levels drop at once and rise only after the faster band has been clear for
``rise_delay`` - a level that flickers up between frames would undo the
margin it exists for. With no obstacle data for ``stale_after`` the governor
stops the robot: silence from the ring is not the same as a clear path.
"""

import math
import time

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from mobile_base_tools import tof_palette as palette
from mobile_base_tools.bug_model import (
    CAUTION, CRAWL, CRUISE, LEVELS, SLOW, STOP, SpeedLevels,
    corridor_distance, side_clearance, slowest, split_walls)

LEVEL_COLOUR = {
    CRUISE: palette.CLEAR, CAUTION: palette.CAUTION, SLOW: palette.CAUTION,
    CRAWL: palette.CRITICAL, STOP: palette.CRITICAL,
}


class SpeedGovernor(Node):
    """Scale commanded translation to the level the corridor allows."""

    def __init__(self):
        super().__init__('speed_governor')
        value = self.declare_parameter
        value('cmd_in', '/governor/cmd_vel')
        value('cmd_out', '/mobile_base/cmd_vel')
        value('obstacle_topic', '/tof/obstacles')
        value('rise_delay', 0.5)
        value('stale_after', 0.5)
        for name, default in (
                ('cruise', 0.50), ('caution', 0.33), ('slow', 0.20),
                ('crawl', 0.10), ('caution_range', 1.0),
                ('wall_caution_range', 0.65), ('slow_range', 0.4),
                ('crawl_range', 0.25), ('stop_range', 0.12),
                ('squeeze_slow', 0.15), ('squeeze_crawl', 0.06)):
            value('levels.' + name, default)
        get = self.get_parameter
        self.levels = SpeedLevels(**{
            name: get('levels.' + name).value for name in (
                'cruise', 'caution', 'slow', 'crawl', 'caution_range',
                'wall_caution_range', 'slow_range', 'crawl_range',
                'stop_range', 'squeeze_slow', 'squeeze_crawl')})
        self.rise_delay = get('rise_delay').value
        self.stale_after = get('stale_after').value

        self.walls, self.others = [], []
        self.last_cloud = None
        self.level = STOP
        self.faster_since = None

        self.publisher = self.create_publisher(
            TwistStamped, get('cmd_out').value, 10)
        self.level_publisher = self.create_publisher(
            String, '/speed_governor/level', 10)
        self.marker_publisher = self.create_publisher(
            Marker, '/speed_governor/marker', 10)
        self.create_subscription(
            PointCloud2, get('obstacle_topic').value, self.on_cloud, 5)
        self.create_subscription(
            TwistStamped, get('cmd_in').value, self.on_command, 10)

    def on_cloud(self, message):
        points = [
            (float(x), float(y), float(z))
            for x, y, z in point_cloud2.read_points(
                message, field_names=('x', 'y', 'z'), skip_nans=True)
        ]
        self.walls, self.others = split_walls(points)
        self.last_cloud = time.monotonic()

    def allowed(self, vx, vy):
        """The level the corridor along (vx, vy) allows right now."""
        if (self.last_cloud is None
                or time.monotonic() - self.last_cloud > self.stale_after):
            return STOP
        if math.hypot(vx, vy) < 1e-6:
            return CRUISE
        return slowest(
            self.levels.level(corridor_distance(self.walls, vx, vy), True),
            self.levels.level(corridor_distance(self.others, vx, vy)),
            self.levels.squeeze(
                side_clearance(self.walls + self.others, vx, vy)))

    def settle(self, wanted):
        """Drop at once; rise only once the faster level has held a while."""
        if LEVELS.index(wanted) <= LEVELS.index(self.level):
            self.level, self.faster_since = wanted, None
            return
        now = time.monotonic()
        if self.faster_since is None:
            self.faster_since = now
        elif now - self.faster_since >= self.rise_delay:
            # One step at a time, so a clear corridor ramps back up.
            self.level = LEVELS[LEVELS.index(self.level) + 1]
            self.faster_since = now

    def on_command(self, message):
        twist = message.twist
        vx, vy = twist.linear.x, twist.linear.y
        self.settle(self.allowed(vx, vy))
        limit = self.levels.speeds[self.level]
        speed = math.hypot(vx, vy)
        scale = 1.0 if speed <= limit else limit / speed
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = message.header.frame_id or 'base_footprint'
        out.twist.linear.x = vx * scale
        out.twist.linear.y = vy * scale
        out.twist.angular.z = twist.angular.z
        self.publisher.publish(out)
        # Level and its cap, e.g. 'caution 0.33', for the panel.
        self.level_publisher.publish(
            String(data='{} {:.2f}'.format(self.level, limit)))
        self.marker_publisher.publish(self.badge(limit))

    def badge(self, limit):
        """A label over the robot: the level and its speed cap."""
        marker = Marker()
        marker.header.frame_id = 'base_footprint'
        marker.ns = 'speed_level'
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.z = 0.28
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.05
        marker.color = palette.rgba(LEVEL_COLOUR[self.level], 0.95)
        marker.text = '{}  {:.2f} m/s'.format(self.level.upper(), limit)
        return marker


def main(args=None):
    """Run the speed governor."""
    rclpy.init(args=args)
    node = SpeedGovernor()
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
