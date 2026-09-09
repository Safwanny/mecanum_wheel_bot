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

"""Turn eight ToF zone grids into the obstacle cloud a costmap should mark.

The costmap must never see the floor. Every sensor in the ring is aimed partly
at the ground, so feeding the raw clouds to a costmap paints the floor lethal
and the robot walls itself in behind its own returns. The usual defence is a
``min_obstacle_height`` cut in the costmap, but that is a blunt slice in z: the
moment the floor is not where the slice assumes, phantom obstacles appear.
Classifying here means the floor never enters the costmap at all.

Two things keep it steady, and both were added after watching it misbehave.

**The floor is fitted every frame** from the steep near rows of all eight
sensors pooled together. One sensor's three rows span 150 mm and would fit a
plane badly; the ring pins the tilt in both axes.

**A zone must be seen repeatedly before it is published, and survives a few
missed frames after.** Measured on open floor, consecutive frames agreed on only
38 percent of their obstacle points - the marks appeared and vanished from one
frame to the next, and a costmap fed that flicker accumulates the union of every
spurious mark and boxes the robot in. Requiring agreement across frames costs
about 130 ms of lag and removes almost all of it, because noise does not repeat
in the same zone while a real obstacle does.
"""

import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import tf2_ros

from mobile_base_tools.tof_floor_model import (
    FLOOR,
    OBSTACLE,
    FloorGeometry,
    classify_zone,
    fit_plane,
    floor_candidates,
)
from mobile_base_tools.tof_ray_markers import zone_directions


FACES = (
    'front', 'rear', 'left', 'right',
    'front_left', 'front_right', 'rear_left', 'rear_right',
)

POINT_FIELDS = (
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
)


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def rotate_yaw(direction, yaw):
    """Turn a sensor-frame ray into the body frame using its mount yaw."""
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    x, y, z = direction
    return (x * cos_yaw - y * sin_yaw, x * sin_yaw + y * cos_yaw, z)


def make_cloud(frame_id, stamp, points):
    """Pack XYZ float32 points into an unorganised PointCloud2."""
    header = Header()
    header.frame_id = frame_id
    header.stamp = stamp
    data = b''.join(struct.pack('<fff', *point) for point in points)
    return PointCloud2(
        header=header,
        height=1,
        width=len(points),
        is_dense=True,
        is_bigendian=False,
        fields=list(POINT_FIELDS),
        point_step=12,
        row_step=12 * len(points),
        data=data,
    )


class ToFFloorClassifier(Node):
    """Classify every ToF zone and publish only what a costmap should mark."""

    def __init__(self):
        super().__init__('tof_floor_classifier')
        self.declare_parameter('faces', list(FACES))
        self.declare_parameter('cloud_topic_template', '/tof/{face}/points')
        # base_footprint sits on the ground, so every height this node computes
        # is directly a height above the floor and the costmap's obstacle-height
        # band means what it says.
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('sensor_frame_template', 'tof_{face}_link')
        self.declare_parameter('zones', 8)
        self.declare_parameter('field_of_view', math.radians(60.0))
        self.declare_parameter('min_range', 0.02)
        self.declare_parameter('max_range', 3.5)
        # Marking range. Past 1.083 m every ray in this ring clears the robot's
        # own 99.5 mm roof, so nothing beyond that could be an obstacle to it.
        self.declare_parameter('mark_max_range', 0.8)
        self.declare_parameter('robot_height', 0.0995)
        self.declare_parameter('floor_tolerance', 0.020)
        self.declare_parameter('obstacle_min_height', 0.025)
        self.declare_parameter('range_shortfall', 0.030)
        self.declare_parameter(
            'plane_angle_tolerance', math.radians(1.2))
        # A zone must be seen this many frames running before it is
        # published, and is held for this many after it stops being seen.
        self.declare_parameter('confirm_frames', 2)
        self.declare_parameter('hold_frames', 4)
        # How much of each new reading to accept. Range noise moves a mark
        # about a centimetre frame to frame even when the zone is steadily
        # looking at the same wall, which reads as shimmer.
        self.declare_parameter('position_smoothing', 0.25)
        # Rows that feed the floor fit: the steep ones, which reach the floor
        # within 150 mm and shift 2.5-13 mm per degree of attitude error. Row 3
        # shifts 116 and is deliberately excluded.
        self.declare_parameter('floor_rows', [0, 1, 2])
        self.declare_parameter('publish_rate', 15.0)
        self.declare_parameter('publish_floor', True)

        self.faces = list(self.get_parameter('faces').value)
        self.base_frame = self.get_parameter('base_frame').value
        self.sensor_frame_template = self.get_parameter(
            'sensor_frame_template').value
        self.zones = self.get_parameter('zones').value
        field_of_view = self.get_parameter('field_of_view').value
        zone_angle = field_of_view / self.zones
        self.directions = zone_directions(
            self.zones, field_of_view / 2.0 - zone_angle / 2.0)
        self.floor_rows = set(self.get_parameter('floor_rows').value)
        self.mark_max_range = self.get_parameter('mark_max_range').value
        self.publish_floor = self.get_parameter('publish_floor').value

        self.geometry = FloorGeometry(
            robot_height=self.get_parameter('robot_height').value,
            floor_tolerance=self.get_parameter('floor_tolerance').value,
            obstacle_min_height=self.get_parameter(
                'obstacle_min_height').value,
            range_shortfall=self.get_parameter('range_shortfall').value,
            plane_angle_tolerance=self.get_parameter(
                'plane_angle_tolerance').value,
            max_range=self.get_parameter('max_range').value,
            min_range=self.get_parameter('min_range').value,
        )

        self.latest = {}
        # Extrinsics are resolved once and cached. These joints are fixed, so a
        # lookup can only fail while TF is still filling up at startup.
        self.placements = {}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        template = self.get_parameter('cloud_topic_template').value
        self.cloud_subscriptions = {
            face: self.create_subscription(
                PointCloud2,
                template.format(face=face),
                lambda message, name=face: self.latest.__setitem__(
                    name, message),
                qos,
            )
            for face in self.faces
        }

        self.confirm_frames = self.get_parameter('confirm_frames').value
        self.hold_frames = self.get_parameter('hold_frames').value
        self.smoothing = self.get_parameter('position_smoothing').value
        # (face, zone) -> [score, last point]. Score rises while a zone reads as
        # an obstacle and falls when it does not; a zone is published once the
        # score reaches confirm_frames and keeps being published until it falls
        # back below, which is what stops the marks flickering.
        self.tracked = {}

        self.obstacle_publisher = self.create_publisher(
            PointCloud2, '/tof/obstacles', 5)
        self.floor_publisher = self.create_publisher(
            PointCloud2, '/tof/floor', 5)

        self.timer = self.create_timer(
            1.0 / self.get_parameter('publish_rate').value, self.publish)

    def placement(self, face):
        """Return (origin, body-frame directions) for one sensor, or None."""
        if face in self.placements:
            return self.placements[face]
        frame = self.sensor_frame_template.format(face=face)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, frame, rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        rotation = transform.transform.rotation
        yaw = yaw_from_quaternion(
            rotation.x, rotation.y, rotation.z, rotation.w)
        offset = transform.transform.translation
        placement = (
            (offset.x, offset.y, offset.z),
            [rotate_yaw(direction, yaw) for direction in self.directions],
        )
        self.placements[face] = placement
        return placement

    def zone_ranges(self, message):
        """Return one range per zone, None where the zone did not return."""
        expected = self.zones * self.zones
        points = list(point_cloud2.read_points(
            message, field_names=('x', 'y', 'z'), skip_nans=False))
        if len(points) != expected:
            return [None] * expected
        ranges = []
        for x, y, z in points:
            distance = math.sqrt(x * x + y * y + z * z)
            ranges.append(distance if math.isfinite(distance) else None)
        return ranges

    def observations(self):
        """Yield (face, origin, directions, ranges) for every ready sensor.

        The face name is carried through because the obstacle tracker keys on
        it: a zone has to be identifiable frame to frame for repeated sightings
        to mean anything.
        """
        for face, message in self.latest.items():
            placed = self.placement(face)
            if placed is None:
                continue
            origin, directions = placed
            yield face, origin, directions, self.zone_ranges(message)

    def publish(self):
        if not self.latest:
            return
        resolved = list(self.observations())
        if not resolved:
            return

        # Pool the near rows from the whole ring before fitting. One sensor's
        # three rows span 150 mm and would fit a plane badly; eight sensors'
        # worth ring the robot and pin the tilt in both axes.
        candidates = []
        for _, origin, directions, ranges in resolved:
            candidates.extend(floor_candidates(
                origin, directions, ranges, self.floor_rows, self.zones,
                self.geometry.max_range))
        plane = fit_plane(candidates)

        seen = set()
        floor = []
        for face, origin, directions, ranges in resolved:
            for index, (direction, measured) in enumerate(
                    zip(directions, ranges)):
                label, point, _ = classify_zone(
                    self.geometry, plane, origin, direction, measured)
                if point is None:
                    continue
                if math.dist(origin, point) > self.mark_max_range:
                    continue
                if label == OBSTACLE:
                    key = (face, index)
                    seen.add(key)
                    score, previous = self.tracked.get(key, (0, None))
                    if previous is not None:
                        blend = self.smoothing
                        point = tuple(
                            blend * new + (1.0 - blend) * old
                            for new, old in zip(point, previous))
                    self.tracked[key] = (
                        min(score + 1, self.hold_frames), point)
                elif label == FLOOR and self.publish_floor:
                    floor.append(point)

        obstacles = []
        for key, (score, point) in list(self.tracked.items()):
            if key not in seen:
                score -= 1
                if score <= 0:
                    del self.tracked[key]
                    continue
                self.tracked[key] = (score, point)
            if score >= self.confirm_frames:
                obstacles.append(point)

        stamp = self.get_clock().now().to_msg()
        self.obstacle_publisher.publish(
            make_cloud(self.base_frame, stamp, obstacles))
        if self.publish_floor:
            self.floor_publisher.publish(
                make_cloud(self.base_frame, stamp, floor))


def main(args=None):
    """Run the ToF floor classifier node."""
    rclpy.init(args=args)
    node = ToFFloorClassifier()
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
