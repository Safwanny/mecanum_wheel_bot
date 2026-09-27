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

"""Draw the classified ToF scene as translucent walls, objects and floor.

Reads ``/tof/obstacles`` and ``/tof/floor`` from the floor classifier and
publishes a MarkerArray on ``/tof/shapes``. The geometry lives in
``tof_shape_model``; this node only moves messages and picks colours.

Each frame replaces the whole array (DELETEALL first), so a shape the tracker
drops can never linger in RViz under a stale id.
"""

import math

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray

from mobile_base_tools.tof_palette import (
    FLOOR_FILL,
    LABEL,
    warning_colour,
)
from mobile_base_tools.tof_shape_model import (
    WALL,
    ShapeTracker,
    cluster_points,
    fill_profile,
    fit_shapes,
    floor_profile,
    merge_walls,
)

# Walls and objects are told apart by form (slab vs box), so colour is free to
# say how close they are. Walls are drawn more solid: they are what the robot
# will follow and must not read as faint.
FILL_ALPHA = {WALL: 0.55, 'object': 0.45}


def read_points(message):
    return [
        (float(x), float(y), float(z))
        for x, y, z in point_cloud2.read_points(
            message, field_names=('x', 'y', 'z'), skip_nans=True)
    ]


def shape_marker(shape, marker_id, header):
    marker = Marker()
    marker.header = header
    marker.ns = shape.kind
    marker.id = marker_id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose.position.x = shape.x
    marker.pose.position.y = shape.y
    marker.pose.position.z = (shape.z_min + shape.z_max) / 2.0
    marker.pose.orientation.z = math.sin(shape.yaw / 2.0)
    marker.pose.orientation.w = math.cos(shape.yaw / 2.0)
    marker.scale.x = max(shape.length, 1e-3)
    marker.scale.y = max(shape.width, 1e-3)
    marker.scale.z = max(shape.z_max - shape.z_min, 1e-3)
    marker.color = warning_colour(shape.distance(), FILL_ALPHA[shape.kind])
    return marker


def floor_marker(radii, header, height=0.001):
    """One fan of triangles: the whole visible floor as a single region."""
    marker = Marker()
    marker.header = header
    marker.ns = 'floor'
    marker.id = 0
    marker.type = Marker.TRIANGLE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0
    marker.color = FLOOR_FILL
    width = 2.0 * math.pi / len(radii)
    centre = Point(z=height)
    for index, radius in enumerate(radii):
        following = radii[(index + 1) % len(radii)]
        # Bin centres, matching floor_profile's binning from -pi.
        start = -math.pi + (index + 0.5) * width
        end = start + width
        marker.points.extend([
            centre,
            Point(x=radius * math.cos(start), y=radius * math.sin(start),
                  z=height),
            Point(x=following * math.cos(end), y=following * math.sin(end),
                  z=height),
        ])
    return marker


def label_marker(shape, marker_id, header):
    marker = Marker()
    marker.header = header
    marker.ns = shape.kind + '_label'
    marker.id = marker_id
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.pose.position = Point(x=shape.x, y=shape.y, z=shape.z_max + 0.04)
    marker.pose.orientation.w = 1.0
    marker.scale.z = 0.028
    marker.color = LABEL
    # "seen to" because the ring never sees above ~100 mm: the height is a
    # lower bound, and the label should not suggest otherwise.
    marker.text = '{}  {:.2f} m  |  h>={:.0f} mm'.format(
        shape.kind.upper(), shape.distance(), shape.z_max * 1000.0)
    return marker


class ToFSceneShapes(Node):
    """Cluster classified ToF points into shapes and publish them as markers."""

    def __init__(self):
        super().__init__('tof_scene_shapes')
        self.declare_parameter('obstacle_topic', '/tof/obstacles')
        self.declare_parameter('floor_topic', '/tof/floor')
        self.declare_parameter('marker_topic', '/tof/shapes')
        self.declare_parameter('zones', 8)
        self.declare_parameter('field_of_view', math.radians(60.0))
        self.declare_parameter('min_cluster_points', 3)
        self.declare_parameter('wall_aspect_ratio', 3.0)
        self.declare_parameter('wall_min_length', 0.15)
        self.declare_parameter('confirm_frames', 2)
        self.declare_parameter('hold_frames', 4)
        self.declare_parameter('smoothing', 0.3)
        self.declare_parameter('publish_floor', True)
        # Off by default: one label per shape crowds the view.
        self.declare_parameter('publish_labels', False)
        self.declare_parameter('floor_bins', 48)

        value = self.get_parameter
        self.zone_angle = value('field_of_view').value / value('zones').value
        self.min_cluster_points = value('min_cluster_points').value
        self.wall_aspect_ratio = value('wall_aspect_ratio').value
        self.wall_min_length = value('wall_min_length').value
        self.publish_floor = value('publish_floor').value
        self.publish_labels = value('publish_labels').value

        def tracker():
            return ShapeTracker(
                confirm_frames=value('confirm_frames').value,
                hold_frames=value('hold_frames').value,
                smoothing=value('smoothing').value)

        self.obstacle_tracker = tracker()
        self.floor_bins = value('floor_bins').value
        self.smoothing = value('smoothing').value
        # Smoothed floor radius per bearing bin; the floor's own tracker.
        self.floor_radii = None
        self.floor_points = []

        self.publisher = self.create_publisher(
            MarkerArray, value('marker_topic').value, 5)
        self.create_subscription(
            PointCloud2, value('floor_topic').value, self.on_floor, 5)
        # The classifier publishes both clouds on one timer, obstacles first
        # or last; drawing on the obstacle cloud with the latest floor is at
        # most one frame out, which the trackers smooth over anyway.
        self.create_subscription(
            PointCloud2, value('obstacle_topic').value, self.on_obstacles, 5)

    def on_floor(self, message):
        self.floor_points = read_points(message)

    def on_obstacles(self, message):
        clusters = cluster_points(
            read_points(message), self.zone_angle,
            min_points=self.min_cluster_points)
        walls_and_objects = self.obstacle_tracker.update(merge_walls([
            shape for cluster in clusters for shape in fit_shapes(
                cluster, wall_aspect_ratio=self.wall_aspect_ratio,
                wall_min_length=self.wall_min_length)
        ]))
        floor = self.update_floor() if self.publish_floor else None

        # Stamped zero so RViz draws with the latest transform: the clouds
        # arrive a few ms ahead of odom -> base_footprint, and a marker it
        # cannot transform at its own stamp is dropped - the blink that
        # tof_ray_markers already avoids the same way.
        header = message.header
        header.stamp = Time().to_msg()
        markers = [Marker(action=Marker.DELETEALL)]
        for track_id, shape in walls_and_objects.items():
            markers.append(shape_marker(shape, track_id, header))
            if self.publish_labels:
                markers.append(label_marker(shape, track_id, header))
        if floor is not None:
            markers.append(floor_marker(floor, header))
        self.publisher.publish(MarkerArray(markers=markers))

    def update_floor(self):
        """Blend this frame's floor profile into the running one."""
        seen = fill_profile(floor_profile(self.floor_points, self.floor_bins))
        if seen is None:
            return self.floor_radii
        if self.floor_radii is None:
            self.floor_radii = seen
        else:
            self.floor_radii = [
                old + self.smoothing * (new - old)
                for old, new in zip(self.floor_radii, seen)
            ]
        return self.floor_radii


def main(args=None):
    """Run the ToF scene shapes node."""
    rclpy.init(args=args)
    node = ToFSceneShapes()
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
