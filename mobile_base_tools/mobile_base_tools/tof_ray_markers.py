#!/usr/bin/env python3
"""Draw ToF zone rays as RViz line markers, hits and misses alike.

The point clouds the ToF sensors publish carry only the zones that returned
something, which is exactly the wrong picture for judging coverage: the zones
that saw nothing are invisible, and a fan that covers half of what you assumed
looks identical to one that covers all of it.

So this node draws the grid rather than the cloud. Every zone gets a segment
whether or not it returned, running from the sensor's minimum range out to the
point of contact, or out to maximum range where there was no contact. The near
dead zone is deliberately left blank: nothing is measured inside it, so drawing
a line there would claim coverage the part does not have.

Hits and misses are drawn in different colours because the distinction is the
one that matters on a real robot. A drop-off does not read as a long range - it
reads as no return at all, the same as an open doorway, and any cliff logic
built on this has to treat the two apart.

Every sensor goes into one MarkerArray published on a timer, rather than one
array per sensor published on arrival. Publishing per arrival put eight
single-marker messages on the wire per cycle, so a dropped message left one
sensor stale while its neighbours moved on. One timed message carries the whole
ring, so the worst a drop costs is a frame of staleness everywhere at once.

The markers are stamped zero on purpose, which asks RViz for the latest
available transform instead of the transform at a particular instant. Measured
against a running simulation, 89 percent of the clouds arrive stamped ahead of
the newest odom to base_footprint transform, by 9.5 ms on average - the sensors
are driven by Gazebo and the transform by the EKF behind it. RViz cannot
transform a marker into the fixed frame at a stamp it has no transform for, so
it drops it, and the ring blinks. Nothing is lost by asking for the latest
instead: the ray geometry is expressed in the sensor's own frame and is exact
there regardless, so the stamp only decides where the robot is drawn, and 9.5 ms
of robot motion is far below a line width.
"""

import math

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import UInt8MultiArray
from visualization_msgs.msg import Marker, MarkerArray

from mobile_base_tools.tof_palette import (
    HIT_ALERT, LABEL, RAY_ALERT, RAY_HIT, RAY_MISS, SENSOR_OFF)


def zone_directions(zones, half_span):
    """Return unit ray directions for a zones x zones grid, row major.

    The angles are zone centres, not field edges: with `zones` samples across
    the field of view the outermost ray sits half a zone inside the edge, which
    is what `half_span` already accounts for.

    Row and column both run from -half_span upward, which is the order Gazebo
    fills the cloud in: row 0 is the bottom of the grid, not the top, and the
    ring field on each point carries that same row index. Getting this
    backwards mirrors the miss rays about the horizon, which is worse than it
    sounds - the misses are what a drop-off looks like, so an inverted grid
    draws the cliff into the ceiling.
    """
    if zones < 2:
        steps = [0.0]
    else:
        step = 2.0 * half_span / (zones - 1)
        steps = [-half_span + index * step for index in range(zones)]
    directions = []
    for pitch in steps:
        for yaw in steps:
            directions.append((
                math.cos(pitch) * math.cos(yaw),
                math.cos(pitch) * math.sin(yaw),
                math.sin(pitch),
            ))
    return directions


def scaled(direction, distance):
    """Return a Point at `distance` along a unit `direction`."""
    point = Point()
    point.x = direction[0] * distance
    point.y = direction[1] * distance
    point.z = direction[2] * distance
    return point


class ToFRayMarkers(Node):
    """Publish one LINE_LIST marker per ToF sensor, one segment per zone."""

    def __init__(self):
        super().__init__('tof_ray_markers')
        self.declare_parameter(
            'cloud_topics',
            [
                '/tof/front/points',
                '/tof/rear/points',
                '/tof/left/points',
                '/tof/right/points',
                '/tof/front_left/points',
                '/tof/front_right/points',
                '/tof/rear_left/points',
                '/tof/rear_right/points',
            ],
        )
        self.declare_parameter('marker_topic', '/tof/rays')
        self.declare_parameter('zones', 8)
        self.declare_parameter('field_of_view', math.radians(60.0))
        self.declare_parameter('min_range', 0.02)
        self.declare_parameter('max_range', 3.5)
        self.declare_parameter('line_width', 0.001)
        # How far a ray that saw nothing is drawn. The sensor reaches further,
        # but nothing past the marking range is acted on, and 3.5 m spokes on
        # every miss bury the returns that matter.
        self.declare_parameter('miss_length', 1.0)
        self.declare_parameter('publish_rate', 15.0)

        self.zones = self.get_parameter('zones').value
        field_of_view = self.get_parameter('field_of_view').value
        zone_angle = field_of_view / self.zones
        self.half_span = field_of_view / 2.0
        self.directions = zone_directions(
            self.zones, field_of_view / 2.0 - zone_angle / 2.0)
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.line_width = self.get_parameter('line_width').value
        self.miss_length = self.get_parameter('miss_length').value

        # Shared palette: every sensor draws identically.
        self.hit_colour = RAY_HIT
        self.miss_colour = RAY_MISS
        self.declare_parameter('hit_marker_size', 0.014)
        self.hit_marker_size = self.get_parameter('hit_marker_size').value

        # Sensor data is best effort, and the bridge republishes it as such.
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(
            MarkerArray, self.get_parameter('marker_topic').value, 10)
        self.latest = {}
        # face -> set of zone indices the classifier confirmed as obstacles.
        self.obstacle_zones = {}
        # Sensors the power switch has turned off, drawn as an OFF wedge.
        self.disabled = set()
        self.create_subscription(
            UInt8MultiArray, '/tof/obstacle_zones', self.on_zones, 5)
        self.create_subscription(
            DiagnosticArray, '/sensors/status', self.on_status, 5)
        self.subscriptions_by_topic = {}
        for index, topic in enumerate(
                self.get_parameter('cloud_topics').value):
            self.subscriptions_by_topic[topic] = self.create_subscription(
                PointCloud2,
                topic,
                lambda message, marker_id=index, name=topic:
                    self.latest.__setitem__(marker_id, (name, message)),
                qos,
            )
        self.timer = self.create_timer(
            1.0 / self.get_parameter('publish_rate').value, self.publish_rays)

    def on_zones(self, message):
        if not message.layout.dim:
            return
        faces = message.layout.dim[0].label.split(',')
        zones = message.layout.dim[1].size
        self.obstacle_zones = {
            face: {
                zone for zone in range(zones)
                if message.data[row * zones + zone]
            }
            for row, face in enumerate(faces)
        }

    def on_status(self, message):
        self.disabled = {
            status.name for status in message.status
            if status.message == 'off'
        }

    def zone_endpoints(self, message):
        """Return one contact point per zone, None where the zone missed.

        Gazebo delivers an organised cloud, one point per zone in grid order,
        with misses left non-finite. A cloud that arrives filtered to hits only
        cannot be indexed back onto the grid, so it is reported as all misses
        rather than silently mapping the wrong ranges onto the wrong zones.

        Contact points are passed through as measured rather than rebuilt from
        the zone index. Only the miss rays are drawn along a reconstructed
        direction, so if the grid ordering here ever disagreed with Gazebo's,
        the hits would still land where the sensor actually saw them.
        """
        expected = self.zones * self.zones
        points = list(point_cloud2.read_points(
            message, field_names=('x', 'y', 'z'), skip_nans=False))
        if len(points) != expected:
            return [None] * expected
        endpoints = []
        for x, y, z in points:
            distance = math.sqrt(x * x + y * y + z * z)
            if not math.isfinite(distance) or distance <= self.min_range:
                endpoints.append(None)
            else:
                endpoints.append(Point(x=float(x), y=float(y), z=float(z)))
        return endpoints

    def sensor_markers(self, topic, message, marker_id):
        """Turn one sensor's cloud into its rays and its obstacle hits.

        A ray is drawn orange only when the classifier confirmed its zone as
        an obstacle, not whenever it returns: the downward rows hit the floor
        constantly, and colouring every return would colour everything.
        """
        face = face_of(topic)
        frame = message.header.frame_id or 'tof_{}_link'.format(face)
        if face in self.disabled:
            return self.off_markers(frame, marker_id)
        alerts = self.obstacle_zones.get(face, set())

        rays = self.marker(frame, 'tof_rays', marker_id, Marker.LINE_LIST)
        rays.scale.x = self.line_width
        hits = self.marker(
            frame, 'tof_hits', marker_id, Marker.SPHERE_LIST)
        hits.scale.x = hits.scale.y = hits.scale.z = self.hit_marker_size
        hits.color = HIT_ALERT

        for zone, (direction, endpoint) in enumerate(
                zip(self.directions, self.zone_endpoints(message))):
            rays.points.append(scaled(direction, self.min_range))
            if endpoint is None:
                rays.points.append(scaled(direction, self.miss_length))
                colour = self.miss_colour
            elif zone in alerts:
                rays.points.append(endpoint)
                hits.points.append(endpoint)
                colour = RAY_ALERT
            else:
                rays.points.append(endpoint)
                colour = self.hit_colour
            rays.colors.append(colour)
            rays.colors.append(colour)
        # An empty SPHERE_LIST is still sent, so a hit that clears is removed.
        return [rays, hits]

    def off_markers(self, frame, marker_id, radius=0.18):
        """Outline a switched-off sensor's field of view and label it OFF."""
        half = self.half_span
        outline = self.marker(
            frame, 'tof_rays', marker_id, Marker.LINE_STRIP)
        outline.scale.x = 0.003
        outline.color = SENSOR_OFF
        outline.points.append(Point())
        steps = 12
        for step in range(steps + 1):
            angle = -half + 2.0 * half * step / steps
            outline.points.append(Point(
                x=radius * math.cos(angle), y=radius * math.sin(angle)))
        outline.points.append(Point())
        label = self.marker(
            frame, 'tof_hits', marker_id, Marker.TEXT_VIEW_FACING)
        label.pose.position.x = radius + 0.04
        label.scale.z = 0.03
        label.color = LABEL
        label.text = 'OFF'
        return [outline, label]

    @staticmethod
    def marker(frame, namespace, marker_id, kind):
        """A marker in ``frame``, stamped zero so RViz uses the latest TF."""
        marker = Marker()
        marker.header.frame_id = frame
        marker.ns = namespace
        marker.id = marker_id
        marker.type = kind
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def publish_rays(self):
        """Publish the whole ring as one array, on the timer."""
        if not self.latest:
            return
        markers = []
        for marker_id, (topic, message) in sorted(self.latest.items()):
            markers.extend(self.sensor_markers(topic, message, marker_id))
        self.publisher.publish(MarkerArray(markers=markers))


def face_of(topic):
    """'/tof/front_left/points' -> 'front_left'."""
    return topic.strip('/').split('/')[-2]


def main(args=None):
    """Run the ToF ray marker node."""
    rclpy.init(args=args)
    node = ToFRayMarkers()
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
