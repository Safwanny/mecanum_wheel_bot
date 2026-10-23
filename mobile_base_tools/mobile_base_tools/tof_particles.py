#!/usr/bin/env python3
"""Remember ToF obstacle hits as particles in the odometry frame.

Each point the classifier publishes on ``/tof/obstacles`` becomes a particle at
that spot in ``odom``, stamped with when it was seen. A hit within
``merge_distance`` of an existing particle refreshes it rather than adding a
new one, which bounds the count. A particle not seen again for ``decay``
seconds is dropped.

Because the particles are fixed in ``odom``, an obstacle stays where it was
while the robot moves on and it slips into a gap between fans or under the
ring. ``/tof/particles`` republishes the memory at ``publish_rate`` with an
``intensity`` field running from 1 (just seen) down to 0 (about to expire),
so RViz can fade old particles, and the local costmap marks from it.

One particle per zone hit: a real zone is a 7.5 degree cone, so a particle is
where the cone's centre line met something, not the exact surface point.
"""

import struct

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32,
               count=1),
]


class ParticleMemory:
    """Hits keyed on a ``merge_distance`` grid, each with its last sighting."""

    def __init__(self, decay=5.0, merge_distance=0.01):
        self.decay = decay
        self.cell = merge_distance
        self.particles = {}

    def add(self, points, now):
        for x, y, z in points:
            key = (round(x / self.cell), round(y / self.cell),
                   round(z / self.cell))
            self.particles[key] = (x, y, z, now)

    def snapshot(self, now):
        """Drop expired particles; return ``(x, y, z, freshness)`` for the rest."""
        alive = {}
        out = []
        for key, (x, y, z, seen) in self.particles.items():
            age = now - seen
            if age < self.decay:
                alive[key] = (x, y, z, seen)
                out.append((x, y, z, 1.0 - age / self.decay))
        self.particles = alive
        return out


class TofParticles(Node):

    def __init__(self):
        super().__init__('tof_particles')
        self.declare_parameter('input_topic', '/tof/obstacles')
        self.declare_parameter('output_topic', '/tof/particles')
        self.declare_parameter('frame', 'odom')
        self.declare_parameter('decay', 5.0)
        self.declare_parameter('merge_distance', 0.01)
        self.declare_parameter('publish_rate', 10.0)
        value = self.get_parameter
        self.frame = value('frame').value
        self.memory = ParticleMemory(
            value('decay').value, value('merge_distance').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            PointCloud2, value('input_topic').value, self.on_cloud, 10)
        self.publisher = self.create_publisher(
            PointCloud2, value('output_topic').value, 10)
        self.create_timer(1.0 / value('publish_rate').value, self.publish)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cloud(self, message):
        try:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.frame, message.header.frame_id,
                    Time.from_msg(message.header.stamp))
            except TransformException:
                # odom often trails the ToF stamp by a frame (~20 ms); the
                # latest pose is centimetres off at most, a lost frame is
                # a lost sighting.
                transform = self.tf_buffer.lookup_transform(
                    self.frame, message.header.frame_id, Time())
        except TransformException as error:
            self.get_logger().warn(
                f'no {self.frame} transform: {error}',
                throttle_duration_sec=5.0)
            return
        cloud = do_transform_cloud(message, transform)
        self.memory.add(
            point_cloud2.read_points_list(
                cloud, field_names=('x', 'y', 'z'), skip_nans=True),
            self.now())

    def publish(self):
        particles = self.memory.snapshot(self.now())
        header = Header(frame_id=self.frame,
                        stamp=self.get_clock().now().to_msg())
        data = b''.join(struct.pack('<ffff', *p) for p in particles)
        self.publisher.publish(PointCloud2(
            header=header, height=1, width=len(particles), is_dense=True,
            is_bigendian=False, fields=FIELDS, point_step=16,
            row_step=16 * len(particles), data=data))


def main(args=None):
    rclpy.init(args=args)
    node = TofParticles()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
