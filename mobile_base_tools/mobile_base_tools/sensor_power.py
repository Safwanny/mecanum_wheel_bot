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

"""Switch the LiDAR and each ToF sensor on and off, as if cutting its power.

The Gazebo bridge publishes every sensor under ``/sensors/raw``; this node
relays the enabled ones onto the topics the rest of the stack already reads
(``/scan``, ``/tof/<face>/points``). A disabled sensor goes silent exactly as
unpowered hardware would, so nothing downstream needs to know switches exist.

Messages are relayed serialized, never decoded: nine sensors at 10-15 Hz
through Python would otherwise cost a core for no benefit.

Turning a sensor off also publishes one empty message on its output. RViz holds
the last cloud of a display until the next arrives, and the classifier holds
the last grid per sensor, so without it a switched-off sensor would freeze on
screen and keep marking obstacles from its final frame.

Each switch is a boolean parameter, ``enabled.<sensor>``, so it can be set from
the panel, ``ros2 param set``, or a launch file alike. State and measured rates
go out on ``/sensors/status`` as diagnostics.
"""

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan, PointCloud2

from mobile_base_tools.tof_floor_classifier import FACES, make_cloud

LIDAR = 'lidar'
SENSORS = (LIDAR,) + FACES


def output_topic(sensor):
    """The topic downstream nodes read for ``sensor``."""
    return '/scan' if sensor == LIDAR else '/tof/{}/points'.format(sensor)


def input_topic(sensor):
    """Where the bridge publishes ``sensor`` before the switch."""
    return '/sensors/raw' + output_topic(sensor)


class SensorPower(Node):
    """Relay enabled sensors; silence and clear disabled ones."""

    def __init__(self):
        super().__init__('sensor_power')
        self.declare_parameter('status_rate', 2.0)
        self.enabled = {}
        for sensor in SENSORS:
            self.enabled[sensor] = self.declare_parameter(
                'enabled.' + sensor, True).value

        self.last_raw = {}
        self.arrivals = {sensor: [] for sensor in SENSORS}
        self.message_types = {
            sensor: LaserScan if sensor == LIDAR else PointCloud2
            for sensor in SENSORS
        }

        # Best effort in accepts the bridge whatever its reliability; reliable
        # out still reaches best-effort subscribers such as RViz.
        inbound = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publishers_by_sensor = {}
        self.subscriptions_by_sensor = {}
        for sensor in SENSORS:
            kind = self.message_types[sensor]
            self.publishers_by_sensor[sensor] = self.create_publisher(
                kind, output_topic(sensor), 5)
            self.subscriptions_by_sensor[sensor] = self.create_subscription(
                kind, input_topic(sensor),
                lambda raw, name=sensor: self.relay(name, raw),
                inbound, raw=True)

        self.status_publisher = self.create_publisher(
            DiagnosticArray, '/sensors/status', 5)
        self.add_on_set_parameters_callback(self.on_parameters)
        self.create_timer(
            1.0 / self.get_parameter('status_rate').value, self.publish_status)

    def relay(self, sensor, raw):
        self.last_raw[sensor] = raw
        self.arrivals[sensor].append(time.monotonic())
        if self.enabled[sensor]:
            self.publishers_by_sensor[sensor].publish(raw)

    def on_parameters(self, parameters):
        for parameter in parameters:
            if not parameter.name.startswith('enabled.'):
                continue
            sensor = parameter.name[len('enabled.'):]
            if sensor not in self.enabled:
                return SetParametersResult(
                    successful=False, reason='unknown sensor ' + sensor)
            if not isinstance(parameter.value, bool):
                return SetParametersResult(
                    successful=False, reason='enabled.* must be a bool')
            was = self.enabled[sensor]
            self.enabled[sensor] = parameter.value
            if was and not parameter.value:
                self.clear(sensor)
            self.get_logger().info('{} {}'.format(
                sensor, 'on' if parameter.value else 'off'))
        return SetParametersResult(successful=True)

    def clear(self, sensor):
        """Publish one empty message so displays and consumers drop the sensor."""
        stamp = self.get_clock().now().to_msg()
        if sensor == LIDAR:
            raw = self.last_raw.get(sensor)
            if raw is None:
                return
            scan = deserialize_message(raw, LaserScan)
            scan.header.stamp = stamp
            scan.ranges = [math.inf] * len(scan.ranges)
            scan.intensities = []
            self.publishers_by_sensor[sensor].publish(scan)
            return
        self.publishers_by_sensor[sensor].publish(make_cloud(
            'tof_{}_link'.format(sensor), stamp, []))

    def rate(self, sensor, window=2.0):
        """Messages per second from the bridge over the last ``window``."""
        now = time.monotonic()
        recent = [t for t in self.arrivals[sensor] if now - t <= window]
        self.arrivals[sensor] = recent
        return len(recent) / window

    def publish_status(self):
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        for sensor in SENSORS:
            rate = self.rate(sensor)
            status = DiagnosticStatus(name=sensor, hardware_id=sensor)
            if not self.enabled[sensor]:
                status.level, status.message = DiagnosticStatus.OK, 'off'
            elif rate == 0.0:
                status.level, status.message = DiagnosticStatus.STALE, 'no data'
            else:
                status.level, status.message = DiagnosticStatus.OK, 'on'
            status.values = [
                KeyValue(key='enabled', value=str(self.enabled[sensor])),
                KeyValue(key='rate_hz', value='{:.1f}'.format(rate)),
            ]
            array.status.append(status)
        self.status_publisher.publish(array)


def main(args=None):
    """Run the sensor power switchboard."""
    rclpy.init(args=args)
    node = SensorPower()
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
