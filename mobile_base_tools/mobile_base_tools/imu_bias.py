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

"""Learn the gyro's bias whenever the robot stands still, and remove it.

A MEMS gyro reads a small non-zero rate at rest, and that offset drifts with
temperature. Integrated it turns straight into heading drift - the one error
the wheels cannot correct, because mecanum rollers slip in rotation. When the
wheel odometry says the base has been still for ``settle`` seconds, every gyro
sample is a direct measurement of the bias; this node averages those and
subtracts the estimate from everything it republishes.

The same stillness is a zero-velocity update in all but name: this node only
estimates the bias, the EKF still does the fusing.
"""

import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class BiasEstimator:
    """Running bias estimate from samples taken while still (ROS-free)."""

    def __init__(self, settle=1.0, rate=0.02, still_linear=0.005,
                 still_angular=0.01):
        self.settle = settle
        self.rate = rate          # weight of each new still sample
        self.still_linear = still_linear
        self.still_angular = still_angular
        self.bias = [0.0, 0.0, 0.0]
        self.still_since = None
        self.samples = 0

    def wheels(self, time, vx, vy, wz):
        """Record the wheel twist; track how long the base has been still."""
        still = (math.hypot(vx, vy) < self.still_linear
                 and abs(wz) < self.still_angular)
        if not still:
            self.still_since = None
        elif self.still_since is None:
            self.still_since = time

    def gyro(self, time, rates):
        """Feed one gyro sample; return it with the bias removed."""
        if (self.still_since is not None
                and time - self.still_since >= self.settle):
            # First samples average in fast, later ones track slow drift.
            weight = max(self.rate, 1.0 / (self.samples + 1))
            self.bias = [b + weight * (r - b)
                         for b, r in zip(self.bias, rates)]
            self.samples += 1
        return [r - b for r, b in zip(rates, self.bias)]


class ImuBias(Node):
    """Republish ``/imu/data`` bias-corrected on ``/imu/data_unbiased``."""

    def __init__(self):
        super().__init__('imu_bias')
        self.declare_parameter('settle', 1.0)
        self.estimator = BiasEstimator(
            settle=self.get_parameter('settle').value)
        self.publisher = self.create_publisher(Imu, '/imu/data_unbiased', 20)
        self.create_subscription(Imu, '/imu/data', self.on_imu, 20)
        self.create_subscription(
            Odometry, '/mobile_base_controller/odometry', self.on_wheels, 20)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_wheels(self, message):
        twist = message.twist.twist
        self.estimator.wheels(
            self.now(), twist.linear.x, twist.linear.y, twist.angular.z)

    def on_imu(self, message):
        rate = message.angular_velocity
        x, y, z = self.estimator.gyro(self.now(), [rate.x, rate.y, rate.z])
        message.angular_velocity.x = x
        message.angular_velocity.y = y
        message.angular_velocity.z = z
        self.publisher.publish(message)


def main(args=None):
    """Run the gyro bias estimator."""
    rclpy.init(args=args)
    node = ImuBias()
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
