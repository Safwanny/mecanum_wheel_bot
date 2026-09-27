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

"""Pin the gyro bias estimator and the laser pose differentiation."""

import math

import pytest

from mobile_base_tools.imu_bias import BiasEstimator
from mobile_base_tools.laser_odometry import body_velocity


def test_bias_is_learned_only_after_the_base_has_settled():
    estimator = BiasEstimator(settle=1.0)
    estimator.wheels(0.0, 0.0, 0.0, 0.0)
    # Within the settle time: passed through untouched.
    assert estimator.gyro(0.5, [0.0, 0.0, 0.02])[2] == pytest.approx(0.02)
    for step in range(200):
        estimator.gyro(1.0 + step * 0.01, [0.0, 0.0, 0.02])
    assert estimator.bias[2] == pytest.approx(0.02, abs=1e-3)
    assert estimator.gyro(3.0, [0.0, 0.0, 0.02])[2] == pytest.approx(
        0.0, abs=1e-3)


def test_bias_is_frozen_while_moving():
    estimator = BiasEstimator(settle=0.0)
    estimator.wheels(0.0, 0.0, 0.0, 0.0)
    estimator.gyro(0.1, [0.0, 0.0, 0.01])
    learned = estimator.bias[2]
    estimator.wheels(0.2, 0.3, 0.0, 0.5)   # turning: the gyro sees motion
    estimator.gyro(0.3, [0.0, 0.0, 0.5])
    assert estimator.bias[2] == learned
    # And a turning robot is corrected by the bias, not zeroed.
    assert estimator.gyro(0.4, [0.0, 0.0, 0.5])[2] == pytest.approx(
        0.5 - learned)


def test_laser_pose_differentiates_to_body_velocity_including_strafe():
    # Facing +y, moving +x in the world: a pure strafe to the right.
    vx, vy, wz = body_velocity(
        (0.0, 0.0, math.pi / 2), (0.1, 0.0, math.pi / 2), 0.5)
    assert vx == pytest.approx(0.0, abs=1e-9)
    assert vy == pytest.approx(-0.2)
    assert wz == pytest.approx(0.0)


def test_laser_pose_turn_and_wraparound():
    vx, vy, wz = body_velocity(
        (0.0, 0.0, math.pi - 0.05), (0.0, 0.0, -math.pi + 0.05), 0.1)
    assert wz == pytest.approx(1.0)
