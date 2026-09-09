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

"""Decide what each ToF zone is looking at: floor, obstacle, or a drop.

The whole design turns on one asymmetry. A zone that returns nothing means
opposite things depending on where it was pointed:

  - a ray angled at the floor, whose intercept is close enough to reach,
    returning nothing means the floor is not there any more. That is a cliff.
  - a ray pointed at open space returning nothing means open space.

Neither the range nor the point tells you which case you are in. Only the zone's
own direction does, which is why classification happens here, on the organised
grid, and not downstream in a costmap that has already thrown the grid away.

Most of the rest falls out of a single quantity: the height of the return above
the floor the robot is standing on.

    z_hit = aperture_height + range * direction.z

Above the floor by enough is an obstacle. Below it by enough is a drop. Near
zero is floor. Above the robot's own roof is something it drives under, which is
not an obstacle at all.

Height alone is not sufficient, and the reason is worth stating because it is
easy to miss. A ray angled downward can never report a height above the aperture
it left - 28.5 mm on this robot - so a low bar's vertical face always reads as
near-floor no matter how tall the bar is. On height alone, the four downward
rows could only ever say floor or cliff, and obstacles would be left to the
upward rows, which do not reach low objects until the robot is almost touching
them.

So a second test runs alongside: a ray that was going to meet the floor at a
known distance, and stopped measurably short of it, hit something standing in
between. That catches the vertical faces the height test cannot, and it is what
makes the downward rows useful for obstacles rather than only for floor.

This module holds no ROS. It takes geometry and ranges and returns labels, so
the interesting cases - a cliff at the edge of range, a shallow ray under pitch
- can be tested at a desk without a simulator.
"""

import math


FLOOR = 'floor'
OBSTACLE = 'obstacle'
CLIFF = 'cliff'
OVERHEAD = 'overhead'
FREE = 'free'


def rotate_pitch_roll(direction, pitch, roll):
    """Rotate a body-frame direction into the ground frame.

    Applied as roll about X then pitch about Y, which is the order that leaves
    yaw untouched - yaw is already carried by each sensor's fixed mount and
    does not affect height above the floor.

    Signs follow REP-103, so this is the standard rotation about +Y: positive
    pitch tips the nose DOWN. A robot climbing a ramp therefore reports negative
    pitch. Getting this backwards moves the floor the wrong way and turns every
    ramp into a wall of phantom cliffs, so it is asserted in the tests.
    """
    x, y, z = direction
    cos_roll, sin_roll = math.cos(roll), math.sin(roll)
    y, z = y * cos_roll - z * sin_roll, y * sin_roll + z * cos_roll
    cos_pitch, sin_pitch = math.cos(pitch), math.sin(pitch)
    x, z = x * cos_pitch + z * sin_pitch, -x * sin_pitch + z * cos_pitch
    return (x, y, z)


class FloorGeometry:
    """The heights and thresholds that separate floor from obstacle from drop.

    Defaults come from the measured robot rather than from taste:

    ``robot_height`` 0.0995 m is the top of the upper deck. A return above it is
    something the robot passes under - a table edge, a shelf - and marking it
    would refuse doorways the robot fits through.

    ``obstacle_min_height`` 0.025 m is set by what can be separated from the
    floor, not by what would collide. Ground clearance is only 9.5 mm, so
    strictly anything taller blocks the robot, but range noise projects into
    height as ``range_noise * |sin(pitch)|`` - 4.4 mm on the steepest row at the
    simulated 10 mm stddev. 25 mm is about six sigma there. Features shorter
    than this are genuinely below the noise floor, and the honest response is to
    say so rather than to lower the threshold and mark the floor.

    ``cliff_min_depth`` 0.030 m distinguishes a drop worth stopping for from a
    dip in the floor.

    ``range_shortfall`` 0.030 m is how much closer than the expected floor a
    return must be before it counts as something standing in the way. It has to
    clear range noise and the pitch error in the expected distance, and the
    shallow rows carry most of that error, so this is the loosest of the four.
    """

    def __init__(self, robot_height=0.0995, floor_tolerance=0.020,
                 obstacle_min_height=0.025, cliff_min_depth=0.030,
                 range_shortfall=0.030, max_range=3.5, min_range=0.02):
        self.robot_height = robot_height
        self.floor_tolerance = floor_tolerance
        self.obstacle_min_height = obstacle_min_height
        self.cliff_min_depth = cliff_min_depth
        self.range_shortfall = range_shortfall
        self.max_range = max_range
        self.min_range = min_range


class SensorPlacement:
    """One sensor's aperture height and its zone directions in the body frame.

    ``directions`` are the unit rays from ``zone_directions`` in
    ``tof_ray_markers``, already rotated by the sensor's fixed mount yaw, so
    this class never needs to know which face it belongs to.
    """

    def __init__(self, frame_id, height, directions):
        self.frame_id = frame_id
        self.height = height
        self.directions = directions


def expected_floor_range(height, direction_z, max_range):
    """Return the range at which a ray would meet a flat floor.

    ``None`` when the ray never gets there: pointed level or upward, or angled
    so shallowly that the floor lies beyond the sensor's reach. That second case
    is the one that matters, and it is why this is computed live from attitude
    rather than baked in per row. A few degrees of nose-up pitch walks the
    shallowest row's intercept out past the maximum range, and a ray that cannot
    reach the floor must never be read as evidence the floor is missing.
    """
    if direction_z >= 0.0:
        return None
    distance = height / -direction_z
    if distance > max_range:
        return None
    return distance


def classify_zone(geometry, height, direction, measured_range):
    """Label one zone. ``measured_range`` is None when the zone did not return.

    Returns ``(label, z_hit)``, where ``z_hit`` is the height of the return
    above the floor, or None for a zone that returned nothing.
    """
    expected = expected_floor_range(
        height, direction[2], geometry.max_range)

    if measured_range is None or not math.isfinite(measured_range):
        # The asymmetry this module exists for. A ray that was aimed at reachable
        # floor and saw nothing is a drop; a ray aimed anywhere else is space.
        return (CLIFF if expected is not None else FREE), None

    if measured_range <= geometry.min_range:
        return FREE, None

    z_hit = height + measured_range * direction[2]

    if z_hit > geometry.robot_height:
        return OVERHEAD, z_hit
    if z_hit > geometry.obstacle_min_height:
        return OBSTACLE, z_hit
    if (expected is not None
            and measured_range < expected - geometry.range_shortfall):
        # The ray was going to meet the floor and stopped short, so something
        # is standing in between. This is the only test that catches a low
        # obstacle's vertical face: a downward ray cannot report a height above
        # the aperture it left, so on height alone every such face reads as
        # floor however tall the object behind it is.
        return OBSTACLE, z_hit
    if z_hit < -geometry.cliff_min_depth:
        return CLIFF, z_hit
    # Anything left is floor, including returns between the floor band and the
    # obstacle threshold: real, but too small to separate from the floor, so
    # reported as floor rather than marked.
    return FLOOR, z_hit


def classify_sensor(geometry, placement, ranges, pitch=0.0, roll=0.0):
    """Classify every zone of one sensor.

    Yields ``(index, label, direction, z_hit)`` per zone, with the direction
    already tilted into the ground frame so callers can place the point.
    """
    for index, (direction, measured) in enumerate(
            zip(placement.directions, ranges)):
        tilted = rotate_pitch_roll(direction, pitch, roll)
        label, z_hit = classify_zone(
            geometry, placement.height, tilted, measured)
        yield index, label, tilted, z_hit


def floor_residual(geometry, height, direction, measured_range):
    """Return how far a return sits above the expected floor, or None.

    Positive is above the floor. Used for the plane quality estimate rather than
    for classification, which reads ``z_hit`` directly.
    """
    expected = expected_floor_range(height, direction[2], geometry.max_range)
    if expected is None or measured_range is None:
        return None
    if not math.isfinite(measured_range):
        return None
    return height + measured_range * direction[2]
