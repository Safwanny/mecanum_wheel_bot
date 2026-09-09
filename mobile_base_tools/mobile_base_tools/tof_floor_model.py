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

"""Decide whether each ToF zone is looking at the floor or at an obstacle.

The ring exists to see the band the LiDAR cannot: the scan plane sits at 91.5 mm
and ground clearance is 9.5 mm, so anything between them stops the robot while
being wholly invisible to it. A shoe, a cable, a doorsill.

Two tests decide it, and both are needed.

**Height above the floor.** ``z_hit = origin.z + range * direction.z``, measured
against a plane fitted from the sensors themselves rather than assumed level.
Above a threshold is an obstacle; above the robot's own roof is clearance it
drives under.

**Range shortfall.** A ray that was going to meet the floor and stopped short of
it hit something standing in between. This is the only test that catches a low
obstacle's vertical face, because a downward ray can never report a height above
the aperture it left - on height alone the downward rows would be blind to
obstacles entirely.

The shortfall threshold scales with ``cot(elevation)``, and that is the whole
difference between this working and not. The predicted floor distance for a ray
at elevation ``theta`` moves by ``cot(theta)`` for every unit of error in the
fitted plane: 2.0 for the steepest row, but 15.3 for the shallowest. Measured on
open floor with a fixed threshold, that one row produced 28 false obstacles per
frame, all of them at 0.5-0.9 m and all sitting within 4 mm of the ground. Every
other row was clean. Scaling by cot means the shallow row must miss its
prediction by a wide margin before anything is marked, while the steep rows keep
their millimetre sensitivity.

The floor is fitted, not assumed. On a slope the robot and the surface tilt
together, so in the robot's own frame the floor has not moved; a gravity
referenced attitude would report a slope that is not there.

This module holds no ROS, so the interesting cases can be tested at a desk.
"""

import math


FLOOR = 'floor'
OBSTACLE = 'obstacle'
OVERHEAD = 'overhead'
FREE = 'free'


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

    ``range_shortfall`` 0.030 m is the floor on how much closer than the
    predicted floor a return must be to count as an obstacle, and
    ``plane_angle_tolerance`` 1.2 degrees is what that grows by on shallow rays.
    The predicted distance moves by cot(elevation) per unit of plane error, so
    the shallowest row needs a margin an order of magnitude wider than the
    steepest before anything it reports can be believed.
    """

    def __init__(self, robot_height=0.0995, floor_tolerance=0.020,
                 obstacle_min_height=0.025, range_shortfall=0.030,
                 plane_angle_tolerance=math.radians(1.2),
                 max_range=3.5, min_range=0.02):
        self.robot_height = robot_height
        self.floor_tolerance = floor_tolerance
        self.obstacle_min_height = obstacle_min_height
        self.range_shortfall = range_shortfall
        self.plane_angle_tolerance = plane_angle_tolerance
        self.max_range = max_range
        self.min_range = min_range

    def shortfall_at(self, expected, direction_z):
        """How much short of ``expected`` counts as something in the way.

        Grows with cot(elevation): a shallow ray's predicted floor distance is
        far more sensitive to error in the fitted plane than a steep one's, so
        it has to miss by much more before it is believed.
        """
        sine = min(1.0, max(1e-6, abs(direction_z)))
        cotangent = math.sqrt(max(0.0, 1.0 - sine * sine)) / sine
        return max(
            self.range_shortfall,
            expected * cotangent * self.plane_angle_tolerance)


class FloorPlane:
    """The floor as the sensors currently see it: ``z = a*x + b*y + c``.

    Held in the body frame, so a level floor under a level robot is
    ``a = b = c = 0`` and a robot climbing a ramp still measures ``a = b = 0``
    - the ramp is tilted, but so is the robot, and in its own frame the surface
    it is standing on has not moved. That is the whole reason this is fitted
    rather than derived from an IMU.
    """

    def __init__(self, a=0.0, b=0.0, c=0.0, samples=0):
        self.a = a
        self.b = b
        self.c = c
        self.samples = samples

    def height_at(self, x, y):
        """Floor height under a body-frame point."""
        return self.a * x + self.b * y + self.c

    def residual(self, point):
        """How far a body-frame point sits above the floor."""
        return point[2] - self.height_at(point[0], point[1])

    def tilt(self):
        """Slope magnitude, radians. Large values mean a bad or partial fit."""
        return math.atan(math.hypot(self.a, self.b))


def fit_plane(points, max_tilt=math.radians(20.0), rejection=0.020):
    """Least-squares plane through floor candidates, with one rejection pass.

    Returns the level plane when there is too little to fit or the fit comes
    out implausibly steep. Refusing a bad fit matters: a plane dragged onto the
    face of an obstacle would re-label the real floor around it as a drop, so
    the safe failure is to fall back to level rather than to trust three points
    on a wall.
    """
    if len(points) < 6:
        return FloorPlane(samples=len(points))

    def solve(sample):
        n = len(sample)
        sx = sy = sz = sxx = sxy = syy = sxz = syz = 0.0
        for x, y, z in sample:
            sx += x
            sy += y
            sz += z
            sxx += x * x
            sxy += x * y
            syy += y * y
            sxz += x * z
            syz += y * z
        matrix = (
            (sxx, sxy, sx),
            (sxy, syy, sy),
            (sx, sy, float(n)),
        )
        determinant = (
            matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
            - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
            + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
        )
        if abs(determinant) < 1e-12:
            return None
        rhs = (sxz, syz, sz)

        def replaced(column):
            copy = [list(row) for row in matrix]
            for row in range(3):
                copy[row][column] = rhs[row]
            return (
                copy[0][0] * (copy[1][1] * copy[2][2] - copy[1][2] * copy[2][1])
                - copy[0][1] * (copy[1][0] * copy[2][2] - copy[1][2] * copy[2][0])
                + copy[0][2] * (copy[1][0] * copy[2][1] - copy[1][1] * copy[2][0])
            )

        return (
            replaced(0) / determinant,
            replaced(1) / determinant,
            replaced(2) / determinant,
        )

    first = solve(points)
    if first is None:
        return FloorPlane(samples=len(points))
    plane = FloorPlane(*first, samples=len(points))

    kept = [
        point for point in points
        if abs(plane.residual(point)) <= rejection
    ]
    if len(kept) >= 6 and len(kept) < len(points):
        second = solve(kept)
        if second is not None:
            plane = FloorPlane(*second, samples=len(kept))

    if plane.tilt() > max_tilt:
        return FloorPlane(samples=len(points))
    return plane


def expected_floor_range(plane, origin, direction, max_range):
    """Range at which a ray leaving ``origin`` would meet ``plane``.

    ``None`` when it never gets there - aimed level or upward relative to the
    surface, or angled so shallowly the floor lies beyond reach. That second
    case is not a detail. A ray that cannot reach the floor must never be read
    as evidence the floor is missing, or the robot stops dead every time the
    geometry tips a shallow row out of range.
    """
    denominator = (
        direction[2] - plane.a * direction[0] - plane.b * direction[1])
    if denominator >= 0.0:
        return None
    numerator = (
        plane.a * origin[0] + plane.b * origin[1] + plane.c - origin[2])
    distance = numerator / denominator
    if distance <= 0.0 or distance > max_range:
        return None
    return distance


def classify_zone(geometry, plane, origin, direction, measured_range):
    """Label one zone. ``measured_range`` is None when the zone did not return.

    Returns ``(label, point, residual)``. ``point`` is where the return landed
    in the body frame, or where the floor was expected for a zone that saw
    nothing; ``residual`` is its height above the fitted floor.
    """
    expected = expected_floor_range(
        plane, origin, direction, geometry.max_range)

    def along(distance):
        return (
            origin[0] + distance * direction[0],
            origin[1] + distance * direction[1],
            origin[2] + distance * direction[2],
        )

    if measured_range is None or not math.isfinite(measured_range):
        # Nothing came back. Without cliff detection there is nothing to
        # infer from that: an empty zone is simply empty.
        return FREE, None, None

    if measured_range <= geometry.min_range:
        return FREE, None, None

    point = along(measured_range)
    residual = plane.residual(point)

    if residual > geometry.robot_height:
        return OVERHEAD, point, residual
    if residual > geometry.obstacle_min_height:
        return OBSTACLE, point, residual
    if (expected is not None and measured_range
            < expected - geometry.shortfall_at(expected, direction[2])):
        # The ray was going to meet the floor and stopped short, so something
        # is standing in between. This is the only test that catches a low
        # obstacle's vertical face: a downward ray cannot report a height above
        # the aperture it left, so on height alone every such face reads as
        # floor however tall the object behind it is.
        return OBSTACLE, point, residual
    # Anything left is floor, including returns below the floor and those
    # between the floor band and the obstacle threshold: real, but too small to
    # separate from the floor, so reported as floor rather than marked.
    return FLOOR, point, residual


def floor_candidates(origin, directions, ranges, rows, zones, max_range):
    """Body-frame points from the steep near rows, for fitting the floor.

    Only the rows listed in ``rows`` contribute. They reach the floor within
    150 mm and shift 2.5 to 13 mm per degree of attitude error, where the
    shallowest row shifts 116 - fitting on that one would hand the noisiest
    sample the longest lever arm.
    """
    points = []
    for index, (direction, measured) in enumerate(zip(directions, ranges)):
        if index // zones not in rows:
            continue
        if measured is None or not math.isfinite(measured):
            continue
        if measured <= 0.0 or measured > max_range:
            continue
        if direction[2] >= 0.0:
            continue
        points.append((
            origin[0] + measured * direction[0],
            origin[1] + measured * direction[1],
            origin[2] + measured * direction[2],
        ))
    return points
