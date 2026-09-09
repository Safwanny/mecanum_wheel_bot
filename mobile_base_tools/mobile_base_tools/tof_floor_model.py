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

The floor those heights are measured from is fitted from the sensors, not
assumed level and not taken from the IMU. That choice matters more than it
looks. An IMU reports attitude against gravity, but what decides whether a
return is floor is attitude against *the floor*, and the two disagree exactly
where it counts: on a steady ramp the robot is tilted and the ramp is tilted
with it, so in the robot's own frame nothing has moved at all. Subtracting IMU
pitch there would tilt the model away from a surface that never went anywhere
and paint the whole ramp as a drop-off.

Fitting instead from the steep near rows - the ones that reach the floor within
150 mm and move only 2.5 to 13 mm per degree - gives a reference that is right
on a ramp, right under acceleration, and right on a floor that is simply not
where the model said it was. It is also self-correcting: the measurement that
defines the floor is the same measurement being classified against it.

This module holds no ROS. It takes geometry and ranges and returns labels, so
the interesting cases - a cliff at the edge of range, a ramp, a floor that is
not level - can be tested at a desk without a simulator.
"""

import math


FLOOR = 'floor'
OBSTACLE = 'obstacle'
CLIFF = 'cliff'
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

    ``cliff_min_depth`` 0.030 m distinguishes a drop worth stopping for from a
    dip in the floor.

    ``range_shortfall`` 0.030 m is how much closer than the expected floor a
    return must be before it counts as something standing in the way, and
    ``shortfall_fraction`` 0.10 adds a term proportional to how far away that
    floor was. The proportional part is not decoration: a fixed threshold is
    three sigma of range noise against the near rows' 60-150 mm intercepts but
    only marginal against the shallow row's 436 mm, where the pitch error is
    also largest. Measured on flat deck, a fixed 30 mm left a scatter of stray
    marks from the far row alone.
    """

    def __init__(self, robot_height=0.0995, floor_tolerance=0.020,
                 obstacle_min_height=0.025, cliff_min_depth=0.030,
                 range_shortfall=0.030, shortfall_fraction=0.10,
                 max_range=3.5, min_range=0.02):
        self.robot_height = robot_height
        self.floor_tolerance = floor_tolerance
        self.obstacle_min_height = obstacle_min_height
        self.cliff_min_depth = cliff_min_depth
        self.range_shortfall = range_shortfall
        self.shortfall_fraction = shortfall_fraction
        self.max_range = max_range
        self.min_range = min_range

    def shortfall_at(self, expected):
        """How much short of ``expected`` counts as something in the way."""
        return max(
            self.range_shortfall, self.shortfall_fraction * expected)


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
        # The asymmetry this module exists for. A ray aimed at reachable floor
        # that saw nothing is a drop; a ray aimed anywhere else is open space.
        if expected is None:
            return FREE, None, None
        return CLIFF, along(expected), None

    if measured_range <= geometry.min_range:
        return FREE, None, None

    point = along(measured_range)
    residual = plane.residual(point)

    if residual > geometry.robot_height:
        return OVERHEAD, point, residual
    if residual > geometry.obstacle_min_height:
        return OBSTACLE, point, residual
    if (expected is not None
            and measured_range < expected - geometry.shortfall_at(expected)):
        # The ray was going to meet the floor and stopped short, so something
        # is standing in between. This is the only test that catches a low
        # obstacle's vertical face: a downward ray cannot report a height above
        # the aperture it left, so on height alone every such face reads as
        # floor however tall the object behind it is.
        return OBSTACLE, point, residual
    if residual < -geometry.cliff_min_depth:
        # Marked where the floor should have been, not where the beam finally
        # landed. Over an edge the ray carries on and can strike the lower
        # ground metres away; that point is past the drop and marking it would
        # put the lethal cell somewhere the robot was never going to be, while
        # leaving the edge itself clear.
        return CLIFF, (along(expected) if expected is not None else point), \
            residual
    # Anything left is floor, including returns between the floor band and the
    # obstacle threshold: real, but too small to separate from the floor, so
    # reported as floor rather than marked.
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
