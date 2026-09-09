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

"""Pin the ToF zone grid to the order Gazebo actually fills the cloud in."""

import math

from mobile_base_tools.tof_ray_markers import scaled, zone_directions
import pytest


ZONES = 8
FOV = math.radians(60.0)
HALF_SPAN = FOV / 2.0 - (FOV / ZONES) / 2.0


def angles(direction):
    """Return (pitch, yaw) of a unit direction, in degrees."""
    return (
        math.degrees(math.asin(direction[2])),
        math.degrees(math.atan2(direction[1], direction[0])),
    )


def test_grid_is_square_and_unit_length():
    directions = zone_directions(ZONES, HALF_SPAN)
    assert len(directions) == ZONES * ZONES
    for direction in directions:
        assert math.isclose(math.dist((0.0, 0.0, 0.0), direction), 1.0,
                            abs_tol=1e-9)


def test_outermost_rays_sit_at_zone_centres():
    # Eight zones across 60 degrees puts the outermost zone centre at 26.25,
    # half a zone inside the field edge. Using the edge instead would splay
    # every ray outward and overstate the coverage by a zone's width.
    assert math.degrees(HALF_SPAN) == pytest.approx(26.25)
    directions = zone_directions(ZONES, HALF_SPAN)
    pitch, yaw = angles(directions[0])
    assert pitch == pytest.approx(-26.25)
    assert yaw == pytest.approx(-26.25)
    pitch, yaw = angles(directions[-1])
    assert pitch == pytest.approx(26.25)
    assert yaw == pytest.approx(26.25)


def test_row_zero_is_the_bottom_of_the_grid():
    """Row 0 points down, matching how Gazebo fills the cloud.

    This was measured off a running simulation rather than assumed: the ring
    field on each point equals its row index, and row 0 comes back at the
    minimum vertical angle. Inverting it mirrors the miss rays about the
    horizon, which draws a drop-off into the ceiling.
    """
    directions = zone_directions(ZONES, HALF_SPAN)
    rows = [directions[index * ZONES:(index + 1) * ZONES]
            for index in range(ZONES)]
    pitches = [angles(row[0])[0] for row in rows]
    assert pitches[0] == pytest.approx(-26.25)
    assert pitches[-1] == pytest.approx(26.25)
    assert pitches == sorted(pitches), 'rows must run bottom to top'
    # Pitch is constant across a row and yaw constant down a column, which is
    # the separable grid the sensor reports and Gazebo reproduces.
    for row in rows:
        assert len({round(angles(d)[0], 9) for d in row}) == 1
    for column in range(ZONES):
        yaws = {round(angles(rows[r][column])[1], 9) for r in range(ZONES)}
        assert len(yaws) == 1


def test_half_the_grid_looks_below_the_horizon():
    # A symmetric fan splits evenly, which is what puts the bottom rows on the
    # floor close in front of the robot.
    directions = zone_directions(ZONES, HALF_SPAN)
    assert sum(1 for d in directions if d[2] < 0.0) == ZONES * ZONES // 2


def test_scaled_walks_along_the_ray():
    point = scaled((1.0, 0.0, 0.0), 0.02)
    assert (point.x, point.y, point.z) == pytest.approx((0.02, 0.0, 0.0))


def test_single_zone_degenerates_to_one_forward_ray():
    assert zone_directions(1, HALF_SPAN) == [(1.0, 0.0, 0.0)]
