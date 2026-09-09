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

"""Pin the floor/obstacle/cliff decision, especially where it is asymmetric."""

import math

from mobile_base_tools.tof_floor_model import (
    CLIFF,
    FLOOR,
    FREE,
    OBSTACLE,
    OVERHEAD,
    FloorGeometry,
    classify_zone,
    expected_floor_range,
    rotate_pitch_roll,
)
import pytest


# Measured off the built robot: aperture 28.5 mm up, zone centres every 7.5
# degrees out to 26.25, the outermost row half a zone inside the 30 degree
# field edge.
HEIGHT = 0.028544
ROWS = (-26.25, -18.75, -11.25, -3.75, 3.75, 11.25, 18.75, 26.25)


def ray(pitch_degrees):
    """A unit direction in the vertical plane, pitch measured from level."""
    angle = math.radians(pitch_degrees)
    return (math.cos(angle), 0.0, math.sin(angle))


def geometry(**overrides):
    return FloorGeometry(**overrides)


def test_downward_rows_reach_the_floor_where_measured():
    """The four downward rows land where the simulation put them."""
    expected = (0.0639, 0.0888, 0.1481, 0.4363)
    for pitch, want in zip(ROWS[:4], expected):
        got = expected_floor_range(HEIGHT, ray(pitch)[2], 3.5)
        assert got == pytest.approx(want, abs=2e-3), pitch


def test_upward_rows_never_reach_the_floor():
    for pitch in ROWS[4:]:
        assert expected_floor_range(HEIGHT, ray(pitch)[2], 3.5) is None


def test_a_ray_too_shallow_to_reach_the_floor_reports_none():
    # The floor is 28.5 mm down; at a fifth of a degree it is 8 m away, past
    # any range this sensor has. Such a ray carries no floor information.
    assert expected_floor_range(HEIGHT, ray(-0.2)[2], 3.5) is None
    assert expected_floor_range(HEIGHT, ray(-0.2)[2], 100.0) is not None


@pytest.mark.parametrize(
    ('pitch_degrees', 'measured', 'expected_label'),
    [
        # A flat floor, read by each downward row, is floor.
        (-26.25, 0.0639, FLOOR),
        (-11.25, 0.1481, FLOOR),
        (-3.75, 0.4363, FLOOR),
        # An upward ray meeting something tall enough to block the robot.
        (3.75, 0.50, OBSTACLE),
        (11.25, 0.30, OBSTACLE),
        # A downward ray stopping short of its floor intercept: it met the
        # vertical face of something standing in the way.
        (-11.25, 0.09, OBSTACLE),
        (-3.75, 0.30, OBSTACLE),
        # High enough that the robot passes underneath it.
        (18.75, 0.50, OVERHEAD),
        # The floor has fallen away: a downward ray reaching much further.
        (-26.25, 0.40, CLIFF),
        (-11.25, 1.00, CLIFF),
    ],
)
def test_returns_are_classified_by_height_above_the_floor(
        pitch_degrees, measured, expected_label):
    label, _ = classify_zone(
        geometry(), HEIGHT, ray(pitch_degrees), measured)
    assert label == expected_label


def test_missing_return_means_cliff_only_where_floor_was_expected():
    """The asymmetry the whole module exists for.

    The same absence of data is a drop from a ray aimed at reachable floor and
    open space from any other ray. Getting this backwards either blinds the
    robot to stairs or paints every doorway as a hole.
    """
    down, _ = classify_zone(geometry(), HEIGHT, ray(-26.25), None)
    assert down == CLIFF

    level, _ = classify_zone(geometry(), HEIGHT, ray(3.75), None)
    assert level == FREE

    up, _ = classify_zone(geometry(), HEIGHT, ray(26.25), None)
    assert up == FREE


def test_missing_return_on_an_unreachable_floor_ray_is_free_not_cliff():
    """A ray angled so shallowly the floor is out of range proves nothing.

    Reading it as a cliff would stop the robot dead every time it pitched
    nose-up, which is exactly what happens under acceleration and on a ramp.
    """
    label, _ = classify_zone(geometry(), HEIGHT, ray(-0.2), None)
    assert label == FREE


def test_non_finite_range_is_treated_as_no_return():
    for value in (math.inf, math.nan):
        label, _ = classify_zone(geometry(), HEIGHT, ray(-26.25), value)
        assert label == CLIFF


def test_a_downward_ray_can_never_report_a_height_above_its_aperture():
    """The geometric fact that forces the range-shortfall rule to exist.

    Every downward ray starts at the aperture and only descends, so height alone
    can never see a low obstacle's face. Without the shortfall test the four
    downward rows would be blind to obstacles entirely.
    """
    for distance in (0.05, 0.10, 0.20, 0.40):
        _, z_hit = classify_zone(
            geometry(range_shortfall=99.0), HEIGHT, ray(-11.25), distance)
        assert z_hit <= HEIGHT + 1e-9


def test_a_low_feature_near_its_floor_intercept_stays_unmarked():
    """A shallow bump that neither rises nor shortens enough reads as floor.

    Range noise projects into height as range_noise * |sin(pitch)|, 4.4 mm on
    the steepest row at the simulated 10 mm stddev. Marking at that scale would
    mean marking the floor itself.
    """
    settings = geometry()
    expected = expected_floor_range(HEIGHT, ray(-11.25)[2], 3.5)
    barely_short = expected - 0.5 * settings.range_shortfall
    label, z_hit = classify_zone(settings, HEIGHT, ray(-11.25), barely_short)
    assert 0.0 < z_hit < settings.obstacle_min_height
    assert label == FLOOR


def test_an_upward_ray_reports_true_obstacle_height():
    settings = geometry()
    distance = (0.045 - HEIGHT) / ray(3.75)[2]
    label, z_hit = classify_zone(settings, HEIGHT, ray(3.75), distance)
    assert z_hit == pytest.approx(0.045, abs=1e-6)
    assert label == OBSTACLE


def test_a_shallow_dip_is_not_a_cliff_but_a_real_drop_is():
    settings = geometry()
    dip = (HEIGHT + 0.010) / abs(ray(-26.25)[2])
    assert classify_zone(settings, HEIGHT, ray(-26.25), dip)[0] == FLOOR
    drop = (HEIGHT + 0.120) / abs(ray(-26.25)[2])
    assert classify_zone(settings, HEIGHT, ray(-26.25), drop)[0] == CLIFF


def test_returns_inside_the_dead_zone_are_discarded():
    label, _ = classify_zone(geometry(), HEIGHT, ray(-26.25), 0.01)
    assert label == FREE


def test_positive_pitch_is_nose_down():
    """REP-103 sign convention, asserted because reversing it inverts ramps."""
    assert rotate_pitch_roll((1.0, 0.0, 0.0), math.radians(10.0), 0.0)[2] < 0.0
    assert rotate_pitch_roll((1.0, 0.0, 0.0), math.radians(-10.0), 0.0)[2] > 0.0


def test_pitch_moves_the_shallow_row_far_more_than_the_steep_one():
    """The sensitivity spread that decides which rows can be trusted.

    Row 0 moves about 2.5 mm per degree of pitch and row 3 about 116, which is
    why the shallow row is admitted only as coarse warning and never as a height
    reference. Measured over a tenth of a degree and scaled, because 1/tan is
    steep enough that a whole-degree step is not the derivative.
    """
    step = 0.1

    def sensitivity(pitch_degrees):
        def intercept(tilt):
            tilted = rotate_pitch_roll(
                ray(pitch_degrees), math.radians(tilt), 0.0)
            distance = expected_floor_range(HEIGHT, tilted[2], 3.5)
            return distance * tilted[0]
        return abs(intercept(step) - intercept(0.0)) / step

    steep = sensitivity(-26.25)
    shallow = sensitivity(-3.75)
    assert steep == pytest.approx(0.0025, abs=3e-4)
    assert shallow == pytest.approx(0.1165, abs=6e-3)
    assert shallow > 40.0 * steep


def test_nose_up_pitch_pushes_the_shallow_row_out_of_reach():
    """On a ramp the shallowest row stops seeing floor entirely.

    It must degrade to FREE rather than to CLIFF, or every ramp the robot
    climbs reads as a drop-off and it refuses to move.
    """
    tilted = rotate_pitch_roll(ray(-3.75), math.radians(-4.0), 0.0)
    assert expected_floor_range(HEIGHT, tilted[2], 3.5) is None
    assert classify_zone(geometry(), HEIGHT, tilted, None)[0] == FREE


def test_rotation_preserves_length_and_leaves_level_rays_level():
    for pitch in ROWS:
        rotated = rotate_pitch_roll(ray(pitch), 0.2, -0.1)
        assert math.dist((0.0, 0.0, 0.0), rotated) == pytest.approx(1.0)
    forward = rotate_pitch_roll((1.0, 0.0, 0.0), 0.0, 0.0)
    assert forward == pytest.approx((1.0, 0.0, 0.0))
