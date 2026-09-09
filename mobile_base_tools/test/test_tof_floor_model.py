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
    FloorPlane,
    classify_zone,
    expected_floor_range,
    fit_plane,
    floor_candidates,
)
import pytest


# Measured off the built robot: the front aperture 130 mm forward and 28.5 mm
# above the ground, zone centres every 7.5 degrees out to 26.25 - half a zone
# inside the 30 degree field edge.
ORIGIN = (0.130, 0.0, 0.028544)
ROWS = (-26.25, -18.75, -11.25, -3.75, 3.75, 11.25, 18.75, 26.25)
LEVEL = FloorPlane()


def ray(pitch_degrees):
    """A unit direction in the vertical plane, pitch measured from level."""
    angle = math.radians(pitch_degrees)
    return (math.cos(angle), 0.0, math.sin(angle))


def geometry(**overrides):
    return FloorGeometry(**overrides)


def label_of(measured, pitch_degrees=-11.25, plane=LEVEL, **settings):
    return classify_zone(
        geometry(**settings), plane, ORIGIN, ray(pitch_degrees), measured)[0]


def test_downward_rows_reach_the_floor_where_measured():
    """The four downward rows land where the simulation put them."""
    for pitch, want in zip(ROWS[:4], (0.0639, 0.0888, 0.1481, 0.4363)):
        got = expected_floor_range(LEVEL, ORIGIN, ray(pitch), 3.5)
        assert got == pytest.approx(want, abs=2e-3), pitch


def test_upward_rows_never_reach_the_floor():
    for pitch in ROWS[4:]:
        assert expected_floor_range(LEVEL, ORIGIN, ray(pitch), 3.5) is None


def test_a_ray_too_shallow_to_reach_the_floor_reports_none():
    # The floor is 28.5 mm down; at a fifth of a degree it is 8 m away, past
    # any range this sensor has. Such a ray carries no floor information.
    assert expected_floor_range(LEVEL, ORIGIN, ray(-0.2), 3.5) is None
    assert expected_floor_range(LEVEL, ORIGIN, ray(-0.2), 100.0) is not None


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
    assert label_of(measured, pitch_degrees) == expected_label


def test_missing_return_means_cliff_only_where_floor_was_expected():
    """The asymmetry the whole module exists for.

    The same absence of data is a drop from a ray aimed at reachable floor and
    open space from any other ray. Getting this backwards either blinds the
    robot to stairs or paints every doorway as a hole.
    """
    assert label_of(None, -26.25) == CLIFF
    assert label_of(None, 3.75) == FREE
    assert label_of(None, 26.25) == FREE


def test_a_cliff_with_no_return_is_marked_where_the_floor_was_expected():
    """A zone that saw nothing has no point of its own, so use the prediction.

    This is the commonest cliff by far - beyond an edge the beam usually comes
    back with nothing at all - so a classifier that could only place measured
    points would place almost no cliffs.
    """
    label, point, _ = classify_zone(
        geometry(), LEVEL, ORIGIN, ray(-26.25), None)
    assert label == CLIFF
    assert point[0] == pytest.approx(0.130 + 0.0573, abs=2e-3)
    assert point[2] == pytest.approx(0.0, abs=1e-9)


def test_a_cliff_is_marked_at_the_edge_not_where_the_beam_landed():
    """Over a drop the ray carries on and hits the lower ground far away.

    Measured on the proving ground: the shallow row leaves the deck at 436 mm
    and strikes the floor 120 mm below it at 2.27 m. Marking that point puts
    the lethal cell past the drop and leaves the edge itself clear, and a range
    gate then throws the whole detection away.
    """
    edge = expected_floor_range(LEVEL, ORIGIN, ray(-3.75), 3.5)
    label, point, _ = classify_zone(
        geometry(), LEVEL, ORIGIN, ray(-3.75), 2.27)
    assert label == CLIFF
    marked = math.dist(ORIGIN, point)
    assert marked == pytest.approx(edge, abs=1e-6)
    assert marked < 0.8, 'the mark must survive the costmap range gate'


def test_missing_return_on_an_unreachable_floor_ray_is_free_not_cliff():
    """A ray angled so shallowly the floor is out of range proves nothing."""
    assert label_of(None, -0.2) == FREE


def test_non_finite_range_is_treated_as_no_return():
    for value in (math.inf, math.nan):
        assert label_of(value, -26.25) == CLIFF


def test_a_downward_ray_can_never_report_a_height_above_its_aperture():
    """The geometric fact that forces the range-shortfall rule to exist.

    Every downward ray starts at the aperture and only descends, so height
    alone can never see a low obstacle's face. Without the shortfall test the
    four downward rows would be blind to obstacles entirely.
    """
    for distance in (0.05, 0.10, 0.20, 0.40):
        _, _, residual = classify_zone(
            geometry(range_shortfall=99.0), LEVEL, ORIGIN,
            ray(-11.25), distance)
        assert residual <= ORIGIN[2] + 1e-9


def test_a_low_feature_near_its_floor_intercept_stays_unmarked():
    """A shallow bump that neither rises nor shortens enough reads as floor."""
    settings = geometry()
    expected = expected_floor_range(LEVEL, ORIGIN, ray(-11.25), 3.5)
    _, _, residual = classify_zone(
        settings, LEVEL, ORIGIN, ray(-11.25),
        expected - 0.5 * settings.range_shortfall)
    assert 0.0 < residual < settings.obstacle_min_height
    assert label_of(expected - 0.5 * settings.range_shortfall) == FLOOR


def test_the_shortfall_threshold_grows_with_the_expected_distance():
    """A fixed threshold is too tight on the far row, where error is largest.

    Measured on flat deck, a fixed 30 mm let range noise on the 436 mm row
    scatter stray obstacle marks across open floor.
    """
    settings = geometry()
    near = expected_floor_range(LEVEL, ORIGIN, ray(-26.25), 3.5)
    far = expected_floor_range(LEVEL, ORIGIN, ray(-3.75), 3.5)
    assert settings.shortfall_at(near) == pytest.approx(0.030)
    assert settings.shortfall_at(far) > 0.040
    # Three sigma of range noise on the far row is no longer enough to mark.
    assert label_of(far - 0.030, -3.75) == FLOOR
    # A real obstacle standing in the way still is.
    assert label_of(far * 0.5, -3.75) == OBSTACLE


def test_a_shallow_dip_is_not_a_cliff_but_a_real_drop_is():
    # 0.087 m puts the return 10 mm below the floor, inside the tolerance;
    # 0.330 m puts it 117 mm down, which is a step the robot would fall off.
    assert label_of(0.087, -26.25) == FLOOR
    assert label_of(0.330, -26.25) == CLIFF


def test_returns_inside_the_dead_zone_are_discarded():
    assert label_of(0.01, -26.25) == FREE


# --- the fitted floor -------------------------------------------------------


def flat_points(height=0.0, slope=0.0, count=24):
    """Floor samples on a plane tilted by ``slope`` about the y axis."""
    points = []
    for index in range(count):
        x = 0.15 + 0.02 * index
        y = 0.05 * ((index % 5) - 2)
        points.append((x, y, height + slope * x))
    return points


def test_a_level_floor_fits_to_a_level_plane():
    plane = fit_plane(flat_points())
    assert plane.a == pytest.approx(0.0, abs=1e-9)
    assert plane.b == pytest.approx(0.0, abs=1e-9)
    assert plane.c == pytest.approx(0.0, abs=1e-9)


def test_a_ramp_under_the_robot_is_not_a_tilted_plane_in_its_own_frame():
    """The reason the floor is fitted rather than taken from an IMU.

    A robot on a steady ramp is tilted, and so is the ramp, so the surface it
    stands on has not moved in its own frame. An IMU would report the ramp
    angle and subtracting it would tilt the model away from a floor that is
    exactly where it always was, painting the whole ramp as a drop-off.
    """
    plane = fit_plane(flat_points())
    for pitch in ROWS[:3]:
        distance = expected_floor_range(plane, ORIGIN, ray(pitch), 3.5)
        assert classify_zone(
            geometry(), plane, ORIGIN, ray(pitch), distance)[0] == FLOOR


def test_a_floor_that_is_genuinely_sloped_relative_to_the_robot_is_fitted():
    plane = fit_plane(flat_points(slope=0.10))
    assert plane.a == pytest.approx(0.10, abs=1e-6)
    assert plane.tilt() == pytest.approx(math.atan(0.10), abs=1e-6)


def test_a_floor_offset_downward_shifts_the_plane_not_the_labels():
    """A floor 40 mm lower than nominal is still floor once fitted."""
    lowered = flat_points(height=-0.040)
    # Against the nominal level plane this reads as a 40 mm drop.
    assert classify_zone(
        geometry(), LEVEL, ORIGIN, ray(-26.25),
        (ORIGIN[2] + 0.040) / abs(ray(-26.25)[2]))[0] == CLIFF
    plane = fit_plane(lowered)
    assert plane.c == pytest.approx(-0.040, abs=1e-9)
    distance = expected_floor_range(plane, ORIGIN, ray(-26.25), 3.5)
    assert classify_zone(
        geometry(), plane, ORIGIN, ray(-26.25), distance)[0] == FLOOR


def test_an_implausibly_steep_fit_falls_back_to_level():
    """Refusing a bad fit is the safe failure.

    A plane dragged onto the face of an obstacle would re-label the real floor
    around it as a drop, so a fit that comes out steeper than any floor the
    robot could climb is discarded rather than trusted.
    """
    wall = [(0.15 + 0.001 * i, 0.0, 0.20 * i / 24.0) for i in range(24)]
    plane = fit_plane(wall)
    assert plane.a == 0.0 and plane.b == 0.0 and plane.c == 0.0


def test_too_few_samples_falls_back_to_level():
    plane = fit_plane([(0.2, 0.0, 0.0), (0.3, 0.0, 0.0)])
    assert plane.a == 0.0 and plane.b == 0.0 and plane.c == 0.0
    assert plane.samples == 2


def test_outliers_are_rejected_before_the_final_fit():
    points = flat_points()
    points[3] = (points[3][0], points[3][1], 0.25)
    points[9] = (points[9][0], points[9][1], -0.25)
    plane = fit_plane(points)
    assert plane.c == pytest.approx(0.0, abs=2e-3)
    assert plane.samples < len(points)


def test_only_the_steep_near_rows_feed_the_fit():
    """Row 3 shifts 116 mm per degree and must not carry the floor estimate."""
    directions = [ray(pitch) for pitch in ROWS]
    ranges = [
        expected_floor_range(LEVEL, ORIGIN, direction, 3.5)
        for direction in directions
    ]
    points = floor_candidates(
        ORIGIN, directions, ranges, rows={0, 1, 2}, zones=1, max_range=3.5)
    assert len(points) == 3
    assert max(point[0] for point in points) < 0.30


def test_floor_candidates_skip_missing_and_upward_rays():
    directions = [ray(pitch) for pitch in ROWS]
    ranges = [None] * 4 + [1.0] * 4
    points = floor_candidates(
        ORIGIN, directions, ranges, rows=set(range(8)), zones=1,
        max_range=3.5)
    assert points == []
