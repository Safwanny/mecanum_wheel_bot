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

"""Pin the floor/obstacle decision, and the margins that keep it quiet."""

import math

from mobile_base_tools.tof_floor_model import (
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
        # vertical face of something standing in the way. The shallow row needs
        # to fall a long way short before it is believed - that is the price of
        # not marking open floor, and it is paid deliberately.
        (-11.25, 0.09, OBSTACLE),
        (-3.75, 0.25, OBSTACLE),
        # High enough that the robot passes underneath it.
        (18.75, 0.50, OVERHEAD),
    ],
)
def test_returns_are_classified_by_height_above_the_floor(
        pitch_degrees, measured, expected_label):
    assert label_of(measured, pitch_degrees) == expected_label


def test_a_zone_that_saw_nothing_is_free():
    """With cliff detection gone, an empty zone carries no other meaning."""
    for pitch in (-26.25, -0.2, 3.75, 26.25):
        assert label_of(None, pitch) == FREE
    for value in (math.inf, math.nan):
        assert label_of(value, -26.25) == FREE


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


def test_the_shortfall_margin_scales_with_cotangent_of_elevation():
    """The single fix that removed 28 false obstacles per frame.

    A ray's predicted floor distance moves by cot(elevation) for every unit of
    error in the fitted plane: 2.0 on the steepest row, 15.3 on the shallowest.
    Measured on open floor with a flat threshold, that one shallow row produced
    all of the false marks, every one of them 0.5-0.9 m out and within 4 mm of
    the ground, while the steep rows were clean.
    """
    settings = geometry()
    steep = expected_floor_range(LEVEL, ORIGIN, ray(-26.25), 3.5)
    shallow = expected_floor_range(LEVEL, ORIGIN, ray(-3.75), 3.5)

    # The steep row keeps the floor margin; the shallow one needs far more.
    assert settings.shortfall_at(steep, ray(-26.25)[2]) == pytest.approx(0.030)
    shallow_margin = settings.shortfall_at(shallow, ray(-3.75)[2])
    assert shallow_margin > 4.0 * settings.range_shortfall

    # Plane error that used to mark open floor no longer does.
    assert label_of(shallow - 0.060, -3.75) == FLOOR
    # Something genuinely in the way still marks.
    assert label_of(shallow * 0.5, -3.75) == OBSTACLE
    # And the steep row keeps its millimetre sensitivity.
    assert label_of(steep - 0.040, -26.25) == OBSTACLE


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


def _grid(zones=8):
    return [(FREE, None, None)] * (zones * zones)


def test_marks_respect_the_range_of_each_kind():
    from mobile_base_tools.tof_floor_model import MarkRanges, select_marks
    geometry, limits, origin = FloorGeometry(), MarkRanges(), (0.0, 0.0, 0.0)
    results = _grid()
    # Low (caught by shortfall only) at 0.3 and 0.6 m; tall at 0.9 and 1.2 m.
    results[0] = (OBSTACLE, (0.3, 0.0, 0.01), 0.01)
    results[1] = (OBSTACLE, (0.6, 0.0, 0.01), 0.01)
    results[2] = (OBSTACLE, (0.9, 0.0, 0.05), 0.05)
    results[3] = (OBSTACLE, (1.2, 0.0, 0.05), 0.05)
    marked = [index for index, _ in select_marks(
        geometry, limits, origin, results, 8)]
    assert marked == [0, 2]


def test_a_column_at_one_distance_is_a_wall_out_to_two_metres():
    from mobile_base_tools.tof_floor_model import MarkRanges, select_marks
    geometry, limits, origin = FloorGeometry(), MarkRanges(), (0.0, 0.0, 0.0)
    results = _grid()
    column = 3
    for row in range(4, 8):
        results[row * 8 + column] = (
            OVERHEAD, (1.8, 0.0, 0.1 + 0.2 * row), 0.1 + 0.2 * row)
    marked = [index for index, _ in select_marks(
        geometry, limits, origin, results, 8)]
    assert marked == [row * 8 + column for row in range(4, 8)]


def test_an_overhang_is_not_a_wall():
    from mobile_base_tools.tof_floor_model import MarkRanges, select_marks
    geometry, limits, origin = FloorGeometry(), MarkRanges(), (0.0, 0.0, 0.0)
    results = _grid()
    # A table top: upper rows return, the lowest upward row sees past it.
    for row in range(5, 8):
        results[row * 8] = (OVERHEAD, (0.9, 0.0, 0.3), 0.3)
    assert select_marks(geometry, limits, origin, results, 8) == []
