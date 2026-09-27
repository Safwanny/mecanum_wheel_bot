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

"""Pin how classified ToF points become walls, objects and floor patches."""

import math
import random

import pytest

from mobile_base_tools.tof_shape_model import (
    OBJECT,
    WALL,
    Shape,
    ShapeTracker,
    cluster_points,
    fill_profile,
    fit_shape,
    fit_shapes,
    floor_profile,
    merge_walls,
    split_segments,
)

ZONE_ANGLE = math.radians(7.5)


def wall_points(distance=0.5, half_length=0.2, count=9, z=0.05):
    """A wall across the front at x = distance, as zones would sample it."""
    return [
        (distance, -half_length + 2 * half_length * i / (count - 1), z)
        for i in range(count)
    ]


def blob(x, y, radius=0.02, z=0.04):
    return [(x + dx, y + dy, z) for dx, dy in
            ((0, 0), (radius, 0), (0, radius), (-radius, 0), (0, -radius))]


def test_line_of_points_becomes_one_wall_facing_the_robot():
    clusters = cluster_points(wall_points(), ZONE_ANGLE)
    assert len(clusters) == 1
    shape = fit_shape(clusters[0])
    assert shape.kind == WALL
    assert shape.x == pytest.approx(0.5, abs=1e-6)
    assert shape.length == pytest.approx(0.4, abs=1e-6)
    # Runs along y: yaw is +-90 degrees.
    assert abs(math.cos(shape.yaw)) == pytest.approx(0.0, abs=1e-6)
    assert shape.distance() == pytest.approx(0.49, abs=1e-6)
    assert shape.z_max == pytest.approx(0.05)


def test_compact_blob_becomes_an_object_enclosing_its_points():
    points = blob(0.3, 0.1)
    shape = fit_shape(cluster_points(points, ZONE_ANGLE)[0])
    assert shape.kind == OBJECT
    cos_yaw, sin_yaw = math.cos(shape.yaw), math.sin(shape.yaw)
    for px, py, _ in points:
        along = (px - shape.x) * cos_yaw + (py - shape.y) * sin_yaw
        across = -(px - shape.x) * sin_yaw + (py - shape.y) * cos_yaw
        assert abs(along) <= shape.length / 2.0 + 1e-9
        assert abs(across) <= shape.width / 2.0 + 1e-9


def test_separate_objects_stay_separate_and_lone_points_are_dropped():
    points = blob(0.3, 0.15) + blob(0.3, -0.15) + [(0.6, 0.6, 0.05)]
    clusters = cluster_points(points, ZONE_ANGLE)
    assert sorted(len(c) for c in clusters) == [5, 5]


def test_link_distance_grows_with_range():
    # Zones 7.5 degrees apart at 0.75 m sit ~0.1 m apart: beyond the 0.05 m
    # floor on the link, but still one surface.
    step = 0.75 * ZONE_ANGLE
    points = [(0.75, step * i, 0.05) for i in range(4)]
    assert len(cluster_points(points, ZONE_ANGLE)) == 1


def test_short_line_is_not_a_wall():
    points = wall_points(half_length=0.05, count=4)
    assert fit_shape(points).kind == OBJECT


def test_corner_splits_into_two_walls():
    # An L: a wall ahead at x = 0.5 and one to the left at y = 0.5.
    ahead = [(0.5, -0.3 + 0.05 * i, 0.05) for i in range(17)]
    left = [(0.5 - 0.05 * i, 0.5, 0.05) for i in range(1, 11)]
    shapes = fit_shapes(ahead + left)
    assert [s.kind for s in shapes] == [WALL, WALL]
    yaws = sorted(abs(math.cos(s.yaw)) for s in shapes)
    assert yaws[0] == pytest.approx(0.0, abs=0.05)
    assert yaws[1] == pytest.approx(1.0, abs=0.05)


def test_wall_split_at_a_sensor_seam_is_merged_back():
    left = fit_shape([(0.5, 0.05 + 0.03 * i, 0.05) for i in range(8)])
    right = fit_shape([(0.5, -0.3 + 0.03 * i, 0.05) for i in range(8)])
    merged = merge_walls([left, right])
    assert len(merged) == 1
    assert merged[0].length == pytest.approx(0.56, abs=0.01)


def test_corner_walls_are_not_merged():
    ahead = fit_shape([(0.5, -0.2 + 0.03 * i, 0.05) for i in range(10)])
    side = fit_shape([(0.2 + 0.03 * i, 0.3, 0.05) for i in range(10)])
    assert len(merge_walls([ahead, side])) == 2


def test_straight_noisy_wall_does_not_split():
    rng = random.Random(3)
    points = [(0.5 + rng.gauss(0.0, 0.004), -0.3 + 0.03 * i, 0.05)
              for i in range(21)]
    assert len(split_segments(points)) == 1


def test_floor_profile_fills_gaps_around_the_ring():
    points = [(0.3 * math.cos(a), 0.3 * math.sin(a), 0.0)
              for a in (0.1, 1.7, 3.0, -1.5)]
    radii = fill_profile(floor_profile(points, bins=16))
    assert len(radii) == 16
    assert all(r == pytest.approx(0.3) for r in radii)
    assert fill_profile([None] * 8) is None


def test_tracker_confirms_holds_then_drops():
    tracker = ShapeTracker(confirm_frames=2, hold_frames=4)
    shape = Shape(OBJECT, 0.3, 0.0, 0.0, 0.05, 0.05, 0.0, 0.05)
    assert tracker.update([shape]) == {}
    assert len(tracker.update([shape])) == 1
    tracker.update([shape])
    # Score 3 -> held through one missed frame, gone after enough misses.
    assert len(tracker.update([])) == 1
    for _ in range(3):
        last = tracker.update([])
    assert last == {}


def test_tracker_smooths_jitter():
    rng = random.Random(1)
    tracker = ShapeTracker(confirm_frames=1, smoothing=0.3)
    raw, smoothed = [], []
    for _ in range(200):
        x = 0.4 + rng.gauss(0.0, 0.01)
        raw.append(x)
        smoothed.extend(
            s.x for s in tracker.update(
                [Shape(OBJECT, x, 0.0, 0.0, 0.05, 0.05, 0.0, 0.05)]).values())

    def spread(values):
        mean = sum(values) / len(values)
        return math.sqrt(sum((v - mean) ** 2 for v in values) / len(values))

    assert spread(smoothed[20:]) < 0.6 * spread(raw[20:])


def test_tracker_yaw_blend_does_not_spin_half_a_turn():
    tracker = ShapeTracker(confirm_frames=1, smoothing=0.5)
    tracker.update([Shape(WALL, 0.5, 0.0, math.pi / 2 - 0.01,
                          0.4, 0.02, 0.0, 0.05)])
    shape, = tracker.update([Shape(WALL, 0.5, 0.0, -math.pi / 2 + 0.01,
                                   0.4, 0.02, 0.0, 0.05)]).values()
    assert abs(math.cos(shape.yaw)) < 0.02


def test_warning_bands_step_from_critical_to_clear():
    from mobile_base_tools.tof_palette import (
        CAUTION, CLEAR, CRITICAL, warning_hex)
    assert warning_hex(0.10) == CRITICAL
    assert warning_hex(0.30) == CAUTION
    assert warning_hex(0.70) == CLEAR

