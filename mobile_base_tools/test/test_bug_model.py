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

"""Pin the speed levels, the corridor, and Bug2 on a kinematic robot."""

import math

import pytest

from mobile_base_tools.bug_model import (
    ARRIVED, CAUTION, CRAWL, CRUISE, SLOW, STOP, UNREACHABLE,
    Bug2, SpeedLevels, body_support, corridor_distance, slowest,
    side_clearance, split_walls, stopping_speed)


def test_each_level_is_within_the_stopping_limit_at_its_near_edge():
    levels = SpeedLevels()
    for level, edge in ((CAUTION, levels.slow_range),
                        (SLOW, levels.crawl_range),
                        (CRAWL, levels.stop_range + 0.03)):
        assert levels.speeds[level] <= stopping_speed(edge) + 0.01
    # And the cruise speed is safe wherever caution begins.
    assert levels.speeds[CRUISE] <= stopping_speed(
        levels.wall_caution_range)


def test_levels_by_distance_and_walls_come_closer():
    levels = SpeedLevels()
    assert levels.level(1.5) == CRUISE
    assert levels.level(0.8) == CAUTION
    assert levels.level(0.8, wall=True) == CRUISE
    assert levels.level(0.5, wall=True) == CAUTION
    assert levels.level(0.3) == SLOW
    assert levels.level(0.2) == CRAWL
    assert levels.level(0.05) == STOP
    assert slowest(CRUISE, SLOW, CAUTION) == SLOW


def test_corridor_ignores_a_wall_beside_the_direction_of_travel():
    wall_on_left = [(x / 10.0, 0.30, 0.05) for x in range(-5, 6)]
    # Sliding forward along it: nothing ahead.
    assert corridor_distance(wall_on_left, 1.0, 0.0) == math.inf
    # Strafing into it: 0.30 m minus the half width.
    assert corridor_distance(wall_on_left, 0.0, 1.0) == pytest.approx(
        0.30 - body_support(0.0, 1.0))


def test_points_under_a_wall_column_count_as_wall():
    points = [(0.5, 0.0, 0.05), (0.5, 0.0, 0.3), (0.3, 0.2, 0.04)]
    walls, others = split_walls(points)
    assert len(walls) == 2 and others == [(0.3, 0.2, 0.04)]


def box_points(pose, box, reach=0.8, step=0.02):
    """Body-frame points on the outline of ``box`` within ``reach``.

    ``box`` is (x_min, y_min, x_max, y_max) in odom; the ring sees its faces.
    """
    x0, y0, x1, y1 = box
    outline = []
    for i in range(int((x1 - x0) / step) + 1):
        x = x0 + i * step
        outline += [(x, y0), (x, y1)]
    for i in range(int((y1 - y0) / step) + 1):
        y = y0 + i * step
        outline += [(x0, y), (x1, y)]
    cos_yaw, sin_yaw = math.cos(pose[2]), math.sin(pose[2])
    points = []
    for x, y in outline:
        dx, dy = x - pose[0], y - pose[1]
        bx, by = dx * cos_yaw + dy * sin_yaw, -dx * sin_yaw + dy * cos_yaw
        if math.hypot(bx, by) <= reach:
            points.append((bx, by, 0.05))
    return points


def drive(bug, pose, goal, box, steps=3000, dt=0.05):
    """Integrate Bug2's body velocities; return the path and final state."""
    bug.set_goal(pose, goal)
    path = [pose]
    for _ in range(steps):
        vx, vy = bug.step(pose, box_points(pose, box))
        if bug.state in (ARRIVED, UNREACHABLE):
            break
        cos_yaw, sin_yaw = math.cos(pose[2]), math.sin(pose[2])
        pose = (pose[0] + (vx * cos_yaw - vy * sin_yaw) * dt,
                pose[1] + (vx * sin_yaw + vy * cos_yaw) * dt, pose[2])
        path.append(pose)
    return path, bug.state


def clearance(path, box):
    x0, y0, x1, y1 = box
    return min(
        math.hypot(max(x0 - x, 0.0, x - x1), max(y0 - y, 0.0, y - y1))
        for x, y, _ in path)


@pytest.mark.parametrize('yaw', [0.0, 0.7, -1.9])
def test_bug2_goes_round_a_box_to_the_goal(yaw):
    box = (1.0, -0.4, 1.4, 0.4)
    path, state = drive(Bug2(speed=0.3), (0.0, 0.0, yaw), (2.5, 0.0), box)
    assert state == ARRIVED
    # Never closer to the box than the body's widest reach: no contact.
    assert clearance(path, box) > math.hypot(0.13, 0.11)


def test_bug2_reports_a_goal_inside_an_obstacle_unreachable():
    box = (1.0, -0.6, 2.0, 0.6)
    _, state = drive(Bug2(speed=0.3), (0.0, 0.0, 0.0), (1.5, 0.0), box)
    assert state == UNREACHABLE


def boxes_points(pose, boxes, reach=0.8):
    return [p for box in boxes for p in box_points(pose, box, reach)]


def test_bug2_rounds_a_wall_end_instead_of_switching_to_the_next_wall():
    # navigation_basic: the interior wall ends 1 m short of the north wall.
    interior = (1.425, -1.0, 1.575, 3.0)
    north = (-5.0, 3.925, 5.0, 4.075)
    bug = Bug2(speed=0.3)
    pose, goal = (0.0, 0.0, 0.0), (3.0, 1.0)
    bug.set_goal(pose, goal)
    path = [pose]
    for _ in range(4000):
        vx, vy = bug.step(pose, boxes_points(pose, (interior, north)))
        if bug.state in (ARRIVED, UNREACHABLE):
            break
        pose = (pose[0] + vx * 0.05, pose[1] + vy * 0.05, 0.0)
        path.append(pose)
    assert bug.state == ARRIVED
    length = sum(math.dist(a[:2], b[:2]) for a, b in zip(path, path[1:]))
    assert length < 10.0



@pytest.mark.parametrize('gap', [0.45, 0.38, 0.32])
def test_bug2_drives_through_gaps_it_fits(gap):
    # Two 0.2 x 0.6 m posts either side of the path.
    half = gap / 2.0
    posts = ((1.0, half, 1.2, half + 0.6), (1.0, -half - 0.6, 1.2, -half))
    bug = Bug2(speed=0.3)
    pose, goal = (0.0, 0.0, 0.0), (2.5, 0.0)
    bug.set_goal(pose, goal)
    path = [pose]
    for _ in range(2000):
        vx, vy = bug.step(pose, boxes_points(pose, posts))
        if bug.state in (ARRIVED, UNREACHABLE):
            break
        pose = (pose[0] + vx * 0.05, pose[1] + vy * 0.05, 0.0)
        path.append(pose)
    assert bug.state == ARRIVED
    # Straight through: no detour round the posts.
    assert sum(math.dist(a[:2], b[:2]) for a, b in zip(path, path[1:])) < 2.8
    # And the body never overlaps a post.
    inside = [p for p in path if 1.0 - 0.13 <= p[0] <= 1.2 + 0.13]
    assert all(abs(p[1]) + 0.11 < half for p in inside)


def test_a_squeeze_slows_the_robot_even_with_the_way_ahead_clear():
    levels = SpeedLevels()
    # Driving forward through a 0.38 m gap: posts 0.08 m off each side.
    posts = [(x / 20.0, y, 0.05) for x in range(-2, 3) for y in (0.19, -0.19)]
    assert corridor_distance(posts, 1.0, 0.0) == math.inf
    assert side_clearance(posts, 1.0, 0.0) == pytest.approx(0.08)
    assert levels.squeeze(side_clearance(posts, 1.0, 0.0)) == SLOW
    assert levels.squeeze(0.04) == CRAWL
    assert levels.squeeze(0.5) == CRUISE


def test_standoff_is_generous_in_the_open_and_centres_when_narrow():
    bug = Bug2()
    wall = [(x / 10.0, -0.40, 0.05) for x in range(-5, 6)]  # right side
    # Open on the left: keep the full standoff.
    assert bug.target_standoff(wall, 0.0, -1.0, 0.29) == bug.standoff
    # A wall 0.25 m off the left side as well: take the middle.
    corridor = wall + [(x / 10.0, 0.36, 0.05) for x in range(-5, 6)]
    target = bug.target_standoff(corridor, 0.0, -1.0, 0.29)
    assert target == pytest.approx((0.29 + 0.25) / 2.0, abs=0.02)
    # Barely wider than the body: never below the minimum.
    tight = wall + [(x / 10.0, -0.25, 0.05) for x in range(-5, 6)]
    assert bug.target_standoff(
        tight + [(0.0, 0.13, 0.05)], 0.0, -1.0, 0.02) == bug.min_standoff


def test_following_in_the_open_keeps_its_distance():
    box = (1.0, -0.4, 1.4, 0.4)
    path, state = drive(Bug2(speed=0.3), (0.0, 0.0, 0.0), (2.5, 0.0), box)
    assert state == ARRIVED
    # Nothing else around, so it never closes to the minimum.
    assert clearance(path, box) > 0.13 + 0.15
