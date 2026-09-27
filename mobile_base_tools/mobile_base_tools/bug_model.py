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

"""ToF-only navigation: speed levels and a holonomic Bug2, without ROS.

Everything here works in the body frame (``base_footprint``: x forward, y
left) on the classifier's obstacle points, and in ``odom`` for the robot's own
pose. No map and no LiDAR are involved.

**Speed levels.** The limit is set by the free distance *along the direction of
travel*: only points inside a corridor as wide as the body, swept along the
velocity, count. A holonomic base sliding along a wall has that wall beside it,
not ahead, so it is not slowed by it. Each level's speed satisfies

    2 * (v * t + v^2 / (2 * a)) <= d - margin

at the near edge of its band, with t = 0.35 s (a 15 Hz frame, two frames to
confirm, control and motor lag), a = 1.0 m/s^2 (conservative until braking is
measured on hardware), a 5 cm margin and a safety factor of 2.

**Bug2, holonomic.** Heading is held constant; the robot only translates.
Moving to the goal it slides along the start-goal line (the m-line). Meeting
an obstacle it follows the boundary with a vector field - tangent to the
nearest obstacle point plus a correction toward a standoff - which any
face of the ring can serve, because the ring is 360 degrees. It leaves the
boundary where Bug2 does: back on the m-line, closer to the goal than where it
hit, with the way to the goal clear.

The standoff adapts to the space. With the far side open the robot keeps a
comfortable ``standoff`` from the wall it follows; as the far side closes in -
a corridor, a gap - it moves towards the middle, down to ``min_standoff``. It
only gets close to things when there is no room not to.
"""

import math

# Body outline, from the URDF: 0.26 m bumper to bumper, 0.22 m across wheels.
HALF_LENGTH = 0.13
HALF_WIDTH = 0.11

CRUISE, CAUTION, SLOW, CRAWL, STOP = 'cruise', 'caution', 'slow', 'crawl', 'stop'
LEVELS = (STOP, CRAWL, SLOW, CAUTION, CRUISE)  # slowest first

GO_TO_GOAL, FOLLOW, ARRIVED, UNREACHABLE, IDLE = (
    'go_to_goal', 'follow_boundary', 'arrived', 'unreachable', 'idle')


def body_support(ux, uy):
    """How far the body reaches from its centre in direction (ux, uy)."""
    return abs(ux) * HALF_LENGTH + abs(uy) * HALF_WIDTH


def corridor_distance(points, ux, uy, margin=0.03):
    """Free distance from the body's face along (ux, uy), inside its swath.

    Only points ahead of the body and within the corridor the body sweeps
    (plus ``margin`` either side) count. Returns ``math.inf`` when the
    corridor is empty.
    """
    norm = math.hypot(ux, uy)
    if norm < 1e-9:
        return math.inf
    ux, uy = ux / norm, uy / norm
    half_swath = body_support(-uy, ux) + margin
    front = body_support(ux, uy)
    nearest = math.inf
    for point in points:
        along = point[0] * ux + point[1] * uy
        across = abs(-point[0] * uy + point[1] * ux)
        if along > 0.0 and across <= half_swath:
            nearest = min(nearest, along - front)
    return max(nearest, 0.0)


class SpeedLevels:
    """Distance bands and their speeds; walls may come closer before slowing.

    A wall is a known, flat, fixed surface, so it only counts as an obstacle
    once it is within ``wall_caution``; anything else from ``caution``.
    """

    def __init__(self, cruise=0.50, caution=0.33, slow=0.20, crawl=0.10,
                 caution_range=1.0, wall_caution_range=0.65, slow_range=0.4,
                 crawl_range=0.25, stop_range=0.12, squeeze_slow=0.15,
                 squeeze_crawl=0.06):
        self.speeds = {CRUISE: cruise, CAUTION: caution, SLOW: slow,
                       CRAWL: crawl, STOP: 0.0}
        self.caution_range = caution_range
        self.wall_caution_range = wall_caution_range
        self.slow_range = slow_range
        self.crawl_range = crawl_range
        self.stop_range = stop_range
        self.squeeze_slow = squeeze_slow
        self.squeeze_crawl = squeeze_crawl

    def level(self, distance, wall=False):
        """The level for one distance from the body face."""
        if distance < self.stop_range:
            return STOP
        if distance < self.crawl_range:
            return CRAWL
        if distance < self.slow_range:
            return SLOW
        if distance < (self.wall_caution_range if wall
                       else self.caution_range):
            return CAUTION
        return CRUISE

    def squeeze(self, clearance):
        """The level for a gap beside the body: slow through narrow places."""
        if clearance < self.squeeze_crawl:
            return CRAWL
        if clearance < self.squeeze_slow:
            return SLOW
        return CRUISE


def side_clearance(points, ux, uy, reach=0.10):
    """Smallest gap between the body's sides and anything alongside it.

    "Alongside" is the stretch the body occupies along (ux, uy), plus
    ``reach`` ahead. This is what a squeeze through a gap looks like: nothing
    in the corridor ahead, but walls a few centimetres off either side.
    """
    norm = math.hypot(ux, uy)
    if norm < 1e-9:
        return math.inf
    ux, uy = ux / norm, uy / norm
    front = body_support(ux, uy)
    side = body_support(-uy, ux)
    nearest = math.inf
    for point in points:
        along = point[0] * ux + point[1] * uy
        across = abs(-point[0] * uy + point[1] * ux)
        if -front <= along <= front + reach and across > side:
            nearest = min(nearest, across - side)
    return nearest


def slowest(*levels):
    """The most restrictive of several levels."""
    return min(levels, key=LEVELS.index)


def stopping_speed(distance, reaction=0.35, decel=1.0, margin=0.05,
                   factor=2.0):
    """Largest v with factor * (v t + v^2 / 2a) <= distance - margin."""
    room = (distance - margin) / factor
    if room <= 0.0:
        return 0.0
    # v^2 / (2a) + v t - room = 0
    return decel * (-reaction + math.sqrt(reaction ** 2 + 2.0 * room / decel))


def split_walls(points, roof=0.0995, xy_tolerance=0.03):
    """Split obstacle points into (walls, others).

    The classifier publishes nothing above the roof except confirmed wall, so a
    point shares a column with something above the roof only if it is wall.
    """
    high = [p for p in points if p[2] > roof]
    walls, others = [], []
    for point in points:
        if point[2] > roof or any(
                math.hypot(point[0] - h[0], point[1] - h[1]) <= xy_tolerance
                for h in high):
            walls.append(point)
        else:
            others.append(point)
    return walls, others


def to_body(x, y, pose):
    """An odom point in the body frame of ``pose`` = (x, y, yaw)."""
    dx, dy = x - pose[0], y - pose[1]
    cos_yaw, sin_yaw = math.cos(pose[2]), math.sin(pose[2])
    return dx * cos_yaw + dy * sin_yaw, -dx * sin_yaw + dy * cos_yaw


def line_distance(point, start, goal):
    """Perpendicular distance from ``point`` to the start-goal line."""
    ex, ey = goal[0] - start[0], goal[1] - start[1]
    length = math.hypot(ex, ey)
    if length < 1e-9:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    return abs(ex * (start[1] - point[1]) - ey * (start[0] - point[0])) / length


def nearest_clearance(points):
    """(clearance from the body face, unit direction) to the nearest point."""
    best = None
    for point in points:
        reach = math.hypot(point[0], point[1])
        if reach < 1e-6:
            continue
        ux, uy = point[0] / reach, point[1] / reach
        clearance = reach - body_support(ux, uy)
        if best is None or clearance < best[0]:
            best = (clearance, ux, uy)
    return best


class Bug2:
    """A holonomic Bug2. Call ``step`` each cycle; it returns a body velocity.

    ``side`` is which way round obstacles it goes: 'left' keeps them on the
    robot's right, as a person walking anticlockwise round a room would.
    """

    def __init__(self, speed=0.5, standoff=0.30, min_standoff=0.05,
                 hit_distance=0.35,
                 goal_tolerance=0.10, line_tolerance=0.05, leave_gain=0.10,
                 gain=2.0, follow_range=0.8, side='left',
                 contact_window=math.radians(100.0)):
        self.speed = speed
        self.standoff = standoff
        self.min_standoff = min_standoff
        self.hit_distance = hit_distance
        self.goal_tolerance = goal_tolerance
        self.line_tolerance = line_tolerance
        self.leave_gain = leave_gain
        self.gain = gain
        self.follow_range = follow_range
        self.turn = 1.0 if side == 'left' else -1.0
        self.contact_window = contact_window
        # Body-frame direction to the boundary being followed. Heading is held
        # constant, so the body frame does not rotate under it.
        self.contact = None
        self.state = IDLE
        self.start = self.goal = self.hit = None
        self.hit_to_goal = math.inf
        self.left_hit = False

    def set_goal(self, pose, goal):
        """Start a new run from ``pose`` (x, y, yaw) to ``goal`` (x, y)."""
        self.start, self.goal = (pose[0], pose[1]), goal
        self.state = GO_TO_GOAL
        self.hit = None

    def to_goal(self, pose):
        return math.hypot(self.goal[0] - pose[0], self.goal[1] - pose[1])

    def step(self, pose, points):
        """One cycle: (vx, vy) in the body frame for ``pose`` and ``points``.

        ``points`` are obstacle points in the body frame.
        """
        if self.state in (IDLE, ARRIVED, UNREACHABLE):
            return 0.0, 0.0
        if self.to_goal(pose) <= self.goal_tolerance:
            self.state = ARRIVED
            return 0.0, 0.0

        gx, gy = to_body(self.goal[0], self.goal[1], pose)
        toward = math.hypot(gx, gy)
        gx, gy = gx / toward, gy / toward
        ahead = corridor_distance(points, gx, gy)

        if self.state == GO_TO_GOAL:
            if ahead > self.hit_distance:
                return self.speed * gx, self.speed * gy
            self.state = FOLLOW
            self.contact = (gx, gy)
            self.hit = (pose[0], pose[1])
            self.hit_to_goal = self.to_goal(pose)
            self.left_hit = False

        # FOLLOW
        here = (pose[0], pose[1])
        if not self.left_hit and math.dist(here, self.hit) > 0.3:
            self.left_hit = True
        if (line_distance(here, self.start, self.goal) <= self.line_tolerance
                and self.to_goal(pose) < self.hit_to_goal - self.leave_gain
                and ahead > self.hit_distance + 0.1):
            self.state = GO_TO_GOAL
            return self.speed * gx, self.speed * gy
        if self.left_hit and math.dist(here, self.hit) < 0.15:
            self.state = UNREACHABLE
            return 0.0, 0.0

        # Only points on the side already being followed: within
        # contact_window of the last direction to the boundary. At a wall end
        # the end itself is behind that side and the next wall is ahead of
        # it, so the robot rounds the end instead of jumping to the next wall.
        limit = math.cos(self.contact_window)
        near = [
            p for p in points
            if math.hypot(p[0], p[1]) <= self.follow_range
            and (p[0] * self.contact[0] + p[1] * self.contact[1])
            >= limit * math.hypot(p[0], p[1])
        ]
        found = nearest_clearance(near)
        if found is None:
            # Lost the boundary (it fell out of range round a corner): head
            # for the goal again rather than wander; Bug2 resumes from here.
            self.state = GO_TO_GOAL
            return self.speed * gx, self.speed * gy
        clearance, nx, ny = found
        self.contact = (nx, ny)
        target = self.target_standoff(points, nx, ny, clearance)
        # Tangent: the obstacle direction turned a quarter, towards `side`.
        tx, ty = -self.turn * ny, self.turn * nx
        pull = max(-1.0, min(1.0, self.gain * (clearance - target)))
        vx, vy = tx + pull * nx, ty + pull * ny
        norm = math.hypot(vx, vy)
        return self.speed * vx / norm, self.speed * vy / norm

    def target_standoff(self, points, nx, ny, clearance):
        """How far to keep from the followed boundary, given the far side.

        The far side is whatever lies within 60 degrees of straight away from
        the boundary. Open: keep ``standoff``. Narrowing: take the middle of
        the space, never less than ``min_standoff``.
        """
        limit = math.cos(math.radians(60.0))
        away = [
            p for p in points
            if -(p[0] * nx + p[1] * ny) >= limit * math.hypot(p[0], p[1])
        ]
        found = nearest_clearance(away)
        if found is None:
            return self.standoff
        room = clearance + found[0]
        return max(self.min_standoff, min(self.standoff, room / 2.0))
