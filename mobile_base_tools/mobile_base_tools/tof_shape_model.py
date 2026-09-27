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

"""Summarise classified ToF points as walls, objects and the visible floor.

The input is the classifier's output, never the raw clouds. Raw returns carry
the floor, and the shallow rows' cot(elevation) error, straight into any shape
built from them; the classifier has already paid for removing both.

Walls and objects are rectangles in the ground plane with a height, one
``Shape`` type for both; the floor is a radius per bearing, one region. Heights are what the ring observed, not
what is there: nothing above about 100 mm is ever seen, so a wall drawn 90 mm
tall means "at least 90 mm", not "90 mm".
"""

import math

WALL = 'wall'
OBJECT = 'object'


class Shape:
    """An oriented rectangle on the floor, extruded from z_min to z_max."""

    __slots__ = ('kind', 'x', 'y', 'yaw', 'length', 'width', 'z_min', 'z_max')

    def __init__(self, kind, x, y, yaw, length, width, z_min, z_max):
        self.kind = kind
        self.x = x
        self.y = y
        self.yaw = yaw
        self.length = length
        self.width = width
        self.z_min = z_min
        self.z_max = z_max

    def distance(self):
        """Distance from the body origin to the nearest point of the shape."""
        cos_yaw, sin_yaw = math.cos(self.yaw), math.sin(self.yaw)
        along = -(self.x * cos_yaw + self.y * sin_yaw)
        across = self.x * sin_yaw - self.y * cos_yaw
        outside_along = max(abs(along) - self.length / 2.0, 0.0)
        outside_across = max(abs(across) - self.width / 2.0, 0.0)
        return math.hypot(outside_along, outside_across)


def cluster_points(points, zone_angle, min_link=0.05, link_scale=1.5,
                   min_points=3):
    """Group points by XY single linkage; drop groups below ``min_points``.

    Adjacent zones separate by ``range * zone_angle``, so the link distance
    grows with range - a fixed one would split a far wall into fragments or
    merge near objects into one. Small groups are dropped because a real
    surface fills several neighbouring zones while noise rarely does.
    """
    remaining = list(range(len(points)))
    clusters = []
    while remaining:
        frontier = [remaining.pop()]
        members = []
        while frontier:
            index = frontier.pop()
            members.append(index)
            px, py = points[index][0], points[index][1]
            link = max(min_link,
                       math.hypot(px, py) * zone_angle * link_scale)
            near = [
                other for other in remaining
                if math.hypot(points[other][0] - px,
                              points[other][1] - py) <= link
            ]
            for other in near:
                remaining.remove(other)
            frontier.extend(near)
        if len(members) >= min_points:
            clusters.append([points[index] for index in members])
    return clusters


def principal_axes(points):
    """Return (centre_x, centre_y, yaw of the major axis) of XY points."""
    count = len(points)
    mean_x = sum(p[0] for p in points) / count
    mean_y = sum(p[1] for p in points) / count
    sxx = sum((p[0] - mean_x) ** 2 for p in points)
    syy = sum((p[1] - mean_y) ** 2 for p in points)
    sxy = sum((p[0] - mean_x) * (p[1] - mean_y) for p in points)
    return mean_x, mean_y, 0.5 * math.atan2(2.0 * sxy, sxx - syy)


def oriented_extent(points, x, y, yaw):
    """Fit the tightest box at ``yaw``: (centre_x, centre_y, length, width)."""
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    along = [(p[0] - x) * cos_yaw + (p[1] - y) * sin_yaw for p in points]
    across = [-(p[0] - x) * sin_yaw + (p[1] - y) * cos_yaw for p in points]
    mid_along = (max(along) + min(along)) / 2.0
    mid_across = (max(across) + min(across)) / 2.0
    return (
        x + mid_along * cos_yaw - mid_across * sin_yaw,
        y + mid_along * sin_yaw + mid_across * cos_yaw,
        max(along) - min(along),
        max(across) - min(across),
    )


def fit_shape(cluster, wall_aspect_ratio=3.0, wall_min_length=0.15,
              wall_thickness=0.02, min_extent=0.03):
    """Turn one obstacle cluster into a WALL or an OBJECT.

    The ring cannot see behind a surface, so a long thin cluster is the face of
    something, drawn as a thin slab. Compact clusters get a box that encloses
    them, clamped to ``min_extent`` so a three-zone object is still visible.
    Every shape stands on the floor: the ring saw its face, not its base.
    """
    x, y, yaw = principal_axes(cluster)
    x, y, length, width = oriented_extent(cluster, x, y, yaw)
    z_max = max(p[2] for p in cluster)
    if (length >= wall_min_length
            and length >= wall_aspect_ratio * max(width, 1e-6)):
        return Shape(WALL, x, y, yaw, length, wall_thickness, 0.0, z_max)
    return Shape(OBJECT, x, y, yaw, max(length, min_extent),
                 max(width, min_extent), 0.0, max(z_max, min_extent))


def split_segments(cluster, tolerance=0.03, min_points=3):
    """Break a cluster into straight runs, so a corner becomes two walls.

    Points are ordered by bearing from the robot - the order the ring sweeps a
    surface seen from inside a room - and the run is split at the point
    furthest from the chord between its ends while that point sits more than
    ``tolerance`` off it (split-and-merge line extraction). A straight wall
    never splits; an L splits once at its corner. Pieces too small to fit are
    kept whole rather than dropped, so nothing the classifier marked vanishes.
    """
    ordered = sorted(cluster, key=lambda p: math.atan2(p[1], p[0]))
    # A cluster straddling the rear (bearing +-pi) would sort into two halves;
    # rotate the order to start at the widest angular gap instead.
    bearings = [math.atan2(p[1], p[0]) for p in ordered]
    gaps = [
        (bearings[(i + 1) % len(bearings)] - bearings[i]) % (2.0 * math.pi)
        for i in range(len(bearings))
    ]
    start = (gaps.index(max(gaps)) + 1) % len(ordered)
    ordered = ordered[start:] + ordered[:start]

    def split(run):
        if len(run) < 2 * min_points:
            return [run]
        (ax, ay), (bx, by) = run[0][:2], run[-1][:2]
        chord = math.hypot(bx - ax, by - ay)
        if chord < 1e-6:
            return [run]
        deviations = [
            abs((bx - ax) * (ay - p[1]) - (ax - p[0]) * (by - ay)) / chord
            for p in run
        ]
        index = max(range(len(run)), key=deviations.__getitem__)
        if deviations[index] <= tolerance:
            return [run]
        index = min(max(index, min_points), len(run) - min_points)
        # The corner point belongs to both walls: they meet there.
        return split(run[:index + 1]) + split(run[index:])

    return split(ordered)


def fit_shapes(cluster, **options):
    """Fit one shape per straight run of ``cluster`` (see split_segments)."""
    return [fit_shape(run, **options) for run in split_segments(cluster)]


def _wall_ends(wall):
    """The two ends of a wall's centre line."""
    dx = math.cos(wall.yaw) * wall.length / 2.0
    dy = math.sin(wall.yaw) * wall.length / 2.0
    return (wall.x - dx, wall.y - dy), (wall.x + dx, wall.y + dy)


def merge_walls(shapes, max_angle=math.radians(10.0), max_offset=0.04,
                max_gap=0.25):
    """Join wall pieces that are one wall seen through a gap.

    Clustering splits a wall wherever neighbouring zones land further apart
    than the link distance - typically at the seam between two sensors. Pieces
    that are parallel, on the same line and close end to end are one surface,
    and are redrawn as a single wall spanning both. Corners are untouched: two
    walls at an angle fail the parallel test.
    """
    walls = [s for s in shapes if s.kind == WALL]
    others = [s for s in shapes if s.kind != WALL]
    merged = True
    while merged:
        merged = False
        for i in range(len(walls)):
            for j in range(i + 1, len(walls)):
                a, b = walls[i], walls[j]
                if abs(_angle_step(a.yaw, b.yaw)) > max_angle:
                    continue
                cos_yaw, sin_yaw = math.cos(a.yaw), math.sin(a.yaw)
                offset = abs(-(b.x - a.x) * sin_yaw + (b.y - a.y) * cos_yaw)
                if offset > max_offset:
                    continue
                along = [
                    (px - a.x) * cos_yaw + (py - a.y) * sin_yaw
                    for px, py in _wall_ends(a) + _wall_ends(b)
                ]
                gap = (max(along[:2] + along[2:]) - min(along[:2] + along[2:])
                       - a.length - b.length)
                if gap > max_gap:
                    continue
                low, high = min(along), max(along)
                middle = (low + high) / 2.0
                walls[i] = Shape(
                    WALL, a.x + middle * cos_yaw, a.y + middle * sin_yaw,
                    a.yaw, high - low, a.width, 0.0, max(a.z_max, b.z_max))
                del walls[j]
                merged = True
                break
            if merged:
                break
    return walls + others


def floor_profile(points, bins=48):
    """Furthest confirmed floor, per bearing bin; None where nothing was seen.

    One radius per bin describes the whole visible floor as a single region
    around the robot - it reaches out where the floor is clear and pulls in
    where something stands on it.
    """
    width = 2.0 * math.pi / bins
    radii = [None] * bins
    for x, y, _ in points:
        index = int((math.atan2(y, x) + math.pi) // width) % bins
        distance = math.hypot(x, y)
        if radii[index] is None or distance > radii[index]:
            radii[index] = distance
    return radii


def fill_profile(radii):
    """Interpolate empty bins from their nearest seen neighbours, circularly.

    Adjacent sensors overlap, so a gap is a bin between two zones, not a hole
    in the floor. Returns None when no bin was seen at all.
    """
    seen = [i for i, r in enumerate(radii) if r is not None]
    if not seen:
        return None
    count = len(radii)
    filled = list(radii)
    for i in range(count):
        if filled[i] is not None:
            continue
        before = max((j for j in seen if j < i), default=seen[-1] - count)
        after = min((j for j in seen if j > i), default=seen[0] + count)
        share = (i - before) / float(after - before)
        filled[i] = ((1.0 - share) * radii[before % count]
                     + share * radii[after % count])
    return filled


def _angle_step(current, target):
    """Signed step from ``current`` to ``target``, rectangles being symmetric.

    A rectangle at yaw and yaw + pi is the same rectangle, so the step is
    taken modulo pi to stop a blend spinning a shape half a turn.
    """
    step = (target - current) % math.pi
    return step - math.pi if step > math.pi / 2.0 else step


class ShapeTracker:
    """Hold shapes steady across frames, as the classifier holds zones.

    A shape is matched to the nearest tracked shape of the same kind, blended
    into it, and only published once seen ``confirm_frames`` times. An unseen
    shape survives ``hold_frames`` frames. Without this the boxes inherit the
    per-frame churn of the points underneath them.
    """

    def __init__(self, confirm_frames=2, hold_frames=4, smoothing=0.3,
                 match_distance=0.12):
        self.confirm_frames = confirm_frames
        self.hold_frames = hold_frames
        self.smoothing = smoothing
        self.match_distance = match_distance
        self.tracks = {}
        self.next_id = 0

    def blend(self, old, new):
        keep = 1.0 - self.smoothing

        def mix(a, b):
            return keep * a + self.smoothing * b

        return Shape(
            old.kind, mix(old.x, new.x), mix(old.y, new.y),
            old.yaw + self.smoothing * _angle_step(old.yaw, new.yaw),
            mix(old.length, new.length), mix(old.width, new.width),
            mix(old.z_min, new.z_min), mix(old.z_max, new.z_max))

    def update(self, shapes):
        """Fold in one frame; return {track id: shape} for confirmed shapes."""
        unmatched = set(self.tracks)
        for shape in shapes:
            best, best_distance = None, self.match_distance
            for track_id in unmatched:
                tracked = self.tracks[track_id][1]
                if tracked.kind != shape.kind:
                    continue
                gap = math.hypot(tracked.x - shape.x, tracked.y - shape.y)
                if gap <= best_distance:
                    best, best_distance = track_id, gap
            if best is None:
                self.tracks[self.next_id] = [1, shape]
                self.next_id += 1
                continue
            unmatched.discard(best)
            score, tracked = self.tracks[best]
            self.tracks[best] = [min(score + 1, self.hold_frames),
                                 self.blend(tracked, shape)]
        for track_id in unmatched:
            self.tracks[track_id][0] -= 1
            if self.tracks[track_id][0] <= 0:
                del self.tracks[track_id]
        return {
            track_id: shape
            for track_id, (score, shape) in self.tracks.items()
            if score >= self.confirm_frames
        }
