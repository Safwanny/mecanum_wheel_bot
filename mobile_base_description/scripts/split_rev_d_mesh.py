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

"""
Split the Rev D whole-robot STL into a body mesh and four turning wheels.

The CAD export ros2/mecanum_01_full.stl (metres, REP-103, origin on the ground
under the robot centre) is ~200 closed parts saved into one file. Parts share
no vertices, so connected components give them back whole. Each part then
goes to the body or to one corner's wheel mesh:

  wheel     inside the wheel cylinder (rollers, plates, hub, hub bolts)
  drive     on the axle line outboard of the gearbox: motor shaft, coupler,
            axle and the three grub screws - they turn with the wheel
  dropped   bearings and heat-set inserts, hidden inside the plastic
  body      everything else

Wheel meshes are written in their wheel joint frame (origin on the axle at the
wheel centre), the body in the ground frame. The CAD folder is only read.

  split_rev_d_mesh.py <Rev_D dir> <output dir>

Fails loudly if a corner is missing a drive-line part or the wheels are not
where the README says. Re-run whenever the CAD changes; a Fusion export of the
same five file names can replace these without touching the URDF.
"""

from pathlib import Path
import sys

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

WHEEL_X = 0.105          # wheelbase 210 / 2
WHEEL_Y = 0.09325        # track 186.5 / 2
WHEEL_Z = 0.02996        # axle height = roller envelope radius
WHEEL_R = 0.0305         # roller envelope 59.92 / 2, plus margin
WHEEL_INNER_Y = 0.069    # hub inner end; the wheel support stops before it
GEARBOX_FACE_Y = 0.034   # drive-line parts start outboard of this
DRIVE_R = 0.008          # coupler is the widest turning part, r 7
TOL = 0.3e-3

# Sorted bounding-box dimensions (m), from 02_cad/parts.
SIG = {
    'axle': (4.5, 5.0, 40.0),
    'coupler': (14.0, 14.0, 18.5),
    'motor_shaft': (6.0, 6.0, 14.0),
    'grub': (2.9, 2.9, 5.0),
    'bearing': (4.0, 10.0, 10.0),
    'insert': (5.1, 5.2, 6.5),        # hull bosses, 12
    'deck_insert': (4.6, 4.6, 5.7),   # deck-lip pads, 8 (sunk deeper)
}
DRIVE_LINE = sorted(['axle', 'coupler', 'motor_shaft', 'grub', 'grub', 'grub'])
CORNERS = {'fl': (1, 1), 'fr': (1, -1), 'rl': (-1, 1), 'rr': (-1, -1)}


def load_stl(path):
    data = Path(path).read_bytes()
    n = int(np.frombuffer(data[80:84], '<u4')[0])
    rec = np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    return np.frombuffer(data[84:84 + 50 * n], rec)['v'].astype(np.float64)


def save_stl(path, tris):
    tris = tris.astype('<f4')
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(tris), np.dtype(
        [('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
    rec['n'], rec['v'] = n, tris
    with open(path, 'wb') as f:
        f.write(b'Mecanum_01 Rev D split'.ljust(80, b' '))
        f.write(np.uint32(len(tris)).tobytes())
        f.write(rec.tobytes())


def components(tris):
    q = np.round(tris.reshape(-1, 3) * 1e6).astype(np.int64)
    _, vid = np.unique(q, axis=0, return_inverse=True)
    t = len(tris)
    rows = np.repeat(np.arange(t), 3)
    cols = t + vid.ravel()
    size = t + vid.max() + 1
    graph = coo_matrix((np.ones(len(rows)), (rows, cols)), shape=(size, size))
    _, label = connected_components(graph, directed=False)
    label = label[:t]
    return [tris[label == k] for k in np.unique(label)]


def simplify(tris, cell):
    """Vertex clustering: merge vertices sharing a grid cell, drop slivers."""
    pts = tris.reshape(-1, 3)
    key = np.floor(pts / cell).astype(np.int64)
    _, cid = np.unique(key, axis=0, return_inverse=True)
    cid = cid.ravel()
    mean = np.zeros((cid.max() + 1, 3))
    np.add.at(mean, cid, pts)
    mean /= np.bincount(cid)[:, None]
    f = cid.reshape(-1, 3)
    keep = (f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])
    f = f[keep]
    _, first = np.unique(np.sort(f, axis=1), axis=0, return_index=True)
    return mean[f[np.sort(first)]]


def signature(part):
    pts = part.reshape(-1, 3)
    dims = np.sort(pts.max(0) - pts.min(0)) * 1e3
    for name, sig in SIG.items():
        if np.all(np.abs(dims - sig) < TOL * 1e3):
            return name
    return None


def classify(part):
    """Return (corner, 'wheel'|'drive') or (None, 'drop'|'body')."""
    pts = part.reshape(-1, 3)
    name = signature(part)
    if name in ('bearing', 'insert', 'deck_insert'):
        return None, 'drop'
    for corner, (sx, sy) in CORNERS.items():
        y = pts[:, 1] * sy
        r = np.hypot(pts[:, 0] - sx * WHEEL_X, pts[:, 2] - WHEEL_Z).max()
        if y.min() >= GEARBOX_FACE_Y and r <= DRIVE_R:
            return corner, 'drive'
        if y.min() >= WHEEL_INNER_Y and r <= WHEEL_R:
            return corner, 'wheel'
    return None, 'body'


def save_simplified(path, parts, offset):
    """Simplify each part on its own scale, so small screws keep their shape."""
    out = []
    for part in parts:
        pts = part.reshape(-1, 3)
        size = (pts.max(0) - pts.min(0)).max()
        out.append(simplify(part, float(np.clip(size / 15, 0.3e-3, 1.5e-3))))
    mesh = np.concatenate(out) - offset
    save_stl(path, mesh)
    return len(mesh)


def main(rev_dir, out_dir):
    tris = load_stl(Path(rev_dir) / 'ros2' / 'mecanum_01_full.stl')
    parts = components(tris)
    assert sum(len(p) for p in parts) == len(tris)

    groups = {g: [] for g in ('body', *CORNERS)}
    drive = {c: [] for c in CORNERS}
    dropped = 0
    for part in parts:
        corner, kind = classify(part)
        if kind == 'drop':
            dropped += 1
            continue
        if kind == 'drive':
            name = signature(part)
            assert name, f'{corner}: unknown part on the axle line'
            drive[corner].append(name)
        groups[corner or 'body'].append(part)

    for c in CORNERS:
        assert sorted(drive[c]) == DRIVE_LINE, f'{c} drive line: {drive[c]}'
    assert dropped == 8 + 20, f'expected 8 bearings + 20 inserts, got {dropped}'
    raw_counts = {sum(len(p) for p in groups[c]) for c in CORNERS}
    assert len(raw_counts) <= 2, f'corners differ beyond hand A/B: {raw_counts}'

    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    for c, (sx, sy) in CORNERS.items():
        pts = np.concatenate(groups[c]).reshape(-1, 3)
        # Wheel centre across the roller envelope, checked against the README.
        rim = pts[np.hypot(pts[:, 0] - sx * WHEEL_X, pts[:, 2] - WHEEL_Z)
                  > 0.025]
        lo, hi = rim.min(0), rim.max(0)
        centre = np.array([sx * WHEEL_X, sy * WHEEL_Y, WHEEL_Z])
        diameter = 2 * np.hypot(rim[:, 0] - centre[0],
                                rim[:, 2] - centre[2]).max()
        assert abs(diameter - 0.0599) < 0.3e-3, f'{c} diameter {diameter}'
        assert abs((hi[1] - lo[1]) - 0.0315) < 0.3e-3, f'{c} wheel width'
        assert abs((lo[1] + hi[1]) / 2 - centre[1]) < 0.3e-3, f'{c} track'
        n = save_simplified(out / f'wheel_{c}.stl', groups[c], centre)
        print(f'wheel_{c}: {n} tris, centre {np.round(centre, 5)}')

    n = save_simplified(out / 'body.stl', groups['body'], np.zeros(3))
    print(f'body: {n} tris; {len(parts)} parts, {dropped} dropped')


if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
