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

"""Generate the deck plate mesh from the description's own properties."""

# The deck outline is the body rectangle with the four wheel wells cut out and
# the four bumper corners chamfered at 45 degrees from the wheel line. URDF has
# no prism primitive, so the outline cannot be built from boxes the way the
# collision limbs are - it has to be a mesh.
#
# A generated mesh can drift from the properties it was generated from, so the
# dimensions are read back through xacro rather than duplicated here, and
# --check regenerates and compares so a stale mesh fails the test suite.
#
# Both decks are identical, so this emits one mesh used twice.

import argparse
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET


# Read straight out of the description rather than restating the derivations:
# whatever the URDF builds from is what the mesh is built from.
PROBE = """<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="deck_probe">
  <xacro:include filename="{properties}"/>
  <deck_outline
    half_length="${{chassis_length / 2.0}}"
    spine_half_width="${{chassis_width / 2.0}}"
    cap_inner_x="${{cap_inner_x}}"
    cap_length="${{cap_length}}"
    wing_half_length="${{wing_length / 2.0}}"
    wheel_outer_y="${{wheel_outer_y}}"
    thickness="${{deck_thickness}}"/>
</robot>
"""

MM = 1000.0


def read_dimensions(properties):
    """Evaluate the deck dimensions with xacro itself."""
    with tempfile.NamedTemporaryFile(
            mode='w', suffix='.xacro', dir=properties.parent) as probe:
        probe.write(PROBE.format(properties=properties.name))
        probe.flush()
        expanded = subprocess.run(
            ['xacro', probe.name], check=True, capture_output=True, text=True)
    outline = ET.fromstring(expanded.stdout).find('deck_outline')
    return {name: float(value) for name, value in outline.attrib.items()}


def deck_outline(d):
    """Return the deck outline polygon, counter-clockwise, in metres."""
    # The chamfer runs at 45 degrees, so it eats one cap_length of width over
    # the cap's own depth. Vertices are listed from the front edge round the
    # left side to the rear, then back along the right.
    half_length = d['half_length']
    spine_half_width = d['spine_half_width']
    cap_inner_x = d['cap_inner_x']
    outer_y = d['wheel_outer_y']
    wing_half_length = d['wing_half_length']
    # Half width of the bumper edge once the 45 degree chamfer has run its
    # course across the depth of the cap.
    nose_half_width = outer_y - d['cap_length']

    if nose_half_width <= spine_half_width:
        raise RuntimeError(
            f'a 45 degree chamfer over a {d["cap_length"]:.4f} m cap '
            f'cuts past the spine width; the bumper would come to a '
            f'point. Shorten chassis_length or widen the spine.')

    def side(sign):
        """One side of the outline, from the front cap round to the rear."""
        return [
            (cap_inner_x, sign * outer_y),
            (cap_inner_x, sign * spine_half_width),
            (wing_half_length, sign * spine_half_width),
            (wing_half_length, sign * outer_y),
            (-wing_half_length, sign * outer_y),
            (-wing_half_length, sign * spine_half_width),
            (-cap_inner_x, sign * spine_half_width),
            (-cap_inner_x, sign * outer_y),
        ]

    def bumper(sign):
        """Return one bumper edge, split where the cap tiling splits it."""
        # Carrying those points means the walls and the cap faces share
        # their vertices instead of meeting at a T-junction.
        y = [-nose_half_width, -spine_half_width,
             spine_half_width, nose_half_width]
        return [(sign * half_length, value)
                for value in (y if sign > 0 else list(reversed(y)))]

    return (
        bumper(1.0) + side(1.0) + bumper(-1.0) + list(reversed(side(-1.0)))
    )


def deck_faces(d):
    """Tile the outline with quads that share their vertices exactly."""
    # The obvious tiling - one piece per limb - would leave T-junctions where a
    # long edge meets a short one, because the spine's edge runs further than
    # the wing's that butts onto it. The surface would still render but would
    # not be watertight, and T-junctions show as hairline seams. So the pieces
    # are split at every join line: the spine splits where the wings meet it,
    # and each cap splits where the spine meets it. Every interior edge is then
    # shared by exactly two quads, and every boundary edge is one outline edge.
    half_length = d['half_length']
    w = d['spine_half_width']
    cap_inner_x = d['cap_inner_x']
    outer_y = d['wheel_outer_y']
    wing = d['wing_half_length']
    nose = outer_y - d['cap_length']

    def rectangle(x0, x1, y0, y1):
        return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]

    def cap(sign):
        """One chamfered end cap, split where the spine butts onto it."""
        inner = sign * cap_inner_x
        outer = sign * half_length
        pieces = [rectangle(*sorted((inner, outer)), -w, w)]
        for edge in (1.0, -1.0):
            corners = [(inner, edge * w), (outer, edge * w),
                       (outer, edge * nose), (inner, edge * outer_y)]
            pieces.append(corners if sign * edge > 0 else corners[::-1])
        return pieces

    return [
        rectangle(-wing, wing, -w, w),                 # spine, centre
        rectangle(-cap_inner_x, -wing, -w, w),         # spine, rear
        rectangle(wing, cap_inner_x, -w, w),           # spine, front
        rectangle(-wing, wing, w, outer_y),            # left wing
        rectangle(-wing, wing, -outer_y, -w),          # right wing
    ] + cap(1.0) + cap(-1.0)


def build_triangles(d):
    """Return the closed surface of the extruded outline, in millimetres."""
    top = d['thickness'] / 2.0
    bottom = -top
    triangles = []

    # The same tiling top and bottom, wound opposite ways so both normals
    # point out of the plate.
    for piece in deck_faces(d):
        for i in range(1, len(piece) - 1):
            corners = [piece[0], piece[i], piece[i + 1]]
            triangles.append([(x, y, top) for x, y in corners])
            triangles.append([(x, y, bottom) for x, y in reversed(corners)])

    outline = deck_outline(d)

    # One wall quad per outline edge, wound so its normal points outward.
    for i, (x0, y0) in enumerate(outline):
        x1, y1 = outline[(i + 1) % len(outline)]
        triangles.append([(x0, y0, bottom), (x1, y1, bottom), (x1, y1, top)])
        triangles.append([(x0, y0, bottom), (x1, y1, top), (x0, y0, top)])

    return [[(x * MM, y * MM, z * MM) for x, y, z in t] for t in triangles]


def encode_stl(triangles):
    """Pack triangles into a binary STL, with a facet normal on each."""
    out = [b'deck plate, generated by generate_deck_mesh.py'.ljust(80, b' '),
           struct.pack('<I', len(triangles))]
    for a, b, c in triangles:
        u = [b[i] - a[i] for i in range(3)]
        v = [c[i] - a[i] for i in range(3)]
        n = (u[1] * v[2] - u[2] * v[1],
             u[2] * v[0] - u[0] * v[2],
             u[0] * v[1] - u[1] * v[0])
        length = sum(component * component for component in n) ** 0.5
        if length:
            n = tuple(component / length for component in n)
        out.append(struct.pack('<3f', *n))
        for corner in (a, b, c):
            out.append(struct.pack('<3f', *corner))
        out.append(struct.pack('<H', 0))
    return b''.join(out)


def main():
    """Write the deck mesh, or check the one on disk is up to date."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--properties', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument(
        '--check', action='store_true',
        help='fail if the mesh on disk is not what the properties produce')
    args = parser.parse_args()

    stl = encode_stl(build_triangles(read_dimensions(Path(args.properties))))
    output = Path(args.output)

    if args.check:
        if not output.is_file():
            print(f'{output} is missing; run this script without --check',
                  file=sys.stderr)
            return 1
        if output.read_bytes() != stl:
            print(f'{output} is stale: it does not match the current '
                  f'properties.xacro. Regenerate it with this script.',
                  file=sys.stderr)
            return 1
        return 0

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(stl)
    return 0


if __name__ == '__main__':
    sys.exit(main())
