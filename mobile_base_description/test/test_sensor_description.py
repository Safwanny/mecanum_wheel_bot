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

"""Validate the sensor URDF, fixed-frame tree, and Gazebo contracts."""

import math
from pathlib import Path
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET


WHEEL_JOINTS = {
    'front_left_wheel_joint',
    'front_right_wheel_joint',
    'rear_right_wheel_joint',
    'rear_left_wheel_joint',
}


def generated_robot(xacro_path, use_gazebo):
    text = subprocess.run(
        [
            'xacro',
            str(xacro_path),
            f'use_gazebo:={str(use_gazebo).lower()}',
            'controllers_file:=unused.yaml',
        ],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    return text, ET.fromstring(text)


def assert_unique(elements):
    names = [element.attrib['name'] for element in elements]
    assert len(names) == len(set(names)), 'duplicate names: ' + str(names)


def assert_fixed_child(joints, child, expected_parent):
    parents = [
        joint for joint in joints
        if joint.find('child').attrib['link'] == child
    ]
    assert len(parents) == 1
    joint = parents[0]
    assert joint.attrib['type'] == 'fixed'
    assert joint.find('parent').attrib['link'] == expected_parent


def xyz(element):
    if element is None:
        return (0.0, 0.0, 0.0)
    return tuple(float(value) for value in element.attrib['xyz'].split())


def main():
    xacro_path = Path(sys.argv[1])
    urdf_text, robot = generated_robot(xacro_path, use_gazebo=False)
    links = robot.findall('link')
    joints = robot.findall('joint')
    assert_unique(links)
    assert_unique(joints)

    link_names = {link.attrib['name'] for link in links}
    joint_names = {joint.attrib['name'] for joint in joints}
    for required in (
        'base_footprint',
        'base_link',
        'lidar_link',
        'imu_link',
        'camera_link',
        'camera_optical_frame',
    ):
        assert required in link_names
    assert WHEEL_JOINTS <= joint_names
    assert not [name for name in link_names if '_roller_' in name]
    assert not [name for name in joint_names if '_roller_' in name]

    assert_fixed_child(joints, 'lidar_link', 'base_link')
    assert_fixed_child(joints, 'imu_link', 'base_link')
    assert_fixed_child(joints, 'camera_link', 'base_link')
    assert_fixed_child(joints, 'camera_optical_frame', 'camera_link')

    link_by_name = {link.attrib['name']: link for link in links}
    joint_by_name = {joint.attrib['name']: joint for joint in joints}
    chassis = link_by_name['base_link']
    imu_link = link_by_name['imu_link']
    lidar_link = link_by_name['lidar_link']

    # base_link is a dual-deck plate frame, not a solid block: one visual per
    # printed part, and collision boxes bounding all of them. The planner must
    # never see the open electronics bay as passable.
    #
    # Each deck is the full body rectangle with the four wheel wells cut out,
    # built from five butted boxes: a central spine, a full-width end cap at
    # each end, and a wing per side between that side's two wheels. So a deck
    # is five visuals, not one.
    BASE_LINK_HEIGHT = 0.040372215
    WHEELBASE = 0.150
    WHEEL_RADIUS = 0.03074443
    # Widest point of the body, reached by both the caps and the wings.
    OVERALL_WIDTH = 0.16760543
    SPINE = (0.21948886, 0.100)
    CAP = (0.02025557, OVERALL_WIDTH)
    WING = (0.08051114, 0.033802715)

    def ground(z):
        return z + BASE_LINK_HEIGHT

    def z_extent(element, parent_z=0.0):
        """Return the ground-referenced (bottom, top) of one element."""
        # parent_z is the height of the link the element belongs to, measured
        # from base_link, so sensor geometry resolves to the same datum as
        # the body geometry.
        centre = ground(parent_z + xyz(element.find('origin'))[2])
        box = element.find('./geometry/box')
        if box is not None:
            height = float(box.attrib['size'].split()[2])
        else:
            height = float(element.find('./geometry/cylinder').attrib['length'])
        return centre - height / 2.0, centre + height / 2.0

    chassis_visuals = chassis.findall('visual')

    def boxes_of_footprint(visuals, footprint):
        # Derived xacro dimensions carry float noise in the last bits, so
        # footprints are matched to a micrometre rather than exactly.
        def matches(visual):
            box = visual.find('./geometry/box')
            if box is None:
                return False
            size = [float(value) for value in box.attrib['size'].split()]
            return all(
                math.isclose(actual, expected, abs_tol=1e-6)
                for actual, expected in zip(size[:2], footprint)
            )

        return [visual for visual in visuals if matches(visual)]

    decks = boxes_of_footprint(chassis_visuals, SPINE)
    assert len(decks) == 2, 'expected exactly two deck spines'
    deck_heights = {round(z_extent(deck)[0], 9) for deck in decks}

    def mirrored_pairs(elements, axis):
        offsets = sorted(xyz(e.find('origin'))[axis] for e in elements)
        assert math.isclose(offsets[0], -offsets[3], abs_tol=1e-9)
        assert math.isclose(offsets[1], -offsets[2], abs_tol=1e-9)

    # Two wings per deck, one either side, at matching heights.
    wings = boxes_of_footprint(chassis_visuals, WING)
    assert len(wings) == 4, 'expected two wings on each of the two decks'
    mirrored_pairs(wings, 1)
    # The wings reach the wheel outer faces and no further.
    for wing in wings:
        half_span = abs(xyz(wing.find('origin'))[1]) + WING[1] / 2.0
        assert math.isclose(2.0 * half_span, OVERALL_WIDTH, abs_tol=1e-6)

    # Two end caps per deck, fore and aft. Each runs the full body width, so
    # nothing can strike a wheel head on without hitting deck first.
    caps = boxes_of_footprint(chassis_visuals, CAP)
    assert len(caps) == 4, 'expected an end cap at each end of both decks'
    mirrored_pairs(caps, 0)
    for cap in caps:
        assert math.isclose(
            xyz(cap.find('origin'))[1], 0.0, abs_tol=1e-9), (
            'an end cap is off the centreline')

    # Every limb shares a deck's height, so they are parts of the plates
    # rather than floating slabs.
    for limb in wings + caps:
        assert round(z_extent(limb)[0], 9) in deck_heights
    deck_thicknesses = {
        float(visual.find('./geometry/box').attrib['size'].split()[2])
        for visual in decks
    }
    assert deck_thicknesses == {0.010}, 'both plates print at the same 10 mm'
    lower_deck, upper_deck = sorted(decks, key=lambda v: z_extent(v)[0])
    lower_bottom, lower_top = z_extent(lower_deck)
    upper_bottom, upper_top = z_extent(upper_deck)

    # The decks must not intersect, and the bay between them has to be tall
    # enough for the motors that stand flat on the lower deck.
    assert lower_top < upper_bottom
    assert upper_bottom - lower_top >= 0.0224

    # Ground clearance is not free: it is the wheel radius minus the motor
    # shaft height above its resting face, minus the plate thickness.
    assert 0.008 < lower_bottom < 0.012

    # Collision follows the same outline the visuals do: one box per limb. A
    # single box over the whole envelope would reach the wheel outer faces
    # along the entire body length and swallow the wheels.
    chassis_collisions = chassis.findall('collision')
    assert len(chassis_collisions) == 5, 'body collision is one box per limb'
    spine_collisions = boxes_of_footprint(chassis_collisions, SPINE)
    cap_collisions = boxes_of_footprint(chassis_collisions, CAP)
    wing_collisions = boxes_of_footprint(chassis_collisions, WING)
    assert len(spine_collisions) == 1
    assert len(cap_collisions) == 2
    assert len(wing_collisions) == 2

    # Every limb spans the full body height, so nothing routes over or under.
    for collision in chassis_collisions:
        bottom, top = z_extent(collision)
        assert math.isclose(bottom, lower_bottom, abs_tol=1e-9)
        assert math.isclose(top, upper_top, abs_tol=1e-9)

    def footprint(element, size):
        centre = xyz(element.find('origin'))
        return (
            (centre[0] - size[0] / 2.0, centre[0] + size[0] / 2.0),
            (centre[1] - size[1] / 2.0, centre[1] + size[1] / 2.0),
        )

    def box_size(element):
        return [
            float(value)
            for value in element.find('./geometry/box').attrib['size'].split()
        ]

    collision_footprints = [
        footprint(collision, box_size(collision))
        for collision in chassis_collisions
    ]

    # The collision set must actually bound the body: every visual lies inside
    # the height, and every box visual's footprint inside some collision limb.
    for visual in chassis_visuals:
        bottom, top = z_extent(visual)
        assert bottom >= lower_bottom - 1e-9
        assert top <= upper_top + 1e-9
        if visual.find('./geometry/box') is None:
            continue
        (vx0, vx1), (vy0, vy1) = footprint(visual, box_size(visual))
        assert any(
            cx0 - 1e-9 <= vx0 and vx1 <= cx1 + 1e-9
            and cy0 - 1e-9 <= vy0 and vy1 <= cy1 + 1e-9
            for (cx0, cx1), (cy0, cy1) in collision_footprints
        ), 'a deck visual escapes the collision cross'

    # The four wheel wells must stay open. A wheel spans wheel_x_inner to
    # wheel_x_outer along the body at the outboard Y band, so a wing that
    # reached past its inner bound, or a cap that reached past its outer one,
    # would sit inside a rotating wheel.
    wheel_x_inner = WHEELBASE / 2.0 - WHEEL_RADIUS
    wheel_x_outer = WHEELBASE / 2.0 + WHEEL_RADIUS
    for collision in wing_collisions:
        (wx0, wx1), _ = footprint(collision, box_size(collision))
        assert max(abs(wx0), abs(wx1)) < wheel_x_inner
    for collision in cap_collisions:
        (cx0, cx1), _ = footprint(collision, box_size(collision))
        assert min(abs(cx0), abs(cx1)) > wheel_x_outer

    # The caps span the full body width: that is what puts deck in front of
    # and behind each wheel rather than leaving the wheel exposed end on.
    for collision in cap_collisions:
        _, (cy0, cy1) = footprint(collision, box_size(collision))
        assert math.isclose(cy1 - cy0, OVERALL_WIDTH, abs_tol=1e-6)

    # Each motor stands on the lower deck and shares its wheel's joint origin,
    # so the motor frame sits on the output shaft it drives. Motors carry no
    # collision: the body box covers them, and an extra collision would break
    # the one-surface-per-wheel contract the Gazebo SDF generator relies on.
    motor_links = sorted(
        name for name in link_names if name.endswith('_motor_link')
    )
    assert len(motor_links) == 4
    for motor in motor_links:
        wheel = motor.replace('_motor_link', '_wheel_joint')
        motor_joint = motor.replace('_link', '_joint')
        assert_fixed_child(joints, motor, 'base_link')
        assert not link_by_name[motor].findall('collision')
        assert xyz(joint_by_name[motor_joint].find('origin')) == xyz(
            joint_by_name[wheel].find('origin')
        )

    # The deck components are fixed-joint children of base_link whose link
    # origin is the part's own centroid, so the Gazebo reduction lumps each in
    # with the right parallel-axis term and every component frame is where its
    # mass actually is. None carries collision: the chassis cross already
    # bounds the bay they sit in.
    LOWER_DECK_SURFACE = 0.01954443
    UPPER_DECK_SURFACE = 0.06954443
    COMPONENTS = {
        'battery_link': (LOWER_DECK_SURFACE, 0.025, 0.250),
        'left_motor_driver_link': (LOWER_DECK_SURFACE, 0.0117, 0.003),
        'right_motor_driver_link': (LOWER_DECK_SURFACE, 0.0117, 0.003),
        'mcu_link': (UPPER_DECK_SURFACE, 0.0122, 0.012),
    }
    for name, (surface, height, mass) in COMPONENTS.items():
        assert name in link_names, f'{name} is missing'
        assert_fixed_child(joints, name, 'base_link')
        component = link_by_name[name]
        assert not component.findall('collision')

        inertial = component.find('inertial')
        assert math.isclose(float(inertial.find('mass').attrib['value']), mass)
        # The inertial sits at the link origin, which is the centroid.
        assert xyz(inertial.find('origin')) == (0.0, 0.0, 0.0)

        # The part rests on its deck: centroid one half-height above it.
        centre = ground(xyz(joint_by_name[
            name.replace('_link', '_joint')].find('origin'))[2])
        assert math.isclose(centre - height / 2.0, surface, abs_tol=1e-6), (
            f'{name} does not sit on its deck')

    # The drivers are mirrored about the centreline and flank the battery.
    driver_y = [
        xyz(joint_by_name[f'{side}_motor_driver_joint'].find('origin'))[1]
        for side in ('left', 'right')
    ]
    assert math.isclose(driver_y[0], -driver_y[1], abs_tol=1e-9)
    assert min(abs(y) for y in driver_y) > 0.047 / 2.0

    # base_link's mass after the Gazebo reduction lumps in every fixed-joint
    # child. The deck plates carry the balance, so this total is what
    # total_body_mass in properties.xacro declares.
    joint_by_child = {
        joint.find('child').attrib['link']: joint for joint in joints
    }

    def lumps_into_base_link(name):
        """Report whether every joint up to base_link is fixed."""
        while name != 'base_link':
            joint = joint_by_child.get(name)
            if joint is None or joint.attrib['type'] != 'fixed':
                return False
            name = joint.find('parent').attrib['link']
        return True

    lumped = float(chassis.find('./inertial/mass').attrib['value'])
    for link in links:
        name = link.attrib['name']
        inertial = link.find('inertial')
        if name == 'base_link' or inertial is None:
            continue
        if lumps_into_base_link(name):
            lumped += float(inertial.find('mass').attrib['value'])
    assert math.isclose(lumped, 1.80, abs_tol=1e-9), (
        f'lumped body mass is {lumped}, expected total_body_mass 1.80')

    imu_size = tuple(
        float(value)
        for value in imu_link.find('./visual/geometry/box').attrib['size'].split()
    )
    # Matches imu_length/imu_width/imu_height in properties.xacro.
    assert imu_size == (0.025, 0.025, 0.004)

    imu_joint_origin = xyz(
        joint_by_name['base_link_to_imu_link_joint'].find('origin')
    )
    lidar_joint_origin = xyz(
        joint_by_name['base_link_to_lidar_link_joint'].find('origin')
    )
    camera_joint_origin = xyz(
        joint_by_name['base_link_to_camera_link_joint'].find('origin')
    )
    assert imu_joint_origin[:2] == (0.0, 0.0)
    assert lidar_joint_origin[:2] == (0.0, 0.0)

    # The IMU hangs under the upper deck, inside the shielded bay.
    imu_origin = xyz(imu_link.find('./visual/origin'))
    imu_centre = ground(imu_joint_origin[2] + imu_origin[2])
    imu_top = imu_centre + imu_size[2] / 2.0
    assert math.isclose(imu_top, upper_bottom, abs_tol=1e-9)

    # The LiDAR is a D500-style square base carrying a cylindrical rotating
    # head, and lidar_link is the scan plane at mid height of that head.
    lidar_visuals = lidar_link.findall('visual')
    assert len(lidar_visuals) == 2
    base_visual = [
        visual for visual in lidar_visuals
        if visual.find('./geometry/box') is not None
    ]
    head_visual = [
        visual for visual in lidar_visuals
        if visual.find('./geometry/cylinder') is not None
    ]
    assert len(base_visual) == 1 and len(head_visual) == 1
    base_size = tuple(
        float(value)
        for value in base_visual[0].find('./geometry/box').attrib['size'].split()
    )
    head_radius = float(
        head_visual[0].find('./geometry/cylinder').attrib['radius']
    )
    # The base is square in plan and wider than the head it carries.
    assert base_size[0] == base_size[1]
    assert base_size[0] / 2.0 > head_radius
    base_bottom, base_top = z_extent(base_visual[0], lidar_joint_origin[2])
    head_bottom, head_top = z_extent(head_visual[0], lidar_joint_origin[2])

    # The sensor stands on the upper deck and the head stacks on its base.
    assert math.isclose(base_bottom, upper_top, abs_tol=1e-9)
    assert math.isclose(head_bottom, base_top, abs_tol=1e-9)
    # The scan plane bisects the head, and clears the whole body.
    scan_plane = ground(lidar_joint_origin[2])
    assert math.isclose(scan_plane, (head_bottom + head_top) / 2.0, abs_tol=1e-9)
    assert scan_plane > upper_top

    # One collision box spans the whole sensor, base and head together.
    lidar_collisions = lidar_link.findall('collision')
    assert len(lidar_collisions) == 1
    lidar_collision_bottom, lidar_collision_top = z_extent(
        lidar_collisions[0], lidar_joint_origin[2])
    assert math.isclose(lidar_collision_bottom, base_bottom, abs_tol=1e-9)
    assert math.isclose(lidar_collision_top, head_top, abs_tol=1e-9)

    # The camera is mounted on the front face of the upper deck, looking
    # forward, centred on the plate's thickness.
    assert camera_joint_origin[0] > 0.0
    assert camera_joint_origin[1] == 0.0
    camera_height = ground(camera_joint_origin[2])
    assert math.isclose(
        camera_height, (upper_bottom + upper_top) / 2.0, abs_tol=1e-9)

    parent_by_child = {
        joint.find('child').attrib['link']:
        joint.find('parent').attrib['link']
        for joint in joints
    }
    for sensor_frame in ('lidar_link', 'imu_link'):
        current = sensor_frame
        visited = set()
        while current != 'base_link':
            assert current not in visited
            visited.add(current)
            current = parent_by_child[current]

    with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf') as urdf_file:
        urdf_file.write(urdf_text)
        urdf_file.flush()
        subprocess.run(
            ['check_urdf', urdf_file.name],
            check=True,
            capture_output=True,
            text=True,
        )

    _, gazebo_robot = generated_robot(xacro_path, use_gazebo=True)
    sensors = {
        sensor.attrib['name']: sensor
        for sensor in gazebo_robot.findall('.//sensor')
    }
    assert set(sensors) == {'camera', 'lidar', 'imu'}

    lidar = sensors['lidar']
    assert lidar.attrib['type'] == 'gpu_lidar'
    assert lidar.findtext('topic') == '/scan'
    assert lidar.findtext('gz_frame_id') == 'lidar_link'
    assert int(lidar.findtext('lidar/scan/horizontal/samples')) == 720
    assert math.isclose(
        float(lidar.findtext('lidar/scan/horizontal/min_angle')),
        -math.pi,
        abs_tol=1e-9,
    )
    assert math.isclose(
        float(lidar.findtext('lidar/scan/horizontal/max_angle')),
        math.pi,
        abs_tol=1e-9,
    )
    assert float(lidar.findtext('lidar/range/min')) == 0.10
    assert float(lidar.findtext('lidar/range/max')) == 4.0
    assert float(lidar.findtext('update_rate')) == 10.0
    assert float(lidar.findtext('lidar/noise/stddev')) == 0.01

    imu = sensors['imu']
    assert imu.attrib['type'] == 'imu'
    assert imu.findtext('topic') == '/imu/data'
    assert imu.findtext('gz_frame_id') == 'imu_link'
    assert float(imu.findtext('update_rate')) == 50.0
    for noise in imu.findall('.//noise'):
        assert noise.attrib['type'] == 'gaussian'
        assert math.isfinite(float(noise.findtext('mean')))
        assert float(noise.findtext('stddev')) > 0.0


if __name__ == '__main__':
    main()
