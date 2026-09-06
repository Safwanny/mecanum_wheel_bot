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
    # printed part, and a single collision box bounding all of them. The
    # planner must never see the open electronics bay as passable.
    BASE_LINK_HEIGHT = 0.040372215
    FOOTPRINT = (0.216, 0.100)

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
    decks = [
        visual for visual in chassis_visuals
        if visual.find('./geometry/box') is not None
        and tuple(
            float(value)
            for value in visual.find('./geometry/box').attrib['size'].split()
        )[:2] == FOOTPRINT
    ]
    assert len(decks) == 2, 'expected exactly two deck plates'
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

    chassis_collisions = chassis.findall('collision')
    assert len(chassis_collisions) == 1, 'body collision must be one box'
    collision = chassis_collisions[0]
    collision_size = tuple(
        float(value)
        for value in collision.find('./geometry/box').attrib['size'].split()
    )
    assert collision_size[:2] == FOOTPRINT
    collision_bottom, collision_top = z_extent(collision)

    # The bounding box must actually bound: every visual part lies inside it.
    assert math.isclose(collision_bottom, lower_bottom, abs_tol=1e-9)
    assert math.isclose(collision_top, upper_top, abs_tol=1e-9)
    for visual in chassis_visuals:
        bottom, top = z_extent(visual)
        assert bottom >= collision_bottom - 1e-9
        assert top <= collision_top + 1e-9

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

    imu_size = tuple(
        float(value)
        for value in imu_link.find('./visual/geometry/box').attrib['size'].split()
    )
    assert imu_size == (0.050, 0.050, 0.008)

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
    assert scan_plane > collision_top

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
