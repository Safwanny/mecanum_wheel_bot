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
    assert len([name for name in link_names if '_roller_' in name]) == 40
    assert len([name for name in joint_names if '_roller_' in name]) == 40

    assert_fixed_child(joints, 'lidar_link', 'base_link')
    assert_fixed_child(joints, 'imu_link', 'base_link')
    assert_fixed_child(joints, 'camera_link', 'base_link')
    assert_fixed_child(joints, 'camera_optical_frame', 'camera_link')

    link_by_name = {link.attrib['name']: link for link in links}
    joint_by_name = {joint.attrib['name']: joint for joint in joints}
    chassis = link_by_name['base_link']
    imu_link = link_by_name['imu_link']
    lidar_link = link_by_name['lidar_link']

    chassis_size = tuple(
        float(value)
        for value in chassis.find('./visual/geometry/box').attrib['size'].split()
    )
    imu_size = tuple(
        float(value)
        for value in imu_link.find('./visual/geometry/box').attrib['size'].split()
    )
    lidar_cylinder = lidar_link.find('./visual/geometry/cylinder')
    lidar_radius = float(lidar_cylinder.attrib['radius'])
    lidar_height = float(lidar_cylinder.attrib['length'])

    assert chassis_size == (0.216, 0.099, 0.050)
    assert imu_size == (0.050, 0.050, 0.008)
    assert math.isclose(lidar_radius, 0.020, abs_tol=1e-12)
    assert math.isclose(lidar_height, 0.008, abs_tol=1e-12)

    chassis_origin = xyz(chassis.find('./visual/origin'))
    imu_origin = xyz(imu_link.find('./visual/origin'))
    lidar_origin = xyz(lidar_link.find('./visual/origin'))
    imu_joint_origin = xyz(
        joint_by_name['base_link_to_imu_link_joint'].find('origin')
    )
    lidar_joint_origin = xyz(
        joint_by_name['base_link_to_lidar_link_joint'].find('origin')
    )

    assert chassis_origin == (0.0, 0.0, 0.0)
    assert imu_joint_origin[:2] == (0.0, 0.0)
    assert lidar_joint_origin[:2] == (0.0, 0.0)

    chassis_top = chassis_origin[2] + chassis_size[2] / 2.0
    imu_bottom = imu_joint_origin[2] + imu_origin[2] - imu_size[2] / 2.0
    imu_top = imu_joint_origin[2] + imu_origin[2] + imu_size[2] / 2.0
    lidar_bottom = (
        lidar_joint_origin[2] + lidar_origin[2] - lidar_height / 2.0
    )
    lidar_top = (
        lidar_joint_origin[2] + lidar_origin[2] + lidar_height / 2.0
    )

    assert math.isclose(imu_bottom, chassis_top, abs_tol=1e-12)
    assert math.isclose(lidar_bottom, imu_top, abs_tol=1e-12)
    assert math.isclose(lidar_top, lidar_joint_origin[2], abs_tol=1e-12)
    assert lidar_joint_origin[2] > chassis_top

    for sensor_link in (imu_link, lidar_link):
        assert xyz(sensor_link.find('./visual/origin')) == xyz(
            sensor_link.find('./collision/origin')
        )

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
