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

"""Validate the canonical four-wheel mecanum URDF."""

import math
from pathlib import Path
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET


WHEELS = {
    'front_left': ((0.075, 0.067), 'mecanum_wheel_FL.stl'),
    'front_right': ((0.075, -0.067), 'mecanum_wheel_FR.stl'),
    'rear_right': ((-0.075, -0.067), 'mecanum_wheel_RR.stl'),
    'rear_left': ((-0.075, 0.067), 'mecanum_wheel_RL.stl'),
}
RADIUS = 0.03074443
WIDTH = 0.03360543
MASS = 0.12


def vector(text):
    return tuple(float(value) for value in text.split())


def load_robot(xacro_file, prefix=''):
    result = subprocess.run(
        ['xacro', str(xacro_file), f'prefix:={prefix}'],
        check=True, capture_output=True, text=True)
    return result.stdout, ET.fromstring(result.stdout)


def validate(xacro_file):
    text, robot = load_robot(xacro_file)
    links = {link.attrib['name']: link for link in robot.findall('link')}
    joints = {joint.attrib['name']: joint for joint in robot.findall('joint')}

    assert len(links) == len(robot.findall('link'))
    assert len(joints) == len(robot.findall('joint'))
    assert not [name for name in links if '_roller_' in name]
    assert not [name for name in joints if '_roller_' in name]

    for wheel, (position, mesh_file) in WHEELS.items():
        link_name = f'{wheel}_wheel_link'
        joint_name = f'{wheel}_wheel_joint'
        link = links[link_name]
        joint = joints[joint_name]

        visuals = link.findall('visual')
        assert len(visuals) == 1
        uri = visuals[0].find('geometry/mesh').attrib['filename']
        assert uri.endswith('/' + mesh_file)
        assert visuals[0].find('geometry/mesh').attrib['scale'] == \
            '0.001 0.001 0.001'

        collisions = link.findall('collision')
        assert len(collisions) == 1
        assert collisions[0].attrib['name'] == 'wheel_collision'
        cylinder = collisions[0].find('geometry/cylinder')
        assert cylinder is not None
        assert math.isclose(float(cylinder.attrib['radius']), RADIUS)
        assert math.isclose(float(cylinder.attrib['length']), WIDTH)
        assert vector(collisions[0].find('origin').attrib['rpy']) == \
            (math.pi / 2.0, 0.0, 0.0)

        inertial = link.find('inertial')
        assert math.isclose(float(inertial.find('mass').attrib['value']), MASS)
        inertia = inertial.find('inertia')
        for field in ('ixx', 'iyy', 'izz'):
            assert math.isfinite(float(inertia.attrib[field]))
            assert float(inertia.attrib[field]) > 0.0

        assert joint.attrib['type'] == 'continuous'
        assert joint.find('parent').attrib['link'] == 'base_link'
        assert joint.find('child').attrib['link'] == link_name
        origin = vector(joint.find('origin').attrib['xyz'])
        assert origin[:2] == position
        assert vector(joint.find('axis').attrib['xyz']) == (0.0, 1.0, 0.0)
        dynamics = joint.find('dynamics')
        assert float(dynamics.attrib['damping']) == 0.001
        assert float(dynamics.attrib['friction']) == 0.001

    wheel_links = [name for name in links if name.endswith('_wheel_link')]
    wheel_joints = [name for name in joints if name.endswith('_wheel_joint')]
    assert len(wheel_links) == 4
    assert len(wheel_joints) == 4

    control_joints = robot.findall('ros2_control/joint')
    assert [joint.attrib['name'] for joint in control_joints] == [
        'front_left_wheel_joint', 'front_right_wheel_joint',
        'rear_right_wheel_joint', 'rear_left_wheel_joint']
    for joint in control_joints:
        assert [item.attrib['name'] for item in joint.findall('command_interface')] \
            == ['velocity']
        assert [item.attrib['name'] for item in joint.findall('state_interface')] \
            == ['position', 'velocity']

    with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf') as urdf_file:
        urdf_file.write(text)
        urdf_file.flush()
        subprocess.run(
            ['check_urdf', urdf_file.name], check=True,
            capture_output=True, text=True)

    _, prefixed = load_robot(xacro_file, prefix='robot_')
    assert prefixed.find("link[@name='robot_front_left_wheel_link']") is not None
    assert prefixed.find("joint[@name='robot_front_left_wheel_joint']") is not None


if __name__ == '__main__':
    validate(Path(sys.argv[1]))
    print('PASS canonical mecanum wheel description')
