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

import math
from pathlib import Path
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET


WHEEL_RADIUS = 0.03074443
ROLLER_CENTER_RADIUS = 0.02505
ROLLER_RADIUS = WHEEL_RADIUS - ROLLER_CENTER_RADIUS
ROLLER_LENGTH = 0.02580
WHEEL_MASS = 0.12
ROLLER_MASS = WHEEL_MASS * 0.03
HUB_MASS = WHEEL_MASS - 10 * ROLLER_MASS
WHEELBASE = 0.150
WHEEL_SEPARATION = 0.134
TOLERANCE = 1e-6
BARREL_SPHERES = {
    'barrel_center_collision': (0.0, 0.00569443),
    'barrel_negative_3mm_collision': (-0.003, 0.00558),
    'barrel_positive_3mm_collision': (0.003, 0.00558),
    'barrel_negative_5p8mm_collision': (-0.0058, 0.00526),
    'barrel_positive_5p8mm_collision': (0.0058, 0.00526),
    'barrel_negative_8p2mm_collision': (-0.0082, 0.0048),
    'barrel_positive_8p2mm_collision': (0.0082, 0.0048),
    'barrel_negative_10p8mm_collision': (-0.0108, 0.00405),
    'barrel_positive_10p8mm_collision': (0.0108, 0.00405),
}

WHEELS = {
    'front_left': {
        'mesh': 'mecanum_wheel_FL.stl',
        'visual_xyz': (0.105356784, 0.000987069, 0.084068176),
        'visual_rpy': (-math.pi / 2.0, -0.44387938, 0.0),
        'mesh_center_mm': (-131.249672, 30.676275, -0.987069),
        'inner_normal': (0.0, 0.0, -1.0),
        'mesh_handedness': 1,
        'position_xy': (WHEELBASE / 2.0, WHEEL_SEPARATION / 2.0),
        'handedness': -1,
        'phase': 0.22193969,
        'axial_offset': -0.00059729,
    },
    'front_right': {
        'mesh': 'mecanum_wheel_FR.stl',
        'visual_xyz': (0.050182544, 0.126865288, 0.125047948),
        'visual_rpy': (-math.pi / 2.0, -0.96060838, 0.0),
        'mesh_center_mm': (-131.237514, 30.528544, -126.865288),
        'inner_normal': (0.0, 0.0, 1.0),
        'mesh_handedness': -1,
        'position_xy': (WHEELBASE / 2.0, -WHEEL_SEPARATION / 2.0),
        'handedness': 1,
        'phase': 0.48030419,
        'axial_offset': 0.00059729,
    },
    'rear_right': {
        'mesh': 'mecanum_wheel_RR.stl',
        'visual_xyz': (-0.030863306, 0.126451466, 0.020233040),
        'visual_rpy': (-math.pi / 2.0, -0.39337164, 0.0),
        'mesh_center_mm': (20.750607, 30.517730, -126.451466),
        'inner_normal': (0.0, 0.0, 1.0),
        'mesh_handedness': 1,
        'position_xy': (-WHEELBASE / 2.0, -WHEEL_SEPARATION / 2.0),
        'handedness': -1,
        'phase': 0.19668582,
        'axial_offset': 0.00059729,
    },
    'rear_left': {
        'mesh': 'mecanum_wheel_RL.stl',
        'visual_xyz': (-0.032786922, 0.001235564, 0.016969848),
        'visual_rpy': (-math.pi / 2.0, -0.49581568, 0.0),
        'mesh_center_mm': (20.765359, 30.524709, -1.235564),
        'inner_normal': (0.0, 0.0, -1.0),
        'mesh_handedness': -1,
        'position_xy': (-WHEELBASE / 2.0, WHEEL_SEPARATION / 2.0),
        'handedness': 1,
        'phase': 0.24790784,
        'axial_offset': -0.00059729,
    },
}


def vector(text):
    return tuple(float(value) for value in text.split())


def assert_vector_close(actual, expected, tolerance=TOLERANCE):
    assert len(actual) == len(expected)
    for actual_value, expected_value in zip(actual, expected):
        assert math.isclose(
            actual_value, expected_value, abs_tol=tolerance, rel_tol=0.0
        ), (actual, expected)


def angle_error(actual, expected):
    return math.atan2(
        math.sin(actual - expected), math.cos(actual - expected)
    )


def rotated_local_z(rpy):
    roll, pitch, yaw = rpy
    assert math.isclose(roll, 0.0, abs_tol=TOLERANCE)
    return (
        math.cos(yaw) * math.sin(pitch),
        math.sin(yaw) * math.sin(pitch),
        math.cos(pitch),
    )


def rotation_matrix(rpy):
    roll, pitch, yaw = rpy
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr,
         cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr,
         sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def rotate(matrix, value):
    return tuple(
        sum(row[index] * value[index] for index in range(3))
        for row in matrix
    )


def stl_bounds(path):
    minimum = [math.inf, math.inf, math.inf]
    maximum = [-math.inf, -math.inf, -math.inf]
    size = path.stat().st_size
    with path.open('rb') as stream:
        stream.seek(80)
        triangle_count = struct.unpack('<I', stream.read(4))[0]
        if size == 84 + triangle_count * 50:
            record = struct.Struct('<12fH')
            for _ in range(triangle_count):
                values = record.unpack(stream.read(record.size))
                for offset in (3, 6, 9):
                    for axis in range(3):
                        coordinate = values[offset + axis]
                        minimum[axis] = min(minimum[axis], coordinate)
                        maximum[axis] = max(maximum[axis], coordinate)
            return tuple(minimum), tuple(maximum)

    for line in path.read_text(errors='strict').splitlines():
        fields = line.split()
        if len(fields) == 4 and fields[0].lower() == 'vertex':
            for axis, text in enumerate(fields[1:]):
                coordinate = float(text)
                minimum[axis] = min(minimum[axis], coordinate)
                maximum[axis] = max(maximum[axis], coordinate)
    return tuple(minimum), tuple(maximum)


def load_robot(xacro_file, mappings=None):
    mappings = mappings or {}
    generated = subprocess.run(
        [
            'xacro',
            xacro_file,
            'use_gazebo:=true',
            'controllers_file:=unused.yaml',
            *(f'{name}:={value}' for name, value in mappings.items()),
        ],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    return ET.fromstring(generated)


def main():
    xacro_file = sys.argv[1]
    root = load_robot(xacro_file)
    urdf_directory = Path(xacro_file).resolve().parent
    description_directory = urdf_directory.parent
    mesh_directory = description_directory.parent / 'meshes' / 'wheels'
    links = {link.attrib['name']: link for link in root.findall('link')}
    joints = {joint.attrib['name']: joint for joint in root.findall('joint')}

    wheel_links = {
        name: link for name, link in links.items()
        if name.endswith('_wheel_link')
    }
    driven_joints = {
        name: joint for name, joint in joints.items()
        if name.endswith('_wheel_joint')
    }
    roller_links = {
        name: link for name, link in links.items() if '_roller_' in name
    }
    roller_joints = {
        name: joint for name, joint in joints.items() if '_roller_' in name
    }
    assert len(wheel_links) == 4
    assert len(driven_joints) == 4
    assert len(roller_links) == 40
    assert len(roller_joints) == 40

    gazebo_by_reference = {
        gazebo.attrib['reference']: gazebo
        for gazebo in root.findall('gazebo')
        if 'reference' in gazebo.attrib
    }

    xacro_namespace = 'http://www.ros.org/wiki/xacro'
    base_root = ET.parse(urdf_directory / 'base.xacro').getroot()
    wheel_calls = base_root.findall(
        f'.//{{{xacro_namespace}}}mecanum_wheel'
    )
    assert {call.attrib['name'] for call in wheel_calls} == set(WHEELS)
    for call in wheel_calls:
        expected = WHEELS[call.attrib['name']]
        assert 'visual_rpy' in call.attrib
        assert call.attrib['mesh'] == expected['mesh']
        assert int(call.attrib['roller_handedness']) == expected['handedness']
        phase_argument = wheel_name_to_phase_argument(call.attrib['name'])
        assert call.attrib['roller_phase'] == f'$(arg {phase_argument})'
        assert math.isclose(
            float(call.attrib['roller_axial_offset']),
            expected['axial_offset'],
            abs_tol=TOLERANCE,
        )

    for wheel_name, expected in WHEELS.items():
        wheel_link_name = wheel_name + '_wheel_link'
        wheel_joint_name = wheel_name + '_wheel_joint'
        wheel_link = links[wheel_link_name]
        wheel_joint = joints[wheel_joint_name]

        mesh = wheel_link.find('visual/geometry/mesh')
        assert mesh is not None
        assert mesh.attrib['filename'].endswith('/' + expected['mesh'])
        assert_vector_close(
            vector(wheel_link.find('visual/origin').attrib['xyz']),
            expected['visual_xyz'],
        )
        visual_rpy = vector(
            wheel_link.find('visual/origin').attrib['rpy']
        )
        assert_vector_close(visual_rpy, expected['visual_rpy'])
        assert all(math.isfinite(value) for value in expected['visual_xyz'])
        assert all(math.isfinite(value) for value in visual_rpy)
        scale = vector(mesh.attrib['scale'])
        assert_vector_close(scale, (0.001, 0.001, 0.001))
        assert all(value > 0.0 for value in scale)

        mesh_path = mesh_directory / expected['mesh']
        bounds_minimum, bounds_maximum = stl_bounds(mesh_path)
        measured_center = tuple(
            (minimum + maximum) / 2.0
            for minimum, maximum in zip(bounds_minimum, bounds_maximum)
        )
        assert_vector_close(
            measured_center, expected['mesh_center_mm'], tolerance=2e-5
        )
        rotation = rotation_matrix(visual_rpy)
        center_in_link = rotate(
            rotation,
            tuple(value * scale[axis]
                  for axis, value in enumerate(measured_center)),
        )
        center_in_link = tuple(
            value + expected['visual_xyz'][axis]
            for axis, value in enumerate(center_in_link)
        )
        assert_vector_close(center_in_link, (0.0, 0.0, 0.0))

        inner_normal = rotate(rotation, expected['inner_normal'])
        expected_inward_y = -1.0 if wheel_name in (
            'front_left', 'rear_left'
        ) else 1.0
        assert math.isclose(
            inner_normal[1], expected_inward_y, abs_tol=TOLERANCE
        )
        assert math.isclose(inner_normal[0], 0.0, abs_tol=TOLERANCE)
        assert math.isclose(inner_normal[2], 0.0, abs_tol=TOLERANCE)

        mesh_theta = expected['phase']
        mesh_radial = (
            math.cos(mesh_theta), math.sin(mesh_theta), 0.0
        )
        mesh_tangent = (
            -math.sin(mesh_theta), math.cos(mesh_theta), 0.0
        )
        mesh_axis = (
            expected['mesh_handedness'] * mesh_tangent[0]
            * math.sqrt(0.5),
            expected['mesh_handedness'] * mesh_tangent[1]
            * math.sqrt(0.5),
            math.sqrt(0.5),
        )
        radial_in_link = rotate(rotation, mesh_radial)
        axis_in_link = rotate(rotation, mesh_axis)
        link_theta = math.atan2(radial_in_link[2], radial_in_link[0])
        link_tangent = (
            -math.sin(link_theta), 0.0, math.cos(link_theta)
        )
        if axis_in_link[1] < 0.0:
            axis_in_link = tuple(-value for value in axis_in_link)
        tangent_component = sum(
            axis_value * tangent_value
            for axis_value, tangent_value
            in zip(axis_in_link, link_tangent)
        )
        assert math.isclose(
            axis_in_link[1], math.sqrt(0.5), abs_tol=TOLERANCE
        )
        assert math.isclose(
            tangent_component,
            expected['handedness'] * math.sqrt(0.5),
            abs_tol=TOLERANCE,
        )

        assert wheel_link.find('collision') is None
        assert_vector_close(vector(wheel_joint.find('axis').attrib['xyz']),
                            (0.0, 1.0, 0.0))
        joint_xyz = vector(wheel_joint.find('origin').attrib['xyz'])
        assert_vector_close(joint_xyz[:2], expected['position_xy'])

        hub_inertial = wheel_link.find('inertial')
        hub_mass = float(hub_inertial.find('mass').attrib['value'])
        assert math.isclose(hub_mass, HUB_MASS, abs_tol=TOLERANCE)
        assert_positive_finite_inertia(hub_inertial)

        wheel_gazebo = gazebo_by_reference[wheel_link_name]
        assert wheel_gazebo.find('selfCollide').text == 'false'
        assert wheel_gazebo.find('mu1') is None
        assert wheel_gazebo.find('mu2') is None
        assert wheel_gazebo.find('fdir1') is None

        actual_angles = []
        roller_mass_total = 0.0
        for index in range(10):
            link_name = f'{wheel_name}_roller_{index}_link'
            joint_name = f'{wheel_name}_roller_{index}_joint'
            roller_link = links[link_name]
            roller_joint = joints[joint_name]
            assert roller_joint.attrib['type'] == 'continuous'
            assert roller_joint.find('parent').attrib['link'] == wheel_link_name
            assert roller_joint.find('child').attrib['link'] == link_name
            assert_vector_close(
                vector(roller_joint.find('axis').attrib['xyz']),
                (0.0, 0.0, 1.0),
            )

            origin = roller_joint.find('origin')
            xyz = vector(origin.attrib['xyz'])
            rpy = vector(origin.attrib['rpy'])
            theta = math.atan2(xyz[2], xyz[0])
            expected_theta = expected['phase'] + index * 2.0 * math.pi / 10
            assert abs(angle_error(theta, expected_theta)) < TOLERANCE
            assert math.isclose(
                math.hypot(xyz[0], xyz[2]),
                ROLLER_CENTER_RADIUS,
                abs_tol=TOLERANCE,
            )
            assert math.isclose(
                xyz[1], expected['axial_offset'], abs_tol=TOLERANCE
            )
            actual_angles.append(theta % (2.0 * math.pi))

            axis = rotated_local_z(rpy)
            expected_axis = (
                -expected['handedness'] * math.sin(math.pi / 4.0)
                * math.sin(expected_theta),
                math.cos(math.pi / 4.0),
                expected['handedness'] * math.sin(math.pi / 4.0)
                * math.cos(expected_theta),
            )
            assert_vector_close(axis, expected_axis)
            assert math.isclose(
                math.sqrt(sum(component * component for component in axis)),
                1.0,
                abs_tol=TOLERANCE,
            )
            tangent = (-math.sin(expected_theta), 0.0,
                       math.cos(expected_theta))
            axle_component = axis[1]
            tangent_component = sum(
                axis_value * tangent_value
                for axis_value, tangent_value in zip(axis, tangent)
            )
            assert math.isclose(
                axle_component, math.sqrt(0.5), abs_tol=TOLERANCE
            )
            assert math.isclose(
                tangent_component,
                expected['handedness'] * math.sqrt(0.5),
                abs_tol=TOLERANCE,
            )

            collisions = roller_link.findall('collision')
            assert len(collisions) == 9
            assert {collision.attrib['name'] for collision in collisions} \
                == set(BARREL_SPHERES)
            for collision in collisions:
                axial_position, radius = BARREL_SPHERES[
                    collision.attrib['name']]
                sphere = collision.find('geometry/sphere')
                assert sphere is not None
                assert math.isclose(
                    float(sphere.attrib['radius']), radius,
                    abs_tol=TOLERANCE)
                origin = collision.find('origin')
                actual_position = 0.0 if origin is None else vector(
                    origin.attrib['xyz'])[2]
                assert math.isclose(
                    actual_position, axial_position, abs_tol=TOLERANCE)

            inertial = roller_link.find('inertial')
            mass = float(inertial.find('mass').attrib['value'])
            assert mass > 0.0 and math.isfinite(mass)
            assert math.isclose(mass, ROLLER_MASS, abs_tol=TOLERANCE)
            roller_mass_total += mass
            assert_positive_finite_inertia(inertial)

            dynamics = roller_joint.find('dynamics')
            assert math.isclose(
                float(dynamics.attrib['damping']), 0.0, abs_tol=TOLERANCE
            )
            assert math.isclose(
                float(dynamics.attrib['friction']), 0.0,
                abs_tol=TOLERANCE,
            )

            # mu1/mu2/kp/kd are Gazebo-Classic extension tags that gz sdf -p
            # discards, so authoring them here would silently do nothing.
            # Contact surfaces are injected at SDF generation instead; see
            # mobile_base_bringup/test/test_roller_contact_surfaces.py.
            roller_gazebo = gazebo_by_reference[link_name]
            assert roller_gazebo.find('mu1') is None
            assert roller_gazebo.find('mu2') is None
            assert roller_gazebo.find('kp') is None
            assert roller_gazebo.find('kd') is None
            assert roller_gazebo.find('fdir1') is None
            assert roller_gazebo.find('selfCollide').text == 'false'

        angles = sorted(actual_angles)
        circular_gaps = [
            (angles[(index + 1) % 10] - angles[index]) % (2.0 * math.pi)
            for index in range(10)
        ]
        for gap in circular_gaps:
            assert math.isclose(
                gap, 2.0 * math.pi / 10, abs_tol=TOLERANCE
            )
        assert math.isclose(
            hub_mass + roller_mass_total, WHEEL_MASS, abs_tol=TOLERANCE
        )

    assert WHEELS['front_left']['handedness'] == \
        WHEELS['rear_right']['handedness']
    assert WHEELS['front_right']['handedness'] == \
        WHEELS['rear_left']['handedness']
    assert WHEELS['front_left']['handedness'] != \
        WHEELS['front_right']['handedness']
    assert math.isclose(
        ROLLER_CENTER_RADIUS + ROLLER_RADIUS,
        WHEEL_RADIUS,
        abs_tol=TOLERANCE,
    )
    assert not root.findall('.//fdir1')

    control_joints = root.findall('ros2_control/joint')
    command_interfaces = [
        (joint.attrib['name'], interface.attrib['name'])
        for joint in control_joints
        for interface in joint.findall('command_interface')
    ]
    assert command_interfaces == [
        ('front_left_wheel_joint', 'velocity'),
        ('front_right_wheel_joint', 'velocity'),
        ('rear_right_wheel_joint', 'velocity'),
        ('rear_left_wheel_joint', 'velocity'),
    ]
    state_by_joint = {
        joint.attrib['name']: {
            interface.attrib['name']
            for interface in joint.findall('state_interface')
        }
        for joint in control_joints
    }
    assert len(state_by_joint) == 44
    for joint_name in roller_joints:
        assert state_by_joint[joint_name] == {'position', 'velocity'}
        control_joint = next(
            joint for joint in control_joints
            if joint.attrib['name'] == joint_name
        )
        assert not control_joint.findall('command_interface')

    relevant_sources = (
        (urdf_directory / 'base.xacro').read_text()
        + (urdf_directory / 'macros' / 'mecanum_wheels.xacro').read_text()
        + (urdf_directory / 'macros' / 'mecanum_rollers.xacro').read_text()
    )
    assert ('traction' + '_sign') not in relevant_sources
    assert '<fdir1' not in relevant_sources
    assert '>100<' not in relevant_sources

    calibrated = load_robot(xacro_file, {
        'roller_joint_damping': '0.0',
        'roller_joint_friction': '0.0',
        'roller_contact_mu': '0.6',
        'front_left_roller_phase': '0.3',
        'front_right_roller_phase': '0.3',
        'rear_right_roller_phase': '0.3',
        'rear_left_roller_phase': '0.3',
    })
    calibrated_joints = {
        joint.attrib['name']: joint for joint in calibrated.findall('joint')}
    calibrated_gazebo = {
        gazebo.attrib['reference']: gazebo
        for gazebo in calibrated.findall('gazebo')
        if 'reference' in gazebo.attrib}
    for wheel_name in WHEELS:
        joint = calibrated_joints[f'{wheel_name}_roller_0_joint']
        dynamics = joint.find('dynamics')
        assert float(dynamics.attrib['damping']) == 0.0
        assert float(dynamics.attrib['friction']) == 0.0
        theta = math.atan2(*reversed(vector(joint.find('origin').attrib['xyz'])[::2]))
        assert abs(angle_error(theta, 0.3)) < TOLERANCE
        # roller_contact_mu deliberately leaves no trace in the URDF; it is
        # applied when the simulation SDF is generated. The corresponding
        # override check lives in the generator test.
        gazebo = calibrated_gazebo[f'{wheel_name}_roller_0_link']
        assert gazebo.find('mu1') is None

    cylinder_robot = load_robot(xacro_file, {
        'roller_collision_model': 'cylinder',
    })
    cylinder_roller_links = [
        link for link in cylinder_robot.findall('link')
        if '_roller_' in link.attrib['name']
    ]
    assert len(cylinder_roller_links) == 40
    for link in cylinder_roller_links:
        collisions = link.findall('collision')
        assert len(collisions) == 1
        cylinder = collisions[0].find('geometry/cylinder')
        assert cylinder is not None
        assert math.isclose(
            float(cylinder.attrib['radius']),
            ROLLER_RADIUS,
            abs_tol=TOLERANCE,
        )
        assert math.isclose(
            float(cylinder.attrib['length']),
            ROLLER_LENGTH,
            abs_tol=TOLERANCE,
        )


def assert_positive_finite_inertia(inertial):
    inertia = inertial.find('inertia')
    for attribute in ('ixx', 'iyy', 'izz'):
        value = float(inertia.attrib[attribute])
        assert value > 0.0 and math.isfinite(value)
    for attribute in ('ixy', 'ixz', 'iyz'):
        assert math.isfinite(float(inertia.attrib[attribute]))


def wheel_name_to_phase_argument(wheel_name):
    return wheel_name + '_roller_phase'


if __name__ == '__main__':
    main()
