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

"""Generate Gazebo SDF with canonical anisotropic mecanum wheel contact."""

import argparse
import math
from pathlib import Path
import re
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory


WHEEL_URI_PREFIXES = (
    'package://mobile_base_description/meshes/wheels/',
    'package://mobile_base/meshes/wheels/',
    'model://mobile_base/meshes/wheels/',
    'model://mobile_base_description/meshes/wheels/',
)
MOTOR_URI_PREFIXES = (
    'package://mobile_base_description/meshes/motors/',
    'package://mobile_base/meshes/motors/',
    'model://mobile_base/meshes/motors/',
    'model://mobile_base_description/meshes/motors/',
)
COMPONENT_URI_PREFIXES = (
    'package://mobile_base_description/meshes/electronics/',
    'package://mobile_base/meshes/electronics/',
    'model://mobile_base/meshes/electronics/',
    'model://mobile_base_description/meshes/electronics/',
)
MOTOR_FILES = {'tt_gearmotor.stl'}
# The battery is a plain box, so only the boards appear here.
COMPONENT_FILES = {'esp32_s3_devkit_shield.stl', 'tb6612fng.stl'}
COMPONENT_MESH_REFERENCES = 3
WHEEL_FILES = {
    'mecanum_wheel_FL.stl', 'mecanum_wheel_FR.stl',
    'mecanum_wheel_RL.stl', 'mecanum_wheel_RR.stl',
}
WHEEL_NAMES = ('front_left', 'front_right', 'rear_right', 'rear_left')
WHEEL_RADIUS = 0.03074443
WHEEL_WIDTH = 0.03360543
WHEEL_CONTACT_MU = 0.8
WHEEL_CONTACT_MU2 = 0.2
WHEEL_CONTACT_SLIP1 = 0.0
WHEEL_CONTACT_SLIP2 = 0.0
GZ_SCHEMA_NS = 'http://gazebosim.org/schema'
FRICTION_FRAME = 'base_footprint'
# The visual roller direction follows an X pattern across the four wheels.
WHEEL_DIRECTION_SIGN = {
    'front_left': -1.0,
    'front_right': 1.0,
    'rear_right': -1.0,
    'rear_left': 1.0,
}
PASSIVE_ROLLER_BODY = re.compile(r'_roller_[0-9]+_(?:link|joint)$')


def _wheel_file_uris():
    share = Path(get_package_share_directory('mobile_base_description'))
    wheels = share / 'meshes' / 'wheels'
    missing = sorted(name for name in WHEEL_FILES if not (wheels / name).is_file())
    if missing:
        raise RuntimeError('Missing installed wheel meshes: ' + ', '.join(missing))
    return {name: (wheels / name).resolve().as_uri() for name in WHEEL_FILES}


def _motor_file_uris():
    share = Path(get_package_share_directory('mobile_base_description'))
    motors = share / 'meshes' / 'motors'
    missing = sorted(name for name in MOTOR_FILES if not (motors / name).is_file())
    if missing:
        raise RuntimeError('Missing installed motor meshes: ' + ', '.join(missing))
    return {name: (motors / name).resolve().as_uri() for name in MOTOR_FILES}


def _component_file_uris():
    share = Path(get_package_share_directory('mobile_base_description'))
    electronics = share / 'meshes' / 'electronics'
    missing = sorted(
        name for name in COMPONENT_FILES if not (electronics / name).is_file())
    if missing:
        raise RuntimeError(
            'Missing installed component meshes: ' + ', '.join(missing))
    return {
        name: (electronics / name).resolve().as_uri()
        for name in COMPONENT_FILES
    }


def _nonnegative_finite(value, field):
    number = float(value)
    if not math.isfinite(number) or number < 0.0:
        raise RuntimeError(f'{field} must be finite and nonnegative')
    return number


def _subelement_with_text(parent, tag, value):
    element = ET.SubElement(parent, tag)
    element.text = str(value)
    return element


def _wheel_links(root):
    links = {}
    for link in root.findall('.//link'):
        name = link.attrib.get('name', '')
        for wheel in WHEEL_NAMES:
            if name == f'{wheel}_wheel_link':
                if wheel in links:
                    raise RuntimeError(f'Duplicate wheel link for {wheel}')
                links[wheel] = link
    missing = sorted(set(WHEEL_NAMES) - set(links))
    if missing:
        raise RuntimeError('Missing wheel links: ' + ', '.join(missing))
    return links


def inject_wheel_surfaces(
        root, mu=WHEEL_CONTACT_MU, mu2=WHEEL_CONTACT_MU2,
        slip1=WHEEL_CONTACT_SLIP1, slip2=WHEEL_CONTACT_SLIP2):
    """Inject one direction-dependent friction surface per driven wheel."""
    mu = _nonnegative_finite(mu, 'mu')
    mu2 = _nonnegative_finite(mu2, 'mu2')
    slip1 = _nonnegative_finite(slip1, 'slip1')
    slip2 = _nonnegative_finite(slip2, 'slip2')
    if math.isclose(mu, mu2, rel_tol=1e-9):
        raise RuntimeError('Anisotropic mecanum contact requires mu != mu2')

    root.set('xmlns:gz', GZ_SCHEMA_NS)
    injected = 0
    for wheel, link in _wheel_links(root).items():
        collisions = link.findall('collision')
        if len(collisions) != 1:
            raise RuntimeError(
                f'{wheel}_wheel_link must have exactly one collision; '
                f'found {len(collisions)}')
        collision = collisions[0]
        for existing in collision.findall('surface'):
            collision.remove(existing)
        surface = ET.SubElement(collision, 'surface')
        friction = ET.SubElement(surface, 'friction')
        ode = ET.SubElement(friction, 'ode')
        _subelement_with_text(ode, 'mu', mu)
        _subelement_with_text(ode, 'mu2', mu2)
        _subelement_with_text(ode, 'slip1', slip1)
        _subelement_with_text(ode, 'slip2', slip2)
        fdir1 = _subelement_with_text(
            ode, 'fdir1', f'1 {WHEEL_DIRECTION_SIGN[wheel]:g} 0')
        # DART resolves this literal name. Namespace registration would make
        # ElementTree write ns0:expressed_in, which the engine ignores.
        fdir1.set('gz:expressed_in', FRICTION_FRAME)
        injected += 1
    return injected


def validate_mecanum_wheel_contact(root):
    """Reject an SDF that could silently lose the canonical contact model."""
    for element in (*root.findall('.//link'), *root.findall('.//joint')):
        name = element.attrib.get('name', '')
        if PASSIVE_ROLLER_BODY.search(name):
            raise RuntimeError(f'Passive roller body remains in SDF: {name}')

    expected_joints = {f'{wheel}_wheel_joint' for wheel in WHEEL_NAMES}
    actual_joints = [
        joint.attrib.get('name') for joint in root.findall('.//joint')
        if joint.attrib.get('name') in expected_joints
    ]
    if len(actual_joints) != 4 or set(actual_joints) != expected_joints:
        raise RuntimeError('Generated SDF must contain four driven wheel joints')
    if not any(link.attrib.get('name') == FRICTION_FRAME
               for link in root.findall('.//link')):
        raise RuntimeError(
            f'Friction direction frame {FRICTION_FRAME} is not an SDF link')

    for wheel, link in _wheel_links(root).items():
        collisions = link.findall('collision')
        if len(collisions) != 1:
            raise RuntimeError(
                f'{wheel}_wheel_link must have exactly one collision; '
                f'found {len(collisions)}')
        collision = collisions[0]
        cylinder = collision.find('geometry/cylinder')
        if cylinder is None:
            raise RuntimeError(f'{wheel} wheel collision must be a cylinder')
        radius = cylinder.findtext('radius')
        length = cylinder.findtext('length')
        if radius is None or not math.isclose(
                float(radius), WHEEL_RADIUS, rel_tol=1e-9):
            raise RuntimeError(f'{wheel} wheel collision has wrong radius')
        if length is None or not math.isclose(
                float(length), WHEEL_WIDTH, rel_tol=1e-9):
            raise RuntimeError(f'{wheel} wheel collision has wrong width')

        ode = collision.find('surface/friction/ode')
        if ode is None:
            raise RuntimeError(f'{wheel} wheel collision has no friction surface')
        expected = {
            'mu': WHEEL_CONTACT_MU,
            'mu2': WHEEL_CONTACT_MU2,
            'slip1': WHEEL_CONTACT_SLIP1,
            'slip2': WHEEL_CONTACT_SLIP2,
        }
        for field, wanted in expected.items():
            value = ode.findtext(field)
            if value is None or not math.isclose(
                    float(value), wanted, rel_tol=1e-9):
                raise RuntimeError(
                    f'{wheel} wheel contact {field} must be {wanted}')
        fdir1 = ode.find('fdir1')
        wanted_direction = f'1 {WHEEL_DIRECTION_SIGN[wheel]:g} 0'
        if fdir1 is None or fdir1.text != wanted_direction:
            raise RuntimeError(f'{wheel} wheel has wrong friction direction')
        if fdir1.attrib.get('gz:expressed_in') != FRICTION_FRAME:
            raise RuntimeError(
                f'{wheel} wheel friction direction must be expressed in '
                f'{FRICTION_FRAME}')


def _rewrite_wheel_mesh_uris(root):
    wheel_file_uris = _wheel_file_uris()
    referenced = []
    for uri in root.findall('.//uri'):
        for prefix in WHEEL_URI_PREFIXES:
            if uri.text and uri.text.startswith(prefix):
                mesh_name = uri.text[len(prefix):]
                if mesh_name not in wheel_file_uris:
                    raise RuntimeError('Unresolved wheel visual URI: ' + uri.text)
                uri.text = wheel_file_uris[mesh_name]
                referenced.append(mesh_name)
                break
    if (len(referenced) != 4 or set(referenced) != WHEEL_FILES
            or any(referenced.count(name) != 1 for name in WHEEL_FILES)):
        raise RuntimeError(
            'Generated SDF must reference each wheel visual mesh exactly once')


def _rewrite_motor_mesh_uris(root):
    """Resolve the shared motor mesh to an absolute path for Gazebo."""
    # All four motor links reuse one mesh, so unlike the wheels this counts
    # references rather than requiring each file exactly once.
    motor_file_uris = _motor_file_uris()
    referenced = 0
    for uri in root.findall('.//uri'):
        for prefix in MOTOR_URI_PREFIXES:
            if uri.text and uri.text.startswith(prefix):
                mesh_name = uri.text[len(prefix):]
                if mesh_name not in motor_file_uris:
                    raise RuntimeError('Unresolved motor visual URI: ' + uri.text)
                uri.text = motor_file_uris[mesh_name]
                referenced += 1
                break
    if referenced != 4:
        raise RuntimeError(
            'Generated SDF must reference the motor visual mesh once per '
            f'motor; found {referenced}')


def _rewrite_component_mesh_uris(root):
    """Resolve the deck component meshes to absolute paths for Gazebo."""
    # Like the motors and unlike the wheels this counts references rather than
    # requiring each file once: the two motor drivers share one mesh. An
    # unrewritten URI is not a cosmetic problem - Gazebo cannot resolve a
    # package:// path and the part silently vanishes from the scene, so the
    # count is asserted rather than left to a visual check.
    component_file_uris = _component_file_uris()
    referenced = 0
    for uri in root.findall('.//uri'):
        for prefix in COMPONENT_URI_PREFIXES:
            if uri.text and uri.text.startswith(prefix):
                mesh_name = uri.text[len(prefix):]
                if mesh_name not in component_file_uris:
                    raise RuntimeError(
                        'Unresolved component visual URI: ' + uri.text)
                uri.text = component_file_uris[mesh_name]
                referenced += 1
                break
    if referenced != COMPONENT_MESH_REFERENCES:
        raise RuntimeError(
            'Generated SDF must reference the component visual meshes '
            f'{COMPONENT_MESH_REFERENCES} times; found {referenced}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--xacro', required=True)
    parser.add_argument('--controllers', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()

    urdf = subprocess.run(
        ['xacro', args.xacro, 'use_gazebo:=true',
         'controllers_file:=' + args.controllers],
        check=True, capture_output=True, text=True).stdout
    with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf') as urdf_file:
        urdf_file.write(urdf)
        urdf_file.flush()
        sdf = subprocess.run(
            ['gz', 'sdf', '-p', urdf_file.name],
            check=True, capture_output=True, text=True).stdout

    root = ET.fromstring(sdf)
    _rewrite_wheel_mesh_uris(root)
    _rewrite_motor_mesh_uris(root)
    _rewrite_component_mesh_uris(root)
    inject_wheel_surfaces(root)
    validate_mecanum_wheel_contact(root)
    ET.ElementTree(root).write(
        args.output, encoding='utf-8', xml_declaration=True)


if __name__ == '__main__':
    main()
