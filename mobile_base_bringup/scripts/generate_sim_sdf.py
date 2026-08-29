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
WHEEL_FILES = {
    'mecanum_wheel_FL.stl',
    'mecanum_wheel_FR.stl',
    'mecanum_wheel_RL.stl',
    'mecanum_wheel_RR.stl',
}
WHEEL_NAMES = ('front_left', 'front_right', 'rear_right', 'rear_left')
EXPLICIT_ROLLER_MODELS = ('barrel', 'cylinder')
GZ_SCHEMA_NS = 'http://gazebosim.org/schema'
# Frame the wheel friction direction is expressed in. Must be a link that
# survives URDF-to-SDF conversion; base_link does not.
FRICTION_FRAME = 'base_footprint'
# Roller handedness per wheel, kept identical to base.xacro. The X-pattern
# gives diagonally opposite wheels the same handedness.
WHEEL_HANDEDNESS = {
    'front_left': -1.0,
    'front_right': 1.0,
    'rear_right': -1.0,
    'rear_left': 1.0,
}
ROLLER_LINK = re.compile(
    r'^(front_left|front_right|rear_right|rear_left)_roller_([0-9]+)_link$'
)
ROLLER_JOINT = re.compile(
    r'^(front_left|front_right|rear_right|rear_left)_roller_([0-9]+)_joint$'
)


def _wheel_file_uris():
    share = Path(get_package_share_directory('mobile_base_description'))
    wheels = share / 'meshes' / 'wheels'
    missing = sorted(name for name in WHEEL_FILES if not (wheels / name).is_file())
    if missing:
        raise RuntimeError(
            'Missing installed wheel meshes: ' + ', '.join(missing)
        )
    return {name: (wheels / name).resolve().as_uri() for name in WHEEL_FILES}


def _validated_named_elements(root, tag, pattern):
    elements = {}
    for element in root.findall('.//' + tag):
        name = element.attrib.get('name', '')
        match = pattern.fullmatch(name)
        if match:
            if name in elements:
                raise RuntimeError('Duplicate roller name in SDF: ' + name)
            elements[name] = element
    return elements


def _nonnegative_finite(value, field):
    number = float(value)
    if not math.isfinite(number) or number < 0.0:
        raise RuntimeError(f'{field} must be finite and nonnegative')
    return number


def _subelement_with_text(parent, tag, value):
    element = ET.SubElement(parent, tag)
    element.text = str(value)
    return element


def build_roller_surface(mu, mu2, kp, kd, torsional_coefficient):
    """Build one SDF contact surface for a roller collision."""
    # Element order follows the installed sdformat surface schema: friction
    # (torsional before ode) and then contact.
    surface = ET.Element('surface')
    friction = ET.SubElement(surface, 'friction')
    torsional = ET.SubElement(friction, 'torsional')
    _subelement_with_text(torsional, 'coefficient', torsional_coefficient)
    ode_friction = ET.SubElement(friction, 'ode')
    _subelement_with_text(ode_friction, 'mu', mu)
    _subelement_with_text(ode_friction, 'mu2', mu2)
    contact = ET.SubElement(surface, 'contact')
    ode_contact = ET.SubElement(contact, 'ode')
    _subelement_with_text(ode_contact, 'kp', kp)
    _subelement_with_text(ode_contact, 'kd', kd)
    return surface


def inject_roller_surfaces(root, mu, mu2, kp, kd, torsional_coefficient):
    """Write an explicit contact surface into every roller collision."""
    # gz sdf -p does not implement the Gazebo-Classic <gazebo reference>
    # friction vocabulary, so mu1, mu2, kp, and kd authored in the xacro are
    # discarded during URDF-to-SDF conversion. Without this injection every
    # roller collision silently falls back to the sdformat defaults (mu and
    # mu2 of 1.0) and the configured contact parameters never reach the
    # physics engine. Returns the number of collisions given a surface.
    mu = _nonnegative_finite(mu, 'mu')
    mu2 = _nonnegative_finite(mu2, 'mu2')
    kp = _nonnegative_finite(kp, 'kp')
    kd = _nonnegative_finite(kd, 'kd')
    torsional_coefficient = _nonnegative_finite(
        torsional_coefficient, 'torsional_coefficient')

    injected = 0
    roller_links = _validated_named_elements(root, 'link', ROLLER_LINK)
    for link in roller_links.values():
        for collision in link.findall('collision'):
            for existing in collision.findall('surface'):
                collision.remove(existing)
            collision.append(
                build_roller_surface(mu, mu2, kp, kd, torsional_coefficient)
            )
            injected += 1
    if injected == 0:
        raise RuntimeError(
            'No roller collisions found to receive a contact surface')
    return injected


def inject_wheel_surfaces(root, mu, mu2, slip1, slip2):
    """Give each driven hub one anisotropic friction cone."""
    # A mecanum roller rolls across its own axis and grips along it, so
    # traction is high along the roller axis (fdir1) and low across it.
    #
    # fdir1 is otherwise interpreted in the collision frame, which is fixed to
    # the spinning wheel: the friction direction would rotate with the wheel
    # and the mecanum effect would smear away. gz:expressed_in pins it to the
    # chassis instead.
    #
    # Two details that make this silently do nothing if got wrong. dartsim
    # looks the attribute up by its literal name, so it must be written as
    # "gz:expressed_in" rather than through ElementTree namespace machinery,
    # which emits "ns0:expressed_in" and never matches. And the frame must be
    # base_footprint, not base_link: URDF-to-SDF fixed-joint reduction lumps
    # base_link into base_footprint, leaving base_link as an SDF <frame> that
    # does not resolve here. base_footprint is a pure translation from
    # base_link (rpy 0 0 0), so the orientation is identical.
    mu = _nonnegative_finite(mu, 'mu')
    mu2 = _nonnegative_finite(mu2, 'mu2')
    slip1 = _nonnegative_finite(slip1, 'slip1')
    slip2 = _nonnegative_finite(slip2, 'slip2')
    if math.isclose(mu, mu2, rel_tol=1e-9):
        raise RuntimeError(
            'husarion_cylinder needs mu != mu2; isotropic friction cannot '
            'produce mecanum motion')

    root.set('xmlns:gz', GZ_SCHEMA_NS)
    injected = 0
    for link in root.findall('.//link'):
        name = link.attrib.get('name', '')
        wheel = next(
            (w for w in WHEEL_NAMES if name == f'{w}_wheel_link'), None)
        if wheel is None:
            continue
        handedness = WHEEL_HANDEDNESS[wheel]
        for collision in link.findall('collision'):
            for existing in collision.findall('surface'):
                collision.remove(existing)
            surface = ET.SubElement(collision, 'surface')
            friction = ET.SubElement(surface, 'friction')
            ode = ET.SubElement(friction, 'ode')
            _subelement_with_text(ode, 'mu', mu)
            _subelement_with_text(ode, 'mu2', mu2)
            _subelement_with_text(ode, 'slip1', slip1)
            _subelement_with_text(ode, 'slip2', slip2)
            # Mapping taken from the shipping ROSbot XL wheel description,
            # whose per-wheel fdir is FL "1 -1 0", FR "1 1 0", RL "1 1 0",
            # RR "1 -1 0" -- exactly "1 {handedness} 0" for this robot's
            # handedness. Measured under otherwise identical settings, this
            # mapping gives 0.47% mean diagnostic error against 170% for the
            # inverted one, so it is not a free choice.
            fdir1 = _subelement_with_text(
                ode, 'fdir1', f'1 {handedness:g} 0')
            fdir1.set('gz:expressed_in', FRICTION_FRAME)
            injected += 1
    if injected != 4:
        raise RuntimeError(
            f'Expected one collision on each of the four driven hubs; '
            f'gave surfaces to {injected}')
    return injected


def _write_sdf(root, output):
    ET.ElementTree(root).write(
        output,
        encoding='utf-8',
        xml_declaration=True,
    )


def validate_husarion_cylinder(root, expected_mu, expected_mu2):
    """Check the single-cylinder anisotropic wheel model."""
    # Unlike the explicit-roller models this one is allowed -- required -- to
    # carry fdir1, because the friction direction is the declared mechanism
    # rather than a shortcut hidden inside a physical model.
    roller_links = _validated_named_elements(root, 'link', ROLLER_LINK)
    if roller_links:
        raise RuntimeError(
            f'husarion_cylinder must emit no roller links; '
            f'found {len(roller_links)}')

    for wheel in WHEEL_NAMES:
        link = next(
            (link for link in root.findall('.//link')
             if link.attrib.get('name') == f'{wheel}_wheel_link'), None)
        if link is None:
            raise RuntimeError(f'Missing wheel link for {wheel}')
        collisions = link.findall('collision')
        if len(collisions) != 1:
            raise RuntimeError(
                f'{wheel}_wheel_link must have exactly one collision in '
                f'husarion_cylinder mode; found {len(collisions)}')
        collision = collisions[0]
        mu = collision.findtext('surface/friction/ode/mu')
        mu2 = collision.findtext('surface/friction/ode/mu2')
        fdir1 = collision.find('surface/friction/ode/fdir1')
        if mu is None or mu2 is None:
            raise RuntimeError(
                f'{wheel}_wheel_link collision lost its friction during SDF '
                'conversion; anisotropic contact would silently fall back to '
                'the isotropic sdformat default')
        if not math.isclose(float(mu), expected_mu, rel_tol=1e-9):
            raise RuntimeError(
                f'{wheel}_wheel_link has mu {mu}, expected {expected_mu}')
        if not math.isclose(float(mu2), expected_mu2, rel_tol=1e-9):
            raise RuntimeError(
                f'{wheel}_wheel_link has mu2 {mu2}, expected {expected_mu2}')
        if fdir1 is None:
            raise RuntimeError(
                f'{wheel}_wheel_link collision is missing fdir1, which is the '
                'entire mecanum mechanism in husarion_cylinder mode')
        if fdir1.attrib.get('gz:expressed_in') != FRICTION_FRAME:
            raise RuntimeError(
                f'{wheel}_wheel_link fdir1 must carry '
                f'gz:expressed_in="{FRICTION_FRAME}"; without a resolvable '
                'frame the friction direction is silently ignored')
        if not any(link.attrib.get('name') == FRICTION_FRAME
                   for link in root.findall('.//link')):
            raise RuntimeError(
                f'fdir1 references frame {FRICTION_FRAME}, which is not a '
                'link in the generated SDF; the friction direction would be '
                'silently ignored')
        if math.isclose(float(mu), float(mu2), rel_tol=1e-9):
            raise RuntimeError(
                'husarion_cylinder needs mu != mu2; isotropic friction '
                'cannot produce mecanum motion')


def _validate_explicit_rollers(root, collision_model, expected_mu):
    driven_joint_names = {name + '_wheel_joint' for name in WHEEL_NAMES}
    driven_joints = [
        joint.attrib.get('name') for joint in root.findall('.//joint')
        if joint.attrib.get('name') in driven_joint_names
    ]
    if len(driven_joints) != 4 or set(driven_joints) != driven_joint_names:
        missing = sorted(driven_joint_names - set(driven_joints))
        raise RuntimeError(
            'Expected four driven wheel joints; missing: '
            + ', '.join(missing)
        )

    roller_links = _validated_named_elements(root, 'link', ROLLER_LINK)
    roller_joints = _validated_named_elements(root, 'joint', ROLLER_JOINT)
    if len(roller_links) != 40:
        raise RuntimeError(
            f'Expected 40 roller links, found {len(roller_links)}'
        )
    if len(roller_joints) != 40:
        raise RuntimeError(
            f'Expected 40 roller joints, found {len(roller_joints)}'
        )

    for wheel_name in WHEEL_NAMES:
        expected_links = {
            f'{wheel_name}_roller_{index}_link' for index in range(10)
        }
        expected_joints = {
            f'{wheel_name}_roller_{index}_joint' for index in range(10)
        }
        actual_links = {
            name for name in roller_links
            if name.startswith(wheel_name + '_roller_')
        }
        actual_joints = {
            name for name in roller_joints
            if name.startswith(wheel_name + '_roller_')
        }
        if actual_links != expected_links:
            raise RuntimeError(
                f'{wheel_name} must have exactly roller links 0 through 9'
            )
        if actual_joints != expected_joints:
            raise RuntimeError(
                f'{wheel_name} must have exactly roller joints 0 through 9'
            )
        for index in range(10):
            joint_name = f'{wheel_name}_roller_{index}_joint'
            joint = roller_joints[joint_name]
            parent = joint.findtext('parent')
            child = joint.findtext('child')
            if parent != wheel_name + '_wheel_link':
                raise RuntimeError(
                    f'Roller joint {joint_name} has invalid parent {parent}'
                )
            if child != f'{wheel_name}_roller_{index}_link':
                raise RuntimeError(
                    f'Roller joint {joint_name} has invalid child {child}'
                )

    collision_count = 0
    for name, link in roller_links.items():
        collisions = link.findall('collision')
        expected_collision_count = 9 if collision_model == 'barrel' else 1
        if len(collisions) != expected_collision_count:
            raise RuntimeError(
                f'Roller link {name} must have exactly '
                f'{expected_collision_count} collisions; '
                f'found {len(collisions)}'
            )
        collision_count += len(collisions)
        expected_geometry = 'sphere' if collision_model == 'barrel' \
            else 'cylinder'
        for collision in collisions:
            geometry = collision.find('geometry')
            if geometry is None or len(geometry) != 1:
                raise RuntimeError(
                    f'Roller link {name} must have one geometry per collision')
            if geometry[0].tag != expected_geometry:
                raise RuntimeError(
                    f'Roller link {name} expected {expected_geometry} '
                    f'collision, found {geometry[0].tag}')
            # gz sdf -p drops the Gazebo-Classic friction vocabulary, so an
            # absent surface here means the contact parameters silently
            # reverted to the sdformat defaults instead of the configured
            # values. Fail loudly rather than simulate the wrong contact.
            friction_mu = collision.findtext('surface/friction/ode/mu')
            if friction_mu is None:
                raise RuntimeError(
                    f'Roller link {name} has a collision with no '
                    'surface/friction/ode/mu; contact friction would silently '
                    'fall back to the sdformat default'
                )
            if not math.isclose(
                    float(friction_mu), expected_mu, rel_tol=1e-9,
                    abs_tol=1e-12):
                raise RuntimeError(
                    f'Roller link {name} has contact mu {friction_mu}, '
                    f'expected {expected_mu}'
                )
    expected_total = 360 if collision_model == 'barrel' else 40
    if collision_count != expected_total:
        raise RuntimeError(
            f'Expected {expected_total} roller collisions, '
            f'found {collision_count}'
        )

    friction_directions = root.findall('.//fdir1')
    if friction_directions:
        raise RuntimeError(
            f'Explicit roller SDF must contain no fdir1 elements; '
            f'found {len(friction_directions)}'
        )


def main():
    """Generate and validate Gazebo SDF with 40 explicit passive rollers."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--xacro', required=True)
    parser.add_argument('--controllers', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--roller-joint-damping', default='0.0')
    parser.add_argument('--roller-joint-friction', default='0.0')
    parser.add_argument('--roller-contact-mu', default='1.0')
    parser.add_argument('--roller-contact-mu2', default='')
    parser.add_argument('--roller-contact-kp', default='100000.0')
    parser.add_argument('--roller-contact-kd', default='10.0')
    parser.add_argument('--roller-torsional-coefficient', default='0.0')
    parser.add_argument(
        '--roller-collision-model',
        choices=('cylinder', 'barrel', 'husarion_cylinder'),
        default='barrel')
    parser.add_argument('--wheel-contact-mu', default='0.8')
    parser.add_argument('--wheel-contact-mu2', default='0.2')
    parser.add_argument('--wheel-contact-slip1', default='0.0')
    parser.add_argument('--wheel-contact-slip2', default='0.0')
    parser.add_argument('--front-left-roller-phase', default='0.22193969')
    parser.add_argument('--front-right-roller-phase', default='0.48030419')
    parser.add_argument('--rear-right-roller-phase', default='0.19668582')
    parser.add_argument('--rear-left-roller-phase', default='0.24790784')
    args = parser.parse_args()

    urdf = subprocess.run(
        [
            'xacro',
            args.xacro,
            'use_gazebo:=true',
            'controllers_file:=' + args.controllers,
            'roller_joint_damping:=' + args.roller_joint_damping,
            'roller_joint_friction:=' + args.roller_joint_friction,
            'roller_contact_mu:=' + args.roller_contact_mu,
            'roller_collision_model:=' + args.roller_collision_model,
            'wheel_contact_mu:=' + args.wheel_contact_mu,
            'wheel_contact_mu2:=' + args.wheel_contact_mu2,
            'wheel_contact_slip1:=' + args.wheel_contact_slip1,
            'wheel_contact_slip2:=' + args.wheel_contact_slip2,
            'front_left_roller_phase:=' + args.front_left_roller_phase,
            'front_right_roller_phase:=' + args.front_right_roller_phase,
            'rear_right_roller_phase:=' + args.rear_right_roller_phase,
            'rear_left_roller_phase:=' + args.rear_left_roller_phase,
        ],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf') as urdf_file:
        urdf_file.write(urdf)
        urdf_file.flush()
        sdf = subprocess.run(
            ['gz', 'sdf', '-p', urdf_file.name],
            check=True,
            capture_output=True,
            text=True,
        ).stdout

    root = ET.fromstring(sdf)
    wheel_file_uris = _wheel_file_uris()
    referenced_wheel_files = []
    for uri in root.findall('.//uri'):
        for prefix in WHEEL_URI_PREFIXES:
            if uri.text and uri.text.startswith(prefix):
                mesh_name = uri.text[len(prefix):]
                if mesh_name not in wheel_file_uris:
                    raise RuntimeError(
                        'Unresolved wheel visual URI: ' + uri.text
                    )
                uri.text = wheel_file_uris[mesh_name]
                referenced_wheel_files.append(mesh_name)
                break

    if (
        len(referenced_wheel_files) != 4
        or set(referenced_wheel_files) != WHEEL_FILES
        or any(referenced_wheel_files.count(name) != 1 for name in WHEEL_FILES)
    ):
        missing = sorted(WHEEL_FILES - set(referenced_wheel_files))
        raise RuntimeError(
            'Generated SDF must reference each of the four wheel visual '
            'meshes exactly once; missing: ' + ', '.join(missing)
        )

    for uri in root.findall('.//uri'):
        if uri.text and any(
                uri.text.startswith(prefix) for prefix in WHEEL_URI_PREFIXES):
            raise RuntimeError('Unresolved wheel visual URI: ' + uri.text)

    if args.roller_collision_model == 'husarion_cylinder':
        inject_wheel_surfaces(
            root,
            args.wheel_contact_mu,
            args.wheel_contact_mu2,
            args.wheel_contact_slip1,
            args.wheel_contact_slip2,
        )
        validate_husarion_cylinder(
            root,
            float(args.wheel_contact_mu),
            float(args.wheel_contact_mu2),
        )
        _write_sdf(root, args.output)
        return

    contact_mu = float(args.roller_contact_mu)
    contact_mu2 = float(args.roller_contact_mu2 or args.roller_contact_mu)
    inject_roller_surfaces(
        root,
        contact_mu,
        contact_mu2,
        args.roller_contact_kp,
        args.roller_contact_kd,
        args.roller_torsional_coefficient,
    )

    _validate_explicit_rollers(root, args.roller_collision_model, contact_mu)
    _write_sdf(root, args.output)


if __name__ == '__main__':
    main()
