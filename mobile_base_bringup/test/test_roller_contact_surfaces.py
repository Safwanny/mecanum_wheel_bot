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

"""Verify roller contact surfaces are injected into the generated SDF."""

# gz sdf -p discards the Gazebo-Classic mu1/mu2/kp/kd extension tags, so the
# simulation once ran every roller collision on the sdformat default mu of 1.0
# while the configuration claimed 0.8. These checks operate on the generator's
# pure functions, so they need no Gazebo installation and run in normal CI.

import importlib.util
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


WHEEL_NAMES = ('front_left', 'front_right', 'rear_right', 'rear_left')


def load_generator(package_dir):
    """Import generate_sim_sdf.py by path; it installs as a program."""
    path = Path(package_dir) / 'scripts' / 'generate_sim_sdf.py'
    spec = importlib.util.spec_from_file_location('generate_sim_sdf', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def build_roller_model(collision_count):
    """Build a minimal SDF tree with 40 rollers and no contact surfaces."""
    root = ET.Element('sdf', {'version': '1.9'})
    model = ET.SubElement(root, 'model', {'name': 'mobile_base'})
    for wheel_name in WHEEL_NAMES:
        for index in range(10):
            link = ET.SubElement(
                model, 'link',
                {'name': f'{wheel_name}_roller_{index}_link'})
            for collision_index in range(collision_count):
                collision = ET.SubElement(
                    link, 'collision',
                    {'name': f'collision_{collision_index}'})
                geometry = ET.SubElement(collision, 'geometry')
                ET.SubElement(geometry, 'sphere')
            joint = ET.SubElement(
                model, 'joint',
                {'name': f'{wheel_name}_roller_{index}_joint',
                 'type': 'revolute'})
            ET.SubElement(joint, 'parent').text = f'{wheel_name}_wheel_link'
            ET.SubElement(joint, 'child').text = \
                f'{wheel_name}_roller_{index}_link'
        ET.SubElement(
            model, 'joint',
            {'name': f'{wheel_name}_wheel_joint', 'type': 'revolute'})
    return root


def test_every_roller_collision_receives_a_surface(generator):
    root = build_roller_model(collision_count=9)
    assert root.findall('.//surface') == []

    injected = generator.inject_roller_surfaces(root, 0.8, 0.8, 1e5, 10.0, 0.0)

    assert injected == 360
    surfaces = root.findall('.//surface')
    assert len(surfaces) == 360
    for surface in surfaces:
        assert float(surface.findtext('friction/ode/mu')) == 0.8
        assert float(surface.findtext('friction/ode/mu2')) == 0.8
        assert float(surface.findtext('friction/torsional/coefficient')) == 0.0
        assert float(surface.findtext('contact/ode/kp')) == 1e5
        assert float(surface.findtext('contact/ode/kd')) == 10.0


def test_injection_is_idempotent(generator):
    root = build_roller_model(collision_count=1)
    generator.inject_roller_surfaces(root, 0.8, 0.8, 1e5, 10.0, 0.0)
    generator.inject_roller_surfaces(root, 0.6, 0.6, 1e5, 10.0, 0.0)

    surfaces = root.findall('.//surface')
    assert len(surfaces) == 40, 'repeat injection must replace, not stack'
    assert all(
        float(surface.findtext('friction/ode/mu')) == 0.6
        for surface in surfaces
    )


def test_anisotropic_values_are_preserved(generator):
    root = build_roller_model(collision_count=1)
    generator.inject_roller_surfaces(root, 0.9, 0.2, 1e5, 10.0, 0.0)
    surface = root.find('.//surface')
    assert float(surface.findtext('friction/ode/mu')) == 0.9
    assert float(surface.findtext('friction/ode/mu2')) == 0.2


def test_invalid_contact_values_are_rejected(generator):
    for bad in (-0.1, float('nan'), float('inf')):
        root = build_roller_model(collision_count=1)
        try:
            generator.inject_roller_surfaces(root, bad, 0.8, 1e5, 10.0, 0.0)
        except RuntimeError:
            continue
        raise AssertionError(f'mu={bad} must be rejected')


def test_missing_surface_fails_validation(generator):
    """The regression guard: a model with no surfaces must not be emitted."""
    root = build_roller_model(collision_count=9)
    try:
        generator._validate_explicit_rollers(root, 'barrel', 0.8)
    except RuntimeError as error:
        assert 'surface/friction/ode/mu' in str(error)
        return
    raise AssertionError(
        'validation must reject rollers without a contact surface')


def test_wrong_mu_fails_validation(generator):
    root = build_roller_model(collision_count=9)
    generator.inject_roller_surfaces(root, 1.0, 1.0, 1e5, 10.0, 0.0)
    try:
        generator._validate_explicit_rollers(root, 'barrel', 0.8)
    except RuntimeError as error:
        assert 'expected 0.8' in str(error)
        return
    raise AssertionError('validation must reject an unexpected contact mu')


def test_injected_model_passes_validation(generator):
    root = build_roller_model(collision_count=9)
    generator.inject_roller_surfaces(root, 0.8, 0.8, 1e5, 10.0, 0.0)
    generator._validate_explicit_rollers(root, 'barrel', 0.8)


def build_husarion_model():
    """No roller links; one collision on each driven hub."""
    root = ET.Element('sdf', {'version': '1.9'})
    model = ET.SubElement(root, 'model', {'name': 'mobile_base'})
    ET.SubElement(model, 'link', {'name': 'base_footprint'})
    for wheel_name in WHEEL_NAMES:
        link = ET.SubElement(
            model, 'link', {'name': f'{wheel_name}_wheel_link'})
        collision = ET.SubElement(
            link, 'collision', {'name': 'wheel_collision'})
        geometry = ET.SubElement(collision, 'geometry')
        ET.SubElement(geometry, 'cylinder')
    return root


def test_husarion_uses_the_upstream_handedness_mapping(generator):
    """FL/RR share one friction direction, FR/RL the other."""
    root = build_husarion_model()
    assert generator.inject_wheel_surfaces(root, 0.8, 0.2, 0.0, 0.0) == 4

    # Matches the shipping ROSbot XL wheel description.
    expected = {'front_left': '1 -1 0', 'front_right': '1 1 0',
                'rear_right': '1 -1 0', 'rear_left': '1 1 0'}
    for wheel, want in expected.items():
        link = next(item for item in root.findall('.//link')
                    if item.attrib.get('name') == f'{wheel}_wheel_link')
        fdir1 = link.find('collision/surface/friction/ode/fdir1')
        assert fdir1.text == want, f'{wheel}: {fdir1.text!r} != {want!r}'
        # The literal attribute name matters: dartsim looks it up by string,
        # so an ElementTree-namespaced "ns0:expressed_in" silently does
        # nothing and the friction direction spins with the wheel.
        assert fdir1.attrib.get('gz:expressed_in') == 'base_footprint'
    generator.validate_husarion_cylinder(root, 0.8, 0.2)


def test_husarion_rejects_unresolvable_friction_frame(generator):
    """base_link does not survive URDF-to-SDF; referencing it is inert."""
    root = build_husarion_model()
    generator.inject_wheel_surfaces(root, 0.8, 0.2, 0.0, 0.0)
    base = next(item for item in root.findall('.//link')
                if item.attrib.get('name') == 'base_footprint')
    root.find('model').remove(base)
    try:
        generator.validate_husarion_cylinder(root, 0.8, 0.2)
    except RuntimeError as error:
        assert 'not a link' in str(error)
        return
    raise AssertionError('an unresolvable friction frame must be rejected')


def test_husarion_rejects_isotropic_friction(generator):
    root = build_husarion_model()
    try:
        generator.inject_wheel_surfaces(root, 0.8, 0.8, 0.0, 0.0)
    except RuntimeError as error:
        assert 'mu != mu2' in str(error)
        return
    raise AssertionError('mu == mu2 cannot produce mecanum motion')


def test_husarion_rejects_leftover_rollers(generator):
    """Both mechanisms acting at once would double-count the contact."""
    root = build_husarion_model()
    generator.inject_wheel_surfaces(root, 0.8, 0.2, 0.0, 0.0)
    ET.SubElement(
        root.find('model'), 'link', {'name': 'front_left_roller_0_link'})
    try:
        generator.validate_husarion_cylinder(root, 0.8, 0.2)
    except RuntimeError as error:
        assert 'no roller links' in str(error)
        return
    raise AssertionError('roller links must not survive in this mode')


def main():
    package_dir = sys.argv[1] if len(sys.argv) > 1 else str(
        Path(__file__).resolve().parent.parent)
    generator = load_generator(package_dir)
    tests = [value for name, value in sorted(globals().items())
             if name.startswith('test_')]
    for test in tests:
        test(generator)
        print(f'PASS {test.__name__}')
    print(f'{len(tests)} roller contact surface checks passed')


if __name__ == '__main__':
    main()
