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

"""Verify canonical anisotropic wheel surfaces and the public launch API."""

import importlib.util
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


WHEEL_NAMES = ('front_left', 'front_right', 'rear_right', 'rear_left')


def load_generator(package_dir):
    path = Path(package_dir) / 'scripts' / 'generate_sim_sdf.py'
    spec = importlib.util.spec_from_file_location('generate_sim_sdf', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def build_model(collision_counts=None):
    collision_counts = collision_counts or {wheel: 1 for wheel in WHEEL_NAMES}
    root = ET.Element('sdf', {'version': '1.9'})
    model = ET.SubElement(root, 'model', {'name': 'mobile_base'})
    ET.SubElement(model, 'link', {'name': 'base_footprint'})
    for wheel in WHEEL_NAMES:
        link = ET.SubElement(model, 'link', {'name': f'{wheel}_wheel_link'})
        for index in range(collision_counts[wheel]):
            collision = ET.SubElement(
                link, 'collision', {'name': f'wheel_collision_{index}'})
            geometry = ET.SubElement(collision, 'geometry')
            cylinder = ET.SubElement(geometry, 'cylinder')
            ET.SubElement(cylinder, 'radius').text = '0.03074443'
            ET.SubElement(cylinder, 'length').text = '0.03360543'
        ET.SubElement(
            model, 'joint',
            {'name': f'{wheel}_wheel_joint', 'type': 'revolute'})
    return root


def test_surfaces_and_direction_pattern(generator):
    root = build_model()
    assert generator.inject_wheel_surfaces(root) == 4
    expected = {
        'front_left': '1 -1 0', 'front_right': '1 1 0',
        'rear_right': '1 -1 0', 'rear_left': '1 1 0',
    }
    for wheel, direction in expected.items():
        link = next(
            item for item in root.findall('.//link')
            if item.attrib.get('name') == f'{wheel}_wheel_link')
        ode = link.find('collision/surface/friction/ode')
        assert float(ode.findtext('mu')) == 0.8
        assert float(ode.findtext('mu2')) == 0.2
        assert float(ode.findtext('slip1')) == 0.0
        assert float(ode.findtext('slip2')) == 0.0
        fdir1 = ode.find('fdir1')
        assert fdir1.text == direction
        assert fdir1.attrib['gz:expressed_in'] == 'base_footprint'
    generator.validate_mecanum_wheel_contact(root)


def test_injection_replaces_existing_surface(generator):
    root = build_model()
    generator.inject_wheel_surfaces(root)
    generator.inject_wheel_surfaces(root)
    assert len(root.findall('.//surface')) == 4


def test_invalid_or_isotropic_friction_is_rejected(generator):
    for bad in (-0.1, float('nan'), float('inf')):
        try:
            generator.inject_wheel_surfaces(build_model(), mu=bad)
        except RuntimeError:
            pass
        else:
            raise AssertionError(f'mu={bad} must be rejected')
    try:
        generator.inject_wheel_surfaces(build_model(), mu=0.8, mu2=0.8)
    except RuntimeError as error:
        assert 'mu != mu2' in str(error)
    else:
        raise AssertionError('isotropic friction must be rejected')


def test_missing_or_duplicate_collision_is_rejected(generator):
    for count in (0, 2):
        counts = {wheel: 1 for wheel in WHEEL_NAMES}
        counts['front_left'] = count
        try:
            generator.inject_wheel_surfaces(build_model(counts))
        except RuntimeError as error:
            assert 'exactly one collision' in str(error)
        else:
            raise AssertionError(f'collision count {count} must be rejected')


def test_passive_roller_body_is_rejected(generator):
    root = build_model()
    generator.inject_wheel_surfaces(root)
    ET.SubElement(
        root.find('model'), 'link', {'name': 'front_left_roller_0_link'})
    try:
        generator.validate_mecanum_wheel_contact(root)
    except RuntimeError as error:
        assert 'Passive roller body' in str(error)
    else:
        raise AssertionError('passive roller links must be rejected')


def test_launch_files_have_no_contact_model_api(generator, package_dir):
    del generator
    launch_dir = Path(package_dir) / 'launch'
    obsolete = (
        'roller_' + 'collision_model',
        'roller_' + 'joint_damping',
        'roller_' + 'joint_friction',
        'roller_' + 'phase',
        'wheel_' + 'contact_mu',
    )
    for name in (
            'simulation.launch.py', 'mapping.launch.py',
            'localization.launch.py', 'camera_view.launch.py'):
        text = (launch_dir / name).read_text(encoding='utf-8')
        for token in obsolete:
            assert token not in text, f'{token} remains in {name}'


def main():
    package_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else \
        Path(__file__).resolve().parent.parent
    generator = load_generator(package_dir)
    tests = [value for name, value in sorted(globals().items())
             if name.startswith('test_')]
    for test in tests:
        if test.__name__ == 'test_launch_files_have_no_contact_model_api':
            test(generator, package_dir)
        else:
            test(generator)
        print(f'PASS {test.__name__}')
    print(f'{len(tests)} mecanum wheel contact checks passed')


if __name__ == '__main__':
    main()
