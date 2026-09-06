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

"""Check that every shipped world is local and sensor-ready."""

from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET


REQUIRED_SYSTEMS = {
    'gz-sim-physics-system',
    'gz-sim-user-commands-system',
    'gz-sim-scene-broadcaster-system',
    'gz-sim-sensors-system',
    'gz-sim-imu-system',
}
EXPECTED_WORLDS = {
    'navigation_basic.sdf',
    'my_world.sdf',
}


def main():
    package = Path(sys.argv[1])
    worlds = package / 'worlds'
    paths = sorted(worlds.glob('*.sdf'))
    assert {path.name for path in paths} == EXPECTED_WORLDS

    for path in paths:
        root = ET.parse(path).getroot()
        world = root.find('world')
        assert world is not None
        assert not world.findall('.//include'), (
            f'{path.name} must not depend on downloaded Fuel models'
        )

        plugins = {
            plugin.get('filename'): plugin for plugin in world.findall('plugin')
        }
        assert REQUIRED_SYSTEMS <= plugins.keys()
        sensors = plugins['gz-sim-sensors-system']
        assert sensors.findtext('render_engine') == 'ogre2'
        assert world.find("./model[@name='ground_plane']") is not None

        result = subprocess.run(
            ['gz', 'sdf', '-k', str(path)],
            check=False,
            capture_output=True,
            text=True,
        )
        assert result.returncode == 0, result.stdout + result.stderr

    cmake = (package / 'CMakeLists.txt').read_text(encoding='utf-8')
    assert 'install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})' in cmake


if __name__ == '__main__':
    main()
