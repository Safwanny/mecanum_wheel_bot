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

"""Validate the static Phase 2 frame, mode, sensor, and launch contracts."""

from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml


def _read(path):
    return Path(path).read_text(encoding='utf-8')


def _parameters(path, node_name):
    return yaml.safe_load(_read(path))[node_name]['ros__parameters']


def main():
    localization = Path(sys.argv[1])
    bringup = Path(sys.argv[2])

    slam = _parameters(
        localization / 'config' / 'slam_toolbox.yaml', 'slam_toolbox'
    )
    assert slam['mode'] == 'mapping'
    assert slam['map_frame'] == 'map'
    assert slam['odom_frame'] == 'odom'
    assert slam['base_frame'] == 'base_footprint'
    assert slam['scan_topic'] == '/scan'
    assert slam['min_laser_range'] == 0.10
    assert slam['max_laser_range'] == 4.0
    assert slam['scan_queue_size'] > 0
    assert slam['transform_publish_period'] > 0.0
    assert slam['use_scan_matching'] is True
    assert slam['do_loop_closing'] is True
    assert slam['use_map_saver'] is True

    amcl = _parameters(localization / 'config' / 'amcl.yaml', 'amcl')
    assert amcl['global_frame_id'] == 'map'
    assert amcl['odom_frame_id'] == 'odom'
    assert amcl['base_frame_id'] == 'base_footprint'
    assert amcl['scan_topic'] == '/scan'
    assert amcl['robot_model_type'] == 'nav2_amcl::OmniMotionModel'
    assert amcl['tf_broadcast'] is True
    assert amcl['laser_max_range'] == 4.0
    assert amcl['alpha5'] > 0.0
    assert amcl['set_initial_pose'] is False

    map_server = _parameters(
        localization / 'config' / 'map_server.yaml', 'map_server'
    )
    assert map_server['frame_id'] == 'map'
    assert map_server['topic_name'] == 'map'

    mapping_launch = _read(localization / 'launch' / 'mapping.launch.py')
    assert 'online_async_launch.py' in mapping_launch
    assert "FindPackageShare('slam_toolbox')" in mapping_launch
    assert "package='nav2_amcl'" not in mapping_launch
    assert "package='nav2_map_server'" not in mapping_launch

    amcl_launch = _read(localization / 'launch' / 'amcl.launch.py')
    assert "package='nav2_map_server'" in amcl_launch
    assert "package='nav2_amcl'" in amcl_launch
    assert "package='nav2_lifecycle_manager'" in amcl_launch
    assert "'node_names': ['map_server', 'amcl']" in amcl_launch
    assert "DeclareLaunchArgument(\n            'map'," in amcl_launch
    assert "package='slam_toolbox'" not in amcl_launch

    mapping_wrapper = _read(bringup / 'launch' / 'mapping.launch.py')
    localization_wrapper = _read(
        bringup / 'launch' / 'localization.launch.py'
    )
    for wrapper in (mapping_wrapper, localization_wrapper):
        assert "'localization': 'true'" in wrapper
        assert "'rviz': 'false'" in wrapper
        assert "'phase2_rviz', LaunchConfiguration('rviz')" in wrapper
        assert "'rviz': LaunchConfiguration('phase2_rviz')" in wrapper
        assert (
            "DeclareLaunchArgument('render_engine', default_value='ogre2')"
            in wrapper
        )
        assert 'simulation.launch.py' in wrapper
    assert 'mapping.launch.py' in mapping_wrapper
    assert 'amcl.launch.py' in localization_wrapper

    mapping_rviz = _read(localization / 'rviz' / 'mapping.rviz')
    assert 'Name: MAPPING MODE' in mapping_rviz
    assert 'Fixed Frame: map' in mapping_rviz
    assert 'LIVE MAP - MAPPING MODE' in mapping_rviz
    assert 'slam_toolbox::SlamToolboxPlugin' in mapping_rviz
    assert '/scan' in mapping_rviz
    assert '/odometry/filtered' in mapping_rviz

    localization_rviz = _read(localization / 'rviz' / 'localization.rviz')
    assert 'Name: LOCALIZATION MODE' in localization_rviz
    assert 'SAVED MAP - LOCALIZATION MODE' in localization_rviz
    assert 'nav2_rviz_plugins/ParticleCloud' in localization_rviz
    assert 'rviz_default_plugins/PoseWithCovariance' in localization_rviz
    assert 'rviz_default_plugins/SetInitialPose' in localization_rviz
    assert '/initialpose' in localization_rviz
    assert '/amcl_pose' in localization_rviz

    package = ET.parse(localization / 'package.xml').getroot()
    dependencies = {element.text for element in package.findall('exec_depend')}
    assert {
        'nav2_amcl',
        'nav2_lifecycle_manager',
        'nav2_map_server',
        'nav2_rviz_plugins',
        'slam_toolbox',
    } <= dependencies


if __name__ == '__main__':
    main()
