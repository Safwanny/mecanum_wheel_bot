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

"""Check the cleanup sweep matches leftovers and spares everything else."""

from mobile_base_tools.stop_simulation import (
    PARENT_MARKERS,
    PATTERNS,
    PROTECTED,
    command_line,
)
import os


def matches(cmd):
    """Mirror the selection rule in targets(), without touching /proc."""
    if any(spare in cmd for spare in PROTECTED):
        return False
    return any(pattern in cmd for pattern in PATTERNS)


# Real command lines, copied from processes that were actually left running.
LEFTOVERS = (
    'gz sim -r -s /tmp/mobile_base_world_301119_ogre.sdf',
    '/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch mobile_base_bringup '
    'simulation.launch.py world:=proving_ground',
    '/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher '
    '--ros-args --params-file /tmp/launch_params_88e3jxg7',
    '/opt/ros/jazzy/lib/robot_localization/ekf_node --ros-args '
    '-r __node:=ekf_filter_node',
    '/usr/bin/python3 -c from ros2cli.daemon.daemonize import main; main() '
    '--name ros2-daemon --ros-domain-id 44',
    '/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge /clock@rosgraph_msgs',
    '/home/safwan/ros2_ws/install/mobile_base_tools/lib/mobile_base_tools/'
    'tof_floor_classifier --ros-args',
)


def test_every_observed_leftover_is_matched():
    for cmd in LEFTOVERS:
        assert matches(cmd), cmd


def test_the_gazebo_server_is_matched_despite_being_named_ruby():
    """Its process name is 'ruby'; only the command line says 'gz sim'.

    Three of these accumulated at ~640 MB each and starved the controller
    manager until launch tests failed for no visible reason.
    """
    assert matches('gz sim -r -s /tmp/mobile_base_world_1_ogre2.sdf')


def test_robot_state_publisher_is_matched_without_a_package_name():
    """Its command line names no package, only a generated parameter file."""
    cmd = ('/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher '
           '--ros-args --params-file /tmp/launch_params_abc123')
    assert matches(cmd)
    assert 'mobile_base' not in cmd


def test_editors_terminals_and_the_script_itself_are_spared():
    for cmd in (
        '/usr/share/code/code --unity-launch',
        '/home/safwan/.config/Claude/claude-code/claude --output-format',
        '/opt/ros/jazzy/lib/python3/stop_simulation --dry-run',
        '/usr/libexec/gvfsd-trash --spawner',
        'brave --type=gpu-process',
    ):
        assert not matches(cmd), cmd


def test_unrelated_processes_are_not_matched():
    for cmd in ('/usr/bin/bash', 'sshd: safwan@pts/0', '/usr/bin/python3 -m pytest'):
        assert not matches(cmd), cmd


def test_launch_parents_are_identified_so_they_die_first():
    parent = ('/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch '
              'mobile_base_bringup simulation.launch.py')
    child = ('/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher '
             '--ros-args --params-file /tmp/launch_params_x')
    assert any(marker in parent for marker in PARENT_MARKERS)
    assert not any(marker in child for marker in PARENT_MARKERS)


def test_command_line_reads_this_process_and_tolerates_a_dead_one():
    assert command_line(os.getpid()) is not None
    # A pid that cannot exist on Linux.
    assert command_line(2 ** 31 - 1) is None
