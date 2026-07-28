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

"""Run one short bounded Gazebo-to-odometry evaluation."""

import json
from pathlib import Path
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import yaml


OUTPUT_DIR = Path(tempfile.mkdtemp(prefix='mobile_base_eval_test_'))
CONFIG_FILE = OUTPUT_DIR / 'odometry_tests.yaml'


def generate_test_description():
    tools_share = Path(get_package_share_directory('mobile_base_tools'))
    configuration = yaml.safe_load(
        (tools_share / 'config' / 'odometry_tests.yaml').read_text()
    )
    forward = configuration['profiles']['forward_1m']
    forward['termination']['target'] = 0.05
    forward['profile_timeout'] = 8.0
    forward['settle_before'] = 0.5
    forward['settle_after'] = 0.5
    forward['repetitions'] = 1
    CONFIG_FILE.write_text(yaml.safe_dump(configuration))

    launch_file = (
        get_package_share_directory('mobile_base_bringup')
        + '/launch/odometry_evaluation.launch.py'
    )
    evaluation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            'world': 'empty',
            'test_profile': 'forward_1m',
            'repetitions': '1',
            'config_file': str(CONFIG_FILE),
            'output_dir': str(OUTPUT_DIR),
            'gui': 'false',
            'rviz': 'false',
            'shutdown_on_complete': 'false',
        }.items(),
    )
    return LaunchDescription([
        evaluation,
        launch_testing.actions.ReadyToTest(),
    ])


class TestOdometryEvaluationRuntime(unittest.TestCase):
    """Verify both pose sources move and a complete report is produced."""

    def test_short_forward_report(self):
        report_path = OUTPUT_DIR / 'summary.json'
        final_plot = (
            OUTPUT_DIR / 'plots' / 'final_displacement_comparison.png'
        )
        deadline = time.monotonic() + 90.0
        while time.monotonic() < deadline and not final_plot.is_file():
            time.sleep(0.1)
        self.assertTrue(report_path.is_file(), 'summary.json was not produced')
        self.assertTrue(final_plot.is_file(), 'summary plots were not completed')
        time.sleep(0.5)
        report = json.loads(report_path.read_text())
        self.assertEqual(report['ground_truth_source'].split()[0], 'Gazebo')
        self.assertEqual(len(report['runs']), 1)
        run = report['runs'][0]
        self.assertEqual(run['completion_status'], 'completed')
        self.assertGreater(
            abs(run['ground_truth_delta_x'])
            + abs(run['ground_truth_delta_y']),
            0.02,
        )
        self.assertGreater(
            abs(run['odometry_delta_x']) + abs(run['odometry_delta_y']),
            0.02,
        )
        self.assertGreater(
            abs(run['filtered_delta_x']) + abs(run['filtered_delta_y']),
            0.02,
        )
        self.assertIn('position_error_improvement', run)
        self.assertTrue((OUTPUT_DIR / 'runs.csv').is_file())
        self.assertTrue((OUTPUT_DIR / 'summary.csv').is_file())
        self.assertTrue(any((OUTPUT_DIR / 'plots').glob('*.png')))
