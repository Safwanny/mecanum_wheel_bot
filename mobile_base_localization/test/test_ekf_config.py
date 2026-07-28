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

"""Validate the Phase 1 EKF contract."""

from pathlib import Path
import sys

import yaml


def main() -> None:
    """Assert frames, inputs, output behavior, and enabled state fields."""
    path = Path(sys.argv[1])
    params = yaml.safe_load(path.read_text())['ekf_filter_node'][
        'ros__parameters'
    ]
    assert params['two_d_mode'] is True
    assert params['publish_tf'] is True
    assert params['world_frame'] == 'odom'
    assert params['odom_frame'] == 'odom'
    assert params['base_link_frame'] == 'base_footprint'
    assert params['odom0'] == '/mobile_base_controller/odometry'
    assert params['imu0'] == '/imu/data'
    assert 'ground_truth' not in str(params)

    expected_odom = [False] * 15
    for index in (6, 7, 11):
        expected_odom[index] = True
    expected_imu = [False] * 15
    expected_imu[11] = True
    assert params['odom0_config'] == expected_odom
    assert params['imu0_config'] == expected_imu
    assert len(params['process_noise_covariance']) == 225


if __name__ == '__main__':
    main()
