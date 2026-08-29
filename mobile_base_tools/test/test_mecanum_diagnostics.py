import math

import pytest

from mobile_base_tools.mecanum_diagnostics import (
    chassis_motion_metrics,
    classify_multi_segment,
    classify_root_cause,
    expected_wheel_velocities,
    ideal_planar_pose,
    multi_segment_loop_closure_metrics,
    wheel_tracking_metrics,
)


def test_expected_wheel_signs_cover_primitives():
    forward = expected_wheel_velocities(1.0, 0.0, 0.0, 0.5, 0.4)
    assert list(forward.values()) == [2.0, 2.0, 2.0, 2.0]
    left = expected_wheel_velocities(0.0, 1.0, 0.0, 0.5, 0.4)
    assert left == {
        'front_left': -2.0, 'front_right': 2.0,
        'rear_right': -2.0, 'rear_left': 2.0}
    rotate = expected_wheel_velocities(0.0, 0.0, 1.0, 0.5, 0.4)
    assert rotate == {
        'front_left': -0.8, 'front_right': 0.8,
        'rear_right': 0.8, 'rear_left': -0.8}


def test_ideal_pose_integrates_body_twist():
    assert ideal_planar_pose(0.2, -0.1, 0.0, 2.0) == {
        'x': 0.4, 'y': -0.2, 'yaw': 0.0}
    arc = ideal_planar_pose(1.0, 0.0, math.pi / 2.0, 1.0)
    assert arc['x'] == pytest.approx(2.0 / math.pi)
    assert arc['y'] == pytest.approx(2.0 / math.pi)


def sample(elapsed, expected=2.0, actual=2.0, gt_x=None, odom_x=None):
    data = {
        'simulation_elapsed': elapsed,
        'commanded_linear_x': 1.0,
        'commanded_linear_y': 0.0,
        'commanded_angular_z': 0.0,
        'ground_truth_x': elapsed if gt_x is None else gt_x,
        'ground_truth_y': 0.0,
        'ground_truth_yaw': 0.0,
        'odometry_x': elapsed if odom_x is None else odom_x,
        'odometry_y': 0.0,
        'odometry_yaw': 0.0,
    }
    for wheel in ('front_left', 'front_right', 'rear_right', 'rear_left'):
        data[f'expected_{wheel}_wheel_velocity'] = expected
        data[f'actual_{wheel}_wheel_velocity'] = actual
    return data


def test_wheel_tracking_and_chassis_metrics_are_zero_for_ideal_motion():
    samples = [sample(0.0), sample(0.5), sample(1.0)]
    wheels = wheel_tracking_metrics(samples)
    assert all(item['velocity_rmse_rad_s'] == 0.0 for item in wheels.values())
    chassis = chassis_motion_metrics(samples, 'longitudinal')
    assert chassis['endpoint_error_metres'] == pytest.approx(0.0)
    assert chassis['odometry_endpoint_error_metres'] == pytest.approx(0.0)
    assert classify_root_cause(wheels, chassis)['case'] == 'within_threshold'


def test_classification_distinguishes_wheels_contacts_and_odometry():
    bad_wheels = wheel_tracking_metrics([
        sample(0.0, actual=1.0), sample(1.0, actual=1.0)])
    chassis = chassis_motion_metrics(
        [sample(0.0), sample(1.0)], 'longitudinal')
    assert classify_root_cause(bad_wheels, chassis)['case'] == 'A'

    good_wheels = wheel_tracking_metrics([sample(0.0), sample(1.0)])
    contact = chassis_motion_metrics([
        sample(0.0, gt_x=0.0), sample(1.0, gt_x=0.5)], 'longitudinal')
    assert classify_root_cause(good_wheels, contact)['case'] == 'B'

    odometry = chassis_motion_metrics([
        sample(0.0), sample(1.0, odom_x=0.5)], 'longitudinal')
    assert classify_root_cause(good_wheels, odometry)['case'] == 'C'


def test_multi_segment_metrics_report_loop_closure_without_case_label():
    wheels = wheel_tracking_metrics([sample(0.0), sample(1.0)])
    run = {
        'ground_truth_delta_x': 0.003,
        'ground_truth_delta_y': -0.004,
        'ground_truth_delta_yaw': 0.01,
        'odometry_delta_x': 0.006,
        'odometry_delta_y': -0.012,
        'odometry_delta_yaw': 0.015,
        'position_error_norm': math.hypot(0.003, -0.008),
        'yaw_error': 0.005,
        'distance_travelled_ground_truth': 3.96,
        'distance_travelled_odometry': 4.01,
    }
    metrics = multi_segment_loop_closure_metrics(run, 123)
    assert metrics['measurement_scope'] == 'multi_segment_loop_closure'
    assert metrics['sample_count'] == 123
    assert metrics['ground_truth_loop_closure_translation_metres'] \
        == pytest.approx(0.005)
    assert metrics['odometry_endpoint_error_metres'] \
        == pytest.approx(math.hypot(0.003, -0.008))
    classification = classify_multi_segment(wheels)
    assert classification['case'] == 'not_applicable'
    assert classification['normalized_chassis_error'] is None
    assert classification['maximum_wheel_normalized_rmse'] == 0.0


def test_multi_segment_metrics_reject_incomplete_results():
    with pytest.raises(ValueError, match='incomplete'):
        multi_segment_loop_closure_metrics({}, 0)
