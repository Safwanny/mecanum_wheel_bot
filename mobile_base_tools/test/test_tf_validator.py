from mobile_base_tools.tf_validator import (
    evaluate_contract,
    graph_failures,
    TransformStatistics,
)


def updated_statistics():
    return TransformStatistics(
        sample_count=3,
        first_timestamp=1.0,
        last_timestamp=1.2,
    )


def valid_transforms():
    dynamic = {
        ('odom', 'base_footprint'): updated_statistics(),
        ('base_link', 'front_left_wheel_link'): updated_statistics(),
        ('base_link', 'front_right_wheel_link'): updated_statistics(),
        ('base_link', 'rear_right_wheel_link'): updated_statistics(),
        ('base_link', 'rear_left_wheel_link'): updated_statistics(),
    }
    static = {
        ('base_footprint', 'base_link'): TransformStatistics(sample_count=1),
        ('base_link', 'imu_link'): TransformStatistics(sample_count=1),
        ('base_link', 'lidar_link'): TransformStatistics(sample_count=1),
        ('base_link', 'camera_link'): TransformStatistics(sample_count=1),
        ('camera_link', 'camera_optical_frame'):
            TransformStatistics(sample_count=1),
    }
    return dynamic, static


def test_valid_fused_contract_passes():
    dynamic, static = valid_transforms()
    result = evaluate_contract(dynamic, static, 'fused', {
        'controller_enable_odom_tf': False,
        'ekf_publish_tf': True,
        'dynamic_publishers': [
            '/ekf_filter_node', '/mobile_base_controller',
            '/robot_state_publisher'],
    })
    assert result['passed'], result['failure_reasons']


def test_valid_raw_contract_passes():
    dynamic, static = valid_transforms()
    result = evaluate_contract(dynamic, static, 'raw', {
        'controller_enable_odom_tf': True,
        'ekf_publish_tf': None,
        'dynamic_publishers': [
            '/mobile_base_controller', '/robot_state_publisher'],
    })
    assert result['passed'], result['failure_reasons']


def test_missing_frame_and_stalled_dynamic_transform_fail():
    dynamic, static = valid_transforms()
    static.pop(('base_link', 'imu_link'))
    dynamic[('odom', 'base_footprint')] = TransformStatistics(
        sample_count=2, first_timestamp=1.0, last_timestamp=1.0)
    result = evaluate_contract(dynamic, static, 'fused', {
        'controller_enable_odom_tf': False,
        'ekf_publish_tf': True,
        'dynamic_publishers': ['/ekf_filter_node', '/robot_state_publisher'],
    })
    assert not result['passed']
    assert any('imu_link' in reason for reason in result['failure_reasons'])
    assert any('did not update' in reason for reason in result['failure_reasons'])


def test_duplicate_authority_and_unexpected_publisher_fail():
    dynamic, static = valid_transforms()
    result = evaluate_contract(dynamic, static, 'fused', {
        'controller_enable_odom_tf': True,
        'ekf_publish_tf': True,
        'dynamic_publishers': [
            '/ekf_filter_node', '/mobile_base_controller',
            '/robot_state_publisher', '/rogue_tf'],
    })
    assert not result['passed']
    assert any('exactly one enabled owner' in reason
               for reason in result['failure_reasons'])
    assert any('unexpected /tf publishers' in reason
               for reason in result['failure_reasons'])


def test_map_ground_truth_and_loops_fail():
    dynamic, static = valid_transforms()
    dynamic[('map', 'odom')] = updated_statistics()
    static[('camera_optical_frame', 'base_footprint')] = (
        TransformStatistics(sample_count=1))
    static[('base_link', 'gazebo_ground_truth')] = (
        TransformStatistics(sample_count=1))
    result = evaluate_contract(dynamic, static, 'fused', {
        'controller_enable_odom_tf': False,
        'ekf_publish_tf': True,
        'dynamic_publishers': ['/ekf_filter_node', '/robot_state_publisher'],
    })
    assert not result['passed']
    assert any('map -> odom' in reason for reason in result['failure_reasons'])
    assert any('contaminate' in reason for reason in result['failure_reasons'])
    assert any('loop' in reason for reason in result['failure_reasons'])


def test_graph_rejects_multiple_parents():
    failures = graph_failures({('a', 'child'), ('b', 'child')})
    assert any('multiple parents' in reason for reason in failures)
