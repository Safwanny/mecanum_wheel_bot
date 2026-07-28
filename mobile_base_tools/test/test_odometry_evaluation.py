import json
import math

from mobile_base_tools.motion_profiles import Command, MotionProfile, Segment
from mobile_base_tools.odometry_evaluator import (
    calculate_run_metrics,
    termination_reached,
    timeout_expired,
)
from mobile_base_tools.pose_math import (
    data_is_stale,
    diagonal_errors,
    normalize_yaw,
    Pose2D,
    quaternion_to_yaw,
    relative_pose,
    square_closure,
    validate_monotonic_timestamp,
)
from mobile_base_tools.report_writer import (
    build_report,
    metric_statistics,
    write_report,
)
import pytest


def profile(kind='longitudinal'):
    return MotionProfile(
        name='synthetic',
        kind=kind,
        segments=(
            Segment(
                'segment',
                Command(0.1, 0.0, 0.0),
                'ground_truth_translation',
                1.0,
                20.0,
            ),
        ),
        settle_before=1.0,
        settle_after=1.0,
        repetitions=1,
        profile_timeout=20.0,
    )


def test_quaternion_to_yaw_and_normalization():
    yaw = 1.25
    assert quaternion_to_yaw(
        0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    ) == pytest.approx(yaw)
    assert normalize_yaw(3.0 * math.pi) == pytest.approx(math.pi)
    assert normalize_yaw(-3.0 * math.pi) == pytest.approx(-math.pi)


def test_initial_frame_displacement_uses_initial_yaw():
    initial = Pose2D(2.0, 3.0, math.pi / 2.0)
    final = Pose2D(2.0, 4.0, math.pi / 2.0)
    delta = relative_pose(initial, final)
    assert delta.x == pytest.approx(1.0)
    assert delta.y == pytest.approx(0.0, abs=1e-12)
    assert delta.yaw == pytest.approx(0.0)


def test_metrics_include_position_cross_axis_and_path_errors():
    gt = [Pose2D(0.0, 0.0, 0.0), Pose2D(1.0, 0.1, 0.02)]
    odom = [Pose2D(0.0, 0.0, 0.0), Pose2D(1.1, 0.0, 0.03)]
    result = calculate_run_metrics(profile(), gt, odom, 10.0)
    assert result['position_error_x'] == pytest.approx(0.1)
    assert result['position_error_y'] == pytest.approx(-0.1)
    assert result['position_error_norm'] == pytest.approx(math.sqrt(0.02))
    assert result['cross_axis_drift'] == pytest.approx(0.1)
    assert result['yaw_error'] == pytest.approx(0.01)
    assert 'path_length_error' in result


def test_filtered_metrics_improvements_and_settling_overshoot():
    gt = [Pose2D(0.0, 0.0, 0.0), Pose2D(1.0, 0.1, 0.02)]
    raw = [Pose2D(0.0, 0.0, 0.0), Pose2D(1.2, 0.0, 0.08)]
    filtered = [Pose2D(0.0, 0.0, 0.0), Pose2D(1.05, 0.08, 0.03)]
    result = calculate_run_metrics(
        profile(),
        gt,
        raw,
        10.0,
        filtered_poses=filtered,
        motion_end_ground_truth=Pose2D(0.98, 0.09, 0.01),
        motion_end_raw_odometry=Pose2D(1.18, 0.0, 0.07),
        motion_end_filtered_odometry=Pose2D(1.04, 0.07, 0.02),
    )
    assert result['position_error_improvement'] > 0.0
    assert result['absolute_yaw_error_improvement'] > 0.0
    assert result[
        'ground_truth_motion_end_to_settled_translation'
    ] > 0.0
    assert result['raw_motion_end_to_settled_yaw'] == pytest.approx(0.01)


def test_diagonal_cross_track_error():
    along, cross = diagonal_errors(1.0, 1.0, Pose2D(1.0, 0.0, 0.0))
    assert along == pytest.approx(1.0 / math.sqrt(2.0))
    assert cross == pytest.approx(-1.0 / math.sqrt(2.0))


def test_square_closure_error():
    position, yaw = square_closure(
        Pose2D(0.0, 0.0, 0.0),
        Pose2D(0.03, -0.04, 0.02),
    )
    assert position == pytest.approx(0.05)
    assert yaw == pytest.approx(0.02)


def test_termination_reached_and_not_reached():
    segment = profile().segments[0]
    assert not termination_reached(
        segment, Pose2D(0.0, 0.0, 0.0), Pose2D(0.99, 0.0, 0.0)
    )
    assert termination_reached(
        segment, Pose2D(0.0, 0.0, 0.0), Pose2D(1.0, 0.0, 0.0)
    )


def test_timeout_handling_is_bounded():
    assert not timeout_expired(10.0, 14.9, 5.0)
    assert timeout_expired(10.0, 15.0, 5.0)
    with pytest.raises(ValueError):
        timeout_expired(10.0, 10.0, 0.0)


def test_pose_stream_timestamp_and_staleness_validation():
    validate_monotonic_timestamp(2.0, 1.0)
    with pytest.raises(ValueError, match='nonmonotonic'):
        validate_monotonic_timestamp(0.5, 1.0)
    with pytest.raises(ValueError, match='positive'):
        validate_monotonic_timestamp(0.0)
    assert data_is_stale(1.0, 3.1, 2.0)
    assert not data_is_stale(1.0, 3.0, 2.0)


def test_statistics_and_report_serialization(tmp_path):
    values = metric_statistics([1.0, 2.0, 3.0])
    assert values['mean'] == pytest.approx(2.0)
    assert values['median'] == pytest.approx(2.0)
    assert values['minimum'] == pytest.approx(1.0)
    assert values['maximum'] == pytest.approx(3.0)
    assert values['standard_deviation'] == pytest.approx(1.0)
    run = {
        'profile': 'synthetic',
        'repetition': 1,
        'completion_status': 'completed',
        'position_error_norm': 0.1,
        'yaw_error': 0.01,
        'cross_axis_drift': 0.02,
        'duration': 1.0,
        'path_length_error': 0.03,
    }
    report = build_report({'world_name': 'empty'}, {'sample_rate': 20}, [run])
    write_report(tmp_path, report)
    parsed = json.loads((tmp_path / 'summary.json').read_text())
    assert parsed['world_name'] == 'empty'
    assert (tmp_path / 'summary.csv').is_file()
    assert (tmp_path / 'runs.csv').is_file()


def test_failed_runs_are_not_hidden():
    report = build_report(
        {},
        {},
        [{
            'profile': 'synthetic',
            'repetition': 1,
            'completion_status': 'timeout',
            'timeout_status': True,
            'reason': 'segment timed out',
        }],
    )
    summary = report['per_test_statistics']['synthetic']
    assert summary['run_count'] == 1
    assert summary['completed_count'] == 0
    assert summary['failed_count'] == 1
