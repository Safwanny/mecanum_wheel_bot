import json
import math
import subprocess

from mobile_base_tools.evaluation_campaign import (
    completed_repetitions,
    launch_command,
    main as campaign_main,
    partial_counts,
    write_summary,
)
from mobile_base_tools.motion_profiles import Command, MotionProfile, Segment
from mobile_base_tools.odometry_evaluator import (
    calculate_run_metrics,
    command_progress,
    command_stamp_status,
    ordered_profiles,
    post_reset_streams_fresh,
    termination_reached,
    timeout_expired,
    timeout_reason,
    update_settle_window,
)
from mobile_base_tools.odometry_test_runner import (
    EvaluationError,
    OdometryTestRunner,
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


def test_simulation_timeout_is_independent_of_slow_wall_execution():
    assert timeout_reason(10.0, 14.0, 5.0, 20.0, 40.0, 30.0, 39.0, 5.0) is None
    assert timeout_reason(
        10.0, 15.0, 5.0, 20.0, 41.0, 30.0, 40.9, 5.0
    ) == 'simulation timeout'


def test_wall_watchdog_and_stalled_clock_are_distinct():
    assert timeout_reason(
        10.0, 10.5, 5.0, 20.0, 26.0, 30.0, 20.0, 5.0
    ) == 'clock stalled'
    assert timeout_reason(
        10.0, 11.0, 5.0, 20.0, 51.0, 30.0, 50.9, 5.0
    ) == 'wall watchdog expired'
    assert timeout_reason(
        10.0, 9.0, 5.0, 20.0, 21.0, 30.0, 21.0, 5.0
    ) == 'nonmonotonic simulation time'


def test_diagonal_progress_reports_projection_and_cross_track():
    progress = command_progress(
        Command(1.0, 1.0, 0.0),
        Pose2D(0.0, 0.0, 0.0),
        Pose2D(1.0, 0.0, 0.0),
        1.0,
    )
    assert progress['along_track_displacement'] == pytest.approx(
        1.0 / math.sqrt(2.0))
    assert progress['cross_track_displacement'] == pytest.approx(
        -1.0 / math.sqrt(2.0))
    assert progress['euclidean_displacement'] == pytest.approx(1.0)


def test_profile_randomization_is_deterministic():
    configured = ['forward', 'left', 'right', 'backward']
    assert ordered_profiles(configured, 'configured', 17) == configured
    first = ordered_profiles(configured, 'randomized', 17)
    second = ordered_profiles(configured, 'randomized', 17)
    assert first == second
    assert first != configured


def test_fresh_campaign_uses_independent_mode_and_localization(tmp_path):
    raw = launch_command('forward_1m', 'raw_only', 5, tmp_path / 'raw')
    fused = launch_command(
        'forward_1m', 'raw_and_filtered', 5, tmp_path / 'fused')
    assert 'localization:=false' in raw
    assert 'localization:=true' in fused
    assert 'repetitions:=5' in raw
    assert 'repetition_offset:=0' in raw
    assert raw != fused


def test_campaign_summary_is_atomically_replaced(tmp_path):
    write_summary(tmp_path, [{'profile': 'forward_1m'}], 'running')
    first = json.loads((tmp_path / 'campaign_summary.json').read_text())
    assert first['schema_version'] == 3
    assert first['status'] == 'running'
    assert first['runs'] == [{'profile': 'forward_1m'}]
    assert not (tmp_path / '.campaign_summary.json.tmp').exists()

    write_summary(tmp_path, [], 'interrupted')
    second = json.loads((tmp_path / 'campaign_summary.json').read_text())
    assert second['status'] == 'interrupted'
    assert second['runs'] == []


def test_campaign_resume_requires_manifest(tmp_path, monkeypatch):
    (tmp_path / 'unrelated.txt').write_text('do not overwrite')
    monkeypatch.setattr(
        'sys.argv',
        ['evaluation_campaign', '--output-dir', str(tmp_path), '--resume'],
    )
    with pytest.raises(SystemExit) as error:
        campaign_main()
    assert error.value.code == 2


def test_campaign_partial_counts_survive_missing_summary(tmp_path):
    run_results = tmp_path / 'run_results'
    run_results.mkdir()
    (run_results / 'run_01.json').write_text(json.dumps({
        'completion_status': 'completed'}))
    (run_results / 'run_02.json').write_text(json.dumps({
        'completion_status': 'interrupted'}))
    (run_results / 'truncated.json').write_text('{')
    assert partial_counts(tmp_path) == (1, 2)


def test_campaign_resume_expands_legacy_grouped_completion():
    results = [
        {'mode': 'raw_only', 'profile': 'forward_1m',
         'success': True, 'completed': 5},
        {'mode': 'raw_only', 'profile': 'strafe_left_1m',
         'success': False, 'completed': 1},
        {'mode': 'raw_only', 'profile': 'strafe_left_1m',
         'repetition': 2, 'success': True},
    ]
    assert completed_repetitions(
        results, 'raw_only', 'forward_1m') == {1, 2, 3, 4, 5}
    assert completed_repetitions(
        results, 'raw_only', 'strafe_left_1m') == {1, 2}


def test_command_timestamps_must_be_nonzero_and_monotonic():
    assert command_stamp_status(None, 0.0) == 'zero'
    assert command_stamp_status(None, 1.0) == 'fresh'
    assert command_stamp_status(1.0, 1.0) == 'duplicate'
    assert command_stamp_status(2.0, 1.0) == 'nonmonotonic'
    assert command_stamp_status(1.0, 2.0) == 'fresh'


def test_reset_requires_a_continuous_settled_interval():
    start, complete = update_settle_window(None, 1.0, True, 0.5)
    assert not complete
    start, complete = update_settle_window(start, 1.3, True, 0.5)
    assert not complete
    start, complete = update_settle_window(start, 1.4, False, 0.5)
    assert start is None and not complete
    start, complete = update_settle_window(start, 2.0, True, 0.5)
    start, complete = update_settle_window(start, 2.5, True, 0.5)
    assert complete


def test_reset_requires_fresh_ground_truth_raw_and_optional_filtered():
    assert not post_reset_streams_fresh(1.0, 1.0, 2.0, 1.0)
    assert not post_reset_streams_fresh(2.0, 1.0, 1.0, 1.0)
    assert post_reset_streams_fresh(2.0, 1.0, 2.0, 1.0)
    assert not post_reset_streams_fresh(
        2.0, 1.0, 2.0, 1.0, 1.0, 1.0, require_filtered=True)
    assert post_reset_streams_fresh(
        2.0, 1.0, 2.0, 1.0, 2.0, 1.0, require_filtered=True)


def test_execute_run_preserves_samples_after_segment_failure(monkeypatch):
    runner = OdometryTestRunner.__new__(OdometryTestRunner)
    start = Pose2D(0.0, 0.0, 0.0, 1.0)
    last = Pose2D(0.1, 0.0, 0.0, 2.0)
    runner.latest_ground_truth = last
    runner.latest_odometry = last
    runner.latest_filtered = None
    runner.evaluation_mode = 'raw_only'
    runner.ground_truth_speed = 0.1
    runner.raw_odometry_speed = 0.1
    runner._sim_now = lambda: 2.0
    runner.reset_repetition = lambda duration: (start, start, None)
    runner.safe_stop = lambda *args: None
    runner._sample = lambda command, *args: {'sample': len(preserved)}

    def fail_segment(segment, samples, *args):
        samples.append({'sample': 'partial'})
        raise EvaluationError('motion did not start after command')

    preserved = []
    runner._execute_segment = fail_segment
    runner._build_failure_result = lambda *args: {
        'completion_status': 'failed'}
    runner._write_run_artifacts = (
        lambda profile_value, repetition, samples, result:
        preserved.extend(samples)
    )
    result = runner.execute_run(profile(), 1)
    assert result['completion_status'] == 'failed'
    assert {'sample': 'partial'} in preserved
    assert len(preserved) == 2


def test_execute_run_persists_samples_before_reraising_interrupt():
    runner = OdometryTestRunner.__new__(OdometryTestRunner)
    start = Pose2D(0.0, 0.0, 0.0, 1.0)
    runner.latest_ground_truth = start
    runner.latest_odometry = start
    runner.latest_filtered = None
    runner.evaluation_mode = 'raw_only'
    runner.ground_truth_speed = 0.1
    runner.raw_odometry_speed = 0.1
    runner._sim_now = lambda: 2.0
    runner.reset_repetition = lambda duration: (start, start, None)
    runner.safe_stop = lambda *args: None
    runner._sample = lambda command, *args: {'sample': 'initial'}

    def interrupt_segment(segment, samples, *args):
        samples.append({'sample': 'partial'})
        raise KeyboardInterrupt

    persisted = []
    runner._execute_segment = interrupt_segment
    runner._build_failure_result = lambda *args: {
        'completion_status': 'interrupted'}
    runner._write_run_artifacts = (
        lambda profile_value, repetition, samples, result:
        persisted.extend(samples)
    )
    with pytest.raises(KeyboardInterrupt):
        runner.execute_run(profile(), 1)
    assert persisted == [
        {'sample': 'initial'}, {'sample': 'partial'}]


def test_failed_run_artifacts_are_generated_for_partial_samples(tmp_path):
    runner = OdometryTestRunner.__new__(OdometryTestRunner)
    runner.output_dir = tmp_path
    sample = {
        'timestamp': 1.0,
        'commanded_linear_x': 0.1,
        'commanded_linear_y': 0.0,
        'commanded_angular_z': 0.0,
        'ground_truth_x': 0.0,
        'ground_truth_y': 0.0,
        'ground_truth_yaw': 0.0,
        'ground_truth_stamp': 1.0,
        'odometry_x': 0.0,
        'odometry_y': 0.0,
        'odometry_yaw': 0.0,
        'raw_odometry_stamp': 1.0,
        'filtered_x': '',
        'filtered_y': '',
        'filtered_yaw': '',
        'filtered_odometry_stamp': '',
        'wall_elapsed': 0.0,
        'simulation_elapsed': 0.0,
        'real_time_factor': 0.0,
        'ground_truth_speed': 0.0,
        'raw_odometry_speed': 0.0,
    }
    result = {'completion_status': 'timeout', 'reason': 'simulation timeout'}
    runner._write_run_artifacts(profile(), 1, [sample, dict(sample)], result)
    assert (tmp_path / 'trajectories/synthetic_run_01.csv').is_file()
    assert (tmp_path / 'plots/synthetic_run_01.png').is_file()
    assert (tmp_path / 'run_results/synthetic_run_01.json').is_file()


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


def test_raw_only_report_does_not_require_filtered_metrics(tmp_path):
    run = {
        'profile': 'synthetic',
        'repetition': 1,
        'completion_status': 'completed',
        'position_error_norm': 0.2,
        'yaw_error': 0.03,
        'cross_axis_drift': 0.01,
        'duration': 2.0,
        'path_length_error': 0.04,
    }
    report = build_report(
        {'evaluation_mode': 'raw_only'}, {}, [run])
    write_report(tmp_path, report)
    parsed = json.loads((tmp_path / 'summary.json').read_text())
    metrics = parsed['per_test_statistics']['synthetic']['metrics']
    assert parsed['evaluation_mode'] == 'raw_only'
    assert metrics['filtered_position_error_norm'] is None


def test_raw_and_filtered_report_preserves_filtered_metrics():
    run = {
        'profile': 'synthetic',
        'repetition': 1,
        'completion_status': 'completed',
        'position_error_norm': 0.2,
        'yaw_error': 0.03,
        'cross_axis_drift': 0.01,
        'duration': 2.0,
        'path_length_error': 0.04,
        'raw_position_error_norm': 0.2,
        'filtered_position_error_norm': 0.1,
    }
    report = build_report(
        {'evaluation_mode': 'raw_and_filtered'}, {}, [run])
    metrics = report['per_test_statistics']['synthetic']['metrics']
    assert metrics['filtered_position_error_norm']['mean'] == 0.1


def test_repository_commit_is_resolved_from_output_path(tmp_path):
    repository = tmp_path / 'repository'
    repository.mkdir()
    subprocess.run(
        ['git', 'init', '--quiet'], cwd=repository, check=True)
    subprocess.run(
        ['git', 'config', 'user.email', 'test@example.com'],
        cwd=repository, check=True,
    )
    subprocess.run(
        ['git', 'config', 'user.name', 'Test'],
        cwd=repository, check=True,
    )
    (repository / 'tracked').write_text('evidence')
    subprocess.run(
        ['git', 'add', 'tracked'], cwd=repository, check=True)
    subprocess.run(
        ['git', 'commit', '--quiet', '-m', 'baseline'],
        cwd=repository, check=True,
    )
    runner = OdometryTestRunner.__new__(OdometryTestRunner)
    runner.output_dir = repository / 'phase1_results' / 'raw_baseline'
    expected = subprocess.run(
        ['git', 'rev-parse', 'HEAD'],
        cwd=repository,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    assert runner._repository_commit() == expected
