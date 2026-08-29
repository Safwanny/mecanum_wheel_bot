"""Machine-readable odometry evaluation report generation."""

import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import statistics


SUMMARY_METRICS = (
    'raw_position_error_norm',
    'filtered_position_error_norm',
    'position_error_improvement',
    'raw_yaw_error',
    'filtered_yaw_error',
    'absolute_yaw_error_improvement',
    'raw_path_length_error',
    'filtered_path_length_error',
    'path_length_error_improvement',
    'position_error_norm',
    'yaw_error',
    'cross_axis_drift',
    'duration',
    'path_length_error',
)


def metric_statistics(values):
    """Return deterministic descriptive statistics."""
    finite_values = [float(value) for value in values]
    if not finite_values:
        return None
    return {
        'mean': statistics.fmean(finite_values),
        'median': statistics.median(finite_values),
        'minimum': min(finite_values),
        'maximum': max(finite_values),
        'standard_deviation': (
            statistics.stdev(finite_values)
            if len(finite_values) > 1 else 0.0
        ),
    }


def summarize_runs(runs):
    """Group successful and failed repetitions without hiding failures."""
    summary = {}
    for name in sorted({run['profile'] for run in runs}):
        profile_runs = [run for run in runs if run['profile'] == name]
        successful = [
            run for run in profile_runs
            if run['completion_status'] == 'completed'
        ]
        summary[name] = {
            'run_count': len(profile_runs),
            'completed_count': len(successful),
            'failed_count': len(profile_runs) - len(successful),
            'metrics': {
                metric: metric_statistics(
                    [run[metric] for run in successful if metric in run]
                )
                for metric in SUMMARY_METRICS
            },
        }
    return summary


def build_report(metadata, configuration, runs):
    """Build the complete JSON report dictionary."""
    summaries = summarize_runs(runs)
    ranking = sorted(
        (
            {
                'profile': name,
                'mean_position_error': data['metrics'][
                    'position_error_norm'
                ]['mean'],
            }
            for name, data in summaries.items()
            if data['metrics']['position_error_norm'] is not None
        ),
        key=lambda item: item['mean_position_error'],
        reverse=True,
    )
    classifications = {}
    for run in runs:
        root_cause = run.get('mecanum_diagnostics', {}).get('root_cause', {})
        case = root_cause.get('case')
        if case:
            classifications[case] = classifications.get(case, 0) + 1
    return {
        'timestamp': datetime.now(timezone.utc).isoformat(),
        **metadata,
        'test_configuration': configuration,
        'runs': runs,
        'per_test_statistics': summaries,
        'largest_position_errors': ranking,
        'mecanum_root_cause_summary': classifications,
    }


def _write_csv(path, rows, fieldnames):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_trajectory(path, samples):
    """Write one sampled trajectory CSV."""
    fields = (
        'timestamp', 'commanded_linear_x', 'commanded_linear_y',
        'commanded_angular_z', 'ground_truth_x', 'ground_truth_y',
        'ground_truth_yaw', 'ground_truth_stamp',
        'odometry_x', 'odometry_y', 'odometry_yaw', 'raw_odometry_stamp',
        'filtered_x', 'filtered_y', 'filtered_yaw',
        'filtered_odometry_stamp',
        'wall_elapsed', 'simulation_elapsed', 'real_time_factor',
        'ground_truth_speed', 'raw_odometry_speed',
        'ground_truth_linear_x', 'ground_truth_linear_y',
        'ground_truth_angular_z',
        'raw_odometry_linear_x', 'raw_odometry_linear_y',
        'raw_odometry_angular_z',
        'filtered_odometry_linear_x', 'filtered_odometry_linear_y',
        'filtered_odometry_angular_z', 'imu_angular_velocity_z',
        'controller_reference_linear_x', 'controller_reference_linear_y',
        'controller_reference_angular_z',
        'expected_front_left_wheel_velocity',
        'expected_front_right_wheel_velocity',
        'expected_rear_right_wheel_velocity',
        'expected_rear_left_wheel_velocity',
        'actual_front_left_wheel_velocity',
        'actual_front_right_wheel_velocity',
        'actual_rear_right_wheel_velocity',
        'actual_rear_left_wheel_velocity',
        'controller_front_left_wheel_velocity',
        'controller_front_right_wheel_velocity',
        'controller_rear_right_wheel_velocity',
        'controller_rear_left_wheel_velocity',
        'front_left_wheel_position', 'front_right_wheel_position',
        'rear_right_wheel_position', 'rear_left_wheel_position',
    )
    _write_csv(Path(path), samples, fields)


def write_run_result(path, result):
    """Write a structured result for one completed or failed attempt."""
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )


def write_report(output_dir, report):
    """Write summary JSON/CSV and per-run CSV files."""
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    (output / 'summary.json').write_text(
        json.dumps(report, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    metadata = {
        key: value for key, value in report.items()
        if key not in (
            'runs', 'per_test_statistics', 'largest_position_errors',
            'test_configuration',
        )
    }
    (output / 'metadata.json').write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    run_fields = sorted({key for run in report['runs'] for key in run})
    _write_csv(output / 'runs.csv', report['runs'], run_fields)
    summary_rows = []
    for profile, data in report['per_test_statistics'].items():
        row = {
            'profile': profile,
            'run_count': data['run_count'],
            'completed_count': data['completed_count'],
            'failed_count': data['failed_count'],
        }
        for metric, values in data['metrics'].items():
            if values:
                for statistic_name, value in values.items():
                    row[f'{metric}_{statistic_name}'] = value
        summary_rows.append(row)
    summary_fields = sorted({key for row in summary_rows for key in row})
    _write_csv(output / 'summary.csv', summary_rows, summary_fields)

    lines = [
        '# Phase 1 odometry evaluation',
        '',
        f"- World: `{report.get('world_name', 'unknown')}`",
        f"- Robot entity: `{report.get('robot_entity_name', 'unknown')}`",
        f"- Ground truth: {report.get('ground_truth_source', 'unknown')}",
        f"- Raw odometry: `{report.get('odometry_topic', 'unknown')}`",
        (
            '- Filtered odometry: `'
            + report.get('filtered_odometry_topic', 'not recorded') + '`'
        ),
        f"- IMU: `{report.get('imu_topic', 'unknown')}`",
        '',
        '## Profiles',
        '',
        '| Profile | Completed | Failed | Raw position mean (m) | '
        'Filtered position mean (m) |',
        '|---|---:|---:|---:|---:|',
    ]
    for name, data in report['per_test_statistics'].items():
        metrics = data['metrics']
        raw = metrics.get('raw_position_error_norm')
        filtered = metrics.get('filtered_position_error_norm')
        raw_mean = '{:.6f}'.format(raw['mean']) if raw else 'n/a'
        filtered_mean = (
            '{:.6f}'.format(filtered['mean']) if filtered else 'n/a'
        )
        completed_count = data['completed_count']
        failed_count = data['failed_count']
        lines.append(
            f'| {name} | {completed_count} | '
            f'{failed_count} | {raw_mean} | '
            f'{filtered_mean} |'
        )
    lines.extend([
        '',
        'Positive improvement values mean the filtered estimate is better. '
        'The EKF does not alter physical Gazebo motion or correct wheel slip.',
        '',
    ])
    (output / 'phase1_report.md').write_text(
        '\n'.join(lines), encoding='utf-8')
