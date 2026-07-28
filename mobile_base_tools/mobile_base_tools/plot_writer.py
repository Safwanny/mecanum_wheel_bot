"""Headless Matplotlib plots for odometry evaluation reports."""

from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402


def _save(path, title, xlabel, ylabel):
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.legend()
    plt.tight_layout()
    plt.savefig(path)
    plt.close()


def plot_trajectory(path, profile_name, samples):
    """Plot Gazebo and wheel-odometry XY trajectories."""
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    plt.figure()
    plt.plot(
        [sample['ground_truth_x'] for sample in samples],
        [sample['ground_truth_y'] for sample in samples],
        label='Gazebo ground truth',
    )
    plt.plot(
        [sample['odometry_x'] for sample in samples],
        [sample['odometry_y'] for sample in samples],
        label='Wheel odometry',
    )
    if samples and samples[0].get('filtered_x', '') != '':
        plt.plot(
            [sample['filtered_x'] for sample in samples],
            [sample['filtered_y'] for sample in samples],
            label='Filtered odometry',
        )
    plt.axis('equal')
    _save(
        output, f'{profile_name}: XY trajectory',
        'X position (m)', 'Y position (m)',
    )


def _bar_plot(path, values, title, ylabel):
    plt.figure()
    names = list(values)
    plt.bar(names, [values[name] for name in names], label='Mean')
    plt.xticks(rotation=35, ha='right')
    _save(path, title, 'Motion profile', ylabel)


def write_summary_plots(output_dir, report):
    """Generate all cross-profile comparison plots."""
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    statistics = report['per_test_statistics']
    plot_specs = (
        ('position_error_norm', 'position_error_comparison.png',
         'Position-error comparison', 'Position error (m)'),
        ('yaw_error', 'yaw_error_comparison.png',
         'Yaw-error comparison', 'Yaw error (rad)'),
        ('cross_axis_drift', 'cross_axis_drift_comparison.png',
         'Cross-axis drift comparison', 'Cross-axis drift (m)'),
    )
    for metric, filename, title, ylabel in plot_specs:
        values = {
            name: abs(data['metrics'][metric]['mean'])
            for name, data in statistics.items()
            if data['metrics'][metric] is not None
        }
        if values:
            _bar_plot(output / filename, values, title, ylabel)

    comparison_specs = (
        ('raw_position_error_norm', 'filtered_position_error_norm',
         'raw_vs_filtered_position_error.png',
         'Raw versus filtered final position error', 'Position error (m)'),
        ('raw_yaw_error', 'filtered_yaw_error',
         'raw_vs_filtered_yaw_error.png',
         'Raw versus filtered yaw error', 'Absolute yaw error (rad)'),
        ('raw_path_length_error', 'filtered_path_length_error',
         'raw_vs_filtered_path_length_error.png',
         'Raw versus filtered path-length error',
         'Absolute path-length error (m)'),
    )
    completed_filtered = [
        run for run in report['runs']
        if run['completion_status'] == 'completed'
        and 'filtered_position_error_norm' in run
    ]
    for raw_key, filtered_key, filename, title, ylabel in comparison_specs:
        if not completed_filtered:
            continue
        names = [run['profile'] for run in completed_filtered]
        indices = range(len(names))
        plt.figure()
        plt.bar(
            [index - 0.2 for index in indices],
            [abs(run[raw_key]) for run in completed_filtered],
            width=0.4,
            label='Raw',
        )
        plt.bar(
            [index + 0.2 for index in indices],
            [abs(run[filtered_key]) for run in completed_filtered],
            width=0.4,
            label='Filtered',
        )
        plt.xticks(list(indices), names, rotation=35, ha='right')
        _save(output / filename, title, 'Motion profile', ylabel)

    plt.figure()
    for name, data in statistics.items():
        values = [
            run['position_error_norm'] for run in report['runs']
            if run['profile'] == name
            and run['completion_status'] == 'completed'
        ]
        if values:
            plt.scatter([name] * len(values), values, label=name)
    plt.xticks(rotation=35, ha='right')
    _save(
        output / 'repeatability_spread.png',
        'Position-error repeatability spread',
        'Motion profile',
        'Position error (m)',
    )

    if completed_filtered:
        plt.figure()
        for name in sorted({run['profile'] for run in completed_filtered}):
            values = [
                run['filtered_position_error_norm']
                for run in completed_filtered if run['profile'] == name
            ]
            plt.scatter([name] * len(values), values, label=name)
        plt.xticks(rotation=35, ha='right')
        _save(
            output / 'filtered_repeatability_spread.png',
            'Filtered position-error repeatability spread',
            'Motion profile',
            'Filtered position error (m)',
        )

        plt.figure()
        names = [run['profile'] for run in completed_filtered]
        indices = range(len(names))
        plt.bar(
            [index - 0.2 for index in indices],
            [
                run['ground_truth_motion_end_to_settled_translation']
                for run in completed_filtered
            ],
            width=0.4,
            label='Gazebo ground truth',
        )
        plt.bar(
            [index + 0.2 for index in indices],
            [
                run['raw_motion_end_to_settled_translation']
                for run in completed_filtered
            ],
            width=0.4,
            label='Raw odometry',
        )
        plt.xticks(list(indices), names, rotation=35, ha='right')
        _save(
            output / 'motion_end_vs_settled_overshoot.png',
            'Motion-end versus settled-final translation',
            'Motion profile',
            'Additional translation while settling (m)',
        )

        plt.figure()
        plt.bar(
            names,
            [
                run['position_error_improvement']
                for run in completed_filtered
            ],
            label='Raw minus filtered',
        )
        plt.axhline(0.0, color='black', linewidth=0.8)
        plt.xticks(rotation=35, ha='right')
        _save(
            output / 'per_profile_improvement.png',
            'Filtered position-error improvement',
            'Motion profile',
            'Improvement (m; positive is better)',
        )

    completed = [
        run for run in report['runs']
        if run['completion_status'] == 'completed'
    ]
    if completed:
        plt.figure()
        plt.scatter(
            [
                (run['ground_truth_delta_x'] ** 2
                 + run['ground_truth_delta_y'] ** 2) ** 0.5
                for run in completed
            ],
            [
                (run['odometry_delta_x'] ** 2
                 + run['odometry_delta_y'] ** 2) ** 0.5
                for run in completed
            ],
            label='Final displacement',
        )
        limits = plt.axis()
        low = min(limits[0], limits[2])
        high = max(limits[1], limits[3])
        plt.plot([low, high], [low, high], linestyle='--', label='Ideal')
        _save(
            output / 'final_displacement_comparison.png',
            'Ground truth versus odometry final displacement',
            'Ground-truth displacement (m)',
            'Odometry displacement (m)',
        )
