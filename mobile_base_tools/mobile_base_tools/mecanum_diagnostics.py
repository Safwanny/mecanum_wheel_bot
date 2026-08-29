"""Pure measurement and classification helpers for mecanum diagnostics."""

import math


WHEELS = ('front_left', 'front_right', 'rear_right', 'rear_left')


def expected_wheel_velocities(vx, vy, wz, radius, center_projection_sum):
    """Return controller-convention wheel angular rates in rad/s.

    The order and signs match the Jazzy mecanum_drive_controller convention:
    front-left, front-right, rear-right, rear-left.
    """
    values = (vx, vy, wz, radius, center_projection_sum)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('mecanum inputs must be finite')
    if radius <= 0.0 or center_projection_sum <= 0.0:
        raise ValueError('radius and center projection sum must be positive')
    rotation = center_projection_sum * wz
    return {
        'front_left': (vx - vy - rotation) / radius,
        'front_right': (vx + vy + rotation) / radius,
        'rear_right': (vx - vy + rotation) / radius,
        'rear_left': (vx + vy - rotation) / radius,
    }


def ideal_planar_pose(vx, vy, wz, elapsed):
    """Integrate a constant body twist from the origin."""
    if elapsed < 0.0 or not all(
            math.isfinite(value) for value in (vx, vy, wz, elapsed)):
        raise ValueError('twist and nonnegative elapsed time must be finite')
    if abs(wz) <= 1e-12:
        return {'x': vx * elapsed, 'y': vy * elapsed, 'yaw': 0.0}
    angle = wz * elapsed
    return {
        'x': (vx * math.sin(angle) + vy * (math.cos(angle) - 1.0)) / wz,
        'y': (vx * (1.0 - math.cos(angle)) + vy * math.sin(angle)) / wz,
        'yaw': angle,
    }


def _mean(values):
    return sum(values) / len(values) if values else None


def _rmse(values):
    return math.sqrt(sum(value * value for value in values) / len(values)) \
        if values else None


def wheel_tracking_metrics(samples, tolerance_fraction=0.05,
                           absolute_tolerance=0.05):
    """Calculate per-wheel tracking metrics from trajectory samples."""
    result = {}
    for wheel in WHEELS:
        expected_key = f'expected_{wheel}_wheel_velocity'
        actual_key = f'actual_{wheel}_wheel_velocity'
        usable = [
            sample for sample in samples
            if isinstance(sample.get(expected_key), (int, float))
            and isinstance(sample.get(actual_key), (int, float))
        ]
        errors = [
            sample[actual_key] - sample[expected_key] for sample in usable]
        expected_magnitudes = [abs(sample[expected_key]) for sample in usable]
        peak_expected = max(expected_magnitudes, default=0.0)
        threshold = max(absolute_tolerance, tolerance_fraction * peak_expected)
        steady = usable[len(usable) // 2:]
        steady_errors = [
            sample[actual_key] - sample[expected_key] for sample in steady]
        rise_time = None
        for sample in usable:
            if peak_expected > absolute_tolerance and (
                abs(sample[actual_key]) >= 0.9 * peak_expected
                and sample[actual_key] * sample[expected_key] >= 0.0
            ):
                rise_time = sample.get('simulation_elapsed')
                break
        peak_actual = max(
            (abs(sample[actual_key]) for sample in usable), default=0.0)
        overshoot = (
            100.0 * max(0.0, peak_actual - peak_expected) / peak_expected
            if peak_expected > absolute_tolerance else 0.0
        )
        settling_time = None
        for index, sample in enumerate(usable):
            remaining = usable[index:]
            if remaining and all(
                abs(item[actual_key] - item[expected_key]) <= threshold
                for item in remaining
            ):
                settling_time = sample.get('simulation_elapsed')
                break
        result[wheel] = {
            'sample_count': len(usable),
            'velocity_rmse_rad_s': _rmse(errors),
            'mean_steady_state_error_rad_s': _mean(steady_errors),
            'peak_absolute_error_rad_s': max(
                (abs(value) for value in errors), default=None),
            'normalized_rmse': (
                _rmse(errors) / peak_expected
                if peak_expected > 0.0 else None),
            'normalized_steady_state_rmse': (
                _rmse(steady_errors) / peak_expected
                if peak_expected > 0.0 else None),
            'rise_time_seconds': rise_time,
            'overshoot_percent': overshoot,
            'settling_time_seconds': settling_time,
            'peak_expected_rad_s': peak_expected,
            'peak_actual_rad_s': peak_actual,
        }
    return result


def chassis_motion_metrics(samples, kind):
    """Compare commanded ideal motion, Gazebo motion, and raw odometry."""
    usable = [sample for sample in samples if (
        abs(sample.get('commanded_linear_x', 0.0))
        + abs(sample.get('commanded_linear_y', 0.0))
        + abs(sample.get('commanded_angular_z', 0.0)) > 0.0
    ) and all(
        isinstance(sample.get(key), (int, float)) for key in (
            'simulation_elapsed', 'commanded_linear_x', 'commanded_linear_y',
            'commanded_angular_z', 'ground_truth_x', 'ground_truth_y',
            'ground_truth_yaw', 'odometry_x', 'odometry_y', 'odometry_yaw'))]
    if not usable:
        return {'sample_count': 0}
    first = usable[0]
    gt_origin = (first['ground_truth_x'], first['ground_truth_y'],
                 first['ground_truth_yaw'])
    odom_origin = (first['odometry_x'], first['odometry_y'],
                   first['odometry_yaw'])
    path_errors = []
    odometry_errors = []
    final = None
    for sample in usable:
        ideal = ideal_planar_pose(
            sample['commanded_linear_x'], sample['commanded_linear_y'],
            sample['commanded_angular_z'], sample['simulation_elapsed'])
        gt_dx = sample['ground_truth_x'] - gt_origin[0]
        gt_dy = sample['ground_truth_y'] - gt_origin[1]
        gt_dyaw = math.atan2(
            math.sin(sample['ground_truth_yaw'] - gt_origin[2]),
            math.cos(sample['ground_truth_yaw'] - gt_origin[2]))
        odom_dx = sample['odometry_x'] - odom_origin[0]
        odom_dy = sample['odometry_y'] - odom_origin[1]
        odom_dyaw = math.atan2(
            math.sin(sample['odometry_yaw'] - odom_origin[2]),
            math.cos(sample['odometry_yaw'] - odom_origin[2]))
        path_errors.append(math.hypot(gt_dx - ideal['x'], gt_dy - ideal['y']))
        odometry_errors.append(math.hypot(odom_dx - gt_dx, odom_dy - gt_dy))
        final = (sample, ideal, gt_dx, gt_dy, gt_dyaw,
                 odom_dx, odom_dy, odom_dyaw)
    sample, ideal, gt_dx, gt_dy, gt_dyaw, odom_dx, odom_dy, odom_dyaw = final
    vx = sample['commanded_linear_x']
    vy = sample['commanded_linear_y']
    speed = math.hypot(vx, vy)
    if speed > 0.0:
        ux, uy = vx / speed, vy / speed
        along = gt_dx * ux + gt_dy * uy
        cross = -gt_dx * uy + gt_dy * ux
        ideal_along = ideal['x'] * ux + ideal['y'] * uy
    else:
        along = cross = ideal_along = 0.0
    distance = math.hypot(gt_dx, gt_dy)
    yaw_amount = abs(gt_dyaw)
    result = {
        'sample_count': len(usable),
        'duration_seconds': sample['simulation_elapsed'],
        'ideal_endpoint': ideal,
        'ground_truth_delta': {'x': gt_dx, 'y': gt_dy, 'yaw': gt_dyaw},
        'raw_odometry_delta': {
            'x': odom_dx, 'y': odom_dy, 'yaw': odom_dyaw},
        'path_rmse_metres': _rmse(path_errors),
        'endpoint_error_metres': math.hypot(
            gt_dx - ideal['x'], gt_dy - ideal['y']),
        'odometry_endpoint_error_metres': math.hypot(
            odom_dx - gt_dx, odom_dy - gt_dy),
        'yaw_error_radians': gt_dyaw - ideal['yaw'],
        'odometry_yaw_error_radians': odom_dyaw - gt_dyaw,
    }
    if kind != 'rotation':
        result.update({
            'primary_axis_distance_error_metres': along - ideal_along,
            'cross_axis_drift_metres': cross,
            'yaw_drift_radians': gt_dyaw,
            'yaw_drift_per_metre': (
                gt_dyaw / distance if distance > 0.0 else None),
        })
    else:
        translation_drift = math.hypot(gt_dx, gt_dy)
        result.update({
            'translation_drift_metres': translation_drift,
            'translation_drift_per_radian': (
                translation_drift / yaw_amount if yaw_amount > 0.0 else None),
        })
    return result


def classify_root_cause(wheel_metrics, chassis_metrics):
    """Classify evidence using explicit, conservative diagnostic thresholds."""
    normalized = [
        metrics['normalized_rmse'] for metrics in wheel_metrics.values()
        if metrics.get('normalized_rmse') is not None]
    wheel_error = max(normalized, default=0.0)
    steady_normalized = [
        metrics['normalized_steady_state_rmse']
        for metrics in wheel_metrics.values()
        if metrics.get('normalized_steady_state_rmse') is not None]
    steady_wheel_error = max(steady_normalized, default=0.0)
    ideal_distance = math.hypot(
        chassis_metrics.get('ideal_endpoint', {}).get('x', 0.0),
        chassis_metrics.get('ideal_endpoint', {}).get('y', 0.0))
    ideal_motion = max(
        ideal_distance,
        abs(chassis_metrics.get('ideal_endpoint', {}).get('yaw', 0.0)),
        1e-9,
    )
    chassis_error = max(
        chassis_metrics.get('endpoint_error_metres', 0.0),
        abs(chassis_metrics.get('yaw_error_radians', 0.0)),
    ) / ideal_motion
    odometry_error = max(
        chassis_metrics.get('odometry_endpoint_error_metres', 0.0),
        abs(chassis_metrics.get('odometry_yaw_error_radians', 0.0)),
    ) / ideal_motion
    if steady_wheel_error > 0.10:
        case = 'A'
        diagnosis = 'wheel command/tracking mismatch'
    elif wheel_error > 0.10:
        case = 'D'
        diagnosis = 'wheel error is concentrated in command transients'
    elif chassis_error > 0.10:
        case = 'B'
        diagnosis = (
            'wheel tracking is good but chassis/contact motion diverges')
    elif odometry_error > 0.10:
        case = 'C'
        diagnosis = 'physical motion is good but odometry diverges'
    else:
        case = 'within_threshold'
        diagnosis = 'no dominant error exceeds the 10 percent threshold'
    return {
        'case': case,
        'diagnosis': diagnosis,
        'maximum_wheel_normalized_rmse': wheel_error,
        'maximum_wheel_steady_state_normalized_rmse': steady_wheel_error,
        'normalized_chassis_error': chassis_error,
        'normalized_odometry_error': odometry_error,
        'threshold': 0.10,
    }


def multi_segment_loop_closure_metrics(run_metrics, sample_count):
    """Summarize loop closure without assuming one constant body command."""
    required = (
        'ground_truth_delta_x', 'ground_truth_delta_y',
        'ground_truth_delta_yaw', 'odometry_delta_x', 'odometry_delta_y',
        'odometry_delta_yaw', 'position_error_norm', 'yaw_error',
        'distance_travelled_ground_truth', 'distance_travelled_odometry',
    )
    if not all(isinstance(run_metrics.get(key), (int, float))
               for key in required):
        raise ValueError('loop-closure run metrics are incomplete')
    return {
        'sample_count': sample_count,
        'measurement_scope': 'multi_segment_loop_closure',
        'ground_truth_loop_closure_translation_metres': math.hypot(
            run_metrics['ground_truth_delta_x'],
            run_metrics['ground_truth_delta_y']),
        'raw_odometry_loop_closure_translation_metres': math.hypot(
            run_metrics['odometry_delta_x'],
            run_metrics['odometry_delta_y']),
        'odometry_endpoint_error_metres': run_metrics[
            'position_error_norm'],
        'ground_truth_loop_closure_yaw_radians': run_metrics[
            'ground_truth_delta_yaw'],
        'raw_odometry_loop_closure_yaw_radians': run_metrics[
            'odometry_delta_yaw'],
        'odometry_yaw_error_radians': run_metrics['yaw_error'],
        'ground_truth_path_length_metres': run_metrics[
            'distance_travelled_ground_truth'],
        'raw_odometry_path_length_metres': run_metrics[
            'distance_travelled_odometry'],
    }


def classify_multi_segment(wheel_metrics):
    """Return wheel evidence while declining an invalid single-command case."""
    wheel_only = classify_root_cause(wheel_metrics, {})
    return {
        **wheel_only,
        'case': 'not_applicable',
        'diagnosis': (
            'single-command Case A/B/C/D classification is not applicable '
            'to multi-segment profiles; use loop-closure metrics'),
        'normalized_chassis_error': None,
        'normalized_odometry_error': None,
    }
