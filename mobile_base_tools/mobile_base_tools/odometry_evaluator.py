"""Pure calculations for raw wheel-odometry characterization."""

import math
import random

from mobile_base_tools.pose_math import (
    cross_axis_drift,
    diagonal_errors,
    normalize_yaw,
    path_length,
    position_error,
    relative_pose,
    square_closure,
)


def timeout_expired(start_time, current_time, timeout):
    """Return whether a bounded operation has reached its timeout."""
    if timeout <= 0.0:
        raise ValueError('timeout must be positive')
    return current_time - start_time >= timeout


def timeout_reason(
        simulation_start, simulation_now, simulation_timeout,
        wall_start, wall_now, wall_watchdog,
        last_clock_advance_wall, clock_stall_timeout):
    """Classify motion, watchdog, and stopped-clock timeouts."""
    if simulation_now < simulation_start:
        return 'nonmonotonic simulation time'
    if timeout_expired(
            last_clock_advance_wall, wall_now, clock_stall_timeout):
        return 'clock stalled'
    if timeout_expired(simulation_start, simulation_now, simulation_timeout):
        return 'simulation timeout'
    if timeout_expired(wall_start, wall_now, wall_watchdog):
        return 'wall watchdog expired'
    return None


def command_progress(command, initial_pose, current_pose, target):
    """Return Euclidean, along-track, and cross-track motion progress."""
    delta = relative_pose(initial_pose, current_pose)
    linear_norm = math.hypot(command.linear_x, command.linear_y)
    if linear_norm > 0.0:
        unit_x = command.linear_x / linear_norm
        unit_y = command.linear_y / linear_norm
        along = delta.x * unit_x + delta.y * unit_y
        cross = -delta.x * unit_y + delta.y * unit_x
        euclidean = math.hypot(delta.x, delta.y)
        achieved = euclidean
    else:
        direction = 1.0 if command.angular_z >= 0.0 else -1.0
        along = direction * delta.yaw
        cross = math.hypot(delta.x, delta.y)
        euclidean = cross
        achieved = along
    return {
        'along_track_displacement': along,
        'cross_track_displacement': cross,
        'euclidean_displacement': euclidean,
        'percentage_of_target_reached': (
            100.0 * achieved / target if target > 0.0 else 0.0
        ),
    }


def ordered_profiles(profiles, order, seed):
    """Return configured or deterministically shuffled profile values."""
    selected = list(profiles)
    if order == 'configured':
        return selected
    if order == 'randomized':
        random.Random(seed).shuffle(selected)
        return selected
    raise ValueError('order must be configured or randomized')


def command_stamp_status(previous_stamp, current_stamp):
    """Classify a command stamp before publication."""
    if current_stamp <= 0.0:
        return 'zero'
    if previous_stamp is None:
        return 'fresh'
    if current_stamp < previous_stamp:
        return 'nonmonotonic'
    if current_stamp == previous_stamp:
        return 'duplicate'
    return 'fresh'


def update_settle_window(
        settled_since, current_simulation_time, stopped, required_duration):
    """Advance or reset a continuous-settling interval."""
    if required_duration <= 0.0:
        raise ValueError('required_duration must be positive')
    if not stopped:
        return None, False
    start = (
        current_simulation_time
        if settled_since is None else settled_since
    )
    return start, current_simulation_time - start >= required_duration


def post_reset_streams_fresh(
        ground_truth_stamp, previous_ground_truth_stamp,
        raw_stamp, previous_raw_stamp,
        filtered_stamp=None, previous_filtered_stamp=None,
        require_filtered=False):
    """Return whether required streams have advanced beyond reset."""
    if ground_truth_stamp <= previous_ground_truth_stamp:
        return False
    if raw_stamp <= previous_raw_stamp:
        return False
    if require_filtered:
        return (
            filtered_stamp is not None
            and previous_filtered_stamp is not None
            and filtered_stamp > previous_filtered_stamp
        )
    return True


def termination_reached(segment, initial_pose, current_pose):
    """Evaluate a segment's ground-truth termination condition."""
    delta = relative_pose(initial_pose, current_pose)
    if segment.termination_type == 'ground_truth_translation':
        return math.hypot(delta.x, delta.y) >= segment.target
    if segment.termination_type == 'ground_truth_rotation':
        direction = 1.0 if segment.command.angular_z > 0.0 else -1.0
        return direction * delta.yaw >= segment.target
    raise ValueError(f'unsupported termination: {segment.termination_type}')


def calculate_run_metrics(
        profile, ground_truth_poses, odometry_poses, duration,
        completion_status='completed', reason='', filtered_poses=None,
        motion_end_ground_truth=None, motion_end_raw_odometry=None,
        motion_end_filtered_odometry=None):
    """Calculate required metrics from aligned trajectory samples."""
    if len(ground_truth_poses) < 2 or len(odometry_poses) < 2:
        raise ValueError('at least two poses from each source are required')
    gt_delta = relative_pose(ground_truth_poses[0], ground_truth_poses[-1])
    odom_delta = relative_pose(odometry_poses[0], odometry_poses[-1])
    error_x, error_y, error_norm = position_error(gt_delta, odom_delta)
    result = {
        'ground_truth_delta_x': gt_delta.x,
        'ground_truth_delta_y': gt_delta.y,
        'ground_truth_delta_yaw': gt_delta.yaw,
        'raw_odometry_delta_x': odom_delta.x,
        'raw_odometry_delta_y': odom_delta.y,
        'raw_odometry_delta_yaw': odom_delta.yaw,
        'odometry_delta_x': odom_delta.x,
        'odometry_delta_y': odom_delta.y,
        'odometry_delta_yaw': odom_delta.yaw,
        'raw_position_error_x': error_x,
        'raw_position_error_y': error_y,
        'raw_position_error_norm': error_norm,
        'position_error_x': error_x,
        'position_error_y': error_y,
        'position_error_norm': error_norm,
        'raw_yaw_error': normalize_yaw(odom_delta.yaw - gt_delta.yaw),
        'yaw_error': normalize_yaw(odom_delta.yaw - gt_delta.yaw),
        'raw_cross_axis_drift': cross_axis_drift(profile.kind, gt_delta),
        'cross_axis_drift': cross_axis_drift(profile.kind, gt_delta),
        'distance_travelled_ground_truth': path_length(ground_truth_poses),
        'distance_travelled_odometry': path_length(odometry_poses),
        'duration': duration,
        'timeout_status': completion_status == 'timeout',
        'completion_status': completion_status,
        'reason': reason,
    }
    result['path_length_error'] = (
        result['distance_travelled_odometry']
        - result['distance_travelled_ground_truth']
    )
    result['raw_path_length_error'] = result['path_length_error']
    if profile.kind == 'diagonal':
        command = profile.segments[0].command
        along, cross = diagonal_errors(
            command.linear_x, command.linear_y, gt_delta
        )
        result['along_track_displacement'] = along
        result['along_track_error'] = along - profile.segments[0].target
        result['cross_track_error'] = cross
        result['cross_axis_drift'] = abs(cross)
        result['raw_cross_axis_drift'] = result['cross_axis_drift']
    if profile.kind == 'rotation':
        result['unintended_translation_x'] = gt_delta.x
        result['unintended_translation_y'] = gt_delta.y
        result['translation_norm_during_rotation'] = math.hypot(
            gt_delta.x, gt_delta.y
        )
        result['cross_axis_drift'] = result['translation_norm_during_rotation']
        result['raw_cross_axis_drift'] = result['cross_axis_drift']
    if profile.kind == 'square':
        closure, yaw_closure = square_closure(
            ground_truth_poses[0], ground_truth_poses[-1]
        )
        result['final_position_closure_error'] = closure
        result['final_yaw_closure_error'] = yaw_closure
        result['cross_axis_drift'] = closure
        result['raw_cross_axis_drift'] = closure

    if filtered_poses is not None:
        if len(filtered_poses) < 2:
            raise ValueError('at least two filtered poses are required')
        filtered_delta = relative_pose(
            filtered_poses[0], filtered_poses[-1])
        filtered_error = position_error(gt_delta, filtered_delta)
        filtered_path_error = (
            path_length(filtered_poses) - path_length(ground_truth_poses)
        )
        filtered_yaw_error = normalize_yaw(
            filtered_delta.yaw - gt_delta.yaw)
        result.update({
            'filtered_delta_x': filtered_delta.x,
            'filtered_delta_y': filtered_delta.y,
            'filtered_delta_yaw': filtered_delta.yaw,
            'filtered_position_error_x': filtered_error[0],
            'filtered_position_error_y': filtered_error[1],
            'filtered_position_error_norm': filtered_error[2],
            'filtered_yaw_error': filtered_yaw_error,
            'filtered_cross_axis_drift': cross_axis_drift(
                profile.kind, filtered_delta),
            'distance_travelled_filtered': path_length(filtered_poses),
            'filtered_path_length_error': filtered_path_error,
            'position_error_improvement': (
                error_norm - filtered_error[2]),
            'absolute_yaw_error_improvement': (
                abs(result['raw_yaw_error']) - abs(filtered_yaw_error)),
            'path_length_error_improvement': (
                abs(result['raw_path_length_error'])
                - abs(filtered_path_error)),
        })
        position_improvement = result['position_error_improvement']
        if position_improvement > 1e-3:
            result['filtered_position_assessment'] = 'better'
        elif position_improvement < -1e-3:
            result['filtered_position_assessment'] = 'worse'
        else:
            result['filtered_position_assessment'] = 'similar'
        if profile.kind == 'diagonal':
            command = profile.segments[0].command
            filtered_along, filtered_cross = diagonal_errors(
                command.linear_x, command.linear_y, filtered_delta)
            result.update({
                'filtered_along_track_displacement': filtered_along,
                'filtered_along_track_error': (
                    filtered_along - profile.segments[0].target),
                'filtered_cross_track_error': filtered_cross,
                'filtered_cross_axis_drift': abs(filtered_cross),
            })
        if profile.kind == 'rotation':
            result.update({
                'filtered_unintended_translation_x': filtered_delta.x,
                'filtered_unintended_translation_y': filtered_delta.y,
                'filtered_translation_norm_during_rotation': math.hypot(
                    filtered_delta.x, filtered_delta.y),
            })
            result['filtered_cross_axis_drift'] = result[
                'filtered_translation_norm_during_rotation']
        if profile.kind == 'square':
            result.update({
                'raw_odometric_closure_error': math.hypot(
                    odom_delta.x, odom_delta.y),
                'raw_odometric_yaw_closure_error': abs(odom_delta.yaw),
                'filtered_odometric_closure_error': math.hypot(
                    filtered_delta.x, filtered_delta.y),
                'filtered_odometric_yaw_closure_error': abs(
                    filtered_delta.yaw),
            })

    if motion_end_ground_truth is not None:
        settled_gt = ground_truth_poses[-1]
        gt_settle = relative_pose(motion_end_ground_truth, settled_gt)
        result.update({
            'ground_truth_motion_end_to_settled_translation': math.hypot(
                gt_settle.x, gt_settle.y),
            'ground_truth_motion_end_to_settled_yaw': gt_settle.yaw,
            'ground_truth_additional_cross_axis_drift_while_settling': (
                abs(gt_settle.y)
                if profile.kind == 'longitudinal' else abs(gt_settle.x)),
        })
    if motion_end_raw_odometry is not None:
        raw_settle = relative_pose(
            motion_end_raw_odometry, odometry_poses[-1])
        result.update({
            'raw_motion_end_to_settled_translation': math.hypot(
                raw_settle.x, raw_settle.y),
            'raw_motion_end_to_settled_yaw': raw_settle.yaw,
            'raw_additional_cross_axis_drift_while_settling': (
                abs(raw_settle.y)
                if profile.kind == 'longitudinal' else abs(raw_settle.x)),
        })
    if motion_end_filtered_odometry is not None and filtered_poses is not None:
        filtered_settle = relative_pose(
            motion_end_filtered_odometry, filtered_poses[-1])
        result.update({
            'filtered_motion_end_to_settled_translation': math.hypot(
                filtered_settle.x, filtered_settle.y),
            'filtered_motion_end_to_settled_yaw': filtered_settle.yaw,
            'filtered_additional_cross_axis_drift_while_settling': (
                abs(filtered_settle.y)
                if profile.kind == 'longitudinal'
                else abs(filtered_settle.x)),
        })
    return result
