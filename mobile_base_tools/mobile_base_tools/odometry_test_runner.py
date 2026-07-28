#!/usr/bin/env python3
"""Run YAML-defined raw wheel-odometry evaluation profiles."""

import math
import os
from datetime import datetime, timezone
from pathlib import Path
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TwistStamped
from mobile_base_tools.motion_profiles import load_profiles
from mobile_base_tools.odometry_evaluator import (
    calculate_run_metrics,
    termination_reached,
    timeout_expired,
)
from mobile_base_tools.plot_writer import (
    plot_trajectory,
    write_summary_plots,
)
from mobile_base_tools.pose_math import (
    data_is_stale,
    Pose2D,
    position_error,
    quaternion_to_yaw,
    relative_pose,
    validate_monotonic_timestamp,
)
from mobile_base_tools.report_writer import (
    build_report,
    write_report,
    write_trajectory,
)
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose


class EvaluationError(RuntimeError):
    """Bounded evaluation failure with a reportable reason."""


class OdometryTestRunner(Node):
    """Compare controller odometry with evaluation-only Gazebo pose data."""

    def __init__(self):
        super().__init__('odometry_test_runner')
        share = get_package_share_directory('mobile_base_tools')
        defaults = {
            'config_file': str(Path(share) / 'config' / 'odometry_tests.yaml'),
            'output_dir': '/tmp/mobile_base_phase1',
            'test_profile': 'all',
            'evaluation_mode': 'raw_and_filtered',
            'repetitions': 0,
            'sample_rate': 20.0,
            'startup_timeout': 60.0,
            'data_timeout': 2.0,
            'stop_publish_duration': 0.5,
            'stop_velocity_threshold': 0.01,
            'reset_position_tolerance': 0.03,
            'reset_yaw_tolerance': 0.05,
            'reset_timeout': 5.0,
            'max_source_skew': 0.1,
            'world': 'empty',
            'robot_entity': 'mobile_base',
            'spawn_x': 0.0,
            'spawn_y': 0.0,
            'spawn_z': 0.08,
            'spawn_yaw': 0.0,
            'ground_truth_topic': (
                '/mobile_base/evaluation/ground_truth'
            ),
            'set_pose_service': '/world/empty/set_pose',
            'command_topic': '/mobile_base_controller/reference',
            'odometry_topic': '/mobile_base_controller/odometry',
            'filtered_odometry_topic': '/odometry/filtered',
            'command_frame': 'base_link',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.config_file = self._string('config_file')
        self.output_dir = Path(self._string('output_dir')).expanduser()
        self.selected_profile = self._string('test_profile')
        self.evaluation_mode = self._string('evaluation_mode')
        if self.evaluation_mode not in ('raw_only', 'raw_and_filtered'):
            raise ValueError(
                'evaluation_mode must be raw_only or raw_and_filtered')
        self.repetitions_override = int(
            self.get_parameter('repetitions').value
        )
        self.sample_rate = self._positive('sample_rate')
        self.startup_timeout = self._positive('startup_timeout')
        self.data_timeout = self._positive('data_timeout')
        self.stop_publish_duration = self._positive('stop_publish_duration')
        self.stop_velocity_threshold = self._positive(
            'stop_velocity_threshold'
        )
        self.reset_position_tolerance = self._positive(
            'reset_position_tolerance')
        self.reset_yaw_tolerance = self._positive('reset_yaw_tolerance')
        self.reset_timeout = self._positive('reset_timeout')
        self.max_source_skew = self._positive('max_source_skew')
        self.world = self._string('world')
        self.robot_entity = self._string('robot_entity')
        self.command_frame = self._string('command_frame')
        self.spawn = Pose2D(
            float(self.get_parameter('spawn_x').value),
            float(self.get_parameter('spawn_y').value),
            float(self.get_parameter('spawn_yaw').value),
        )
        self.spawn_z = float(self.get_parameter('spawn_z').value)
        if not all(math.isfinite(value) for value in (
                self.spawn.x, self.spawn.y, self.spawn.yaw, self.spawn_z)):
            raise ValueError('spawn pose must contain finite values')

        self.profiles = load_profiles(self.config_file)
        if (
            self.selected_profile != 'all'
            and self.selected_profile not in self.profiles
        ):
            raise ValueError(
                f'unknown test_profile: {self.selected_profile}'
            )

        self.latest_ground_truth = None
        self.latest_ground_truth_wall = 0.0
        self.latest_odometry = None
        self.latest_odometry_message = None
        self.latest_odometry_wall = 0.0
        self.latest_filtered = None
        self.latest_filtered_wall = 0.0
        self._last_sample_stamp = None
        self._shutdown_requested = False

        self.command_publisher = self.create_publisher(
            TwistStamped, self._string('command_topic'), 10
        )
        self.ground_truth_subscription = self.create_subscription(
            PoseStamped,
            self._string('ground_truth_topic'),
            self._ground_truth_callback,
            qos_profile_sensor_data,
        )
        self.odometry_subscription = self.create_subscription(
            Odometry,
            self._string('odometry_topic'),
            self._odometry_callback,
            10,
        )
        self.filtered_subscription = self.create_subscription(
            Odometry,
            self._string('filtered_odometry_topic'),
            self._filtered_callback,
            10,
        )
        self.set_pose_client = self.create_client(
            SetEntityPose, self._string('set_pose_service')
        )

    def _string(self, name):
        value = str(self.get_parameter(name).value)
        if not value:
            raise ValueError(f'{name} must not be empty')
        return value

    def _positive(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    @staticmethod
    def _pose_from_message(pose, stamp):
        quaternion = pose.orientation
        result = Pose2D(
            pose.position.x,
            pose.position.y,
            quaternion_to_yaw(
                quaternion.x, quaternion.y, quaternion.z, quaternion.w
            ),
            stamp,
        )
        if not all(math.isfinite(value) for value in (
                result.x, result.y, result.yaw, result.stamp)):
            raise ValueError('pose contains non-finite values')
        return result

    def _ground_truth_callback(self, message):
        stamp = message.header.stamp
        stamp_seconds = stamp.sec + stamp.nanosec / 1e9
        if stamp_seconds <= 0.0:
            return
        try:
            pose = self._pose_from_message(message.pose, stamp_seconds)
        except ValueError as error:
            self.get_logger().error(f'Invalid Gazebo pose: {error}')
            return
        try:
            validate_monotonic_timestamp(
                pose.stamp,
                self.latest_ground_truth.stamp
                if self.latest_ground_truth is not None else None,
            )
        except ValueError:
            self.get_logger().error('Nonmonotonic Gazebo pose timestamp')
            return
        self.latest_ground_truth = pose
        self.latest_ground_truth_wall = time.monotonic()

    def _filtered_callback(self, message):
        stamp = message.header.stamp
        stamp_seconds = stamp.sec + stamp.nanosec / 1e9
        if stamp_seconds <= 0.0:
            return
        try:
            pose = self._pose_from_message(
                message.pose.pose, stamp_seconds)
        except ValueError as error:
            self.get_logger().error(f'Invalid filtered odometry pose: {error}')
            return
        if (
            self.latest_filtered is not None
            and pose.stamp < self.latest_filtered.stamp
        ):
            self.get_logger().error(
                'Nonmonotonic filtered odometry timestamp')
            return
        self.latest_filtered = pose
        self.latest_filtered_wall = time.monotonic()

    def _odometry_callback(self, message):
        stamp = message.header.stamp
        stamp_seconds = stamp.sec + stamp.nanosec / 1e9
        if stamp_seconds <= 0.0:
            return
        try:
            pose = self._pose_from_message(
                message.pose.pose, stamp_seconds
            )
        except ValueError as error:
            self.get_logger().error(f'Invalid odometry pose: {error}')
            return
        if (
            self.latest_odometry is not None
            and pose.stamp < self.latest_odometry.stamp
        ):
            self.get_logger().error('Nonmonotonic odometry timestamp')
            return
        self.latest_odometry = pose
        self.latest_odometry_message = message
        self.latest_odometry_wall = time.monotonic()

    def _spin_until(self, predicate, timeout, reason):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise EvaluationError(reason)

    def wait_until_ready(self):
        """Wait boundedly for controller, odometry, Gazebo pose, and reset."""
        self._spin_until(
            lambda: (
                self.latest_ground_truth is not None
                and self.latest_odometry is not None
                and (
                    self.evaluation_mode == 'raw_only'
                    or self.latest_filtered is not None
                )
                and self.set_pose_client.service_is_ready()
                and self.command_publisher.get_subscription_count() > 0
            ),
            self.startup_timeout,
            'startup timeout waiting for controller, ground truth, '
            'odometry streams, or reset',
        )

    def publish_command(self, command=None):
        """Publish a fully initialized native controller command."""
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.command_frame
        if command is not None:
            message.twist.linear.x = command.linear_x
            message.twist.linear.y = command.linear_y
            message.twist.angular.z = command.angular_z
        self.command_publisher.publish(message)

    def safe_stop(self, duration=None):
        """Publish repeated zero commands and verify odometry settles."""
        duration = duration or self.stop_publish_duration
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_command()
            rclpy.spin_once(self, timeout_sec=1.0 / self.sample_rate)
        message = self.latest_odometry_message
        if message is None:
            raise EvaluationError('cannot confirm stop without odometry')
        twist = message.twist.twist
        if any(abs(value) > self.stop_velocity_threshold for value in (
                twist.linear.x, twist.linear.y, twist.angular.z)):
            raise EvaluationError('robot did not settle below stop threshold')

    def _request_reset(self):
        request = SetEntityPose.Request()
        request.entity.name = self.robot_entity
        request.entity.type = Entity.MODEL
        request.pose.position.x = self.spawn.x
        request.pose.position.y = self.spawn.y
        request.pose.position.z = self.spawn_z
        half_yaw = self.spawn.yaw / 2.0
        request.pose.orientation.z = math.sin(half_yaw)
        request.pose.orientation.w = math.cos(half_yaw)
        future = self.set_pose_client.call_async(request)
        self._spin_until(
            future.done, self.reset_timeout, 'set-pose service timed out'
        )
        response = future.result()
        if response is None or not response.success:
            raise EvaluationError('Gazebo rejected entity pose reset')

    def reset_repetition(self, settle_duration):
        """Stop, reset the Gazebo entity, confirm, and record fresh inputs."""
        self.safe_stop()
        previous_gt_stamp = self.latest_ground_truth.stamp
        self._request_reset()
        self._spin_until(
            lambda: (
                self.latest_ground_truth.stamp > previous_gt_stamp
                and math.hypot(
                    self.latest_ground_truth.x - self.spawn.x,
                    self.latest_ground_truth.y - self.spawn.y,
                ) <= self.reset_position_tolerance
                and abs(math.atan2(
                    math.sin(
                        self.latest_ground_truth.yaw - self.spawn.yaw),
                    math.cos(
                        self.latest_ground_truth.yaw - self.spawn.yaw),
                )) <= self.reset_yaw_tolerance
            ),
            self.reset_timeout,
            'Gazebo ground truth did not verify the requested reset pose',
        )
        deadline = time.monotonic() + settle_duration
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_command()
            rclpy.spin_once(self, timeout_sec=1.0 / self.sample_rate)
        self._assert_fresh_data()
        return (
            self.latest_ground_truth,
            self.latest_odometry,
            self.latest_filtered,
        )

    def _assert_fresh_data(self):
        now = time.monotonic()
        if self.latest_ground_truth is None or data_is_stale(
                self.latest_ground_truth_wall, now, self.data_timeout):
            raise EvaluationError('ground-truth pose is missing or stale')
        if self.latest_odometry is None or data_is_stale(
                self.latest_odometry_wall, now, self.data_timeout):
            raise EvaluationError('odometry pose is missing or stale')
        if self.evaluation_mode == 'raw_and_filtered':
            if self.latest_filtered is None or data_is_stale(
                    self.latest_filtered_wall, now, self.data_timeout):
                raise EvaluationError(
                    'filtered odometry pose is missing or stale')
            stamps = (
                self.latest_ground_truth.stamp,
                self.latest_odometry.stamp,
                self.latest_filtered.stamp,
            )
            if max(stamps) - min(stamps) > self.max_source_skew:
                raise EvaluationError(
                    'ground-truth/raw/filtered timestamp skew exceeded limit')

    def _sample(self, command):
        self._assert_fresh_data()
        stamp = max(
            self.latest_ground_truth.stamp,
            self.latest_odometry.stamp,
            self.latest_filtered.stamp
            if self.latest_filtered is not None else 0.0,
        )
        if self._last_sample_stamp is not None and stamp < self._last_sample_stamp:
            raise EvaluationError('nonmonotonic sampled trajectory timestamp')
        self._last_sample_stamp = stamp
        return {
            'timestamp': stamp,
            'commanded_linear_x': command.linear_x,
            'commanded_linear_y': command.linear_y,
            'commanded_angular_z': command.angular_z,
            'ground_truth_x': self.latest_ground_truth.x,
            'ground_truth_y': self.latest_ground_truth.y,
            'ground_truth_yaw': self.latest_ground_truth.yaw,
            'ground_truth_stamp': self.latest_ground_truth.stamp,
            'odometry_x': self.latest_odometry.x,
            'odometry_y': self.latest_odometry.y,
            'odometry_yaw': self.latest_odometry.yaw,
            'raw_odometry_stamp': self.latest_odometry.stamp,
            'filtered_x': (
                self.latest_filtered.x
                if self.latest_filtered is not None else ''),
            'filtered_y': (
                self.latest_filtered.y
                if self.latest_filtered is not None else ''),
            'filtered_yaw': (
                self.latest_filtered.yaw
                if self.latest_filtered is not None else ''),
            'filtered_odometry_stamp': (
                self.latest_filtered.stamp
                if self.latest_filtered is not None else ''),
        }

    def _execute_segment(
            self, segment, samples, profile_started, profile_timeout):
        initial = self.latest_ground_truth
        initial_odometry = self.latest_odometry
        started = time.monotonic()
        while rclpy.ok():
            if timeout_expired(started, time.monotonic(), segment.timeout):
                raise EvaluationError(f'{segment.name} timed out')
            if timeout_expired(
                    profile_started, time.monotonic(), profile_timeout):
                raise EvaluationError('profile timed out')
            self.publish_command(segment.command)
            rclpy.spin_once(self, timeout_sec=1.0 / self.sample_rate)
            samples.append(self._sample(segment.command))
            if termination_reached(
                    segment, initial, self.latest_ground_truth):
                self.publish_command()
                ground_truth_delta = relative_pose(
                    initial, self.latest_ground_truth
                )
                odometry_delta = relative_pose(
                    initial_odometry, self.latest_odometry
                )
                error_x, error_y, error_norm = position_error(
                    ground_truth_delta, odometry_delta
                )
                if abs(segment.command.linear_x) > 0.0:
                    segment_drift = abs(ground_truth_delta.y)
                else:
                    segment_drift = abs(ground_truth_delta.x)
                return {
                    'name': segment.name,
                    'duration': (
                        self.latest_ground_truth.stamp - initial.stamp
                    ),
                    'ground_truth_start': initial.to_dict(),
                    'ground_truth_final': self.latest_ground_truth.to_dict(),
                    'odometry_start': initial_odometry.to_dict(),
                    'odometry_final': self.latest_odometry.to_dict(),
                    'position_error_x': error_x,
                    'position_error_y': error_y,
                    'position_error_norm': error_norm,
                    'cross_axis_drift': segment_drift,
                }

    def execute_run(self, profile, repetition):
        """Execute one reset-isolated repetition and calculate its metrics."""
        samples = []
        segment_results = []
        self._last_sample_stamp = None
        initial_gt, initial_odom, initial_filtered = self.reset_repetition(
            profile.settle_before)
        zero = profile.segments[0].command.__class__(0.0, 0.0, 0.0)
        samples.append(self._sample(zero))
        profile_started = time.monotonic()
        motion_end_gt = initial_gt
        motion_end_odom = initial_odom
        motion_end_filtered = initial_filtered
        try:
            for segment in profile.segments:
                segment_results.append(self._execute_segment(
                    segment,
                    samples,
                    profile_started,
                    profile.profile_timeout,
                ))
                motion_end_gt = self.latest_ground_truth
                motion_end_odom = self.latest_odometry
                motion_end_filtered = self.latest_filtered
                self.safe_stop()
                settle_deadline = time.monotonic() + profile.settle_after
                while time.monotonic() < settle_deadline:
                    self.publish_command()
                    rclpy.spin_once(
                        self, timeout_sec=1.0 / self.sample_rate
                    )
                    samples.append(self._sample(zero))
        finally:
            self.safe_stop()
        final_gt = self.latest_ground_truth
        final_odom = self.latest_odometry
        final_filtered = self.latest_filtered
        ground_truth_poses = [
            Pose2D(
                sample['ground_truth_x'], sample['ground_truth_y'],
                sample['ground_truth_yaw'], sample['timestamp'],
            )
            for sample in samples
        ]
        odometry_poses = [
            Pose2D(
                sample['odometry_x'], sample['odometry_y'],
                sample['odometry_yaw'], sample['timestamp'],
            )
            for sample in samples
        ]
        filtered_poses = None
        if self.evaluation_mode == 'raw_and_filtered':
            filtered_poses = [
                Pose2D(
                    sample['filtered_x'],
                    sample['filtered_y'],
                    sample['filtered_yaw'],
                    sample['filtered_odometry_stamp'],
                )
                for sample in samples
            ]
        ground_truth_poses[0] = initial_gt
        odometry_poses[0] = initial_odom
        ground_truth_poses[-1] = final_gt
        odometry_poses[-1] = final_odom
        if filtered_poses is not None:
            filtered_poses[0] = initial_filtered
            filtered_poses[-1] = final_filtered
        result = calculate_run_metrics(
            profile,
            ground_truth_poses,
            odometry_poses,
            final_gt.stamp - initial_gt.stamp,
            filtered_poses=filtered_poses,
            motion_end_ground_truth=motion_end_gt,
            motion_end_raw_odometry=motion_end_odom,
            motion_end_filtered_odometry=motion_end_filtered,
        )
        if profile.kind == 'square':
            result['accumulated_drift'] = sum(
                segment['cross_axis_drift'] for segment in segment_results
            )
        result.update({
            'timestamp': datetime.now(timezone.utc).isoformat(),
            'command': profile.segments[0].command.__dict__,
            'profile': profile.name,
            'repetition': repetition,
            'ground_truth_initial': initial_gt.to_dict(),
            'ground_truth_final': final_gt.to_dict(),
            'odometry_initial': initial_odom.to_dict(),
            'odometry_final': final_odom.to_dict(),
            'motion_end_ground_truth': motion_end_gt.to_dict(),
            'motion_end_raw_odometry': motion_end_odom.to_dict(),
            'settled_final_ground_truth': final_gt.to_dict(),
            'settled_final_raw_odometry': final_odom.to_dict(),
            'segments': segment_results,
        })
        if filtered_poses is not None:
            result.update({
                'filtered_odometry_initial': initial_filtered.to_dict(),
                'filtered_odometry_final': final_filtered.to_dict(),
                'motion_end_filtered_odometry': (
                    motion_end_filtered.to_dict()),
                'settled_final_filtered_odometry': final_filtered.to_dict(),
            })
        trajectory_name = f'{profile.name}_run_{repetition:02d}.csv'
        trajectory_path = self.output_dir / 'trajectories' / trajectory_name
        write_trajectory(trajectory_path, samples)
        plot_trajectory(
            self.output_dir / 'plots' / trajectory_name.replace('.csv', '.png'),
            profile.name,
            samples,
        )
        if profile.kind == 'square':
            plot_trajectory(
                self.output_dir / 'plots' / 'square_path_closure.png',
                'Square-path closure',
                samples,
            )
        result['trajectory_file'] = str(trajectory_path)
        return result

    @staticmethod
    def _version(command):
        try:
            return subprocess.run(
                command,
                check=True,
                capture_output=True,
                text=True,
                timeout=5,
            ).stdout.strip().splitlines()[0]
        except (OSError, subprocess.SubprocessError, IndexError):
            return 'unavailable'

    def run_suite(self):
        """Run selected profiles, preserving failures in the final report."""
        self.wait_until_ready()
        selected = (
            self.profiles.values()
            if self.selected_profile == 'all'
            else (self.profiles[self.selected_profile],)
        )
        runs = []
        try:
            for profile in selected:
                repetitions = (
                    self.repetitions_override
                    if self.repetitions_override > 0
                    else profile.repetitions
                )
                for repetition in range(1, repetitions + 1):
                    self.get_logger().info(
                        f'Running {profile.name}, repetition {repetition}/'
                        f'{repetitions}'
                    )
                    run_started = time.monotonic()
                    try:
                        runs.append(self.execute_run(profile, repetition))
                    except EvaluationError as error:
                        self.publish_command()
                        runs.append({
                            'timestamp': datetime.now(timezone.utc).isoformat(),
                            'profile': profile.name,
                            'repetition': repetition,
                            'completion_status': (
                                'timeout' if 'timed out' in str(error)
                                else 'failed'
                            ),
                            'timeout_status': 'timed out' in str(error),
                            'reason': str(error),
                            'duration': time.monotonic() - run_started,
                            'command': (
                                profile.segments[0].command.__dict__),
                        })
                        self.get_logger().error(str(error))
        finally:
            try:
                self.safe_stop()
            except EvaluationError as error:
                self.get_logger().error(f'Final safe stop failed: {error}')

        configuration = {
            'config_file': self.config_file,
            'selected_profile': self.selected_profile,
            'repetitions_override': self.repetitions_override,
            'sample_rate': self.sample_rate,
            'evaluation_mode': self.evaluation_mode,
        }
        metadata = {
            'git_commit': self._version(
                ['git', 'rev-parse', 'HEAD']
            ),
            'ros_distribution': os.environ.get('ROS_DISTRO', 'unknown'),
            'gazebo_version': self._version(['gz', 'sim', '--version']),
            'world_name': self.world,
            'robot_entity_name': self.robot_entity,
            'lidar_max_range': 4.0,
            'controller_topic': self._string('command_topic'),
            'odometry_topic': self._string('odometry_topic'),
            'filtered_odometry_topic': self._string(
                'filtered_odometry_topic'),
            'imu_topic': '/imu/data',
            'ground_truth_source': (
                f'Gazebo /world/{self.world}/dynamic_pose/info '
                f'identity-selected entity {self.robot_entity}'
            ),
            'pose_reset_service': self._string('set_pose_service'),
        }
        report = build_report(metadata, configuration, runs)
        write_report(self.output_dir, report)
        write_summary_plots(self.output_dir / 'plots', report)
        self.get_logger().info(f'Report written to {self.output_dir}')
        return all(run['completion_status'] == 'completed' for run in runs)


def main(args=None):
    """Run the evaluator and always issue a final zero command."""
    rclpy.init(args=args)
    node = None
    exit_code = 1
    try:
        node = OdometryTestRunner()
        exit_code = 0 if node.run_suite() else 1
    except (EvaluationError, ValueError, OSError) as error:
        if node is not None:
            node.get_logger().error(f'Evaluation aborted: {error}')
    except KeyboardInterrupt:
        if node is not None and rclpy.ok():
            node.get_logger().warning('Evaluation cancelled')
    finally:
        if node is not None:
            if rclpy.ok():
                try:
                    node.publish_command()
                    rclpy.spin_once(node, timeout_sec=0.1)
                except (
                    rclpy.executors.ExternalShutdownException,
                    RuntimeError,
                ):
                    pass
            try:
                node.destroy_node()
            except RuntimeError:
                pass
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)
