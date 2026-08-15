#!/usr/bin/env python3
"""Run YAML-defined raw wheel-odometry evaluation profiles."""

from datetime import datetime, timezone
import math
import os
from pathlib import Path
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from control_msgs.msg import MecanumDriveControllerState
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseStamped, TwistStamped
from mobile_base_tools.mecanum_diagnostics import (
    chassis_motion_metrics,
    classify_root_cause,
    expected_wheel_velocities,
    wheel_tracking_metrics,
)
from mobile_base_tools.motion_profiles import load_profiles
from mobile_base_tools.odometry_evaluator import (
    calculate_run_metrics,
    command_progress,
    command_stamp_status,
    ordered_profiles,
    post_reset_streams_fresh,
    termination_reached,
    timeout_reason,
    update_settle_window,
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
    write_run_result,
    write_trajectory,
)
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import Imu, JointState


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
            'profile_sequence': '',
            'evaluation_mode': 'raw_and_filtered',
            'localization': True,
            'repetitions': 0,
            'repetition_offset': 0,
            'sample_rate': 20.0,
            'startup_timeout': 60.0,
            'data_timeout': 2.0,
            'stop_publish_duration': 0.5,
            'stop_velocity_threshold': 0.01,
            'reset_position_tolerance': 0.03,
            'reset_yaw_tolerance': 0.05,
            'reset_timeout': 5.0,
            'settle_window': 0.5,
            'max_source_skew': 0.1,
            'wall_watchdog_factor': 3.0,
            'minimum_wall_watchdog': 30.0,
            'clock_stall_timeout': 5.0,
            'motion_start_timeout': 2.0,
            'motion_start_speed_threshold': 0.005,
            'progress_log_interval': 2.0,
            'diagnostic_verbose': False,
            'profile_order': 'configured',
            'random_seed': 0,
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
            'list_controllers_service': (
                '/controller_manager/list_controllers'
            ),
            'controller_name': 'mobile_base_controller',
            'command_topic': '/mobile_base_controller/reference',
            'odometry_topic': '/mobile_base_controller/odometry',
            'filtered_odometry_topic': '/odometry/filtered',
            'controller_state_topic': (
                '/mobile_base_controller/controller_state'),
            'joint_states_topic': '/joint_states',
            'imu_topic': '/imu/data',
            'wheels_radius': 0.03074443,
            'center_projection_sum': 0.142,
            'command_frame': 'base_link',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.config_file = self._string('config_file')
        self.output_dir = Path(self._string('output_dir')).expanduser()
        self.selected_profile = self._string('test_profile')
        self.profile_sequence = str(
            self.get_parameter('profile_sequence').value).strip()
        self.evaluation_mode = self._string('evaluation_mode')
        self.localization_enabled = bool(
            self.get_parameter('localization').value)
        if self.evaluation_mode not in ('raw_only', 'raw_and_filtered'):
            raise ValueError(
                'evaluation_mode must be raw_only or raw_and_filtered')
        self.repetitions_override = int(
            self.get_parameter('repetitions').value
        )
        self.repetition_offset = int(
            self.get_parameter('repetition_offset').value)
        if self.repetition_offset < 0:
            raise ValueError('repetition_offset must not be negative')
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
        self.settle_window = self._positive('settle_window')
        self.max_source_skew = self._positive('max_source_skew')
        self.wall_watchdog_factor = self._positive('wall_watchdog_factor')
        self.minimum_wall_watchdog = self._positive(
            'minimum_wall_watchdog')
        self.clock_stall_timeout = self._positive('clock_stall_timeout')
        self.motion_start_timeout = self._positive('motion_start_timeout')
        self.motion_start_speed_threshold = self._positive(
            'motion_start_speed_threshold')
        self.progress_log_interval = self._positive(
            'progress_log_interval')
        self.diagnostic_verbose = bool(
            self.get_parameter('diagnostic_verbose').value)
        self.profile_order = self._string('profile_order')
        if self.profile_order not in ('configured', 'randomized'):
            raise ValueError(
                'profile_order must be configured or randomized')
        self.random_seed = int(self.get_parameter('random_seed').value)
        self.wheels_radius = self._positive('wheels_radius')
        self.center_projection_sum = self._positive(
            'center_projection_sum')
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
        sequence_names = [
            name.strip() for name in self.profile_sequence.split(',')
            if name.strip()
        ]
        unknown_sequence = set(sequence_names) - set(self.profiles)
        if unknown_sequence:
            raise ValueError(
                'unknown profile_sequence entries: '
                + ', '.join(sorted(unknown_sequence)))
        if (not sequence_names and self.selected_profile != 'all'
                and self.selected_profile not in self.profiles):
            raise ValueError(
                f'unknown test_profile: {self.selected_profile}'
            )
        self.profile_sequence_names = sequence_names

        self.latest_ground_truth = None
        self.latest_ground_truth_wall = 0.0
        self.latest_odometry = None
        self.latest_odometry_message = None
        self.latest_odometry_wall = 0.0
        self.latest_filtered = None
        self.latest_filtered_message = None
        self.latest_filtered_wall = 0.0
        self.latest_controller_state = None
        self.latest_controller_state_wall = 0.0
        self.latest_joint_state = None
        self.latest_joint_state_wall = 0.0
        self.latest_imu = None
        self.latest_imu_wall = 0.0
        self._last_sample_stamp = None
        self._previous_ground_truth = None
        self._previous_odometry = None
        self.ground_truth_speed = 0.0
        self.ground_truth_velocity = {
            'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': 0.0}
        self.raw_odometry_speed = 0.0
        self.command_diagnostics = self._new_command_diagnostics()
        self.last_command = None
        self.last_command_stamp = None
        self.last_command_wall = None
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
        self.controller_state_subscription = self.create_subscription(
            MecanumDriveControllerState,
            self._string('controller_state_topic'),
            self._controller_state_callback,
            10,
        )
        self.joint_state_subscription = self.create_subscription(
            JointState,
            self._string('joint_states_topic'),
            self._joint_state_callback,
            qos_profile_sensor_data,
        )
        self.imu_subscription = self.create_subscription(
            Imu,
            self._string('imu_topic'),
            self._imu_callback,
            qos_profile_sensor_data,
        )
        self.set_pose_client = self.create_client(
            SetEntityPose, self._string('set_pose_service')
        )
        self.list_controllers_client = self.create_client(
            ListControllers, self._string('list_controllers_service')
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
    def _new_command_diagnostics():
        return {
            'commands_published': 0,
            'duplicate_command_timestamps': 0,
            'nonmonotonic_command_timestamps': 0,
            'rate_limited_command_attempts': 0,
            'maximum_publish_to_current_sim_time_age': 0.0,
            'motion_start_failures': 0,
        }

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
        previous = self.latest_ground_truth
        if previous is not None and pose.stamp > previous.stamp:
            delta = relative_pose(previous, pose)
            duration = pose.stamp - previous.stamp
            self.ground_truth_speed = math.hypot(delta.x, delta.y) / duration
            self.ground_truth_velocity = {
                'linear_x': delta.x / duration,
                'linear_y': delta.y / duration,
                'angular_z': delta.yaw / duration,
            }
        self._previous_ground_truth = previous
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
        self.latest_filtered_message = message
        self.latest_filtered_wall = time.monotonic()

    def _controller_state_callback(self, message):
        self.latest_controller_state = message
        self.latest_controller_state_wall = time.monotonic()

    def _joint_state_callback(self, message):
        required = {
            'front_left_wheel_joint', 'front_right_wheel_joint',
            'rear_right_wheel_joint', 'rear_left_wheel_joint'}
        if required <= set(message.name):
            self.latest_joint_state = message
            self.latest_joint_state_wall = time.monotonic()

    def _imu_callback(self, message):
        self.latest_imu = message
        self.latest_imu_wall = time.monotonic()

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
        previous = self.latest_odometry
        if previous is not None and pose.stamp > previous.stamp:
            delta = relative_pose(previous, pose)
            self.raw_odometry_speed = (
                math.hypot(delta.x, delta.y) / (pose.stamp - previous.stamp)
            )
        self._previous_odometry = previous
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
            self.list_controllers_client.service_is_ready,
            self.startup_timeout,
            'startup timeout waiting for controller manager',
        )
        deadline = time.monotonic() + self.startup_timeout
        controller_active = False
        while rclpy.ok() and time.monotonic() < deadline:
            future = self.list_controllers_client.call_async(
                ListControllers.Request())
            self._spin_until(
                future.done,
                min(10.0, max(0.1, deadline - time.monotonic())),
                'controller-manager list request timed out',
            )
            response = future.result()
            controller_active = any(
                controller.name == self._string('controller_name')
                and controller.state == 'active'
                for controller in response.controller
            )
            if controller_active:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        if not controller_active:
            raise EvaluationError(
                f'{self._string("controller_name")} did not become active')

        # Discard any samples produced before activation and require a fresh,
        # mutually aligned startup baseline.
        self.latest_ground_truth = None
        self.latest_odometry = None
        self.latest_odometry_message = None
        self.latest_filtered = None
        self.latest_filtered_message = None
        self.latest_controller_state = None
        self.latest_joint_state = None
        self.latest_imu = None
        self._spin_until(
            lambda: (
                self.latest_ground_truth is not None
                and self.latest_odometry is not None
                and self._sim_now() > 0.0
                and abs(
                    self._sim_now() - self.latest_ground_truth.stamp
                ) <= self.max_source_skew
                and (
                    self.evaluation_mode == 'raw_only'
                    or self.latest_filtered is not None
                )
                and self.latest_controller_state is not None
                and self.latest_joint_state is not None
                and self.latest_imu is not None
                and self.set_pose_client.service_is_ready()
                and self.command_publisher.get_subscription_count() > 0
            ),
            self.startup_timeout,
            'startup timeout waiting for controller, ground truth, '
            'odometry streams, or reset',
        )

    def _sim_now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _wait_for_clock_advance(self, previous, timeout=None):
        timeout = timeout or self.clock_stall_timeout
        self._spin_until(
            lambda: self._sim_now() > max(0.0, previous),
            timeout,
            'clock stalled',
        )
        return self._sim_now()

    def publish_command(self, command=None, force=False):
        """Publish one fresh command, never bursting a duplicate ROS stamp."""
        rclpy.spin_once(self, timeout_sec=0.0)
        stamp = self._sim_now()
        stamp_status = command_stamp_status(self.last_command_stamp, stamp)
        if stamp_status == 'zero':
            raise EvaluationError('simulation clock is zero')
        if stamp_status == 'nonmonotonic':
            self.command_diagnostics[
                'nonmonotonic_command_timestamps'] += 1
            raise EvaluationError(
                'nonmonotonic command timestamp: '
                f'current={stamp:.9f}, previous={self.last_command_stamp:.9f}'
            )
        if stamp_status == 'duplicate':
            self.command_diagnostics[
                'duplicate_command_timestamps'] += 1
            return False
        if (
            not force
            and self.last_command_stamp is not None
            and stamp - self.last_command_stamp < 1.0 / self.sample_rate
        ):
            self.command_diagnostics[
                'rate_limited_command_attempts'] += 1
            return False
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.command_frame
        if command is not None:
            message.twist.linear.x = command.linear_x
            message.twist.linear.y = command.linear_y
            message.twist.angular.z = command.angular_z
        self.command_publisher.publish(message)
        publish_wall = time.monotonic()
        current_sim = self._sim_now()
        age = max(0.0, current_sim - stamp)
        diagnostics = self.command_diagnostics
        diagnostics['commands_published'] += 1
        diagnostics['maximum_publish_to_current_sim_time_age'] = max(
            diagnostics['maximum_publish_to_current_sim_time_age'], age)
        self.last_command = {
            'linear_x': message.twist.linear.x,
            'linear_y': message.twist.linear.y,
            'angular_z': message.twist.angular.z,
        }
        self.last_command_stamp = stamp
        self.last_command_wall = publish_wall
        return True

    def safe_stop(self, duration=None):
        """Publish zero commands and require continuously settled odometry."""
        duration = duration or self.stop_publish_duration
        wall_deadline = time.monotonic() + max(
            self.minimum_wall_watchdog,
            duration * self.wall_watchdog_factor,
        )
        settled_since = None
        first_zero_command = True
        last_clock = self._sim_now()
        last_clock_advance_wall = time.monotonic()
        while rclpy.ok() and time.monotonic() < wall_deadline:
            rclpy.spin_once(self, timeout_sec=1.0 / self.sample_rate)
            current_clock = self._sim_now()
            if current_clock > last_clock:
                last_clock = current_clock
                last_clock_advance_wall = time.monotonic()
            if self.publish_command(force=first_zero_command):
                first_zero_command = False
            message = self.latest_odometry_message
            if message is None:
                settled_since = None
                continue
            twist = message.twist.twist
            stopped = all(
                abs(value) <= self.stop_velocity_threshold for value in (
                    twist.linear.x, twist.linear.y, twist.angular.z
                )
            )
            settled_since, complete = update_settle_window(
                settled_since,
                current_clock,
                stopped,
                max(duration, self.settle_window),
            )
            if complete:
                return
            if (
                time.monotonic() - last_clock_advance_wall
                >= self.clock_stall_timeout
            ):
                raise EvaluationError('clock stalled while stopping')
        raise EvaluationError('robot did not remain settled continuously')

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
        previous_raw_stamp = self.latest_odometry.stamp
        previous_filtered_stamp = (
            self.latest_filtered.stamp
            if self.latest_filtered is not None else None
        )
        reset_boundary = self._sim_now()
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
        self._spin_until(
            lambda: post_reset_streams_fresh(
                self.latest_ground_truth.stamp,
                previous_gt_stamp,
                self.latest_odometry.stamp,
                previous_raw_stamp,
            ),
            self.reset_timeout,
            'raw odometry did not update after reset',
        )
        if self.evaluation_mode == 'raw_and_filtered':
            self._spin_until(
                lambda: post_reset_streams_fresh(
                    self.latest_ground_truth.stamp,
                    previous_gt_stamp,
                    self.latest_odometry.stamp,
                    previous_raw_stamp,
                    (
                        self.latest_filtered.stamp
                        if self.latest_filtered is not None else None
                    ),
                    previous_filtered_stamp,
                    require_filtered=True,
                ),
                self.reset_timeout,
                'filtered odometry did not update after reset',
            )
        self._wait_for_clock_advance(reset_boundary)
        self.safe_stop(max(settle_duration, self.settle_window))
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
        for name, message, received in (
            ('controller state', self.latest_controller_state,
             self.latest_controller_state_wall),
            ('joint state', self.latest_joint_state,
             self.latest_joint_state_wall),
            ('IMU', self.latest_imu, self.latest_imu_wall),
        ):
            if message is None or data_is_stale(
                    received, now, self.data_timeout):
                raise EvaluationError(f'{name} is missing or stale')

    @staticmethod
    def _joint_value(message, joint, field):
        index = message.name.index(joint)
        values = getattr(message, field)
        return values[index] if index < len(values) else None

    def _commanded_motion_rate(self, command):
        """Return the measured rate relevant to the commanded motion type."""
        if math.hypot(command.linear_x, command.linear_y) > 1.0e-9:
            return max(self.ground_truth_speed, self.raw_odometry_speed)
        return max(
            abs(self.ground_truth_velocity['angular_z']),
            abs(self.latest_odometry_message.twist.twist.angular.z),
        )

    def _sample(
            self, command, wall_elapsed=0.0, simulation_elapsed=0.0):
        self._assert_fresh_data()
        stamp = max(
            self.latest_ground_truth.stamp,
            self.latest_odometry.stamp,
            self.latest_filtered.stamp
            if self.latest_filtered is not None else 0.0,
        )
        if (
            self._last_sample_stamp is not None
            and stamp < self._last_sample_stamp
        ):
            raise EvaluationError('nonmonotonic sampled trajectory timestamp')
        self._last_sample_stamp = stamp
        expected = expected_wheel_velocities(
            command.linear_x, command.linear_y, command.angular_z,
            self.wheels_radius, self.center_projection_sum)
        controller = self.latest_controller_state
        joint_state = self.latest_joint_state
        reference = controller.reference_velocity
        wheel_joints = {
            'front_left': 'front_left_wheel_joint',
            'front_right': 'front_right_wheel_joint',
            'rear_right': 'rear_right_wheel_joint',
            'rear_left': 'rear_left_wheel_joint',
        }
        sample = {
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
            'wall_elapsed': wall_elapsed,
            'simulation_elapsed': simulation_elapsed,
            'real_time_factor': (
                simulation_elapsed / wall_elapsed
                if wall_elapsed > 0.0 else 0.0
            ),
            'ground_truth_speed': self.ground_truth_speed,
            'ground_truth_linear_x': self.ground_truth_velocity['linear_x'],
            'ground_truth_linear_y': self.ground_truth_velocity['linear_y'],
            'ground_truth_angular_z': (
                self.ground_truth_velocity['angular_z']),
            'raw_odometry_speed': self.raw_odometry_speed,
            'raw_odometry_linear_x': (
                self.latest_odometry_message.twist.twist.linear.x),
            'raw_odometry_linear_y': (
                self.latest_odometry_message.twist.twist.linear.y),
            'raw_odometry_angular_z': (
                self.latest_odometry_message.twist.twist.angular.z),
            'filtered_odometry_linear_x': (
                self.latest_filtered_message.twist.twist.linear.x
                if self.latest_filtered_message is not None else ''),
            'filtered_odometry_linear_y': (
                self.latest_filtered_message.twist.twist.linear.y
                if self.latest_filtered_message is not None else ''),
            'filtered_odometry_angular_z': (
                self.latest_filtered_message.twist.twist.angular.z
                if self.latest_filtered_message is not None else ''),
            'imu_angular_velocity_z': self.latest_imu.angular_velocity.z,
            'controller_reference_linear_x': reference.linear.x,
            'controller_reference_linear_y': reference.linear.y,
            'controller_reference_angular_z': reference.angular.z,
        }
        controller_velocities = {
            'front_left': controller.front_left_wheel_velocity,
            'front_right': controller.front_right_wheel_velocity,
            'rear_right': controller.back_right_wheel_velocity,
            'rear_left': controller.back_left_wheel_velocity,
        }
        for wheel, joint in wheel_joints.items():
            sample[f'expected_{wheel}_wheel_velocity'] = expected[wheel]
            sample[f'actual_{wheel}_wheel_velocity'] = self._joint_value(
                joint_state, joint, 'velocity')
            sample[f'{wheel}_wheel_position'] = self._joint_value(
                joint_state, joint, 'position')
            sample[f'controller_{wheel}_wheel_velocity'] = (
                controller_velocities[wheel])
        return sample

    def _execute_segment(
            self, segment, samples, profile_sim_started, profile_wall_started,
            profile_timeout):
        initial = self.latest_ground_truth
        initial_odometry = self.latest_odometry
        simulation_started = self._sim_now()
        wall_started = time.monotonic()
        wall_watchdog = max(
            self.minimum_wall_watchdog,
            segment.timeout * self.wall_watchdog_factor,
        )
        profile_wall_watchdog = max(
            self.minimum_wall_watchdog,
            profile_timeout * self.wall_watchdog_factor,
        )
        last_clock = simulation_started
        last_clock_advance_wall = wall_started
        motion_started = False
        next_progress_log = self.progress_log_interval
        self._active_segment_diagnostics = {
            'segment': segment.name,
            'target': segment.target,
            'simulation_started': simulation_started,
            'wall_started': wall_started,
            'motion_started': False,
        }
        self._wait_for_clock_advance(simulation_started)
        self.get_logger().info(
            f'Starting segment {segment.name}; simulation timeout '
            f'{segment.timeout:.3f}s, wall watchdog {wall_watchdog:.3f}s'
        )
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=1.0 / self.sample_rate)
            simulation_now = self._sim_now()
            wall_now = time.monotonic()
            if simulation_now > last_clock:
                last_clock = simulation_now
                last_clock_advance_wall = wall_now
            reason = timeout_reason(
                simulation_started, simulation_now, segment.timeout,
                wall_started, wall_now, wall_watchdog,
                last_clock_advance_wall, self.clock_stall_timeout,
            )
            if reason is None:
                reason = timeout_reason(
                    profile_sim_started, simulation_now, profile_timeout,
                    profile_wall_started, wall_now, profile_wall_watchdog,
                    last_clock_advance_wall, self.clock_stall_timeout,
                )
                if reason is not None:
                    reason = 'profile ' + reason
            if reason is not None:
                if reason == 'simulation timeout':
                    reason = (
                        f'{segment.name}: motion started but target was '
                        'not reached before simulation timeout'
                    )
                raise EvaluationError(reason)
            published = self.publish_command(segment.command)
            simulation_elapsed = simulation_now - simulation_started
            wall_elapsed = wall_now - wall_started
            if published:
                samples.append(self._sample(
                    segment.command, wall_elapsed, simulation_elapsed))
            speed = self._commanded_motion_rate(segment.command)
            if (
                not motion_started
                and speed >= self.motion_start_speed_threshold
            ):
                motion_started = True
                self._active_segment_diagnostics['motion_started'] = True
                self.get_logger().info(
                    f'{segment.name}: motion start confirmed at '
                    f'{simulation_elapsed:.3f}s simulation time'
                )
            if (
                not motion_started
                and simulation_elapsed >= self.motion_start_timeout
            ):
                self.command_diagnostics['motion_start_failures'] += 1
                raise EvaluationError('motion did not start after command')
            progress = command_progress(
                segment.command, initial, self.latest_ground_truth,
                segment.target)
            self._active_segment_diagnostics.update({
                **progress,
                'wall_elapsed': wall_elapsed,
                'simulation_elapsed': simulation_elapsed,
                'real_time_factor': (
                    simulation_elapsed / wall_elapsed
                    if wall_elapsed > 0.0 else 0.0
                ),
            })
            if (
                self.diagnostic_verbose
                or simulation_elapsed >= next_progress_log
            ):
                percentage = progress['percentage_of_target_reached']
                along = progress['along_track_displacement']
                cross = progress['cross_track_displacement']
                real_time_factor = self._active_segment_diagnostics[
                    'real_time_factor']
                self.get_logger().info(
                    f'{segment.name}: {percentage:.1f}% target, '
                    f'along={along:.3f}, cross={cross:.3f}, '
                    f'RTF={real_time_factor:.2f}'
                )
                next_progress_log += self.progress_log_interval
            if termination_reached(
                    segment, initial, self.latest_ground_truth):
                self.publish_command(force=True)
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
                    'wall_duration': wall_elapsed,
                    **progress,
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
        started_at = datetime.now(timezone.utc).isoformat()
        samples = []
        segment_results = []
        self.command_diagnostics = self._new_command_diagnostics()
        self.last_command = None
        self.last_command_stamp = None
        self.last_command_wall = None
        self._active_segment_diagnostics = {}
        self._last_sample_stamp = None
        initial_gt, initial_odom, initial_filtered = self.reset_repetition(
            profile.settle_before)
        zero = profile.segments[0].command.__class__(0.0, 0.0, 0.0)
        samples.append(self._sample(zero))
        profile_wall_started = time.monotonic()
        profile_sim_started = self._sim_now()
        motion_end_gt = initial_gt
        motion_end_odom = initial_odom
        motion_end_filtered = initial_filtered
        failure = None
        interrupted = False
        try:
            for segment in profile.segments:
                segment_results.append(self._execute_segment(
                    segment,
                    samples,
                    profile_sim_started,
                    profile_wall_started,
                    profile.profile_timeout,
                ))
                motion_end_gt = self.latest_ground_truth
                motion_end_odom = self.latest_odometry
                motion_end_filtered = self.latest_filtered
                self.safe_stop(max(
                    profile.settle_after,
                    self.settle_window,
                ))
                samples.append(self._sample(zero))
        except EvaluationError as error:
            failure = error
        except KeyboardInterrupt:
            failure = EvaluationError('interrupted by operator')
            interrupted = True
        finally:
            if rclpy.ok():
                try:
                    self.safe_stop()
                except (EvaluationError, RuntimeError) as stop_error:
                    if failure is None:
                        failure = stop_error
        final_gt = self.latest_ground_truth
        final_odom = self.latest_odometry
        final_filtered = self.latest_filtered
        if failure is not None:
            result = self._build_failure_result(
                profile, repetition, str(failure), samples,
                initial_gt, initial_odom, initial_filtered,
                profile_wall_started, profile_sim_started,
            )
            result['started_at'] = started_at
            result['finished_at'] = datetime.now(timezone.utc).isoformat()
            self._write_run_artifacts(profile, repetition, samples, result)
            if interrupted:
                raise KeyboardInterrupt
            return result
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
            'started_at': started_at,
            'finished_at': datetime.now(timezone.utc).isoformat(),
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
            'wall_duration': time.monotonic() - profile_wall_started,
            'simulation_duration': (
                self._sim_now() - profile_sim_started),
            'timeout_basis': (
                'simulation time with independent wall watchdog'),
            'command_diagnostics': dict(self.command_diagnostics),
        })
        wheel_metrics = wheel_tracking_metrics(samples)
        chassis_metrics = chassis_motion_metrics(samples, profile.kind)
        result['mecanum_diagnostics'] = {
            'measurement_only': True,
            'kinematics': {
                'wheels_radius': self.wheels_radius,
                'center_projection_sum': self.center_projection_sum,
            },
            'wheel_tracking': wheel_metrics,
            'chassis_motion': chassis_metrics,
            'root_cause': classify_root_cause(
                wheel_metrics, chassis_metrics),
        }
        if filtered_poses is not None:
            result.update({
                'filtered_odometry_initial': initial_filtered.to_dict(),
                'filtered_odometry_final': final_filtered.to_dict(),
                'motion_end_filtered_odometry': (
                    motion_end_filtered.to_dict()),
                'settled_final_filtered_odometry': final_filtered.to_dict(),
            })
        trajectory_path = self._write_run_artifacts(
            profile, repetition, samples, result)
        if profile.kind == 'square':
            plot_trajectory(
                self.output_dir / 'plots' / 'square_path_closure.png',
                'Square-path closure',
                samples,
            )
        result['trajectory_file'] = str(trajectory_path)
        return result

    def _write_run_artifacts(self, profile, repetition, samples, result):
        base = f'{profile.name}_run_{repetition:02d}'
        trajectory_path = self.output_dir / 'trajectories' / (base + '.csv')
        write_trajectory(trajectory_path, samples)
        if len(samples) >= 2:
            plot_trajectory(
                self.output_dir / 'plots' / (base + '.png'),
                profile.name,
                samples,
            )
            result['trajectory_plot'] = str(
                self.output_dir / 'plots' / (base + '.png'))
        result['trajectory_file'] = str(trajectory_path)
        result_path = self.output_dir / 'run_results' / (base + '.json')
        result['run_result_file'] = str(result_path)
        write_run_result(result_path, result)
        return trajectory_path

    def _build_failure_result(
            self, profile, repetition, reason, samples,
            initial_gt, initial_odom, initial_filtered,
            wall_started, simulation_started):
        final_gt = self.latest_ground_truth
        final_odom = self.latest_odometry
        final_filtered = self.latest_filtered
        wall_duration = time.monotonic() - wall_started
        simulation_duration = max(0.0, self._sim_now() - simulation_started)
        segment = self._active_segment_diagnostics.get(
            'segment', profile.segments[0].name)
        active = next(
            (item for item in profile.segments if item.name == segment),
            profile.segments[0],
        )
        progress = command_progress(
            active.command, initial_gt, final_gt, active.target)
        gt_delta = relative_pose(initial_gt, final_gt)
        raw_delta = relative_pose(initial_odom, final_odom)
        average_speed = (
            math.hypot(gt_delta.x, gt_delta.y) / simulation_duration
            if simulation_duration > 0.0 else 0.0
        )
        stamps = {
            'ground_truth': final_gt.stamp,
            'raw_odometry': final_odom.stamp,
            'filtered_odometry': (
                final_filtered.stamp if final_filtered is not None else None),
            'command': self.last_command_stamp,
        }
        valid_stamps = [
            value for value in stamps.values() if value is not None
        ]
        now_sim = self._sim_now()
        result = {
            'timestamp': datetime.now(timezone.utc).isoformat(),
            'profile': profile.name,
            'repetition': repetition,
            'completion_status': (
                'timeout' if (
                    'timeout' in reason or 'watchdog' in reason
                    or 'clock stalled' in reason
                ) else 'failed'
            ),
            'timeout_status': (
                'timeout' in reason or 'watchdog' in reason
                or 'clock stalled' in reason
            ),
            'reason': reason,
            'wall_duration': wall_duration,
            'simulation_duration': simulation_duration,
            'duration': simulation_duration,
            'timeout_basis': (
                'simulation time with independent wall watchdog'),
            'segment': segment,
            'sample_count': len(samples),
            'last_command': self.last_command,
            'last_command_stamp': self.last_command_stamp,
            'initial_ground_truth_pose': initial_gt.to_dict(),
            'last_ground_truth_pose': final_gt.to_dict(),
            'ground_truth_displacement': gt_delta.to_dict(),
            'initial_raw_odometry': initial_odom.to_dict(),
            'last_raw_odometry': final_odom.to_dict(),
            'raw_displacement': raw_delta.to_dict(),
            'last_message_timestamps': stamps,
            'message_ages': {
                name: (now_sim - stamp if stamp is not None else None)
                for name, stamp in stamps.items()
            },
            'maximum_source_skew': (
                max(valid_stamps) - min(valid_stamps)
                if valid_stamps else None
            ),
            'target_displacement_or_rotation': active.target,
            **progress,
            'average_ground_truth_speed': average_speed,
            'recent_ground_truth_speed': self.ground_truth_speed,
            'recent_raw_odometry_speed': self.raw_odometry_speed,
            'command_diagnostics': dict(self.command_diagnostics),
            'segment_diagnostics': dict(self._active_segment_diagnostics),
            'command': profile.segments[0].command.__dict__,
        }
        if initial_filtered is not None and final_filtered is not None:
            result.update({
                'initial_filtered_odometry': initial_filtered.to_dict(),
                'last_filtered_odometry': final_filtered.to_dict(),
                'filtered_displacement': relative_pose(
                    initial_filtered, final_filtered).to_dict(),
            })
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

    def _repository_revision(self, revision='HEAD'):
        """Resolve a revision in the repository owning the output directory."""
        output = self.output_dir.resolve()
        for directory in (output, *output.parents):
            if (directory / '.git').exists():
                return self._version(
                    ['git', '-C', str(directory), 'rev-parse', revision])
        return 'unavailable'

    def _repository_commit(self):
        return self._repository_revision('HEAD')

    def run_suite(self):
        """Run selected profiles, preserving failures in the final report."""
        self.wait_until_ready()
        if self.profile_sequence_names:
            selected = [
                self.profiles[name] for name in self.profile_sequence_names]
        else:
            selected = list(
                self.profiles.values()
                if self.selected_profile == 'all'
                else (self.profiles[self.selected_profile],)
            )
        selected = ordered_profiles(
            selected, self.profile_order, self.random_seed)
        actual_execution_order = [profile.name for profile in selected]
        runs = []
        run_counters = {
            profile.name: self.repetition_offset for profile in selected}
        try:
            for profile in selected:
                repetitions = (
                    self.repetitions_override
                    if self.repetitions_override > 0
                    else profile.repetitions
                )
                for _ in range(repetitions):
                    repetition = run_counters.get(profile.name, 0) + 1
                    run_counters[profile.name] = repetition
                    self.get_logger().info(
                        f'Running {profile.name}, repetition {repetition}/'
                        f'{self.repetition_offset + repetitions}'
                    )
                    run_started = time.monotonic()
                    run_started_at = datetime.now(timezone.utc).isoformat()
                    try:
                        runs.append(self.execute_run(profile, repetition))
                    except EvaluationError as error:
                        self.publish_command()
                        runs.append({
                            'timestamp': (
                                datetime.now(timezone.utc).isoformat()
                            ),
                            'started_at': run_started_at,
                            'finished_at': (
                                datetime.now(timezone.utc).isoformat()
                            ),
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
                        failed = runs[-1]
                        result_path = (
                            self.output_dir / 'run_results'
                            / f'{profile.name}_run_{repetition:02d}.json'
                        )
                        failed['run_result_file'] = str(result_path)
                        write_run_result(result_path, failed)
                        self.get_logger().error(str(error))
        finally:
            if rclpy.ok():
                try:
                    self.safe_stop()
                except (EvaluationError, RuntimeError) as error:
                    self.get_logger().error(f'Final safe stop failed: {error}')

        configuration = {
            'config_file': self.config_file,
            'selected_profile': self.selected_profile,
            'profile_sequence': self.profile_sequence_names,
            'repetitions_override': self.repetitions_override,
            'repetition_offset': self.repetition_offset,
            'sample_rate': self.sample_rate,
            'evaluation_mode': self.evaluation_mode,
            'timeout_basis': (
                'simulation time with independent wall watchdog'),
            'wall_watchdog_factor': self.wall_watchdog_factor,
            'minimum_wall_watchdog': self.minimum_wall_watchdog,
            'clock_stall_timeout': self.clock_stall_timeout,
            'motion_start_timeout': self.motion_start_timeout,
            'profile_order': self.profile_order,
            'random_seed': self.random_seed,
            'actual_execution_order': actual_execution_order,
        }
        metadata = {
            'git_commit': self._repository_commit(),
            'diagnostic_baseline_commit': self._repository_revision('main'),
            'evaluation_mode': self.evaluation_mode,
            'localization_enabled': self.localization_enabled,
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
            'model_state_reset_service': None,
            'actual_execution_order': actual_execution_order,
            'reset_behavior': (
                'continuous stop, explicit Gazebo pose placement, fresh '
                'ground-truth/raw/filtered samples, clock advancement, and '
                'continuous settle; joint, controller-odometry, and EKF '
                'state are not reset, so multi-profile campaigns use the '
                'fresh-process campaign runner'
            ),
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
            except (KeyboardInterrupt, RuntimeError):
                pass
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)
