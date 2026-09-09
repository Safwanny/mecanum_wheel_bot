#!/usr/bin/env python3
"""Validate bounded timestamp, rate, gap, age, and finite-value contracts."""

import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
import yaml


TOPIC_TYPES = {
    'sensor_msgs/msg/Imu': (Imu, qos_profile_sensor_data),
    'sensor_msgs/msg/LaserScan': (LaserScan, qos_profile_sensor_data),
    'sensor_msgs/msg/PointCloud2': (PointCloud2, qos_profile_sensor_data),
    'nav_msgs/msg/Odometry': (Odometry, 10),
}


@dataclass(frozen=True)
class StreamContract:
    """Pass/fail thresholds for one operational stream."""

    name: str
    topic: str
    message_type: str
    expected_rate_hz: float
    minimum_rate_hz: float
    minimum_sample_count: int
    allow_duplicate_timestamps: bool
    maximum_gap_sec: float
    maximum_age_sec: float
    required_in: tuple[str, ...]


@dataclass
class StreamStatistics:
    """Collected validation state for one bounded topic stream."""

    sample_count: int = 0
    valid_timestamp_count: int = 0
    first_timestamp: float | None = None
    last_timestamp: float | None = None
    zero_timestamps: int = 0
    nonfinite_timestamps: int = 0
    nonmonotonic_timestamps: int = 0
    duplicate_timestamps: int = 0
    maximum_inter_message_gap: float = 0.0
    maximum_message_age: float = 0.0
    nonfinite_numeric_messages: int = 0

    def observe(self, timestamp, simulation_time, numeric_values):
        """Record one message without discarding invalid observations."""
        self.sample_count += 1
        if not math.isfinite(timestamp):
            self.nonfinite_timestamps += 1
        elif timestamp <= 0.0:
            self.zero_timestamps += 1
        else:
            self.valid_timestamp_count += 1
            if self.first_timestamp is None:
                self.first_timestamp = timestamp
            if self.last_timestamp is not None:
                gap = timestamp - self.last_timestamp
                if gap < 0.0:
                    self.nonmonotonic_timestamps += 1
                elif gap == 0.0:
                    self.duplicate_timestamps += 1
                else:
                    self.maximum_inter_message_gap = max(
                        self.maximum_inter_message_gap, gap)
            self.last_timestamp = timestamp
            if math.isfinite(simulation_time):
                self.maximum_message_age = max(
                    self.maximum_message_age,
                    max(0.0, simulation_time - timestamp),
                )
        if not all(math.isfinite(value) for value in numeric_values):
            self.nonfinite_numeric_messages += 1

    def measured_rate_hz(self):
        """Return the source-stamp rate over the bounded observation window."""
        if (
            self.valid_timestamp_count < 2
            or self.first_timestamp is None
            or self.last_timestamp is None
            or self.last_timestamp <= self.first_timestamp
        ):
            return 0.0
        return (
            (self.valid_timestamp_count - 1)
            / (self.last_timestamp - self.first_timestamp)
        )


def _positive_finite(value, field):
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f'{field} must be finite and positive')
    return result


def load_contracts(path, mode):
    """Load and validate configuration-driven stream contracts."""
    if mode not in ('raw', 'fused'):
        raise ValueError('mode must be raw or fused')
    document = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    streams = document.get('timestamp_contracts') if isinstance(document, dict) else None
    if not isinstance(streams, dict) or not streams:
        raise ValueError('configuration must contain timestamp_contracts')
    contracts = []
    for name, item in streams.items():
        required_in = tuple(item.get('required_in', ('raw', 'fused')))
        if not set(required_in) <= {'raw', 'fused'}:
            raise ValueError(f'{name}.required_in contains an invalid mode')
        if mode not in required_in:
            continue
        message_type = str(item.get('type', ''))
        if message_type not in TOPIC_TYPES:
            raise ValueError(f'{name}.type is unsupported: {message_type}')
        minimum_sample_count = int(item.get('minimum_sample_count', 0))
        if minimum_sample_count <= 0:
            raise ValueError(f'{name}.minimum_sample_count must be positive')
        expected = _positive_finite(
            item.get('expected_rate_hz'), f'{name}.expected_rate_hz')
        minimum = _positive_finite(
            item.get('minimum_rate_hz'), f'{name}.minimum_rate_hz')
        if minimum > expected:
            raise ValueError(
                f'{name}.minimum_rate_hz must not exceed expected_rate_hz')
        contracts.append(StreamContract(
            name=name,
            topic=str(item.get('topic', '')),
            message_type=message_type,
            expected_rate_hz=expected,
            minimum_rate_hz=minimum,
            minimum_sample_count=minimum_sample_count,
            allow_duplicate_timestamps=bool(
                item.get('allow_duplicate_timestamps', False)),
            maximum_gap_sec=_positive_finite(
                item.get('maximum_gap_sec'), f'{name}.maximum_gap_sec'),
            maximum_age_sec=_positive_finite(
                item.get('maximum_age_sec'), f'{name}.maximum_age_sec'),
            required_in=required_in,
        ))
    if not contracts:
        raise ValueError(f'configuration defines no streams for {mode} mode')
    if any(not contract.topic for contract in contracts):
        raise ValueError('contract topics must not be empty')
    if len({contract.topic for contract in contracts}) != len(contracts):
        raise ValueError('contract topics must be unique')
    return contracts


def evaluate_stream(contract, statistics):
    """Return a complete machine-readable contract evaluation."""
    measured_rate = statistics.measured_rate_hz()
    checks = {
        'minimum_sample_count': (
            statistics.sample_count >= contract.minimum_sample_count),
        'nonzero_timestamps': statistics.zero_timestamps == 0,
        'finite_timestamps': statistics.nonfinite_timestamps == 0,
        'monotonic_timestamps': statistics.nonmonotonic_timestamps == 0,
        'duplicate_timestamp_policy': (
            contract.allow_duplicate_timestamps
            or statistics.duplicate_timestamps == 0),
        'minimum_rate': measured_rate >= contract.minimum_rate_hz,
        'maximum_gap': (
            statistics.maximum_inter_message_gap
            <= contract.maximum_gap_sec),
        'maximum_age': (
            statistics.maximum_message_age <= contract.maximum_age_sec),
        'finite_numeric_values': (
            statistics.nonfinite_numeric_messages == 0),
    }
    reasons = [name.replace('_', ' ') for name, passed in checks.items() if not passed]
    return {
        'passed': all(checks.values()),
        'failure_reasons': reasons,
        'checks': checks,
        'contract': asdict(contract),
        'statistics': {
            **asdict(statistics),
            'measured_rate_hz': measured_rate,
        },
    }


def _stamp_seconds(message):
    stamp = message.header.stamp
    return stamp.sec + stamp.nanosec / 1e9


def _numeric_values(message):
    if isinstance(message, LaserScan):
        return (
            message.angle_min, message.angle_max, message.angle_increment,
            message.range_min, message.range_max,
        )
    if isinstance(message, PointCloud2):
        # Only the cloud layout is checked. The payload of a ToF cloud is
        # expected to carry non-finite values - those are the zones that saw
        # nothing, which is exactly what a drop-off looks like - so running a
        # finiteness check over the points would fail on correct data.
        return (
            float(message.height), float(message.width),
            float(message.point_step), float(message.row_step),
        )
    if isinstance(message, Imu):
        return (
            message.angular_velocity.x, message.angular_velocity.y,
            message.angular_velocity.z, message.linear_acceleration.x,
            message.linear_acceleration.y, message.linear_acceleration.z,
            *message.orientation_covariance,
            *message.angular_velocity_covariance,
            *message.linear_acceleration_covariance,
        )
    return (
        message.pose.pose.position.x, message.pose.pose.position.y,
        message.pose.pose.position.z, message.pose.pose.orientation.x,
        message.pose.pose.orientation.y, message.pose.pose.orientation.z,
        message.pose.pose.orientation.w, message.twist.twist.linear.x,
        message.twist.twist.linear.y, message.twist.twist.angular.z,
        *message.pose.covariance, *message.twist.covariance,
    )


class TimestampValidator(Node):
    """Subscribe to configured Phase 1 streams for a bounded interval."""

    def __init__(self, contracts):
        super().__init__('timestamp_validator')
        self.contracts = contracts
        self.statistics = {
            contract.topic: StreamStatistics() for contract in contracts
        }
        self._topic_subscriptions = []
        for contract in contracts:
            message_type, qos = TOPIC_TYPES[contract.message_type]
            self._topic_subscriptions.append(self.create_subscription(
                message_type,
                contract.topic,
                lambda message, name=contract.topic: self._observe(name, message),
                qos,
            ))

    def _observe(self, topic, message):
        now = self.get_clock().now().nanoseconds / 1e9
        self.statistics[topic].observe(
            _stamp_seconds(message), now, _numeric_values(message))


def build_report(contracts, statistics, duration, mode, use_sim_time):
    """Build the complete timestamp contract report."""
    streams = {
        contract.name: evaluate_stream(
            contract, statistics[contract.topic])
        for contract in contracts
    }
    return {
        'schema_version': 2,
        'duration_seconds': duration,
        'mode': mode,
        'use_sim_time': use_sim_time,
        'passed': all(stream['passed'] for stream in streams.values()),
        'streams': streams,
    }


def validate(duration, output_path, config_path, mode):
    """Run the bounded validator and write JSON plus a concise summary."""
    contracts = load_contracts(config_path, mode)
    rclpy.init()
    node = TimestampValidator(contracts)
    started = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - started < duration:
            rclpy.spin_once(node, timeout_sec=0.1)
        report = build_report(
            contracts, node.statistics, duration, mode,
            node.get_parameter('use_sim_time').value,
        )
        path = Path(output_path).expanduser().resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(report, indent=2, sort_keys=True) + '\n',
            encoding='utf-8',
        )
        for name, stream in report['streams'].items():
            status = 'PASS' if stream['passed'] else 'FAIL'
            reasons = ', '.join(stream['failure_reasons']) or 'all checks passed'
            rate = stream['statistics']['measured_rate_hz']
            print(f'{status} {name}: {rate:.2f} Hz; {reasons}')
        print('PASS' if report['passed'] else 'FAIL')
        return report['passed']
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    default_config = str(
        Path(get_package_share_directory('mobile_base_tools'))
        / 'config' / 'timestamp_contracts.yaml')
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--output', required=True)
    parser.add_argument('--config', default=default_config)
    parser.add_argument('--mode', choices=('raw', 'fused'), default='fused')
    arguments, _ = parser.parse_known_args()
    if not math.isfinite(arguments.duration) or arguments.duration <= 0.0:
        parser.error('--duration must be finite and positive')
    try:
        passed = validate(
            arguments.duration, arguments.output, arguments.config,
            arguments.mode)
    except (OSError, ValueError, yaml.YAMLError) as error:
        parser.error(str(error))
    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main()
