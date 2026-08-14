#!/usr/bin/env python3
"""Bounded deterministic validation of the mobile-base Phase 1 TF contract."""

import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import time

from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from tf2_msgs.msg import TFMessage


ODOM_EDGE = ('odom', 'base_footprint')
REQUIRED_EDGES = {('base_footprint', 'base_link')}
REQUIRED_CONNECTED_FRAMES = {
    'base_link',
    'front_left_wheel_link',
    'front_right_wheel_link',
    'rear_right_wheel_link',
    'rear_left_wheel_link',
    'imu_link',
    'lidar_link',
    'camera_link',
    'camera_optical_frame',
}
REQUIRED_DYNAMIC_EDGES = {
    ODOM_EDGE,
    ('base_link', 'front_left_wheel_link'),
    ('base_link', 'front_right_wheel_link'),
    ('base_link', 'rear_right_wheel_link'),
    ('base_link', 'rear_left_wheel_link'),
}


@dataclass
class TransformStatistics:
    """Observed state for one parent-child transform."""

    sample_count: int = 0
    first_timestamp: float | None = None
    last_timestamp: float | None = None
    zero_timestamps: int = 0
    nonmonotonic_timestamps: int = 0
    duplicate_timestamps: int = 0
    invalid_numeric_samples: int = 0

    def observe(self, transform, dynamic):
        """Record one transform and its source stamp."""
        values = (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        )
        quaternion_norm = math.sqrt(sum(value * value for value in values[3:]))
        if (
            not all(math.isfinite(value) for value in values)
            or quaternion_norm <= 1e-12
        ):
            self.invalid_numeric_samples += 1
        self.sample_count += 1
        if not dynamic:
            return
        stamp = transform.header.stamp
        timestamp = stamp.sec + stamp.nanosec / 1e9
        if timestamp <= 0.0 or not 0 <= stamp.nanosec < 1_000_000_000:
            self.zero_timestamps += 1
            return
        if self.first_timestamp is None:
            self.first_timestamp = timestamp
        if self.last_timestamp is not None:
            if timestamp < self.last_timestamp:
                self.nonmonotonic_timestamps += 1
            elif timestamp == self.last_timestamp:
                self.duplicate_timestamps += 1
        self.last_timestamp = timestamp


def _edge_name(edge):
    return f'{edge[0]} -> {edge[1]}'


def _reachable(edges, start):
    children = {}
    for parent, child in edges:
        children.setdefault(parent, set()).add(child)
    visited = {start}
    pending = [start]
    while pending:
        for child in children.get(pending.pop(), ()):
            if child not in visited:
                visited.add(child)
                pending.append(child)
    return visited


def graph_failures(edges):
    """Return loop and multiple-parent failures for a TF edge set."""
    failures = []
    parents = {}
    for parent, child in edges:
        parents.setdefault(child, set()).add(parent)
    for child, values in sorted(parents.items()):
        if len(values) > 1:
            failures.append(
                f'{child} has multiple parents: {", ".join(sorted(values))}')

    children = {}
    for parent, child in edges:
        children.setdefault(parent, set()).add(child)
    visiting = set()
    visited = set()

    def visit(frame):
        if frame in visiting:
            return True
        if frame in visited:
            return False
        visiting.add(frame)
        if any(visit(child) for child in children.get(frame, ())):
            return True
        visiting.remove(frame)
        visited.add(frame)
        return False

    if any(visit(frame) for frame in set(children) | set(parents)):
        failures.append('TF graph contains a loop')
    return failures


def evaluate_contract(dynamic, static, mode, authority):
    """Evaluate topology, update, timestamp, contamination, and authority rules."""
    dynamic_edges = set(dynamic)
    static_edges = set(static)
    edges = dynamic_edges | static_edges
    failures = graph_failures(edges)

    for edge in sorted(REQUIRED_EDGES):
        if edge not in edges:
            failures.append(f'missing required transform {_edge_name(edge)}')
    if ODOM_EDGE not in dynamic_edges:
        failures.append('odom -> base_footprint is missing or not dynamic')

    connected = _reachable(edges, 'base_footprint')
    for frame in sorted(REQUIRED_CONNECTED_FRAMES - connected):
        failures.append(f'{frame} is not connected below base_footprint')

    if ('map', 'odom') in edges:
        failures.append('map -> odom is forbidden during Phase 1')
    contaminated = sorted({
        frame for edge in edges for frame in edge
        if any(token in frame.lower() for token in (
            'ground_truth', 'gazebo', 'world'))
    })
    if contaminated:
        failures.append(
            'evaluation-only frames contaminate TF: '
            + ', '.join(contaminated))

    for edge in sorted(REQUIRED_DYNAMIC_EDGES):
        statistics = dynamic.get(edge)
        if statistics is None:
            continue
        if (
            statistics.sample_count < 2
            or statistics.first_timestamp is None
            or statistics.last_timestamp is None
            or statistics.last_timestamp <= statistics.first_timestamp
        ):
            failures.append(f'{_edge_name(edge)} did not update over time')
    for edge, statistics in sorted(dynamic.items()):
        if statistics.zero_timestamps:
            failures.append(f'{_edge_name(edge)} has zero/invalid timestamps')
        if statistics.nonmonotonic_timestamps:
            failures.append(f'{_edge_name(edge)} has backwards timestamps')
        if statistics.invalid_numeric_samples:
            failures.append(f'{_edge_name(edge)} has invalid numeric values')
    for edge, statistics in sorted(static.items()):
        if statistics.invalid_numeric_samples:
            failures.append(f'{_edge_name(edge)} has invalid numeric values')

    controller_enabled = authority.get('controller_enable_odom_tf')
    ekf_enabled = authority.get('ekf_publish_tf')
    publishers = set(authority.get('dynamic_publishers', ()))
    publisher_basenames = {name.rsplit('/', 1)[-1] for name in publishers}
    # ros2_control creates its /tf publisher endpoint while inactive for odom
    # TF. Endpoint presence is therefore supporting graph evidence, not proof
    # of authority; the controller/EKF publication parameters are authoritative.
    allowed = {
        'robot_state_publisher', 'mobile_base_controller',
        'controller_manager', 'ekf_filter_node',
    }
    if mode == 'fused':
        if controller_enabled is not False:
            failures.append('controller enable_odom_tf is not false in fused mode')
        if ekf_enabled is not True:
            failures.append('EKF publish_tf is not true in fused mode')
        if 'ekf_filter_node' not in publisher_basenames:
            failures.append('EKF is not an observed /tf publisher')
        expected_publisher_observed = 'ekf_filter_node' in publisher_basenames
    elif mode == 'raw':
        if controller_enabled is not True:
            failures.append('controller enable_odom_tf is not true in raw mode')
        if ekf_enabled not in (None, False):
            failures.append('EKF must not publish TF in raw mode')
        if not ({'mobile_base_controller', 'controller_manager'} & publisher_basenames):
            failures.append('controller is not an observed /tf publisher')
        expected_publisher_observed = bool(
            {'mobile_base_controller', 'controller_manager'}
            & publisher_basenames)
    else:
        raise ValueError('mode must be raw or fused')

    unexpected = sorted(publisher_basenames - allowed)
    if unexpected:
        failures.append('unexpected /tf publishers: ' + ', '.join(unexpected))
    if 'robot_state_publisher' not in publisher_basenames:
        failures.append('robot_state_publisher is not an observed /tf publisher')
    enabled_owners = int(controller_enabled is True) + int(ekf_enabled is True)
    if enabled_owners != 1:
        failures.append(
            'odom -> base_footprint does not have exactly one enabled owner')

    checks = {
        'odom_transform_exists': ODOM_EDGE in dynamic_edges,
        'required_frames_connected': REQUIRED_CONNECTED_FRAMES <= connected,
        'no_map_to_odom': ('map', 'odom') not in edges,
        'no_ground_truth_contamination': not contaminated,
        'no_graph_failures': not graph_failures(edges),
        'dynamic_transforms_advance': all(
            edge in dynamic
            and dynamic[edge].sample_count >= 2
            and dynamic[edge].first_timestamp is not None
            and dynamic[edge].last_timestamp is not None
            and dynamic[edge].last_timestamp > dynamic[edge].first_timestamp
            for edge in REQUIRED_DYNAMIC_EDGES
        ),
        'single_expected_authority': (
            enabled_owners == 1
            and expected_publisher_observed
            and not unexpected
        ),
    }
    return {
        'passed': not failures,
        'failure_reasons': failures,
        'checks': checks,
        'connected_frames': sorted(connected),
    }


class TfValidator(Node):
    """Collect dynamic/static TF messages and graph publisher evidence."""

    def __init__(self):
        super().__init__('tf_validator')
        self.dynamic = {}
        self.static = {}
        dynamic_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        static_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.dynamic_subscription = self.create_subscription(
            TFMessage, '/tf',
            lambda message: self._observe(message, self.dynamic, True),
            dynamic_qos,
        )
        self.static_subscription = self.create_subscription(
            TFMessage, '/tf_static',
            lambda message: self._observe(message, self.static, False),
            static_qos,
        )

    @staticmethod
    def _observe(message, collection, dynamic):
        for transform in message.transforms:
            edge = (transform.header.frame_id, transform.child_frame_id)
            collection.setdefault(edge, TransformStatistics()).observe(
                transform, dynamic)

    def publisher_names(self, topic):
        """Return fully qualified publisher node names for a topic."""
        names = []
        for endpoint in self.get_publishers_info_by_topic(topic):
            namespace = endpoint.node_namespace.rstrip('/')
            names.append(
                f'{namespace}/{endpoint.node_name}'
                if namespace else f'/{endpoint.node_name}')
        return sorted(set(names))

    def bool_parameter(self, node_name, parameter, timeout=5.0):
        """Read one boolean parameter, returning None when the node is absent."""
        service = node_name.rstrip('/') + '/get_parameters'
        client = self.create_client(GetParameters, service)
        deadline = time.monotonic() + timeout
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                if client.service_is_ready():
                    break
                rclpy.spin_once(self, timeout_sec=0.05)
            else:
                return None
            request = GetParameters.Request()
            request.names = [parameter]
            future = client.call_async(request)
            while rclpy.ok() and time.monotonic() < deadline and not future.done():
                rclpy.spin_once(self, timeout_sec=0.05)
            if not future.done() or future.result() is None:
                return None
            values = future.result().values
            return values[0].bool_value if values else None
        finally:
            self.destroy_client(client)


def validate(duration, output_path, mode):
    """Collect a bounded sample and write JSON plus a human-readable summary."""
    rclpy.init()
    node = TfValidator()
    started = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - started < duration:
            rclpy.spin_once(node, timeout_sec=0.1)
        authority = {
            'controller_enable_odom_tf': node.bool_parameter(
                '/mobile_base_controller', 'enable_odom_tf'),
            'ekf_publish_tf': node.bool_parameter(
                '/ekf_filter_node', 'publish_tf'),
            'dynamic_publishers': node.publisher_names('/tf'),
            'static_publishers': node.publisher_names('/tf_static'),
        }
        result = evaluate_contract(node.dynamic, node.static, mode, authority)
        report = {
            'schema_version': 1,
            'duration_seconds': duration,
            'mode': mode,
            'use_sim_time': node.get_parameter('use_sim_time').value,
            'passed': result['passed'],
            'failure_reasons': result['failure_reasons'],
            'checks': result['checks'],
            'connected_frames': result['connected_frames'],
            'authority': authority,
            'dynamic_transforms': {
                _edge_name(edge): asdict(statistics)
                for edge, statistics in sorted(node.dynamic.items())
            },
            'static_transforms': {
                _edge_name(edge): asdict(statistics)
                for edge, statistics in sorted(node.static.items())
            },
        }
        path = Path(output_path).expanduser().resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(report, indent=2, sort_keys=True) + '\n',
            encoding='utf-8',
        )
        print(('PASS' if report['passed'] else 'FAIL') + f' TF contract ({mode})')
        for reason in report['failure_reasons']:
            print('- ' + reason)
        return report['passed']
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--output', required=True)
    parser.add_argument('--mode', choices=('raw', 'fused'), default='fused')
    arguments, _ = parser.parse_known_args()
    if not math.isfinite(arguments.duration) or arguments.duration <= 0.0:
        parser.error('--duration must be finite and positive')
    raise SystemExit(
        0 if validate(arguments.duration, arguments.output, arguments.mode) else 1)


if __name__ == '__main__':
    main()
