"""Planar pose mathematics for raw odometry evaluation."""

from dataclasses import asdict, dataclass
import math


@dataclass(frozen=True)
class Pose2D:
    """Planar pose with an optional ROS timestamp."""

    x: float
    y: float
    yaw: float
    stamp: float = 0.0

    def to_dict(self):
        """Return a JSON-serializable representation."""
        return asdict(self)


def normalize_yaw(angle):
    """Normalize an angle to [-pi, pi]."""
    wrapped = (angle + math.pi) % (2.0 * math.pi) - math.pi
    if wrapped == -math.pi and angle > 0.0:
        return math.pi
    return wrapped


def quaternion_to_yaw(x, y, z, w):
    """Convert a finite normalized quaternion to planar yaw."""
    values = (x, y, z, w)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('quaternion contains a non-finite value')
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-12:
        raise ValueError('quaternion norm is zero')
    x, y, z, w = (value / norm for value in values)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def relative_pose(initial, final):
    """Express final-minus-initial displacement in the initial body frame."""
    dx_world = final.x - initial.x
    dy_world = final.y - initial.y
    cosine = math.cos(initial.yaw)
    sine = math.sin(initial.yaw)
    return Pose2D(
        x=cosine * dx_world + sine * dy_world,
        y=-sine * dx_world + cosine * dy_world,
        yaw=normalize_yaw(final.yaw - initial.yaw),
        stamp=final.stamp - initial.stamp,
    )


def path_length(poses):
    """Return accumulated XY path length."""
    return sum(
        math.hypot(current.x - previous.x, current.y - previous.y)
        for previous, current in zip(poses, poses[1:])
    )


def position_error(ground_truth_delta, odometry_delta):
    """Return component and Euclidean odometry position error."""
    error_x = odometry_delta.x - ground_truth_delta.x
    error_y = odometry_delta.y - ground_truth_delta.y
    return error_x, error_y, math.hypot(error_x, error_y)


def cross_axis_drift(profile_kind, ground_truth_delta):
    """Return unintended displacement for pure translation profiles."""
    if profile_kind == 'longitudinal':
        return abs(ground_truth_delta.y)
    if profile_kind == 'lateral':
        return abs(ground_truth_delta.x)
    if profile_kind in ('diagonal', 'rotation', 'square'):
        return 0.0
    raise ValueError(f'unsupported profile kind: {profile_kind}')


def diagonal_errors(command_x, command_y, ground_truth_delta):
    """Return along-track and cross-track error for a diagonal target."""
    magnitude = math.hypot(command_x, command_y)
    if magnitude <= 0.0:
        raise ValueError('diagonal command must have non-zero translation')
    unit_x = command_x / magnitude
    unit_y = command_y / magnitude
    along_track = (
        ground_truth_delta.x * unit_x + ground_truth_delta.y * unit_y
    )
    cross_track = (
        -ground_truth_delta.x * unit_y + ground_truth_delta.y * unit_x
    )
    return along_track, cross_track


def square_closure(initial, final):
    """Return final XY and yaw closure error for a square path."""
    delta = relative_pose(initial, final)
    return math.hypot(delta.x, delta.y), abs(delta.yaw)


def validate_monotonic_timestamp(timestamp, previous_timestamp=None):
    """Reject invalid, zero, or backward source timestamps."""
    if not math.isfinite(timestamp) or timestamp <= 0.0:
        raise ValueError('timestamp must be finite and positive')
    if (
        previous_timestamp is not None
        and timestamp < previous_timestamp
    ):
        raise ValueError('timestamp is nonmonotonic')


def data_is_stale(received_time, current_time, timeout):
    """Return whether a source has exceeded its freshness timeout."""
    if not math.isfinite(timeout) or timeout <= 0.0:
        raise ValueError('timeout must be finite and positive')
    return current_time - received_time > timeout
