"""Load and validate YAML-defined odometry motion profiles."""

from dataclasses import dataclass
import math
from pathlib import Path

import yaml


REQUIRED_PROFILES = {
    'forward_1m',
    'backward_1m',
    'strafe_left_1m',
    'strafe_right_1m',
    'rotate_positive_90deg',
    'rotate_negative_90deg',
    'diagonal_forward_left',
    'diagonal_forward_right',
    'diagonal_backward_left',
    'diagonal_backward_right',
    'square_1m',
}
KINDS = {'longitudinal', 'lateral', 'diagonal', 'rotation', 'square'}
TERMINATIONS = {'ground_truth_translation', 'ground_truth_rotation'}


@dataclass(frozen=True)
class Command:
    """Constant body-frame velocity command."""

    linear_x: float
    linear_y: float
    angular_z: float


@dataclass(frozen=True)
class Segment:
    """One motion segment and its ground-truth termination."""

    name: str
    command: Command
    termination_type: str
    target: float
    timeout: float


@dataclass(frozen=True)
class MotionProfile:
    """Validated evaluation profile."""

    name: str
    kind: str
    segments: tuple
    settle_before: float
    settle_after: float
    repetitions: int
    profile_timeout: float


def _finite_positive(value, field):
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f'{field} must be finite and greater than zero')
    return value


def _command(data, field):
    required = {'linear_x', 'linear_y', 'angular_z'}
    if set(data) != required:
        raise ValueError(f'{field} must contain exactly {sorted(required)}')
    values = tuple(float(data[name]) for name in required)
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f'{field} contains a non-finite velocity')
    return Command(
        linear_x=float(data['linear_x']),
        linear_y=float(data['linear_y']),
        angular_z=float(data['angular_z']),
    )


def _segment(name, data, default_timeout):
    termination = data.get('termination', {})
    termination_type = termination.get('type')
    if termination_type not in TERMINATIONS:
        raise ValueError(f'{name} has invalid termination type')
    return Segment(
        name=name,
        command=_command(data.get('command', {}), name + '.command'),
        termination_type=termination_type,
        target=_finite_positive(termination.get('target'), name + '.target'),
        timeout=_finite_positive(
            data.get('timeout', default_timeout), name + '.timeout'
        ),
    )


def load_profiles(path):
    """Load all validated profiles from a YAML file."""
    document = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(document, dict) or not isinstance(
            document.get('profiles'), dict):
        raise ValueError('configuration must contain a profiles mapping')
    profiles = {}
    for name, data in document['profiles'].items():
        if data.get('kind') not in KINDS:
            raise ValueError(f'{name} has invalid kind')
        repetitions = int(data.get('repetitions', 0))
        if repetitions <= 0:
            raise ValueError(f'{name}.repetitions must be positive')
        profile_timeout = _finite_positive(
            data.get('profile_timeout'), name + '.profile_timeout'
        )
        segment_timeout = _finite_positive(
            data.get('segment_timeout', profile_timeout),
            name + '.segment_timeout',
        )
        if 'segments' in data:
            raw_segments = data['segments']
            if data['kind'] != 'square' or len(raw_segments) != 4:
                raise ValueError(f'{name} must define exactly four segments')
            segments = tuple(
                _segment(
                    segment.get('name', f'segment_{index + 1}'),
                    segment,
                    segment_timeout,
                )
                for index, segment in enumerate(raw_segments)
            )
        else:
            segments = (_segment(name, data, segment_timeout),)
        profiles[name] = MotionProfile(
            name=name,
            kind=data['kind'],
            segments=segments,
            settle_before=_finite_positive(
                data.get('settle_before'), name + '.settle_before'
            ),
            settle_after=_finite_positive(
                data.get('settle_after'), name + '.settle_after'
            ),
            repetitions=repetitions,
            profile_timeout=profile_timeout,
        )
    missing = REQUIRED_PROFILES - set(profiles)
    if missing:
        raise ValueError('missing required profiles: ' + ', '.join(sorted(missing)))
    return profiles
