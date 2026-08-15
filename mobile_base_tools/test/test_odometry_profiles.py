from pathlib import Path

from mobile_base_tools.motion_profiles import load_profiles, REQUIRED_PROFILES
import pytest
import yaml


CONFIG = (
    Path(__file__).parents[1] / 'config' / 'odometry_tests.yaml'
)


def test_required_profiles_are_valid():
    profiles = load_profiles(CONFIG)
    assert REQUIRED_PROFILES <= set(profiles)
    assert {
        'diagnostic_low_forward', 'diagnostic_low_backward',
        'diagnostic_low_left', 'diagnostic_low_right',
        'diagnostic_low_rotate_positive',
        'diagnostic_low_rotate_negative',
        'diagnostic_low_forward_left',
        'diagnostic_low_forward_right',
        'diagnostic_low_backward_left',
        'diagnostic_low_backward_right',
    } <= set(profiles)
    assert len(profiles['square_1m'].segments) == 4
    assert all(profiles[name].repetitions == 5 for name in REQUIRED_PROFILES)
    assert all(
        profile.repetitions == 1
        for name, profile in profiles.items()
        if name.startswith('diagnostic_low_')
    )
    for profile in profiles.values():
        assert profile.settle_before > 0.0
        assert profile.settle_after > 0.0
        assert profile.profile_timeout > 0.0
        for segment in profile.segments:
            assert segment.timeout > 0.0
            assert segment.target > 0.0
            assert segment.timeout <= profile.profile_timeout


def test_invalid_velocity_is_rejected(tmp_path):
    document = yaml.safe_load(CONFIG.read_text())
    document['profiles']['forward_1m']['command']['linear_x'] = float('nan')
    path = tmp_path / 'invalid.yaml'
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError, match='non-finite velocity'):
        load_profiles(path)


def test_square_requires_four_segments(tmp_path):
    document = yaml.safe_load(CONFIG.read_text())
    document['profiles']['square_1m']['segments'].pop()
    path = tmp_path / 'invalid.yaml'
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError, match='exactly four segments'):
        load_profiles(path)
