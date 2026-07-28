import math

from mobile_base_tools.mecanum_motion_test import build_sequence
import pytest


def test_segment_order():
    names = [segment.name for segment in build_sequence()]
    assert names == [
        'settle', 'square forward', 'square left', 'square backward',
        'square right', 'square stop', 'diamond forward-left',
        'diamond backward-left', 'diamond backward-right',
        'diamond forward-right', 'diamond stop', 'rotate +90',
        'rotate -180', 'rotate +90 return', 'final stop',
    ]


def test_duration_calculations():
    sequence = build_sequence(speed=0.2, side_length=1.0,
                              angular_speed=0.5)
    assert sequence[1].duration == pytest.approx(5.0)
    assert sequence[11].duration == pytest.approx(math.pi)
    assert sequence[12].duration == pytest.approx(2.0 * math.pi)


def test_diagonal_speed_is_normalized():
    diagonal = build_sequence()[6]
    magnitude = math.hypot(diagonal.linear_x, diagonal.linear_y)
    assert magnitude == pytest.approx(0.15)


def test_net_commanded_translation_is_zero():
    sequence = build_sequence()
    net_x = sum(item.linear_x * item.duration for item in sequence)
    net_y = sum(item.linear_y * item.duration for item in sequence)
    assert net_x == pytest.approx(0.0)
    assert net_y == pytest.approx(0.0)


def test_net_commanded_rotation_is_zero():
    sequence = build_sequence()
    net_yaw = sum(item.angular_z * item.duration for item in sequence)
    assert net_yaw == pytest.approx(0.0)


def test_final_command_is_zero():
    final = build_sequence()[-1]
    assert (final.linear_x, final.linear_y, final.angular_z) == (0.0, 0.0, 0.0)


@pytest.mark.parametrize(
    'parameter',
    ['speed', 'side_length', 'angular_speed', 'settle_duration',
     'pause_duration', 'final_stop_duration'],
)
@pytest.mark.parametrize('value', [0.0, -1.0])
def test_invalid_parameters_are_rejected(parameter, value):
    arguments = {parameter: value}
    with pytest.raises(ValueError):
        build_sequence(**arguments)
