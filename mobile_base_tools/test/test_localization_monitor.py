import math

from mobile_base_tools.localization_monitor import correction, relative_to


def test_relative_to_rotated_origin():
    origin = (1.0, 2.0, math.pi / 2)
    x, y, yaw = relative_to(origin, (1.0, 3.0, math.pi / 2))
    assert math.isclose(x, 1.0, abs_tol=1e-9)
    assert math.isclose(y, 0.0, abs_tol=1e-9)
    assert math.isclose(yaw, 0.0, abs_tol=1e-9)
    assert relative_to(origin, origin) == (0.0, 0.0, 0.0)


def test_correction_classes():
    assert correction((0, 0, 0), (0.01, 0, 0))[0] is None
    assert correction((0, 0, 0), (0.05, 0, 0))[0] == 'match'
    assert correction((0, 0, 0), (0, 0, math.radians(1)))[0] == 'match'
    assert correction((0, 0, 0), (0.2, 0, 0))[0] == 'closure'
    # Wrap-around is not a jump.
    assert correction((0, 0, math.pi - 1e-4), (0, 0, -math.pi + 1e-4))[0] is None
