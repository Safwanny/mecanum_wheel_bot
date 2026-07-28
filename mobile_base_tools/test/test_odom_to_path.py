import math

from mobile_base_tools.odom_to_path import Sample, TrajectoryAccumulator


def sample(stamp, x=0.0, y=0.0, yaw=0.0, frame='odom'):
    return Sample(stamp, frame, x, y, yaw, object())


def test_first_sample_is_accepted():
    accumulator = TrajectoryAccumulator()
    assert accumulator.add(sample(1))


def test_below_thresholds_is_rejected():
    accumulator = TrajectoryAccumulator()
    accumulator.add(sample(1))
    assert not accumulator.add(sample(2, x=0.01, yaw=0.02))


def test_distance_threshold_is_accepted():
    accumulator = TrajectoryAccumulator()
    accumulator.add(sample(1))
    assert accumulator.add(sample(2, x=0.02))


def test_pure_rotation_is_accepted():
    accumulator = TrajectoryAccumulator()
    accumulator.add(sample(1))
    assert accumulator.add(sample(2, yaw=0.051))


def test_yaw_wrap_is_small():
    accumulator = TrajectoryAccumulator(min_yaw=0.05)
    accumulator.add(sample(1, yaw=math.pi - 0.01))
    assert not accumulator.add(sample(2, yaw=-math.pi + 0.01))


def test_capacity_discards_oldest():
    accumulator = TrajectoryAccumulator(max_points=2, min_distance=0.0)
    first = sample(1)
    accumulator.add(first)
    accumulator.add(sample(2, x=1.0))
    accumulator.add(sample(3, x=2.0))
    assert len(accumulator.samples) == 2
    assert first not in accumulator.samples


def test_backwards_time_clears_path():
    accumulator = TrajectoryAccumulator(min_distance=0.0)
    accumulator.add(sample(2))
    accumulator.add(sample(1, x=1.0))
    assert [item.stamp_ns for item in accumulator.samples] == [1]


def test_frame_change_clears_path():
    accumulator = TrajectoryAccumulator(min_distance=0.0)
    accumulator.add(sample(1))
    accumulator.add(sample(2, frame='new_odom'))
    assert [item.frame_id for item in accumulator.samples] == ['new_odom']


def test_reset_clears_path():
    accumulator = TrajectoryAccumulator()
    accumulator.add(sample(1))
    accumulator.reset()
    assert not accumulator.samples
