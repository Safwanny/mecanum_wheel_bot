import math
from pathlib import Path

from mobile_base_tools.timestamp_validator import (
    evaluate_stream,
    load_contracts,
    StreamContract,
    StreamStatistics,
)
import pytest


CONFIG = Path(__file__).parents[1] / 'config' / 'timestamp_contracts.yaml'


def contract(**overrides):
    values = {
        'name': 'test',
        'topic': '/test',
        'message_type': 'nav_msgs/msg/Odometry',
        'expected_rate_hz': 10.0,
        'minimum_rate_hz': 8.0,
        'minimum_sample_count': 5,
        'allow_duplicate_timestamps': False,
        'maximum_gap_sec': 0.2,
        'maximum_age_sec': 0.1,
        'required_in': ('raw', 'fused'),
    }
    values.update(overrides)
    return StreamContract(**values)


def valid_statistics():
    statistics = StreamStatistics()
    for index in range(5):
        timestamp = 1.0 + index * 0.1
        statistics.observe(timestamp, timestamp + 0.02, [0.0, 1.0])
    return statistics


def test_config_selects_raw_and_fused_streams():
    raw = load_contracts(CONFIG, 'raw')
    fused = load_contracts(CONFIG, 'fused')
    assert {item.name for item in raw} == {'imu', 'scan', 'raw_odometry'}
    assert {item.name for item in fused} == {
        'imu', 'scan', 'raw_odometry', 'filtered_odometry'}


def test_valid_stream_passes_all_contract_checks():
    result = evaluate_stream(contract(), valid_statistics())
    assert result['passed']
    assert result['statistics']['measured_rate_hz'] == pytest.approx(10.0)
    assert not result['failure_reasons']


@pytest.mark.parametrize(
    ('mutate', 'reason'),
    [
        (lambda value: value.observe(0.0, 1.0, [0.0]),
         'nonzero timestamps'),
        (lambda value: value.observe(math.nan, 1.0, [0.0]),
         'finite timestamps'),
        (lambda value: value.observe(0.5, 1.0, [0.0]),
         'monotonic timestamps'),
        (lambda value: value.observe(1.4, 1.4, [0.0]),
         'duplicate timestamp policy'),
        (lambda value: value.observe(2.0, 2.0, [0.0]),
         'maximum gap'),
        (lambda value: value.observe(1.5, 2.0, [0.0]),
         'maximum age'),
        (lambda value: value.observe(1.5, 1.5, [math.nan]),
         'finite numeric values'),
    ],
)
def test_invalid_streams_report_specific_failure(mutate, reason):
    statistics = valid_statistics()
    mutate(statistics)
    result = evaluate_stream(contract(), statistics)
    assert not result['passed']
    assert reason in result['failure_reasons']


def test_insufficient_rate_fails():
    statistics = StreamStatistics()
    for timestamp in (1.0, 1.5, 2.0, 2.5, 3.0):
        statistics.observe(timestamp, timestamp, [0.0])
    result = evaluate_stream(contract(maximum_gap_sec=1.0), statistics)
    assert not result['passed']
    assert 'minimum rate' in result['failure_reasons']


def test_no_samples_fails_count_and_rate():
    result = evaluate_stream(contract(), StreamStatistics())
    assert not result['passed']
    assert 'minimum sample count' in result['failure_reasons']
    assert 'minimum rate' in result['failure_reasons']
