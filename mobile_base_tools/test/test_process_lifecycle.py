import os
import signal
import subprocess
import sys
import time

from mobile_base_tools.evaluation_campaign import run_launch
from mobile_base_tools.process_lifecycle import (
    group_processes,
    process_snapshot,
    project_processes,
    RUN_TOKEN_ENV,
    terminate_process_group,
    token_processes,
)


def test_process_snapshot_contains_process_identity_and_resources():
    current = next(
        item for item in process_snapshot() if item['pid'] == os.getpid())
    assert current['ppid'] > 0
    assert current['pgid'] > 0
    assert current['sid'] > 0
    assert current['state'] != 'Z'
    assert current['elapsed_seconds'] >= 0.0
    assert current['rss_kib'] is not None


def test_project_process_detection_is_specific_to_runtime_commands():
    snapshot = [
        {'pid': 1, 'command': 'ruby harmless.rb'},
        {'pid': 2, 'command': 'ruby /opt/gz gz sim -r world.sdf'},
        {'pid': 3, 'command': '/opt/ros/jazzy/bin/robot_state_publisher'},
        {'pid': 4, 'command': 'python3 unrelated.py'},
    ]
    assert [item['pid'] for item in project_processes(snapshot)] == [2, 3]


def test_token_identifies_exact_owned_process():
    token = 'unit-test-owned-process'
    environment = os.environ.copy()
    environment[RUN_TOKEN_ENV] = token
    process = subprocess.Popen(['sleep', '60'], env=environment,
                               start_new_session=True)
    try:
        deadline = time.monotonic() + 2.0
        owned = []
        while time.monotonic() < deadline and not owned:
            owned = token_processes(token)
            time.sleep(0.02)
        assert [item['pid'] for item in owned] == [process.pid]
    finally:
        terminate_process_group(process.pid, (0.2, 0.2, 0.5))
        process.wait(timeout=2.0)


def test_process_group_cleanup_escalates_and_reaps_leader():
    child_code = (
        'import signal,time; '
        'signal.signal(signal.SIGINT, signal.SIG_IGN); '
        'signal.signal(signal.SIGTERM, signal.SIG_IGN); '
        'time.sleep(60)'
    )
    process = subprocess.Popen(
        [sys.executable, '-c', child_code], start_new_session=True)
    try:
        time.sleep(0.1)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not group_processes(process.pid):
            time.sleep(0.02)
        report = terminate_process_group(process.pid, (0.15, 0.15, 0.5))
        process.wait(timeout=2.0)
        assert report['clean']
        assert [action['signal'] for action in report['actions']] == [
            'SIGINT', 'SIGTERM', 'SIGKILL']
        assert process.returncode == -signal.SIGKILL
        assert group_processes(process.pid) == []
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2.0)


def test_normal_launch_exit_cleans_surviving_descendant(tmp_path, monkeypatch):
    monkeypatch.setattr(
        'mobile_base_tools.evaluation_campaign.wait_for_ros_baseline',
        lambda *args, **kwargs: {'clean': True, 'observations': []})
    child_code = (
        'import subprocess,time; '
        f'subprocess.Popen([{sys.executable!r}, "-c", "import time; time.sleep(60)"]); '
        'time.sleep(0.1)'
    )
    result = run_launch(
        [sys.executable, '-c', child_code], os.environ.copy(),
        tmp_path / 'launch.log')
    assert result['returncode'] == 0
    assert result['cleanup']['actions']
    assert result['cleanup']['clean']
    assert group_processes(result['pgid']) == []
