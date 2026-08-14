"""Small, dependency-free process lifecycle helpers for evaluation campaigns."""

from datetime import datetime, timezone
import os
from pathlib import Path
import signal
import subprocess
import time


GPU_SAFETY_TEXT = (
    'dm_irq_work_func [amdgpu] hogged CPU for >10000us 4 times'
)
RUN_TOKEN_ENV = 'MOBILE_BASE_CAMPAIGN_RUN_ID'

_PROJECT_EXECUTABLES = {
    'controller_manager',
    'ekf_node',
    'ground_truth_selector',
    'gz',
    'gzclient',
    'gzserver',
    'odom_to_path',
    'odometry_test_runner',
    'parameter_bridge',
    'robot_state_publisher',
}


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def _read(path, binary=False):
    try:
        if binary:
            return path.read_bytes()
        return path.read_text(encoding='utf-8', errors='replace')
    except (OSError, PermissionError):
        return b'' if binary else ''


def process_snapshot():
    """Return a machine-readable /proc process snapshot."""
    processes = []
    page_kib = os.sysconf('SC_PAGE_SIZE') // 1024
    clock_ticks = os.sysconf('SC_CLK_TCK')
    try:
        uptime = float(_read(Path('/proc/uptime')).split()[0])
    except (IndexError, ValueError):
        uptime = 0.0
    resources = host_resources()
    memory_total = resources['memory_total_kib'] or 0
    for directory in Path('/proc').iterdir():
        if not directory.name.isdigit():
            continue
        stat = _read(directory / 'stat')
        end = stat.rfind(')')
        if end < 0:
            continue
        fields = stat[end + 2:].split()
        if len(fields) < 22:
            continue
        cmdline = _read(directory / 'cmdline', binary=True)
        command = cmdline.replace(b'\0', b' ').decode(
            'utf-8', errors='replace').strip()
        status = _read(directory / 'status')
        rss_kib = None
        for line in status.splitlines():
            if line.startswith('VmRSS:'):
                try:
                    rss_kib = int(line.split()[1])
                except (IndexError, ValueError):
                    pass
        try:
            elapsed = max(0.0, uptime - int(fields[19]) / clock_ticks)
            rss = int(fields[21]) * page_kib
            cpu_time = (int(fields[11]) + int(fields[12])) / clock_ticks
            processes.append({
                'pid': int(directory.name),
                'ppid': int(fields[1]),
                'pgid': int(fields[2]),
                'sid': int(fields[3]),
                'state': fields[0],
                'start_ticks': int(fields[19]),
                'elapsed_seconds': round(elapsed, 3),
                'cpu_percent_lifetime': (
                    round(100.0 * cpu_time / elapsed, 3) if elapsed else 0.0),
                'rss_kib': rss_kib,
                'memory_percent': (
                    round(100.0 * rss / memory_total, 5)
                    if memory_total else None),
                'command': command or stat[stat.find('(') + 1:end],
                'stat_rss_kib': rss,
            })
        except (IndexError, ValueError):
            continue
    return sorted(processes, key=lambda item: item['pid'])


def group_processes(pgid, snapshot=None):
    snapshot = process_snapshot() if snapshot is None else snapshot
    return [item for item in snapshot if item['pgid'] == pgid]


def token_processes(token, snapshot=None):
    """Find exact campaign-owned descendants by their inherited environment."""
    snapshot = process_snapshot() if snapshot is None else snapshot
    owned = []
    marker = f'{RUN_TOKEN_ENV}={token}'.encode()
    by_pid = {item['pid']: item for item in snapshot}
    for pid, item in by_pid.items():
        environ = _read(Path('/proc') / str(pid) / 'environ', binary=True)
        if marker in environ.split(b'\0'):
            owned.append(item)
    return owned


def project_processes(snapshot=None, excluded_pids=()):
    """Find likely mobile-base runtime processes without claiming ownership."""
    snapshot = process_snapshot() if snapshot is None else snapshot
    excluded = set(excluded_pids)
    result = []
    for item in snapshot:
        if item['pid'] in excluded:
            continue
        command = item['command']
        words = command.split()
        executable = Path(words[0]).name if words else ''
        is_gazebo_ruby = executable == 'ruby' and 'gz sim' in command
        is_mobile_launch = (
            executable == 'ros2'
            and ' launch mobile_base_bringup ' in f' {command} '
        )
        if executable in _PROJECT_EXECUTABLES or is_gazebo_ruby or is_mobile_launch:
            result.append(item)
    return result


def host_resources():
    memory = {}
    for line in _read(Path('/proc/meminfo')).splitlines():
        if ':' not in line:
            continue
        key, value = line.split(':', 1)
        try:
            memory[key] = int(value.split()[0])
        except (IndexError, ValueError):
            continue
    try:
        load = [float(value) for value in _read(Path('/proc/loadavg')).split()[:3]]
    except ValueError:
        load = []
    return {
        'timestamp': utc_now(),
        'load_average': load,
        'memory_total_kib': memory.get('MemTotal'),
        'memory_available_kib': memory.get('MemAvailable'),
        'memory_used_kib': (
            memory.get('MemTotal', 0) - memory.get('MemAvailable', 0)
            if 'MemTotal' in memory and 'MemAvailable' in memory else None
        ),
    }


def kernel_gpu_events():
    """Read the current boot journal; the subprocess is bounded and reaped."""
    try:
        result = subprocess.run(
            ['journalctl', '-k', '-b', '--no-pager', '-o', 'short-iso-precise'],
            capture_output=True, text=True, timeout=10.0, check=False,
        )
        lines = [line for line in result.stdout.splitlines()
                 if GPU_SAFETY_TEXT in line]
        return {'available': result.returncode == 0, 'count': len(lines),
                'events': lines, 'error': result.stderr.strip()}
    except (OSError, subprocess.SubprocessError) as error:
        return {'available': False, 'count': 0, 'events': [],
                'error': str(error)}


def kernel_relevant_events():
    """Capture AMDGPU warnings and Ruby shutdown crashes for correlation."""
    try:
        result = subprocess.run(
            ['journalctl', '-k', '-b', '--no-pager', '-o',
             'short-iso-precise'],
            capture_output=True, text=True, timeout=10.0, check=False)
        lines = [
            line for line in result.stdout.splitlines()
            if (GPU_SAFETY_TEXT in line
                or ('kernel: ruby[' in line and 'segfault' in line))
        ]
        return {'available': result.returncode == 0, 'count': len(lines),
                'events': lines, 'error': result.stderr.strip()}
    except (OSError, subprocess.SubprocessError) as error:
        return {'available': False, 'count': 0, 'events': [],
                'error': str(error)}


def ros_nodes(environment, timeout=8.0):
    """Query nodes without the persistent ROS CLI daemon."""
    try:
        result = subprocess.run(
            ['ros2', 'node', 'list', '--no-daemon'], env=environment,
            capture_output=True, text=True, timeout=timeout, check=False,
        )
        nodes = sorted({line.strip() for line in result.stdout.splitlines()
                        if line.strip()})
        return {'available': result.returncode == 0, 'nodes': nodes,
                'returncode': result.returncode, 'error': result.stderr.strip()}
    except (OSError, subprocess.SubprocessError) as error:
        return {'available': False, 'nodes': [], 'returncode': None,
                'error': str(error)}


def ros_graph_snapshot(environment, timeout=8.0):
    """Capture the four requested ROS graph inventories with bounded CLIs."""
    graph = {'timestamp': utc_now()}
    for kind in ('node', 'topic', 'service', 'action'):
        command = ['ros2', kind, 'list']
        if kind == 'node':
            command.append('--no-daemon')
        try:
            result = subprocess.run(
                command, env=environment, capture_output=True, text=True,
                timeout=timeout, check=False)
            graph[kind + 's'] = sorted({
                line.strip() for line in result.stdout.splitlines()
                if line.strip()})
            graph[kind + '_returncode'] = result.returncode
            graph[kind + '_error'] = result.stderr.strip()
        except (OSError, subprocess.SubprocessError) as error:
            graph[kind + 's'] = []
            graph[kind + '_returncode'] = None
            graph[kind + '_error'] = str(error)
    graph['available'] = all(
        graph[kind + '_returncode'] == 0
        for kind in ('node', 'topic', 'service', 'action'))
    return graph


def wait_for_ros_baseline(environment, expected_nodes, timeout=12.0,
                          settle=1.0, interval=0.5):
    """Require the expected graph continuously for a bounded settle window."""
    deadline = time.monotonic() + timeout
    clean_since = None
    observations = []
    while time.monotonic() < deadline:
        observation = ros_nodes(environment)
        observations.append(observation)
        if observation['available'] and observation['nodes'] == expected_nodes:
            clean_since = clean_since or time.monotonic()
            if time.monotonic() - clean_since >= settle:
                return {'clean': True, 'observations': observations}
        else:
            clean_since = None
        time.sleep(interval)
    return {'clean': False, 'observations': observations}


def _signal_group(pgid, signum):
    if pgid <= 1 or pgid == os.getpgrp():
        raise RuntimeError(f'refusing unsafe process group {pgid}')
    try:
        os.killpg(pgid, signum)
        return True
    except ProcessLookupError:
        return False


def wait_group_empty(pgid, timeout, interval=0.1):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        members = group_processes(pgid)
        if not members:
            return []
        time.sleep(interval)
    return group_processes(pgid)


def terminate_process_group(pgid, timeouts=(8.0, 5.0, 3.0)):
    """Escalate an isolated run group and verify that no live member remains."""
    actions = []
    for signum, timeout in zip(
            (signal.SIGINT, signal.SIGTERM, signal.SIGKILL), timeouts):
        before = group_processes(pgid)
        if not before:
            break
        live = [item for item in before if item['state'] != 'Z']
        if not live:
            wait_group_empty(pgid, timeout)
            break
        sent = _signal_group(pgid, signum)
        actions.append({
            'timestamp': utc_now(), 'signal': signal.Signals(signum).name,
            'sent': sent, 'members_before': before,
        })
        after = wait_group_empty(pgid, timeout)
        actions[-1]['members_after'] = after
    remaining = group_processes(pgid)
    return {
        'pgid': pgid,
        'actions': actions,
        'remaining': remaining,
        'live_remaining': [item for item in remaining if item['state'] != 'Z'],
        # A killed direct child can remain as a zombie until its Popen owner
        # calls wait(). The campaign performs that reap and then verifies the
        # group is completely empty before allowing another launch.
        'clean': not any(item['state'] != 'Z' for item in remaining),
    }
