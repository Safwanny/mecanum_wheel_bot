#!/usr/bin/env python3
"""Run odometry evaluations with one isolated simulation per repetition."""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

from mobile_base_tools.process_lifecycle import (
    group_processes,
    host_resources,
    kernel_gpu_events,
    kernel_relevant_events,
    process_snapshot,
    project_processes,
    ros_graph_snapshot,
    ros_nodes,
    RUN_TOKEN_ENV,
    terminate_process_group,
    token_processes,
    utc_now,
    wait_for_ros_baseline,
    wait_group_empty,
)


PROFILES = (
    'forward_1m', 'backward_1m', 'strafe_left_1m', 'strafe_right_1m',
    'rotate_positive_90deg', 'rotate_negative_90deg',
    'diagonal_forward_left', 'diagonal_forward_right',
    'diagonal_backward_left', 'diagonal_backward_right', 'square_1m',
)


class LifecycleBlocked(RuntimeError):
    """The next simulation must not start because cleanup is unverified."""


def launch_command(profile, mode, repetitions, output, repetition_offset=0):
    """Build an evaluator command (campaign calls this with repetitions=1)."""
    localization = 'true' if mode == 'raw_and_filtered' else 'false'
    return [
        'ros2', 'launch', 'mobile_base_bringup',
        'odometry_evaluation.launch.py', 'world:=empty',
        f'test_profile:={profile}', f'repetitions:={repetitions}',
        f'repetition_offset:={repetition_offset}',
        f'evaluation_mode:={mode}', f'localization:={localization}',
        'gui:=false', 'rviz:=false', 'shutdown_on_complete:=true',
        f'output_dir:={output}',
    ]


def completed(output):
    report = json.loads((output / 'summary.json').read_text())
    runs = report['runs']
    return (sum(run['completion_status'] == 'completed' for run in runs),
            len(runs))


def partial_counts(output):
    summary = output / 'summary.json'
    if summary.is_file():
        return completed(output)
    results = []
    for path in sorted((output / 'run_results').glob('*.json')):
        try:
            results.append(json.loads(path.read_text(encoding='utf-8')))
        except (OSError, json.JSONDecodeError):
            continue
    return (sum(item.get('completion_status') == 'completed'
                for item in results), len(results))


def write_summary(root, results, status, blocked_reason=None):
    """Atomically persist state after every individually isolated repetition."""
    root.mkdir(parents=True, exist_ok=True)
    path = root / 'campaign_summary.json'
    temporary = root / '.campaign_summary.json.tmp'
    document = {
        'schema_version': 3,
        'isolation': 'one process session per trajectory repetition',
        'status': status,
        'updated_at': datetime.now(timezone.utc).isoformat(),
        'runs': results,
    }
    if blocked_reason:
        document['blocked_reason'] = blocked_reason
    temporary.write_text(
        json.dumps(document, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    temporary.replace(path)


def completed_repetitions(results, mode, profile):
    """Interpret both legacy grouped entries and schema-3 repetition entries."""
    repetitions = set()
    for item in results:
        if item.get('mode') != mode or item.get('profile') != profile:
            continue
        if 'repetition' in item:
            if item.get('success') is True:
                repetitions.add(int(item['repetition']))
        else:
            repetitions.update(range(1, int(item.get('completed', 0)) + 1))
    return repetitions


def append_jsonl(path, record):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('a', encoding='utf-8') as stream:
        stream.write(json.dumps(record, sort_keys=True) + '\n')


def cleanup_gate(run_token=None, tracked_pgid=None):
    """Clean exact owned children; block on any unowned project runtime."""
    cleanups = []
    if tracked_pgid is not None and group_processes(tracked_pgid):
        cleanups.append(terminate_process_group(tracked_pgid))
    snapshot = process_snapshot()
    owned = token_processes(run_token, snapshot) if run_token else []
    for pgid in sorted({item['pgid'] for item in owned}):
        if pgid != tracked_pgid:
            cleanups.append(terminate_process_group(pgid))
    if owned:
        snapshot = process_snapshot()
        owned = token_processes(run_token, snapshot)
    owned_pids = {item['pid'] for item in owned}
    unexpected = project_processes(
        snapshot, excluded_pids={os.getpid(), os.getppid(), *owned_pids})
    if owned:
        raise LifecycleBlocked(
            f'campaign-owned processes survived cleanup: {[p["pid"] for p in owned]}')
    if any(not cleanup['clean'] for cleanup in cleanups):
        raise LifecycleBlocked(
            f'process group {tracked_pgid} survived escalation')
    if unexpected:
        raise LifecycleBlocked(
            'unowned project-related processes present: '
            + ', '.join(f'{p["pid"]}:{p["command"]}' for p in unexpected))
    return {'clean': True, 'cleanups': cleanups, 'snapshot': snapshot,
            'project_processes': []}


def run_launch(command, environment, log_path, expected_nodes=None,
               gpu_count_before=0):
    """Run, reap, clean, and graph-verify one isolated process session."""
    expected_nodes = [] if expected_nodes is None else expected_nodes
    process = None
    returncode = None
    interrupted = False
    gpu_triggered = False
    gpu_monitor_error = None
    pgid = None
    with log_path.open('w', encoding='utf-8') as log:
        try:
            process = subprocess.Popen(
                command, env=environment, stdout=log,
                stderr=subprocess.STDOUT, start_new_session=True)
            pgid = process.pid
            next_gpu_check = time.monotonic()
            while True:
                try:
                    returncode = process.wait(timeout=1.0)
                    break
                except subprocess.TimeoutExpired:
                    if time.monotonic() < next_gpu_check:
                        continue
                    gpu = kernel_gpu_events()
                    next_gpu_check = time.monotonic() + 5.0
                    if not gpu['available']:
                        gpu_monitor_error = gpu['error'] or (
                            'kernel safety journal unavailable')
                        interrupted = True
                        break
                    if gpu['count'] > gpu_count_before:
                        gpu_triggered = True
                        interrupted = True
                        break
        except KeyboardInterrupt:
            interrupted = True
        finally:
            cleanup = (
                terminate_process_group(pgid) if pgid is not None else
                {'clean': True, 'actions': [], 'remaining': [],
                 'live_remaining': [], 'pgid': None}
            )
            if process is not None and process.poll() is None:
                try:
                    returncode = process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    cleanup = terminate_process_group(pgid, (0.5, 0.5, 1.0))
                    returncode = process.wait(timeout=2.0)
            elif process is not None:
                returncode = process.wait()
            if pgid is not None:
                remaining = wait_group_empty(pgid, 2.0)
                cleanup['remaining'] = remaining
                cleanup['live_remaining'] = [
                    item for item in remaining if item['state'] != 'Z']
                cleanup['clean'] = not remaining
    graph = wait_for_ros_baseline(environment, expected_nodes)
    result = {
        'returncode': returncode,
        'pgid': pgid,
        'interrupted': interrupted,
        'gpu_triggered': gpu_triggered,
        'gpu_monitor_error': gpu_monitor_error,
        'cleanup': cleanup,
        'graph': graph,
    }
    if not cleanup['clean'] or not graph['clean']:
        raise LifecycleBlocked(json.dumps(result, sort_keys=True))
    if interrupted:
        raise KeyboardInterrupt
    return result


def telemetry(stage, run_index, mode, profile, repetition, environment,
              run_token, tracked_pgid=None):
    snapshot = process_snapshot()
    project = project_processes(snapshot, excluded_pids={os.getpid(), os.getppid()})
    owned = token_processes(run_token, snapshot) if run_token else []
    return {
        'timestamp': utc_now(), 'stage': stage, 'run_index': run_index,
        'mode': mode, 'profile': profile, 'repetition': repetition,
        'ros_domain_id': environment.get('ROS_DOMAIN_ID'),
        'gz_partition': environment.get('GZ_PARTITION'),
        'resources': host_resources(), 'gpu': kernel_gpu_events(),
        'project_process_count': len(project), 'project_processes': project,
        'owned_process_count': len(owned), 'owned_processes': owned,
        'tracked_group': group_processes(tracked_pgid) if tracked_pgid else [],
    }


def _install_term_handler():
    previous = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGTERM,
                  lambda signum, frame: (_ for _ in ()).throw(KeyboardInterrupt()))
    return previous


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--profiles', default=','.join(PROFILES))
    parser.add_argument('--modes', default='raw_only')
    parser.add_argument('--repetitions', type=int, default=1)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--launch-settle-wall', type=float, default=3.0)
    parser.add_argument('--startup-attempts', type=int, default=3)
    parser.add_argument('--resume', action='store_true')
    args = parser.parse_args()
    profiles = [value.strip() for value in args.profiles.split(',') if value]
    modes = [value.strip() for value in args.modes.split(',') if value]
    if set(profiles) - set(PROFILES):
        parser.error('unknown profile')
    if set(modes) - {'raw_only', 'raw_and_filtered'}:
        parser.error('unknown mode')
    if args.repetitions <= 0 or args.startup_attempts <= 0:
        parser.error('repetitions and startup attempts must be positive')

    root = Path(args.output_dir).expanduser().resolve()
    summary_path = root / 'campaign_summary.json'
    if root.exists() and any(root.iterdir()) and not args.resume:
        parser.error('output directory is not empty; choose a new directory or use --resume')
    if args.resume and root.exists() and any(root.iterdir()) and not summary_path.is_file():
        parser.error('cannot resume non-empty output directory without campaign_summary.json')
    results = []
    if args.resume and summary_path.is_file():
        try:
            results = json.loads(summary_path.read_text(encoding='utf-8'))['runs']
        except (OSError, KeyError, TypeError, json.JSONDecodeError) as error:
            parser.error(f'cannot resume invalid campaign summary: {error}')

    diagnostics = root / 'diagnostics' / 'process_lifecycle'
    telemetry_path = diagnostics / 'per_run_telemetry.jsonl'
    diagnostics.mkdir(parents=True, exist_ok=True)
    gpu_initial = kernel_gpu_events()
    baseline_environment = os.environ.copy()
    baseline_graph = ros_graph_snapshot(baseline_environment)
    baseline = {
        'timestamp': utc_now(), 'processes': process_snapshot(),
        'project_processes': project_processes(excluded_pids={os.getpid(), os.getppid()}),
        'resources': host_resources(), 'gpu': gpu_initial,
        'kernel_relevant_events': kernel_relevant_events(),
        'ros_graph': baseline_graph,
    }
    (diagnostics / 'baseline.json').write_text(
        json.dumps(baseline, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    if baseline['project_processes']:
        reason = 'pre-campaign baseline contains project-related processes'
        write_summary(root, results, 'blocked', reason)
        return 2
    if not gpu_initial['available']:
        reason = 'mandatory GPU safety journal is unavailable'
        write_summary(root, results, 'blocked', reason)
        return 2
    if gpu_initial['count']:
        reason = 'mandatory AMDGPU safety trigger already exists in current boot'
        write_summary(root, results, 'blocked', reason)
        return 2

    previous_term = _install_term_handler()
    active = None
    success = True
    run_index = 0
    write_summary(root, results, 'running')
    try:
        for mode in modes:
            for profile in profiles:
                done = completed_repetitions(results, mode, profile)
                for repetition in range(1, args.repetitions + 1):
                    if repetition in done:
                        continue
                    run_index += 1
                    run_token = f'{os.getpid()}-{uuid.uuid4().hex}'
                    completed_count = attempted_count = 0
                    launch_result = None
                    output = log_path = environment = None
                    started_at = utc_now()
                    for attempt in range(1, args.startup_attempts + 1):
                        cleanup_gate()
                        output = (root / mode / profile / 'isolated_repetitions'
                                  / f'repetition_{repetition:02d}'
                                  / f'attempt_{attempt:02d}')
                        if output.exists() and any(output.iterdir()):
                            raise LifecycleBlocked(f'non-empty attempt output: {output}')
                        output.mkdir(parents=True, exist_ok=True)
                        command = launch_command(
                            profile, mode, 1, output, repetition - 1)
                        environment = os.environ.copy()
                        environment[RUN_TOKEN_ENV] = run_token
                        identity = run_index * args.startup_attempts + attempt
                        environment['ROS_DOMAIN_ID'] = str(10 + (os.getpid() + identity) % 220)
                        environment['GZ_PARTITION'] = (
                            f'mobile_base_campaign_{os.getpid()}_{identity}')
                        graph_before = ros_nodes(environment)
                        if not graph_before['available'] or graph_before['nodes']:
                            raise LifecycleBlocked(f'non-clean pre-run ROS graph: {graph_before}')
                        log_path = output / 'launch.log'
                        active = {
                            'mode': mode, 'profile': profile,
                            'repetition': repetition, 'started_at': started_at,
                            'command': command, 'output': str(output),
                            'launch_log': str(log_path),
                            'ros_domain_id': environment['ROS_DOMAIN_ID'],
                            'gz_partition': environment['GZ_PARTITION'],
                            'run_token': run_token, 'startup_attempts_used': attempt,
                            'requested_repetitions': 1,
                        }
                        before = telemetry('before', run_index, mode, profile,
                                           repetition, environment, run_token)
                        append_jsonl(telemetry_path, before)
                        launch_result = run_launch(
                            command, environment, log_path,
                            expected_nodes=graph_before['nodes'],
                            gpu_count_before=before['gpu']['count'])
                        after = telemetry('after', run_index, mode, profile,
                                          repetition, environment, run_token,
                                          launch_result['pgid'])
                        append_jsonl(telemetry_path, after)
                        cleanup_gate(run_token, launch_result['pgid'])
                        completed_count, attempted_count = partial_counts(output)
                        if attempted_count:
                            break
                        if args.launch_settle_wall > 0:
                            time.sleep(args.launch_settle_wall)
                    log_text = log_path.read_text(encoding='utf-8', errors='replace')
                    run_success = (
                        launch_result['returncode'] == 0
                        and completed_count == attempted_count == 1)
                    success = success and run_success
                    results.append({
                        **active, 'success': run_success,
                        'finished_at': utc_now(), 'completed': completed_count,
                        'attempted': attempted_count,
                        'process_returncode': launch_result['returncode'],
                        'cleanup': launch_result['cleanup'],
                        'ros_graph_cleanup': launch_result['graph'],
                        'gpu_triggered': launch_result['gpu_triggered'],
                        'gazebo_shutdown_crashes': log_text.count('Segmentation fault'),
                        'stale_command_warnings': (
                            log_text.count('older by')
                            + log_text.count('older than allowed timeout')),
                        'clock_stall_warnings': log_text.count('clock stalled'),
                        'motion_start_failures': log_text.count(
                            'motion did not start after command'),
                    })
                    write_summary(root, results, 'running')
                    active = None
    except (KeyboardInterrupt, LifecycleBlocked) as error:
        reason = str(error) or 'interrupted by signal or GPU safety monitor'
        if active is not None:
            output = Path(active['output'])
            completed_count, attempted_count = partial_counts(output)
            results.append({
                **active, 'success': False, 'completion_status': 'interrupted',
                'finished_at': utc_now(), 'completed': completed_count,
                'attempted': attempted_count, 'process_returncode': 130,
                'blocked_reason': reason,
            })
        write_summary(root, results, 'blocked', reason)
        return 130 if isinstance(error, KeyboardInterrupt) else 2
    finally:
        signal.signal(signal.SIGTERM, previous_term)
    write_summary(root, results, 'completed' if success else 'incomplete')
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
