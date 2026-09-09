#!/usr/bin/env python3

# Copyright 2026 Safwan
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Shut down every simulation process this workspace can leave behind.

Killing simulation by name does not work here, and the reasons are worth
writing down because each one has cost real debugging time:

  - The Gazebo server runs as ``ruby``. Its command line says ``gz sim`` but
    its process name does not, so ``pkill gz`` misses it entirely. Three of
    these accumulated at roughly 640 MB each and starved the controller
    manager until launch tests began failing for no visible reason.
  - ``robot_state_publisher`` is started with a generated parameter file, so
    its command line contains no package name at all - only
    ``/tmp/launch_params_<random>``. No package-shaped pattern will find it.
  - Gazebo does not always honour SIGINT. The launch logs say so plainly:
    "failed to terminate 5 seconds after receiving SIGINT".
  - A launch started with ``setsid`` or ``nohup`` outlives the shell that
    started it, so closing the terminal leaves the whole stack running.

So this matches on the full command line, kills parents before children so
launch trees come down cleanly, and escalates only where it must.
"""

import argparse
import os
import signal
import time


# Matched against the whole command line, never the process name.
PATTERNS = (
    'gz sim',
    'gz_tools_vendor',
    '/opt/ros/jazzy/lib/',
    'ros2 launch',
    'ros2cli.daemon',
    'ros2_ws/install/mobile_base',
    'launch_params_',
    'ros_gz',
    'rviz2',
)

# Anything whose command line contains one of these is never touched, so an
# editor or the terminal running this script cannot be caught by accident.
PROTECTED = (
    'claude',
    'chrome_crashpad',
    '/usr/share/code',
    'brave',
    'gvfsd',
    'stop_simulation',
)

# Killed first: taking a launch parent down usually takes its children with it.
PARENT_MARKERS = ('ros2 launch', 'ros2cli.daemon')


def command_line(pid):
    """Return one process's full command line, or None if it has gone."""
    try:
        with open(f'/proc/{pid}/cmdline', 'rb') as handle:
            raw = handle.read()
    except OSError:
        return None
    text = raw.decode('utf-8', 'replace').replace('\0', ' ').strip()
    return text or None


def targets():
    """Return [(pid, command line)] for every process worth stopping."""
    found = []
    mine = {os.getpid(), os.getppid()}
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid in mine:
            continue
        cmd = command_line(pid)
        if cmd is None:
            continue
        if any(spare in cmd for spare in PROTECTED):
            continue
        if any(pattern in cmd for pattern in PATTERNS):
            found.append((pid, cmd))
    return found


def send(pids, number):
    """Signal each pid, ignoring the ones that have already exited."""
    delivered = 0
    for pid in pids:
        try:
            os.kill(pid, number)
            delivered += 1
        except (OSError, ProcessLookupError):
            pass
    return delivered


def sweep(dry_run=False, verbose=True):
    """Stop everything, and report what is left. Returns the survivor count."""
    initial = targets()
    if not initial:
        if verbose:
            print('nothing to stop')
        return 0

    if verbose:
        for pid, cmd in initial:
            print(f'  {pid:>7}  {cmd[:96]}')
    if dry_run:
        print(f'\n{len(initial)} process(es) would be stopped')
        return len(initial)

    parents = [
        pid for pid, cmd in initial
        if any(marker in cmd for marker in PARENT_MARKERS)
    ]
    others = [pid for pid, _ in initial if pid not in parents]

    send(parents, signal.SIGTERM)
    time.sleep(3.0)
    send(others, signal.SIGTERM)
    time.sleep(2.0)
    # Gazebo in particular ignores SIGINT and sometimes SIGTERM.
    send([pid for pid, _ in targets()], signal.SIGKILL)
    time.sleep(1.0)

    survivors = targets()
    if verbose:
        if survivors:
            print(f'\n{len(survivors)} still running:')
            for pid, cmd in survivors:
                print(f'  {pid:>7}  {cmd[:96]}')
        else:
            print(f'\nstopped {len(initial)}, none left')
    return len(survivors)


def remove_temporary_files(verbose=True):
    """Delete the generated worlds and parameter files launches leave in /tmp."""
    removed = 0
    for name in os.listdir('/tmp'):
        if (name.startswith('mobile_base_world_')
                or name.startswith('mobile_base_sim_')
                or name.startswith('launch_params_')):
            try:
                os.unlink(os.path.join('/tmp', name))
                removed += 1
            except OSError:
                pass
    if verbose and removed:
        print(f'removed {removed} temporary file(s)')
    return removed


def main():
    """Stop simulation processes and clear what they left in /tmp."""
    parser = argparse.ArgumentParser(
        description='Stop every simulation process and clean up /tmp.')
    parser.add_argument(
        '--dry-run', action='store_true',
        help='list what would be stopped and exit')
    parser.add_argument(
        '--keep-temp', action='store_true',
        help='leave generated world and parameter files in /tmp')
    arguments = parser.parse_args()

    survivors = sweep(dry_run=arguments.dry_run)
    if not arguments.dry_run and not arguments.keep_temp:
        remove_temporary_files()
    raise SystemExit(1 if survivors else 0)
