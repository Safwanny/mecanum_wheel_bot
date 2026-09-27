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

"""Run the bug navigator through a series of goals and score every run.

Start ``bug_navigation.launch.py`` first, then for example::

    ros2 run mobile_base_tools bug_trials --world navigation_basic --random 5
    ros2 run mobile_base_tools bug_trials --world my_world --random 8 --seed 3
    ros2 run mobile_base_tools bug_trials --world my_world --goals 3,-1.5 -3,2

Goals are chained: each starts where the last one ended. Random goals are
drawn from the world's own SDF, clear of every obstacle by ``--clearance`` and
at least ``--min-distance`` from the robot, so each seed is a new, repeatable
course. Clearance to obstacles is measured against the same SDF geometry.

Positions come from odometry, which is all the robot has; ``--spawn`` must
match the launch's x, y, yaw so odom can be placed in the world. Odometry
drift therefore shows up in the clearance figures too - it is the robot's own
belief about where it passed.
"""

import argparse
import csv
import math
import random
import sys
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Half the body diagonal: the furthest the body reaches from its centre.
BODY_REACH = math.hypot(0.13, 0.11)


def load_obstacles(world):
    """Boxes (x, y, yaw, sx, sy) and cylinders (x, y, r) from a world SDF."""
    path = '{}/worlds/{}.sdf'.format(
        get_package_share_directory('mobile_base_gazebo'), world)
    boxes, cylinders = [], []
    for model in ET.parse(path).getroot().iter('model'):
        pose = [float(v) for v in (model.findtext('pose') or
                                   '0 0 0 0 0 0').split()]
        for collision in model.iter('collision'):
            local = [float(v) for v in (collision.findtext('pose') or
                                        '0 0 0 0 0 0').split()]
            yaw = pose[5]
            x = pose[0] + local[0] * math.cos(yaw) - local[1] * math.sin(yaw)
            y = pose[1] + local[0] * math.sin(yaw) + local[1] * math.cos(yaw)
            geometry = collision.find('geometry')
            box, cylinder = geometry.find('box'), geometry.find('cylinder')
            if box is not None:
                sx, sy, _ = (float(v) for v in box.findtext('size').split())
                if sx > 50 or sy > 50:
                    continue  # a ground plane, not an obstacle
                boxes.append((x, y, yaw + local[5], sx, sy))
            elif cylinder is not None:
                cylinders.append((x, y, float(cylinder.findtext('radius'))))
    return boxes, cylinders


def clearance(point, obstacles):
    """Distance from a point to the nearest obstacle surface."""
    boxes, cylinders = obstacles
    best = math.inf
    for x, y, yaw, sx, sy in boxes:
        dx, dy = point[0] - x, point[1] - y
        along = dx * math.cos(yaw) + dy * math.sin(yaw)
        across = -dx * math.sin(yaw) + dy * math.cos(yaw)
        best = min(best, math.hypot(max(abs(along) - sx / 2.0, 0.0),
                                    max(abs(across) - sy / 2.0, 0.0)))
    for x, y, radius in cylinders:
        best = min(best, max(math.hypot(point[0] - x, point[1] - y)
                             - radius, 0.0))
    return best


def bounds(obstacles):
    """The world's footprint, from the extent of its walls."""
    boxes, cylinders = obstacles
    xs = [b[0] for b in boxes] + [c[0] for c in cylinders]
    ys = [b[1] for b in boxes] + [c[1] for c in cylinders]
    return min(xs), max(xs), min(ys), max(ys)


def random_goal(rng, obstacles, near, margin, min_distance):
    x0, x1, y0, y1 = bounds(obstacles)
    for _ in range(5000):
        goal = (rng.uniform(x0, x1), rng.uniform(y0, y1))
        if (clearance(goal, obstacles) >= margin
                and math.dist(goal, near) >= min_distance):
            return goal
    raise RuntimeError('no free goal found; lower --clearance')


class Trials(Node):
    """Send goals, follow the runs, collect the scores."""

    def __init__(self, spawn):
        super().__init__('bug_trials')
        self.spawn = spawn
        self.pose = None
        self.state = None
        self.level = None
        self.goals = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.create_subscription(
            Odometry, '/odometry/filtered', self.on_odometry, 10)
        self.create_subscription(
            String, '/bug_navigator/state',
            lambda m: setattr(self, 'state', m.data), 10)
        self.create_subscription(
            String, '/speed_governor/level',
            lambda m: setattr(self, 'level', m.data.split()[0]), 10)

    def on_odometry(self, message):
        p = message.pose.pose.position
        # odom starts at the spawn pose: place it in the world frame.
        x0, y0, yaw = self.spawn
        self.pose = (x0 + p.x * math.cos(yaw) - p.y * math.sin(yaw),
                     y0 + p.x * math.sin(yaw) + p.y * math.cos(yaw))

    def send(self, goal):
        """Publish a world-frame goal in odom."""
        x0, y0, yaw = self.spawn
        dx, dy = goal[0] - x0, goal[1] - y0
        message = PoseStamped()
        message.header.frame_id = 'odom'
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = dx * math.cos(yaw) + dy * math.sin(yaw)
        message.pose.position.y = -dx * math.sin(yaw) + dy * math.cos(yaw)
        message.pose.orientation.w = 1.0
        self.goals.publish(message)

    def wait(self, seconds, until=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if until is not None and until():
                return True
        return False

    def run(self, goal, obstacles, timeout):
        start = self.pose
        self.state = None
        self.send(goal)
        began = time.monotonic()
        path, levels, low = [start], {}, math.inf
        while time.monotonic() - began < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pose != path[-1]:
                path.append(self.pose)
                low = min(low, clearance(self.pose, obstacles))
            if self.level:
                levels[self.level] = levels.get(self.level, 0) + 1
            if self.state in ('arrived', 'unreachable'):
                break
        elapsed = time.monotonic() - began
        length = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
        straight = math.dist(start, goal)
        total = sum(levels.values()) or 1
        return {
            'goal': '{:+.2f},{:+.2f}'.format(*goal),
            'result': self.state if self.state in (
                'arrived', 'unreachable') else 'timeout',
            'time_s': round(elapsed, 1),
            'path_m': round(length, 2),
            'straight_m': round(straight, 2),
            'efficiency': round(straight / length, 2) if length else 0.0,
            'body_clearance_m': round(low - BODY_REACH, 3),
            'end_error_m': round(math.dist(self.pose, goal), 3),
            'levels': ' '.join('{}:{:.0%}'.format(k, v / total)
                               for k, v in sorted(levels.items())),
        }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--world', default='navigation_basic')
    parser.add_argument('--goals', nargs='*', default=[],
                        help='world-frame goals as x,y')
    parser.add_argument('--random', type=int, default=0,
                        help='number of random goals')
    parser.add_argument('--seed', type=int, default=None)
    parser.add_argument('--spawn', default='0,0,0',
                        help='launch spawn pose x,y,yaw')
    parser.add_argument('--clearance', type=float, default=0.45,
                        help='m between a random goal and any obstacle')
    parser.add_argument('--min-distance', type=float, default=1.5)
    parser.add_argument('--timeout', type=float, default=150.0)
    parser.add_argument('--csv', default=None, help='also write results here')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(
        argv if argv is not None else sys.argv)[1:])

    obstacles = load_obstacles(args.world)
    rng = random.Random(args.seed)
    spawn = tuple(float(v) for v in args.spawn.split(','))

    rclpy.init()
    node = Trials(spawn)
    print('waiting for odometry and the navigator...')
    if not node.wait(60.0, until=lambda: node.pose is not None):
        print('no /odometry/filtered - is bug_navigation.launch.py running?')
        return 1
    node.wait(2.0)

    fixed = [tuple(float(v) for v in g.split(',')) for g in args.goals]
    count = len(fixed) + args.random
    results = []
    for index in range(count):
        goal = fixed[index] if index < len(fixed) else random_goal(
            rng, obstacles, node.pose, args.clearance, args.min_distance)
        print('[{}/{}] goal ({:+.2f}, {:+.2f}) from ({:+.2f}, {:+.2f})'.format(
            index + 1, count, goal[0], goal[1], *node.pose))
        result = node.run(goal, obstacles, args.timeout)
        results.append(result)
        print('   {result}  {time_s}s  path {path_m} m  '
              'efficiency {efficiency}  body clearance {body_clearance_m} m'
              .format(**result))
        node.wait(1.5)

    columns = list(results[0].keys()) if results else []
    print()
    print('  '.join('{:>16}'.format(c) for c in columns[:-1]) + '  levels')
    for result in results:
        print('  '.join('{:>16}'.format(str(result[c]))
                        for c in columns[:-1]) + '  ' + result['levels'])
    arrived = sum(r['result'] == 'arrived' for r in results)
    touched = sum(r['body_clearance_m'] <= 0.0 for r in results)
    # Clearance assumes the body reaches its full diagonal every way, so a
    # run listed here came close, not necessarily into contact.
    print('\n{} of {} arrived; {} run(s) within the body diagonal of an '
          'obstacle'.format(arrived, len(results), touched))
    if args.csv:
        with open(args.csv, 'w', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=columns)
            writer.writeheader()
            writer.writerows(results)
        print('written to', args.csv)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
