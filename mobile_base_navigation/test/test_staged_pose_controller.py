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

"""
Drive the staged pose controller with a fake robot and check the sequence.

The fake integrates the twists the controller publishes, so the loop is
genuinely closed - the controller is not being fed a scripted pose that would
reach the goal no matter what it commanded.
"""

import math
from pathlib import Path
import sys
import threading
import time

from geometry_msgs.msg import TwistStamped

from mobile_base_interfaces.action import GoToPose

from nav_msgs.msg import Odometry

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'scripts'))

from staged_pose_controller import (  # noqa: E402
    ALIGN,
    ORIENT,
    StagedPoseController,
    TRANSLATE,
)

RATE = 50.0


class FakeRobot(Node):
    """Integrate commanded twists and republish the resulting pose."""

    def __init__(self):
        super().__init__('fake_robot')
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.commands = []
        self.create_subscription(
            TwistStamped, 'cmd_vel_nav', self._on_command, 20)
        self._odometry = self.create_publisher(
            Odometry, '/odometry/filtered', 20)
        self._last = None
        self.paused = False
        self.create_timer(1.0 / RATE, self._publish)

    def _on_command(self, message):
        self.commands.append((
            message.twist.linear.x,
            message.twist.linear.y,
            message.twist.angular.z,
        ))
        self._last = message.twist

    def _publish(self):
        if self.paused:
            return
        dt = 1.0 / RATE
        if self._last is not None:
            # Body-frame twist integrated into the world frame.
            self.x += (self._last.linear.x * math.cos(self.theta)
                       - self._last.linear.y * math.sin(self.theta)) * dt
            self.y += (self._last.linear.x * math.sin(self.theta)
                       + self._last.linear.y * math.cos(self.theta)) * dt
            self.theta += self._last.angular.z * dt
        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'odom'
        message.pose.pose.position.x = self.x
        message.pose.pose.position.y = self.y
        message.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        message.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        self._odometry.publish(message)


class Watcher(Node):
    """Record the ordered state sequence reported by action feedback."""

    def __init__(self):
        super().__init__('watcher')
        self.states = []
        self.omega_while_translating = []
        self.client = ActionClient(self, GoToPose, 'go_to_pose')

    def on_feedback(self, message):
        state = message.feedback.state
        if not self.states or self.states[-1] != state:
            self.states.append(state)


def _run(goal_x, goal_y, goal_theta, timeout=90.0):
    """Run one goal and return (states, result, robot, commands-by-state)."""
    rclpy.init()
    controller = StagedPoseController()
    robot = FakeRobot()
    watcher = Watcher()
    executor = MultiThreadedExecutor()
    for node in (controller, robot, watcher):
        executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    assert watcher.client.wait_for_server(timeout_sec=15.0), 'no action server'
    goal = GoToPose.Goal()
    goal.x, goal.y, goal.theta_final = goal_x, goal_y, goal_theta

    # Sample the controller's own state alongside the commands it publishes,
    # so omega during TRANSLATE can be attributed to the right phase.
    per_state = {}
    stop = threading.Event()

    def sample():
        while not stop.is_set():
            state = controller._state
            if robot.commands:
                per_state.setdefault(state, []).append(robot.commands[-1])
            time.sleep(0.02)

    sampler = threading.Thread(target=sample, daemon=True)
    sampler.start()

    handle_future = watcher.client.send_goal_async(
        goal, feedback_callback=watcher.on_feedback)
    deadline = time.time() + timeout
    while not handle_future.done() and time.time() < deadline:
        time.sleep(0.05)
    handle = handle_future.result()
    assert handle is not None and handle.accepted, 'goal was not accepted'

    result_future = handle.get_result_async()
    while not result_future.done() and time.time() < deadline:
        time.sleep(0.05)
    stop.set()
    result = result_future.result().result if result_future.done() else None

    executor.shutdown()
    for node in (controller, robot, watcher):
        node.destroy_node()
    rclpy.shutdown()
    return watcher.states, result, robot, per_state


def test_state_sequence_and_heading_hold():
    """IDLE->ALIGN->TRANSLATE->ORIENT->DONE, and omega stays small mid-phase."""
    states, result, robot, per_state = _run(1.0, 1.0, math.pi / 2.0)

    # Feedback only starts once the goal is executing, so IDLE is not
    # necessarily observed. What matters is that the driving phases each
    # appear exactly once, in order - a repeat would mean the dwell
    # hysteresis failed to stop the machine flapping at a tolerance boundary.
    phases = ('ALIGN', 'TRANSLATE', 'ORIENT')
    driving = [state for state in states if state in phases]
    assert driving == list(phases), states

    assert result is not None and result.success, result
    assert result.final_position_error < 0.10, result.final_position_error
    assert result.final_heading_error < 0.10, result.final_heading_error

    # During TRANSLATE the heading is held, not driven: the secondary loop
    # should only ever be trimming, never commanding a real rotation.
    translating = per_state.get('TRANSLATE', [])
    assert translating, 'never observed a TRANSLATE command'
    worst = max(abs(omega) for _, _, omega in translating)
    assert worst < 0.10, f'heading hold drifted, max |omega| = {worst}'
    # ...and it must actually be translating while it does so.
    fastest = max(math.hypot(vx, vy) for vx, vy, _ in translating)
    assert fastest > 0.01, 'TRANSLATE never commanded linear motion'


class Recorder:
    """Stand in for the command publisher so _tick can be driven directly."""

    def __init__(self):
        self.sent = []

    def publish(self, message):
        self.sent.append((
            message.twist.linear.x,
            message.twist.linear.y,
            message.twist.angular.z,
        ))


def _tick_once(state, pose, goal, hold_heading=None):
    """
    Run exactly one control tick in a known state and return the command.

    Driving _tick directly keeps this deterministic. Sampling the state and
    the command separately from a thread races: the state can flip while the
    last command of the previous phase is still the newest one published.
    """
    rclpy.init()
    controller = StagedPoseController()
    recorder = Recorder()
    controller._commands = recorder
    controller._odometry = pose
    controller._odometry_time = controller._now()
    controller._goal = goal
    controller._hold_heading = hold_heading
    controller._state = state
    controller._state_entered = controller._now()
    controller._tick()
    controller.destroy_node()
    rclpy.shutdown()
    return recorder.sent


def test_align_commands_no_linear_motion():
    """ALIGN rotates in place: vx and vy are exactly zero."""
    sent = _tick_once(ALIGN, (0.0, 0.0, 0.0), (1.0, 1.0, 0.0))
    assert sent, 'ALIGN published nothing'
    vx, vy, omega = sent[-1]
    assert (vx, vy) == (0.0, 0.0), sent[-1]
    # The goal is at 45 degrees, so it must actually be turning toward it.
    assert omega > 0.0, sent[-1]


def test_orient_commands_no_linear_motion():
    """ORIENT rotates in place: vx and vy are exactly zero."""
    sent = _tick_once(ORIENT, (1.0, 1.0, 0.0), (1.0, 1.0, math.pi / 2.0))
    assert sent, 'ORIENT published nothing'
    vx, vy, omega = sent[-1]
    assert (vx, vy) == (0.0, 0.0), sent[-1]
    assert omega > 0.0, sent[-1]


def test_translate_error_is_rotated_into_the_body_frame():
    """A goal to the robot's left becomes +vy, not +vx."""
    # Robot at the origin facing +x; goal one metre to its left (+y world).
    sent = _tick_once(TRANSLATE, (0.0, 0.0, 0.0), (0.0, 1.0, 0.0),
                      hold_heading=0.0)
    vx, vy, _ = sent[-1]
    assert abs(vx) < 1e-6, sent[-1]
    assert vy > 0.0, sent[-1]

    # Same goal, robot rotated 90 degrees left: now it is straight ahead.
    sent = _tick_once(TRANSLATE, (0.0, 0.0, math.pi / 2.0), (0.0, 1.0, 0.0),
                      hold_heading=math.pi / 2.0)
    vx, vy, _ = sent[-1]
    assert vx > 0.0, sent[-1]
    assert abs(vy) < 1e-6, sent[-1]


def test_translate_heading_hold_corrects_drift():
    """The secondary loop opposes yaw drift rather than ignoring it."""
    # Held heading is 0, but the robot has drifted +0.2 rad: expect -omega.
    sent = _tick_once(TRANSLATE, (0.0, 0.0, 0.2), (1.0, 0.0, 0.0),
                      hold_heading=0.0)
    assert sent[-1][2] < 0.0, sent[-1]
    sent = _tick_once(TRANSLATE, (0.0, 0.0, -0.2), (1.0, 0.0, 0.0),
                      hold_heading=0.0)
    assert sent[-1][2] > 0.0, sent[-1]


def test_commands_are_clamped_to_the_limits():
    """A far goal must not command more than max_linear_vel."""
    sent = _tick_once(TRANSLATE, (0.0, 0.0, 0.0), (100.0, 0.0, 0.0),
                      hold_heading=0.0)
    vx, vy, _ = sent[-1]
    assert math.hypot(vx, vy) <= 0.15 + 1e-9, sent[-1]


def test_stops_when_odometry_goes_stale():
    """A dead odometry source must hold zero, not coast on the last command."""
    rclpy.init()
    controller = StagedPoseController()
    robot = FakeRobot()
    executor = MultiThreadedExecutor()
    executor.add_node(controller)
    executor.add_node(robot)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    time.sleep(1.0)

    # Drive the state machine by hand, then starve it of odometry.
    controller._goal = (2.0, 0.0, 0.0)
    controller._transition(ALIGN)
    time.sleep(0.5)
    # The fake must stop publishing, or _on_odometry refreshes the timestamp
    # 20 ms later and the source is never actually stale.
    robot.paused = True
    time.sleep(1.0)
    robot.commands.clear()
    time.sleep(0.5)

    assert robot.commands, 'controller stopped publishing entirely'
    assert all(v == (0.0, 0.0, 0.0) for v in robot.commands), robot.commands

    executor.shutdown()
    controller.destroy_node()
    robot.destroy_node()
    rclpy.shutdown()
