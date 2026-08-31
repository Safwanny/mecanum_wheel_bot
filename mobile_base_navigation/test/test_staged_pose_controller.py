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

from geometry_msgs.msg import TransformStamped, TwistStamped

from mobile_base_interfaces.action import GoToPose

from nav_msgs.msg import Odometry


import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'scripts'))

from staged_pose_controller import (  # noqa: E402
    ALIGN,
    OFFSETS,
    ORIENT,
    StagedPoseController,
    TRANSLATE,
)

from tf2_ros import TransformBroadcaster  # noqa: E402

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
        # The controller takes its pose from TF, in the goal frame.
        self._tf = TransformBroadcaster(self)
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
        if self.paused:
            return
        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.z = math.sin(self.theta / 2.0)
        transform.transform.rotation.w = math.cos(self.theta / 2.0)
        self._tf.sendTransform(transform)


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


def test_state_sequence_and_primitive_selection():
    """ALIGN -> TRANSLATE -> ORIENT, using a diagonal for a diagonal goal."""
    states, result, robot, per_state = _run(1.0, 1.0, math.pi / 2.0)

    # Feedback carries the primitive as "STATE[PRIMITIVE]", so compare on the
    # base state. Feedback only starts once the goal is executing, so IDLE is
    # not necessarily observed; what matters is that the driving phases each
    # appear once, in order. A repeat would mean the dwell hysteresis failed
    # to stop the machine flapping at a tolerance boundary.
    phases = ('ALIGN', 'TRANSLATE', 'ORIENT')
    driving = []
    for state in (s.split('[')[0] for s in states):
        if state in phases and (not driving or driving[-1] != state):
            driving.append(state)
    assert driving == list(phases), states

    # The goal sits 45 degrees off a robot facing +x, so the cheapest
    # primitive is the front-left diagonal and it needs no rotation at all.
    # This is the end-to-end evidence that diagonals are genuinely used.
    assert any('DIAGONAL_FRONT_LEFT' in state for state in states), states

    assert result is not None and result.success, result
    assert result.final_position_error < 0.10, result.final_position_error
    assert result.final_heading_error < 0.10, result.final_heading_error

    # One motion at a time: the translation carries no rotation whatsoever.
    translating = per_state.get('TRANSLATE', [])
    assert translating, 'never observed a TRANSLATE command'
    assert all(omega == 0.0 for _, _, omega in translating), translating[:5]
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


def normalize(angle):
    """Wrap to [-pi, pi]; a local copy keeps this test self-contained."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _tick_once(state, pose, goal, primitive=None, target_heading=None):
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
    # Stub the TF lookup rather than standing up a broadcaster: this keeps the
    # per-phase checks deterministic and free of graph timing.
    controller._pose = lambda: pose
    controller._goal = goal
    controller._primitive = primitive
    controller._target_heading = target_heading
    controller._state = state
    controller._state_entered = controller._now()
    controller._tick()
    controller.destroy_node()
    rclpy.shutdown()
    return recorder.sent


def test_align_commands_no_linear_motion():
    """ALIGN rotates in place: vx and vy are exactly zero."""
    # Bearing 0.3 rad is nearest FORWARD, so a real turn is required. A goal
    # at exactly 45 degrees would need no rotation at all - the front-left
    # diagonal already points straight at it - which is the point of the
    # model but makes for a poor rotation test.
    sent = _tick_once(ALIGN, (0.0, 0.0, 0.0),
                      (math.cos(0.3), math.sin(0.3), 0.0))
    assert sent, 'ALIGN published nothing'
    vx, vy, omega = sent[-1]
    assert (vx, vy) == (0.0, 0.0), sent[-1]
    assert omega > 0.0, sent[-1]


def test_orient_commands_no_linear_motion():
    """ORIENT rotates in place: vx and vy are exactly zero."""
    sent = _tick_once(ORIENT, (1.0, 1.0, 0.0), (1.0, 1.0, math.pi / 2.0))
    assert sent, 'ORIENT published nothing'
    vx, vy, omega = sent[-1]
    assert (vx, vy) == (0.0, 0.0), sent[-1]
    assert omega > 0.0, sent[-1]


def test_translate_never_rotates():
    """
    A translation is a pure translation.

    The previous model held heading with a live omega term during the run.
    That is two motions at once, which this model excludes: drift is corrected
    by dropping back into a rotate-only phase instead.
    """
    for primitive in ('FORWARD', 'STRAFE_LEFT', 'DIAGONAL_FRONT_LEFT'):
        sent = _tick_once(TRANSLATE, (0.0, 0.0, 0.1), (2.0, 0.0, 0.0),
                          primitive=primitive, target_heading=0.0)
        assert sent[-1][2] == 0.0, (primitive, sent[-1])


def test_each_primitive_commands_only_its_own_motion():
    """Each of the eight directions produces exactly the axes it should."""
    checks = {
        'FORWARD': lambda vx, vy: vx > 0 and abs(vy) < 1e-9,
        'BACKWARD': lambda vx, vy: vx < 0 and abs(vy) < 1e-9,
        'STRAFE_LEFT': lambda vx, vy: abs(vx) < 1e-9 and vy > 0,
        'STRAFE_RIGHT': lambda vx, vy: abs(vx) < 1e-9 and vy < 0,
    }
    for name, ok in checks.items():
        sent = _tick_once(TRANSLATE, (0.0, 0.0, 0.0), (5.0, 0.0, 0.0),
                          primitive=name, target_heading=0.0)
        vx, vy, omega = sent[-1]
        assert ok(vx, vy), (name, sent[-1])
        assert omega == 0.0, (name, sent[-1])


def test_diagonals_are_true_diagonals():
    """|vx| equals |vy| on a diagonal - not a forward run with strafe mixed in."""
    diagonals = {
        'DIAGONAL_FRONT_LEFT': (1, 1),
        'DIAGONAL_FRONT_RIGHT': (1, -1),
        'DIAGONAL_BACK_LEFT': (-1, 1),
        'DIAGONAL_BACK_RIGHT': (-1, -1),
    }
    for name, (sx, sy) in diagonals.items():
        sent = _tick_once(TRANSLATE, (0.0, 0.0, 0.0), (5.0, 0.0, 0.0),
                          primitive=name, target_heading=0.0)
        vx, vy, omega = sent[-1]
        assert abs(abs(vx) - abs(vy)) < 1e-9, (name, sent[-1])
        assert vx * sx > 0.0 and vy * sy > 0.0, (name, sent[-1])
        assert omega == 0.0, (name, sent[-1])


def test_primitive_chosen_needs_the_least_rotation():
    """A goal off the shoulder is a diagonal run, not a square-up and strafe."""
    rclpy.init()
    controller = StagedPoseController()
    try:
        # Robot faces +x. Goal at 45 degrees to its left: driving diagonally
        # needs no rotation at all, where forward would need 45 degrees.
        name, offset, heading = controller._choose_primitive(
            math.pi / 4.0, 0.0)
        assert name == 'DIAGONAL_FRONT_LEFT', name
        assert abs(normalize(heading - 0.0)) < 1e-9, heading

        # Goal straight ahead: forward, no rotation.
        name, _, _ = controller._choose_primitive(0.0, 0.0)
        assert name == 'FORWARD', name

        # Goal directly to the left: strafing needs no rotation.
        name, _, _ = controller._choose_primitive(math.pi / 2.0, 0.0)
        assert name == 'STRAFE_LEFT', name

        # Goal behind: backward, rather than a 180 degree turn.
        name, _, _ = controller._choose_primitive(math.pi, 0.0)
        assert name == 'BACKWARD', name

        # Every choice must need at most half the 45 degree spacing.
        for bearing in [i * 0.17 for i in range(-20, 21)]:
            _, _, heading = controller._choose_primitive(bearing, 0.3)
            assert abs(normalize(heading - 0.3)) <= math.pi / 8.0 + 1e-9
    finally:
        controller.destroy_node()
        rclpy.shutdown()


def test_diagonals_can_be_switched_off():
    """With use_diagonals false only the four axial motions are offered."""
    rclpy.init()
    controller = StagedPoseController()
    try:
        controller.set_parameters(
            [rclpy.parameter.Parameter(
                'use_diagonals', rclpy.Parameter.Type.BOOL, False)])
        names = {name for name, _ in controller._primitive_set()}
        assert names == {'FORWARD', 'BACKWARD', 'STRAFE_LEFT',
                         'STRAFE_RIGHT'}, names
        name, _, _ = controller._choose_primitive(math.pi / 4.0, 0.0)
        assert name in ('FORWARD', 'STRAFE_LEFT'), name
    finally:
        controller.destroy_node()
        rclpy.shutdown()


def test_commands_are_clamped_to_the_limits():
    """A far goal must not command more than max_linear_vel."""
    for name in OFFSETS:
        sent = _tick_once(TRANSLATE, (0.0, 0.0, 0.0), (100.0, 0.0, 0.0),
                          primitive=name, target_heading=0.0)
        vx, vy, _ = sent[-1]
        assert math.hypot(vx, vy) <= 0.15 + 1e-9, (name, sent[-1])


def test_stops_when_pose_becomes_unavailable():
    """A missing transform must hold zero, not coast on the last command."""
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
    # The fake must stop broadcasting, or the transform is refreshed 20 ms
    # later and the pose is never actually unavailable.
    robot.paused = True
    time.sleep(1.5)
    robot.commands.clear()
    time.sleep(0.5)

    assert robot.commands, 'controller stopped publishing entirely'
    assert all(v == (0.0, 0.0, 0.0) for v in robot.commands), robot.commands

    executor.shutdown()
    controller.destroy_node()
    robot.destroy_node()
    rclpy.shutdown()
