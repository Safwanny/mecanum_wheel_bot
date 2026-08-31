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
Drive to a pose in three sequential phases instead of blending every axis.

ALIGN rotates in place to face the goal, TRANSLATE moves holonomically toward
it while a light secondary loop holds that heading, and ORIENT rotates in place
to the requested final orientation. The phases are explicit states rather than
one combined controller, which makes each one separately observable and
tunable.

This is a straight-line primitive: it has no planner and no costmap, so it will
drive into anything between the robot and the goal. It publishes into
cmd_vel_nav, the same input Nav2's controller_server uses, so it inherits the
velocity smoother, the collision monitor, arbitration and the emergency stop
rather than reaching the wheels directly. That also means it and Nav2's
controller must not run together - planning.launch.py runs one or the other.
"""

import math
import threading

from geometry_msgs.msg import TwistStamped

from mobile_base_interfaces.action import GoToPose

from mobile_base_tools.pose_math import (
    data_is_stale,
    normalize_yaw,
    quaternion_to_yaw,
)

from nav_msgs.msg import Odometry

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

IDLE = 'IDLE'
ALIGN = 'ALIGN'
TRANSLATE = 'TRANSLATE'
ORIENT = 'ORIENT'
DONE = 'DONE'

# Below this separation the direction to the goal is numerically meaningless,
# so ALIGN has nothing to point at and is skipped.
MIN_ALIGN_DISTANCE = 1e-3


def _clamp(value, limit):
    """Clamp a scalar to +/- limit."""
    return max(-limit, min(limit, value))


class Pid:
    """A PID with integral clamping, usable on a scalar or a 2D vector."""

    def __init__(self, kp, ki, kd, integral_limit):
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._integral_limit = integral_limit
        self.reset()

    def reset(self):
        """
        Clear the integral and derivative history.

        Called on every state transition. Without it the integral wound up
        while closing one phase's error keeps pushing during the next, which
        on this controller shows up as an overshoot right after a transition.
        """
        self._integral = 0.0
        self._previous = None

    def step(self, error, dt):
        """Advance the loop by dt seconds and return the command."""
        if dt <= 0.0:
            return 0.0
        self._integral = _clamp(
            self._integral + error * dt, self._integral_limit)
        derivative = 0.0
        if self._previous is not None:
            derivative = (error - self._previous) / dt
        self._previous = error
        return self._kp * error + self._ki * self._integral + self._kd * derivative


class StagedPoseController(Node):
    """Run the ALIGN, TRANSLATE and ORIENT state machine for one goal."""

    def __init__(self):
        super().__init__('staged_pose_controller')
        self._declare_parameters()

        self._lock = threading.Lock()
        self._state = IDLE
        self._state_entered = None
        self._odometry = None
        self._odometry_time = None
        self._goal = None
        self._hold_heading = None
        self._active_handle = None
        self._cancelled = False
        # Built here as well as per goal, so _tick and _transition are safe
        # regardless of call order. _execute rebuilds them so a parameter
        # change between goals takes effect.
        self._loops = self._gains()

        group = ReentrantCallbackGroup()
        self._commands = self.create_publisher(
            TwistStamped, self.get_parameter('command_topic').value, 10)
        self.create_subscription(
            Odometry, self.get_parameter('odometry_topic').value,
            self._on_odometry, 20, callback_group=group)
        self._server = ActionServer(
            self, GoToPose, 'go_to_pose',
            execute_callback=self._execute,
            goal_callback=self._on_goal_request,
            cancel_callback=self._on_cancel,
            callback_group=group)

        period = 1.0 / float(self.get_parameter('control_frequency').value)
        self.create_timer(period, self._tick, callback_group=group)
        self.get_logger().info(
            'Staged pose controller ready. Send a goal to '
            f'{self.get_namespace().rstrip("/")}/go_to_pose'
        )

    def _declare_parameters(self):
        """Declare every tunable, with defaults sized to this robot."""
        # Gains. Defaults are deliberately gentle: the robot's characterised
        # envelope is 0.10 m/s and 0.30 rad/s, and the downstream smoother
        # limits acceleration anyway, so there is nothing to gain from
        # aggressive proportional terms here.
        self.declare_parameter('Kp_align', 1.2)
        self.declare_parameter('Ki_align', 0.0)
        self.declare_parameter('Kd_align', 0.05)
        self.declare_parameter('Kp_translate_pos', 0.9)
        self.declare_parameter('Ki_translate_pos', 0.0)
        self.declare_parameter('Kd_translate_pos', 0.05)
        # Secondary heading hold during TRANSLATE. A real correction term, not
        # a zero: mecanum lateral odometry drifts, and without this the robot
        # slowly yaws while strafing.
        self.declare_parameter('Kp_translate_heading', 0.8)
        self.declare_parameter('Kp_orient', 1.2)
        self.declare_parameter('Ki_orient', 0.0)
        self.declare_parameter('Kd_orient', 0.05)

        self.declare_parameter('angle_tolerance_rad', 0.05)
        self.declare_parameter('position_tolerance_m', 0.05)
        # Bounds, not targets. Kept inside the MPPI limits the rest of the
        # stack uses so this controller cannot command anything the other one
        # would not.
        self.declare_parameter('max_linear_vel', 0.15)
        self.declare_parameter('max_angular_vel', 0.60)
        # Hysteresis. Tolerance noise around the boundary would otherwise let
        # the machine flap between phases every tick.
        self.declare_parameter('min_state_dwell_time_s', 0.3)
        self.declare_parameter('blend_transitions', False)
        # How far out, in multiples of the tolerance, the next phase starts
        # ramping in when blending is enabled.
        self.declare_parameter('blend_window_scale', 3.0)
        self.declare_parameter('integral_limit', 0.5)

        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('odometry_timeout_s', 0.5)
        self.declare_parameter('goal_timeout_s', 120.0)
        # Feeds the same input Nav2's controller_server uses, so the smoother,
        # collision monitor, mux and e-stop all still apply.
        self.declare_parameter('command_topic', 'cmd_vel_nav')
        self.declare_parameter('odometry_topic', '/odometry/filtered')
        self.declare_parameter('frame_id', 'base_link')

    def _gains(self):
        """Build the three loops from the current parameter values."""
        limit = float(self.get_parameter('integral_limit').value)

        def value(name):
            return float(self.get_parameter(name).value)

        return (
            Pid(value('Kp_align'), value('Ki_align'), value('Kd_align'), limit),
            Pid(value('Kp_translate_pos'), value('Ki_translate_pos'),
                value('Kd_translate_pos'), limit),
            Pid(value('Kp_orient'), value('Ki_orient'), value('Kd_orient'),
                limit),
        )

    def _on_odometry(self, message):
        """Latch the newest pose and the time it arrived."""
        try:
            yaw = quaternion_to_yaw(
                message.pose.pose.orientation.x,
                message.pose.pose.orientation.y,
                message.pose.pose.orientation.z,
                message.pose.pose.orientation.w,
            )
        except ValueError as error:
            self.get_logger().warn(f'Ignoring odometry: {error}')
            return
        with self._lock:
            self._odometry = (
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                yaw,
            )
            self._odometry_time = self._now()

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _on_goal_request(self, goal_request):
        """Accept one goal at a time."""
        if not all(math.isfinite(v) for v in (
                goal_request.x, goal_request.y, goal_request.theta_final)):
            self.get_logger().error('Rejecting goal: non-finite value')
            return GoalResponse.REJECT
        with self._lock:
            busy = self._active_handle is not None
        if busy:
            self.get_logger().warn('Rejecting goal: one is already running')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        """Accept cancellation; the tick loop stops the robot."""
        del goal_handle
        with self._lock:
            self._cancelled = True
        return CancelResponse.ACCEPT

    def _publish(self, vx, vy, omega):
        """Send one body-frame twist downstream."""
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(self.get_parameter('frame_id').value)
        message.twist.linear.x = vx
        message.twist.linear.y = vy
        message.twist.angular.z = omega
        self._commands.publish(message)

    def _stop(self):
        """Hold zero. Used on cancel, fault, completion and while idle."""
        self._publish(0.0, 0.0, 0.0)

    def _transition(self, state):
        """Enter a state, resetting every integral term."""
        previous = self._state
        self._state = state
        self._state_entered = self._now()
        for loop in self._loops:
            loop.reset()
        if previous != state:
            self.get_logger().info(f'{previous} -> {state}')

    def _dwelled(self):
        """Return whether the current state has been held long enough."""
        dwell = float(self.get_parameter('min_state_dwell_time_s').value)
        return (self._state_entered is not None
                and self._now() - self._state_entered >= dwell)

    def _blend(self, error, tolerance):
        """
        Ramp 0 to 1 as the error closes on its tolerance.

        Returns 1.0 when blending is disabled, so callers can multiply
        unconditionally. Only used to fade the *next* phase in early; the
        current phase is never faded out, so the robot is never left
        uncommanded mid-transition.
        """
        if not bool(self.get_parameter('blend_transitions').value):
            return 0.0
        scale = float(self.get_parameter('blend_window_scale').value)
        window = tolerance * max(scale, 1.0)
        if window <= tolerance:
            return 0.0
        if abs(error) >= window:
            return 0.0
        if abs(error) <= tolerance:
            return 1.0
        return (window - abs(error)) / (window - tolerance)

    def _execute(self, goal_handle):
        """Run one goal to completion, cancellation or timeout."""
        request = goal_handle.request
        self._loops = self._gains()
        with self._lock:
            self._goal = (request.x, request.y,
                          normalize_yaw(request.theta_final))
            self._active_handle = goal_handle
            self._cancelled = False
            self._hold_heading = None
        self._transition(ALIGN)
        self.get_logger().info(
            f'Goal ({request.x:.3f}, {request.y:.3f}, '
            f'{normalize_yaw(request.theta_final):.3f} rad)')

        started = self._now()
        timeout = float(self.get_parameter('goal_timeout_s').value)
        result = GoToPose.Result()

        while rclpy.ok():
            with self._lock:
                state = self._state
                cancelled = self._cancelled
            if cancelled:
                # Errors must be read before _finish, which clears the goal.
                self._fill_errors(result)
                self._finish(DONE)
                goal_handle.canceled()
                result.success = False
                result.message = 'cancelled by request'
                return result
            if state == DONE:
                break
            if self._now() - started > timeout:
                self._fill_errors(result)
                self._finish(DONE)
                goal_handle.abort()
                result.success = False
                result.message = f'timed out after {timeout:.0f} s'
                return result
            self._publish_feedback(goal_handle)
            self._sleep()

        self._fill_errors(result)
        self._finish(DONE)
        goal_handle.succeed()
        result.success = True
        result.message = 'goal reached'
        return result

    def _sleep(self):
        rate = float(self.get_parameter('control_frequency').value)
        self.get_clock().sleep_for(
            rclpy.duration.Duration(seconds=1.0 / rate))

    def _fill_errors(self, result):
        """Record how close the robot actually ended up."""
        with self._lock:
            pose, goal = self._odometry, self._goal
        if pose is None or goal is None:
            result.final_position_error = float('nan')
            result.final_heading_error = float('nan')
            return
        result.final_position_error = math.hypot(
            goal[0] - pose[0], goal[1] - pose[1])
        result.final_heading_error = abs(normalize_yaw(goal[2] - pose[2]))

    def _publish_feedback(self, goal_handle):
        with self._lock:
            pose, goal, state = self._odometry, self._goal, self._state
        if pose is None or goal is None:
            return
        feedback = GoToPose.Feedback()
        feedback.state = state
        feedback.distance_remaining = math.hypot(
            goal[0] - pose[0], goal[1] - pose[1])
        if state == ORIENT:
            feedback.heading_error = normalize_yaw(goal[2] - pose[2])
        elif state == TRANSLATE and self._hold_heading is not None:
            feedback.heading_error = normalize_yaw(
                self._hold_heading - pose[2])
        else:
            feedback.heading_error = normalize_yaw(
                math.atan2(goal[1] - pose[1], goal[0] - pose[0]) - pose[2])
        goal_handle.publish_feedback(feedback)

    def _finish(self, state):
        """Leave the machine idle and the robot stopped."""
        with self._lock:
            self._state = state
            self._active_handle = None
            self._goal = None
        self._stop()

    def _tick(self):
        """Close the loop once. Everything that commands motion is here."""
        with self._lock:
            state = self._state
            pose = self._odometry
            goal = self._goal
            odometry_time = self._odometry_time
            cancelled = self._cancelled

        if state in (IDLE, DONE) or goal is None:
            return
        if cancelled:
            self._stop()
            return

        timeout = float(self.get_parameter('odometry_timeout_s').value)
        if pose is None or odometry_time is None or data_is_stale(
                odometry_time, self._now(), timeout):
            # A dead odometry source is the one fault that must not coast.
            self.get_logger().warn(
                'Odometry stale or missing; holding zero.',
                throttle_duration_sec=2.0)
            self._stop()
            return

        rate = float(self.get_parameter('control_frequency').value)
        dt = 1.0 / rate
        align, position, orient = self._loops
        angle_tolerance = float(self.get_parameter('angle_tolerance_rad').value)
        position_tolerance = float(
            self.get_parameter('position_tolerance_m').value)
        max_linear = float(self.get_parameter('max_linear_vel').value)
        max_angular = float(self.get_parameter('max_angular_vel').value)

        x, y, theta = pose
        goal_x, goal_y, goal_theta = goal
        dx, dy = goal_x - x, goal_y - y
        distance = math.hypot(dx, dy)

        if state == ALIGN:
            if distance < MIN_ALIGN_DISTANCE:
                # Already on the goal position; there is no direction to face.
                self._transition(ORIENT)
                self._stop()
                return
            bearing = math.atan2(dy, dx)
            error = normalize_yaw(bearing - theta)
            omega = _clamp(align.step(error, dt), max_angular)
            blend = self._blend(error, angle_tolerance)
            vx, vy = 0.0, 0.0
            if blend > 0.0:
                vx, vy = self._translate_command(
                    dx, dy, theta, position, dt, max_linear)
                vx, vy = vx * blend, vy * blend
            self._publish(vx, vy, omega)
            if abs(error) <= angle_tolerance and self._dwelled():
                with self._lock:
                    self._hold_heading = bearing
                self._transition(TRANSLATE)
            return

        if state == TRANSLATE:
            vx, vy = self._translate_command(
                dx, dy, theta, position, dt, max_linear)
            # Secondary heading hold. Without this the base yaws away while
            # strafing and the "straight line" stops being straight.
            hold = self._hold_heading if self._hold_heading is not None else theta
            heading_error = normalize_yaw(hold - theta)
            gain = float(self.get_parameter('Kp_translate_heading').value)
            omega = _clamp(gain * heading_error, max_angular)
            blend = self._blend(distance, position_tolerance)
            if blend > 0.0:
                final_error = normalize_yaw(goal_theta - theta)
                omega += blend * _clamp(
                    orient.step(final_error, dt), max_angular)
                omega = _clamp(omega, max_angular)
            self._publish(vx, vy, omega)
            if distance <= position_tolerance and self._dwelled():
                self._transition(ORIENT)
            return

        if state == ORIENT:
            error = normalize_yaw(goal_theta - theta)
            omega = _clamp(orient.step(error, dt), max_angular)
            self._publish(0.0, 0.0, omega)
            if abs(error) <= angle_tolerance and self._dwelled():
                self._transition(DONE)
                self._stop()
            return

    def _translate_command(self, dx, dy, theta, loop, dt, max_linear):
        """Rotate the world-frame error into the body frame and run the PID."""
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        error_x = cos_t * dx + sin_t * dy
        error_y = -sin_t * dx + cos_t * dy
        magnitude = math.hypot(error_x, error_y)
        if magnitude <= 0.0:
            return 0.0, 0.0
        # One position PID on the error magnitude, applied along the body-frame
        # direction, so both axes share a single integral and a single speed
        # limit rather than two loops fighting over one budget.
        speed = loop.step(magnitude, dt)
        speed = max(-max_linear, min(max_linear, speed))
        return speed * error_x / magnitude, speed * error_y / magnitude


def main(args=None):
    """Run the staged pose controller."""
    rclpy.init(args=args)
    node = StagedPoseController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, RuntimeError):
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == '__main__':
    main()
