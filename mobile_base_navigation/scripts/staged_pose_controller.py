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
Reach a pose using exactly one motion primitive at a time.

The robot never blends axes. Every command is one of:

  - a pure rotation in place, or
  - a pure translation along one of eight body-frame directions: forward,
    backward, strafe left, strafe right, and the four true diagonals.

A diagonal is a real diagonal - |vx| equals |vy| - not a forward run with a
strafe mixed into it.

Reaching a goal is therefore: rotate until the goal lies exactly along one of
those eight directions, translate along it, then rotate to the requested final
orientation. The heading chosen for the translation is whichever of the eight
needs the least rotation from where the robot already points, which is what
puts the diagonals to work rather than leaving them decorative: a goal off the
robot's shoulder is reached with a 45 degree turn and a diagonal run instead of
a 90 degree turn and a strafe.

Heading drift during a translation is corrected by stopping and rotating, not
by mixing a correction term into the translation. A blended correction would be
two motions at once, which is exactly what this model excludes. The machine
drops back into ALIGN, re-picks the primitive for the new bearing, and resumes.

This is a straight-line primitive with no planner and no costmap: it drives at
the goal and will hit anything in between. It publishes cmd_vel_nav, the same
input Nav2's controller_server uses, so the velocity smoother, the collision
monitor, arbitration and the emergency stop all still apply - but the collision
monitor only limits speed, it does not route around anything.
"""

import math
import threading

from geometry_msgs.msg import PoseStamped, TwistStamped

from mobile_base_interfaces.action import GoToPose

from mobile_base_tools.pose_math import normalize_yaw, quaternion_to_yaw

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import tf2_ros
from tf2_ros import Buffer, TransformListener

IDLE = 'IDLE'
ALIGN = 'ALIGN'
TRANSLATE = 'TRANSLATE'
ORIENT = 'ORIENT'
DONE = 'DONE'

QUARTER = math.pi / 2.0
EIGHTH = math.pi / 4.0

# The eight permitted translation directions, as body-frame angles. Driving
# along one of these is a single motion; anything between them would need two
# blended together, which this model does not do.
PRIMITIVES = (
    ('FORWARD', 0.0),
    ('DIAGONAL_FRONT_LEFT', EIGHTH),
    ('STRAFE_LEFT', QUARTER),
    ('DIAGONAL_BACK_LEFT', 3.0 * EIGHTH),
    ('BACKWARD', math.pi),
    ('DIAGONAL_BACK_RIGHT', -3.0 * EIGHTH),
    ('STRAFE_RIGHT', -QUARTER),
    ('DIAGONAL_FRONT_RIGHT', -EIGHTH),
)
OFFSETS = dict(PRIMITIVES)
# Subset used when diagonals are switched off.
AXIAL_ONLY = ('FORWARD', 'STRAFE_LEFT', 'BACKWARD', 'STRAFE_RIGHT')

# Below this separation the direction to the goal is numerically meaningless,
# so there is nothing to align to.
MIN_ALIGN_DISTANCE = 1e-3


def _clamp(value, limit):
    """Clamp a scalar to +/- limit."""
    return max(-limit, min(limit, value))


class Pid:
    """A PID with integral clamping."""

    def __init__(self, kp, ki, kd, integral_limit):
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._integral_limit = integral_limit
        self.reset()

    def reset(self):
        """
        Clear the integral and derivative history.

        Called on every state transition. Without it, the integral wound up
        while closing one phase's error keeps pushing during the next, which
        shows up as an overshoot immediately after a transition.
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
        return (self._kp * error
                + self._ki * self._integral
                + self._kd * derivative)


class StagedPoseController(Node):
    """Drive to a pose using one motion primitive at a time."""

    def __init__(self):
        super().__init__('staged_pose_controller')
        self._declare_parameters()

        self._lock = threading.Lock()
        self._state = IDLE
        self._state_entered = None
        self._goal = None
        # The primitive chosen for the current translation and the heading it
        # requires. Both are recomputed on every entry to ALIGN.
        self._primitive = None
        self._target_heading = None
        self._active_handle = None
        self._cancelled = False
        self._settle_until = 0.0
        self._loops = self._gains()

        group = ReentrantCallbackGroup()
        # Pose comes from TF, not from odometry. Goals arrive in the map
        # frame, and /odometry/filtered is in the odom frame - the two differ
        # by AMCL's correction, which grows as the robot drives. Comparing a
        # map-frame goal against an odom-frame pose silently drives the robot
        # to the wrong place once that correction becomes non-trivial.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._commands = self.create_publisher(
            TwistStamped, self.get_parameter('command_topic').value, 10)
        self._server = ActionServer(
            self, GoToPose, 'go_to_pose',
            execute_callback=self._execute,
            goal_callback=self._on_goal_request,
            cancel_callback=self._on_cancel,
            callback_group=group)
        # RViz's 2D Goal Pose tool publishes here. Accepting it keeps the
        # click-to-drive workflow working with no bt_navigator present.
        self._goal_client = ActionClient(
            self, GoToPose, 'go_to_pose', callback_group=group)
        self.create_subscription(
            PoseStamped, self.get_parameter('goal_topic').value,
            self._on_goal_pose, 1, callback_group=group)

        period = 1.0 / float(self.get_parameter('control_frequency').value)
        self.create_timer(period, self._tick, callback_group=group)
        self.get_logger().info(
            'Discrete motion controller ready: one primitive at a time. '
            'Click a goal in RViz, or send one to go_to_pose.')

    def _declare_parameters(self):
        """Declare every tunable, with defaults sized to this robot."""
        self.declare_parameter('Kp_align', 1.2)
        self.declare_parameter('Ki_align', 0.0)
        self.declare_parameter('Kd_align', 0.05)
        self.declare_parameter('Kp_translate_pos', 0.9)
        self.declare_parameter('Ki_translate_pos', 0.0)
        self.declare_parameter('Kd_translate_pos', 0.05)
        self.declare_parameter('Kp_orient', 1.2)
        self.declare_parameter('Ki_orient', 0.0)
        self.declare_parameter('Kd_orient', 0.05)

        self.declare_parameter('angle_tolerance_rad', 0.05)
        self.declare_parameter('position_tolerance_m', 0.05)
        self.declare_parameter('max_linear_vel', 0.15)
        self.declare_parameter('max_angular_vel', 0.60)
        self.declare_parameter('min_state_dwell_time_s', 0.3)
        # Zero-command hold when the motion type changes, so the downstream
        # smoother finishes ramping the old motion down before the new one
        # starts. Must exceed one smoother period (1/20 s) with margin.
        self.declare_parameter('settle_time_s', 0.4)

        # Include the four diagonals. With this off the robot only drives
        # along its own axes, which needs more rotation to reach the same goal.
        self.declare_parameter('use_diagonals', True)
        # Heading drift that sends the machine back to ALIGN mid-translation.
        # Larger than angle_tolerance_rad so a run is not abandoned the instant
        # it leaves perfect alignment. Correcting by rotating is what keeps one
        # motion at a time true; a blended correction would break it.
        self.declare_parameter('realign_threshold_rad', 0.15)

        self.declare_parameter('integral_limit', 0.5)
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('goal_timeout_s', 180.0)
        self.declare_parameter('command_topic', 'cmd_vel_nav')
        self.declare_parameter('goal_topic', '/goal_pose')
        # Goals are expressed in this frame, and the robot's pose is looked up
        # in it. Must match the frame RViz publishes goals in.
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('frame_id', 'base_link')
        # How stale a transform may be before the controller stops.
        self.declare_parameter('transform_timeout_s', 0.5)

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

    def _primitive_set(self):
        """Return the permitted primitives for the current parameters."""
        if bool(self.get_parameter('use_diagonals').value):
            return PRIMITIVES
        return tuple(p for p in PRIMITIVES if p[0] in AXIAL_ONLY)

    def _choose_primitive(self, bearing, theta):
        """
        Pick the primitive whose heading is closest to where the robot points.

        For each candidate direction the robot would have to face
        `bearing - offset` for the goal to lie exactly along it. Choosing the
        smallest required turn is what makes the diagonals earn their place.
        """
        best = None
        for name, offset in self._primitive_set():
            heading = normalize_yaw(bearing - offset)
            turn = abs(normalize_yaw(heading - theta))
            if best is None or turn < best[0]:
                best = (turn, name, offset, heading)
        return best[1], best[2], best[3]

    def _pose(self):
        """
        Return (x, y, yaw) of the robot in the goal frame, or None.

        None means the transform is missing or stale, which the caller must
        treat as a fault and stop - never as "carry on with the last pose".
        """
        goal_frame = str(self.get_parameter('goal_frame').value)
        base_frame = str(self.get_parameter('robot_base_frame').value)
        try:
            transform = self._tf_buffer.lookup_transform(
                goal_frame, base_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, tf2_ros.TransformException):
            return None
        stamp = transform.header.stamp
        age = self._now() - (stamp.sec + stamp.nanosec / 1e9)
        timeout = float(self.get_parameter('transform_timeout_s').value)
        # A zero stamp means the source never filled it in; treat as fresh
        # rather than as infinitely old, since static transforms do this.
        if stamp.sec != 0 and age > timeout:
            return None
        try:
            yaw = quaternion_to_yaw(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            )
        except ValueError:
            return None
        return (transform.transform.translation.x,
                transform.transform.translation.y,
                yaw)

    def _on_goal_pose(self, message):
        """Turn an RViz goal click into a goal on this node's own server."""
        try:
            yaw = quaternion_to_yaw(
                message.pose.orientation.x, message.pose.orientation.y,
                message.pose.orientation.z, message.pose.orientation.w)
        except ValueError as error:
            self.get_logger().warn(f'Ignoring goal click: {error}')
            return
        if not self._goal_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Own action server is unavailable')
            return
        goal = GoToPose.Goal()
        goal.x = message.pose.position.x
        goal.y = message.pose.position.y
        goal.theta_final = yaw
        self._goal_client.send_goal_async(goal)

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
        # Hold zero briefly when the motion type changes. The velocity
        # smoother ramps between commands, so switching straight from a
        # translation to a rotation puts a blended twist on the wire for a few
        # ticks - two motions at once, which is what this model excludes.
        # Letting the old motion ramp to zero first keeps the rule true at
        # the wheels and not merely at this node's output.
        if previous != state and {previous, state} != {IDLE, DONE}:
            self._settle_until = self._now() + float(
                self.get_parameter('settle_time_s').value)
        for loop in self._loops:
            loop.reset()
        if previous != state:
            self.get_logger().info(f'{previous} -> {state}')

    def _dwelled(self):
        """Return whether the current state has been held long enough."""
        dwell = float(self.get_parameter('min_state_dwell_time_s').value)
        return (self._state_entered is not None
                and self._now() - self._state_entered >= dwell)

    def _execute(self, goal_handle):
        """Run one goal to completion, cancellation or timeout."""
        request = goal_handle.request
        self._loops = self._gains()
        with self._lock:
            self._goal = (request.x, request.y,
                          normalize_yaw(request.theta_final))
            self._active_handle = goal_handle
            self._cancelled = False
            self._primitive = None
            self._target_heading = None
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
        pose = self._pose()
        with self._lock:
            goal = self._goal
        if pose is None or goal is None:
            result.final_position_error = float('nan')
            result.final_heading_error = float('nan')
            return
        result.final_position_error = math.hypot(
            goal[0] - pose[0], goal[1] - pose[1])
        result.final_heading_error = abs(normalize_yaw(goal[2] - pose[2]))

    def _publish_feedback(self, goal_handle):
        pose = self._pose()
        with self._lock:
            goal = self._goal
            state, primitive = self._state, self._primitive
            target = self._target_heading
        if pose is None or goal is None:
            return
        feedback = GoToPose.Feedback()
        # The primitive rides in the state string so the caller can see which
        # of the eight motions is running without a new interface field.
        feedback.state = (f'{state}[{primitive}]'
                          if primitive and state in (ALIGN, TRANSLATE)
                          else state)
        feedback.distance_remaining = math.hypot(
            goal[0] - pose[0], goal[1] - pose[1])
        if state == ORIENT:
            feedback.heading_error = normalize_yaw(goal[2] - pose[2])
        elif target is not None:
            feedback.heading_error = normalize_yaw(target - pose[2])
        else:
            feedback.heading_error = 0.0
        goal_handle.publish_feedback(feedback)

    def _finish(self, state):
        """Leave the machine idle and the robot stopped."""
        with self._lock:
            self._state = state
            self._active_handle = None
            self._goal = None
            self._primitive = None
            self._target_heading = None
        self._stop()

    def _tick(self):
        """Close the loop once. Everything that commands motion is here."""
        with self._lock:
            state = self._state
            goal = self._goal
            cancelled = self._cancelled
        pose = self._pose()

        if state in (IDLE, DONE) or goal is None:
            return
        if cancelled:
            self._stop()
            return

        if pose is None:
            # A missing or stale transform is the one fault that must not
            # coast. Holding the last pose would drive on stale information.
            self.get_logger().warn(
                'Robot pose unavailable or stale; holding zero.',
                throttle_duration_sec=2.0)
            self._stop()
            return

        if self._now() < self._settle_until:
            # Ramping the previous motion down. Nothing new is commanded yet.
            self._stop()
            return

        dt = 1.0 / float(self.get_parameter('control_frequency').value)
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
            if distance <= max(position_tolerance, MIN_ALIGN_DISTANCE):
                # Already there; there is no direction left to point at.
                self._transition(ORIENT)
                self._stop()
                return
            bearing = math.atan2(dy, dx)
            with self._lock:
                held = self._primitive
            if held is None:
                # New leg: pick the cheapest primitive for this bearing.
                name, offset, heading = self._choose_primitive(bearing, theta)
            else:
                # Mid-leg drift correction. Re-picking here is what made the
                # robot oscillate: near a 45 degree boundary a small heading
                # change flips the cheapest choice, so it turned one way,
                # drifted, flipped, and turned back. Keep the leg's primitive
                # and rotate to the heading this bearing now needs for it.
                name = held
                offset = OFFSETS[name]
                heading = normalize_yaw(bearing - offset)
            with self._lock:
                self._primitive = name
                self._target_heading = heading
            error = normalize_yaw(heading - theta)
            # Pure rotation. No linear terms at all.
            omega = _clamp(align.step(error, dt), max_angular)
            self._publish(0.0, 0.0, omega)
            if abs(error) <= angle_tolerance and self._dwelled():
                self.get_logger().info(
                    f'Translating {name} ({math.degrees(offset):+.0f} deg in '
                    f'the body frame), {distance:.3f} m to run')
                self._transition(TRANSLATE)
            return

        if state == TRANSLATE:
            if distance <= position_tolerance and self._dwelled():
                with self._lock:
                    # The leg is finished; the next one picks afresh.
                    self._primitive = None
                    self._target_heading = None
                self._transition(ORIENT)
                self._stop()
                return
            with self._lock:
                name, target = self._primitive, self._target_heading
            if target is None or name is None:
                self._transition(ALIGN)
                self._stop()
                return
            drift = abs(normalize_yaw(target - theta))
            threshold = float(
                self.get_parameter('realign_threshold_rad').value)
            if drift > threshold and self._dwelled():
                # Correct by rotating, not by mixing omega into the
                # translation. Re-entering ALIGN also re-picks the primitive
                # for the bearing from wherever the robot has actually got to.
                self.get_logger().info(
                    f'Heading drifted {drift:.3f} rad; re-aligning')
                self._transition(ALIGN)
                self._stop()
                return
            offset = OFFSETS[name]
            # One primitive only: the direction is fixed, and the PID sets how
            # fast to run along it so the robot decelerates into the goal.
            speed = _clamp(position.step(distance, dt), max_linear)
            self._publish(
                speed * math.cos(offset), speed * math.sin(offset), 0.0)
            return

        if state == ORIENT:
            error = normalize_yaw(goal_theta - theta)
            # Pure rotation. No linear terms at all.
            omega = _clamp(orient.step(error, dt), max_angular)
            self._publish(0.0, 0.0, omega)
            if abs(error) <= angle_tolerance and self._dwelled():
                self._transition(DONE)
                self._stop()
            return


def main(args=None):
    """Run the discrete motion controller."""
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
