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
Latching emergency stop, held as a twist_mux lock.

The lock sits at a priority above every command source, so engaging it masks
autonomy and teleop alike. Engage and reset are services rather than topics so
that the caller gets a confirmation, and the stop latches: it clears only when
a human asks, never because the triggering condition went away on its own.

The heartbeat is the fail-safe half. twist_mux treats a lock whose timeout has
elapsed as locked:

    bool isLocked() const { return hasExpired() || getMessage().data; }

so if this node dies, stops publishing, or is partitioned away, the lock
engages by itself. Publishing false continuously is what permits motion, which
is why the publish rate must stay comfortably inside the lock timeout.

Reset restores permission, not motion. Nothing here replays a command, and the
robot stays stationary until a fresh goal or teleop input arrives.
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool

from std_srvs.srv import Trigger


class EstopGate(Node):
    """Publish the twist_mux lock heartbeat and latch the stop state."""

    def __init__(self):
        super().__init__('estop_gate')
        self.declare_parameter('publish_frequency', 10.0)
        self.declare_parameter('lock_topic', 'safety/estop_active')
        self.declare_parameter('start_engaged', True)

        frequency = self.get_parameter('publish_frequency').value
        topic = self.get_parameter('lock_topic').value
        self._engaged = bool(self.get_parameter('start_engaged').value)
        self._reason = 'startup' if self._engaged else ''

        self._lock = self.create_publisher(Bool, topic, 1)
        self.create_timer(1.0 / frequency, self._publish)
        self.create_service(Trigger, '~/engage', self._on_engage)
        self.create_service(Trigger, '~/reset', self._on_reset)

        self.get_logger().warn(
            f'E-stop {"ENGAGED" if self._engaged else "clear"} at startup. '
            'Clear it with: ros2 service call /estop_gate/reset '
            'std_srvs/srv/Trigger'
        )

    def _publish(self):
        """Heartbeat the lock. Silence here is what stops the robot."""
        self._lock.publish(Bool(data=self._engaged))

    def _on_engage(self, request, response):
        """Latch the stop."""
        del request
        if not self._engaged:
            self._engaged = True
            self._reason = 'operator request'
            self.get_logger().error('E-STOP ENGAGED: operator request')
        response.success = True
        response.message = 'e-stop engaged'
        return response

    def _on_reset(self, request, response):
        """Clear the latch, restoring permission but not motion."""
        del request
        if not self._engaged:
            response.success = True
            response.message = 'e-stop was already clear'
            return response
        self._engaged = False
        self.get_logger().warn(
            f'E-stop reset (was: {self._reason}). Motion is permitted again, '
            'but nothing resumes until a fresh command arrives.'
        )
        self._reason = ''
        response.success = True
        response.message = 'e-stop reset; send a new goal to move'
        return response


def main(args=None):
    """Run the e-stop gate node."""
    rclpy.init(args=args)
    node = EstopGate()
    try:
        rclpy.spin(node)
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
