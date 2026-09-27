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

"""A top-down panel of the robot with a power switch on every sensor.

The robot is drawn front-up with each ToF sensor's 60 degree field of view as a
wedge where it sits on the body, and the LiDAR in the middle. Clicking a wedge's
switch sets ``enabled.<sensor>`` on ``sensor_power``; the panel never assumes
the switch took, it redraws from ``/sensors/status``, so what it shows is what
the robot is actually doing.
"""

import json
import math
import sys

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer
from PyQt5.QtGui import QBrush, QColor, QFont, QPainter, QPainterPath, QPen
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from PyQt5.QtWidgets import (
    QApplication, QGridLayout, QHBoxLayout, QLabel, QMessageBox, QPushButton,
    QVBoxLayout, QWidget)
from std_msgs.msg import String
from std_srvs.srv import SetBool
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.node import Node

from mobile_base_tools import tof_palette as palette
from mobile_base_tools.sensor_power import LIDAR, SENSORS

# Mount yaw of each face, counter-clockwise from the front, as in the URDF.
MOUNT_YAW = {
    'front': 0.0, 'front_left': 45.0, 'left': 90.0, 'rear_left': 135.0,
    'rear': 180.0, 'rear_right': -135.0, 'right': -90.0, 'front_right': -45.0,
}
FIELD_OF_VIEW = 60.0
# Everything below is drawn to one scale, the LiDAR ring filling the view, so
# the difference in reach is what the eye sees first.
LIDAR_RANGE = 4.0   # lidar_max_range, properties.xacro
TOF_RANGE = 1.0     # obstacle_max_range, tof_floor_classifier
TOF_WALL_RANGE = 2.0  # wall_max_range: walls only
BODY_LENGTH = 0.26  # chassis_length, bumper to bumper
BODY_WIDTH = 0.22   # across the wheels
RING = 0.9          # LiDAR ring radius, as a share of the half-view
SWITCH_RING = 0.56  # where the ToF switches sit, likewise

BACKGROUND = QColor('#24262A')
BODY = QColor('#3A3F47')
BODY_EDGE = QColor('#6B7480')
TEXT = QColor(palette.TEXT)
ON = QColor(palette.CLEAR)
OFF = QColor(palette.OFF)
STALE = QColor(palette.CAUTION)


def title(sensor):
    return sensor.replace('_', ' ').upper()


class RobotView(QWidget):
    """Paint the body, the wedges and the LiDAR ring; host the switches."""

    def __init__(self, on_toggle):
        super().__init__()
        self.setMinimumSize(520, 520)
        self.on_toggle = on_toggle
        self.state = {sensor: ('unknown', 0.0) for sensor in SENSORS}
        self.buttons = {}
        for sensor in SENSORS:
            button = QPushButton(self)
            button.setCheckable(True)
            button.setCursor(Qt.PointingHandCursor)
            button.clicked.connect(
                lambda checked, name=sensor: self.on_toggle(name, checked))
            self.buttons[sensor] = button
        self.update_state({})

    def centre_and_scale(self):
        side = min(self.width(), self.height())
        return QPointF(self.width() / 2.0, self.height() / 2.0), side / 2.0

    @staticmethod
    def screen_angle(yaw):
        # Front is up: robot yaw 0 points to screen -y, CCW stays CCW.
        return math.radians(yaw + 90.0)

    def colour(self, sensor):
        message, _ = self.state[sensor]
        if message == 'off':
            return OFF
        if message == 'on':
            return ON
        return STALE

    def metres(self, radius):
        """Pixels per metre: the LiDAR's reach fills RING of the half-view."""
        return radius * RING / LIDAR_RANGE

    def mount(self, centre, scale, yaw):
        """Where a face's sensor sits on the body outline, on screen."""
        angle = self.screen_angle(yaw)
        half_w, half_l = BODY_WIDTH * scale / 2.0, BODY_LENGTH * scale / 2.0
        # Walk out along the bearing until the rectangle's edge.
        reach = min(
            half_w / abs(math.cos(angle)) if abs(math.cos(angle)) > 1e-6
            else float('inf'),
            half_l / abs(math.sin(angle)) if abs(math.sin(angle)) > 1e-6
            else float('inf'))
        return QPointF(centre.x() + math.cos(angle) * reach,
                       centre.y() - math.sin(angle) * reach)

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), BACKGROUND)
        centre, radius = self.centre_and_scale()
        scale = self.metres(radius)

        lidar = self.colour(LIDAR)
        ring = QColor(lidar)
        ring.setAlpha(28 if self.state[LIDAR][0] != 'off' else 12)
        painter.setPen(QPen(lidar, 1.5, Qt.DashLine))
        painter.setBrush(QBrush(ring))
        painter.drawEllipse(centre, LIDAR_RANGE * scale, LIDAR_RANGE * scale)

        wedge = TOF_RANGE * scale
        wall = TOF_WALL_RANGE * scale
        for sensor, yaw in MOUNT_YAW.items():
            colour = self.colour(sensor)
            origin = self.mount(centre, scale, yaw)
            # Outer, fainter fan: the distance out to which only walls count.
            reach = QColor(colour)
            reach.setAlpha(30 if self.state[sensor][0] != 'off' else 10)
            path = QPainterPath(origin)
            path.arcTo(QRectF(origin.x() - wall, origin.y() - wall,
                              2 * wall, 2 * wall),
                       yaw + 90.0 - FIELD_OF_VIEW / 2.0, FIELD_OF_VIEW)
            path.closeSubpath()
            painter.setPen(Qt.NoPen)
            painter.setBrush(QBrush(reach))
            painter.drawPath(path)
            fill = QColor(colour)
            fill.setAlpha(110 if self.state[sensor][0] != 'off' else 40)
            path = QPainterPath(origin)
            path.arcTo(QRectF(origin.x() - wedge, origin.y() - wedge,
                              2 * wedge, 2 * wedge),
                       yaw + 90.0 - FIELD_OF_VIEW / 2.0, FIELD_OF_VIEW)
            path.closeSubpath()
            painter.setPen(QPen(colour, 1.2))
            painter.setBrush(QBrush(fill))
            painter.drawPath(path)
            # Leader from the wedge tip out to its switch.
            angle = self.screen_angle(yaw)
            tip = QPointF(origin.x() + math.cos(angle) * wedge,
                          origin.y() - math.sin(angle) * wedge)
            end = QPointF(centre.x() + math.cos(angle) * radius * SWITCH_RING,
                          centre.y() - math.sin(angle) * radius * SWITCH_RING)
            leader = QColor(colour)
            leader.setAlpha(120)
            painter.setPen(QPen(leader, 1, Qt.DotLine))
            painter.drawLine(tip, end)

        painter.setPen(QPen(BODY_EDGE, 1.5))
        painter.setBrush(QBrush(BODY))
        painter.drawRoundedRect(
            QRectF(centre.x() - BODY_WIDTH * scale / 2,
                   centre.y() - BODY_LENGTH * scale / 2,
                   BODY_WIDTH * scale, BODY_LENGTH * scale), 3, 3)
        # Front marker: a small arrow on the body itself, no text to collide.
        tip_y = centre.y() - BODY_LENGTH * scale / 2 + 3
        arrow = QPainterPath(QPointF(centre.x(), tip_y))
        arrow.lineTo(centre.x() - 5, tip_y + 8)
        arrow.lineTo(centre.x() + 5, tip_y + 8)
        arrow.closeSubpath()
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(TEXT))
        painter.drawPath(arrow)

    def resizeEvent(self, _event):
        centre, radius = self.centre_and_scale()
        for sensor, button in self.buttons.items():
            button.resize(112, 40)
            if sensor == LIDAR:
                # On the ring itself, at the bottom: it switches the ring.
                point = QPointF(centre.x(), centre.y() + radius * RING)
            else:
                angle = self.screen_angle(MOUNT_YAW[sensor])
                point = QPointF(
                    centre.x() + math.cos(angle) * radius * SWITCH_RING,
                    centre.y() - math.sin(angle) * radius * SWITCH_RING)
            button.move(int(point.x() - 56), int(point.y() - 20))

    def update_state(self, state):
        self.state.update(state)
        for sensor, button in self.buttons.items():
            message, rate = self.state[sensor]
            button.setChecked(message != 'off')
            detail = {'off': 'OFF', 'unknown': '…'}.get(
                message, '{:.1f} Hz'.format(rate) if message == 'on'
                else 'NO DATA')
            button.setText('{}\n{}'.format(title(sensor), detail))
            colour = self.colour(sensor)
            button.setStyleSheet(
                'QPushButton {{ background: {bg}; color: {fg};'
                ' border: 1px solid {edge}; border-radius: 6px;'
                ' font: bold 9pt; }}'.format(
                    bg='#2E333A' if message != 'off' else '#2A2426',
                    fg=TEXT.name(), edge=colour.name()))
        self.update()


LEVEL_COLOURS = {
    'cruise': palette.CLEAR, 'caution': palette.CAUTION,
    'slow': palette.CAUTION, 'crawl': palette.CRITICAL,
    'stop': palette.CRITICAL,
}


class Telemetry(QWidget):
    """Live readouts: motion from odometry, the governor, the navigator.

    These used to be text in RViz; kept here they do not cover the scene.
    """

    FIELDS = (
        # Motion
        ('speed', 'Speed'), ('velocity', 'Velocity (body)'),
        ('heading', 'Heading'), ('position', 'Position (odom)'),
        # Safety
        ('level', 'Speed level'), ('limit', 'Limited by'),
        ('nearest', 'Nearest obstacle'), ('state', 'Navigator'),
        # Localisation
        ('sigma', 'Pose uncertainty (1σ)'), ('gap', 'Wheel vs fused'),
        ('goal', 'To goal'),
    )

    def __init__(self):
        super().__init__()
        grid = QGridLayout(self)
        grid.setHorizontalSpacing(18)
        grid.setVerticalSpacing(2)
        for column in range(4):
            grid.setColumnStretch(column, 1)
        self.values = {}
        for index, (key, name) in enumerate(self.FIELDS):
            caption = QLabel(name.upper().replace('Σ', 'σ'))
            caption.setStyleSheet('color: #8A94A0; font: 8pt;')
            value = QLabel('—')
            value.setStyleSheet('font: bold 11pt;')
            row, column = divmod(index, 4)
            grid.addWidget(caption, row * 2, column, Qt.AlignLeft)
            grid.addWidget(value, row * 2 + 1, column, Qt.AlignLeft)
            self.values[key] = value

    def set(self, key, text, colour=None):
        self.values[key].setText(text)
        if colour is not None:
            self.values[key].setStyleSheet(
                'font: bold 11pt; color: {};'.format(colour))


class SensorPanel(QWidget):
    """Window: the robot view, all-on/all-off, telemetry, a status line."""

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle('Mobile base control panel')
        self.setStyleSheet('background: {}; color: {};'.format(
            BACKGROUND.name(), TEXT.name()))
        self.view = RobotView(self.toggle)
        self.status = QLabel('Waiting for the robot…')
        all_on = QPushButton('All ToF on')
        all_off = QPushButton('All ToF off')
        all_on.clicked.connect(lambda: self.set_all(True))
        all_off.clicked.connect(lambda: self.set_all(False))
        self.stop_button = QPushButton('STOP')
        self.stop_button.setMinimumHeight(44)
        self.stop_button.clicked.connect(self.toggle_stop)
        self.stopped = False
        self.style_stop()
        row = QHBoxLayout()
        row.addWidget(self.stop_button)
        for widget in (all_on, all_off):
            widget.setStyleSheet(
                'QPushButton { background: #2E333A; border: 1px solid #6B7480;'
                ' border-radius: 6px; padding: 6px 14px; }')
            row.addWidget(widget)
        row.addStretch()
        row.addWidget(self.status)
        self.telemetry = Telemetry()
        legend = QLabel(
            '▲ front is up  ·  drawn to scale:  LiDAR {:.1f} m  ·  ToF '
            'obstacles {:.1f} m  ·  walls {:.1f} m'.format(
                LIDAR_RANGE, TOF_RANGE, TOF_WALL_RANGE))
        legend.setAlignment(Qt.AlignCenter)
        legend.setStyleSheet('color: #8A94A0; font: 8pt;')
        layout = QVBoxLayout(self)
        layout.addWidget(self.view)
        layout.addWidget(legend)
        layout.addWidget(self.telemetry)
        layout.addLayout(row)
        self.pose = None
        self.wheel = None
        self.goal = None

        self.client = node.create_client(
            SetParameters, '/sensor_power/set_parameters')
        node.create_subscription(
            DiagnosticArray, '/sensors/status', self.on_status, 5)
        node.create_subscription(
            Odometry, '/odometry/filtered', self.on_odometry, 10)
        node.create_subscription(
            String, '/speed_governor/status', self.on_governor, 10)
        node.create_subscription(
            Odometry, '/mobile_base_controller/odometry', self.on_wheel, 10)
        self.stop_client = node.create_client(
            SetBool, '/speed_governor/stop')
        node.create_subscription(
            String, '/bug_navigator/state', self.on_state, 10)
        node.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)

    def on_odometry(self, message):
        pose, twist = message.pose.pose, message.twist.twist
        q = pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (pose.position.x, pose.position.y)
        vx, vy = twist.linear.x, twist.linear.y
        show = self.telemetry.set
        show('speed', '{:.2f} m/s'.format(math.hypot(vx, vy)))
        show('velocity', '{:+.2f}, {:+.2f}  ω {:+.2f}'.format(
            vx, vy, twist.angular.z))
        show('heading', '{:+.1f}°'.format(math.degrees(yaw)))
        show('position', '{:+.2f}, {:+.2f} m'.format(*self.pose))
        # Covariance diagonal: x at 0, y at 7, yaw at 35.
        c = message.pose.covariance
        show('sigma', '±{:.1f} cm  ±{:.1f}°'.format(
            100.0 * math.sqrt(max(c[0], c[7], 0.0)),
            math.degrees(math.sqrt(max(c[35], 0.0)))))
        if self.wheel is not None:
            # How far the fused pose has moved away from wheels alone: a
            # growing gap is slip, or the IMU correcting the wheels.
            show('gap', '{:.2f} m  {:+.1f}°'.format(
                math.dist(self.pose, self.wheel[:2]),
                math.degrees(math.atan2(math.sin(yaw - self.wheel[2]),
                                        math.cos(yaw - self.wheel[2])))))
        if self.goal is not None:
            show('goal', '{:.2f} m'.format(math.dist(self.pose, self.goal)))

    def on_wheel(self, message):
        pose = message.pose.pose
        q = pose.orientation
        self.wheel = (pose.position.x, pose.position.y, math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def on_governor(self, message):
        status = json.loads(message.data)
        level = status['level']
        self.telemetry.set(
            'level', '{}  ≤ {:.2f} m/s'.format(level.upper(), status['cap']),
            LEVEL_COLOURS.get(level, TEXT.name()))
        self.telemetry.set('limit', status['reason'])
        if status['nearest'] is None:
            self.telemetry.set('nearest', 'none in range', TEXT.name())
        else:
            # Bearing in the body frame: 0 is the front, positive to the left.
            self.telemetry.set(
                'nearest', '{:.2f} m  at {:+.0f}°'.format(
                    status['nearest'], status['bearing']),
                palette.warning_hex(status['nearest']))
        if status['stopped'] != self.stopped:
            self.stopped = status['stopped']
            self.style_stop()

    def toggle_stop(self):
        if not self.stop_client.service_is_ready():
            self.status.setText('speed_governor not running')
            return
        self.stop_client.call_async(SetBool.Request(data=not self.stopped))

    def style_stop(self):
        """Red STOP while driving is allowed; amber RELEASE while latched."""
        if self.stopped:
            text, colour = 'RELEASE', palette.CAUTION
        else:
            text, colour = 'STOP', palette.CRITICAL
        self.stop_button.setText(text)
        self.stop_button.setStyleSheet(
            'QPushButton {{ background: {}; color: #111; border-radius: 6px;'
            ' font: bold 12pt; padding: 6px 28px; }}'.format(colour))

    def on_state(self, message):
        self.telemetry.set('state', message.data.replace('_', ' ').upper())

    def on_goal(self, message):
        self.goal = (message.pose.position.x, message.pose.position.y)

    def on_status(self, message):
        state = {}
        for status in message.status:
            if status.name not in SENSORS:
                continue
            rate = next((float(v.value) for v in status.values
                         if v.key == 'rate_hz'), 0.0)
            text = status.message
            if status.level == DiagnosticStatus.STALE:
                text = 'stale'
            state[status.name] = (text, rate)
        self.view.update_state(state)
        self.status.setText('Live')

    def toggle(self, sensor, enabled):
        if sensor == LIDAR and not enabled:
            answer = QMessageBox.warning(
                self, 'Turn the LiDAR off?',
                'SLAM, localisation and Nav2 all read /scan. With the LiDAR '
                'off they stop updating, and the robot navigates on '
                'odometry and ToF alone.',
                QMessageBox.Ok | QMessageBox.Cancel)
            if answer != QMessageBox.Ok:
                self.view.buttons[LIDAR].setChecked(True)
                return
        self.send({sensor: enabled})

    def set_all(self, enabled):
        self.send({sensor: enabled for sensor in SENSORS if sensor != LIDAR})

    def send(self, switches):
        if not self.client.service_is_ready():
            self.status.setText('sensor_power not running')
            return
        request = SetParameters.Request(parameters=[
            Parameter(name='enabled.' + sensor, value=ParameterValue(
                type=ParameterType.PARAMETER_BOOL, bool_value=value))
            for sensor, value in switches.items()
        ])
        self.client.call_async(request)


def main(args=None):
    """Open the sensor power panel."""
    rclpy.init(args=args)
    node = Node('sensor_panel')
    app = QApplication(sys.argv)
    panel = SensorPanel(node)
    panel.resize(600, 660)
    panel.show()
    spin = QTimer()
    spin.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    spin.start(20)
    code = app.exec_()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(code)
