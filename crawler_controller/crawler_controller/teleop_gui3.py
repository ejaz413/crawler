#!/usr/bin/env python3
import sys
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import Empty

from PyQt5.QtCore import Qt, QTimer, QSize, QEvent
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QLabel, QSlider, QCheckBox,
    QHBoxLayout, QVBoxLayout, QGridLayout, QGroupBox, QSizePolicy
)

# ---------------- Helpers ----------------

def quat_to_yaw_deg(qz: float, qw: float) -> float:
    """Planar yaw from quaternion (z,w)."""
    yaw_rad = 2.0 * math.atan2(qz, qw)
    yaw_deg = math.degrees(yaw_rad)
    # wrap to [-180, 180)
    while yaw_deg >= 180.0:
        yaw_deg -= 360.0
    while yaw_deg < -180.0:
        yaw_deg += 360.0
    return yaw_deg

def radps_to_rpm(rad_s: float) -> float:
    return (rad_s * 60.0) / (2.0 * math.pi)


# ---------------- ROS Node ----------------

class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_gui")

        # ---------- Parameters ----------
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("vel_topic", "/vel")                 # robot v,w topic (Twist)
        self.declare_parameter("wheel_speeds_topic", "/wheel_speeds")  # wheel speeds (Twist)
        self.declare_parameter("quick_stop_topic", "/quick_stop")
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("linear_sign", 1.0)
        self.declare_parameter("angular_sign", 1.0)
        # Fallback wheel speed calc if /wheel_speeds missing:
        self.declare_parameter("wheel_separation", 0.512)  # [m]
        # Optional console printing of publishes
        self.declare_parameter("print_cmds", False)

        # Header image
        self.declare_parameter("help_image_path", "/home/pavetech/crawler_ws/src/crawler_controller/crawler_controller/logo.png")
        self.declare_parameter("help_image_max_height", 180)  # px cap

        # Read params
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        self.odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.vel_topic = self.get_parameter("vel_topic").get_parameter_value().string_value
        self.wh_topic = self.get_parameter("wheel_speeds_topic").get_parameter_value().string_value
        self.quick_stop_topic = self.get_parameter("quick_stop_topic").get_parameter_value().string_value
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.linear_sign = float(self.get_parameter("linear_sign").value)
        self.angular_sign = float(self.get_parameter("angular_sign").value)
        self.wheel_sep = float(self.get_parameter("wheel_separation").value)
        self.print_cmds = bool(self.get_parameter("print_cmds").value)

        self.help_image_path = self.get_parameter("help_image_path").get_parameter_value().string_value
        self.help_image_max_h = int(self.get_parameter("help_image_max_height").value)

        # ---------- Publishers / Subscribers / Clients ----------
        self.pub_cmd = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.pub_qs  = self.create_publisher(Bool, self.quick_stop_topic, 10)

        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)
        self.sub_vel  = self.create_subscription(Twist, self.vel_topic, self._vel_cb, 10)
        self.sub_wh   = self.create_subscription(Twist, self.wh_topic, self._wheel_cb, 10)

        self.cli_reset = self.create_client(Empty, "/reset_odom")  # optional service

        # ---------- Odometry/State ----------
        self._have_odom = False
        self._last_x = None
        self._last_y = None

        # Heading (raw from odom) + GUI offset to zero on demand
        self.heading_deg_raw = 0.0
        self.heading_offset_deg = 0.0
        self.heading_deg = 0.0

        # Robot speed from /vel or /odom.twist
        self.robot_v_mps = 0.0
        self.robot_w_rps = 0.0

        # Wheel speeds from /wheel_speeds (preferred) or fallback from v,w
        self.v_left_mps = 0.0
        self.v_right_mps = 0.0
        self.w_left_radps = 0.0
        self.w_right_radps = 0.0

        # Distance
        self.trip_m = 0.0
        self.total_m = 0.0

        # Safety status
        self.last_qs_status = "—"

        # Flags & publish diagnostics
        self._wheel_msg_seen = False
        self._vel_seen = False

        self.sent_count = 0        # number of /cmd_vel publishes
        self.last_cmd_v = 0.0      # last sent m/s
        self.last_cmd_w = 0.0      # last sent rad/s

    # ---------- Publish commands ----------
    def send(self, v_cmps: float, w_degps: float):
        """Publish cmd_vel (converted from cm/s and deg/s)."""
        msg = Twist()
        msg.linear.x = (v_cmps / 100.0) * self.linear_sign
        msg.angular.z = math.radians(w_degps) * self.angular_sign
        self.pub_cmd.publish(msg)
        # Bookkeeping / diagnostics
        self.sent_count += 1
        self.last_cmd_v = msg.linear.x
        self.last_cmd_w = msg.angular.z
        if self.print_cmds and (self.sent_count % 10 == 0):  # print every 10th to avoid spam
            print(f"[GUI] /cmd_vel #{self.sent_count}: v={self.last_cmd_v:+.3f} m/s  w={self.last_cmd_w:+.3f} rad/s")

    # ---------- Safety ----------
    def quick_stop(self):
        self.pub_qs.publish(Bool(data=True))
        self.last_qs_status = "QuickStop sent (True)"

    def recover(self):
        self.pub_qs.publish(Bool(data=False))
        self.last_qs_status = "Recover sent (False)"

    # ---------- Resets ----------
    def reset_trip(self):
        self.trip_m = 0.0

    def reset_all(self):
        """Stop robot, zero Trip, zero displayed heading, zero displayed speeds, call /reset_odom if available."""
        # Stop
        self.send(0.0, 0.0)
        # Reset distances/speeds (display)
        self.trip_m = 0.0
        self.robot_v_mps = 0.0
        self.robot_w_rps = 0.0
        self.v_left_mps = 0.0
        self.v_right_mps = 0.0
        self.w_left_radps = 0.0
        self.w_right_radps = 0.0
        # Heading zero (display)
        self.heading_offset_deg = self.heading_deg_raw
        # Reset last pose so trip integrates from here
        self._last_x = None
        self._last_y = None
        # Call /reset_odom if ready
        if self.cli_reset.service_is_ready():
            try:
                self.cli_reset.call_async(Empty.Request())
                self.last_qs_status = "reset_odom called"
            except Exception:
                pass

    # ---------- Subscribers ----------
    def _odom_cb(self, odom: Odometry):
        # Position for trip
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y

        # Heading
        qz = odom.pose.pose.orientation.z
        qw = odom.pose.pose.orientation.w
        self.heading_deg_raw = quat_to_yaw_deg(qz, qw)
        self.heading_deg = self.heading_deg_raw - self.heading_offset_deg
        # wrap display to [-180, 180)
        while self.heading_deg >= 180.0:
            self.heading_deg -= 360.0
        while self.heading_deg < -180.0:
            self.heading_deg += 360.0

        # If /vel is not present, fall back to odom.twist
        if not self._vel_seen:
            self.robot_v_mps = float(odom.twist.twist.linear.x)
            self.robot_w_rps = float(odom.twist.twist.angular.z)
            # Fallback wheel speeds if /wheel_speeds missing
            if not self._wheel_msg_seen:
                half = 0.5 * self.wheel_sep
                self.v_right_mps = self.robot_v_mps + self.robot_w_rps * half
                self.v_left_mps  = self.robot_v_mps - self.robot_w_rps * half
                # wheel rad/s unknown in fallback
                self.w_right_radps = 0.0
                self.w_left_radps  = 0.0

        # Trip integrate
        if self._last_x is not None:
            dx = x - self._last_x
            dy = y - self._last_y
            ds = math.hypot(dx, dy)
            if ds > 1e-5:
                self.trip_m += ds
                self.total_m += ds
                self._have_odom = True
        self._last_x, self._last_y = x, y

    def _vel_cb(self, msg: Twist):
        """Robot v,w from /vel."""
        self._vel_seen = True
        self.robot_v_mps = float(msg.linear.x)
        self.robot_w_rps = float(msg.angular.z)

    def _wheel_cb(self, msg: Twist):
        """
        Wheel speeds from /wheel_speeds (Twist):
          linear.x = left m/s
          linear.y = right m/s
          angular.x = left rad/s
          angular.y = right rad/s
        """
        self._wheel_msg_seen = True
        self.v_left_mps  = float(msg.linear.x)
        self.v_right_mps = float(msg.linear.y)
        self.w_left_radps  = float(msg.angular.x)
        self.w_right_radps = float(msg.angular.y)


# ---------------- PyQt GUI ----------------

class TeleopGUI(QWidget):
    def __init__(self, node: TeleopNode, app: QApplication):
        super().__init__()
        self.node = node
        self.app = app

        # Motion caps (GUI)
        self.lin_cap_cm = 50.0
        self.ang_cap_deg = 120.0
        self.step_cm = 10.0
        self.step_deg = 15.0

        # Motion state (GUI)
        self.v_cm = 0.0
        self.w_deg = 0.0
        self.decay = 0.9
        self.left_held = False
        self.right_held = False
        self.inplace_when_steering = False

        # Image state
        self.help_pixmap = None

        # Build UI
        self.setWindowTitle("Pavetech Teleop")
        self._build_ui()

        # Timers
        self.dt = 1.0 / max(1.0, self.node.publish_hz)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.on_tick)
        self.timer.start(int(self.dt * 1000))

        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self._update_status)
        self.ui_timer.start(100)

        # Install key filter
        self.app.installEventFilter(self)

    # ---------- UI ----------
    def _build_ui(self):
        # Header
        self.title_label = QLabel("Pavetech")
        self.title_label.setAlignment(Qt.AlignLeft)
        self.title_label.setStyleSheet(
            "font-size: 50px; font-weight: 800; letter-spacing: 1px; margin: 8px 0;"
        )

        self.help_label = QLabel()
        self.help_label.setAlignment(Qt.AlignCenter)
        self.help_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        self.help_label.setMinimumHeight(80)
        self._load_help_image()

        gb_help = QGroupBox("")
        lay_help = QVBoxLayout()
        lay_help.addWidget(self.title_label)
        lay_help.addWidget(self.help_label)
        gb_help.setLayout(lay_help)

        # Motion Buttons
        self.btn_fwd = QPushButton("Forward (W)")
        self.btn_back = QPushButton("Backward (S)")
        self.btn_stop = QPushButton("STOP (Space)")
        self.btn_left = QPushButton("◄ Left (A)")
        self.btn_right = QPushButton("Right (D) ►")
        self.btn_stop.setStyleSheet("font-weight: bold; background:#ffdddd;")

        self.btn_left.pressed.connect(self._press_left)
        self.btn_left.released.connect(self._release_left)
        self.btn_right.pressed.connect(self._press_right)
        self.btn_right.released.connect(self._release_right)
        self.btn_fwd.clicked.connect(self._click_forward)
        self.btn_back.clicked.connect(self._click_backward)
        self.btn_stop.clicked.connect(self._click_stop)

        self.chk_inplace = QCheckBox("In-place when steering")
        self.chk_inplace.stateChanged.connect(
            lambda s: setattr(self, "inplace_when_steering", s == Qt.Checked)
        )

        # Safety
        self.btn_qs = QPushButton("EMERGENCY STOP (Q)")
        self.btn_qs.setStyleSheet("font-weight: bold; background:#ff3b3b; color:white;")
        self.btn_qs.clicked.connect(self._click_quick_stop)

        self.btn_recover = QPushButton("Recover (R)")
        self.btn_recover.setStyleSheet("font-weight: bold; background:#e6ffe6;")
        self.btn_recover.clicked.connect(self._click_recover)

        self.lbl_qs = QLabel("Quick Stop: —")
        self.lbl_qs.setStyleSheet("font-family: monospace;")

        # Sliders
        self.lbl_lin = QLabel()
        self.lbl_ang = QLabel()
        self.sld_lin = QSlider(Qt.Horizontal)
        self.sld_ang = QSlider(Qt.Horizontal)
        self.sld_lin.setMinimum(0); self.sld_lin.setMaximum(150)
        self.sld_ang.setMinimum(0); self.sld_ang.setMaximum(360)
        self.sld_lin.setValue(int(self.lin_cap_cm))
        self.sld_ang.setValue(int(self.ang_cap_deg))
        self.sld_lin.valueChanged.connect(self._slider_lin)
        self.sld_ang.valueChanged.connect(self._slider_ang)

        # Publish diagnostics
        self.lbl_status = QLabel()
        self.lbl_status.setStyleSheet("font-family: monospace;")
        self.lbl_pub = QLabel("pub: 0")
        self.lbl_pub.setStyleSheet("font-family: monospace;")
        self.led = QLabel("●")
        self.led.setStyleSheet("color: #888; font-weight: bold;")  # gray idle

        # Telemetry
        self.lbl_odom = QLabel("Odometry: waiting…")
        self.lbl_wheels = QLabel("Wheels: L 0.00 m/s (0.0 rpm) | R 0.00 m/s (0.0 rpm)")
        self.lbl_wheels.setStyleSheet("font-family: monospace;")

        self.btn_trip_reset = QPushButton("Reset Trip")
        self.btn_trip_reset.clicked.connect(self.node.reset_trip)

        self.btn_reset_all = QPushButton("Reset ALL")
        self.btn_reset_all.setStyleSheet("font-weight: bold; background:#ddeeff;")
        self.btn_reset_all.clicked.connect(self.node.reset_all)

        # Layouts
        grid = QGridLayout()
        grid.addWidget(self.btn_left, 0, 0)
        grid.addWidget(self.btn_stop, 0, 1)
        grid.addWidget(self.btn_right, 0, 2)
        grid.addWidget(self.btn_fwd, 1, 1)
        grid.addWidget(self.btn_back, 2, 1)

        gb_ctrl = QGroupBox("Controls")
        gb = QVBoxLayout()
        gb.addLayout(grid)
        gb.addWidget(self.chk_inplace)
        gb_ctrl.setLayout(gb)

        gb_qs = QGroupBox("Safety")
        lay_qs = QHBoxLayout()
        lay_qs.addWidget(self.btn_qs)
        lay_qs.addWidget(self.btn_recover)
        lay_qs.addWidget(self.lbl_qs)
        gb_qs.setLayout(lay_qs)

        row_lin = QHBoxLayout()
        row_lin.addWidget(QLabel("Linear cap "))
        row_lin.addWidget(self.sld_lin)
        row_lin.addWidget(self.lbl_lin)

        row_ang = QHBoxLayout()
        row_ang.addWidget(QLabel("Angular cap "))
        row_ang.addWidget(self.sld_ang)
        row_ang.addWidget(self.lbl_ang)

        # add publish diagnostics row
        row_pub = QHBoxLayout()
        row_pub.addWidget(QLabel("Publish:"))
        row_pub.addWidget(self.lbl_pub)
        row_pub.addWidget(self.led)
        row_pub.addStretch(1)

        gb_spd = QGroupBox("Speed Limits")
        lay_spd = QVBoxLayout()
        lay_spd.addLayout(row_lin)
        lay_spd.addLayout(row_ang)
        lay_spd.addLayout(row_pub)
        gb_spd.setLayout(lay_spd)

        gb_odom = QGroupBox("Telemetry")
        lay_odom = QVBoxLayout()
        lay_odom.addWidget(self.lbl_odom)
        lay_odom.addWidget(self.lbl_wheels)
        row_trip = QHBoxLayout()
        row_trip.addWidget(self.btn_trip_reset)
        row_trip.addWidget(self.btn_reset_all)
        lay_odom.addLayout(row_trip)
        gb_odom.setLayout(lay_odom)

        root = QVBoxLayout()
        root.addWidget(gb_help)
        root.addWidget(gb_ctrl)
        root.addWidget(gb_qs)
        root.addWidget(gb_spd)
        root.addWidget(self.lbl_status)
        root.addWidget(gb_odom)
        self.setLayout(root)
        self.resize(820, 620)
        self.setFocusPolicy(Qt.StrongFocus)

        self._refresh_help_image()

    # ---------- Image helpers ----------
    def _load_help_image(self):
        pm = QPixmap(self.node.help_image_path) if self.node.help_image_path else QPixmap()
        if pm.isNull():
            self.help_label.hide()
            self.help_pixmap = None
        else:
            self.help_pixmap = pm
            self.help_label.show()

    def _refresh_help_image(self):
        if not self.help_pixmap or self.help_label.width() <= 1:
            return
        scaled = self.help_pixmap.scaledToWidth(self.help_label.width(), Qt.SmoothTransformation)
        if scaled.height() > self.node.help_image_max_h:
            scaled = self.help_pixmap.scaled(
                QSize(int(self.help_pixmap.width() * self.node.help_image_max_h / self.help_pixmap.height()),
                      self.node.help_image_max_h),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
        self.help_label.setPixmap(scaled)

    def resizeEvent(self, e):
        super().resizeEvent(e)
        self._refresh_help_image()

    # ---------- Event filter (keyboard) ----------
    def eventFilter(self, obj, e):
        if e.type() == QEvent.KeyPress:
            if e.isAutoRepeat():
                return False
            self._on_key_press(e.key()); return True
        elif e.type() == QEvent.KeyRelease:
            if e.isAutoRepeat():
                return False
            self._on_key_release(e.key()); return True
        return False

    def _on_key_press(self, k):
        if k in (Qt.Key_W, Qt.Key_Up): self._click_forward()
        elif k in (Qt.Key_S, Qt.Key_Down): self._click_backward()
        elif k == Qt.Key_Space: self._click_stop()
        elif k in (Qt.Key_A, Qt.Key_Left): self._press_left()
        elif k in (Qt.Key_D, Qt.Key_Right): self._press_right()
        elif k == Qt.Key_Comma: self._slower()
        elif k == Qt.Key_Period: self._faster()
        elif k == Qt.Key_Q: self._click_quick_stop()
        elif k == Qt.Key_R: self._click_recover()

    def _on_key_release(self, k):
        if k in (Qt.Key_A, Qt.Key_Left): self._release_left()
        elif k in (Qt.Key_D, Qt.Key_Right): self._release_right()

    # ---------- Motion controls ----------
    def _press_left(self):
        self.left_held = True; self.right_held = False
        if self.inplace_when_steering: self.v_cm = 0.0
    def _release_left(self): self.left_held = False
    def _press_right(self):
        self.right_held = True; self.left_held = False
        if self.inplace_when_steering: self.v_cm = 0.0
    def _release_right(self): self.right_held = False
    def _click_forward(self): self.v_cm = +self.lin_cap_cm
    def _click_backward(self): self.v_cm = -self.lin_cap_cm
    def _click_stop(self):
        self.v_cm = 0.0; self.w_deg = 0.0
        self.left_held = self.right_held = False

    # ---------- Safety buttons ----------
    def _click_quick_stop(self):
        try:
            self.node.quick_stop()
            self.v_cm = 0.0; self.w_deg = 0.0
            self.left_held = self.right_held = False
        except Exception:
            pass
        self._update_status()

    def _click_recover(self):
        try:
            self.node.recover()
        except Exception:
            pass
        self._update_status()

    # ---------- Speed helpers ----------
    def _slower(self):
        self.lin_cap_cm = max(0.0, round(self.lin_cap_cm - self.step_cm, 1))
        self.ang_cap_deg = max(0.0, round(self.ang_cap_deg - self.step_deg, 1))
        self.sld_lin.setValue(int(self.lin_cap_cm))
        self.sld_ang.setValue(int(self.ang_cap_deg))
        self.v_cm = math.copysign(self.lin_cap_cm, self.v_cm) if self.v_cm != 0 else 0.0
    def _faster(self):
        self.lin_cap_cm = min(150.0, round(self.lin_cap_cm + self.step_cm, 1))
        self.ang_cap_deg = min(360.0, round(self.ang_cap_deg + self.step_deg, 1))
        self.sld_lin.setValue(int(self.lin_cap_cm))
        self.sld_ang.setValue(int(self.ang_cap_deg))
        self.v_cm = math.copysign(self.lin_cap_cm, self.v_cm) if self.v_cm != 0 else 0.0

    def _slider_lin(self, val):
        self.lin_cap_cm = float(val)
        self.v_cm = math.copysign(self.lin_cap_cm, self.v_cm) if self.v_cm != 0 else 0.0
    def _slider_ang(self, val):
        self.ang_cap_deg = float(val)
        if self.left_held:
            self.w_deg = +self.ang_cap_deg
        elif self.right_held:
            self.w_deg = -self.ang_cap_deg

    # ---------- Tick ----------
    def on_tick(self):
        if self.left_held:
            self.w_deg = +self.ang_cap_deg
        elif self.right_held:
            self.w_deg = -self.ang_cap_deg
        else:
            if abs(self.w_deg) > 0.5:
                self.w_deg *= self.decay
            else:
                self.w_deg = 0.0

        try:
            self.node.send(self.v_cm, self.w_deg)
        except Exception:
            pass

        self._update_status()

    # ---------- UI refresh ----------
    def _update_status(self):
        self.lbl_lin.setText(f"{self.lin_cap_cm:.0f} ")
        self.lbl_ang.setText(f"{self.ang_cap_deg:.0f} ")
        self.lbl_qs.setText(f"Quick Stop: {self.node.last_qs_status}")

        # Status line (commanded)
        v_mps_cmd = (self.v_cm / 100.0) * self.node.linear_sign
        w_rps_cmd = math.radians(self.w_deg) * self.node.angular_sign
        self.lbl_status.setText(
            f"cmd: v={v_mps_cmd:+.2f} m/s  w={math.degrees(w_rps_cmd):+.1f} deg/s"
        )

        # Show last sent message info + blink LED when count changes
        self.lbl_pub.setText(
            f"pub: {self.node.sent_count}  v={self.node.last_cmd_v:+.2f} m/s  w={math.degrees(self.node.last_cmd_w):+.1f} deg/s"
        )
        if getattr(self, "_last_seen_count", -1) != self.node.sent_count:
            self.led.setStyleSheet("color: #1abc9c; font-weight: bold;")  # teal when just published
        else:
            self.led.setStyleSheet("color: #888; font-weight: bold;")     # gray idle
        self._last_seen_count = self.node.sent_count

        # Telemetry
        if self.node._have_odom:
            # Robot v,w from /vel (preferred) or odom fallback
            v = self.node.robot_v_mps
            w = self.node.robot_w_rps
            # Wheel speeds
            vl = self.node.v_left_mps
            vr = self.node.v_right_mps
            # If we have wheel angular speeds, add rpms
            l_rpm = radps_to_rpm(self.node.w_left_radps) if self.node.w_left_radps else 0.0
            r_rpm = radps_to_rpm(self.node.w_right_radps) if self.node.w_right_radps else 0.0

            self.lbl_odom.setText(
                f"Heading: {self.node.heading_deg:+.1f}°   "
                f"Robot Speed: {v:+.2f} m/s   Rot: {math.degrees(w):+.1f} deg/s   "
                f"Trip: {self.node.trip_m:.2f} m   Total: {self.node.total_m:.2f} m"
            )
            if self.node._wheel_msg_seen and (abs(l_rpm) > 0 or abs(r_rpm) > 0):
                self.lbl_wheels.setText(
                    f"Wheels: L {vl:+.2f} m/s ({l_rpm:+.1f} rpm) | R {vr:+.2f} m/s ({r_rpm:+.1f} rpm)"
                )
            else:
                self.lbl_wheels.setText(
                    f"Wheels: L {vl:+.2f} m/s | R {vr:+.2f} m/s"
                )
        else:
            self.lbl_odom.setText("Odometry: (no /odom yet)")
            self.lbl_wheels.setText("Wheels: L 0.00 m/s | R 0.00 m/s")

    # ---------- Close ----------
    def closeEvent(self, e):
        try:
            self.node.send(0.0, 0.0)
        except Exception:
            pass
        e.accept()


# ---------------- Main ----------------

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    node = TeleopNode()
    gui = TeleopGUI(node, app)
    gui.show()

    # Spin ROS in the Qt loop
    spin_timer = QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    spin_timer.start(10)

    try:
        rc = app.exec_()
    finally:
        try:
            node.send(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
