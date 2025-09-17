#!/usr/bin/env python3
import math
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QLabel, QSlider, QCheckBox,
    QHBoxLayout, QVBoxLayout, QGridLayout, QGroupBox
)

HELP_TEXT = (
    "W/A/S/D to move, SPACE to stop, ',' slower, '.' faster\n"
    "• Hold A/D to steer; release → auto-straight.\n"
    "• Forward/Backward are latched (click once).\n"
    "• Units: linear in cm/s, angular in deg/s (ROS publishes SI).\n"
)

def quat_to_yaw_deg(qz: float, qw: float) -> float:
    # For planar bases we use z,w only (qx,qy≈0).
    # yaw = 2 * atan2(z, w) in radians
    yaw_rad = 2.0 * math.atan2(qz, qw)
    yaw_deg = math.degrees(yaw_rad)
    # Normalize to [-180,180)
    while yaw_deg >= 180.0: yaw_deg -= 360.0
    while yaw_deg < -180.0: yaw_deg += 360.0
    return yaw_deg

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_gui')

        # Parameters
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('publish_hz', 50.0)
        # Signs let you match your driver without editing code:
        self.declare_parameter('linear_sign',  1.0)  # set -1.0 if W should send negative x
        self.declare_parameter('angular_sign', 1.0)  # set -1.0 if steer feels inverted

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.odom_topic    = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.publish_hz    = float(self.get_parameter('publish_hz').value)
        self.linear_sign   = float(self.get_parameter('linear_sign').value)
        self.angular_sign  = float(self.get_parameter('angular_sign').value)

        # Pub/Sub
        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)

        # Odom state
        self._have_odom = False
        self._last_x = None
        self._last_y = None
        self.trip_m = 0.0
        self.total_m = 0.0
        self.odom_speed_mps = 0.0
        self.heading_deg = 0.0

    def send(self, v_cmps: float, w_degps: float):
        """Publish Twist. Inputs in cm/s and deg/s; apply signs & convert to SI."""
        msg = Twist()
        msg.linear.x  = float((v_cmps / 100.0) * self.linear_sign)       # m/s
        msg.angular.z = float(math.radians(w_degps) * self.angular_sign) # rad/s
        self.pub.publish(msg)

    def reset_trip(self):
        self.trip_m = 0.0

    # ----- Odometry -----
    def _odom_cb(self, odom: Odometry):
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        qz = odom.pose.pose.orientation.z
        qw = odom.pose.pose.orientation.w

        self.heading_deg = quat_to_yaw_deg(qz, qw)
        self.odom_speed_mps = odom.twist.twist.linear.x

        if self._last_x is not None:
            dx = x - self._last_x
            dy = y - self._last_y
            ds = math.hypot(dx, dy)
            # Filter out tiny jitter
            if ds > 1e-5:
                self.trip_m  += ds
                self.total_m += ds
                self._have_odom = True
        self._last_x, self._last_y = x, y


class TeleopWindow(QWidget):
    def __init__(self, node: TeleopNode):
        super().__init__()
        self.node = node

        # Caps & state in USER UNITS
        self.lin_cap_cm  = 50.0      # 0..150 cm/s by slider
        self.ang_cap_deg = 120.0     # 0..360 deg/s by slider
        self.step_cm  = 10.0
        self.step_deg = 15.0

        self.v_cm  = 0.0
        self.w_deg = 0.0
        self.decay = 0.9          # steer decay on release
        self.left_held = False
        self.right_held = False
        self.inplace_when_steering = False

        self.setWindowTitle("Crawler Teleop (cm/s, deg/s) + Odom Trip")
        self._build_ui()

        # Publish timer
        self.dt = 1.0 / max(1.0, self.node.publish_hz)
        self.timer = QTimer(self); self.timer.timeout.connect(self.on_tick)
        self.timer.start(int(self.dt * 1000))

        # UI refresh timer (odometry panel)
        self.ui_timer = QTimer(self); self.ui_timer.timeout.connect(self._update_status)
        self.ui_timer.start(100)

        self._update_status()

    # ---------- UI ----------
    def _build_ui(self):
        info = QLabel(HELP_TEXT); info.setWordWrap(True)

        # Buttons
        self.btn_fwd  = QPushButton("Forward (W)")
        self.btn_back = QPushButton("Backward (S)")
        self.btn_stop = QPushButton("STOP (Space)")
        self.btn_left = QPushButton("◄  Left (A)")
        self.btn_right= QPushButton("Right (D)  ►")
        self.btn_stop.setStyleSheet("font-weight: bold; background:#ffdddd;")

        # Steer press/hold
        self.btn_left.setAutoRepeat(False)
        self.btn_right.setAutoRepeat(False)
        self.btn_left.pressed.connect(self._press_left)
        self.btn_left.released.connect(self._release_left)
        self.btn_right.pressed.connect(self._press_right)
        self.btn_right.released.connect(self._release_right)

        # Latch forward/back/stop
        self.btn_fwd.clicked.connect(self._click_forward)
        self.btn_back.clicked.connect(self._click_backward)
        self.btn_stop.clicked.connect(self._click_stop)

        # In-place steering toggle
        self.chk_inplace = QCheckBox("In-place when steering (A/D sets v=0)")
        self.chk_inplace.stateChanged.connect(
            lambda s: setattr(self, 'inplace_when_steering', s == Qt.Checked))

        # Sliders
        self.lbl_lin = QLabel()
        self.lbl_ang = QLabel()
        self.sld_lin = QSlider(Qt.Horizontal)
        self.sld_ang = QSlider(Qt.Horizontal)
        self.sld_lin.setMinimum(0);   self.sld_lin.setMaximum(150); self.sld_lin.setSingleStep(5)
        self.sld_ang.setMinimum(0);   self.sld_ang.setMaximum(360); self.sld_ang.setSingleStep(5)
        self.sld_lin.setValue(int(self.lin_cap_cm))
        self.sld_ang.setValue(int(self.ang_cap_deg))
        self.sld_lin.valueChanged.connect(self._slider_lin)
        self.sld_ang.valueChanged.connect(self._slider_ang)

        # Odom panel
        self.lbl_odom = QLabel("Odometry: waiting…")
        self.btn_trip_reset = QPushButton("Reset Trip")
        self.btn_trip_reset.clicked.connect(lambda: self.node.reset_trip())

        # Layouts
        grid = QGridLayout()
        grid.addWidget(self.btn_left,  0, 0)
        grid.addWidget(self.btn_stop,  0, 1)
        grid.addWidget(self.btn_right, 0, 2)
        grid.addWidget(self.btn_fwd,   1, 1)
        grid.addWidget(self.btn_back,  2, 1)

        gb_ctrl = QGroupBox("Controls"); gb = QVBoxLayout()
        gb.addLayout(grid); gb.addWidget(self.chk_inplace); gb_ctrl.setLayout(gb)

        row_lin = QHBoxLayout(); row_lin.addWidget(QLabel("Linear cap (cm/s)")); row_lin.addWidget(self.sld_lin); row_lin.addWidget(self.lbl_lin)
        row_ang = QHBoxLayout(); row_ang.addWidget(QLabel("Angular cap (deg/s)")); row_ang.addWidget(self.sld_ang); row_ang.addWidget(self.lbl_ang)
        gb_spd = QGroupBox("Speed Limits")
        lay_spd = QVBoxLayout(); lay_spd.addLayout(row_lin); lay_spd.addLayout(row_ang); gb_spd.setLayout(lay_spd)

        self.lbl_status = QLabel(); self.lbl_status.setStyleSheet("font-family: monospace;")
        gb_odom = QGroupBox("Odometry (from /odom)")
        lay_odom = QVBoxLayout(); lay_odom.addWidget(self.lbl_odom); lay_odom.addWidget(self.btn_trip_reset); gb_odom.setLayout(lay_odom)

        root = QVBoxLayout()
        root.addWidget(info)
        root.addWidget(gb_ctrl)
        root.addWidget(gb_spd)
        root.addWidget(self.lbl_status)
        root.addWidget(gb_odom)
        self.setLayout(root)
        self.resize(640, 420)
        self.setFocusPolicy(Qt.StrongFocus)

    # ---------- Keys ----------
    def keyPressEvent(self, e):
        if e.isAutoRepeat(): return
        k = e.key()
        if   k in (Qt.Key_W, Qt.Key_Up):    self._click_forward()
        elif k in (Qt.Key_S, Qt.Key_Down):  self._click_backward()
        elif k == Qt.Key_Space:             self._click_stop()
        elif k in (Qt.Key_A, Qt.Key_Left):  self._press_left()
        elif k in (Qt.Key_D, Qt.Key_Right): self._press_right()
        elif k == Qt.Key_Comma:             self._slower()
        elif k == Qt.Key_Period:            self._faster()
    def keyReleaseEvent(self, e):
        if e.isAutoRepeat(): return
        k = e.key()
        if   k in (Qt.Key_A, Qt.Key_Left):  self._release_left()
        elif k in (Qt.Key_D, Qt.Key_Right): self._release_right()

    # ---------- Button/slider handlers ----------
    def _press_left(self):
        self.left_held = True; self.right_held = False
        if self.inplace_when_steering: self.v_cm = 0.0
        self._update_status()
    def _release_left(self):
        self.left_held = False; self._update_status()
    def _press_right(self):
        self.right_held = True; self.left_held = False
        if self.inplace_when_steering: self.v_cm = 0.0
        self._update_status()
    def _release_right(self):
        self.right_held = False; self._update_status()

    def _click_forward(self):
        self.v_cm = +self.lin_cap_cm; self._update_status()
    def _click_backward(self):
        self.v_cm = -self.lin_cap_cm; self._update_status()
    def _click_stop(self):
        self.v_cm = 0.0; self.w_deg = 0.0
        self.left_held = self.right_held = False
        self._update_status()

    def _slower(self):
        self.lin_cap_cm  = max(0.0, round(self.lin_cap_cm - self.step_cm,  1))
        self.ang_cap_deg = max(0.0, round(self.ang_cap_deg - self.step_deg, 1))
        self.sld_lin.setValue(int(self.lin_cap_cm))
        self.sld_ang.setValue(int(self.ang_cap_deg))
        self.v_cm = math.copysign(self.lin_cap_cm, self.v_cm) if self.v_cm != 0 else 0.0
        if self.left_held:   self.w_deg = +self.ang_cap_deg
        elif self.right_held:self.w_deg = -self.ang_cap_deg
        self._update_status()
    def _faster(self):
        self.lin_cap_cm  = min(150.0, round(self.lin_cap_cm + self.step_cm,  1))
        self.ang_cap_deg = min(360.0, round(self.ang_cap_deg + self.step_deg, 1))
        self.sld_lin.setValue(int(self.lin_cap_cm))
        self.sld_ang.setValue(int(self.ang_cap_deg))
        self.v_cm = math.copysign(self.lin_cap_cm, self.v_cm) if self.v_cm != 0 else 0.0
        if self.left_held:   self.w_deg = +self.ang_cap_deg
        elif self.right_held:self.w_deg = -self.ang_cap_deg
        self._update_status()

    def _slider_lin(self, val):
        self.lin_cap_cm = float(val)
        self.v_cm = math.copysign(self.lin_cap_cm, self.v_cm) if self.v_cm != 0 else 0.0
        self._update_status()
    def _slider_ang(self, val):
        self.ang_cap_deg = float(val)
        if self.left_held:   self.w_deg = +self.ang_cap_deg
        elif self.right_held:self.w_deg = -self.ang_cap_deg
        self._update_status()

    # ---------- Tick / Publish ----------
    def on_tick(self):
        # Steer hold or decay back to straight
        if self.left_held:
            self.w_deg = +self.ang_cap_deg
        elif self.right_held:
            self.w_deg = -self.ang_cap_deg
        else:
            if abs(self.w_deg) > 0.5:
                self.w_deg *= self.decay
            else:
                self.w_deg = 0.0

        # Publish (convert to SI + apply signs in node)
        try:
            self.node.send(self.v_cm, self.w_deg)
        except Exception:
            pass

        # Status
        self._update_status()

    def _update_status(self):
        v_mps = (self.v_cm/100.0) * self.node.linear_sign
        w_rps = math.radians(self.w_deg) * self.node.angular_sign
        self.lbl_lin.setText(f"{self.lin_cap_cm:.0f} cm/s")
        self.lbl_ang.setText(f"{self.ang_cap_deg:.0f} deg/s")
        self.lbl_status.setText(
            f"[cmd] v={self.v_cm:+.0f} cm/s ({v_mps:+.2f} m/s)   "
            f"w={self.w_deg:+.0f} deg/s ({w_rps:+.2f} rad/s)"
        )

        if self.node._have_odom:
            self.lbl_odom.setText(
                f"Heading: {self.node.heading_deg:+.1f}°   "
                f"Speed: {self.node.odom_speed_mps:+.2f} m/s   "
                f"Trip: {self.node.trip_m:.2f} m   Total: {self.node.total_m:.2f} m"
            )
        else:
            self.lbl_odom.setText("Odometry: (no /odom yet)")

    def closeEvent(self, e):
        try: self.node.send(0.0, 0.0)
        except Exception: pass
        e.accept()


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    node = TeleopNode()
    win = TeleopWindow(node); win.show()

    # Service ROS events
    spin_timer = QTimer(); spin_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    spin_timer.start(10)

    try:
        rc = app.exec_()
    finally:
        try: node.send(0.0, 0.0)
        except Exception: pass
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)

if __name__ == "__main__":
    main()
