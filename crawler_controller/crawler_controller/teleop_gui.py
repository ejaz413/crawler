#!/usr/bin/env python3
import sys
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from PyQt5.QtCore import Qt, QTimer, QSize, QEvent
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QLabel, QSlider, QCheckBox,
    QHBoxLayout, QVBoxLayout, QGridLayout, QGroupBox, QSizePolicy
)

def quat_to_yaw_deg(qz: float, qw: float) -> float:
    yaw_rad = 2.0 * math.atan2(qz, qw)
    yaw_deg = math.degrees(yaw_rad)
    while yaw_deg >= 180.0:
        yaw_deg -= 360.0
    while yaw_deg < -180.0:
        yaw_deg += 360.0
    return yaw_deg


class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_gui")

        # Parameters
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("quick_stop_topic", "/quick_stop")
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("linear_sign", 1.0)
        self.declare_parameter("angular_sign", 1.0)

        # Image header (replace text help). Default path set for convenience; override via ROS param.
        self.declare_parameter("help_image_path", "/home/pavetech/crawler_ws/src/crawler_controller/crawler_controller/logo.png")
        self.declare_parameter("help_image_max_height", 180)  # px cap

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        self.odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.quick_stop_topic = self.get_parameter("quick_stop_topic").get_parameter_value().string_value
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.linear_sign = float(self.get_parameter("linear_sign").value)
        self.angular_sign = float(self.get_parameter("angular_sign").value)

        self.help_image_path = self.get_parameter("help_image_path").get_parameter_value().string_value
        self.help_image_max_h = int(self.get_parameter("help_image_max_height").value)

        # Publishers / Subscribers
        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)
        self.pub_qs = self.create_publisher(Bool, self.quick_stop_topic, 10)

        # Odometry state
        self._have_odom = False
        self._last_x = None
        self._last_y = None
        self.trip_m = 0.0
        self.total_m = 0.0
        self.odom_speed_mps = 0.0
        self.heading_deg = 0.0

        # Quick Stop status text for UI
        self.last_qs_status = "—"

    def send(self, v_cmps: float, w_degps: float):
        msg = Twist()
        msg.linear.x = (v_cmps / 100.0) * self.linear_sign
        msg.angular.z = math.radians(w_degps) * self.angular_sign
        self.pub.publish(msg)

    # Quick Stop helpers
    def quick_stop(self):
        self.pub_qs.publish(Bool(data=True))
        self.last_qs_status = "QuickStop sent (True)"

    def recover(self):
        self.pub_qs.publish(Bool(data=False))
        self.last_qs_status = "Recover sent (False)"

    def reset_trip(self):
        self.trip_m = 0.0

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
            if ds > 1e-5:
                self.trip_m += ds
                self.total_m += ds
                self._have_odom = True

        self._last_x, self._last_y = x, y


class TeleopGUI(QWidget):
    def __init__(self, node: TeleopNode, app: QApplication):
        super().__init__()
        self.node = node
        self.app = app

        # Speed caps
        self.lin_cap_cm = 50.0
        self.ang_cap_deg = 120.0
        self.step_cm = 10.0
        self.step_deg = 15.0

        # Motion state
        self.v_cm = 0.0
        self.w_deg = 0.0
        self.decay = 0.9
        self.left_held = False
        self.right_held = False
        self.inplace_when_steering = False

        # Image state
        self.help_pixmap = None

        self.setWindowTitle("Pavetech")
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
        # Header image (replaces text help)
        # --- Header: big bold title + logo ---
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

        gb_help = QGroupBox("")  # no small groupbox caption
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

        # Quick Stop Buttons
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

        # Odometry
        self.lbl_odom = QLabel("Odometry: waiting…")
        self.btn_trip_reset = QPushButton("Reset Trip")
        self.btn_trip_reset.clicked.connect(self.node.reset_trip)

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

        gb_spd = QGroupBox("Tuning Paramters for Linear and Angular Speed")
        lay_spd = QVBoxLayout()
        lay_spd.addLayout(row_lin)
        lay_spd.addLayout(row_ang)
        gb_spd.setLayout(lay_spd)

        self.lbl_status = QLabel()
        self.lbl_status.setStyleSheet("font-family: monospace;")

        gb_odom = QGroupBox("Odometry (from /odom)")
        lay_odom = QVBoxLayout()
        lay_odom.addWidget(self.lbl_odom)
        lay_odom.addWidget(self.btn_trip_reset)
        gb_odom.setLayout(lay_odom)

        # Root layout
        root = QVBoxLayout()
        root.addWidget(gb_help)
        root.addWidget(gb_ctrl)
        root.addWidget(gb_qs)
        root.addWidget(gb_spd)
        root.addWidget(self.lbl_status)
        root.addWidget(gb_odom)
        self.setLayout(root)
        self.resize(760, 560)
        self.setFocusPolicy(Qt.StrongFocus)

        # Set initial scaled image
        self._refresh_help_image()

    # --- image loading & scaling ---
    def _load_help_image(self):
        pm = QPixmap(self.node.help_image_path) if self.node.help_image_path else QPixmap()
        if pm.isNull():
            # Hide the group if not found (explicitly no text fallback)
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

    # ---------- Event filter ----------
    def eventFilter(self, obj, e):
        if e.type() == QEvent.KeyPress:
            if e.isAutoRepeat():
                return False
            self._on_key_press(e.key())
            return True
        elif e.type() == QEvent.KeyRelease:
            if e.isAutoRepeat():
                return False
            self._on_key_release(e.key())
            return True
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

    # ---------- Motion ----------
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

    # ---------- Quick Stop buttons ----------
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

    def _update_status(self):
        v_mps = (self.v_cm / 100.0) * self.node.linear_sign
        w_rps = math.radians(self.w_deg) * self.node.angular_sign
        self.lbl_lin.setText(f"{self.lin_cap_cm:.0f} ")
        self.lbl_ang.setText(f"{self.ang_cap_deg:.0f} ")
        
        self.lbl_qs.setText(f"Quick Stop: {self.node.last_qs_status}")

        if self.node._have_odom:
            self.lbl_odom.setText(
                f"Heading: {self.node.heading_deg:+.1f}°   "
                f"Speed: {self.node.odom_speed_mps:+.2f} m/s   "
                f"Trip: {self.node.trip_m:.2f} m   Total: {self.node.total_m:.2f} m"
            )
        else:
            self.lbl_odom.setText("Odometry: (no /odom yet)")

    def closeEvent(self, e):
        try:
            self.node.send(0.0, 0.0)
        except Exception:
            pass
        e.accept()


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    node = TeleopNode()
    gui = TeleopGUI(node, app)
    gui.show()

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
