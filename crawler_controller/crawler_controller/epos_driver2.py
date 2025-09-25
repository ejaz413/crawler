#!/usr/bin/env python3
import ctypes
import math
import time
import atexit

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import Empty

import tf2_ros
from tf_transformations import quaternion_from_euler

DEVICE_NAME    = b"EPOS4"
PROTOCOL_STACK = b"MAXON SERIAL V2"
INTERFACE      = b"USB"


class EPOSDriver(Node):
    def __init__(self):
        super().__init__('epos_driver')

        # -------- Params --------
        # Comm / handles
        self.declare_parameter('ports', ['USB0', 'USB1'])
        self.declare_parameter('node_ids', [1, 1])
        self.declare_parameter('invert', [True, False])
        self.declare_parameter('library_path', '')

        # Command mapping (cmd_vel -> EPOS rpm)
        self.declare_parameter('max_rpm', 6000)
        self.declare_parameter('turn_gain', 0.6)
        self.declare_parameter('acc_rpm_s', 1000)
        self.declare_parameter('dec_rpm_s', 1000)

        # Geometry / sampling
        self.declare_parameter('sample_period', 0.02)      # 50 Hz default
        self.declare_parameter('wheel_radius', 0.07490)    # [m] YOUR wheel radius
        self.declare_parameter('wheel_separation', 0.512)  # [m] axle track
        self.declare_parameter('gear_ratio', 913.0)        # motor:wheel

        # Odom source & units
        self.declare_parameter('odom_source', 'position')  # 'position' | 'velocity'
        self.declare_parameter('velocity_unit', 'rpm')     # if odom_source='velocity'
        self.declare_parameter('encoder_cpr', 2048)        # counts/rev at motor (for position/velocity counts)
        self.declare_parameter('use_averaged', True)       # use GetVelocityIsAveraged if available

        # Filters / guards
        self.declare_parameter('rpm_deadband', 5.0)        # for rpm velocity reads
        self.declare_parameter('max_abs_meas', 1.0e9)      # clamp any readouts
        self.declare_parameter('max_dt', 0.5)              # skip integration if dt > max_dt
        self.declare_parameter('max_ds_tick', 0.5)         # [m] skip if per-tick ds too large (spike guard)

        # Calibration scales (post conversion)
        self.declare_parameter('lin_scale', 1.0)           # multiplies v (to fix radius/ratio)
        self.declare_parameter('ang_scale', 1.0)           # multiplies w (to fix separation)

        # Optional SDO for averaging window (model specific)
        self.declare_parameter('avg_cfg_enable', False)
        self.declare_parameter('avg_idx', 0x0000)
        self.declare_parameter('avg_subidx', 0x00)
        self.declare_parameter('avg_value_u32', 0)

        # Read params
        ports        = self.get_parameter('ports').get_parameter_value().string_array_value
        node_ids     = self.get_parameter('node_ids').get_parameter_value().integer_array_value
        invert       = self.get_parameter('invert').get_parameter_value().bool_array_value
        lib_path     = self.get_parameter('library_path').value

        self.max_rpm = int(self.get_parameter('max_rpm').value)
        self.turn_k  = float(self.get_parameter('turn_gain').value)
        self.acc     = int(self.get_parameter('acc_rpm_s').value)
        self.dec     = int(self.get_parameter('dec_rpm_s').value)

        self.sample_period = float(self.get_parameter('sample_period').value)
        self.wheel_radius  = float(self.get_parameter('wheel_radius').value)
        self.wheel_sep     = float(self.get_parameter('wheel_separation').value)
        self.gear_ratio    = float(self.get_parameter('gear_ratio').value)

        self.odom_source   = self.get_parameter('odom_source').get_parameter_value().string_value
        self.velocity_unit = self.get_parameter('velocity_unit').get_parameter_value().string_value
        self.encoder_cpr   = int(self.get_parameter('encoder_cpr').value)
        self.use_averaged  = bool(self.get_parameter('use_averaged').value)

        self.rpm_deadband  = float(self.get_parameter('rpm_deadband').value)
        self.max_abs_meas  = float(self.get_parameter('max_abs_meas').value)
        self.max_dt        = float(self.get_parameter('max_dt').value)
        self.max_ds_tick   = float(self.get_parameter('max_ds_tick').value)

        self.lin_scale     = float(self.get_parameter('lin_scale').value)
        self.ang_scale     = float(self.get_parameter('ang_scale').value)

        self.avg_cfg_enable = bool(self.get_parameter('avg_cfg_enable').value)
        self.avg_idx        = int(self.get_parameter('avg_idx').value)
        self.avg_subidx     = int(self.get_parameter('avg_subidx').value)
        self.avg_value_u32  = int(self.get_parameter('avg_value_u32').value)

        # Load EPOS library
        try:
            self.epos = ctypes.cdll.LoadLibrary(lib_path)
        except Exception as e:
            self.get_logger().fatal(f"Failed to load EPOS library: {e}")
            raise
        self.error_code = ctypes.c_uint()

        # Motors
        self.motors = []
        for i in range(2):
            self.motors.append({
                "port": ports[i].encode(),
                "node_id": int(node_ids[i]),
                "invert": bool(invert[i]),
                "handle": None,
            })

        # Open/init
        for m in self.motors:
            h = self._open(m["port"])
            if h == 0:
                raise RuntimeError(f"Could not open device at {m['port'].decode()}")
            m["handle"] = h
            self._init_node(h, m["node_id"])

        # Optional averaging SDO
        if self.avg_cfg_enable and self.avg_idx != 0:
            for m in self.motors:
                self._write_avg_window(m["handle"], m["node_id"])

        self.get_logger().info(f"Ready (/cmd_vel, /quick_stop). Odom source = {self.odom_source}")

        # ROS I/O
        qos = QoSProfile(depth=10)
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.sub_qs  = self.create_subscription(Bool, '/quick_stop', self.on_quick_stop, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.srv_reset = self.create_service(Empty, '/reset_odom', self._on_reset_odom)

        atexit.register(self._halt_all)

        # Odom state
        self.x = 0.0; self.y = 0.0; self.th = 0.0
        self.last_time = self.get_clock().now()
        self.quick_stopped = False
        self._vel_first_log = True

        # Position odom memory (for delta counts)
        self._last_pos_L = None
        self._last_pos_R = None

        self.odom_timer = self.create_timer(self.sample_period, self.update_odometry)

    # ---------- EPOS wrappers ----------
    def _open(self, port):
        return self.epos.VCS_OpenDevice(DEVICE_NAME, PROTOCOL_STACK, INTERFACE, port, ctypes.byref(self.error_code))

    def _close(self, h):
        self.epos.VCS_CloseDevice(h, ctypes.byref(self.error_code))

    def _get_fault(self, h, nid):
        f = ctypes.c_uint()
        if self.epos.VCS_GetFaultState(h, nid, ctypes.byref(f), ctypes.byref(self.error_code)) != 0:
            return f.value
        return None

    def _clear_fault(self, h, nid):
        return self.epos.VCS_ClearFault(h, nid, ctypes.byref(self.error_code)) != 0

    def _set_vel_mode(self, h, nid):
        return self.epos.VCS_SetOperationMode(h, nid, ctypes.c_int(3), ctypes.byref(self.error_code)) != 0  # Profile Velocity

    def _enable(self, h, nid):
        self.epos.VCS_SetState(h, nid, 2, ctypes.byref(self.error_code))
        time.sleep(0.05)
        self.epos.VCS_SetState(h, nid, 3, ctypes.byref(self.error_code))
        time.sleep(0.05)
        self.epos.VCS_SetEnableState(h, nid, ctypes.byref(self.error_code))
        time.sleep(0.05)

    def _set_vel_profile(self, h, nid, acc, dec):
        return self.epos.VCS_SetVelocityProfile(h, nid, ctypes.c_uint(acc), ctypes.c_uint(dec), ctypes.byref(self.error_code)) != 0

    def _move_with_velocity(self, h, nid, rpm):
        return self.epos.VCS_MoveWithVelocity(h, nid, ctypes.c_long(int(rpm)), ctypes.byref(self.error_code)) != 0

    def _halt(self, h, nid):
        if hasattr(self.epos, "VCS_HaltVelocity"):
            self.epos.VCS_HaltVelocity(h, nid, ctypes.byref(self.error_code))
        else:
            self.epos.VCS_MoveWithVelocity(h, nid, ctypes.c_long(0), ctypes.byref(self.error_code))

    def _quick_stop(self, h, nid):
        if hasattr(self.epos, "VCS_ActivateQuickStopState"):
            return self.epos.VCS_ActivateQuickStopState(h, nid, ctypes.byref(self.error_code)) != 0
        self._halt(h, nid); return True

    def _recover_from_quick_stop(self, h, nid):
        try:
            self._move_with_velocity(h, nid, 0)
            time.sleep(0.05)
            self._enable(h, nid)
            self._set_vel_mode(h, nid)
            self._set_vel_profile(h, nid, self.acc, self.dec)
            return True
        except Exception as e:
            self.get_logger().error(f"Recover from QuickStop failed: {e}")
            return False

    def _write_avg_window(self, h, nid):
        try:
            u32 = ctypes.c_uint32(self.avg_value_u32)
            ok = self.epos.VCS_SetObject(
                h, nid,
                ctypes.c_ushort(self.avg_idx),
                ctypes.c_ubyte(self.avg_subidx),
                ctypes.byref(u32),
                ctypes.c_uint(4),
                ctypes.byref(self.error_code)
            )
            if ok == 0:
                self.get_logger().warn(
                    f"Failed to set avg window 0x{self.avg_idx:04X}:{self.avg_subidx}={self.avg_value_u32} (err=0x{self.error_code.value:08X})"
                )
            else:
                self.get_logger().info(
                    f"Set avg window 0x{self.avg_idx:04X}:{self.avg_subidx}={self.avg_value_u32}"
                )
        except AttributeError:
            self.get_logger().warn("VCS_SetObject not found; skipping averaging SDO.")

    # ---------- Init/Halt ----------
    def _init_node(self, h, nid):
        f = self._get_fault(h, nid)
        if f:
            self.get_logger().warn(f"Fault 0x{f:08X} -> clearing")
            self._clear_fault(h, nid)
            time.sleep(0.1)
        self._set_vel_mode(h, nid)
        self._enable(h, nid)
        self._set_vel_profile(h, nid, self.acc, self.dec)

    def _halt_all(self):
        for m in self.motors:
            try:
                self._halt(m["handle"], m["node_id"])
                self._close(m["handle"])
            except Exception:
                pass
        self.get_logger().info("Motors halted and devices closed.")

    # ---------- ROS callbacks ----------
    def on_cmd_vel(self, msg: Twist):
        if self.quick_stopped:
            self.get_logger().warn("QuickStop active: ignoring /cmd_vel")
            return

        v = float(msg.linear.x)   # m/s
        w = float(msg.angular.z)  # rad/s

        # Simple mapping -> rpm
        k_lin = self.max_rpm
        k_ang = self.max_rpm * self.turn_k
        left_rpm_robot  = self._clamp(int(k_lin * v + k_ang * w), -self.max_rpm, self.max_rpm)
        right_rpm_robot = self._clamp(int(k_lin * v - k_ang * w), -self.max_rpm, self.max_rpm)

        left_dev  = -left_rpm_robot  if self.motors[0]["invert"] else left_rpm_robot
        right_dev = -right_rpm_robot if self.motors[1]["invert"] else right_rpm_robot

        self._move_with_velocity(self.motors[0]["handle"], self.motors[0]["node_id"], left_dev)
        self._move_with_velocity(self.motors[1]["handle"], self.motors[1]["node_id"], right_dev)

    def on_quick_stop(self, msg: Bool):
        if msg.data:
            ok_all = True
            for m in self.motors:
                ok_all &= self._quick_stop(m["handle"], m["node_id"])
            self.quick_stopped = True
            self.get_logger().warn("QuickStop activated." if ok_all else "QuickStop failed on a motor.")
        else:
            ok_all = True
            for m in self.motors:
                ok_all &= self._recover_from_quick_stop(m["handle"], m["node_id"])
            self.quick_stopped = not ok_all
            self.get_logger().info("Recovered from QuickStop." if ok_all else "Recovery failed on a motor.")

    def _on_reset_odom(self, req, resp):
        self.x = 0.0; self.y = 0.0; self.th = 0.0
        self.last_time = self.get_clock().now()
        self._last_pos_L = None
        self._last_pos_R = None
        self.get_logger().info("Odometry reset.")
        return resp

    # ---------- Low-level reads ----------
    def _read_velocity_raw(self):
        """Return motor velocity (L,R) in library units (rpm/counts/s/rad/s), inverted per param."""
        left = ctypes.c_long(); right = ctypes.c_long()
        # Choose function
        if self.use_averaged and hasattr(self.epos, "VCS_GetVelocityIsAveraged"):
            fn = self.epos.VCS_GetVelocityIsAveraged
        elif hasattr(self.epos, "VCS_GetVelocityIs"):
            fn = self.epos.VCS_GetVelocityIs
            if self.use_averaged and not hasattr(self, "_warned_no_avg"):
                self.get_logger().warn("Averaged velocity not available; using instantaneous.")
                self._warned_no_avg = True
        else:
            raise RuntimeError("No GetVelocity function found.")

        # Left
        if fn(self.motors[0]["handle"], self.motors[0]["node_id"], ctypes.byref(left), ctypes.byref(self.error_code)) == 0:
            self.get_logger().error(f"Read left velocity failed (err=0x{self.error_code.value:08X})")
        # Right
        if fn(self.motors[1]["handle"], self.motors[1]["node_id"], ctypes.byref(right), ctypes.byref(self.error_code)) == 0:
            self.get_logger().error(f"Read right velocity failed (err=0x{self.error_code.value:08X})")

        lv = -left.value  if self.motors[0]["invert"] else left.value
        rv = -right.value if self.motors[1]["invert"] else right.value
        lv = max(-self.max_abs_meas, min(self.max_abs_meas, float(lv)))
        rv = max(-self.max_abs_meas, min(self.max_abs_meas, float(rv)))
        return lv, rv

    def _read_position_counts(self):
        """Return motor absolute position counts (L,R), inverted per param."""
        left = ctypes.c_long(); right = ctypes.c_long()
        if not hasattr(self.epos, "VCS_GetPositionIs"):
            raise RuntimeError("VCS_GetPositionIs not available for position-based odometry.")
        if self.epos.VCS_GetPositionIs(self.motors[0]["handle"], self.motors[0]["node_id"], ctypes.byref(left), ctypes.byref(self.error_code)) == 0:
            self.get_logger().error(f"Read left position failed (err=0x{self.error_code.value:08X})")
        if self.epos.VCS_GetPositionIs(self.motors[1]["handle"], self.motors[1]["node_id"], ctypes.byref(right), ctypes.byref(self.error_code)) == 0:
            self.get_logger().error(f"Read right position failed (err=0x{self.error_code.value:08X})")
        lv = -left.value  if self.motors[0]["invert"] else left.value
        rv = -right.value if self.motors[1]["invert"] else right.value
        return float(lv), float(rv)

    # ---------- Conversions ----------
    def _motor_to_wheel_rad_s(self, motor_val: float) -> float:
        """Convert motor velocity (per velocity_unit) → wheel rad/s."""
        if self.velocity_unit == 'rpm':
            if abs(motor_val) < self.rpm_deadband:
                motor_val = 0.0
            motor_rad_s = (motor_val * 2.0 * math.pi) / 60.0
            return motor_rad_s / self.gear_ratio
        elif self.velocity_unit == 'counts_per_s':
            if self.encoder_cpr <= 0:
                if self._vel_first_log:
                    self.get_logger().error("encoder_cpr must be > 0 in counts_per_s mode")
                return 0.0
            motor_rev_s = motor_val / float(self.encoder_cpr)
            motor_rad_s = motor_rev_s * 2.0 * math.pi
            return motor_rad_s / self.gear_ratio
        elif self.velocity_unit == 'rad_per_s_motor':
            return motor_val / self.gear_ratio
        else:
            if self._vel_first_log:
                self.get_logger().error(f"Unknown velocity_unit '{self.velocity_unit}'")
            return 0.0

    def _counts_to_wheel_rad(self, delta_counts: float) -> float:
        """Convert motor delta counts → wheel radians."""
        if self.encoder_cpr <= 0:
            self.get_logger().error("encoder_cpr must be > 0 for position odometry")
            return 0.0
        motor_rev = delta_counts / float(self.encoder_cpr)
        motor_rad = motor_rev * 2.0 * math.pi
        wheel_rad = motor_rad / self.gear_ratio
        return wheel_rad

    # ---------- Odometry ----------
    def update_odometry(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0 or dt > self.max_dt:
            self.last_time = now
            return
        self.last_time = now

        if self.odom_source == 'position':
            # Read absolute positions
            pL, pR = self._read_position_counts()
            if self._last_pos_L is None or self._last_pos_R is None:
                self._last_pos_L, self._last_pos_R = pL, pR
                return  # wait for next tick to have deltas

            dL_counts = pL - self._last_pos_L
            dR_counts = pR - self._last_pos_R
            self._last_pos_L, self._last_pos_R = pL, pR

            # Convert to wheel travel [m]
            dL_rad = self._counts_to_wheel_rad(dL_counts)
            dR_rad = self._counts_to_wheel_rad(dR_counts)
            sL = dL_rad * self.wheel_radius
            sR = dR_rad * self.wheel_radius

            # Robot incremental motion
            ds = (sR + sL) * 0.3
            dth = (sR - sL) / self.wheel_sep

            # Spike guard
            if abs(ds) > self.max_ds_tick:
                self.get_logger().warn(f"Skip ds spike: {ds:.3f} m (>{self.max_ds_tick} m)")
                return

            # Apply calibration scales as **rates** over dt
            v = (ds / dt) * 0.4*self.lin_scale
            w = (dth / dt) * 0.17*self.ang_scale

        else:  # 'velocity'
            mL, mR = self._read_velocity_raw()
            if self._vel_first_log:
                self.get_logger().info(
                    f"Raw motor velocity: L={mL:.3f}, R={mR:.3f} (unit='{self.velocity_unit}', averaged={self.use_averaged})"
                )
            wL = self._motor_to_wheel_rad_s(mL)
            wR = self._motor_to_wheel_rad_s(mR)
            v_l = wL * self.wheel_radius
            v_r = wR * self.wheel_radius
            v = ((v_r + v_l) * 0.5) * self.lin_scale
            w = ((v_r - v_l) / self.wheel_sep) * self.ang_scale

        # Deadbands
        if abs(v) < 1e-4: v = 0.0
        if abs(w) < 1e-4: w = 0.0

        # Integrate pose
        self.x += v * math.cos(self.th) * dt
        self.y += v * math.sin(self.th) * dt
        self.th += w * dt

        # Publish odom
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self.th)
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        # TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

        self._vel_first_log = False

    # ---------- Utils ----------
    @staticmethod
    def _clamp(x, lo, hi):
        return max(lo, min(hi, x))


def main():
    rclpy.init()
    node = EPOSDriver()
    try:
        rclpy.spin(node)
    finally:
        node._halt_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
