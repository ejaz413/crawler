#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import ctypes, time, atexit

DEVICE_NAME    = b"EPOS4"
PROTOCOL_STACK = b"MAXON SERIAL V2"
INTERFACE      = b"USB"

class EPOSDriver(Node):
    def __init__(self):
        super().__init__('epos_driver')

        # Parameters
        self.declare_parameter('ports', ['USB0', 'USB1'])
        self.declare_parameter('node_ids', [1, 1])
        self.declare_parameter('invert', [True, False])
        self.declare_parameter('max_rpm', 6000)
        self.declare_parameter('turn_gain', 0.6)
        self.declare_parameter('acc_rpm_s', 1000)
        self.declare_parameter('dec_rpm_s', 1000)
        self.declare_parameter('library_path', '')
        self.declare_parameter('sample_period', 0.2)

        ports        = self.get_parameter('ports').get_parameter_value().string_array_value
        node_ids     = self.get_parameter('node_ids').get_parameter_value().integer_array_value
        invert       = self.get_parameter('invert').get_parameter_value().bool_array_value
        self.max_rpm = int(self.get_parameter('max_rpm').value)
        self.turn_k  = float(self.get_parameter('turn_gain').value)
        self.acc     = int(self.get_parameter('acc_rpm_s').value)
        self.dec     = int(self.get_parameter('dec_rpm_s').value)
        lib_path     = self.get_parameter('library_path').value

        # Load EPOS lib
        try:
            self.epos = ctypes.cdll.LoadLibrary(lib_path)
        except Exception as e:
            self.get_logger().fatal(f"Failed to load EPOS library: {e}")
            raise

        self.error_code = ctypes.c_uint()

        # Motors: left index 0, right index 1
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
        self.get_logger().info("EPOS devices ready (subscribing to /cmd_vel)")

        # Subscriptions
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)

        # Safety
        atexit.register(self._halt_all)

    # ---- EPOS wrappers ----
    def _open(self, port):
        return self.epos.VCS_OpenDevice(
            DEVICE_NAME, PROTOCOL_STACK, INTERFACE, port, ctypes.byref(self.error_code)
        )

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
        # 3 = Profile Velocity
        return self.epos.VCS_SetOperationMode(h, nid, ctypes.c_int(3), ctypes.byref(self.error_code)) != 0

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

    # ---- Read wheel velocity ----
    def get_wheel_velocity(self):
        """Return (left_vel, right_vel) in RPM"""
        left_rpm = ctypes.c_long()
        right_rpm = ctypes.c_long()

        if self.epos.VCS_GetVelocity(self.motors[0]["handle"], self.motors[0]["node_id"], ctypes.byref(left_rpm), ctypes.byref(self.error_code)) == 0:
            left_rpm_val = 0
        else:
            left_rpm_val = -left_rpm.value if self.motors[0]["invert"] else left_rpm.value

        if self.epos.VCS_GetVelocity(self.motors[1]["handle"], self.motors[1]["node_id"], ctypes.byref(right_rpm), ctypes.byref(self.error_code)) == 0:
            right_rpm_val = 0
        else:
            right_rpm_val = -right_rpm.value if self.motors[1]["invert"] else right_rpm.value

        return left_rpm_val, right_rpm_val

    # ---- Init/halt helpers ----
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

    # ---- /cmd_vel handler ----
    def on_cmd_vel(self, msg: Twist):
        v = float(msg.linear.x)    # forward +, backward -
        w = float(msg.angular.z)   # left +, right -

        # Unitless mapping for now (m/s, rad/s → RPM can be added later)
        k_lin = self.max_rpm
        k_ang = self.max_rpm * self.turn_k

        left_rpm_robot  = self._clamp(int(k_lin * v + k_ang * w), -self.max_rpm, self.max_rpm)
        right_rpm_robot = self._clamp(int(k_lin * v - k_ang * w), -self.max_rpm, self.max_rpm)

        left_dev  = -left_rpm_robot  if self.motors[0]["invert"] else left_rpm_robot
        right_dev = -right_rpm_robot if self.motors[1]["invert"] else right_rpm_robot

        self._move_with_velocity(self.motors[0]["handle"], self.motors[0]["node_id"], left_dev)
        self._move_with_velocity(self.motors[1]["handle"], self.motors[1]["node_id"], right_dev)

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
