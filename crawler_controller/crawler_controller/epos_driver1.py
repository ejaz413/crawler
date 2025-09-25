#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
import ctypes, time, math, atexit
from tf_transformations import quaternion_from_euler
import tf2_ros
from rclpy.qos import QoSProfile

DEVICE_NAME    = b"EPOS4"
PROTOCOL_STACK = b"MAXON SERIAL V2"
INTERFACE      = b"USB"

class EPOSDriver(Node):
    def __init__(self):
        super().__init__('epos_driver')

        # --- Parameters ---
        self.declare_parameter('ports', ['USB0','USB1'])
        self.declare_parameter('node_ids', [1,1])
        self.declare_parameter('invert', [True,False])
        self.declare_parameter('max_rpm', 6000)
        self.declare_parameter('turn_gain', 0.6)
        self.declare_parameter('acc_rpm_s', 1000)
        self.declare_parameter('dec_rpm_s', 1000)
        self.declare_parameter('library_path', '/home/pavetech/EPOS_Linux_Library/lib/arm/v8/libEposCmd.so.6.8.1.0')
        self.declare_parameter('wheel_radius', 0.06891)
        self.declare_parameter('wheel_separation', 0.512)
        self.declare_parameter('sample_period', 0.2)

        ports = self.get_parameter('ports').get_parameter_value().string_array_value
        node_ids = self.get_parameter('node_ids').get_parameter_value().integer_array_value
        invert = self.get_parameter('invert').get_parameter_value().bool_array_value
        self.max_rpm = self.get_parameter('max_rpm').value
        self.turn_k = self.get_parameter('turn_gain').value
        self.acc = self.get_parameter('acc_rpm_s').value
        self.dec = self.get_parameter('dec_rpm_s').value
        lib_path = self.get_parameter('library_path').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_sep = self.get_parameter('wheel_separation').value
        self.sample_period = self.get_parameter('sample_period').value

        # --- Load EPOS library ---
        self.epos = ctypes.cdll.LoadLibrary(lib_path)
        self.error_code = ctypes.c_uint()

        # --- Setup motors ---
        self.motors = []
        for i in range(2):
            self.motors.append({
                "port": ports[i].encode(),
                "node_id": int(node_ids[i]),
                "invert": bool(invert[i]),
                "handle": None
            })

        for m in self.motors:
            h = self._open(m["port"])
            if h == 0: raise RuntimeError(f"Cannot open device at {m['port'].decode()}")
            m["handle"] = h
            self._init_node(h, m["node_id"])
        self.get_logger().info("EPOS devices ready")

        # --- Subscriptions ---
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)

        # --- Odometry setup ---
        self.x = 0.0; self.y = 0.0; self.th = 0.0
        self.last_time = self.get_clock().now()

        qos = QoSProfile(depth=10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.odom_timer = self.create_timer(self.sample_period, self.update_odometry)

        atexit.register(self._halt_all)

    # ---- EPOS wrappers ----
    def _open(self, port):
        return self.epos.VCS_OpenDevice(DEVICE_NAME, PROTOCOL_STACK, INTERFACE, port, ctypes.byref(self.error_code))
    def _close(self, h):
        self.epos.VCS_CloseDevice(h, ctypes.byref(self.error_code))
    def _init_node(self, h, nid):
        self.epos.VCS_SetOperationMode(h, nid, ctypes.c_int(3), ctypes.byref(self.error_code))  # Profile Velocity
        self.epos.VCS_SetState(h, nid, 2, ctypes.byref(self.error_code))
        self.epos.VCS_SetState(h, nid, 3, ctypes.byref(self.error_code))
        self.epos.VCS_SetEnableState(h, nid, ctypes.byref(self.error_code))
        self.epos.VCS_SetVelocityProfile(h, nid, ctypes.c_uint(self.acc), ctypes.c_uint(self.dec), ctypes.byref(self.error_code))

    def _halt_all(self):
        for m in self.motors:
            try:
                self.epos.VCS_HaltVelocity(m["handle"], m["node_id"], ctypes.byref(self.error_code))
                self._close(m["handle"])
            except Exception: pass
        self.get_logger().info("Motors halted")

    # ---- Command velocity ----
    def on_cmd_vel(self, msg):
        v = msg.linear.x; w = msg.angular.z
        k_lin = self.max_rpm; k_ang = self.max_rpm*self.turn_k
        left_rpm = int(k_lin*v + k_ang*w)
        right_rpm = int(k_lin*v - k_ang*w)

        left_rpm = -left_rpm if self.motors[0]["invert"] else left_rpm
        right_rpm = -right_rpm if self.motors[1]["invert"] else right_rpm

        self.epos.VCS_MoveWithVelocity(self.motors[0]["handle"], self.motors[0]["node_id"], ctypes.c_long(left_rpm), ctypes.byref(self.error_code))
        self.epos.VCS_MoveWithVelocity(self.motors[1]["handle"], self.motors[1]["node_id"], ctypes.c_long(right_rpm), ctypes.byref(self.error_code))

    # ---- Odometry ----
    def get_wheel_velocity(self, wheel_index):
        val = ctypes.c_long()
        self.epos.VCS_GetVelocityIs(self.motors[wheel_index]["handle"], self.motors[wheel_index]["node_id"], ctypes.byref(val), ctypes.byref(self.error_code))
        return val.value

    def update_odometry(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        vl_rpm = self.get_wheel_velocity(0)
        vr_rpm = self.get_wheel_velocity(1)

        vl_mps = vl_rpm * 2*math.pi/60.0 * self.wheel_radius
        vr_mps = vr_rpm * 2*math.pi/60.0 * self.wheel_radius

        v = (vl_mps + vr_mps)/2.0
        w = (vr_mps - vl_mps)/self.wheel_sep

        self.x += v*math.cos(self.th)*dt
        self.y += v*math.sin(self.th)*dt
        self.th += w*dt

        q = quaternion_from_euler(0,0,self.th)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)

        t = tf2_ros.TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = EPOSDriver()
    try:
        rclpy.spin(node)
    finally:
        node._halt_all()
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
