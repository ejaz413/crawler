#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class SimpleStopFront(Node):
    """
    Drive a constant speed and STOP if an object is within stop_dist
    inside a ±front_deg_half window in front. No steering.
    """

    def __init__(self):
        super().__init__('simple_stop_front')

        # --- Parameters ---
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('cruise_speed', -0.30)     # your robot's forward = negative x
        self.declare_parameter('stop_dist', 0.40)         # meters (40 cm)
        self.declare_parameter('front_deg_half', 20.0)    # front window ±deg
        self.declare_parameter('scan_yaw_offset_deg', 0.0)# rotate if 0° isn't forward
        self.declare_parameter('min_range_margin', 0.03)  # ignore < range_min + margin

        self.scan_topic   = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.cmd_topic    = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.v_cruise     = float(self.get_parameter('cruise_speed').value)
        self.stop_dist    = float(self.get_parameter('stop_dist').value)
        self.deg_half     = float(self.get_parameter('front_deg_half').value)
        self.yaw_off_deg  = float(self.get_parameter('scan_yaw_offset_deg').value)
        self.min_margin   = float(self.get_parameter('min_range_margin').value)

        # --- ROS I/O ---
        self.pub_cmd = self.create_publisher(Twist, self.cmd_topic, 10)
        self.pub_dbg = self.create_publisher(String, '/simple_stop_debug', 10)
        # ✅ FIX: include the topic name (self.scan_topic) before the callback
        self.sub_scan = self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 10)

        self.get_logger().info(
            f"SimpleStopFront: stop_dist={self.stop_dist:.2f} m, "
            f"front=±{self.deg_half:.1f}°, cruise={self.v_cruise:.2f} m/s, "
            f"yaw_off={self.yaw_off_deg:.1f}°, margin={self.min_margin:.2f} m"
        )

    @staticmethod
    def _clamp(i, lo, hi): return max(lo, min(hi, i))

    def scan_cb(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0 or msg.angle_increment == 0.0:
            return

        # Where is "forward" (0 rad) in this scan array?
        target_rad = math.radians(self.yaw_off_deg)  # rotate if sensor not aligned
        idx_center = int(round((target_rad - msg.angle_min) / msg.angle_increment))
        idx_center = self._clamp(idx_center, 0, n - 1)

        half_rad   = math.radians(self.deg_half)
        half_steps = int(math.ceil(half_rad / abs(msg.angle_increment)))
        i0 = self._clamp(idx_center - half_steps, 0, n - 1)
        i1 = self._clamp(idx_center + half_steps, 0, n - 1)

        # Filter usable ranges
        rmin = max(0.0, msg.range_min + self.min_margin)
        rmax = msg.range_max if msg.range_max > 0.0 else float('inf')

        front_min = float('inf')
        for i in range(i0, i1 + 1):
            r = msg.ranges[i]
            if math.isfinite(r) and rmin <= r <= rmax and r < front_min:
                front_min = r

        # Decide velocity
        v = 0.0 if (front_min <= self.stop_dist) else self.v_cruise
        self._send(v)

        # Debug
        dbg = String()
        dbg.data = f"front_min={front_min:.3f} m | v={v:.2f} m/s"
        self.pub_dbg.publish(dbg)

    def _send(self, v):
        msg = Twist()
        msg.linear.x  = float(v)
        msg.angular.z = 0.0
        self.pub_cmd.publish(msg)

def main():
    rclpy.init()
    node = SimpleStopFront()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._send(0.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
