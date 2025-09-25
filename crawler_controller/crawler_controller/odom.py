#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math
import time

# Import your EPOS driver
from epos_driver import EPOSDriver  # Make sure the python file is in your path

# Wheel parameters
RADIUS = 0.06891  # meters
TRACK  = 0.512    # meters

class TeleopWithOdometry(Node):
    def __init__(self):
        super().__init__('teleop_with_odom')

        # Initialize EPOS driver
        self.epos = EPOSDriver()  # your driver instance

        # Odometry publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.02, self.update_odometry)  # 50 Hz

        # Initial pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

    def rpm_to_rads(self, rpm):
        return rpm * 2.0 * math.pi / 60.0

    def update_odometry(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Get wheel velocities from EPOS driver
        vl_rpm = self.epos.get_wheel_velocity(0)  # left wheel
        vr_rpm = self.epos.get_wheel_velocity(1)  # right wheel

        # Convert to linear velocity (m/s)
        vl = self.rpm_to_rads(vl_rpm) * RADIUS
        vr = self.rpm_to_rads(vr_rpm) * RADIUS

        # Differential drive kinematics
        v = (vr + vl) / 2.0
        w = (vr - vl) / TRACK

        # Integrate pose
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt
        self.theta = (self.theta + math.pi) % (2*math.pi) - math.pi

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        qz = math.sin(self.theta/2.0)
        qw = math.cos(self.theta/2.0)
        odom.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)

def main():
    rclpy.init()
    node = TeleopWithOdometry()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
