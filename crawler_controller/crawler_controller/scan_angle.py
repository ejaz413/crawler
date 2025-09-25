#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class ScanReader(Node):
    def __init__(self):
        super().__init__('scan_reader')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.subscription

    def scan_callback(self, msg: LaserScan):
        angle = msg.angle_min
        for r in msg.ranges:
            distance = r
            deg = math.degrees(angle)
            if distance > 0.0:
                self.get_logger().info(f"Angle: {deg:.1f}, Distance: {distance:0.1f}mm")
                angle +=msg.angle_increment

def main(args=None):
    rclpy.init()
    node = ScanReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()