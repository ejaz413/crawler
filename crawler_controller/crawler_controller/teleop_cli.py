#!/usr/bin/env python3

import sys, termios, tty, select, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


Help ="""
W/A/S/D to move, SPACE to stop, ',' slower, '.' faster, 'q' quit
   W = Backward
   S = Forward
   A = left turn (in place)
   D = right turn (in place)

"""

class TeleopCli(Node):

    def __init__(self):
        super().__init__('teleop_cli')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lin = 0.5
        self.ang = 0.5
        print(Help)


    def send(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)

    def run(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            while True:
                if self._kbhit():
                    c = sys.stdin.read(1)

                    if c in ('q', 'Q'):
                        self.send(0.0, 0.0)
                        print("nQuit.")
                        break
                    elif c in ('w', 'W'):
                        self.send(self.lin, 0.0)
                        print("Forward")
                    elif c in ('s', 'S'):
                        self.send(-self.lin, 0.0)
                        print("Backward")
                    elif c in ('a', 'A'):
                        self.send(0.0, +self.ang)
                        print("Turn Left")
                    elif c in ('d', 'D'):
                        self.send(0.0, -self.ang)
                        print("Turn Right")
                    elif c == ' ':
                        self.send(0.0, 0.0)
                        print("Stop")
                    elif c == ',':
                        self.lin = max(0.1, self.lin - 0.1)
                        self.ang = max(0.1, self.ang - 0.1)
                        print(f"Speed down: lin={self.lin:.1f} ang={self.ang:.1f}")
                    elif c == '.':
                        self.lin = min(1.0, self.lin + 0.1)
                        self.ang = min(1.0, self.ang + 0.1)
                        print(f"Speed up: lin={self.lin:.1f} ang = {self.ang:.1f}")

                # small sleep to avoid busy loop

                time.sleep(0.02)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
    
    @staticmethod
    
    def _kbhit():
        return select.select([sys.stdin], [], [], 0)[0] !=[]
    
def main():
    rclpy.init()

    node = TeleopCli()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




