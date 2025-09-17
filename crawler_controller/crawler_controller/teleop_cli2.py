#!/usr/bin/env python3
import sys, termios, tty, select, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

HELP = """
W/A/S/D to move, SPACE to stop, ',' slower, '.' faster, 'q' quit
   W = Forward         (keeps steering; A/D to steer)
   S = Backward        (keeps steering; A/D to steer)
   A = hold to steer Left    (auto-straighten on release)
   D = hold to steer Right   (auto-straighten on release)
"""

def _sign(x: float) -> int:
    return 1 if x > 0 else (-1 if x < 0 else 0)

class TeleopCli(Node):
    def __init__(self):
        super().__init__('teleop_cli')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Configurable caps
        self.lin_cap = 0.5   # m/s
        self.ang_cap = 0.5   # rad/s
        self.step    = 0.1   # increment for '.' and ','
        self.steer_timeout_s = 0.20  # <- auto-straighten if no A/D key within this time

        # Current commanded state
        self.target_lin = 0.0
        self.target_ang = 0.0

        # Track last time we received a steering key (A/D)
        self._last_steer_ts = 0.0

        print(HELP)
        self._print_status()

    def _print_status(self):
        print(f"[vel] lin={self.target_lin:.2f} m/s  ang={self.target_ang:.2f} rad/s  "
              f"(caps: lin<=±{self.lin_cap:.2f}, ang<=±{self.ang_cap:.2f})")

    def _send(self):
        msg = Twist()
        msg.linear.x = float(self.target_lin)
        msg.angular.z = float(self.target_ang)
        self.pub.publish(msg)

    def _maybe_auto_straighten(self, now):
        # If we haven't seen A/D recently, go straight
        if self._last_steer_ts > 0.0 and (now - self._last_steer_ts) > self.steer_timeout_s:
            if self.target_ang != 0.0:
                self.target_ang = 0.0

    def run(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while rclpy.ok():
                now = time.time()

                # Non-blocking key read
                if self._kbhit():
                    c = sys.stdin.read(1)

                    if c in ('q', 'Q'):
                        self.target_lin = 0.0
                        self.target_ang = 0.0
                        self._send()
                        print("\nQuit.")
                        break

                    elif c in ('w', 'W'):
                        # Forward at current linear cap (steering unchanged; A/D to steer)
                        self.target_lin = +self.lin_cap
                        print("Forward")
                        self._print_status()

                    elif c in ('s', 'S'):
                        # Backward at current linear cap (steering unchanged; A/D to steer)
                        self.target_lin = -self.lin_cap
                        print("Backward")
                        self._print_status()

                    elif c in ('a', 'A'):
                        # HOLD to steer left; record timestamp for auto-straighten
                        self.target_ang = +self.ang_cap
                        self._last_steer_ts = now
                        # optional: print less frequently to avoid spam
                        print("Steer Left (hold)")

                    elif c in ('d', 'D'):
                        # HOLD to steer right; record timestamp for auto-straighten
                        self.target_ang = -self.ang_cap
                        self._last_steer_ts = now
                        print("Steer Right (hold)")

                    elif c == ' ':
                        self.target_lin = 0.0
                        self.target_ang = 0.0
                        print("Stop")
                        self._print_status()

                    elif c == ',':
                        # Decrease caps; rescale current speeds
                        self.lin_cap = max(0.1, self.lin_cap - self.step)
                        self.ang_cap = max(0.1, self.ang_cap - self.step)
                        if self.target_lin != 0.0:
                            self.target_lin = _sign(self.target_lin) * self.lin_cap
                        if self.target_ang != 0.0:
                            self.target_ang = _sign(self.target_ang) * self.ang_cap
                        print("Speed down")
                        self._print_status()

                    elif c == '.':
                        # Increase caps; rescale current speeds
                        self.lin_cap = min(1.5, self.lin_cap + self.step)
                        self.ang_cap = min(1.5, self.ang_cap + self.step)
                        if self.target_lin != 0.0:
                            self.target_lin = _sign(self.target_lin) * self.lin_cap
                        if self.target_ang != 0.0:
                            self.target_ang = _sign(self.target_ang) * self.ang_cap
                        print("Speed up")
                        self._print_status()

                # Auto-straighten if A/D released (no repeats seen recently)
                self._maybe_auto_straighten(now)

                # Always publish the current state (50 Hz)
                self._send()
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(0.02)

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    @staticmethod
    def _kbhit():
        return select.select([sys.stdin], [], [], 0)[0] != []

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
