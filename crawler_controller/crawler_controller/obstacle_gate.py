#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult

def clamp(v, lo, hi): return max(lo, min(hi, v))

class LidarAvoidSides(Node):
    """
    STOP if obstacle < stop_dist in FRONT sector.
    Turn LEFT if obstacle < right_dist in RIGHT sector.
    Turn RIGHT if obstacle < left_dist  in LEFT  sector.

    Angles are robot-frame degrees: 0° forward (sensor arrow), +CCW.
    Works with scans in -180..+180 or 0..360. Use scan_yaw_offset_deg if the
    sensor is rotated relative to robot forward.
    """

    def __init__(self):
        super().__init__('obstacle_gate')

        # Topics
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # Sectors (deg)
        self.declare_parameter('front_deg_min', -25.0)
        self.declare_parameter('front_deg_max',  25.0)
        self.declare_parameter('left_deg_min',    25.0)
        self.declare_parameter('left_deg_max',    90.0)
        self.declare_parameter('right_deg_min',  -90.0)
        self.declare_parameter('right_deg_max',  -25.0)

        # Thresholds (m)
        self.declare_parameter('stop_dist',  0.50)
        self.declare_parameter('left_dist',  0.30)
        self.declare_parameter('right_dist', 0.40)

        # Motion & smoothing
        self.declare_parameter('cruise_lin', 0.30)   # magnitude; sign applied by linear_sign
        self.declare_parameter('slow_lin',   0.15)
        self.declare_parameter('ang_cap',    0.6)
        self.declare_parameter('steer_timeout_s', 0.20)
        self.declare_parameter('decay_factor',    0.85)
        self.declare_parameter('publish_hz',      20.0)

        # Mounting
        self.declare_parameter('scan_yaw_offset_deg', 0.0)

        # Forward sign (+1 normal, -1 if robot's forward is negative Twist.x)
        self.declare_parameter('linear_sign', -1.0)  # default -1.0 for your robot

        # Near-range filter (ignore returns closer than range_min + margin)
        self.declare_parameter('min_range_margin', 0.05)

        # Typed string array to avoid BYTE_ARRAY inference
        self.declare_parameter(
            'ignore_deg_spans_str',
            [''],  # non-empty default => STRING_ARRAY; blanks ignored
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY),
        )

        # ---- Read params ----
        self.scan_topic    = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value

        self.front_deg_min = float(self.get_parameter('front_deg_min').value)
        self.front_deg_max = float(self.get_parameter('front_deg_max').value)
        self.left_deg_min  = float(self.get_parameter('left_deg_min').value)
        self.left_deg_max  = float(self.get_parameter('left_deg_max').value)
        self.right_deg_min = float(self.get_parameter('right_deg_min').value)
        self.right_deg_max = float(self.get_parameter('right_deg_max').value)

        self.stop_dist     = float(self.get_parameter('stop_dist').value)
        self.left_dist     = float(self.get_parameter('left_dist').value)
        self.right_dist    = float(self.get_parameter('right_dist').value)

        self.cruise_lin    = float(self.get_parameter('cruise_lin').value)
        self.slow_lin      = float(self.get_parameter('slow_lin').value)
        self.ang_cap       = float(self.get_parameter('ang_cap').value)
        self.steer_timeout_s = float(self.get_parameter('steer_timeout_s').value)
        self.decay_factor    = float(self.get_parameter('decay_factor').value)
        self.publish_hz      = float(self.get_parameter('publish_hz').value)

        self.scan_yaw_offset_deg = float(self.get_parameter('scan_yaw_offset_deg').value)
        self.linear_sign    = float(self.get_parameter('linear_sign').value)
        self.min_range_margin = float(self.get_parameter('min_range_margin').value)

        spans_str = [str(s) for s in self.get_parameter('ignore_deg_spans_str').value]
        self.ignore_spans = self._parse_ignore_spans(spans_str)

        # ROS I/O
        self.pub_cmd   = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.pub_dbg   = self.create_publisher(Float32MultiArray, '/avoid_debug', 10)
        self.pub_dbg_t = self.create_publisher(String, '/avoid_debug_text', 10)
        self.sub_scan  = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)

        # State
        self._latest = None
        self._ignore_idx = None
        self._range_min = 0.0
        self._range_max = float('inf')
        self.v = 0.0
        self.w = 0.0
        self._last_steer_ts = 0.0
        self.front_min = float('inf')
        self.left_min  = float('inf')
        self.right_min = float('inf')

        # Control loop
        self.dt = 1.0 / self.publish_hz
        self.timer = self.create_timer(self.dt, self.on_tick)

        # Hot-reload for key params
        self.add_on_set_parameters_callback(self._on_param_set)

        self.get_logger().info(
            f"obstacle_gate started. Ignoring spans (deg): {self.ignore_spans}; "
            f"linear_sign={self.linear_sign}, min_range_margin={self.min_range_margin} m"
        )

    # ---------- utils ----------
    @staticmethod
    def _norm360(d):  # to [0,360)
        return (d % 360.0 + 360.0) % 360.0

    @staticmethod
    def _span_contains(deg_norm, d0, d1):
        d0n = (d0 % 360.0 + 360.0) % 360.0
        d1n = (d1 % 360.0 + 360.0) % 360.0
        if d0n <= d1n:
            return d0n <= deg_norm <= d1n
        else:
            return deg_norm >= d0n or deg_norm <= d1n  # wrap across 0°

    def _parse_ignore_spans(self, spans_str_list):
        spans = []
        for s in spans_str_list:
            s = s.strip()
            if not s:
                continue
            try:
                a, b = s.split(',')
                spans.append([float(a.strip()), float(b.strip())])
            except Exception:
                self.get_logger().warn(f"Bad ignore span '{s}', expected 'a,b'")
        return spans

    def _build_ignore_index_set(self, n, angle_min, angle_inc):
        mask = set()
        for i in range(n):
            ang = angle_min + i * angle_inc
            deg_scan = math.degrees(ang)
            deg_robot = self._norm360(deg_scan + self.scan_yaw_offset_deg)
            for d0, d1 in self.ignore_spans:
                if self._span_contains(deg_robot, d0, d1):
                    mask.add(i)
                    break
        return mask

    def _sector_min(self, ranges, angle_min, angle_inc, deg_min, deg_max):
        n = len(ranges)
        # Apply yaw offset, then normalize bounds to 0..360
        d0 = self._norm360(deg_min + self.scan_yaw_offset_deg)
        d1 = self._norm360(deg_max + self.scan_yaw_offset_deg)

        am_deg = math.degrees(angle_min)
        use360 = -10.0 <= am_deg <= 10.0  # if angle_min near 0 -> 0..360 domain

        def deg_to_idx(deg_norm):
            if use360:
                rad = math.radians(deg_norm)
            else:
                rad = math.radians(deg_norm - 360.0) if deg_norm > 180.0 else math.radians(deg_norm)
            idx = int(round((rad - angle_min) / angle_inc))
            return max(0, min(n - 1, idx))

        indices = []
        if d0 <= d1:
            i0, i1 = deg_to_idx(d0), deg_to_idx(d1)
            lo, hi = (i0, i1) if i0 <= i1 else (i1, i0)
            indices.extend(range(lo, hi + 1))
        else:
            iA0 = deg_to_idx(d0); indices.extend(range(iA0, n))
            iB1 = deg_to_idx(d1); indices.extend(range(0, iB1 + 1))

        m = float('inf')
        for i in indices:
            if self._ignore_idx and i in self._ignore_idx:
                continue

            r = ranges[i]
            if not math.isfinite(r):
                continue

            # Near-range / far-range filtering
            if r < (self._range_min) or r > (self._range_max):
                continue

            if r < m:
                m = r
        return m

    # ---------- callbacks ----------
    def on_scan(self, msg: LaserScan):
        self._range_min = max(0.0, msg.range_min + self.min_range_margin)
        self._range_max = msg.range_max if msg.range_max > 0.0 else float('inf')

        ranges = list(msg.ranges)
        self._latest = (ranges, msg.angle_min, msg.angle_increment)
        if self._ignore_idx is None:
            self._ignore_idx = self._build_ignore_index_set(len(ranges), msg.angle_min, msg.angle_increment)

    def on_tick(self):
        now = self.get_clock().now().nanoseconds / 1e9

        desired_v = self.cruise_lin
        steer_req = 0  # -1 right, +1 left

        if self._latest is not None:
            ranges, angle_min, angle_inc = self._latest

            self.front_min = self._sector_min(ranges, angle_min, angle_inc, self.front_deg_min, self.front_deg_max)
            self.left_min  = self._sector_min(ranges, angle_min, angle_inc, self.left_deg_min,  self.left_deg_max)
            self.right_min = self._sector_min(ranges, angle_min, angle_inc, self.right_deg_min, self.right_deg_max)

            if self.front_min < self.stop_dist:
                desired_v = 0.0
                steer_req = 0
            else:
                left_close  = (self.left_min  < self.left_dist)
                right_close = (self.right_min < self.right_dist)

                if right_close and not left_close:
                    steer_req = +1  # obstacle on right -> turn left
                elif left_close and not right_close:
                    steer_req = -1  # obstacle on left  -> turn right
                elif left_close and right_close:
                    steer_req = +1 if self.left_min > self.right_min else -1

        if steer_req != 0 and desired_v > 0.0:
            self.w = self.ang_cap * (1.0 if steer_req > 0 else -1.0)
            self._last_steer_ts = now
            desired_v = max(self.slow_lin, desired_v * 0.75)
        else:
            if (now - self._last_steer_ts) > self.steer_timeout_s:
                self.w = 0.0 if abs(self.w) <= 0.01 else self.w * self.decay_factor

        # Apply sign so physical forward can be negative x
        self.v = desired_v * self.linear_sign

        self._publish_cmd(self.v, self.w)
        self._publish_debug()

    # Hot-reload for key params
    def _on_param_set(self, params):
        rebuild_ignore = False
        for p in params:
            try:
                if p.name == 'scan_yaw_offset_deg':
                    self.scan_yaw_offset_deg = float(p.value); rebuild_ignore = True
                elif p.name == 'ignore_deg_spans_str':
                    spans = []
                    for s in p.value:
                        s = str(s).strip()
                        if not s: continue
                        a, b = s.split(',')
                        spans.append([float(a), float(b)])
                    self.ignore_spans = spans; rebuild_ignore = True
                elif p.name == 'stop_dist':   self.stop_dist  = float(p.value)
                elif p.name == 'left_dist':   self.left_dist  = float(p.value)
                elif p.name == 'right_dist':  self.right_dist = float(p.value)
                elif p.name == 'cruise_lin':  self.cruise_lin = float(p.value)
                elif p.name == 'slow_lin':    self.slow_lin   = float(p.value)
                elif p.name == 'ang_cap':     self.ang_cap    = float(p.value)
                elif p.name == 'linear_sign': self.linear_sign = float(p.value)
                elif p.name == 'min_range_margin': self.min_range_margin = float(p.value)
            except Exception as e:
                return SetParametersResult(successful=False, reason=str(e))
        if rebuild_ignore:
            self._ignore_idx = None
        return SetParametersResult(successful=True)

    # ---------- pubs ----------
    def _publish_cmd(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub_cmd.publish(msg)

    def _publish_debug(self):
        arr = Float32MultiArray()
        arr.data = [float(self.front_min), float(self.left_min), float(self.right_min),
                    float(self.v), float(self.w)]
        self.pub_dbg.publish(arr)

        txt = String()
        txt.data = (f"front_min={self.front_min:.3f} m, "
                    f"left_min={self.left_min:.3f} m, right_min={self.right_min:.3f} m | "
                    f"v={self.v:.2f} m/s, w={self.w:.2f} rad/s")
        self.pub_dbg_t.publish(txt)

def main():
    rclpy.init()
    node = LidarAvoidSides()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish_cmd(0.0, 0.0)  # best-effort stop; ignore shutdown race
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
