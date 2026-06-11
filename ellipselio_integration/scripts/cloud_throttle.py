#!/usr/bin/env python3
# Rate-limit EllipseLIO's /cloud_scan down to /cloud_scan_lite for remote Foxglove.
# /cloud_scan is the fat "original" Ouster point type: 48 bytes/pt x ~8000 pts =
# ~396 KB/msg @20Hz ~= 63 Mbps -- more than a Tailscale/wifi link sustains, so the
# bridge's websocket send queue backs up (~5 MB, ~1s lag). Capping to ~5 Hz
# (~16 Mbps) fits the link. The accumulating /cloud_map (Foxglove Decay 0) + the tiny
# /ellipselio_path/_odom stay full-rate; only the live dense scan is throttled.
# Gates on the message STAMP so the rate is correct under sim time (bag replay) too,
# and resets on stamp rewind (bag restart).
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2

RATE_HZ = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0

class CloudThrottle(Node):
    def __init__(self):
        super().__init__('cloud_throttle')
        be = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(PointCloud2, '/cloud_scan_lite', be)
        self.sub = self.create_subscription(PointCloud2, '/cloud_scan', self.cb, be)
        self.min_dt = 1.0 / RATE_HZ
        self.last = None

    def cb(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.last is None or t < self.last or (t - self.last) >= self.min_dt:
            self.last = t
            self.pub.publish(m)

def main():
    rclpy.init()
    n = CloudThrottle()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
