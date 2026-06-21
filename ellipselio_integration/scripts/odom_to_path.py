#!/usr/bin/env python3
# Republish EllipseLIO's /ellipselio_odom (nav_msgs/Odometry, BEST_EFFORT) as a
# nav_msgs/Path on /ellipselio_path so Foxglove can draw the trajectory as a line.
# EllipseLIO itself publishes only Odometry + TF, no Path. Resets the path when
# sim-time jumps backwards (bag restart) so each replay draws a fresh trajectory.
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

# Match cloud_throttle's 180-deg view rotation baked into the DATA so the path overlays the
# cloud and reads upright (EllipseLIO's /odom_ellipselio Z is down -> a lift draws as -z).
#   native = unchanged, rollX = flip y,z, pitchY = flip x,z, yawZ = flip x,y
MODE = sys.argv[1] if len(sys.argv) > 1 else 'native'
_NEG = {'native': (), 'rollX': ('y', 'z'), 'pitchY': ('x', 'z'), 'yawZ': ('x', 'y')}

class OdomToPath(Node):
    def __init__(self):
        super().__init__('odom_to_path')
        be = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Path, '/ellipselio_path', 10)
        self.sub = self.create_subscription(Odometry, '/ellipselio_odom', self.cb, be)
        self.path = Path()
        self.last_t = None
        self.neg = _NEG.get(MODE, ())
        self.get_logger().info(f'odom_to_path rotate mode={MODE} (negate {self.neg})')

    def cb(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        # bag looped / time went backwards -> start a fresh path
        if self.last_t is not None and t < self.last_t - 1.0:
            self.path.poses = []
        self.last_t = t
        ps = PoseStamped()
        ps.header = m.header
        ps.pose = m.pose.pose
        if self.neg:
            p = ps.pose.position
            if 'x' in self.neg: p.x = -p.x
            if 'y' in self.neg: p.y = -p.y
            if 'z' in self.neg: p.z = -p.z
        self.path.header = m.header  # frame_id = odom_ellipselio
        self.path.poses.append(ps)
        # cap length so it never grows unbounded
        if len(self.path.poses) > 20000:
            self.path.poses = self.path.poses[-20000:]
        self.pub.publish(self.path)

def main():
    rclpy.init()
    n = OdomToPath()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
