#!/usr/bin/env python3
"""Live tracker: compare ArduPilot local_position vs SLAM vision_pose."""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped


class Tracker(Node):
    def __init__(self):
        super().__init__("lp_vp_tracker")
        self.vp = None
        self.lp = None
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(PoseStamped, "/mavros/vision_pose/pose", self._on_vp, qos)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose", self._on_lp, qos)
        self.create_timer(0.5, self._print)
        self.count = 0

    def _on_vp(self, msg):
        self.vp = msg.pose

    def _on_lp(self, msg):
        self.lp = msg.pose

    def _yaw(self, q):
        return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                       1 - 2 * (q.y ** 2 + q.z ** 2)))

    def _print(self):
        if not self.vp or not self.lp:
            return
        v = self.vp.position
        l = self.lp.position
        dx = l.x - v.x
        dy = l.y - v.y
        dz = l.z - v.z
        dist = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
        yv = self._yaw(self.vp.orientation)
        yl = self._yaw(self.lp.orientation)
        dyaw = yl - yv
        if dyaw > 180:
            dyaw -= 360
        if dyaw < -180:
            dyaw += 360

        if self.count % 20 == 0:
            print()
            print(f"{'VP_X':>8s} {'VP_Y':>8s} {'VP_Z':>8s} | "
                  f"{'LP_X':>8s} {'LP_Y':>8s} {'LP_Z':>8s} | "
                  f"{'dX':>7s} {'dY':>7s} {'dZ':>7s} {'dist':>6s} {'dYaw':>6s}")
            print("-" * 100)

        print(f"{v.x:8.3f} {v.y:8.3f} {v.z:8.3f} | "
              f"{l.x:8.3f} {l.y:8.3f} {l.z:8.3f} | "
              f"{dx:+7.3f} {dy:+7.3f} {dz:+7.3f} {dist:6.3f} {dyaw:+6.1f}d")
        self.count += 1


def main():
    rclpy.init()
    rclpy.spin(Tracker())


if __name__ == "__main__":
    main()
