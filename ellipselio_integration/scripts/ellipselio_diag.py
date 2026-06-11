#!/usr/bin/env python3
# EllipseLIO runaway diagnostic: correlate the odom trajectory with EllipseLIO's per-frame
# registration health (/analytics). On each odom sample, snapshot the latest analytics so we
# can see whether feature count / observability COLLAPSE at the moment the pose runs away.
# Usage: ellipselio_diag.py <csv_out> [DUR]
import rclpy, math, signal, sys, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from ellipselio.msg import EllipseLioAnalytics

CSV = sys.argv[1] if len(sys.argv) > 1 else '/root/ros2_ws/results/diag.csv'
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 600.0

class D(Node):
    def __init__(self):
        super().__init__('ellipselio_diag')
        be = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, '/ellipselio_odom', self.odom, be)
        self.create_subscription(EllipseLioAnalytics, '/analytics', self.ana, be)
        self.a = None; self.x0 = None; self.prev = None; self.t0 = None; self.last = -9
        self.f = open(CSV, 'w')
        self.f.write("t,disp,step,scan_size,num_feats,num_planes,num_lines,num_balls,num_reject,res_mean,obs_score,obs_min,rng_mean,kf_iter\n")
    def ana(self, m): self.a = m
    def odom(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec*1e-9
        if self.t0 is None: self.t0 = t
        rt = t - self.t0
        p = m.pose.pose.position
        if self.x0 is None: self.x0 = (p.x,p.y,p.z)
        disp = math.sqrt((p.x-self.x0[0])**2+(p.y-self.x0[1])**2+(p.z-self.x0[2])**2)
        step = 0.0 if self.prev is None else math.sqrt((p.x-self.prev[0])**2+(p.y-self.prev[1])**2+(p.z-self.prev[2])**2)
        self.prev = (p.x,p.y,p.z)
        a = self.a
        if a is not None:
            self.f.write(f"{rt:.3f},{disp:.4f},{step:.4f},{a.scan_size},{a.num_feats},{a.num_planes},"
                         f"{a.num_lines},{a.num_balls},{a.num_reject},{a.res_mean:.4f},{a.obs_score:.4f},"
                         f"{a.obs_min:.4f},{a.rng_mean:.3f},{a.kf_iterations}\n")
            if rt - self.last >= 2.0:
                self.last = rt
                print(f"t={rt:6.1f} disp={disp:6.2f} step={step:4.2f} | feats={a.num_feats:5d} "
                      f"(P{a.num_planes}/L{a.num_lines}/B{a.num_balls}) rej={a.num_reject:4d} "
                      f"obs={a.obs_score:.2f} res={a.res_mean:.3f} rng={a.rng_mean:.1f}", flush=True)

def main():
    rclpy.init(); n = D()
    stop = {'v': False}
    signal.signal(signal.SIGTERM, lambda *a: stop.update(v=True))
    signal.signal(signal.SIGINT, lambda *a: stop.update(v=True))
    s = time.time()
    while rclpy.ok() and not stop['v'] and (time.time()-s) < DUR:
        rclpy.spin_once(n, timeout_sec=0.2)
    n.f.flush(); n.f.close()
    print("DIAG DONE", flush=True)

if __name__ == '__main__':
    main()
