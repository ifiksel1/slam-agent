#!/usr/bin/env python3
# Standalone EllipseLIO trajectory sampler — NO framework-specific deps (just nav_msgs).
# Subscribes /ellipselio_odom and tracks the same FRAME-INVARIANT metrics used to compare
# SuperOdom / FAST-LIO on this rig: total path, peak excursion from start, final
# displacement-from-start (~loop closure if you return to origin), and max single-scan step
# (a divergence/jump detector). Writes a CSV and prints a summary on exit (SIGINT/SIGTERM/DUR).
#
# Usage: ellipselio_traj_sampler.py <csv_out> [DUR_secs] [odom_topic]
import rclpy, math, time, signal, sys
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry

CSV   = sys.argv[1] if len(sys.argv) > 1 else '/root/ros2_ws/results/ellipselio_traj.csv'
DUR   = float(sys.argv[2]) if len(sys.argv) > 2 else 600.0
TOPIC = sys.argv[3] if len(sys.argv) > 3 else '/ellipselio_odom'

class M(Node):
    def __init__(self):
        super().__init__('ellipselio_traj_sampler')
        # EllipseLIO publishes /ellipselio_odom as BEST_EFFORT — a RELIABLE subscriber gets
        # ZERO messages (incompatible QoS). Match BEST_EFFORT so we actually receive odom.
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, TOPIC, self.odom, qos)
        self.x0 = None; self.last = -9
        self.peak = 0.0; self.path = 0.0; self.prev = None
        self.maxstep = 0.0; self.n = 0; self.t0 = None
        self.f = open(CSV, 'w'); self.f.write("t,x,y,z,yaw,disp_from_start,step\n")

    def odom(self, m):
        # use the message stamp so timing matches the bag's sim clock, not wall time
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.t0 is None: self.t0 = t
        rt = t - self.t0
        p = m.pose.pose.position; q = m.pose.pose.orientation
        yaw = math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        if self.x0 is None: self.x0 = (p.x, p.y, p.z)
        disp = math.sqrt((p.x-self.x0[0])**2 + (p.y-self.x0[1])**2 + (p.z-self.x0[2])**2)
        self.peak = max(self.peak, disp)
        step = 0.0
        if self.prev is not None:
            step = math.sqrt((p.x-self.prev[0])**2 + (p.y-self.prev[1])**2 + (p.z-self.prev[2])**2)
            self.path += step; self.maxstep = max(self.maxstep, step)
        self.prev = (p.x, p.y, p.z); self.n += 1
        self.f.write(f"{rt:.3f},{p.x:.4f},{p.y:.4f},{p.z:.4f},{yaw:.2f},{disp:.4f},{step:.4f}\n")
        if rt - self.last >= 2.0:
            self.last = rt
            warn = ""
            if step > 0.5: warn += f" !!STEP={step:.1f}m"
            if disp > 12: warn += " !!EXCURSION>12m"
            print(f"t={rt:6.1f}  pos=({p.x:+6.2f},{p.y:+6.2f},{p.z:+5.2f})  yaw={yaw:+6.1f}  "
                  f"disp={disp:5.2f}m  path={self.path:6.1f}m{warn}", flush=True)

def main():
    rclpy.init(); n = M()
    stop = {'v': False}
    signal.signal(signal.SIGTERM, lambda *a: stop.update(v=True))
    signal.signal(signal.SIGINT,  lambda *a: stop.update(v=True))
    start = time.time()
    while rclpy.ok() and not stop['v'] and (time.time()-start) < DUR:
        rclpy.spin_once(n, timeout_sec=0.2)
    n.f.flush(); n.f.close()
    final_disp = 0.0
    if n.x0 is not None and n.prev is not None:
        final_disp = math.sqrt((n.prev[0]-n.x0[0])**2 + (n.prev[1]-n.x0[1])**2 + (n.prev[2]-n.x0[2])**2)
    print("ELLIPSELIO TRAJ SUMMARY  "
          f"samples={n.n}  path={n.path:.2f}m  peak_excursion={n.peak:.2f}m  "
          f"final_disp_from_start={final_disp:.2f}m  max_single_step={n.maxstep:.2f}m", flush=True)

if __name__ == '__main__':
    main()
