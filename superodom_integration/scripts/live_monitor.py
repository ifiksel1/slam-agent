#!/usr/bin/env python3
# Live SuperOdom motion-test monitor. Prints pose/yaw/health every ~1.5s so the test
# can be watched in real time, logs a CSV, and tracks: max excursion (divergence watch),
# total path, and the running displacement-from-start (loop-closure when you return).
# Runs until Ctrl-C / SIGTERM or DUR seconds; prints a summary on exit.
import rclpy, math, time, signal, sys
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from super_odometry_msgs.msg import OptimizationStats

DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 600.0
CSV = sys.argv[1] if len(sys.argv) > 1 else '/root/ros2_ws/results/live_motion.csv'

class M(Node):
    def __init__(self):
        super().__init__('live_monitor')
        self.create_subscription(Odometry, '/state_estimation', self.odom, 50)
        self.create_subscription(Bool, '/state_estimation_health', self.health, 10)
        self.create_subscription(OptimizationStats, '/super_odometry_stats', self.stats, 10)
        self.x0 = None; self.h = True; self.last = -9
        self.peak = 0.0; self.path = 0.0; self.prev = None
        self.maxstep = 0.0; self.n = 0
        self.f = open(CSV, 'w'); self.f.write("t,x,y,z,yaw,health,disp_from_start,step\n")
        self.t0 = None
    def health(self, m): self.h = m.data
    def stats(self, m): pass
    def odom(self, m):
        t = self.get_clock().now().nanoseconds / 1e9
        if self.t0 is None: self.t0 = t
        rt = t - self.t0
        p = m.pose.pose.position; q = m.pose.pose.orientation
        yaw = math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        if self.x0 is None: self.x0 = (p.x, p.y, p.z)
        disp = math.sqrt((p.x-self.x0[0])**2 + (p.y-self.x0[1])**2)
        self.peak = max(self.peak, disp)
        step = 0.0
        if self.prev is not None:
            step = math.sqrt((p.x-self.prev[0])**2 + (p.y-self.prev[1])**2 + (p.z-self.prev[2])**2)
            self.path += step; self.maxstep = max(self.maxstep, step)
        self.prev = (p.x, p.y, p.z); self.n += 1
        self.f.write(f"{rt:.3f},{p.x:.4f},{p.y:.4f},{p.z:.4f},{yaw:.2f},{int(self.h)},{disp:.4f},{step:.4f}\n")
        if rt - self.last >= 1.5:
            self.last = rt
            warn = ""
            if step > 0.5: warn += f" !!STEP={step:.1f}m"      # >0.5m in one scan @25Hz = >12m/s = divergence
            if disp > 9: warn += " !!EXCURSION>9m"
            if not self.h: warn += " [HEALTH=FALSE]"
            print(f"t={rt:6.1f}  pos=({p.x:+6.2f},{p.y:+6.2f},{p.z:+5.2f})  yaw={yaw:+6.1f}  "
                  f"disp_from_start={disp:5.2f}m  path={self.path:6.1f}m{warn}", flush=True)

def main():
    rclpy.init(); n = M()
    stop = {'v': False}
    signal.signal(signal.SIGTERM, lambda *a: stop.update(v=True))
    signal.signal(signal.SIGINT, lambda *a: stop.update(v=True))
    start = time.time()
    while rclpy.ok() and not stop['v'] and (time.time()-start) < DUR:
        rclpy.spin_once(n, timeout_sec=0.2)
    n.f.flush(); n.f.close()
    print("\n=== LIVE MOTION SUMMARY ===", flush=True)
    print(f"samples={n.n}  path={n.path:.2f}m  peak_excursion_from_start={n.peak:.2f}m  "
          f"final_disp_from_start={ (math.sqrt((n.prev[0]-n.x0[0])**2+(n.prev[1]-n.x0[1])**2) if n.prev and n.x0 else 0):.2f}m  "
          f"max_single_step={n.maxstep:.2f}m", flush=True)
    open(CSV + '.done', 'w').write('1')
    n.destroy_node(); rclpy.shutdown()

main()
