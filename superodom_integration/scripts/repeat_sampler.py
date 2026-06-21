#!/usr/bin/env python3
# Per-run SuperOdom sampler for repeatability (#1) + degeneracy-stats (#3).
# Subscribes /state_estimation (Odometry), /state_estimation_health (Bool),
# /super_odometry_stats (OptimizationStats). Writes a per-scan CSV and prints a
# single RESULT line (parsed by the harness). Self-terminates on idle (no new
# odometry for IDLE_S after >=1 msg) so it ends right after the bag stops, with a
# hard cap for safety.
#
# Usage: repeat_sampler.py <csv_out_path>
import rclpy, math, time, sys
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from super_odometry_msgs.msg import OptimizationStats

IDLE_S = 8.0     # finalize this many seconds after the last odometry msg
HARD_S = 240.0   # absolute safety cap
CSV = sys.argv[1] if len(sys.argv) > 1 else '/root/ros2_ws/results/repeat/run.csv'

class S(Node):
    def __init__(self):
        super().__init__('repeat_sampler')
        self.create_subscription(Odometry, '/state_estimation', self.odom, 50)
        self.create_subscription(Bool, '/state_estimation_health', self.health, 10)
        self.create_subscription(OptimizationStats, '/super_odometry_stats', self.stats, 10)
        self.t0 = None
        self.last_msg_wall = None
        self.h = True
        self.unhealthy_t = 0.0
        self.prev_t = None
        self.rows = []
        # latest stats snapshot, carried onto each odom row
        self.c = dict(ttl=0, tfl=0, nit=0, lat=0, avgd=0,
                      ux=0, uy=0, uz=0, ur=0, up=0, uyw=0, pok=0, prej=0)
        self.f = open(CSV, 'w')
        self.f.write("t,x,y,z,yaw,health,total_translation,trans_from_last,n_iter,latency_ms,"
                     "icp_avg_dist,ux,uy,uz,uroll,upitch,uyaw,plane_ok,plane_rej\n")
    def wall(self): return time.time()
    def health(self, m): self.h = m.data
    def stats(self, m):
        self.c.update(ttl=m.total_translation, tfl=m.translation_from_last,
                      nit=m.n_iterations, lat=m.latency, avgd=m.average_distance,
                      ux=m.uncertainty_x, uy=m.uncertainty_y, uz=m.uncertainty_z,
                      ur=m.uncertainty_roll, up=m.uncertainty_pitch, uyw=m.uncertainty_yaw,
                      pok=m.plane_match_success,
                      prej=m.plane_no_enough_neighbor + m.plane_neighbor_too_far +
                           m.plane_badpca_structure + m.plane_mse_too_large)
    def odom(self, m):
        t = self.get_clock().now().nanoseconds / 1e9
        if self.t0 is None: self.t0 = t; self.prev_t = t
        self.last_msg_wall = self.wall()
        rt = t - self.t0
        if not self.h: self.unhealthy_t += (t - self.prev_t)
        self.prev_t = t
        p = m.pose.pose.position; q = m.pose.pose.orientation
        yaw = math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        c = self.c
        self.rows.append((rt, p.x, p.y, p.z, yaw))
        self.f.write(f"{rt:.3f},{p.x:.4f},{p.y:.4f},{p.z:.4f},{yaw:.2f},{int(self.h)},"
                     f"{c['ttl']:.4f},{c['tfl']:.4f},{c['nit']},{c['lat']:.2f},{c['avgd']:.4f},"
                     f"{c['ux']:.4f},{c['uy']:.4f},{c['uz']:.4f},{c['ur']:.4f},{c['up']:.4f},{c['uyw']:.4f},"
                     f"{c['pok']},{c['prej']}\n")

rclpy.init()
n = S()
start = time.time()
while rclpy.ok():
    rclpy.spin_once(n, timeout_sec=0.2)
    if time.time() - start > HARD_S: break
    if n.last_msg_wall is not None and (time.time() - n.last_msg_wall) > IDLE_S: break
n.f.flush(); n.f.close()
r = n.rows
if r:
    L = 0.0
    for a, b in zip(r, r[1:]):
        L += math.sqrt((b[1]-a[1])**2 + (b[2]-a[2])**2 + (b[3]-a[3])**2)
    s, e = r[0], r[-1]
    net = math.sqrt((e[1]-s[1])**2 + (e[2]-s[2])**2 + (e[3]-s[3])**2)
    # peak horizontal excursion from start (the "out" distance)
    peak = max(math.sqrt((x[1]-s[1])**2 + (x[2]-s[2])**2) for x in r)
    print(f"RESULT samples={len(r)} dur={e[0]:.1f} path={L:.2f} net={net:.2f} peak={peak:.2f} "
          f"end=({e[1]:+.2f},{e[2]:+.2f},{e[3]:+.2f}) end_yaw={e[4]:+.1f} "
          f"health_false_s={n.unhealthy_t:.1f}", flush=True)
else:
    print("RESULT NO_DATA", flush=True)
# done marker for the harness to poll
open(CSV + '.done', 'w').write('1')
n.destroy_node(); rclpy.shutdown()
