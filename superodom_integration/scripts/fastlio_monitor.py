#!/usr/bin/env python3
# FAST-LIO eval monitor: subscribe /Odometry (trajectory + divergence metrics, mirrors the
# SuperOdom live_monitor) and /Laser_map (accumulated map -> binary PCD). Writes CSV + PCD +
# a SUMMARY line at exit. Usage: fastlio_monitor.py <traj.csv> <map.pcd> [DUR=1200]
import rclpy, sys, signal, time, math, numpy as np, csv
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

CSV = sys.argv[1]; PCD = sys.argv[2]
DUR = float(sys.argv[3]) if len(sys.argv) > 3 else 1200.0
def log(*a): print(*a, file=sys.stderr, flush=True)

def yaw_from_q(x, y, z, w):
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

class M(Node):
    def __init__(self):
        super().__init__('fastlio_monitor')
        self.create_subscription(Odometry, '/Odometry', self.odo, 50)
        self.create_subscription(PointCloud2, '/Laser_map', self.mapcb, 1)
        self.rows=[]; self.map=None; self.npts=0; self.last=None; self.start=None; self.t0=None
        self.peak=0.0; self.path=0.0; self.maxstep=0.0; self.lp=0.0
    def odo(self, m):
        p=m.pose.pose.position; q=m.pose.pose.orientation
        t=m.header.stamp.sec + m.header.stamp.nanosec*1e-9
        if self.t0 is None: self.t0=t; self.start=(p.x,p.y,p.z)
        yaw=yaw_from_q(q.x,q.y,q.z,q.w); step=0.0
        if self.last is not None:
            step=math.dist((p.x,p.y,p.z), self.last); self.path+=step; self.maxstep=max(self.maxstep,step)
        self.last=(p.x,p.y,p.z)
        disp=math.dist((p.x,p.y,p.z), self.start); self.peak=max(self.peak,disp)
        self.rows.append((t-self.t0,p.x,p.y,p.z,yaw,1,disp,step))
        now=time.time()
        if now-self.lp>1.5:
            self.lp=now
            log(f"  t={t-self.t0:5.1f}s pos=({p.x:6.2f},{p.y:6.2f},{p.z:6.2f}) yaw={math.degrees(yaw):7.1f} disp={disp:5.2f} step={step:.2f}"+("  <<JUMP" if step>0.5 else ""))
    def mapcb(self, m):
        n=m.width*m.height
        if n>=self.npts: self.map=m; self.npts=n

def write_pcd(m, path):
    off={f.name:f.offset for f in m.fields}; n=m.width*m.height
    raw=np.frombuffer(bytes(m.data),dtype=np.uint8).reshape(n,m.point_step)
    def col(nm): o=off[nm]; return raw[:,o:o+4].copy().view(np.float32).ravel()
    cols=[col('x'),col('y'),col('z')]; names="x y z"; nf=3
    if 'intensity' in off: cols.append(col('intensity')); names="x y z intensity"; nf=4
    arr=np.column_stack(cols).astype(np.float32); arr=arr[np.isfinite(arr).all(axis=1)]
    arr=np.ascontiguousarray(arr); k=arr.shape[0]
    hdr=(f"# .PCD v0.7 - FAST-LIO map\nVERSION 0.7\nFIELDS {names}\nSIZE {' '.join(['4']*nf)}\n"
         f"TYPE {' '.join(['F']*nf)}\nCOUNT {' '.join(['1']*nf)}\nWIDTH {k}\nHEIGHT 1\n"
         f"VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {k}\nDATA binary\n")
    with open(path,'wb') as f: f.write(hdr.encode()); f.write(arr.tobytes())
    return k

def main():
    rclpy.init(); n=M(); stop={'v':False}
    for s in (signal.SIGTERM, signal.SIGINT): signal.signal(s, lambda *a: stop.update(v=True))
    log(f"[fastlio_monitor] /Odometry + /Laser_map (DUR={DUR}s)")
    t0=time.time()
    while rclpy.ok() and not stop['v'] and (time.time()-t0)<DUR:
        rclpy.spin_once(n, timeout_sec=0.1)
    with open(CSV,'w',newline='') as f:
        w=csv.writer(f); w.writerow(['t','x','y','z','yaw','health','disp_from_start','step']); w.writerows(n.rows)
    if n.map is not None:
        k=write_pcd(n.map, PCD); print(f"SAVED MAP: {k} points -> {PCD}", flush=True)
    else:
        print("NO MAP RECEIVED on /Laser_map", flush=True)
    fd = n.rows[-1][6] if n.rows else 0.0
    print(f"FASTLIO SUMMARY samples={len(n.rows)} path={n.path:.2f}m peak_excursion_from_start={n.peak:.2f}m "
          f"final_disp_from_start={fd:.2f}m max_single_step={n.maxstep:.2f}m", flush=True)
    n.destroy_node(); rclpy.shutdown()

main()
