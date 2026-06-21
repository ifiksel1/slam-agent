#!/usr/bin/env python3
# Snapshot SuperOdom's accumulated map (/laser_cloud_map) to a BINARY PCD.
# Callback is O(1) (keeps a reference to the largest map msg, no parsing). At exit it parses
# once with numpy and writes BINARY PCD (raw float32 dump — instant, no per-point work).
# Progress goes to stderr so a wrapper can see it's alive.
# Usage: save_map.py <out.pcd> [topic=/laser_cloud_map] [DUR=300]
import rclpy, sys, signal, time, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

OUT = sys.argv[1] if len(sys.argv) > 1 else '/root/ros2_ws/results/map.pcd'
TOPIC = sys.argv[2] if len(sys.argv) > 2 else '/laser_cloud_map'
DUR = float(sys.argv[3]) if len(sys.argv) > 3 else 300.0
def log(*a): print(*a, file=sys.stderr, flush=True)

class S(Node):
    def __init__(self):
        super().__init__('save_map')
        self.create_subscription(PointCloud2, TOPIC, self.cb, 1)
        self.msg = None; self.npts = 0; self.count = 0
    def cb(self, m):
        self.count += 1
        n = m.width * m.height
        if n >= self.npts:
            self.msg = m; self.npts = n
            if self.count <= 2 or self.count % 20 == 0:
                log(f"  [save_map] map msg {self.count}: {n} pts")

def write_binary_pcd(m, path):
    off = {f.name: f.offset for f in m.fields}
    n = m.width * m.height
    raw = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(n, m.point_step)
    def col(name):
        o = off[name]
        return raw[:, o:o+4].copy().view(np.float32).ravel()
    cols = [col('x'), col('y'), col('z')]
    names = "x y z"; sizes = "4 4 4"; types = "F F F"; counts = "1 1 1"; nf = 3
    if 'intensity' in off:
        cols.append(col('intensity')); names = "x y z intensity"; sizes = "4 4 4 4"
        types = "F F F F"; counts = "1 1 1 1"; nf = 4
    arr = np.column_stack(cols).astype(np.float32)
    good = np.isfinite(arr).all(axis=1)
    arr = np.ascontiguousarray(arr[good])
    k = arr.shape[0]
    header = (f"# .PCD v0.7 - SuperOdom map\nVERSION 0.7\nFIELDS {names}\nSIZE {sizes}\n"
              f"TYPE {types}\nCOUNT {counts}\nWIDTH {k}\nHEIGHT 1\n"
              f"VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {k}\nDATA binary\n")
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(arr.tobytes())
    return k

def main():
    rclpy.init(); n = S()
    stop = {'v': False}
    for s in (signal.SIGTERM, signal.SIGINT):
        signal.signal(s, lambda *a: stop.update(v=True))
    log(f"  [save_map] listening on {TOPIC} (DUR={DUR}s)")
    t0 = time.time()
    while rclpy.ok() and not stop['v'] and (time.time()-t0) < DUR:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.msg is not None:
        t = time.time(); k = write_binary_pcd(n.msg, OUT)
        print(f"SAVED MAP: {k} points -> {OUT} (from {n.count} msgs, write {time.time()-t:.1f}s)", flush=True)
    else:
        print(f"NO MAP RECEIVED on {TOPIC} ({n.count} msgs)", flush=True)
    n.destroy_node(); rclpy.shutdown()

main()
