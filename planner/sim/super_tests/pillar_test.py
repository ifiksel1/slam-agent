#!/usr/bin/env python3
"""SUPER-alone 'obstacle appears' test (SITL only, no FC): a perfect-tracking vehicle (odometry = SUPER's
/planning/pos_cmd), the Mass Robotics 4F map around it as /planner/cloud, one goal; once the vehicle flies
faster than trig_v, a 0.6 x 0.6 x 3 m pillar appears D m ahead along its velocity (as faults obst_fly).
Prints the vehicle's least distance to the pillar (negative: inside it) and whether it stopped.
  pillar_test.py map.pcd sx sy sz gx gy gz D [trig_v] [duration_s]"""
import math, sys, time, json
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand

pcd = sys.argv[1]
sx, sy, sz, gx, gy, gz, D = map(float, sys.argv[2:9])
trig_v = float(sys.argv[9]) if len(sys.argv) > 9 else 0.6
dur = float(sys.argv[10]) if len(sys.argv) > 10 else 40.0
START = (0.01, -0.01, -91.1); LIDAR_UP = 0.25; SIZE = np.array([0.6, 0.6, 3.0])

def load(path):
    with open(path, "rb") as f:
        while True:
            l = f.readline()
            if l.startswith(b"POINTS"): n = int(l.split()[1])
            if l.startswith(b"DATA"): break
        a = np.frombuffer(f.read(n * 16), dtype="<f4").reshape(n, 4)[:, :3].astype(np.float64)
    y = math.radians(START[2]); c, s = math.cos(y), math.sin(y)
    d = a - np.array([START[0], START[1], LIDAR_UP])
    return np.c_[c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1], d[:, 2]]

pts = load(pcd)
mid = np.array([(sx + gx) / 2, (sy + gy) / 2, sz])
pts = pts[np.linalg.norm(pts - mid, axis=1) < 25.0]

def box_points(c):
    lo, hi = c - SIZE / 2, c + SIZE / 2
    g = [np.arange(lo[i], hi[i] + 1e-9, 0.05) for i in range(3)]
    out = []
    for X in (lo[0], hi[0]):
        Y, Z = np.meshgrid(g[1], g[2]); out.append(np.c_[np.full(Y.size, X), Y.ravel(), Z.ravel()])
    for Yv in (lo[1], hi[1]):
        X, Z = np.meshgrid(g[0], g[2]); out.append(np.c_[X.ravel(), np.full(X.size, Yv), Z.ravel()])
    return np.vstack(out)

def sdf(p, c):
    q = np.abs(p - c) - SIZE / 2
    return float(np.linalg.norm(np.maximum(q, 0)) + min(max(q[0], q[1], q[2]), 0))

rospy.init_node("pillar_test")
op = rospy.Publisher("/planner/odom", Odometry, queue_size=10)
cp = rospy.Publisher("/planner/cloud", PointCloud2, queue_size=2)
gp = rospy.Publisher("/planning/click_goal", PoseStamped, queue_size=1, latch=True)
state = {"p": np.array([sx, sy, sz]), "v": np.zeros(3), "t": 0.0}
def on_cmd(m):
    state["p"] = np.array([m.position.x, m.position.y, m.position.z])
    state["v"] = np.array([m.velocity.x, m.velocity.y, m.velocity.z]); state["t"] = time.time()
rospy.Subscriber("/planning/pos_cmd", PositionCommand, on_cmd, queue_size=10)

def cloud_msg(arr, stamp):
    c = np.c_[arr, np.full(len(arr), 50.0)].astype("<f4")
    m = PointCloud2(height=1, width=len(c), is_bigendian=False, point_step=16, row_step=16 * len(c), is_dense=True,
                    fields=[PointField(name=n, offset=4 * i, datatype=7, count=1) for i, n in enumerate(("x", "y", "z", "intensity"))],
                    data=c.tobytes())
    m.header.frame_id = "world"; m.header.stamp = stamp
    return m

t0 = time.time(); k = 0; sent = False; box = None; boxp = None; min_d = 1e9; t_box = None
log = []; r = rospy.Rate(100)
while not rospy.is_shutdown() and time.time() - t0 < dur:
    now = rospy.Time.now(); el = time.time() - t0
    p, v = state["p"].copy(), state["v"].copy()
    if time.time() - state["t"] > 0.2: v = np.zeros(3)
    o = Odometry(); o.header.stamp = now; o.header.frame_id = "world"; o.child_frame_id = "body"
    o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = p
    o.pose.pose.orientation.w = 1.0
    o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z = v
    op.publish(o)
    if box is None and np.linalg.norm(v[:2]) > trig_v:
        u = v[:2] / np.linalg.norm(v[:2])
        box = np.array([p[0] + u[0] * D, p[1] + u[1] * D, sz]); boxp = box_points(box); t_box = el
        print(json.dumps({"t": round(el, 2), "event": "pillar", "center": box.round(3).tolist(), "speed": round(float(np.linalg.norm(v)), 2)}), flush=True)
    if k % 10 == 0:
        near = pts[np.linalg.norm(pts - p, axis=1) < 12.0]
        if boxp is not None: near = np.vstack([near, boxp])
        cp.publish(cloud_msg(near, now))
    if box is not None:
        d = sdf(p, box); min_d = min(min_d, d)
        if k % 20 == 0: log.append((round(el - t_box, 2), round(d, 3), round(float(np.linalg.norm(v)), 2)))
    if not sent and el > 8.0:
        g = PoseStamped(); g.header.stamp = now; g.header.frame_id = "world"
        g.pose.position.x, g.pose.position.y, g.pose.position.z = gx, gy, gz; g.pose.orientation.w = 1.0
        gp.publish(g); sent = True
        print(json.dumps({"t": round(el, 2), "event": "goal", "goal": [gx, gy, gz]}), flush=True)
    k += 1; r.sleep()
print(json.dumps({"event": "trace", "t_since_pillar_dist_speed": log}), flush=True)
print(json.dumps({"event": "result", "pillar": box.tolist() if box is not None else None,
                  "min_dist_to_pillar_m": round(min_d, 3), "entered_pillar": min_d < 0,
                  "final_pos": state["p"].round(3).tolist(), "final_speed": round(float(np.linalg.norm(state["v"])), 3)}), flush=True)
