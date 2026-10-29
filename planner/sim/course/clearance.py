#!/usr/bin/env python3
"""Gate L2 collision check: body clearance against the course map over the WHOLE closed-loop run
(arming to the bag's end: every leg, the landings, the drive, and fly_mission.py's final QRLAND).

  clearance.py <run dir> <course.pcd> [--lidar-up 0.25] [--floor-ignore 0.10]

Extends ~/sim/l2/clear_all.py (box at the odometry point, flying only) for driving:
- the frame: a 0.508 x 0.457 x 0.203 m box (length, width, height) with its centre ODOM_AHEAD 0.193 m
  behind the odometry point (track_core.ODOM_AHEAD) and its bottom LIDAR_UP below it (SITL's vehicle
  stands on the map's z = 0 and the render node's lidar sits LIDAR_UP above it), in the body's attitude;
- floor contact ignored: map points less than --floor-ignore above SITL's ground (map z = 0) are left
  out, in every phase. The merged map's floor is a smear whose top lies around z = 0 with 0.1-0.3 m
  bumps; SITL has no collision with the map, so its vehicle drives and lands on z = 0 through them.
  Anything nearer than 0.10 m is reported with what it is: an added obstacle, the map above 0.35 m
  (walls, the box), or the map's floor smear (0.10-0.35 m, an artefact on the ground);
- clearance to the added obstacles alone, from <course.pcd>.obstacles.json (make_course.py), exact
  geometry, is reported separately: the course is what the gate is about.

Phases from /mission_executive/status in the bag: each LEG by its type (fly, drive), TAKEOFF,
LAND_CHECK, LAND, and after fly_mission.py's "mission end" the final QRLAND. Also per phase: the max
horizontal speed (/planner/odom positions over 0.5 s) and the vehicle height (lidar height - LIDAR_UP).
"""
import argparse
import json
import math

import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore
from scipy.spatial import cKDTree

ap = argparse.ArgumentParser()
ap.add_argument("run")
ap.add_argument("course")
ap.add_argument("--lidar-up", type=float, default=0.25)
ap.add_argument("--floor-ignore", type=float, default=0.10)
ap.add_argument("--odom-ahead", type=float, default=0.193)
a = ap.parse_args()
R = a.run.rstrip("/") + "/"
HALF = np.array([0.508, 0.457, 0.203]) / 2

ev = {}
for line in open(R + "logs/fly_mission.log"):
    w = line.split()
    for k in ("armed in SITL", "start: True", "mission end", "landed and disarmed"):
        if len(w) > 1 and " ".join(w[1:]).startswith(k):
            ev[k] = float(w[0])

ts = get_typestore(Stores.ROS1_NOETIC)
O, ST, V = [], [], []
with Reader(R + "closed_loop.bag") as r:
    want = {"/planner/odom", "/mission_executive/status", "/mavros/local_position/velocity_local"}
    for c, t, d in r.messages(connections=[c for c in r.connections if c.topic in want]):
        m = ts.deserialize_ros1(d, c.msgtype)
        if c.topic == "/planner/odom":
            p, q = m.pose.pose.position, m.pose.pose.orientation
            O.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        elif c.topic == "/mission_executive/status":
            ST.append((t / 1e9, json.loads(m.data)))
        else:
            V.append((t / 1e9, math.hypot(m.twist.linear.x, m.twist.linear.y)))
O = np.array(O)
V = np.array(V) if V else np.zeros((0, 2))

# phases: (t_start, label)
phases = [(0.0, "pre-arm")]
for t, s in ST:
    ph = s.get("phase")
    lab = ("%s %s" % (ph, s.get("type"))) if ph == "LEG" else ph
    if "leg" in s and ph == "LEG":
        lab = "leg %d %s" % (s["leg"], s.get("type"))
    if lab and lab != phases[-1][1]:
        phases.append((t, lab))
if "mission end" in ev:
    phases.append((ev["mission end"], "final QRLAND"))
pt = np.array([p[0] for p in phases])


def phase_of(t):
    """the phase's label, numbered when it repeats (TAKEOFF, TAKEOFF 2)"""
    i = int(np.searchsorted(pt, t, side="right")) - 1
    n = sum(1 for p in phases[:i] if p[1] == phases[i][1])
    return phases[i][1] + ("" if n == 0 else " %d" % (n + 1))


# the map, in the planner frame, without the floor contact
raw = open(a.course, "rb").read()
k = raw.index(b"DATA binary\n") + 12
M = np.frombuffer(raw[k:], dtype="<f4").reshape(-1, 4)[:, :3].astype(float)
m2p = json.load(open(R + "map_to_planner.json"))
yaw, start = math.radians(m2p["yaw_deg"]), np.array(m2p["start"])
c, s = math.cos(-yaw), math.sin(-yaw)
Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
keep = M[:, 2] >= a.floor_ignore            # map z: SITL's ground is z = 0
kind = np.where(M[:, 2] > 0.35, 1, 2)       # 1 map above 0.35 m, 2 floor smear
meta = json.load(open(a.course + ".obstacles.json"))
obs = meta["obstacles"]
kind[meta["n_base"]:] = 0                   # 0 added obstacle (make_course.py appends them)
Mp = (M - start) @ Rz.T
tree = cKDTree(Mp[keep])
Mk, Kk = Mp[keep], kind[keep]
KIND = ("course obstacle", "map above 0.35 m", "floor smear 0.10-0.35 m")


def qm(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def frame_centre(row):
    Rb = qm(*row[4:8])
    return row[1:4] + Rb @ np.array([-a.odom_ahead, 0.0, -(a.lidar_up - HALF[2])]), Rb


def box_clearance(P, ctr, Rb):
    """distance from the oriented box to the nearest of P (negative: depth inside), index"""
    q = np.abs((P - ctr) @ Rb) - HALF
    out = np.linalg.norm(np.maximum(q, 0), axis=1)
    val = np.where(out > 0, out, q.max(axis=1))
    i = int(np.argmin(val))
    return float(val[i]), i


# the added obstacles' exact geometry in the planner frame, sampled finely for the same box test
def obstacle_samples():
    pts = []
    for o in obs:
        z0 = o["z0"]
        if o["shape"] == "box":
            sx, sy, h = o["size"]
            g = np.mgrid[-sx / 2:sx / 2 + 1e-9:0.01, -sy / 2:sy / 2 + 1e-9:0.01, 0:h + 1e-9:0.02].reshape(3, -1).T
            on = (np.isclose(np.abs(g[:, 0]), sx / 2) | np.isclose(np.abs(g[:, 1]), sy / 2) | np.isclose(g[:, 2], h))
            g = g[on]
            cy, sy_ = math.cos(math.radians(o["yaw_deg"])), math.sin(math.radians(o["yaw_deg"]))
            xy = g[:, :2] @ np.array([[cy, -sy_], [sy_, cy]]).T + o["c"]
            pts.append(np.c_[xy, g[:, 2] + z0])
        else:
            an = np.linspace(0, 2 * math.pi, 96, endpoint=False)
            zz = np.arange(0, o["h"] + 1e-9, 0.02)
            A, Z = np.meshgrid(an, zz)
            pts.append(np.c_[o["c"][0] + o["r"] * np.cos(A.ravel()), o["c"][1] + o["r"] * np.sin(A.ravel()), Z.ravel() + z0])
    P = np.vstack(pts)
    return (P - start) @ Rz.T


OB = obstacle_samples()
OB = OB[OB[:, 2] + start[2] >= a.floor_ignore]
otree = cKDTree(OB)

# speeds over 0.5 s
tt = O[:, 0]
j = np.searchsorted(tt, tt - 0.5)
dt = tt - tt[j]
vh = np.where(dt > 0.2, np.hypot(O[:, 1] - O[j, 1], O[:, 2] - O[j, 2]) / np.maximum(dt, 1e-6), 0.0)
height = O[:, 3] - 0.0          # lidar above its start height = vehicle above SITL's ground

t_arm = ev.get("armed in SITL", O[0, 0])
stats = {}
worst = []
for i in range(0, len(O), 2):        # odometry at 20 Hz -> every 0.1 s
    row = O[i]
    if row[0] < t_arm - 1:
        continue
    ph = phase_of(row[0])
    ctr, Rb = frame_centre(row)
    idx = tree.query_ball_point(ctr, 1.5)
    cl, kd = (9.0, None) if not idx else box_clearance(Mk[idx], ctr, Rb)
    if idx:
        kd = Kk[idx][kd]
    oidx = otree.query_ball_point(ctr, 2.0)
    oc = 9.0 if not oidx else box_clearance(OB[oidx], ctr, Rb)[0]
    st = stats.setdefault(ph, {"t0": row[0], "t1": row[0], "clear": 9.0, "clear_kind": None, "clear_at": None,
                               "obstacle": 9.0, "obstacle_at": None, "vmax": 0.0, "vfc": 0.0, "hmax": -9.0, "hmin": 9.0})
    st["t1"] = row[0]
    if cl < st["clear"]:
        st.update(clear=cl, clear_kind=KIND[kd] if kd is not None else None, clear_at=[round(x, 2) for x in row[1:4]])
    if oc < st["obstacle"]:
        st.update(obstacle=oc, obstacle_at=[round(x, 2) for x in row[1:4]])
    st["vmax"] = max(st["vmax"], vh[i])
    k0, k1 = np.searchsorted(V[:, 0], [O[max(i - 2, 0), 0], row[0]]) if len(V) else (0, 0)
    if k1 > k0:
        st["vfc"] = max(st["vfc"], float(V[k0:k1, 1].max()))
    st["hmax"], st["hmin"] = max(st["hmax"], height[i]), min(st["hmin"], height[i])
    if cl < 0.10:
        worst.append((row[0] - t_arm, ph, row[1], row[2], row[3], cl, KIND[kd]))

print("run %s, course %s: frame box %.3f x %.3f x %.3f m, centre %.3f m behind the odometry point, bottom %.2f m below it; "
      "floor contact ignored below map z %.2f m" % (a.run, a.course, *(2 * HALF), a.odom_ahead, a.lidar_up, a.floor_ignore))
print("v odom: horizontal speed of the odometry point over 0.5 s; v fc: /mavros/local_position/velocity_local; "
      "course: clearance to the added obstacles alone")
print("%-16s %7s %6s %6s %6s %6s  %-33s %6s" % ("phase", "t s", "dur s", "h max", "v odom", "v fc", "min clearance (nearest)", "course"))
for ph, st in sorted(stats.items(), key=lambda kv: kv[1]["t0"]):
    print("%-16s %7.1f %6.1f %6.2f %6.2f %6.2f  %6.2f %-26s %6.2f" % (ph, st["t0"] - t_arm, st["t1"] - st["t0"], st["hmax"], st["vmax"],
                                                                     st["vfc"], st["clear"], st["clear_kind"] or "-", st["obstacle"]))


def agg(sel):
    v = [st for ph, st in stats.items() if sel(ph)]
    if not v:
        return None
    b = min(v, key=lambda s_: s_["clear"])
    o = min(v, key=lambda s_: s_["obstacle"])
    return {"clear": round(b["clear"], 3), "clear_kind": b["clear_kind"], "clear_at": b["clear_at"],
            "course_obstacle": round(o["obstacle"], 3), "course_obstacle_at": o["obstacle_at"],
            "vmax_odom": round(max(s_["vmax"] for s_ in v), 2), "vmax_fc": round(max(s_["vfc"] for s_ in v), 2)}


summary = {"fly legs": agg(lambda p: p.endswith(" fly")), "drive legs": agg(lambda p: p.endswith(" drive")),
           "all (arm to end)": agg(lambda p: p != "pre-arm")}
print(json.dumps(summary, indent=1))
print("%d samples nearer than 0.10 m" % len(worst))
for w in worst[:40]:
    print("  %6.1f s %-14s pos %6.2f %6.2f %5.2f  clearance %5.2f  %s" % w)
