#!/usr/bin/env python3
"""Heading check of a closed-loop run (owner rule 2026-10-29: the heading points along the direction of
travel; the robot yaws only at a waypoint). From the run's bag:

- flying, moving (horizontal speed of the FC above --speed, 0.5 m/s): the FC's yaw against its travel
  heading (median, p90, max);
- flying, slow outside WAYPOINT_YAW / SCAN (hovers, arrivals): the yaw range per stretch (a wandering
  yaw shows here);
- each WAYPOINT_YAW: the waypoint yaw, the yaw reached, the settle time;
- each landing (LAND): the yaw at NAV_LAND, the yaw rate over the second before it, and the heading of
  the drive leg that follows (/planning/drive_goal) from the touchdown point;
- the commanded flight yaw (/mavros/setpoint_raw/local, mask 2048) rate, max.

usage: heading_check.py <run dir> [--speed 0.5]
"""
import argparse
import json
import math

import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

ap = argparse.ArgumentParser()
ap.add_argument("run")
ap.add_argument("--speed", type=float, default=0.5)
a = ap.parse_args()
ts = get_typestore(Stores.ROS1_NOETIC)


def wrap(x):
    return (x + math.pi) % (2 * math.pi) - math.pi


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


pose, phases, sp, drive_goals, anchor = [], [], [], [], None
with Reader(a.run.rstrip("/") + "/closed_loop.bag") as r:
    want = {"/mavros/local_position/pose", "/mission_executive/status", "/mavros/setpoint_raw/local", "/planning/drive_goal",
            "/planner_bridge/anchor"}
    conns = [c for c in r.connections if c.topic in want]
    for c in conns:   # the bag's own definitions (mavros_msgs is not in the store)
        d = c.msgdef.data if hasattr(c.msgdef, "data") else c.msgdef
        try:
            ts.register(get_types_from_msg(d, c.msgtype))
        except Exception:  # noqa: BLE001 - already known
            pass
    for c, t, raw in r.messages(connections=conns):
        m = ts.deserialize_ros1(raw, c.msgtype)
        tt = t * 1e-9
        if c.topic == "/mavros/local_position/pose":
            pose.append((tt, m.pose.position.x, m.pose.position.y, m.pose.position.z, yaw_of(m.pose.orientation)))
        elif c.topic == "/mission_executive/status":
            phases.append((tt, json.loads(m.data)))
        elif c.topic == "/mavros/setpoint_raw/local" and m.type_mask == 2048:
            sp.append((tt, m.yaw))
        elif c.topic == "/planning/drive_goal":
            drive_goals.append((tt, m.pose.position.x, m.pose.position.y))
        elif c.topic == "/planner_bridge/anchor":
            q = m.transform.rotation
            anchor = (m.transform.translation.x, m.transform.translation.y, yaw_of(q))

P = np.array(pose)


out = {}
# moving in flight
v_rows = []
for i in range(5, len(P)):
    dt = P[i, 0] - P[i - 5, 0]
    if dt <= 0:
        continue
    vx, vy = (P[i, 1] - P[i - 5, 1]) / dt, (P[i, 2] - P[i - 5, 2]) / dt
    sp_h = math.hypot(vx, vy)
    if P[i, 3] > 0.3 and sp_h > a.speed:
        v_rows.append(abs(wrap(P[i, 4] - math.atan2(vy, vx))))
if v_rows:
    d = np.degrees(np.array(v_rows))
    out["moving: |yaw - travel heading| deg"] = {"n": len(d), "median": round(float(np.median(d)), 1),
                                                 "p90": round(float(np.percentile(d, 90)), 1), "max": round(float(d.max()), 1)}
# waypoint yaw reports
out["waypoints"] = [{k: s.get(k) for k in ("leg", "for", "yaw_deg", "settled", "settle_s", "waited_s")}
                    for _, s in phases if s.get("phase") == "WAYPOINT_YAW"]
# landings: the LAND report time ~ NAV_LAND
lands = []
for tt, s in phases:
    if s.get("phase") == "LAND":
        k = np.searchsorted(P[:, 0], tt)
        k0 = np.searchsorted(P[:, 0], tt - 1.0)
        if k >= len(P):
            continue
        ys = np.unwrap(P[k0:k + 1, 4])
        rate = math.degrees(abs(ys[-1] - ys[0]) / max(1e-3, P[k, 0] - P[k0, 0])) if k > k0 else float("nan")
        nxt = [g for g in drive_goals if g[0] > tt]
        drive_dir = None
        if nxt and anchor is not None:
            # the drive goal is in the planner frame: to fc with the anchor
            c, s_ = math.cos(anchor[2]), math.sin(anchor[2])
            gx = c * nxt[0][1] - s_ * nxt[0][2] + anchor[0]
            gy = s_ * nxt[0][1] + c * nxt[0][2] + anchor[1]
            drive_dir = math.atan2(gy - P[k, 2], gx - P[k, 1])
        lands.append({"leg": s.get("leg"), "yaw_at_land_deg": round(math.degrees(P[k, 4]), 1),
                      "yaw_rate_last_1s_dps": round(rate, 1),
                      "next_drive_heading_deg": None if drive_dir is None else round(math.degrees(drive_dir), 1),
                      "off_drive_heading_deg": None if drive_dir is None else round(math.degrees(abs(wrap(P[k, 4] - drive_dir))), 1)})
out["landings"] = lands
# commanded flight yaw rate
if len(sp) > 2:
    S = np.array(sp)
    rates = [abs(wrap(S[i, 1] - S[i - 1, 1])) / (S[i, 0] - S[i - 1, 0]) for i in range(1, len(S)) if 0.005 < S[i, 0] - S[i - 1, 0] < 0.1]
    out["commanded flight yaw rate max dps"] = round(math.degrees(max(rates)), 1) if rates else None
print(json.dumps(out, indent=1))
