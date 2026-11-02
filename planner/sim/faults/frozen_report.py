#!/usr/bin/env python3
"""The frozen-pose vision cut (fastlio_mavros_bridge.py, owner 2026-11-02) in one failure-test run: when the
SLAM-to-FC bridge and the planner bridge detected the frozen pose, when vision stopped (and resumed), the FC's
modes, the horizontal drift, and the touchdown (vertical and horizontal speed from /sim/truth). Also any vision
gap or frozen-pose report in a run without the fault (baseline, hover_park: false triggers).
Needs rosbags (Shannon: ~/sim/venv/bin/python).

usage: frozen_report.py <run dir> [<run dir> ...]
"""
import json
import math
import os
import re
import sys

from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

LOGT = re.compile(r"\[(?:ERROR|WARN|INFO)\] \[(\d+\.\d+)\]: (.*)")


def log_lines(path, pat):
    out = []
    if os.path.exists(path):
        for ln in open(path, errors="replace"):
            m = LOGT.search(ln)
            if m and re.search(pat, m.group(2)):
                out.append((float(m.group(1)), m.group(2).strip()))
    return out


def report(run):
    run = run.rstrip("/")
    p = os.path.join(run, "fault_result.json")
    if os.path.exists(p):
        res = json.load(open(p))
        t0 = res["fault"]["t"] if res.get("fault") else None
    else:   # inject.py did not finish (the run was torn down first): the fault time from fault.log
        res, t0 = {"case": "?"}, None
        for ln in open(os.path.join(run, "logs", "fault.log"), errors="replace"):
            if '"kind": "fault"' in ln or '"fired_by"' in ln:
                t0 = float(ln.split()[0])
                break
    ts = get_typestore(Stores.ROS1_NOETIC)
    truth, vis, fzs = [], [], []
    with Reader(os.path.join(run, "closed_loop.bag")) as r:
        want = {"/sim/truth", "/mavros/vision_pose/pose", "/fastlio_mavros_bridge/frozen_status"}
        conns = [c for c in r.connections if c.topic in want]
        for c in conns:
            try:
                ts.register(get_types_from_msg(c.msgdef.data, c.msgtype))
            except Exception:  # noqa: BLE001 - already registered
                pass
        for c, t, raw in r.messages(connections=conns):
            m = ts.deserialize_ros1(raw, c.msgtype)
            tt = t * 1e-9
            if c.topic == "/sim/truth":
                p, v = m.pose.pose.position, m.twist.twist.linear
                truth.append((tt, p.x, p.y, p.z, v.x, v.y, v.z))
            elif c.topic == "/mavros/vision_pose/pose":
                vis.append(tt)
            else:
                fzs.append((tt, json.loads(m.data)))
    out = {"run": os.path.basename(run), "case": res["case"]}
    blog = os.path.join(run, "logs", "bridge.log")
    cut = log_lines(blog, r"frozen in the air")
    ground = log_lines(blog, r"frozen \(.*not airborne")
    resume = log_lines(blog, r"vision resumed")
    pb = log_lines(os.path.join(run, "logs", "planner_bridge.log"), r"SLAM pose frozen")
    # vision gaps > 0.5 s (after the first pose)
    gaps = [(a, b) for a, b in zip(vis, vis[1:]) if b - a > 0.5]
    ref = t0 if t0 is not None else (vis[0] if vis else 0.0)
    rel = (lambda x: None if x is None else round(x - ref, 2))
    out["slam_fc_bridge_cut"] = [(rel(t), s[:160]) for t, s in cut]
    out["slam_fc_bridge_ground_reports"] = len(ground)
    out["slam_fc_bridge_resume"] = [(rel(t), s[:120]) for t, s in resume]
    out["planner_bridge_frozen"] = [rel(t) for t, _ in pb][:3]
    out["vision_gaps"] = [(rel(a), round(b - a, 2)) for a, b in gaps]
    if fzs:
        out["frozen_status_max_repeats"] = max(s.get("repeats", 0) for _, s in fzs)
        out["frozen_status_cut_samples"] = sum(1 for _, s in fzs if s.get("vision") == "cut")
        air = [s.get("airborne") for _, s in fzs]
        out["frozen_status_airborne_frac"] = round(sum(1 for a in air if a) / float(len(air)), 2)
        out["airborne_reasons"] = sorted({s.get("airborne_why", "").split(" ")[0] for _, s in fzs})
    if t0 is not None and truth:
        W = [x for x in truth if x[0] >= t0]
        p0 = W[0]
        out["fault_speed"] = round(math.hypot(p0[4], p0[5]), 2)
        out["fault_z"] = round(p0[3], 2)
        out["max_horiz_drift_m"] = round(max(math.hypot(x[1] - p0[1], x[2] - p0[2]) for x in W), 2)
        air = p0[3] > 0.3
        if air:
            td = next((x for x in W if x[3] < 0.06), None)
            if td:
                pre = [x for x in W if td[0] - 0.15 <= x[0] <= td[0]]
                out["touchdown_t"] = rel(td[0])
                out["touchdown_vz"] = round(max(-x[6] for x in pre), 2)
                out["touchdown_vh"] = round(max(math.hypot(x[4], x[5]) for x in pre), 2)
                out["drift_at_touchdown_m"] = round(math.hypot(td[1] - p0[1], td[2] - p0[2]), 2)
                desc = [x for x in W if x[0] <= td[0]]
                out["max_vh_before_touchdown"] = round(max(math.hypot(x[4], x[5]) for x in desc), 2)
        out["end_pos"] = res.get("end", {}).get("pos")
        out["end_mode"] = (res.get("end", {}).get("mode"), res.get("end", {}).get("armed"))
    out["fc_modes"] = [(round(m[0], 2), m[1], m[2]) for m in res.get("modes", [])]
    out["texts"] = [(round(t, 2), s) for t, s in res.get("texts", []) if "Hit ground" in s or "EKF" in s or
                    "SLAM" in s or "ailsafe" in s][:12]
    out["stop"] = res.get("stop")
    return out


for run in sys.argv[1:]:
    print(json.dumps(report(run), indent=1))
