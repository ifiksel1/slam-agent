#!/usr/bin/env python3
"""Fault runner for the L2 failure tests: waits for a trigger in a closed-loop mission, injects a
fault, watches what the stack and the FC do, and writes /run/fault_result.json.

Runs inside an L2 run's ROS container, started by closed_loop.sh's FAULT_HOOK before fly_mission.py
(planner/sim/faults/cases.sh builds the command). Distances come from /sim/truth (SITL's true pose in
the map frame, published by faults/render_fault.py), so they hold whatever the fault does to SLAM,
the planners or MAVROS.

  inject.py --case NAME --trigger SPEC [--action ACTION ...] [--end_hold S] [--max_s S]

Trigger SPEC (comma separated, all must hold):
  leg_type=fly|drive   the executive is on a leg of this type (/mission_executive/status phase LEG)
  leg=N                ... with this index
  phase=NAME           the executive reports this phase (e.g. LAND_CHECK)
  min_speed=V          and the true horizontal speed is at least V m/s ...
  max_wait=S           ... or S s have passed since the leg/phase matched (fires anyway, noted)
  near=x:y:r           and the vehicle is within r m of map (x, y)
  delay=S              then wait S s more
  start                at the mission start (the first phase after WAIT_ARMED)

ACTION (in order; each one is logged on /fault/events):
  pkill:ARGS           run `pkill ARGS` (e.g. "pkill:-9 -x fsm_node"). Anchor -f patterns ("^python3 -u
                       /planner/...") or they match this runner's own command line
  render:JSON          publish JSON on /sim_fault/cmd (render_fault.py: stale, reset, box, box_clear)
  box_ahead:D:SX,SY,SZ:CZ   a box D m ahead of the vehicle along its horizontal velocity, size SX x SY x
                       SZ, centre height CZ (map frame), as render:{"box": ...}
  box_at:X,Y,Z:SX,SY,SZ     a box centred at map (X, Y, Z)
  sleep:S
  none                 observe only (e.g. a goal inside an obstacle: the trigger marks the leg start)

The observation ends END_HOLD_S - 1 s after the executive reports DONE or STOPPED (fly_mission.py
then lands SITL), or when the executive process is gone, or --max_s after the fault.
"""
import argparse
import glob
import hashlib
import json
import math
import subprocess
import threading
import time

import numpy as np
import rospy
from mavros_msgs.msg import State, StatusText
from nav_msgs.msg import Odometry
from std_msgs.msg import String

QR_MODES = {0: "MANUAL", 4: "HOLD", 10: "AUTO", 11: "RTL", 12: "SMART_RTL", 15: "GUIDED", 17: "QRSTABILIZE",
            18: "QRALTHOLD", 19: "QRLOITER", 20: "QRRTL", 21: "QRLAND", 22: "QRCIRCLE", 23: "QRSMARTRTL"}
HALF = np.array([0.508, 0.457, 0.203]) / 2     # the robot's frame half-size (cl_analyze.py)

ap = argparse.ArgumentParser()
ap.add_argument("--case", required=True)
ap.add_argument("--trigger", required=True)
ap.add_argument("--action", action="append", default=[])
ap.add_argument("--end_hold", type=float, default=15.0)
ap.add_argument("--max_s", type=float, default=150.0)
ap.add_argument("--out", default="/run/fault_result.json")
args = ap.parse_args()

rospy.init_node("fault_inject", disable_signals=True)
lock = threading.Lock()
truth = []          # (t, x, y, z, vx, vy, vz)
modes = []          # (t, mode, armed, connected)
execs = []          # (t, dict)
bridge = []         # (t, state, reason, enabled, mode)
texts = []          # (t, text)
track = []          # (t, state, reason)
fstate = []         # (t, dict)


def mode_name(s):
    if s.startswith("CMODE("):
        return QR_MODES.get(int(s[6:-1]), s)
    return s


def on_truth(m):
    p, v = m.pose.pose.position, m.twist.twist.linear
    with lock:
        truth.append((rospy.get_time(), p.x, p.y, p.z, v.x, v.y, v.z))


def on_state(m):
    row = (rospy.get_time(), mode_name(m.mode), bool(m.armed), bool(m.connected))
    with lock:
        if not modes or modes[-1][1:] != row[1:]:
            modes.append(row)


def on_exec(m):
    with lock:
        execs.append((rospy.get_time(), json.loads(m.data)))


def norm(reason):
    return "".join(c for c in (reason or "") if not (c.isdigit() or c == "."))


def on_bridge(m):
    s = json.loads(m.data)
    row = (rospy.get_time(), s.get("state"), s.get("reason", ""), s.get("enabled"), s.get("mode"))
    with lock:
        if not bridge or (bridge[-1][1], norm(bridge[-1][2]), bridge[-1][3], bridge[-1][4]) != (row[1], norm(row[2]), row[3], row[4]):
            bridge.append(row)
        bridge_last[0] = row


def on_text(m):
    with lock:
        texts.append((rospy.get_time(), m.text))


def on_track(m):
    s = json.loads(m.data)
    row = (rospy.get_time(), s.get("state"), s.get("reason", ""))
    with lock:
        if not track or track[-1][1:] != row[1:]:
            track.append(row)


def on_fstate(m):
    with lock:
        fstate.append((rospy.get_time(), json.loads(m.data)))


bridge_last = [None]
rospy.Subscriber("/sim/truth", Odometry, on_truth, queue_size=100)
rospy.Subscriber("/mavros/state", State, on_state, queue_size=20)
rospy.Subscriber("/mission_executive/status", String, on_exec, queue_size=50)
rospy.Subscriber("/planner_bridge/status", String, on_bridge, queue_size=50)
rospy.Subscriber("/mavros/statustext/recv", StatusText, on_text, queue_size=50)
rospy.Subscriber("/track_planner/status", String, on_track, queue_size=50)
rospy.Subscriber("/sim_fault/state", String, on_fstate, queue_size=20)
cmd_pub = rospy.Publisher("/sim_fault/cmd", String, queue_size=10)
ev_pub = rospy.Publisher("/fault/events", String, queue_size=20, latch=True)
events = []


def event(kind, **kw):
    kw.update(t=rospy.get_time(), kind=kind)
    events.append(kw)
    ev_pub.publish(String(data=json.dumps(kw)))
    print("%.2f %s" % (kw["t"], json.dumps(kw)), flush=True)


def now_truth():
    with lock:
        return truth[-1] if truth else None


def last_exec():
    with lock:
        return execs[-1][1] if execs else {}


def exec_alive():
    return subprocess.run(["pgrep", "-f", "/lib/l4_executive/mission_executive|/planner/mission_executive.py"], capture_output=True).returncode == 0


# ---- trigger --------------------------------------------------------------------------------------
spec = {}
for part in args.trigger.split(","):
    k, _, v = part.partition("=")
    spec[k.strip()] = v.strip()
event("armed", case=args.case, trigger=args.trigger, actions=args.action)
matched_at = None
fired_by = None
while not rospy.is_shutdown():
    e = last_exec()
    ph = e.get("phase")
    if ph in ("DONE", "STOPPED"):
        event("not_triggered", phase=ph, exec=e)
        break
    ok = True
    if "start" in spec:
        ok = ph not in (None, "WAIT_ARMED")
    if "phase" in spec:
        ok = ok and ph == spec["phase"]
    if "leg_type" in spec:
        ok = ok and ph == "LEG" and e.get("type") == spec["leg_type"]
    if "leg" in spec:
        ok = ok and e.get("leg") == int(spec["leg"])
    tr = now_truth()
    if ok and "near" in spec and tr is not None:
        x, y, r = (float(a) for a in spec["near"].split(":"))
        ok = math.hypot(tr[1] - x, tr[2] - y) <= r
    if ok and matched_at is None:
        matched_at = rospy.get_time()
        event("matched", exec=e)
    if not ok:
        matched_at = None
    if ok and tr is not None:
        sp = math.hypot(tr[4], tr[5])
        if "min_speed" not in spec or sp >= float(spec["min_speed"]):
            fired_by = "condition"
            break
        if "max_wait" in spec and rospy.get_time() - matched_at >= float(spec["max_wait"]):
            fired_by = "max_wait (speed %.2f m/s below min_speed)" % sp
            break
    time.sleep(0.02)

if fired_by and "delay" in spec:
    time.sleep(float(spec["delay"]))

# ---- inject ---------------------------------------------------------------------------------------
t_fault = None
boxes = []
if fired_by:
    tr = now_truth()
    t_fault = rospy.get_time()
    event("fault", fired_by=fired_by, speed=round(math.hypot(tr[4], tr[5]), 3), pos=[round(x, 3) for x in tr[1:4]],
          exec=last_exec(), bridge=bridge_last[0])
    for a in args.action:
        kind, _, rest = a.partition(":")
        if kind == "pkill":
            r = subprocess.run("pkill " + rest, shell=True, capture_output=True, text=True)
            event("pkill", args=rest, returncode=r.returncode)
        elif kind == "render":
            cmd_pub.publish(String(data=rest))
            event("render", cmd=json.loads(rest))
        elif kind in ("box_ahead", "box_at"):
            tr = now_truth()
            if kind == "box_ahead":
                d, size, cz = rest.split(":")
                v = np.array([tr[4], tr[5]])
                n = float(np.linalg.norm(v))
                u = v / n if n > 0.05 else np.array([1.0, 0.0])
                c = [tr[1] + u[0] * float(d), tr[2] + u[1] * float(d), float(cz)]
            else:
                ctr, size = rest.split(":")
                c = [float(x) for x in ctr.split(",")]
            size = [float(x) for x in size.split(",")]
            bid = "b%d" % len(boxes)
            boxes.append((np.array(c) - np.array(size) / 2, np.array(c) + np.array(size) / 2))
            cmd_pub.publish(String(data=json.dumps({"box": {"id": bid, "center": c, "size": size}})))
            event("box", id=bid, center=[round(x, 3) for x in c], size=size)
        elif kind == "sleep":
            time.sleep(float(rest))
            event("slept", s=float(rest))
        elif kind == "none":
            pass
        else:
            event("unknown_action", action=a)

# ---- observe --------------------------------------------------------------------------------------
t_end = None
end_why = None
t_exec_end = None
while not rospy.is_shutdown() and t_fault is not None:
    now = rospy.get_time()
    e = last_exec()
    if t_exec_end is None and e.get("phase") in ("DONE", "STOPPED"):
        t_exec_end = now
    if t_exec_end is not None and now - t_exec_end >= args.end_hold - 1.0:
        end_why = "executive %s + %.0f s" % (e.get("phase"), args.end_hold - 1.0)
        break
    if t_exec_end is None and not exec_alive():
        end_why = "executive process gone"
        time.sleep(min(args.end_hold, 10.0))
        break
    if now - t_fault >= args.max_s:
        end_why = "max_s %.0f s after the fault" % args.max_s
        break
    time.sleep(0.1)
t_end = rospy.get_time()


# ---- result ---------------------------------------------------------------------------------------
def r3(x):
    return round(float(x), 3)


res = {"case": args.case, "trigger": args.trigger, "actions": args.action, "fired_by": fired_by,
       "end_why": end_why, "events": events}
res["planner_md5"] = {p.split("/")[-1]: hashlib.md5(open(p, "rb").read()).hexdigest()[:8]
                      for p in sorted(glob.glob("/planner/*.py") + glob.glob("/ws*/install/lib/l4_*/*"))}
with lock:
    T = np.array(truth) if truth else np.zeros((0, 7))
    M, E, B, X, K = list(modes), list(execs), list(bridge), list(texts), list(track)
if t_fault is not None and len(T):
    i0 = int(np.searchsorted(T[:, 0], t_fault))
    W = T[(T[:, 0] >= t_fault) & (T[:, 0] <= t_end)]
    p0, v0 = T[min(i0, len(T) - 1), 1:4], T[min(i0, len(T) - 1), 4:7]
    sp0 = float(np.hypot(*v0[:2]))
    u = v0[:2] / sp0 if sp0 > 0.05 else np.zeros(2)
    hs = np.hypot(W[:, 4], W[:, 5])
    stop = None
    for k in range(len(W)):
        j = np.searchsorted(W[:, 0], W[k, 0] + 1.0)
        if j >= len(W):
            break
        if np.all(hs[k:j] < 0.1):
            dp = W[k, 1:3] - p0[:2]
            path = float(np.sum(np.hypot(np.diff(W[:k + 1, 1]), np.diff(W[:k + 1, 2]))))
            stop = {"t_s": r3(W[k, 0] - t_fault), "dist_m": r3(np.hypot(*dp)), "along_m": r3(dp @ u),
                    "path_m": r3(path), "pos": [r3(x) for x in W[k, 1:4]]}
            break
    exc = np.hypot(W[:, 1] - p0[0], W[:, 2] - p0[1])
    at = lambda dt: W[min(int(np.searchsorted(W[:, 0], t_fault + dt)), len(W) - 1)]  # noqa: E731
    res["fault"] = {"t": t_fault, "pos": [r3(x) for x in p0], "speed": r3(sp0), "vz": r3(v0[2])}
    res["stop"] = stop
    res["max_excursion_m"] = r3(exc.max()) if len(exc) else None
    res["along_at_0.5s_m"] = r3((at(0.5)[1:3] - p0[:2]) @ u)
    res["z_min_max"] = [r3(W[:, 3].min()), r3(W[:, 3].max())] if len(W) else None
    res["end"] = {"dt": r3(t_end - t_fault), "pos": [r3(x) for x in W[-1, 1:4]], "speed": r3(hs[-1]),
                  "dist_from_fault_m": r3(np.hypot(*(W[-1, 1:3] - p0[:2]))),
                  "mode": M[-1][1] if M else None, "armed": M[-1][2] if M else None,
                  "connected": M[-1][3] if M else None, "exec_phase": (E[-1][1].get("phase") if E else None),
                  "exec_alive": exec_alive()}
    if boxes:
        dmin = []
        for row in W:
            p = row[1:4]
            dmin.append(min(float(np.linalg.norm(np.maximum(np.maximum(lo - p, p - hi), 0))) for lo, hi in boxes))
        k = int(np.argmin(dmin))
        res["box_min_dist_m"] = {"centre_to_box": r3(dmin[k]), "t_s": r3(W[k, 0] - t_fault),
                                 "pos": [r3(x) for x in W[k, 1:4]],
                                 "note": "FC centre to the box surface; the frame reaches %.2f m out" % HALF[:2].max()}
    rel = lambda t: r3(t - t_fault)  # noqa: E731
    res["modes"] = [[rel(t), m, a] for t, m, a, _c in M if t >= t_fault - 2.0]
    res["mode_before"] = next(([m, a] for t, m, a, _c in reversed(M) if t < t_fault), None)
    res["exec"] = [[rel(t), {k: v for k, v in s.items() if k in ("phase", "leg", "type", "reason", "clear", "resumed", "on_ground", "refused", "waiting")}]
                   for t, s in E if t >= t_fault - 1.0]
    res["bridge"] = [[rel(t), s, r, en, md] for t, s, r, en, md in B if t >= t_fault - 1.0]
    res["texts"] = [[rel(t), x] for t, x in X if t >= t_fault - 1.0]
    res["track"] = [[rel(t), s, r] for t, s, r in K if t >= t_fault - 1.0]
json.dump(res, open(args.out, "w"), indent=1)
print(json.dumps({k: res.get(k) for k in ("case", "fired_by", "fault", "stop", "max_excursion_m", "end", "box_min_dist_m")}), flush=True)
print("wrote", args.out, flush=True)
