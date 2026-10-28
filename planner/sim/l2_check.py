#!/usr/bin/env python3
"""End-to-end check of the L2 sim harness (SITL only): run inside the ROS container,
   sitl_harness.sh exec <name> python3 -u /planner_sim/l2_check.py

1. EKF accepts the bridge's vision (EKF_STATUS_REPORT flags, variances) and tracks SITL's true pose.
2. Arms IN SITL (MANUAL, then GUIDED), NAV_TAKEOFF to 1.5 m, holds.
3. A 2 m setpoint_raw/local goto along fc +x through MAVROS: the move seen in the fc frame
   (LOCAL_POSITION_NED) and in the planner frame (render /Odometry through P) must agree.
4. The rendered /cloud_registered, put back in the map frame, lies on the map.
Talks to SITL on its SERIAL2 (tcp 5763, inside the private namespace) and to MAVROS; nothing else.
"""
import math
import sys
import time

import numpy as np
import rospy
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Odometry
from pymavlink import mavutil
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2

P = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], dtype=float)
results = []


def check(name, ok, detail):
    results.append((name, ok, detail))
    print("%s  %-34s %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)


rospy.init_node("l2_check", anonymous=True)
mav = mavutil.mavlink_connection("tcp:127.0.0.1:5763", source_system=251)
mav.wait_heartbeat(timeout=60)
for msg_id, us in ((193, 100000), (32, 50000)):   # EKF_STATUS_REPORT, LOCAL_POSITION_NED
    mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msg_id, us, 0, 0, 0, 0, 0)

odom = {}


def on_odom(m):
    p = m.pose.pose.position
    odom["planner"] = P @ np.array([p.x, p.y, p.z])


rospy.Subscriber("/Odometry", Odometry, on_odom, queue_size=5)
cloud = {}
rospy.Subscriber("/cloud_registered", PointCloud2, lambda m: cloud.__setitem__("msg", m), queue_size=1)
sp_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=5)


def latest(kind, timeout=5):
    """the newest message of this type (the link buffers everything since the last read)"""
    m = None
    while True:
        x = mav.recv_match(type=kind, blocking=False)
        if x is None:
            break
        m = x
    return m or mav.recv_match(type=kind, blocking=True, timeout=timeout)


def statustexts(seconds):
    t0, out = time.time(), []
    while time.time() - t0 < seconds:
        m = mav.recv_match(type="STATUSTEXT", blocking=True, timeout=0.5)
        if m:
            out.append(m.text)
    return out


# 1. EKF on the bridge's vision
need = 1 | 2 | 8 | 32  # attitude, horiz vel, horiz pos rel, vert pos
t0, ok = time.time(), False
while time.time() - t0 < 90:
    s = latest("EKF_STATUS_REPORT")
    if s and (s.flags & need) == need and not (s.flags & 1024):
        ok = True
        break
check("EKF uses the bridge's vision", ok, "flags 0x%x, pos_horiz_var %.2f, vel_var %.2f" % (s.flags, s.pos_horiz_variance, s.velocity_variance) if s else "no EKF_STATUS_REPORT")
lp = latest("LOCAL_POSITION_NED")
check("EKF local position present", lp is not None, "(%.2f, %.2f, %.2f) NED" % (lp.x, lp.y, lp.z) if lp else "none")

# 2. arm and take off, in SITL
mav.set_mode("MANUAL")
time.sleep(1)
mav.arducopter_arm()
armed = False
t0 = time.time()
while time.time() - t0 < 15 and not armed:
    hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
    # only the autopilot's own heartbeat (MAVROS also speaks as system 1)
    if hb and hb.get_srcSystem() == 1 and hb.get_srcComponent() == 1:
        armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
check("arms in SITL (MANUAL)", armed, "" if armed else "prearm: %s" % "; ".join(t for t in statustexts(3) if "PreArm" in t)[:300])
if not armed:
    sys.exit(1)
mav.set_mode("GUIDED")
time.sleep(1)
mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 1.5)
texts = statustexts(20)
lp = latest("LOCAL_POSITION_NED")
check("Guided takeoff to 1.5 m", lp is not None and 1.2 < -lp.z < 1.9, "height %.2f m; %s" % (-lp.z if lp else float("nan"), "; ".join(t for t in texts if "akeoff" in t)))
hold0 = latest("LOCAL_POSITION_NED")
time.sleep(8)
hold1 = latest("LOCAL_POSITION_NED")
drift = math.dist((hold0.x, hold0.y, hold0.z), (hold1.x, hold1.y, hold1.z))
check("holds in Guided for 8 s", drift < 0.3, "moved %.2f m" % drift)

# 3. a 2 m goto along fc +x (ENU east in MAVROS' local frame), position-only
a = latest("LOCAL_POSITION_NED")
pl0 = odom.get("planner")
target = PositionTarget()
target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
target.type_mask = 0b0000111111111000   # position only
target.position.x, target.position.y, target.position.z = a.y + 2.0, a.x, -a.z   # MAVROS takes ENU
t0 = time.time()
while time.time() - t0 < 12:
    target.header.stamp = rospy.Time.now()
    sp_pub.publish(target)
    time.sleep(0.05)
b = latest("LOCAL_POSITION_NED")
pl1 = odom.get("planner")
d_fc = np.array([b.y - a.y, b.x - a.x, -(b.z - a.z)])          # ENU
d_pl = (pl1 - pl0) if (pl0 is not None and pl1 is not None) else np.full(3, np.nan)
ang = math.degrees(math.atan2(d_pl[1], d_pl[0]) - math.atan2(d_fc[1], d_fc[0]))
check("goto moves +2 m east in the fc frame", abs(d_fc[0] - 2.0) < 0.3 and abs(d_fc[1]) < 0.3, "fc ENU move (%.2f, %.2f, %.2f) m" % tuple(d_fc))
check("same move in the planner frame", abs(np.linalg.norm(d_pl[:2]) - np.linalg.norm(d_fc[:2])) < 0.2,
      "planner move (%.2f, %.2f, %.2f) m, %.1f deg from the fc move (the anchor's yaw)" % (*d_pl, ((ang + 180) % 360) - 180))

# 4. the rendered cloud on the map
m = cloud.get("msg")
if m is not None:
    raw = np.frombuffer(m.data, dtype="<f4").reshape(-1, 4)[:, :3].astype(float)
    tf = rospy.get_param("/sim/map_to_planner")
    yaw, s0 = math.radians(tf["yaw_deg"]), np.array(tf["start"])
    c, s_ = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]])
    pts_map = (Rz @ (P @ raw.T)).T + s0
    pcd = open(rospy.get_param("/render_node/map_pcd"), "rb").read()
    k = pcd.index(b"DATA binary\n") + 12
    M = np.frombuffer(pcd[k:], dtype="<f4").reshape(-1, 4)[:, :3]
    d = cKDTree(M).query(pts_map[::10])[0]
    check("render cloud lies on the map", np.median(d) < 0.1, "%d points, median %.3f m, p95 %.3f m from the map" % (len(raw), np.median(d), np.percentile(d, 95)))
else:
    check("render cloud lies on the map", False, "no /cloud_registered received")

# land in SITL and disarm
mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
time.sleep(15)
mav.arducopter_disarm()
print("SUMMARY %d/%d passed" % (sum(r[1] for r in results), len(results)), flush=True)
