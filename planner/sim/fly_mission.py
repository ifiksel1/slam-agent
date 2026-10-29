#!/usr/bin/env python3
"""L2 closed loop: fly one mission in SITL with SUPER, the planner bridge and the mission executive.

Runs inside an L2 run's ROS container (see closed_loop.sh), after SUPER and the planner bridge are up.

1. Converts the mission's waypoints from the map frame (the MARSIM mission files) to the fc frame,
   through /sim/map_to_planner (map -> planner) and the bridge's /planner_bridge/anchor (planner -> fc),
   and writes /run/mission_fc.yaml. The executive converts them back with the live anchor (D7).
2. Starts the mission executive (the C++ l4_executive node; BRIDGE_IMPL=py: the former Python one) on that file.
3. Arms SITL (SIMULATION ONLY: tcp 5763 inside the run's private network namespace; on the robot the
   pilot arms, D5) and calls /mission_executive/start.
4. Waits for the executive's DONE or STOPPED, then lands in SITL (QRLAND) and disarms.

usage: fly_mission.py <mission_map_frame.yaml> [timeout_s]
"""
import json
import math
import os
import subprocess
import sys
import time

import numpy as np
import rospy
import yaml
from geometry_msgs.msg import TransformStamped
from pymavlink import mavutil
from std_msgs.msg import String
from std_srvs.srv import Trigger

src = sys.argv[1]
TIMEOUT = float(sys.argv[2]) if len(sys.argv) > 2 else 900.0
OUT = "/run/mission_fc.yaml"
QRLAND = 21

rospy.init_node("fly_mission")
anchor = {}
status = {}


def on_anchor(m):
    anchor["t"] = np.array([m.transform.translation.x, m.transform.translation.y, m.transform.translation.z])
    q = m.transform.rotation
    anchor["psi"] = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def on_status(m):
    status.update(json.loads(m.data))
    print("%.1f exec %s" % (time.time(), m.data), flush=True)


rospy.Subscriber("/planner_bridge/anchor", TransformStamped, on_anchor)
rospy.Subscriber("/mission_executive/status", String, on_status)


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


t0 = time.time()
while not rospy.has_param("/sim/map_to_planner") or "t" not in anchor:
    if time.time() - t0 > 120:
        sys.exit("no /sim/map_to_planner or /planner_bridge/anchor after 120 s")
    time.sleep(0.5)
m2p = rospy.get_param("/sim/map_to_planner")
yaw, start = math.radians(m2p["yaw_deg"]), np.array(m2p["start"])
json.dump({"yaw_deg": m2p["yaw_deg"], "start": list(m2p["start"])}, open("/run/map_to_planner.json", "w"))
print("map->planner yaw %.2f deg start %s; anchor psi %.2f deg t %s" % (
    m2p["yaw_deg"], np.round(start, 3), math.degrees(anchor["psi"]), np.round(anchor["t"], 3)), flush=True)


def map_to_fc(p):
    planner = rz(-yaw) @ (np.asarray(p, float) - start)
    return rz(anchor["psi"]) @ planner + anchor["t"]


doc = yaml.safe_load(open(src))
for leg in doc["legs"]:
    if "goal" in leg:
        g = leg["goal"]
        fc = map_to_fc(list(g) + [0.0] * (3 - len(g)))
        leg["goal"] = [round(float(x), 3) for x in fc[:len(g)]]
    if "at" in leg:      # land legs: the spot, on the ground
        leg["at"] = [round(float(x), 3) for x in map_to_fc(list(leg["at"])[:2] + [0.0])[:2]]
    if "yaw" in leg:     # a deliberate waypoint yaw (deg): map heading -> fc heading
        leg["yaw"] = round(float(leg["yaw"]) + math.degrees(anchor["psi"] - yaw), 2)
doc["frame"] = "fc"
doc["note"] = "converted from %s (map frame) by fly_mission.py" % src
yaml.safe_dump(doc, open(OUT, "w"), sort_keys=False)
print("mission in the fc frame: %s" % OUT, flush=True)

# the mission executive: the C++ node (l4_executive, copied into the run by closed_loop.sh), or with
# BRIDGE_IMPL=py the former Python one from a planner/ copy that still has it (A/B only)
if os.environ.get("BRIDGE_IMPL", "cpp") == "py":
    EXEC_CMD = ["python3", "-u", "/planner/mission_executive.py"]
else:
    EXEC_CMD = [os.environ.get("EXEC_BIN", "/ws_bx/install/lib/l4_executive/mission_executive")]
ex = subprocess.Popen(EXEC_CMD + ["__name:=mission_executive", "_mission:=" + OUT, "_on_unreachable:=next"],
                      stdout=open("/run/logs/executive.log", "w"), stderr=subprocess.STDOUT)
rospy.wait_for_service("/mission_executive/start", 60)

# arm, in SITL only (D5: on the robot the pilot arms)
mav = mavutil.mavlink_connection("tcp:127.0.0.1:5763", source_system=252)
mav.wait_heartbeat(timeout=30)
armed = False
for _ in range(10):
    mav.arducopter_arm()
    t1 = time.time()
    while time.time() - t1 < 3:
        hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb and hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            armed = True
            break
    if armed:
        break
if not armed:
    ex.terminate()
    sys.exit("SITL did not arm")
print("%.1f armed in SITL" % time.time(), flush=True)
time.sleep(1)
r = rospy.ServiceProxy("/mission_executive/start", Trigger)()
print("%.1f start: %s %s" % (time.time(), r.success, r.message), flush=True)

t_start = time.time()
while time.time() - t_start < TIMEOUT and status.get("phase") not in ("DONE", "STOPPED"):
    if ex.poll() is not None:
        print("executive exited %s" % ex.returncode, flush=True)
        break
    time.sleep(0.5)
print("%.1f mission end: %s after %.0f s" % (time.time(), status.get("phase"), time.time() - t_start), flush=True)

# failure tests: leave the vehicle to the FC and the stack for END_HOLD_S before landing it (default 0)
end_hold = float(__import__("os").environ.get("END_HOLD_S", "0") or 0)
if end_hold > 0:
    print("%.1f holding %.0f s before landing (END_HOLD_S)" % (time.time(), end_hold), flush=True)
    time.sleep(end_hold)

# land and disarm, in SITL only
mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                          mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, QRLAND, 0, 0, 0, 0, 0)
t2 = time.time()
while time.time() - t2 < 60:
    hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
    if hb and hb.get_srcSystem() == 1 and hb.get_srcComponent() == 1:
        if not hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            break
        if hb.custom_mode == 0:
            break      # QuadRover lands to MANUAL and stays armed (it drives on): on the ground
time.sleep(1)
mav.arducopter_disarm()
ex.terminate()
print("%.1f landed and disarmed in SITL" % time.time(), flush=True)
