#!/usr/bin/env python3
"""The FC as SITL reports it, for the failure tests, independent of MAVROS: a MAVLink link on
SITL's SERIAL3 (sitl_harness.sh up with SITL_ARGS="--serial3=tcp:4", EXTRA_PARMS "SERIAL3_PROTOCOL 2";
faults/cases.sh sets both) that logs the FC's mode and armed state on every change and its status
texts, as JSON lines in /run/logs/mav_watch.jsonl.

--gcs: also be the pilot's ground station link (the SIYI app): a GCS heartbeat at 1 Hz from system
255, the robot's MAV_GCS_SYSID. Killing this process is then the link loss; with the robot's
FS_GCS_ENABLE 1 / FS_GCS_TIMEOUT 2 the FC's GCS failsafe fires 2 s later. Without --gcs it is
system 254, which the FC does not count as its GCS.

SITL only (inside the run's private network namespace). It sends heartbeats and nothing else: no
commands, no arming, no parameters.

usage: mav_watch.py [--gcs] [url]   (default tcp:127.0.0.1:5764)
"""
import json
import sys
import time

from pymavlink import mavutil

gcs = "--gcs" in sys.argv
pos = [a for a in sys.argv[1:] if not a.startswith("--")]
url = pos[0] if pos else "tcp:127.0.0.1:5764"
OUT = "/run/logs/mav_watch.jsonl"
QR = {0: "MANUAL", 4: "HOLD", 10: "AUTO", 11: "RTL", 12: "SMART_RTL", 15: "GUIDED", 17: "QRSTABILIZE",
      18: "QRALTHOLD", 19: "QRLOITER", 20: "QRRTL", 21: "QRLAND", 22: "QRCIRCLE", 23: "QRSMARTRTL"}
m = None
for _ in range(60):
    try:
        m = mavutil.mavlink_connection(url, source_system=255 if gcs else 254, source_component=190,
                                       autoreconnect=True)   # a SITL watchdog reset drops the socket
        break
    except Exception as e:  # SITL not listening yet
        print("waiting for %s: %s" % (url, e), flush=True)
        time.sleep(1)
if m is None:
    sys.exit("no SITL on %s" % url)
out = open(OUT, "a")


def log(**kw):
    kw["t"] = round(time.time(), 3)
    out.write(json.dumps(kw) + "\n")
    out.flush()


log(kind="start", url=url, gcs=gcs)
print("%.1f mav_watch on %s, GCS heartbeat %s" % (time.time(), url, gcs), flush=True)
last = None
next_hb = 0.0
while True:
    now = time.time()
    if gcs and now >= next_hb:
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        next_hb = now + 1.0
    msg = m.recv_match(blocking=True, timeout=0.2)
    if msg is None:
        continue
    t = msg.get_type()
    if t == "HEARTBEAT" and msg.get_srcSystem() == 1 and msg.get_srcComponent() == 1:
        st = (QR.get(msg.custom_mode, str(msg.custom_mode)),
              bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))
        if st != last:
            log(kind="mode", mode=st[0], armed=st[1])
            last = st
    elif t == "STATUSTEXT":
        log(kind="text", text=msg.text)
