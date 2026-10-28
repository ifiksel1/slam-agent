#!/usr/bin/env python3
"""SITL lift-off check: does the vehicle really leave the ground? Compares SITL's true altitude
(SIM_STATE) with the EKF's (LOCAL_POSITION_NED) through a Guided takeoff. SITL only (tcp 5763 inside
an L2 run's private namespace).

usage: lift_test.py [height_m]
"""
import sys
import time

from pymavlink import mavutil

H = float(sys.argv[1]) if len(sys.argv) > 1 else 1.5
mav = mavutil.mavlink_connection("tcp:127.0.0.1:5763", source_system=253)
mav.wait_heartbeat(timeout=60)
for mid in (108, 32):   # SIM_STATE, LOCAL_POSITION_NED at 10 Hz
    mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mid, 100000, 0, 0, 0, 0, 0)
t0 = time.time()
while time.time() - t0 < 90:   # EKF on vision
    m = mav.recv_match(type="EKF_STATUS_REPORT", blocking=True, timeout=1)
    if m and m.flags & 0x10:
        break
time.sleep(10)
alt0 = None
while alt0 is None:
    m = mav.recv_match(type="SIM_STATE", blocking=True, timeout=2)
    alt0 = m.alt if m else None
for _ in range(10):
    mav.arducopter_arm()
    hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
    if hb and hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        break
mav.set_mode(15)
time.sleep(1)
mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, H)
true_z = ekf_z = 0.0
t1 = time.time()
last = 0
peak = 0.0
while time.time() - t1 < 25:
    m = mav.recv_match(type=["SIM_STATE", "LOCAL_POSITION_NED", "STATUSTEXT"], blocking=True, timeout=1)
    if m is None:
        continue
    if m.get_type() == "SIM_STATE":
        true_z = m.alt - alt0
        peak = max(peak, true_z)
    elif m.get_type() == "LOCAL_POSITION_NED":
        ekf_z = -m.z
    else:
        print("  text: %s" % m.text, flush=True)
    if time.time() - last > 2:
        last = time.time()
        print("t %4.1f  true height %5.2f  EKF height %5.2f" % (time.time() - t1, true_z, ekf_z), flush=True)
print("RESULT target %.1f m: true peak %.2f m, true final %.2f m, EKF final %.2f m" % (H, peak, true_z, ekf_z))
mav.set_mode(21)
time.sleep(10)
mav.arducopter_disarm()
