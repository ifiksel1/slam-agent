#!/usr/bin/env python3
"""Reboot the FC and verify a parameter survived, plus which rangefinders report.

Usage: reboot_fc_verify.py NAME EXPECTED [device]

AP_RangeFinder creates its backends in init() at boot, so a RNGFNDx_TYPE change
is only really applied after a reboot even though the parameter has no
@RebootRequired flag.
"""
import sys
import time

from pymavlink import mavutil

name = sys.argv[1]
expect = float(sys.argv[2])
dev = sys.argv[3] if len(sys.argv) > 3 else '/dev/ttyACM0'


def connect(timeout=30):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            c = mavutil.mavlink_connection(dev, baud=115200)
            if c.wait_heartbeat(timeout=6) is not None:
                return c
        except Exception:
            pass
        time.sleep(1.5)
    return None


m = connect()
if m is None:
    sys.exit("no heartbeat before reboot")
print("connected, requesting reboot...")
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                        0, 1, 0, 0, 0, 0, 0, 0)
time.sleep(3)
try:
    m.close()
except Exception:
    pass

print("waiting for the board to come back...")
time.sleep(6)
m = connect(timeout=60)
if m is None:
    sys.exit("FC did not come back after reboot")
print("back up: sys=%d comp=%d" % (m.target_system, m.target_component))

# banner + parameter
m.mav.param_request_read_send(m.target_system, m.target_component,
                              name.encode('ascii'), -1)
val = None
t0 = time.time()
while time.time() - t0 < 12:
    msg = m.recv_match(type=['PARAM_VALUE', 'STATUSTEXT'], blocking=True, timeout=2)
    if msg is None:
        continue
    if msg.get_type() == 'STATUSTEXT':
        print("  MSG: %s" % msg.text)
    elif msg.param_id.strip('\x00') == name:
        val = msg.param_value
        break
print("\n%s = %s  (expected %g)  -> %s" %
      (name, val, expect, "OK" if val is not None and abs(val - expect) < 1e-9 else "MISMATCH"))

# which rangefinder instances actually report now
print("\nlistening 10 s for DISTANCE_SENSOR instances...")
m.mav.request_data_stream_send(m.target_system, m.target_component,
                               mavutil.mavlink.MAV_DATA_STREAM_EXTRA3, 5, 1)
seen = {}
t0 = time.time()
while time.time() - t0 < 10:
    msg = m.recv_match(type='DISTANCE_SENSOR', blocking=True, timeout=2)
    if msg is None:
        continue
    seen.setdefault(msg.id, []).append(msg.current_distance)
if not seen:
    print("  no DISTANCE_SENSOR messages (stream may not be requested by default)")
for i in sorted(seen):
    d = seen[i]
    print("  id %d: %d msgs, %d-%d cm" % (i, len(d), min(d), max(d)))
