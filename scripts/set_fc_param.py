#!/usr/bin/env python3
"""Set one parameter on the FC and verify it read back.

Usage: set_fc_param.py NAME VALUE [device]

Reads the current value first, writes, then re-reads from the autopilot rather
than trusting the PARAM_VALUE echo. Prints the before/after so the change is
traceable. Does not reboot.
"""
import sys
import time

from pymavlink import mavutil

name = sys.argv[1]
value = float(sys.argv[2])
dev = sys.argv[3] if len(sys.argv) > 3 else '/dev/ttyACM0'

m = mavutil.mavlink_connection(dev, baud=115200)
m.wait_heartbeat(timeout=20)
print("heartbeat sys=%d comp=%d" % (m.target_system, m.target_component))


def fetch(pname, timeout=8.0):
    m.mav.param_request_read_send(m.target_system, m.target_component,
                                  pname.encode('ascii'), -1)
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
        if msg is not None and msg.param_id.strip('\x00') == pname:
            return msg.param_value, msg.param_type
    return None, None


before, ptype = fetch(name)
if before is None:
    sys.exit("could not read %s" % name)
print("before: %s = %g (type %s)" % (name, before, ptype))

if abs(before - value) < 1e-9:
    print("already at target, nothing to do")
    sys.exit(0)

for attempt in range(1, 6):
    m.mav.param_set_send(m.target_system, m.target_component,
                         name.encode('ascii'), value, ptype)
    time.sleep(0.6)
    after, _ = fetch(name, timeout=5.0)
    if after is not None and abs(after - value) < 1e-9:
        print("after:  %s = %g   VERIFIED (attempt %d)" % (name, after, attempt))
        sys.exit(0)
    print("  attempt %d: read back %s" % (attempt, after))

sys.exit("FAILED to set %s to %g" % (name, value))
