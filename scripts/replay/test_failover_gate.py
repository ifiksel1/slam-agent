#!/usr/bin/env python3
"""No optical-flow failover while a QuadRover drives (drift_monitor.failover_allowed), and the
autonomous drive modes HOLD through the SLAM reset and resume after it (drive_hold_mode).

QuadRover's copter motors are SHUT_DOWN while driving and spin in every flight mode and Auto
flight leg, so the gate is "any of outputs 1-4 above motor_run_us". Not a QuadRover (~quadrover
false: Copter) and missing or stale outputs keep the old behaviour (failover allowed). Runs the
real functions.
"""
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'overrides', 'FAST-LIO', 'src'))
sys.modules["rospy"] = types.ModuleType("rospy")
for _n in ("std_msgs", "std_msgs.msg", "nav_msgs", "nav_msgs.msg",
           "mavros_msgs", "mavros_msgs.msg", "mavros_msgs.srv"):
    sys.modules[_n] = types.ModuleType(_n)
sys.modules["std_msgs.msg"].Float32MultiArray = object
sys.modules["std_msgs.msg"].String = object
sys.modules["nav_msgs.msg"].Odometry = object

from drift_monitor import drive_hold_mode, failover_allowed  # noqa: E402

DRIVING = [1000, 1000, 1000, 1000, 1700, 1300, 0, 0]       # copter motors shut down, tracks moving
GROUND_IDLE = [1164, 1164, 1164, 1164, 1500, 1500, 0, 0]    # QR mode, landed at ground idle
HOVER = [1420, 1460, 1440, 1450, 1500, 1500, 0, 0]
CASES = [
    (True, DRIVING, 0.1, True, False, 'QuadRover driving: no failover'),
    (True, HOVER, 0.1, True, True, 'QuadRover flying: failover'),
    (True, GROUND_IDLE, 0.1, True, True, 'QuadRover in a QR mode at ground idle: failover (Copter-like)'),
    (True, DRIVING, 0.1, False, True, 'gate disabled (no_failover_driving false): failover'),
    (False, DRIVING, 0.1, True, True, 'not a QuadRover (Copter): always failover'),
    (True, None, 99.0, True, True, 'no outputs seen: failover (old behaviour)'),
    (True, DRIVING, 2.0, True, True, 'outputs stale (2 s): failover (old behaviour)'),
]
failures = 0
for quadrover, out, age, enabled, want, what in CASES:
    got = failover_allowed(quadrover, out, age, 1050.0, enabled)
    ok = got == want
    failures += not ok
    print(('  ok    ' if ok else '  FAIL  ') + '%s: %s' % (what, got))
# ...and while driving, which modes HOLD through the reset and come back (owner 2026-10-20)
HOLD_CASES = [
    (True, True, 'GUIDED', True, 15, 'Guided: hold, resume Guided (15)'),
    (True, True, 'AUTO', True, 10, 'Auto ground leg: hold, resume Auto (10)'),
    (True, True, 'RTL', True, 11, 'RTL: hold, resume RTL (11)'),
    (True, True, 'SMART_RTL', True, 12, 'SmartRTL: hold, resume (12)'),
    (True, True, 'MANUAL', True, None, 'Manual (pilot-driven): leave alone'),
    (True, True, 'ACRO', True, None, 'Acro (pilot-driven): leave alone'),
    (True, True, 'HOLD', True, None, 'already in Hold: nothing to do'),
    (True, False, 'GUIDED', True, None, 'disarmed: leave alone'),
    (True, True, 'GUIDED', False, None, 'hold_driving_on_reset false: leave alone'),
    (False, True, 'GUIDED', True, None, 'not a QuadRover (Copter): never'),
]
for quadrover, armed, mode, enabled, want, what in HOLD_CASES:
    got = drive_hold_mode(quadrover, armed, mode, enabled)
    ok = got == want
    failures += not ok
    print(('  ok    ' if ok else '  FAIL  ') + '%s: %s' % (what, got))
print('ALL PASS' if not failures else '%d FAILED' % failures)
sys.exit(1 if failures else 0)
