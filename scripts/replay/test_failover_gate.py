#!/usr/bin/env python3
"""No optical-flow failover while a QuadRover drives (drift_monitor.failover_allowed).

QuadRover's copter motors are SHUT_DOWN while driving and spin in every flight mode and Auto
flight leg, so the gate is "any of outputs 1-4 above motor_run_us". A Copter FC, an FC not yet
seen, and missing or stale outputs keep the old behaviour (failover allowed). Runs the real
function.
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

from drift_monitor import failover_allowed  # noqa: E402

DRIVING = [1000, 1000, 1000, 1000, 1700, 1300, 0, 0]       # copter motors shut down, tracks moving
GROUND_IDLE = [1164, 1164, 1164, 1164, 1500, 1500, 0, 0]    # QR mode, landed at ground idle
HOVER = [1420, 1460, 1440, 1450, 1500, 1500, 0, 0]
CASES = [
    (10, DRIVING, 0.1, True, False, 'QuadRover driving: no failover'),
    (10, HOVER, 0.1, True, True, 'QuadRover flying: failover'),
    (10, GROUND_IDLE, 0.1, True, True, 'QuadRover in a QR mode at ground idle: failover (Copter-like)'),
    (10, DRIVING, 0.1, False, True, 'gate disabled (no_failover_driving false): failover'),
    (2, DRIVING, 0.1, True, True, 'Copter FC: always failover'),
    (None, DRIVING, 0.1, True, True, 'FC not seen yet: failover (old behaviour)'),
    (10, None, 99.0, True, True, 'no outputs seen: failover (old behaviour)'),
    (10, DRIVING, 2.0, True, True, 'outputs stale (2 s): failover (old behaviour)'),
]
failures = 0
for mav_type, out, age, enabled, want, what in CASES:
    got = failover_allowed(mav_type, out, age, 1050.0, enabled)
    ok = got == want
    failures += not ok
    print(('  ok    ' if ok else '  FAIL  ') + '%s: %s' % (what, got))
print('ALL PASS' if not failures else '%d FAILED' % failures)
sys.exit(1 if failures else 0)
