#!/usr/bin/env python3
"""restart_mode="land" picks QRLand on QuadRover and LAND on Copter (drift_monitor.land_command).

QuadRover (~quadrover, set by fastlio_to_fc_quadrover.launch) flies copter modes on the Rover
firmware: Rover has no "LAND" (QRLand is custom mode 21), and MAVROS's set_mode refuses "CMODE(21)",
so the land goes by number through MAV_CMD_DO_SET_MODE. Copter keeps the name, and must NOT get 21:
on Copter that number is Smart RTL. Runs the real function.
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

from drift_monitor import land_command  # noqa: E402

CASES = [
    (True, 'LAND', 21, ('custom', 21), 'QuadRover: QRLand by number'),
    (False, 'LAND', 21, ('name', 'LAND'), 'Copter: LAND by name, never 21 (Smart RTL)'),
    (True, 'LAND', -1, ('name', 'LAND'), 'land_mode_rover -1: always the name'),
]
failures = 0
for quadrover, name, rover, want, what in CASES:
    got = land_command(quadrover, name, rover)
    ok = got == want
    failures += not ok
    print(('  ok    ' if ok else '  FAIL  ') + '%s: %s' % (what, got))
print('ALL PASS' if not failures else '%d FAILED' % failures)
sys.exit(1 if failures else 0)
