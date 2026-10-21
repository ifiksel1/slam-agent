#!/usr/bin/env python3
"""One STATUSTEXT per event on a QuadRover (drift_monitor.event_text, owner 2026-10-21).

The SIYI app shows only the newest message of a burst: at the HOLD-on-reset run (FC log 64)
drift_monitor sent "SLAM node down 1s", "SLAM fault driving: no OF failover" and "SLAM reset:
HOLD, then back to AUTO" within 10 ms, with the FC's "Mode Hold", and the pilot saw none of them.
Runs the real DriftMonitor methods on an instance built without ROS (FC calls stubbed, _spawn
synchronous) and checks that a QuadRover event sends exactly one message, that the old messages
are unchanged on other robots (~quadrover false), and that every combined message fits the 49
characters _send_statustext keeps.
"""
import itertools
import os
import sys
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'overrides', 'FAST-LIO', 'src'))
rospy = types.ModuleType("rospy")
rospy.logwarn = rospy.loginfo = rospy.logerr = lambda *a, **k: None
sys.modules["rospy"] = rospy
for _n in ("std_msgs", "std_msgs.msg", "nav_msgs", "nav_msgs.msg",
           "mavros_msgs", "mavros_msgs.msg", "mavros_msgs.srv"):
    sys.modules[_n] = types.ModuleType(_n)
sys.modules["std_msgs.msg"].Float32MultiArray = object
sys.modules["std_msgs.msg"].String = lambda **k: k
sys.modules["nav_msgs.msg"].Odometry = object

import drift_monitor  # noqa: E402
from drift_monitor import DriftMonitor, event_text, ROVER_AUTONOMOUS_MODES  # noqa: E402

DRIVING = [1000, 1000, 1000, 1000, 1700, 1300, 0, 0]      # copter motors shut down
HOVER = [1420, 1460, 1440, 1450, 1500, 1500, 0, 0]
failures = []


def check(cond, what):
    print(('  ok    ' if cond else '  FAIL  ') + what)
    if not cond:
        failures.append(what)


def monitor(quadrover, outputs=DRIVING, mode="AUTO", restart_mode="of", fc_ok=True):
    m = DriftMonitor.__new__(DriftMonitor)
    m.sent = []
    m._send_statustext = lambda sev, text, force=False: m.sent.append(text[:49])
    m._spawn = lambda fn, *a: fn(*a)
    m._set_custom_mode = lambda num, label: fc_ok
    m._set_ekf_source = lambda pos, label: fc_ok
    m._command_land = lambda: fc_ok
    m._refresh_interlock = lambda: None
    m.risk_pub = types.SimpleNamespace(publish=lambda msg: None)
    m.quadrover, m._rc_out, m._rc_out_t = quadrover, outputs, time.time()
    m.motor_run_us, m.no_failover_driving, m.hold_driving_on_reset = 1050.0, True, True
    m._mode, m._armed, m._drive_hold = mode, True, None
    m.restart_mode, m.src2_switch_pos = restart_mode, 2
    m.enable_source_failover, m.failover_active, m.enable_auto_restart = True, False, False
    m._suppress_node_kill = True
    m.enable_latency_critical, m._lat_blank_until = True, 0.0
    m._last_health_t, m.health_timeout = time.time() - 1.5, 1.0
    m._health_lost, m._health_wd_fired, m._lat_lock = False, False, False
    m._restart_lock = False
    return m


print('1. QuadRover: one message per event')
m = monitor(True)
m._watchdog_cb(None)
check(m.sent == ["SLAM down: HOLD, then AUTO"] and m._drive_hold == ("AUTO", 10),
      'FAST-LIO dies driving Auto: HOLD, one message %s' % m.sent)
m = monitor(True, outputs=HOVER)
m._watchdog_cb(None)
check(m.sent == ["SLAM down: switched to OF (SRC2)"] and m.failover_active,
      'FAST-LIO dies flying: OF, one message %s' % m.sent)
m = monitor(True, mode="MANUAL")
m._watchdog_cb(None)
check(m.sent == ["SLAM down: driving, no OF"], 'driving Manual: no hold, one message %s' % m.sent)
m = monitor(True)
m._execute_latency_failover("test")
check(m.sent == ["SLAM latency high: HOLD, then AUTO"], 'latency failover driving: %s' % m.sent)
m = monitor(True, outputs=HOVER, restart_mode="land")
m._restart_worker("Drift restart")
check(m.sent == ["Drift restart: switched to OF (SRC2), LANDING"], 'drift restart, land mode, flying: %s' % m.sent)
m = monitor(True)
m._restart_worker("CH9 restart")
check(m.sent == ["CH9 restart: HOLD, then AUTO"], 'CH9 restart driving Auto: %s' % m.sent)
m = monitor(True)
m._drive_hold, m._mode = ("AUTO", 10), "HOLD"
m._resume_after_reset()
check(m.sent == ["SLAM back: resuming AUTO"], 'resume: %s' % m.sent)

print('2. not a QuadRover: the old messages, unchanged')
m = monitor(False, outputs=HOVER)
m._watchdog_cb(None)
check(m.sent == ["SLAM node down 2s", "Switched to OF (SRC2)"], 'FAST-LIO dies: %s' % m.sent)
m = monitor(False, outputs=HOVER)
m._execute_latency_failover("test")
check(m.sent == ["SLAM latency high, switching to OF", "Switched to OF (SRC2)"], 'latency failover: %s' % m.sent)
m = monitor(False, outputs=HOVER, restart_mode="land", fc_ok=False)
m._restart_worker(None)
check(m.sent == ["SRC2 switch FAILED", "LAND cmd FAILED", "Restart in progress"], 'restart, land mode, FC refuses: %s' % m.sent)
m = monitor(False, outputs=HOVER)
m._restart_worker(None)
check(m.sent == ["Switched to OF (SRC2)", "Restart in progress"], 'restart, of mode: %s' % m.sent)

print('3. every combined message fits 49 characters')
causes = ["SLAM down", "SLAM latency high", "Drift restart", "CH9 restart", "SLAM degraded",
          "SLAM degraded, holding on OF"]
switch = ["switched to OF (SRC2)", "SRC2 switch FAILED", "driving, no OF"] + \
         ["HOLD, then %s" % mode for mode in ROVER_AUTONOMOUS_MODES]
land = [None, "LANDING", "LAND FAILED"]
# landing outcomes only on a restart (_restart_worker); the failover events never land
combos = [(c, s, None) for c, s in itertools.product(causes[:2], switch)] + \
    list(itertools.product(causes[2:5], switch + [None], land))
longest = max((event_text(c, *[o for o in (s, l) if o]) for c, s, l in combos), key=len)
check(len(longest) <= 49, 'longest restart/failover message %d: "%s"' % (len(longest), longest))
back = ["resuming %s" % mode for mode in ROVER_AUTONOMOUS_MODES] + \
       ["not resuming %s (mode changed)" % mode for mode in ROVER_AUTONOMOUS_MODES] + \
       ["%s refused" % mode for mode in ROVER_AUTONOMOUS_MODES]
longest = max((event_text("SLAM back", o) for o in back), key=len)
check(len(longest) <= 49, 'longest resume message %d: "%s"' % (len(longest), longest))
check(drift_monitor.event_text("SLAM degraded, holding on OF") == "SLAM degraded, holding on OF",
      'a cause with no outcome is sent alone')

print('\n%s' % ('ALL PASS' if not failures else '%d FAILED' % len(failures)))
sys.exit(1 if failures else 0)
