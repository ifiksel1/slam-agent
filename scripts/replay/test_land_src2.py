#!/usr/bin/env python3
"""Regression test: restart_mode="land" switches the EKF to SRC2 BEFORE commanding LAND.

FC log 233 (2026-09-29, t=1402-1407): a latency CRITICAL in Loiter with restart_mode=land did no
source switch at all, so the EKF kept SLAM height (EK3_SRC1_POSZ=6). SLAM froze, moving ~0.25 m
while the rangefinder read 2.0 -> 4.5 m, and the vehicle climbed at 4 m/s on full throttle with the
pilot's stick at minimum. The LAND that followed would also have descended on SLAM height.

Unlike the other tests here this drives the REAL DriftMonitor, not a mirror: rospy, std_msgs and the
clock are stubbed, the FC calls are recorded instead of sent, and health_cb is fed synthetic
/fastlio_health scans. Nothing here talks to ROS or the FC.

Run: python3 scripts/replay/test_land_src2.py
"""
import importlib.util
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "..", "overrides", "FAST-LIO", "src", "drift_monitor.py")
FAILURES = []
CLOCK = [1000.0]
PARAMS = {}


# ---- stubs: just enough of rospy / std_msgs for DriftMonitor.__init__ and health_cb ----
class _Stub:
    def __init__(self, *a, **k):
        self.__dict__.update(k)

    def publish(self, *a, **k):
        pass


def _noop(*a, **k):
    pass


rospy = types.ModuleType("rospy")
rospy.get_param = lambda name, default=None: PARAMS.get(name, default)
rospy.get_time = lambda: CLOCK[0]
rospy.Publisher = rospy.Subscriber = rospy.Timer = rospy.Duration = _Stub
class _Time(float):
    @staticmethod
    def now():
        return _Time(CLOCK[0])

    def to_sec(self):
        return float(self)


rospy.Time = _Time
for _n in ("init_node", "loginfo", "logwarn", "logerr", "logwarn_throttle", "logerr_throttle", "loginfo_throttle"):
    setattr(rospy, _n, _noop)
std_msgs = types.ModuleType("std_msgs")
std_msgs.msg = types.ModuleType("std_msgs.msg")
std_msgs.msg.Float32MultiArray = std_msgs.msg.String = _Stub
sys.modules.update({"rospy": rospy, "std_msgs": std_msgs, "std_msgs.msg": std_msgs.msg})

spec = importlib.util.spec_from_file_location("drift_monitor", SRC)
dm = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dm)
dm.time = types.SimpleNamespace(time=lambda: CLOCK[0])


def make(mode, src2_ok=True):
    """A DriftMonitor whose FC actions are recorded, in order, instead of sent."""
    PARAMS.clear()
    PARAMS.update({"~restart_mode": mode, "~enable_auto_restart": True,
                   "~enable_source_failover": True, "~enable_latency_critical": True})
    CLOCK[0] = 1000.0
    m = dm.DriftMonitor()
    m.calls = []
    m.banners = []
    m._spawn = lambda fn, *a: fn(*a)            # synchronous, so ordering is observable

    def set_src(pos, label):
        ok = src2_ok if pos == m.src2_switch_pos else True
        m.calls.append(("SRC2" if pos == m.src2_switch_pos else "SRC1") + ("" if ok else ":refused"))
        return ok
    m._set_ekf_source = set_src
    m._command_land = lambda: m.calls.append("LAND") or True
    m._send_statustext = lambda sev, text, force=False: m.banners.append(text)
    dm.subprocess = types.SimpleNamespace(Popen=lambda argv: m.calls.append("kill"))
    return m


def scans(m, n, matched=600.0, eig=150000.0, lat_ms=70.0, dt=0.05):
    for _ in range(n):
        CLOCK[0] += dt
        m.health_cb(types.SimpleNamespace(data=[matched, 0, 0, 0, eig, 0, 0, lat_ms]))


def armed(m, state):
    m.state_cb(types.SimpleNamespace(armed=state, connected=False))


def gap(m, sec=3.0):
    CLOCK[0] += sec                             # laserMapping restarting: no health at all


def check(label, got, want):
    ok = got == want
    print("    %-62s %s" % (label, "PASS" if ok else "FAIL (got %r, want %r)" % (got, want)))
    if not ok:
        FAILURES.append(label)


print("=== land, drift CRITICAL while flying ===")
m = make("land")
scans(m, 500)                                   # past warmup and the 20 s startup blank
armed(m, True)
scans(m, 6, matched=100.0)
check("SRC2 first, then LAND, then the node kill", m.calls, ["SRC2", "LAND", "kill"])
gap(m)
m.calls = []
scans(m, 120)                                   # re-warm, then k_release / lat_k_release
check("SLAM recovered while armed: stays on SRC2", m.calls, [])
check("  and says so", "SLAM back, staying on SRC2" in m.banners, True)
armed(m, False)
scans(m, 60)
check("disarm does not switch back either", m.calls, [])
armed(m, True)
scans(m, 1000)                                  # past the 45 s restart cooldown
scans(m, 6, matched=100.0)
check("the NEXT fault re-sends SRC2 (FC may have been rebooted onto SRC1)",
      m.calls, ["SRC2", "LAND", "kill"])

print("\n=== land, disarmed before SLAM is back ===")
m = make("land")
scans(m, 500)
armed(m, True)
scans(m, 6, matched=100.0)
armed(m, False)                                 # touched down and disarmed during the restart
gap(m)
m.calls = []
scans(m, 120)
check("recovery while disarmed: still no SRC1", m.calls, [])

print("\n=== land, latency CRITICAL: the log 233 path ===")
m = make("land")
scans(m, 500)
armed(m, True)
scans(m, 21, lat_ms=400.0)                      # lat_k_crit = 20
check("latency CRITICAL switches to SRC2 (used to do nothing)", m.calls, ["SRC2"])
m.calls = []
scans(m, 110, lat_ms=400.0)                     # still bad after lat_drain_sec (5 s)
check("drain escalation LANDs without a second switch", m.calls, ["LAND", "kill"])

print("\n=== land, FC refuses the source switch ===")
m = make("land", src2_ok=False)
scans(m, 500)
armed(m, True)
scans(m, 6, matched=100.0)
check("still LANDs: refused switch is no reason to stay up", m.calls, ["SRC2:refused", "LAND", "kill"])
check("  and says the switch failed", "SRC2 switch FAILED" in m.banners, True)
check("  nothing to restore later", m.failover_active, False)

print("\n=== land, laserMapping dies on its own (health watchdog) ===")
m = make("land")
scans(m, 500)
armed(m, True)
CLOCK[0] += 1.2                                 # past health_timeout_sec (1.0), no scans
m._watchdog_cb(None)
check("watchdog switches to SRC2 (used to do nothing in land)", m.calls, ["SRC2"])
check("  no LAND and no kill from the watchdog", "LAND" in m.calls or "kill" in m.calls, False)
CLOCK[0] += 0.5
m._watchdog_cb(None)
check("  one-shot per outage", m.calls, ["SRC2"])
CLOCK[0] += 1.5                                 # respawned laserMapping comes back
m.calls = []
scans(m, 120)
check("recovered: stays on SRC2", m.calls, [])
armed(m, False)
check("  disarm does not switch back", m.calls, [])

print("\n=== of mode is unchanged ===")
m = make("of")
scans(m, 500)
armed(m, True)
scans(m, 6, matched=100.0)
check("SRC2 then kill, no LAND", m.calls, ["SRC2", "kill"])
gap(m)
m.calls = []
scans(m, 120)
check("switches back to SRC1 while flying, as before", m.calls, ["SRC1"])

print()
if FAILURES:
    print("%d FAILED" % len(FAILURES))
    sys.exit(1)
print("ALL PASS")
