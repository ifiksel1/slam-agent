#!/usr/bin/env python3
"""Test of the SLAM-to-FC bridge's frozen-pose vision cut (fastlio_mavros_bridge.py: FrozenPoseCheck,
AirborneGate, FrozenVisionCut; owner decision 2026-11-02).

Synthetic streams at the robot's rates (pose 20 Hz, FC IMU / EKF velocity / setpoints 50 Hz, SERVO_OUTPUT_RAW
10 Hz, HEARTBEAT 1 Hz). Checks:
  1. frozen in flight with a streamed command (an L4 leg, SITL 2b) -> cut within ~0.5 s; vision stays cut;
  2. frozen in a manual flight: no command, the EKF velocity or the gyro is the evidence -> cut;
  3. frozen while driving (lift motors stopped) -> never cut, reported once ("ground");
  4. parked or hovering with the pose repeating and no motion evidence -> no cut (SITL repeats exactly);
  5. the FC stalled (its IMU stale) while the setpoints still stream -> no cut;
  6. the gate unknown (no /mavros/rc/out, stale outputs, stale state, disarmed, spin-arm idle) -> no cut;
  7. the latch: SLAM moving again in the air keeps the cut; landed (lift motors idle 3 s, or disarmed) with
     SLAM still frozen keeps it; landed AND resume_n new poses resumes; one new pose then the frozen one again
     does not;
  8. evidence with breaks never accumulates hold_s;
  9. a real FAST-LIO pose's smallest step (0.075 mm, 1.5e-5 per quaternion component) is not a repeat.
Runs the real module; rospy, tf and the ROS messages are stubbed when not installed (only the classes are used).
"""
import math
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'overrides', 'FAST-LIO', 'src'))


def _stub(name, **attrs):
    m = types.ModuleType(name)
    m.__dict__.update(attrs)
    sys.modules[name] = m
    return m


try:
    import rospy  # noqa: F401
    import tf.transformations  # noqa: F401
except ImportError:
    class _Any(object):
        def __init__(self, *a, **k):
            pass
    _stub('rospy')
    _stub('tf')
    _stub('tf.transformations', quaternion_multiply=None, quaternion_conjugate=None,
          quaternion_from_matrix=None, quaternion_from_euler=None)
    _stub('nav_msgs')
    _stub('nav_msgs.msg', Odometry=_Any)
    _stub('geometry_msgs')
    _stub('geometry_msgs.msg', PoseStamped=_Any, TwistStamped=_Any, Vector3Stamped=_Any)
    _stub('sensor_msgs')
    _stub('sensor_msgs.msg', Imu=_Any)
    _stub('std_msgs')
    _stub('std_msgs.msg', String=_Any)

from fastlio_mavros_bridge import AirborneGate, FrozenPoseCheck, FrozenVisionCut  # noqa: E402

failures = []


def check(cond, what):
    print(('  ok    ' if cond else '  FAIL  ') + what)
    if not cond:
        failures.append(what)


def make():
    return FrozenVisionCut(FrozenPoseCheck(), AirborneGate(), resume_n=5)


def run(cut, t0, t1, pose_at, armed=True, motors=1450.0, gyro=0.05, cmd=None, ekf=0.0, twist=0.0,
        imu_until=None, rcout=True, state=True):
    """step [t0, t1) at 1 ms; returns [(t, dropped)] per pose. pose_at(t) -> 7-tuple; cmd/ekf: float or f(t)"""
    out = []
    n0, n1 = int(round(t0 * 1000)), int(round(t1 * 1000))
    val = (lambda v, t: v(t) if callable(v) else v)
    for k in range(n0, n1):
        t = k / 1000.0
        if state and k % 1000 == 0:
            cut.gate.on_state(t, armed, "GUIDED")
        if rcout and k % 100 == 0:
            m = val(motors, t)
            cut.gate.on_motors(t, [m, m, m, m, 1500, 1500, 0, 0])
        if k % 20 == 0:
            if imu_until is None or t < imu_until:
                cut.check.on_gyro(t, val(gyro, t))
            if cmd is not None:
                cut.check.on_command(t, val(cmd, t))
            cut.check.on_ekf_velocity(t, val(ekf, t))
        if k % 50 == 0:
            out.append((t, cut.on_pose(t, pose_at(t), val(twist, t))))
    return out


def moving(t):
    return (0.5 * t, -0.2 * t, 1.0 + 0.001 * math.sin(t), 0.0, 0.0, math.sin(0.01 * t), math.cos(0.01 * t))


def frozen_at(tf):
    p = moving(tf)
    return lambda t: moving(t) if t < tf else p


def first_drop(out):
    return next((t for t, d in out if d), None)


def kinds(cut):
    return [k for k, _ in cut.events]


print("1. frozen in flight, streamed command (L4 leg)")
c = make()
run(c, 0.0, 5.0, moving, cmd=1.0, ekf=1.0)
out = run(c, 5.0, 12.0, frozen_at(5.0), cmd=1.0, ekf=0.0)
t = first_drop(out)
check(t is not None and t - 5.0 <= 0.6, "cut %.2f s after the freeze (<= 0.6)" % ((t or 99) - 5.0))
check(all(d for tt, d in out if tt >= t), "every pose after the cut dropped")
check(kinds(c) == ["cut"] and "command" in c.cut_evidence, "one cut event, evidence %s" % c.cut_evidence)

print("2. frozen in a manual flight: EKF velocity decaying, then the gyro")
c = make()
run(c, 0.0, 5.0, moving, ekf=1.0)
out = run(c, 5.0, 10.0, frozen_at(5.0), ekf=lambda t: max(0.0, 1.0 - 1.5 * (t - 5.0)))
t = first_drop(out)
check(t is not None and t - 5.0 <= 0.6, "EKF velocity: cut %.2f s after the freeze" % ((t or 99) - 5.0))
c = make()
run(c, 0.0, 5.0, moving)
out = run(c, 5.0, 10.0, frozen_at(5.0), gyro=lambda t: 0.05 if t < 7.0 else 0.4)
t = first_drop(out)
check(t is not None and 7.0 <= t <= 7.6, "no evidence until 7 s (hover), then gyro: cut at %.2f" % (t or 99))
check(c.cut_evidence == "gyro", "evidence gyro")

print("3. frozen while driving (lift motors stopped): vision kept")
c = make()
run(c, 0.0, 5.0, moving, motors=1000.0, cmd=1.0, ekf=1.0)
out = run(c, 5.0, 15.0, frozen_at(5.0), motors=1000.0, cmd=1.0, ekf=1.0, twist=1.0)
check(first_drop(out) is None, "no pose dropped")
check(kinds(c) == ["ground"], "reported once as frozen on the ground: %s" % kinds(c))
check(c.status(15.0)["frozen_on_ground"] is not None and c.status(15.0)["vision"] == "on", "status says so")

print("4. pose repeating without motion evidence: no cut")
c = make()
still = (1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0)
out = run(c, 0.0, 30.0, lambda t: still, motors=1000.0, gyro=0.01, ekf=0.02)
check(first_drop(out) is None and not c.events, "parked on the ground 30 s")
hov = (1.0, 2.0, 1.0, 0.0, 0.0, 0.0, 1.0)
out = run(c, 30.0, 60.0, lambda t: hov, motors=1450.0, gyro=0.05, ekf=0.05, cmd=0.0)
check(first_drop(out) is None and not c.events, "hovering 30 s with the pose repeating, nothing moves")

print("5. FC stalled (IMU stale), setpoints still streaming: no cut")
c = make()
run(c, 0.0, 5.0, moving, cmd=1.0)
out = run(c, 5.0, 8.0, frozen_at(5.0), cmd=1.0, imu_until=5.0)
check(first_drop(out) is None, "no cut while the FC's IMU is stale")

print("6. the airborne gate unknown or false: no cut")
for what, kw in (("no /mavros/rc/out", dict(rcout=False)), ("no /mavros/state", dict(state=False)),
                 ("disarmed", dict(armed=False)), ("spin-arm idle 1100 us", dict(motors=1100.0))):
    c = make()
    run(c, 0.0, 5.0, moving, cmd=1.0, **kw)
    out = run(c, 5.0, 8.0, frozen_at(5.0), cmd=1.0, **kw)
    check(first_drop(out) is None, what)
c = make()
run(c, 0.0, 3.5, moving, cmd=1.0)
run(c, 3.5, 5.0, moving, cmd=1.0, rcout=False)                  # outputs stop 1.5 s before the freeze
out = run(c, 5.0, 8.0, frozen_at(5.0), cmd=1.0, rcout=False)
check(first_drop(out) is None, "motor outputs stale (> 1 s) at the freeze")
g = AirborneGate()
g.on_state(0.0, True, "GUIDED")
g.on_motors(0.0, [1450] * 4)
check(g.airborne(0.5)[0] and not g.airborne(3.5)[0], "state older than 3 s -> not airborne")
check(not g.landed(10.0), "no fresh state or outputs -> not landed (no resume while blind)")

print("7. the latch and the resume")
c = make()
run(c, 0.0, 5.0, moving, cmd=1.0)
run(c, 5.0, 8.0, frozen_at(5.0), cmd=1.0)
check(c.cut, "cut")
out = run(c, 8.0, 12.0, moving, cmd=1.0)                         # SLAM back while still airborne
check(all(d for _, d in out), "SLAM moving again in the air: still cut")
p_frozen = c.frozen_pose
out = run(c, 12.0, 20.0, lambda t: p_frozen, motors=1000.0)      # landed, SLAM frozen again
check(all(d for _, d in out), "landed with the pose still frozen: still cut")
out = run(c, 20.0, 20.3, lambda t: moving(t) if t < 20.06 else p_frozen, motors=1000.0)
check(all(d for _, d in out), "one or two new poses then the frozen one again: still cut")
out = run(c, 20.3, 22.0, moving, motors=1000.0)
t = first_drop([(tt, not d) for tt, d in out])
check(t is not None and t <= 20.3 + 5 * 0.05 + 1e-6 and not c.cut, "landed + 5 new poses: resumed at %.2f" % (t or 99))
check(kinds(c) == ["cut", "resume"], "events %s" % kinds(c))
c = make()
run(c, 0.0, 5.0, moving, cmd=1.0)
run(c, 5.0, 8.0, frozen_at(5.0), cmd=1.0)
out = run(c, 8.0, 8.5, moving, motors=1450.0, armed=False)       # disarmed: landed at once
check(not c.cut, "disarmed + SLAM moving: resumed")
c = make()
run(c, 0.0, 5.0, moving, cmd=1.0)
run(c, 5.0, 8.0, frozen_at(5.0), cmd=1.0)
out = run(c, 8.0, 10.5, moving, motors=1000.0)                   # motors idle only 2.5 s
check(all(d for _, d in out), "lift motors idle < 3 s: still cut")

print("8. evidence with breaks")
c = make()
run(c, 0.0, 5.0, moving)
out = run(c, 5.0, 10.0, frozen_at(5.0), gyro=lambda t: 0.4 if (t * 10) % 5 < 3 else 0.0)   # 0.3 s on, 0.2 off
check(first_drop(out) is None, "gyro on 0.3 s / off 0.2 s never holds 0.4 s")

print("9. the smallest real FAST-LIO step is not a repeat")
c = make()
base = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
k = [0]


def creep(t):
    k[0] += 1
    return (base[0] + 7.5e-5 * k[0], 0.0, 1.0, 0.0, 0.0, 1.5e-5 * (k[0] % 2), 1.0)


out = run(c, 0.0, 5.0, creep, cmd=1.0, gyro=0.4)
check(first_drop(out) is None and c.check.repeats == 0, "0.075 mm steps: no repeats")

print()
if failures:
    print("FAILED: %d" % len(failures))
    sys.exit(1)
print("all passed")
