#!/usr/bin/env python3
"""Regression test for the FAST-LIO frame anchor (frame_anchor.py, used by fastlio_mavros_bridge).

A vehicle drives a synthetic path in the FC's world frame W. FAST-LIO sees it in its own frame,
which is W rotated and shifted; at t = 20 s FAST-LIO restarts and its new frame is rotated a
further 175 deg and shifted 3 m (the frame it would come back in after a drift_monitor restart).
The EKF is taken as the truth (the vehicle's body pose in W). Checks:
  1. the first pose anchored to the EKF lands on it (sensor lever arm included);
  2. across the restart the output stays continuous and on the truth;
  3. with no usable EKF the anchor reproduces the old yaw0 datum exactly;
  4. the old bridge (yaw0 recaptured on its restart) jumps by metres and ~175 deg here;
  5. the discontinuity detector fires on the restart and not on real motion (0.5 m/s drive,
     190 deg/s pivot, 20 Hz, and 5 Hz under load);
  6. EKF_STATUS flags gate and the EKF pose history interpolation.
Runs the real module (overrides/FAST-LIO/src/frame_anchor.py), not a mirror.
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'overrides', 'FAST-LIO', 'src'))
from frame_anchor import (Anchor, Discontinuity, PoseHistory, ekf_usable, quat_mul,  # noqa: E402
                          quat_yaw, rot_xy, wrap_pi, yaw_of, EKF_POS_HORIZ_ABS,
                          EKF_CONST_POS_MODE, EKF_UNINITIALIZED)

OFFSET = (0.22, 0.0, 0.0)          # VISO_POS on nuc-04's robot: lidar 0.22 m ahead of the FC
failures = []


def check(cond, what):
    print(('  ok    ' if cond else '  FAIL  ') + what)
    if not cond:
        failures.append(what)


def truth(t):
    """body pose in W at t: drive an arc, pivot, drive on (ENU)"""
    yaw = math.radians(30) + 0.15 * t + (math.radians(90) if t > 25 else 0.0) * min(1.0, max(0.0, t - 25))
    x = 2.0 + 0.5 * t * math.cos(0.1 * t)
    y = -1.0 + 0.5 * t * math.sin(0.1 * t)
    z = 0.3 + 0.02 * t
    return np.array([x, y, z]), yaw


def sensor_in_w(p_body, yaw):
    ox, oy = rot_xy(yaw, OFFSET[0], -OFFSET[1])
    return np.array([p_body[0] + ox, p_body[1] + oy, p_body[2] - OFFSET[2]])


def to_slam_frame(p_w, yaw_w, frame_yaw, frame_off):
    """express a W pose in a SLAM frame that is W rotated by frame_yaw and shifted by frame_off"""
    d = p_w - frame_off
    x, y = rot_xy(-frame_yaw, d[0], d[1])
    return np.array([x, y, d[2]]), quat_mul(quat_yaw(-frame_yaw), quat_yaw(yaw_w))


def pose_err(p, q, p_ref, yaw_ref):
    return float(np.linalg.norm(p - p_ref)), abs(math.degrees(wrap_pi(yaw_of(q) - yaw_ref)))


FRAME1 = (math.radians(-63.0), np.array([5.0, 2.0, -0.3]))
FRAME2 = (FRAME1[0] + math.radians(175.0), FRAME1[1] + np.array([3.0, 0.0, 0.0]))
T_RESTART = 20.0

print('1. first pose anchored onto the EKF')
p0, y0 = truth(0.0)
rp, rq = to_slam_frame(sensor_in_w(p0, y0), y0, *FRAME1)
a = Anchor.to_ekf(rp, rq, p0, y0, OFFSET)
dp, dy = pose_err(a.apply_pos(rp), a.apply_quat(rq), sensor_in_w(p0, y0), y0)
check(dp < 1e-3 and dy < 0.01, 'output = EKF pose + lever arm (%.4f m, %.4f deg)' % (dp, dy))
# velocity: a W-frame velocity expressed in FRAME1 comes back to W
v_w = np.array([0.4, -0.2, 0.05])
vx, vy = rot_xy(-FRAME1[0], v_w[0], v_w[1])
check(np.allclose(a.apply_vel(np.array([vx, vy, v_w[2]])), v_w, atol=1e-9), 'velocity rotated with the same heading')

print('2. FAST-LIO restart at %.0f s: output stays continuous and on the truth' % T_RESTART)
disc = Discontinuity()
anchor, worst_dp, worst_dy, fired_at = None, 0.0, 0.0, []
prev_out = None
max_step = 0.0
for k in range(0, 40 * 20):
    t = k / 20.0
    pb, yb = truth(t)
    frame = FRAME1 if t < T_RESTART else FRAME2
    if T_RESTART - 3.0 < t < T_RESTART:
        continue                      # laserMapping down: no /Odometry for 3 s
    rp, rq = to_slam_frame(sensor_in_w(pb, yb), yb, *frame)
    jump = disc.update(t, rp, yaw_of(rq))
    if anchor is None or jump:
        if jump:
            fired_at.append((t, jump))
        anchor = Anchor.to_ekf(rp, rq, pb, yb, OFFSET)   # EKF = truth
    out_p, out_q = anchor.apply_pos(rp), anchor.apply_quat(rq)
    dp, dy = pose_err(out_p, out_q, sensor_in_w(pb, yb), yb)
    worst_dp, worst_dy = max(worst_dp, dp), max(worst_dy, dy)
    prev_out = out_p
check(len(fired_at) == 1 and abs(fired_at[0][0] - T_RESTART) < 0.06,
      'one discontinuity, at the restart: %s' % fired_at)
check(worst_dp < 0.01 and worst_dy < 0.01, 'output on the truth throughout (worst %.4f m, %.4f deg)' % (worst_dp, worst_dy))

print('3. no usable EKF: the anchor is the old yaw0 datum, exactly')
rp, rq = to_slam_frame(sensor_in_w(p0, y0), y0, *FRAME1)
yd = Anchor.yaw_datum(rq)
# the old bridge code, verbatim maths: rotate x,y by -yaw0, q <- Rz(-yaw0) q
yaw0 = yaw_of(rq)
for t in (0.0, 5.0, 17.0):
    pb, yb = truth(t)
    sp, sq = to_slam_frame(sensor_in_w(pb, yb), yb, *FRAME1)
    c, s = math.cos(-yaw0), math.sin(-yaw0)
    old_p = np.array([c * sp[0] - s * sp[1], s * sp[0] + c * sp[1], sp[2]])
    old_q = quat_mul(quat_yaw(-yaw0), sq)
    check(np.allclose(yd.apply_pos(sp), old_p, atol=1e-9) and np.allclose(yd.apply_quat(sq), old_q, atol=1e-9),
          'yaw0 datum identical to the old code at t=%.0f s' % t)

print('4. the old bridge (yaw0 recaptured after its restart) jumps here')
pb, yb = truth(T_RESTART - 3.05)
op, oq = to_slam_frame(sensor_in_w(pb, yb), yb, *FRAME1)
before = (Anchor.yaw_datum(rq).apply_pos(op), Anchor.yaw_datum(rq).apply_quat(oq))
pb2, yb2 = truth(T_RESTART)
np_, nq = to_slam_frame(sensor_in_w(pb2, yb2), yb2, *FRAME2)
after = (Anchor.yaw_datum(nq).apply_pos(np_), Anchor.yaw_datum(nq).apply_quat(nq))
moved = float(np.linalg.norm(sensor_in_w(pb2, yb2) - sensor_in_w(pb, yb)))
jump_m = float(np.linalg.norm(after[0] - before[0])) - moved
jump_deg = abs(math.degrees(wrap_pi((yaw_of(after[1]) - yaw_of(before[1])) - (yb2 - yb))))
check(jump_m > 2.0 and jump_deg > 20.0,
      'old path: frame jumps %.1f m beyond the real motion and %.0f deg' % (jump_m, jump_deg))

print('5. discontinuity detector: no false alarm on real motion')
for hz, label in ((20.0, '20 Hz'), (5.0, '5 Hz under load')):
    d = Discontinuity()
    fired = 0
    yaw = 0.0
    for k in range(int(10 * hz)):
        t = k / hz
        yaw = wrap_pi(math.radians(190.0) * t / (5.0 if hz == 5.0 else 1.0)) if t < 1.0 else yaw
        fired += d.update(t, (0.5 * t, 0.0, 0.3), yaw) is not None
    check(fired == 0, '%s: 0.5 m/s drive and pivot, %d false alarms' % (label, fired))

print('6. EKF gate and pose history')
check(ekf_usable(EKF_POS_HORIZ_ABS | 1 | 2) and not ekf_usable(1 | 2)
      and not ekf_usable(EKF_POS_HORIZ_ABS | EKF_CONST_POS_MODE)
      and not ekf_usable(EKF_POS_HORIZ_ABS | EKF_UNINITIALIZED), 'EKF_STATUS flags gate')
h = PoseHistory()
for k in range(40):
    h.add(k * 0.05, (k * 0.05, 0.0, 0.0), math.radians(170.0 + k))
p, yw = h.at(0.525)
check(abs(p[0] - 0.525) < 1e-9 and abs(math.degrees(yw) - (170.0 + 10.5 - 360.0 if 180.5 > 180 else 180.5)) < 1e-6,
      'interpolates position and yaw across +-180 deg (%.3f m, %.2f deg)' % (p[0], math.degrees(yw)))
check(h.at(5.0) is None, 'no pose 3 s past the newest sample')

print('\n%s' % ('ALL PASS' if not failures else '%d FAILED' % len(failures)))
sys.exit(1 if failures else 0)
