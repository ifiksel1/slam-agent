#!/usr/bin/env python3
"""Frame anchor for fastlio_mavros_bridge: keeps the frame sent to the FC continuous.

WHY. The bridge used to zero the heading at the first pose it saw (yaw0) and pass FAST-LIO's
origin through. Every restart - drift_monitor kills laserMapping and the bridge, the health
watchdog respawns laserMapping alone, or the bridge crashes on its own - starts FAST-LIO at its
own origin and the bridge at a new yaw0, so the FC is handed a frame rotated by the heading and
shifted by the position travelled since the first start. ArduPilot cannot absorb that:
  - EKF3 resets POSITION only when VISION_POSITION_ESTIMATE.reset_counter changes
    (AP_NavEKF3_PosVelFusion.cpp:634-641), and MAVROS 1.20 always sends 0;
  - it has no HEADING reset for an ExternalNav frame change: once in flight the new heading fails
    the yaw innovation gate for good (AP_NavEKF3_MagFusion.cpp:1110-1119), and the position is
    later reset onto the rotated frame while the heading stays wrong.
2026-10-19 (nuc-04): after lidar outages the EKF rejected SLAM (test ratios 12-13) until an FC
reboot.

FIX. On a (re)start, and on any FAST-LIO discontinuity, choose the output transform so the new
pose lands exactly on the FC's current EKF pose: the FC then sees one continuous frame and has
nothing to reset. With no usable EKF (first boot) the old yaw0 datum is kept.

ROS-free (numpy only) so scripts/replay/test_frame_anchor.py runs this exact code.
Quaternions are [x, y, z, w] (tf.transformations order); all frames are ENU.
"""
import math

import numpy as np

# EKF_STATUS_REPORT.flags (MAVLink ESTIMATOR_STATUS_FLAGS)
EKF_POS_HORIZ_REL = 8
EKF_POS_HORIZ_ABS = 16
EKF_CONST_POS_MODE = 128
EKF_UNINITIALIZED = 1024


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    """ENU yaw (rad) of quaternion [x, y, z, w]"""
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_yaw(psi):
    """rotation about vertical by psi (rad), as [x, y, z, w]"""
    return (0.0, 0.0, math.sin(psi / 2.0), math.cos(psi / 2.0))


def quat_mul(a, b):
    """Hamilton product a*b, both [x, y, z, w]"""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def rot_xy(psi, x, y):
    c, s = math.cos(psi), math.sin(psi)
    return c * x - s * y, s * x + c * y


def ekf_usable(flags):
    """EKF_STATUS_REPORT flags: a horizontal position, not constant-position mode, initialised"""
    return bool(flags & (EKF_POS_HORIZ_ABS | EKF_POS_HORIZ_REL)) and \
        not (flags & (EKF_CONST_POS_MODE | EKF_UNINITIALIZED))


class Anchor:
    """out = Rz(psi) * in + t for position, Rz(psi) * q for orientation, Rz(psi) * v for velocity.
    The old yaw0 datum is psi = -yaw0, t = 0."""

    def __init__(self, psi=0.0, t=(0.0, 0.0, 0.0), source='identity'):
        self.psi = float(psi)
        self.t = np.array(t, dtype=float)
        self.source = source
        self._q = quat_yaw(self.psi)

    @classmethod
    def yaw_datum(cls, q_enu):
        """the old behaviour: heading zeroed at this pose, FAST-LIO's origin kept"""
        return cls(-yaw_of(q_enu), (0.0, 0.0, 0.0), 'yaw0')

    @classmethod
    def to_ekf(cls, p_enu, q_enu, ekf_p, ekf_yaw, offset_body):
        """the anchor that maps this raw pose onto the EKF pose.

        p_enu, q_enu: the bridge's pose (sensor position, FC-body attitude) in FAST-LIO's ENU frame.
        ekf_p, ekf_yaw: the EKF's position (FC body origin) and ENU yaw at the same instant.
        offset_body: VISO_POS_X/Y/Z, the sensor position in the FC body frame (forward, right,
        down; ArduPilot's convention). The vision pose is the sensor's, the EKF pose the body's,
        so the sensor is expected at ekf_p + R(yaw) * offset (roll and pitch ignored: < 1 cm at
        10 deg for a 0.22 m offset)."""
        psi = wrap_pi(ekf_yaw - yaw_of(q_enu))
        fx, fy, fz = offset_body
        ox, oy = rot_xy(ekf_yaw, fx, -fy)            # body FRD -> ENU, by heading
        sensor = np.array([ekf_p[0] + ox, ekf_p[1] + oy, ekf_p[2] - fz])
        rx, ry = rot_xy(psi, p_enu[0], p_enu[1])
        return cls(psi, sensor - np.array([rx, ry, p_enu[2]]), 'ekf')

    def apply_pos(self, p):
        x, y = rot_xy(self.psi, p[0], p[1])
        return np.array([x + self.t[0], y + self.t[1], p[2] + self.t[2]])

    def apply_quat(self, q):
        return quat_mul(self._q, q)

    def apply_vel(self, v):
        x, y = rot_xy(self.psi, v[0], v[1])
        return np.array([x, y, v[2]])

    def describe(self):
        return 'psi %.1f deg, t (%.2f, %.2f, %.2f) m [%s]' % (
            math.degrees(self.psi), self.t[0], self.t[1], self.t[2], self.source)


class Discontinuity:
    """flags a FAST-LIO frame jump in the raw (pre-anchor) pose stream: a gap (laserMapping
    restarting takes several seconds) or a step no robot makes in one scan (a fresh laserMapping
    starts at its own origin, identity orientation)"""

    def __init__(self, gap_s=1.0, jump_m=0.5, jump_deg=20.0):
        self.gap_s = float(gap_s)
        self.jump_m = float(jump_m)
        self.jump_rad = math.radians(float(jump_deg))
        self._last = None      # (stamp_s, p, yaw)

    def update(self, stamp_s, p, yaw):
        """-> None, or the reason this sample starts a new frame"""
        last, self._last = self._last, (stamp_s, np.array(p, dtype=float), yaw)
        if last is None:
            return None
        dt = stamp_s - last[0]
        if dt > self.gap_s or dt < 0.0:
            return 'gap %.1f s' % dt
        step = float(np.linalg.norm(self._last[1] - last[1]))
        if step > self.jump_m:
            return 'step %.2f m' % step
        dyaw = abs(wrap_pi(yaw - last[2]))
        if dyaw > self.jump_rad:
            return 'step %.0f deg' % math.degrees(dyaw)
        return None


class PoseHistory:
    """the EKF pose over the last few seconds, to anchor at the odometry's own stamp (the vision
    pose is VISO_DELAY_MS old; the EKF pose for 'now' would be off by speed x delay)"""

    def __init__(self, span_s=2.0):
        self.span_s = float(span_s)
        self._h = []           # (stamp_s, p ndarray, yaw)

    def add(self, stamp_s, p, yaw):
        self._h.append((stamp_s, np.array(p, dtype=float), yaw))
        while self._h and self._h[0][0] < stamp_s - self.span_s:
            self._h.pop(0)

    def at(self, stamp_s, max_age_s=0.5):
        """-> (p, yaw) interpolated at stamp_s; the nearest end if outside; None if none is
        within max_age_s"""
        if not self._h:
            return None
        if stamp_s <= self._h[0][0]:
            s = self._h[0]
            return (s[1], s[2]) if self._h[0][0] - stamp_s <= max_age_s else None
        if stamp_s >= self._h[-1][0]:
            s = self._h[-1]
            return (s[1], s[2]) if stamp_s - s[0] <= max_age_s else None
        for (t0, p0, y0), (t1, p1, y1) in zip(self._h, self._h[1:]):
            if t0 <= stamp_s <= t1:
                f = 0.0 if t1 == t0 else (stamp_s - t0) / (t1 - t0)
                return p0 + f * (p1 - p0), wrap_pi(y0 + f * wrap_pi(y1 - y0))
        return None
