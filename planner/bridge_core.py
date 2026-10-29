"""Planner bridge core: frames, the frame anchor, caps, the geofence and the HOLD state machine.

ROS-free (numpy only) so the unit tests run this exact code; planner_bridge.py wraps it in rospy.

Frames (L4 plan, D7):
  raw      FAST-LIO's world frame: +x left, +y up, +z forward (raw /Odometry, /cloud_registered).
  planner  raw rotated z-up by POSITION_MATRIX, the transform fastlio_mavros_bridge.py applies
           before its frame anchor. SUPER and ROG-Map work here.
  fc       MAVROS local ENU ("map"): the frame of /mavros/vision_pose/pose, i.e. the planner frame
           after the bridge's frame anchor  fc = Rz(psi) * planner + t  (frame_anchor.Anchor).
Quaternions are [x, y, z, w]; angles in radians.
"""
import math

import numpy as np

# raw -> planner, the default ~position_matrix / ~orientation_matrix of fastlio_mavros_bridge.py
POSITION_MATRIX = np.array([[0.0, 0.0, 1.0],
                            [1.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0]])


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def quat_from_matrix(m):
    """rotation matrix -> [x, y, z, w]"""
    m = np.asarray(m, dtype=float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        return (0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s)
    if m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        return ((m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s)
    s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
    return ((m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s)


Q_PERM = quat_from_matrix(POSITION_MATRIX)


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_yaw(psi):
    return (0.0, 0.0, math.sin(psi / 2.0), math.cos(psi / 2.0))


# ---- raw -> planner (the bridge's transform before its anchor) ---------------------------------

def raw_to_planner_pos(p):
    return POSITION_MATRIX.dot(np.asarray(p, dtype=float))


def raw_to_planner_quat(q):
    """the bridge's similarity transform q_enu = qR * q * qR^-1"""
    return quat_mul(quat_mul(Q_PERM, tuple(q)), quat_conj(Q_PERM))


def raw_to_planner_points(xyz):
    """N x 3 raw points -> planner frame"""
    return np.asarray(xyz, dtype=float).dot(POSITION_MATRIX.T)


# ---- the anchor: fc <- planner ------------------------------------------------------------------

class Anchor:
    """fc = Rz(psi) * planner + t, the bridge's frame anchor recovered from its output"""

    def __init__(self, psi, t):
        self.psi = float(psi)
        self.t = np.asarray(t, dtype=float)

    @classmethod
    def from_pair(cls, p_planner, q_planner, p_fc, q_fc, mount_yaw=0.0):
        """the anchor that maps one planner pose onto the bridge's vision pose made from it.
        mount_yaw: the bridge's ~fc_mount_yaw_deg in radians (a body-side yaw on the orientation
        only, before the anchor), so psi is not biased by it"""
        # the bridge applies the mount body-side (q * Rz(-mount)) before the anchor's left Rz(psi);
        # a left yaw adds to yaw exactly, so this is exact even when the robot is tilted
        q_m = quat_mul(tuple(q_planner), quat_yaw(-mount_yaw))
        psi = wrap_pi(yaw_of(q_fc) - yaw_of(q_m))
        c, s = math.cos(psi), math.sin(psi)
        p = np.asarray(p_planner, dtype=float)
        rp = np.array([c * p[0] - s * p[1], s * p[0] + c * p[1], p[2]])
        return cls(psi, np.asarray(p_fc, dtype=float) - rp)

    def pos(self, p):
        c, s = math.cos(self.psi), math.sin(self.psi)
        return np.array([c * p[0] - s * p[1] + self.t[0], s * p[0] + c * p[1] + self.t[1], p[2] + self.t[2]])

    def vec(self, v):
        c, s = math.cos(self.psi), math.sin(self.psi)
        return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]])

    def yaw(self, yaw):
        return wrap_pi(yaw + self.psi)

    def inverse_pos(self, p):
        c, s = math.cos(-self.psi), math.sin(-self.psi)
        q = np.asarray(p, dtype=float) - self.t
        return np.array([c * q[0] - s * q[1], s * q[0] + c * q[1], q[2]])

    def differs(self, other, jump_m, jump_rad):
        """-> the change between two anchors as a reason, or None"""
        dpsi = abs(wrap_pi(self.psi - other.psi))
        dt = float(np.linalg.norm(self.t - other.t))
        if dpsi > jump_rad:
            return "anchor yaw changed %.1f deg" % math.degrees(dpsi)
        if dt > jump_m:
            return "anchor moved %.2f m" % dt
        return None


class AnchorTracker:
    """the latest anchor from (planner, vision) pose pairs; a change that persists for
    `persist` pairs is a re-anchor or SLAM reset (one odd pair is a pairing glitch)"""

    def __init__(self, jump_m=0.1, jump_deg=2.0, persist=3, mount_yaw=0.0):
        self.jump_m = float(jump_m)
        self.jump_rad = math.radians(float(jump_deg))
        self.persist = int(persist)
        self.mount_yaw = float(mount_yaw)
        self.anchor = None
        self._candidate = []
        self.jumps = 0

    def update(self, p_planner, q_planner, p_fc, q_fc):
        """-> (anchor, jump_reason or None)"""
        a = Anchor.from_pair(p_planner, q_planner, p_fc, q_fc, self.mount_yaw)
        if self.anchor is None:
            self.anchor = a
            return a, None
        why = a.differs(self.anchor, self.jump_m, self.jump_rad)
        if why is None:
            # small changes are noise: follow them slowly so the anchor stays exact
            self.anchor = Anchor(wrap_pi(self.anchor.psi + 0.1 * wrap_pi(a.psi - self.anchor.psi)),
                                 0.9 * self.anchor.t + 0.1 * a.t)
            self._candidate = []
            return self.anchor, None
        self._candidate.append(a)
        if len(self._candidate) < self.persist:
            return self.anchor, None
        # it persisted: the frame really changed
        consistent = all(c.differs(self._candidate[-1], self.jump_m, self.jump_rad) is None for c in self._candidate)
        self._candidate = []
        if not consistent:
            return self.anchor, None
        self.anchor = a
        self.jumps += 1
        return a, why


# ---- the output target --------------------------------------------------------------------------

def cap_command(p, v, a, p_now, speed_cap, max_lead, fc_pos_p=0.0):
    """scale velocity (and acceleration with it) to the speed cap, and keep the position target
    within max_lead of the current position so a capped vehicle is not dragged after a target
    that runs ahead.

    With fc_pos_p (the FC's horizontal position gain, Q_P_NE_POS_P) the cap bounds what the FC
    actually flies: its velocity demand is the feed-forward plus fc_pos_p times the position error,
    so a vehicle lagging SUPER's reference was demanded up to cap + fc_pos_p * max_lead (1.6 m/s
    flown at a 1.0 cap in the L2 closed loop). The position correction is kept, since it brings the
    vehicle back onto the planned path, and the feed-forward is scaled down until
    |v + fc_pos_p * lead| <= speed_cap."""
    p, v, a, p_now = (np.asarray(x, dtype=float) for x in (p, v, a, p_now))
    speed = float(np.linalg.norm(v))
    capped = False
    if speed > speed_cap > 0:
        k = speed_cap / speed
        v, a = v * k, a * k
        capped = True
    lead = p - p_now
    d = float(np.linalg.norm(lead))
    if d > max_lead > 0:
        lead = lead * (max_lead / d)
        p = p_now + lead
        d = max_lead
        capped = True
    if fc_pos_p > 0 and speed_cap > 0:
        corr = fc_pos_p * lead
        if fc_pos_p * d >= speed_cap:
            # the correction alone reaches the cap: no feed-forward, a shorter lead
            p = p_now + lead * (speed_cap / (fc_pos_p * d))
            v, a = np.zeros(3), np.zeros(3)
            capped = True
        elif float(np.linalg.norm(v + corr)) > speed_cap:
            # the largest k in [0, 1] with |k v + corr| = speed_cap
            qa, qb, qc = float(v @ v), 2.0 * float(v @ corr), float(corr @ corr) - speed_cap ** 2
            k = (-qb + math.sqrt(qb * qb - 4 * qa * qc)) / (2 * qa)
            v, a = v * k, a * k
            capped = True
    return p, v, a, capped


def drive_target(p_cmd, v_cmd, p_now, lookahead, horizon_s=0.5):
    """a position-only driving target at most `lookahead` ahead of the vehicle, towards where the
    command is heading (D4: a dead planner then stops the rover within the lookahead)"""
    p_cmd, v_cmd, p_now = (np.asarray(x, dtype=float) for x in (p_cmd, v_cmd, p_now))
    aim = p_cmd + v_cmd * horizon_s
    d = aim - p_now
    d[2] = 0.0
    n = float(np.linalg.norm(d))
    if n < 1e-6:
        return np.array([p_now[0], p_now[1], p_now[2]])
    step = min(lookahead, n)
    out = p_now + d / n * step
    out[2] = p_now[2]
    return out


def reverse_speed(p_cmd, v_cmd, yaw, p_now, gain=1.0, stop_tol=0.02):
    """a reverse from the track planner (owner 2026-10-28, dead ends): a command whose velocity points
    against its yaw means "back straight up, holding yaw, until the vehicle (the FC) is at p_cmd".
    -> the speed to back up at (>= 0: |v_cmd| at most, tapering at `gain` per metre left, 0 within
    stop_tol of the end or past it), or None for an ordinary (forward) command. Rover Guided takes it
    as a velocity + heading target with a negative speed (GCS_MAVLink_Rover: speed_dir from vx)."""
    v = np.asarray(v_cmd, dtype=float)[:2]
    sp = float(np.linalg.norm(v))
    h = np.array([math.cos(yaw), math.sin(yaw)])
    if sp < 1e-3 or float(v @ h) >= 0.0:
        return None
    left = float((np.asarray(p_now, dtype=float)[:2] - np.asarray(p_cmd, dtype=float)[:2]) @ h)
    if left <= stop_tol:
        return 0.0
    return min(sp, gain * left)


class Geofence:
    def __init__(self, lo, hi):
        self.lo = np.asarray(lo, dtype=float)
        self.hi = np.asarray(hi, dtype=float)

    def inside(self, p):
        p = np.asarray(p, dtype=float)
        return bool(np.all(p >= self.lo) and np.all(p <= self.hi))

    def clamp(self, p):
        return np.minimum(np.maximum(np.asarray(p, dtype=float), self.lo), self.hi)


# ---- the FC pose's freshness --------------------------------------------------------------------

class FcPoseGuard:
    """the FC's EKF pose (/mavros/local_position/pose, ~50 Hz on the robot) is the bridge's p_now
    (cap, drive target) and its HOLD point. It is trusted while the newest message is younger than
    `timeout` (its receive time). Stale: the bridge holds at `good`, the last pose received while it
    was trusted. Fresh again: trusted after `resume_n` poses in a row, each within `timeout` of the
    one before (a trickle of single poses stays stale).

    check(now) -> "never" (none yet: stream nothing), "stale" or "ok"."""

    def __init__(self, timeout=0.3, resume_n=5):
        self.timeout = float(timeout)
        self.resume_n = max(1, int(resume_n))
        self.pose = None        # the newest pose, whatever its age
        self.t = None           # its receive time
        self.good = None        # the last pose received while trusted
        self.stale = False
        self.fresh_n = 0

    def on_pose(self, now, pose):
        if self.t is not None and now - self.t > self.timeout:
            self.stale = True   # a gap the check did not see yet: the pose before it stays the good one
            self.fresh_n = 0
        self.pose, self.t = pose, now
        if self.stale:
            self.fresh_n += 1
            if self.fresh_n >= self.resume_n:
                self.stale = False
        if not self.stale:
            self.good = pose

    def age(self, now):
        return None if self.t is None else now - self.t

    def check(self, now):
        if self.t is None:
            return "never"
        if now - self.t > self.timeout:
            self.stale = True
            self.fresh_n = 0
        return "stale" if self.stale else "ok"


# ---- HOLD ---------------------------------------------------------------------------------------

class HoldLogic:
    """STREAM while every input is fresh and healthy; HOLD (zero velocity at the current position)
    on the first fault. Leaving HOLD needs the cause cleared AND a planner command newer than the
    moment the hold began, so a stale trajectory is never resumed."""

    def __init__(self, planner_timeout=0.3, slam_timeout=0.2):
        self.planner_timeout = float(planner_timeout)
        self.slam_timeout = float(slam_timeout)
        self.state = "WAIT"         # WAIT (nothing yet), STREAM, HOLD
        self.reason = "no planner command yet"
        self.hold_since = None
        self.latched = None         # a latched cause (anchor jump) until a fresh command

    def latch(self, reason, now):
        self.latched = reason
        self._hold(reason, now)

    def _hold(self, reason, now):
        if self.state != "HOLD":
            self.hold_since = now
        self.state = "HOLD"
        self.reason = reason

    def step(self, now, t_cmd, t_slam, faults=()):
        """faults: reasons from the caller (geofence, SLAM health). -> state"""
        causes = list(faults)
        if t_slam is None or now - t_slam > self.slam_timeout:
            causes.append("SLAM stale %.2f s" % (now - t_slam) if t_slam is not None else "no SLAM odometry")
        if t_cmd is None:
            if self.state == "WAIT" and not causes:
                return self.state
            causes.append("no planner command")
        elif now - t_cmd > self.planner_timeout:
            causes.append("planner stale %.2f s" % (now - t_cmd))
        if self.latched is not None:
            if t_cmd is not None and self.hold_since is not None and t_cmd > self.hold_since and not causes:
                self.latched = None
            else:
                causes.insert(0, self.latched)     # a re-anchor is the reason to report first
        if causes:
            self._hold(causes[0], now)
            return self.state
        if self.state == "HOLD" and (t_cmd is None or self.hold_since is None or t_cmd <= self.hold_since):
            self.reason = "waiting for a fresh planner command"
            return self.state
        self.state = "STREAM"
        self.reason = ""
        self.hold_since = None
        return self.state


# ---- SLAM health (pluggable, off unless configured and its topic is seen) -----------------------

class HealthCheck:
    """FAST-LIO health (/fastlio_health: data[0] matched points, data[4] observability eigenvalue,
    data[7] latency ms) and a height cross-check (SLAM height vs a downward rangefinder, the log 233
    case: a consistent but wrong SLAM height the FC alone does not catch)"""

    def __init__(self, matched_min=150.0, eig_min=5000.0, latency_max_ms=250.0, height_tol=0.5, max_age=0.5):
        self.matched_min = matched_min
        self.eig_min = eig_min
        self.latency_max_ms = latency_max_ms
        self.height_tol = height_tol
        self.max_age = max_age
        self.health = None          # (t, matched, eig, latency)
        self.range = None           # (t, range m)
        self.height_offset = None   # fc z minus range, learnt when the vehicle is low and level

    def on_health(self, t, data):
        if len(data) > 7:
            self.health = (t, data[0], data[4], data[7])

    def on_range(self, t, r, fc_z):
        if not (0.05 < r < 20.0):
            return
        self.range = (t, r, fc_z)
        if self.height_offset is None:
            self.height_offset = fc_z - r

    def faults(self, now):
        out = []
        if self.health is not None and now - self.health[0] < self.max_age:
            _, matched, eig, lat = self.health
            if matched < self.matched_min:
                out.append("SLAM matched points %d" % matched)
            if eig < self.eig_min:
                out.append("SLAM observability %.0f" % eig)
            if lat > self.latency_max_ms:
                out.append("SLAM latency %.0f ms" % lat)
        if self.range is not None and self.height_offset is not None and now - self.range[0] < self.max_age:
            err = (self.range[2] - self.range[1]) - self.height_offset
            if abs(err) > self.height_tol:
                out.append("SLAM height disagrees with the rangefinder by %.2f m" % err)
        return out
