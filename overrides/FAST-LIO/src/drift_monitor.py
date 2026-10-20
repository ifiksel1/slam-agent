#!/usr/bin/env python3
"""
drift_monitor — FAST-LIO drift early-warning + EKF source-failover/restart response.

Subscribes /fastlio_health and flags trouble BEFORE the trajectory diverges, on three
dimensions:
  (1) observability   eig_min = data[4]      lower bound
  (2) feature count   matched_pts = data[0]  lower bound
  (3) LATENCY         age-of-measurement     UPPER bound   [added 2026-09-01]
plus three added 2026-10-07 after the 26 Sep crash (FC log 218), each OBSERVE-ONLY by default
(WARN + GCS banner; its CRITICAL is reported as "(obs)" until its enable_*_critical gate is set):
  (4) RESIDUAL        mean point-to-plane residual = data[1], UPPER bound, abs OR x baseline
  (5) EKF INNOVATION  the FC's own position test ratio (EKF_STATUS_REPORT.pos_horiz_variance,
                      = XKF4.SP in the dataflash log) off the raw /mavlink/from stream
  (6) MOTION vs RC    the vehicle moving in a way the sticks did not ask for (velocity error
                      vs the Loiter stick command, vertical motion against the throttle stick,
                      tilt beyond the Loiter lean limit)
  ~bench_mode (runtime rosparam, default false) runs (5)/(6) DISARMED for hand-carried bench
  tests and forces (4)-(6) observe-only; ignored once armed. /slam/drift_risk shows "BENCH".
WHY (4)-(6) EXIST. On 26 Sep 2026 (FC log 218, bag 09_27_2026_00_28_47) FAST-LIO's residual rose
0.019 -> 0.04 -> 0.136 from t=337.8 while the airframe was level, the EKF's SP crossed 1.0 at
339.66 and it stopped fusing vision, Loiter chased an IMU-only position to 25 deg pitch / 40 deg
roll on full throttle, and the vehicle hit at ~341.8. This node said OK throughout: eig_min never
fell below 97k and matched below 703 - both far above their floors. Residual and SP were the two
signals that moved ~2 s before the EKF let go.

FAILOVER + node-restart is triggered by ANY of:
  (A) drift CRITICAL   (eig_min/matched cross the hard floors, K-scan hysteresis), or
  (B) latency CRITICAL (age stays over lat_crit_ms for lat_k_crit scans), or
  (C) pilot CH9 switch -> ~1500 band (3-pos middle, EDGE-triggered once on entry).
All run the SAME action: instead of LANDing, lane-change the FC EKF source from SLAM
(SRC1) to Optical Flow (SRC2) so the vehicle keeps flying on OF while SLAM re-initializes,
then restart laserMapping (gated ~enable_auto_restart) + advisory /slam/arm_interlock=LOCKED
+ GCS banners. On recovery (fresh SLAM healthy for k_release scans) lane-change back SRC2->SRC1.
Source switch = MAV_CMD_DO_AUX_FUNCTION(218), func 90 (EKF Source Set): pos MIDDLE(1)->SRC2,
LOW(0)->SRC1. Gated ~enable_source_failover. Does NOT arm or change flight mode.

WHY LATENCY EXISTS (2026-09-01). On 25 Aug 2026 FAST-LIO's publish latency ramped 82 -> 669 ms
under a real-time overrun (matched 4709 -> 7271 pushed per-scan cost past the 50 ms budget at
20 Hz). ArduPilot is configured VISO_DELAY_MS=75 and hard-caps vision-delay compensation at
250 ms, so the EKF fused stale poses -> 3.7 g hard landing and an uncommanded re-liftoff. This
node published OK throughout, correctly: every test it had was a LOWER bound on geometry, and
matched_pts was RISING, which reads as healthier. A fleet sweep of 22 flights found 21 flat at
67-78 ms, so this is a LOAD THRESHOLD, not a regression. Latency is the one metric that would
have predicted it: it passed 200 ms about 10 s before the EKF failsafe and 15 s before impact.

LATENCY RESPONSE IS TWO-STAGE, and CRITICAL is itself staged:
  >= lat_warn_ms (150)  for lat_k_warn scans  -> WARN: banner + interlock, NO action.
  >= lat_crit_ms (250)  for lat_k_crit scans  -> CRITICAL: lane-change to OF (SRC2) ONLY.
  still >= lat_crit_ms after lat_drain_sec (5) on OF -> escalate to the full node restart.
  still >= lat_crit_ms once the restart blank expires -> command LAND (~enable_latency_land,
  default OFF). Safe at that point and only at that point: we have been stable on SRC2 for ~25 s,
  so EKF height is the rangefinder (EK3_SRC2_POSZ=2), and that sensor is known alive because the
  aircraft is flying on its optical flow. LAND's demand and its control loop then share one
  signal. NOTE the MTF-01 supplies BOTH the flow and the rangefinder, so it is a single device:
  if it dies, SRC2 loses velocity and height together and LAND would descend on the barometer.
  Nothing currently detects that - see the flow-health gap in the plan.
250 ms is not a tuning choice: it is ArduPilot's hard cap on vision-delay compensation, above
which the EKF is provably fusing uncompensated stale poses whatever VISO_DELAY_MS says.
FAST-LIO runs ~2.5x realtime at normal load, so a transient backlog often drains on its own;
a restart costs a fresh cold start, itself a 13.6 s stale window (observed). Hence OF first.

LATENCY SOURCE. Preferred: /fastlio_health data[7], published by laserMapping in the same loop
iteration (immune to our own callback blocking). Fallback: /Odometry, whose header.stamp is
lidar_end_time on the host clock, so now - stamp is a true age-of-measurement. The fallback is
NOT vestigial — every recorded bag predates data[7], so it is what keeps the 24-bag library
usable as a replay corpus.

ARMING BLOCK is enforced by the FLIGHT CONTROLLER's own EKF prearm (vision drops while
laserMapping restarts -> EKF unhealthy -> FC refuses arm; clears when EKF healthy again).
This node does NOT write FC params; /slam/arm_interlock is advisory/telemetry only. NOTE the
gap that leaves: vision that is PRESENT but STALE does not fail EKF prearm, which is how the
vehicle armed and took off on ~2 s old position during a cold-start backlog (log 50). The
latency interlock below raises the banner; hard enforcement needs the separate Lua aux-auth
gate (plan §3).

SAFETY: this node NEVER arms and NEVER changes flight mode. An EKF source switch while
disarmed/grounded is benign (no motion). It replaces the previous LAND response entirely.
"""
import collections
import math
import struct
import subprocess
import threading
import time
import rospy
from std_msgs.msg import Float32MultiArray, String

try:
    from nav_msgs.msg import Odometry
    _HAVE_ODOM = True
except Exception:
    _HAVE_ODOM = False

try:
    from mavros_msgs.msg import StatusText, RCIn, State, DebugValue, Mavlink, VFR_HUD
    from mavros_msgs.srv import SetMode, CommandLong, StreamRate
    _HAVE_MAVROS = True
except Exception:
    _HAVE_MAVROS = False

try:
    from geometry_msgs.msg import TwistStamped, Vector3Stamped
    from sensor_msgs.msg import Imu
    _HAVE_MOTION_MSGS = True
except Exception:
    _HAVE_MOTION_MSGS = False

OK, WARN, CRITICAL = "OK", "WARN", "CRITICAL"
_LEVEL = {OK: 0, WARN: 1, CRITICAL: 2}
_EKF_STATUS_REPORT_MSGID = 193
_HEARTBEAT_MSGID = 0
_MAV_TYPE_GROUND_ROVER = 10     # MAV_TYPE of a Rover-based FC: QuadRover (copter flight on Rover)


def land_command(fc_mav_type, land_mode, land_mode_rover):
    """the FC request restart_mode="land" makes: ('custom', n) = MAV_CMD_DO_SET_MODE with custom
    mode n for a Rover-based FC (QuadRover: QRLand = 21; MAVROS has no names for the QR modes and
    its set_mode refuses "CMODE(n)", and "LAND" does not exist on Rover); otherwise ('name',
    land_mode) through /mavros/set_mode (Copter: "LAND"). An FC not yet seen (no HEARTBEAT) gets
    the name, which a Rover-based FC simply rejects."""
    if fc_mav_type == _MAV_TYPE_GROUND_ROVER and int(land_mode_rover) >= 0:
        return ('custom', int(land_mode_rover))
    return ('name', land_mode)
# Reported to the FC when latency is enabled but no number can be produced (no /Odometry, no
# health, rejected samples). Large enough to trip any sane gate: unmeasurable is treated as bad,
# matching _lat_arm_bad(), which also calls lat_ms=None bad. Note this is NOT how the FC-side gate
# handles our SILENCE -- that fails open by design, because a stuck gate strands the aircraft.
_LAT_UNKNOWN_MS = 9999.0
_SEV_WARNING, _SEV_CRITICAL = 4, 2  # MAV_SEVERITY


def mean(seq):
    # rolling average baseline (over OK scans)
    return (sum(seq) / len(seq)) if seq else 0.0


class LatencyTracker:
    """Age-of-measurement tracker.

    Deliberately pure: it never reads a clock itself, every method takes now_sec. That is
    what lets scripts/replay/latency_replay.py drive the EXACT logic that flies using bag
    timestamps, on bags recorded long before the feature existed.

    Two properties worth knowing:
      * median, not mean — one 13.6 s cold-start sample poisons a mean for a whole window,
        while the median discards it after 3 scans. With sd ~4.5 ms against a 150 ms bar the
        median is otherwise indistinguishable from the instantaneous value.
      * the starvation floor. If samples stop arriving the freshest measurement keeps ageing,
        so effective age = max(median, last_age + time_since_last). A total vision dropout
        therefore falls out of the same metric with correct semantics, and the median can
        never mask a stall.
    """

    def __init__(self, median_n=5, grace_sec=5.0, nominal_hz=20.0,
                 sane_max_ms=30000.0, max_skew_ms=50.0):
        self.median_n = max(1, int(median_n))
        self.grace_sec = float(grace_sec)
        self.period_ms = 1000.0 / max(1e-6, float(nominal_hz))
        self.sane_max_ms = float(sane_max_ms)
        self.max_skew_ms = float(max_skew_ms)
        self._samples = collections.deque(maxlen=self.median_n)
        self._last_rx = None
        self._last_age_ms = None
        self._t0 = None
        self._blank_until = None   # set on a rejected sample; a short blind spot, not forever

    def reset(self, now_sec, reason=""):
        self._samples.clear()
        self._last_rx = None
        self._last_age_ms = None
        self._t0 = now_sec
        self._blank_until = None

    def add_sample(self, now_sec, age_ms):
        """Returns False if the sample was rejected as implausible (clock step, bad stamp)."""
        if self._t0 is None:
            self._t0 = now_sec
        if age_ms < -self.max_skew_ms or age_ms > self.sane_max_ms:
            # Rejecting alone is not enough: no new sample means the starvation term climbs
            # and would fire a false CRITICAL. Blank instead, so a one-off clock step becomes a
            # short blind spot rather than a spurious failover.
            #
            # But do NOT restart the startup grace window here. If EVERY sample is implausible -
            # e.g. the publisher stamps a monotonic clock while we read epoch, which is exactly
            # what the pre-2026-08 bench bags do - resetting _t0 each time kept evaluate() in
            # "init" forever and the detector went permanently, silently blind. Blanking for
            # grace_sec and then escalating to "no_odom" (WARN + interlock, never CRITICAL) is
            # the intended behaviour for a misconfigured source.
            self._samples.clear()
            self._last_rx = None
            self._last_age_ms = None
            if self._blank_until is None:
                # Only the FIRST rejection starts the blind spot. Extending it on every rejection
                # would keep a permanently-bad source in "init" forever - the same silent-blindness
                # bug in a different disguise.
                self._blank_until = now_sec + self.grace_sec
            return False
        self._samples.append(float(age_ms))
        self._last_rx = now_sec
        self._last_age_ms = float(age_ms)
        self._blank_until = None
        return True

    def evaluate(self, now_sec):
        """-> (effective_age_ms or None, note). note in {init, no_odom, stale, ''}."""
        if self._t0 is None:
            self._t0 = now_sec
        if self._last_rx is None:
            blanked = self._blank_until is not None and now_sec < self._blank_until
            if blanked or (now_sec - self._t0) < self.grace_sec:
                return (None, "init")
            return (None, "no_odom")
        ordered = sorted(self._samples)
        med = ordered[len(ordered) // 2]
        starve_ms = max(0.0, (now_sec - self._last_rx) * 1000.0)
        eff = max(med, self._last_age_ms + starve_ms)
        note = "stale" if eff > med + 1.5 * self.period_ms else ""
        return (eff, note)


class HoldTimer:
    """Seconds a condition has held without a break. Pure, like LatencyTracker: takes now."""

    def __init__(self):
        self._since = None

    def update(self, now_sec, cond):
        if not cond:
            self._since = None
            return 0.0
        if self._since is None:
            self._since = now_sec
        return now_sec - self._since

    def reset(self):
        self._since = None


class ResidualCheck:
    """Upper bound on FAST-LIO's mean point-to-plane residual, /fastlio_health data[1].

    Absolute OR relative to a rolling baseline, because the healthy level depends on the room:
    across 28 flight sessions (25-30 Sep) the median ran 0.018 in the 4th-floor rooms and 0.046
    on the open floor, so no single absolute bar fits both. Relative alone is not enough either:
    on 30 Sep 16:10 (LAND, SLAM height went 1.7 -> -0.95 m) the open-floor baseline was 0.040, so
    2.5x is 0.10 and the 0.09-0.11 excursion only crossed it for 0.26 s, while 0.06 abs held 1.24 s.

    Calibration (per-scan, 20 Hz, longest run while armed, 28 sessions):
      warn  >0.055 or >2.0x avg   healthy worst 0.25 s | crash 3.27 s, 30 Sep LAND 1.34 s
      crit  >0.06  or >2.5x avg   healthy worst 0.10 s | crash 3.00 s, 30 Sep LAND 1.24 s
    k = 10 scans (0.5 s) for both: 2x margin for WARN, 5x for CRITICAL.

    The baseline takes only scans that are themselves under the warn bar, so a slow degradation
    cannot drag its own reference up behind it. Pure: no clock, no ROS.
    """

    def __init__(self, warn_abs=0.055, warn_rel=2.0, crit_abs=0.06, crit_rel=2.5,
                 k_warn=10, k_crit=10, win_n=150, warmup_n=30):
        self.warn_abs, self.warn_rel = float(warn_abs), float(warn_rel)
        self.crit_abs, self.crit_rel = float(crit_abs), float(crit_rel)
        self.k_warn, self.k_crit = int(k_warn), int(k_crit)
        self.warmup_n = int(warmup_n)
        self.base = collections.deque(maxlen=int(win_n))
        self.n_warn = self.n_crit = 0
        self.last = 0.0

    def reset(self):
        self.base.clear()
        self.n_warn = self.n_crit = 0

    def avg(self):
        return mean(self.base)

    def update(self, res):
        """-> (crit, warn, raw_bad, reason). crit/warn are hysteresis-latched (k scans);
        raw_bad is this scan alone, for the caller's release counter."""
        self.last = res = float(res)
        if len(self.base) < self.warmup_n:
            self.base.append(res)
            return (False, False, False, "")
        avg = self.avg()
        crit_raw = res > self.crit_abs or res > self.crit_rel * avg
        warn_raw = res > self.warn_abs or res > self.warn_rel * avg
        self.n_crit = self.n_crit + 1 if crit_raw else 0
        self.n_warn = self.n_warn + 1 if warn_raw else 0
        if not warn_raw:
            self.base.append(res)
        crit = self.n_crit >= self.k_crit
        warn = self.n_warn >= self.k_warn
        reason = ("res=%.3f(%.1fxavg)" % (res, res / avg if avg > 0 else 0.0)) if (crit or warn) else ""
        return (crit, warn, warn_raw, reason)


class EkfInnovationCheck:
    """The FC's own verdict on our pose: EKF_STATUS_REPORT.pos_horiz_variance.

    Despite the name ArduPilot fills that field with sqrt(posTestRatio) (NavEKF3 getVariances),
    the same number the dataflash log records as XKF4.SP. >= 1.0 means the EKF is REJECTING the
    vision position and dead-reckoning on its IMU - what it did for 5 s before the 26 Sep crash.

    Calibration (XKF4.SP, primary core, armed, 30 FC logs):
      healthy flights peak at 0.67; benign one-sample spikes reach 1.0-1.45 but hold <= 0.12 s
      (logs 209, 210). Faults: log 218 (26 Sep crash) held >= 1.0 for 5.1 s from 339.66;
      log 211 (25 Sep, 200 deg vision yaw jump) held 0.48 s.
      warn >= 0.8 for 0.4 s, crit >= 1.0 for 0.4 s: 3.3x the benign spikes, catches both.
    Only meaningful while the EKF is fusing vision (SRC1), i.e. armed and not failed over, and
    outside our own restart blank - the caller decides that and passes active.
    """

    def __init__(self, warn=0.8, crit=1.0, warn_hold=0.4, crit_hold=0.4, stale_sec=1.0):
        self.warn, self.crit = float(warn), float(crit)
        self.warn_hold, self.crit_hold = float(warn_hold), float(crit_hold)
        self.stale_sec = float(stale_sec)
        self._warn_t, self._crit_t = HoldTimer(), HoldTimer()
        self.last = None
        self.last_rx = None

    def add_sample(self, now_sec, sp):
        self.last = float(sp)
        self.last_rx = now_sec
        self._warn_t.update(now_sec, self.last >= self.warn)
        self._crit_t.update(now_sec, self.last >= self.crit)

    def evaluate(self, now_sec, active):
        """-> (level, reason)"""
        if not active or self.last_rx is None or (now_sec - self.last_rx) > self.stale_sec:
            self._warn_t.reset(); self._crit_t.reset()
            return (OK, "")
        # held = time since the run started, measured against now: a run that is still going
        # keeps growing between samples, and the stale guard above catches a stream that stops.
        crit_held = self._crit_t.update(now_sec, self.last >= self.crit)
        warn_held = self._warn_t.update(now_sec, self.last >= self.warn)
        if crit_held >= self.crit_hold:
            return (CRITICAL, "sp=%.2f" % self.last)
        if warn_held >= self.warn_hold:
            return (WARN, "sp=%.2f" % self.last)
        return (OK, "")


def stick_frac(pwm, trim, dz, lo, hi):
    """ArduPilot pwm_to_angle_dz, normalised to -1..1: zero inside trim+-dz, linear outside."""
    if pwm > trim + dz:
        return min(1.0, (pwm - trim - dz) / float(max(1, hi - trim - dz)))
    if pwm < trim - dz:
        return max(-1.0, (pwm - trim + dz) / float(max(1, trim - dz - lo)))
    return 0.0


class MotionCheck:
    """Is the vehicle moving in a way the sticks did not ask for? Three tests, all armed+airborne:

      HORIZONTAL (LOITER, LAND): Loiter turns a steady stick into a steady velocity, about
        stick_fraction x LOIT_SPEED x 0.85 in the body frame (fitted on 13k samples, logs 209-233:
        forward -3.5 m/s per unit pitch, right +3.3 per unit roll; pitch PWM high = backwards).
        Error = |actual - commanded| once the sticks have been steady for settle_sec, and only
        with the YAW stick centred: SLAM's velocity is the lidar's, not the CG's, so a pilot spin
        sweeps it in a circle (log 218 t=100-102: 330 deg/s spin, SLAM read 1.4 m/s, EKF 0.3).
        Healthy worst over 1.0 m/s: EKF 0.4 s, SLAM 1.0 s (a landing). warn 0.75 s, crit 1.5 s.
      VERTICAL (LOITER, ALT_HOLD, POSHOLD, LAND): only the DIRECTION is tested, which needs no
        model of the climb-rate curve: throttle centred -> |vz| <= v_err; throttle low (and LAND)
        -> not climbing; throttle high -> not sinking. Healthy flights: zero runs. Found one real
        event: log 216 t=1050 (26 Sep), sticks centred, sank 0.76 m/s on 100% throttle for 1.8 s.
      TILT (LOITER, LAND): Loiter caps lean at LOIT_ANG_MAX (20). Healthy max 24 deg for 0.28 s;
        the 26 Sep crash held >25 for 2 s and >30 for 0.6 s.

    Velocity comes from up to two sources (the FC's EKF and SLAM's own velocity); either one
    disagreeing with the sticks counts, and the reason names which. Neither catches the 29 Sep
    climb (log 233): there EKF and SLAM height were BOTH stale and only the rangefinder saw it.

    Pure: callers feed timestamped inputs, evaluate(now) decides. Frames: velocities ENU, yaw ENU
    (from East, CCW, radians), attitude degrees.
    """

    H_MODES = ("LOITER", "LAND")
    V_MODES = ("LOITER", "ALT_HOLD", "POSHOLD", "LAND")

    def __init__(self, loit_speed=4.0, speed_gain=0.85,
                 rc_trim=1500, rc_dz=20, rc_min=1050, rc_max=1950,
                 thr_trim=1495, thr_dz=20,
                 settle_sec=1.0, min_throttle_out=0.15, input_stale_sec=0.5,
                 h_err=1.0, h_warn_hold=0.75, h_crit_hold=1.5,
                 v_err=0.5, v_warn_hold=0.5, v_crit_hold=1.0,
                 tilt_warn=25.0, tilt_crit=30.0, tilt_hold=0.3):
        self.loit_speed, self.speed_gain = float(loit_speed), float(speed_gain)
        self.rc = (int(rc_trim), int(rc_dz), int(rc_min), int(rc_max))
        self.thr_trim, self.thr_dz = int(thr_trim), int(thr_dz)
        self.settle_sec, self.min_thr = float(settle_sec), float(min_throttle_out)
        self.stale = float(input_stale_sec)
        self.h_err, self.h_warn_hold, self.h_crit_hold = float(h_err), float(h_warn_hold), float(h_crit_hold)
        self.v_err, self.v_warn_hold, self.v_crit_hold = float(v_err), float(v_warn_hold), float(v_crit_hold)
        self.tilt_warn, self.tilt_crit, self.tilt_hold = float(tilt_warn), float(tilt_crit), float(tilt_hold)
        self.mode, self.armed = "", False
        self.bench = False         # bench_mode: run disarmed (see DriftMonitor._poll_bench_mode)
        self._rc = None            # (t, roll_frac, pitch_frac, thr_zone, yaw_centred)
        self._stick_ref = None     # (roll_frac, pitch_frac) at the last stick change
        self._stick_t = None       # when the sticks last moved (or the mode changed)
        self._zone_ref = None
        self._zone_t = None
        self._thr_out = (None, 0.0)
        self._att = None           # (t, roll_deg, pitch_deg, yaw_rad_enu)
        self._vel = {}             # source -> (t, vE, vN, vU)
        self._h_t, self._v_t = HoldTimer(), HoldTimer()
        self._tw_t, self._tc_t = HoldTimer(), HoldTimer()

    # ---- inputs ----
    def set_state(self, now_sec, mode, armed):
        if mode != self.mode:
            self._stick_t = self._zone_t = now_sec   # entering a mode is itself a transient
        self.mode, self.armed = mode, bool(armed)

    def set_rc(self, now_sec, roll_pwm, pitch_pwm, thr_pwm, yaw_pwm):
        trim, dz, lo, hi = self.rc
        rf, pf = stick_frac(roll_pwm, trim, dz, lo, hi), stick_frac(pitch_pwm, trim, dz, lo, hi)
        yaw_c = stick_frac(yaw_pwm, trim, dz, lo, hi) == 0.0
        zone = 1 if thr_pwm > self.thr_trim + self.thr_dz else (-1 if thr_pwm < self.thr_trim - self.thr_dz else 0)
        if (self._stick_ref is None or abs(rf - self._stick_ref[0]) > 0.04
                or abs(pf - self._stick_ref[1]) > 0.04 or not yaw_c):
            self._stick_ref, self._stick_t = (rf, pf), now_sec   # a yawing pilot restarts the settle
        if zone != self._zone_ref:
            self._zone_ref, self._zone_t = zone, now_sec
        self._rc = (now_sec, rf, pf, zone, yaw_c)

    def set_throttle_out(self, now_sec, thr):
        self._thr_out = (now_sec, float(thr))

    def set_attitude(self, now_sec, roll_deg, pitch_deg, yaw_enu_rad):
        self._att = (now_sec, roll_deg, pitch_deg, yaw_enu_rad)

    def set_velocity(self, now_sec, source, v_e, v_n, v_u):
        self._vel[source] = (now_sec, v_e, v_n, v_u)

    def _reset(self):
        for t in (self._h_t, self._v_t, self._tw_t, self._tc_t):
            t.reset()

    def _fresh(self, sample, now_sec):
        return sample is not None and sample[0] is not None and (now_sec - sample[0]) <= self.stale

    def evaluate(self, now_sec):
        """-> (level, reason)"""
        # bench: disarmed, carried by hand. Drop the armed and throttle-out gates and read any
        # non-Loiter mode as LOITER, so walking it with the sticks centred is "uncommanded motion".
        # Only while disarmed: once armed the normal gates apply whatever the flag says.
        bench = self.bench and not self.armed
        mode = self.mode if (not bench or self.mode in self.V_MODES) else "LOITER"
        if (not (self.armed or bench) or not self._fresh(self._rc, now_sec)
                or not self._fresh(self._att, now_sec)
                or (not bench and (not self._fresh(self._thr_out, now_sec)
                                   or self._thr_out[1] < self.min_thr))):
            self._reset()
            return (OK, "")
        _, rf, pf, zone, _ = self._rc
        _, roll, pitch, yaw = self._att
        vels = [(s, v) for s, v in self._vel.items() if self._fresh(v, now_sec)]

        # horizontal: actual body-frame velocity against the Loiter stick command
        h_bad, h_why = False, ""
        if (mode in self.H_MODES and self._stick_t is not None
                and now_sec - self._stick_t >= self.settle_sec):
            k = self.loit_speed * self.speed_gain
            fwd_cmd, right_cmd = -pf * k, rf * k      # pitch PWM high = nose up = backwards
            c, s = math.cos(yaw), math.sin(yaw)
            for src, (_, ve, vn, _) in vels:
                fwd, right = ve * c + vn * s, ve * s - vn * c
                err = math.hypot(fwd - fwd_cmd, right - right_cmd)
                if err > self.h_err:
                    h_bad, h_why = True, "mot:h%.1f(%s)" % (err, src)
                    break
        h_held = self._h_t.update(now_sec, h_bad)

        # vertical: direction against the throttle stick
        v_bad, v_why = False, ""
        if (mode in self.V_MODES and self._zone_t is not None
                and now_sec - self._zone_t >= self.settle_sec):
            want = -1 if mode == "LAND" else zone
            for src, (_, _, _, vu) in vels:
                if ((want == 0 and abs(vu) > self.v_err) or (want < 0 and vu > self.v_err)
                        or (want > 0 and vu < -self.v_err)):
                    v_bad, v_why = True, "mot:vz%+.1f(%s)" % (vu, src)
                    break
        v_held = self._v_t.update(now_sec, v_bad)

        tilt = max(abs(roll), abs(pitch)) if mode in self.H_MODES else 0.0
        tw_held = self._tw_t.update(now_sec, tilt > self.tilt_warn)
        tc_held = self._tc_t.update(now_sec, tilt > self.tilt_crit)
        t_why = "mot:tilt%.0f" % tilt

        if tc_held >= self.tilt_hold:
            return (CRITICAL, t_why)
        if h_held >= self.h_crit_hold:
            return (CRITICAL, h_why)
        if v_held >= self.v_crit_hold:
            return (CRITICAL, v_why)
        if tw_held >= self.tilt_hold:
            return (WARN, t_why)
        if h_held >= self.h_warn_hold:
            return (WARN, h_why)
        if v_held >= self.v_warn_hold:
            return (WARN, v_why)
        return (OK, "")


class DriftMonitor:
    def __init__(self):
        rospy.init_node("drift_monitor", anonymous=False)
        g = rospy.get_param
        self.health_topic = g("~health_topic", "/fastlio_health")
        self.win_n = int(g("~baseline_window", 150))
        self.warmup_n = int(g("~warmup_scans", 30))

        # thresholds (calibrated on tuned config; see calibrate_thresholds.py)
        self.eig_warn_rel = float(g("~eig_warn_rel", 0.2))
        self.eig_warn_abs = float(g("~eig_warn_abs", 40000.0))
        self.eig_crit_abs = float(g("~eig_crit_abs", 5000.0))
        self.matched_warn_rel = float(g("~matched_warn_rel", 0.2))
        self.matched_warn_abs = float(g("~matched_warn_abs", 300.0))
        self.matched_crit_abs = float(g("~matched_crit_abs", 150.0))
        self.k_warn = int(g("~k_warn", 5))
        self.k_crit = int(g("~k_crit", 5))
        self.k_release = int(g("~k_release", 10))
        self.reset_gap = float(g("~reset_gap_sec", 1.5))   # /fastlio_health gap => SLAM restarted => re-warm
        # Registration-residual upper bound on health data[1] (OFF by default; FAST-LIO unchanged).
        # Added for slam_backend:=bievr (PROGRESS P23). On the Sept 26 log-218 replay BIEVR's
        # eig_min/matched dipped no lower than they did in healthy confined flight earlier on the
        # same sortie, so no floor separates the failure; the mean residual did (healthy worst run
        # above 0.0025 = 5 scans, failure held from bag 87.49 s). Units are backend-specific.
        self.enable_residual_check = bool(g("~enable_residual_check", False))
        self.res_warn_abs = float(g("~res_warn_abs", float("inf")))
        self.res_crit_abs = float(g("~res_crit_abs", float("inf")))

        # ---- (4)-(6): FAST-LIO residual trend, EKF innovation, motion vs RC (2026-10-07) ----
        # Separate from the backend-agnostic res_*_abs test above: this one is calibrated on
        # FAST-LIO's units and is abs OR x-baseline (see ResidualCheck). Every one of the three is
        # OBSERVE-ONLY until its enable_*_critical is set: its CRITICAL shows as WARN "(obs)" and a
        # GCS banner, and commands nothing. Set a gate only after flights confirm it stays quiet.
        self.enable_fl_residual_check = bool(g("~enable_fl_residual_check", True))
        self.enable_fl_residual_critical = bool(g("~enable_fl_residual_critical", False))
        self._fl_res = ResidualCheck(
            warn_abs=float(g("~fl_res_warn_abs", 0.055)), warn_rel=float(g("~fl_res_warn_rel", 2.0)),
            crit_abs=float(g("~fl_res_crit_abs", 0.06)), crit_rel=float(g("~fl_res_crit_rel", 2.5)),
            k_warn=int(g("~fl_res_k_warn", 10)), k_crit=int(g("~fl_res_k_crit", 10)),
            win_n=self.win_n, warmup_n=self.warmup_n)
        self.enable_sp_check = bool(g("~enable_sp_check", True))
        self.enable_sp_critical = bool(g("~enable_sp_critical", False))
        self.mavlink_from_topic = g("~mavlink_from_topic", "/mavlink/from")
        self._ekf = EkfInnovationCheck(
            warn=float(g("~sp_warn", 0.8)), crit=float(g("~sp_crit", 1.0)),
            warn_hold=float(g("~sp_warn_hold_sec", 0.4)), crit_hold=float(g("~sp_crit_hold_sec", 0.4)),
            stale_sec=float(g("~sp_stale_sec", 1.0)))
        self._warned_no_sp = False
        # bench_mode: lets (5)/(6) run DISARMED so they can be exercised by carrying the vehicle
        # (motors never armed). While on, all three of (4)-(6) are forced observe-only whatever
        # their enable_*_critical says. It does not touch checks (1)-(3), CH9 or the restart path.
        # Re-read every second, so `rosparam set /drift_monitor/bench_mode true|false` takes
        # effect live. Ignored while armed: in the air (5)/(6) use their normal gates.
        self._bench = False
        self._bench_poll_at = 0.0
        self._bench_armed_noted = False
        self.enable_motion_check = bool(g("~enable_motion_check", True))
        self.enable_motion_critical = bool(g("~enable_motion_critical", False))
        # RC channels are what the FC obeys, i.e. RCMAP_*: in drone mode the tank Lua routes the
        # pilot's roll/pitch/yaw onto 11/15/16 (trim 1500, DZ 20); throttle stays on 3 (trim 1495,
        # THR_DZ 20). /mavros/rc/in carries the routed values (checked across 28 sessions).
        self.motion_roll_ch = int(g("~motion_roll_ch", 11))
        self.motion_pitch_ch = int(g("~motion_pitch_ch", 15))
        self.motion_yaw_ch = int(g("~motion_yaw_ch", 16))
        self.motion_thr_ch = int(g("~motion_thr_ch", 3))
        self.motion_slam_vel_topic = g("~motion_slam_vel_topic", "/mavros/vision_speed/speed_vector")
        self._motion = MotionCheck(
            loit_speed=float(g("~motion_loit_speed", 4.0)), speed_gain=float(g("~motion_speed_gain", 0.85)),
            rc_trim=int(g("~motion_rc_trim", 1500)), rc_dz=int(g("~motion_rc_dz", 20)),
            rc_min=int(g("~motion_rc_min", 1050)), rc_max=int(g("~motion_rc_max", 1950)),
            thr_trim=int(g("~motion_thr_trim", 1495)), thr_dz=int(g("~motion_thr_dz", 20)),
            settle_sec=float(g("~motion_settle_sec", 1.0)),
            min_throttle_out=float(g("~motion_min_throttle_out", 0.15)),
            h_err=float(g("~motion_h_err", 1.0)), h_warn_hold=float(g("~motion_h_warn_hold_sec", 0.75)),
            h_crit_hold=float(g("~motion_h_crit_hold_sec", 1.5)),
            v_err=float(g("~motion_v_err", 0.5)), v_warn_hold=float(g("~motion_v_warn_hold_sec", 0.5)),
            v_crit_hold=float(g("~motion_v_crit_hold_sec", 1.0)),
            tilt_warn=float(g("~motion_tilt_warn_deg", 25.0)), tilt_crit=float(g("~motion_tilt_crit_deg", 30.0)),
            tilt_hold=float(g("~motion_tilt_hold_sec", 0.3)))
        self._new_banner_key = None   # which new-check causes the last GCS banner named

        # ---- latency dimension (added 2026-09-01; see module docstring) ----
        self.enable_latency_check = bool(g("~enable_latency_check", True))
        self.enable_latency_critical = bool(g("~enable_latency_critical", True))
        self.odom_topic = g("~odom_topic", "/Odometry")
        self.lat_warn_ms = float(g("~lat_warn_ms", 150.0))   # ~2x the 67-78 ms fleet nominal
        self.lat_crit_ms = float(g("~lat_crit_ms", 250.0))   # ArduPilot's compensation ceiling
        self.lat_arm_ms = float(g("~lat_arm_ms", 150.0))     # arming uses the conservative bar
        self.lat_drain_sec = float(g("~lat_drain_sec", 5.0)) # OF-only grace before escalating to restart
        # Terminal stage: if a restart did not bring SLAM back, put the aircraft down rather than
        # leave it on a fallback indefinitely. Default OFF - it commands a real descent.
        self.enable_latency_land = bool(g("~enable_latency_land", False))
        self.lat_median_n = int(g("~lat_median_n", 5))
        # 15 (0.75 s) not 10: across 28761 effective-latency samples from 20 healthy flights the
        # worst run above lat_warn_ms is exactly 10 scans, so k=10 fires with ZERO margin - and did,
        # on 07_13_2026_14_16_19. A WARN that cries wolf is the specific failure this whole feature
        # is meant to fix (see the eight self-inflicted "EKF variance" banners), so credibility
        # matters more than the 0.25 s it costs.
        self.lat_k_warn = int(g("~lat_k_warn", 15))          # 0.75 s @20Hz
        # 20 (1.0 s) confirmed by the same data: worst healthy run above lat_crit_ms is 6 scans,
        # so this is a 3.3x margin. Dropping to 10 would buy 0.5 s on the crash profile but leave
        # only 1.7x against an in-flight EKF source switch. Kept.
        self.lat_k_crit = int(g("~lat_k_crit", 20))          # 1.0 s @20Hz
        self.lat_k_release = int(g("~lat_k_release", 40))    # 2.0 s @20Hz, slower than latching
        # deliberately still 10 while lat_k_warn moved to 15: for a BANNER the safe direction is
        # fewer false alarms, but for ARMING it is to block readily. A spurious lock costs 3 s.
        self.lat_k_arm_lock = int(g("~lat_k_arm_lock", 10))      # lock fast
        self.lat_k_arm_release = int(g("~lat_k_arm_release", 60))  # unlock slow (3 s)
        # NAMED_VALUE_FLOAT feed to the FC, consumed by slam_latency_gate.lua as an arming gate.
        # 4 Hz: the gate only has to be current at the moment someone arms, and this shares the
        # telemetry link with everything else. Name is capped at 10 chars by the MAVLink field.
        self.enable_lat_nvf = bool(g("~enable_lat_nvf", True))
        self.lat_nvf_name = str(g("~lat_nvf_name", "SLAMLAT"))[:10]
        self.lat_nvf_hz = float(g("~lat_nvf_hz", 4.0))
        self.lat_startup_grace_sec = float(g("~lat_startup_grace_sec", 5.0))
        self.lat_restart_blank_sec = float(g("~lat_restart_blank_sec", 20.0))
        self.lat_sane_max_ms = float(g("~lat_sane_max_ms", 30000.0))
        self.lat_max_skew_ms = float(g("~lat_max_skew_ms", 50.0))
        self.lat_gates_recovery = bool(g("~lat_gates_recovery", True))
        # Health watchdog. health_cb is driven by the very topic laserMapping publishes, so if
        # laserMapping dies the samples AND the evaluator stop together and this node goes
        # completely silent. Measured in bench test T1 (2026-09-01): 3.37 s with no output at
        # all and the interlock never locking. Nothing driven by a topic can notice that topic's
        # absence, so this needs a timer that fires on its own.
        self.enable_health_watchdog = bool(g("~enable_health_watchdog", True))
        self.health_timeout = float(g("~health_timeout_sec", 1.0))   # 20 missed scans at 20 Hz

        # A replay must never command the FC. rosbag play --clock sets /use_sim_time.
        if rospy.get_param("/use_sim_time", False) and self.enable_latency_critical:
            rospy.logwarn("drift_monitor: use_sim_time=true -> forcing enable_latency_critical=false")
            self.enable_latency_critical = False
        if rospy.get_param("/use_sim_time", False):
            self.enable_fl_residual_critical = self.enable_sp_critical = self.enable_motion_critical = False

        # response gates
        self.enable_auto_restart = bool(g("~enable_auto_restart", False))  # kill->respawn laserMapping
        self.enable_critical_fc = bool(g("~enable_critical_fc", False))    # set_mode LAND (COMMANDS FC)
        self.enable_drift_land = bool(g("~enable_drift_land", True))       # drift CRITICAL drives the action
        self.enable_ch9_land = bool(g("~enable_ch9_land", False))          # CH9 switch drives the action
        self.restart_nodes = g("~restart_nodes", "/laserMapping /fastlio_mavros_bridge")
        self.land_mode = g("~land_mode", "LAND")   # FC flight mode used by restart_mode="land"
        # ...and the custom mode number used instead when the FC is Rover-based (QuadRover):
        # 21 = QRLand (owner 2026-10-20). -1 = always use land_mode. See land_command().
        self.land_mode_rover = int(g("~land_mode_rover", 21))
        self._fc_mav_type = None    # HEARTBEAT.type of the FC (sysid fc_sysid, compid 1)
        self.fc_sysid = int(g("~fc_sysid", 1))
        self.statustext_period = float(g("~statustext_period", 2.0))

        # restart_mode selects the FC response run on trigger (drift CRITICAL, latency CRITICAL, CH9):
        #   "of"   -> EKF source failover SLAM(SRC1)->OF(SRC2), switch back on recovery (DEFAULT;
        #             no flight-mode change, benign disarmed, vehicle keeps flying on OF)
        #   "land" -> the same SRC2 failover FIRST, then command the FC into LAND (set_mode). NEVER
        #             switched back to SRC1: the vehicle stays on SRC2 until the FC is rebooted
        #             (the selection is not saved) or the tank script re-selects TANK_NAV_SRC.
        # Both modes then restart laserMapping. Toggle persistently via switch_restart_mode.sh.
        #
        # Why "land" switches to SRC2 too (2026-10-06). It used to LAND on SRC1, i.e. on SLAM height
        # (EK3_SRC1_POSZ=6, EK3_RNG_USE_HGT=-1), and to skip the source switch on a latency CRITICAL
        # entirely. FC log 233 (2026-09-29, t=1402-1407): latency CRITICAL in Loiter, no switch, SLAM
        # height froze (moved ~0.25 m while the rangefinder read 2.0 -> 4.5 m), the EKF followed it and
        # the vehicle climbed at 4 m/s on full throttle with the pilot's stick at minimum. On SRC2 the
        # EKF height is the rangefinder (EK3_SRC2_POSZ=2), so LAND's control loop and its speed
        # schedule (fc_scripts/land_speed_schedule.lua) then read the same sensor.
        self.restart_mode = str(g("~restart_mode", "of")).lower()
        if self.restart_mode not in ("of", "land"):
            rospy.logwarn("drift_monitor: unknown restart_mode=%r -> defaulting to 'of'", self.restart_mode)
            self.restart_mode = "of"

        # RC stream keepalive: ArduPilot does NOT stream RC_CHANNELS to the companion link
        # unless requested, and mavros doesn't request it on connect -> /mavros/rc/in stays empty
        # and the CH9 trigger is blind. The FC param SR0_RC_CHAN persists, but the *request* is
        # what actually starts the stream. Re-request STREAM_RC_CHANNELS on every mavros (re)connect
        # so CH9 survives reboots and mavros restarts without a manual kick.
        self.request_rc_stream = bool(g("~request_rc_stream", True))
        self.rc_stream_rate = int(g("~rc_stream_rate", 10))
        self._mav_connected = False
        self._armed = False

        # EKF source failover (REPLACES LAND): on restart, lane-change FC EKF source SLAM(SRC1)->OF(SRC2)
        # so the vehicle keeps flying on Optical Flow while SLAM re-inits; switch back SRC2->SRC1 on recovery.
        # Mechanism: MAV_CMD_DO_AUX_FUNCTION(218), func 90 = "EKF Source Set", switch position selects the set.
        self.enable_source_failover = bool(g("~enable_source_failover", True))
        # Minimum seconds between AUTOMATIC node restarts. A restart is a hypothesis: that SLAM
        # is stuck and a fresh process will clear it. When the condition returns immediately the
        # hypothesis was wrong -- starved geometry has no features for a new process to find --
        # and repeating it just kills SLAM every few seconds. Observed 2026-09-03 17:25:17: five
        # restarts in 26 s, one every 5.2 s, matched 26-33 against a floor of 150, each one
        # followed by a clean re-warm that immediately re-fired. Inside the cooldown the
        # FAILOVER still stands (it is correct either way); only the kill is declined. A
        # pilot-commanded CH9 restart is never suppressed.
        self.restart_cooldown_sec = float(g("~restart_cooldown_sec", 45.0))
        self.ekf_src_auxfn = int(g("~ekf_src_auxfn", 90))       # ArduPilot RCx_OPTION 90 = EKF Source Set
        self.src2_switch_pos = int(g("~src2_switch_pos", 1))    # DO_AUX_FUNCTION pos 1=MIDDLE -> source set 2 (OF)
        self.src1_switch_pos = int(g("~src1_switch_pos", 0))    # pos 0=LOW -> source set 1 (SLAM/vision)
        self.failover_active = False                            # True while EKF is on SRC2 awaiting SLAM recovery

        # CH9 RC trigger (1-indexed channel; edge on entering the [low,high] band)
        self.ch9_channel = int(g("~ch9_channel", 9))
        self.ch9_low = int(g("~ch9_low", 1400))    # LAND middle band low
        self.ch9_high = int(g("~ch9_high", 1600))  # LAND middle band high
        self.ch9_dwell = float(g("~ch9_dwell_sec", 1.5))  # must HOLD middle this long -> LAND (transit-proof)
        self.ch9_in_band = None   # None until first RC reading; never fire if booted already in-band
        self.ch9_mid_since = None
        self.ch9_land_fired = False

        self.base_eig = collections.deque(maxlen=self.win_n)
        self.base_matched = collections.deque(maxlen=self.win_n)
        self.n_crit = self.n_warn = self.n_ok = 0
        self.latched = OK
        self.interlock = "RELEASED"
        self._saw_gap_since_lock = False   # restart's health-gap observed since interlock locked?
        self._last_st = rospy.Time(0)
        self._last_health_t = 0.0

        # latency state
        self._lat = LatencyTracker(self.lat_median_n, self.lat_startup_grace_sec,
                                   sane_max_ms=self.lat_sane_max_ms,
                                   max_skew_ms=self.lat_max_skew_ms)
        self.n_lat_crit = self.n_lat_warn = self.n_lat_ok = 0
        self.n_lat_arm = self.n_lat_armok = 0
        self._health_has_latency = False   # laserMapping publishing data[7]? then ignore /Odometry
        # Armed at STARTUP as well as on every health gap. drift_monitor comes up alongside
        # laserMapping, and a fresh laserMapping's cold start is a multi-second backlog (13.6 s
        # observed). Without this, the cold start fires a latency CRITICAL ~2.4 s in and escalates
        # to a node restart ~7.5 s in - restarting laserMapping during its own cold start, forever.
        # Verified against the recorded drain profile; see scripts/replay/latency_replay.py.
        self._lat_blank_until = time.time() + self.lat_restart_blank_sec
        self._lat_failover_at = None       # when the OF-only latency failover fired
        self._lat_no_restart = False       # an OF-only failover is in effect (no health gap will occur)
        self._lat_escalated = False        # the drain timer already escalated to a full restart
        self._lat_land_fired = False       # terminal LAND already commanded (once only)
        self._warned_no_odom = False
        self._node_start = time.time()      # for "health never arrived" detection
        self._warned_no_health_ever = False
        self._health_lost = False           # watchdog: /fastlio_health currently silent?
        self._health_wd_fired = False       # one-shot failover per outage
        self._last_restart_at = None        # for restart_cooldown_sec
        self._cooldown_announced = False    # one banner per cooldown window, not per scan
        self._suppress_node_kill = False    # set by the cooldown, read by _restart_worker

        # interlock is a two-source OR with one arbiter, so the restart-recovery state machine
        # below keeps working untouched while latency can also hold it.
        self._restart_lock = False
        self._lat_lock = False

        self.risk_pub = rospy.Publisher("/slam/drift_risk", String, queue_size=10, latch=True)
        self.interlock_pub = rospy.Publisher("/slam/arm_interlock", String, queue_size=10, latch=True)
        self.st_pub = rospy.Publisher("/mavros/statustext/send", StatusText, queue_size=10) if _HAVE_MAVROS else None

        # Latency feed to the flight controller. Starts at 0 (permissive) rather than
        # _LAT_UNKNOWN_MS so that merely launching this node cannot block arming before it has
        # had a chance to measure anything; the startup grace in _lat_report_value owns that
        # window, and the FC gate fails open on silence anyway.
        self._lat_report_ms = 0.0
        self.nvf_pub = None
        if self.enable_latency_check and self.enable_lat_nvf and _HAVE_MAVROS:
            self.nvf_pub = rospy.Publisher("/mavros/debug_value/send", DebugValue, queue_size=2)
            rospy.Timer(rospy.Duration(1.0 / max(self.lat_nvf_hz, 0.5)), self._publish_lat_nvf)
            rospy.loginfo("drift_monitor: publishing %s to the FC at %.1f Hz",
                          self.lat_nvf_name, self.lat_nvf_hz)
        elif self.enable_latency_check and self.enable_lat_nvf:
            rospy.logwarn("drift_monitor: mavros_msgs unavailable -- no latency feed to the FC")

        self._publish_interlock("RELEASED")

        rospy.Subscriber(self.health_topic, Float32MultiArray, self.health_cb, queue_size=50)
        # queue_size=1: we only ever want the newest measurement. A deeper queue would hand us a
        # backlog whose "age at processing" is inflated by our own stall rather than by FAST-LIO's.
        if self.enable_latency_check and _HAVE_ODOM:
            rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=1)
        elif self.enable_latency_check:
            rospy.logerr("drift_monitor: nav_msgs unavailable -> latency only from health data[7]")
        if _HAVE_MAVROS and (self.enable_ch9_land or self.enable_motion_check):
            rospy.Subscriber("/mavros/rc/in", RCIn, self._rc_in_cb, queue_size=10)
        if _HAVE_MAVROS:
            rospy.Subscriber("/mavros/state", State, self.state_cb, queue_size=10)
        if _HAVE_MAVROS:
            # EKF_STATUS_REPORT for the SP check, and the FC's HEARTBEAT type for land_command()
            rospy.Subscriber(self.mavlink_from_topic, Mavlink, self._mavlink_cb, queue_size=50)
        if self.enable_motion_check and _HAVE_MAVROS and _HAVE_MOTION_MSGS:
            rospy.Subscriber("/mavros/imu/data", Imu, self._imu_cb, queue_size=1)
            rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, self._ekf_vel_cb, queue_size=1)
            rospy.Subscriber(self.motion_slam_vel_topic, Vector3Stamped, self._slam_vel_cb, queue_size=1)
            rospy.Subscriber("/mavros/vfr_hud", VFR_HUD, self._vfr_cb, queue_size=1)
        elif self.enable_motion_check:
            rospy.logerr("drift_monitor: motion check enabled but mavros/geometry/sensor msgs unavailable")

        if self.enable_health_watchdog:
            rospy.Timer(rospy.Duration(0.25), self._watchdog_cb)

        rospy.loginfo("drift_monitor up. drift_land=%s ch9_land=%s | gates: auto_restart=%s critical_fc(LAND)=%s | "
                      "CH9 ch%d band[%d,%d] | K warn=%d crit=%d",
                      self.enable_drift_land, self.enable_ch9_land, self.enable_auto_restart,
                      self.enable_critical_fc, self.ch9_channel, self.ch9_low, self.ch9_high,
                      self.k_warn, self.k_crit)
        rospy.loginfo("drift_monitor: restart cooldown %.0fs", self.restart_cooldown_sec)
        rospy.loginfo("drift_monitor: restart_mode=%s | source_failover=%s auxfn=%d (OF/SRC2 pos=%d, SLAM/SRC1 pos=%d) | land_mode=%s",
                      self.restart_mode, self.enable_source_failover, self.ekf_src_auxfn,
                      self.src2_switch_pos, self.src1_switch_pos, self.land_mode)
        rospy.loginfo("drift_monitor: latency check=%s critical=%s | warn=%.0fms crit=%.0fms arm=%.0fms | "
                      "K warn=%d crit=%d release=%d | drain=%.1fs blank=%.1fs | src=%s",
                      self.enable_latency_check, self.enable_latency_critical,
                      self.lat_warn_ms, self.lat_crit_ms, self.lat_arm_ms,
                      self.lat_k_warn, self.lat_k_crit, self.lat_k_release,
                      self.lat_drain_sec, self.lat_restart_blank_sec, self.odom_topic)
        if self.enable_latency_land:
            rospy.logwarn("drift_monitor: latency LAND stage ENABLED (terminal descent if a restart does not recover SLAM)")
        rospy.loginfo("drift_monitor: fl_residual=%s(critical=%s) sp=%s(critical=%s, %s) motion=%s(critical=%s, rc %d/%d/%d/%d)",
                      self.enable_fl_residual_check, self.enable_fl_residual_critical,
                      self.enable_sp_check, self.enable_sp_critical, self.mavlink_from_topic,
                      self.enable_motion_check, self.enable_motion_critical, self.motion_roll_ch,
                      self.motion_pitch_ch, self.motion_thr_ch, self.motion_yaw_ch)

    # ---- (5) EKF innovation and (6) motion intake. Clock is time.time(), as in health_cb ----
    def _mavlink_cb(self, msg):
        if msg.msgid == _HEARTBEAT_MSGID and msg.sysid == self.fc_sysid and msg.compid == 1:
            # HEARTBEAT: custom_mode u32, then type u8 (MAV_TYPE); MAVLink2 trims zeros, so pad
            raw = b"".join(struct.pack("<Q", w) for w in msg.payload64).ljust(9, b"\0")
            if self._fc_mav_type != raw[4]:
                rospy.loginfo("drift_monitor: FC HEARTBEAT type %d -> restart_mode=land sends %s",
                              raw[4], land_command(raw[4], self.land_mode, self.land_mode_rover))
            self._fc_mav_type = raw[4]
            return
        if msg.msgid != _EKF_STATUS_REPORT_MSGID or not self.enable_sp_check:
            return
        try:
            raw = b"".join(struct.pack("<Q", w) for w in msg.payload64).ljust(22, b"\0")  # MAVLink2 trims zeros
            _vel, pos_horiz, _pos_vert, _compass, _terrain = struct.unpack_from("<5f", raw)
            self._ekf.add_sample(time.time(), pos_horiz)
        except Exception as e:
            rospy.logwarn_throttle(5.0, "drift_monitor: EKF_STATUS_REPORT decode: %s", e)

    def _rc_in_cb(self, msg):
        if self.enable_motion_check:
            ch = msg.channels
            idx = (self.motion_roll_ch, self.motion_pitch_ch, self.motion_thr_ch, self.motion_yaw_ch)
            if all(0 < i <= len(ch) and ch[i - 1] for i in idx):
                self._motion.set_rc(time.time(), *(ch[i - 1] for i in idx))
        if self.enable_ch9_land:
            self.rc_cb(msg)

    def _imu_cb(self, msg):
        q = msg.orientation
        roll = math.degrees(math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)))
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))))
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))   # ENU
        self._motion.set_attitude(time.time(), roll, pitch, yaw)

    def _ekf_vel_cb(self, msg):
        v = msg.twist.linear
        self._motion.set_velocity(time.time(), "ekf", v.x, v.y, v.z)

    def _slam_vel_cb(self, msg):
        v = msg.vector
        self._motion.set_velocity(time.time(), "slam", v.x, v.y, v.z)

    def _vfr_cb(self, msg):
        self._motion.set_throttle_out(time.time(), msg.throttle)   # mavros scales to 0..1

    def _fc_checks(self, now):
        """(5) + (6) -> (level, reason), CRITICAL already downgraded to WARN "(obs)" where gated."""
        out = []
        self._poll_bench_mode(now)
        bench = self._bench
        if self.enable_sp_check:
            # Only while the EKF is meant to be fusing our pose: on SRC2, or inside our own
            # restart outage, SP describes a source we already know is gone.
            active = ((self._armed or bench) and not self.failover_active and not self._health_lost
                      and now >= self._lat_blank_until)
            if (active and self._ekf.last_rx is None and not self._warned_no_sp
                    and now - self._node_start > 10.0):
                self._warned_no_sp = True
                rospy.logwarn("drift_monitor: no EKF_STATUS_REPORT on %s -- SP check blind (SR0_EXTRA3?)",
                              self.mavlink_from_topic)
            out.append(self._ekf.evaluate(now, active) + (self.enable_sp_critical and not bench,))
        if self.enable_motion_check:
            out.append(self._motion.evaluate(now) + (self.enable_motion_critical and not bench,))
        level, reasons = OK, []
        for lvl, why, gate in out:
            if lvl == CRITICAL and not gate:
                lvl, why = WARN, why + "(obs)"
            if lvl != OK:
                reasons.append(why)
                if _LEVEL[lvl] > _LEVEL[level]:
                    level = lvl
        return level, ",".join(reasons)

    def _poll_bench_mode(self, now):
        if now < self._bench_poll_at:
            return
        self._bench_poll_at = now + 1.0
        try:
            on = bool(rospy.get_param("~bench_mode", False))
        except Exception:
            return
        if on and self._armed and not self._bench_armed_noted:
            self._bench_armed_noted = True
            rospy.logwarn("drift_monitor: bench_mode is on but the vehicle is ARMED -- "
                          "motion/SP use their normal armed gates; set bench_mode false")
        if not self._armed:
            self._bench_armed_noted = False
        if on != self._bench:
            self._bench = on
            self._motion.bench = on
            rospy.logwarn("drift_monitor: bench_mode %s (checks 4-6 observe-only%s)",
                          "ON" if on else "OFF", ", SP+motion run disarmed" if on else "")
            self._send_statustext(_SEV_WARNING, "SLAM bench mode " + ("ON" if on else "OFF"), force=True)

    # ---- latency intake ----
    def odom_cb(self, msg):
        """Age is computed AT RECEIPT, on rospy's callback thread, never at consumption.

        health_cb can block for seconds inside wait_for_service; measuring there would let the
        act of responding to a latency problem inflate the measured latency. Ignored entirely
        once laserMapping is seen publishing data[7], which is the better source.
        """
        if self._health_has_latency or not self.enable_latency_check:
            return
        try:
            now = rospy.get_time()
            age_ms = (rospy.Time.now() - msg.header.stamp).to_sec() * 1000.0
            if not self._lat.add_sample(now, age_ms):
                rospy.logwarn_throttle(5.0, "drift_monitor: implausible odom age %.0f ms -> latency blanked", age_ms)
        except Exception as e:
            rospy.logwarn_throttle(5.0, "drift_monitor: odom_cb: %s", e)

    def _evaluate_latency(self, lat_ms, note):
        """Third dimension. NOTE the >= comparisons: latency is an UPPER bound, unlike the six
        lower-bound tests in _evaluate(). Do not 'fix' them to <."""
        if not self.enable_latency_check:
            return (False, False, "")
        if lat_ms is None:
            # 'init' = boot grace, silent. 'no_odom' = a topic-name typo or a dead publisher;
            # WARN and lock arming, but NEVER let a misconfiguration command a source switch.
            if note == "no_odom":
                if not self._warned_no_odom:
                    self._warned_no_odom = True
                    rospy.logerr("drift_monitor: no latency samples on %s or health data[7] -- check ~odom_topic",
                                 self.odom_topic)
                return (False, True, "lat=no_odom")
            return (False, False, "")
        if lat_ms >= self.lat_crit_ms:
            return (True, False, "lat=%.0fms(crit)" % lat_ms)
        if lat_ms >= self.lat_warn_ms:
            return (False, True, "lat=%.0fms" % lat_ms)
        return (False, False, "")

    def _lat_report_value(self, lat_ms, note):
        """The number sent to the FC as NAMED_VALUE_FLOAT.

        Mirrors _lat_arm_bad() so this node and the FC-side gate cannot disagree about the same
        instant: "init" is deliberately permissive (the tracker has no sample yet and the startup
        grace owns that window), and an unmeasurable state is reported as bad rather than omitted,
        because omitting it looks identical to this node being dead - which the gate treats as
        fail-open.
        """
        if note == "init":
            return 0.0
        if lat_ms is None:
            return _LAT_UNKNOWN_MS
        return lat_ms

    def _publish_lat_nvf(self, _evt):
        if self.nvf_pub is None:
            return
        value = self._lat_report_ms
        # Never report a passing number while our own interlock is locked. The couplings are
        # deliberately one-way: we may not under-report relative to our own verdict, but the FC
        # is free to block on a raw value before our (10-scan) hysteresis has latched.
        if self._lat_lock:
            value = max(value, self.lat_arm_ms)
        msg = DebugValue()
        msg.header.stamp = rospy.Time.now()
        msg.type = DebugValue.TYPE_NAMED_VALUE_FLOAT
        msg.name = self.lat_nvf_name
        msg.value_float = float(value)
        self.nvf_pub.publish(msg)

    def _lat_arm_bad(self, lat_ms, note):
        if not self.enable_latency_check or note == "init":
            return False
        return (lat_ms is None) or (lat_ms >= self.lat_arm_ms)

    def _update_latency_interlock(self):
        if not self._lat_lock and self.n_lat_arm >= self.lat_k_arm_lock:
            self._lat_lock = True
            self._refresh_interlock()
            rospy.logwarn("drift_monitor: latency interlock LOCKED (>=%.0f ms)", self.lat_arm_ms)
        elif self._lat_lock and self.n_lat_armok >= self.lat_k_arm_release:
            self._lat_lock = False
            self._refresh_interlock()
            rospy.loginfo("drift_monitor: latency interlock released")
        if self._lat_lock and not self._armed:
            self._send_statustext(_SEV_WARNING, "SLAM latency high - do not arm")

    # ---- watchdog: notice the ABSENCE of /fastlio_health ----
    def _watchdog_cb(self, _evt):
        """Fires on a timer, independently of any topic.

        Bench T1 showed the failure this closes: killing laserMapping stopped /fastlio_health,
        health_cb never ran again, and the node published nothing for 3.37 s while the interlock
        stayed RELEASED. ArduPilot still protects the aircraft (extNavDataIsFresh goes false
        after 500 ms and its own EKF failsafe runs), but this node contributed nothing - no
        banner, no arming block, and crucially no failover to optical flow, which is exactly the
        right response to SLAM disappearing.
        """
        now = time.time()
        if not self._last_health_t:
            # Never seen a single health message. Normal for the first few seconds, a hard fault
            # after that. Bench T4b (2026-09-01): with a typo'd ~health_topic the node published
            # NOTHING and left the interlock RELEASED indefinitely - arming permitted, no warning.
            # The original guard here was "never seen health yet; startup, not a loss", which
            # never expired, so a misconfiguration was indistinguishable from booting. Same class
            # of bug as the LatencyTracker grace window that reset on every rejected sample: a
            # startup guard has to time out.
            if (now - self._node_start) < self.lat_startup_grace_sec:
                return
            if not self._warned_no_health_ever:
                self._warned_no_health_ever = True
                rospy.logerr("drift_monitor: NO %s message has EVER arrived after %.0fs -- check ~health_topic",
                             self.health_topic, self.lat_startup_grace_sec)
            if not self._lat_lock:
                self._lat_lock = True
                self._refresh_interlock()
            self._lat_report_ms = _LAT_UNKNOWN_MS
            self.risk_pub.publish(String(
                data="WARN | no_health_ever | eig=0(avg 0) matched=0(avg 0) | lat=no_health"))
            self._send_statustext(_SEV_WARNING, "SLAM health topic missing")
            # Deliberately never CRITICAL and never a source switch: a configuration typo must
            # not be able to command an in-flight EKF change. Same rule as lat=no_odom.
            return
        silent = now - self._last_health_t

        if silent < self.health_timeout:
            if self._health_lost:
                self._health_lost = False
                self._health_wd_fired = False
                rospy.loginfo("drift_monitor: /fastlio_health resumed after %.1fs", silent)
            return

        if not self._health_lost:
            self._health_lost = True
            rospy.logwarn("drift_monitor: /fastlio_health SILENT %.1fs -- SLAM node down", silent)

        if not self._lat_lock:
            self._lat_lock = True
            self._refresh_interlock()
        # keep publishing so the topic does not simply stop - a silent topic is indistinguishable
        # from a healthy one to anything watching it.
        self._lat_report_ms = _LAT_UNKNOWN_MS
        self.risk_pub.publish(String(
            data="CRITICAL | health_lost:%.1fs | eig=0(avg 0) matched=0(avg 0) | lat=no_health" % silent))
        self._send_statustext(_SEV_CRITICAL, "SLAM node down %.0fs" % silent)

        # One-shot lane-change to OF. Deliberately does NOT restart anything: roslaunch
        # respawn="true" already owns bringing laserMapping back, and a rosnode kill against a
        # dead node achieves nothing. Both modes: in "land" a dead laserMapping otherwise leaves
        # the EKF on SRC1 with no vision at all. No LAND here; the watchdog has never commanded
        # a flight mode. Recovery runs as usual ("land" stays on SRC2).
        if (not self._health_wd_fired and self.enable_latency_critical
                and now >= self._lat_blank_until):
            self._health_wd_fired = True
            rospy.logwarn("drift_monitor: health lost -> OF (SRC2); respawn owns the restart")
            self._restart_lock = True
            self._refresh_interlock()
            if self.enable_source_failover and not self.failover_active:
                self.failover_active = True
                self._spawn(self._failover_worker)

    # ---- detection ----
    def health_cb(self, msg):
        if len(msg.data) < 5:
            return
        matched = float(msg.data[0]); eig = float(msg.data[4]); res = float(msg.data[1])
        now_ros = rospy.get_time()

        # Preferred latency source: laserMapping publishes age-of-measurement as data[7] in the
        # same loop iteration as the odometry, so it is immune to our own callback blocking.
        if len(msg.data) >= 8 and self.enable_latency_check:
            if not self._health_has_latency:
                self._health_has_latency = True
                rospy.loginfo("drift_monitor: latency from /fastlio_health data[7] (ignoring %s)",
                              self.odom_topic)
            self._lat.add_sample(now_ros, float(msg.data[7]))

        # SLAM restart detection: a gap in /fastlio_health means laserMapping was restarted ->
        # the old baseline is stale and the fresh re-init may look transiently degenerate. Re-warm
        # (drop baseline + counters, latch OK) so we don't immediately re-fire CRITICAL (restart loop).
        now = time.time()
        if self._last_health_t and (now - self._last_health_t) > self.reset_gap:
            rospy.logwarn("drift_monitor: /fastlio_health resumed after %.1fs gap -> SLAM restarted; re-warming",
                          now - self._last_health_t)
            self.base_eig.clear(); self.base_matched.clear()
            self._fl_res.reset()
            self.n_crit = self.n_warn = self.n_ok = 0
            self.latched = OK
            self._saw_gap_since_lock = True   # the restart actually happened
            # A fresh laserMapping's cold start IS a high-latency backlog (13.6 s observed).
            # Without this blank, a latency CRITICAL kills the node whose cold start fires the
            # next latency CRITICAL, forever. Warmup alone only covers ~1.5 s.
            self._lat_blank_until = now + self.lat_restart_blank_sec
        self._last_health_t = now

        # ---- latency: evaluated on EVERY scan, warmup included ----
        # The 25 Aug cold-start takeoff happened entirely inside the warmup window, so gating
        # arming during warmup is the most valuable part of this feature.
        lat_ms, lat_note = self._lat.evaluate(now_ros)
        lat_crit, lat_warn, lat_reason = self._evaluate_latency(lat_ms, lat_note)
        blanked = now < self._lat_blank_until
        if lat_crit and (blanked or not self.enable_latency_critical):
            lat_reason = lat_reason.replace("(crit)", "(blank)" if blanked else "(obs)")
            lat_crit, lat_warn = False, True
        self.n_lat_crit = self.n_lat_crit + 1 if lat_crit else 0
        self.n_lat_warn = self.n_lat_warn + 1 if (lat_crit or lat_warn) else 0
        self.n_lat_ok = 0 if (lat_crit or lat_warn) else self.n_lat_ok + 1
        self._lat_report_ms = self._lat_report_value(lat_ms, lat_note)
        arm_bad = self._lat_arm_bad(lat_ms, lat_note)
        self.n_lat_arm = self.n_lat_arm + 1 if arm_bad else 0
        self.n_lat_armok = 0 if arm_bad else self.n_lat_armok + 1
        self._update_latency_interlock()

        # ---- (4) residual trend + (5)/(6) FC-side checks, every scan (ResidualCheck owns its warmup) ----
        res_crit, res_warn, res_raw, res_reason = (
            self._fl_res.update(res) if self.enable_fl_residual_check else (False, False, False, ""))
        if res_crit and (self._bench or not self.enable_fl_residual_critical):
            res_crit, res_warn, res_reason = False, True, res_reason + "(obs)"
        fc_lvl, fc_reason = self._fc_checks(now)
        new_reason = ",".join(x for x in (res_reason, fc_reason) if x)
        extra = self._new_checks_summary()

        if len(self.base_eig) < self.warmup_n:
            # Geometric baselines are not valid yet, but latency already is. Latency may raise
            # the displayed level to WARN here; it must never reach CRITICAL during warmup,
            # and self.latched is deliberately not written. The FC-side checks follow the same rule.
            self.base_eig.append(eig); self.base_matched.append(matched)
            lvl = WARN if (self.n_lat_warn >= self.lat_k_warn or fc_lvl != OK) else OK
            self._publish_risk(lvl, ",".join(x for x in ("warmup", lat_reason, fc_reason) if x),
                               eig, matched, 0.0, 0.0, lat_ms, lat_note, extra)
            return

        avg_eig = mean(self.base_eig); avg_matched = mean(self.base_matched)
        crit, warn, reason = self._evaluate(eig, matched, avg_eig, avg_matched, res)
        # geom_raw keeps owning the baseline, so a sustained latency WARN does NOT freeze it.
        geom_raw = CRITICAL if crit else (WARN if warn else OK)

        self.n_crit = self.n_crit + 1 if crit else 0
        self.n_warn = self.n_warn + 1 if (warn or crit) else 0
        # a high residual also holds off release, like any other geometric fault
        self.n_ok = self.n_ok + 1 if (geom_raw == OK and not res_raw) else 0

        new = self.latched
        if (self.n_crit >= self.k_crit or self.n_lat_crit >= self.lat_k_crit
                or res_crit or fc_lvl == CRITICAL):
            new = CRITICAL
        elif ((self.n_warn >= self.k_warn or self.n_lat_warn >= self.lat_k_warn
               or res_warn or fc_lvl != OK) and self.latched != CRITICAL):
            new = WARN
        # release is ANDed: never drop out of CRITICAL because the geometry recovered while
        # latency (or an FC-side check) is still bad.
        if self.n_ok >= self.k_release and self.n_lat_ok >= self.lat_k_release and fc_lvl == OK:
            new = OK
        # lat_reason first so it survives the 49-char statustext truncation
        merged = ",".join(x for x in (lat_reason, new_reason, reason if reason != "ok" else "") if x) or "ok"
        # Latency-only = nothing but latency is critical. Formerly tested as reason.startswith("lat=")
        # and n_crit < k_crit, which is equivalent without (4)-(6) but would misroute e.g. an SP
        # CRITICAL that happens to coincide with a latency WARN.
        lat_only = (self.n_lat_crit >= self.lat_k_crit and self.n_crit < self.k_crit
                    and not res_crit and fc_lvl != CRITICAL)
        if new != self.latched:
            self._on_transition(self.latched, new, merged, eig, matched, lat_only)
            self.latched = new
        self._announce_new_checks(new_reason)

        # Staged latency response: OF-only first, escalate to a full restart only if the
        # backlog does not drain. On 25 Aug it never drained - it sat at 450-670 ms for 13 s -
        # so a short timer costs nothing and only means acting sooner.
        if (self._lat_no_restart and not self._lat_escalated and self._lat_failover_at is not None
                and (now - self._lat_failover_at) >= self.lat_drain_sec
                and lat_ms is not None and lat_ms >= self.lat_crit_ms):
            self._lat_escalated = True
            rospy.logwarn("drift_monitor: latency still %.0f ms after %.1fs on OF -> escalating to restart",
                          lat_ms, self.lat_drain_sec)
            self._execute_restart("lat:no-drain")

        # TERMINAL STAGE: the restart did not fix it. Rather than fly on the fallback until the
        # battery failsafe lands the vehicle somewhere arbitrary, put it down deliberately.
        #
        # Gated on the restart blank having EXPIRED, not on a fixed timer. A fresh laserMapping's
        # cold start is itself a multi-second backlog, so any fixed delay short enough to be useful
        # would fire during a normal recovery. When the blank expires and latency is STILL over the
        # ceiling, the restart genuinely failed.
        #
        # Why LAND is safe HERE specifically: we only reach this having been stable on SRC2 for
        # ~25 s, so the EKF's height is EK3_SRC2_POSZ=2, the rangefinder - and we know that sensor
        # is alive because the aircraft is flying on its optical flow. LAND's descent demand
        # (get_alt_above_ground_m) and its control loop (the EKF's altitude) are then the SAME
        # signal, which is exactly what they were not on 25 Aug.
        if (self.enable_latency_land and not self._lat_land_fired and self._lat_escalated
                and now >= self._lat_blank_until
                and lat_ms is not None and lat_ms >= self.lat_crit_ms
                and self._armed):
            self._lat_land_fired = True
            rospy.logwarn("drift_monitor: latency still %.0f ms after restart+blank -> commanding LAND", lat_ms)
            self._send_statustext(_SEV_CRITICAL, "SLAM unrecovered - LANDING", force=True)
            self._spawn(self._command_land)

        # advisory interlock auto-release ONLY after the restart's health-gap is observed AND
        # the fresh SLAM is healthy for k_release scans (prevents releasing instantly off stale n_ok).
        # An OF-only latency failover produces no health gap, hence the _lat_no_restart case.
        if (self._restart_lock and self.n_ok >= self.k_release
                and (self._saw_gap_since_lock or self._lat_no_restart or not self.enable_auto_restart)
                and (not self.lat_gates_recovery or self.n_lat_ok >= self.lat_k_release)):
            # fresh SLAM confirmed healthy -> in "of" mode lane-change back OF(SRC2)->SLAM(SRC1).
            # In "land" mode stay on SRC2 for good (operator decision 2026-10-06): getting back onto
            # SLAM height is a deliberate act - an FC reboot - not something to do automatically.
            # The flag is still cleared, so the next fault re-sends SRC2 rather than assuming it,
            # in case the FC was rebooted onto SRC1 meanwhile (re-sending is harmless).
            # lat_gates_recovery above: handing the EKF back to a SLAM that is geometrically
            # perfect but still publishing stale poses is exactly the 25 Aug failure.
            if self.restart_mode == "of" and self.enable_source_failover and self.failover_active:
                self.failover_active = False       # clear first so this cannot re-enter
                self._spawn(self._set_ekf_source, self.src1_switch_pos, "SLAM (SRC1)")
                recovery_msg = "Restart finished, back to SRC1"
            elif self.failover_active:
                self.failover_active = False
                recovery_msg = "SLAM back, staying on SRC2"
            else:
                recovery_msg = "Restarted, ready to arm"
            self._restart_lock = False
            self._lat_no_restart = False
            self._lat_escalated = False
            self._lat_failover_at = None
            self._lat_land_fired = False
            self._refresh_interlock()
            self._send_statustext(6, recovery_msg, force=True)   # MSG 4/4

        if geom_raw == OK:
            self.base_eig.append(eig); self.base_matched.append(matched)
        self._publish_risk(self.latched, merged if self.latched != OK else "ok",
                           eig, matched, avg_eig, avg_matched, lat_ms, lat_note, extra)

    def _evaluate(self, eig, matched, avg_eig, avg_matched, res=None):
        c, w = [], []
        if eig < self.eig_crit_abs: c.append("eig_min<%.0f(crit)" % self.eig_crit_abs)
        if matched < self.matched_crit_abs: c.append("matched<%.0f(starv)" % self.matched_crit_abs)
        if self.enable_residual_check and res is not None:
            if res > self.res_crit_abs: c.append("res>%.4g(crit)" % self.res_crit_abs)
            if res > self.res_warn_abs: w.append("res>%.4g" % self.res_warn_abs)
        if eig < self.eig_warn_rel * avg_eig: w.append("eig_min<%.2fxavg" % self.eig_warn_rel)
        if eig < self.eig_warn_abs: w.append("eig_min<%.0f" % self.eig_warn_abs)
        if matched < self.matched_warn_rel * avg_matched: w.append("matched<%.2fxavg" % self.matched_warn_rel)
        if matched < self.matched_warn_abs: w.append("matched<%.0f" % self.matched_warn_abs)
        return (len(c) > 0), (len(w) > 0), (",".join(c if c else w) or "ok")

    def _announce_new_checks(self, new_reason):
        """GCS banner for (4)-(6). Once when a cause appears, again only when the SET of causes
        changes - not every scan, and not on its value wobbling (sp=1.21 -> 1.34 is the same news)."""
        key = tuple(sorted(set(tok.split("=")[0].split("(")[0].rstrip("0123456789.+-")
                               for tok in new_reason.split(",")))) if new_reason else None
        if key == self._new_banner_key:
            return
        self._new_banner_key = key
        if key is None:
            return
        rospy.logwarn("drift_monitor: new-check WARN: %s", new_reason)
        self._send_statustext(_SEV_WARNING, "SLAM chk: " + new_reason, force=True)

    def _new_checks_summary(self):
        """Fifth /slam/drift_risk segment (2026-10-07); segments 0-3 unchanged."""
        sp = "n/a" if self._ekf.last is None else "%.2f" % self._ekf.last
        return "res=%.3f(avg %.3f) sp=%s%s" % (self._fl_res.last, self._fl_res.avg(), sp,
                                               " BENCH" if self._bench else "")

    def _on_transition(self, old, new, reason, eig, matched, lat_only=False):
        rospy.logwarn("drift_monitor: %s -> %s (%s) eig=%.0f matched=%.0f", old, new, reason, eig, matched)
        # WARN is published to /slam/drift_risk only (no GCS banner -- banner is reserved for the
        # restart messages), EXCEPT for checks (4)-(6): those are observe-only by default, so the
        # banner from _announce_new_checks is the only thing the pilot gets. CRITICAL drives the response.
        if new != CRITICAL:
            return
        if lat_only:
            # latency-only CRITICAL: lane-change to OF and let the backlog drain. Only escalate
            # to a node restart if it is still bad after lat_drain_sec (checked in health_cb).
            self._execute_latency_failover(reason)
        elif self.enable_drift_land:
            self._execute_restart("drift:%s" % reason)

    # ---- CH9 manual trigger (edge into the ~1500 band) ----
    def rc_cb(self, msg):
        idx = self.ch9_channel - 1
        if idx < 0 or idx >= len(msg.channels):
            return
        v = msg.channels[idx]
        if v == 0:           # 0 = channel not present / no signal; ignore
            return
        in_band = self.ch9_low <= v <= self.ch9_high
        if self.ch9_in_band is None:               # first reading: init only, never fire (no dwell start)
            self.ch9_in_band = in_band
            return
        now = time.time()
        if in_band:
            if not self.ch9_in_band:               # just ENTERED middle -> start dwell timer
                self.ch9_in_band = True
                self.ch9_mid_since = now
                self.ch9_land_fired = False
            elif (self.ch9_mid_since is not None and not self.ch9_land_fired
                  and (now - self.ch9_mid_since) >= self.ch9_dwell):   # HELD middle long enough -> fire once
                self.ch9_land_fired = True
                rospy.logwarn("drift_monitor: CH%d held middle >=%.1fs -> restart (%s)", self.ch9_channel, self.ch9_dwell, self.restart_mode)
                self._execute_restart("CH%d-hold" % self.ch9_channel)
        else:                                       # left band (transit, or to HIGH/LOW) -> re-arm, no fire
            self.ch9_in_band = False
            self.ch9_mid_since = None

    # ---- RC stream keepalive: (re)request RC_CHANNELS from the FC on each mavros connect ----
    def state_cb(self, msg):
        self._armed = bool(msg.armed)
        self._motion.set_state(time.time(), getattr(msg, "mode", ""), msg.armed)
        if msg.connected and not self._mav_connected:
            self._mav_connected = True
            if self.request_rc_stream:
                self._request_rc_stream()   # link (re)established -> ensure RC is streaming
        elif not msg.connected:
            self._mav_connected = False

    def _request_rc_stream(self):
        # STREAM_RC_CHANNELS=3. Without this /mavros/rc/in is empty and the CH9 trigger is blind.
        try:
            rospy.wait_for_service("/mavros/set_stream_rate", timeout=5.0)
            rospy.ServiceProxy("/mavros/set_stream_rate", StreamRate)(
                stream_id=3, message_rate=self.rc_stream_rate, on_off=True)
            rospy.loginfo("drift_monitor: requested RC_CHANNELS stream @%dHz (keeps CH9 alive)", self.rc_stream_rate)
        except Exception as e:
            rospy.logwarn("drift_monitor: RC stream request failed (CH9 may be blind): %s", e)

    def _spawn(self, fn, *args):
        """Run a blocking FC service call off the callback thread.

        wait_for_service blocks up to 3 s. Doing that inside health_cb made
        time.time()-_last_health_t exceed reset_gap (1.5 s), firing a spurious 'SLAM restarted'
        re-warm that cleared the baseline and set _saw_gap_since_lock without a restart having
        happened. Latency triggers make that path far more reachable, so it is fixed here.
        """
        threading.Thread(target=fn, args=args, daemon=True).start()

    # ---- latency-only response: source switch, NO node restart (yet) ----
    def _execute_latency_failover(self, reason):
        rospy.logwarn("drift_monitor: LATENCY failover (%s) -> OF only, no restart", reason)
        self._restart_lock = True
        self._refresh_interlock()
        self.n_ok = 0
        self._saw_gap_since_lock = False
        self._lat_no_restart = True
        self._lat_escalated = False
        self._lat_failover_at = time.time()
        self._send_statustext(_SEV_CRITICAL, "SLAM latency high, switching to OF", force=True)
        # Both modes. "land" used to skip this, and the 5 s drain was then flown on stale SLAM
        # height: that is the FC log 233 climb (see restart_mode in __init__).
        if self.enable_source_failover and not self.failover_active:
            self.failover_active = True     # optimistic; the worker clears it if the FC refuses
            self._spawn(self._failover_worker)
        elif not self.enable_source_failover:
            rospy.logwarn("drift_monitor: source failover disabled -> no source switch for latency")

    def _failover_worker(self):
        ok = self._set_ekf_source(self.src2_switch_pos, "OF (SRC2)")
        self.failover_active = ok
        self._send_statustext(_SEV_CRITICAL,
                              "Switched to OF (SRC2)" if ok else "SRC2 switch FAILED", force=True)

    # ---- shared response (drift CRITICAL, CH9, and latency escalation; branches on restart_mode) ----
    def _execute_restart(self, source):
        manual = source.startswith("CH")
        now = time.time()
        cooling = (not manual
                   and self._last_restart_at is not None
                   and (now - self._last_restart_at) < self.restart_cooldown_sec)

        if cooling:
            # Hold the failover, decline to kill SLAM again. _lat_no_restart tells the interlock
            # not to wait for a health gap that is never going to arrive, and the latency blank is
            # deliberately NOT re-armed: there is no outage of our own making to blind it to.
            self._restart_lock = True
            self._lat_no_restart = True
            self._refresh_interlock()
            if not self._cooldown_announced:
                self._cooldown_announced = True
                rospy.logwarn("drift_monitor: %s came back %.1fs after the last restart "
                              "(cooldown %.0fs) -- holding on OF, not restarting again",
                              source, now - self._last_restart_at, self.restart_cooldown_sec)
                self._send_statustext(_SEV_CRITICAL, "Still degraded, holding on OF", force=True)
            self._suppress_node_kill = True
            self._spawn(self._restart_worker)
            return

        self._cooldown_announced = False
        self._last_restart_at = now
        self._suppress_node_kill = False
        rospy.logwarn("drift_monitor: RESTART (%s) triggered by %s", self.restart_mode, source)
        self._restart_lock = True
        self._refresh_interlock()
        self.n_ok = 0; self._saw_gap_since_lock = False   # require fresh post-restart recovery before release
        self._lat_no_restart = False                      # a real restart WILL produce a health gap
        # We are about to kill laserMapping ourselves. Arm the blank now, not when health
        # resumes, so neither the watchdog nor the latency detector treats our own outage and
        # the cold start that follows it as a fresh fault.
        self._lat_blank_until = time.time() + self.lat_restart_blank_sec
        self._health_wd_fired = True                      # suppress the watchdog for this outage
        # MSG 1/4: restart triggered (source-dependent text)
        self._send_statustext(_SEV_CRITICAL,
                              "Restart triggered manually" if manual else "Drift detected, restart triggered",
                              force=True)
        # The FC action and the node kill must stay ORDERED - source switch FIRST, then drop
        # vision - so they run together on one worker thread rather than being split.
        self._spawn(self._restart_worker)

    def _restart_worker(self):
        # MSG 2/4: mode-specific FC action.
        if self.restart_mode == "land":
            # SRC2 FIRST, so LAND descends on rangefinder height rather than on the SLAM that just
            # failed, then LAND, then restart SLAM. If the switch is refused, LAND anyway: that is
            # the old behaviour, and still better than not landing. Never switched back to SRC1.
            if self.enable_source_failover and not self.failover_active:
                ok = self._set_ekf_source(self.src2_switch_pos, "OF (SRC2)")
                self.failover_active = ok
                self._send_statustext(_SEV_CRITICAL,
                                      "Switched to OF (SRC2)" if ok else "SRC2 switch FAILED", force=True)
            ok = self._command_land()
            self._send_statustext(_SEV_CRITICAL, "LANDING" if ok else "LAND cmd FAILED", force=True)
        else:
            # "of": lane-change to Optical Flow (SRC2) INSTEAD OF LANDing, so the vehicle keeps flying
            # on OF while laserMapping re-inits. Switch the source FIRST, then drop vision (restart),
            # so the EKF never loses its position feed. Switched back SRC2->SRC1 on recovery (health_cb).
            if self.enable_source_failover and not self.failover_active:
                ok = self._set_ekf_source(self.src2_switch_pos, "OF (SRC2)")
                self.failover_active = ok
                self._send_statustext(_SEV_CRITICAL,
                                      "Switched to OF (SRC2)" if ok else "SRC2 switch FAILED", force=True)
        if self.enable_auto_restart and not self._suppress_node_kill:
            rospy.logwarn("drift_monitor: restarting [%s] (rosnode kill -> roslaunch respawns)", self.restart_nodes)
            try:
                subprocess.Popen(["rosnode", "kill"] + self.restart_nodes.split())
            except Exception as e:
                rospy.logerr("drift_monitor: restart failed: %s", e)
        # MSG 3/4: restart in progress
        self._send_statustext(_SEV_CRITICAL, "Restart in progress", force=True)

    def _command_land(self):
        # restart_mode="land": set the FC flight mode to LAND via /mavros/set_mode (QRLand by number
        # on a Rover-based FC, land_command()). This is a
        # FLIGHT-MODE CHANGE (unlike the OF source switch). Benign while disarmed (no motion);
        # commands a real descent when flying. Does NOT arm. Gated entirely by restart_mode.
        if not _HAVE_MAVROS:
            rospy.logerr("drift_monitor: restart_mode=land but mavros_msgs unavailable"); return False
        kind, mode = land_command(self._fc_mav_type, self.land_mode, self.land_mode_rover)
        if kind == 'custom':
            # QuadRover: QRLand by number, MAV_CMD_DO_SET_MODE (176), param1 1 = custom mode
            rospy.logwarn("drift_monitor: *** commanding FC custom mode %d (QRLand) ***", mode)
            try:
                rospy.wait_for_service("/mavros/cmd/command", timeout=3.0)
                resp = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)(
                    broadcast=False, command=176, confirmation=0, param1=1.0, param2=float(mode),
                    param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0)
                ok = bool(getattr(resp, "success", False))
                if not ok:
                    rospy.logerr("drift_monitor: custom mode %d REJECTED by FC (result=%s)",
                                 mode, getattr(resp, "result", "?"))
                return ok
            except Exception as e:
                rospy.logerr("drift_monitor: custom mode %d failed: %s", mode, e); return False
        rospy.logwarn("drift_monitor: *** commanding FC set_mode -> %s ***", self.land_mode)
        try:
            rospy.wait_for_service("/mavros/set_mode", timeout=3.0)
            resp = rospy.ServiceProxy("/mavros/set_mode", SetMode)(base_mode=0, custom_mode=self.land_mode)
            ok = bool(getattr(resp, "mode_sent", False))
            if not ok:
                rospy.logerr("drift_monitor: set_mode %s REJECTED by FC", self.land_mode)
            return ok
        except Exception as e:
            rospy.logerr("drift_monitor: set_mode %s failed: %s", self.land_mode, e); return False

    def _set_ekf_source(self, position, label):
        # MAV_CMD_DO_AUX_FUNCTION (218): param1 = aux function (90 = EKF Source Set),
        # param2 = switch position (0=LOW->SRC1, 1=MIDDLE->SRC2, 2=HIGH->SRC3). Works even with
        # no RC channel assigned to the function. Does NOT arm and does NOT change flight mode.
        if not _HAVE_MAVROS:
            rospy.logerr("drift_monitor: source_failover set but mavros_msgs unavailable"); return False
        rospy.logwarn("drift_monitor: *** commanding FC EKF source -> %s (aux %d pos %d) ***",
                      label, self.ekf_src_auxfn, position)
        try:
            rospy.wait_for_service("/mavros/cmd/command", timeout=3.0)
            resp = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)(
                broadcast=False, command=218, confirmation=0,
                param1=float(self.ekf_src_auxfn), param2=float(position),
                param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0)
            ok = bool(getattr(resp, "success", False))
            if not ok:
                rospy.logerr("drift_monitor: EKF source switch to %s REJECTED (result=%s)",
                             label, getattr(resp, "result", "?"))
            return ok
        except Exception as e:
            rospy.logerr("drift_monitor: EKF source switch failed: %s", e); return False

    # ---- pub helpers ----
    def _publish_risk(self, level, reason, eig, matched, avg_eig, avg_matched,
                      lat_ms=None, lat_note="", extra=""):
        # Fourth ' | ' segment appended 2026-09-01, fifth (extra) 2026-10-07. Segments 0-2 are
        # unchanged in form, so any consumer splitting on ' | ' and reading [0..2] is unaffected.
        if not self.enable_latency_check:
            lat = "lat=off"
        elif lat_ms is None:
            lat = "lat=%s" % (lat_note or "n/a")
        else:
            lat = "lat=%.0fms%s" % (lat_ms, "(stale)" if lat_note == "stale" else "")
        self.risk_pub.publish(String(data="%s | %s | eig=%.0f(avg %.0f) matched=%.0f(avg %.0f) | %s%s"
                                          % (level, reason, eig, avg_eig, matched, avg_matched, lat,
                                             (" | " + extra) if extra else "")))

    def _refresh_interlock(self):
        want = "LOCKED" if (self._restart_lock or self._lat_lock) else "RELEASED"
        if want != self.interlock:
            self._publish_interlock(want)

    def _publish_interlock(self, state):
        self.interlock = state
        self.interlock_pub.publish(String(data=state))

    def _send_statustext(self, severity, text, force=False):
        if self.st_pub is None:
            return
        now = rospy.Time.now()
        if not force and (now - self._last_st).to_sec() < self.statustext_period:
            return
        self._last_st = now
        self.st_pub.publish(StatusText(severity=severity, text=text[:49]))


if __name__ == "__main__":
    try:
        DriftMonitor(); rospy.spin()
    except rospy.ROSInterruptException:
        pass
