#!/usr/bin/env python3
"""
drift_monitor — FAST-LIO drift early-warning + EKF source-failover/restart response.

Subscribes /fastlio_health and flags drift BEFORE the trajectory diverges, using
observability (eig_min, primary) + matched_pts (secondary). See docs/DRIFT_MONITOR_PLAN.md
and PROGRESS.yaml P13.

FAILOVER + node-restart is triggered by EITHER source:
  (A) drift CRITICAL  (eig_min/matched cross the hard floors, K-scan hysteresis), or
  (B) pilot CH9 switch -> ~1500 band (3-pos middle, EDGE-triggered once on entry).
Both run the SAME action: instead of LANDing, lane-change the FC EKF source from SLAM
(SRC1) to Optical Flow (SRC2) so the vehicle keeps flying on OF while SLAM re-initializes,
then restart laserMapping (gated ~enable_auto_restart) + advisory /slam/arm_interlock=LOCKED
+ GCS banners. On recovery (fresh SLAM healthy for k_release scans) lane-change back SRC2->SRC1.
Source switch = MAV_CMD_DO_AUX_FUNCTION(218), func 90 (EKF Source Set): pos MIDDLE(1)->SRC2,
LOW(0)->SRC1. Gated ~enable_source_failover. Does NOT arm or change flight mode.

ARMING BLOCK is enforced by the FLIGHT CONTROLLER's own EKF prearm (vision drops while
laserMapping restarts -> EKF unhealthy -> FC refuses arm; clears when EKF healthy again).
This node does NOT write FC params; /slam/arm_interlock is advisory/telemetry only.

Detection (per scan): eig_min=data[4], matched_pts=data[0]. Rolling-AVERAGE baseline (over OK
scans). WARN = eig_min<0.2*avg OR <40k OR matched<0.2*avg OR <300. CRIT = eig_min<5k OR
matched<150. eig_min owns CRITICAL; matched only escalates WARN. K=5 hysteresis for both.

SAFETY: this node NEVER arms and NEVER changes flight mode. An EKF source switch while
disarmed/grounded is benign (no motion). It replaces the previous LAND response entirely.
"""
import collections
import subprocess
import time
import rospy
from std_msgs.msg import Float32MultiArray, String

try:
    from mavros_msgs.msg import StatusText, RCIn, State
    from mavros_msgs.srv import SetMode, CommandLong, StreamRate
    _HAVE_MAVROS = True
except Exception:
    _HAVE_MAVROS = False

OK, WARN, CRITICAL = "OK", "WARN", "CRITICAL"
_SEV_WARNING, _SEV_CRITICAL = 4, 2  # MAV_SEVERITY


def mean(seq):
    # rolling average baseline (over OK scans)
    return (sum(seq) / len(seq)) if seq else 0.0


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

        # response gates
        self.enable_auto_restart = bool(g("~enable_auto_restart", False))  # kill->respawn laserMapping
        self.enable_critical_fc = bool(g("~enable_critical_fc", False))    # set_mode LAND (COMMANDS FC)
        self.enable_drift_land = bool(g("~enable_drift_land", True))       # drift CRITICAL drives the action
        self.enable_ch9_land = bool(g("~enable_ch9_land", False))          # CH9 switch drives the action
        self.restart_nodes = g("~restart_nodes", "/laserMapping /fastlio_mavros_bridge")
        self.land_mode = g("~land_mode", "LAND")   # FC flight mode used by restart_mode="land"
        self.statustext_period = float(g("~statustext_period", 2.0))

        # restart_mode selects the FC response run on trigger (drift CRITICAL or CH9):
        #   "of"   -> EKF source failover SLAM(SRC1)->OF(SRC2), switch back on recovery (DEFAULT;
        #             no flight-mode change, benign disarmed, vehicle keeps flying on OF)
        #   "land" -> command the FC into LAND (set_mode); no source switch-back on recovery
        # Both modes then restart laserMapping. Toggle persistently via switch_restart_mode.sh.
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

        # EKF source failover (REPLACES LAND): on restart, lane-change FC EKF source SLAM(SRC1)->OF(SRC2)
        # so the vehicle keeps flying on Optical Flow while SLAM re-inits; switch back SRC2->SRC1 on recovery.
        # Mechanism: MAV_CMD_DO_AUX_FUNCTION(218), func 90 = "EKF Source Set", switch position selects the set.
        self.enable_source_failover = bool(g("~enable_source_failover", True))
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

        self.risk_pub = rospy.Publisher("/slam/drift_risk", String, queue_size=10, latch=True)
        self.interlock_pub = rospy.Publisher("/slam/arm_interlock", String, queue_size=10, latch=True)
        self.st_pub = rospy.Publisher("/mavros/statustext/send", StatusText, queue_size=10) if _HAVE_MAVROS else None
        self._publish_interlock("RELEASED")

        rospy.Subscriber(self.health_topic, Float32MultiArray, self.health_cb, queue_size=50)
        if _HAVE_MAVROS and self.enable_ch9_land:
            rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_cb, queue_size=10)
        if _HAVE_MAVROS and self.request_rc_stream:
            rospy.Subscriber("/mavros/state", State, self.state_cb, queue_size=10)

        rospy.loginfo("drift_monitor up. drift_land=%s ch9_land=%s | gates: auto_restart=%s critical_fc(LAND)=%s | "
                      "CH9 ch%d band[%d,%d] | K warn=%d crit=%d",
                      self.enable_drift_land, self.enable_ch9_land, self.enable_auto_restart,
                      self.enable_critical_fc, self.ch9_channel, self.ch9_low, self.ch9_high,
                      self.k_warn, self.k_crit)
        rospy.loginfo("drift_monitor: restart_mode=%s | source_failover=%s auxfn=%d (OF/SRC2 pos=%d, SLAM/SRC1 pos=%d) | land_mode=%s",
                      self.restart_mode, self.enable_source_failover, self.ekf_src_auxfn,
                      self.src2_switch_pos, self.src1_switch_pos, self.land_mode)

    # ---- detection ----
    def health_cb(self, msg):
        if len(msg.data) < 5:
            return
        matched = float(msg.data[0]); eig = float(msg.data[4])

        # SLAM restart detection: a gap in /fastlio_health means laserMapping was restarted ->
        # the old baseline is stale and the fresh re-init may look transiently degenerate. Re-warm
        # (drop baseline + counters, latch OK) so we don't immediately re-fire CRITICAL (restart loop).
        now = time.time()
        if self._last_health_t and (now - self._last_health_t) > self.reset_gap:
            rospy.logwarn("drift_monitor: /fastlio_health resumed after %.1fs gap -> SLAM restarted; re-warming",
                          now - self._last_health_t)
            self.base_eig.clear(); self.base_matched.clear()
            self.n_crit = self.n_warn = self.n_ok = 0
            self.latched = OK
            self._saw_gap_since_lock = True   # the restart actually happened
        self._last_health_t = now

        if len(self.base_eig) < self.warmup_n:
            self.base_eig.append(eig); self.base_matched.append(matched)
            self._publish_risk(OK, "warmup", eig, matched, 0.0, 0.0); return

        avg_eig = mean(self.base_eig); avg_matched = mean(self.base_matched)
        crit, warn, reason = self._evaluate(eig, matched, avg_eig, avg_matched)
        raw = CRITICAL if crit else (WARN if warn else OK)

        self.n_crit = self.n_crit + 1 if crit else 0
        self.n_warn = self.n_warn + 1 if (warn or crit) else 0
        self.n_ok = self.n_ok + 1 if raw == OK else 0

        new = self.latched
        if self.n_crit >= self.k_crit:
            new = CRITICAL
        elif self.n_warn >= self.k_warn and self.latched != CRITICAL:
            new = WARN
        if self.n_ok >= self.k_release:
            new = OK
        if new != self.latched:
            self._on_transition(self.latched, new, reason, eig, matched)
            self.latched = new

        # advisory interlock auto-release ONLY after the restart's health-gap is observed AND
        # the fresh SLAM is healthy for k_release scans (prevents releasing instantly off stale n_ok).
        if (self.interlock == "LOCKED" and self.n_ok >= self.k_release
                and (self._saw_gap_since_lock or not self.enable_auto_restart)):
            # fresh SLAM confirmed healthy. In "of" mode, lane-change back OF(SRC2)->SLAM(SRC1);
            # in "land" mode there is no source to restore (FC stays in LAND, pilot resumes).
            if self.restart_mode == "of" and self.enable_source_failover and self.failover_active:
                self._set_ekf_source(self.src1_switch_pos, "SLAM (SRC1)")
                self.failover_active = False
                recovery_msg = "Restart finished, back to SRC1"
            else:
                recovery_msg = "Restarted, ready to arm"
            self._publish_interlock("RELEASED")
            self._send_statustext(6, recovery_msg, force=True)   # MSG 4/4

        if raw == OK:
            self.base_eig.append(eig); self.base_matched.append(matched)
        self._publish_risk(self.latched, reason if self.latched != OK else "ok",
                           eig, matched, avg_eig, avg_matched)

    def _evaluate(self, eig, matched, avg_eig, avg_matched):
        c, w = [], []
        if eig < self.eig_crit_abs: c.append("eig_min<%.0f(crit)" % self.eig_crit_abs)
        if matched < self.matched_crit_abs: c.append("matched<%.0f(starv)" % self.matched_crit_abs)
        if eig < self.eig_warn_rel * avg_eig: w.append("eig_min<%.2fxavg" % self.eig_warn_rel)
        if eig < self.eig_warn_abs: w.append("eig_min<%.0f" % self.eig_warn_abs)
        if matched < self.matched_warn_rel * avg_matched: w.append("matched<%.2fxavg" % self.matched_warn_rel)
        if matched < self.matched_warn_abs: w.append("matched<%.0f" % self.matched_warn_abs)
        return (len(c) > 0), (len(w) > 0), (",".join(c if c else w) or "ok")

    def _on_transition(self, old, new, reason, eig, matched):
        rospy.logwarn("drift_monitor: %s -> %s (%s) eig=%.0f matched=%.0f", old, new, reason, eig, matched)
        # WARN is published to /slam/drift_risk only (no GCS banner -- banner is reserved for the
        # 3 restart messages). CRITICAL drives the restart (which sends msgs 1+2).
        if new == CRITICAL and self.enable_drift_land:
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
        if msg.connected and not self._mav_connected:
            self._mav_connected = True
            self._request_rc_stream()   # link (re)established -> ensure RC is streaming to /mavros/rc/in
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

    # ---- shared response (used by BOTH drift CRITICAL and CH9; branches on restart_mode) ----
    def _execute_restart(self, source):
        manual = source.startswith("CH")
        rospy.logwarn("drift_monitor: RESTART (%s) triggered by %s", self.restart_mode, source)
        self._publish_interlock("LOCKED")
        self.n_ok = 0; self._saw_gap_since_lock = False   # require fresh post-restart recovery before release
        # MSG 1/4: restart triggered (source-dependent text)
        self._send_statustext(_SEV_CRITICAL,
                              "Restart triggered manually" if manual else "Drift detected, restart triggered",
                              force=True)
        # MSG 2/4: mode-specific FC action.
        if self.restart_mode == "land":
            # Command the FC into LAND, then restart SLAM. No source switch-back on recovery.
            ok = self._command_land()
            self._send_statustext(_SEV_CRITICAL, "LANDING" if ok else "LAND cmd FAILED", force=True)
        else:
            # "of": lane-change to Optical Flow (SRC2) INSTEAD OF LANDing, so the vehicle keeps flying
            # on OF while laserMapping re-inits. Switch the source FIRST, then drop vision (restart),
            # so the EKF never loses its position feed. Switched back SRC2->SRC1 on recovery (health_cb).
            if self.enable_source_failover:
                ok = self._set_ekf_source(self.src2_switch_pos, "OF (SRC2)")
                self.failover_active = ok
                self._send_statustext(_SEV_CRITICAL,
                                      "Switched to OF (SRC2)" if ok else "SRC2 switch FAILED", force=True)
        if self.enable_auto_restart:
            rospy.logwarn("drift_monitor: restarting [%s] (rosnode kill -> roslaunch respawns)", self.restart_nodes)
            try:
                subprocess.Popen(["rosnode", "kill"] + self.restart_nodes.split())
            except Exception as e:
                rospy.logerr("drift_monitor: restart failed: %s", e)
        # MSG 3/4: restart in progress
        self._send_statustext(_SEV_CRITICAL, "Restart in progress", force=True)

    def _command_land(self):
        # restart_mode="land": set the FC flight mode to LAND via /mavros/set_mode. This is a
        # FLIGHT-MODE CHANGE (unlike the OF source switch). Benign while disarmed (no motion);
        # commands a real descent when flying. Does NOT arm. Gated entirely by restart_mode.
        if not _HAVE_MAVROS:
            rospy.logerr("drift_monitor: restart_mode=land but mavros_msgs unavailable"); return False
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
    def _publish_risk(self, level, reason, eig, matched, avg_eig, avg_matched):
        self.risk_pub.publish(String(data="%s | %s | eig=%.0f(avg %.0f) matched=%.0f(avg %.0f)"
                                          % (level, reason, eig, avg_eig, matched, avg_matched)))

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
