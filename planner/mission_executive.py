#!/usr/bin/env python3
"""Mission executive for L4 autonomy (D6, D5, D7): flies and drives a YAML mission on the QuadRover.

The pilot arms (D5: this node never arms, disarms or sets parameters). On /mission_executive/start
(std_srvs/Trigger) with the vehicle armed it switches to GUIDED, takes off with Copter's NAV_TAKEOFF,
then works through the mission's legs:

  fly      goal [x, y, z] in the FC frame -> /planning/click_goal for SUPER (planner frame); the
           bridge in flight mode
  drive    goal [x, y] on the ground      -> /planning/drive_goal for the track planner, the bridge
           in drive mode (Rover Guided position targets); only after a land leg, or as the first
           legs of a mission that starts on the ground. Done when the FC is within accept_radius
           or the track planner reports REACHED for this goal
  land     at [x, y]: fly there, ask the landing check (/planner/landing_spot) and require it to
           report the spot clear within ~landing_check_s (a not-clear answer is asked again until
           then: a person walking off clears it), then NAV_LAND (lands and drives on in GUIDED)
  takeoff  height above the liftoff point (NAV_TAKEOFF in GUIDED). The executive first waits for
           the vehicle to stand still (< 2 cm over 1 s) for up to ~takeoff_settle_s (15 s; after a drive
           Rover Guided runs on to its last target for a few seconds). Not still by then: the mission
           stops, "not stationary before takeoff" (owner decision 2026-10-28), in GUIDED with the bridge
           off, no takeoff sent (QuadRover's own 0.2 m/s guard is not relied on). Still: NAV_TAKEOFF,
           repeated while QuadRover answers MAV_RESULT_TEMPORARILY_REJECTED, for up to
           ~takeoff_settle_s

A mission that starts with a fly or land leg gets a takeoff (takeoff_height) first; one that starts
with a drive leg drives first.

The bridge's mode (flight | drive) is set on /planner_bridge/set_mode (latched) before each leg
streams, and the executive waits for the bridge to report it (/planner_bridge/status "mode") before
enabling it, so a target from one planner is never sent in the other mode.

Frames (shared contract): waypoints are in "fc", the FC's EKF frame as MAVROS gives it (local ENU,
metres), which survives SLAM restarts (D7). SUPER plans in "planner" (FAST-LIO rotated z-up); the
planner bridge publishes the fc<-planner transform on /planner_bridge/anchor and its state on
/planner_bridge/status (JSON). Every goal is converted fc->planner with the latest anchor.

Failures:
  * no progress (not 0.2 m closer within no_progress_s; on a drive leg also not ~drive_progress_m driven)
    or leg timeout (goal unreachable) -> QRLOITER (in the air), hold, then the next leg
    (~on_unreachable: next) or stop there (stop)
  * bridge HOLD for a SLAM reset or an anchor jump -> stop sending goals, wait for the bridge to
    report OK plus ~map_rebuild_s, re-convert the goal with the new anchor and resend it (D7)
  * on the ground (driving) nothing is held in QRLOITER: entering a QR mode armed on the ground
    starts QuadRover's drive-to-fly transition (props spool up). There an unreachable drive goal,
    a refused takeoff or a stop leaves the vehicle in GUIDED with the bridge disabled (Rover Guided
    stops at its last target, at most ~drive_lookahead ahead)
  * the FC leaves GUIDED on its own (battery or link-loss failsafe, the pilot) or MAVROS loses the
    FC -> stop commanding for good; the FC's failsafe owns the vehicle
  * landing spot not reported clear within ~landing_check_s -> QRLOITER, mission stopped
  * a takeoff refused or not completed -> mission stopped (no later leg can fly); a refused takeoff
    leaves it on the ground in GUIDED, one that started and did not complete goes to QRLOITER
  * not standing still within ~takeoff_settle_s before a takeoff -> mission stopped on the ground,
    GUIDED, bridge off, no takeoff sent

Ground spots (owner decision 2026-10-28): where the vehicle sat on the ground is known floor for the
landing check (its own body hides the floor under it from the lidar). Each spot is the FC pose
(/mavros/local_position/pose: the odometry point, fc frame, with its yaw) when standing still on the
ground in GUIDED just before a NAV_TAKEOFF ("takeoff"), and when a landing completes ("landed"). All of
the mission's spots so far go out on /mission_executive/ground_spots (std_msgs/String, latched, JSON
{"frame": "fc", "spots": [{"event", "xyz", "yaw", "t"}]}); landing_check.py converts them to the
planner frame with /planner_bridge/anchor.

The planner bridge streams targets only while /planner_bridge/enable (latched) is true: this node
sets it for its fly and drive legs and clears it before every takeoff, landing, mode change and stop,
so a target never overrides the FC's own NAV_TAKEOFF or NAV_LAND.

Mode changes are only GUIDED, QRLOITER, QRLAND, QRSMARTRTL (D5), sent as MAV_CMD_DO_SET_MODE with
QuadRover's custom mode numbers (MAVROS has no names for the QR modes and reports them as CMODE(n)).
"""
import json
import math
import re
import threading

import numpy as np
import rospy
import yaml
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
from mavros_msgs.msg import State, StatusText
from mavros_msgs.srv import CommandLong, CommandTOL
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse

# QuadRover custom modes (Rover/mode.h)
MODES = {"MANUAL": 0, "HOLD": 4, "AUTO": 10, "RTL": 11, "SMART_RTL": 12, "GUIDED": 15,
         "QRSTABILIZE": 17, "QRALTHOLD": 18, "QRLOITER": 19, "QRRTL": 20, "QRLAND": 21,
         "QRCIRCLE": 22, "QRSMARTRTL": 23}
ALLOWED_MODES = {"GUIDED", "QRLOITER", "QRLAND", "QRSMARTRTL"}   # D5
NOT_STATIONARY = "not stationary before takeoff"
MAV_CMD_DO_SET_MODE = 176
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1


def mode_number(name):
    """MAVROS's mode string to QuadRover's number: a known name, or CMODE(n) for the QR modes"""
    m = re.fullmatch(r"CMODE\((\d+)\)", name or "")
    if m:
        return int(m.group(1))
    return MODES.get(name)


# what an unreachable goal does (owner 2026-10-27: next, may change): set per mission under defaults,
# per leg, or with ~on_unreachable for missions that set neither
UNREACHABLE_POLICIES = ("next", "stop")


def load_mission(path):
    """the D6 mission: defaults filled in per leg; raises ValueError on a malformed file"""
    with open(path) as f:
        doc = yaml.safe_load(f)
    if doc.get("frame", "fc") != "fc":
        raise ValueError("mission waypoints must be in the fc frame (D7)")
    d = {"speed": 0.5, "accept_radius": 0.5, "leg_timeout_s": 90.0, "no_progress_s": 20.0, "hold_s": 0.0}
    d.update(doc.get("defaults", {}))
    legs = []
    for i, leg in enumerate(doc["legs"]):
        kind = leg.get("type")
        if kind not in ("fly", "drive", "land", "takeoff"):
            raise ValueError("leg %d: unknown type %r" % (i, kind))
        full = dict(d)
        full.update(leg)
        if kind == "fly" and len(full.get("goal", [])) != 3:
            raise ValueError("leg %d: fly needs goal [x, y, z]" % i)
        if kind == "drive" and len(full.get("goal", [])) < 2:
            raise ValueError("leg %d: drive needs goal [x, y]" % i)
        if kind == "land" and len(full.get("at", [])) < 2:
            raise ValueError("leg %d: land needs at [x, y]" % i)
        if kind == "takeoff" and "height" not in full:
            raise ValueError("leg %d: takeoff needs height" % i)
        if full.get("on_unreachable", "next") not in UNREACHABLE_POLICIES:
            raise ValueError("leg %d: on_unreachable must be one of %s" % (i, ", ".join(UNREACHABLE_POLICIES)))
        legs.append(full)
    return {"name": doc.get("name", path), "takeoff_height": float(doc.get("takeoff_height", 1.0)), "legs": legs}


class Anchor:
    """fc <- planner: p_fc = R p_planner + t"""

    def __init__(self):
        self.R = np.eye(3)
        self.t = np.zeros(3)
        self.stamp = None

    def set(self, msg):
        q = msg.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        self.R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                           [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                           [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        tr = msg.transform.translation
        self.t = np.array([tr.x, tr.y, tr.z])
        self.stamp = msg.header.stamp

    def fc_to_planner(self, p):
        return self.R.T @ (np.asarray(p, dtype=float) - self.t)


class MissionExecutive:
    def __init__(self):
        self.mission = load_mission(rospy.get_param("~mission"))
        self.on_unreachable = rospy.get_param("~on_unreachable", "next")
        self.unreachable_hold_s = rospy.get_param("~unreachable_hold_s", 5.0)
        self.map_rebuild_s = rospy.get_param("~map_rebuild_s", 3.0)
        self.landing_check_s = rospy.get_param("~landing_check_s", 5.0)
        self.takeoff_timeout_s = rospy.get_param("~takeoff_timeout_s", 30.0)
        self.land_timeout_s = rospy.get_param("~land_timeout_s", 60.0)
        self.planner_frame = rospy.get_param("~planner_frame_id", "world")
        self.goal_resend_s = rospy.get_param("~goal_resend_s", 0.0)  # 0: send each goal once
        self.bridge_mode_timeout_s = rospy.get_param("~bridge_mode_timeout_s", 3.0)
        # after a drive Rover Guided runs on to its last target for a few seconds (cl32: ~6 s)
        self.takeoff_settle_s = rospy.get_param("~takeoff_settle_s", 15.0)
        # drive legs: this far driven (not closer) also counts as progress; more than a pivot on the spot moves the FC
        self.drive_progress_m = rospy.get_param("~drive_progress_m", 0.5)
        # the first fly leg after a takeoff: SUPER gets its goal this long before the bridge streams,
        # so its first commands are from the new plan
        self.goal_settle_s = rospy.get_param("~goal_settle_s", 1.0)
        self.lock = threading.Lock()
        self.state = None
        self.pose = None
        self.yaw = None           # the FC pose's yaw (fc frame)
        self.ground_spots = []    # where the vehicle sat on the ground (fc frame), for the landing check
        self.bridge = {"state": "OK", "slam_ok": True}
        self.bridge_ok_since = rospy.get_time()
        self.anchor = Anchor()
        self.texts = []
        self.landing_clear = None
        self.landing_status = {}
        self.track = {}           # the track planner's latest status
        self.track_goals = 0      # its "new goal" acknowledgements: a REACHED counts only after ours
        self.flying = False
        self.landed_once = False  # a takeoff after a landing re-seeds SUPER at the hover point
        self.streaming = False    # the bridge enabled by us
        self.bridge_mode_now = "flight"
        self.started = False
        self.our_mode = None      # the last mode we commanded
        self.phase = "WAIT_ARMED"
        rospy.Subscriber("/mavros/state", State, self._on_state, queue_size=10)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self._on_pose, queue_size=10)
        rospy.Subscriber("/mavros/statustext/recv", StatusText, self._on_text, queue_size=50)
        rospy.Subscriber("/planner_bridge/status", String, self._on_bridge, queue_size=10)
        rospy.Subscriber("/planner_bridge/anchor", TransformStamped, self._on_anchor, queue_size=10)
        rospy.Subscriber(rospy.get_param("~landing_check_topic", "/planner/landing_clear"), Bool,
                         self._on_landing_clear, queue_size=10)
        rospy.Subscriber("/planner/landing_status", String, self._on_landing_status, queue_size=10)
        rospy.Subscriber("/track_planner/status", String, self._on_track_status, queue_size=10)
        self.goal_pub = rospy.Publisher("/planning/click_goal", PoseStamped, queue_size=1, latch=True)
        self.drive_pub = rospy.Publisher("/planning/drive_goal", PoseStamped, queue_size=1, latch=True)
        self.leg_pub = rospy.Publisher("/mission_executive/leg", String, queue_size=1, latch=True)
        self.spot_pub = rospy.Publisher("/planner/landing_spot", PointStamped, queue_size=1)
        self.status_pub = rospy.Publisher("/mission_executive/status", String, queue_size=10, latch=True)
        self.enable_pub = rospy.Publisher("/planner_bridge/enable", Bool, queue_size=1, latch=True)
        self.enable_pub.publish(Bool(data=False))
        self.mode_pub = rospy.Publisher("/planner_bridge/set_mode", String, queue_size=1, latch=True)
        self.ground_pub = rospy.Publisher("/mission_executive/ground_spots", String, queue_size=1, latch=True)
        rospy.Service("/mission_executive/start", Trigger, self._on_start)
        self.cmd = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)
        self.takeoff_srv = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
        self.land_srv = rospy.ServiceProxy("/mavros/cmd/land", CommandTOL)

    # ---- inputs -------------------------------------------------------------------------------
    def _on_state(self, m):
        with self.lock:
            self.state = m

    def _on_pose(self, m):
        p = m.pose.position
        q = m.pose.orientation
        with self.lock:
            self.pose = np.array([p.x, p.y, p.z])
            self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _on_text(self, m):
        with self.lock:
            self.texts.append(m.text)

    def _on_bridge(self, m):
        try:
            s = json.loads(m.data)
        except ValueError:
            return
        # the bridge's own flag; without it (an older or fake sender) only HOLD counts
        s["slam_ok"] = not s.get("slam_hold", s.get("state") == "HOLD")
        with self.lock:
            if s["slam_ok"] and not self.bridge.get("slam_ok", True):
                self.bridge_ok_since = rospy.get_time()
            self.bridge = s

    def _on_anchor(self, m):
        with self.lock:
            self.anchor.set(m)

    def _on_landing_clear(self, m):
        with self.lock:
            self.landing_clear = bool(m.data)

    def _on_landing_status(self, m):
        try:
            s = json.loads(m.data)
        except ValueError:
            return
        with self.lock:
            self.landing_status = s

    def _on_track_status(self, m):
        try:
            s = json.loads(m.data)
        except ValueError:
            return
        with self.lock:
            self.track = s
            if s.get("state") == "DRIVING" and s.get("reason") == "new goal":
                self.track_goals += 1

    def _on_start(self, _req):
        with self.lock:
            st = self.state
        if self.phase != "WAIT_ARMED":
            return TriggerResponse(success=False, message="mission already %s" % self.phase)
        if st is None or not st.connected or not st.armed:
            return TriggerResponse(success=False, message="the pilot arms first (the executive never arms)")
        self.started = True
        return TriggerResponse(success=True, message="mission %s started" % self.mission["name"])

    # ---- helpers ------------------------------------------------------------------------------
    def report(self, **kw):
        self.phase = kw.get("phase", self.phase)
        kw["phase"] = self.phase
        self.status_pub.publish(String(data=json.dumps(kw)))
        rospy.loginfo("[mission] %s", json.dumps(kw))

    def mode(self):
        with self.lock:
            return mode_number(self.state.mode) if self.state else None

    def fc_lost(self):
        """True once the FC has taken the vehicle (mode left what we set) or MAVROS lost it"""
        with self.lock:
            st = self.state
        if st is None or not st.connected or not st.armed:
            return "FC link lost or disarmed"
        m = mode_number(st.mode)
        if self.our_mode is not None and m != MODES[self.our_mode]:
            return "FC mode changed to %s" % st.mode
        return None

    def bridge_enable(self, on):
        self.streaming = bool(on)
        self.enable_pub.publish(Bool(data=bool(on)))

    def bridge_mode(self, name):
        """the bridge's mode for the next leg (flight | drive); True once the bridge reports it (an
        older bridge that does not report a mode is taken at its word)"""
        self.bridge_mode_now = name
        self.mode_pub.publish(String(data=name))
        t0 = rospy.get_time()
        while not rospy.is_shutdown() and rospy.get_time() - t0 < self.bridge_mode_timeout_s:
            with self.lock:
                got = self.bridge.get("mode", name)
            if got == name:
                return True
            rospy.sleep(0.05)
        return False

    def standstill(self, timeout):
        """wait until the vehicle has stopped (< 2 cm over 1 s: QuadRover refuses a takeoff above
        0.2 m/s, and Rover Guided pauses and moves on again while it settles on its last target);
        False after timeout"""
        t0 = rospy.get_time()
        seen = []
        while not rospy.is_shutdown() and rospy.get_time() - t0 < timeout:
            with self.lock:
                p = None if self.pose is None else self.pose.copy()
            now = rospy.get_time()
            if p is not None:
                seen = [(t, q) for t, q in seen if now - t <= 1.5] + [(now, p)]
                last_s = [q for t, q in seen if now - t <= 1.0]
                if now - seen[0][0] >= 1.0 and max(np.linalg.norm(q[:2] - p[:2]) for q in last_s) < 0.02:
                    return True
            rospy.sleep(0.1)
        return False

    def record_ground_spot(self, event):
        """the vehicle sits here on the ground: known floor for the landing check (see the docstring)"""
        with self.lock:
            if self.pose is None or self.yaw is None:
                return
            spot = {"event": event, "xyz": [round(float(x), 3) for x in self.pose], "yaw": round(float(self.yaw), 4),
                    "t": round(rospy.get_time(), 2)}
        self.ground_spots.append(spot)
        self.ground_pub.publish(String(data=json.dumps({"frame": "fc", "spots": self.ground_spots})))

    def set_mode(self, name):
        if name not in ALLOWED_MODES:
            raise RuntimeError("D5: the executive may not set %s" % name)
        self.bridge_enable(False)
        self.our_mode = name
        r = self.cmd(broadcast=False, command=MAV_CMD_DO_SET_MODE, confirmation=0,
                     param1=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, param2=MODES[name],
                     param3=0, param4=0, param5=0, param6=0, param7=0)
        t0 = rospy.get_time()
        while not rospy.is_shutdown() and rospy.get_time() - t0 < 5.0:
            if self.mode() == MODES[name]:
                return True
            rospy.sleep(0.05)
        rospy.logwarn("[mission] mode %s not confirmed (command success %s)", name, r.success)
        return False

    def wait_text(self, text, timeout):
        t0 = rospy.get_time()
        while not rospy.is_shutdown() and rospy.get_time() - t0 < timeout:
            with self.lock:
                if any(text in x for x in self.texts):
                    return True
            if self.fc_lost():
                return False
            rospy.sleep(0.05)
        return False

    def publish_goal(self, p_fc, drive=False):
        with self.lock:
            p = self.anchor.fc_to_planner(p_fc)
        m = PoseStamped()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = self.planner_frame
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, p)
        m.pose.orientation.w = 1.0
        (self.drive_pub if drive else self.goal_pub).publish(m)
        return p

    # ---- legs ---------------------------------------------------------------------------------
    def takeoff(self, height):
        with self.lock:
            self.texts = []
        if self.streaming and self.bridge_mode_now == "drive":
            # after a drive: keep the bridge on until the tracks stop. Once the track planner has
            # reached its goal the bridge holds (streams nothing) and Rover Guided stops at the last
            # target, at most drive_lookahead ahead
            self.report(phase="TAKEOFF", height=height, waiting="the tracks to stop")
            still = self.standstill(self.takeoff_settle_s)
            if not still:
                self.bridge_enable(False)
                self.report(phase="TAKEOFF", height=height, refused=True, reason=NOT_STATIONARY)
                return NOT_STATIONARY
        self.bridge_enable(False)
        self.bridge_mode_now = "flight"
        self.mode_pub.publish(String(data="flight"))   # not streaming: no need to wait for it here
        self.report(phase="TAKEOFF", height=height)
        # owner decision 2026-10-28: no takeoff unless standing still (QuadRover's own guard refuses
        # above 0.2 m/s, but a slow creep passes it); not still within takeoff_settle_s: stop here
        if not self.standstill(self.takeoff_settle_s):
            self.report(phase="TAKEOFF", height=height, refused=True, reason=NOT_STATIONARY)
            return NOT_STATIONARY
        self.record_ground_spot("takeoff")
        # standing still: ask, and again while QuadRover says "temporarily rejected"
        t0 = rospy.get_time()
        tries = 0
        while True:
            tries += 1
            r = self.takeoff_srv(min_pitch=0, yaw=float("nan"), latitude=float("nan"), longitude=float("nan"), altitude=float(height))
            if r.success:
                break
            if r.result != 1 or rospy.get_time() - t0 > self.takeoff_settle_s or rospy.is_shutdown():
                self.report(phase="TAKEOFF", height=height, refused=True, result=int(r.result), tries=tries)
                return "takeoff refused"
            rospy.sleep(1.0)
        with self.lock:
            here = None if self.pose is None else self.pose.copy()
        if self.landed_once and here is not None:
            # SUPER still holds the goal it had before the landing (and has been replanning toward it
            # from the ground): give it the hover point this takeoff ends at, so its reference is
            # back at the vehicle when the next fly leg's goal comes
            self.publish_goal(np.array([here[0], here[1], here[2] + float(height)]))
        if not self.wait_text("Guided takeoff complete", self.takeoff_timeout_s):
            return self.fc_lost() or "takeoff did not complete"
        self.flying = True
        return None

    def go(self, leg, idx, target, drive):
        """send the goal and watch it; returns None when reached, or why not"""
        kind = "drive" if drive else "fly"
        mode = "drive" if drive else "flight"
        if not self.bridge_mode(mode):
            return "bridge did not switch to %s mode" % mode
        self.leg_pub.publish(String(data=json.dumps({"index": idx, "type": kind, "speed": leg["speed"]})))
        with self.lock:
            goals_before = self.track_goals
        planner = self.publish_goal(target, drive)
        if not drive and not self.streaming and self.goal_settle_s > 0:
            rospy.sleep(self.goal_settle_s)     # SUPER plans the new goal before anything streams
        self.bridge_enable(True)
        self.report(phase="LEG", leg=idx, type=kind, bridge_mode=mode, goal_fc=list(map(float, target)),
                    goal_planner=list(map(float, planner)))
        t0 = last_progress_t = rospy.get_time()
        best = None
        moved_from = None     # drive legs: driving on counts as progress (a detour round an obstacle
        last_send = t0        # first takes the vehicle no closer to the goal)
        held = False
        while not rospy.is_shutdown():
            lost = self.fc_lost()
            if lost:
                return lost
            with self.lock:
                pos = None if self.pose is None else self.pose.copy()
                bridge = dict(self.bridge)
                ok_since = self.bridge_ok_since
                track = dict(self.track) if self.track_goals > goals_before else {}
            now = rospy.get_time()
            # D7: the bridge holds on a SLAM reset or an anchor jump; we stop and wait
            if not bridge.get("slam_ok", True):
                if not held:
                    held = True
                    self.report(phase="HOLD_SLAM", leg=idx, reason=bridge.get("reason", "bridge hold"))
                last_progress_t = now
                t0 += 0.1     # the leg clock does not run while the bridge holds
            elif held:
                if now - ok_since >= self.map_rebuild_s:
                    held = False
                    planner = self.publish_goal(target, drive)   # re-validated with the new anchor
                    last_send = now
                    self.report(phase="LEG", leg=idx, type=kind, resumed=True, goal_planner=list(map(float, planner)))
            elif pos is not None:
                d = math.hypot(*(pos[:2] - target[:2])) if drive else float(np.linalg.norm(pos - target))
                if d <= leg["accept_radius"]:
                    return None
                if drive and track.get("state") == "REACHED":
                    # the track planner's own test (its odometry point, ~footprint_offset ahead of
                    # the FC, within its accept radius)
                    self.report(phase="LEG", leg=idx, type=kind, reached_by="track planner", fc_dist=round(d, 3))
                    return None
                if best is None or d < best - 0.2:
                    best = d
                    last_progress_t = now
                    moved_from = pos.copy()
                elif drive and moved_from is not None and math.hypot(*(pos[:2] - moved_from[:2])) > self.drive_progress_m:
                    last_progress_t = now
                    moved_from = pos.copy()
                if now - last_progress_t > leg["no_progress_s"] or now - t0 > leg["leg_timeout_s"]:
                    return "unreachable"
                if self.goal_resend_s > 0 and now - last_send > self.goal_resend_s:
                    self.publish_goal(target, drive)
                    last_send = now
            rospy.sleep(0.1)
        return "shutdown"

    def land(self, leg, idx):
        with self.lock:
            self.landing_clear = None
            self.landing_status = {}
            self.texts = []
        # ask the landing check about this spot (planner frame; z = the height we descend from)
        spot = PointStamped()
        spot.header.frame_id = self.planner_frame
        with self.lock:
            here = None if self.pose is None else self.pose.copy()
            p = self.anchor.fc_to_planner([leg["at"][0], leg["at"][1], here[2] if here is not None else 0.0])
        spot.point.x, spot.point.y, spot.point.z = map(float, p)
        self.report(phase="LAND_CHECK", leg=idx, spot_planner=[round(float(x), 3) for x in p])
        t0 = rospy.get_time()
        last_spot = -1.0
        clear = None
        answers = 0
        while rospy.get_time() - t0 < self.landing_check_s:
            if rospy.get_time() - last_spot >= 0.2:
                spot.header.stamp = rospy.Time.now()
                self.spot_pub.publish(spot)
                last_spot = rospy.get_time()
            with self.lock:
                clear = self.landing_clear
                self.landing_clear = None
            if clear is not None:
                answers += 1
            if clear:      # reported clear: land; not clear (yet): keep asking until landing_check_s
                break
            rospy.sleep(0.05)
        with self.lock:
            why = self.landing_status.get("reason", "no answer from the landing check")
        if not clear:
            self.report(phase="LAND_CHECK", leg=idx, clear=False, answers=answers, reason=why,
                        waited_s=round(rospy.get_time() - t0, 2))
            return "landing spot not reported clear"
        self.bridge_enable(False)
        self.report(phase="LAND", leg=idx, check=why, check_s=round(rospy.get_time() - t0, 2))
        r = self.land_srv(min_pitch=0, yaw=float("nan"), latitude=float("nan"), longitude=float("nan"), altitude=0)
        if not r.success:
            return "landing refused"
        if not self.wait_text("Guided landing complete", self.land_timeout_s):
            return self.fc_lost() or "landing did not complete"
        self.flying = False          # on the ground in GUIDED, driving (QuadRover's NAV_LAND)
        self.landed_once = True
        self.record_ground_spot("landed")
        return None

    def stop(self, why):
        self.bridge_enable(False)
        self.report(phase="STOPPED", reason=why)

    def run(self):
        rate = rospy.Rate(10)
        self.report(phase="WAIT_ARMED", mission=self.mission["name"], legs=len(self.mission["legs"]))
        while not rospy.is_shutdown() and not self.started:
            rate.sleep()
        if rospy.is_shutdown():
            return
        if not self.set_mode("GUIDED"):
            return self.stop("could not enter GUIDED")
        self.flying = False
        legs = self.mission["legs"]
        if not legs or legs[0]["type"] in ("fly", "land"):
            legs = [{"type": "takeoff", "height": self.mission["takeoff_height"]}] + legs
        for idx, leg in enumerate(legs):
            lost = self.fc_lost()
            if lost:
                return self.stop(lost)
            why = None
            if leg["type"] == "takeoff":
                why = self.takeoff(leg["height"])
            elif leg["type"] == "fly":
                if not self.flying:
                    why = self.takeoff(leg["goal"][2])
                why = why or self.go(leg, idx, np.array(leg["goal"], dtype=float), drive=False)
            elif leg["type"] == "drive":
                if self.flying:
                    why = "drive leg while flying: the mission needs a land leg first"
                else:
                    g = list(leg["goal"])[:2]
                    with self.lock:
                        z = 0.0 if self.pose is None else float(self.pose[2])
                    why = self.go(leg, idx, np.array(g + [z]), drive=True)
            elif leg["type"] == "land":
                if not self.flying:
                    why = "land leg on the ground: the mission needs a takeoff first"
                else:
                    with self.lock:
                        z = 0.0 if self.pose is None else float(self.pose[2])
                    at = list(leg["at"])[:2]
                    why = self.go(leg, idx, np.array(at + [z]), drive=False) or self.land(leg, idx)
            if why is None:
                if leg.get("hold_s", 0) > 0:
                    rospy.sleep(leg["hold_s"])
                continue
            if why.startswith("FC ") or why.startswith("FC link"):
                return self.stop(why)            # the FC's failsafe owns the vehicle now
            if why == "shutdown":
                return
            if why in ("takeoff refused", NOT_STATIONARY):
                # still on the ground, driving in GUIDED: QRLOITER there would start the drive-to-fly
                # transition; stay in GUIDED with the bridge off (Rover Guided stops)
                return self.stop(why)            # nothing after a failed takeoff can fly
            if why.startswith("takeoff"):
                self.set_mode("QRLOITER")
                return self.stop(why)
            if why == "landing spot not reported clear":
                self.set_mode("QRLOITER")
                return self.stop(why)
            if not self.flying:
                # on the ground: an unreachable drive goal, a mission error. Hold in GUIDED, bridge off
                self.bridge_enable(False)
                self.report(phase="UNREACHABLE", leg=idx, reason=why, on_ground=True)
                if why.startswith(("bridge", "land leg", "drive leg")):
                    return self.stop(why)
                rospy.sleep(self.unreachable_hold_s)
                if leg.get("on_unreachable", self.on_unreachable) != "next" or self.fc_lost():
                    return self.stop(why)
                continue
            # in the air: unreachable goal, refused landing, a bridge that did not switch: QRLoiter
            self.report(phase="UNREACHABLE", leg=idx, reason=why)
            self.set_mode("QRLOITER")
            if why.startswith(("bridge", "drive leg")):
                return self.stop(why)
            rospy.sleep(self.unreachable_hold_s)
            if leg.get("on_unreachable", self.on_unreachable) != "next" or self.fc_lost():
                return self.stop(why)
            if not self.set_mode("GUIDED"):
                return self.stop("could not re-enter GUIDED after %s" % why)
        self.bridge_enable(False)
        self.report(phase="DONE")


if __name__ == "__main__":
    rospy.init_node("mission_executive")
    MissionExecutive().run()
    rospy.spin()
