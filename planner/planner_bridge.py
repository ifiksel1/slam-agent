#!/usr/bin/env python3
"""Planner bridge (L4 autonomy, L2): SUPER's commands to QuadRover Guided targets, and FAST-LIO's
output to SUPER's frame. The core logic is bridge_core.py (ROS-free, unit-tested).

In:   /Odometry, /cloud_registered          FAST-LIO, raw frame
      /mavros/vision_pose/pose              the SLAM-to-FC bridge's output (fc frame), for the anchor
      /mavros/local_position/pose           the FC's EKF pose (fc frame): the vehicle position for the
                                            caps, the drive target, the HOLD point and the geofence;
                                            trusted only while fresh (~fc_pose_timeout, see below)
      /mavros/state                         stream only in GUIDED
      /planner_bridge/enable                std_msgs/Bool, latched, from the mission executive: stream
                                            only while true (its fly and drive legs), never during the
                                            FC's own takeoff or landing, which a target would override
      /planner_bridge/set_mode              std_msgs/String, latched, from the mission executive:
                                            "flight" or "drive" (~mode is only the start value)
      /planning/pos_cmd                     SUPER (quadrotor_msgs/PositionCommand, planner frame), used
                                            in flight mode
      ~drive_cmd_topic                      the track planner's carrot (/track_planner/pos_cmd,
                                            PositionCommand, position + yaw), used in drive mode
      /fastlio_health, ~range_topic         optional SLAM health (off when absent)
Out:  /planner/odom, /planner/cloud         planner frame, for SUPER and ROG-Map
      /mavros/setpoint_raw/local            mavros_msgs/PositionTarget, 50 Hz, fc frame
      /planner_bridge/anchor                geometry_msgs/TransformStamped, fc <- planner
      /planner_bridge/status                std_msgs/String, JSON (its "mode" is the mode in use)
      /planner_bridge/mode                  std_msgs/String, latched: the mode in use, on every change
      /planner_bridge/mode_code             std_msgs/Int8, 5 Hz, for plots: 0 not streaming,
                                            1 streaming flight targets, 2 streaming drive targets

Modes. flight: SUPER's PositionCommand -> PosVelAccel + yaw, capped (Guided flight). drive: the track
planner's target (its carrot run out along the carrot's bearing, frame-checked: track_core.run_out)
-> a position-only target within ~drive_lookahead of the vehicle (Rover Guided on the ground rejects any
target with acceleration; its fast-update path asks ~0.5 m/s per metre to the target, hence 2.5 m). The two planners publish on separate topics (SUPER keeps
publishing toward its last goal after a landing); the bridge listens only to the one of its mode. A
mode change drops the last command and holds (latched) until a command from the new mode's planner
arrives, so a target of one planner is never sent in the other mode.

It never arms, never changes modes and never sets parameters (D5): mode changes belong to the
mission executive. In HOLD in flight it streams a zero-velocity target at the current position, so a
fault never leaves the FC coasting on the last velocity until Q_GUID_TIMEOUT (D4). In HOLD in drive
mode it streams nothing: Rover Guided drives on to the last target it was sent (at most
~drive_lookahead ahead, in the direction of travel), counts it reached and brakes to a stop (D4: a
dead planner stops the rover within the lookahead). A position hold at the vehicle does not stop a
rover: taken while it moves, the point is soon behind it and Rover Guided orbits it (SITL cl36,
~0.28 m/s for 15 s); a zero-speed heading target let SITL's rover lurch into reverse and creep (cl37, cl38).
(Those runs had SITL's conserved sideways slide after a touchdown, sitl_track_friction.patch.)

Reversing (owner 2026-10-28, dead ends). A track planner command whose velocity points against its yaw
means: back straight up, holding the yaw, until the FC is at the command's position
(bridge_core.reverse_speed). It goes out as a Rover Guided velocity + heading target (mask 2503: no
position; MAVROS turns the ENU velocity (0, -speed) into NED vx -speed, which Rover reads as
reverse: GCS_MAVLink_Rover speed_dir), speed tapering to 0 at the end point: a Guided setpoint,
within D5 (no DO_SET_REVERSE, no mode change). Rover's velocity + heading submode keeps the last
speed for 3 s without targets, so a HOLD after a reverse streams a zero-speed one (a dead bridge:
Rover stops itself after 3 s, <= 0.75 m at the planner's 0.25 m/s).

A stale FC pose (owner decision 2026-10-28). The FC pose is the bridge's "where the vehicle is now"
(p_now): every target is capped or placed relative to it. Its age is measured on the receive time.
  * none received yet: nothing is streamed (status "fc_pose": "never", reason "no FC pose yet");
  * older than ~fc_pose_timeout (0.3 s; ~50 Hz on the robot): HOLD with reason "fc_pose_stale",
    whatever the planner says. In flight the hold target is the last pose received while it was
    fresh (bridge_core.FcPoseGuard.good), streamed at the full rate with zero velocity so Guided does
    not time out on Q_GUID_TIMEOUT; in drive mode nothing is sent (Rover Guided brakes at its last
    target), except during a reverse: a zero-speed heading target, as in any drive HOLD then (Rover's
    velocity + heading target would back up on for 3 s; it needs no pose). A hold that began earlier
    (say, planner stale) keeps its point;
  * fresh again: trusted after ~fc_pose_resume_n poses in a row (5, 0.1 s at 50 Hz), each within
    ~fc_pose_timeout of the one before; then the usual resume (a planner command newer than the hold).
Status: "fc_pose" ("never" | "stale" | "ok"), "fc_pose_age" (s, null before the first) and the reason.
"fc_pose_stale" is not a SLAM hold ("slam_hold" false): the executive's leg clock keeps running.
"""
import json
import math
import os
import sys

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from mavros_msgs.msg import PositionTarget, State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Range
from std_msgs.msg import Bool, Float64MultiArray, Int8, String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bridge_core as bc  # noqa: E402

try:
    from quadrotor_msgs.msg import PositionCommand
except ImportError:  # SUPER's workspace not sourced: the node cannot run without it
    PositionCommand = None

# MAVROS PositionTarget type_mask bits (MAVLink POSITION_TARGET_TYPEMASK)
IGNORE_PX, IGNORE_PY, IGNORE_PZ = 1, 2, 4
IGNORE_VX, IGNORE_VY, IGNORE_VZ = 8, 16, 32
IGNORE_AFX, IGNORE_AFY, IGNORE_AFZ = 64, 128, 256
FORCE, IGNORE_YAW, IGNORE_YAW_RATE = 512, 1024, 2048
# flight: position + velocity + acceleration + yaw (QuadRover Guided flight -> set_pos_vel_accel_NED_m)
MASK_FLIGHT = IGNORE_YAW_RATE
# driving: position + yaw only (Rover Guided rejects any target with acceleration set)
MASK_DRIVE = IGNORE_VX | IGNORE_VY | IGNORE_VZ | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ | IGNORE_YAW_RATE
# reversing (dead ends): velocity + yaw, no position (Rover Guided HeadingAndSpeed; a negative NED vx
# backs up holding the yaw; stops by itself 3 s after the last target)
MASK_REVERSE = IGNORE_PX | IGNORE_PY | IGNORE_PZ | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ | IGNORE_YAW_RATE


def vec(m):
    return np.array([m.x, m.y, m.z])


def quat(m):
    return (m.x, m.y, m.z, m.w)


SLAM_HOLD_REASONS = ("SLAM", "no SLAM", "frame re-anchored", "no frame anchor")
MODES = ("flight", "drive")


class PlannerBridge:
    def __init__(self):
        g = rospy.get_param
        self.mode = g("~mode", "flight")                       # flight | drive: the start value; the
        if self.mode not in MODES:                             # executive sets it on /planner_bridge/set_mode
            raise ValueError("~mode must be one of %s" % (MODES,))
        self.rate = float(g("~rate_hz", 50.0))
        self.speed_cap = float(g("~speed_cap", 0.5))           # 1.0 in sim, 0.5 robot (D10)
        self.max_lead = float(g("~max_lead", 1.0))
        self.fc_pos_p = float(g("~fc_pos_p", 1.0))                # the FC's Q_P_NE_POS_P: the cap bounds the FC's demand
        self.drive_lookahead = float(g("~drive_lookahead", 2.5))     # above the track planner's run-out (<= 2.1)
        self.guided_mode = g("~guided_mode", "GUIDED")
        self.fence = bc.Geofence(g("~geofence_min", [-50.0, -50.0, -1.0]), g("~geofence_max", [50.0, 50.0, 10.0]))
        self.hold = bc.HoldLogic(g("~planner_timeout", 0.3), g("~slam_timeout", 0.2))
        self.fc_guard = bc.FcPoseGuard(g("~fc_pose_timeout", 0.3), g("~fc_pose_resume_n", 5))
        self.tracker = bc.AnchorTracker(g("~anchor_jump_m", 0.1), g("~anchor_jump_deg", 2.0),
                                        g("~anchor_persist", 3), math.radians(g("~fc_mount_yaw_deg", 0.0)))
        self.health = bc.HealthCheck(g("~health_matched_min", 150.0), g("~health_eig_min", 5000.0),
                                     g("~health_latency_max_ms", 250.0), g("~height_tol", 0.5))
        self.health_on = bool(g("~health_enable", False))
        # the executive's go-ahead; ~require_enable false streams in every GUIDED (bench use only)
        self.enabled = not bool(g("~require_enable", True))

        self.t_cmd = None
        self.cmd = None             # (p, v, a, yaw) planner frame
        self.t_slam = None
        self.last_planner = None    # (p, q) from the newest /Odometry, planner frame
        self.fc_pose = None         # (p, yaw) from /mavros/local_position/pose, the newest whatever its age
        self.fc_mode = ""
        self.hold_point = None
        self.reversing = None       # the yaw of a reverse under way (drive mode), else None
        self.n_out = 0

        self.pub_odom = rospy.Publisher("/planner/odom", Odometry, queue_size=20)
        self.pub_cloud = rospy.Publisher("/planner/cloud", PointCloud2, queue_size=5)
        self.pub_sp = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        self.pub_anchor = rospy.Publisher("/planner_bridge/anchor", TransformStamped, queue_size=5, latch=True)
        self.pub_status = rospy.Publisher("/planner_bridge/status", String, queue_size=5)
        self.pub_mode = rospy.Publisher("/planner_bridge/mode", String, queue_size=1, latch=True)
        self.pub_mode_code = rospy.Publisher("/planner_bridge/mode_code", Int8, queue_size=5)
        self.pub_mode.publish(String(data=self.mode))

        rospy.Subscriber("/Odometry", Odometry, self.on_odom, queue_size=50)
        rospy.Subscriber("/cloud_registered", PointCloud2, self.on_cloud, queue_size=5)
        rospy.Subscriber("/mavros/vision_pose/pose", PoseStamped, self.on_vision, queue_size=50)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.on_fc_pose, queue_size=20)
        rospy.Subscriber("/mavros/state", State, self.on_state, queue_size=5)
        rospy.Subscriber("/planner_bridge/enable", Bool, self.on_enable, queue_size=5)
        rospy.Subscriber("/planner_bridge/set_mode", String, self.on_set_mode, queue_size=5)
        if PositionCommand is not None:
            rospy.Subscriber("/planning/pos_cmd", PositionCommand, self.on_cmd, callback_args="flight", queue_size=20)
            rospy.Subscriber(g("~drive_cmd_topic", "/track_planner/pos_cmd"), PositionCommand, self.on_cmd,
                             callback_args="drive", queue_size=20)
        else:
            rospy.logerr("quadrotor_msgs not found: source SUPER's workspace; no planner input")
        if self.health_on:
            rospy.Subscriber("/fastlio_health", Float64MultiArray, self.on_health, queue_size=10)
            rospy.Subscriber(g("~range_topic", "/mavros/rangefinder/rangefinder"), Range, self.on_range, queue_size=10)
        rospy.Timer(rospy.Duration(1.0 / self.rate), self.tick)
        rospy.Timer(rospy.Duration(0.2), self.publish_status)
        rospy.loginfo("planner bridge: mode %s, %.0f Hz, speed cap %.2f m/s, geofence %s..%s",
                      self.mode, self.rate, self.speed_cap, self.fence.lo, self.fence.hi)

    # ---- inputs --------------------------------------------------------------------------------
    def on_odom(self, m):
        p = bc.raw_to_planner_pos(vec(m.pose.pose.position))
        q = bc.raw_to_planner_quat(quat(m.pose.pose.orientation))
        v = bc.raw_to_planner_pos(vec(m.twist.twist.linear))   # FAST-LIO twist is world-frame
        self.last_planner = (p, q)
        self.t_slam = rospy.get_time()
        o = Odometry()
        o.header.stamp = m.header.stamp
        o.header.frame_id = "world"
        o.child_frame_id = "body"
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = (float(x) for x in p)
        o.pose.pose.orientation.x, o.pose.pose.orientation.y, o.pose.pose.orientation.z, o.pose.pose.orientation.w = q
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z = (float(x) for x in v)
        self.pub_odom.publish(o)

    def on_cloud(self, m):
        # rotate x, y, z in place (fields are float32 at offsets 0, 4, 8 in FAST-LIO's clouds)
        step = m.point_step
        a = np.frombuffer(m.data, dtype=np.uint8).reshape(-1, step).copy()
        xyz = np.ascontiguousarray(a[:, :12]).view(np.float32).reshape(-1, 3).astype(float)
        a[:, :12] = bc.raw_to_planner_points(xyz).astype(np.float32).view(np.uint8).reshape(-1, 12)
        out = PointCloud2(header=m.header, height=m.height, width=m.width, fields=m.fields,
                          is_bigendian=m.is_bigendian, point_step=step, row_step=m.row_step,
                          data=a.tobytes(), is_dense=m.is_dense)
        out.header.frame_id = "world"
        self.pub_cloud.publish(out)

    def on_vision(self, m):
        # the bridge publishes one vision pose per /Odometry, right after it: pair with the newest
        if self.last_planner is None:
            return
        p, q = self.last_planner
        anchor, jump = self.tracker.update(p, q, vec(m.pose.position), quat(m.pose.orientation))
        if jump:
            rospy.logwarn("planner bridge: %s: HOLD until a fresh plan", jump)
            self.hold.latch("frame re-anchored (%s)" % jump, rospy.get_time())
        t = TransformStamped()
        t.header.stamp = m.header.stamp
        t.header.frame_id = "map"
        t.child_frame_id = "planner"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (float(x) for x in anchor.t)
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = bc.quat_yaw(anchor.psi)
        self.pub_anchor.publish(t)

    def on_fc_pose(self, m):
        self.fc_pose = (vec(m.pose.position), bc.yaw_of(quat(m.pose.orientation)))
        self.fc_guard.on_pose(rospy.get_time(), self.fc_pose)

    def on_state(self, m):
        self.fc_mode = m.mode

    def on_enable(self, m):
        self.enabled = bool(m.data)

    def on_set_mode(self, m):
        mode = m.data.strip().lower()
        if mode not in MODES:
            rospy.logerr("planner bridge: unknown mode %r ignored (flight | drive)", m.data)
            return
        if mode == self.mode:
            return
        # the other planner's last command must never be sent in this mode: drop it and hold until
        # the new mode's planner sends a command (HoldLogic.latch needs one newer than now)
        self.mode = mode
        self.cmd = None
        self.t_cmd = None
        self.hold_point = None
        self.reversing = None
        self.hold.latch("mode changed to %s" % mode, rospy.get_time())
        self.pub_mode.publish(String(data=mode))
        rospy.loginfo("planner bridge: mode %s", mode)

    def on_cmd(self, m, source="flight"):
        if source != self.mode:
            return                      # the other mode's planner
        self.cmd = (vec(m.position), vec(m.velocity), vec(m.acceleration), float(m.yaw))
        self.t_cmd = rospy.get_time()

    def on_health(self, m):
        self.health.on_health(rospy.get_time(), m.data)

    def on_range(self, m):
        if self.fc_pose is not None:
            self.health.on_range(rospy.get_time(), m.range, self.fc_pose[0][2])

    # ---- output --------------------------------------------------------------------------------
    def tick(self, _):
        now = rospy.get_time()
        faults = self.health.faults(now) if self.health_on else []
        anchor = self.tracker.anchor
        mode, cmd = self.mode, self.cmd     # one read each: the executive may switch modes meanwhile
        target = None
        if cmd is not None and anchor is not None:
            p, v, a, yaw = cmd
            target = (anchor.pos(p), anchor.vec(v), anchor.vec(a), anchor.yaw(yaw))
            if not self.fence.inside(target[0]):
                faults = faults + ["target outside the geofence"]
        if self.fc_pose is not None and not self.fence.inside(self.fc_pose[0]):
            faults = faults + ["vehicle outside the geofence"]
        if anchor is None:
            faults = faults + ["no frame anchor yet (no vision pose)"]
        fc = self.fc_guard.check(now)
        if fc == "never":
            faults = ["no FC pose yet"] + faults
        elif fc == "stale":
            faults = ["fc_pose_stale"] + faults     # the reason to report: first
        state = self.hold.step(now, self.t_cmd, self.t_slam, faults)
        if self.fc_mode != self.guided_mode or not self.enabled or fc == "never":
            self.hold_point = None
            return                      # never stream outside Guided (the executive owns modes), nor
                                        # during the FC's own takeoff or landing (not enabled), nor
                                        # without ever having had the vehicle's position
        # stale: the last pose received while fresh (the state is HOLD then: nothing is placed on it
        # but the hold point)
        p_now, yaw_now = self.fc_guard.good if fc == "stale" else self.fc_pose
        if state == "STREAM" and target is not None:
            self.hold_point = None
            if mode == "drive":
                rs = bc.reverse_speed(target[0], target[1], target[3], p_now)
                if rs is not None:
                    self.send_reverse(rs, target[3])
                    return
                self.reversing = None
                self.send(bc.drive_target(target[0], target[1], p_now, self.drive_lookahead), None, None, target[3], MASK_DRIVE)
            else:
                p, v, a, _ = bc.cap_command(target[0], target[1], target[2], p_now, self.speed_cap, self.max_lead,
                                              self.fc_pos_p)
                self.send(p, v, a, target[3], MASK_FLIGHT)
            return
        if mode == "drive":
            if self.reversing is not None:
                # a reverse under way: Rover's velocity + heading target would back up on for 3 s
                self.send_reverse(0.0, self.reversing)
            return                      # HOLD on the ground: Rover Guided stops at the last target
        # HOLD (or WAIT) in flight: stay where the vehicle is, zero velocity and acceleration
        if self.hold_point is None:
            self.hold_point = (self.fence.clamp(p_now), yaw_now)
        hp, hy = self.hold_point
        self.send(hp, np.zeros(3), np.zeros(3), hy, MASK_FLIGHT)

    def send_reverse(self, speed, yaw):
        """back up at speed (m/s, >= 0) holding yaw (fc frame, ENU): MAVROS turns the ENU velocity
        (0, -speed) into NED vx = -speed, which Rover Guided reads as reverse (speed_dir -1); at 0 a
        zero-speed heading hold"""
        self.reversing = yaw
        self.send(np.zeros(3), np.array([0.0, -abs(speed), 0.0]), None, yaw, MASK_REVERSE)

    def send(self, p, v, a, yaw, mask):
        sp = PositionTarget()
        sp.header.stamp = rospy.Time.now()
        sp.header.frame_id = "map"
        # MAVROS takes ENU values here and converts them to the FC's NED
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        sp.type_mask = mask
        sp.position.x, sp.position.y, sp.position.z = (float(x) for x in p)
        if v is not None:
            sp.velocity.x, sp.velocity.y, sp.velocity.z = (float(x) for x in v)
        if a is not None:
            sp.acceleration_or_force.x, sp.acceleration_or_force.y, sp.acceleration_or_force.z = (float(x) for x in a)
        sp.yaw = float(yaw)
        self.pub_sp.publish(sp)
        self.n_out += 1

    def publish_status(self, _):
        now = rospy.get_time()
        age = lambda t: None if t is None else round(now - t, 3)  # noqa: E731
        a = self.tracker.anchor
        streaming = self.fc_mode == self.guided_mode and self.enabled
        self.pub_mode_code.publish(Int8(data=(MODES.index(self.mode) + 1) if streaming else 0))
        self.pub_status.publish(String(json.dumps({
            "state": self.hold.state, "reason": self.hold.reason, "mode": self.mode, "fc_mode": self.fc_mode,
            # the mission executive pauses its leg only for SLAM-side holds (D7), not for a planner
            # that has not planned yet or a geofence stop
            "slam_hold": self.hold.state == "HOLD" and self.hold.reason.startswith(SLAM_HOLD_REASONS),
            "enabled": self.enabled,
            "streaming": streaming, "cmd_age": age(self.t_cmd), "slam_age": age(self.t_slam),
            "fc_pose": self.fc_guard.check(now), "fc_pose_age": age(self.fc_guard.t),
            "speed_cap": self.speed_cap, "anchor": None if a is None else {"psi_deg": round(math.degrees(a.psi), 2),
                                                                           "t": [round(x, 3) for x in a.t]},
            "anchor_jumps": self.tracker.jumps, "targets_sent": self.n_out})))


if __name__ == "__main__":
    rospy.init_node("planner_bridge")
    PlannerBridge()
    rospy.spin()
