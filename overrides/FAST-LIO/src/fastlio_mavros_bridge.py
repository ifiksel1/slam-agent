#!/usr/bin/env python3

"""
FAST-LIO to MAVROS Bridge
Converts FAST-LIO odometry (/Odometry) to MAVROS vision pose
(/mavros/vision_pose/pose) for ArduPilot/PX4 external-nav fusion.

POSITION and ORIENTATION are transformed SEPARATELY, because a combined bench test
(PROGRESS T18 + the translation+rotation run) showed they live in DIFFERENT frames in
/Odometry:
  - POSITION is already correct ENU/FLU (forward->+x, left->+y, up->+z), so the default
    ~position_matrix is identity. It stays a 3x3 param so a vehicle that DOES need a
    swap/sign fix can be recalibrated without code changes.
  - ORIENTATION is a same-handed 3-axis cyclic permutation of ENU (up=+y; det=+1, no
    mirror). ~remap_axes rebuilds the ENU quaternion as euler(odom_yaw, odom_roll,
    odom_pitch)  ==  (FC_roll<-odom_yaw, FC_pitch<-odom_roll, FC_yaw<-odom_pitch).
This replaces the old single-~rotation_matrix design (which applied ONE matrix to both
and could not represent a permutation correctly), and retires the position-only y-flip.
"""

import json
import os
import struct
import sys
import threading

import rospy
import numpy as np
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf.transformations import (quaternion_multiply, quaternion_conjugate,
                                quaternion_from_matrix, quaternion_from_euler)

# frame_anchor.py sits next to this file; catkin's relay runs it with __file__ set to the source
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from frame_anchor import Anchor, Discontinuity, PoseHistory, ekf_usable, yaw_of  # noqa: E402

try:
    from mavros_msgs.msg import Mavlink, PositionTarget, RCOut, State, StatusText
    from mavros_msgs.srv import ParamGet
    _HAVE_MAVROS = True
except ImportError:
    _HAVE_MAVROS = False

_EKF_STATUS_REPORT_MSGID = 193


# ---- FROZEN SLAM POSE: cut vision (owner decisions 2026-11-02 in the air, 2026-11-03 on the ground) ---
# FAST-LIO stuck on its last pose, republished with fresh stamps, passes every freshness check, and the FC's
# EKF fuses it: the FC then flies on a position that does not move (SITL L2 fault 2b: 45 m at 8.4 m/s on
# 10-28; 9 m and a 1.39 m/s ground strike on 11-02, the executive's QRLAND still fusing the frozen pose).
# Owner 2026-11-02: when the pose freezes while AIRBORNE, stop sending vision (pose and speed), so the FC's own
# EKF failsafe (FS_EKF_ACTION: QRLAND) lands it on its IMU instead (SITL 7c: ~5.6 m of drift, a 2-2.5 m/s
# sideways touchdown: accepted). Owner 2026-11-03: cut on the ground too, then (same day, after SITL) kept OFF by default (~frozen_ground_cut,
# default false): with vision cut the EKF dead-reckons 7-44 m and 2-6 m/s with no failsafe for ~16 s, only Rover's
# stop latch prevents a runaway, and one 2d run stopped later (4.96 m) than without the cut (RESULTS_20261102.md).
# When true:
# driving on a frozen pose, the EKF follows it and Rover's zero-speed stop (planner bridge / executive, MASK_STOP)
# does not brake (SITL 2d: the rover coasts ~4.5 m). With vision cut the EKF runs on its IMU and the stop brakes on
# a velocity that moves (SITL results: planner/sim/faults/RESULTS_20261102.md, "Ground cut"). An Oct 30 SITL cut
# WITHOUT the planner's stop ran away ~65 m (Rover dead-reckoning on a stale target). A ground cut resumes as soon
# as SLAM moves again (no landing to wait for); false: the old rule (reported, vision kept). This covers manual
# flights too, so it has its own detection (no dependency on the planner bridge), the same rule as the planner
# bridge's FrozenPoseCheck (planner/ros/l4_bridge bridge_core.h) with its thresholds:
#   * a repeat: a raw /Odometry pose within eps_m of the one before and every quaternion component within eps_q
#     (a real FAST-LIO pose never repeats: 4 recordings, ~9200 pairs, smallest step 0.075 mm / 1.5e-5);
#   * frozen: at least min_n repeats in a row AND evidence that the vehicle moves or is made to move, active
#     without a break for hold_s. Evidence (each no older than evidence_age):
#       command  a setpoint streamed to the FC (/mavros/setpoint_raw/local) asking for >= cmd_speed horizontally,
#       gyro     the FC's IMU (/mavros/imu/data) turning at >= gyro_min,
#       twist    the repeated /Odometry itself carrying >= twist_min,
#       ekf      the FC's EKF velocity (/mavros/local_position/velocity_local) >= ekf_speed_min (the planner
#                bridge has commands for this; a manual flight has none);
#   * the FC must be alive: once an IMU sample was seen, nothing counts while the newest is older than
#     evidence_age (SITL host stalls freeze the FC and the pose together).


class FrozenPoseCheck(object):
    """Repeated raw poses plus evidence of motion (a port of l4_bridge's FrozenPoseCheck, no ROS)."""

    def __init__(self, eps_m=1e-5, eps_q=1e-6, hold_s=0.4, min_n=4, cmd_speed=0.2, gyro_min=0.2,
                 twist_min=0.2, ekf_speed_min=0.2, evidence_age=0.25):
        self.eps_m, self.eps_q, self.hold_s, self.min_n = eps_m, eps_q, hold_s, max(1, int(min_n))
        self.cmd_speed, self.gyro_min, self.twist_min = cmd_speed, gyro_min, twist_min
        self.ekf_speed_min, self.evidence_age = ekf_speed_min, evidence_age
        self.repeats = 0              # repeated poses in a row
        self.run_start = None         # receive time of the pose the run repeats
        self.evidence_from = None     # the evidence active since (within this run)
        self.last = None              # the previous raw pose (x, y, z, qx, qy, qz, qw)
        self.twist = 0.0
        self.t_cmd_move = self.t_gyro = self.t_ekf_move = self.t_pose = None
        self.gyro = 0.0

    def same(self, a, b):
        return (a is not None and b is not None
                and math.sqrt(sum((x - y) ** 2 for x, y in zip(a[:3], b[:3]))) <= self.eps_m
                and all(abs(x - y) <= self.eps_q for x, y in zip(a[3:], b[3:])))

    def on_pose(self, t, pose, twist_norm):
        if self.same(pose, self.last):
            if self.repeats == 0:
                self.run_start = self.t_pose
            self.repeats += 1
        else:
            self.repeats = 0
            self.run_start = self.evidence_from = None
        self.last = tuple(pose)
        self.twist = twist_norm
        self.t_pose = t

    def on_command(self, t, speed):
        if speed >= self.cmd_speed:
            self.t_cmd_move = t

    def on_gyro(self, t, rate):
        self.t_gyro, self.gyro = t, rate

    def on_ekf_velocity(self, t, speed):
        if speed >= self.ekf_speed_min:
            self.t_ekf_move = t

    def evidence(self, now):
        e = []
        if self.t_cmd_move is not None and now - self.t_cmd_move <= self.evidence_age:
            e.append("command")
        if self.t_gyro is not None and now - self.t_gyro <= self.evidence_age and self.gyro >= self.gyro_min:
            e.append("gyro")
        if self.repeats > 0 and self.twist >= self.twist_min:
            e.append("twist")
        if self.t_ekf_move is not None and now - self.t_ekf_move <= self.evidence_age:
            e.append("ekf")
        return "+".join(e)

    def check(self, now):
        """-> the evidence when frozen, else None. Called on every pose (it runs the evidence clock)."""
        if self.repeats == 0:
            self.evidence_from = None
            return None
        fc_alive = self.t_gyro is None or now - self.t_gyro <= self.evidence_age
        e = self.evidence(now) if fc_alive else ""
        if not e:
            self.evidence_from = None    # the evidence must last without a break
            return None
        if self.evidence_from is None:
            self.evidence_from = now
        if self.repeats >= self.min_n and now - self.evidence_from >= self.hold_s:
            return e
        return None


class AirborneGate(object):
    """Is the vehicle in the air? From the FC's own outputs, never from SLAM.

    QuadRover reports no landed state (EXTENDED_SYS_STATE's landed_state is UNDEFINED in Rover) and GUIDED is
    both a drive and a flight mode, so the mode alone cannot tell. The lift motors can: SERVO_OUTPUT_RAW
    (/mavros/rc/out, the MAV1_RC_CHAN stream, 10 Hz on the robot and in SITL) on the lift-motor channels
    (SERVO1-4 = Motor1-4 on the robot and in SITL). Airborne = armed AND the lift motors' mean output at least
    pwm_min (above spin-min: producing thrust; drive mode has them stopped, spin-arm idle is below it), with
    both /mavros/state and /mavros/rc/out fresh. Anything unknown (no or stale sample, disarmed) is "not
    airborne": the default is not to cut. Landed (for the resume) = disarmed, or the lift motors below
    pwm_min without a break for landed_s.
    """

    def __init__(self, channels=(1, 2, 3, 4), pwm_min=1200.0, max_age=1.0, state_max_age=3.0, landed_s=3.0):
        self.channels = [int(c) for c in channels]
        self.pwm_min, self.max_age, self.state_max_age, self.landed_s = pwm_min, max_age, state_max_age, landed_s
        self.armed = None
        self.mode = None
        self.t_state = None
        self.t_motors = None
        self.motors = None            # mean lift-motor output (us)
        self.idle_since = None        # lift motors below pwm_min (or disarmed) since

    def on_state(self, t, armed, mode):
        self.armed, self.mode, self.t_state = bool(armed), mode, t
        if not self.armed and self.idle_since is None:
            self.idle_since = t

    def on_motors(self, t, channels_us):
        vals = [channels_us[c - 1] for c in self.channels if 0 < c <= len(channels_us)]
        if not vals:
            return
        self.motors, self.t_motors = sum(vals) / float(len(vals)), t
        if self.motors >= self.pwm_min:
            self.idle_since = None
        elif self.idle_since is None:
            self.idle_since = t

    def airborne(self, now):
        """-> (True | False, why)"""
        if self.t_state is None or now - self.t_state > self.state_max_age:
            return False, "no FC state"
        if not self.armed:
            return False, "disarmed"
        if self.t_motors is None or now - self.t_motors > self.max_age:
            return False, "no motor outputs (/mavros/rc/out)"
        if self.motors < self.pwm_min:
            return False, "lift motors %.0f us < %.0f" % (self.motors, self.pwm_min)
        return True, "armed, lift motors %.0f us" % self.motors

    def landed(self, now):
        """True once disarmed, or the lift motors stayed below pwm_min for landed_s (fresh samples)"""
        if self.t_state is not None and now - self.t_state <= self.state_max_age and not self.armed:
            return True
        return (self.idle_since is not None and self.t_motors is not None and now - self.t_motors <= self.max_age
                and now - self.idle_since >= self.landed_s)


class FrozenVisionCut(object):
    """The latch: cut vision on a frozen pose, in the air and (ground_cut) on the ground; keep it cut until SLAM
    moves again (and, after a cut in the air, the vehicle has landed).

    on_pose() -> True when this pose must NOT go to the FC. Cut: FrozenPoseCheck frozen; where: AirborneGate's
    word at the cut ("air" | "ground").
      * in the air: latched for the rest of the flight (SLAM coming back in the air does not resume: its frame
        has a hole the EKF dead-reckoned through, and the FC is already landing). Resume: landed (disarmed, or
        the lift motors idle for landed_s) AND resume_n poses in a row that differ from the frozen pose;
      * on the ground (owner decision 2026-11-03; ground_cut, else only reported, vision kept): resume once
        SLAM moves again (resume_n new poses in a row) while the gate still says not airborne; airborne at
        that moment (a takeoff with vision cut): held until landed, as an air cut.
    The same pose still repeated keeps it cut. The resumed pose is a gap to the frame tracker, so it
    re-anchors onto the EKF (enable_reanchor) as after a laserMapping restart.
    """

    def __init__(self, check, gate, resume_n=5, ground_cut=False):
        self.check, self.gate, self.resume_n = check, gate, max(1, int(resume_n))
        self.ground_cut = bool(ground_cut)
        self.cut = False
        self.cut_where = None         # "air" | "ground" while cut
        self.frozen_pose = None
        self.cut_at = None
        self.cut_evidence = ""
        self.moving_n = 0             # poses in a row that differ from the frozen pose (while cut)
        self.ground_frozen = None     # the evidence of a frozen pose on the ground (not cut), else None
        self.events = []              # (kind, text) for the caller to log; it clears them

    def on_pose(self, t, pose, twist_norm):
        self.check.on_pose(t, pose, twist_norm)
        if self.cut:
            self.moving_n = 0 if self.check.same(pose, self.frozen_pose) else self.moving_n + 1
            if self.moving_n >= self.resume_n:
                if self.cut_where == "ground":
                    air, why = self.gate.airborne(t)
                    ok, how = not air, "on the ground (%s)" % why
                else:
                    ok, how = self.gate.landed(t), "landed"
                if ok:
                    self.cut = False
                    self.events.append(("resume", "%s and SLAM moving again (%d new poses): vision resumed after "
                                        "%.1f s (cut %s)" % (how, self.moving_n, t - self.cut_at, self.cut_where)))
                    self.frozen_pose = self.cut_where = None
                    return False
            return True
        e = self.check.check(t)
        if e is None:
            self.ground_frozen = None
            return False
        air, why = self.gate.airborne(t)
        if not air and not self.ground_cut:
            if self.ground_frozen is None:
                self.events.append(("ground", "SLAM pose frozen (%d repeats, evidence %s) but not airborne (%s): "
                                    "vision kept" % (self.check.repeats, e, why)))
            self.ground_frozen = e
            return False
        self.cut, self.cut_at, self.cut_evidence = True, t, e
        self.cut_where = "air" if air else "ground"
        self.frozen_pose = tuple(pose)
        self.moving_n = 0
        self.ground_frozen = None
        span = t - self.check.run_start if self.check.run_start is not None else 0.0
        if air:
            self.events.append(("cut", "SLAM pose frozen in the air (%d repeats over %.2f s, evidence %s; %s): "
                                "vision pose and speed STOPPED, the FC's EKF failsafe takes over"
                                % (self.check.repeats, span, e, why)))
        else:
            self.events.append(("cut_ground", "SLAM pose frozen on the ground (%d repeats over %.2f s, evidence %s; "
                                "not airborne: %s): vision pose and speed STOPPED until SLAM moves again"
                                % (self.check.repeats, span, e, why)))
        return True

    def status(self, now):
        air, why = self.gate.airborne(now)
        c = self.check
        return {"vision": "cut" if self.cut else "on", "latched": self.cut,
                "cut_where": self.cut_where if self.cut else None,
                "cut_for_s": round(now - self.cut_at, 2) if self.cut else None,
                "cut_evidence": self.cut_evidence if self.cut else None,
                "repeats": c.repeats,
                "repeat_for_s": round(now - c.run_start, 2) if c.repeats and c.run_start is not None else 0.0,
                "evidence": c.evidence(now) if c.repeats else "",
                "frozen_on_ground": self.ground_frozen, "ground_cut": self.ground_cut,
                "airborne": air, "airborne_why": why, "landed": self.gate.landed(now),
                "armed": self.gate.armed, "mode": self.gate.mode, "lift_motors_us": self.gate.motors}

class FastLioMavrosBridge:
    def __init__(self):
        rospy.init_node('fastlio_mavros_bridge', anonymous=False)

        # Frame configuration
        self.vision_frame = rospy.get_param('~vision_frame', 'map')
        self.body_frame = rospy.get_param('~body_frame', 'base_link')

        # Re-stamp with system time. The LiDAR/FAST-LIO timestamps are on the sensor's
        # internal clock (~years off the system/FC clock), so ArduPilot cannot time-align
        # vision against its IMU and rejects it under motion. Stamping with rospy.Time.now()
        # puts the pose on the system clock, which mavros keeps timesynced to the FC over
        # MAVLink; the residual age (~scan period + ~14 ms processing) is absorbed by VISO_DELAY_MS.
        # Disable with ~restamp_system_time:=false to forward the original sensor stamp.
        self.restamp_system_time = rospy.get_param('~restamp_system_time', True)

        # POSITION transform: FAST-LIO position -> ENU. Row-major 9 values. Raw /Odometry frame is
        # (+x=LEFT, +y=UP, +z=FORWARD), so mapping to ENU (fwd=+x,left=+y,up=+z) needs the SAME cyclic
        # permutation as orientation: ENU_x=odom_z, ENU_y=odom_x, ENU_z=odom_y -> [0,0,1, 1,0,0, 0,1,0].
        # (An earlier identity default came from a mislabeled bench and caused in-flight drift -- fixed.)
        # Kept as a param (not forced to a proper rotation) for per-vehicle recalibration.
        flat = rospy.get_param('~position_matrix', [0.0, 0.0, 1.0,
                                                    1.0, 0.0, 0.0,
                                                    0.0, 1.0, 0.0])
        self.Mpos = np.array(flat, dtype=float).reshape(3, 3)

        # Zero the initial heading: capture yaw0 from the first published pose and rotate
        # every subsequent pose by -yaw0 about vertical, so the FC frame starts at yaw=0
        # (forward -> +x). The SAME -yaw0 is applied to BOTH position and orientation.
        # Disable with ~zero_initial_yaw:=false to keep the absolute heading. Only used when
        # there is no EKF to anchor to (first boot, or ~enable_reanchor:=false): see FRAME ANCHOR.
        self.zero_initial_yaw = rospy.get_param('~zero_initial_yaw', True)

        # ORIENTATION transform (PROGRESS T18). Raw /Odometry orientation is a same-handed 3-axis
        # CYCLIC PERMUTATION of ENU (up=+y; det=+1, no mirror): ENU_x=odom_z, ENU_y=odom_x, ENU_z=odom_y.
        # Applied as a QUATERNION SIMILARITY transform  q_enu = qR * q_odom * qR^-1  (gimbal-lock free).
        # NOTE: an euler-based remap was tried first and FAILED on large yaw -- a physical yaw maps to
        # odom_pitch -> ~90 deg, so euler_from_quaternion(odom) hits gimbal lock and yaw came out wrong
        # (CCW read -108 instead of +90). The quaternion form has no such degeneracy. Toggle ~remap_axes.
        of = rospy.get_param('~orientation_matrix', [0.0, 0.0, 1.0,
                                                     1.0, 0.0, 0.0,
                                                     0.0, 1.0, 0.0])
        Mo = np.eye(4)
        Mo[:3, :3] = np.array(of, dtype=float).reshape(3, 3)
        self.q_perm = quaternion_from_matrix(Mo)   # [x,y,z,w]
        # BODY side of the orientation transform: q_enu = q_world * q_odom * q_body^-1. The default
        # (unset) is the same matrix as ~orientation_matrix, which is exactly the similarity form above
        # and right for FAST-LIO, whose world frame IS its starting IMU frame. A backend whose world
        # frame is NOT its IMU frame needs them different: BIEVR-LIO levels its world at start, so
        # world -> ENU is [0,-1,0, 1,0,0, 0,0,1] while the IMU body -> FLU stays [0,0,1, 1,0,0, 0,1,0]
        # (PROGRESS P23; verified on 4 flight replays against the FC attitude).
        ob = rospy.get_param('~orientation_body_matrix', of)
        Mb = np.eye(4)
        Mb[:3, :3] = np.array(ob, dtype=float).reshape(3, 3)
        self.q_body = quaternion_from_matrix(Mb)
        self.remap_axes = rospy.get_param('~remap_axes', True)

        # LiDAR-to-FC MOUNT YAW. The permutation above is an exact 90-degree relabel, but the
        # LiDAR is physically mounted slightly yawed relative to the FC board. Fitting the /lidar_imu
        # gyro against the FC gyro (/mavros/imu/data_raw) gives w_fc = Rz(psi) * P * w_lidar, where psi
        # is ~fc_mount_yaw_deg (logs/mount_yaw_20260929/mount_yaw.cpp). So the FC body attitude is
        # R_world_lidar' * Rz(-psi), a BODY-side (right) multiply on orientation only. Position and
        # velocity are world-frame and do not depend on how the body is mounted. 0.0 = no correction.
        self.fc_mount_yaw_deg = float(rospy.get_param('~fc_mount_yaw_deg', 0.0))
        self.mount_quat = quaternion_from_euler(0.0, 0.0, -math.radians(self.fc_mount_yaw_deg))
        # FRAME ANCHOR (frame_anchor.py): the transform from FAST-LIO's frame to the one the FC
        # sees. The first pose after a (re)start, and the first after any FAST-LIO frame jump,
        # is anchored onto the FC's current EKF pose, so the FC keeps one continuous frame across
        # laserMapping and bridge restarts (it can reset neither heading nor, through MAVROS,
        # position). With no usable EKF (first boot) the old yaw-datum is used instead.
        # Off by default: the old behaviour exactly (yaw-datum once, never again). QuadRover robots
        # turn it on (fastlio_to_fc_quadrover.launch); the others keep the old bridge until
        # QuadRover is switched to for good (owner 2026-10-20).
        self.enable_reanchor = bool(rospy.get_param('~enable_reanchor', False))
        self.discontinuity = Discontinuity(rospy.get_param('~reanchor_gap_s', 1.0),
                                           rospy.get_param('~reanchor_jump_m', 0.5),
                                           rospy.get_param('~reanchor_jump_deg', 20.0))
        # sensor position in the FC body frame (forward, right, down): VISO_POS_X/Y/Z, read from
        # the FC at the first anchor; these params are the fallback if the FC does not answer
        self.sensor_offset = [float(rospy.get_param('~sensor_offset_x', 0.0)),
                              float(rospy.get_param('~sensor_offset_y', 0.0)),
                              float(rospy.get_param('~sensor_offset_z', 0.0))]
        self._sensor_offset_from_fc = False
        if self.enable_reanchor and _HAVE_MAVROS:
            # off the pose callback: /mavros/param/get can block while MAVROS loads the params
            threading.Thread(target=self.read_sensor_offset, daemon=True).start()
        self.anchor = None
        # on a (re)start, wait this long for the FC's EKF status and pose before falling back to
        # the yaw-datum: poses that arrive before EKF_STATUS_REPORT would otherwise always take
        # the first-boot path (seen in the side-by-side test: first pose 4 ms after start)
        self.ekf_wait_s = float(rospy.get_param('~ekf_wait_s', 3.0))
        self.t_start = None
        self.ekf_hist = PoseHistory()
        self.ekf_flags = None
        self.ekf_flags_rx = 0.0

        # Velocity: forward FAST-LIO's world-frame EKF velocity (now populated in
        # /Odometry.twist.twist.linear, see laserMapping.cpp) to ArduPilot as
        # VISION_SPEED_ESTIMATE. mavros listens on /mavros/vision_speed/speed_vector
        # (Vector3Stamped) when vision_speed/listen_twist:=false (current config), expecting
        # ENU world-frame velocity. We apply the SAME transform chain as position
        # (position_matrix then the frame anchor) so velocity and position share one frame.
        # NOTE: to actually fuse this, set EK3_SRC1_VELXY=6 and EK3_SRC1_VELZ=6 on the FC and
        # tune VISO_VEL_M_NSE. With those at 0 (None) the FC ignores it (safe no-op).
        self.publish_velocity = rospy.get_param('~publish_velocity', True)
        # Frame of odom twist.linear: "world" (FAST-LIO: IEKF velocity in camera_init) or "body"
        # (BIEVR-LIO: child/IMU frame, per nav_msgs convention). "body" is rotated into the backend's
        # world frame by the odom orientation first, then follows the same chain as position.
        self.twist_frame = rospy.get_param('~twist_frame', 'world')
        if self.twist_frame not in ('world', 'body'):
            raise ValueError("~twist_frame must be 'world' or 'body', got %r" % self.twist_frame)
        self.odom_topic = rospy.get_param('~odom_topic', '/Odometry')

        # FROZEN POSE IN THE AIR -> vision cut (see FrozenVisionCut above; owner decision 2026-11-02).
        # The thresholds are the planner bridge's FrozenPoseCheck's. ~frozen_enable false: the old bridge.
        self.frozen_enable = bool(rospy.get_param('~frozen_enable', True)) and _HAVE_MAVROS
        self.frozen = FrozenVisionCut(
            FrozenPoseCheck(eps_m=float(rospy.get_param('~frozen_eps_m', 1e-5)),
                            eps_q=float(rospy.get_param('~frozen_eps_q', 1e-6)),
                            hold_s=float(rospy.get_param('~frozen_hold_s', 0.4)),
                            min_n=int(rospy.get_param('~frozen_min_n', 4)),
                            cmd_speed=float(rospy.get_param('~frozen_cmd_speed', 0.2)),
                            gyro_min=float(rospy.get_param('~frozen_gyro_min', 0.2)),
                            twist_min=float(rospy.get_param('~frozen_twist_min', 0.2)),
                            ekf_speed_min=float(rospy.get_param('~frozen_ekf_speed_min', 0.2)),
                            evidence_age=float(rospy.get_param('~frozen_evidence_age', 0.25))),
            AirborneGate(channels=rospy.get_param('~frozen_motor_channels', [1, 2, 3, 4]),
                         pwm_min=float(rospy.get_param('~frozen_airborne_pwm', 1200.0)),
                         max_age=float(rospy.get_param('~frozen_motor_max_age', 1.0)),
                         landed_s=float(rospy.get_param('~frozen_landed_s', 3.0))),
            resume_n=int(rospy.get_param('~frozen_resume_n', 5)),
            ground_cut=bool(rospy.get_param('~frozen_ground_cut', False)))
        self.frozen_lock = threading.Lock()
        self._ground_text_t = -1e9

        # Publishers and Subscribers
        self.vision_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=50)
        self.vel_pub = rospy.Publisher('/mavros/vision_speed/speed_vector', Vector3Stamped, queue_size=50)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odometry_callback)
        self.st_pub = None
        if (self.enable_reanchor or self.frozen_enable) and _HAVE_MAVROS:
            self.st_pub = rospy.Publisher('/mavros/statustext/send', StatusText, queue_size=5)
        if self.enable_reanchor and _HAVE_MAVROS:
            rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.ekf_pose_callback, queue_size=20)
            rospy.Subscriber(rospy.get_param('~mavlink_from_topic', '/mavlink/from'), Mavlink,
                             self.mavlink_callback, queue_size=50)
        elif self.enable_reanchor:
            rospy.logwarn("mavros_msgs not available: no EKF to anchor to, yaw-datum only")
        if self.frozen_enable:
            rospy.Subscriber('/mavros/imu/data', Imu, self.frozen_imu_cb, queue_size=50)
            rospy.Subscriber('/mavros/state', State, self.frozen_state_cb, queue_size=5)
            rospy.Subscriber('/mavros/rc/out', RCOut, self.frozen_rcout_cb, queue_size=10)
            rospy.Subscriber('/mavros/setpoint_raw/local', PositionTarget, self.frozen_setpoint_cb, queue_size=20)
            rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.frozen_ekf_vel_cb,
                             queue_size=20)
            # ~frozen_status (std_msgs/String, JSON): vision on/cut, the repeat run, the evidence, the gate; 2 Hz
            # and at every event
            self.frozen_pub = rospy.Publisher('~frozen_status', String, queue_size=5, latch=True)
            rospy.Timer(rospy.Duration(0.5), lambda _e: self.publish_frozen_status())
            rospy.Timer(rospy.Duration(10.0), self.frozen_gate_watch, oneshot=True)
            rospy.loginfo("Frozen-pose vision cut ON (in the air%s): airborne = armed + lift motors (SERVO%s) >= %.0f us",
                          " and on the ground" if self.frozen.ground_cut else "; on the ground reported only",
                          self.frozen.gate.channels, self.frozen.gate.pwm_min)
        else:
            rospy.logwarn("Frozen-pose vision cut OFF (~frozen_enable false or no mavros_msgs)")

        rospy.loginfo("FAST-LIO to MAVROS bridge started (odom_topic=%s, position_matrix det=%.3f, remap_axes=%s, publish_velocity=%s, twist_frame=%s, fc_mount_yaw_deg=%.3f)",
                      self.odom_topic, np.linalg.det(self.Mpos), self.remap_axes, self.publish_velocity,
                      self.twist_frame, self.fc_mount_yaw_deg)
        rospy.loginfo("Publishing vision pose to /mavros/vision_pose/pose")
        if self.publish_velocity:
            rospy.loginfo("Publishing vision speed to /mavros/vision_speed/speed_vector")

    # ---- frozen pose in the air (FrozenVisionCut) ----
    def frozen_imu_cb(self, msg):
        w = msg.angular_velocity
        with self.frozen_lock:
            self.frozen.check.on_gyro(rospy.get_time(), math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z))

    def frozen_state_cb(self, msg):
        if not msg.connected:
            return   # no FC: the state ages out (neither airborne nor landed), never "disarmed"
        with self.frozen_lock:
            self.frozen.gate.on_state(rospy.get_time(), msg.armed, msg.mode)

    def frozen_rcout_cb(self, msg):
        with self.frozen_lock:
            self.frozen.gate.on_motors(rospy.get_time(), list(msg.channels))

    def frozen_setpoint_cb(self, msg):
        # a flight target's horizontal velocity feed-forward (PosVelAccel from the planner bridge)
        if msg.type_mask & (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY):
            return
        with self.frozen_lock:
            self.frozen.check.on_command(rospy.get_time(), math.hypot(msg.velocity.x, msg.velocity.y))

    def frozen_ekf_vel_cb(self, msg):
        v = msg.twist.linear
        with self.frozen_lock:
            self.frozen.check.on_ekf_velocity(rospy.get_time(), math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z))

    def frozen_gate_watch(self, _evt):
        """10 s after start: say so loudly if the airborne gate has no motor outputs (the cut could never fire)"""
        with self.frozen_lock:
            seen = self.frozen.gate.t_motors is not None
        if not seen:
            rospy.logwarn("Frozen-pose vision cut: no /mavros/rc/out yet (SERVO_OUTPUT_RAW, MAV1_RC_CHAN): "
                          "the airborne gate reads 'not airborne', so a frozen pose would NOT cut vision")

    def publish_frozen_status(self):
        with self.frozen_lock:
            s = self.frozen.status(rospy.get_time())
        self.frozen_pub.publish(String(data=json.dumps(s)))

    def frozen_filter(self, odom_msg):
        """True: this pose must not go to the FC (vision cut). Logs and reports the latch's events."""
        p, q = odom_msg.pose.pose.position, odom_msg.pose.pose.orientation
        lv = odom_msg.twist.twist.linear
        with self.frozen_lock:
            drop = self.frozen.on_pose(rospy.get_time(), (p.x, p.y, p.z, q.x, q.y, q.z, q.w),
                                       math.sqrt(lv.x * lv.x + lv.y * lv.y + lv.z * lv.z))
            events, self.frozen.events = self.frozen.events, []
        for kind, text in events:
            if kind == "cut":
                rospy.logerr(text)
                self.send_statustext("SLAM frozen in air: vision CUT", StatusText.CRITICAL)
            elif kind == "cut_ground":
                rospy.logerr(text)
                self.send_statustext("SLAM frozen on ground: vision CUT", StatusText.CRITICAL)
            elif kind == "resume":
                rospy.logwarn(text)
                self.send_statustext("SLAM moving again: vision resumed", StatusText.WARNING)
            else:
                rospy.logwarn(text)
                now = rospy.get_time()
                if now - self._ground_text_t > 30.0:
                    self._ground_text_t = now
                    self.send_statustext("SLAM frozen on ground: vision kept", StatusText.WARNING)
        if events:
            self.publish_frozen_status()
        if drop:
            rospy.logerr_throttle(2.0, "SLAM pose frozen: vision CUT (%s; latched until %s)" % (
                self.frozen.cut_where, "landed and SLAM moves" if self.frozen.cut_where == "air" else "SLAM moves"))
        return drop

    def odometry_callback(self, odom_msg):
        """Publish FAST-LIO odometry as MAVROS vision pose, keeping the original timestamp."""
        try:
            if self.frozen_enable and self.frozen_filter(odom_msg):
                return   # nothing to the FC, nor to the frame tracker: the cut is a gap (re-anchor on resume)
            pose_msg = PoseStamped()
            # System-time stamp (mavros-timesynced to FC), not the sensor's internal clock.
            pose_msg.header.stamp = rospy.Time.now() if self.restamp_system_time else odom_msg.header.stamp
            pose_msg.header.frame_id = self.vision_frame

            # Position: apply position_matrix (default identity -- odom position is already ENU).
            p = odom_msg.pose.pose.position
            x, y, z = self.Mpos.dot(np.array([p.x, p.y, p.z]))

            # Orientation: ENU axis remap (T18) as a quaternion similarity transform, decoupled
            # from the position transform. q_enu = qR * q_odom * qR^-1 (gimbal-lock free).
            q = odom_msg.pose.pose.orientation
            qin = (q.x, q.y, q.z, q.w)
            if self.remap_axes:
                tq = quaternion_multiply(quaternion_multiply(self.q_perm, qin),
                                         quaternion_conjugate(self.q_body))
            else:
                tq = qin
            # Body-side mount correction, BEFORE the yaw-datum capture, so heading 0 is the FC's forward
            if self.fc_mount_yaw_deg != 0.0:
                tq = quaternion_multiply(tq, self.mount_quat)

            # Frame anchor: one transform (heading + offset) for position, orientation and
            # velocity, chosen at the first pose and again after any FAST-LIO frame jump
            raw_p = np.array([x, y, z])
            stamp_s = odom_msg.header.stamp.to_sec()
            jump = self.discontinuity.update(stamp_s, raw_p, yaw_of(tq))
            if self.anchor is None and self.waiting_for_ekf():
                return
            if self.anchor is None or (jump and self.enable_reanchor):
                self.update_anchor(raw_p, tq, stamp_s, jump)
            x, y, z = self.anchor.apply_pos(raw_p)
            tq = self.anchor.apply_quat(tq)

            pose_msg.pose.position.x = float(x)
            pose_msg.pose.position.y = float(y)
            pose_msg.pose.position.z = float(z)
            pose_msg.pose.orientation.x = tq[0]
            pose_msg.pose.orientation.y = tq[1]
            pose_msg.pose.orientation.z = tq[2]
            pose_msg.pose.orientation.w = tq[3]

            self.vision_pub.publish(pose_msg)

            # ---- Velocity -> VISION_SPEED_ESTIMATE ----
            # /Odometry twist.linear is the world-frame (camera_init) EKF velocity. Apply the
            # SAME transform as position so the two share one frame: position_matrix -> ENU,
            # then the same frame-anchor heading rotation (z component unchanged).
            if self.publish_velocity:
                lv = odom_msg.twist.twist.linear
                v = np.array([lv.x, lv.y, lv.z])
                if self.twist_frame == 'body':
                    vq = quaternion_multiply(quaternion_multiply(qin, (v[0], v[1], v[2], 0.0)),
                                             quaternion_conjugate(qin))
                    v = np.array(vq[:3])
                vx, vy, vz = self.anchor.apply_vel(self.Mpos.dot(v))
                vel_msg = Vector3Stamped()
                vel_msg.header.stamp = pose_msg.header.stamp
                vel_msg.header.frame_id = self.vision_frame
                vel_msg.vector.x = float(vx)
                vel_msg.vector.y = float(vy)
                vel_msg.vector.z = float(vz)
                self.vel_pub.publish(vel_msg)

            rospy.loginfo_throttle(5,
                f"Vision pose: x={pose_msg.pose.position.x:.2f}, "
                f"y={pose_msg.pose.position.y:.2f}, "
                f"z={pose_msg.pose.position.z:.2f}")

        except Exception as e:
            rospy.logerr(f"Error processing odometry: {e}")

    def update_anchor(self, raw_p, raw_q, stamp_s, jump):
        """(re)choose the frame anchor: onto the EKF when it has a position, else the
        yaw-datum on the first pose; after a jump with no usable EKF the old anchor stays (the
        frame is then wrong, and nothing on this side can know the right one)"""
        ekf = self.usable_ekf_pose(stamp_s) if self.enable_reanchor else None
        if ekf is not None:
            new = Anchor.to_ekf(raw_p, raw_q, ekf[0], ekf[1], self.get_sensor_offset())
            if self.anchor is None:
                rospy.loginfo("Frame anchored to the EKF: %s", new.describe())
                self.send_statustext("SLAM frame anchored to EKF")
            else:
                # how far the FC frame would have jumped without the re-anchor
                d = np.linalg.norm(new.apply_pos(raw_p) - self.anchor.apply_pos(raw_p))
                dyaw = math.degrees(math.atan2(math.sin(new.psi - self.anchor.psi),
                                               math.cos(new.psi - self.anchor.psi)))
                rospy.logwarn("FAST-LIO frame jump (%s): re-anchored to the EKF, avoided %.2f m / %.0f deg; %s",
                              jump, d, dyaw, new.describe())
                self.send_statustext("SLAM re-anchored: %.1fm %.0fdeg" % (d, dyaw))
            self.anchor = new
        elif self.anchor is None:
            self.anchor = Anchor.yaw_datum(raw_q) if self.zero_initial_yaw else Anchor()
            rospy.loginfo("No usable EKF yet: %s", self.anchor.describe())
        else:
            rospy.logwarn("FAST-LIO frame jump (%s) but no usable EKF to anchor to: frame unchanged", jump)
            self.send_statustext("SLAM frame jump, no EKF to re-anchor")

    def waiting_for_ekf(self):
        """True while the first anchor should wait for the EKF (status or pose not in yet)"""
        if not (self.enable_reanchor and _HAVE_MAVROS):
            return False
        now = rospy.get_time()
        if self.t_start is None:
            self.t_start = now
        known = self.ekf_flags is not None and (not ekf_usable(self.ekf_flags) or self.ekf_hist.at(now) is not None)
        if known or now - self.t_start > self.ekf_wait_s:
            return False
        rospy.loginfo_throttle(1.0, "Waiting for the FC's EKF before the first anchor")
        return True

    def usable_ekf_pose(self, stamp_s):
        """(position, ENU yaw) of the EKF at stamp_s if EKF_STATUS_REPORT says it has a
        horizontal position (and is fresh), else None"""
        now = rospy.get_time()
        if self.ekf_flags is None or now - self.ekf_flags_rx > 2.0 or not ekf_usable(self.ekf_flags):
            return None
        return self.ekf_hist.at(stamp_s) or self.ekf_hist.at(now)

    def ekf_pose_callback(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        self.ekf_hist.add(msg.header.stamp.to_sec(), (p.x, p.y, p.z), yaw_of((q.x, q.y, q.z, q.w)))

    def mavlink_callback(self, msg):
        if msg.msgid != _EKF_STATUS_REPORT_MSGID:
            return
        # 5 floats then uint16 flags; MAVLink2 trims trailing zeros, so pad
        raw = b"".join(struct.pack("<Q", w) for w in msg.payload64).ljust(22, b"\0")
        self.ekf_flags = struct.unpack_from("<H", raw, 20)[0]
        self.ekf_flags_rx = rospy.get_time()

    def get_sensor_offset(self):
        if not self._sensor_offset_from_fc:
            rospy.logwarn("VISO_POS not read from the FC yet: anchoring with %s", self.sensor_offset)
        return self.sensor_offset

    def read_sensor_offset(self):
        """VISO_POS_X/Y/Z from the FC, retried every 5 s until it answers (thread)"""
        get = rospy.ServiceProxy('/mavros/param/get', ParamGet)
        while not rospy.is_shutdown():
            try:
                vals = []
                for name in ('VISO_POS_X', 'VISO_POS_Y', 'VISO_POS_Z'):
                    r = get(param_id=name)
                    if not r.success:
                        raise RuntimeError(name + ' not found')
                    vals.append(r.value.real)
                self.sensor_offset = vals
                self._sensor_offset_from_fc = True
                rospy.loginfo("Sensor offset from the FC: VISO_POS %.2f %.2f %.2f", *vals)
                return
            except Exception:  # noqa: BLE001 - FC or MAVROS not ready: keep trying
                rospy.sleep(5.0)

    def send_statustext(self, text, severity=None):
        if self.st_pub is not None:
            m = StatusText()
            m.severity = StatusText.NOTICE if severity is None else severity
            m.text = text[:50]
            self.st_pub.publish(m)

    def run(self):
        """Main loop"""
        rospy.spin()

if __name__ == '__main__':
    try:
        bridge = FastLioMavrosBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass
