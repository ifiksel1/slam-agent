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

import os
import struct
import sys
import threading

import rospy
import numpy as np
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from tf.transformations import (quaternion_multiply, quaternion_conjugate,
                                quaternion_from_matrix, quaternion_from_euler)

# frame_anchor.py sits next to this file; catkin's relay runs it with __file__ set to the source
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from frame_anchor import Anchor, Discontinuity, PoseHistory, ekf_usable, yaw_of  # noqa: E402

try:
    from mavros_msgs.msg import Mavlink, StatusText
    from mavros_msgs.srv import ParamGet
    _HAVE_MAVROS = True
except ImportError:
    _HAVE_MAVROS = False

_EKF_STATUS_REPORT_MSGID = 193

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

        # Publishers and Subscribers
        self.vision_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=50)
        self.vel_pub = rospy.Publisher('/mavros/vision_speed/speed_vector', Vector3Stamped, queue_size=50)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odometry_callback)
        self.st_pub = None
        if self.enable_reanchor and _HAVE_MAVROS:
            rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.ekf_pose_callback, queue_size=20)
            rospy.Subscriber(rospy.get_param('~mavlink_from_topic', '/mavlink/from'), Mavlink,
                             self.mavlink_callback, queue_size=50)
            self.st_pub = rospy.Publisher('/mavros/statustext/send', StatusText, queue_size=5)
        elif self.enable_reanchor:
            rospy.logwarn("mavros_msgs not available: no EKF to anchor to, yaw-datum only")

        rospy.loginfo("FAST-LIO to MAVROS bridge started (odom_topic=%s, position_matrix det=%.3f, remap_axes=%s, publish_velocity=%s, twist_frame=%s, fc_mount_yaw_deg=%.3f)",
                      self.odom_topic, np.linalg.det(self.Mpos), self.remap_axes, self.publish_velocity,
                      self.twist_frame, self.fc_mount_yaw_deg)
        rospy.loginfo("Publishing vision pose to /mavros/vision_pose/pose")
        if self.publish_velocity:
            rospy.loginfo("Publishing vision speed to /mavros/vision_speed/speed_vector")

    def odometry_callback(self, odom_msg):
        """Publish FAST-LIO odometry as MAVROS vision pose, keeping the original timestamp."""
        try:
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

    def send_statustext(self, text):
        if self.st_pub is not None:
            m = StatusText()
            m.severity = StatusText.NOTICE
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
