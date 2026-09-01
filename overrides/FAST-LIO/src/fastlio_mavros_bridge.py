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

import rospy
import numpy as np
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from tf.transformations import (quaternion_multiply, quaternion_conjugate,
                                quaternion_from_matrix, quaternion_from_euler,
                                euler_from_quaternion)

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
        # Disable with ~zero_initial_yaw:=false to keep the absolute heading.
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
        self.remap_axes = rospy.get_param('~remap_axes', True)
        self.yaw0 = None              # captured once, radians
        self.cos_y0 = 1.0             # cos(-yaw0)
        self.sin_y0 = 0.0             # sin(-yaw0)
        self.yaw0_quat = (0.0, 0.0, 0.0, 1.0)  # Rz(-yaw0) as quaternion [x,y,z,w]

        # Velocity: forward FAST-LIO's world-frame EKF velocity (now populated in
        # /Odometry.twist.twist.linear, see laserMapping.cpp) to ArduPilot as
        # VISION_SPEED_ESTIMATE. mavros listens on /mavros/vision_speed/speed_vector
        # (Vector3Stamped) when vision_speed/listen_twist:=false (current config), expecting
        # ENU world-frame velocity. We apply the SAME transform chain as position
        # (position_matrix then Rz(-yaw0)) so velocity and position share one frame.
        # NOTE: to actually fuse this, set EK3_SRC1_VELXY=6 and EK3_SRC1_VELZ=6 on the FC and
        # tune VISO_VEL_M_NSE. With those at 0 (None) the FC ignores it (safe no-op).
        self.publish_velocity = rospy.get_param('~publish_velocity', True)

        # Publishers and Subscribers
        self.vision_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=50)
        self.vel_pub = rospy.Publisher('/mavros/vision_speed/speed_vector', Vector3Stamped, queue_size=50)
        self.odom_sub = rospy.Subscriber('/Odometry', Odometry, self.odometry_callback)

        rospy.loginfo("FAST-LIO to MAVROS bridge started (position_matrix det=%.3f, remap_axes=%s, publish_velocity=%s)",
                      np.linalg.det(self.Mpos), self.remap_axes, self.publish_velocity)
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
                                         quaternion_conjugate(self.q_perm))
            else:
                tq = qin

            # Yaw-datum: capture yaw0 from the first pose, then rotate every pose by
            # -yaw0 about vertical so the FC frame starts at heading 0 (forward->+x).
            if self.zero_initial_yaw:
                if self.yaw0 is None:
                    self.yaw0 = euler_from_quaternion(tq)[2]  # ENU yaw, radians
                    self.cos_y0 = math.cos(-self.yaw0)
                    self.sin_y0 = math.sin(-self.yaw0)
                    self.yaw0_quat = quaternion_from_euler(0.0, 0.0, -self.yaw0)
                    rospy.loginfo("Yaw-datum captured: yaw0=%.1f deg -> heading zeroed",
                                  math.degrees(self.yaw0))
                # rotate position about z by -yaw0 (z unchanged)
                x, y = self.cos_y0 * x - self.sin_y0 * y, self.sin_y0 * x + self.cos_y0 * y
                # rotate orientation by the SAME -yaw0
                tq = quaternion_multiply(self.yaw0_quat, tq)

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
            # then the same Rz(-yaw0) yaw-datum rotation (z component unchanged).
            if self.publish_velocity:
                lv = odom_msg.twist.twist.linear
                vx, vy, vz = self.Mpos.dot(np.array([lv.x, lv.y, lv.z]))
                if self.zero_initial_yaw and self.yaw0 is not None:
                    vx, vy = self.cos_y0 * vx - self.sin_y0 * vy, self.sin_y0 * vx + self.cos_y0 * vy
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

    def run(self):
        """Main loop"""
        rospy.spin()

if __name__ == '__main__':
    try:
        bridge = FastLioMavrosBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass
