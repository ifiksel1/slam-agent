#!/usr/bin/env python3
#
# zero_initial_yaw.py — yaw-only initial-heading zeroing for FAST-LIVO2 vision pose.
#
# WHY: FAST-LIVO2 runs gravity-alignment at init (z-up is correct, roll/pitch good),
# but the world yaw is arbitrary/non-repeatable because the upside-down Ouster IMU +
# Eigen FromTwoVectors picks a yaw that depends on the exact startup tilt. At rest &
# level we saw base_link expressed in camera_init at ~ -81 deg yaw, which leaked into
# /mavros/vision_pose/pose as an ~81 deg flip.
#
# WHAT: After SLAM init, measure yaw0 = yaw component of base_link in camera_init,
# then publish a LATCHED static TF  odom -> camera_init = pure Rz(sign*yaw0), T=[0,0,0].
# With sign = -1 (default) this cancels the initial yaw so that, through the chain
#   map -> odom -> camera_init(Rz(-yaw0)) -> aft_mapped -> base_link,
# base_link expressed in map reads ~identity yaw at rest. Roll/pitch (gravity-aligned
# z-up) are PRESERVED — we only rotate about world Z.
#
# The node measures ONCE (after a settle + circular-mean of several samples), publishes
# the latched static transform, then spins to keep the StaticTransformBroadcaster alive.

import math

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def main():
    rospy.init_node("zero_initial_yaw")

    # Static TF this node owns:  parent_frame -> child_frame = Rz(sign * yaw0)
    parent_frame = rospy.get_param("~parent_frame", "odom")
    child_frame = rospy.get_param("~child_frame", "camera_init")

    # Transform we MEASURE to extract the initial yaw (independent of the TF we publish):
    #   yaw0 = yaw of measure_child expressed in measure_parent.
    measure_parent = rospy.get_param("~measure_parent", "camera_init")
    measure_child = rospy.get_param("~measure_child", "base_link")

    settle_time = rospy.get_param("~settle_time", 5.0)       # seconds after first TF
    num_samples = int(rospy.get_param("~num_samples", 20))    # averaged samples
    sample_interval = rospy.get_param("~sample_interval", 0.1)
    # Publish Rz(sign * yaw0). sign=-1 cancels the measured yaw. Flip to +1 if the
    # resulting world yaw lands at ~ -2*yaw0 instead of ~0.
    sign = rospy.get_param("~sign", -1.0)

    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)

    # 1) Wait for the measurement transform (needs FAST-LIVO2 camera_init->aft_mapped
    #    plus the static aft_mapped->base_link). Does NOT depend on odom->camera_init,
    #    so there is no chicken-and-egg with the transform we are about to publish.
    rospy.loginfo("zero_initial_yaw: waiting for TF %s -> %s ...", measure_parent, measure_child)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            buf.lookup_transform(measure_parent, measure_child, rospy.Time(0), rospy.Duration(0.5))
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()

    if rospy.is_shutdown():
        return

    rospy.loginfo("zero_initial_yaw: TF available; settling %.1fs before measuring", settle_time)
    rospy.sleep(settle_time)

    # 2) Circular-mean the yaw over several samples (robust to wrap-around / jitter).
    sum_sin = 0.0
    sum_cos = 0.0
    n = 0
    for _ in range(num_samples):
        if rospy.is_shutdown():
            break
        try:
            tf = buf.lookup_transform(measure_parent, measure_child, rospy.Time(0), rospy.Duration(0.5))
            q = tf.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            sum_sin += math.sin(yaw)
            sum_cos += math.cos(yaw)
            n += 1
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
        rospy.sleep(sample_interval)

    if n == 0:
        rospy.logerr("zero_initial_yaw: no TF samples; publishing identity %s->%s", parent_frame, child_frame)
        yaw0 = 0.0
    else:
        yaw0 = math.atan2(sum_sin, sum_cos)

    correction_yaw = sign * yaw0
    rospy.loginfo(
        "zero_initial_yaw: yaw0=%.4f rad (%.2f deg) over %d samples -> publishing %s->%s = Rz(%.4f rad / %.2f deg)",
        yaw0, math.degrees(yaw0), n, parent_frame, child_frame,
        correction_yaw, math.degrees(correction_yaw),
    )

    # 3) Latched static TF parent->child = Rz(correction_yaw), zero translation.
    static_br = tf2_ros.StaticTransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
    qz = quaternion_from_euler(0.0, 0.0, correction_yaw)
    t.transform.rotation.x = qz[0]
    t.transform.rotation.y = qz[1]
    t.transform.rotation.z = qz[2]
    t.transform.rotation.w = qz[3]
    static_br.sendTransform(t)

    rospy.loginfo("zero_initial_yaw: static %s->%s published (latched); spinning to keep alive.",
                  parent_frame, child_frame)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
