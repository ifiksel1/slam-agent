#!/usr/bin/env python3
# Re-publish the three poses as named TF frames under a common parent so Foxglove's
# 3D panel shows them as comparable coordinate-axis triads. Read-only: only emits /tf.
#   /Odometry                      -> viz_odom    (raw FAST-LIO)
#   /mavros/vision_pose/pose       -> viz_vision  (bridge output to FC)
#   /mavros/local_position/pose    -> viz_fc      (EKF fused)
# Params: ~parent_frame (default map), ~zero_position (default false; true=overlay at origin
# to compare orientation only).
import rospy, tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry

class VizFrames:
    def __init__(self):
        rospy.init_node('viz_frames')
        self.parent = rospy.get_param('~parent_frame', 'map')
        self.zero = rospy.get_param('~zero_position', False)
        self.br = tf2_ros.TransformBroadcaster()
        rospy.Subscriber('/Odometry', Odometry,
                         lambda m: self.emit('viz_odom', m.pose.pose), queue_size=20)
        rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped,
                         lambda m: self.emit('viz_vision', m.pose), queue_size=20)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped,
                         lambda m: self.emit('viz_fc', m.pose), queue_size=20)
        rospy.loginfo("viz_frames: publishing viz_odom / viz_vision / viz_fc under '%s' (zero_pos=%s)",
                      self.parent, self.zero)

    def emit(self, child, pose):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.parent
        t.child_frame_id = child
        if self.zero:
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
        else:
            t.transform.translation.x = pose.position.x
            t.transform.translation.y = pose.position.y
            t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation
        self.br.sendTransform(t)


if __name__ == '__main__':
    VizFrames()
    rospy.spin()
