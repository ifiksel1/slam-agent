#!/usr/bin/env python3
"""
BIEVR-LIO health adapter (slam_backend:=bievr, PROGRESS P23).

drift_monitor consumes /fastlio_health (Float32MultiArray, no header), which laserMapping fills
from inside its update loop. BIEVR-LIO (our patch, overrides/BIEVR-LIO) publishes the same
quantities as three Vector3Stamped topics per scan, all with header.stamp = scan end time:

  /bievr_lio/health/registration = (effective points, mean Huber cost per effective point, source points)
  /bievr_lio/health/eig_pos      = translation-block Hessian eigenvalues, ascending
  /bievr_lio/health/compute      = (processing ms, min rotation-block eigenvalue, 0)

This node joins them by stamp and republishes them in the /fastlio_health layout, so
drift_monitor runs unchanged with ~health_topic:=/bievr_health:

  data[0] effective points   (FAST-LIO: effct_feat_num)   -> drift_monitor "matched"
  data[1] mean residual cost (FAST-LIO: res_mean_last)     -- different units, not comparable
  data[2] processing ms      (FAST-LIO: EKF update ms)
  data[3] source points      (FAST-LIO: feats_down_size)
  data[4..6] eig_pos min/mid/max                          -> drift_monitor "eig_min" = data[4]
  data[7] age of measurement ms = now - scan end stamp     -> drift_monitor latency
  data[8] min rotation-block eigenvalue (extra; drift_monitor ignores it)

The Hessian here is BIEVR's height-image residual Hessian, NOT FAST-LIO's point-to-plane one,
and "effective points" is counted differently, so drift_monitor thresholds must be calibrated
for this backend (scripts/replay/bievr_calibrate.py). Never reuse the FAST-LIO numbers.

If any of the three inputs stops, nothing is published, so drift_monitor's health watchdog
fires exactly as it does when laserMapping dies.
"""

import rospy
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32MultiArray

PARTS = ('registration', 'eig_pos', 'compute')


class BievrHealth:
    def __init__(self):
        rospy.init_node('bievr_health')
        ns = rospy.get_param('~bievr_ns', '/bievr_lio')
        self.pub = rospy.Publisher(rospy.get_param('~health_topic', '/bievr_health'),
                                   Float32MultiArray, queue_size=50)
        self.pending = {}  # stamp (ns) -> {part: Vector3}
        for part in PARTS:
            rospy.Subscriber('%s/health/%s' % (ns, part), Vector3Stamped,
                             self.cb, callback_args=part, queue_size=50)

    def cb(self, msg, part):
        key = msg.header.stamp.to_nsec()
        entry = self.pending.setdefault(key, {})
        entry[part] = msg.vector
        if len(entry) < len(PARTS):
            # Drop stale partial entries (a part that never arrives must not leak memory).
            if len(self.pending) > 20:
                for k in sorted(self.pending)[:-20]:
                    del self.pending[k]
            return
        del self.pending[key]
        reg, eig, comp = entry['registration'], entry['eig_pos'], entry['compute']
        age_ms = (rospy.Time.now() - msg.header.stamp).to_sec() * 1000.0
        out = Float32MultiArray()
        out.data = [reg.x, reg.y, comp.x, reg.z, eig.x, eig.y, eig.z, age_ms, comp.y]
        self.pub.publish(out)


if __name__ == '__main__':
    BievrHealth()
    rospy.spin()
