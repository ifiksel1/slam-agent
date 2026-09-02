#!/usr/bin/env python3
"""Record GenZ-ICP trajectory + per-scan alpha (degeneracy telemetry) to CSV.

WHY THIS EXISTS: alpha = N_planar/(N_planar+N_non_planar) is the paper's adaptive
weighting coefficient (Registration.cpp:202), but it is a LOCAL VARIABLE -- the C++/ROS
path never publishes it (only the separate Python pipeline prints it). It IS recoverable:
RegisterFrame returns the planar / non-planar point sets of the final ICP iteration and the
ROS node publishes both as clouds, so alpha = width_planar/(width_planar+width_non_planar).
That is alpha of the converged iteration, which is exactly the number we want.

Requires the node to run with visualize:=true (gates publish_debug_clouds_).

Usage: genz_traj_sampler.py <out.csv> <max_seconds> [odom_topic]
"""
import csv, sys, threading
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

out_path = sys.argv[1]
max_sec = float(sys.argv[2])
odom_topic = sys.argv[3] if len(sys.argv) > 3 else "/genz/odometry"

lock = threading.Lock()
latest = {"planar": None, "non_planar": None}
rows = []


def on_planar(msg):
    with lock:
        latest["planar"] = msg.width * msg.height


def on_non_planar(msg):
    with lock:
        latest["non_planar"] = msg.width * msg.height


def on_odom(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    with lock:
        npl, nnp = latest["planar"], latest["non_planar"]
    # alpha is only defined once both debug clouds have arrived for this scan
    if npl is None or nnp is None or (npl + nnp) == 0:
        alpha, npl_o, nnp_o = "", "", ""
    else:
        alpha = "%.6f" % (float(npl) / float(npl + nnp))
        npl_o, nnp_o = npl, nnp
    rows.append([
        "%.9f" % msg.header.stamp.to_sec(),
        "%.6f" % p.x, "%.6f" % p.y, "%.6f" % p.z,
        "%.9f" % q.x, "%.9f" % q.y, "%.9f" % q.z, "%.9f" % q.w,
        alpha, npl_o, nnp_o,
    ])


def write_csv():
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["stamp", "x", "y", "z", "qx", "qy", "qz", "qw",
                    "alpha", "n_planar", "n_non_planar"])
        w.writerows(rows)
    rospy.loginfo("genz_traj_sampler: wrote %d rows -> %s", len(rows), out_path)


rospy.init_node("genz_traj_sampler", anonymous=True, disable_signals=True)
rospy.Subscriber(odom_topic, Odometry, on_odom, queue_size=2000)
rospy.Subscriber("/genz/planar_points", PointCloud2, on_planar, queue_size=50)
rospy.Subscriber("/genz/non_planar_points", PointCloud2, on_non_planar, queue_size=50)

# Hard time bound so this can never orphan and hold the container open.
# Write the CSV on ANY exit path: normal timeout, SIGINT from the runner's drain
# step (arrives as KeyboardInterrupt because we run with disable_signals=True),
# or rospy shutdown. Losing the recording to an unhandled signal is the whole
# reason this is wrapped so defensively.
rospy.loginfo("genz_traj_sampler: recording %s for <=%.0fs", odom_topic, max_sec)
try:
    rospy.sleep(max_sec)
except (KeyboardInterrupt, SystemExit, rospy.ROSInterruptException, rospy.ROSTimeMovedBackwardsException):
    pass
finally:
    write_csv()
