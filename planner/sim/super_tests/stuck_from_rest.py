#!/usr/bin/env python3
"""stuck_from_rest.py: SUPER-alone leak repro (SITL only, no FC): a static hover pose wedged next to an obstacle on the
Mass Robotics 4F map, the map around it as /planner/cloud, then one goal ~1 m away.
Planner frame = Rz(-yaw)(map - start), z - LIDAR_UP (render_node.py / mr4f.env)."""
import math, sys, time
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped

pcd, rx, ry, rz, gx, gy, gz = sys.argv[1], *map(float, sys.argv[2:8])
goal_every = float(sys.argv[8]) if len(sys.argv) > 8 else 0.0   # >0: re-send the goal every N s
START = (0.01, -0.01, -91.1); LIDAR_UP = 0.25

def load(path):
    with open(path, "rb") as f:
        while True:
            l = f.readline()
            if l.startswith(b"POINTS"): n = int(l.split()[1])
            if l.startswith(b"DATA"): break
        a = np.frombuffer(f.read(n * 16), dtype="<f4").reshape(n, 4)[:, :3].astype(np.float64)
    y = math.radians(START[2]); c, s = math.cos(y), math.sin(y)
    d = a - np.array([START[0], START[1], LIDAR_UP])
    return np.c_[c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1], d[:, 2]]

pts = load(pcd)
robot = np.array([rx, ry, rz])
near = pts[np.linalg.norm(pts - robot, axis=1) < 12.0]
print(f"cloud {len(near)} pts; nearest obstacle to start {np.min(np.linalg.norm(near - robot, axis=1)):.3f} m", flush=True)

rospy.init_node("leak_feeder")
op = rospy.Publisher("/planner/odom", Odometry, queue_size=10)
cp = rospy.Publisher("/planner/cloud", PointCloud2, queue_size=2)
gp = rospy.Publisher("/planning/click_goal", PoseStamped, queue_size=1, latch=True)
c = np.c_[near, np.full(len(near), 50.0)].astype("<f4")
cloud = PointCloud2(height=1, width=len(c), is_bigendian=False, point_step=16, row_step=16 * len(c), is_dense=True,
                    fields=[PointField(name=n, offset=4 * i, datatype=7, count=1) for i, n in enumerate("xyz") ] +
                           [PointField(name="intensity", offset=12, datatype=7, count=1)], data=c.tobytes())
cloud.header.frame_id = "world"
t0 = time.time(); sent = 0; k = 0
r = rospy.Rate(100)
while not rospy.is_shutdown():
    now = rospy.Time.now()
    o = Odometry(); o.header.stamp = now; o.header.frame_id = "world"; o.child_frame_id = "body"
    o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = rx, ry, rz
    o.pose.pose.orientation.w = 1.0
    op.publish(o)
    if k % 10 == 0:
        cloud.header.stamp = now; cp.publish(cloud)
    el = time.time() - t0
    if math.isnan(gx): pass
    elif (sent == 0 and el > 8.0) or (goal_every > 0 and sent and el > 8.0 + sent * goal_every):
        g = PoseStamped(); g.header.stamp = now; g.header.frame_id = "world"
        g.pose.position.x, g.pose.position.y, g.pose.position.z = gx, gy, gz
        g.pose.orientation.w = 1.0
        gp.publish(g); sent += 1
        print(f"{el:.1f}s goal {sent} -> ({gx},{gy},{gz})", flush=True)
    k += 1
    r.sleep()
