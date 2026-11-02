#!/usr/bin/env python3
"""SC-PGO input from a flight recording's LIVE FAST-LIO (offline tool).

  live_to_pgo_bag.py <out.bag> <in.bag> [<in.bag> ...] [--every N] [--voxel m]

Writes /Odometry (the live FAST-LIO odometry as recorded: camera_init -> body, the IMU) and
/cloud_registered_body: each /lidar_points scan (every N-th; the live stamps match /Odometry's 1:1) moved
into the IMU body frame with the live lidar->IMU extrinsic (config/hesai_jt128.yaml), range 0.4-60 m,
thinned to one point per voxel, stamped with the odometry's stamp. SC-PGO (alaserPGO) pairs a cloud with
the odometry of the same stamp.

Why not a FAST-LIO replay: a replay starts cold. The 2026-10-30 Mass Robotics recording starts with the
robot armed and moving; the replay's odometry ended 64 m from its start (live: 10.6 m) with metres of
height drift, and SC-PGO could not repair that. The live FAST-LIO had been running (converged) before the
recording began.
"""
import argparse

import numpy as np
from rosbags.rosbag1 import Reader, Writer
from rosbags.typesys import Stores, get_typestore

R_LI = np.array([[0.999976, 0.003838, -0.005696],
                 [-0.003852, 0.999990, -0.002412],
                 [0.005687, 0.002433, 0.999981]])
T_LI = np.array([-0.0076, 0.01363, 0.01271])

ap = argparse.ArgumentParser()
ap.add_argument("out"); ap.add_argument("bags", nargs="+")
ap.add_argument("--every", type=int, default=2)
ap.add_argument("--voxel", type=float, default=0.05)
a = ap.parse_args()

ts = get_typestore(Stores.ROS1_NOETIC)
PC2 = ts.types["sensor_msgs/msg/PointCloud2"]
PF = ts.types["sensor_msgs/msg/PointField"]
fields = [PF(name=n, offset=4 * i, datatype=7, count=1) for i, n in enumerate("x y z intensity".split())]
n_odom = n_cloud = k = 0
with Writer(a.out) as w:
    co = w.add_connection("/Odometry", "nav_msgs/msg/Odometry", typestore=ts)
    cc = w.add_connection("/cloud_registered_body", "sensor_msgs/msg/PointCloud2", typestore=ts)
    for path in a.bags:
        with Reader(path) as r:
            conns = [c for c in r.connections if c.topic in ("/Odometry", "/lidar_points")]
            for con, t, raw in r.messages(connections=conns):
                if con.topic == "/Odometry":
                    w.write(co, t, raw); n_odom += 1
                    continue
                k += 1
                if k % a.every:
                    continue
                m = ts.deserialize_ros1(raw, con.msgtype)
                step = m.point_step
                buf = np.frombuffer(m.data, np.uint8).reshape(-1, step)
                xyzi = buf[:, :16].copy().view("<f4").reshape(-1, 4)
                rng = np.linalg.norm(xyzi[:, :3], axis=1)
                ok = np.isfinite(rng) & (rng > 0.4) & (rng < 60.0)
                P = xyzi[ok, :3].astype(np.float64) @ R_LI.T + T_LI
                I = xyzi[ok, 3]
                key = np.floor(P / a.voxel).astype(np.int64)
                kk = (key[:, 0] + 2**20) * 2**42 + (key[:, 1] + 2**20) * 2**21 + (key[:, 2] + 2**20)
                _, first = np.unique(kk, return_index=True)
                out = np.c_[P[first], I[first]].astype("<f4")
                msg = PC2(header=m.header.__class__(seq=m.header.seq, stamp=m.header.stamp, frame_id="body"), height=1,
                          width=len(out), fields=fields, is_bigendian=False, point_step=16,
                          row_step=16 * len(out), data=np.frombuffer(out.tobytes(), np.uint8), is_dense=True)
                w.write(cc, t, ts.serialize_ros1(msg, "sensor_msgs/msg/PointCloud2"))
                n_cloud += 1
print("odom %d, clouds %d (of %d scans)" % (n_odom, n_cloud, k))
