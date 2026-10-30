"""Make a simulated mission's bag ready to watch in Foxglove.

Also /map_low (the map between MAP_LOW z0,z1, default 0.05,1.2 m) for top-down views.
Adds the map as /map (sensor_msgs/PointCloud2 in "world", 15 cm grid, every 5 s so it is there
wherever you seek), thins the live lidar /cloud_registered to a 10 cm grid, copies everything else
as is except /fsm_node/fsm/path (SUPER republishes the whole path so far many times a second,
~770 MB a run; /perfect_tracking/path shows the same trail), and writes the bag LZ4-compressed.

usage: foxglove_bag.py <run.bag> <map.pcd> <out.bag> [odom_topic cloud_topic yaw_deg sx sy sz]

The optional arguments are for L2 closed-loop bags (planner frame): the robot pose from odom_topic
(/planner/odom), the live cloud from cloud_topic (/planner/cloud), and the map moved into the planner
frame with /sim/map_to_planner (planner = Rz(-yaw) (map - start)).

Every other topic passes through as recorded (the drive-and-fly closed loop's /planner/landing_markers,
/track_planner/path, /planning/drive_goal, /planner_bridge/mode_code, ...). Added for plots, at 20 Hz
from the odometry: /viz/speed (std_msgs/Float64, horizontal speed m/s) and /viz/height (std_msgs/Float64,
odometry z, m); and /viz/phase (std_msgs/Float64) from /mission_executive/status for a plot band:
0 waiting, 1 takeoff, 2 fly leg, 3 drive leg, 4 landing check, 5 landing, 6 hold/unreachable, 7 done,
-1 stopped.
If the source has no /tf (the closed-loop bags), /tf (tf2_msgs/TFMessage) <odom frame_id> -> "drone" is added
from every odom_topic message (its own rate and stamps), so a 3D panel can follow the robot as in the
planner-only bags.
"""
import json
import math
import os
import sys

import numpy as np
from rosbags.rosbag1 import Reader, Writer
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

src, pcd, dst = sys.argv[1:4]
ODOM, CLOUD = (sys.argv[4], sys.argv[5]) if len(sys.argv) > 5 else ("/lidar_slam/odom", "/cloud_registered")
MAP_TF = (float(sys.argv[6]), np.array([float(x) for x in sys.argv[7:10]])) if len(sys.argv) > 9 else None
ts = get_typestore(Stores.ROS1_NOETIC)
PointCloud2 = ts.types["sensor_msgs/msg/PointCloud2"]
PointField = ts.types["sensor_msgs/msg/PointField"]
Header = ts.types["std_msgs/msg/Header"]
Time = ts.types["builtin_interfaces/msg/Time"]


def grid(P, g):
    k = np.floor(P[:, :3] / g).astype(np.int64)
    _, i = np.unique(k, axis=0, return_index=True)
    return P[np.sort(i)]


def cloud(P, stamp_ns, frame="world"):
    P = np.ascontiguousarray(P[:, :4], dtype="<f4")
    fields = [PointField(name=n, offset=4 * i, datatype=7, count=1) for i, n in enumerate(("x", "y", "z", "intensity"))]
    hdr = Header(seq=0, stamp=Time(sec=int(stamp_ns // 1_000_000_000), nanosec=int(stamp_ns % 1_000_000_000)), frame_id=frame)
    return PointCloud2(header=hdr, height=1, width=len(P), fields=fields, is_bigendian=False, point_step=16,
                       row_step=16 * len(P), data=np.frombuffer(P.tobytes(), dtype=np.uint8), is_dense=True)


DROP = {"/fsm_node/fsm/path"}

# the robot as SUPER plans it (a sphere) and as it is (a box), coloured by clearance to the map
ROBOT_R = 0.44
HALF = np.array([0.508, 0.457, 0.203]) / 2  # length (forward), width, height
Marker = ts.types["visualization_msgs/msg/Marker"]
Pose = ts.types["geometry_msgs/msg/Pose"]
Point = ts.types["geometry_msgs/msg/Point"]
Quaternion = ts.types["geometry_msgs/msg/Quaternion"]
Vector3 = ts.types["geometry_msgs/msg/Vector3"]
ColorRGBA = ts.types["std_msgs/msg/ColorRGBA"]
Duration = ts.types["builtin_interfaces/msg/Duration"]
BANDS = ((0.3, (0.20, 0.80, 0.35)), (0.1, (0.95, 0.80, 0.15)), (0.0, (1.00, 0.50, 0.10)), (-1e9, (0.90, 0.15, 0.15)))


def band(margin, alpha):
    for lo, c in BANDS:
        if margin > lo:
            return ColorRGBA(r=c[0], g=c[1], b=c[2], a=alpha)


def quat_matrix(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def marker(stamp_ns, ns, mtype, pos, quat, scale, color, points=(), text=""):
    hdr = Header(seq=0, stamp=Time(sec=int(stamp_ns // 1_000_000_000), nanosec=int(stamp_ns % 1_000_000_000)), frame_id="world")
    return Marker(header=hdr, ns=ns, id=0, type=mtype, action=0,
                  pose=Pose(position=Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2])),
                            orientation=Quaternion(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3]))),
                  scale=Vector3(x=float(scale[0]), y=float(scale[1]), z=float(scale[2])), color=color,
                  lifetime=Duration(sec=0, nanosec=300_000_000), frame_locked=False,
                  points=[Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in points], colors=[], text=text,
                  mesh_resource="", mesh_use_embedded_materials=False)


def body_clearance(tree, P, pos, R):
    """distance from the oriented box to the nearest map point (negative inside)"""
    idx = tree.query_ball_point(pos, 1.5)
    if not idx:
        return 1.5
    q = np.abs((P[idx] - pos) @ R) - HALF   # into the body frame
    out = np.linalg.norm(np.maximum(q, 0), axis=1)
    inside = np.minimum(q.max(axis=1), 0)
    return float(np.min(np.where(out > 0, out, inside)))

raw = open(pcd, "rb").read()
k = raw.index(b"DATA binary\n") + len(b"DATA binary\n")
M = np.frombuffer(raw[k:], dtype="<f4").reshape(-1, 4).copy()
if MAP_TF is not None:
    yaw, start = math.radians(MAP_TF[0]), MAP_TF[1]
    c, s_ = math.cos(-yaw), math.sin(-yaw)
    R = np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]])
    M[:, :3] = ((M[:, :3].astype(np.float64) - start) @ R.T).astype(np.float32)
from scipy.spatial import cKDTree
MAP_TREE = cKDTree(M[:, :3].astype(np.float64))   # clearance against the full-resolution map
MAP_P = M[:, :3].astype(np.float64)
M = grid(M, 0.15)
print("map: %d points at 15 cm" % len(M))
# /map_low: the map between MAP_LOW z0,z1 (m, planner frame; default 0.05,1.2): walls, doors and obstacles
# without floors or ceilings, for top-down views of driving (from above, ceilings hide everything)
LOW_Z = [float(v) for v in os.environ.get("MAP_LOW", "0.05,1.2").split(",")]
M_LOW = M[(M[:, 2] >= LOW_Z[0]) & (M[:, 2] <= LOW_Z[1])]
print("map_low: %d points, z %.2f..%.2f" % (len(M_LOW), LOW_Z[0], LOW_Z[1]))

w = Writer(dst)
w.set_compression(Writer.CompressionFormat.LZ4)
with Reader(src) as r, w:
    conns = {}
    for c in r.connections:
        if c.topic in DROP:
            continue
        if c.topic not in conns:
            # reuse the source definition: some types (quadrotor_msgs) are not in the built-in type store
            conns[c.topic] = w.add_connection(c.topic, c.msgtype, msgdef=c.msgdef.data, md5sum=c.digest)
    map_conn = w.add_connection("/map", "sensor_msgs/msg/PointCloud2", typestore=ts)
    map_low_conn = w.add_connection("/map_low", "sensor_msgs/msg/PointCloud2", typestore=ts)
    mk = {k: w.add_connection(k, "visualization_msgs/msg/Marker", typestore=ts)
          for k in ("/robot/sphere", "/robot/body", "/robot/nearest", "/robot/clearance")}
    Float64 = ts.types["std_msgs/msg/Float64"]
    viz = {k: w.add_connection(k, "std_msgs/msg/Float64", typestore=ts) for k in ("/viz/speed", "/viz/height", "/viz/phase")}
    PHASE = {"WAIT_ARMED": 0, "TAKEOFF": 1, "LAND_CHECK": 4, "LAND": 5, "HOLD_SLAM": 6, "UNREACHABLE": 6,
             "DONE": 7, "STOPPED": -1}
    # /tf <odom frame> -> drone from the odometry, when the source has none (so the 3D panel can follow "drone")
    tf_conn = None
    if not any(c.topic == "/tf" for c in r.connections):
        if "tf2_msgs/msg/TFMessage" not in ts.types:
            ts.register(get_types_from_msg("geometry_msgs/TransformStamped[] transforms", "tf2_msgs/msg/TFMessage"))
        tf_conn = w.add_connection("/tf", "tf2_msgs/msg/TFMessage", typestore=ts)
    next_map = None
    next_mk = 0
    n_scan = 0
    worst = [9.0, 9.0]
    for con, t, data in r.messages():
        if con.topic in DROP:
            continue
        if next_map is None or t >= next_map:
            w.write(map_conn, t, ts.serialize_ros1(cloud(M, t), "sensor_msgs/msg/PointCloud2"))
            w.write(map_low_conn, t, ts.serialize_ros1(cloud(M_LOW, t), "sensor_msgs/msg/PointCloud2"))
            next_map = t + 5_000_000_000
        if tf_conn is not None and con.topic == ODOM:
            o = ts.deserialize_ros1(data, con.msgtype)
            p, q = o.pose.pose.position, o.pose.pose.orientation
            tfm = ts.types["tf2_msgs/msg/TFMessage"](transforms=[ts.types["geometry_msgs/msg/TransformStamped"](
                header=o.header, child_frame_id="drone", transform=ts.types["geometry_msgs/msg/Transform"](
                    translation=Vector3(x=p.x, y=p.y, z=p.z), rotation=q))])
            w.write(tf_conn, t, ts.serialize_ros1(tfm, "tf2_msgs/msg/TFMessage"))
        if con.topic == ODOM and t >= next_mk:
            next_mk = t + 50_000_000   # 20 Hz
            o = ts.deserialize_ros1(data, con.msgtype)
            p, q = o.pose.pose.position, o.pose.pose.orientation
            pos = np.array([p.x, p.y, p.z])
            quat = (q.x, q.y, q.z, q.w)
            tw = o.twist.twist.linear
            w.write(viz["/viz/speed"], t, ts.serialize_ros1(Float64(data=float(math.hypot(tw.x, tw.y))), "std_msgs/msg/Float64"))
            w.write(viz["/viz/height"], t, ts.serialize_ros1(Float64(data=float(p.z)), "std_msgs/msg/Float64"))
            R = quat_matrix(*quat)
            d, i = MAP_TREE.query(pos)
            sphere_m = d - ROBOT_R
            body_m = body_clearance(MAP_TREE, MAP_P, pos, R)
            worst = [min(worst[0], sphere_m), min(worst[1], body_m)]
            for key, m in (
                ("/robot/sphere", marker(t, "sphere", 2, pos, (0, 0, 0, 1), (2 * ROBOT_R,) * 3, band(sphere_m, 0.30))),
                ("/robot/body", marker(t, "body", 1, pos, quat, 2 * HALF, band(body_m, 0.90))),
                ("/robot/nearest", marker(t, "nearest", 5, (0, 0, 0), (0, 0, 0, 1), (0.02, 0, 0), band(body_m, 1.0),
                                          points=(pos, MAP_P[i]))),
                ("/robot/clearance", marker(t, "clearance", 9, pos + [0, 0, 0.6], (0, 0, 0, 1), (0, 0, 0.18),
                                            ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0),
                                            text="sphere %+.2f m   body %.2f m" % (sphere_m, body_m))),
            ):
                w.write(mk[key], t, ts.serialize_ros1(m, "visualization_msgs/msg/Marker"))
        if con.topic == "/mission_executive/status":
            st = json.loads(ts.deserialize_ros1(data, con.msgtype).data)
            ph = st.get("phase")
            v = (3 if st.get("type") == "drive" else 2) if ph == "LEG" else PHASE.get(ph)
            if v is not None:
                w.write(viz["/viz/phase"], t, ts.serialize_ros1(Float64(data=float(v)), "std_msgs/msg/Float64"))
        if con.topic == CLOUD:
            m = ts.deserialize_ros1(data, con.msgtype)
            a = np.frombuffer(m.data, dtype=np.uint8).reshape(-1, m.point_step)
            xyz = a[:, :12].copy().view(np.float32).reshape(-1, 3)
            S = grid(np.c_[xyz, np.zeros(len(xyz), np.float32)], 0.10)
            stamp = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
            w.write(conns[con.topic], t, ts.serialize_ros1(cloud(S, stamp), "sensor_msgs/msg/PointCloud2"))
            n_scan += 1
        else:
            w.write(conns[con.topic], t, data)
print("wrote %s (%d scans thinned); tightest: sphere %+.2f m, body %.2f m" % (dst, n_scan, worst[0], worst[1]))
