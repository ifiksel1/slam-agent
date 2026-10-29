#!/usr/bin/env python3
"""Stand-in for FAST-LIO in the L2 closed loop: SITL's true pose in, /Odometry and /cloud_registered out.

The pose comes from QuadRover SITL's SIM_STATE (MAVLink, its own TCP port), is placed in the map,
and the lidar (0.178 m ahead of the FC, level, pointing straight ahead) is raycast against the map
with the JT128's real ring pattern (~/sim/marsim/jt128_raycast.py, distance-field march, 0.4-30 m).

~lidar_up (m, default 0): the lidar above SITL's ground when the vehicle stands on it (SITL's ground is
the map's z = 0). The merged maps' floor is a smear ~0.1-0.3 m thick whose top lies around z = 0, so a
lidar at 0 sits inside it on the ground and sees the smear all round (an obstacle ring and no ground
for the track planner). The robot's lidar is ~0.08 m above a real floor; the drive-and-fly closed loop
uses 0.25 to keep it above the smear. Only the sensor moves: the raw frame's origin is the lidar's start
position as before, so poses are unchanged relative to the start.

Frames, as on the robot:
  map     the map PCD's frame (z up, ground at 0). The sim start is placed at (~start_x, ~start_y) on
          the ground, facing ~start_yaw.
  raw     FAST-LIO's world frame: the IMU/lidar frame at start (+x left, +y up, +z forward), origin at
          the lidar's start position. /Odometry (camera_init -> body) and /cloud_registered
          (camera_init) are in it, so the robot's own bridge turns them into vision for the FC.
  planner raw rotated z-up by the bridge's position_matrix P (the shared contract). The map -> planner
          transform is published once as the param /sim/map_to_planner (yaw, start position).

Timing: one pose per scan at ~odom_hz (FAST-LIO publishes /Odometry once per scan), stamped at
capture and published ~latency_s later, always in stamp order (the robot's bridge reads a stamp going
backwards as a SLAM restart and re-anchors); the cloud every (odom_hz / scan_hz) scans, published when
the raycast is done (never before latency_s).

Faults for the failure tests (~fault, from ~fault_start_s after the first pose):
  none, stop (nothing published), freeze (the last pose and cloud repeated), drift (pose and cloud
  slide by ~fault_rate m/s along raw +z, i.e. forward), height_bias (log 233: the height falls
  ~fault_rate m/s low, raw -y).
"""
import math
import threading
import time

import numpy as np
import rospy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from pymavlink import mavutil
from sensor_msgs.msg import PointCloud2, PointField

import sys
sys.path.insert(0, rospy.get_param("/render_node/raycast_dir", "/marsim"))
from jt128_raycast import JT128Raycaster  # noqa: E402
from raycast_surface import SurfaceRaycaster  # noqa: E402  (next to this file)

# raw (x left, y up, z forward) -> body FLU / ENU-like: the bridge's position_matrix
P = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], dtype=float)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def ned_euler_to_enu_flu(roll, pitch, yaw):
    """ArduPilot attitude (NED, FRD body, ZYX) -> rotation of the FLU body in ENU"""
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    r_ned_frd = np.array([[cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
                          [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
                          [-sp, sr * cp, cr * cp]])
    t = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1.0]])
    f = np.diag([1.0, -1.0, -1.0])
    return t @ r_ned_frd @ f


def quat(R):
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return Quaternion(x=x, y=y, z=z, w=w)


def load_pcd(path):
    raw = open(path, "rb").read()
    k = raw.index(b"DATA binary\n") + len(b"DATA binary\n")
    return np.frombuffer(raw[k:], dtype="<f4").reshape(-1, 4)[:, :3].astype(np.float64)


class RenderNode:
    def __init__(self):
        self.start = np.array([rospy.get_param("~start_x", 0.0), rospy.get_param("~start_y", 0.0), 0.0])
        self.start_yaw = math.radians(rospy.get_param("~start_yaw_deg", 0.0))
        self.lidar_fwd = rospy.get_param("~lidar_fwd", 0.178)
        self.lidar_up = rospy.get_param("~lidar_up", 0.0)
        self.odom_hz = rospy.get_param("~odom_hz", 20.0)
        self.scan_every = max(1, int(round(self.odom_hz / rospy.get_param("~scan_hz", 10.0))))
        self.latency = rospy.get_param("~latency_s", 0.07)
        self.fault = rospy.get_param("~fault", "none")
        self.fault_start = rospy.get_param("~fault_start_s", 1e9)
        self.fault_rate = rospy.get_param("~fault_rate", 0.05)
        table = np.load(rospy.get_param("~rings", "/marsim/jt128_rings.npy"))
        xyz = load_pcd(rospy.get_param("~map_pcd"))
        t = time.time()
        # ~surface_hits (default true): returns on the surfaces (raycast_surface.py); false: the stock
        # raycaster, whose returns land 0.05-0.1 m in front of them (the 0.90 m Gate L2 door: 0.75 m)
        self.caster = (SurfaceRaycaster if rospy.get_param("~surface_hits", True) else JT128Raycaster)(xyz, table)
        rospy.loginfo("render_node: %d map points, distance field %s in %.1f s, %d rays per scan",
                      len(xyz), self.caster.dims, time.time() - t, len(self.caster.rays))
        self.odom_pub = rospy.Publisher("/Odometry", Odometry, queue_size=20)
        self.cloud_pub = rospy.Publisher("/cloud_registered", PointCloud2, queue_size=2)
        self.mav = mavutil.mavlink_connection(rospy.get_param("~sitl_url", "tcp:127.0.0.1:5762"), source_system=250)
        self.mav.wait_heartbeat(timeout=60)
        # SIM_STATE (108) at 50 Hz
        self.mav.mav.command_long_send(self.mav.target_system, self.mav.target_component,
                                       mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 108, 20000, 0, 0, 0, 0, 0)
        self.state = None
        self.lock = threading.Lock()
        self.origin_ll = None
        self.theta = None
        self.raw0 = None       # (p_s0, R_ws0): the raw frame in the map
        self.t0 = None
        self.last = None       # last (stamp, pos_raw, R_raw, v_raw, cloud_raw) for the freeze fault
        self.busy = False
        self.out = []          # poses waiting for their latency, in stamp order
        self.stats = {"scans": 0, "raycast_ms": [], "lag_ms": []}
        threading.Thread(target=self.read_sitl, daemon=True).start()

    def read_sitl(self):
        while not rospy.is_shutdown():
            m = self.mav.recv_match(type="SIM_STATE", blocking=True, timeout=1)
            if m is None:
                continue
            # lat/lon are degE7 in float32 (0.36 m steps here, which the bridge reads as jumps); the
            # int32 lat_int/lon_int carry full precision
            lat, lon = m.lat_int * 1e-7, m.lon_int * 1e-7
            with self.lock:
                self.state = (time.time(), lat, lon, m.alt, m.roll, m.pitch, m.yaw, m.vn, m.ve, m.vd,
                              m.xgyro, m.ygyro, m.zgyro)

    def map_pose(self, s):
        """true FC position, body rotation and velocity in the map frame"""
        _, lat, lon, alt, roll, pitch, yaw, vn, ve, vd = s[:10]
        if self.origin_ll is None:
            self.origin_ll = (lat, lon, alt)
            # turn ENU so the vehicle's start heading becomes start_yaw in the map
            self.theta = self.start_yaw - (math.pi / 2 - yaw)
        k = math.radians(1.0) * 6378137.0
        enu = np.array([(lon - self.origin_ll[1]) * k * math.cos(math.radians(self.origin_ll[0])),
                        (lat - self.origin_ll[0]) * k, alt - self.origin_ll[2]])
        Rz = rot_z(self.theta)
        pos = self.start + Rz @ enu
        R_b = Rz @ ned_euler_to_enu_flu(roll, pitch, yaw)
        vel = Rz @ np.array([ve, vn, -vd])
        return pos, R_b, vel

    @staticmethod
    def map_rates(s, R_b):
        """the body's angular velocity in the map frame (SIM_STATE's body rates are FRD)"""
        gx, gy, gz = s[10:13]
        return R_b @ np.array([gx, -gy, -gz])

    def fault_offset(self, t_rel):
        if self.fault in ("drift", "height_bias") and t_rel > self.fault_start:
            d = self.fault_rate * (t_rel - self.fault_start)
            return np.array([0.0, 0.0, d]) if self.fault == "drift" else np.array([0.0, -d, 0.0])
        return np.zeros(3)

    def tick(self, _evt):
        with self.lock:
            s = self.state
        if s is None:
            return
        stamp = rospy.Time.now()
        pos, R_b, vel = self.map_pose(s)
        arm = R_b @ np.array([self.lidar_fwd, 0.0, self.lidar_up])
        p_s = pos + arm
        # the odometry point's own velocity, as FAST-LIO's twist (the IMU's): SIM_STATE's is the FC's.
        # The FC takes vision speed at VISO_POS and removes w x VISO_POS itself, so the FC's velocity
        # sent as the sensor's put -w x 0.178 m (0.19 m/s at a 1.05 rad/s pivot) into the EKF
        vel = vel + np.cross(self.map_rates(s, R_b), arm)
        R_ws = R_b @ P                         # the sensor (raw axes) in the map
        if self.raw0 is None:
            self.raw0 = (p_s.copy(), R_ws.copy())
            self.t0 = time.time()
            rospy.set_param("/sim/map_to_planner", {
                "note": "planner = Rz(-yaw) (map - start); raw = P^T planner",
                "yaw_deg": float(math.degrees(math.atan2(R_ws[1, 2], R_ws[0, 2]))),
                "start": [float(x) for x in p_s]})
        p0, R0 = self.raw0
        t_rel = time.time() - self.t0
        if self.fault == "stop" and t_rel > self.fault_start:
            return
        if self.fault == "freeze" and t_rel > self.fault_start and self.last is not None:
            _, pr, Rr, vr, cl = self.last
            self.queue_odom(stamp, pr, Rr, np.zeros(3))
            if self.stats["scans"] % self.scan_every == 0 and cl is not None:
                self.publish_cloud(stamp, cl)
            self.stats["scans"] += 1
            return
        off = self.fault_offset(t_rel)
        pr = R0.T @ (p_s - p0) + off
        Rr = R0.T @ R_ws
        vr = R0.T @ vel
        self.last = (stamp, pr, Rr, vr, self.last[4] if self.last else None)
        # every pose goes out in stamp order, latency_s after capture; a cloud follows when rendered
        self.queue_odom(stamp, pr, Rr, vr)
        if self.stats["scans"] % self.scan_every == 0 and not self.busy:
            self.busy = True
            threading.Thread(target=self.render, args=(stamp, p_s, R_ws, off), daemon=True).start()
        self.stats["scans"] += 1

    def queue_odom(self, stamp, pr, Rr, vr):
        with self.lock:
            self.out.append((stamp, pr, Rr, vr))

    def flush(self, _evt):
        """publish queued poses whose latency has passed, oldest first"""
        now = rospy.Time.now()
        while True:
            with self.lock:
                if not self.out or (now - self.out[0][0]).to_sec() < self.latency:
                    return
                stamp, pr, Rr, vr = self.out.pop(0)
            self.publish_odom(stamp, pr, Rr, vr)

    def render(self, stamp, p_s, R_ws, off):
        t = time.time()
        hits = self.caster.scan(p_s, R_ws)
        self.stats["raycast_ms"].append(1000 * (time.time() - t))
        p0, R0 = self.raw0
        cloud = (hits - p0) @ R0 + off        # R0.T @ (h - p0) for every hit
        if self.last is not None:
            self.last = self.last[:4] + (cloud,)
        wait = self.latency - (rospy.Time.now() - stamp).to_sec()
        if wait > 0:
            rospy.sleep(wait)
        self.publish_cloud(stamp, cloud)
        self.busy = False

    def publish_odom(self, stamp, pr, Rr, vr):
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = "camera_init"
        o.child_frame_id = "body"
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = map(float, pr)
        o.pose.pose.orientation = quat(Rr)
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z = map(float, vr)
        self.odom_pub.publish(o)
        self.stats["lag_ms"].append(1000 * (rospy.Time.now() - stamp).to_sec())

    def publish_cloud(self, stamp, cloud):
        if cloud is None or not len(cloud):
            return
        c = np.c_[cloud, np.full(len(cloud), 50.0)].astype("<f4")
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_init"
        msg.height, msg.width = 1, len(c)
        msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate(("x", "y", "z", "intensity"))]
        msg.is_bigendian, msg.point_step, msg.row_step, msg.is_dense = False, 16, 16 * len(c), True
        msg.data = c.tobytes()
        self.cloud_pub.publish(msg)

    def report(self, _evt):
        r, l = self.stats["raycast_ms"][-50:], self.stats["lag_ms"][-100:]
        if r and l:
            rospy.loginfo("render_node: raycast %.0f ms (median of last %d), publish lag %.0f ms median, %.0f ms max",
                          float(np.median(r)), len(r), float(np.median(l)), float(np.max(l)))


if __name__ == "__main__":
    rospy.init_node("render_node")
    n = RenderNode()
    rospy.Timer(rospy.Duration(1.0 / n.odom_hz), n.tick)
    rospy.Timer(rospy.Duration(0.005), n.flush)
    rospy.Timer(rospy.Duration(10.0), n.report)
    rospy.spin()
