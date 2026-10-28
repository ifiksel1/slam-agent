#!/usr/bin/env python3
"""Landing-spot check node (D3): answers the mission executive's "is this spot clear to land?"

  in:  /planner/cloud          sensor_msgs/PointCloud2, planner frame (z up), from the planner bridge;
                               each scan kept on a 5 cm voxel grid for the last ~window_s seconds,
                               with the sensor position at its arrival (the latest /planner/odom)
       /planner/odom           nav_msgs/Odometry, planner frame: the lidar (returns nearer than
                               ~near_range read high on recorded data and are dropped) and the
                               heading: the disc is centred on the frame, ~footprint_offset
                               (track_core.ODOM_AHEAD, 0.193 m) behind the spot; no odom yet ->
                               not clear, "no heading"
       /planner/landing_spot   geometry_msgs/PointStamped, planner frame: the spot the executive
                               wants to land on (z = the height it descends from)
  out: /planner/landing_clear  std_msgs/Bool, every tick_s while a spot request is fresh (< spot_timeout_s)
       /planner/landing_status std_msgs/String, JSON: the reason and the numbers behind it
                               ("spot", and "centre": the disc's)
       /planner/landing_markers visualization_msgs/MarkerArray, in /planner/cloud's frame (world),
                               on every evaluation; DELETEALL when the spot request goes stale
                               (> spot_timeout_s) or the robot is more than ~marker_max_dist away.
                               Namespaces (one marker each, id 0):
                                 landing_disc        CYLINDER at the floor, alpha 0.35: green clear,
                                                     red occupied / not flat, amber uncertain /
                                                     not observed / no heading
                                 landing_footprint   LINE_STRIP, the frame's rectangle (white)
                                 landing_occupied    CUBE_LIST, 10 cm cells, top at their height
                                                     above the floor (red)
                                 landing_unobserved  CUBE_LIST, flat grey cells at the floor
                                 landing_label       TEXT_VIEW_FACING, decision + reason + layer

Rules in landing_core.py (scan by scan: each scan's floor is fitted to the track planner's local
ground model, and the latest scans decide whether a cell is occupied). Anything uncertain is not clear.
"""
import json
import math
import threading
import time

import numpy as np
import rospy
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

import track_core as tc
from landing_core import FOOTPRINT, LandingCheck

GREEN, RED, AMBER = (0.1, 0.8, 0.2), (0.9, 0.1, 0.1), (1.0, 0.65, 0.0)


def decision(clear, reason):
    """-> (label word, rgb)"""
    if clear:
        return "CLEAR", GREEN
    if reason.startswith("occupied"):
        return "OCCUPIED", RED
    if reason.startswith("not flat"):
        return "NOT FLAT", RED
    if reason.startswith("not observed"):
        return "NOT OBSERVED", AMBER
    if reason.startswith("no heading"):
        return "NO HEADING", AMBER
    return "UNCERTAIN", AMBER


def label_text(clear, st):
    word, _ = decision(clear, st.get("reason", ""))
    layer = st.get("floor_layer_m")
    if word == "CLEAR":
        return "CLEAR layer %.2f m" % layer
    if word == "OCCUPIED":
        return "OCCUPIED %d cells up to %.2f m" % (st.get("obstacle_cells", 0), st.get("max_obstacle_h", 0.0))
    if word == "NOT OBSERVED" and "observed" in st:
        return "NOT OBSERVED %.0f%%" % (100 * st["observed"])
    if word == "NOT FLAT":
        return "NOT FLAT slope %.1f deg, resid %.3f m" % (st.get("slope_deg", 0.0), st.get("residual_m", 0.0))
    if word == "UNCERTAIN" and layer is not None:
        return "UNCERTAIN layer %.2f m" % layer
    return "%s %s" % (word, st.get("reason", ""))


def build_markers(clear, st, viz, frame, stamp, fallback_z):
    """the landing check's markers (see the module docstring) from LandingCheck.viz"""
    _, rgb = decision(clear, st.get("reason", ""))
    viz = viz or {}
    cx, cy = viz.get("centre") or st["centre"]
    fz = viz.get("floor_z")
    if fz is None:
        fz = fallback_z
    a, b = viz.get("plane") or (0.0, 0.0)
    cell = viz.get("cell", 0.1)

    def mk(ns, typ):
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = stamp
        m.ns, m.id, m.type, m.action = ns, 0, typ, Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    out = MarkerArray()
    d = mk("landing_disc", Marker.CYLINDER)
    r = viz.get("radius", st["radius_m"])
    d.pose.position.x, d.pose.position.y, d.pose.position.z = cx, cy, fz + 0.005
    d.scale.x = d.scale.y = 2 * r
    d.scale.z = 0.01
    d.color = ColorRGBA(rgb[0], rgb[1], rgb[2], 0.35)
    out.markers.append(d)

    yaw = viz.get("yaw")
    f = mk("landing_footprint", Marker.LINE_STRIP)
    f.scale.x = 0.02
    f.color = ColorRGBA(1.0, 1.0, 1.0, 0.9)
    if yaw is not None:
        c, s = math.cos(yaw), math.sin(yaw)
        hl, hw = FOOTPRINT[0] / 2, FOOTPRINT[1] / 2
        for u, v in ((hl, hw), (-hl, hw), (-hl, -hw), (hl, -hw), (hl, hw)):
            f.points.append(Point(cx + c * u - s * v, cy + s * u + c * v, fz + 0.02))
        f.points.append(Point(cx, cy, fz + 0.02))   # a line to the front edge's middle: the heading
        f.points.append(Point(cx + c * hl, cy + s * hl, fz + 0.02))
    else:
        f.action = Marker.DELETE
    out.markers.append(f)

    o = mk("landing_occupied", Marker.CUBE_LIST)
    o.scale.x = o.scale.y = cell
    o.scale.z = 0.05
    o.color = ColorRGBA(RED[0], RED[1], RED[2], 0.9)
    for x, y, h in viz.get("occupied", ()):
        o.points.append(Point(x, y, fz + a * (x - cx) + b * (y - cy) + h - 0.025))
    if not o.points:
        o.action = Marker.DELETE
    out.markers.append(o)

    u = mk("landing_unobserved", Marker.CUBE_LIST)
    u.scale.x = u.scale.y = cell * 0.9
    u.scale.z = 0.01
    u.color = ColorRGBA(0.5, 0.5, 0.5, 0.6)
    for x, y in viz.get("unobserved", ()):
        u.points.append(Point(x, y, fz + a * (x - cx) + b * (y - cy) + 0.01))
    if not u.points:
        u.action = Marker.DELETE
    out.markers.append(u)

    t = mk("landing_label", Marker.TEXT_VIEW_FACING)
    t.pose.position.x, t.pose.position.y, t.pose.position.z = cx, cy, fz + 0.6
    t.scale.z = 0.15
    t.color = ColorRGBA(rgb[0], rgb[1], rgb[2], 1.0)
    t.text = label_text(clear, st)
    out.markers.append(t)
    return out


def cloud_xyz(m):
    a = np.frombuffer(m.data, dtype=np.uint8).reshape(-1, m.point_step)
    offs = {f.name: f.offset for f in m.fields}
    cols = [a[:, offs[k]:offs[k] + 4].copy().view(np.float32).ravel() for k in ("x", "y", "z")]
    P = np.stack(cols, 1).astype(float)
    return P[np.isfinite(P).all(1)]


class LandingCheckNode:
    def __init__(self):
        g = rospy.get_param
        self.check = LandingCheck(margin=g("~margin", 0.2), observed_min=g("~observed_min", 0.7),
                                  slope_max_deg=g("~slope_max_deg", 5.0), residual_max=g("~residual_max", 0.03),
                                  obstacle_min_h=g("~obstacle_min_h", 0.05), floor_layer_max=g("~floor_layer_max", 0.20),
                                  near_range=g("~near_range", 1.2),
                                  footprint_offset=g("~footprint_offset", tc.ODOM_AHEAD))
        self.window_s = g("~window_s", 10.0)
        self.voxel = g("~voxel", 0.05)
        self.spot_timeout_s = g("~spot_timeout_s", 2.0)
        self.odom_timeout_s = g("~odom_timeout_s", 0.5)
        self.marker_max_dist = g("~marker_max_dist", 5.0)   # the robot left: markers deleted
        self.frame = g("~frame_id", "world")   # replaced by /planner/cloud's frame_id when it arrives
        self.yaw = None
        self.markers_up = False
        self.scans = []        # (time, voxelised points, sensor xyz or None)
        self.spot = None       # (time, x, y, z)
        self.odom = None       # (time, xyz)
        self.busy = threading.Lock()
        self.pub_clear = rospy.Publisher("/planner/landing_clear", Bool, queue_size=10)
        self.pub_status = rospy.Publisher("/planner/landing_status", String, queue_size=10)
        self.pub_markers = rospy.Publisher("/planner/landing_markers", MarkerArray, queue_size=2)
        rospy.Subscriber("/planner/odom", Odometry, self.on_odom, queue_size=20)
        rospy.Subscriber("/planner/cloud", PointCloud2, self.on_cloud, queue_size=5)
        rospy.Subscriber("/planner/landing_spot", PointStamped, self.on_spot, queue_size=5)
        rospy.Timer(rospy.Duration(g("~tick_s", 0.25)), self.tick)

    def on_odom(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom = (time.monotonic(), np.array([p.x, p.y, p.z]))

    def on_cloud(self, m):
        if m.header.frame_id:
            self.frame = m.header.frame_id
        P = cloud_xyz(m)
        k = np.unique(np.floor(P / self.voxel).astype(np.int64), axis=0, return_index=True)[1]
        now = time.monotonic()
        od = self.odom
        sensor = od[1] if od is not None and now - od[0] <= self.odom_timeout_s else None
        self.scans = [s for s in self.scans if now - s[0] <= self.window_s] + [(now, P[k], sensor)]

    def on_spot(self, m):
        new = self.spot is None or time.monotonic() - self.spot[0] > self.spot_timeout_s
        self.spot = (time.monotonic(), m.point.x, m.point.y, m.point.z)
        if new:
            self.tick(None)     # a new request: answer at once

    def delete_markers(self):
        if self.markers_up:
            m = Marker(action=Marker.DELETEALL)
            m.header.frame_id = self.frame
            m.header.stamp = rospy.Time.now()
            self.pub_markers.publish(MarkerArray(markers=[m]))
            self.markers_up = False

    def tick(self, _):
        if self.spot is None or time.monotonic() - self.spot[0] > self.spot_timeout_s:
            self.delete_markers()
            return
        if not self.busy.acquire(blocking=False):
            return
        try:
            self.evaluate()
        finally:
            self.busy.release()

    def evaluate(self):
        _, x, y, z = self.spot
        t0 = time.monotonic()
        yaw, od = self.yaw, self.odom
        if yaw is None:
            clear, st = False, {"spot": [round(x, 2), round(y, 2)], "centre": [round(x, 2), round(y, 2)],
                                "radius_m": round(self.check.radius, 3), "reason": "no heading: no /planner/odom yet"}
            self.check.viz = {}
        else:
            scans = [(s[1], s[2]) if s[2] is not None else s[1] for s in self.scans] or [np.zeros((0, 3))]
            clear, st = self.check.evaluate(None, (x, y), descend_from=z if z > 0 else None, scans=scans, yaw=yaw)
            st["scans"] = len(self.scans)
        st["eval_ms"] = round(1000 * (time.monotonic() - t0), 1)
        self.pub_clear.publish(Bool(data=bool(clear)))
        self.pub_status.publish(String(json.dumps(st)))
        if od is not None and np.hypot(od[1][0] - x, od[1][1] - y) > self.marker_max_dist:
            self.delete_markers()
            return
        try:
            fallback = od[1][2] - 1.0 if od is not None else z   # no floor known: ~1 m under the robot
            self.pub_markers.publish(build_markers(clear, st, self.check.viz, self.frame, rospy.Time.now(), fallback))
            self.markers_up = True
        except Exception as e:   # the markers never stop the answer
            rospy.logwarn_throttle(10.0, "landing check: markers failed: %s" % e)


if __name__ == "__main__":
    rospy.init_node("landing_check")
    LandingCheckNode()
    rospy.spin()
