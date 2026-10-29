#!/usr/bin/env python3
"""render_node.py with live faults, for the L2 failure tests (planner/sim/faults/).

Runs in place of render_node.py when sitl_harness.sh is started with
RENDER_SCRIPT=/planner_sim/faults/render_fault.py (the faults' cases.sh does that). Without a
command it behaves exactly like render_node.py; it adds:

  /sim/truth        nav_msgs/Odometry, the FC's true pose and velocity in the map frame (frame
                    "sim_map"), every tick (20 Hz), from SITL's SIM_STATE. Independent of every
                    fault below and of MAVROS, so the fault runner measures distances on it.
  /sim_fault/state  std_msgs/String JSON, latched: the faults in force.
  /sim_fault/cmd    std_msgs/String JSON, one or more of:
      {"stale": "stop"}      nothing published (/Odometry, /cloud_registered): FAST-LIO dead or hung
      {"stale": "freeze"}    the last pose and cloud repeated with fresh stamps (FAST-LIO stuck)
      {"stale": "none"}      back to normal
      {"reset": {"gap_s": 2.0}}  FAST-LIO restart: nothing for gap_s, then a new raw frame with its
                             origin at the lidar's current position, level, at the current heading
                             (a fresh laserMapping starts at its own origin). gap_s 0: an instant pose
                             jump to the new origin (relocalisation)
      {"box": {"id": "b1", "center": [x, y, z], "size": [sx, sy, sz]}}  an axis-aligned box in the
                             map frame that the lidar sees from now on (a person or crate stepping in)
      {"box_clear": "b1"} or {"box_clear": "all"}

The boxes are raycast analytically (slab test per JT128 ray) on top of the map's distance-field
march, so adding one is instant. They are only seen: SITL has no obstacles, nothing collides.
"""
import json
import math
import sys
import threading
import time

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

sys.path.insert(0, "/planner_sim")
import render_node as rn  # noqa: E402
import jt128_raycast as jr  # noqa: E402  (on the path through render_node)


class FaultRender(rn.RenderNode):
    def __init__(self):
        super().__init__()
        self.flock = threading.Lock()
        self.stale = "none"
        self.boxes = {}             # id -> (lo, hi) map frame
        self.gap_until = 0.0
        self.reset_pending = False
        self.resets = 0
        self.truth_pub = rospy.Publisher("/sim/truth", Odometry, queue_size=50)
        self.fstate_pub = rospy.Publisher("/sim_fault/state", String, queue_size=5, latch=True)
        rospy.Subscriber("/sim_fault/cmd", String, self.on_cmd, queue_size=20)
        self.pub_fstate("start")

    def pub_fstate(self, why):
        with self.flock:
            s = {"why": why, "stale": self.stale, "resets": self.resets,
                 "gap": time.time() < self.gap_until or self.reset_pending,
                 "boxes": {k: {"lo": [round(float(x), 3) for x in lo], "hi": [round(float(x), 3) for x in hi]}
                           for k, (lo, hi) in self.boxes.items()}}
        self.fstate_pub.publish(String(data=json.dumps(s)))
        rospy.loginfo("render_fault: %s", json.dumps(s))

    def on_cmd(self, m):
        try:
            c = json.loads(m.data)
        except ValueError:
            rospy.logerr("render_fault: bad command %r", m.data)
            return
        with self.flock:
            if "stale" in c:
                self.stale = c["stale"] if c["stale"] in ("stop", "freeze", "none") else "none"
            if "reset" in c:
                self.gap_until = time.time() + float((c["reset"] or {}).get("gap_s", 2.0))
                self.reset_pending = True
            if "box" in c:
                b = c["box"]
                ctr, size = np.array(b["center"], float), np.abs(np.array(b["size"], float))
                self.boxes[str(b.get("id", "b%d" % len(self.boxes)))] = (ctr - size / 2, ctr + size / 2)
            if "box_clear" in c:
                if c["box_clear"] == "all":
                    self.boxes = {}
                else:
                    self.boxes.pop(str(c["box_clear"]), None)
        self.pub_fstate("cmd %s" % m.data)

    def publish_truth(self, stamp, pos, R_b, vel):
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = "sim_map"
        o.child_frame_id = "fc_truth"
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = map(float, pos)
        o.pose.pose.orientation = rn.quat(R_b)
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z = map(float, vel)
        self.truth_pub.publish(o)

    def tick(self, _evt):
        with self.lock:
            s = self.state
        if s is None:
            return
        stamp = rospy.Time.now()
        pos, R_b, vel = self.map_pose(s)
        self.publish_truth(stamp, pos, R_b, vel)
        p_s = pos + R_b @ np.array([self.lidar_fwd, 0.0, self.lidar_up])
        R_ws = R_b @ rn.P
        if self.raw0 is None:
            self.raw0 = (p_s.copy(), R_ws.copy())
            self.t0 = time.time()
            rospy.set_param("/sim/map_to_planner", {
                "note": "planner = Rz(-yaw) (map - start); raw = P^T planner",
                "yaw_deg": float(math.degrees(math.atan2(R_ws[1, 2], R_ws[0, 2]))),
                "start": [float(x) for x in p_s]})
        with self.flock:
            stale, gap, reset = self.stale, time.time() < self.gap_until, self.reset_pending
        if gap:
            return                              # laserMapping restarting: nothing out
        if reset:
            # a fresh laserMapping: origin at the lidar now, level, at the current heading
            yaw = math.atan2(R_b[1, 0], R_b[0, 0])
            self.raw0 = (p_s.copy(), rn.rot_z(yaw) @ rn.P)
            self.last = None
            with self.flock:
                self.reset_pending = False
                self.resets += 1
            rospy.set_param("/sim/map_to_planner_reset%d" % self.resets, {
                "yaw_deg": float(math.degrees(yaw)), "start": [float(x) for x in p_s], "t": time.time()})
            self.pub_fstate("reset applied")
        p0, R0 = self.raw0
        if stale == "stop":
            return
        if stale == "freeze" and self.last is not None:
            _, pr, Rr, _vr, cl = self.last
            self.queue_odom(stamp, pr, Rr, np.zeros(3))
            if self.stats["scans"] % self.scan_every == 0 and cl is not None:
                self.publish_cloud(stamp, cl)
            self.stats["scans"] += 1
            return
        pr = R0.T @ (p_s - p0)
        Rr = R0.T @ R_ws
        vr = R0.T @ vel
        self.last = (stamp, pr, Rr, vr, self.last[4] if self.last else None)
        self.queue_odom(stamp, pr, Rr, vr)
        if self.stats["scans"] % self.scan_every == 0 and not self.busy:
            self.busy = True
            threading.Thread(target=self.render, args=(stamp, p_s, R_ws, np.zeros(3)), daemon=True).start()
        self.stats["scans"] += 1

    def scan(self, origin, R):
        """the map march (jt128_raycast.JT128Raycaster.scan) plus the boxes: the nearer hit per ray"""
        c = self.caster
        with self.flock:
            boxes = list(self.boxes.values())
        if not boxes:
            return c.scan(origin, R)
        d = c.rays @ R.T
        t = np.full(len(d), jr.MIN_R)
        alive = np.ones(len(d), bool)
        hit = np.zeros(len(d), bool)
        for _ in range(64):
            a = np.where(alive)[0]
            if not len(a):
                break
            p = origin + d[a] * t[a, None]
            g = np.floor((p - c.lo) / jr.VOX).astype(int)
            inside = np.all((g >= 0) & (g < c.dims), axis=1)
            dist = np.full(len(a), np.inf, np.float32)
            gi = g[inside]
            dist[inside] = c.df[gi[:, 0], gi[:, 1], gi[:, 2]]
            h = dist <= jr.VOX
            hit[a[h]] = True
            step = np.maximum(dist - jr.VOX * 0.5, jr.VOX * 0.5)
            t[a] += np.where(h, 0, step)
            alive[a[h | ~inside]] = False
            alive &= t <= jr.MAX_R
        t_map = np.where(hit, t, np.inf)
        dd = np.where(np.abs(d) < 1e-9, 1e-9, d)
        t_box = np.full(len(d), np.inf)
        for lo, hi in boxes:
            t1, t2 = (lo - origin) / dd, (hi - origin) / dd
            tmin, tmax = np.minimum(t1, t2).max(1), np.maximum(t1, t2).min(1)
            ok = (tmax >= tmin) & (tmin >= jr.MIN_R) & (tmin <= jr.MAX_R)
            t_box = np.where(ok & (tmin < t_box), tmin, t_box)
        tt = np.minimum(t_map, t_box)
        keep = np.isfinite(tt)
        return origin + d[keep] * tt[keep, None]

    def render(self, stamp, p_s, R_ws, off):
        t = time.time()
        hits = self.scan(p_s, R_ws)
        self.stats["raycast_ms"].append(1000 * (time.time() - t))
        p0, R0 = self.raw0
        cloud = (hits - p0) @ R0 + off
        if self.last is not None:
            self.last = self.last[:4] + (cloud,)
        wait = self.latency - (rospy.Time.now() - stamp).to_sec()
        if wait > 0:
            rospy.sleep(wait)
        with self.flock:
            dropped = self.stale == "stop" or time.time() < self.gap_until or self.reset_pending
        if not dropped:
            self.publish_cloud(stamp, cloud)
        self.busy = False


if __name__ == "__main__":
    rospy.init_node("render_node")
    n = FaultRender()
    rospy.Timer(rospy.Duration(1.0 / n.odom_hz), n.tick)
    rospy.Timer(rospy.Duration(0.005), n.flush)
    rospy.Timer(rospy.Duration(10.0), n.report)
    rospy.spin()
