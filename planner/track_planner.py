#!/usr/bin/env python3
"""Track planner node: drives the tracked QuadRover to /planning/drive_goal on the live scans.

  in   /planner/cloud         sensor_msgs/PointCloud2, planner frame (z up), world-registered scans
                              (FAST-LIO's dense /cloud_registered through the planner bridge)
       /planner/odom          nav_msgs/Odometry, planner frame (FAST-LIO's IMU: the odometry point,
                              ~footprint_offset 0.193 m ahead of the frame centre, see track_core.ODOM_AHEAD)
       /planning/drive_goal   geometry_msgs/PoseStamped, planner frame (from the mission executive)
  out  ~cmd_topic            quadrotor_msgs/PositionCommand (default /track_planner/pos_cmd): a carrot
                              ~lookahead ahead on the path, position + yaw only (zero velocity and
                              acceleration), streamed by the planner bridge in its drive mode (D4:
                              position + yaw, mask 2552). Not SUPER's /planning/pos_cmd: SUPER keeps
                              publishing toward its last goal after a landing, and the bridge takes
                              each mode's commands from its own planner's topic only
       /track_planner/path    nav_msgs/Path
       /track_planner/status  std_msgs/String JSON: state (IDLE, DRIVING, REACHED, NO_PATH), reason,
                              plan_ms (the last replan's wall time)

The map is the last ~window_s seconds of scans (track_core.LiveWindow), not an ever-growing union:
z drift between visits and moved objects (people, doors) made whole maps block the robot's own
path. Each scan is deduplicated as it arrives; each replan (~replan_hz, default 10) builds a height
grid over the start-goal box with a local ground model, corrects it with a decaying per-cell hit /
see-through memory (moved people cleared, walls seen earlier or sparsely held; track_core.LiveWindow)
and plans with the oriented frame (track_core). Unknown space is driven through at a
cost and replanned as it is seen. A failed replan keeps the last path while what is left of it
still checks in the new window; otherwise it clears the path and the bridge holds.
"""
import json
import threading
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

import track_core as tc

try:
    from quadrotor_msgs.msg import PositionCommand
except ImportError:  # SUPER's workspace not sourced
    PositionCommand = None

_NP = {7: np.float32, 8: np.float64}   # PointField FLOAT32, FLOAT64


def cloud_xyz(m):
    """PointCloud2 -> (N, 3) float32 without the per-point Python loop of point_cloud2.read_points"""
    f = {x.name: x for x in m.fields}
    dt = np.dtype({"names": ["x", "y", "z"], "formats": [_NP[f[k].datatype] for k in "xyz"],
                   "offsets": [f[k].offset for k in "xyz"], "itemsize": m.point_step})
    a = np.frombuffer(m.data, dtype=dt, count=m.width * m.height)
    p = np.stack([a["x"], a["y"], a["z"]], axis=1).astype(np.float32)
    return p[np.isfinite(p).all(1)]


class TrackPlanner:
    def __init__(self):
        g = rospy.get_param
        self.cfg = tc.live_config(res=g("~res", 0.1), margin=g("~margin", 0.05), z_lo=g("~z_lo", 0.15),
                                  z_hi=g("~z_hi", 0.60), min_pts=g("~min_pts", 2), lookahead=g("~lookahead", 0.5),
                                  pad=g("~pad", 3.0), footprint_offset=g("~footprint_offset", tc.ODOM_AHEAD),
                                  min_tall_only=g("~min_tall_only", 2), mem_min_range=g("~mem_min_range", 1.5))
        self.win = tc.LiveWindow(self.cfg, window_s=g("~window_s", 3.0), chunk_s=g("~chunk_s", 0.5),
                                 mem_tau=g("~mem_tau", 10.0), mem_hits=g("~mem_hits", 1.5),
                                 mem_clear_ratio=g("~mem_clear_ratio", 1.5))
        self.accept = g("~accept_radius", 0.3)
        self.lock = threading.Lock()
        self.odom = None
        self.goal = None
        self.path = None
        self.plan_ms = None
        self.state, self.reason = "IDLE", ""
        self.pub_cmd = rospy.Publisher(g("~cmd_topic", "/track_planner/pos_cmd"), PositionCommand,
                                       queue_size=10) if PositionCommand else None
        self.pub_path = rospy.Publisher("/track_planner/path", Path, queue_size=1, latch=True)
        self.pub_status = rospy.Publisher("/track_planner/status", String, queue_size=1, latch=True)
        rospy.Subscriber("/planner/cloud", PointCloud2, self.on_cloud, queue_size=2, buff_size=2**24)
        rospy.Subscriber("/planner/odom", Odometry, self.on_odom, queue_size=20)
        rospy.Subscriber("/planning/drive_goal", PoseStamped, self.on_goal, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / g("~replan_hz", 10.0)), self.replan)
        rospy.Timer(rospy.Duration(0.1), self.stream)

    def on_odom(self, m):
        p = m.pose.pose.position
        self.odom = np.array([p.x, p.y, p.z])

    def on_cloud(self, m):
        if self.odom is None:
            return
        pts = cloud_xyz(m)
        if len(pts):
            with self.lock:
                self.win.add_scan(m.header.stamp.to_sec(), pts, self.odom.copy())

    def on_goal(self, m):
        self.goal = np.array([m.pose.position.x, m.pose.position.y])
        self.path = None
        self.set_state("DRIVING", "new goal")
        self.replan(None)

    def set_state(self, s, why=""):
        self.state, self.reason = s, why
        self.pub_status.publish(String(json.dumps({"state": s, "reason": why, "plan_ms": self.plan_ms})))

    def replan(self, _):
        if self.goal is None or self.odom is None or self.state == "REACHED":
            return
        pos = self.odom.copy()
        if np.linalg.norm(pos[:2] - self.goal) < self.accept:
            self.path = None
            self.set_state("REACHED")
            return
        t0 = time.monotonic()
        with self.lock:
            out = self.win.plan(pos[:2], self.goal)
        self.plan_ms = round(1000 * (time.monotonic() - t0), 1)
        if not out["ok"]:
            # keep the last path while what is left of it still checks in the new window, so one
            # failed replan does not stop the robot; otherwise clear it and the bridge holds
            win = out.get("grid")
            if self.path is not None and win is not None and win.follow_free(tc.remaining(self.path, pos[:2])) is None:
                if self.reason != "kept the last path":
                    self.set_state("DRIVING", "kept the last path")
                return
            self.path = None
            self.set_state("NO_PATH", out["reason"])
            return
        self.path = out["path"]
        if self.state != "DRIVING" or self.reason != "path found":
            self.set_state("DRIVING", "path found")
        msg = Path()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        for xy in self.path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = xy[0], xy[1], pos[2]
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    def stream(self, _):
        # no command when idle, reached or without a path: the bridge then holds (zero velocity)
        path = self.path
        if self.pub_cmd is None or self.state != "DRIVING" or path is None or self.odom is None:
            return
        c, yaw = tc.carrot(path, self.odom[:2], self.cfg.lookahead)
        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "world"
        cmd.position.x, cmd.position.y, cmd.position.z = float(c[0]), float(c[1]), float(self.odom[2])
        cmd.yaw = float(yaw)
        cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        self.pub_cmd.publish(cmd)


if __name__ == "__main__":
    rospy.init_node("track_planner")
    TrackPlanner()
    rospy.spin()
