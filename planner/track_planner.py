#!/usr/bin/env python3
"""Track planner node: drives the tracked QuadRover to /planning/drive_goal on the live scans.

  in   /planner/cloud         sensor_msgs/PointCloud2, planner frame (z up), world-registered scans
                              (FAST-LIO's dense /cloud_registered through the planner bridge)
       /planner/odom          nav_msgs/Odometry, planner frame (FAST-LIO's IMU: the odometry point,
                              ~footprint_offset 0.193 m ahead of the frame centre, see track_core.ODOM_AHEAD)
       /planning/drive_goal   geometry_msgs/PoseStamped, planner frame (from the mission executive)
  out  ~cmd_topic            quadrotor_msgs/PositionCommand (default /track_planner/pos_cmd), streamed
                              by the planner bridge in its drive mode. Driving: the carrot ~lookahead
                              ahead on the path run out along its bearing (track_core.run_out: up to
                              ~runout_s x ~drive_speed from the FC, frame-checked, none at stop points,
                              while pivoting or turning), position + yaw, zero velocity (D4: position
                              + yaw, mask 2552). Reversing out of a dead end (track_core.plan_reverse):
                              the FC's end point, a velocity of ~reverse_speed against the yaw, the
                              heading held (the bridge's reverse: bridge_core.reverse_speed). Not
                              SUPER's /planning/pos_cmd: SUPER keeps publishing toward its last goal
                              after a landing, and the bridge takes each mode's commands from its own
                              planner's topic only
       /track_planner/path    nav_msgs/Path (each pose's orientation: the heading driven or, at a
                              pivot's arrival and leaving points, the heading before and after it)
       /track_planner/status  std_msgs/String JSON: state (IDLE, DRIVING, REACHED, NO_PATH), reason,
                              plan_ms (the last replan's wall time), phase (drive, approach: the
                              carrot holds at the next pivot, pivot: turning in place there,
                              reverse: backing out of a dead end),
                              pivots: [{x, y (the frame centre), turn_deg, yaw_in_deg, yaw_out_deg,
                              start}] of the current path

Pivots (track_core.pivot_pass, follow_plan). A turn over ~pivot_angle_deg (Rover's WP_PIVOT_ANGLE)
is a pivot in place about the frame centre, planned as its own step where the pivot circle is
clear; the carrot holds ~pivot_lead past the centre (half the robot's WP_RADIUS 0.3: Rover stops
within WP_RADIUS of a held target) until the robot has arrived, then sits on the leaving ray until
the heading is within ~pivot_done_deg, so Rover pivots there rather than arcing. The robot's
heading (/planner/odom orientation, the nose in the planner frame) is the start heading of each
replan: a turn on the spot is a start pivot and is checked where the robot stands. The pivot
circle's margin is ~pivot_margin (its own, owner 2026-10-28; the frame's ~margin stays for driving).

Dead ends (owner 2026-10-28; track_core.plan_reverse). With the goal behind (beyond pivot_angle) and
no room for the start pivot where the robot stands, it backs straight up the way it came, at most
~reverse_max (2 m), over cells observed free (floor seen in the live window, or seen through in the
evidence memory: never unknown space, the JT128 dome sees nothing behind) with the oriented frame
clear, to the nearest spot the start pivot fits; then pivots and drives on.

Speed (track_core.run_out). Rover Guided's fast-update path asks ~0.5 m/s per metre to its target
and steers like pure pursuit on it: the carrot alone held it to ~0.3 m/s; the run-out gives it
~runout_s (2.1) x ~drive_speed (the FC's WP_SPEED) of target on straights, and none at stop points
or while it turns. ~drive_lookahead on the bridge must cover it.

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
import math
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
                                  min_tall_only=g("~min_tall_only", 2), mem_min_range=g("~mem_min_range", 1.5),
                                  pivot_angle=g("~pivot_angle_deg", 60.0), pivot_done=g("~pivot_done_deg", 10.0),
                                  pivot_tol=g("~pivot_tol", 0.2), pivot_search=g("~pivot_search", 0.6),
                                  pivot_margin=g("~pivot_margin", tc.Config().pivot_margin),
                                  reverse_max=g("~reverse_max", 2.0))
        self.pivot_lead = g("~pivot_lead", 0.15)
        # the run-out (track_core.run_out): the target the FC sees, up to runout_s x drive_speed ahead
        # along the carrot's bearing. Rover Guided's fast-update path asks ~0.5 m/s per metre to its
        # target (SITL d10: 0.58 m/s at 1.15 m), so WP_SPEED needs ~2 s of it. drive_speed: the FC's
        # WP_SPEED (0.5 on the robot: 1.05 m). runout_s 0: the bare carrot
        self.drive_speed = g("~drive_speed", 0.5)
        self.runout = g("~runout_s", 2.1) * self.drive_speed
        self.fc_behind = g("~fc_behind", 0.178)     # the FC (Rover's position) behind the odometry point, m
        # dead ends (track_core.plan_reverse): back up at this speed, straight, holding the heading
        self.reverse_speed = g("~reverse_speed", 0.25)
        self.win = tc.LiveWindow(self.cfg, window_s=g("~window_s", 3.0), chunk_s=g("~chunk_s", 0.5),
                                 mem_tau=g("~mem_tau", 10.0), mem_hits=g("~mem_hits", 1.5),
                                 mem_clear_ratio=g("~mem_clear_ratio", 1.5))
        self.accept = g("~accept_radius", 0.3)
        self.lock = threading.Lock()
        self.odom = None
        self.yaw = None
        self.goal = None
        self.path = None
        self.route = None           # the path before the pivot steps (to re-check what is left of it)
        self.pivots = []
        self.phase = "drive"
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
        q = m.pose.pose.orientation
        self.odom = np.array([p.x, p.y, p.z])
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def on_cloud(self, m):
        if self.odom is None:
            return
        pts = cloud_xyz(m)
        if len(pts):
            with self.lock:
                self.win.add_scan(m.header.stamp.to_sec(), pts, self.odom.copy())

    def on_goal(self, m):
        self.goal = np.array([m.pose.position.x, m.pose.position.y])
        self.set_path(None)
        self.phase = "drive"              # nothing under way from an earlier leg
        self.set_state("DRIVING", "new goal")
        self.replan(None)

    def set_path(self, path, pivots=(), route=None, grid=None, reverse=None):
        # one assignment: stream() runs on another timer thread. grid: the window the path was
        # checked in, for the run-out check; reverse: a dead end's reverse before the path
        # (track_core.plan_reverse: the path starts where it ends)
        self.path, self.pivots, self.route = (path, list(pivots), route) if path is not None else (None, [], None)
        self.cur = (self.path, self.pivots, grid if path is not None else None, reverse if path is not None else None)

    def status(self):
        pv = [{"x": round(p["centre"][0], 3), "y": round(p["centre"][1], 3), "turn_deg": round(math.degrees(p["turn"]), 1),
               "yaw_in_deg": round(math.degrees(p["yaw_in"]), 1), "yaw_out_deg": round(math.degrees(p["yaw_out"]), 1),
               "start": p["start"]} for p in self.pivots]
        return {"state": self.state, "reason": self.reason, "plan_ms": self.plan_ms, "phase": self.phase, "pivots": pv}

    def set_state(self, s, why=""):
        self.state, self.reason = s, why
        self.pub_status.publish(String(json.dumps(self.status())))

    def replan(self, _):
        if self.goal is None or self.odom is None or self.state == "REACHED":
            return
        pos, yaw = self.odom.copy(), self.yaw
        if np.linalg.norm(pos[:2] - self.goal) < self.accept:
            self.set_path(None)
            self.set_state("REACHED")
            return
        pivoting = self.phase == "pivot"
        before = [(round(p["centre"][0], 2), round(p["centre"][1], 2)) for p in self.pivots]
        t0 = time.monotonic()
        with self.lock:
            out = self.win.plan(pos[:2], self.goal, start_yaw=yaw, start_pivoting=pivoting)
        self.plan_ms = round(1000 * (time.monotonic() - t0), 1)
        if not out["ok"]:
            # keep the last path while what is left of it still checks in the new window (the frame
            # along it and its pivots, from where the robot is now), so one failed replan does not
            # stop the robot; otherwise clear it and the bridge holds
            win = out.get("grid")
            if self.route is not None and win is not None:
                kept = tc.check_route(win, tc.remaining(self.route, pos[:2]), yaw, pivoting)
                if kept["ok"]:
                    self.set_path(kept["path"], kept["pivots"], tc.remaining(self.route, pos[:2]), win)
                    if self.reason != "kept the last path":
                        self.set_state("DRIVING", "kept the last path")
                    return
            self.set_path(None)
            # no command streams now (the bridge holds; Rover finishes turning toward its last target and
            # stops): a 'pivot' phase left over would make every replan from here a pivot under way
            # (any turn over pivot_done needs the circle where it stands: d7 sat 21 s in NO_PATH)
            self.phase = "drive"
            why = out["reason"] + ("; reverse: " + out["reverse_reason"] if out.get("reverse_reason") else "")
            self.set_state("NO_PATH", why)
            return
        if pivoting and self.route is not None:
            # a pivot under way: finish it on the route it began on while that still checks from
            # here (a new route mid-turn can send the robot back the other way)
            rest = tc.remaining(self.route, pos[:2])
            kept = tc.check_route(out["grid"], rest, yaw, True)
            if kept["ok"]:
                out = {"path": kept["path"], "pivots": kept["pivots"], "route": rest, "grid": out["grid"]}
        self.set_path(out["path"], out["pivots"], out["route"], out.get("grid"), out.get("reverse"))
        after =[(round(p["centre"][0], 2), round(p["centre"][1], 2)) for p in self.pivots]
        if self.state != "DRIVING" or self.reason != "path found" or after != before:
            self.set_state("DRIVING", "path found")
        msg = Path()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        heads = self.headings(self.path, self.pivots)
        for xy, h in zip(self.path, heads):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = xy[0], xy[1], pos[2]
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(h / 2.0), math.cos(h / 2.0)
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    @staticmethod
    def headings(path, pivots):
        """per path point: the heading of the segment leaving it (the last: arriving); a pivot's
        arrival and leaving points: its headings before and after"""
        P = np.asarray(path, float)
        h = [math.atan2(*(P[k + 1] - P[k])[::-1]) for k in range(len(P) - 1)]
        h = h + [h[-1] if h else 0.0]
        for p in pivots:
            h[p["i"]], h[p["i"] + 1] = p["yaw_in"], p["yaw_out"]
        return h

    def stream(self, _):
        # no command when idle, reached or without a path: the bridge then holds (zero velocity)
        path, pivots, grid, rev = getattr(self, "cur", (None, [], None, None))
        if self.pub_cmd is None or self.state != "DRIVING" or path is None or self.odom is None:
            return
        vel = (0.0, 0.0)
        if rev is not None and tc.reverse_left(rev, self.odom[:2]) > 0.03:
            # a dead end: back straight up to where the pivot fits: the FC's end point, a velocity
            # against the heading (the bridge's reverse: bridge_core.reverse_speed), the heading held
            yaw, phase = rev["yaw"], "reverse"
            u = np.array([math.cos(yaw), math.sin(yaw)])
            c = np.asarray(rev["to"], float) - self.fc_behind * u
            vel = tuple(-self.reverse_speed * u)
        else:
            c, yaw, phase, j = tc.follow_plan(path, pivots, self.odom[:2], self.yaw, self.cfg, self.pivot_lead)
            c, _ = tc.run_out(path, pivots, self.odom[:2], self.yaw, c, phase, j, self.cfg, self.runout, self.fc_behind,
                              self.pivot_lead, grid)
        if phase != self.phase:
            self.phase = phase
            self.pub_status.publish(String(json.dumps(self.status())))
        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "world"
        cmd.position.x, cmd.position.y, cmd.position.z = float(c[0]), float(c[1]), float(self.odom[2])
        cmd.velocity.x, cmd.velocity.y = float(vel[0]), float(vel[1])
        cmd.yaw = float(yaw)
        cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        self.pub_cmd.publish(cmd)


if __name__ == "__main__":
    rospy.init_node("track_planner")
    TrackPlanner()
    rospy.spin()
