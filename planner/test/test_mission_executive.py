#!/usr/bin/env python3
"""Tests for planner/mission_executive.py against a fake MAVROS, a fake bridge and a fake SUPER.

No SITL: this process is the vehicle. It serves /mavros/cmd/command (DO_SET_MODE), /mavros/cmd/takeoff
and /mavros/cmd/land, and also the arming and parameter services so that any call to them is caught.
It publishes /mavros/state, /mavros/local_position/pose and the takeoff/landing status texts, plays the
bridge (/planner_bridge/anchor, /planner_bridge/status; it moves only while /planner_bridge/enable, and only
toward the target of the planner its mode (/planner_bridge/set_mode, reported back in its status) listens
to), SUPER (flies toward each /planning/click_goal converted planner -> fc with the current anchor) and the
track planner (drives toward /planning/drive_goal on the ground, /track_planner/status). Like QuadRover
it refuses a takeoff while the tracks move (MAV_RESULT_TEMPORARILY_REJECTED). Each scenario runs the real
executive as a child process.

Run with ROS Noetic and mavros_msgs sourced:  python3 planner/test/test_mission_executive.py
"""
import json
import math
import os
import subprocess
import sys
import tempfile
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from mavros_msgs.msg import State, StatusText
from mavros_msgs.srv import (CommandBool, CommandBoolResponse, CommandLong, CommandLongResponse, CommandTOL,
                             CommandTOLResponse, ParamGet, ParamGetResponse, ParamSet, ParamSetResponse, SetMode,
                             SetModeResponse)
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

HERE = os.path.dirname(os.path.abspath(__file__))
EXEC = os.path.join(HERE, "..", "mission_executive.py")
NAMES = {0: "MANUAL", 4: "HOLD", 15: "GUIDED"}   # MAVROS reports the QR modes as CMODE(n)


def yaw_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


class FakeWorld:
    def __init__(self):
        self.forbidden = []          # arming, set_mode by name, parameter calls
        self.mode_cmds = []          # (time, mode number)
        self.goals = []              # (time, planner xyz)
        self.drive_goals = []
        self.status = []             # executive status JSON
        self.reset()
        rospy.Service("/mavros/cmd/command", CommandLong, self.on_command)
        rospy.Service("/mavros/cmd/takeoff", CommandTOL, self.on_takeoff)
        rospy.Service("/mavros/cmd/land", CommandTOL, self.on_land)
        rospy.Service("/mavros/cmd/arming", CommandBool, self.on_forbidden("arming", CommandBoolResponse))
        rospy.Service("/mavros/set_mode", SetMode, self.on_forbidden("set_mode", SetModeResponse))
        rospy.Service("/mavros/param/set", ParamSet, self.on_forbidden("param/set", ParamSetResponse))
        rospy.Service("/mavros/param/get", ParamGet, self.on_forbidden("param/get", ParamGetResponse))
        self.state_pub = rospy.Publisher("/mavros/state", State, queue_size=10)
        self.pose_pub = rospy.Publisher("/mavros/local_position/pose", PoseStamped, queue_size=10)
        self.text_pub = rospy.Publisher("/mavros/statustext/recv", StatusText, queue_size=10)
        self.status_pub = rospy.Publisher("/planner_bridge/status", String, queue_size=10)
        self.anchor_pub = rospy.Publisher("/planner_bridge/anchor", TransformStamped, queue_size=10)
        self.clear_pub = rospy.Publisher("/planner/landing_clear", Bool, queue_size=10)
        rospy.Subscriber("/planning/click_goal", PoseStamped, self.on_goal)
        rospy.Subscriber("/planning/drive_goal", PoseStamped, self.on_drive_goal)
        rospy.Subscriber("/mission_executive/status", String, lambda m: self.status.append(json.loads(m.data)))
        rospy.Subscriber("/planner_bridge/enable", Bool, self.on_enable)
        rospy.Subscriber("/planner_bridge/set_mode", String, self.on_set_mode)
        self.track_pub = rospy.Publisher("/track_planner/status", String, queue_size=10)
        rospy.Timer(rospy.Duration(0.05), self.step)

    def reset(self):
        self.pos = np.zeros(3)
        self.target = None
        self.mode = 0
        self.armed = True
        self.connected = True
        self.climb_to = None
        self.landing = False
        self.block = set()           # goal indices SUPER cannot reach
        self.goal_count = 0
        self.anchor_t = np.zeros(3)
        self.anchor_yaw = 0.0
        self.bridge_state = {"state": "OK"}
        self.landing_clear = True
        self.refuse_takeoff = False
        self.enabled = False
        self.enabled_during_fc_move = 0   # steps the bridge was enabled during the FC's takeoff/landing
        self.bridge_mode = "flight"       # None: a bridge that does not report its mode
        self.bridge_stuck = False         # never switches mode
        self.target_kind = None           # "flight" (SUPER) or "drive" (the track planner)
        self.mode_mismatch = 0            # steps enabled with the bridge in the wrong mode for the vehicle
        self.drive_steps = 0              # steps the fake moved on the ground (bridge in drive mode)
        self.drive_short = 0.0            # the track planner stops this short of the goal and says REACHED
        self.refuse_temp = 0              # refuse this many takeoffs "temporarily" (result 1)
        self.speed = 0.0
        self.coast = 0                    # steps the tracks keep moving after the stream stops
        self.takeoff_calls = []           # (speed at the call, result)
        self.on_ground_since_landing = False
        self.forbidden.clear(); self.mode_cmds.clear(); self.goals.clear(); self.drive_goals.clear(); self.status.clear()

    # ---- services ---------------------------------------------------------------------------
    def on_forbidden(self, name, resp):
        def h(_req):
            self.forbidden.append(name)
            return resp()
        return h

    def on_command(self, req):
        if int(req.command) != 176:
            self.forbidden.append("command %d" % req.command)
            return CommandLongResponse(success=False, result=4)
        self.mode = int(req.param2)
        self.mode_cmds.append((time.time(), self.mode))
        return CommandLongResponse(success=True, result=0)

    def on_enable(self, m):
        self.enabled = m.data

    def on_set_mode(self, m):
        if not self.bridge_stuck:
            self.bridge_mode = m.data

    def on_takeoff(self, req):
        moving = self.speed > 0.2
        if self.mode != 15 or not self.armed or self.refuse_takeoff:
            self.takeoff_calls.append((self.speed, 4))
            return CommandTOLResponse(success=False, result=4)
        if moving or self.refuse_temp > 0:
            # QuadRover: "Guided takeoff refused: moving" (MAV_RESULT_TEMPORARILY_REJECTED)
            self.refuse_temp = max(0, self.refuse_temp - 1)
            self.takeoff_calls.append((self.speed, 1))
            return CommandTOLResponse(success=False, result=1)
        self.takeoff_calls.append((self.speed, 0))
        self.target = None
        self.climb_to = self.pos[2] + req.altitude
        return CommandTOLResponse(success=True, result=0)

    def on_land(self, _req):
        if self.mode != 15:
            return CommandTOLResponse(success=False, result=4)
        self.landing = True
        self.target = None
        return CommandTOLResponse(success=True, result=0)

    # ---- fake SUPER: fly toward the goal, converted planner -> fc with the anchor ------------
    def planner_to_fc(self, p):
        c, s = math.cos(self.anchor_yaw), math.sin(self.anchor_yaw)
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        return R @ p + self.anchor_t

    def on_goal(self, m):
        p = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
        self.goals.append((time.time(), p))
        idx = self.goal_count
        self.goal_count += 1
        self.target = None if idx in self.block else self.planner_to_fc(p)
        self.target_kind = "flight"

    def on_drive_goal(self, m):
        p = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
        self.drive_goals.append((time.time(), p))
        q = self.planner_to_fc(p)
        idx = self.goal_count
        self.goal_count += 1
        self.target = None if idx in self.block else np.array([q[0], q[1], self.pos[2]])
        self.target_kind = "drive"
        self.track_pub.publish(String(data=json.dumps({"state": "DRIVING", "reason": "new goal"})))

    def step(self, _e):
        dt = 0.05
        if (self.climb_to is not None or self.landing) and self.enabled:
            self.enabled_during_fc_move += 1
        if self.climb_to is not None:
            self.pos[2] = min(self.climb_to, self.pos[2] + 1.5 * dt)
            if self.pos[2] >= self.climb_to - 1e-6:
                self.climb_to = None
                self.text_pub.publish(StatusText(severity=6, text="Guided takeoff complete"))
        elif self.landing:
            self.pos[2] = max(0.0, self.pos[2] - 1.0 * dt)
            if self.pos[2] <= 0.0:
                self.landing = False
                self.text_pub.publish(StatusText(severity=6, text="Guided landing complete"))
        elif self.target is not None and self.mode == 15 and self.enabled and self.bridge_state.get("state") == "OK" \
                and self.bridge_mode in (self.target_kind, None):
            d = self.target - self.pos
            n = np.linalg.norm(d)
            if self.target_kind == "drive":
                n -= self.drive_short
            if n > 1e-6:
                step = d / np.linalg.norm(d) * min(n, 2.0 * dt)
                self.pos += step
                self.speed = np.linalg.norm(step) / dt
                self.coast = 20 if self.target_kind == "drive" else 0
                if self.target_kind == "drive":
                    self.drive_steps += 1
            elif self.target_kind == "drive":
                self.speed = 0.0
                self.track_pub.publish(String(data=json.dumps({"state": "REACHED", "reason": ""})))
        elif self.coast > 0 and self.pos[2] < 0.05:
            self.coast -= 1      # the tracks run on for a second after the stream stops
            self.pos[0] += 0.5 * dt
            self.speed = 0.5
        else:
            self.speed = 0.0
        if self.enabled and self.bridge_mode is not None:
            on_ground = self.pos[2] < 0.05
            if self.bridge_mode != ("drive" if on_ground else "flight"):
                self.mode_mismatch += 1
        st = State(connected=self.connected, armed=self.armed, guided=True,
                   mode=NAMES.get(self.mode, "CMODE(%d)" % self.mode))
        self.state_pub.publish(st)
        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = "map"
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, self.pos)
        ps.pose.orientation.w = 1.0
        self.pose_pub.publish(ps)
        bs = dict(self.bridge_state)
        if self.bridge_mode is not None:
            bs["mode"] = self.bridge_mode
        self.status_pub.publish(String(data=json.dumps(bs)))
        tf = TransformStamped()
        tf.header.stamp = rospy.Time.now()
        tf.header.frame_id = "fc"
        tf.child_frame_id = "planner"
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, self.anchor_t)
        q = yaw_quat(self.anchor_yaw)
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = q
        self.anchor_pub.publish(tf)
        self.clear_pub.publish(Bool(data=self.landing_clear))


def mission_file(legs, **defaults):
    d = {"speed": 1.0, "accept_radius": 0.3, "leg_timeout_s": 30, "no_progress_s": 3}
    d.update(defaults)
    lines = ["name: test", "frame: fc", "takeoff_height: 1.0", "defaults: %s" % json.dumps(d), "legs:"]
    lines += ["  - %s" % json.dumps(l) for l in legs]
    f = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
    f.write("\n".join(lines) + "\n")
    f.close()
    return f.name


def run_executive(world, legs, setup=None, during=None, timeout=40, start=True, **defaults):
    path = mission_file(legs, **defaults)
    proc = subprocess.Popen([sys.executable, EXEC, "__name:=mission_executive", "_mission:=" + path,
                             "_unreachable_hold_s:=1.0", "_map_rebuild_s:=1.0", "_landing_check_s:=2.0",
                             "_goal_settle_s:=0.2", "_takeoff_settle_s:=4.0", "_bridge_mode_timeout_s:=1.0"],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        rospy.wait_for_service("/mission_executive/start", timeout=20)
        if setup:
            setup()
        time.sleep(0.5)
        if start:
            r = rospy.ServiceProxy("/mission_executive/start", Trigger)()
            world.start_reply = r
        t0 = time.time()
        while time.time() - t0 < timeout:
            phases = [s["phase"] for s in world.status]
            if phases and phases[-1] in ("DONE", "STOPPED"):
                break
            if during:
                during(time.time() - t0)
            time.sleep(0.05)
        return [s["phase"] for s in world.status]
    finally:
        proc.terminate()
        proc.wait(timeout=10)
        time.sleep(1.0)


RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append((name, ok, detail))
    print("%s  %-58s %s" % ("PASS" if ok else "FAIL", name, detail))


def main():
    rospy.init_node("fake_mavros_world")
    w = FakeWorld()
    time.sleep(1.0)

    # 1. a normal mission with an offset anchor: every leg type, goals converted fc -> planner
    w.reset()
    w.anchor_t = np.array([1.0, 2.0, 0.0]); w.anchor_yaw = math.pi / 2
    legs = [{"type": "takeoff", "height": 1.0}, {"type": "fly", "goal": [3.0, 0.0, 1.0]},
            {"type": "fly", "goal": [3.0, 3.0, 1.5]}, {"type": "land", "at": [2.0, 3.0]},
            {"type": "drive", "goal": [0.0, 3.0]}]
    ph = run_executive(w, legs)
    check("normal mission completes", ph[-1:] == ["DONE"], ",".join(ph))
    g0 = w.goals[0][1] if w.goals else None
    exp = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]]) @ (np.array([3.0, 0.0, 1.0]) - w.anchor_t)
    check("goal converted fc -> planner with the anchor", g0 is not None and np.allclose(g0, exp, atol=1e-6),
          "got %s want %s" % (None if g0 is None else g0.round(3).tolist(), exp.round(3).tolist()))
    check("drive leg sent to the track planner", len(w.drive_goals) == 1)
    check("drive leg driven with the bridge in drive mode, never in the wrong mode",
          w.drive_steps > 0 and w.mode_mismatch == 0, "drive steps %d, mismatched steps %d" % (w.drive_steps, w.mode_mismatch))
    check("modes commanded: GUIDED only", [m for _, m in w.mode_cmds] == [15], str([m for _, m in w.mode_cmds]))
    check("never arms or touches parameters", not w.forbidden, str(w.forbidden))
    check("bridge not enabled during the FC's takeoff and landing", w.enabled_during_fc_move == 0,
          "%d steps" % w.enabled_during_fc_move)
    check("bridge disabled at the end", not w.enabled)

    # 2. an unreachable goal: QRLOITER, hold, back to GUIDED, next leg
    w.reset()
    w.block = {0}
    legs = [{"type": "fly", "goal": [4.0, 0.0, 1.0]}, {"type": "fly", "goal": [0.0, 2.0, 1.0]}]
    ph = run_executive(w, legs)
    modes = [m for _, m in w.mode_cmds]
    check("unreachable goal -> UNREACHABLE, then the next leg", "UNREACHABLE" in ph and ph[-1:] == ["DONE"], ",".join(ph))
    check("unreachable goal -> QRLOITER (19) then GUIDED", modes == [15, 19, 15], str(modes))
    check("never arms or touches parameters (unreachable)", not w.forbidden, str(w.forbidden))

    # 3. an anchor jump: the bridge holds, the executive waits, then resends with the new anchor
    w.reset()
    legs = [{"type": "fly", "goal": [6.0, 0.0, 1.0]}]
    jumped = {"done": 0.0}

    def jump(t):
        # mid-leg, once the robot is 2 m along: the bridge re-anchors and holds for 1.5 s
        if w.pos[0] > 2.0 and not jumped["done"]:
            jumped["done"] = t
            w.bridge_state = {"state": "HOLD", "reason": "anchor_jump"}
            w.anchor_t = np.array([0.0, 0.5, 0.0])
        if jumped["done"] and t > jumped["done"] + 1.5 and w.bridge_state.get("state") != "OK":
            w.bridge_state = {"state": "OK"}
    ph = run_executive(w, legs, during=jump)
    check("anchor jump -> HOLD_SLAM, resume, mission completes", "HOLD_SLAM" in ph and ph[-1:] == ["DONE"], ",".join(ph))
    resent = len(w.goals) >= 2 and np.allclose(w.goals[-1][1], np.array([6.0, 0.0, 1.0]) - np.array([0.0, 0.5, 0.0]))
    check("goal resent after the hold, converted with the new anchor", resent,
          str([g.round(2).tolist() for _, g in w.goals]))

    # 4. the FC takes the vehicle (QRSmartRTL failsafe) mid-leg: the executive stops for good
    w.reset()
    legs = [{"type": "fly", "goal": [8.0, 0.0, 1.0]}, {"type": "fly", "goal": [0.0, 0.0, 1.0]}]
    taken = {"t": None}

    def failsafe(t):
        if t > 4.0 and taken["t"] is None:
            taken["t"] = time.time()
            w.mode = 23
    ph = run_executive(w, legs, during=failsafe)
    after_goals = [g for tg, g in w.goals if taken["t"] and tg > taken["t"]]
    after_modes = [m for tm, m in w.mode_cmds if taken["t"] and tm > taken["t"]]
    last = w.status[-1] if w.status else {}
    check("FC failsafe -> STOPPED", ph[-1:] == ["STOPPED"] and "FC mode" in last.get("reason", ""), json.dumps(last))
    check("no goal or mode command after the failsafe", not after_goals and not after_modes,
          "goals %d modes %s" % (len(after_goals), after_modes))

    # 5. landing spot not reported clear: QRLOITER and stop
    w.reset()
    w.landing_clear = False
    legs = [{"type": "land", "at": [1.0, 0.0]}]
    ph = run_executive(w, legs)
    check("landing spot not clear -> QRLOITER, STOPPED", ph[-1:] == ["STOPPED"] and 19 in [m for _, m in w.mode_cmds], ",".join(ph))
    check("no landing on a spot not reported clear", not any(s["phase"] == "LAND" for s in w.status), ",".join(ph))

    # 5b. not clear at first, clear within landing_check_s (a person walked off): lands
    w.reset()
    w.landing_clear = False
    asked = {"t": None}

    def walk_off(t):
        if any(s["phase"] == "LAND_CHECK" for s in w.status):
            asked["t"] = asked["t"] or time.time()
            if time.time() - asked["t"] > 1.0:
                w.landing_clear = True
    ph = run_executive(w, [{"type": "land", "at": [1.0, 0.0]}], during=walk_off)
    check("spot clear within landing_check_s -> lands, DONE", ph[-1:] == ["DONE"] and "LAND" in ph, ",".join(ph))

    # 6. start refused while disarmed; nothing commanded
    w.reset()
    w.armed = False
    ph = run_executive(w, [{"type": "fly", "goal": [1.0, 0.0, 1.0]}], timeout=3)
    r = getattr(w, "start_reply", None)
    check("start refused while disarmed (the pilot arms)", r is not None and not r.success, r.message if r else "")
    check("nothing commanded while disarmed", not w.mode_cmds and not w.goals and not w.forbidden)

    # 7. takeoff refused: stop, no goal sent (the closed-loop sim's first run skipped on to fly legs)
    w.reset()
    w.refuse_takeoff = True
    ph = run_executive(w, [{"type": "takeoff", "height": 1.0}, {"type": "fly", "goal": [1.0, 0.0, 1.0]}])
    check("takeoff refused -> STOPPED, no goal sent", ph[-1:] == ["STOPPED"] and not w.goals, ",".join(ph))

    # 8. land, drive, take off again (the owner's drive-and-fly mission): the bridge switches modes,
    # the takeoff waits for the tracks to stop, SUPER is re-seeded at the hover point
    w.reset()
    legs = [{"type": "takeoff", "height": 1.0}, {"type": "fly", "goal": [2.0, 0.0, 1.0]},
            {"type": "land", "at": [2.0, 1.0]}, {"type": "drive", "goal": [4.0, 1.0]},
            {"type": "drive", "goal": [4.0, -1.0]}, {"type": "takeoff", "height": 1.2},
            {"type": "fly", "goal": [0.0, 0.0, 1.2]}, {"type": "land", "at": [0.0, 0.5]}]
    ph = run_executive(w, legs, timeout=60)
    modes = [m for _, m in w.mode_cmds]
    check("land, drive, take off, fly, land: DONE", ph[-1:] == ["DONE"], ",".join(ph))
    check("bridge mode follows the vehicle (drive on the ground, flight in the air)",
          w.mode_mismatch == 0 and w.drive_steps > 0, "mismatched %d, drive steps %d" % (w.mode_mismatch, w.drive_steps))
    check("takeoff after a drive only once stopped (never refused for moving)",
          len(w.takeoff_calls) == 2 and all(r == 0 and sp <= 0.2 for sp, r in w.takeoff_calls), str(w.takeoff_calls))
    hover = [g for _, g in w.goals if abs(g[2] - 1.2) < 0.05 and np.linalg.norm(g[:2] - w.drive_goals[-1][1][:2]) < 1.0] \
        if w.drive_goals else []
    check("second takeoff re-seeds SUPER at its hover point", len(hover) >= 1, str([g.round(2).tolist() for _, g in w.goals]))
    check("modes commanded: GUIDED only (drive and fly)", modes == [15], str(modes))
    check("never arms or touches parameters (drive and fly)", not w.forbidden, str(w.forbidden))
    check("bridge not enabled during the second takeoff and landing", w.enabled_during_fc_move == 0,
          "%d steps" % w.enabled_during_fc_move)

    # 9. QuadRover refuses the takeoff "temporarily" twice: asked again, flies
    w.reset()
    w.refuse_temp = 2
    ph = run_executive(w, [{"type": "takeoff", "height": 1.0}, {"type": "fly", "goal": [1.0, 0.0, 1.0]}])
    check("takeoff temporarily rejected twice -> asked again, DONE",
          ph[-1:] == ["DONE"] and [r for _, r in w.takeoff_calls] == [1, 1, 0], "%s %s" % (",".join(ph), w.takeoff_calls))

    # 10. a mission that starts on the ground with a drive: no takeoff first; the track planner's
    # REACHED ends the leg (the FC 0.2 m behind its odometry point, outside a 0.1 m accept radius)
    w.reset()
    w.drive_short = 0.2
    ph = run_executive(w, [{"type": "drive", "goal": [2.0, 0.0], "accept_radius": 0.1}])
    reached = [s for s in w.status if s.get("reached_by") == "track planner"]
    check("drive-first mission: no takeoff, REACHED ends the leg",
          ph[-1:] == ["DONE"] and not w.takeoff_calls and reached, "%s takeoffs %s" % (",".join(ph), w.takeoff_calls))

    # 11. an unreachable drive goal on the ground: never QRLOITER (it would spool the props), next leg
    w.reset()
    w.block = {0}
    ph = run_executive(w, [{"type": "drive", "goal": [3.0, 0.0]}, {"type": "drive", "goal": [0.0, 1.0]}])
    modes = [m for _, m in w.mode_cmds]
    check("unreachable drive goal -> hold in GUIDED (no QR mode), next leg",
          "UNREACHABLE" in ph and ph[-1:] == ["DONE"] and modes == [15], "%s modes %s" % (",".join(ph), modes))

    # 12. the bridge does not switch to drive mode: no drive goal streamed, STOPPED on the ground
    w.reset()
    w.bridge_stuck = True
    ph = run_executive(w, [{"type": "drive", "goal": [2.0, 0.0]}])
    last = w.status[-1] if w.status else {}
    check("bridge stuck in flight mode -> STOPPED, never enabled, GUIDED only",
          ph[-1:] == ["STOPPED"] and "bridge did not switch" in last.get("reason", "") and w.drive_steps == 0
          and [m for _, m in w.mode_cmds] == [15], json.dumps(last))

    # 13. takeoff refused on the ground: no QRLOITER (QuadRover's drive-to-fly transition)
    w.reset()
    w.refuse_takeoff = True
    ph = run_executive(w, [{"type": "takeoff", "height": 1.0}])
    check("takeoff refused on the ground -> stays GUIDED", ph[-1:] == ["STOPPED"] and [m for _, m in w.mode_cmds] == [15],
          str([m for _, m in w.mode_cmds]))

    failed = [n for n, ok, _ in RESULTS if not ok]
    print("\n%d of %d checks passed" % (len(RESULTS) - len(failed), len(RESULTS)))
    rospy.signal_shutdown("done")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
