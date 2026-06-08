#!/usr/bin/env python3
"""
Live FAST-LIO + Flight-Controller monitor.

Refreshing terminal dashboard for the running SLAM->FC pipeline. Shows, for each
signal, the publish rate (Hz), age of the last message, and the current pose/state
so you can see at a glance whether FAST-LIO is alive and whether ArduPilot is
actually fusing the vision pose.

Pipeline (see config/fastlio_to_fc.launch):
    FAST-LIO  ->  /Odometry
              ->  fastlio_mavros_bridge.py  ->  /mavros/vision_pose/pose  ->  FC
    FC        ->  /mavros/local_position/pose   (its fused estimate)
              ->  /mavros/state                 (armed / mode / connected)

Run it inside the SLAM container (where ROS + mavros_msgs live):

    docker exec -it slam-hesai-fastlio bash -c \
        "source /root/slam_ws/devel/setup.bash && python3 /root/slam_logs/live_monitor.py"

or, since ./scripts is not mounted, copy it in first:

    docker cp scripts/live_monitor.py slam-hesai-fastlio:/tmp/live_monitor.py
    docker exec -it slam-hesai-fastlio bash -c \
        "source /root/slam_ws/devel/setup.bash && python3 /tmp/live_monitor.py"

Ctrl-C to quit.
"""

import argparse
import math
import sys
import threading
import time
from collections import deque

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped

try:
    from mavros_msgs.msg import State as MavrosState
except ImportError:  # mavros not on the path -> still run, just skip FC state
    MavrosState = None


# ---- ANSI helpers ---------------------------------------------------------
RESET = "\033[0m"
BOLD = "\033[1m"
DIM = "\033[2m"
RED = "\033[31m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
CYAN = "\033[36m"
CLEAR = "\033[2J\033[H"   # clear screen + home
HOME = "\033[H"


def yaw_deg(q):
    """Yaw (deg) from a geometry_msgs Quaternion."""
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


class Signal:
    """Tracks rate + last value for one topic."""

    def __init__(self, name, topic):
        self.name = name
        self.topic = topic
        self._stamps = deque(maxlen=50)   # wall-clock recv times for rate
        self.last_recv = None             # wall-clock of last message
        self.count = 0
        self.value = None                 # latest extracted display dict
        self._lock = threading.Lock()

    def mark(self, value):
        now = time.monotonic()
        with self._lock:
            self._stamps.append(now)
            self.last_recv = now
            self.count += 1
            self.value = value

    def snapshot(self):
        with self._lock:
            stamps = list(self._stamps)
            last = self.last_recv
            value = dict(self.value) if self.value else None
            count = self.count
        rate = 0.0
        if len(stamps) >= 2:
            span = stamps[-1] - stamps[0]
            if span > 0:
                rate = (len(stamps) - 1) / span
        age = (time.monotonic() - last) if last is not None else None
        return rate, age, count, value


class Monitor:
    def __init__(self, ns):
        self.odom = Signal("FAST-LIO  /Odometry", "/Odometry")
        self.vision = Signal("Vision->FC  vision_pose/pose", ns + "/vision_pose/pose")
        self.fc_pose = Signal("FC pose  local_position/pose", ns + "/local_position/pose")
        self.fc_vel = Signal("FC vel  local_position/velocity_local", ns + "/local_position/velocity_local")
        self.state = Signal("FC state  /mavros/state", ns + "/state")

        rospy.Subscriber(self.odom.topic, Odometry, self._on_odom, queue_size=20)
        rospy.Subscriber(self.vision.topic, PoseStamped, self._on_vision, queue_size=20)
        rospy.Subscriber(self.fc_pose.topic, PoseStamped, self._on_fc_pose, queue_size=20)
        rospy.Subscriber(self.fc_vel.topic, TwistStamped, self._on_fc_vel, queue_size=20)
        if MavrosState is not None:
            rospy.Subscriber(self.state.topic, MavrosState, self._on_state, queue_size=10)

    # ---- callbacks: extract just what we display --------------------------
    def _on_odom(self, m):
        p = m.pose.pose.position
        v = m.twist.twist.linear
        self.odom.mark({
            "x": p.x, "y": p.y, "z": p.z,
            "yaw": yaw_deg(m.pose.pose.orientation),
            "speed": math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2),
            "frame": m.header.frame_id,
        })

    def _on_vision(self, m):
        p = m.pose.position
        self.vision.mark({"x": p.x, "y": p.y, "z": p.z,
                          "yaw": yaw_deg(m.pose.orientation)})

    def _on_fc_pose(self, m):
        p = m.pose.position
        self.fc_pose.mark({"x": p.x, "y": p.y, "z": p.z,
                           "yaw": yaw_deg(m.pose.orientation)})

    def _on_fc_vel(self, m):
        v = m.twist.linear
        self.fc_vel.mark({"speed": math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2),
                          "vx": v.x, "vy": v.y, "vz": v.z})

    def _on_state(self, m):
        self.state.mark({"connected": m.connected, "armed": m.armed,
                         "guided": m.guided, "mode": m.mode})

    # ---- rendering --------------------------------------------------------
    @staticmethod
    def _rate_color(rate, age, want):
        if age is None:
            return RED      # never received
        if age > 1.0:
            return RED      # stale
        if rate < want * 0.5:
            return YELLOW   # alive but slow
        return GREEN

    def _fmt_signal(self, sig, want_hz):
        rate, age, count, val = sig.snapshot()
        color = self._rate_color(rate, age, want_hz)
        if age is None:
            status = f"{color}  --  no data{RESET}"
            rate_s = f"{color}{'--':>6}{RESET}"
            age_s = f"{DIM}{'--':>6}{RESET}"
        else:
            status = ""
            rate_s = f"{color}{rate:5.1f}{RESET}"
            age_s = f"{age*1000:5.0f}ms" if age < 1.0 else f"{RED}{age:5.1f}s{RESET}"
        head = f"  {BOLD}{sig.name:<38}{RESET} {rate_s} Hz  age {age_s}"
        return head, val, status

    def render(self):
        lines = []
        lines.append(f"{BOLD}{CYAN}== FAST-LIO + FC live monitor =={RESET}   "
                     f"{DIM}{time.strftime('%H:%M:%S')}  (Ctrl-C to quit){RESET}")
        lines.append("")

        # FAST-LIO
        head, val, status = self._fmt_signal(self.odom, 10.0)
        lines.append(head)
        if val:
            lines.append(f"      x={val['x']:+8.3f}  y={val['y']:+8.3f}  z={val['z']:+8.3f}  "
                         f"yaw={val['yaw']:+7.1f}d  |v|={val['speed']:5.2f} m/s  "
                         f"{DIM}frame={val['frame']}{RESET}")
        else:
            lines.append(f"      {status}")
        lines.append("")

        # Vision -> FC
        head, val, status = self._fmt_signal(self.vision, 10.0)
        lines.append(head)
        if val:
            lines.append(f"      x={val['x']:+8.3f}  y={val['y']:+8.3f}  z={val['z']:+8.3f}  "
                         f"yaw={val['yaw']:+7.1f}d")
        else:
            lines.append(f"      {status}")
        lines.append("")

        # FC fused pose
        head, val, status = self._fmt_signal(self.fc_pose, 10.0)
        lines.append(head)
        if val:
            lines.append(f"      x={val['x']:+8.3f}  y={val['y']:+8.3f}  z={val['z']:+8.3f}  "
                         f"yaw={val['yaw']:+7.1f}d")
        else:
            lines.append(f"      {status}")
        lines.append("")

        # FC velocity
        head, val, status = self._fmt_signal(self.fc_vel, 10.0)
        lines.append(head)
        if val:
            lines.append(f"      |v|={val['speed']:5.2f} m/s  "
                         f"vx={val['vx']:+6.2f} vy={val['vy']:+6.2f} vz={val['vz']:+6.2f}")
        else:
            lines.append(f"      {status}")
        lines.append("")

        # FC state
        if MavrosState is not None:
            _, age, _, val = self.state.snapshot()
            lines.append(f"  {BOLD}{self.state.name:<38}{RESET}")
            if val:
                conn = f"{GREEN}YES{RESET}" if val["connected"] else f"{RED}NO{RESET}"
                armed = f"{RED}ARMED{RESET}" if val["armed"] else f"{GREEN}disarmed{RESET}"
                lines.append(f"      connected={conn}  {BOLD}{armed}{RESET}  "
                             f"mode={CYAN}{val['mode']}{RESET}  guided={val['guided']}")
            else:
                lines.append(f"      {RED}-- no /mavros/state (mavros down?) --{RESET}")
            lines.append("")

        # Drift: FC fused pose vs what SLAM is feeding it
        _, va, _, vis = self.vision.snapshot()
        _, fa, _, fc = self.fc_pose.snapshot()
        if vis and fc and va is not None and fa is not None and va < 1.0 and fa < 1.0:
            dx, dy, dz = fc["x"] - vis["x"], fc["y"] - vis["y"], fc["z"] - vis["z"]
            dist = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
            dyaw = (fc["yaw"] - vis["yaw"] + 180) % 360 - 180
            dcol = GREEN if dist < 0.5 else (YELLOW if dist < 2.0 else RED)
            lines.append(f"  {BOLD}SLAM<->FC drift{RESET}  "
                         f"d={dcol}{dist:5.2f} m{RESET}  "
                         f"dx={dx:+6.2f} dy={dy:+6.2f} dz={dz:+6.2f}  dyaw={dyaw:+6.1f}d")
        else:
            lines.append(f"  {DIM}SLAM<->FC drift  -- waiting for both vision_pose and FC pose --{RESET}")

        sys.stdout.write(CLEAR + "\n".join(lines) + "\n")
        sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser(description="Live FAST-LIO + flight-controller monitor")
    ap.add_argument("--mavros-ns", default="/mavros", help="mavros namespace (default: /mavros)")
    ap.add_argument("--hz", type=float, default=4.0, help="dashboard refresh rate (default: 4)")
    args = ap.parse_args()

    rospy.init_node("live_monitor", anonymous=True, disable_signals=True)
    mon = Monitor(args.mavros_ns.rstrip("/"))

    rate = rospy.Rate(args.hz)
    try:
        while not rospy.is_shutdown():
            mon.render()
            rate.sleep()
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        pass
    finally:
        sys.stdout.write(RESET + "\n")
        sys.stdout.flush()


if __name__ == "__main__":
    main()
