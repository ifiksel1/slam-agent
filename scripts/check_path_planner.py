#!/usr/bin/env python3
"""Path planner diagnostic for GPS-denied drone SLAM systems."""

import argparse
import json
import math
import subprocess
import sys
from typing import Dict, List, Optional, Tuple

RESET  = "\033[0m"; BLUE = "\033[94m"; GREEN = "\033[92m"
YELLOW = "\033[93m"; RED  = "\033[91m"
PASS = f"{GREEN}✓{RESET}"; FAIL = f"{RED}✗{RESET}"
WARN = f"{YELLOW}⚠{RESET}"; INFO = f"{BLUE}ℹ{RESET}"

PLANNERS = {
    "waypoint_nav": {
        "packages": ["mavros", "mavros_msgs"],
        "needs_octomap": False,
        "output_topics": ["/mavros/setpoint_position/local"],
        "mode": "GUIDED",
        "desc": "Waypoint navigation via MAVROS setpoints",
    },
    "super": {
        "packages": ["super_planner", "rog_map"],
        "needs_octomap": False,
        "output_topics": ["/planning/pos_cmd", "/planning_cmd/poly_traj",
                          "/mavros/setpoint_position/local"],
        "mode": "GUIDED",
        "desc": "SUPER safety-assured planner + ROG-Map (HKU-Mars)",
    },
    "rog_map": {
        "packages": ["rog_map"],
        "needs_octomap": False,
        "output_topics": ["/rog_map/occ", "/rog_map/inf_occ", "/rog_map/frontier"],
        "mode": "GUIDED",
        "desc": "ROG-Map robocentric occupancy grid (standalone)",
    },
    "ego_planner": {
        "packages": ["ego_planner", "bspline_opt", "traj_utils"],
        "needs_octomap": False,
        "output_topics": ["/planning/trajectory", "/planning/bspline",
                          "/mavros/setpoint_position/local"],
        "mode": "GUIDED",
        "desc": "EGO-Planner gradient-based trajectory optimizer",
    },
    "fuel": {
        "packages": ["fuel_planner", "active_perception"],
        "needs_octomap": True,
        "output_topics": ["/fuel/trajectory", "/mavros/setpoint_position/local"],
        "mode": "GUIDED",
        "desc": "FUEL frontier-based exploration planner",
    },
    "nav2": {
        "packages": ["nav2_bringup", "nav2_planner", "nav2_controller"],
        "needs_octomap": False,
        "output_topics": ["/cmd_vel", "/plan"],
        "mode": "GUIDED",
        "desc": "ROS 2 Nav2 navigation stack",
    },
}
OCTOMAP_PKGS       = ["octomap_server", "octomap_msgs"]
SLAM_ODOM_CANDS    = ["/Odometry", "/lio_sam/mapping/odometry", "/coin_lio/odometry",
                      "/fast_lio/odometry", "/odometry/filtered", "/mavros/local_position/odom"]
CLOUD_CANDS        = ["/ouster/points", "/os_cloud_node/points", "/velodyne_points",
                      "/livox/lidar", "/points_raw"]
SP_TOPIC, ST_TOPIC, LP_TOPIC = ("/mavros/setpoint_position/local",
                                 "/mavros/state", "/mavros/local_position/pose")
TF_FRAMES = ["map", "odom", "base_link"]
MIN_HZ    = 5.0

class Runner:
    def __init__(self, container: Optional[str], ros_version: str, ros_distro: str):
        self.container  = container
        self.ros1       = ros_version.upper() == "ROS1"
        self._src       = f"source /opt/ros/{ros_distro}/setup.bash"

    def run(self, cmd: str, timeout: int = 10) -> Tuple[int, str]:
        full = f"{self._src} && {cmd}"
        args = (["docker", "exec", self.container, "bash", "-c", full]
                if self.container else ["bash", "-c", full])
        try:
            r = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
            return r.returncode, (r.stdout + r.stderr).strip()
        except subprocess.TimeoutExpired:
            return -1, f"[TIMEOUT {timeout}s]"
        except Exception as e:
            return -1, f"[ERROR: {e}]"

    def pkg_exists(self, pkg: str) -> bool:
        cmd = f"rospack find {pkg}" if self.ros1 else f"ros2 pkg prefix {pkg}"
        rc, _ = self.run(f"{cmd} 2>/dev/null")
        return rc == 0

    def topic_list(self) -> List[str]:
        cmd = "rostopic list" if self.ros1 else "ros2 topic list"
        rc, out = self.run(f"{cmd} 2>/dev/null")
        return [t.strip() for t in out.splitlines() if t.strip()] if rc == 0 else []

    def topic_hz(self, topic: str, dur: int = 3) -> Optional[float]:
        cmd = (f"timeout {dur+1} rostopic hz {topic} --window 10"
               if self.ros1 else
               f"timeout {dur+1} ros2 topic hz --window 10 {topic}")
        rc, out = self.run(f"{cmd} 2>/dev/null", timeout=dur + 5)
        for line in out.splitlines():
            if "average rate" in line.lower() or "average:" in line.lower():
                try:
                    return float(line.split(":")[-1].strip().split()[0])
                except (ValueError, IndexError):
                    pass
        return None

    def echo_one(self, topic: str) -> str:
        cmd = (f"timeout 5 rostopic echo -n 1 {topic}"
               if self.ros1 else
               f"timeout 5 ros2 topic echo --once {topic}")
        rc, out = self.run(f"{cmd} 2>/dev/null", timeout=8)
        return out if rc in (0, 124) else ""

    def mavros_mode(self) -> Optional[str]:
        raw = self.echo_one(ST_TOPIC)
        for line in raw.splitlines():
            if "mode:" in line.lower():
                return line.split(":", 1)[-1].strip().strip('"').strip("'")
        return None

    def tf_frames_seen(self) -> List[str]:
        cmd = ("timeout 3 rostopic echo -n 1 /tf 2>/dev/null | grep frame_id | awk '{print $2}'"
               if self.ros1 else
               "timeout 3 ros2 topic echo --once /tf 2>/dev/null | grep frame_id | awk '{print $2}'")
        rc, out = self.run(cmd, timeout=6)
        return [l.strip().strip('"') for l in out.splitlines() if l.strip()]

def chk(name: str, status: str, detail: str) -> Dict:
    return {"name": name, "status": status, "detail": detail}

def sym(status: str) -> str:
    return {"pass": PASS, "fail": FAIL, "warn": WARN, "skip": INFO}.get(status, INFO)

def show(c: Dict, verbose: bool):
    if verbose:
        print(f"  {sym(c['status'])} {c['name'].split(':',1)[-1]:<32}  {c['detail']}")

def check_packages(r: Runner, planner: str, v: bool) -> List[Dict]:
    pkgs = list(PLANNERS[planner]["packages"])
    if PLANNERS[planner]["needs_octomap"]:
        pkgs += OCTOMAP_PKGS
    out = []
    for pkg in pkgs:
        ok = r.pkg_exists(pkg)
        c = chk(f"pkg:{pkg}", "pass" if ok else "fail",
                f"Package '{pkg}' found" if ok else
                f"Package '{pkg}' missing — apt install ros-$ROS_DISTRO-{pkg.replace('_','-')}")
        out.append(c); show(c, v)
    return out

def check_octomap(r: Runner, planner: str, v: bool) -> List[Dict]:
    if not PLANNERS[planner]["needs_octomap"]:
        c = chk("octomap", "skip", f"OctoMap not required for '{planner}'")
        show(c, v); return [c]
    out = []
    for pkg in OCTOMAP_PKGS:
        ok = r.pkg_exists(pkg)
        c = chk(f"octomap:{pkg}", "pass" if ok else "fail",
                f"OctoMap '{pkg}' found" if ok else
                f"OctoMap '{pkg}' missing — apt install ros-$ROS_DISTRO-{pkg.replace('_','-')}")
        out.append(c); show(c, v)
    return out

def check_topics(r: Runner, planner: str, v: bool) -> List[Dict]:
    live = set(r.topic_list())
    out  = []

    def one(label: str, candidates: List[str], required: bool = True) -> Optional[str]:
        found = next((t for t in candidates if t in live), None)
        if found:
            c = chk(f"topic:{label}", "pass", f"{label}: {found}")
        else:
            c = chk(f"topic:{label}", "fail" if required else "warn",
                    f"{label} not found — tried: {', '.join(candidates)}")
        out.append(c); show(c, v)
        return found

    one("SLAM odometry",  SLAM_ODOM_CANDS)
    one("Point cloud",    CLOUD_CANDS)
    for t in PLANNERS[planner]["output_topics"]:
        one(f"Planner output ({t})", [t])
    one("MAVROS setpoint",  [SP_TOPIC])
    one("MAVROS state",     [ST_TOPIC])
    one("MAVROS local pos", [LP_TOPIC])
    return out

def check_rates(r: Runner, planner: str, v: bool) -> List[Dict]:
    out_topics = PLANNERS[planner]["output_topics"]
    rate_topics = [t for t in out_topics
                   if any(kw in t for kw in ("setpoint", "cmd_vel", "trajectory"))] or out_topics[:1]
    out = []
    for topic in rate_topics:
        if v:
            print(f"  {INFO} measuring rate on {topic} (3 s)…")
        hz = r.topic_hz(topic)
        if hz is None:
            c = chk(f"rate:{topic}", "warn",
                    f"{topic} rate unknown — topic may not be publishing")
        elif hz < MIN_HZ:
            c = chk(f"rate:{topic}", "fail",
                    f"{topic} rate {hz:.1f} Hz < {MIN_HZ} Hz minimum")
        else:
            c = chk(f"rate:{topic}", "pass",
                    f"{topic} rate {hz:.1f} Hz (>= {MIN_HZ} Hz)")
        out.append(c); show(c, v)
    return out

def check_tf(r: Runner, v: bool) -> List[Dict]:
    frames = set(r.tf_frames_seen())
    out = []
    for f in TF_FRAMES:
        if not frames:
            c = chk(f"tf:{f}", "warn",
                    f"Frame '{f}': could not query /tf — start SLAM first")
        elif f in frames:
            c = chk(f"tf:{f}", "pass", f"TF frame '{f}' present")
        else:
            c = chk(f"tf:{f}", "fail", f"TF frame '{f}' missing from tree")
        out.append(c); show(c, v)
    return out

def check_mode(r: Runner, planner: str, v: bool) -> List[Dict]:
    expected = PLANNERS[planner]["mode"]
    mode = r.mavros_mode()
    if mode is None:
        c = chk("ardupilot_mode", "warn",
                f"Could not read mode from {ST_TOPIC} — MAVROS running?")
    elif mode.upper() == expected.upper():
        c = chk("ardupilot_mode", "pass",
                f"Flight mode: {mode} (expected {expected})")
    else:
        c = chk("ardupilot_mode", "warn",
                f"Flight mode: {mode} (expected {expected} for autonomous nav)")
    show(c, v); return [c]

def check_safety(r: Runner, v: bool) -> List[Dict]:
    cmd = ("timeout 5 rosrun mavros mavparam get FENCE_ENABLE 2>/dev/null"
           if r.ros1 else
           "timeout 5 ros2 param get /mavros FENCE_ENABLE 2>/dev/null")
    rc, out = r.run(cmd, timeout=8)
    fence_ok = rc == 0 and out and "[TIMEOUT" not in out

    if fence_ok:
        enabled = "1" in out
        c = chk("geofence",
                "pass" if enabled else "warn",
                f"FENCE_ENABLE active ({out.strip()})" if enabled else
                f"FENCE_ENABLE={out.strip()} — enable geofence for GPS-denied flight")
    else:
        # Fallback: look for planner config with bounds
        rc2, out2 = r.run(
            "find / -name '*.yaml' -path '*/planner/*' 2>/dev/null | "
            "xargs grep -l 'max_vel\\|bounding_box\\|fence' 2>/dev/null | head -1",
            timeout=6,
        )
        if rc2 == 0 and out2.strip():
            c = chk("geofence", "warn",
                    f"MAVROS fence unavailable; bounds config: {out2.strip()}")
        else:
            c = chk("geofence", "warn",
                    "Could not verify geofence — set FENCE_ENABLE=1 in ArduPilot")
    show(c, v); return [c]

def check_setpoint(r: Runner, v: bool) -> List[Dict]:
    if v:
        print(f"  {INFO} echoing one setpoint from {SP_TOPIC}…")
    raw = r.echo_one(SP_TOPIC)
    if not raw:
        c = chk("setpoint_validity", "warn",
                f"No setpoint received from {SP_TOPIC} — planner publishing?")
        show(c, v); return [c]

    coords: Dict[str, Optional[float]] = {"x": None, "y": None, "z": None}
    for line in raw.splitlines():
        s = line.strip()
        for ax in ("x", "y", "z"):
            if s.startswith(f"{ax}:"):
                try:
                    coords[ax] = float(s.split(":", 1)[1].strip())
                except ValueError:
                    pass

    issues = []
    if any(val is None for val in coords.values()):
        issues.append("could not parse x/y/z")
    else:
        x, y, z = coords["x"], coords["y"], coords["z"]
        for ax, val in (("x", x), ("y", y), ("z", z)):
            if val is not None and math.isnan(val):
                issues.append(f"{ax} is NaN")
        if x is not None and y is not None and z is not None:
            dist = math.sqrt(x**2 + y**2 + z**2)
            if dist < 0.01:
                issues.append(f"at origin ({x:.3f},{y:.3f},{z:.3f}) — uninitialized?")
            if dist > 500:
                issues.append(f"magnitude {dist:.1f} m > 500 m — likely invalid")

    if issues:
        c = chk("setpoint_validity", "warn", "Setpoint issues: " + "; ".join(issues))
    else:
        x, y, z = coords["x"], coords["y"], coords["z"]
        c = chk("setpoint_validity", "pass",
                f"Setpoint valid: x={x:.3f} y={y:.3f} z={z:.3f}")
    show(c, v); return [c]

def section(title: str):
    print(f"\n{BLUE}{title}{RESET}\n{'─'*60}")

def print_checks(checks: List[Dict], verbose: bool):
    if not verbose:
        for c in checks:
            label = c["name"].split(":", 1)[-1]
            print(f"  {sym(c['status'])} {label:<35}  {c['detail']}")

def print_summary(checks: List[Dict], planner: str, ros_ver: str):
    passed   = sum(1 for c in checks if c["status"] == "pass")
    failed   = sum(1 for c in checks if c["status"] == "fail")
    warnings = sum(1 for c in checks if c["status"] == "warn")
    skipped  = sum(1 for c in checks if c["status"] == "skip")
    total    = len(checks) - skipped

    print(f"\n{'='*60}")
    print(f"Path Planner Diagnostic  —  {planner}  ({ros_ver})")
    print(f"{'='*60}")
    print(f"  {GREEN}Passed{RESET}:   {passed}/{total}")
    print(f"  {YELLOW}Warnings{RESET}: {warnings}/{total}")
    print(f"  {RED}Failed{RESET}:   {failed}/{total}")
    if skipped:
        print(f"  {BLUE}Skipped{RESET}:  {skipped}")
    print(f"{'='*60}\n")

    if failed == 0 and warnings == 0:
        print(f"{GREEN}All checks passed — planner setup looks healthy.{RESET}\n")
    elif failed == 0:
        print(f"{YELLOW}No failures, but review warnings before flight.{RESET}\n")
    else:
        print(f"{RED}Failures detected — fix before flying.{RESET}")
        print(f"\nCommon fixes:")
        print(f"  • Missing packages:  sudo apt install ros-$ROS_DISTRO-<name>")
        print(f"  • Topics not active: verify SLAM and planner nodes are running")
        print(f"  • Wrong mode:        set ArduPilot to {PLANNERS[planner]['mode']} mode")
        print(f"  • Geofence:          set FENCE_ENABLE=1 in Mission Planner / QGC\n")

def main():
    parser = argparse.ArgumentParser(
        description="Path planner diagnostic for GPS-denied drone SLAM systems",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --planner ego_planner
  %(prog)s --planner waypoint_nav --ros-version ROS1 --ros-distro noetic
  %(prog)s --planner fuel --container slam_ros2 --verbose
  %(prog)s --planner nav2 --json
        """,
    )
    parser.add_argument("--planner",     choices=list(PLANNERS.keys()), required=True)
    parser.add_argument("--ros-version", choices=["ROS1", "ROS2"], default="ROS2")
    parser.add_argument("--ros-distro",  default="humble",
                        help="ROS distro, e.g. humble, noetic (default: humble)")
    parser.add_argument("--container",   default=None,
                        help="Docker container name (optional)")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Show per-item detail during checks")
    parser.add_argument("--json",        action="store_true",
                        help="Output results as JSON (for MCP tool consumption)")
    args = parser.parse_args()

    r  = Runner(args.container, args.ros_version, args.ros_distro)
    v  = args.verbose
    js = args.json
    p  = args.planner

    if not js:
        print(f"\n{'='*60}")
        print(f"Path Planner Diagnostic")
        print(f"  planner     : {p}  ({PLANNERS[p]['desc']})")
        print(f"  ros_version : {args.ros_version}  ({args.ros_distro})")
        if args.container:
            print(f"  container   : {args.container}")
        print(f"{'='*60}")

    all_checks: List[Dict] = []

    steps = [
        ("1. Package Check",           lambda: check_packages(r, p, v)),
        ("2. OctoMap Check",           lambda: check_octomap(r, p, v)),
        ("3. Topic Check",             lambda: check_topics(r, p, v)),
        ("4. Rate Check",              lambda: check_rates(r, p, v)),
        ("5. TF Frame Check",          lambda: check_tf(r, v)),
        ("6. ArduPilot Mode Check",    lambda: check_mode(r, p, v)),
        ("7. Safety / Geofence Check", lambda: check_safety(r, v)),
        ("8. Setpoint Validity",       lambda: check_setpoint(r, v)),
    ]

    for title, fn in steps:
        if not js:
            section(title)
        checks = fn()
        all_checks.extend(checks)
        if not js:
            print_checks(checks, v)

    if js:
        passed   = sum(1 for c in all_checks if c["status"] == "pass")
        failed   = sum(1 for c in all_checks if c["status"] == "fail")
        warnings = sum(1 for c in all_checks if c["status"] == "warn")
        print(json.dumps({
            "planner":     p,
            "ros_version": args.ros_version,
            "ros_distro":  args.ros_distro,
            "container":   args.container,
            "checks":      all_checks,
            "summary":     {"passed": passed, "failed": failed, "warnings": warnings},
        }, indent=2))
    else:
        print_summary(all_checks, p, args.ros_version)

    sys.exit(0 if sum(1 for c in all_checks if c["status"] == "fail") == 0 else 1)

if __name__ == "__main__":
    main()
