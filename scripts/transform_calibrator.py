#!/usr/bin/env python3
"""
Interactive Transform Calibrator for SLAM Drone Systems

Guides the user through 5 physical movements, records SLAM odometry via
docker exec, analyzes discrepancies between expected and observed motion,
and computes corrected extrinsic transform parameters for URDF and FAST-LIO.

Usage:
  transform_calibrator.py start                     # Init session
  transform_calibrator.py record baseline           # Hold still 5s
  transform_calibrator.py record forward            # Move forward ~1m
  transform_calibrator.py record right              # Move right ~1m
  transform_calibrator.py record up                 # Lift up ~0.5m
  transform_calibrator.py record yaw                # Rotate 90° CW
  transform_calibrator.py analyze                   # Compute corrections
  transform_calibrator.py apply [--dry-run]         # Preview/apply fixes

MCP Integration:
  run_diagnostic("transform_calibrator", "start --json")
  run_diagnostic("transform_calibrator", "record forward --json")
  run_diagnostic("transform_calibrator", "analyze --json")
  run_diagnostic("transform_calibrator", "apply --dry-run --json")
"""

import argparse
import json
import math
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import yaml

# ── Constants ─────────────────────────────────────────────────────────────────

FLIGHTS_DIR = Path("/home/dev/slam-agent/flights")
DEFAULT_CONFIG_DIR = Path("/home/dev/slam-gpu/config")
DEFAULT_CONTAINER = "slam_gpu_system"
SLAM_ODOM_TOPIC = "/Odometry"

# Bag recording uses the container's mounted bags directory.
# Detected at runtime via docker inspect; these are defaults for slam-gpu setup.
CONTAINER_BAGS_DIR = "/opt/slam_ws/bags"
HOST_BAGS_DIR = Path("/home/dev/slam-gpu/bags")

STEP_DURATIONS = {
    "baseline": 5,
    "forward": 10,
    "right": 10,
    "up": 10,
    "yaw": 10,
}

STEP_INSTRUCTIONS = {
    "baseline": "Hold the drone COMPLETELY STILL. Do not move it.",
    "forward": "Move the drone FORWARD ~1m along its nose direction, then stop.",
    "right": "Move the drone RIGHT ~1m (to its right side), then stop.",
    "up": "Lift the drone STRAIGHT UP ~0.5m, then stop.",
    "yaw": "Rotate the drone 90° CLOCKWISE (from above) about its vertical axis, then stop.",
}

# ── Quaternion / Rotation Helpers ──────────────────────────────────────────────

def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Multiply two quaternions [x, y, z, w]."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ])


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    """Conjugate of quaternion [x, y, z, w] → [-x, -y, -z, w]."""
    return np.array([-q[0], -q[1], -q[2], q[3]])


def quat_to_euler(q: np.ndarray) -> Tuple[float, float, float]:
    """Convert quaternion [x, y, z, w] to roll, pitch, yaw (radians)."""
    x, y, z, w = q
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    sinp = 2*(w*y - z*x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return roll, pitch, yaw


def euler_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Compute 3x3 rotation matrix from RPY (radians), ZYX convention."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr           ],
    ])


def rotation_matrix_to_euler(R: np.ndarray) -> Tuple[float, float, float]:
    """Extract RPY from 3x3 rotation matrix (ZYX convention)."""
    sy = math.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def normalize_quat(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    return q / n if n > 1e-9 else np.array([0.0, 0.0, 0.0, 1.0])


# ── CalibrationSession ─────────────────────────────────────────────────────────

class CalibrationSession:
    """Manages session state in flights/calibration_YYYYMMDD_HHMMSS/session.yaml."""

    def __init__(self, container: str = DEFAULT_CONTAINER,
                 config_dir: Path = DEFAULT_CONFIG_DIR,
                 session_dir: Optional[Path] = None):
        self.container = container
        self.config_dir = Path(config_dir)
        self._session_dir = Path(session_dir) if session_dir else None

    @property
    def session_dir(self) -> Optional[Path]:
        if self._session_dir:
            return self._session_dir
        # Find latest calibration session
        if not FLIGHTS_DIR.exists():
            return None
        dirs = sorted(FLIGHTS_DIR.glob("calibration_*"), reverse=True)
        return dirs[0] if dirs else None

    def start(self) -> Dict[str, Any]:
        """Create a new session directory and verify SLAM is reachable."""
        # Verify container is running
        result = subprocess.run(
            ["docker", "ps", "--filter", f"name={self.container}", "--format", "{{.Names}}"],
            capture_output=True, text=True
        )
        container_running = self.container in result.stdout
        if not container_running:
            return {
                "status": "error",
                "error": f"Container '{self.container}' is not running. Start SLAM first.",
            }

        # Check SLAM topic availability (inject ROS env from container process)
        _, _, ros_env = _inspect_container(self.container)
        env_flags = []
        for k, v in ros_env.items():
            env_flags += ["-e", f"{k}={v}"]
        ros_setup = (
            "source /opt/ros/humble/install/setup.bash 2>/dev/null || "
            "source /opt/ros/humble/setup.bash 2>/dev/null; "
            "source /opt/slam_ws/install/setup.bash 2>/dev/null; "
        )
        check = subprocess.run(
            ["docker", "exec"] + env_flags + [self.container, "bash", "-c",
             ros_setup + f"timeout 4 ros2 topic echo {SLAM_ODOM_TOPIC} 2>/dev/null"],
            capture_output=True, text=True, timeout=12
        )
        slam_available = "header" in check.stdout or "position" in check.stdout

        # Create session dir
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        session_dir = FLIGHTS_DIR / f"calibration_{ts}"
        session_dir.mkdir(parents=True, exist_ok=True)

        state = {
            "created": ts,
            "container": self.container,
            "config_dir": str(self.config_dir),
            "slam_topic": SLAM_ODOM_TOPIC,
            "slam_available": slam_available,
            "steps": {},
            "analysis": None,
        }
        self._session_dir = session_dir
        self._save_yaml(state)
        return {"status": "ok", "session_dir": str(session_dir), "slam_available": slam_available}

    def get_session(self) -> Optional[Dict[str, Any]]:
        d = self.session_dir
        if not d:
            return None
        p = d / "session.yaml"
        if not p.exists():
            return None
        with open(p) as f:
            return yaml.safe_load(f)

    def save_step_result(self, step: str, data: Dict[str, Any]) -> None:
        d = self.session_dir
        if not d:
            raise RuntimeError("No active session")
        state = self.get_session() or {}
        state.setdefault("steps", {})[step] = data
        self._session_dir = d
        self._save_yaml(state)

    def save_analysis(self, analysis: Dict[str, Any]) -> None:
        state = self.get_session() or {}
        state["analysis"] = analysis
        self._save_yaml(state)

    def _save_yaml(self, data: Dict[str, Any]) -> None:
        d = self._session_dir or self.session_dir
        (d / "session.yaml").write_text(yaml.dump(data, default_flow_style=False))


# ── DataRecorder ───────────────────────────────────────────────────────────────

def _inspect_container(container: str) -> Tuple[str, Path, Dict[str, str]]:
    """Return (container_bags_dir, host_bags_dir, ros_env) for a running container."""
    container_bags = CONTAINER_BAGS_DIR
    host_bags = HOST_BAGS_DIR
    ros_env: Dict[str, str] = {}

    try:
        # Get mounts
        result = subprocess.run(
            ["docker", "inspect", container,
             "--format", "{{range .Mounts}}{{.Source}}|{{.Destination}}\n{{end}}"],
            capture_output=True, text=True, timeout=5
        )
        for line in result.stdout.splitlines():
            parts = line.strip().split("|")
            if len(parts) == 2:
                src, dst = parts
                if "bags" in dst:
                    container_bags, host_bags = dst, Path(src)

        # Get ROS_DOMAIN_ID from container's PID 1 environment.
        # Only inject domain ID — setting RMW_IMPLEMENTATION explicitly breaks Fast RTPS.
        env_result = subprocess.run(
            ["docker", "exec", container, "bash", "-c",
             "cat /proc/1/environ 2>/dev/null | tr '\\0' '\\n' | "
             "grep -E '^ROS_DOMAIN_ID='"],
            capture_output=True, text=True, timeout=5
        )
        for line in env_result.stdout.splitlines():
            if "=" in line:
                k, _, v = line.partition("=")
                ros_env[k.strip()] = v.strip()
    except Exception:
        pass

    return container_bags, host_bags, ros_env


def _detect_bags_dir(container: str) -> Tuple[str, Path]:
    """Return (container_bags_dir, host_bags_dir) by inspecting docker mounts."""
    container_bags, host_bags, _ = _inspect_container(container)
    return container_bags, host_bags


class DataRecorder:
    """Records short bags via docker exec, reads back odometry from mcap or live echo."""

    def __init__(self, session: CalibrationSession):
        self.session = session
        self._container_bags_dir, self._host_bags_dir, self._ros_env = \
            _inspect_container(session.container)

    def _docker_exec(self, container: str, cmd: str, timeout: int = 30) -> subprocess.CompletedProcess:
        """Run bash -c cmd inside container with correct ROS env vars injected."""
        env_flags = []
        for k, v in self._ros_env.items():
            env_flags += ["-e", f"{k}={v}"]
        ros_setup = (
            "source /opt/ros/humble/install/setup.bash 2>/dev/null || "
            "source /opt/ros/humble/setup.bash 2>/dev/null; "
            "source /opt/slam_ws/install/setup.bash 2>/dev/null; "
        )
        return subprocess.run(
            ["docker", "exec"] + env_flags + [container, "bash", "-c", ros_setup + cmd],
            capture_output=True, text=True, timeout=timeout
        )

    def record(self, step: str) -> Dict[str, Any]:
        duration = STEP_DURATIONS[step]
        instructions = STEP_INSTRUCTIONS[step]
        container = self.session.container
        session_ts = self.session.session_dir.name
        container_step = f"{self._container_bags_dir}/{session_ts}/{step}"
        host_step = self._host_bags_dir / session_ts / step

        print(f"\n{'='*60}")
        print(f"STEP: {step.upper()}")
        print(f"{'='*60}")
        print(f"Instructions: {instructions}")
        print(f"Recording for {duration} seconds.")
        print("Get ready...")

        for i in range(3, 0, -1):
            print(f"  Starting in {i}...")
            time.sleep(1)
        print("  RECORDING NOW!")

        # Record bag in background, send SIGINT after N seconds for clean shutdown
        record_cmd = (
            f"mkdir -p {container_step} && cd {container_step} && "
            f"ros2 bag record -o recording {SLAM_ODOM_TOPIC} &"
            f"RECPID=$! && sleep {duration} && kill -INT $RECPID && wait $RECPID 2>/dev/null"
        )
        try:
            self._docker_exec(container, record_cmd, timeout=duration + 10)
        except subprocess.TimeoutExpired:
            pass

        print("  Recording complete.")

        # Read back: try bag first, then live-echo snapshot
        messages = self._read_from_bag(host_step)
        if len(messages) < 2:
            print("  Bag empty/unreadable — using live snapshot fallback...")
            messages = self._read_live_echo(container, duration=3)

        if len(messages) < 2:
            return {
                "status": "warn",
                "message": f"Only {len(messages)} messages captured. May be insufficient.",
                "message_count": len(messages),
                "displacement": [0.0, 0.0, 0.0],
                "rotation_euler": [0.0, 0.0, 0.0],
                "noise_rms": 0.0,
            }

        result = self._compute_displacement(messages)
        result["status"] = "ok"
        result["message_count"] = len(messages)
        print(f"  Captured {len(messages)} odometry messages.")
        print(f"  Displacement: x={result['displacement'][0]:.3f}  y={result['displacement'][1]:.3f}  z={result['displacement'][2]:.3f} m")
        return result

    def _read_from_bag(self, host_step: Path) -> List[Dict]:
        """Read odometry from rosbag2 sqlite3 (.db3) directly, bypassing metadata.yaml."""
        recording_dir = host_step / "recording"
        if not recording_dir.exists():
            return []

        # Find .db3 file
        db3_files = sorted(recording_dir.glob("*.db3"))
        if db3_files:
            return self._parse_db3(db3_files[0])

        # Try mcap if present
        mcap_files = list(recording_dir.glob("*.mcap"))
        if mcap_files:
            return self._parse_mcap(mcap_files[0])

        # Try rosbags Reader (needs metadata.yaml)
        if (recording_dir / "metadata.yaml").exists():
            return self._parse_rosbags(recording_dir)

        return []

    def _parse_db3(self, db3_path: Path) -> List[Dict]:
        """Read nav_msgs/Odometry from rosbag2 sqlite3 db3 file via rosbags CDR deserializer."""
        try:
            import sqlite3 as _sqlite3
            from rosbags.typesys import Stores, get_typestore
            typestore = get_typestore(Stores.ROS2_HUMBLE)

            db = _sqlite3.connect(str(db3_path))
            # Find topic id for our odometry topic
            row = db.execute(
                "SELECT id, type FROM topics WHERE name=?", (SLAM_ODOM_TOPIC,)
            ).fetchone()
            if not row:
                db.close()
                return []
            topic_id, msgtype = row

            messages = []
            for ts, data in db.execute(
                "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
                (topic_id,)
            ):
                try:
                    msg = typestore.deserialize_cdr(bytes(data), msgtype)
                    pos = msg.pose.pose.position
                    ori = msg.pose.pose.orientation
                    messages.append({
                        "t": ts / 1e9,
                        "x": float(pos.x), "y": float(pos.y), "z": float(pos.z),
                        "qx": float(ori.x), "qy": float(ori.y),
                        "qz": float(ori.z), "qw": float(ori.w),
                    })
                except Exception:
                    pass
            db.close()
            return messages
        except ImportError:
            return []
        except Exception:
            return []

    def _parse_rosbags(self, recording_dir: Path) -> List[Dict]:
        """Read via rosbags.rosbag2.Reader (needs metadata.yaml)."""
        try:
            from rosbags.rosbag2 import Reader
            messages = []
            with Reader(str(recording_dir)) as reader:
                connections = [c for c in reader.connections if c.topic == SLAM_ODOM_TOPIC]
                if not connections:
                    return []
                conn = connections[0]
                for c, timestamp, rawdata in reader.messages(connections=connections):
                    msg = reader.deserialize(rawdata, conn.msgtype)
                    pos = msg.pose.pose.position
                    ori = msg.pose.pose.orientation
                    messages.append({
                        "t": timestamp / 1e9,
                        "x": float(pos.x), "y": float(pos.y), "z": float(pos.z),
                        "qx": float(ori.x), "qy": float(ori.y),
                        "qz": float(ori.z), "qw": float(ori.w),
                    })
            return messages
        except Exception:
            return []

    def _read_live_echo(self, container: str, duration: int = 5) -> List[Dict]:
        """Capture live odometry by echoing the topic for N seconds."""
        cmd = f"timeout {duration} ros2 topic echo {SLAM_ODOM_TOPIC} 2>/dev/null"
        result = self._docker_exec(container, cmd, timeout=duration + 8)
        return self._parse_echo_output(result.stdout)

    def _read_odometry_via_docker(self, container: str, container_path: str) -> List[Dict]:
        """Legacy: read from bag via docker exec play."""
        bag_path = f"{container_path}/recording"
        cmd = (
            f"ros2 bag play {bag_path} --topics {SLAM_ODOM_TOPIC} & "
            f"sleep 2; "
            f"timeout 15 ros2 topic echo {SLAM_ODOM_TOPIC} 2>/dev/null | "
            f"grep -A3 'position:' | head -120"
        )
        result = self._docker_exec(container, cmd, timeout=25)
        return self._parse_echo_output(result.stdout)

    def _parse_mcap(self, mcap_path: Path) -> List[Dict]:
        """Parse mcap file using the mcap Python library if available."""
        try:
            from mcap.reader import make_reader
            from mcap_ros2.decoder import DecoderFactory
            messages = []
            with open(mcap_path, "rb") as f:
                reader = make_reader(f, decoder_factories=[DecoderFactory()])
                for schema, channel, message, ros_msg in reader.iter_decoded_messages(
                    topics=[SLAM_ODOM_TOPIC]
                ):
                    pos = ros_msg.pose.pose.position
                    ori = ros_msg.pose.pose.orientation
                    messages.append({
                        "t": message.log_time / 1e9,
                        "x": pos.x, "y": pos.y, "z": pos.z,
                        "qx": ori.x, "qy": ori.y, "qz": ori.z, "qw": ori.w,
                    })
            return messages
        except ImportError:
            return []
        except Exception:
            return []

    def _parse_echo_output(self, text: str) -> List[Dict]:
        """Parse ros2 topic echo output for nav_msgs/Odometry messages.

        Tracks context (position vs orientation section) to correctly
        assign x/y/z/w fields.
        """
        messages = []
        lines = text.splitlines()

        # State machine: track which section we're in
        in_position = False
        in_orientation = False
        pos: Dict[str, float] = {}
        ori: Dict[str, float] = {}

        for line in lines:
            stripped = line.strip()

            # Section markers
            if stripped == "position:":
                in_position = True
                in_orientation = False
                pos = {}
                continue
            if stripped == "orientation:":
                in_position = False
                in_orientation = True
                ori = {}
                continue
            # New message starts with "---" or "header:"
            if stripped in ("---", "header:"):
                if len(pos) == 3 and len(ori) == 4:
                    messages.append({
                        "t": 0.0,
                        "x": pos["x"], "y": pos["y"], "z": pos["z"],
                        "qx": ori["x"], "qy": ori["y"], "qz": ori["z"], "qw": ori["w"],
                    })
                in_position = in_orientation = False
                pos = {}
                ori = {}
                continue

            # Parse scalar fields (x: val, y: val, z: val, w: val)
            if ":" in stripped:
                key, _, val_str = stripped.partition(":")
                key = key.strip()
                val_str = val_str.strip()
                if key in ("x", "y", "z", "w") and val_str:
                    try:
                        val = float(val_str)
                        if in_position and key in ("x", "y", "z"):
                            pos[key] = val
                        elif in_orientation and key in ("x", "y", "z", "w"):
                            ori[key] = val
                    except ValueError:
                        pass

        # Flush last message
        if len(pos) == 3 and len(ori) == 4:
            messages.append({
                "t": 0.0,
                "x": pos["x"], "y": pos["y"], "z": pos["z"],
                "qx": ori["x"], "qy": ori["y"], "qz": ori["z"], "qw": ori["w"],
            })
        return messages

    def _compute_displacement(self, messages: List[Dict]) -> Dict[str, Any]:
        """Compute displacement vector, rotation delta, and noise RMS from pose list."""
        positions = np.array([[m["x"], m["y"], m["z"]] for m in messages])
        quats = np.array([[m["qx"], m["qy"], m["qz"], m["qw"]] for m in messages])

        first_pos = positions[0]
        last_pos = positions[-1]
        displacement = (last_pos - first_pos).tolist()

        # Rotation: q_delta = q_first^-1 * q_last
        q_first = normalize_quat(quats[0])
        q_last = normalize_quat(quats[-1])
        q_delta = quat_multiply(quat_conjugate(q_first), q_last)
        q_delta = normalize_quat(q_delta)
        roll_d, pitch_d, yaw_d = quat_to_euler(q_delta)

        # Noise: RMS of position jitter around the linear trend
        if len(positions) > 4:
            t = np.linspace(0, 1, len(positions))
            residuals = []
            for axis in range(3):
                coeffs = np.polyfit(t, positions[:, axis], 1)
                trend = np.polyval(coeffs, t)
                residuals.extend((positions[:, axis] - trend).tolist())
            noise_rms = float(np.sqrt(np.mean(np.array(residuals)**2)))
        else:
            noise_rms = 0.0

        return {
            "displacement": displacement,
            "rotation_euler": [math.degrees(roll_d), math.degrees(pitch_d), math.degrees(yaw_d)],
            "rotation_quat": q_delta.tolist(),
            "noise_rms": noise_rms,
            "first_pos": first_pos.tolist(),
            "last_pos": last_pos.tolist(),
        }


# ── MovementAnalyzer ───────────────────────────────────────────────────────────

class MovementAnalyzer:
    """Extracts per-step diagnostics from recorded displacement data."""

    def analyze_baseline(self, data: Dict) -> Dict[str, Any]:
        disp = np.array(data.get("displacement", [0, 0, 0]))
        drift = float(np.linalg.norm(disp))
        noise = data.get("noise_rms", 0.0)
        return {
            "drift_m": drift,
            "noise_rms": noise,
            "ok": drift < 0.05,
            "note": "OK" if drift < 0.05 else f"High drift {drift:.3f}m — check SLAM is stable",
        }

    def analyze_translation(self, data: Dict, expected_axis: int, expected_sign: float) -> Dict[str, Any]:
        disp = np.array(data.get("displacement", [0, 0, 0]))
        magnitude = float(np.linalg.norm(disp[:3]))
        if magnitude < 0.05:
            return {"magnitude_m": magnitude, "ok": False, "note": "Displacement too small to analyze"}

        # Angle between observed displacement and expected axis
        expected_vec = np.zeros(3)
        expected_vec[expected_axis] = expected_sign
        disp_norm = disp[:3] / magnitude
        dot = float(np.clip(np.dot(disp_norm, expected_vec), -1.0, 1.0))
        angle_deg = math.degrees(math.acos(abs(dot)))
        sign_ok = np.dot(disp_norm, expected_vec) > 0

        return {
            "magnitude_m": magnitude,
            "observed_axis": disp[:3].tolist(),
            "angle_from_expected_deg": angle_deg,
            "sign_ok": sign_ok,
            "ok": angle_deg < 15.0 and sign_ok,
            "note": f"Angle error {angle_deg:.1f}°" + ("" if sign_ok else ", SIGN FLIPPED"),
        }

    def analyze_rotation(self, data: Dict, expected_yaw_deg: float) -> Dict[str, Any]:
        rot = data.get("rotation_euler", [0, 0, 0])
        observed_yaw = rot[2]  # yaw in degrees
        yaw_error = observed_yaw - expected_yaw_deg
        sign_ok = (observed_yaw * expected_yaw_deg) > 0
        return {
            "observed_yaw_deg": observed_yaw,
            "expected_yaw_deg": expected_yaw_deg,
            "yaw_error_deg": yaw_error,
            "sign_ok": sign_ok,
            "ok": abs(yaw_error) < 20.0 and sign_ok,
            "note": f"Yaw error {yaw_error:.1f}°" + ("" if sign_ok else ", SIGN FLIPPED"),
        }


# ── TransformSolver ────────────────────────────────────────────────────────────

class TransformSolver:
    """Computes correction transforms from observed vs expected motion."""

    def solve(self, steps: Dict[str, Any]) -> Dict[str, Any]:
        corrections = {
            "yaw_correction_deg": 0.0,
            "pitch_correction_deg": 0.0,
            "roll_correction_deg": 0.0,
            "z_sign_flip": False,
            "yaw_sign_flip": False,
            "issues": [],
            "confidence": "low",
        }

        fwd = steps.get("forward", {})
        right = steps.get("right", {})
        up = steps.get("up", {})
        yaw_step = steps.get("yaw", {})

        fwd_disp = np.array(fwd.get("displacement", [0, 0, 0]))
        right_disp = np.array(right.get("displacement", [0, 0, 0]))
        up_disp = np.array(up.get("displacement", [0, 0, 0]))

        available = []

        # --- Yaw offset from forward movement ---
        fwd_mag = float(np.linalg.norm(fwd_disp[:2]))
        if fwd_mag > 0.05:
            yaw_from_fwd = math.degrees(math.atan2(fwd_disp[1], fwd_disp[0]))
            # Expected: forward = +X = angle 0
            yaw_correction_fwd = -yaw_from_fwd
            corrections["yaw_correction_deg"] = yaw_correction_fwd
            corrections["issues"].append(
                f"Forward movement: SLAM reported [{fwd_disp[0]:.2f}, {fwd_disp[1]:.2f}] — "
                f"yaw offset {yaw_correction_fwd:.1f}°"
            )
            available.append("forward")

        # --- Cross-validate with right movement ---
        right_mag = float(np.linalg.norm(right_disp[:2]))
        if right_mag > 0.05:
            yaw_from_right = math.degrees(math.atan2(right_disp[1], right_disp[0]))
            # Expected: right = -Y = angle -90° (270°)
            yaw_correction_right = -(yaw_from_right + 90.0)
            # Normalize to [-180, 180]
            yaw_correction_right = (yaw_correction_right + 180) % 360 - 180
            if "forward" in available:
                delta = abs(yaw_correction_right - corrections["yaw_correction_deg"])
                if delta < 20.0:
                    # Average the two estimates
                    corrections["yaw_correction_deg"] = (
                        corrections["yaw_correction_deg"] + yaw_correction_right
                    ) / 2
                    corrections["confidence"] = "high"
                else:
                    corrections["issues"].append(
                        f"WARNING: yaw estimates disagree "
                        f"(fwd={corrections['yaw_correction_deg']:.1f}°, right={yaw_correction_right:.1f}°)"
                    )
            else:
                corrections["yaw_correction_deg"] = yaw_correction_right
                available.append("right")

        # --- Z sign and pitch from up movement ---
        up_mag = float(np.linalg.norm(up_disp))
        if up_mag > 0.05:
            z_observed = up_disp[2]
            if z_observed < -0.05:
                corrections["z_sign_flip"] = True
                corrections["issues"].append(
                    f"Up movement: SLAM reported Z={z_observed:.3f} (should be +) — Z IS FLIPPED"
                )
            else:
                corrections["issues"].append(f"Up movement: Z correct ({z_observed:.3f}m)")
            # Pitch offset from XZ plane deviation
            xz_mag = math.sqrt(up_disp[0]**2 + up_disp[2]**2)
            if xz_mag > 0.05:
                pitch_from_up = math.degrees(math.atan2(up_disp[0], abs(up_disp[2])))
                corrections["pitch_correction_deg"] = -pitch_from_up
            available.append("up")

        # --- Yaw sign from yaw step ---
        if yaw_step:
            observed_yaw = yaw_step.get("rotation_euler", [0, 0, 0])[2]
            if observed_yaw > 10.0:
                corrections["yaw_sign_flip"] = True
                corrections["issues"].append(
                    f"Yaw rotation: SLAM reported +{observed_yaw:.1f}° (CW should be −) — YAW SIGN FLIPPED"
                )
            elif abs(observed_yaw) > 10.0:
                corrections["issues"].append(
                    f"Yaw rotation: {observed_yaw:.1f}° (expected ~−90°)"
                )
            available.append("yaw")

        if len(available) >= 2:
            corrections["confidence"] = corrections.get("confidence") or "medium"

        # Build corrected URDF and FAST-LIO values
        corrections["urdf"] = self._map_to_urdf(corrections)
        corrections["fastlio"] = self._map_to_fastlio(corrections)
        corrections["vision_bridge"] = self._map_to_vision_bridge(corrections)
        return corrections

    def _map_to_urdf(self, c: Dict) -> Dict[str, Any]:
        """Compute new URDF RPY for base_link_to_os_sensor."""
        # Read actual current URDF if possible; fall back to known default
        try:
            from pathlib import Path as _P
            updater = ConfigUpdater()
            _, current_rpy_list = updater.read_current_urdf()
            current_rpy = tuple(current_rpy_list)
        except Exception:
            current_rpy = (0.0, math.pi, 0.0)
        yaw_corr_deg = c["yaw_correction_deg"]
        pitch_corr_deg = c["pitch_correction_deg"]
        roll_corr_deg = c.get("roll_correction_deg", 0.0)

        # If corrections are negligible, preserve the current URDF values as-is
        # to avoid Euler decomposition ambiguity (e.g. (0,π,0) ≡ (π,0,π))
        if abs(yaw_corr_deg) < 0.5 and abs(pitch_corr_deg) < 0.5 and abs(roll_corr_deg) < 0.5:
            new_rpy = list(current_rpy)
        else:
            yaw_corr_rad = math.radians(yaw_corr_deg)
            pitch_corr_rad = math.radians(pitch_corr_deg)
            roll_corr_rad = math.radians(roll_corr_deg)
            R_current = euler_to_rotation_matrix(*current_rpy)
            R_corr = euler_to_rotation_matrix(roll_corr_rad, pitch_corr_rad, yaw_corr_rad)
            R_new = R_corr @ R_current
            new_rpy = list(rotation_matrix_to_euler(R_new))

        return {
            "rpy": [round(v, 6) for v in new_rpy],
            "rpy_deg": [round(math.degrees(v), 2) for v in new_rpy],
            "xml_snippet": f'<origin xyz="0.0 0.0 -0.060" rpy="{new_rpy[0]:.6f} {new_rpy[1]:.6f} {new_rpy[2]:.6f}"/>',
        }

    def _map_to_fastlio(self, c: Dict) -> Dict[str, Any]:
        """Compute new extrinsic_R for FAST-LIO."""
        yaw_corr_rad = math.radians(c["yaw_correction_deg"])
        pitch_corr_rad = math.radians(c["pitch_correction_deg"])
        roll_corr_rad = math.radians(c.get("roll_correction_deg", 0.0))
        R_corr = euler_to_rotation_matrix(roll_corr_rad, pitch_corr_rad, yaw_corr_rad)

        # Current extrinsic_R (180° yaw = [-1,0,0, 0,-1,0, 0,0,1])
        R_current = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]], dtype=float)
        if c.get("z_sign_flip"):
            R_current[2, 2] = -1.0

        R_new = R_corr @ R_current
        flat = [round(float(v), 6) for v in R_new.flatten()]
        return {
            "matrix_flat": flat,
            "yaml_snippet": (
                f"extrinsic_R: [{flat[0]}, {flat[1]}, {flat[2]},\n"
                f"              {flat[3]}, {flat[4]}, {flat[5]},\n"
                f"              {flat[6]}, {flat[7]}, {flat[8]}]"
            ),
        }

    def _map_to_vision_bridge(self, c: Dict) -> Dict[str, str]:
        yaw = c["yaw_correction_deg"]
        z_flip = c.get("z_sign_flip", False)
        lines = []
        if abs(yaw) > 5.0:
            lines.append(f"  yaw_cam: adjust by {yaw:.1f}° in vision bridge config")
        if z_flip:
            lines.append("  apply_y180_correction: check Z sign in vision bridge")
        if not lines:
            lines.append("  No vision bridge changes needed")
        return {"recommendation": "\n".join(lines)}


# ── ConfigUpdater ──────────────────────────────────────────────────────────────

class ConfigUpdater:
    """Reads and writes URDF and FAST-LIO config files."""

    def __init__(self, config_dir: Path = DEFAULT_CONFIG_DIR):
        self.config_dir = Path(config_dir)
        self.urdf_path = self.config_dir / "robot.urdf"
        self.fastlio_path = self.config_dir / "fast_lio_gpu.yaml"

    def read_current_urdf(self) -> Tuple[List[float], List[float]]:
        """Return current xyz and rpy of base_link_to_os_sensor joint."""
        tree = ET.parse(self.urdf_path)
        root = tree.getroot()
        for joint in root.iter("joint"):
            if joint.get("name") == "base_link_to_os_sensor":
                origin = joint.find("origin")
                if origin is not None:
                    xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
                    rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
                    return xyz, rpy
        return [0.0, 0.0, -0.06], [0.0, math.pi, 0.0]

    def read_current_fastlio(self) -> List[float]:
        """Return current extrinsic_R as flat list of 9 values."""
        with open(self.fastlio_path) as f:
            data = yaml.safe_load(f)
        try:
            return data["/**"]["ros__parameters"]["mapping"]["extrinsic_R"]
        except KeyError:
            return [-1, 0, 0, 0, -1, 0, 0, 0, 1]

    def write_urdf(self, new_rpy: List[float], dry_run: bool = False) -> str:
        """Update base_link_to_os_sensor origin rpy."""
        content = self.urdf_path.read_text()
        # Find and replace the origin line for this joint
        # We patch the XML attribute directly via string replacement for safety
        xyz, old_rpy = self.read_current_urdf()
        old_rpy_str = f"{old_rpy[0]:.5f} {old_rpy[1]:.5f} {old_rpy[2]:.5f}"
        new_rpy_str = f"{new_rpy[0]:.6f} {new_rpy[1]:.6f} {new_rpy[2]:.6f}"
        # Use XML parser for safe replacement
        tree = ET.parse(self.urdf_path)
        root = tree.getroot()
        changed = False
        for joint in root.iter("joint"):
            if joint.get("name") == "base_link_to_os_sensor":
                origin = joint.find("origin")
                if origin is not None:
                    origin.set("rpy", " ".join(f"{v:.6f}" for v in new_rpy))
                    changed = True
        if not changed:
            return "ERROR: Could not find joint base_link_to_os_sensor in URDF"
        if dry_run:
            return f"[DRY RUN] Would update URDF rpy to: {new_rpy_str}"
        ET.indent(tree, space="  ")
        tree.write(self.urdf_path, xml_declaration=True, encoding="unicode")
        return f"Updated URDF rpy → {new_rpy_str}"

    def write_fastlio(self, new_R_flat: List[float], dry_run: bool = False) -> str:
        """Update extrinsic_R in FAST-LIO config."""
        formatted = (
            f"[{new_R_flat[0]}, {new_R_flat[1]}, {new_R_flat[2]},\n"
            f"              {new_R_flat[3]}, {new_R_flat[4]}, {new_R_flat[5]},\n"
            f"              {new_R_flat[6]}, {new_R_flat[7]}, {new_R_flat[8]}]"
        )
        if dry_run:
            return f"[DRY RUN] Would update extrinsic_R to:\n      {formatted}"
        # Read-modify-write preserving comments using text replacement
        content = self.fastlio_path.read_text()
        # Replace the extrinsic_R block
        import re
        pattern = r'extrinsic_R:.*?(?=\n\s*\w|\Z)'
        new_block = f"extrinsic_R: {formatted}"
        updated = re.sub(pattern, new_block, content, flags=re.DOTALL)
        if updated == content:
            return "WARNING: Could not locate extrinsic_R block to replace"
        self.fastlio_path.write_text(updated)
        return f"Updated fast_lio_gpu.yaml extrinsic_R"


# ── ReportGenerator ────────────────────────────────────────────────────────────

class ReportGenerator:
    """Generates human-readable + JSON reports."""

    def generate(self, session: Dict[str, Any], corrections: Dict[str, Any],
                 session_dir: Path) -> Dict[str, Any]:
        steps = session.get("steps", {})
        baseline = steps.get("baseline", {})

        lines = [
            "=" * 60,
            "  TRANSFORM CALIBRATION REPORT",
            f"  Session: {session.get('created', 'unknown')}",
            "=" * 60,
            "",
            "── BASELINE ──",
            f"  Drift: {baseline.get('displacement', [0,0,0])}",
            f"  Noise RMS: {baseline.get('noise_rms', 0.0):.4f} m",
            "",
            "── OBSERVED DISPLACEMENTS ──",
        ]
        for step_name in ["forward", "right", "up", "yaw"]:
            d = steps.get(step_name, {})
            disp = d.get("displacement", [0, 0, 0])
            rot = d.get("rotation_euler", [0, 0, 0])
            lines.append(f"  {step_name:8s}: xyz=[{disp[0]:.3f}, {disp[1]:.3f}, {disp[2]:.3f}]  yaw={rot[2]:.1f}°")

        lines += [
            "",
            "── ISSUES DETECTED ──",
        ]
        for issue in corrections.get("issues", []):
            lines.append(f"  • {issue}")
        if not corrections.get("issues"):
            lines.append("  ✓ No significant transform errors detected")

        lines += [
            "",
            "── CORRECTIONS ──",
            f"  Yaw correction:   {corrections.get('yaw_correction_deg', 0):.2f}°",
            f"  Pitch correction: {corrections.get('pitch_correction_deg', 0):.2f}°",
            f"  Z sign flip:      {corrections.get('z_sign_flip', False)}",
            f"  Yaw sign flip:    {corrections.get('yaw_sign_flip', False)}",
            f"  Confidence:       {corrections.get('confidence', 'unknown')}",
            "",
            "── CORRECTED URDF ──",
        ]
        urdf = corrections.get("urdf", {})
        lines.append(f"  {urdf.get('xml_snippet', 'N/A')}")

        lines += [
            "",
            "── CORRECTED FAST-LIO extrinsic_R ──",
        ]
        fastlio = corrections.get("fastlio", {})
        lines.append(f"  {fastlio.get('yaml_snippet', 'N/A')}")

        lines += [
            "",
            "── VISION BRIDGE ──",
            corrections.get("vision_bridge", {}).get("recommendation", "N/A"),
            "",
            "To apply corrections:",
            "  transform_calibrator.py apply --dry-run   # preview",
            "  transform_calibrator.py apply             # apply",
            "=" * 60,
        ]

        report_text = "\n".join(lines)
        print(report_text)

        report_path_txt = session_dir / "report.txt"
        report_path_json = session_dir / "report.json"
        report_path_txt.write_text(report_text)

        report_data = {
            "session": session.get("created"),
            "steps": steps,
            "corrections": corrections,
        }
        report_path_json.write_text(json.dumps(report_data, indent=2))

        return report_data


# ── Subcommand Handlers ────────────────────────────────────────────────────────

def cmd_start(args) -> int:
    session = CalibrationSession(
        container=args.container,
        config_dir=args.config_dir,
    )
    result = session.start()
    if args.json:
        print(json.dumps(result, indent=2))
    elif result["status"] == "ok":
        print(f"✓ Session started: {result['session_dir']}")
        if not result.get("slam_available"):
            print("⚠ SLAM odometry topic not yet available — start FAST-LIO before recording")
    else:
        print(f"✗ {result.get('error', 'Unknown error')}", file=sys.stderr)
        return 1
    return 0


def cmd_record(args) -> int:
    step = args.step
    if step not in STEP_DURATIONS:
        print(f"Unknown step '{step}'. Choose from: {list(STEP_DURATIONS)}", file=sys.stderr)
        return 1

    session = CalibrationSession(
        container=args.container,
        config_dir=args.config_dir,
        session_dir=args.session,
    )
    if not session.session_dir:
        print("No active session. Run: transform_calibrator.py start", file=sys.stderr)
        return 1

    recorder = DataRecorder(session)
    result = recorder.record(step)
    session.save_step_result(step, result)

    if args.json:
        print(json.dumps({"step": step, **result}, indent=2))
    return 0 if result.get("status") in ("ok", "warn") else 1


def cmd_analyze(args) -> int:
    session_obj = CalibrationSession(
        container=args.container,
        config_dir=args.config_dir,
        session_dir=args.session,
    )
    session = session_obj.get_session()
    if not session:
        print("No session found. Run: transform_calibrator.py start", file=sys.stderr)
        return 1

    steps = session.get("steps", {})
    recorded = list(steps.keys())
    print(f"Analyzing steps: {recorded}")

    analyzer = MovementAnalyzer()
    diagnostics = {}
    if "baseline" in steps:
        diagnostics["baseline"] = analyzer.analyze_baseline(steps["baseline"])
    if "forward" in steps:
        diagnostics["forward"] = analyzer.analyze_translation(steps["forward"], 0, +1.0)
    if "right" in steps:
        diagnostics["right"] = analyzer.analyze_translation(steps["right"], 1, -1.0)
    if "up" in steps:
        diagnostics["up"] = analyzer.analyze_translation(steps["up"], 2, +1.0)
    if "yaw" in steps:
        diagnostics["yaw"] = analyzer.analyze_rotation(steps["yaw"], -90.0)

    solver = TransformSolver()
    corrections = solver.solve(steps)
    corrections["diagnostics"] = diagnostics

    reporter = ReportGenerator()
    session_dir = session_obj.session_dir
    report = reporter.generate(session, corrections, session_dir)
    session_obj.save_analysis(corrections)

    if args.json:
        print(json.dumps({"status": "ok", "corrections": corrections, "diagnostics": diagnostics}, indent=2))
    return 0


def cmd_apply(args) -> int:
    session_obj = CalibrationSession(
        container=args.container,
        config_dir=args.config_dir,
        session_dir=args.session,
    )
    session = session_obj.get_session()
    if not session:
        print("No session found. Run analyze first.", file=sys.stderr)
        return 1

    analysis = session.get("analysis")
    if not analysis:
        print("No analysis found. Run: transform_calibrator.py analyze", file=sys.stderr)
        return 1

    updater = ConfigUpdater(config_dir=args.config_dir)
    results = {}

    urdf_corrections = analysis.get("urdf", {})
    new_rpy = urdf_corrections.get("rpy")
    if new_rpy:
        msg = updater.write_urdf(new_rpy, dry_run=args.dry_run)
        results["urdf"] = msg
        print(f"URDF: {msg}")

    fastlio_corrections = analysis.get("fastlio", {})
    new_R = fastlio_corrections.get("matrix_flat")
    if new_R:
        msg = updater.write_fastlio(new_R, dry_run=args.dry_run)
        results["fastlio"] = msg
        print(f"FAST-LIO: {msg}")

    vision_rec = analysis.get("vision_bridge", {}).get("recommendation", "")
    results["vision_bridge"] = vision_rec
    print(f"Vision bridge recommendation:\n{vision_rec}")

    if not args.dry_run:
        print("\n✓ Config files updated. Restart FAST-LIO to apply changes:")
        print("  run_diagnostic('transform_calibrator', 'apply --json')")

    if args.json:
        print(json.dumps({"status": "ok", "dry_run": args.dry_run, "results": results}, indent=2))
    return 0


# ── Main ───────────────────────────────────────────────────────────────────────

def main() -> int:
    # Shared parent parser so --json and common flags work before OR after subcommand
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--container", default=DEFAULT_CONTAINER,
                        help="Docker container name (default: slam_gpu_system)")
    common.add_argument("--config-dir", default=str(DEFAULT_CONFIG_DIR),
                        help="Path to config directory (default: /home/dev/slam-gpu/config)")
    common.add_argument("--session", default=None,
                        help="Path to specific session directory (default: latest)")
    common.add_argument("--json", action="store_true",
                        help="Output JSON format for MCP compatibility")

    parser = argparse.ArgumentParser(
        description="Interactive Transform Calibrator for SLAM drone systems",
        parents=[common],
    )

    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("start", parents=[common], add_help=False,
                   help="Initialize a new calibration session")

    rec = sub.add_parser("record", parents=[common], add_help=False,
                         help="Record a calibration step")
    rec.add_argument("step", choices=list(STEP_DURATIONS),
                     help="Which movement to record")

    sub.add_parser("analyze", parents=[common], add_help=False,
                   help="Analyze recorded steps and compute corrections")

    apply_p = sub.add_parser("apply", parents=[common], add_help=False,
                             help="Apply computed corrections to config files")
    apply_p.add_argument("--dry-run", action="store_true",
                         help="Preview changes without writing files")

    # Ensure --dry-run has a default on all subcommands
    parser.set_defaults(dry_run=False)

    args = parser.parse_args()

    dispatch = {
        "start": cmd_start,
        "record": cmd_record,
        "analyze": cmd_analyze,
        "apply": cmd_apply,
    }
    return dispatch[args.command](args)


if __name__ == "__main__":
    sys.exit(main())
