#!/usr/bin/env python3
"""arm_monitor.py - Auto-start/stop ROS bag recording when ArduPilot arms/disarms.

Runs INSIDE the Docker container where MAVROS lives.
Subscribes to /mavros/state and manages bag recording lifecycle.

State machine:
  IDLE --(armed=True, connected=True)--> RECORDING
  RECORDING --(armed=False, connected=True)--> DEBOUNCE (3s timer)
  DEBOUNCE --(armed=True)--> RECORDING (cancel timer)
  DEBOUNCE --(timer expires)--> IDLE (stop recording)
  RECORDING --(connected=False)--> RECORDING (keep recording, log warning)
  Any --(SIGTERM/SIGINT)--> stop recording if active, exit
"""

import argparse
import datetime
import json
import logging
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time

# ---------------------------------------------------------------------------
# Logging setup
# ---------------------------------------------------------------------------

class TimestampedFormatter(logging.Formatter):
    def format(self, record):
        ts = datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S")
        return f"[arm_monitor] {ts} {record.levelname}: {record.getMessage()}"


def _setup_logging(json_output: bool) -> logging.Logger:
    logger = logging.getLogger("arm_monitor")
    logger.setLevel(logging.DEBUG)
    handler = logging.StreamHandler(sys.stdout)
    if not json_output:
        handler.setFormatter(TimestampedFormatter())
    logger.addHandler(handler)
    return logger


# ---------------------------------------------------------------------------
# ROS version detection
# ---------------------------------------------------------------------------

def _detect_ros_version():
    """Return ('ros2', rclpy) or ('ros1', rospy), or raise ImportError."""
    try:
        import rclpy  # noqa: F401
        return "ros2", rclpy
    except ImportError:
        pass
    try:
        import rospy  # noqa: F401
        return "ros1", rospy
    except ImportError:
        pass
    raise ImportError("Neither rclpy nor rospy found. Is ROS sourced in this environment?")


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

STATE_IDLE = "IDLE"
STATE_RECORDING = "RECORDING"
STATE_DEBOUNCE = "DEBOUNCE"

PID_FILE = "/tmp/flight_recorder.pid"
CURRENT_FLIGHT_FILE = "/tmp/flight_recorder_current"
MODE_FILE = "/tmp/flight_recorder_mode"

MIN_FREE_BYTES = 2 * 1024 ** 3  # 2 GB

ROS2_TOPICS_DEFAULT = [
    "/ouster/imu",
    "/fast_lio_gpu/odometry",
    "/fast_lio_gpu/path",
    "/tf",
    "/tf_static",
    "/mavros/vision_pose/pose",
    "/mavros/local_position/pose",
    "/mavros/local_position/velocity_local",
    "/mavros/state",
    "/mavros/battery",
    "/mavros/imu/data",
    "/mavros/imu/data_raw",
    "/mavros/rc/in",
    "/mavros/rc/out",
    "/mavros/setpoint_raw/target_attitude",
    "/mavros/target_actuator_control",
    "/mavros/esc_telemetry/telemetry",
    "/mavros/esc_status/status",
    "/mavros/global_position/global",
    "/mavros/mission/waypoints",
]

ROS1_TOPICS_DEFAULT = [
    "/ouster/imu",
    "/Odometry",
    "/slam/path",
    "/tf",
    "/tf_static",
    "/mavros/vision_pose/pose",
    "/mavros/local_position/pose",
    "/mavros/local_position/velocity_local",
    "/mavros/state",
    "/mavros/battery",
    "/mavros/imu/data",
    "/mavros/imu/data_raw",
    "/mavros/rc/in",
    "/mavros/rc/out",
    "/mavros/esc_telemetry",
    "/mavros/esc_status",
    "/mavros/global_position/global",
    "/mavros/mission/waypoints",
]

CANDIDATE_FLIGHTS_DIRS = [
    "/ws/flights",
    "/opt/slam_ws/flights",
    "/root/slam_ws/flights",
]


# ---------------------------------------------------------------------------
# ArmMonitor
# ---------------------------------------------------------------------------

class ArmMonitor:
    def __init__(self, args):
        self.args = args
        self.json_output = args.json
        self.dry_run = args.dry_run
        self.debounce_secs = args.debounce
        self.logger = _setup_logging(self.json_output)

        self.ros_version, self._ros_lib = _detect_ros_version()
        self.logger.info(f"Detected ROS version: {self.ros_version}")

        self.flights_dir = args.flights_dir or self._detect_flights_dir()
        self.logger.info(f"Using flights directory: {self.flights_dir}")

        self._state = STATE_IDLE
        self._state_lock = threading.Lock()
        self._debounce_timer: threading.Timer | None = None

        self._prev_armed: bool | None = None
        self._prev_connected: bool | None = None

        self._recording_proc: subprocess.Popen | None = None
        self._recording_start: datetime.datetime | None = None
        self._current_flight_dir: str | None = None
        self._current_flight_number: str | None = None
        self._topics: list[str] = self._build_topic_list()

        self._shutdown_event = threading.Event()
        self._watchdog_thread: threading.Thread | None = None

        signal.signal(signal.SIGTERM, self._handle_signal)
        signal.signal(signal.SIGINT, self._handle_signal)

    # ------------------------------------------------------------------
    # Topic list
    # ------------------------------------------------------------------

    def _build_topic_list(self) -> list[str]:
        if self.args.topics:
            topics = list(self.args.topics)
        elif self.ros_version == "ros2":
            topics = list(ROS2_TOPICS_DEFAULT)
        else:
            topics = list(ROS1_TOPICS_DEFAULT)

        if self.args.full and "/ouster/points" not in topics:
            topics.append("/ouster/points")

        return topics

    # ------------------------------------------------------------------
    # Flights directory
    # ------------------------------------------------------------------

    def _detect_flights_dir(self) -> str:
        for candidate in CANDIDATE_FLIGHTS_DIRS:
            if os.path.isdir(candidate):
                return candidate
        # Fall back to first candidate, create it
        d = CANDIDATE_FLIGHTS_DIRS[0]
        os.makedirs(d, exist_ok=True)
        self.logger.warning(f"No existing flights dir found; created {d}")
        return d

    # ------------------------------------------------------------------
    # Flight numbering
    # ------------------------------------------------------------------

    def _get_next_flight_number(self) -> str:
        max_num = 0
        if os.path.isdir(self.flights_dir):
            pattern = re.compile(r"^(\d+)_")
            for entry in os.listdir(self.flights_dir):
                m = pattern.match(entry)
                if m and os.path.isdir(os.path.join(self.flights_dir, entry)):
                    num = int(m.group(1), 10)
                    if num > max_num:
                        max_num = num
        return f"{max_num + 1:03d}"

    # ------------------------------------------------------------------
    # Disk space
    # ------------------------------------------------------------------

    def _check_disk_space(self) -> bool:
        try:
            usage = shutil.disk_usage(self.flights_dir)
            free = usage.free
            if free < MIN_FREE_BYTES:
                self.logger.error(
                    f"Insufficient disk space: {free / 1024**3:.1f} GB free, "
                    f"need at least {MIN_FREE_BYTES / 1024**3:.0f} GB"
                )
                return False
            return True
        except Exception as exc:
            self.logger.error(f"Disk space check failed: {exc}")
            return False

    # ------------------------------------------------------------------
    # Coordination lock
    # ------------------------------------------------------------------

    def _check_coordination_lock(self) -> bool:
        """Return True if it is safe to start recording (no manual recording active)."""
        if not os.path.exists(PID_FILE):
            return True
        try:
            content = open(PID_FILE).read().strip()
            # format: container_name:pid  or just pid
            parts = content.split(":")
            pid = int(parts[-1])
            # Check if that PID is still alive
            os.kill(pid, 0)
            # PID alive — check mode
            if os.path.exists(MODE_FILE):
                mode = open(MODE_FILE).read().strip()
                if mode == "manual":
                    self.logger.warning(
                        "Manual recording in progress (mode=manual). Skipping auto-start."
                    )
                    return False
            # mode=auto or file missing — another auto instance? skip anyway
            self.logger.warning(
                f"flight_recorder.pid exists with live PID {pid}. Skipping auto-start."
            )
            return False
        except (ValueError, ProcessLookupError, FileNotFoundError):
            # Stale PID file — safe to proceed
            self._remove_coordination_files()
            return True

    def _write_coordination_files(self, pid: int):
        try:
            with open(PID_FILE, "w") as f:
                f.write(f"auto:{pid}\n")
            with open(MODE_FILE, "w") as f:
                f.write("auto\n")
            if self._current_flight_dir:
                with open(CURRENT_FLIGHT_FILE, "w") as f:
                    f.write(self._current_flight_dir + "\n")
        except Exception as exc:
            self.logger.error(f"Failed to write coordination files: {exc}")

    def _remove_coordination_files(self):
        for path in (PID_FILE, MODE_FILE, CURRENT_FLIGHT_FILE):
            try:
                if os.path.exists(path):
                    os.remove(path)
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Recording start / stop
    # ------------------------------------------------------------------

    def _start_recording(self):
        if self.dry_run:
            self.logger.info("[DRY-RUN] Would start bag recording.")
            self._emit_json_event("recording_start_dry_run", {})
            return

        if not self._check_coordination_lock():
            return

        if not self._check_disk_space():
            return

        flight_number = self._get_next_flight_number()
        ts = datetime.datetime.now()
        ts_str = ts.strftime("%Y%m%d_%H%M%S")
        flight_dir_name = f"{flight_number}_{ts_str}"
        flight_dir = os.path.join(self.flights_dir, flight_dir_name)
        bag_dir = os.path.join(flight_dir, "recording")

        try:
            os.makedirs(bag_dir, exist_ok=True)
        except Exception as exc:
            self.logger.error(f"Failed to create flight directory {flight_dir}: {exc}")
            return

        self._current_flight_number = flight_number
        self._current_flight_dir = flight_dir
        self._recording_start = ts

        # Write metadata
        self._write_metadata(
            flight_dir,
            flight_number=flight_number,
            ts=ts,
            status="recording",
        )

        # Build subprocess command
        if self.ros_version == "ros2":
            cmd = ["ros2", "bag", "record", "-o", "recording"] + self._topics
            env = self._build_ros2_env()
        else:
            cmd = ["rosbag", "record", "-o", "recording"] + self._topics
            env = os.environ.copy()

        self.logger.info(
            f"Starting recording flight {flight_number} in {flight_dir} "
            f"({len(self._topics)} topics)"
        )

        try:
            proc = subprocess.Popen(
                cmd,
                cwd=flight_dir,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
            )
        except Exception as exc:
            self.logger.error(f"Failed to spawn recording process: {exc}")
            self._remove_coordination_files()
            return

        self._recording_proc = proc
        self._write_coordination_files(proc.pid)

        self.logger.info(f"Recording started (PID {proc.pid})")
        self._emit_json_event("recording_start", {
            "flight_number": flight_number,
            "flight_dir": flight_dir,
            "pid": proc.pid,
            "topics": self._topics,
        })

        # Start watchdog to detect if process dies unexpectedly
        self._watchdog_thread = threading.Thread(
            target=self._watchdog, daemon=True
        )
        self._watchdog_thread.start()

    def _stop_recording(self):
        if self.dry_run:
            self.logger.info("[DRY-RUN] Would stop bag recording.")
            self._emit_json_event("recording_stop_dry_run", {})
            return

        proc = self._recording_proc
        if proc is None:
            return

        self.logger.info(f"Stopping recording (PID {proc.pid})...")

        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except (ProcessLookupError, OSError) as exc:
            self.logger.warning(f"SIGINT to process group failed: {exc}")

        # Wait up to 10 seconds for graceful exit
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            ret = proc.poll()
            if ret is not None:
                break
            time.sleep(0.2)
        else:
            self.logger.warning("Recording process did not exit in 10s; sending SIGKILL")
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except (ProcessLookupError, OSError):
                pass

        stop_time = datetime.datetime.now()
        self._recording_proc = None
        self._remove_coordination_files()

        # Update metadata
        if self._current_flight_dir and self._recording_start:
            duration_s = (stop_time - self._recording_start).total_seconds()
            bag_size = self._get_bag_size(self._current_flight_dir)
            self._update_metadata(
                self._current_flight_dir,
                stop_time=stop_time,
                duration_s=duration_s,
                bag_size=bag_size,
            )
            self._update_flight_index(
                flight_number=self._current_flight_number,
                flight_dir_name=os.path.basename(self._current_flight_dir),
                flight_dir=self._current_flight_dir,
                ts=self._recording_start,
            )
            self.logger.info(
                f"Recording stopped. Duration: {duration_s:.1f}s, "
                f"Size: {bag_size}"
            )
            self._emit_json_event("recording_stop", {
                "flight_number": self._current_flight_number,
                "flight_dir": self._current_flight_dir,
                "duration_s": duration_s,
                "bag_size": bag_size,
            })

        self._current_flight_dir = None
        self._current_flight_number = None
        self._recording_start = None

    # ------------------------------------------------------------------
    # Watchdog
    # ------------------------------------------------------------------

    def _watchdog(self):
        """Monitor recording process. If it dies unexpectedly, return to IDLE."""
        proc = self._recording_proc
        if proc is None:
            return
        proc.wait()
        with self._state_lock:
            if self._state == STATE_RECORDING and self._recording_proc is proc:
                self.logger.warning(
                    f"Recording process (PID {proc.pid}) exited unexpectedly "
                    f"(returncode={proc.returncode}). Returning to IDLE."
                )
                self._recording_proc = None
                self._remove_coordination_files()
                self._current_flight_dir = None
                self._current_flight_number = None
                self._recording_start = None
                self._state = STATE_IDLE
                self._emit_json_event("recording_died", {"returncode": proc.returncode})

    # ------------------------------------------------------------------
    # Metadata helpers
    # ------------------------------------------------------------------

    def _write_metadata(
        self,
        flight_dir: str,
        flight_number: str,
        ts: datetime.datetime,
        status: str,
    ):
        meta = {
            "flight_number": flight_number,
            "timestamp": ts.isoformat(),
            "ros_version": self.ros_version,
            "recording_mode": "auto",
            "notes": "Auto-recorded on arm",
            "topics_recorded": self._topics,
            "status": status,
            "start_time": ts.isoformat(),
        }
        path = os.path.join(flight_dir, "metadata.yaml")
        try:
            with open(path, "w") as f:
                f.write(self._dict_to_yaml(meta))
        except Exception as exc:
            self.logger.error(f"Failed to write metadata: {exc}")

    def _update_metadata(
        self,
        flight_dir: str,
        stop_time: datetime.datetime,
        duration_s: float,
        bag_size: str,
    ):
        path = os.path.join(flight_dir, "metadata.yaml")
        try:
            with open(path) as f:
                content = f.read()

            # Simple line-based append for fields we want to add/update
            updates = {
                "status": "completed",
                "stop_time": stop_time.isoformat(),
                "duration": f"{duration_s:.0f}s",
                "bag_size": bag_size,
            }
            for key, value in updates.items():
                # Remove existing line if present
                content = re.sub(rf"^{key}:.*\n", "", content, flags=re.MULTILINE)
                content += f"{key}: {value}\n"

            with open(path, "w") as f:
                f.write(content)
        except Exception as exc:
            self.logger.error(f"Failed to update metadata: {exc}")

    def _update_flight_index(
        self,
        flight_number: str,
        flight_dir_name: str,
        flight_dir: str,
        ts: datetime.datetime,
    ):
        index_path = os.path.join(self.flights_dir, "flight_index.yaml")
        entry = (
            f"- flight_number: \"{flight_number}\"\n"
            f"  directory: {flight_dir_name}\n"
            f"  timestamp: {ts.isoformat()}\n"
            f"  path: {flight_dir}\n"
        )
        try:
            with open(index_path, "a") as f:
                f.write(entry)
        except Exception as exc:
            self.logger.error(f"Failed to update flight_index.yaml: {exc}")

    @staticmethod
    def _dict_to_yaml(d: dict) -> str:
        lines = []
        for key, value in d.items():
            if isinstance(value, list):
                lines.append(f"{key}:")
                for item in value:
                    lines.append(f"  - {item}")
            else:
                lines.append(f"{key}: {value}")
        return "\n".join(lines) + "\n"

    @staticmethod
    def _get_bag_size(flight_dir: str) -> str:
        total = 0
        for dirpath, _dirs, files in os.walk(flight_dir):
            for f in files:
                try:
                    total += os.path.getsize(os.path.join(dirpath, f))
                except OSError:
                    pass
        if total >= 1024 ** 3:
            return f"{total / 1024**3:.1f}G"
        elif total >= 1024 ** 2:
            return f"{total / 1024**2:.0f}M"
        else:
            return f"{total / 1024:.0f}K"

    # ------------------------------------------------------------------
    # ROS 2 environment for subprocess
    # ------------------------------------------------------------------

    @staticmethod
    def _build_ros2_env() -> dict:
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = env.get("ROS_DOMAIN_ID", "1")
        # Source humble setup if not already sourced
        ros_setup = "/opt/ros/humble/setup.bash"
        if os.path.exists(ros_setup) and "humble" not in env.get("AMENT_PREFIX_PATH", ""):
            # We can't source in Python directly, but we can add the key paths
            humble_lib = "/opt/ros/humble/lib"
            humble_bin = "/opt/ros/humble/bin"
            current_path = env.get("PATH", "")
            if humble_bin not in current_path:
                env["PATH"] = f"{humble_bin}:{current_path}"
            current_ld = env.get("LD_LIBRARY_PATH", "")
            if humble_lib not in current_ld:
                env["LD_LIBRARY_PATH"] = f"{humble_lib}:{current_ld}"
            env["AMENT_PREFIX_PATH"] = f"/opt/ros/humble:{env.get('AMENT_PREFIX_PATH', '')}"
        return env

    # ------------------------------------------------------------------
    # JSON event emitter
    # ------------------------------------------------------------------

    def _emit_json_event(self, event: str, data: dict):
        if not self.json_output:
            return
        payload = {
            "event": event,
            "timestamp": datetime.datetime.now().isoformat(),
            **data,
        }
        print(json.dumps(payload), flush=True)

    # ------------------------------------------------------------------
    # MAVROS wait
    # ------------------------------------------------------------------

    def _wait_for_mavros(self, timeout: int = 120):
        """Block until /mavros/state has publishers."""
        self.logger.info(f"Waiting for /mavros/state (timeout={timeout}s)...")
        deadline = time.monotonic() + timeout

        if self.ros_version == "ros2":
            self._wait_for_mavros_ros2(deadline)
        else:
            self._wait_for_mavros_ros1(deadline)

    def _wait_for_mavros_ros2(self, deadline: float):
        import rclpy
        from rclpy.node import Node

        # Probe node to check publisher count
        probe = rclpy.create_node("arm_monitor_probe")
        try:
            while time.monotonic() < deadline and not self._shutdown_event.is_set():
                pub_count = probe.count_publishers("/mavros/state")
                if pub_count > 0:
                    self.logger.info("/mavros/state is available.")
                    return
                self.logger.debug("Waiting for /mavros/state publisher...")
                time.sleep(2.0)
            self.logger.warning(
                "/mavros/state not available after timeout; continuing anyway."
            )
        finally:
            probe.destroy_node()

    def _wait_for_mavros_ros1(self, deadline: float):
        import rospy

        while time.monotonic() < deadline and not self._shutdown_event.is_set():
            try:
                pubs = rospy.get_published_topics()
                if any(t == "/mavros/state" for t, _ in pubs):
                    self.logger.info("/mavros/state is available.")
                    return
            except Exception:
                pass
            self.logger.debug("Waiting for /mavros/state publisher...")
            time.sleep(2.0)
        self.logger.warning(
            "/mavros/state not available after timeout; continuing anyway."
        )

    # ------------------------------------------------------------------
    # State machine — message callback
    # ------------------------------------------------------------------

    def _on_state_msg(self, msg):
        armed = bool(msg.armed)
        connected = bool(msg.connected)

        with self._state_lock:
            prev_armed = self._prev_armed
            self._prev_armed = armed
            self._prev_connected = connected

            # Detect edges
            armed_edge = (prev_armed is not None) and (armed != prev_armed)

            if self._state == STATE_IDLE:
                if armed and connected:
                    if prev_armed is None or armed_edge:
                        self.logger.info("Armed detected → starting recording.")
                        self._state = STATE_RECORDING
                        threading.Thread(
                            target=self._start_recording, daemon=True
                        ).start()

            elif self._state == STATE_RECORDING:
                if not connected:
                    self.logger.warning(
                        "MAVROS disconnected while recording — keeping recording active."
                    )
                    return
                if not armed and armed_edge:
                    self.logger.info(
                        f"Disarmed detected → entering DEBOUNCE ({self.debounce_secs}s)."
                    )
                    self._state = STATE_DEBOUNCE
                    self._debounce_timer = threading.Timer(
                        self.debounce_secs, self._debounce_expired
                    )
                    self._debounce_timer.start()

            elif self._state == STATE_DEBOUNCE:
                if not connected:
                    # Keep in debounce; connectivity issue
                    return
                if armed and armed_edge:
                    self.logger.info("Re-armed during debounce → cancelling timer, staying RECORDING.")
                    if self._debounce_timer is not None:
                        self._debounce_timer.cancel()
                        self._debounce_timer = None
                    self._state = STATE_RECORDING

    def _debounce_expired(self):
        with self._state_lock:
            if self._state != STATE_DEBOUNCE:
                return
            self.logger.info("Debounce timer expired → stopping recording.")
            self._state = STATE_IDLE
            self._debounce_timer = None

        # Stop outside the lock to avoid deadlock
        self._stop_recording()

    # ------------------------------------------------------------------
    # Signal handling
    # ------------------------------------------------------------------

    def _handle_signal(self, signum, frame):
        sig_name = signal.Signals(signum).name
        self.logger.info(f"Received {sig_name}; shutting down.")
        self._shutdown_event.set()

        with self._state_lock:
            if self._debounce_timer is not None:
                self._debounce_timer.cancel()
                self._debounce_timer = None
            prev_state = self._state
            self._state = STATE_IDLE

        if prev_state in (STATE_RECORDING, STATE_DEBOUNCE):
            self._stop_recording()

        # Give ROS a moment to clean up
        time.sleep(0.5)
        sys.exit(0)

    # ------------------------------------------------------------------
    # Main run loop
    # ------------------------------------------------------------------

    def run(self):
        os.makedirs(self.flights_dir, exist_ok=True)

        self._wait_for_mavros()

        if self.ros_version == "ros2":
            self._run_ros2()
        else:
            self._run_ros1()

    def _run_ros2(self):
        import rclpy
        from rclpy.node import Node
        from mavros_msgs.msg import State

        rclpy.init()
        node = rclpy.create_node("arm_monitor")
        self.logger.info("arm_monitor node started (ROS 2).")
        self._emit_json_event("started", {"ros_version": "ros2"})

        node.create_subscription(
            State,
            "/mavros/state",
            self._on_state_msg,
            10,
        )

        try:
            while not self._shutdown_event.is_set():
                rclpy.spin_once(node, timeout_sec=1.0)
        finally:
            node.destroy_node()
            try:
                rclpy.shutdown()
            except Exception:
                pass

    def _run_ros1(self):
        import rospy
        from mavros_msgs.msg import State

        rospy.init_node("arm_monitor", anonymous=False)
        self.logger.info("arm_monitor node started (ROS 1).")
        self._emit_json_event("started", {"ros_version": "ros1"})

        rospy.Subscriber("/mavros/state", State, self._on_state_msg, queue_size=10)

        # spin until shutdown
        rate = rospy.Rate(1)
        while not rospy.is_shutdown() and not self._shutdown_event.is_set():
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    parser = argparse.ArgumentParser(
        description="Auto-start/stop ROS bag recording on ArduPilot arm/disarm.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--flights-dir",
        metavar="PATH",
        default=None,
        help="Override auto-detected flights directory.",
    )
    parser.add_argument(
        "--debounce",
        type=float,
        default=3.0,
        metavar="N",
        help="Seconds to wait after disarm before stopping recording.",
    )
    parser.add_argument(
        "--full",
        action="store_true",
        help="Also record /ouster/points (large; use with fast storage).",
    )
    parser.add_argument(
        "--topics",
        nargs="+",
        metavar="TOPIC",
        default=None,
        help="Override default topic list.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Log transitions but do not actually start/stop recording.",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Emit JSON events to stdout (for MCP integration).",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    monitor = ArmMonitor(args)
    monitor.run()


if __name__ == "__main__":
    main()
