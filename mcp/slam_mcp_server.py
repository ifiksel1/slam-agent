#!/usr/bin/env python3
"""SLAM Integration MCP Server.

Provides tools for script execution, profile management, learning persistence,
and git-backed knowledge sharing for the SLAM integration agent.
"""

import fcntl
import json
import os
import subprocess
import sys
import tempfile
from datetime import datetime
from pathlib import Path
from typing import Optional

import yaml
from mcp.server.fastmcp import FastMCP

mcp = FastMCP("slam-tools")

PROJECT_ROOT = Path(__file__).resolve().parent.parent
SCRIPTS_DIR = PROJECT_ROOT / "scripts"
LEARNED_DIR = PROJECT_ROOT / "docs" / "learned"
PROFILES_DIR = PROJECT_ROOT / "docs" / "profiles"
HARDWARE_PROFILES_FILE = LEARNED_DIR / "hardware_profiles.yaml"
SOLUTIONS_LOG_FILE = LEARNED_DIR / "solutions_log.yaml"
KNOWN_GOOD_CONFIGS_DIR = LEARNED_DIR / "known_good_configs"

# Whitelisted scripts to prevent arbitrary execution
INSTALL_SCRIPTS = {
    "install_slam_integration": "install_slam_integration.sh",
    "install_core_ros_packages": "install_core_ros_packages.sh",
    "install_lidar_driver": "install_lidar_driver.sh",
    "install_camera_driver": "install_camera_driver.sh",
    "install_slam_algorithm": "install_slam_algorithm.sh",
    "install_mavros": "install_mavros.sh",
    "install_vision_to_mavros": "install_vision_to_mavros.sh",
    "install_dds_bridge": "install_dds_bridge.sh",
}

DIAGNOSTIC_SCRIPTS = {
    "verify_installation": ("verify_installation.sh", "bash"),
    "slam_diagnostics": ("slam_diagnostics.sh", "bash"),
    "check_topic_pipeline": ("check_topic_pipeline.py", "python3"),
    "check_tf_tree": ("check_tf_tree.py", "python3"),
    "check_autopilot_params": ("check_autopilot_params.py", "python3"),
    "check_sensor_time_sync": ("check_sensor_time_sync.py", "python3"),
    "check_urdf": ("check_urdf.py", "python3"),
    "analyze_slam_bag": ("analyze_slam_bag.py", "python3"),
    "docker_diagnostics": ("docker_diagnostics.py", "python3"),
    "preflight_check_docker": ("preflight_check_docker.sh", "bash"),
}

DEPLOY_SCRIPTS = {
    "deploy_docker_slam": "deploy_docker_slam.sh",
}


def _run_command(cmd: list[str], timeout: int = 300) -> str:
    """Run a command and return combined stdout/stderr."""
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout,
            cwd=str(PROJECT_ROOT),
        )
        output = ""
        if result.stdout:
            output += result.stdout
        if result.stderr and result.stderr.strip():
            output += f"\n--- STDERR ---\n{result.stderr}" if output else result.stderr
        if result.returncode != 0:
            output += f"\n[Exit code: {result.returncode}]"
        return output.strip() or "[No output]"
    except subprocess.TimeoutExpired:
        return f"[Error: Command timed out after {timeout}s]"
    except FileNotFoundError as e:
        return f"[Error: Command not found: {e}]"
    except Exception as e:
        return f"[Error: {e}]"


def _read_yaml(path: Path) -> dict:
    """Read a YAML file, returning empty dict on error."""
    try:
        with open(path) as f:
            data = yaml.safe_load(f)
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def _write_yaml_atomic(path: Path, data: dict) -> None:
    """Write YAML atomically with file locking."""
    path.parent.mkdir(parents=True, exist_ok=True)
    fd = None
    tmp_path = None
    try:
        fd = os.open(str(path), os.O_RDWR | os.O_CREAT)
        fcntl.flock(fd, fcntl.LOCK_EX)
        tmp_fd, tmp_path = tempfile.mkstemp(
            dir=str(path.parent), suffix=".yaml.tmp"
        )
        with os.fdopen(tmp_fd, "w") as tmp_f:
            yaml.dump(data, tmp_f, default_flow_style=False, sort_keys=False)
        os.replace(tmp_path, str(path))
        tmp_path = None
    finally:
        if fd is not None:
            fcntl.flock(fd, fcntl.LOCK_UN)
            os.close(fd)
        if tmp_path and os.path.exists(tmp_path):
            os.unlink(tmp_path)


# ─── Installation Tools ──────────────────────────────────────────────

@mcp.tool()
def run_install_script(script_name: str, args: str = "") -> str:
    """Run a SLAM installation script.

    Available scripts:
    - install_slam_integration [config_file.yaml]
    - install_core_ros_packages <ROS_VERSION> <ROS_DISTRO>
    - install_lidar_driver <LIDAR_TYPE> <WORKSPACE_PATH> <ROS_VERSION>
    - install_camera_driver <CAMERA_TYPE> <WORKSPACE_PATH> <ROS_VERSION>
    - install_slam_algorithm <SLAM_ALGORITHM> <WORKSPACE_PATH> <ROS_VERSION>
    - install_mavros <ROS_VERSION> <ROS_DISTRO>
    - install_vision_to_mavros <WORKSPACE_PATH> <ROS_VERSION>
    - install_dds_bridge <ROS_VERSION> <ROS_DISTRO> <FLIGHT_CONTROLLER> <WORKSPACE_PATH>

    Args:
        script_name: Name without .sh extension (e.g., "install_lidar_driver")
        args: Space-separated arguments to pass to the script
    """
    if script_name not in INSTALL_SCRIPTS:
        return (
            f"Unknown install script '{script_name}'. "
            f"Available: {', '.join(sorted(INSTALL_SCRIPTS))}"
        )
    script_file = SCRIPTS_DIR / INSTALL_SCRIPTS[script_name]
    if not script_file.exists():
        return f"Script not found: {script_file}"
    cmd = ["bash", str(script_file)]
    if args:
        cmd.extend(args.split())
    return _run_command(cmd, timeout=600)


@mcp.tool()
def run_diagnostic(
    diagnostic_name: str, args: str = ""
) -> str:
    """Run a SLAM diagnostic or validation script.

    Available diagnostics:
    - verify_installation <ROS_VERSION> <ROS_DISTRO> <WORKSPACE_PATH>
    - slam_diagnostics (no args)
    - check_topic_pipeline [--sensor TOPIC --slam TOPIC --vision TOPIC --mavros TOPIC --duration SEC]
    - check_tf_tree [--frames frame1 frame2 --source SRC --target TGT --verbose --visualize]
    - check_autopilot_params [--autopilot ardupilot|px4 --generate --baseline FILE]
    - check_sensor_time_sync <topic1> <topic2> [--duration SEC --log FILE]
    - check_urdf [urdf_file] [--from-param --required-frames f1 f2 --verbose --visualize]
    - analyze_slam_bag <bag_file> [--slam-topic TOPIC --start SEC --end SEC --plot --report]
    - docker_diagnostics (no args) — full health check of containerized SLAM system
    - preflight_check_docker (no args) — pre-flight verification for Docker SLAM + Ouster

    Args:
        diagnostic_name: Name without extension (e.g., "check_tf_tree")
        args: Space-separated arguments to pass to the script
    """
    if diagnostic_name not in DIAGNOSTIC_SCRIPTS:
        return (
            f"Unknown diagnostic '{diagnostic_name}'. "
            f"Available: {', '.join(sorted(DIAGNOSTIC_SCRIPTS))}"
        )
    script_file_name, runner = DIAGNOSTIC_SCRIPTS[diagnostic_name]
    script_path = SCRIPTS_DIR / script_file_name
    if not script_path.exists():
        return f"Script not found: {script_path}"
    cmd = [runner, str(script_path)]
    if args:
        cmd.extend(args.split())
    return _run_command(cmd)


# ─── Deployment Tools ────────────────────────────────────────────────

@mcp.tool()
def run_deploy_script(script_name: str, args: str = "") -> str:
    """Run a SLAM Docker deployment script.

    Available scripts:
    - deploy_docker_slam [--build|--start|--stop|--test|--all]

    Args:
        script_name: Name without .sh extension (e.g., "deploy_docker_slam")
        args: Space-separated arguments/flags to pass to the script
    """
    if script_name not in DEPLOY_SCRIPTS:
        return (
            f"Unknown deploy script '{script_name}'. "
            f"Available: {', '.join(sorted(DEPLOY_SCRIPTS))}"
        )
    script_file = SCRIPTS_DIR / DEPLOY_SCRIPTS[script_name]
    if not script_file.exists():
        return f"Script not found: {script_file}"
    cmd = ["bash", str(script_file)]
    if args:
        cmd.extend(args.split())
    return _run_command(cmd, timeout=600)


# ─── Profile Tools ───────────────────────────────────────────────────

@mcp.tool()
def search_profiles(
    platform: str = "",
    sensor: str = "",
    algorithm: str = "",
    autopilot: str = "",
) -> str:
    """Search hardware profiles for matching configurations.

    Searches both learned profiles (docs/learned/hardware_profiles.yaml)
    and curated starter profiles (docs/profiles/*.yaml).
    Returns matching profiles with fingerprints and validation status.

    Args:
        platform: e.g., "jetson_orin", "nuc12", "raspberry_pi"
        sensor: e.g., "ouster_os1_64", "livox_mid360", "realsense_d435i"
        algorithm: e.g., "fast_lio2", "lio_sam", "orb_slam3"
        autopilot: e.g., "ardupilot", "px4"
    """
    search_terms = {
        k: v.lower() for k, v in {
            "platform": platform,
            "sensor": sensor,
            "algorithm": algorithm,
            "autopilot": autopilot,
        }.items() if v
    }

    if not search_terms:
        return "Provide at least one search term (platform, sensor, algorithm, or autopilot)."

    matches = []

    # Search learned profiles
    data = _read_yaml(HARDWARE_PROFILES_FILE)
    for profile in data.get("profiles", []):
        fp = profile.get("fingerprint", "").lower()
        if all(term in fp for term in search_terms.values()):
            matches.append({
                "source": "learned",
                "fingerprint": profile.get("fingerprint"),
                "date": profile.get("date"),
                "validated": profile.get("validated", False),
                "integration_complete": profile.get("integration_complete", False),
            })

    # Search curated profiles
    if PROFILES_DIR.exists():
        for yaml_file in PROFILES_DIR.glob("*.yaml"):
            try:
                with open(yaml_file) as f:
                    profile = yaml.safe_load(f)
                if not isinstance(profile, dict):
                    continue
                fp = profile.get("fingerprint", "").lower()
                if all(term in fp for term in search_terms.values()):
                    matches.append({
                        "source": "curated",
                        "fingerprint": profile.get("fingerprint"),
                        "date": profile.get("date"),
                        "validated": profile.get("validated", False),
                        "integration_complete": profile.get("integration_complete", False),
                        "description": profile.get("description", ""),
                    })
            except Exception:
                continue

    if not matches:
        return f"No profiles match: {search_terms}. Start Phase 1 to create a new profile."

    lines = [f"Found {len(matches)} matching profile(s):\n"]
    for m in matches:
        status = []
        if m.get("integration_complete"):
            status.append("INTEGRATION COMPLETE")
        elif m.get("validated"):
            status.append("VALIDATED")
        else:
            status.append("unvalidated")
        lines.append(
            f"- [{m['source']}] {m['fingerprint']} "
            f"({', '.join(status)}) "
            f"date={m.get('date', 'unknown')}"
        )
        if m.get("description"):
            lines.append(f"  {m['description']}")
    return "\n".join(lines)


@mcp.tool()
def get_profile(fingerprint: str) -> str:
    """Get a complete hardware profile by fingerprint.

    Returns the full profile YAML including phase1_config and hardware details.

    Args:
        fingerprint: Hardware fingerprint, e.g., "jetson_orin-ouster_os1_64-fast_lio2-ardupilot-humble"
    """
    # Check learned profiles
    data = _read_yaml(HARDWARE_PROFILES_FILE)
    for profile in data.get("profiles", []):
        if profile.get("fingerprint") == fingerprint:
            return yaml.dump(profile, default_flow_style=False, sort_keys=False)

    # Check curated profiles
    if PROFILES_DIR.exists():
        for yaml_file in PROFILES_DIR.glob("*.yaml"):
            try:
                with open(yaml_file) as f:
                    profile = yaml.safe_load(f)
                if isinstance(profile, dict) and profile.get("fingerprint") == fingerprint:
                    return yaml.dump(profile, default_flow_style=False, sort_keys=False)
            except Exception:
                continue

    return f"No profile found with fingerprint: {fingerprint}"


@mcp.tool()
def get_known_good_config(fingerprint: str) -> str:
    """Retrieve known good configuration files for a hardware fingerprint.

    Returns the contents of all config files (SLAM config, URDF, launch file,
    ArduPilot params) saved after a successful Phase 5 integration.

    Args:
        fingerprint: Hardware fingerprint
    """
    config_dir = KNOWN_GOOD_CONFIGS_DIR / fingerprint
    if not config_dir.exists():
        return f"No known good configs for: {fingerprint}"

    output = [f"Known good configs for {fingerprint}:\n"]
    for config_file in sorted(config_dir.iterdir()):
        if config_file.is_file():
            try:
                content = config_file.read_text()
                output.append(f"--- {config_file.name} ---")
                output.append(content)
                output.append("")
            except Exception as e:
                output.append(f"--- {config_file.name} --- [Error reading: {e}]")
    return "\n".join(output) if len(output) > 1 else f"Config directory exists but is empty: {config_dir}"


# ─── Learning Tools ──────────────────────────────────────────────────

@mcp.tool()
def save_hardware_profile(
    fingerprint: str,
    hardware_yaml: str,
    phase1_config_yaml: str,
) -> str:
    """Save a new hardware profile after Phase 1 assessment.

    Creates a new entry in hardware_profiles.yaml with validated=false.
    Call update_profile_status() after Phase 2/5 to update flags.

    Args:
        fingerprint: Generated hardware fingerprint (format: platform-sensor-algorithm-autopilot-ros)
        hardware_yaml: YAML string of hardware details (platform, lidar, camera, imu, FC, etc.)
        phase1_config_yaml: Full YAML config output from Phase 1 assessment
    """
    try:
        hardware = yaml.safe_load(hardware_yaml)
        phase1_config = yaml.safe_load(phase1_config_yaml)
    except yaml.YAMLError as e:
        return f"Invalid YAML input: {e}"

    data = _read_yaml(HARDWARE_PROFILES_FILE)
    profiles = data.get("profiles", [])

    # Check for duplicate fingerprint
    for p in profiles:
        if p.get("fingerprint") == fingerprint:
            return f"Profile already exists for fingerprint: {fingerprint}. Use update_profile_status() to modify."

    next_id = max((p.get("id", 0) for p in profiles), default=0) + 1
    new_profile = {
        "id": next_id,
        "date": datetime.now().strftime("%Y-%m-%d"),
        "fingerprint": fingerprint,
        "hardware": hardware,
        "phase1_config": phase1_config,
        "validated": False,
        "integration_complete": False,
    }
    profiles.append(new_profile)
    data["profiles"] = profiles
    _write_yaml_atomic(HARDWARE_PROFILES_FILE, data)
    return f"Saved hardware profile #{next_id}: {fingerprint}"


@mcp.tool()
def update_profile_status(
    fingerprint: str,
    validated: Optional[bool] = None,
    integration_complete: Optional[bool] = None,
) -> str:
    """Update validation or integration status of a hardware profile.

    Call with validated=true after Phase 2 passes.
    Call with integration_complete=true after Phase 5 passes.

    Args:
        fingerprint: Hardware fingerprint to update
        validated: Set to true after Phase 2 compatibility validation passes
        integration_complete: Set to true after Phase 5 progressive testing passes
    """
    data = _read_yaml(HARDWARE_PROFILES_FILE)
    profiles = data.get("profiles", [])
    for p in profiles:
        if p.get("fingerprint") == fingerprint:
            if validated is not None:
                p["validated"] = validated
            if integration_complete is not None:
                p["integration_complete"] = integration_complete
            data["profiles"] = profiles
            _write_yaml_atomic(HARDWARE_PROFILES_FILE, data)
            return f"Updated profile {fingerprint}: validated={p.get('validated')}, integration_complete={p.get('integration_complete')}"
    return f"No profile found with fingerprint: {fingerprint}"


@mcp.tool()
def save_solution(
    phase: int,
    hardware_summary: str,
    symptom: str,
    root_cause: str,
    fix: str,
    files_changed: str,
    tags: str,
) -> str:
    """Save a troubleshooting solution to the solutions log.

    Called after Phase 6 resolves an issue. The solution is stored so future
    sessions can search for it before loading full troubleshooting guides.

    Args:
        phase: Phase number where the issue occurred (e.g., 5 or 6)
        hardware_summary: Relevant hardware info as YAML string
        symptom: What the user reported (e.g., "SLAM publishes odometry but drone flies wrong direction")
        root_cause: Why it happened (e.g., "LiDAR mounted 90 deg rotated, extrinsic yaw missing")
        fix: What resolved it (e.g., "Added -1.5708 rad yaw to extrinsicRot in config")
        files_changed: Comma-separated list of files modified to fix the issue
        tags: Comma-separated searchable keywords (e.g., "coordinate_frames,extrinsics,fast_lio2")
    """
    try:
        hardware = yaml.safe_load(hardware_summary) if hardware_summary else {}
    except yaml.YAMLError:
        hardware = {"raw": hardware_summary}

    data = _read_yaml(SOLUTIONS_LOG_FILE)
    solutions = data.get("solutions", [])
    next_id = max((s.get("id", 0) for s in solutions), default=0) + 1

    new_solution = {
        "id": next_id,
        "date": datetime.now().strftime("%Y-%m-%d"),
        "phase": phase,
        "hardware": hardware,
        "symptom": symptom,
        "root_cause": root_cause,
        "fix": fix,
        "files_changed": [f.strip() for f in files_changed.split(",") if f.strip()],
        "tags": [t.strip() for t in tags.split(",") if t.strip()],
    }
    solutions.append(new_solution)
    data["solutions"] = solutions
    _write_yaml_atomic(SOLUTIONS_LOG_FILE, data)
    return f"Saved solution #{next_id}: {symptom[:80]}"


@mcp.tool()
def search_solutions(tags: str = "", symptom: str = "") -> str:
    """Search the solutions log for previously solved problems.

    Check here BEFORE loading troubleshooting guide files.

    Args:
        tags: Comma-separated keywords to match against solution tags (e.g., "coordinate_frames,ekf")
        symptom: Text to search for in symptom descriptions
    """
    if not tags and not symptom:
        return "Provide at least one of: tags or symptom text."

    search_tags = {t.strip().lower() for t in tags.split(",") if t.strip()}
    symptom_lower = symptom.lower().strip()

    data = _read_yaml(SOLUTIONS_LOG_FILE)
    solutions = data.get("solutions", [])
    matches = []

    for s in solutions:
        sol_tags = {t.lower() for t in s.get("tags", [])}
        sol_symptom = s.get("symptom", "").lower()

        tag_match = bool(search_tags & sol_tags) if search_tags else False
        symptom_match = symptom_lower in sol_symptom if symptom_lower else False

        if tag_match or symptom_match:
            matches.append(s)

    if not matches:
        return "No matching solutions found. Load the appropriate troubleshooting guide."

    lines = [f"Found {len(matches)} matching solution(s):\n"]
    for s in matches:
        lines.append(f"--- Solution #{s.get('id')} ({s.get('date')}) ---")
        lines.append(f"Symptom: {s.get('symptom')}")
        lines.append(f"Root cause: {s.get('root_cause')}")
        lines.append(f"Fix: {s.get('fix')}")
        lines.append(f"Tags: {', '.join(s.get('tags', []))}")
        lines.append("")
    return "\n".join(lines)


@mcp.tool()
def save_known_good_config(fingerprint: str, configs_json: str) -> str:
    """Save validated configuration files after Phase 5 success.

    Saves the complete set of config files that produced a working integration.

    Args:
        fingerprint: Hardware fingerprint
        configs_json: JSON string of {"filename": "file_content"} for each config file
                      (e.g., slam_config.yaml, robot.urdf, slam_integration.launch, ardupilot_params.param)
    """
    try:
        configs = json.loads(configs_json)
    except json.JSONDecodeError as e:
        return f"Invalid JSON: {e}"

    if not isinstance(configs, dict) or not configs:
        return "configs_json must be a JSON object with {filename: content} entries."

    config_dir = KNOWN_GOOD_CONFIGS_DIR / fingerprint
    config_dir.mkdir(parents=True, exist_ok=True)

    saved = []
    for filename, content in configs.items():
        safe_name = Path(filename).name  # prevent path traversal
        file_path = config_dir / safe_name
        file_path.write_text(content)
        saved.append(safe_name)

    # Write metadata
    metadata = {
        "fingerprint": fingerprint,
        "date": datetime.now().strftime("%Y-%m-%d"),
        "files": saved,
    }
    (config_dir / "metadata.yaml").write_text(
        yaml.dump(metadata, default_flow_style=False)
    )
    return f"Saved {len(saved)} config files to {config_dir.relative_to(PROJECT_ROOT)}: {', '.join(saved)}"


# ─── Node Control Tools ──────────────────────────────────────────────

# Whitelisted actions for node control scripts
_ALLOWED_NODE_ACTIONS = {"start", "stop", "restart", "status", "logs"}


@mcp.tool()
def control_node(
    workspace: str,
    node: str,
    action: str,
    args: str = "",
) -> str:
    """Start, stop, restart, or check status of a SLAM node via its control script.

    Each SLAM workspace should have control scripts at <workspace>/scripts/<node>.sh
    that accept actions: start, stop, restart, status, logs [N].

    Examples:
        control_node("/home/dev/slam-gpu", "fastlio", "restart")
        control_node("/home/dev/slam-gpu", "fastlio", "status")
        control_node("/home/dev/slam-gpu", "fastlio", "logs", "50")
        control_node("/home/dev/slam-gpu", "foxglove", "start")

    Args:
        workspace: Absolute path to the SLAM workspace (e.g., "/home/dev/slam-gpu")
        node: Node name matching the script filename without .sh (e.g., "fastlio")
        action: One of: start, stop, restart, status, logs
        args: Additional arguments (e.g., line count for logs)
    """
    if action not in _ALLOWED_NODE_ACTIONS:
        return f"Invalid action '{action}'. Allowed: {', '.join(sorted(_ALLOWED_NODE_ACTIONS))}"

    # Sanitize node name (alphanumeric, underscore, hyphen only)
    if not all(c.isalnum() or c in "_-" for c in node):
        return f"Invalid node name '{node}'. Use only alphanumeric, underscore, or hyphen."

    script_path = Path(workspace) / "scripts" / f"{node}.sh"
    if not script_path.exists():
        # List available scripts
        scripts_dir = Path(workspace) / "scripts"
        if scripts_dir.exists():
            available = [
                f.stem for f in scripts_dir.glob("*.sh")
                if f.stem not in ("start_foxglove_bridge", "test_foxglove_bridge")
            ]
            return (
                f"Script not found: {script_path}\n"
                f"Available control scripts: {', '.join(sorted(available)) or 'none'}"
            )
        return f"Scripts directory not found: {scripts_dir}"

    cmd = ["bash", str(script_path), action]
    if args:
        cmd.extend(args.split())
    return _run_command(cmd, timeout=30)


# ─── Topic Inspection Tools ──────────────────────────────────────────

_ALLOWED_TOPIC_COMMANDS = {"list", "info", "hz", "echo", "bw", "type"}


@mcp.tool()
def inspect_topic(
    command: str,
    topic: str = "",
    container: str = "",
    ros_version: str = "ROS2",
    ros_distro: str = "humble",
    count: int = 1,
    duration: int = 5,
) -> str:
    """Inspect ROS topics (works both inside Docker containers and on host).

    Available commands:
    - list: Show all topics (no topic arg needed)
    - info: Show topic details, publishers, subscribers, and QoS (ROS 2 only)
    - hz: Measure publish rate (uses duration parameter)
    - echo: Print one or more messages (uses count parameter)
    - bw: Measure bandwidth (uses duration parameter)
    - type: Show message type

    Examples:
        # Docker container:
        inspect_topic("list", container="slam_gpu_system")
        inspect_topic("info", "/ouster/points", container="slam_gpu_system")
        inspect_topic("hz", "/ouster/points", container="slam_gpu_system", duration=10)
        inspect_topic("echo", "/ouster/imu", container="slam_gpu_system", count=1)

        # Host ROS:
        inspect_topic("list", ros_version="ROS2", ros_distro="humble")
        inspect_topic("hz", "/ouster/points", ros_version="ROS1")

    Args:
        command: One of: list, info, hz, echo, bw, type
        topic: Topic name (required for all except list). Must start with /.
        container: Docker container name (if empty, runs on host)
        ros_version: ROS1 or ROS2 (default: ROS2, ignored if container is set)
        ros_distro: ROS distro name (default: humble, ignored if container is set)
        count: Number of messages for echo (default 1, max 10)
        duration: Seconds to sample for hz and bw (default 5, max 30)
    """
    # Validate command
    if command not in _ALLOWED_TOPIC_COMMANDS:
        return f"Invalid command '{command}'. Allowed: {', '.join(sorted(_ALLOWED_TOPIC_COMMANDS))}"

    # Validate ROS version
    if ros_version not in ("ROS1", "ROS2"):
        return f"Invalid ros_version '{ros_version}'. Must be ROS1 or ROS2."

    # ROS 1 doesn't support 'info' command
    if command == "info" and ros_version == "ROS1":
        return "Command 'info' is only available for ROS 2. For ROS 1, use 'type' or 'hz' to get basic topic information."

    # Validate topic (required for all commands except list)
    if command != "list":
        if not topic:
            return f"Command '{command}' requires a topic argument."
        if not topic.startswith("/"):
            return f"Topic must start with '/'. Got: {topic}"

    # Sanitize container name if provided
    if container and not all(c.isalnum() or c in "_-" for c in container):
        return f"Invalid container name '{container}'. Use only alphanumeric, underscore, or hyphen."

    # Validate parameters
    if count < 1:
        count = 1
    elif count > 10:
        count = 10

    if duration < 1:
        duration = 1
    elif duration > 30:
        duration = 30

    # Build the topic command based on ROS version
    if ros_version == "ROS2":
        cmd_parts = ["ros2", "topic", command]

        if command == "list":
            cmd_parts.append("-t")  # Show topic types
        elif command == "info":
            cmd_parts.extend([topic, "--verbose"])
        elif command == "hz":
            cmd_parts.append(topic)
        elif command == "echo":
            if count == 1:
                cmd_parts.extend([topic, "--once"])
            else:
                # No --max-msgs in Humble; rely on timeout to stop
                cmd_parts.append(topic)
        elif command == "bw":
            cmd_parts.append(topic)
        elif command == "type":
            cmd_parts.append(topic)
    else:  # ROS1
        cmd_parts = ["rostopic", command]

        if command == "list":
            cmd_parts.append("-v")  # Show verbose output with types
        elif command == "hz":
            cmd_parts.append(topic)
        elif command == "echo":
            cmd_parts.extend([topic, "-n", str(count)])
        elif command == "bw":
            cmd_parts.append(topic)
        elif command == "type":
            cmd_parts.append(topic)

    topic_cmd = " ".join(cmd_parts)

    # Determine timeout based on command
    if command in ("hz", "bw"):
        cmd_timeout = duration + 5  # Add buffer
    elif command == "echo":
        cmd_timeout = duration + 5
    else:
        cmd_timeout = 30

    # Build execution command
    if container:
        # Docker execution with ROS 2 environment
        setup = (
            "source /opt/ros/humble/install/setup.bash && "
            "source /opt/slam_ws/install/setup.bash && "
            "export ROS_DOMAIN_ID=1"
        )
        if command in ("hz", "bw", "echo"):
            inner = f"{setup} && timeout {duration} {topic_cmd}"
        else:
            inner = f"{setup} && {topic_cmd}"

        cmd = ["docker", "exec", container, "bash", "-c", inner]
    else:
        # Host execution
        if ros_version == "ROS2":
            setup_script = f"/opt/ros/{ros_distro}/setup.bash"
        else:
            setup_script = f"/opt/ros/{ros_distro}/setup.bash"

        if command in ("hz", "bw", "echo"):
            inner = f"source {setup_script} && timeout {duration} {topic_cmd}"
        else:
            inner = f"source {setup_script} && {topic_cmd}"

        cmd = ["bash", "-c", inner]

    result = _run_command(cmd, timeout=cmd_timeout)

    # For hz/bw/echo, timeout and the Python traceback from SIGTERM are expected.
    # Strip the traceback and exit code noise, keep only the useful measurement lines.
    if command in ("hz", "bw", "echo"):
        lines = result.split("\n")
        useful_lines = []
        skip = False
        for line in lines:
            if line.startswith("Traceback (most recent call last):"):
                skip = True
            elif line.startswith("[Exit code:"):
                continue
            elif not skip:
                useful_lines.append(line)
        if useful_lines:
            return "\n".join(useful_lines).strip()

    return result


# ─── Git Learning Loop ───────────────────────────────────────────────

@mcp.tool()
def commit_learning(message: str = "") -> str:
    """Commit and push learned data (profiles, solutions, configs) to git.

    Stages ONLY files under docs/learned/ and docs/profiles/.
    Uses the commit prefix 'learn:' for easy identification.

    Args:
        message: Description of what was learned (e.g., "validated integration: jetson_orin-ouster-fast_lio2")
    """
    # Stage learned files
    add_result = _run_command(
        ["git", "add", "docs/learned/", "docs/profiles/"],
        timeout=30,
    )
    if "[Error" in add_result:
        return f"Git add failed: {add_result}"

    # Check if there's anything to commit
    status = _run_command(["git", "diff", "--cached", "--stat"], timeout=10)
    if not status or status == "[No output]":
        return "No changes to commit in docs/learned/ or docs/profiles/."

    # Commit
    commit_msg = f"learn: {message}" if message else "learn: update learned data"
    commit_result = _run_command(
        ["git", "commit", "-m", commit_msg],
        timeout=30,
    )
    if "[Error" in commit_result and "nothing to commit" not in commit_result:
        return f"Git commit failed: {commit_result}"

    # Push
    branch_result = _run_command(
        ["git", "rev-parse", "--abbrev-ref", "HEAD"],
        timeout=10,
    )
    branch = branch_result.strip() if "[Error" not in branch_result else "main"
    push_result = _run_command(
        ["git", "push", "origin", branch],
        timeout=60,
    )

    return f"Committed and pushed:\n{commit_result}\n\nPush result:\n{push_result}"


@mcp.tool()
def pull_latest_learning() -> str:
    """Pull latest learned data from git remote.

    Call at the START of every session to sync profiles and solutions
    from other sessions/users.
    """
    result = _run_command(
        ["git", "pull", "--rebase", "origin"],
        timeout=60,
    )
    # Report what's available
    profile_data = _read_yaml(HARDWARE_PROFILES_FILE)
    num_profiles = len(profile_data.get("profiles", []))
    solution_data = _read_yaml(SOLUTIONS_LOG_FILE)
    num_solutions = len(solution_data.get("solutions", []))

    num_configs = 0
    if KNOWN_GOOD_CONFIGS_DIR.exists():
        num_configs = sum(1 for d in KNOWN_GOOD_CONFIGS_DIR.iterdir() if d.is_dir())

    return (
        f"Git pull: {result}\n\n"
        f"Learning database: {num_profiles} hardware profile(s), "
        f"{num_solutions} solution(s), {num_configs} known good config(s)"
    )


if __name__ == "__main__":
    mcp.run()
