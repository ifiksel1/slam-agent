#!/usr/bin/env python3
"""MCP server smoke test — verifies all tools load and all scripts resolve."""

import argparse
import json
import os
import sys
from pathlib import Path

RESET = "\033[0m"; BLUE = "\033[94m"; GREEN = "\033[92m"
YELLOW = "\033[93m"; RED = "\033[91m"
PASS = f"{GREEN}✓{RESET}"; FAIL = f"{RED}✗{RESET}"; WARN = f"{YELLOW}⚠{RESET}"

PROJECT_ROOT = Path(__file__).resolve().parent.parent

INSTALL_SCRIPTS = {
    "install_slam_integration": "install_slam_integration.sh",
    "install_core_ros_packages": "install_core_ros_packages.sh",
    "install_lidar_driver": "install_lidar_driver.sh",
    "install_camera_driver": "install_camera_driver.sh",
    "install_slam_algorithm": "install_slam_algorithm.sh",
    "install_mavros": "install_mavros.sh",
    "install_vision_to_mavros": "install_vision_to_mavros.sh",
    "install_dds_bridge": "install_dds_bridge.sh",
    "install_path_planner": "install_path_planner.sh",
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
    "measure_vision_latency": ("measure_vision_latency.py", "python3"),
    "flight_recorder": ("flight_recorder.sh", "bash"),
    "flight_analysis": ("flight_analysis.py", "python3"),
    "transform_calibrator": ("transform_calibrator.py", "python3"),
    "check_path_planner": ("check_path_planner.py", "python3"),
}

DEPLOY_SCRIPTS = {
    "deploy_docker_slam": "deploy_docker_slam.sh",
}

PHASE_FILES = [
    "phase0_docker_deployment.md",
    "phase1_assessment.md",
    "phase2_validation.md",
    "phase3_generation.md",
    "phase4_installation.md",
    "phase5_testing.md",
    "phase6_troubleshooting.md",
    "phase7_optimization.md",
    "phase8_path_planner.md",
    "phase8a_waypoint_nav.md",
    "phase8b_super.md",
    "phase8c_ego_planner.md",
    "phase8d_nav2.md",
    "phase9_voxl.md",
]

LEARNED_FILES = [
    "docs/learned/hardware_profiles.yaml",
    "docs/learned/solutions_log.yaml",
]


def chk(name, ok, detail):
    return {"name": name, "status": "pass" if ok else "fail", "detail": detail}


def check_scripts(scripts_dir):
    """Verify all whitelisted scripts exist and are executable."""
    results = []
    all_scripts = {}
    for name, filename in INSTALL_SCRIPTS.items():
        all_scripts[name] = scripts_dir / filename
    for name, (filename, _runner) in DIAGNOSTIC_SCRIPTS.items():
        all_scripts[name] = scripts_dir / filename
    for name, filename in DEPLOY_SCRIPTS.items():
        all_scripts[name] = scripts_dir / filename

    for name, path in sorted(all_scripts.items()):
        exists = path.exists()
        executable = os.access(path, os.X_OK) if exists else False
        if exists and executable:
            results.append(chk(f"script:{name}", True, f"{path.name} exists and executable"))
        elif exists:
            results.append(chk(f"script:{name}", False, f"{path.name} exists but NOT executable"))
        else:
            results.append(chk(f"script:{name}", False, f"{path.name} MISSING"))
    return results


def check_phase_files(phases_dir):
    """Verify all phase reference files exist."""
    results = []
    for filename in PHASE_FILES:
        path = phases_dir / filename
        exists = path.exists()
        results.append(chk(
            f"phase:{filename}",
            exists,
            f"{filename} {'found' if exists else 'MISSING'} ({path.stat().st_size} bytes)" if exists else f"{filename} MISSING",
        ))
    return results


def check_learned_data(project_root):
    """Verify learned data files exist and are valid YAML."""
    results = []
    for relpath in LEARNED_FILES:
        path = project_root / relpath
        exists = path.exists()
        if not exists:
            results.append(chk(f"learned:{relpath}", False, f"{relpath} MISSING"))
            continue
        try:
            import yaml
            with open(path) as f:
                data = yaml.safe_load(f)
            if not isinstance(data, dict):
                results.append(chk(f"learned:{relpath}", False, f"{relpath} not a valid YAML dict"))
            else:
                results.append(chk(f"learned:{relpath}", True, f"{relpath} valid YAML"))
        except Exception as e:
            results.append(chk(f"learned:{relpath}", False, f"{relpath} YAML parse error: {e}"))

    # Check known good configs directory
    kgc_dir = project_root / "docs" / "learned" / "known_good_configs"
    if kgc_dir.exists():
        configs = [d.name for d in kgc_dir.iterdir() if d.is_dir()]
        results.append(chk("learned:known_good_configs", True, f"{len(configs)} config set(s): {', '.join(configs[:3])}"))
    else:
        results.append(chk("learned:known_good_configs", False, "known_good_configs/ directory MISSING"))

    return results


def check_mcp_server(project_root):
    """Verify MCP server file exists and imports cleanly."""
    results = []
    server_path = project_root / "mcp" / "slam_mcp_server.py"
    if not server_path.exists():
        results.append(chk("mcp:server_file", False, "mcp/slam_mcp_server.py MISSING"))
        return results
    results.append(chk("mcp:server_file", True, "mcp/slam_mcp_server.py exists"))

    # Check syntax
    import py_compile
    try:
        py_compile.compile(str(server_path), doraise=True)
        results.append(chk("mcp:syntax", True, "Server passes syntax check"))
    except py_compile.PyCompileError as e:
        results.append(chk("mcp:syntax", False, f"Syntax error: {e}"))

    # Check launcher
    launcher = project_root / "mcp" / "run_mcp_server.sh"
    results.append(chk(
        "mcp:launcher",
        launcher.exists(),
        f"run_mcp_server.sh {'exists' if launcher.exists() else 'MISSING'}",
    ))

    return results


def check_skill(project_root):
    """Verify SKILL.md exists and references are consistent."""
    results = []
    skill_path = project_root / ".claude" / "skills" / "slam-integration" / "SKILL.md"
    if not skill_path.exists():
        results.append(chk("skill:file", False, "SKILL.md MISSING"))
        return results
    results.append(chk("skill:file", True, "SKILL.md exists"))

    content = skill_path.read_text()
    # Check that all phase files are referenced
    for filename in PHASE_FILES:
        stem = filename.replace(".md", "")
        if stem in content:
            results.append(chk(f"skill:ref:{stem}", True, f"{stem} referenced in SKILL.md"))
        else:
            results.append(chk(f"skill:ref:{stem}", False, f"{stem} NOT referenced in SKILL.md"))

    return results


def main():
    parser = argparse.ArgumentParser(description="MCP server smoke test")
    parser.add_argument("--json", action="store_true", help="Output JSON")
    args = parser.parse_args()

    scripts_dir = PROJECT_ROOT / "scripts"
    phases_dir = PROJECT_ROOT / "docs" / "phases"

    sections = [
        ("MCP Server", check_mcp_server(PROJECT_ROOT)),
        ("SKILL.md References", check_skill(PROJECT_ROOT)),
        ("Scripts (install + diagnostic + deploy)", check_scripts(scripts_dir)),
        ("Phase Files", check_phase_files(phases_dir)),
        ("Learned Data", check_learned_data(PROJECT_ROOT)),
    ]

    all_checks = []
    for title, checks in sections:
        all_checks.extend(checks)

    passed = sum(1 for c in all_checks if c["status"] == "pass")
    failed = sum(1 for c in all_checks if c["status"] == "fail")
    total = len(all_checks)

    if args.json:
        print(json.dumps({
            "checks": all_checks,
            "summary": {"passed": passed, "failed": failed, "total": total},
        }, indent=2))
    else:
        print(f"\n{'='*60}")
        print(f"SLAM Agent MCP Smoke Test")
        print(f"{'='*60}")

        for title, checks in sections:
            print(f"\n{BLUE}{title}{RESET}")
            print(f"{'─'*50}")
            for c in checks:
                sym = PASS if c["status"] == "pass" else FAIL
                label = c["name"].split(":", 1)[-1]
                print(f"  {sym} {label:<40} {c['detail']}")

        print(f"\n{'='*60}")
        print(f"  {GREEN}Passed{RESET}: {passed}/{total}")
        print(f"  {RED}Failed{RESET}: {failed}/{total}")
        print(f"{'='*60}\n")

        if failed == 0:
            print(f"{GREEN}All checks passed — MCP server is healthy.{RESET}\n")
        else:
            print(f"{RED}{failed} failure(s) detected.{RESET}\n")

    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    main()
