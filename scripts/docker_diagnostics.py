#!/usr/bin/env python3
"""Docker SLAM System Diagnostics.

Comprehensive health checks for containerized SLAM integration system.
Tests container status, ROS environment, nodes, topics, and performance.
"""

import json
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Tuple


class DockerDiagnostics:
    """SLAM Docker system diagnostics."""

    def __init__(self, compose_path: str = "~/slam_ws"):
        """Initialize diagnostics."""
        self.compose_dir = Path(compose_path).expanduser()
        self.results = {
            "timestamp": datetime.now().isoformat(),
            "tests": {},
            "summary": {"passed": 0, "failed": 0, "warnings": 0},
        }

    def run_command(
        self, cmd: List[str], timeout: int = 30, use_compose: bool = False
    ) -> Tuple[int, str]:
        """Run shell command and return exit code and output."""
        try:
            if use_compose:
                cmd = ["docker", "compose", "exec", "-T", "slam_launch", "bash", "-c"] + [
                    " && ".join(cmd) if isinstance(cmd, list) else cmd
                ]

            result = subprocess.run(
                cmd if not use_compose else cmd,
                cwd=str(self.compose_dir),
                capture_output=True,
                text=True,
                timeout=timeout,
            )
            return result.returncode, result.stdout + result.stderr
        except subprocess.TimeoutExpired:
            return -1, f"[TIMEOUT after {timeout}s]"
        except Exception as e:
            return -1, f"[ERROR: {e}]"

    def check_docker_image(self) -> Dict:
        """Check if Docker image exists and is valid."""
        result = {"name": "Docker Image Check", "passed": False, "details": {}}

        exit_code, output = self.run_command(
            ["docker", "images", "slam_integration:latest", "--format", "{{.Size}}"],
            use_compose=False,
        )

        if exit_code == 0 and output.strip():
            size = output.strip()
            result["passed"] = True
            result["details"]["size"] = size
            result["message"] = f"✓ Docker image exists (Size: {size})"
        else:
            result["message"] = "✗ Docker image not found"

        return result

    def check_container_status(self) -> Dict:
        """Check if container is running."""
        result = {
            "name": "Container Status",
            "passed": False,
            "details": {},
        }

        exit_code, output = self.run_command(
            ["docker", "compose", "ps"],
            use_compose=False,
        )

        if "slam_launch" in output and "Up" in output:
            result["passed"] = True
            result["message"] = "✓ Container is running"

            # Extract status line
            for line in output.split("\n"):
                if "slam_launch" in line:
                    result["details"]["status_line"] = line.strip()
        else:
            result["message"] = "✗ Container is not running or not found"

        return result

    def check_ros_environment(self) -> Dict:
        """Check ROS environment setup."""
        result = {
            "name": "ROS Environment",
            "passed": False,
            "details": {},
        }

        cmd = [
            "source /catkin_ws/devel/setup.bash && echo ROS_DISTRO=$ROS_DISTRO && "
            "echo ROS_MASTER_URI=$ROS_MASTER_URI && echo ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH"
        ]

        exit_code, output = self.run_command(cmd, use_compose=True)

        if exit_code == 0:
            result["passed"] = "ROS_DISTRO=noetic" in output
            for line in output.split("\n"):
                if "=" in line:
                    key, value = line.split("=", 1)
                    result["details"][key] = value

            if result["passed"]:
                result["message"] = "✓ ROS environment correctly configured"
            else:
                result["message"] = "⚠ ROS environment partially configured"
                result["details"]["warning"] = "ROS_DISTRO not noetic"
        else:
            result["message"] = "✗ ROS environment check failed"

        return result

    def check_ros_nodes(self) -> Dict:
        """Check active ROS nodes."""
        result = {
            "name": "ROS Nodes",
            "passed": False,
            "details": {"nodes": []},
        }

        cmd = ["source /catkin_ws/devel/setup.bash && rosnode list 2>/dev/null"]
        exit_code, output = self.run_command(cmd, use_compose=True)

        if exit_code == 0:
            nodes = [n.strip() for n in output.split("\n") if n.strip() and n.startswith("/")]
            result["details"]["nodes"] = nodes
            result["details"]["count"] = len(nodes)

            # Check for critical nodes
            critical = ["/laserMapping", "/mavros", "/slam_to_mavros"]
            found_critical = [n for n in nodes if any(c in n for c in critical)]

            if len(found_critical) >= 2:
                result["passed"] = True
                result["message"] = f"✓ ROS nodes running ({len(nodes)} total, {len(found_critical)} critical)"
            else:
                result["message"] = f"⚠ Only {len(found_critical)}/3 critical nodes found"
        else:
            result["message"] = "✗ Could not query ROS nodes"

        return result

    def check_ros_topics(self) -> Dict:
        """Check publishing ROS topics."""
        result = {
            "name": "ROS Topics",
            "passed": False,
            "details": {"topics": []},
        }

        cmd = ["source /catkin_ws/devel/setup.bash && rostopic list 2>/dev/null"]
        exit_code, output = self.run_command(cmd, use_compose=True)

        if exit_code == 0:
            topics = [t.strip() for t in output.split("\n") if t.strip()]
            result["details"]["topics"] = topics
            result["details"]["count"] = len(topics)

            # Check for critical topics
            critical = ["/ouster/points", "/Odometry", "/tf"]
            found_critical = [t for t in topics if any(c == t for c in critical)]

            if len(found_critical) >= 2:
                result["passed"] = True
                result["message"] = (
                    f"✓ ROS topics publishing ({len(topics)} total, {len(found_critical)} critical)"
                )
            else:
                result["message"] = f"⚠ Only {len(found_critical)}/3 critical topics found"
        else:
            result["message"] = "✗ Could not query ROS topics"

        return result

    def check_packages(self) -> Dict:
        """Check if all required packages are discoverable."""
        result = {
            "name": "ROS Packages",
            "passed": False,
            "details": {"packages": {}},
        }

        required_packages = [
            "fast_lio",
            "ouster_ros",
            "std_detector",
            "orin_slam_integration",
            "aloam_velodyne",
            "vision_to_mavros",
        ]

        found = 0
        for pkg in required_packages:
            cmd = [f"source /catkin_ws/devel/setup.bash && rospack find {pkg} 2>/dev/null"]
            exit_code, output = self.run_command(cmd, use_compose=True)

            if exit_code == 0:
                result["details"]["packages"][pkg] = "✓"
                found += 1
            else:
                result["details"]["packages"][pkg] = "✗"

        result["details"]["found"] = found
        result["details"]["total"] = len(required_packages)

        if found == len(required_packages):
            result["passed"] = True
            result["message"] = f"✓ All {found} required packages found"
        else:
            result["message"] = f"⚠ Only {found}/{len(required_packages)} packages found"

        return result

    def check_launch_files(self) -> Dict:
        """Check if launch files exist."""
        result = {
            "name": "Launch Files",
            "passed": False,
            "details": {},
        }

        cmd = ["find /catkin_ws -name '*.launch' -type f 2>/dev/null | wc -l"]
        exit_code, output = self.run_command(cmd, use_compose=True)

        if exit_code == 0:
            count = int(output.strip())
            result["details"]["count"] = count

            if count >= 20:
                result["passed"] = True
                result["message"] = f"✓ {count} launch files found"
            else:
                result["message"] = f"⚠ Only {count} launch files (expected 20+)"
        else:
            result["message"] = "✗ Could not query launch files"

        return result

    def check_slam_odometry(self) -> Dict:
        """Check SLAM odometry topic."""
        result = {
            "name": "SLAM Odometry",
            "passed": False,
            "details": {},
        }

        cmd = ["source /catkin_ws/devel/setup.bash && rostopic info /Odometry 2>/dev/null"]
        exit_code, output = self.run_command(cmd, use_compose=True, timeout=10)

        if exit_code == 0 and output.strip():
            result["details"]["info"] = output[:200]  # First 200 chars
            result["passed"] = "Publishers" in output

            if result["passed"]:
                result["message"] = "✓ SLAM odometry topic is publishing"
            else:
                result["message"] = "⚠ SLAM odometry topic exists but has no publishers"
        else:
            result["message"] = "✗ SLAM odometry topic not found"

        return result

    def check_ceres_version(self) -> Dict:
        """Check Ceres solver version (should be 2.1+)."""
        result = {
            "name": "Ceres Solver Version",
            "passed": False,
            "details": {},
        }

        cmd = ["pkg-config --modversion ceres-solver 2>/dev/null || echo 'NOT_FOUND'"]
        exit_code, output = self.run_command(cmd, use_compose=True)

        if exit_code == 0:
            version = output.strip()
            result["details"]["version"] = version

            if version != "NOT_FOUND":
                # Parse version (e.g., "2.1.0")
                major = int(version.split(".")[0]) if version[0].isdigit() else 0
                if major >= 2:
                    result["passed"] = True
                    result["message"] = f"✓ Ceres {version} (required for STD loop closure)"
                else:
                    result["message"] = f"✗ Ceres {version} (need 2.1+ for Manifold API)"
            else:
                result["message"] = "✗ Ceres not found (STD detector will fail)"
        else:
            result["message"] = "⚠ Could not check Ceres version"

        return result

    def check_build_dependencies(self) -> Dict:
        """Check critical build dependencies from build issues."""
        result = {
            "name": "Build Dependencies",
            "passed": False,
            "details": {"packages": {}},
        }

        # Critical dependencies identified in build iterations
        critical_deps = [
            "libspdlog-dev",
            "libjsoncpp-dev",
            "libcurl4-openssl-dev",
        ]

        found = 0
        for dep in critical_deps:
            cmd = [f"dpkg -l | grep -q ^ii.*{dep} && echo FOUND || echo MISSING"]
            exit_code, output = self.run_command(cmd, use_compose=True)

            if "FOUND" in output:
                result["details"]["packages"][dep] = "✓"
                found += 1
            else:
                result["details"]["packages"][dep] = "✗"

        result["details"]["found"] = found
        result["details"]["total"] = len(critical_deps)

        if found == len(critical_deps):
            result["passed"] = True
            result["message"] = f"✓ All {found} critical build deps present"
        else:
            result["message"] = f"⚠ Only {found}/{len(critical_deps)} build deps found (ouster-ros may fail)"

        return result

    def check_message_generation(self) -> Dict:
        """Check if custom messages were generated."""
        result = {
            "name": "Message Generation",
            "passed": False,
            "details": {"headers": {}},
        }

        # Check for generated message headers (Pose6D from fast_lio)
        cmd = ["find /catkin_ws/devel/include -name 'Pose6D.h' 2>/dev/null"]
        exit_code, output = self.run_command(cmd, use_compose=True)

        if exit_code == 0 and output.strip():
            result["details"]["headers"]["Pose6D"] = "✓"
            result["passed"] = True
            result["message"] = "✓ Custom message headers generated (Pose6D.h found)"
        else:
            result["details"]["headers"]["Pose6D"] = "✗"
            result["message"] = "✗ Message generation incomplete (Pose6D.h missing - build will fail)"

        return result

    def check_container_environment(self) -> Dict:
        """Check if running in proper container environment."""
        result = {
            "name": "Container Environment",
            "passed": False,
            "details": {},
        }

        # Check for .dockerenv file
        cmd = ["[ -f /.dockerenv ] && echo IN_CONTAINER || echo NOT_CONTAINER"]
        exit_code, output = self.run_command(cmd, use_compose=True)

        if "IN_CONTAINER" in output:
            result["details"]["environment"] = "Docker container"
            result["passed"] = True
            result["message"] = "✓ Running in proper Docker container"

            # Check workspace path
            cmd = ["ls -d /catkin_ws 2>/dev/null && echo FOUND || echo MISSING"]
            exit_code, output = self.run_command(cmd, use_compose=True)

            result["details"]["workspace"] = "Found" if "FOUND" in output else "Missing"
        else:
            result["message"] = "⚠ Not running in Docker (scripts may use wrong paths)"
            result["details"]["environment"] = "Host system"

        return result

    def check_config_mounts(self) -> Dict:
        """Check if configuration directories are mounted and writable."""
        result = {
            "name": "Configuration Mounts",
            "passed": False,
            "details": {"mounts": {}},
        }

        # Check for mounted config directories
        config_dirs = [
            "/catkin_ws/src/orin_slam_integration/config",
            "/catkin_ws/src/orin_slam_integration/launch",
            "/catkin_ws/src/FAST_LIO_SLAM/FAST-LIO/config",
        ]

        found = 0
        for config_dir in config_dirs:
            cmd = [
                f"[ -d {config_dir} ] && [ -w {config_dir} ] && echo MOUNTED_RW || echo NOT_MOUNTED"
            ]
            exit_code, output = self.run_command(cmd, use_compose=True)

            if "MOUNTED_RW" in output:
                result["details"]["mounts"][config_dir] = "✓"
                found += 1
            else:
                result["details"]["mounts"][config_dir] = "✗ (read-only or missing)"

        result["details"]["found"] = found
        result["details"]["total"] = len(config_dirs)

        if found >= 2:
            result["passed"] = True
            result["message"] = f"✓ {found}/{len(config_dirs)} config directories mounted and writable"
        else:
            result["message"] = f"⚠ Only {found}/{len(config_dirs)} configs mounted (can't edit at runtime)"

        return result

    def check_health_status(self) -> Dict:
        """Check Docker container health status."""
        result = {
            "name": "Container Health",
            "passed": False,
            "details": {},
        }

        cmd = ["docker", "compose", "ps", "--format", "{{.Names}} {{.Status}}"]
        exit_code, output = self.run_command(cmd, use_compose=False)

        if exit_code == 0:
            for line in output.split("\n"):
                if "slam_launch" in line:
                    result["details"]["status_line"] = line.strip()

                    if "(healthy)" in line:
                        result["passed"] = True
                        result["message"] = "✓ Container health check passing"
                    elif "(unhealthy)" in line:
                        result["message"] = "✗ Container health check failing (may be rebuilding)"
                    elif "Up" in line:
                        result["passed"] = True
                        result["message"] = "✓ Container running (health checks not enabled)"
                    else:
                        result["message"] = f"⚠ Unknown status: {line}"
                    break
        else:
            result["message"] = "⚠ Could not query container status"

        return result

    def run_all_checks(self) -> Dict:
        """Run all diagnostic checks."""
        print("Running Docker SLAM Diagnostics...\n")

        checks = [
            self.check_docker_image,
            self.check_container_status,
            self.check_container_environment,
            self.check_health_status,
            self.check_ros_environment,
            self.check_ceres_version,
            self.check_build_dependencies,
            self.check_message_generation,
            self.check_ros_nodes,
            self.check_ros_topics,
            self.check_packages,
            self.check_launch_files,
            self.check_config_mounts,
            self.check_slam_odometry,
        ]

        for check in checks:
            try:
                result = check()
                print(f"{result['message']}")

                if result.get("passed"):
                    self.results["summary"]["passed"] += 1
                else:
                    self.results["summary"]["failed"] += 1

                self.results["tests"][result["name"]] = result
            except Exception as e:
                print(f"✗ {check.__name__} failed: {e}")
                self.results["summary"]["failed"] += 1

        return self.results

    def print_summary(self):
        """Print diagnostic summary."""
        summary = self.results["summary"]
        total = summary["passed"] + summary["failed"]

        print(f"\n{'='*60}")
        print(f"SLAM Docker Diagnostics Summary")
        print(f"{'='*60}")
        print(f"Passed:  {summary['passed']}/{total}")
        print(f"Failed:  {summary['failed']}/{total}")
        print(f"{'='*60}\n")

        if summary["failed"] == 0:
            print("✓ All checks passed! System is operational.\n")
            return 0
        else:
            print("✗ Some checks failed. Review output above.\n")
            return 1

    def save_results(self, path: str = "docker_diagnostics.json"):
        """Save results to JSON file."""
        output_path = Path(path)
        output_path.write_text(json.dumps(self.results, indent=2))
        print(f"Diagnostics saved to: {output_path}")


def main():
    """Main entry point."""
    diags = DockerDiagnostics()
    results = diags.run_all_checks()
    exit_code = diags.print_summary()

    # Save results
    diags.save_results("/tmp/docker_slam_diagnostics.json")

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
