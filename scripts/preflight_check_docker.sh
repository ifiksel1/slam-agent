#!/bin/bash
# Pre-Flight System Verification for SLAM + Ouster (Docker Version)
# Adapted for running inside Docker container

set -e

echo ""
echo "╔════════════════════════════════════════════════════════════════════╗"
echo "║           SLAM SYSTEM PRE-FLIGHT VERIFICATION CHECK              ║"
echo "║                      (Docker Container)                          ║"
echo "╚════════════════════════════════════════════════════════════════════╝"
echo ""

PASSED=0
FAILED=0
WARNINGS=0

check_item() {
  local name="$1"
  local result="$2"

  if [ "$result" = "PASS" ]; then
    echo "  ✓ $name"
    ((PASSED++))
  elif [ "$result" = "FAIL" ]; then
    echo "  ✗ $name"
    ((FAILED++))
  else
    echo "  ⚠ $name"
    ((WARNINGS++))
  fi
}

# 1. Network Connectivity
echo "[1] NETWORK CONNECTIVITY"
OUSTER_IP=${OUSTER_IP:-169.254.56.220}
if ping -c 1 -W 2 $OUSTER_IP &>/dev/null; then
  check_item "Ouster sensor reachable ($OUSTER_IP)" "PASS"
else
  check_item "Ouster sensor reachable ($OUSTER_IP)" "FAIL"
fi

# 2. ROS Infrastructure
echo ""
echo "[2] ROS INFRASTRUCTURE"
if rostopic list &>/dev/null; then
  check_item "ROS Master running" "PASS"
else
  check_item "ROS Master running" "FAIL"
  echo ""
  echo "Starting ROS Master..."
  roscore &
  sleep 3

  if rostopic list &>/dev/null; then
    check_item "ROS Master started successfully" "PASS"
    ((PASSED++))
  else
    check_item "ROS Master started successfully" "FAIL"
    ((FAILED++))
  fi
fi

# 3. Package Verification
echo ""
echo "[3] PACKAGE INSTALLATION"
if [ -d /root/slam_ws/devel/lib/fast_lio ]; then
  check_item "FAST-LIO package built" "PASS"
else
  check_item "FAST-LIO package built" "FAIL"
fi

if [ -d /root/slam_ws/devel/lib/ouster_ros ]; then
  check_item "Ouster ROS driver built" "PASS"
else
  check_item "Ouster ROS driver built" "FAIL"
fi

# 4. Configuration Files
echo ""
echo "[4] CONFIGURATION VERIFICATION"
CONFIG_FILE="/root/slam_ws/src/FAST_LIO_SLAM/FAST-LIO/config/ouster64.yaml"
if [ -f "$CONFIG_FILE" ]; then
  check_item "SLAM config file exists" "PASS"

  if grep -q 'lid_topic.*"/ouster/points"' "$CONFIG_FILE"; then
    check_item "SLAM config points to /ouster/points" "PASS"
  else
    check_item "SLAM config points to /ouster/points" "FAIL"
  fi

  if grep -q 'imu_topic.*"/ouster/imu"' "$CONFIG_FILE"; then
    check_item "SLAM config points to /ouster/imu" "PASS"
  else
    check_item "SLAM config points to /ouster/imu" "FAIL"
  fi
else
  check_item "SLAM config file exists" "FAIL"
fi

# 5. Launch File Check
echo ""
echo "[5] LAUNCH FILES"
if [ -f /root/slam_ws/src/FAST_LIO_SLAM/FAST-LIO/launch/mapping_ouster64_docker.launch ]; then
  check_item "Docker launch file available" "PASS"
elif [ -f /root/slam_ws/src/FAST_LIO_SLAM/FAST-LIO/launch/mapping_ouster64.launch ]; then
  check_item "Standard launch file available" "PASS"
else
  check_item "Launch file available" "FAIL"
fi

# Summary
echo ""
echo "╔════════════════════════════════════════════════════════════════════╗"
echo "║                        SUMMARY RESULTS                             ║"
echo "╚════════════════════════════════════════════════════════════════════╝"
echo ""
echo "  Passed:   $PASSED"
echo "  Warnings: $WARNINGS"
echo "  Failed:   $FAILED"
echo ""

if [ $FAILED -eq 0 ]; then
  echo "✅ ALL CRITICAL CHECKS PASSED - CONTAINER READY"
  echo ""
  exit 0
else
  echo "❌ CRITICAL FAILURES DETECTED"
  echo ""
  echo "Container may need rebuilding or network configuration adjustment"
  echo ""
  exit 1
fi
