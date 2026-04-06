#!/bin/bash
# Pre-Flight System Verification for SLAM + Ouster
# Run this before starting autonomous missions to catch configuration issues early

set -e

echo ""
echo "╔════════════════════════════════════════════════════════════════════╗"
echo "║           SLAM SYSTEM PRE-FLIGHT VERIFICATION CHECK              ║"
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
if ping -c 1 169.254.56.220 &>/dev/null; then
  check_item "Ouster sensor reachable (169.254.56.220)" "PASS"
else
  check_item "Ouster sensor reachable (169.254.56.220)" "FAIL"
fi

if ip link show eth0 | grep -q "LOWER_UP"; then
  check_item "eth0 physical link UP" "PASS"
else
  check_item "eth0 physical link UP" "FAIL"
fi

# 2. ROS Infrastructure
echo ""
echo "[2] ROS INFRASTRUCTURE"
if rostopic list &>/dev/null; then
  check_item "ROS Master running" "PASS"
else
  check_item "ROS Master running" "FAIL"
  exit 1
fi

# 3. Driver Status
echo ""
echo "[3] OUSTER DRIVER STATUS"
if rostopic list | grep -q "^/ouster/points"; then
  check_item "Ouster driver publishing /ouster/points" "PASS"

  # Check if actually has data
  if rostopic info /ouster/points 2>/dev/null | grep -q "Publishers:"; then
    check_item "  → Data flowing (has publishers)" "PASS"
  else
    check_item "  → Data flowing (has publishers)" "FAIL"
  fi
else
  check_item "Ouster driver publishing /ouster/points" "FAIL"
fi

if rostopic list | grep -q "^/ouster/imu"; then
  check_item "Ouster IMU topic available" "PASS"
else
  check_item "Ouster IMU topic available" "FAIL"
fi

# 4. Configuration Files
echo ""
echo "[4] CONFIGURATION VERIFICATION"
CONFIG_FILE="/home/dev/slam_ws/src/FAST_LIO_SLAM/FAST-LIO/config/ouster64.yaml"
if [ -f "$CONFIG_FILE" ]; then
  if grep -q 'lid_topic.*"/ouster/points"' "$CONFIG_FILE"; then
    check_item "SLAM config points to /ouster/points" "PASS"
  else
    check_item "SLAM config points to /ouster/points" "FAIL"
    echo "    FIX: Update $CONFIG_FILE line 2 to: lid_topic:  \"/ouster/points\""
  fi

  if grep -q 'imu_topic.*"/ouster/imu"' "$CONFIG_FILE"; then
    check_item "SLAM config points to /ouster/imu" "PASS"
  else
    check_item "SLAM config points to /ouster/imu" "FAIL"
    echo "    FIX: Update $CONFIG_FILE line 3 to: imu_topic:  \"/ouster/imu\""
  fi
else
  check_item "SLAM config file exists" "FAIL"
fi

# 5. SLAM Status
echo ""
echo "[5] SLAM STATUS"
if rosnode list 2>/dev/null | grep -q "laserMapping"; then
  check_item "SLAM node (laserMapping) running" "PASS"

  # Check if generating odometry
  if rostopic list | grep -q "^/Odometry"; then
    check_item "  → Odometry topic available" "PASS"

    # Try to get actual message
    if timeout 2 rostopic echo /Odometry -n 1 &>/dev/null; then
      check_item "  → Odometry data flowing" "PASS"
    else
      check_item "  → Odometry data flowing" "WARN"
      echo "    Note: SLAM may still be initializing (takes 20-30 seconds)"
    fi
  else
    check_item "  → Odometry topic available" "FAIL"
  fi
else
  check_item "SLAM node (laserMapping) running" "FAIL"
fi

# 6. Data Pipeline Test
echo ""
echo "[6] DATA PIPELINE VERIFICATION"

# Check LiDAR
if timeout 2 rostopic echo /ouster/points -n 1 &>/dev/null; then
  check_item "LiDAR data (ouster/points)" "PASS"
else
  check_item "LiDAR data (ouster/points)" "FAIL"
fi

# Check Odometry
if timeout 2 rostopic echo /Odometry -n 1 &>/dev/null; then
  check_item "SLAM odometry (/Odometry)" "PASS"
else
  check_item "SLAM odometry (/Odometry)" "WARN"
fi

# Summary
echo ""
echo "╔════════════════════════════════════════════════════════════════════╗"
echo "║                        SUMMARY RESULTS                             ║"
echo "╚════════════════════════════════════════════════════════════════════╝"
echo ""
echo "  Passed:  $PASSED"
echo "  Warnings: $WARNINGS"
echo "  Failed:  $FAILED"
echo ""

if [ $FAILED -eq 0 ]; then
  echo "✅ ALL CRITICAL CHECKS PASSED - SYSTEM READY FOR OPERATION"
  echo ""
  exit 0
else
  echo "❌ CRITICAL FAILURES DETECTED - FIX BEFORE OPERATION"
  echo ""
  echo "Common fixes:"
  echo "  1. Verify eth0 cable is connected: ip link show eth0"
  echo "  2. Restart Ouster driver: rosnode kill /ouster/os_driver"
  echo "  3. Check SLAM config has correct topics (see above)"
  echo "  4. Verify SLAM node is running: rosnode list"
  echo ""
  exit 1
fi
