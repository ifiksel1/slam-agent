#!/bin/bash
# SLAM Integration Diagnostics Script
# Performs systematic checks of SLAM to ArduPilot integration
# Version: 1.0
# Date: November 22, 2025

set +e  # Don't exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Configuration - modify these for your specific SLAM algorithm
SLAM_ODOM_TOPIC="/lio_sam/mapping/odometry"  # Change to your SLAM odometry topic
POINTCLOUD_TOPIC="/ouster/points_aligned"

echo "============================================="
echo "  SLAM Integration Diagnostics"
echo "  Ultra-onboard System Health Check"
echo "============================================="
echo ""

# Helper functions
check_pass() {
    echo -e "${GREEN}✓${NC} $1"
}

check_fail() {
    echo -e "${RED}✗${NC} $1"
}

check_warn() {
    echo -e "${YELLOW}⚠${NC} $1"
}

# Level 1: MAVROS Connection
echo "========================================"
echo "[Level 1] Checking MAVROS Connection..."
echo "========================================"

if timeout 3 rostopic echo /mavros/state -n 1 &>/dev/null; then
    STATE=$(timeout 3 rostopic echo /mavros/state -n 1 2>/dev/null)
    CONNECTED=$(echo "$STATE" | grep "connected:" | awk '{print $2}')
    MODE=$(echo "$STATE" | grep "mode:" | awk '{print $2}' | tr -d '"')
    ARMED=$(echo "$STATE" | grep "armed:" | awk '{print $2}')
    
    if [[ "$CONNECTED" == "True" ]]; then
        check_pass "MAVROS connected to ArduPilot"
        echo "   Mode: $MODE, Armed: $ARMED"
    else
        check_fail "MAVROS NOT connected to ArduPilot"
        echo "   → Check USB/serial connection to flight controller"
        echo "   → Verify ArduPilot powered on"
        exit 1
    fi
else
    check_fail "Cannot read /mavros/state topic"
    echo "   → Is MAVROS running? (rosnode list | grep mavros)"
    echo "   → Launch with: roslaunch mavros apm.launch"
    exit 1
fi

echo ""

# Level 2: Sensor Data Flow
echo "========================================"
echo "[Level 2] Checking Sensor Data Flow..."
echo "========================================"

# Check LiDAR
echo -n "LiDAR ($POINTCLOUD_TOPIC): "
if timeout 5 rostopic hz $POINTCLOUD_TOPIC --window 10 2>&1 | grep -q "average rate"; then
    RATE=$(timeout 5 rostopic hz $POINTCLOUD_TOPIC --window 10 2>&1 | grep "average rate" | awk '{print $3}')
    if (( $(echo "$RATE > 5.0" | bc -l) )); then
        check_pass "Publishing at ${RATE} Hz"
    else
        check_warn "Low rate: ${RATE} Hz (expected 10-20 Hz)"
    fi
else
    check_fail "NOT publishing"
    echo "   → Check Ouster driver launched"
    echo "   → Verify LiDAR network connection: ping 169.254.9.99"
fi

# Check IMU
echo -n "IMU (/mavros/imu/data): "
if timeout 5 rostopic hz /mavros/imu/data --window 10 2>&1 | grep -q "average rate"; then
    RATE=$(timeout 5 rostopic hz /mavros/imu/data --window 10 2>&1 | grep "average rate" | awk '{print $3}')
    if (( $(echo "$RATE > 50.0" | bc -l) )); then
        check_pass "Publishing at ${RATE} Hz"
    else
        check_warn "Low rate: ${RATE} Hz (expected 100+ Hz)"
    fi
else
    check_fail "NOT publishing"
    echo "   → Check MAVROS connection (Level 1)"
fi

echo ""

# Level 3: TF Tree
echo "========================================"
echo "[Level 3] Checking TF Tree..."
echo "========================================"

echo -n "Transform map → base_link: "
if timeout 5 rosrun tf tf_echo map base_link 2>&1 | grep -q "At time"; then
    check_pass "Available"
    TRANSFORM=$(timeout 2 rosrun tf tf_echo map base_link 2>&1 | grep -A 3 "Translation")
    echo "$TRANSFORM" | head -n 4 | sed 's/^/   /'
else
    check_fail "NOT available"
    echo "   → SLAM not publishing transforms OR not initialized"
    echo "   → Move robot to build initial map"
    echo "   → Check: rosrun tf view_frames"
fi

echo -n "Robot State Publisher: "
if rosnode list 2>/dev/null | grep -q robot_state_publisher; then
    check_pass "Running"
else
    check_warn "Not running (URDF-based transforms unavailable)"
    echo "   → Launch robot_state_publisher if using URDF"
fi

echo ""

# Level 4: SLAM Operation
echo "========================================"
echo "[Level 4] Checking SLAM Operation..."
echo "========================================"

echo -n "SLAM Node: "
if rosnode list 2>/dev/null | grep -qiE "(slam|lio|fast|coin)"; then
    SLAM_NODE=$(rosnode list 2>/dev/null | grep -iE "(slam|lio|fast|coin)" | head -n 1)
    check_pass "Running ($SLAM_NODE)"
else
    check_fail "No SLAM node detected"
    echo "   → Launch your SLAM algorithm"
    exit 1
fi

echo -n "SLAM Odometry ($SLAM_ODOM_TOPIC): "
if timeout 5 rostopic hz $SLAM_ODOM_TOPIC --window 10 2>&1 | grep -q "average rate"; then
    RATE=$(timeout 5 rostopic hz $SLAM_ODOM_TOPIC --window 10 2>&1 | grep "average rate" | awk '{print $3}')
    check_pass "Publishing at ${RATE} Hz"
    
    # Check for valid data
    ODOM_DATA=$(timeout 2 rostopic echo $SLAM_ODOM_TOPIC -n 1 2>/dev/null)
    if echo "$ODOM_DATA" | grep -q "position:"; then
        POS_X=$(echo "$ODOM_DATA" | grep -A 3 "position:" | grep "x:" | awk '{print $2}')
        POS_Y=$(echo "$ODOM_DATA" | grep -A 3 "position:" | grep "y:" | awk '{print $2}')
        POS_Z=$(echo "$ODOM_DATA" | grep -A 3 "position:" | grep "z:" | awk '{print $2}')
        echo "   Position: x=$POS_X, y=$POS_Y, z=$POS_Z"
        
        # Check for NaN
        if echo "$POS_X $POS_Y $POS_Z" | grep -q "nan"; then
            check_warn "Position contains NaN values!"
        fi
    fi
else
    check_fail "NOT publishing"
    echo "   → Check SLAM subscriptions (IMU, point cloud)"
    echo "   → Check SLAM logs: rostopic echo /rosout | grep -i error"
fi

echo ""

# Level 5: vision_to_mavros Bridge
echo "========================================"
echo "[Level 5] Checking vision_to_mavros..."
echo "========================================"

echo -n "vision_to_mavros Node: "
if rosnode list 2>/dev/null | grep -q vision_to_mavros; then
    BRIDGE_NODE=$(rosnode list 2>/dev/null | grep vision_to_mavros | head -n 1)
    check_pass "Running ($BRIDGE_NODE)"
else
    check_fail "Not running"
    echo "   → Launch vision_to_mavros node"
    exit 1
fi

echo -n "Vision Pose (/mavros/vision_pose/pose): "
if timeout 5 rostopic hz /mavros/vision_pose/pose --window 10 2>&1 | grep -q "average rate"; then
    RATE=$(timeout 5 rostopic hz /mavros/vision_pose/pose --window 10 2>&1 | grep "average rate" | awk '{print $3}')
    if (( $(echo "$RATE > 20.0" | bc -l) )); then
        check_pass "Publishing at ${RATE} Hz"
    else
        check_warn "Low rate: ${RATE} Hz (target 30+ Hz)"
        echo "   → Increase output_rate parameter"
    fi
else
    check_fail "NOT publishing"
    echo "   → Check vision_to_mavros can lookup map→base_link transform"
    echo "   → Check node logs: rosnode log $BRIDGE_NODE"
fi

echo ""

# Level 6: ArduPilot EKF Configuration
echo "========================================"
echo "[Level 6] Checking ArduPilot Config..."
echo "========================================"

echo "Critical Parameters:"

# AHRS_EKF_TYPE
AHRS=$(timeout 5 rosrun mavros mavparam get AHRS_EKF_TYPE 2>&1 | grep "value:" | awk '{print $2}')
if [[ "$AHRS" == "3.0" ]]; then
    check_pass "AHRS_EKF_TYPE = 3 (EKF3 enabled)"
else
    check_fail "AHRS_EKF_TYPE = $AHRS (should be 3)"
    echo "   → Fix: rosrun mavros mavparam set AHRS_EKF_TYPE 3"
fi

# VISO_TYPE
VISO=$(timeout 5 rosrun mavros mavparam get VISO_TYPE 2>&1 | grep "value:" | awk '{print $2}')
if [[ "$VISO" == "2.0" ]]; then
    check_pass "VISO_TYPE = 2 (MAVLink vision enabled)"
else
    check_fail "VISO_TYPE = $VISO (should be 2)"
    echo "   → Fix: rosrun mavros mavparam set VISO_TYPE 2"
fi

# EK3_SRC1_POSXY
POSXY=$(timeout 5 rosrun mavros mavparam get EK3_SRC1_POSXY 2>&1 | grep "value:" | awk '{print $2}')
if [[ "$POSXY" == "6.0" ]]; then
    check_pass "EK3_SRC1_POSXY = 6 (External nav for position)"
else
    check_warn "EK3_SRC1_POSXY = $POSXY (use 6 for GPS-denied, 3 for GPS)"
    if [[ "$POSXY" == "3.0" ]]; then
        echo "   → Using GPS mode (OK for outdoor flight)"
    else
        echo "   → Fix for indoor: rosrun mavros mavparam set EK3_SRC1_POSXY 6"
    fi
fi

# EK3_SRC1_VELXY
VELXY=$(timeout 5 rosrun mavros mavparam get EK3_SRC1_VELXY 2>&1 | grep "value:" | awk '{print $2}')
if [[ "$VELXY" == "6.0" ]]; then
    check_pass "EK3_SRC1_VELXY = 6 (External nav for velocity)"
else
    check_warn "EK3_SRC1_VELXY = $VELXY (use 6 for GPS-denied, 3 for GPS)"
fi

# Check EKF origin
echo -n "EKF Origin: "
if timeout 3 rostopic echo /mavros/global_position/global -n 1 2>&1 | grep -q "latitude:"; then
    LAT=$(timeout 3 rostopic echo /mavros/global_position/global -n 1 2>&1 | grep "latitude:" | awk '{print $2}')
    LON=$(timeout 3 rostopic echo /mavros/global_position/global -n 1 2>&1 | grep "longitude:" | awk '{print $2}')
    if [[ "$LAT" != "0.0" ]] && [[ "$LON" != "0.0" ]]; then
        check_pass "Set (lat=$LAT, lon=$LON)"
    else
        check_warn "Not set (0, 0)"
        echo "   → Run: rosrun vision_to_mavros set_origin2.py"
    fi
else
    check_fail "Cannot read origin"
fi

echo ""

# Level 7: EKF Fusion
echo "========================================"
echo "[Level 7] Checking EKF Fusion..."
echo "========================================"

echo -n "EKF Local Position: "
if timeout 5 rostopic hz /mavros/local_position/pose --window 10 2>&1 | grep -q "average rate"; then
    RATE=$(timeout 5 rostopic hz /mavros/local_position/pose --window 10 2>&1 | grep "average rate" | awk '{print $3}')
    check_pass "Publishing at ${RATE} Hz"
    
    # Get current position
    LOCAL_POS=$(timeout 2 rostopic echo /mavros/local_position/pose -n 1 2>/dev/null)
    if echo "$LOCAL_POS" | grep -q "position:"; then
        POS_X=$(echo "$LOCAL_POS" | grep -A 3 "position:" | grep "x:" | awk '{print $2}')
        POS_Y=$(echo "$LOCAL_POS" | grep -A 3 "position:" | grep "y:" | awk '{print $2}')
        POS_Z=$(echo "$LOCAL_POS" | grep -A 3 "position:" | grep "z:" | awk '{print $2}')
        echo "   Position: x=$POS_X, y=$POS_Y, z=$POS_Z"
    fi
else
    check_fail "NOT updating"
    echo "   → EKF not fusing vision data"
    echo "   → Check all previous levels"
fi

# Check EKF status
echo -n "EKF Status: "
if timeout 3 rostopic echo /mavros/state -n 1 2>&1 | grep -q "system_status:"; then
    STATUS=$(timeout 3 rostopic echo /mavros/state -n 1 2>&1 | grep "system_status:" | awk '{print $2}')
    if [[ "$STATUS" -ge "3" ]]; then
        check_pass "System status: $STATUS (STANDBY or better)"
    else
        check_warn "System status: $STATUS (not ready)"
    fi
else
    check_warn "Cannot read EKF status"
fi

echo ""

# Summary
echo "============================================="
echo "  DIAGNOSTIC SUMMARY"
echo "============================================="
echo ""

# Count issues
ISSUES=0

# Recheck critical items
if ! timeout 3 rostopic echo /mavros/state -n 1 2>&1 | grep -q "connected: True"; then
    echo -e "${RED}✗${NC} CRITICAL: MAVROS not connected"
    ((ISSUES++))
fi

if ! timeout 5 rostopic hz $SLAM_ODOM_TOPIC --window 10 2>&1 | grep -q "average rate"; then
    echo -e "${RED}✗${NC} CRITICAL: SLAM not publishing odometry"
    ((ISSUES++))
fi

if ! timeout 5 rostopic hz /mavros/vision_pose/pose --window 10 2>&1 | grep -q "average rate"; then
    echo -e "${RED}✗${NC} CRITICAL: vision_to_mavros not publishing"
    ((ISSUES++))
fi

VISO_CHECK=$(timeout 5 rosrun mavros mavparam get VISO_TYPE 2>&1 | grep "value:" | awk '{print $2}')
if [[ "$VISO_CHECK" != "2.0" ]]; then
    echo -e "${RED}✗${NC} CRITICAL: VISO_TYPE not set to 2"
    ((ISSUES++))
fi

if [[ $ISSUES -eq 0 ]]; then
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}  ✓ ALL CHECKS PASSED${NC}"
    echo -e "${GREEN}  System ready for operation${NC}"
    echo -e "${GREEN}========================================${NC}"
else
    echo -e "${RED}========================================${NC}"
    echo -e "${RED}  ✗ FOUND $ISSUES CRITICAL ISSUE(S)${NC}"
    echo -e "${RED}  Review output above for details${NC}"
    echo -e "${RED}========================================${NC}"
fi

echo ""
echo "For detailed troubleshooting, see:"
echo "  docs/SLAM_INTEGRATION_DIAGNOSTICS.md"
echo ""
echo "To run manual tests:"
echo "  rostopic hz /mavros/vision_pose/pose /slam/odometry /mavros/local_position/pose"
echo "  rosrun tf view_frames && evince frames.pdf"
echo ""

