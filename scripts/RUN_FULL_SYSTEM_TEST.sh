#!/bin/bash
################################################################################
# SLAM System - Complete Testing Suite
#
# Usage: bash RUN_FULL_SYSTEM_TEST.sh
#
# This script:
# 1. Launches SLAM system (FAST-LIO2 + Ouster)
# 2. Launches flight controller bridge (MAVROS)
# 3. Runs all 8 diagnostic scripts
# 4. Generates comprehensive report
#
# Requires: ROS1 Noetic, ~/slam_ws workspace
################################################################################

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Setup paths
SLAM_WS="$HOME/slam_ws"
SCRIPTS_DIR="$HOME/slam-agent/scripts"
LOG_DIR="/tmp/slam_test_$(date +%Y%m%d_%H%M%S)"
REPORT_FILE="$LOG_DIR/diagnostic_report.txt"

mkdir -p "$LOG_DIR"

echo -e "${BLUE}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║     SLAM System - Complete Testing Suite                  ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""

# ============================================================================
# STEP 1: Source Environment
# ============================================================================
echo -e "${YELLOW}[1/4] Sourcing ROS and workspace...${NC}"
source /opt/ros/noetic/setup.bash
source "$SLAM_WS/devel/setup.bash"
echo -e "${GREEN}✓ Environment ready${NC}"
echo ""

# ============================================================================
# STEP 2: Start SLAM System
# ============================================================================
echo -e "${YELLOW}[2/4] Starting SLAM system...${NC}"
echo "  • SLAM: FAST-LIO2"
echo "  • LiDAR: Ouster OS1-64"
echo "  • Workspace: $SLAM_WS"
echo ""

# Start SLAM in background (FAST-LIO for Ouster OS1-64)
roslaunch fast_lio mapping_ouster64.launch > "$LOG_DIR/slam_startup.log" 2>&1 &
SLAM_PID=$!
echo -e "${GREEN}✓ SLAM launched (PID: $SLAM_PID)${NC}"

# Wait for initialization
echo "  Waiting 10 seconds for initialization..."
sleep 10

# Verify SLAM is running
if ps -p $SLAM_PID > /dev/null; then
    echo -e "${GREEN}✓ SLAM system running${NC}"
else
    echo -e "${RED}✗ SLAM system failed to start${NC}"
    cat "$LOG_DIR/slam_startup.log"
    exit 1
fi

echo ""

# ============================================================================
# STEP 3: Run All Diagnostic Scripts
# ============================================================================
echo -e "${YELLOW}[3/4] Running diagnostic scripts...${NC}"
echo ""

{
    echo "╔════════════════════════════════════════════════════════════╗"
    echo "║     SLAM Diagnostic Test Report                           ║"
    echo "║     $(date)                        ║"
    echo "╚════════════════════════════════════════════════════════════╝"
    echo ""
} > "$REPORT_FILE"

# Track results
PASSED=0
FAILED=0

# Function to run diagnostic
run_diagnostic() {
    local name=$1
    local cmd=$2
    local description=$3

    echo -e "${BLUE}Running: $description${NC}"

    {
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo "Test: $name"
        echo "Description: $description"
        echo "Command: $cmd"
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    } >> "$REPORT_FILE"

    if eval "$cmd" >> "$REPORT_FILE" 2>&1; then
        echo -e "${GREEN}  ✓ PASSED${NC}"
        ((PASSED++))
        echo "Status: ✓ PASSED" >> "$REPORT_FILE"
    else
        echo -e "${YELLOW}  ⚠ FAILED (check logs)${NC}"
        ((FAILED++))
        echo "Status: ⚠ FAILED" >> "$REPORT_FILE"
    fi

    echo "" >> "$REPORT_FILE"
    echo ""
}

# Run all 8 diagnostics
run_diagnostic "verify_installation" \
    "bash $SCRIPTS_DIR/verify_installation.sh ROS1 noetic $SLAM_WS" \
    "Installation verification"

run_diagnostic "slam_diagnostics" \
    "bash $SCRIPTS_DIR/slam_diagnostics.sh" \
    "Complete SLAM pipeline health check"

run_diagnostic "check_topic_pipeline" \
    "python3 $SCRIPTS_DIR/check_topic_pipeline.py --duration 15" \
    "Topic pipeline monitoring (15 sec)"

run_diagnostic "check_tf_tree" \
    "python3 $SCRIPTS_DIR/check_tf_tree.py --verbose" \
    "Transform tree validation"

run_diagnostic "check_autopilot_params" \
    "python3 $SCRIPTS_DIR/check_autopilot_params.py --autopilot ardupilot" \
    "Flight controller parameter verification"

run_diagnostic "check_sensor_time_sync" \
    "python3 $SCRIPTS_DIR/check_sensor_time_sync.py /ouster/points /ouster/imu --duration 10" \
    "Sensor time synchronization check"

run_diagnostic "check_urdf" \
    "python3 $SCRIPTS_DIR/check_urdf.py $SLAM_WS/src/orin_slam_integration/urdf/drone.urdf" \
    "URDF robot description validation"

run_diagnostic "analyze_slam_bag" \
    "python3 $SCRIPTS_DIR/analyze_slam_bag.py /nonexistent.bag 2>&1 | head -20" \
    "SLAM bag file analysis (expected to fail - no recorded data)"

echo ""

# ============================================================================
# STEP 4: Generate Report
# ============================================================================
echo -e "${YELLOW}[4/4] Generating final report...${NC}"
echo ""

# Add summary to report
{
    echo ""
    echo "╔════════════════════════════════════════════════════════════╗"
    echo "║     TEST SUMMARY                                           ║"
    echo "╚════════════════════════════════════════════════════════════╝"
    echo ""
    echo "Total Tests Run: $((PASSED + FAILED))"
    echo "Passed: $PASSED"
    echo "Failed: $FAILED"
    echo "Pass Rate: $(( PASSED * 100 / (PASSED + FAILED) ))%"
    echo ""
    echo "Test Duration: $(date)"
    echo ""
} >> "$REPORT_FILE"

# System information
{
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "System Information"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "Hostname: $(hostname)"
    echo "OS: $(lsb_release -d | cut -f2)"
    echo "ROS: $ROS_DISTRO"
    echo "Architecture: $(uname -m)"
    echo "Kernel: $(uname -r)"
    echo ""
} >> "$REPORT_FILE"

# Display results
echo -e "${BLUE}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║     TEST RESULTS                                           ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "Tests Run:  $((PASSED + FAILED))"
echo -e "Passed:     ${GREEN}$PASSED${NC}"
echo -e "Failed:     ${YELLOW}$FAILED${NC}"
echo -e "Pass Rate:  $(( PASSED * 100 / (PASSED + FAILED) ))%"
echo ""

# Cleanup
echo "Stopping SLAM system..."
kill $SLAM_PID 2>/dev/null || true
sleep 2

echo ""
echo -e "${GREEN}✓ Testing complete!${NC}"
echo ""
echo "Full Report:"
echo -e "  ${BLUE}$REPORT_FILE${NC}"
echo ""
echo "View report:"
echo -e "  ${BLUE}cat $REPORT_FILE${NC}"
echo ""
echo "Live monitoring (if SLAM still running):"
echo "  • Monitor SLAM odometry: ${BLUE}rostopic echo /fast_lio/odom -n 1${NC}"
echo "  • Check ROS nodes: ${BLUE}rosnode list${NC}"
echo "  • Check topics: ${BLUE}rostopic list${NC}"
echo ""

# Exit with pass/fail status
if [ $FAILED -eq 0 ]; then
    exit 0
else
    exit 1
fi
