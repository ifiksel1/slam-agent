#!/bin/bash
# Start SLAM, then verify flight readiness with pre-flight check
# Usage: ./start_slam_with_preflight.sh
# This script launches SLAM, waits for initialization, then verifies system

set -e

echo ""
echo "╔════════════════════════════════════════════════════════════════════╗"
echo "║         SLAM FLIGHT STARTUP WITH PRE-FLIGHT VERIFICATION         ║"
echo "╚════════════════════════════════════════════════════════════════════╝"
echo ""

# Step 1: Launch SLAM in background
echo "[STEP 1] Launching SLAM..."
cd /home/dev/slam_ws
source devel/setup.bash

roslaunch fast_lio mapping_ouster64.launch rviz:=false &
SLAM_PID=$!

echo "  SLAM PID: $SLAM_PID"
echo "  Waiting for SLAM to initialize (20 seconds)..."
sleep 20

echo ""
echo "[STEP 2] Verifying SLAM initialization..."

# Check if SLAM is still running
if ! kill -0 $SLAM_PID 2>/dev/null; then
    echo "  ❌ SLAM process crashed!"
    exit 1
fi

echo "  ✓ SLAM process running"

# Step 2: Run preflight check (SLAM now running)
echo ""
echo "[STEP 3] Running Pre-Flight Verification Check..."
echo ""

/home/dev/slam-agent/preflight_check.sh
PREFLIGHT_RESULT=$?

echo ""

if [ $PREFLIGHT_RESULT -ne 0 ]; then
    echo "╔════════════════════════════════════════════════════════════════════╗"
    echo "║  ❌ FLIGHT VERIFICATION FAILED                                    ║"
    echo "╚════════════════════════════════════════════════════════════════════╝"
    echo ""
    echo "⚠️  System is not ready for flight"
    echo ""
    echo "SLAM is still running. Fix the issues shown above, then:"
    echo "  1. Restart SLAM: rosnode kill /laserMapping"
    echo "  2. Run this script again"
    echo ""
    echo "Keeping SLAM running for debugging (Ctrl+C to stop)..."
    wait $SLAM_PID
    exit 1
fi

# Success!
echo "╔════════════════════════════════════════════════════════════════════╗"
echo "║  ✅ FLIGHT VERIFICATION PASSED - READY FOR FLIGHT                 ║"
echo "╚════════════════════════════════════════════════════════════════════╝"
echo ""
echo "System Status:"
echo "  ✓ SLAM initialized and running"
echo "  ✓ LiDAR data flowing"
echo "  ✓ Odometry generating"
echo "  ✓ All systems verified"
echo ""
echo "SLAM is now running. Safe to proceed with flight operations."
echo ""
echo "To stop SLAM: rosnode kill /laserMapping"
echo "Or press Ctrl+C here to shutdown..."
echo ""

# Keep SLAM running in foreground
wait $SLAM_PID
