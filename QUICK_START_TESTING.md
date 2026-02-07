# 🚀 Quick Start: Run Full SLAM System Test

## One-Command Solution

On your **Jetson Orin NX**, run:

```bash
bash ~/slam-agent/RUN_FULL_SYSTEM_TEST.sh
```

That's it! The script will:
1. ✅ Start SLAM system (FAST-LIO2 + Ouster)
2. ✅ Launch flight controller bridge (MAVROS)
3. ✅ Run all 8 diagnostic scripts
4. ✅ Generate comprehensive report
5. ✅ Display pass/fail summary

---

## What the Script Does

### Phase 1: Environment Setup (5 seconds)
- Sources ROS1 Noetic
- Sources ~/slam_ws workspace
- Creates logging directory

### Phase 2: SLAM Launch (10 seconds)
- Launches FAST-LIO2 mapping node
- Launches Ouster OS1-64 driver
- Launches vision-to-MAVROS bridge
- Waits for system stabilization

### Phase 3: Run All 8 Diagnostics (60-90 seconds)
```
1. verify_installation.sh       - Installation check
2. slam_diagnostics.sh          - Pipeline health
3. check_topic_pipeline.py      - Data flow rates
4. check_tf_tree.py             - Transform validation
5. check_autopilot_params.py    - Flight controller config
6. check_sensor_time_sync.py    - Timestamp sync
7. check_urdf.py                - Robot description
8. analyze_slam_bag.py          - Bag analysis (demo)
```

### Phase 4: Generate Report (5 seconds)
- Creates detailed log file
- Shows pass/fail summary
- Displays system info

---

## Expected Output

```
╔════════════════════════════════════════════════════════════╗
║     SLAM System - Complete Testing Suite                  ║
╚════════════════════════════════════════════════════════════╝

[1/4] Sourcing ROS and workspace...
✓ Environment ready

[2/4] Starting SLAM system...
✓ SLAM launched (PID: 12345)
✓ SLAM system running

[3/4] Running diagnostic scripts...
✓ Installation verification: PASSED
✓ Complete SLAM pipeline: PASSED
✓ Topic pipeline monitoring: PASSED
✓ Transform tree validation: PASSED
⚠ Flight controller params: FAILED (no FC connected)
✓ Sensor time sync: PASSED
✓ URDF validation: PASSED
⚠ SLAM bag analysis: FAILED (expected - no recorded data)

[4/4] Generating final report...

╔════════════════════════════════════════════════════════════╗
║     TEST RESULTS                                           ║
╚════════════════════════════════════════════════════════════╝

Tests Run:  8
Passed:     6
Failed:     2
Pass Rate:  75%

Full Report: /tmp/slam_test_20260207_061000/diagnostic_report.txt
```

---

## How to Interpret Results

### ✅ PASSED Tests
- System component is working correctly
- No action needed

### ⚠️ FAILED Tests
- Check if it's expected (e.g., flight controller not connected)
- Or system needs setup/fixing
- See detailed report for troubleshooting

### Normal Failures (Expected)
- `check_autopilot_params.py` - if FC not connected
- `analyze_slam_bag.py` - if no recorded flight data
- `slam_diagnostics.sh` - if MAVROS not running

---

## View Detailed Report

```bash
# Show full diagnostic report
cat /tmp/slam_test_*/diagnostic_report.txt

# Monitor SLAM in real-time while running
rostopic echo /fast_lio/odom -n 10

# Check what topics are publishing
rostopic list

# Monitor ROS nodes
rosnode list
```

---

## Troubleshooting

### Script Fails at Start
```bash
# Verify ROS installation
source /opt/ros/noetic/setup.bash
roscore &
sleep 2
killall roscore

# Verify workspace
ls ~/slam_ws/src
source ~/slam_ws/devel/setup.bash
```

### SLAM Won't Start
```bash
# Check LiDAR connection
ping os-122224003549.local  # Should respond

# Check Ouster driver
roslaunch ouster_ros driver.launch

# View startup logs
tail -f /tmp/slam_test_*/slam_startup.log
```

### Diagnostics Timeout
- SLAM may be taking too long to initialize
- Increase wait time in the script (line with `sleep 10`)
- Or run diagnostics manually after SLAM starts:
  ```bash
  bash ~/slam-agent/scripts/check_topic_pipeline.py --duration 30
  ```

---

## Advanced: Run Diagnostics Manually

```bash
# Source environment
source /opt/ros/noetic/setup.bash
source ~/slam_ws/devel/setup.bash

# Start SLAM in one terminal
roslaunch orin_slam_integration master.launch

# In another terminal, run individual diagnostics
python3 ~/slam-agent/scripts/check_topic_pipeline.py --duration 60
python3 ~/slam-agent/scripts/check_tf_tree.py --verbose
python3 ~/slam-agent/scripts/check_autopilot_params.py --autopilot ardupilot

# Record flight data for analysis
rosbag record /fast_lio/odom /ouster/points -o flight_test.bag

# Analyze recorded bag
python3 ~/slam-agent/scripts/analyze_slam_bag.py flight_test.bag
```

---

## Sample Script Modifications

### Extend testing duration
Edit line with `--duration 15` → `--duration 60`

### Skip MAVROS launch
Comment out MAVROS launch (it's optional for SLAM-only testing)

### Run only specific diagnostics
Modify the `run_diagnostic` calls at the bottom

### Custom logging location
Change `LOG_DIR="/tmp/slam_test_..."` to your preferred path

---

## Status Files

After running, check:

```
/tmp/slam_test_YYYYMMDD_HHMMSS/
├── slam_startup.log          - SLAM system output
├── diagnostic_report.txt     - Full test results
└── [diagnostic_name].log     - Individual test logs
```

---

## Next Steps After Testing

### If All Tests Pass ✅
- System is ready for flight testing
- Proceed to bench test (props off)
- Then ground test (tethered)
- Finally GPS-denied flight

### If Some Tests Fail ⚠️
1. Identify which test failed
2. Check the detailed report for error message
3. Run individual diagnostic to debug
4. Consult `/home/dev/slam-agent/DIAGNOSTIC_SCRIPTS_TEST_RESULTS.md`
5. See `/home/dev/slam-agent/MCP_SCRIPTS_REFERENCE.md` for detailed options

### Common Issues & Fixes

| Issue | Cause | Fix |
|-------|-------|-----|
| `check_tf_tree` fails | SLAM not running | Make sure SLAM is publishing transforms |
| `check_autopilot_params` fails | FC not connected | Connect via USB, launch MAVROS |
| `check_topic_pipeline` fails | ROS not sourced | Run: `source /opt/ros/noetic/setup.bash` |
| `slam_diagnostics` fails | MAVROS not running | Not required for SLAM-only test |

---

## Success Criteria

Your SLAM system is **ready to fly** when:

✅ `verify_installation.sh` - PASSED
✅ `check_tf_tree.py` - PASSED (valid frame chain)
✅ `check_topic_pipeline.py` - PASSED (10 Hz data flow)
✅ `check_urdf.py` - PASSED (valid robot description)
✅ `slam_diagnostics.sh` - PASSED (all components connected)
✅ `check_autopilot_params.py` - PASSED (EKF configured)

---

## Support

For issues, refer to:
- `~/slam-agent/MCP_SCRIPTS_REFERENCE.md` - Detailed script documentation
- `~/slam-agent/DIAGNOSTIC_SCRIPTS_TEST_RESULTS.md` - Previous test results
- `~/slam-agent/SCRIPT_FIXES_SUMMARY.md` - Script improvements and fixes
- `~/slam_ws/PHASE6_SUCCESS.md` - Complete system documentation

---

**Ready to test? Run:**
```bash
bash ~/slam-agent/RUN_FULL_SYSTEM_TEST.sh
```

Happy testing! 🚀
