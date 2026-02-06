# Software Dependencies & Troubleshooting Flowchart

Load when user has build failures, missing packages, version conflicts, or needs a diagnostic decision tree.

---

## SLAM-Specific Installation Help

For SLAM algorithm build issues, check: https://github.com/engcang/SLAM-application

Covers troubleshooting fixes for 20+ SLAM systems including:
FAST-LIO2, LIO-SAM, LVI-SAM, Point-LIO, KISS-ICP, DLO, DLIO, Ada-LIO, PV-LIO, and more.

**Common issues covered**:
- PCL version conflicts (`find_package(PCL 1.8 REQUIRED)` errors)
- Ceres version incompatibilities
- OpenCV 3.x vs 4.x issues
- Missing LVR2, lvr_ros, mesh_tools dependencies (for SLAMesh)
- CGAL version issues (for ImMesh - requires Ubuntu 20.04+)

Use browser tools to navigate to the specific SLAM algorithm folder for known fixes.

---

## Common Dependency Resolution

```bash
# Install all ROS dependencies at once:
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y

# Specific common packages (ROS Noetic):
sudo apt install \
  ros-noetic-tf2-sensor-msgs \
  ros-noetic-pcl-ros \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport \
  ros-noetic-mavros \
  ros-noetic-mavros-extras
```

## Version Conflicts

```bash
rosversion -d          # Should match your ROS distro
python3 --version      # 3.6+ for Noetic, 3.8+ for Humble
```

**Fix version mismatches**:
- Rebuild package with correct ROS distro
- Check package.xml for version constraints
- Use Docker to isolate environments
- Check SLAM-application repo for known version compatibility issues

---

## Quick Troubleshooting Flowchart

Diagnostic decision tree for users:

```
SLAM not working?
├─ No sensor data?
│  ├─ Check network (LiDAR) → Ethernet setup from Phase 1
│  └─ Check roslaunch output for driver errors
│
├─ Sensor data OK, no SLAM output?
│  ├─ Check CPU usage (>100% = overloaded) → performance.md
│  ├─ Check log messages: rosout, dmesg
│  └─ Verify SLAM subscribed to correct topics → visualization_debugging.md
│
├─ SLAM publishing, wrong movement direction?
│  ├─ Check coordinate frames → coordinate_frames.md
│  └─ Verify TF tree → visualization_debugging.md
│
├─ SLAM works, ArduPilot doesn't use it?
│  ├─ Check vision_pose publishing → phase6_troubleshooting.md
│  └─ Verify EKF parameters → Phase 4 ArduPilot params
│
└─ Everything works but drifts?
   ├─ Check IMU calibration → sensor_calibration.md
   ├─ Enable loop closure
   └─ Add environmental features → hardware_data_quality.md
```

---

## Self-Improvement: Documenting New Solutions

When you solve a problem NOT covered in existing troubleshooting files:

1. Identify the appropriate troubleshooting file (or create a new one)
2. Document using this format:
```markdown
## [Problem Name]
**Problem**: [Description]
**Symptoms**: [What user sees]
**Root Cause**: [Why it happens]
**Solution**: [Step-by-step fix]
**Verification**: [How to confirm fix worked]
```
3. Update the flowchart above if it's a common issue
4. Update COORDINATOR.md if a new troubleshooting file was created
