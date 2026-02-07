# MCP Learning System Index

**System Status**: Production-Ready
**Last Updated**: 2026-02-07
**Total Documented Systems**: 1 (Jetson Orin + Ouster)

---

## 📂 Documented Hardware Profiles

### 1. Jetson Orin NX + Ouster OS1-64 + FAST-LIO2 + ArduPilot

**File**: `hardware_profiles/jetson_orin_ouster_fast_lio2_ardupilot_noetic.yaml`

**Fingerprint**: `jetson_orin_nx-ouster_os1_64-fast_lio2-ardupilot-noetic`

**Status**: ✅ `integration_complete: true`

**Details**:
- Platform: Jetson Orin NX 8GB (ARM64, Ubuntu 20.04)
- LiDAR: Ouster OS1-64 (64 channels, 10 Hz, built-in ICM-20948 IMU)
- SLAM: FAST-LIO2 (LiDAR-inertial odometry)
- Autopilot: ArduPilot EKF3 with vision fusion
- ROS: ROS1 Noetic
- Mission Duration: 35+ minutes (tested with GPS-denied flight)
- Flight Tested: Yes (all 4 progressive stages)

**Use Case**:
- GPS-denied autonomous drone navigation
- Mixed indoor/outdoor environments (75m x 75m areas)
- Real-time 10 Hz odometry and position fusion

**Key Features**:
- ✅ Complete data pipeline verified (10 Hz)
- ✅ MAVLink stream issues resolved
- ✅ EKF origin auto-set for GPS-denied
- ✅ LiDAR orientation calibrated (180° rotation)
- ✅ Pre-flight checklist included
- ✅ Safety parameters configured
- ✅ Progressive flight test stages documented

---

## 📋 Known-Good Configurations

### 1. Jetson Orin + Ouster + FAST-LIO2 + ArduPilot (Noetic)

**File**: `known_good_configs/jetson_orin_ouster_fast_lio2_ardupilot_noetic_config.yaml`

**Contents**:
- ✅ FAST-LIO2 configuration (Ouster-optimized parameters)
- ✅ ArduPilot EKF3 vision fusion setup
- ✅ URDF transforms with actual measurements
- ✅ Launch file specifications
- ✅ ROS topic mappings
- ✅ TF tree configuration
- ✅ Installation steps (cloned directly)
- ✅ Pre-flight verification checklist
- ✅ Flight progression (4 stages)
- ✅ Troubleshooting procedures

**How to Use**:
1. Copy parameters from this config
2. Apply to your Jetson Orin + Ouster system
3. No tuning needed for similar hardware
4. Run pre-flight checklist before flying

---

## 🔧 Solutions & Troubleshooting

### 1. ArduPilot MAVLink Stream Rate Issue

**File**: `solutions/ardupilot_mavlink_stream_rate_issue.yaml`

**Problem**: Local position topic empty despite correct EKF vision config

**Symptoms**:
- ✅ Vision pose publishing @ 10 Hz
- ✅ MAVROS connected
- ✅ EKF parameters correct
- ❌ Local position empty

**Root Cause**: MAVLink streams must be explicitly enabled

**Solution**:
```bash
rosservice call /mavros/set_stream_rate 0 10 1
```

**Permanent Fix**: `enable_mavlink_streams.sh` (called from master.launch)

**Impact**: This was blocking all vision fusion. Now auto-resolved on startup.

**Lessons for Future**:
- If vision pose works but local_position empty → stream rate issue (99%)
- Not an EKF problem
- Check this FIRST before debugging EKF

---

## 🚀 How to Use This Learning System

### Search for Matching Hardware

```bash
# From MCP server
search_profiles("jetson ouster fast_lio")

# Response: Returns hardware_profiles/jetson_orin_ouster_fast_lio2_ardupilot_noetic.yaml
```

### Retrieve Complete Configuration

```bash
# Get full profile
get_profile("jetson_orin_nx-ouster_os1_64-fast_lio2-ardupilot-noetic")

# Get known-good config (copy-paste ready)
get_known_good_config("jetson_orin_nx-ouster_os1_64-fast_lio2-ardupilot-noetic")
```

### Look Up Solutions

```bash
# Search solutions by component
search_solutions("autopilot ekf vision")

# Find MAVLink stream issue
search_solutions("stream_rate mavlink")
```

### Run Diagnostics

```bash
# Verify system health
run_diagnostic("verify_installation", "ROS1 noetic ~/slam_ws")

# Check TF tree
run_diagnostic("check_tf_tree", "--frames map odom base_link")

# Monitor SLAM odometry
run_diagnostic("check_topic_pipeline", "/fast_lio/odom /mavros/vision_pose/pose")
```

---

## 📊 Documentation Completeness

| Section | Files | Status |
|---------|-------|--------|
| Hardware Profiles | 1 | ✅ Complete |
| Known-Good Configs | 1 | ✅ Complete |
| Solutions | 1 | ✅ Complete |
| **TOTAL** | **3** | **✅ READY** |

---

## 🎯 Future Documentation Opportunities

When these are completed, they should be documented:

1. **Loop Closure (STD)**
   - Status: Eigen/Ceres version mismatch (Eigen 3.3.7 vs 3.4.0)
   - Use Case: 35+ minute missions with revisits
   - Workaround: SC-A-LOAM alternative

2. **Camera-Based Alternative**
   - VINS-Mono or ORB-SLAM3
   - Status: Potential future variant
   - Advantage: RGB-D more robust indoors

3. **PX4 Variant**
   - Same hardware with PX4 autopilot
   - Would be: `jetson_orin_nx-ouster_os1_64-fast_lio2-px4-humble`
   - Status: Not yet implemented

4. **Jetson AGX Orin Variant**
   - Higher compute (12-core vs 6-core)
   - More GPU (12,800 CUDA cores vs 1,024)
   - Status: Not yet tested

---

## 🔗 Related Files in Repository

### Main System Files
- `~/slam_ws/`: Complete working workspace
- `~/slam_ws/src/orin_slam_integration/`: Integration package
- `~/slam_ws/PHASE6_SUCCESS.md`: Flight test documentation
- `~/slam_ws/slam_integration_progress.yaml`: Session progress

### Diagnostic Files
- `~/slam_integration/docs/SLAM_INTEGRATION_DIAGNOSTICS.md`: Troubleshooting
- `~/slam_integration/docs/AI_SYSTEM_BUILDER_GUIDE.md`: Technical reference

### Learning System Files
- `docs/learned/hardware_profiles/`: Hardware specs and configurations
- `docs/learned/known_good_configs/`: Copy-paste-ready parameters
- `docs/learned/solutions/`: Issues and fixes

---

## 📝 How Profiles Are Used

### Scenario 1: New Jetson Orin + Ouster System
1. AI assistant searches profiles for "jetson ouster"
2. Finds matching profile with `integration_complete: true`
3. Offers to skip to Phase 4 (installation)
4. Provides known-good configuration
5. User deploys without re-tuning

### Scenario 2: Troubleshooting Vision Fusion
1. User reports "local position empty"
2. AI searches solutions for "vision" or "stream"
3. Finds MAVLink stream rate issue
4. Provides fix: `rosservice call /mavros/set_stream_rate 0 10 1`
5. Problem resolved in seconds instead of hours

### Scenario 3: Documentation Needed
1. Team wants to replicate system
2. AI provides complete hardware profile
3. Includes all config files, launch sequences, safety setup
4. Team can deploy identical system without manual tuning

---

## 🎓 Learning System Philosophy

**Goal**: Make successful systems reproducible and debuggable.

**Key Principles**:
1. **Capture Success**: Document working systems completely
2. **Share Solutions**: Solve problems once, document forever
3. **Prevent Rework**: Avoid re-tuning similar hardware
4. **Enable Fast Recovery**: Quick answers for known issues
5. **Build Institutional Memory**: Knowledge survives staff changes

---

## 🚀 Getting Started

### For New Similar Systems
1. Check `hardware_profiles/` for matching fingerprint
2. If found with `integration_complete: true`, use its config
3. Copy parameters directly from `known_good_configs/`
4. No tuning needed - system ready to deploy

### For Troubleshooting
1. Identify symptom
2. Search `solutions/` for matching issue
3. Apply documented fix
4. If not found, report new issue with solution

### For Enhancement
1. Add new hardware profile when complete
2. Document any new issues encountered
3. Share solutions to prevent future debugging
4. Update fingerprints as new variants tested

---

**Status**: ✅ Ready for production use
**Last Updated**: 2026-02-07
**Maintained by**: Claude Code AI Assistant
**Git Commit**: abfd170

---

For detailed information on any profile, config, or solution, see the individual YAML files in this directory.
