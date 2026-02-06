# Phase 6: Operational Troubleshooting

Post-integration issues. Load this when the system is built but not working correctly.

## Pre-Check: Solutions Log
Before diagnosing, read `docs/learned/solutions_log.yaml` and search for matching tags/symptoms.
If a prior solution exists, present it first: "This matches a previously solved issue. Try this fix first."

---

## Issue 1: SLAM Not Initializing

**Symptoms**: No odometry output, TF not publishing

**Diagnosis**:
```bash
# ROS1:
rostopic hz /slam/odometry
rosnode info /slam_node
rostopic echo /rosout | grep slam

# ROS2:
ros2 topic hz /slam/odometry
ros2 node info /slam_node
```

**Solutions**:
1. Ensure sufficient motion during startup (move drone to build initial map)
2. Check environment has features (walls, structures, not open field)
3. Verify IMU data arriving: `rostopic hz /mavros/imu/data`
4. Check point cloud quality: RViz → Add PointCloud2
5. Review SLAM logs for initialization messages

---

## Issue 2: Vision Pose Not Reaching ArduPilot

**Symptoms**: `/mavros/vision_pose/pose` not publishing

**Diagnosis**:
```bash
rostopic hz /mavros/vision_pose/pose
rosnode list | grep vision_to_mavros
rosnode info /slam_to_mavros
```

**Solutions**:
1. Check vision_to_mavros node running
2. Verify TF tree complete: `rosrun tf view_frames`
3. Check `use_tf` parameter matches SLAM TF publishing behavior
4. If SLAM doesn't publish TF, set `use_tf=false` and remap `input_odom`
5. Check `gamma_world` parameter (0.0 for LiDAR, -1.5708 for some cameras)

---

## Issue 3: EKF Not Using Vision Data

**Symptoms**: `/mavros/local_position/pose` not tracking SLAM

**Diagnosis**:
```bash
rosrun mavros mavparam get EK3_SRC1_POSXY  # Should be 6
rosrun mavros mavparam get VISO_TYPE        # Should be 2
rostopic echo /mavros/global_position/global # Check origin set
```

**Solutions**:
1. Set `EK3_SRC1_POSXY=6` and `EK3_SRC1_VELXY=6`
2. Set `VISO_TYPE=2`
3. Run `set_origin2.py` to initialize EKF origin
4. Reboot flight controller after parameter changes
5. Verify vision pose timestamp is recent (not stale data)

---

## Issue 4: Large Drift Over Time

**Symptoms**: Position drifts even when stationary

**Diagnosis**:
```bash
rostopic echo /slam/odometry/pose/covariance  # Check uncertainty
rostopic echo /mavros/local_position/pose     # Monitor drift
```

**Solutions**:
1. Enable loop closure in SLAM config (`loopClosureEnableFlag: true`)
2. Verify IMU noise parameters match flight controller (run Allan variance)
3. Check extrinsics correct (LiDAR-to-IMU transform)
4. Improve environment (add features if feature-poor)
5. Reduce SLAM downsample rate for more features
6. Tune SLAM parameters for your environment (see Phase 7)

---

## Save to Learning System
After resolving any issue, append to `docs/learned/solutions_log.yaml`:
- Auto-increment `id`
- Record: date, phase, hardware fingerprint, symptom, root_cause, fix, files_changed, tags
- Tags should be searchable keywords (e.g., `[ekf, vision_pose, ardupilot, parameter]`)

## Output
After resolving issues, return to Phase 5 testing or proceed to Phase 7 for optimization.
