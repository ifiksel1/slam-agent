# Phase 5: Testing & Validation

## Progressive Testing Protocol

### Test 1: Bench Test (Props OFF)
- [ ] LiDAR publishing point clouds (`rostopic hz /LIDAR_TOPIC` → 10-20 Hz)
- [ ] SLAM publishing odometry (`rostopic hz /slam/odometry` → 10-20 Hz)
- [ ] Vision bridge publishing (`rostopic hz /mavros/vision_pose/pose` → 30 Hz)
- [ ] MAVROS connected (`rostopic echo /mavros/state` → connected=true)
- [ ] TF tree complete (`rosrun tf view_frames`)
- [ ] Manual movement tracked by SLAM
- [ ] EKF local_position updating

### Test 2: Ground Test (Props ON, Tethered)
- [ ] Tethered, safety area clear (5m radius)
- [ ] RC transmitter ready
- [ ] Arms successfully in STABILIZE
- [ ] Position holds when armed
- [ ] Mode switching works (STABILIZE ↔ GUIDED)

### Test 3: Flight Test (GPS mode, outdoor)
- [ ] GPS lock ≥10 satellites
- [ ] EK3_SRC1_POSXY=3 (GPS mode)
- [ ] Geofence active
- [ ] Takeoff to 2m LOITER
- [ ] Switch to GUIDED, send 5m waypoint
- [ ] Position error < 1m
- [ ] RTL successful

### Test 4: GPS-Denied Test (Indoor)
- [ ] EK3_SRC1_POSXY=6, EK3_SRC1_VELXY=6 (vision mode)
- [ ] ARMING_CHECK=388598
- [ ] EKF origin set (run set_origin2.py)
- [ ] Vision pose publishing before arm
- [ ] Hover 30s, drift < 20cm
- [ ] 1m waypoint test, accuracy < 50cm
- [ ] RTL successful

## Performance Metrics
- SLAM rate: 10-20 Hz
- Vision pose rate: 30 Hz
- Latency: < 150ms
- Drift: < 5% of distance traveled
- CPU: < 60%, RAM: < 10GB

## Post-Flight Analysis
```bash
rosbag record -O flight_test.bag /slam/odometry /mavros/vision_pose/pose \
  /mavros/local_position/pose /mavros/imu/data /LIDAR_TOPIC /mavros/state
./scripts/analyze_slam_bag.py flight_test.bag --plot
```

Drift interpretation: <5% good, 5-15% needs tuning, >15% investigate.

## Save to Learning System
After ALL tests pass:
1. Save full config set to `docs/learned/known_good_configs/<fingerprint>/`
   - Include: SLAM config, URDF, launch files, ArduPilot params, vision bridge config
   - Create `metadata.yaml` with date, test results, performance metrics
2. Update `docs/learned/hardware_profiles.yaml`: set `integration_complete: true` for this profile
