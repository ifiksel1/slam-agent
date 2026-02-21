# Phase 5: Testing & Validation

## Progressive Testing Protocol

### Pre-Test: Transform Calibration

**Before any test**, verify the LiDAR-to-body transform is correct using the interactive calibrator:

```bash
# Via MCP (preferred):
run_diagnostic("transform_calibrator", "start --json")
# --- Hold drone still for 5 seconds ---
run_diagnostic("transform_calibrator", "record baseline --json")
# --- Move drone forward ~1m along its nose ---
run_diagnostic("transform_calibrator", "record forward --json")
# --- Move drone right ~1m ---
run_diagnostic("transform_calibrator", "record right --json")
# --- Lift drone up ~0.5m ---
run_diagnostic("transform_calibrator", "record up --json")
# --- Rotate drone 90° clockwise ---
run_diagnostic("transform_calibrator", "record yaw --json")
# Review analysis and proposed corrections:
run_diagnostic("transform_calibrator", "analyze --json")
# Preview changes without writing:
run_diagnostic("transform_calibrator", "apply --dry-run --json")
# Apply corrections if they look correct:
run_diagnostic("transform_calibrator", "apply --json")
```

If corrections are applied, restart FAST-LIO before proceeding:
```bash
control_node("/home/dev/slam-gpu", "fastlio", "restart")
```

Then re-run `analyze` to verify residual errors are near zero.

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

## Flight Data Recording

**Before each test**, start the flight recorder to capture all telemetry:

```bash
# Via MCP (preferred):
run_diagnostic("flight_recorder", "start --notes 'bench test 1'")

# Or directly:
./scripts/flight_recorder.sh start --notes "bench test 1"

# For full point cloud recording (high bandwidth):
run_diagnostic("flight_recorder", "start --notes 'full capture' --full")
```

**After each test**, stop recording:
```bash
run_diagnostic("flight_recorder", "stop")
```

The recorder automatically:
- Creates a structured flight directory (`flights/NNN_YYYYMMDD_HHMMSS/`)
- Snapshots all config files (SLAM, Ouster, URDF, ArduPilot params)
- Records 20 default topics: odometry, IMU, RC, battery, ESC telemetry, motor outputs, etc.
- Updates `flight_index.yaml` master index on stop

## Post-Flight Analysis

**Generate an HTML dashboard** after each test:

```bash
# Via MCP (preferred):
run_diagnostic("flight_analysis", "001 --json")   # JSON summary
run_diagnostic("flight_analysis", "001")           # Full HTML report

# Or directly:
./scripts/flight_analysis.py 001
```

The analysis report includes:
- **SLAM vs EKF**: Position comparison, drift analysis, 3D trajectory overlay
- **Motor Analysis**: PWM time-series, motor balance, hover percentage, RPM
- **ATC Performance**: Desired vs actual rates, tracking error, step response
- **Power Analysis**: Voltage/current curves, energy consumption, efficiency (Wh/km)
- **Vibration**: Accel FFT spectrum, RMS vs ArduPilot thresholds, clipping detection
- **Topic Rates**: Rate heatmap, gap detection for critical topics

Drift interpretation: <5% good, 5-15% needs tuning, >15% investigate.

**Manage flight data:**
```bash
run_diagnostic("flight_recorder", "list")              # List all flights
run_diagnostic("flight_recorder", "last")              # Show last flight
run_diagnostic("flight_recorder", "clean --keep-last 10")  # Clean old flights
```

## Save to Learning System
After ALL tests pass:
1. Save full config set to `docs/learned/known_good_configs/<fingerprint>/`
   - Include: SLAM config, URDF, launch files, ArduPilot params, vision bridge config
   - Create `metadata.yaml` with date, test results, performance metrics
2. Update `docs/learned/hardware_profiles.yaml`: set `integration_complete: true` for this profile
