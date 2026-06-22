# ArduPilot VISO Parameter Tuning Guide

## VISO_DELAY_MS Tuning (2026-02-16)

**Problem:** `local_position` lags/floats around `vision_pose` — this is the EKF smoothing but can be excessive if VISO_DELAY_MS is misconfigured.

**Root Cause Found:** Default or manually-set `VISO_DELAY_MS=250` but actual pipeline latency was ~50ms. The EKF over-compensates for delay that doesn't exist, causing floaty behavior.

### Measurement Results (Jetson Orin NX + FAST-LIO GPU + MAVROS sidecar)
- Vision bridge latency (`/Odometry` → `vision_pose`): ~30ms avg, 84ms max
- EKF processing (`vision_pose` → `local_position`): 0-45ms
- End-to-end: ~50ms typical
- Correct `VISO_DELAY_MS`: **60** (p95 + 10ms margin)

### How to Set VISO_DELAY_MS
1. Measure actual latency: `run_diagnostic("measure_vision_latency", "--duration 30 --json")`
2. Use suggested value from script output (p95 end-to-end + 10ms)
3. Set via: `ros2 param set /mavros/param VISO_DELAY_MS <value>` (NOT the deprecated `ros2 service call` method)

### VISO_POS_X/Y/Z — LiDAR Offset from FCU
**Always set from URDF.** This is the physical offset from flight controller to LiDAR in ArduPilot FRD frame.

**Coordinate conversion (ROS FLU → ArduPilot FRD):**
- `VISO_POS_X` = URDF X (forward is same)
- `VISO_POS_Y` = -URDF Y (right = -left)
- `VISO_POS_Z` = -URDF Z (down = -up)

**Example (Ouster OS1-64 mounted 60mm below base_link, centered):**
- URDF: `xyz="0.0 0.0 -0.060"` → VISO_POS_X=0.0, Y=0.0, Z=0.06

### Full VISO Parameter Checklist for Integration
| Parameter | How to Set | Notes |
|-----------|-----------|-------|
| `VISO_TYPE` | 1 | Enable MAVLink vision input |
| `VISO_DELAY_MS` | Measure with latency script | p95 + 10ms margin |
| `VISO_POS_M_NSE` | 0.1-0.2 for good SLAM, 0.4 default | Lower = trust SLAM more |
| `VISO_VEL_M_NSE` | 0.2 | Velocity noise |
| `VISO_YAW_M_NSE` | 0.2 | Yaw noise |
| `VISO_POS_X/Y/Z` | From URDF, converted FLU→FRD | Lever arm compensation |

### Common Mistakes
1. **VISO_DELAY_MS too high** → floaty, laggy position tracking
2. **VISO_DELAY_MS too low** → EKF uses stale vision data, position jumps
3. **VISO_POS_X/Y/Z not set** → rotation-induced position errors during flight
4. **VISO_POS_M_NSE too low (<0.05)** → EKF can't reject SLAM glitches
5. **Using `ros2 service call` for params** → unreliable with MAVROS 2.x, use `ros2 param set/get`

### MAVROS 2.x Parameter API
```bash
# Correct (ROS 2 param API):
ros2 param set /mavros/param VISO_DELAY_MS 60
ros2 param get /mavros/param VISO_DELAY_MS

# Wrong (deprecated, hangs):
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSet ...
```
