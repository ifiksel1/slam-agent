# Vision Pipeline Latency Measurement Tool

**Script:** `/home/dev/slam-agent/scripts/measure_vision_latency.py`
**Purpose:** Measure end-to-end latency in the SLAM-to-MAVROS vision pose pipeline and recommend optimal `VISO_DELAY_MS` ArduPilot parameter.

---

## Overview

The vision pipeline latency measurement tool analyzes the time delays between:

1. **SLAM Output** (`/Odometry`) → **Vision Bridge** (`/mavros/vision_pose/pose`)
2. **Vision Bridge** → **EKF Local Position** (`/mavros/local_position/pose`)
3. **Total End-to-End** (`/Odometry` → `/mavros/local_position/pose`)

It computes statistical measures (mean, median, p95, max) for each stage and recommends an optimal `VISO_DELAY_MS` value based on the p95 end-to-end latency plus a 10ms safety margin.

---

## Quick Start

### Via MCP (Recommended)

```python
# From Claude Code or MCP client
run_diagnostic("measure_vision_latency", "--duration 30 --json")
```

### Direct Execution (Inside ROS 2 Container)

```bash
# Inside slam-gpu container
source /opt/ros/humble/setup.bash
cd /home/dev/slam-agent/scripts
./measure_vision_latency.py --duration 30 --verbose
```

### From Host (Docker Container Mode)

```bash
# Run from host, executes inside container
cd /home/dev/slam-agent/scripts
./measure_vision_latency.py --container slam-gpu --duration 30 --json
```

---

## Command-Line Options

| Option | Description | Default |
|--------|-------------|---------|
| `--duration SECONDS` | Measurement duration in seconds | 30 |
| `--json` | Output JSON format (MCP-compatible) | Human-readable |
| `--container NAME` | Execute inside Docker container | Host execution |
| `--verbose` | Show per-message latencies during collection | Silent |

---

## Output Format

### Human-Readable Mode (Default)

```
==================================================================
SLAM Vision Pipeline Latency Measurement Report
==================================================================
Measurement Duration: 30s
Timestamp: 2026-02-16T10:30:45.123456

Stage 1: SLAM Output → Vision Bridge
  Topic: /Odometry → /mavros/vision_pose/pose
  Samples: 890
  Mean:     12.3 ms
  Median:   11.8 ms
  P95:      18.5 ms
  Max:      25.1 ms

Stage 2: Vision Pose → EKF Local Position
  Topic: /mavros/vision_pose/pose → /mavros/local_position/pose
  Samples: 885
  Mean:      8.7 ms
  Median:    8.2 ms
  P95:      12.4 ms
  Max:      18.9 ms

Total Pipeline: SLAM → EKF (End-to-End)
  Topic: /Odometry → /mavros/local_position/pose
  Samples: 885
  Mean:     21.0 ms
  Median:   20.1 ms
  P95:      30.9 ms
  Max:      44.0 ms

──────────────────────────────────────────────────────────────────
VISO_DELAY_MS Recommendation:
  Suggested Value: 41 ms
  Basis: p95 end-to-end latency (30.9ms) + 10ms margin
  Current Value:   50 ms
  Assessment:      ACCEPTABLE

  ✓ Current value (50ms) is within acceptable range.
==================================================================
```

### JSON Mode (--json)

```json
{
  "timestamp": "2026-02-16T10:30:45.123456",
  "measurement_duration_s": 30,
  "latencies": {
    "slam_to_vision": {
      "count": 890,
      "mean_ms": 12.3,
      "median_ms": 11.8,
      "p95_ms": 18.5,
      "max_ms": 25.1,
      "min_ms": 8.2
    },
    "vision_to_ekf": {
      "count": 885,
      "mean_ms": 8.7,
      "median_ms": 8.2,
      "p95_ms": 12.4,
      "max_ms": 18.9,
      "min_ms": 5.1
    },
    "end_to_end": {
      "count": 885,
      "mean_ms": 21.0,
      "median_ms": 20.1,
      "p95_ms": 30.9,
      "max_ms": 44.0,
      "min_ms": 15.3
    }
  },
  "recommendation": {
    "suggested_value_ms": 41,
    "basis": "p95 end-to-end latency (30.9ms) + 10ms margin",
    "current_value_ms": 50,
    "assessment": "ACCEPTABLE",
    "message": "Current value (50ms) is within acceptable range."
  },
  "errors": []
}
```

---

## Recommendation Logic

### VISO_DELAY_MS Calculation

```
VISO_DELAY_MS = ceil(p95_end_to_end_latency + 10ms_safety_margin)
```

### Assessment Categories

| Assessment | Condition | Meaning |
|------------|-----------|---------|
| `TOO_LOW` | Current < Suggested - 20ms | EKF may use stale vision data, causing drift |
| `ACCEPTABLE` | Suggested - 20ms ≤ Current ≤ Suggested + 50ms | Current value is appropriate |
| `TOO_HIGH` | Current > Suggested + 50ms | Excessive delay reduces fusion responsiveness |

---

## Troubleshooting

### Error: Timeout waiting for topics

**Symptoms:**
```
ERROR: Timeout waiting for topics
  /Odometry: ✗
  /mavros/vision_pose/pose: ✗
```

**Causes & Fixes:**
1. **SLAM not running** → Start FAST-LIO: `fastlio.sh start`
2. **Vision bridge not running** → Start bridge: `ros2 launch mavros_bridge mavros_bridge_launch.py`
3. **MAVROS not running** → Start MAVROS: `ros2 launch mavros apm.launch fcu_url:=udp://:14550@`
4. **Wrong ROS_DOMAIN_ID** → Verify: `echo $ROS_DOMAIN_ID` (should match across containers)

### Error: No SLAM→Vision latency data collected

**Symptoms:**
- Script runs but collects 0 samples
- Topics exist but no synchronization

**Causes & Fixes:**
1. **Clock skew** → Timestamps differ by >200ms between topics
2. **Low publish rate** → Topics publishing <1Hz (check with `ros2 topic hz`)
3. **Message buffering** → Increase buffer size in script (edit `maxlen=100` to `maxlen=500`)

### Warning: /mavros/local_position/pose not publishing

**Symptoms:**
```
⚠ Errors/Warnings:
  - /mavros/local_position/pose not publishing (EKF may not be fusing vision data)
```

**Causes & Fixes:**
1. **EKF not configured** → Set `EK3_SRC1_POSXY=6` (ExternalNav) in ArduPilot
2. **EKF not armed** → Check `AHRS_EKF_TYPE=3` (EKF3)
3. **Vision health check failing** → Check `VISO_TYPE=1` (MAV_ODOMETRY_POS)

---

## Integration with SLAM Workflow

### Phase 5: Bench Testing Checklist

1. Start SLAM pipeline:
   ```bash
   fastlio.sh start
   ouster.sh start  # or use rosbag
   ```

2. Start MAVROS + vision bridge:
   ```bash
   ros2 launch mavros apm.launch fcu_url:=udp://:14550@
   ros2 launch mavros_bridge mavros_bridge_launch.py
   ```

3. Measure latency:
   ```bash
   measure_vision_latency.py --duration 30 --verbose
   ```

4. Apply recommended VISO_DELAY_MS:
   ```bash
   # Via MAVProxy or QGroundControl
   param set VISO_DELAY_MS <recommended_value>
   param write
   ```

5. Validate fusion:
   ```bash
   ros2 topic echo /mavros/local_position/pose  # Should show smooth motion
   ```

---

## Dependencies

### Python Packages (ROS 2 Humble)
- `rclpy` (ROS 2 Python client library)
- `nav_msgs` (Odometry message type)
- `geometry_msgs` (PoseStamped message type)

### System Requirements
- ROS 2 Humble environment
- Active SLAM pipeline publishing to `/Odometry`
- MAVROS vision bridge publishing to `/mavros/vision_pose/pose`
- (Optional) MAVROS EKF publishing to `/mavros/local_position/pose`

---

## MCP Registration

The script is registered in the MCP diagnostic system:

**File:** `/home/dev/slam-agent/mcp/slam_mcp_server.py`

```python
DIAGNOSTIC_SCRIPTS = {
    ...
    "measure_vision_latency": ("measure_vision_latency.py", "python3"),
}
```

**Usage via MCP:**
```python
run_diagnostic("measure_vision_latency", "--duration 30 --json --container slam-gpu")
```

---

## Development Notes

### Why p95 instead of max?

Max latency includes outliers (GC pauses, CPU throttling, etc.). P95 provides a more reliable upper bound for typical system behavior while still being conservative.

### Why 10ms safety margin?

Accounts for:
- Clock jitter between systems
- Scheduler variance
- Network delays (if using remote MAVLink)
- MAVROS plugin processing overhead

### Message Matching Algorithm

Uses nearest-timestamp matching within a 200ms window:
1. For each received message, search the buffer for the closest-in-time predecessor
2. Compute time delta as latency
3. Discard matches >200ms apart (likely from different message pairs)
4. Filter latencies outside 0-500ms range (sanity check)

---

## Future Enhancements

- [ ] Add histogram visualization of latency distribution
- [ ] Support multiple measurement runs with statistical comparison
- [ ] Auto-detect optimal measurement duration based on topic publish rate
- [ ] Add support for ROS 1 Noetic (separate script variant)
- [ ] Export latency data to CSV for external analysis
- [ ] Real-time latency monitoring mode (continuous output)

---

**Last Updated:** 2026-02-16
**Author:** SLAM Integration Team
**Platform:** ROS 2 Humble, ARM64 Jetson Orin NX
