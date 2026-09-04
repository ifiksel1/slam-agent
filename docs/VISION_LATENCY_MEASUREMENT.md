# Vision Pipeline Latency Measurement Tool

**Purpose:** Measure latency in the SLAM-to-MAVROS vision pose pipeline and recommend an optimal `VISO_DELAY_MS` ArduPilot parameter.

## ⚠️ Pick the script that matches your ROS version

| Stack | Script | Notes |
|---|---|---|
| **ROS 1 Noetic** — Hesai JT128 / NUC (the *current* rig) | `scripts/measure_vision_latency_noetic.py` | Use this one. See [ROS 1 Noetic](#ros-1-noetic-hesai-jt128--fast-lio) below. |
| ROS 2 Humble — Jetson Orin / Ouster (legacy) | `scripts/measure_vision_latency.py` | `rclpy` only; **cannot run on Noetic**. Rest of this document describes it. |

The two use **different measurement methods and produce different numbers.** The ROS 2 script's method measures the wrong path on the Noetic rig — see [Why the methods differ](#why-the-methods-differ). Do not carry a `VISO_DELAY_MS` value between the two platforms.

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

## ROS 1 Noetic (Hesai JT128 / FAST-LIO)

**Script:** `scripts/measure_vision_latency_noetic.py`
**Platform:** Intel NUC, Docker container `slam-hesai-fastlio`, Hesai JT128 (128 lines @ 20 Hz, internal IMU), FAST-LIO on ROS 1 Noetic, ArduPilot on mRo Pixracer Pro over USB CDC-ACM `/dev/ttyACM0:921600`, ExternalNav (`EK3_SRC1_{POSXY,VELXY,YAW}=6`, `VISO_TYPE=1`).

Read-only: subscribers and nothing else. It never writes a param or commands the FC.

```bash
docker cp scripts/measure_vision_latency_noetic.py slam-hesai-fastlio:/tmp/
docker exec slam-hesai-fastlio bash -lc \
  'source /opt/ros/noetic/setup.bash; source /root/slam_ws/devel/setup.bash; \
   python3 /tmp/measure_vision_latency_noetic.py --duration 30'
```

`--json` emits an MCP-compatible object.

### Why the methods differ

`VISO_DELAY_MS` is defined by ArduPilot as the delay between the vision sensor **measuring** the position and the autopilot **receiving** the message.

`now() - header.stamp` at the **last** ROS hop directly measures the delay up to MAVLink serialisation. That works because the vision-pose stamp is exactly the instant the pose refers to:

- FAST-LIO propagates the state to `pcl_end_time` and deskews every point into the **end-of-scan** frame (`IMU_Processing.hpp:291-292`, `319-324`), and stamps `/Odometry` with that same `lidar_end_time` (`laserMapping.cpp:613`, `405`).
- `fastlio_mavros_bridge` runs `restamp_system_time: false`, so it forwards that stamp untouched (`fastlio_mavros_bridge.py:106`).

Deskew target and stamp therefore coincide, and the age at `/mavros/vision_pose/pose` is the delay against the true measurement instant.

> **Do not decompose this per-hop.** `header.stamp` does **not** propagate unchanged along the chain — the stamps are taken against *different reference instants*, so subtracting the ages is meaningless:
>
> - the driver stamps `/lidar_points` with `frame.points[0].timestamp` (`source_driver_ros1.hpp:247`) — the **first** point, i.e. sweep **start**;
> - FAST-LIO restamps `/Odometry` to `lidar_end_time` — sweep **end**, ~50 ms later at 20 Hz.
>
> An earlier revision of this document claimed the stamp was carried through unchanged and derived a "+17 ms FAST-LIO compute" term from the difference. That figure was wrong; the real transport-to-transport latency is ~70 ms (see below). The final `VISO_DELAY_MS` was unaffected, because it is measured end-to-end at the last hop and never relied on the decomposition.

The ROS 2 script instead takes `p95` of `/Odometry → /mavros/local_position/pose`. That is wrong here on two counts:

1. **It is a round trip.** `/mavros/local_position/pose` is the FC's *output* returning over telemetry, so it includes FC→host latency that is not part of the sensor delay — while missing the `scan → /lidar_points` segment entirely.
2. **`p95 + margin` is the wrong statistic.** `VISO_DELAY_MS` is a fixed constant the EKF uses to index its state-history buffer, so it wants the **central** value. Padding it high makes the EKF fuse each measurement against a too-old state. Over- and under-estimating are both harmful.

### Measured budget (2026-08-10, idle bench, 25 s, n≈495 per topic)

Ages, each against that topic's **own** stamp — not subtractable, see the warning above:

| Topic | Stamp reference | Median age | sd |
|---|---|---|---|
| `/lidar_points` | sweep **start** | 54.0 ms | 2.2 ms |
| `/Odometry` | sweep **end** | 73.5 ms | 4.5 ms |
| `/mavros/vision_pose/pose` | sweep **end** (forwarded) | 73.9 ms | 4.5 ms |

Derived by pairing each `/Odometry` message with the cloud it came from:

| Interval | Median |
|---|---|
| Sweep duration (`odom stamp − cloud stamp`) | 50.6 ms |
| Cloud assembly beyond the sweep | 3.5 ms |
| FAST-LIO transport-to-transport (`odom recv − cloud recv`) | 69.8 ms |
| EKF solve alone (`/fastlio_health` field 2) | 6–10 ms |

**`VISO_DELAY_MS` = age at `/mavros/vision_pose/pose` + ~1.5 ms USB CDC-ACM transit = 73.9 + 1.5 ≈ 75 ms.** Independently re-measured 2026-08-10 and unchanged from the 2026-08-09 value of 75 (was 60 before that).

#### The ~70 ms FAST-LIO term is latency, not a throughput deficit

Measured rates: `/lidar_points` 19.95 Hz, `/Odometry` 19.94 Hz, `/mavros/vision_pose/pose` 19.93 Hz. **No scans are being dropped**, and the age is stable over the run (sd 4.5 ms, no upward drift) — so per-scan CPU is comfortably under the 50 ms scan period, consistent with the 6–10 ms EKF solve.

A stable ~70 ms latency at 20 Hz with matched throughput is the signature of a standing **one-scan backlog**: ~50 ms of queue plus ~20 ms of preprocessing, solve and transport. If that is right, draining the backlog would remove ~50 ms of control latency — worth investigating, but unconfirmed. It does not affect `VISO_DELAY_MS`, which measures the delay as it actually is.

### Limitations

- **The result is a lower bound.** `header.stamp` is the host's *packet parse* time (`udp1_4_parser.cc:274`, `use_timestamp_type: 1`), not the true scan instant, so LiDAR-internal acquisition and ethernet transit fall outside the measurement.
- ~~Deskew convention adds up to ±25 ms of uncertainty~~ — **resolved, no longer a caveat.** FAST-LIO deskews to sweep end *and* stamps at sweep end (`IMU_Processing.hpp:319-324`, `laserMapping.cpp:613`), so the two agree exactly and no ambiguity remains. This had been listed as the largest single uncertainty.
- **Validate in motion, handheld and disarmed** — never as a flight test. Watch whether FC position innovation correlates with velocity; if it grows with speed, step the value and re-check.
- **Re-measure after any FAST-LIO tuning change.** The ~17 ms compute term moves with `point_filter_num`, `max_iteration`, and the voxel filter sizes, and idle bench CPU is less contended than flight.

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
- [x] Add support for ROS 1 Noetic (separate script variant) — `measure_vision_latency_noetic.py`
- [ ] Export latency data to CSV for external analysis
- [ ] Real-time latency monitoring mode (continuous output)

---

**Last Updated:** 2026-08-09
**Author:** SLAM Integration Team
**Platforms:** ROS 2 Humble / ARM64 Jetson Orin NX (legacy) — ROS 1 Noetic / x86 NUC + Hesai JT128 (current)
