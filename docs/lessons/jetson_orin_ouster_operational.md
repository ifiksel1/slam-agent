# Jetson Orin NX + Ouster OS1-64 — operational lessons

Cross-cutting lessons that apply to **every** LIO framework on this rig (EllipseLIO,
SuperOdom, FAST-LIO). Rig facts: Orin NX **8GB / 6-core**, CTI Hadron carrier, host
**Ubuntu 20.04 / ROS Noetic only** (all ROS 2 runs in Docker). Ouster OS1-64-U13 fw 2.4.0,
sensor `192.168.2.60`, udp_dest/host `192.168.2.50`, `1024x20` LEGACY (edit per network).

---

## 1. Use CycloneDDS, not Fast-DDS (the single most important runtime knob)

Default **Fast-DDS (`rmw_fastrtps_cpp`) stalls the large `/ouster/points` PointCloud2
publish to ~15 Hz** whenever several LIO DDS participants are alive. **CycloneDDS holds a
steady 20 Hz.** Baked into both integration images as `ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`;
the run scripts also export it for the driver **and** the SLAM nodes (they must match to interoperate).

How it was isolated (2026-06-09, every compute hypothesis ruled out by direct live test):
RAM (24% used) → no; single-core saturation (driver on a dedicated core, still ~17 Hz) → no;
UDP buffer (0 live drops) → no; `ros2 topic hz` artifact → no; memory bandwidth (`EMC 0%`) → no;
thermal (66–70 °C) → no; CPU clock (raised to 1984 MHz, util dropped, rate still ~15 Hz) → no.
**Decisive:** raw `/ouster/lidar_packets` held 1234 Hz rock-steady under load while assembled
`/ouster/points` collapsed → the loss is in the driver's large-PointCloud2 assembly/publish
path under Fast-DDS multi-participant contention. `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` →
`/ouster/points` 20.0 Hz steady (std 0.011 s).

---

## 2. Boost the CPU clock for accurate odometry (compute margin, not thermal, is the limit)

Boosting the Orin NX max clock from the stock 25 W cap (1497600 kHz) to silicon max
(1984000 kHz) costs only **~+3 °C** (load is ~2.3 of 6 cores; ~25 °C headroom either way)
and is the user-preferred way to get accurate, glitch-free SLAM when a node is CPU-bound.

SuperOdom live A/B, same hand-carried loop:

| | stock 1497 MHz | boost 1984 MHz |
|---|---|---|
| feature_extraction load | 100–107% (**core saturated**) | ~80% (~20% headroom) |
| loop closure (20–30 m walk) | 0.17 m | **0.07 m** |
| max single-scan pose step | **1.10 m glitch** (~27 m/s, non-physical) | 0.08 m (clean) |
| vertical (z) drift over loop | **−0.78 m, never recovered** | −0.03 m |

z-drift at stock is a third symptom of the *same* starvation, not a z weakness — a saturated
core can't converge the per-scan optimization, so even the well-constrained vertical axis drifts.

**How to apply**
- Prefer **boost for flight/accuracy** when a node is CPU-bound. If you must stay at 25 W,
  instead drop the LiDAR mode (1024x20 → 512x20) to halve per-scan compute (see §3).
- Set it: `sudo ~/superodom_ws/set_cpu_clock.sh boost|stock` (writes `scaling_max_freq` on all
  6 cores; **non-persistent**, reverts on reboot — needs sudo, the agent can't run it).
- **`nvpmodel -m 0` (MAXN) is BROKEN on this 6-core 8GB module** — stock conf references
  CORE_6/7 → `NVPM ERROR cpu6/online`. Use the sysfs override, not MAXN. (Modes: 0=MAXN broken,
  1=10W, 2=15W, 3=25W.)

---

## 3. Match LiDAR mode to compute (per-scan processing must fit the inter-scan interval)

FAST-LIO processes each scan sequentially; if processing exceeds the scan interval, frames drop.

| Mode | Points/scan | Scan rate | FAST-LIO output | Notes |
|------|-----------|-----------|-----------------|-------|
| 1024x10 | ~65K | 10 Hz | 10 Hz | No drops, low update rate |
| 1024x20 | ~32K | 20 Hz | ~14 Hz | Drops ~30%, but stable tracking, no drift |
| 512x20  | ~16K | 20 Hz | 20 Hz | No drops, but too sparse for small rooms → tracking loss / 60 m+ jumps |

Tune by: start high (1024x20) → check `ros2 topic hz /Odometry` inside the container → if
dropping, halve horizontal res and retest → target output rate == scan rate. With matched
filtering (`point_filter_num: 4`, `filter_size: 0.5`) 1024x20 works at 20 Hz on this rig.
**FAST-LIO publishes once per LiDAR scan (LiDAR-locked), not at IMU rate.**

---

## 4. Validate every field bag for sensor dropouts BEFORE benchmarking

The #1 thing that silently ruins a benchmark bag is a **sensor blackout during recording**,
not the SLAM algorithm. The Ouster OS1 sends LiDAR **and** IMU over **one ethernet cable**, so
a **simultaneous identical gap in BOTH `/ouster/points` and `/ouster/imu`** = the link dropped
(not packet loss). Every framework dead-reckons on nothing during the gap and diverges when
data resumes several meters later. **Fix the recording, not the SLAM.**

Caused by: start recording on a stationary computer, then walk away carrying only the LiDAR
while data is still tethered by ethernet → cable runs out of slack → link drops for seconds.

**How to apply**
- Don't stretch the tether — stay within slack, or **carry the Jetson on the rig** (runs on
  ~25 W from USB-C PD) so the only cable is a short LiDAR↔Jetson link.
- Validate before trusting: `superodom_integration/scripts/check_bag_gaps.py <bag_dir>` (pure
  stdlib sqlite, no ROS) → `✓ CLEAN` or `⚠⚠ RE-RECORD` with the dropout timestamp. A real
  blackout is a >0.3 s gap; normal jitter is <0.15 s (20 Hz points) / <0.05 s (~95 Hz IMU).
  `<10 msgs` is flagged `RE-RECORD — NO sensor data`, not falsely passed as CLEAN.
- `field_record.sh` auto-runs the check after each clip and re-checks the live rate before each
  clip (the sensor can drop between clips → silent empty bags otherwise).
- **Empty-bag sibling failure:** the driver can report "configured successfully" while the UDP
  data stream never starts (`init_client(): timeout waiting for sensor to initialize`) — usually
  transient (sensor power-cycled on field battery). Pinging the sensor isn't enough; the data
  stream must flow. Fix = restart the driver once the sensor settles.

---

## 5. "Stop the LiDAR" means sensor STANDBY, not killing the driver

Killing the ROS driver does **not** stop the hardware — the Ouster keeps spinning/lasing as
long as it has PoE power. The driver is just a network client. Two misleading signals: (1)
`ros2 launch` can respawn the driver node; (2) `/ouster/points` lingers in `ros2 topic list`
with **Publisher count: 0** while a subscriber is alive — check `ros2 topic info <topic>`
publisher count, not topic presence.

- **Stop:** `curl -s -X POST http://<sensor_ip>/api/v1/sensor/config -H "Content-Type: application/json" -d '{"operating_mode":"STANDBY"}'`,
  then poll `…/api/v1/sensor/metadata/sensor_info` until `status: STANDBY`.
- **Resume:** same POST with `{"operating_mode":"NORMAL"}`, then restart the driver.

---

## 6. Per-cycle ROS 2 node teardown — reap by PID, never `ps -o comm`

In replay-loop / benchmark scripts that relaunch a node each cycle, tearing down via
`ps -eo pid,comm | awk '/component_container/{print $1}'` **silently fails**: `ps -o comm`
truncates to 15 chars, so `component_container_mt` shows as `component_conta` and the regex
never matches → the old node is never killed → every cycle stacks another → 2+ estimators
publishing the same topics (e.g. `/ellipselio_odom` Publisher count > 1) → garbled/jittery
pose that *looks* like a "zombie process."

**How to apply**
- Per-cycle teardown: capture `LAUNCH_PID=$!`, then reap by PID:
  `kill -INT $LAUNCH_PID; sleep 3; pkill -9 -P $LAUNCH_PID; kill -9 $LAUNCH_PID`.
- Never `pkill -f <pattern>` where the pattern also appears in your own kill command (self-match
  → exit 137). Use `ps -o args` (full cmdline), never `ps -o comm`, when matching node processes.
- **Verify after any replay loop:** `ros2 topic info <odom_topic>` → Publisher count must be 1.
- Repeated A/B or determinism runs need a clean DDS slate per run: `docker restart <ctr>`
  (orchestrate from the **host** — you can't `docker restart` from inside the container being
  restarted) → wait → verify `ros2 node list` is EMPTY before relaunching. See the
  non-determinism section of [ellipselio_superodom_findings.md](ellipselio_superodom_findings.md).
