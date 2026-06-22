---
name: lidar-field-recording-dropout
description: "A simultaneous gap in BOTH Ouster LiDAR and IMU topics = the single ethernet link dropped (tether stretched while walking); it blacks out the sensor and diverges every SLAM framework — validate field bags before benchmarking"
metadata:
  node_type: memory
  type: feedback
  originSessionId: 957e3751-78b0-4a42-987f-34e5eb506fb3
---

The #1 thing that silently ruins a SLAM benchmark/tuning bag on the Ouster rig is a **sensor blackout during recording**, not the SLAM algorithm. Confirmed 2026-06-11 on the user's first real field bag (`mixed_route`): a **7.8-second gap appeared in BOTH `/ouster/points` and `/ouster/imu` simultaneously at t=76.3s**, and every framework (SuperOdom AND FAST-LIO) diverged catastrophically right when data resumed.

**Diagnosis = both topics gap together.** The Ouster OS1 sends LiDAR and IMU over **one ethernet cable**. A simultaneous identical gap in both ⇒ the single ethernet link went down (link renegotiation + sensor stream restart), not packet loss on one stream. (Query the db3 directly: `select timestamp from messages where topic_id=… order by timestamp`, diff consecutive — a real blackout is a >0.3s gap; normal jitter is <0.15s for 20Hz points, <0.05s for ~95Hz IMU.)

**The user's setup that caused it:** they **start recording on a stationary computer, then walk away carrying only the LiDAR** (powered by a battery, but DATA still tethered by ethernet back to the computer). As they walk, the cable runs out of slack / jostles at the connector → link drops for several seconds → blackout. Why it's fatal: during the blackout the estimator dead-reckons on nothing (IMU also gone); when data resumes the operator has moved several meters, scan-to-map association fails → divergence. **Unavoidable for ANY odometry — fix the recording, not the SLAM.**

**Fixes / how to apply:**
- **Don't stretch the tether.** Either stay within the ethernet cable's slack, OR **carry the compute (Jetson Orin NX) on the rig** with the LiDAR + battery (Orin NX runs on ~25W from a USB-C PD bank) so the only cable is a short, taped-down LiDAR↔Jetson link. The handheld-rig approach is the real answer for walking into varied environments.
- **Validate every bag before benchmarking:** `superodom_integration/scripts/check_bag_gaps.py <bag_dir>` (pure stdlib sqlite, no ROS) → prints `✓ CLEAN` or `⚠⚠ RE-RECORD` with the dropout timestamp. Exit 0/1.
- `field_record.sh` now **auto-runs this check after each clip** (the `check_gaps` function) and prints CLEAN/RE-RECORD on the spot + logs `gaps:` to `field_log.txt`, so a blacked-out take is caught in the field instead of at home.
- For a benchmark bag you don't need to traverse a building — a clean LOOP within cable reach that returns to the exact start (for the drift metric) is ideal.

**Sibling failure — the EMPTY bag (2026-06-11):** a field clip came back **0 messages / 0 topics** because the Ouster **driver failed to init** while recording: `init_client(): A timeout occurred while waiting for the sensor to initialize`. The sensor's HTTP config SUCCEEDS ("configured successfully") but the **UDP data stream never starts** — usually transient (sensor just power-cycled on the field battery, or the tether was momentarily down). Pinging the sensor / having host IP 192.168.2.50 on eth0 is NOT enough; the data stream must actually flow. **Fix = just restart the driver once the sensor settles** (re-running the launch worked immediately, 20Hz). Hardened `field_record.sh`: preflight now **hard-exits** if `/ouster/points` isn't publishing, and re-checks the live rate **before each clip** (the sensor can drop between clips) — no more silent empty bags. The gap check (`check_bag_gaps.py` + inline) now flags `<10 msgs` as `⚠⚠ RE-RECORD — NO sensor data` instead of falsely passing it as CLEAN.

Related: [[new-slam-frameworks-2026-06]] (the FAST-LIO vs SuperOdom comparison this came out of), [[jetson-cpu-clock-boost]].
