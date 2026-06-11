# SuperOdom Field Collection — Cheat Sheet (screenshot this)

You'll be **offline** (no WiFi / no agent) in the field. Plan: **record raw bags now, build maps + evaluate at home with the agent.** Nothing to debug live — you're just capturing sensor data.

## Before you leave WiFi
Optional — only if the rig might reboot in the field (otherwise skip):
```
# bags don't need SLAM compute, so STOCK clock is fine (saves battery)
```

## In the field — ONE command, then follow prompts
From a **local terminal on the Jetson** (screen, or phone-SSH on local hotspot):
```
~/slam-agent/superodom_integration/scripts/field_record.sh
```
It will:
1. check the sensor + start the Ouster driver,
2. ask for an **environment name**,
3. ask you to stand at your **START** point → press ENTER to record,
4. you **walk your route**, then press ENTER to stop,
5. **auto-check the take for sensor dropouts** → prints `VERDICT: ✓ CLEAN` or `⚠⚠ RE-RECORD`,
6. ask if you returned to start, then loop for the next spot.

## ⚠ The #1 thing that ruins a take: a sensor blackout
The Ouster sends LiDAR **and** IMU over **one ethernet cable**. If that link drops even for a second, the whole stream blacks out and **every SLAM framework diverges** when it resumes — the clip is dead past the gap.
- **Don't stretch the tether.** If you record on a stationary computer and walk away with the LiDAR, the ethernet cable will eventually yank loose. Either **stay within the cable's slack**, or **carry the compute (Jetson) on the rig** with the LiDAR + battery so the only cable is a short, taped-down LiDAR↔Jetson link.
- After each clip the recorder tells you `✓ CLEAN` or `⚠⚠ RE-RECORD`. **If it says RE-RECORD, re-walk that spot** — don't carry home a blacked-out bag.
- Check any existing bag yourself: `scripts/check_bag_gaps.py ~/superodom_ws/field/<name>`

## How to walk for good data
- **Walk a LOOP and return to the EXACT start point + facing.** Return-to-start = our drift metric (no GPS truth otherwise).
- Keep each clip **2–4 min** (~7–14 GB each; you have ~130 min of disk total).
- **Move smoothly**, especially turns — fast yaw is what LiDAR-inertial hates most.
- **Hit varied environments** (the point!): open field, vegetation, near buildings, transitions. Open/featureless = the hard case (like the corridor) — great to test.
- One environment per recording. Name them clearly ("openfield", "treeline", "parkinglot").

## Back home (with the agent)
Reconnect to WiFi and just say: **"process the field bags."** Or per bag:
```
scripts/process_field_bag.sh <env_name>   # SuperOdom  -> map.pcd + trajectory + metrics
scripts/run_fastlio.sh      <env_name>   # FAST-LIO   -> fastlio_map.pcd + trajectory + metrics
scripts/benchmark_bag.sh    <env_name>   # BOTH on the same bag + a comparison table
```
`benchmark_bag.sh` first validates the bag is dropout-free, then replays it through each framework
so you can compare drift / divergence / map sharpness apples-to-apples — the point of recording a
benchmark bag. **Boost the clock first** for clean results: `sudo ~/superodom_ws/set_cpu_clock.sh boost`.

Bags land in `~/superodom_ws/field/`. The recorder logs a summary to `~/superodom_ws/field/field_log.txt`.

## If something looks wrong in the field
- **`⛔ /ouster/points is NOT publishing`** (preflight) → the sensor isn't streaming (HTTP config can pass while the UDP data stream fails — often a just-power-cycled sensor or a dropped cable). Wait ~20s, check the LiDAR power/cable, re-run. The recorder **refuses to record into a dead stream** (no more silent empty bags).
- **`⛔ /ouster/points not flowing — skipping this clip`** → the sensor dropped between clips; fix the LiDAR, pick the spot again.
- "sensor NOT reachable" → check Ouster power + ethernet.
- After each clip you get a verdict: **`✓ CLEAN`** (good) or **`⚠⚠ RE-RECORD`** (empty *or* blacked-out → re-walk it). Trust the verdict — that's what makes a bag safe to benchmark.
