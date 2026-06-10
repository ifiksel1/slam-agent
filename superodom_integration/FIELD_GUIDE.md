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
5. ask if you returned to start, then loop for the next spot.

## How to walk for good data
- **Walk a LOOP and return to the EXACT start point + facing.** Return-to-start = our drift metric (no GPS truth otherwise).
- Keep each clip **2–4 min** (~7–14 GB each; you have ~130 min of disk total).
- **Move smoothly**, especially turns — fast yaw is what LiDAR-inertial hates most.
- **Hit varied environments** (the point!): open field, vegetation, near buildings, transitions. Open/featureless = the hard case (like the corridor) — great to test.
- One environment per recording. Name them clearly ("openfield", "treeline", "parkinglot").

## Back home (with the agent)
Reconnect to WiFi and just say: **"process the field bags."** Or per bag:
```
~/slam-agent/superodom_integration/scripts/process_field_bag.sh <env_name>   # the folder name
```
→ builds `map.pcd` (open in CloudCompare/meshlab), records the trajectory, and reports
loop-closure drift + any divergence. We can also run EllipseLIO on the same bags to compare.

Bags land in `~/superodom_ws/field/`. The recorder logs a summary to `~/superodom_ws/field/field_log.txt`.

## If something looks wrong in the field
- "sensor NOT reachable" → check Ouster power + ethernet.
- "recorder did not start" → re-run the script (it restarts the driver).
- That's it — if it says **● RECORDING**, it's capturing. Trust it; we evaluate at home.
