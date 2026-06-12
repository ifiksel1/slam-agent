# Foxglove Visualization Setup

## What It Is

Foxglove Studio is a free, browser-based robotics visualization tool (like RViz but runs on any device with a browser -- laptop, tablet, phone). It connects to your SLAM system over WebSocket and lets you visualize:
- Live 3D point clouds from your LiDAR
- SLAM odometry/trajectory in real-time
- TF tree, IMU data, diagnostics
- All without installing anything on the viewing device

It's especially useful for field testing -- you can monitor SLAM from your laptop while the drone is running, without needing X11 forwarding or a display attached to the companion computer.

## Setup by ROS Version

### ROS 2 (Humble/Jazzy)

Build the C++ `foxglove_bridge` from `foxglove/foxglove-sdk` repo (not the deprecated `ros-foxglove-bridge`):

- Dependencies: `rosx_introspection` (build from source), `rapidjson-dev`, `nlohmann-json3-dev`, `libasio-dev`
- Fix required: Add `find_package(sensor_msgs REQUIRED)` to foxglove_bridge CMakeLists.txt
- Service introspection errors are non-fatal -- topic bridging works fine
- Add to launch file with `enable_foxglove:=true` argument (default: true)
- Note: The Python `foxglove-websocket` 0.1.4 package is protocol-incompatible with modern Foxglove Studio -- always use the C++ bridge

### ROS 1 (Noetic)

Use `apt install ros-noetic-rosbridge-server` and Foxglove Studio's rosbridge connection mode.

### Docker

Expose port 8765 (already done if using host networking). Add to launch file so it starts automatically with SLAM.

## ⚠️ Whitelist topics — never stream the raw organized LiDAR cloud over a remote link

The single biggest "I connected but see no topics / nothing renders" cause: the bridge tries
to stream the **raw organized point cloud** (e.g. Ouster `/ouster/points`, 1024×64 ≈ **3 MB/msg
@20 Hz ≈ 480 Mbps**). Over Tailscale/wifi this instantly fills the websocket send queue (seen as
a multi-MB `Send-Q` on the server socket); the client stays "connected" but channel
advertisements and data never drain, so **the topic list looks empty**. The client is fine — the
link is saturated.

**Fix: restrict the bridge to the viz topics you actually need** via a params file:
```yaml
/**:
  ros__parameters:
    port: 8765
    address: 0.0.0.0
    topic_whitelist:                 # std::regex, full-match
      - "^/cloud_scan$"              # SLAM's processed/deskewed cloud — dense, NaN-free, small
      - "^/cloud_map$"               # accumulated map (set Foxglove Decay time:0 to keep it)
      - "^/ellipselio_path$"         # trajectory line
      - "^/ellipselio_odom$"
      - "^/tf$"
      - "^/tf_static$"
      - "^/clock$"                   # needed for use_sim_time / bag replay
    send_buffer_limit: 100000000
```
`ros2 run foxglove_bridge foxglove_bridge --ros-args --params-file foxglove_params.yaml`

Rules of thumb:
- **Stream the SLAM framework's processed cloud, not the raw sensor cloud.** No NaN warning (raw
  organized clouds carry NaN for every no-return beam — normal, but Foxglove warns).
- **Even the processed cloud may be too heavy for a remote link — throttle it.** EllipseLIO's
  `/cloud_scan` is the fat "original" Ouster point type: **48 bytes/pt × ~8k pts @20Hz ≈ 63 Mbps**,
  which still backs up Tailscale (~5 MB Send-Q, ~1 s lag). Throttle to ~5 Hz (`cloud_throttle.py`
  → `/cloud_scan_lite`, ~16 Mbps) and whitelist the throttled topic. The accumulating `/cloud_map`
  (Foxglove Decay 0) + `/ellipselio_path` are tiny and stay full-rate, so the live dense scan is
  the only thing that needs capping.
- A diagnostic check when "no topics" appears: on the host, `ss -tnp | grep :8765` — a large
  `Send-Q` to the client = link saturation, not a discovery problem.
- This is the Foxglove-transport twin of the Fast-DDS large-cloud lesson: big PointCloud2 messages
  break naive transports.

For **bag replay into Foxglove**, don't use `ros2 bag play --loop` with a persistent SLAM node —
sim-time jumps backward at loop restart and the node floods "time out of order" and stops
publishing. Instead loop by relaunching the node + replaying the bag once per cycle (see
`ellipselio_integration/scripts/foxglove_replay_loop.sh`).

## ⚠️ Start the bridge AFTER the node — BEST_EFFORT QoS auto-detection

Second "I connected but only see /tf" cause (distinct from the whitelist one): a **QoS
mismatch**. EllipseLIO (and most LIO) publish their viz topics **BEST_EFFORT**
(`rclcpp::SensorDataQoS()`); `/tf` is **RELIABLE**. `foxglove_bridge` auto-detects a topic's
QoS from a **live publisher** when it first subscribes. If the bridge starts **before** any
publisher exists (e.g. bridge launched first, or a looping demo that relaunches the node), it
defaults to **RELIABLE** and **silently drops every best-effort topic** — so you see only `/tf`
(the one reliable topic).

Diagnosis: `ros2 topic info -v /ellipselio_odom` shows `Reliability: BEST_EFFORT`; `/tf` shows
`RELIABLE`; and only `/tf` renders in Foxglove.

**Fix: start `foxglove_bridge` AFTER the node is publishing** (a few seconds into bag play /
after the live sensor is up), so it auto-detects BEST_EFFORT. This is the ordering
`run_ellipselio.sh` uses (node in step 3, bridge in step 5). For a looping bag demo, start the
bridge a few seconds into the **first** bag play, not before the first node — see
`coinlio_fusion_integration/scripts/foxglove_bag_demo.sh`. (Restarting just the bridge while the
publishers are live also fixes a session already in the broken state.)

Note: the foxglove-sdk C++ bridge has no `best_effort_qos_topic_whitelist` param (that was the
deprecated `ros-foxglove-bridge`); it relies on auto-detection, so ordering is the lever. The
`max_qos_depth` param (e.g. 25) silences the "Limiting history depth for /tf" warning.

## Control Script

Generate `scripts/foxglove.sh` with start/stop/restart/status/logs commands following the node control script pattern.

## User Instructions

Open Foxglove Studio at https://app.foxglove.dev, click "Open connection", enter `ws://<jetson-ip>:8765`.
