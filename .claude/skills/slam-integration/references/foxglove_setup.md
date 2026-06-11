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
- **Stream the SLAM framework's processed cloud (`/cloud_scan`), not the raw sensor cloud.** Same
  view, a fraction of the bytes, and no NaN warning (raw organized clouds carry NaN for every
  no-return beam — normal, but Foxglove warns).
- A diagnostic check when "no topics" appears: on the host, `ss -tnp | grep :8765` — a large
  `Send-Q` to the client = link saturation, not a discovery problem.
- This is the Foxglove-transport twin of the Fast-DDS large-cloud lesson: big PointCloud2 messages
  break naive transports.

For **bag replay into Foxglove**, don't use `ros2 bag play --loop` with a persistent SLAM node —
sim-time jumps backward at loop restart and the node floods "time out of order" and stops
publishing. Instead loop by relaunching the node + replaying the bag once per cycle (see
`ellipselio_integration/scripts/foxglove_replay_loop.sh`).

## Control Script

Generate `scripts/foxglove.sh` with start/stop/restart/status/logs commands following the node control script pattern.

## User Instructions

Open Foxglove Studio at https://app.foxglove.dev, click "Open connection", enter `ws://<jetson-ip>:8765`.
