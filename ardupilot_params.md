# ArduPilot Parameter Access via MAVROS ROS 2

## Key Discovery (2026-02-21)

MAVROS ROS 2 (2.x) uses a **different API** than ROS 1 for ArduPilot parameters.

### Service Types (NOT what you'd expect)

The `/mavros/param` node exposes:
- `/mavros/param/pull` — type: `mavros_msgs/srv/ParamPull`
- `/mavros/param/set` — type: **`mavros_msgs/srv/ParamSetV2`** (NOT `ParamSet`)
- `/mavros/param/get` — **listed in `ros2 service list` but NOT in node info** — unreliable, don't use
- `/mavros/param/event` — topic: `mavros_msgs/msg/ParamEvent` (publishes during pull)

**Note:** ArduPilot does not use DDS — it communicates purely via MAVLink over serial (unlike PX4 which supports DDS/micro-ROS). The DDS issues below are purely ROS 2 inter-process (FastRTPS between `ros2 service call` CLI and the MAVROS node).

### The Working Approach: ROS 2 Parameter API

After a param pull, all ArduPilot params are accessible as **ROS 2 node parameters** on `/mavros/param`:

```bash
# Pull all params first (takes ~60s for 1099 params)
ros2 service call /mavros/param/pull mavros_msgs/srv/ParamPull "{force_pull: true}"
# Wait for param_index=65535 on /mavros/param/event to confirm completion

# Read params via ROS 2 param API
ros2 param get /mavros/param AHRS_EKF_TYPE
ros2 param list /mavros/param | grep SR

# Set params via ParamSetV2
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 \
  "{force_set: true, param_id: VISO_DELAY_MS, value: {type: 2, integer_value: 60}}"
```

### What Does NOT Work

- `ros2 service call /mavros/param/get mavros_msgs/srv/ParamGet` — service exists in listing but never becomes available to clients (DDS discovery issue with FastRTPS). Always times out with `rcl node's context is invalid`.
- Setting params before pull completes — `ParamSetV2` returns `success=False` if the param hasn't been pulled yet.

### Param Pull Details

- ArduPilot Copter has ~1099 parameters
- Pull takes ~60 seconds over USB serial
- Monitor progress: `ros2 topic echo /mavros/param/event --once`
  - `param_index: 345` = still pulling (345 of 1099)
  - `param_index: 65535` = pull complete (sentinel value)
- Pull must complete before get/set works

### Stream Rate Parameters (SR0_*)

**ArduPilot Copter 4.5+ removed `SR0_*` parameters entirely.**
- `ros2 param list /mavros/param | grep SR` returns only `SRTL_*` (SmartRTL), no stream rates
- Stream rates are controlled ONLY via MAVLink `REQUEST_DATA_STREAM` command
- Use: `ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 50, on_off: true}"`
- LP rate is controlled by `SERIAL0_BAUD` on the FC, NOT by stream rate commands
  - `SERIAL0_BAUD=115` (default) → LP caps at 5-12.5Hz
  - `SERIAL0_BAUD=921` (921600) → LP at 20Hz+ achievable
  - Must reboot FC after changing: `ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong "{command: 246, param1: 1}"`
  - Then restart MAVROS (FC reboot causes USB re-enumeration)
- `set_stream_rate` and `MAV_CMD_SET_MESSAGE_INTERVAL` (511) are accepted but effectively ignored at low baud rates
- VP rate is limited by SLAM odometry input — FAST-LIO publishes once per LiDAR scan

### Environment Variables Required

Always set these when running `ros2` commands inside the MAVROS container:
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=1
source /opt/ros/humble/setup.bash
```

### Container Context

All commands run inside `slam_mavros` container:
```bash
docker exec slam_mavros bash -c '
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=1
source /opt/ros/humble/setup.bash
ros2 param get /mavros/param AHRS_EKF_TYPE'
```
