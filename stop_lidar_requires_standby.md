---
name: ""
metadata: 
  node_type: memory
  originSessionId: 99071776-fedd-469f-8232-e61ce099b071
---

To actually "stop the lidar" you must put the **physical Ouster sensor** into STANDBY — killing the ROS driver process (`ouster.sh stop`, any `stop_lidar`) does NOT stop the hardware; the sensor keeps spinning and lasing as long as it has PoE power.

**Why:** The driver is just a network client. The sensor (e.g. `169.254.56.220`, default `operating_mode: NORMAL`, `status: RUNNING`) runs independently. Two misleading signals make a driver-only stop look like the lidar "came back": (1) `ros2 launch` (container PID 1, slam_launch.py) can respawn the driver node; (2) `/ouster/points` still appears in `ros2 topic list` even with **Publisher count: 0** as long as a subscriber (FAST-LIO) is alive — check `ros2 topic info <topic>` publisher count, not topic presence.

**How to apply:**
- Stop: `curl -s -X POST http://<sensor_ip>/api/v1/sensor/config -H "Content-Type: application/json" -d '{"operating_mode":"STANDBY"}'` then poll `http://<sensor_ip>/api/v1/sensor/metadata/sensor_info` until `status: STANDBY`.
- Resume: same POST with `{"operating_mode":"NORMAL"}`, then restart the driver / `docker restart slam_gpu_system`.
- Sensor IP lives in `config/ouster_driver.yaml` (`sensor_hostname`). Driver runs inside container `slam_gpu_system` as part of `slam_launch.py` (lifecycle include), workspace `/home/dev/slam-gpu`.
