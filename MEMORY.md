# Claude Code Memory - SLAM Agent

## Code Work Model Policy
- **Use Sonnet 4.5** for code writing/debugging/refactoring (via Task tool delegation)
- **Only use Haiku** for trivial tasks or code reading
- This is automatic, no need to request each time

---

## Active Projects
- [FAST-LIVO2 camera selection](fast_livo2_camera_selection.md) — Orin NX on CTI Hadron-DM carrier; camera choice blocked on Connect Tech external-trigger answer (emailed 2026-06-06)
- [New SLAM frameworks + PX4 migration](new_slam_frameworks_2026_06.md) — EllipseLIO + SuperOdom added to the agent (un-validated, need Phase 5); ArduPilot→PX4 migration scoped for MRS UAV stack
- [Hangar 737 inspection architecture](hangar_737_inspection.md) — THE target mission: autonomous cm-pose drone inspection of a 737 in a fixed hangar; decided architecture = LIO + absolute anchor (fiducials / prior-map relocalization) + degeneracy gating, NOT appearance loop closure
- [BIEVR-LIO evaluation](bievr_lio_evaluation.md) — ethz-asl RSS2026 degenerate-geometry LIO, built native-arm64 on Orin NX (ROS1 Noetic Docker). **2026-07-11: on the hard 3rd_floor bag = 1.35cm return-to-origin (vs COIN-LIO ~3.8m) AND byte-identical across 8 multi-thread + 1 single-thread runs → fully deterministic, the OPPOSITE of EllipseLIO. Strong hangar candidate; may replace the EllipseLIO base.** Bag-validated only; live/MAVROS/flight pending
- [IMU Allan-variance calibration](imu_allan_variance_calibration.md) — 3h static Allan cal of Ouster OS1-64 IMU (2026-07-13): only EllipseLIO consumes it; hand-set config over-trusted IMU (accel 15x, gyro 2x); calibrated `os1_64_ouster_allan.yaml`; A/B on 3rd_floor = NULL (20% vs 30% runaway, n=10, noise) — runaways are threading/FP-order not IMU prior; keep cal anyway (accuracy-neutral)
- [IRIS-LIO (COIN-LIO ⊕ EllipseLIO photometric fusion)](coinlio_ellipselio_photometric_fusion.md) — LiDAR-intensity photometric residual fused into EllipseLIO's IKFoM. v1 stable+neutral on apartment (photo_scale=1e-9). **KEY REFRAME (2026-06-22): on the hard 3rd_floor bag, UPSTREAM COIN-LIO is rock-stable (0/6 runaway, deterministic ~3.8m loop closure) where EllipseLIO-base is 17% runaway and the photometric graft is 67% → the instability is the EllipseLIO BASE estimator / the graft, NOT the intensity-fusion idea.** Decouple was functionally done but efficacy NEGATIVE. Branch feat/coinlio-photometric-fusion, container coinlio_fusion; new ROS1 harness slam-agent/coinlio_integration/

---

## Critical Lessons

### Topic Configuration (2026-02-07)
SLAM config hardcoded to `/os_cloud_node/points` but Ouster publishes `/ouster/points`. Silent failure. Always verify topics FIRST: `rostopic list | grep ouster`

### OpenCV 4.2/4.5 ABI Conflict (2026-02-22)
**See:** `opencv_abi_conflict.md` for full details
- ROS Noetic cv_bridge loads OpenCV 4.2, system has 4.5 → `setSize` crash in `cv::filter2D`
- **Fix:** LD_PRELOAD in launch file + link OpenCV before catkin in CMakeLists.txt
- Affects any ROS Noetic package using OpenCV on Jetson

### Ouster ROS Driver ARM64 Crash (2026-02-09)
Ouster ROS 1 driver nodelets crash with exit 255 on Jetson Orin NX (ARM64).
**Workaround:** Use pre-built driver from `ultraviewdev_ws` which works on ARM64.

---

## Workspaces

### COIN-LIO (`~/coinlio_ws/`) - 2026-02-22
**See:** `coinlio_workspace.md` for full details
- ETH Zurich LiDAR-Inertial Odometry with intensity fusion
- Upgraded: ARM64 threading, C++17, OpenCV fix, vision bridge, Ouster v0.13.9
- Full pipeline tested: Ouster → COIN-LIO (20Hz) → vision_bridge → MAVROS → CubeOrange
- GitHub: `ifiksel1/coin-lio`, `ifiksel1/LiDAR_IMU_Init`, `ifiksel1/ouster-ros`

### FAST-GPU ROS 2 (`~/slam-gpu/`) - 2026-02-14
Docker image `slam-gpu:latest` (16.3GB, ARM64) with:
- Ouster ROS 2 driver + FAST_LIO_GPU (CUDA SM 8.7)
- MAVROS deferred (version mismatch)
- Key build fixes: pcl_ros→tf2_ros, PCL filters component, GeographicLib order

### ROS 2 Full Support Plan (2026-02-12)
Plan at `/home/dev/.claude/plans/sorted-dazzling-patterson.md`
- Adds ROS 2 Humble + Jazzy support with full feature parity
- 17 files, 3-4 weeks implementation

---

## Docker SLAM Automation (2026-02-09)
SLAM Launcher system for Docker service management:
- `/home/dev/slam_ws/scripts/slam_launcher.py` (MCP interface)
- `/home/dev/slam_ws/scripts/start_slam_system_bg.sh` (service management)
- Commands: `verify`, `status`, `start`, `stop`, `restart`

---

## Key Patterns

### Sudo Multi-Step Commands
Create shell scripts for multi-step sudo commands instead of individual Bash tool calls.

### MCP Server Python Isolation (2026-02-14)
Use `python3.12 -I` (isolated mode) in `mcp/run_mcp_server.sh` to prevent ROS PYTHONPATH conflicts with modern `attrs`.

### SLAM Node Control Scripts (2026-02-14)
Generate per-node bash control scripts (`start|stop|restart|status|logs`). Template: `/home/dev/slam-gpu/scripts/fastlio.sh`

### "Stop the lidar" = sensor STANDBY, not driver kill (2026-06-14)
**See:** `stop_lidar_requires_standby.md`
- Killing the ROS driver does NOT stop the Ouster hardware (keeps spinning/lasing on PoE). Stop = POST `{"operating_mode":"STANDBY"}` to `http://<sensor_ip>/api/v1/sensor/config`; resume = `NORMAL`. Sensor IP in `config/ouster_driver.yaml`
- Don't trust topic presence: `/ouster/points` lingers in `topic list` with Publisher count 0 while FAST-LIO subscribes — check `ros2 topic info` publisher count

### LiDAR Field-Recording Dropout (2026-06-11)
**See:** `lidar_field_recording_dropout.md`
- Simultaneous gap in BOTH `/ouster/points` and `/ouster/imu` = the single ethernet link dropped (tether stretched while walking) → sensor blackout → every SLAM diverges
- Validate field bags FIRST: `scripts/check_bag_gaps.py <bag>`; `field_record.sh` now auto-checks each take (CLEAN/RE-RECORD)
- Fix: carry the Jetson on the rig (no long tether) or stay within cable slack
- FAST-LIO beat SuperOdom on real handheld data (smoother, deterministic) — see `new_slam_frameworks_2026_06.md`; harness: `run_fastlio.sh`, `benchmark_bag.sh`

### Clean DDS Slate + EllipseLIO is Inherently Non-Deterministic (2026-06-13)
**See:** `clean_slate_between_repeated_runs.md`
- Repeated benchmark/A-B/determinism runs MUST `docker restart <ctr>` (orchestrate from HOST) + verify `ros2 node list` empty BEFORE each run — removes a stale-node/DDS contamination confound (necessary for a valid comparison)
- BUT clean slates are NOT sufficient: prod EllipseLIO with verified-empty slate per run, boosted clock, 14 runs on 3rd_floor (rate 1.0) is a near COIN-FLIP — 7/14 tight (~0.02m), **6/14 BLEW UP >5m (up to 302m)**. Inherent non-determinism — likely multi-threaded FP reduction order (`component_container_mt` + parallel tensor-voting) tipping a knife-edge spot, NOT zombies (tested+refuted) and NOT scan-drop (sample counts didn't track drift)
- CONSEQUENCE: every single-run SLAM number on this bag is meaningless (4-way table, neutrality test). Base-estimator determinism must be fixed before any IRIS-LIO/photometric benefit is measurable. Next probes: `--rate 0.5`, SingleThreadedExecutor

### ROS Node Teardown — never `ps -o comm` (2026-06-13)
**See:** `ros_node_teardown_comm_truncation.md`
- `ps -o comm` truncates to 15 chars → `component_container_mt` becomes `component_conta`, so an `awk /component_container/` kill silently matches nothing → leaked duplicate estimators (2+ publishers on `/ellipselio_odom`, jittery pose that looks like a "zombie")
- Reap per-cycle nodes by PID (`LAUNCH_PID=$!` → `kill -INT` + `pkill -P`); `pkill -f` self-kills the caller if the pattern is in its own cmdline
- Verify replay loops with `ros2 topic info <odom>` → Publisher count must be 1. Fixed runner: `run_iris_lio_foxglove.sh`

### CPU Clock Boost for Accuracy (2026-06-10)
**See:** `jetson_cpu_clock_boost.md`
- Orin NX: boost CPU 1497→1984MHz costs only ~3°C, gives ~20% compute margin → glitch-free SLAM
- Stock 25W saturates feature_extraction → pose glitches; boost (or drop to 512×20) fixes it
- `sudo ~/superodom_ws/set_cpu_clock.sh boost|stock` (non-persistent). MAXN broken on 6-core module.

### LiDAR Mode Tuning (2026-02-21)
**See:** `lidar_mode_tuning.md`
- Jetson Orin NX 8GB: `512x20` for FAST-LIO (no drops), `1024x20` drops ~30%
- With matched filtering (point_filter_num:4, filter_size:0.5), `1024x20` works at 20Hz for both

### COIN-LIO vs FAST-LIO GPU (2026-02-22)
**See:** `slam_comparison.md`
- Both run 20Hz with matched filters on Ouster OS1-64
- FAST-LIO GPU needs `point_type: original` in ouster_driver.yaml (not `xyzi`)
- ROS 2 Ouster driver needs 180-deg yaw extrinsic `[-1,0,0, 0,-1,0, 0,0,1]`
- ROS 1 driver uses identity extrinsic — bag conversion doesn't change frame convention

### ArduPilot VISO Parameter Tuning (2026-02-16)
**See:** `ardupilot_viso_tuning.md`
- Measure latency with `measure_vision_latency`, set VISO_DELAY_MS to p95+10ms

### ArduPilot Parameter Access via MAVROS ROS 2 (2026-02-21)
**See:** `ardupilot_params.md`
- `/mavros/param/get` is BROKEN — use `ros2 param get` after ParamPull
- Copter 4.5+ has no SR0_* params — use `set_stream_rate` service

### MAVROS Sidecar Container (2026-02-15)
Use official `ros:humble` image for MAVROS (apt install works cleanly).
Critical: Use XML `apm.launch` via `AnyLaunchDescriptionSource`, not direct Node() — avoids plugin topic collisions.
Files: `/home/dev/slam-gpu/mavros/`

### Foxglove Bridge for ROS 2 (2026-02-14)
Build C++ `foxglove_bridge` from `foxglove-sdk` repo. Need `rosx_introspection` from source.
Add `find_package(sensor_msgs REQUIRED)` to CMakeLists.txt line 125.

### Jetson ARM64 Package Management
Use Miniforge (conda) instead of PPAs — Deadsnakes and others don't support ARM64.
