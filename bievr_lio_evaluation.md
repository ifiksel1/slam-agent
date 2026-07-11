---
name: bievr_lio_evaluation
description: "BIEVR-LIO (ethz-asl, RSS 2026) built native-arm64 on Orin NX and is deterministic + cm-accurate on the hard 3rd_floor bag — strong hangar candidate, sidesteps EllipseLIO nondeterminism"
metadata: 
  node_type: memory
  type: project
  originSessionId: d7eb90d0-8945-4ea2-bdf0-257d3e430de4
---

BIEVR-LIO = ethz-asl "Bump-Image-Enhanced Voxel maps" LIO (RSS 2026, arXiv 2604.14421), same lab as COIN-LIO. Targets degenerate/feature-sparse geometry — directly relevant to [[hangar_737_inspection]]. Pure LIO, **no explicit loop closure**; keeps a persistent voxel-image map (implicit closure on revisit).

**Built & validated 2026-07-11** on Jetson Orin NX 8GB, Ouster OS1-64, ROS1 Noetic Docker. Profile: `jetson_orin_nx-ouster_os1_64-bievr_lio-ardupilot-noetic` (validated). Repo cloned at `/home/dev/BIEVR-LIO`; image `bievr_lio:noetic` (arm64). Build harness: `docker/Dockerfile_ros1.ceres` (Ceres stage) + `docker/Dockerfile_ros1.stage2` (workspace). Ouster sensor config: `config/sensor_configs/ouster_os1.yaml`. Bench artifacts: `/home/dev/coinlio_ws/results/bievr/`.

**Build gotchas (see also learned solutions #9/#10):** upstream `FROM osrf/ros:noetic-desktop-full` is amd64-only → silently QEMU-emulated on arm64 (wrong-arch image); swap to `ros:noetic-perception` (arm64). `build_ros1.sh` SSH-re-clones the repo + builds Livox (both unnecessary) → use local COPY, drop Livox (optional via find_package QUIET). Needs `ros-noetic-tf-conversions libtbb-dev libyaml-cpp-dev`. Node is C++20/-fconcepts. Ceres 2.2 builds native-arm64 (~10min) with LAPACK+SuiteSparse.

**KEY RESULT — 3rd_floor bag (374m path, 57m out-and-back, ROS1 `.bag` at `~/coinlio_ws/bags/3rd_floor_20260613_222459.bag`):**
- Return-to-origin drift = **1.35 cm** vs COIN-LIO ~3.8m (~280x tighter) vs adaptive_lio 3.32m floor.
- **Fully deterministic**: 8x `max_num_threads:0` runs + 1x single-thread control = **byte-identical trajectories** (same md5). 0/8 blow-ups.
- This is the OPPOSITE of EllipseLIO on this exact bag (7/14 tight, 6/14 blew up to 302m — see [[clean_slate_between_repeated_runs]]). BIEVR's parallel reduction is order-invariant → single-run numbers ARE trustworthy here.
- Real-time on Orin: ~30-36ms/scan avg (20Hz budget 50ms), occasional 268ms spike. Multi-thread 2.6x faster than single, same output.
- Auto-detected the 180° upside-down mount from IMU (roll ~179.6°); extrinsic reuse from prior rig was correct. Yellow flag: accel bias grew to 0.8/1.5 m/s² over the run (estimator absorbing something) — didn't hurt drift, watch on other bags.

**MULTI-BAG (2026-07-11, all ROS1, results in /home/dev/bievr_ws/results/):**
- **apartment_ros1.bag** (superodom_ws, 64.8m path, 11x5.5m, CLOSED loop, sensor UPRIGHT roll~0 here — auto-detected): **1.3cm return-to-origin** — confirms 3rd_floor's cm-level wasn't a fluke; holds across scale + mounting orientation.
- **degen_bag_ros1.bag** (superodom_ws, 43.3m path in a tiny 7x5m feature-poor room, OPEN path, NO ground truth): 64.2cm start-end gap (unscoreable w/o GT), but BOUNDED (max 7.4m, no divergence) AND **byte-identical across full 9-run battery** → determinism HOLDS on genuinely degenerate geometry (where EllipseLIO collapsed). Key result.
- Gap for hangar case: need a CLOSED-loop degenerate bag or GT (mocap/survey/prior-map) to actually score accuracy on degenerate geometry.
- PCD+LAS map export: no builtin; accumulate /bievr_lio/points/registered via /home/dev/bievr_ws/maps tooling (accumulate_map.py). 3rd_floor map = 9.96M pts @5cm voxel, in /home/dev/bievr_ws/maps/. Path sidecar (odom->nav_msgs/Path) at bievr_ws for Foxglove (BIEVR publishes no Path). Battery harness generalized: /home/dev/bievr_ws/run_battery.sh <bagmount> <bag> <outdir>.

**Status:** bag/bench validated only. NOT yet: live sensor (process_topics + Ouster driver on shared master), MAVROS→Cube Orange bridge, ground/flight (Phase 5). Container kept estimator-only by design — reuse existing ARM64 Ouster driver + MAVROS sidecar on shared ROS master (don't bake into image).

**Next probes:** other/harder bags (does 1.35cm hold or is 3rd_floor easy for it?), apartment bag, the accel-bias growth, then live bring-up.
