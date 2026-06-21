# Option: CTU MRS UAV System (full control/estimation/planning/sim stack)

A selectable **alternative deployment stack**, not a SLAM algorithm. Load this when the
user wants the [CTU MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system) — a complete,
research-grade multirotor stack for control, state estimation, trajectory tracking, planning,
computer vision, and simulation/validation. Deep migration analysis + risk gates live in
`docs/MRS_PX4_MIGRATION_ASSESSMENT.md` — read it before committing hardware.

## What you get (full capability set)
- **Control & tracking:** `mrs_uav_controllers` (e.g. SE(3)/MPC), `mrs_uav_trackers` (smooth
  trajectory tracking with feasibility constraints).
- **State estimation:** `mrs_uav_core` estimation manager; pluggable odometry — `mrs_open_vins_core`
  (VIO) and `mrs_point_lio_core` (LiDAR-inertial) are ported.
- **Planning / mapping:** `mrs_octomap_mapping_planning` (octomap + planners).
- **Simulation / validation:** MRS Multirotor Simulator, Gazebo, FlightForge — the core reason to
  adopt MRS (safe real-world experimental validation workflow).
- **Hardware abstraction:** API plugins — `mrs_uav_px4_api`, `mrs_uav_dji_tello_api`.

## Hard requirements (these gate everything — verified 2026-06-09)
| Requirement | Detail |
|---|---|
| **Autopilot = PX4** | MRS has **no ArduPilot plugin** and none planned. Adopting MRS ⇒ ArduPilot→PX4 on the FC. |
| **ROS 2 Jazzy / Ubuntu 24.04 only** | All packages `ros-jazzy-*`; no Humble branch. |
| **FC link = MAVROS 2** | `mrs_uav_px4_api` (ros2) uses MAVROS, not uXRCE-DDS. |
| **Maturity** | ROS 2 port is "in development" but CI-green for the core flight stack. Research tool, not certified. |

## Enabling it on THIS rig (Orin NX 8GB, CTI Hadron, Ubuntu 20.04, Cube Orange)
Host is Ubuntu 20.04 — Jazzy is impossible natively (needs 24.04). Two paths:
- **Path A — Docker (recommended):** `docker pull ctumrs/mrs_uav_system` (multiarch, ARM64
  confirmed) and run as a Jazzy container. Fits the existing ROS2-in-Docker pattern. No reflash.
- **Path B — JetPack 7.2 (Ubuntu 24.04):** native `apt install ros-jazzy-mrs-uav-system-full`.
  Clean but bleeding-edge and re-ports the whole pipeline. Avoid until MRS is the committed stack.

FC side: flash **PX4** to the Cube Orange (`make cubepilot_cubeorange` — confirm Orange vs
Orange+; firmware NOT interchangeable). EKF2 external-vision params for GPS-denied:
`EKF2_EV_CTRL` (bitmask), `EKF2_HGT_REF=3`, `EKF2_EV_DELAY` (measure), `EKF2_EV_POS_*`.

## How SLAM feeds MRS
- Keep our LIO (EllipseLIO / SuperOdom / FAST-LIO) feeding PX4 EKF2 directly — lowest-risk first
  step — **or** adopt MRS's `mrs_point_lio_core` inside the stack. Recommend keeping ours first.

## When to choose this option (routing)
- Choose MRS when the user wants the **integrated control + trajectory + estimation + simulation/
  validation framework** — i.e. the autonomy stack, not just odometry. This replaces the
  ArduPilot + MAVROS + Phase-8 path-planner track.
- Do **NOT** choose MRS just for better confined-space SLAM — EllipseLIO/SuperOdom cover that on
  the existing ArduPilot stack with zero autopilot/OS churn.
- Selecting MRS reroutes Phase 1 (autopilot=PX4, ros=jazzy, docker=true) and supersedes Phase 8
  (MRS provides trackers/planners). Drive the decision through the 5-gate plan in the migration
  assessment (SITL → bench PX4 → PX4+our-SLAM hover → layer MRS → decide); never convert the
  production aircraft past a failed gate.
