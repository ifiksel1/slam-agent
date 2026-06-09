# ArduPilot → PX4 + CTU MRS UAV System — Migration Assessment

**Date:** 2026-06-09 · **Status:** Scoping / decision document (no work executed)
**Trigger:** Evaluate adopting the [CTU MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system)
(control, estimation, planning, simulation) on the Orin NX / Ouster OS1-64 rig.

---

## TL;DR

Adopting MRS is an **autopilot + OS + middleware migration**, not a package add. MRS is
**PX4-only** (no ArduPilot plugin exists or is planned) and **ROS 2 Jazzy / Ubuntu 24.04 only**.
Your rig is ArduPilot / Cube Orange / JetPack 6 (Ubuntu 22.04 / Humble). So adopting MRS forces
three coupled changes:

1. **Flight controller firmware:** ArduPilot → PX4 on the Cube Orange.
2. **Companion OS/middleware:** Ubuntu 22.04 / Humble → Ubuntu 24.04 / Jazzy (native via **JetPack 7.2**, or **Docker** on JetPack 6).
3. **Vision/estimation plumbing:** ArduPilot EK3 + `vision_to_mavros` → PX4 EKF2 (or MRS's own estimator) via MAVROS 2 / uXRCE-DDS.

**Recommendation:** This is worth a **time-boxed SITL/Docker spike**, not an immediate
flip of the production aircraft. MRS's ROS 2 port is real and CI-green for the core flight
stack but self-labeled "in development." Validate it in simulation and on a **second/bench
aircraft** before touching the working ArduPilot drone. Keep the current ArduPilot +
FAST-LIO/EllipseLIO pipeline as the production baseline throughout.

---

## Hard constraints (verified 2026-06-09)

| Constraint | Detail | Source of truth |
|---|---|---|
| MRS is PX4-only | Hardware API plugins: `mrs_uav_px4_api`, `mrs_uav_dji_tello_api`. **No** `mrs_uav_ardupilot_api`, no community equivalent. | ctu-mrs org repo list |
| MRS is Jazzy-only | All packages `ros-jazzy-*`; README: "built on ROS Jazzy". No Humble branch/PPA. | mrs_uav_system `ros2` branch README |
| Jazzy needs Ubuntu 24.04 | JetPack 6.2.2 (latest 6.x) is Ubuntu 22.04 → no native Jazzy. **JetPack 7.2** (Ubuntu 24.04, ~early June 2026) adds Orin NX support. | NVIDIA JetPack pages |
| MRS↔PX4 link is MAVROS 2 | `mrs_uav_px4_api` (ros2) depends on `mavros`/`mavros_extras`; configs `mavros_px4_config*.yaml`. Not uXRCE-DDS. | px4_api `ros2` branch `package.xml`/`api.cpp` |
| MRS ROS 2 = "in development" | Core control/estimation/trackers + `mrs_uav_px4_api` + `mrs_point_lio_core` + OpenVINS core ported & CI-tracked. Not a certified flight stack. | mrs README "ROS2 Build status (in development)" |

**Implication:** there is no "keep ArduPilot, add MRS" option. If MRS is the goal, PX4 is mandatory.

---

## Two adoption paths

### Path A — Docker MRS on current JetPack 6 (lower disruption)
Run `docker pull ctumrs/mrs_uav_system` (multiarch, **ARM64 confirmed**) as an Ubuntu-24.04/Jazzy
container on the existing JetPack 6 host. You already run a Dockerized SLAM stack, so this fits
your operating model.
- ✅ No host OS reflash; keeps JetPack 6.2.2's mature CUDA stack.
- ✅ Reversible — delete the container to roll back.
- ⚠️ Container/network management overhead (host networking for MAVROS + Ouster UDP + DDS discovery — same class of issues documented in `docs/docker/DOCKER_NETWORK_GUIDE.md`).
- ⚠️ Jazzy-in-Docker over a 22.04 kernel; GPU passthrough via `jetson-containers`-style images if any MRS component needs CUDA (core flight stack does not).

### Path B — JetPack 7.2 native Jazzy (clean but bleeding-edge)
Flash the Orin NX to **JetPack 7.2** (Jetson Linux 39.2, Ubuntu 24.04) and `apt install
ros-jazzy-mrs-uav-system-full`.
- ✅ Native, simplest runtime once installed.
- ⚠️ JetPack 7.2 is very new (~June 2026) — driver/library maturity risk vs 6.2.2.
- ⚠️ Forces re-porting your **entire** existing 22.04/Humble pipeline (Ouster driver, FAST-LIO/EllipseLIO, MAVROS, all the ARM64 build fixes in this repo) to 24.04/Jazzy. Large, irreversible-ish.

**Verdict:** Start with **Path A (Docker)** for evaluation. Only consider Path B if MRS becomes the
committed long-term stack and the whole pipeline is being moved to 24.04 anyway.

---

## What the migration touches in *this* agent / pipeline

The agent already has partial PX4 support (Phase 3 `px4_params.txt` template, DDS bridge stub,
ROS2+PX4 row in the Phase 2 comms matrix). Migration work, by layer:

| Layer | Today (ArduPilot) | After (PX4 + MRS) |
|---|---|---|
| FC firmware | ArduPilot 4.5 on Cube Orange | PX4 (`make cubepilot_cubeorange` — **confirm Orange vs Orange+**, firmware NOT interchangeable) |
| EKF | EK3 `EK3_SRC1_*=6 (ExternalNav)`, `VISO_TYPE=2` | EKF2 `EKF2_EV_CTRL` (bitmask), `EKF2_HGT_REF=3` (vision height), `EKF2_EV_DELAY` (50–150 ms, measure), `EKF2_EV_POS_*` |
| Comms | MAVROS (`apm.launch`) | MAVROS 2 (MRS path) **or** uXRCE-DDS for a standalone bridge |
| Vision bridge | `vision_to_mavros` → `/mavros/vision_pose/pose` | MRS consumes its own estimator; standalone PX4 = publish `px4_msgs/VehicleOdometry` (NED!) to `/fmu/in/vehicle_visual_odometry`, or `nav_msgs/Odometry` → `/mavros/odometry/out` |
| Frame convention | ENU throughout | **ENU→NED conversion required** for the DDS path (silent failure if wrong — see `docs/troubleshooting/coordinate_frames.md`) |
| SLAM | FAST-LIO / EllipseLIO / SuperOdom (ours) | Either keep ours feeding PX4 EKF2, **or** use MRS's `mrs_point_lio_core`. Keeping ours is the lower-risk first step. |

A useful intermediate, decoupled from MRS: stand up **PX4 + our existing SLAM** via uXRCE-DDS
first (Phase 3 already has the DDS bridge skeleton). Prove GPS-denied hover on PX4 *before*
layering MRS control/estimation on top. That isolates two big unknowns.

---

## Phased plan (with go/no-go gates)

1. **SITL spike (desktop, ~days).** PX4 SITL + `ctumrs/mrs_uav_system` Docker + MRS Gazebo/MRS
   Multirotor sim. Fly a trajectory in sim. **Gate:** MRS control/estimation behaves and the
   ROS 2 port is stable enough to trust. If not → stop; revisit in a release or two.
2. **Bench PX4 on a spare Cube Orange (~days).** Flash PX4 (correct Cube variant), confirm
   uXRCE-DDS or MAVROS 2 link from the Jetson, push our EllipseLIO/FAST-LIO odom into EKF2,
   verify `EKF2` accepts external vision at 30–50 Hz on the bench (no props). **Gate:** stable
   external-vision fusion, valid TF chain.
3. **PX4 + our SLAM, no MRS (ground/hover).** Progressive bench → ground → hover per the
   standard safety order, on a test aircraft. **Gate:** GPS-denied hover drift comparable to the
   ArduPilot baseline.
4. **Layer MRS control/estimation (hover/flight).** Swap in MRS trackers/controllers. **Gate:**
   matches or beats baseline on a test airframe before any production aircraft is converted.
5. **Decide.** Keep ArduPilot baseline as production until MRS clears step 4 on hardware.

Each gate is a real stop point — do not advance the production aircraft past a failed gate.

---

## Risk summary

| Risk | Severity | Mitigation |
|---|---|---|
| No ArduPilot in MRS → PX4 mandatory | High | Accept only if MRS's planning/estimation features justify the switch; otherwise stay ArduPilot |
| Jazzy-only vs JetPack 6 / 22.04 | High | Path A Docker (`ctumrs/mrs_uav_system` ARM64) — no reflash |
| MRS ROS 2 "in development" | Medium | SITL + bench gates before any real flight; keep ArduPilot baseline |
| Cube Orange vs Orange+ firmware mismatch | Medium | Verify exact variant before flashing; targets are not interchangeable |
| ENU→NED frame errors on PX4 bridge | Medium | Reuse coordinate-frame diagnostics; verify on bench with known motions |
| JetPack 7.2 immaturity (if Path B) | Med-Low | Prefer Path A until MRS is committed; let 7.2 settle |
| Re-porting whole pipeline to 24.04/Jazzy | High (Path B only) | Avoid until MRS is the committed stack |

---

## When NOT to do this

Stay on ArduPilot if the goal is just better confined-space odometry — that's already addressed
by **EllipseLIO / SuperOdom** (added 2026-06-08) on the existing stack, with zero autopilot/OS
churn. Pursue PX4+MRS specifically if you want MRS's **integrated trajectory tracking, control,
estimation, and simulation/validation framework** — i.e., the control+planning stack, not just SLAM.

---

## Open questions to resolve before committing

- Which MRS capability is actually the driver — control/trackers, estimation, the
  simulation/validation framework, or planning? (Determines whether the full switch pays off.)
- Exact Cube variant on the aircraft (Orange vs Orange+) — sets the PX4 firmware target.
- Is there a spare airframe + spare Cube for the bench/hover gates, so the production ArduPilot
  drone is never the test vehicle?
- Keep our SLAM feeding PX4 EKF2, or adopt `mrs_point_lio_core` inside MRS? (Recommend: keep ours first.)
