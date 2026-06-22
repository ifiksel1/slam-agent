# Field Lessons — Jetson Orin NX + Ouster OS1-64 SLAM rig

Hard-won operational knowledge from validating EllipseLIO, SuperOdom, and FAST-LIO on
this rig (Jetson Orin NX 8GB / 6-core, CTI Hadron carrier, Ubuntu 20.04 host = ROS Noetic
only → all ROS 2 work runs in Docker). Ported from the agent's working memory so a fresh
clone carries it.

| Doc | What it covers |
|-----|----------------|
| [jetson_orin_ouster_operational.md](jetson_orin_ouster_operational.md) | Cross-cutting rig lessons that apply to **every** LIO framework here: CycloneDDS (not Fast-DDS), CPU clock boost, LiDAR mode tuning, field-recording sensor dropout, stopping the LiDAR (STANDBY), per-cycle ROS 2 node teardown. |
| [ellipselio_superodom_findings.md](ellipselio_superodom_findings.md) | EllipseLIO & SuperOdom specifics: validation status, the 3-way benchmark, **non-determinism / flyaway behavior and why estimator confidence can't gate it**, gating guidance, and the must-get-right config facts (scan rate, extrinsic, QoS, yaw-sign inversion, frame orientation). |

## The one-paragraph version
On this rig, **use CycloneDDS** (Fast-DDS stalls the large `/ouster/points` publish to ~15 Hz),
**boost the CPU clock** for accurate odometry (compute margin, not thermal, is the limit),
**validate every field bag** for sensor dropouts before trusting it, and **never gate a
flyaway on the estimator's own health/uncertainty/obs_score** — all three frameworks diverge
silently and (sometimes) smoothly, so a flight needs an external kinematic gate *and* an
absolute anchor (fiducial / prior-map relocalization). EllipseLIO is the smoothest/tightest
on clean data but compute-marginal at stock clock; SuperOdom is solid given the cores to
itself; both inherit a yaw-sign inversion that `vision_to_mavros` must negate.
