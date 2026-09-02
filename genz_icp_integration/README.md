# GenZ-ICP integration (ROS 1 Noetic, native arm64)

GenZ-ICP — "Generalizable and Degeneracy-Robust LiDAR Odometry Using an Adaptive Weighting",
RA-L 2025, cocel-postech. Evaluated here as a **degeneracy control experiment** on the 12 real
737-hangar bags, NOT as a flight candidate.

> **This is not GenZ-LIO.** GenZ-LIO (arXiv 2603.16273) is the LiDAR-*inertial* successor;
> its source is unreleased ("upon publication"). GenZ-ICP is LiDAR-ONLY: no IMU, `deskew: false`.

## Why we are testing it

BIEVR-LIO failed on these bags with metre-to-km drift, including two numerical divergences
(863 km, 362 km) on pristine data. GenZ-ICP blends point-to-plane with point-to-point
residuals, weighted by `alpha = N_planar / (N_planar + N_non_planar)`. Because a
point-to-point residual has an **identity** translational Jacobian block, mixing any of it in
bounds the Hessian's smallest eigenvalue away from zero. The hypothesis under test:

* it should **eliminate the km-scale divergences** (those are ill-posed solves), and
* it should **not fix true unobservability** (metre-scale drift should remain).

Distinguishing those two is the point. It does not challenge the absolute-anchor architecture.

## Usage

```bash
./scripts/run_bag.sh <bag.bag> [config] [tag] [rate] [play_seconds] [visualize]
# smoke test (60s window):
./scripts/run_bag.sh 737_700_FRONT_2025-05-17-04-39-04.bag hangar.yaml _smoke 0.5 60
```

Outputs land in `results/<bag><tag>/`: `trajectory.csv` (pose + per-scan alpha),
`mavros_ref.csv` (onboard EKF reference, NOT an estimator input), and logs.

## Gotchas found during bring-up

1. **Do NOT use `--net=host`.** The host already runs a `rosmaster` on 11311 (the
   `foxglove_bridge` stack, `ROS_IP=192.168.2.50`). Sharing the host netns makes the
   in-container `roscore` fail to bind, so nodes silently register with *that* master and
   cannot route to each other — symptom is `XmlRpcClient::writeRequest: Connection refused`
   and a completely empty trajectory. The container is fully self-contained; keep it isolated.
2. **alpha is not published.** It is a local in `Registration.cpp:202`; only the separate
   Python pipeline prints it. It is recovered here from the sizes of `/genz/planar_points`
   and `/genz/non_planar_points`, which requires `visualize:=true`. Measured cost of those
   debug clouds: none (682 vs 709 poses over the same window).
3. **Upstream `odometry.launch` launches rviz** under the same `visualize` arg that gates the
   debug clouds. `launch/genz_hangar.launch` mirrors it without rviz (headless, ros-base).
4. **Not real-time at 20 Hz on the Orin NX.** ~11.8 Hz sustained at replay rate 1.0 (59% of
   scans); ~17.8 Hz at rate 0.5 (89%). Cost is the ICP itself (`max_num_iterations: 100`
   converges slowly in degenerate scenes), not the debug clouds. **Replay the sweep at 0.5.**
