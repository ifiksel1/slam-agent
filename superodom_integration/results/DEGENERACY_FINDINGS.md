# SuperOdom Degeneracy Characterization — Orin NX 8GB / OS1-64 (2026-06-09)

> **CAVEAT (extrinsic):** This run used a placeholder *identity* LiDAR↔IMU extrinsic. That was
> wrong for this rig and caused the yaw instability (±130° jumps) and "yaw sign inverted" below.
> Corrected later that day to `[-1,0,0,0,-1,0,0,0,1]` + `use_imu_roll_pitch:true` → replayed path
> is stable. The **qualitative degeneracy signal (x/yaw uncertainty maxed in corridor) and the
> health-flag-insensitivity finding stand**; the specific drift/yaw-jump numbers should be
> re-measured with the corrected extrinsic. See README "Extrinsic correction".

Featureless-corridor test. Because live SuperOdom + ROS2 Ouster driver is compute-marginal
on this rig (see "Live limitation"), the data was captured to a bag with the **driver alone**
(clean 20 Hz) and **replayed offline into SuperOdom at 0.5×**. This is the validated method
for testing on this hardware.

- Bag: `~/superodom_ws/degen_bag` (10 GB, NOT in git) — 176 s, 3386 clouds @19 Hz, 17591 IMU @100 Hz.
- Trajectory log: `degen_corridor_replay_2026-06-09.log`.

## Result — walk: ~54.67 m path, out to ~7 m, returned to ~start
| Signal | Value | Interpretation |
|---|---|---|
| uncertainty_yaw | min 0.88, **mean 1.00** | Heading = most degenerate DOF; pinned at max almost always |
| uncertainty_x | mean 0.99, **maxed in ~94% of samples** | Along-corridor translation starved |
| uncertainty_y | mean 0.99, min 0.60 | Cross-corridor also starved, brief relief |
| uncertainty_z | mean 0.52, min 0.14 | Best-constrained (floor/ceiling planes) |
| ICP avg_distance | 0.03 (good) → **0.81** (worst) | Spikes mark worst geometry; usable degeneracy proxy |
| Drift (loop) | **0.79 m over 54.67 m ≈ 1.4%** | Held together despite pervasive degeneracy |
| yaw stability | **jumps to +132°/+137°** at far end (~6.4 m out) | Heading genuinely came apart in deepest/most-degenerate stretch |
| `/state_estimation_health` | **`true` the ENTIRE run** | Did NOT respond to the degeneracy |

## ⚠️ Flight-integration takeaways
1. **Gate EKF fusion on `/super_odometry_stats` per-axis uncertainty, NOT `/state_estimation_health`.**
   The boolean health flag stayed green through a degenerate corridor where x/yaw uncertainty
   was pinned at max — trusting it would feed bad heading to ArduPilot (flyaway risk).
2. **Yaw is the weak DOF** (as `uncertainty_yaw=1.0` predicted) — heading jumped ±130° at the
   far end. Heavily distrust yaw in confined/symmetric spaces; consider a magnetometer/other
   yaw source there. Compounds with the known yaw-sign-vs-ENU issue.
3. **z is the most reliable axis** in corridors (planes above/below).
4. Cold start needs motion — SuperOdom won't initialize while stationary (first ~20 s of the
   walk produced no pose until movement began).

## Live limitation (why offline replay)
On this 8 GB / 6-core Orin, the ROS2 Ouster driver (1024×20) + SuperOdom together drops the
LiDAR to ~11 Hz irregular → feature_extraction throws every scan ("lidar more recent than imu")
→ no tracking. Driver alone holds 20 Hz. **For LIVE flight on this rig SuperOdom at 1024×20 is
not viable**; options: run 512×20 live (lighter, needs validation), reduce SuperOdom CPU, move to
a beefier compute, or use the lighter **EllipseLIO** for live use and keep SuperOdom for
offline/analysis.
