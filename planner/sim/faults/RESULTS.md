# L2 failure tests and coast test: results (2026-10-28)

QuadRover SITL closed loop on Shannon (`planner/sim/closed_loop.sh`), Hensel Phelps map
(`hp_1015_carved.pcd`, start 0, 0, yaw -90.7), mission `missions/example_hp_drive_fly.yaml` (clean run
cl39) or a variant under `faults/missions/`. One case per run, runs `~/sim/l2/runs/f*` (bags kept).

Code under test: the planner files as synced to `~/sim/l2/planner` at 15:44-17:32, including the other
agent's stale-FC-pose hold, the not-stationary-before-takeoff stop and the ground spots (md5 per run in
`fault_result.json` → `planner_md5`; bridge_core 2f862d3d, planner_bridge bbcf6060, mission_executive
f8d5fe9a, track_core 67773539, track_planner 3d0daa71 in every run). Speed cap 1.0 m/s (sim).

## How

- `faults/cases.sh <case> <run>`: one command per case (`cases.sh list`). It runs closed_loop.sh with:
  - `faults/render_fault.py` in place of render_node.py: `/sim/truth` (SITL's true pose, map frame) and
    live faults on `/sim_fault/cmd` (SLAM stop / freeze, FAST-LIO restart / pose jump, boxes seen by the
    lidar);
  - `faults/mav_watch.py` on SITL's SERIAL3: the FC's modes and texts as SITL reports them,
    independent of MAVROS (`logs/mav_watch.jsonl`); in the link cases also the GCS heartbeat (sysid 255);
  - `faults/inject.py`: waits for the trigger (fly leg 3 at ≥ 0.9 m/s, or the drive leg at ≥ 0.5 m/s),
    injects, observes until the executive's DONE/STOPPED + 14 s, writes `fault_result.json`;
  - fly_mission.py waits `END_HOLD_S` (15 s, 40-60 s for link loss) after the mission ends before it
    lands SITL, so what happens after the stop is seen.
- `faults/batch.sh case:run ...` runs a list and reruns a case whose SITL hit its watchdog
  ("GOT SIGALRM", Shannon load 15-25 from other jobs: f1, f7, f12, f12r, f13, f17, f25 were rerun).
- `faults/results.py runs/f*` gives the fact table; `--detail` the timelines; `faults/trace.py <run>`
  the true trajectory against the setpoints and phases (from the bag).
- SITL-only params (EXTRA_PARMS): `SERIAL3_PROTOCOL 2` (all runs; plus `--serial3=tcp:4`), and in the
  GCS-link cases `FS_GCS_ENABLE 1`, `FS_GCS_TIMEOUT 2` (the robot's own values, which
  sitl_overrides.parm turns off). The closed loop's DRIVE_PARMS as before. `Q_GUID_TIMEOUT 0.5` is in
  sitl_overrides.parm and is the only setting of it in the runs' defaults.parm (checked in f19).
- Distances are from `/sim/truth`. "Stop" = horizontal speed < 0.1 m/s for 1 s. "Along" = along the
  velocity at the fault. Box clearance = FC centre to the box surface; the frame reaches 0.25 m out.

## Coast test (D4, Q_GUID_TIMEOUT 0.5 s)

| what was killed | run | speed at the kill | stopping distance | what the FC did after |
|---|---|---|---|---|
| SUPER (`fsm_node`), flying | f3 | 1.20 m/s | **0.35 m** along in 1.9 s (max 0.42 m) | bridge HOLD after 0.34 s (planner stale) → zero-velocity target at the vehicle; held in GUIDED ±5 cm; executive "no progress" after 20 s → QRLOITER → STOPPED |
| planner bridge, flying | f19 | 0.92 m/s | overshoot **1.03 m** at +2 s, then back to **0.49 m** and held (4.5 s) | Q_GUID_TIMEOUT: velocity/accel zeroed, the FC flew to the last position target (≤ 1 m ahead) and held it in GUIDED; executive noticed nothing for 22 s → QRLOITER → STOPPED |
| track planner, driving | f20 | 0.52 m/s | slowed to ~0.2 m/s in 2 s (~0.8 m); **never stopped** | Rover Guided circled the last target (~0.6 m radius) at 0.17-0.22 m/s for 80 s, 11.9 m max excursion; executive gave up only at the 90 s leg timeout → STOPPED in GUIDED, still moving |
| planner bridge, driving | f6 | 0.51 m/s | slowed to ~0.17 m/s in 2 s (~0.6 m); **never stopped** | same circling at 0.12-0.17 m/s, 9.4 m excursion; STOPPED at +87 s, still moving |

Flight meets D4. Driving does not: "Rover Guided drives on to the last target, counts it reached and
brakes" (bridge docstring, README) did not happen in any drive run here; the rover orbits the last
target, as cl36 saw for a hold point. The fault-free baseline (f2) does the same after a normal drive leg.

## Results table

Expected = the design (docstrings, D4/D5/D7, the task). Pass = it ended in a hold, stop, landing or
QRSmartRTL as designed, without an unsafe excursion.

| # | case (phase) | run | injected how | expected | observed (mode sequence; distance after the fault; end state) | result |
|---|---|---|---|---|---|---|
| 0 | baseline, no fault | f2 | none (render_fault + mav_watch only) | DONE as cl39 | After the drive leg REACHED the bridge (drive HOLD) sent nothing; the rover **circled at 0.2-0.5 m/s for 30 s** in GUIDED, its centre within 5 cm of / over the 0.9 m box's edge for 3.6 s; the takeoff was refused "not stationary before takeoff" → STOPPED in GUIDED, still moving 0.24 m/s | **FAIL** (pre-existing in the drive stop) |
| 1a | planner killed (flying) | f3 | `pkill -9 -x fsm_node` at 1.20 m/s | bridge HOLD, hover; executive hold then next/stop | GUIDED HOLD at +0.7 s; stopped 0.35 m / 1.9 s, z 1.02-1.09; UNREACHABLE +20.5 → QRLOITER → STOPPED (mission `on_unreachable: stop`) | PASS |
| 1b | planner killed (driving) | f20 | `pkill -9 -f track_planner.py` at 0.52 m/s | rover stops within the 0.5 m lookahead, GUIDED, bridge off | never stopped; circled 80 s, 11.9 m; STOPPED at +87 s still moving (coast table) | **FAIL** |
| 1c | bridge killed (flying) | f19 | `pkill -9 -f planner_bridge.py` at 0.92 m/s | FC stops on Q_GUID_TIMEOUT | 1.03 m overshoot, settled 0.49 m, held in GUIDED; QRLOITER +22 → STOPPED | PASS |
| 1d | bridge killed (driving) | f6 | same, at 0.51 m/s | rover stops within the lookahead | never stopped; circled 80 s, 9.4 m | **FAIL** |
| 2a | SLAM stale: feed stopped 8 s (flying) | f21 | render: no /Odometry, no cloud for 8 s, at 0.91 m/s | bridge HOLD (SLAM), executive HOLD_SLAM, resume after map_rebuild_s | HOLD at +0.3 s, HOLD_SLAM +0.4; stopped 0.39 m / 3.7 s; while blind the EKF dead-reckoned: crept up to 0.26 m/s, ~0.5 m, z -0.1 m; SLAM back → SLAM bridge re-anchored (~0.75 m) → resumed +11.2, flew on; the land spot (fc frame) now lay 0.8 m off in the world, read "occupied: 2 cells up to 0.11 m" (map floor smear) for 5 s → QRLOITER → STOPPED | PASS (hold, then QRLoiter); see notes 6, 7 |
| 2b | SLAM stale: frozen pose (flying) | f22 | render: last pose and cloud repeated with fresh stamps for 8 s, at 1.00 m/s | hold | **Nothing held.** The bridge kept streaming (stamps fresh, FC pose fresh); the EKF, fed the frozen vision pose, believed the vehicle was at the target; true speed rose to **8.4 m/s**; SITL "Hit ground at 0.70 m/s" at +6.6 s, again at 1.49 m/s at +12.8 s; **45 m** away; EKF failsafe at +10.6 s → QRALTHOLD (FC's own), executive STOPPED on the mode change | **FAIL, UNSAFE** |
| 2c | SLAM stale: feed stopped 8 s (driving) | f23 | as 2a, at 0.51 m/s | bridge stops, rover stops | HOLD_SLAM at +0.3 s, bridge sent nothing; the rover kept creeping 0.15-0.2 m/s (~1.2 m in 8 s); resumed after the re-anchor (0.75 m), REACHED; then the post-drive orbit (a 1.3 m/s spike) → "not stationary before takeoff" → STOPPED in GUIDED, moving | **FAIL** (no stop) |
| 2d | SLAM stale: frozen pose (driving) | f24 | as 2b, at 0.51 m/s | hold, rover stops | bridge kept streaming; rover accelerated to **2.0 m/s**, 12.7 m in 9 s, its centre over the box's edge for 1.1 s; EKF failsafe → HOLD at +9.1 s, then coasted ~4 m more; executive STOPPED on the mode change | **FAIL, UNSAFE** |
| 3a | SLAM reset: laserMapping restart, 2 s gap (flying) | f7r | render: nothing for 2 s, then a new raw frame at the current pose | bridge HOLD, re-anchor, executive waits map_rebuild_s and resends the goal (D7) | HOLD (SLAM stale) +0.4, stopped 0.24 m / 3.2 s; SLAM bridge re-anchored ("avoided 6.73 m / -6 deg"); planner bridge latched "anchor yaw changed 6.1 deg"; SUPER's replans then failed (`PlanFromRest failed`: its map holds the old frame) → no fresh command → the latch never cleared → executive in HOLD_SLAM, leg clock frozen, **hovering until the 150 s observation ended**, no timeout | **FAIL** (deadlock; safe hover but never ends) |
| 3b | SLAM pose jump, no gap (flying) | f8 | render: instant new raw frame (a 6.5 m step) at 0.92 m/s | as 3a | SLAM bridge re-anchored ("step 6.52 m"); planner bridge latched "anchor yaw changed 7.6 deg" but SUPER's next command (still planned on the pre-jump odometry) released it within one 0.2 s status period, so the executive never saw HOLD_SLAM; the targets jumped: **surge to 1.46 m/s, climb 1.05 → 1.55 m, 2.5 m along**; then SUPER stalled → bridge HOLD → UNREACHABLE +23 → QRLOITER → STOPPED | **FAIL** (D7 hold ineffective, uncommanded climb) |
| 3c | SLAM reset, 2 s gap (driving) | f9 | as 3a, at 0.52 m/s | as 3a | HOLD_SLAM +0.4, resumed +5.2, drive REACHED; then the post-drive orbit → "not stationary before takeoff" → STOPPED moving | PASS for the reset; FAIL for the stop (as 0) |
| 4a | goal inside an obstacle (flying) | f10 | mission `goal_in_box_fly.yaml`: goal (-2.6, -7.5, 0.45) inside the 0.9 m box, `on_unreachable: next` | hold, then the next leg | SUPER sent nothing for 16 s (bridge HOLD, vehicle held), then flew over the box (FC centre 0.18 m above its top, 1.13 m); UNREACHABLE +41 → QRLOITER 5 s → GUIDED → land leg → landed, DONE | PASS (clearance 0.18 m to the box top is tight) |
| 4b | goal inside an obstacle (driving) | f11 | mission `goal_in_box_drive.yaml`: drive goal at the box centre, `on_unreachable: next` | rover stops (GUIDED, bridge off), next leg | track planner NO_PATH / HOLD most of the leg, but the rover **wandered 6 m at 0.16-0.25 m/s for 90 s** (leg timeout; the drive-progress check counts the wandering as progress); then the next leg drove over the box's south edge (centre within 5 cm for 2 s); DONE | **FAIL** (no stop; contact) |
| 5a | obstacle appears on the path (flying) | f25r | 0.6 x 0.6 x 3 m pillar 2.5 m ahead at 0.92 m/s (14-31k points per scan in /planner/cloud from +0.12 s) | SUPER replans around it or stops (hold) | SUPER stopped short (1 s), hovered 2 s, then **flew straight through the pillar** (FC centre inside it at +4.4 s, 1.0 m/s); went on to land and drive; later legs climbed to 2.6 m near it; UNREACHABLE → QRLOITER | **FAIL, UNSAFE** |
| 5b | obstacle appears on the path (driving) | f13r | 0.5 x 0.5 x 1.2 m box 1.5 m ahead at 0.51 m/s (13-27k points per scan in /planner/cloud from +0.16 s) | track planner replans around or NO_PATH (stop) | the track planner kept its path; **the rover drove through the box** (centre inside it at +2.0 s, 0.6-0.7 m/s); first NO_PATH at +8.9 s; REACHED → "not stationary" → STOPPED | **FAIL, UNSAFE** |
| 6 | landing spot occupied (flying) | f14 | 0.6 m crate on the spot (-6, -9) as the land leg starts | QRLoiter, STOPPED | "occupied: 46 cells, up to 0.57 m" for 5 s → QRLOITER at +9.8 s → STOPPED; hovered over the crate, FC centre 0.29 m above its top | PASS |
| 7a | GCS link loss (flying) | f15 | robot's FS_GCS_* (SITL-only); the sysid-255 heartbeat killed at 0.92 m/s | FC GCS failsafe → QRSmartRTL; executive stops | "GCS Failsafe" +1.2 s → QRSMARTRTL "returning along the path" → flew 4 m back to home, landed ("Hit ground 0.45 m/s"), disarmed +16 s; executive STOPPED on the mode change, bridge off | PASS |
| 7b | GCS link loss (driving) | f16 | as 7a, at 0.41 m/s | QRSmartRTL (owner 2026-10-22: also flies back over what it flew) | QRSMARTRTL +3.0 s: drove back along the path, then **"transitioning to FLY", took off** (up to 1.26 m) and flew back over the flown part, landed home, disarmed +47 s; executive STOPPED | PASS as designed; note 10 |
| 7c | MAVROS link loss (flying) | f17r, f26 | `pkill -9 -x mavros_node` at 0.94 / 0.92 m/s: setpoints and the FC's vision feed both stop | FC stops (Q_GUID_TIMEOUT); executive stops for good (FC link lost) | stopped 0.28 m in 1.8 s; then the EKF without vision drifted the vehicle 3.5 m (up to 1.5 m/s) in GUIDED; EKF failsafe +16 s → **QRALTHOLD (no position hold) → drifted 22 m (f17r) / 27.6 m (f26)** at ~1 m until the harness landed it; executive blind (/mavros/state just stops, "connected" stays true), at +20 s `set_mode(QRLOITER)` → **ServiceException, executive crashed** | **FAIL, UNSAFE** (reproduced) |
| 7d | MAVROS link loss (driving) | f18 | as 7c, at 0.53 m/s | rover stops | the rover drove on 4.3 m in 20 s (EKF dead-reckoning, Rover Guided chasing the last target), across the box's footprint; EKF failsafe → HOLD +19 s, still creeping 0.18 m/s after (11.5 m from the fault point by the end of the window); executive UNREACHABLE +20 → STOPPED (no crash on the ground: no mode command) | **FAIL** |

## Notes and flags

1. **Frozen SLAM is not detected anywhere (2b, 2d): the most dangerous result.** The bridge's SLAM
   check is on receive time and the new FC-pose check is on the EKF's pose, both fresh. The FC fuses
   the frozen pose and flies the vehicle away (8.4 m/s, ground strike, 45 m). It is the log 218/233 class.
   Needs a frozen-pose detector (identical poses while commanding motion; SLAM velocity vs FC
   IMU/velocity; FAST-LIO health), in the bridge and ideally in the SLAM-to-FC bridge before the FC sees it.
2. **The FC's EKF failsafe in flight is QRALTHOLD (2b, 7c).** With vision gone, QRALTHOLD has no
   position hold: 22-28 m drift at 1 m indoors. QRLAND would be safer than QRALTHOLD there
   (`FS_EKF_ACTION` / the QuadRover failsafe code; a param or firmware question for the owner, not changed here).
3. **Rover Guided does not stop at the last target (0, 1b, 1d, 2c, 3c, 4b).** It circles it at 0.1-0.3 m/s
   for as long as it is left, and the drive-progress check (`drive_progress_m`) counts the circling as
   progress, so only the 90 s leg timeout ends the leg. D4's drive stop needs an explicit stop: e.g.
   a stop target the firmware brakes on, or Rover HOLD (which D5 does not list for the executive).
4. **Obstacles that appear are flown and driven through (5a, 5b)**, with the points in /planner/cloud
   within 0.2 s. SUPER paused and then went through the pillar; the track planner never replanned in time.
5. **SLAM reset handling (D7) does not work end to end (3a, 3b).** 3a deadlocks: the bridge waits for a
   fresh SUPER command, SUPER cannot plan on its old-frame map, the executive waits for the bridge
   before it resends the goal, and HOLD_SLAM has no timeout. 3b: a "fresh" command right after the
   jump is still pre-jump, so the latch releases at once and the 5 Hz status never shows it. The fix
   needs SUPER's map reset or reseeded on a re-anchor, the latch to need a command planned after the
   jump (or a minimum hold), and a HOLD_SLAM timeout → QRLoiter/land.
6. **SLAM outages move the mission in the world (2a, 2c).** After an 8 s outage the FC frame carries the
   EKF's dead-reckoning drift (0.75-0.8 m), so fc-frame waypoints and land spots shift by that much.
7. The executive's STOPPED report after a refused land check quotes the newest landing status (f21:
   "reason": "clear", `clear: false`), not the reason it refused on. The answer turned clear 10 ms after
   the 5 s window closed.
8. **The executive does not survive MAVROS dying (7c):** an uncaught `ServiceException` in `set_mode`
   (also in f1, a SITL watchdog reset). It also cannot see the loss: it checks `connected` on the last
   /mavros/state message, which never updates once MAVROS is gone; it should treat a stale /mavros/state as lost.
9. The executive does not notice a dead planner or bridge (1a, 1c): it waits the 20 s no-progress
   window, hovering. Acceptable as a hold; a planner/bridge heartbeat check would end it sooner.
10. GCS link loss on the ground makes QuadRover take off on its own to retrace the flown part (7b). That is
    the 2026-10-22 decision; flagging it because an autonomous takeoff on a link loss is a big step.
11. **fc_pose_stale flaps in normal flight and driving**: 18 holds in 93 s in the baseline (f2), 10 in 40 s
    (f3). MAVROS's local_position arrives with gaps over the 0.3 s `fc_pose_timeout` under Shannon's
    load. Each flap holds the bridge until a fresh planner command (flight) or drops the drive stream.
    The timeout or the pose rate (MAV1_POSITION) needs a look before the robot.
12. **The 1.0 m/s cap is exceeded** in fault-free flight: true speed up to 1.55 m/s (f2), 1.47 m/s (cl39,
    MAVROS velocity), mostly at goal switches. 1.20 m/s at the f3 kill.
13. The coordinator's "known issue" (SLAM and FC pose stale together): stopping SLAM does not make the FC
    pose stale in SITL (the EKF keeps publishing on dead reckoning), so 2a took the SLAM hold. Both
    go stale only when MAVROS dies (7c). There the bridge reported fc_pose_stale, the executive did not
    pause (slam_hold false), the leg ran out at 20 s and its QRLOITER command crashed the executive.

## Not tested, and why

- **The GCS link loss as MAVROS's GCS link:** MAVROS runs with `gcs_url ""`; the FC's GCS failsafe
  watches sysid 255 (the SIYI app), so the link loss is the faults' own heartbeat (mav_watch `--gcs`).
- **The RC transmitter failsafe:** SITL has no RC input (FS_THR_ENABLE 0).
- **SLAM-stale cases with a longer outage (≥ 20 s):** not run; 8 s outages only.
- **obst_fly in f12/f12r/f25:** SITL watchdog resets (load); f25r is the valid run.
- **Contact:** SITL has no obstacles, so a run through a box is measured, not felt; on the robot it is a collision.

## Foxglove bags

`~/sim/foxglove/faults/fault_slam_freeze_fly/fault_slam_freeze_fly.bag` (f22), `~/sim/foxglove/faults/fault_obstacle_appears_fly/fault_obstacle_appears_fly.bag` (f25r),
`~/sim/foxglove/faults/fault_mavros_link_loss_fly/fault_mavros_link_loss_fly.bag` (f17r). The run bags stay in `~/sim/l2/runs/f*`.
