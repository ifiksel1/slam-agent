# L2 closed-loop sim (SITL only, on Shannon)

QuadRover SITL + MAVROS + the robot's own SLAM-to-FC bridge + a render node standing in for FAST-LIO,
one private network namespace per run. Nothing here talks to a real flight controller: the ROS
container runs with `--network none` and SITL's container joins its namespace, so ports 5760-5763
inside a run are not the revolute-gcs SITL's ports on Shannon's host network. At most 2 runs at once
(Shannon is shared).

## What runs

| piece | where | notes |
|---|---|---|
| `ardurover --model quadrover` | `l2-sitl-<name>` (ubuntu:22.04, needs glibc 2.34) | defaults `rover.parm`, `quadrover.parm`, the robot's `quadrover_from_pixracer.param` (filtered, see below), `sitl_overrides.parm` |
| roscore, MAVROS | `l2-ros-<name>` (`l4-sim-l2:latest`, `Dockerfile.l2`) | MAVROS with `config/mavros_apm_config.yaml`, `tcp://127.0.0.1:5760`; EKF origin from `set_gp_origin` every 10 s |
| the robot's bridge | same | `overrides/FAST-LIO/src/fastlio_mavros_bridge.py` + `frame_anchor.py`, QuadRover params (`bridge_params.yaml`, from `config/fastlio_to_fc_quadrover.launch`) |
| `render_node.py` | same | SIM_STATE (tcp 5762) -> `/Odometry` + `/cloud_registered` in FAST-LIO's raw frame, JT128 raycast |

SITL ports inside a run: SERIAL0 5760 MAVROS, SERIAL1 5762 render node, SERIAL2 5763 checks.

Params SITL cannot take from the robot (all reasons in `sitl_harness.sh` and `sitl_overrides.parm`):
IMU/compass/baro calibration, battery wiring, motor outputs and `Q_M_*`, the attitude controller's
gains and input shaping (`Q_A_RAT_*`, `Q_A_ANG_*_P`, `Q_A_ACC_*`, `Q_A_INPUT_TC`, `Q_A_ANG_LIM_TC`,
`Q_A_RATE_*`), CAN, the ToF ring. The position/velocity loops, speeds, land/RTL and the EKF stay the
robot's. The battery thresholds are halved because SITL's thrust scales with `SIM_BATT_VOLTAGE`.
SITL uses its own rangefinder and flow sensor, with GPS off and `Q_GUID_TIMEOUT` 0.5.

## Frames

- **raw**: FAST-LIO's world frame. +x left, +y up, +z forward, with the origin at the lidar's start
  position. The lidar sits 0.178 m ahead of the FC.
- **planner**: raw with `P = [0,0,1; 1,0,0; 0,1,0]` applied (the bridge's `position_matrix`).
- **fc**: MAVROS local ENU. The bridge anchors raw to the EKF.
- **map -> planner**: the rosparam `/sim/map_to_planner` holds `{yaw_deg, start}`, and
  `map = Rz(yaw) * planner + start`.

## Use

```bash
# on Shannon (ssh -i ~/.ssh/id_shannon_sim shannon-mini@192.168.0.124)
H=~/sim/l2/planner_sim/sitl_harness.sh
$H up t1 ~/sim/marsim/maps/hp_1015_carved.pcd 0 0 -90.7      # start x y (map, m), yaw (deg)
$H status t1
$H exec t1 python3 -u /planner_sim/l2_check.py                # EKF, arm IN SITL, takeoff, hold, goto, cloud
$H exec t1 rostopic hz /Odometry
$H down t1
```

Each run's logs go to `~/sim/l2/runs/<name>/logs` (roscore, mavros, bridge, render), and SITL's own
logs to `runs/<name>/sitl/logs`. SITL writes them as root, so remove an old run with
`docker run --rm -v ~/sim/l2/runs:/r alpine rm -rf /r/<name>`.

Environment variables for `up`:
- `LATENCY_S` (default 0.07): the pose is stamped at capture and published this much later.
- `FAULT`: one of `stop`, `freeze`, `drift` (raw +z, forward) or `height_bias` (raw -y, down).
  Set `FAULT_START_S` (seconds after the first pose) and `FAULT_RATE` (m/s) with it.
- `HW_DROP` / `HW_KEEP`: regexes on the robot's params, for bisecting.
- `HOME_YAW`: SITL's start heading.
- `CPUS` (default 6): CPUs for the ROS container.

## Closed loop with the landing check

`closed_loop.sh` starts the C++ planner bridge (`l4_bridge`), SUPER and the C++ `landing_check` node (`l4_landing`; it reads `/planner/cloud` and
`/planner/odom` and answers `/planner/landing_spot` on `/planner/landing_clear` and
`/planner/landing_status`, all recorded in the bag). A mission's `land` legs fly over the spot,
ask the check and land only on a clear answer; otherwise the executive holds in QRLoiter and stops
(`fly_mission.py` then lands SITL in QRLAND). `fly_mission.py` converts `land` legs' `at` to the fc
frame like the other goals. The executive's decisions are in `logs/fly_mission.log` (`LAND_CHECK`,
`LAND`, `STOPPED` with the check's reason).

```bash
cd ~/sim/l2 && planner_sim/closed_loop.sh cl20 ~/sim/marsim/maps/canusa_2034.pcd 0 0 0 \
  ~/sim/l2/planner/missions/example_canusa_A_land.yaml ~/sim/l2/planner_sim/super_closed_loop_canusa.yaml 600
```

Runs 2026-10-28: cl21 CAN USA A + land at (1.50, -2.00): clear in 0.35 s, landed, DONE 43 s; cl22
Hensel Phelps + land at (-6.00, -9.00): clear in 0.65 s, landed, DONE 33 s; cl23 land at (1.00, -3.25)
by the 0.85 m stack: "occupied: 9 cells up to 0.80 m" for 5 s (14 answers), QRLoiter, STOPPED; cl20
land at the real landing spot (-0.42, 1.45): refused, "occupied: 7 cells up to 0.25 m". The render
node raycasts the merged map, so its floor is the upper envelope of the map's smear: in SITL much of
the CAN USA floor carries 0.1-0.3 m bumps (moved objects and drift baked in) and reads occupied.
Pick sim land spots that read clear from every side (scratch probe: JT128 raycasts on an approach,
then the check).

## Closed loop that drives and flies

`closed_loop.sh` also starts the C++ `track_planner` node (`l4_track`). A mission's `drive` legs go to the track planner on
`/planning/drive_goal`. The executive switches the bridge's mode per leg on `/planner_bridge/set_mode`
(latched) and enables the bridge only once the bridge's status reports that mode. In drive mode
the bridge takes only the track planner's carrot (`/track_planner/pos_cmd`) and streams Rover
Guided position targets (mask 2552). In flight mode it takes only SUPER's `/planning/pos_cmd`.
SUPER keeps publishing toward its last goal after a landing, so the two planners publish on
separate topics.

The track planner and the landing check are C++ (`planner/ros`: `l4_track`, `l4_landing`; ported from
Python 2026-10-29). `closed_loop.sh` copies a catkin install space of them (`L4_WS`, default
`~/sim/cpp_ws/install`) into the ROS container and starts them with `rosrun`. Build it, and run their
tests and the real-map gate, in the l2 image, network-less:

```bash
# on Shannon: the packages in ~/sim/cpp_ws/src/{l4_track,l4_landing} (copies of planner/ros/*)
docker run --rm --network none --cpus 8 -u $(id -u):$(id -g) -e HOME=/tmp -v ~/sim/cpp_ws:/ws -w /ws \
  --entrypoint bash l4-sim-l2:latest -c "source /opt/ros/noetic/setup.bash; source /super_ws/devel/setup.bash;
  catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/ws/install -j8 install tests"
# gtests (real-data cases with LANDING_MAPS / LANDING_SIM set), rostests (rostest l4_track track_planner_node.test,
# rostest l4_landing landing_check_node.test), and the real-map gate (formerly test_track_planner.py --gate):
taskset -c 20-23 ~/sim/cpp_ws/devel/lib/l4_track/track_gate --gate --map hp_1015:pcd=...,meta=...,kf=... ...
```

The planner bridge and the mission executive are C++ too (`planner/ros`: `l4_bridge`, `l4_executive`,
with the shared headers in `l4_common`; ported from Python 2026-10-29: same node names, topics,
parameters, services, status JSON and mission format). `closed_loop.sh` copies their install space
(`L4_BRIDGE_WS`, default `~/sim/cpp_ws_bx/install`) to `/ws_bx/install` and starts
`/ws_bx/install/lib/l4_bridge/planner_bridge`; `fly_mission.py` starts
`/ws_bx/install/lib/l4_executive/mission_executive`. `BRIDGE_IMPL=py` with a `PLANNER_DIR` that still
holds the Python files (Shannon: `~/sim/l2/planner_bx_py`) runs the former Python pair (A/B only).

```bash
# on Shannon: ~/sim/cpp_ws_bx/planner/{ros/{l4_common,l4_bridge,l4_executive},config,missions} (copies)
~/sim/cpp_ws_bx/build.sh install     # catkin_make --source planner/ros ... install (l4-sim-l2, network-less)
~/sim/cpp_ws_bx/test.sh              # run_tests: gtests and rostests, then catkin_test_results
~/sim/cpp_ws_bx/ab/ab_bridge.sh b1 py cpp   # A/B: the Python and C++ bridges on run b1's recorded inputs
```

Since 2026-10-29 the bridge points the nose along the travel direction in flight (`~flight_yaw travel`;
SUPER's own yaw swung up to 100 deg in hovers), and the executive yaws at waypoints: at the end of a fly
leg it faces the next fly / land leg, over a land spot it faces the drive that follows, and it waits for
the yaw to settle (WAYPOINT_YAW: < 5 deg/s and < 0.1 m/s over 1 s) before the next leg or the landing
check; a leg's `yaw` overrides the direction, `{type: scan, yaw_sweep_deg: 360}` turns on the spot
(`missions/format_reference.yaml`). The bag records `/planner_bridge/yaw_cmd`.

How QuadRover takes the transitions (firmware `Rover/mode_guided*.cpp`, seen in SITL):
- `NAV_LAND` in GUIDED lands, says "Guided landing complete" and drives on in GUIDED, armed.
- Position targets then go to Rover's waypoint controller, which needs the robot's `WP_RADIUS 0.3`
  (rover.parm's 3 m counts a 0.5 m carrot as reached).
- `NAV_TAKEOFF` from GUIDED on the ground is refused while the tracks move faster than 0.2 m/s
  (`MAV_RESULT_TEMPORARILY_REJECTED`), so the executive waits for a standstill and asks again.
- Entering a QR mode armed on the ground starts the drive-to-fly transition, so the executive
  never holds in QRLoiter on the ground. It stays in GUIDED with the bridge off.
- A position hold at the vehicle is not a stop on the ground. The hold point is taken while the
  robot moves, so it is soon behind it, and Rover Guided orbits it (cl36: about 0.28 m/s for
  15 s, and the takeoff went in during a slowdown). A zero-speed heading target let SITL's rover
  lurch into reverse and creep (cl37, cl38). In drive mode the bridge's HOLD for a live but
  silent planner (NO_PATH, REACHED) therefore streams nothing: Rover Guided drives on to the last
  target it was sent, counts it reached and brakes.
  A dead planner is different (owner, 2026-10-29: the robot just stops, no continuation): once
  the planner process is gone the bridge latches `planner_dead` and sends a zero-speed velocity +
  heading target at the current heading (no run-out target); the executive goes STOPPED "planner
  died" in GUIDED with the bridge off (p2: stopped 0.54 m from the kill at 0.47 m/s, with the
  friction-fixed SITL; the cl37/38 reverse lurch did not recur). Flying, it holds, then QRLoiter
  and STOPPED. Before a takeoff that follows a drive, the executive keeps the bridge on until
  the vehicle has stood still for 1 s, and stops the mission if it doesn't.

`DRIVE_PARMS` (closed_loop.sh) adds the nuc-04 robot's driving params to SITL through
`EXTRA_PARMS`: `WP_SPEED 0.5`, `WP_RADIUS 0.3`, `WP_PIVOT_ANGLE 60`, `CRUISE_SPEED 0.5`,
`CRUISE_THROTTLE 50`, `ATC_STR_RAT_FF 0.3`. They come from `quadrover_overrides_nuc-04.param`;
the generic hardware file SITL loads has none of them.
- With them, the SITL rover tracks the track planner's path to about 5 cm (cl33, cl36).
- With rover.parm's own calibration (5 m/s at 30 % throttle, steering FF 0.75) it drove 1 m arcs
  off the path (cl38).
- The robot's CRUISE values over-drive SITL's motor model: speed peaks at about 1 m/s against
  `WP_SPEED 0.5`.

`LIDAR_UP` (closed_loop.sh default 0.25 m; `sitl_harness.sh up` default 0) raises the render
node's lidar above SITL's ground, which is the map's z = 0. The merged maps' floor is a 0.1-0.3 m
smear whose top lies around z = 0. With the lidar at 0, the robot standing on the ground sees the
smear all round it: in cl30 the track planner got an obstacle ring around the robot, a ground
model 1.9 m up and NO_PATH, and the robot pivoted on the spot. The robot's real lidar is about
0.08 m above a clean floor. 0.25 m is a sim caveat: on the ground the sim sees more floor than
the robot will.

`TRACK_Z_LO` (closed_loop.sh default 0.30 m; the node's own default is 0.15) is the track
planner's obstacle band floor in SITL. The rendered floor also steps 0.15-0.25 m between areas
(map drift baked in). At 0.15 the live window's evidence memory held most of the floor beyond
1.5 m as walls, and there was no path (cl31). 0.30 is the track planner's whole-map value (l4_track::Config) for the same
smear. It is a sim-only param: on the robot the live scans have no map drift.

The bag also records the bridge mode (`/planner_bridge/mode`, `/planner_bridge/mode_code`),
`/planning/drive_goal`, `/track_planner/{pos_cmd,path,status}`, `/planner/landing_markers` and
the FC's status texts.

```bash
cd ~/sim/l2 && planner_sim/closed_loop.sh cl30 ~/sim/marsim/maps/hp_1015_carved.pcd 0 0 -90.7 \
  ~/sim/l2/planner/missions/example_hp_drive_fly.yaml ~/sim/l2/planner_sim/super_closed_loop_hp.yaml 900
```

The sources live in this folder and are copied to Shannon's `~/sim/l2/planner_sim/`. The bridge and
the MAVROS config are copied to `~/sim/l2/slam/{overrides,config}`.
