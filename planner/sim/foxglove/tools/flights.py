# Flight library data: one entry per flight. Used by gen.py on Shannon to write notes.md, README.md, index.html.
# Times are seconds on the Foxglove timeline (bag time); the videos start playback ~1 s in.

HP_MAP = "Hensel Phelps, flight of 2026-10-15 21:06:50 (`hp_1015_carved.pcd`)"
CAN_MAP = "CAN USA warehouse, flight of 2026-07-15 20:34:50 (`canusa_2034.pcd`)"
COURSE_MAP = ("Gate L2 obstacle course: the Hensel Phelps map plus a 2.5 m tower, two thin poles, a 2.2 m wall "
              "with a 0.90 m door and a 0.6 m crate (`l2/planner_sim/course/hp_gate_l2.pcd`, obstacles in `hp_gate_l2.json`)")

CATS = [
    ("planner_only", "Planner only (SUPER in MARSIM, perfect tracking)",
     "SUPER plans in the MARSIM simulator and the simulated drone follows its trajectory exactly (\"perfect "
     "tracking\"): no flight controller, no bridge, no executive. These show what the planner does on its own "
     "on a map of a real site. A waypoint file is fed to SUPER one goal at a time. Every run records 80 s "
     "whatever the mission's length; once the last waypoint is reached the drone just hovers there.",
     "super_sim_layout_robot.json"),
    ("closed_loop", "Closed loop in SITL, flight only",
     "SUPER, the planner bridge and the mission executive fly the QuadRover firmware in ArduPilot SITL through "
     "MAVROS, as on the robot. Flight legs only.",
     "closed_loop_layout.json"),
    ("drive_fly", "Closed loop in SITL, drive and fly",
     "The full L2 mission: fly, land after a landing check, drive with the track planner, take off again, fly, land.",
     "closed_loop_drive_fly_layout.json"),
    ("gate_l2", "Gate L2 obstacle course",
     "The drive-and-fly mission through an obstacle course at 1 m/s, the Gate L2 test.",
     "closed_loop_drive_fly_layout.json"),
    ("container", "Hensel Phelps container",
     "QuadRover missions through the open-ended shipping container on the Hensel Phelps map (6.1 m long, "
     "2.1 m wide and 2.2 m tall inside), on the Gate L2 course map. Mission files `planner/missions/hp_container_*.yaml`.",
     "closed_loop_drive_fly_layout.json"),
    ("drive", "Drive only",
     "Missions driven entirely on the tracks with the track planner, no takeoff. The wide video is a top-down "
     "view of `/map_low` (the map between 0.1 m and 1.0 m above the floor: walls and obstacles without ceilings).",
     "closed_loop_drive_fly_layout.json"),
    ("faults", "Fault tests",
     "One fault injected into the drive-and-fly mission during fly leg 3 (at ~0.9-1.0 m/s). Full write-up of all "
     "fault cases: `~/sim/l2/planner_sim/faults/RESULTS.md`.",
     "fault_layout.json"),
]

F = []


def add(**kw):
    F.append(kw)


# ---------------------------------------------------------------- planner only
add(cat="planner_only", name="canusa_A_inside", run="MARSIM `~/sim/marsim/runs/fg_whA`", map=CAN_MAP,
    desc="Out and back across the open warehouse floor at 1.5 m.",
    mission="Mission A (`wp_A.txt`): (-0.5, -1.0) -> (5.2, -4.2) -> (8.8, -3.8) -> (5.2, -4.2) -> (0, 0), all at 1.5 m.",
    result="PASS: 5 of 5 waypoints, 393 successful replans.",
    watch=["The coloured boxes are SUPER's safe flight corridors; the lines are its exploration and backup trajectories.",
           "The plot shows the drone's velocity (vx, vy) and height. The mission is done by ~40 s."],
    caveats=["The 3D view is fixed to the map; around the far waypoint (~15-25 s) the drone sits at the bottom-right "
             "edge of the video's view."])
add(cat="planner_only", name="canusa_B_climb", run="MARSIM `~/sim/marsim/runs/r7_B`", map=CAN_MAP,
    desc="The real flight's climb: straight up to 6.5 m and back down to 1.0 m.",
    mission="Mission B (`wp_B.txt`): (-0.3, 1.4, 6.5) -> (-0.4, 1.45, 1.0).",
    result="PASS: 2 of 2 waypoints.",
    watch=["The robot-model markers (`/robot/*`: body box, planning sphere, nearest map point, clearance label) "
           "are in this bag.",
           "SUPER config `r7_wh.yaml`: corridor seed line allowed up to 8 m (a straight 5.1 m climb aborted SUPER at "
           "the default 5 m), map 20 m tall, virtual ground lowered to -0.3 m, max acceleration 1.0 m/s^2."],
    caveats=[])
add(cat="planner_only", name="canusa_C_out_the_gap", run="MARSIM `~/sim/marsim/runs/fg_whC`", map=CAN_MAP,
    desc="Out of the warehouse through the south gap at 1.5 m.",
    mission="Mission C (`wp_C.txt`): (4.0, -3.8) -> (6.6, -6.6) -> (6.2, -9.5) -> (4.5, -11.0), all at 1.5 m.",
    result="PASS: 4 of 4 waypoints.",
    watch=["The narrow gap near the end: watch the corridors shrink to fit it."],
    caveats=[])
add(cat="planner_only", name="hensel_phelps_1_real_route", run="MARSIM `~/sim/marsim/runs/fg_hp1`", map=HP_MAP,
    desc="The route the pilot really flew, at its real 1.0-1.1 m height (then a climb to 3.3 m).",
    mission="HP mission 1 (`wp_hp1.txt`): 12 waypoints, 9 at 1.0-1.1 m south and west round the site, then 3 at 3.3 m.",
    result="FAIL: 2 of 12 waypoints. SUPER could not plan from rest (6971 'PlanFromRest failed'); the drone stays near the start.",
    watch=["This is the baseline the three variants below fix. With SUPER's default virtual ground (0.1 m) and "
           "its 0.44 m planning sphere, a 1.0-1.1 m route leaves it almost no room; the variants raise the "
           "waypoints (1.3m) or lower the virtual ground (floor_lowered, robot)."],
    caveats=["Not much moves in this video; that is the result."])
add(cat="planner_only", name="hensel_phelps_1_real_route_1.3m", run="MARSIM `~/sim/marsim/runs/fg_hp1h`", map=HP_MAP,
    desc="The real route with the low waypoints raised to 1.3 m.",
    mission="`wp_hp1h.txt`: the 12 waypoints of HP mission 1, the first 9 at 1.3 m.",
    result="PASS: 12 of 12 waypoints.",
    watch=["Compare with `hensel_phelps_1_real_route` (same route at 1.0-1.1 m, fails)."],
    caveats=[])
add(cat="planner_only", name="hensel_phelps_1_real_route_floor_lowered",
    run="MARSIM `~/sim/marsim/runs/r6_hp1_vg03` / `r6_hp1_vg05` (both ran together; the bag matches both start times)",
    map=HP_MAP,
    desc="The real route at its real height, with SUPER's virtual ground lowered.",
    mission="HP mission 1 (`wp_hp1.txt`), unchanged. SUPER config `r6_vg-0.3.yaml` / `r6_vg-0.5.yaml`: virtual "
            "ground -0.3 / -0.5 m, each goal's own height, max acceleration 1.0 m/s^2.",
    result="PASS: 12 of 12 waypoints.",
    watch=["Flies the route the pilot flew, at 1.0-1.1 m, then climbs to 3.3 m for the last three waypoints."],
    caveats=[])
add(cat="planner_only", name="hensel_phelps_1_real_route_robot",
    run="MARSIM `~/sim/marsim/runs/r6_hp1_vg03` / `r6_hp1_vg05`", map=HP_MAP,
    desc="As `floor_lowered`, plus the robot-model markers.",
    mission="HP mission 1 (`wp_hp1.txt`), virtual ground lowered (as `floor_lowered`).",
    result="PASS: 12 of 12 waypoints.",
    watch=["The robot body box, the 0.44 m planning sphere, the nearest map point and the clearance label "
           "(`sphere ... m  body ... m`: distance from the sphere / body to the nearest map point, negative = inside)."],
    caveats=[])
add(cat="planner_only", name="hensel_phelps_2_tight_loop", run="MARSIM `~/sim/marsim/runs/fg_hp2`", map=HP_MAP,
    desc="A tight loop round the south part of the site at 1.2 m.",
    mission="HP mission 2 (`wp_hp2.txt`), from (-0.3, -0.5): (-0.3, -4.0) -> (-1.0, -8.8) -> (-4.3, -8.8) -> "
            "(-4.6, -3.5) -> (-2.75, 0.66) -> (-0.3, -0.5), all at 1.2 m.",
    result="PARTIAL: 5 of 6 waypoints in the 80 s run (1667 'PlanFromRest failed' along the way).",
    watch=["Hesitations where SUPER fails to plan from rest and retries."],
    caveats=["The run ends (80 s) before the last waypoint."])
add(cat="planner_only", name="hensel_phelps_3_north", run="MARSIM `~/sim/marsim/runs/fg_hp3`", map=HP_MAP,
    desc="From the south-west room north through the site at 1.2 m.",
    mission="HP mission 3 (`wp_hp3.txt`), from (-5.7, -8.4): (-8.3, -4.0) -> (-8.6, 2.0) -> (-7.85, 8.56) -> "
            "(-4.0, 10.0) -> (1.0, 8.5), all at 1.2 m.",
    result="PASS: 5 of 5 waypoints.",
    watch=["The longest straight run in the set; watch the corridors chain along it."],
    caveats=[])

# ---------------------------------------------------------------- closed loop, flight only
add(cat="closed_loop", name="closed_loop_hp_real_route", run="cl4 (`~/sim/l2/runs/cl4`)", map=HP_MAP,
    desc="The real Hensel Phelps route flown closed loop through SITL.",
    mission="`example_hp_real_route`: take off to 1.0 m, 12 fly legs (the pilot's route), the last 3 at 3.3 m.",
    result="PASS: DONE after 46 s.",
    watch=["~17 s armed, 19 s take off, 22-64 s the 12 legs (climb to 3.3 m at ~51 s), 64 s DONE.",
           "Plot: height and the velocity setpoints the bridge sends to the FC."],
    caveats=["The end dips through a container roof: that is the test harness's straight-down QRLAND after DONE "
             "(64-76 s), through a roof that is in the map (SITL has no collisions), not the planner.",
             "Older bag: no `/viz/*` plot topics, no landing check."])
add(cat="closed_loop", name="closed_loop_canusa_A", run="cl7 (`~/sim/l2/runs/cl7`)", map=CAN_MAP,
    desc="CAN USA mission A (out and back at 1.5 m) closed loop through SITL.",
    mission="`example_canusa_A_inside`: take off to 1.5 m, 5 fly legs as planner-only mission A.",
    result="PASS: DONE after 37 s.",
    watch=["18 s take off, 24-55 s the 5 legs, 55 s DONE, then the harness's QRLAND."],
    caveats=["Recorded before SUPER's corridor and trajectory markers were added to the bag: only the robot and the cloud."])
add(cat="closed_loop", name="closed_loop_canusa_A_corridors_fixed", run="cl12 (`~/sim/l2/runs/cl12`)", map=CAN_MAP,
    desc="The same mission with SUPER's corridors and trajectories, after the SITL parameter fix.",
    mission="`example_canusa_A_inside` (as cl7).",
    result="PASS: DONE after 36 s.",
    watch=["18 s take off, 23-53 s the legs with SUPER's corridors, 53 s DONE.",
           "Compare with `superseded/closed_loop_canusa_A_corridors` (cl8, 48 s, wrong FC parameters)."],
    caveats=[])

# ---------------------------------------------------------------- drive and fly
add(cat="drive_fly", name="closed_loop_drive_fly", run="cl39 (`~/sim/l2/runs/cl39`), the reference run", map=HP_MAP,
    desc="The reference drive-and-fly run: fly, land, drive past the box, take off, fly back, land.",
    mission="`example_hp_drive_fly`: take off 1.0 m, 4 fly legs south down the east side, land at (-6.0, -9.0) "
            "after a landing check, drive 4.3 m east past the ~1.2 x 1 m box to (-1.7, -8.6) with the track planner, "
            "take off to 1.3 m, 2 fly legs north, land at (-0.2, -3.2).",
    result="PASS: DONE after 91 s.",
    watch=["17 s armed, 18 s take off; 24-46 s fly legs; 46 s landing check (clear) and landing; 51-65 s the drive "
           "(cyan line = track planner path); 65-80 s the executive waits for the tracks to stop; 86 s take off; "
           "91-104 s fly back; 104 s landing check and landing; 109 s DONE.",
           "Right panels: executive status and landing-check answer. Bottom: height, speed, bridge mode; mode and phase bands."],
    caveats=["The 15 s wait before the second take-off is the rover circling its last drive target in Rover Guided "
             "(see RESULTS.md note 3)."])
add(cat="drive_fly", name="closed_loop_drive_fly_cl33", run="cl33 (`~/sim/l2/runs/cl33`), an earlier run", map=HP_MAP,
    desc="An earlier drive-and-fly run, first landing spot at (-5.0, -8.2).",
    mission="`example_hp_drive_fly` as of cl33: as cl39 but 3 fly legs and the first landing at (-5.0, -8.2).",
    result="PASS: DONE after 95 s.",
    watch=["18 s Guided, 33 s take off (15 s wait), 38-60 s fly legs, 60 s land; 64-76 s drive; 91 s take off; "
           "96-109 s fly back and land; 113 s DONE."],
    caveats=["Superseded by cl39 as the reference; kept for comparison. The first landing spot was later moved "
             "(the frame-centred landing disc took in map bumps west of it in cl35)."])

# ---------------------------------------------------------------- gate L2
add(cat="gate_l2", name="gate_l2_course", run="g7 (`~/sim/l2/runs/g7`), first attempt", map=COURSE_MAP,
    desc="First Gate L2 attempt: the flight part passes, the 1 m/s drive fails at the door.",
    mission="`gate_l2_hp_course`: take off, fly south past the tower, land at (-3.7, -9.6), drive west through the "
            "0.90 m door past the crate to (-8.5, -9.0) at 1 m/s (SITL-only drive parameters), take off, fly north "
            "past pole_b, land at (-6.0, -3.5).",
    result="FAIL: the drive leg never gets through the door; UNREACHABLE at the 90 s leg timeout, STOPPED after 129 s.",
    watch=["23-46 s fly legs (tower on leg 3), 46 s landing; 51-141 s the drive leg trying to reach the door; "
           "141 s UNREACHABLE, 146 s STOPPED."],
    caveats=["The course obstacles are only in the map and the rendered lidar; SITL has no collisions."])
add(cat="gate_l2", name="gate_l2_course_pass", run="d18 (`~/sim/l2/runs/d18`), the drive-control work", map=COURSE_MAP,
    desc="Gate L2 passed: the whole course, drive through the door included.",
    mission="`gate_l2_hp_course` (as g7).",
    result="PASS: DONE after 87 s.",
    watch=["19 s take off; 24-63 s fly legs (leg 3 past the tower takes 26 s, 30-56 s); 63 s landing check and landing; "
           "68-80 s drive through the door; 86 s take off; 92-100 s fly past pole_b; 100 s land; 105 s DONE."],
    caveats=["Added by the drive-control agent at 18:32 on 2026-10-28; its own notes on the drive changes are with "
             "that work, not here."])

add(cat="gate_l2", name="gate_l2_course_b11", run="b11 (`~/sim/l2/runs/b11`), the final-code pass", map=COURSE_MAP,
    desc="Gate L2 passed with the final code: all four L4 nodes in C++, waypoint hold, slowing on curves, turn after take-off.",
    mission="`gate_l2_hp_course` (as g7 and d18). Code: track planner, landing check, planner bridge and mission "
            "executive all in C++; the executive holds at each waypoint and turns to the next leg's heading, the "
            "bridge slows on curves, and the robot turns to the first leg's heading after each take-off.",
    result="PASS: DONE after 101 s. Heading while moving (|yaw - travel heading|): median 6.6 deg, 90th percentile "
           "21 deg. Tightest clearance 0.12 m driving (the crate), 0.35 m flying (the tower). Both landings settled "
           "(yaw rate 0.1 and 1.1 deg/s); the first landed 0.5 deg off the drive leg's heading. Leg 7 needed 2 SUPER "
           "re-seeds. Details: `runs/b11/clearance.json`, `heading.json`.",
    watch=["Times are on the bag timeline (the videos are ~1 s later). 16.7 s armed, 18.6 s take off to 1.0 m.",
           "23.2-24.2 s the turn after take-off: the robot turns to leg 1's heading (4 deg) and holds until settled "
           "(1.0 s) before SUPER is seeded; leg 1 starts at 24.7 s.",
           "Waypoint holds and turns: at each waypoint the robot stops, turns to the next leg's heading and waits "
           "until settled: 28.4 s (1.5 s), 36.9 s (1.6 s), 49.4 s (the big turn, to -79 deg, 1.9 s), 58.4 s "
           "(-97 deg, the drive heading, before the landing, 1.6 s).",
           "Slowing on curves: in the speed plot the ~1 m/s flight speed dips to ~0.5 m/s where the path bends: "
           "46-47 s round the tower on leg 3 (0.35 m, the tightest flight clearance), 55 s on leg 4, 106 s on leg 7.",
           "60.0 s landing check (clear) and landing, settled and 0.5 deg off the drive heading, so the drive "
           "starts already lined up.",
           "64.6-74.5 s the drive (cyan line = track planner path): through the 0.90 m door at ~68 s, past the crate "
           "at ~69.5 s (0.12 m, the tightest clearance of the run), then a slow approach to the drive goal; "
           "74.5-81.5 s the executive waits for the tracks to stop.",
           "81.5 s take off to 1.3 m; 86.4-90.2 s the turn after take-off to 158 deg (3.8 s).",
           "90.7-100 s leg 7 stalls: SUPER goes silent for 3 s twice and the executive re-seeds it (95.0 s and "
           "99.8 s); it then flies past pole_b (102-108 s).",
           "108.3 s waypoint turn to 150 deg; 110.6 s leg 8 to the landing spot; 113.2 s turn and hold; 114.7 s "
           "landing check (clear) and landing; 119.0 s DONE, then the harness's QRLAND.",
           "The heading itself (`/planner_bridge/yaw_cmd`, the commanded waypoint yaw, and `/planner_bridge/status`) "
           "is in the bag but has no panel in the layouts; add a Raw Messages or Plot panel to see it."],
    caveats=["The 3D view is fixed to the map; on the far west side of the course (the drive and leg 7) the robot "
             "is near the left edge of the wide video's view. The close-up follows it."])

# ---------------------------------------------------------------- container
add(cat="container", name="hp_container_fly", run="k1 (`~/sim/l2/runs/k1`)", map=COURSE_MAP,
    desc="Fly through the container on its centreline, out of the east end over the low object, turn and land.",
    mission="`hp_container_fly` (`planner/missions/hp_container_fly.yaml`): take off at the real takeoff spot; "
            "W1 west of the container's north-west corner; A 0.75 m outside the west mouth; C inside the east end, "
            "all at 1.05 m (planner frame; the roof underside is at 2.12-2.18 m, the floor at -0.1 m); D past the "
            "0.9 m high object outside the east end at 1.45 m; land on the Gate L2 first landing spot. Code and "
            "configs as b11 (all four L4 nodes in C++, `super_closed_loop_hp.yaml` unchanged).",
    result="PASS: DONE after 56 s. Inside the container (frame centre at planner x -0.5..6.0): body clearance 0.48 m "
           "at the west mouth (the end frame), 0.67-0.77 m along the rest; height 1.40 m entering, 1.05 m from x 4.5 "
           "on; y within 0.06 m of the centreline. Whole run: 0.32 m flying (at the west mouth), no sample under "
           "0.10 m. Heading while moving: median 0.8 deg, 90th percentile 4.1 deg. Landing check clear in 0.15 s. "
           "SUPER stalled 9 s at the west mouth (2 re-seeds). Details: `runs/k1/clearance.json`, `heading.json`.",
    watch=["Times are on the bag timeline (the videos are ~1 s later). 16.5 s armed, 17.9 s take off to 1.0 m; "
           "22.5-26.4 s the turn after take-off; 26.9 s leg 1 to W1; 29.7 s turn; 32.1 s leg 2 to A, the west mouth.",
           "35.0-38.0 s at A the robot turns to face the container (-2 deg).",
           "38-50 s the stall at the west mouth: SUPER's backup trajectory fails ~1100 times (\"Minco backup_traj "
           "opt failed\"), the executive re-seeds it at 41.6 s and 46.4 s, and the vehicle rises from 1.05 m to "
           "1.64 m while SUPER is silent.",
           "50.0 s it enters the container at 1.40 m and descends to the 1.05 m goal height by x 4.5 (55 s), on the "
           "centreline; 55.3 s waypoint turn at C inside the east end.",
           "56.9-62.7 s leg 4: out of the east end, climbing to 1.45-1.55 m over the low object (the body ~0.5 m above its top).",
           "62.7 s turn to the landing spot; 65.1 s land leg; 68.6 s landing check (clear) and landing; 73.7 s DONE, "
           "then the harness's QRLAND."],
    caveats=["SITL has no collisions: the clearances come from `clearance.py` (body box against the map), not "
             "from the simulator.",
             "The floor in front of the west mouth is missing from the map (a ~2.5 m gap), so the rendered lidar "
             "sees no floor there."])

add(cat="container", name="hp_container_drive", run="n1 (`~/sim/l2/runs/n1`)", map=COURSE_MAP,
    desc="Land in front of the west mouth, drive through the container to the east side, take off and land.",
    mission="`hp_container_drive` (`planner/missions/hp_container_drive.yaml`): take off at the real takeoff spot; "
            "W1 west of the container's north-west corner at 1.05 m; land (landing check) on the filled floor 0.85 m "
            "outside the west mouth, facing east; drive I through the container on its centreline to 0.65 m inside "
            "the east end; drive G out of the east end and south-east past the low object; take off; land on the "
            "Gate L2 first landing spot. Track planner with the per-tile per-scan evidence (2026-10-29); course map "
            "with the west-mouth floor patch (`hp_gate_l2.json` \"floor\").",
    result="PASS: DONE after 79 s. The drive went through the container: inside it (frame centre at planner "
           "x -0.5..6.0) the odometry point stayed at y -2.85..-2.27 (inner walls at about -1.5 / -3.6), at up to "
           "1.07 m/s; body clearance inside 0.23 m at the west mouth and 0.53-0.73 m along the rest, with the "
           "container floor left out (`contclear`, floor ignored below map z 0.30). Whole run with the floor smear "
           "left out (`clearance.py --floor-ignore 0.30`): 0.17 m; with the default 0.10 m the drive reads -0.10 m "
           "against the container floor, which SITL's flat ground puts above the body's bottom (a SITL artefact). "
           "Landing checks clear in 0.1 s and 3.0 s. Heading while moving: median 2 deg, 90th percentile 6.8 deg.",
    watch=["Times are on the bag timeline (the videos are ~1 s later). 17.8 s armed, 18.8 s take off to 1.0 m; "
           "28.1 s leg 1 to W1; 32.8 s leg 2 to the landing spot outside the west mouth.",
           "35.2-38.2 s turn to face the container; 38.3 s landing check (clear) and landing.",
           "42.7-52.1 s drive leg 3: straight through the container on its centreline, over the floor step at both ends.",
           "52.1-71.9 s drive leg 4: out of the east end and south-east past the low object.",
           "74.5 s take off; 82.5 s fly leg to the Gate L2 landing spot; 89.4 s landing check, 92.4 s landing; "
           "97.0 s DONE, then the harness's QRLAND."],
    caveats=["SITL has no collisions and a flat ground at map z = 0: the simulated robot drives 0.13-0.20 m below "
             "the container floor, so the clearances inside leave the floor out.",
             "The floor in front of the west mouth is a flat patch added to the course map (planner z -0.24); the "
             "real scan has a hole there."])

# ---------------------------------------------------------------- drive only
add(cat="drive", name="hp_drive_tour", run="t1 (`~/sim/l2/runs/t1`)", map=COURSE_MAP,
    desc="Drive-only tour of the Hensel Phelps course map: 14 legs, about 53 m, round the container and back through it.",
    mission="`hp_drive_tour` (`planner/missions/hp_drive_tour.yaml`), map frame, start (0, 0) facing south: 1-2 south "
            "down the corridor east of the container; 3 the box-tower gap; 4 the box-pole_a gap; 5 through the 0.90 m "
            "door past the crate; 6-7 north in the west room past pole_b; 8 round the north end of the door wall; "
            "9 up the corridor west of the container; 10-11 back south; 12 into the container's south mouth; 13 north "
            "through the container and out; 14 near the start. Route picture: `route.png` in this folder (red: "
            "waypoints and legs, green: the driven track, magenta: course obstacles). Track planner with the clearance "
            "preference (2026-10-30), before the hairpin and overhang changes.",
    result="PASS: DONE after 153 s, no takeoff, no unreachable legs, no reverses. Min body clearance 0.12 m (the "
           "container's floor edge at its south mouth), door jamb 0.15 m, tower 0.17 m, pole_a 0.30 m "
           "(`clearance.py --floor-ignore 0.30`). Max 1.19 m/s, slowing to 0.5-0.8 m/s at the door, the gaps and the "
           "container mouth. Pivots: ~120 deg at waypoint 3, ~145 deg at waypoint 9, ~60 deg before the south mouth, "
           "21 deg past the crate.",
    watch=["Bag time (the videos are ~0.6 s later). Leg starts: 18.2, 22.8, 27.7, 38.0, 47.5, 79.8, 86.7, 103.9, "
           "110.2, 115.7, 126.0, 130.4, 150.2, 163.5 s; DONE 170.9 s.",
           "38.9-41.2 s pivot ~120 deg at waypoint 3; 51-55 s slows and goes through the door; 57-78 s past the crate "
           "(21 deg pivot at 77.8 s).",
           "86.7-103.9 s leg 7 swings wide north; 116.7-119.5 s pivot ~143 deg at waypoint 9; 131.8-133.2 s pivot "
           "~60 deg before the south mouth.",
           "~139 s closest point (0.12 m) entering the south mouth; 141.5 s into the container; ~151-161 s through it "
           "and out of the north mouth."],
    caveats=["The close-up uses the owner's `closeup_layout` (camera ~2.5 m behind, following heading); it is saved in "
             "Foxglove, not as a file here. The wide video zooms in partway through: the owner zoomed the view during the "
             "recording.",
             "SITL ground is flat: the robot drives below the container floor; floor-smear contacts are left out "
             "of the clearance."])

# ---------------------------------------------------------------- faults
add(cat="faults", name="fault_slam_freeze_fly", run="f22 (`~/sim/l2/runs/f22`), RESULTS.md case 2b", map=HP_MAP,
    desc="Frozen SLAM pose in flight: nothing detects it, the EKF flies the vehicle away.",
    mission="`example_hp_drive_fly`; at 30.8 s (fly leg 3, 1.03 m/s) the renderer repeats the last pose and cloud "
            "with fresh time stamps for 8 s.",
    result="FAIL, UNSAFE: true speed up to 8.4 m/s, ground strikes at 37.4 s and 43.6 s, 45 m away; the FC's EKF "
           "failsafe (41.4 s) switches to QRAltHold; the executive stops on the mode change.",
    watch=["Plot: 'height (odometry z)' stays flat at ~1 m while 'true height (sim truth)' falls to the floor: the "
           "believed and the real vehicle part at 30.8 s.",
           "Right panels: `/fault/events` (the injection) and the executive status. Mode band: CMODE(18) = QRAltHold, "
           "CMODE(21) = QRLand."],
    caveats=["`/sim/truth` is in its own frame (`sim_map`) with no transform in the bag, so the true position is only "
             "in the plot, not in the 3D view."])
add(cat="faults", name="fault_obstacle_appears_fly", run="f25r (`~/sim/l2/runs/f25r`), RESULTS.md case 5a", map=HP_MAP,
    desc="A 0.6 x 0.6 x 3 m pillar appears 2.5 m ahead; SUPER flies through it.",
    mission="`example_hp_drive_fly`; at 33.0 s (fly leg 3, 0.92 m/s) a pillar appears at (0.12, -6.49) in the live lidar.",
    result="FAIL, UNSAFE: SUPER stopped short, hovered 2 s, then flew straight through the pillar (~37 s); the "
           "mission went on (land 49 s, drive 53 s, take off 79 s); leg 8 climbed to 2.6 m near the pillar; "
           "UNREACHABLE at 108 s -> QRLoiter -> STOPPED.",
    watch=["33-38 s: the pillar is the column of white live-lidar points ahead of the robot.",
           "Mode band: CMODE(19) = QRLoiter, CMODE(21) = QRLand."],
    caveats=["The pillar is only in the rendered lidar (`/planner/cloud`), not in `/map`, so it shows as white points."])
add(cat="faults", name="fault_mavros_link_loss_fly", run="f17r (`~/sim/l2/runs/f17r`), RESULTS.md case 7c", map=HP_MAP,
    desc="MAVROS killed in flight: setpoints and the vision feed stop; the vehicle drifts in QRAltHold.",
    mission="`example_hp_drive_fly`; at 30.9 s (fly leg 3, 0.97 m/s) `pkill -9 mavros_node`.",
    result="FAIL, UNSAFE: stopped 0.28 m in 1.8 s, then drifted 3.5 m in Guided without vision; the FC's EKF failsafe "
           "(+16 s) went to QRAltHold and the vehicle drifted 22 m at ~1 m height until the harness landed it; the "
           "executive could not see the loss and crashed on its QRLoiter command (+20 s).",
    watch=["After 30.9 s every `/mavros/*` topic stops: the mode band stays at GUIDED and the FC's own mode changes "
           "are not in the bag (they are in the run's `logs/mav_watch.jsonl`).",
           "The robot in 3D and the true-height plot keep moving (they come from the simulator, not MAVROS): watch the drift."],
    caveats=[])

SUPERSEDED = dict(
    name="closed_loop_canusa_A_corridors", run="cl8 (`~/sim/l2/runs/cl8`)", map=CAN_MAP,
    desc="CAN USA mission A with SUPER's corridors, run with a harness bug.",
    note="SUPERSEDED by `closed_loop/closed_loop_canusa_A_corridors_fixed` (cl12). This run had a test-harness bug "
         "that left 77 FC parameters at firmware defaults: 1 Hz position updates to MAVROS and jittery setpoints. "
         "It finished (DONE after 48 s, against 36 s in cl12), but its flight behaviour is not representative. "
         "Kept for reference only; no video.",
    layout="closed_loop_layout.json")
