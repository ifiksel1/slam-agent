// Mission executive for L4 autonomy (D6, D5, D7): flies and drives a YAML mission on the QuadRover.
// C++ port of the former planner/mission_executive.py: node name, topics, services, parameters, mission
// format, status JSON and behaviour unchanged.
//
// The pilot arms (D5: this node never arms, disarms or sets parameters). On /mission_executive/start
// (std_srvs/Trigger) with the vehicle armed it switches to GUIDED, takes off with Copter's NAV_TAKEOFF,
// then works through the mission's legs:
//
//   fly      goal [x, y, z] in the FC frame -> /planning/click_goal for SUPER (planner frame); the
//            bridge in flight mode
//   drive    goal [x, y] on the ground      -> /planning/drive_goal for the track planner, the bridge
//            in drive mode (Rover Guided position targets). In the air it lands first (here or at a clear
//            spot nearby: the landing-spot search below). Done when the FC is within accept_radius or the
//            track planner reports REACHED for this goal
//   land     at [x, y]: fly there, climb to the check height, ask the landing check (/planner/landing_spot);
//            refused: the landing-spot search nearby, as a drive from the air (below; owner 2026-11-02), then
//            NAV_LAND (lands and drives on in GUIDED). ~land_search false: require the spot clear within
//            ~landing_check_s (a not-clear answer is asked again until then: a person walking off clears it)
//   takeoff  height above the liftoff point (NAV_TAKEOFF in GUIDED). The executive first waits for the
//            vehicle to stand still (moved under ~takeoff_still_m 5 cm over the last ~takeoff_still_window_s
//            1 s, the FC's pose; owner 2026-11-03, was 2 cm) for up to ~takeoff_settle_s (15 s). Not still by
//            then: the mission stops, "not stationary before takeoff" (owner decision 2026-10-28), in
//            GUIDED with the bridge off, no takeoff sent. Still: NAV_TAKEOFF, repeated while QuadRover
//            answers MAV_RESULT_TEMPORARILY_REJECTED, for up to ~takeoff_settle_s
//
// A mission that starts with a fly or land leg gets a takeoff (takeoff_height) first; one that starts
// with a drive leg drives first.
//
// The bridge's mode (flight | drive) is set on /planner_bridge/set_mode (latched) before each leg
// streams, and the executive waits for the bridge to report it (/planner_bridge/status "mode") before
// enabling it, so a target from one planner is never sent in the other mode.
//
// Frames (shared contract): waypoints are in "fc", the FC's EKF frame as MAVROS gives it (local ENU,
// metres), which survives SLAM restarts (D7: the SLAM-to-FC bridge re-anchors vision onto the EKF). SUPER plans
// in "planner"; the planner bridge publishes the fc<-planner transform on /planner_bridge/anchor and its state
// on /planner_bridge/status (JSON). Every goal is converted fc->planner with the latest anchor. Since
// 2026-11-02 evening the planner frame carries over a FAST-LIO frame reset instead of jumping with it (the
// bridge's FrameCarry, planner_bridge_node.cpp): SUPER's map and trajectory stay valid, and the anchor stays
// as it was over a carried reset; it changes on a re-anchor in the same raw frame (the EKF's drift over a gap).
//
// Failures:
//   * no progress (not 0.2 m closer within no_progress_s; on a drive leg also not ~drive_progress_m
//     driven) or leg timeout (goal unreachable) -> QRLOITER (in the air), hold, then the next leg
//     (~on_unreachable: next) or stop there (stop)
//   * the planner died (owner decision 2026-10-29): the bridge reports HOLD "planner_dead" -> the mission
//     stops at once, STOPPED "planner died", whatever ~on_unreachable says. In the air: QRLOITER. On the
//     ground: GUIDED, bridge off (its last target, the zero-speed stop, stays Rover's)
//   * bridge HOLD for a SLAM reset or an anchor jump (D7: hold, re-anchor, resend the goal) -> HOLD_SLAM, stop
//     sending goals. The bridge first holds a raw pose after a gap or step until its vision pose says whether the
//     raw frame changed (a new frame: carried over, FrameCarry; the same frame: the anchor moves by the EKF's
//     drift). In flight a re-anchor hold lasts until SUPER publishes a trajectory started after the re-anchor,
//     and SUPER plans only on a goal: with SLAM fresh, the goal goes again (fc->planner with the anchor now
//     published, checked against the bridge's "planner_pos") after ~map_rebuild_s, re-seeded when SUPER stays
//     silent (below). Other holds: the goal again ~map_rebuild_s after the bridge reports OK. Over
//     ~hold_slam_timeout_s: "SLAM hold timed out"
//   * the SLAM pose froze (L2 faults 2b, 2d, 2026-10-30): the bridge reports HOLD "slam_frozen" (FAST-LIO's
//     pose repeated while the vehicle moves or is made to move; the EKF follows the frozen pose, so the FC's
//     position is not known) -> the mission stops at once, STOPPED "SLAM pose frozen". In the air:
//     ~slam_frozen_flight_mode (QRLAND; QRLOITER and QRSMARTRTL hold or retrace a position the EKF no longer
//     has, GUIDED would hover on it). On the ground: GUIDED, bridge off (its last target, the zero-speed
//     stop, stays Rover's). Watched during legs and waypoint hovers
//   * on the ground (driving) nothing is held in QRLOITER: entering a QR mode armed on the ground starts
//     QuadRover's drive-to-fly transition (props spool up). There an unreachable drive goal, a refused
//     takeoff or a stop leaves the vehicle in GUIDED with the bridge disabled
//   * the FC leaves GUIDED on its own (failsafe, the pilot) or MAVROS loses the FC -> stop commanding for
//     good; the FC's failsafe owns the vehicle
//   * no clear landing spot (the search) or, ~land_search false, the spot not reported clear within
//     ~landing_check_s -> QRLOITER, mission stopped
//   * a takeoff refused or not completed -> mission stopped; a refused takeoff leaves it on the ground in
//     GUIDED, one that started and did not complete goes to QRLOITER
//   * not standing still within ~takeoff_settle_s before a takeoff -> mission stopped on the ground,
//     GUIDED, bridge off, no takeoff sent
//
// Ground spots (owner decision 2026-10-28): where the vehicle sat on the ground is known floor for the
// landing check. Each spot is the FC pose (fc frame, with its yaw) when standing still on the ground in
// GUIDED just before a NAV_TAKEOFF ("takeoff"), and when a landing completes ("landed"). All of the
// mission's spots so far go out on /mission_executive/ground_spots (std_msgs/String, latched, JSON
// {"frame": "fc", "spots": [{"event", "xyz", "yaw", "t"}]}).
//
// The planner bridge streams targets only while /planner_bridge/enable (latched) is true: this node sets
// it for its fly and drive legs and clears it before every takeoff, landing, mode change and stop.
//
// Mode changes are only GUIDED, QRLOITER, QRLAND, QRSMARTRTL (D5), sent as MAV_CMD_DO_SET_MODE with
// QuadRover's custom mode numbers (MAVROS has no names for the QR modes and reports them as CMODE(n)).
//
// Waypoints and the heading (owner rule 2026-10-29: the heading points along the direction of travel;
// the robot yaws only at a waypoint). The bridge points the nose along the travel direction in flight
// (~flight_yaw travel). At the end of a fly leg followed by another fly or land leg, the executive sends
// the bridge a waypoint yaw facing that leg (/planner_bridge/yaw_cmd, latched std_msgs/Float64, fc frame,
// rad; NaN = none) and waits, hovering, until the yaw has settled (WAYPOINT_YAW: yaw rate under
// ~yaw_settle_rate_deg 5 deg/s and horizontal speed under ~yaw_settle_speed 0.1 m/s over 1 s, within
// ~yaw_tol_deg 10 of the waypoint yaw) before the next leg. While the waypoint yaw is set the bridge holds
// a fixed point (the reached goal) instead of following SUPER, whose hover round a reached goal never
// stands still (Gate L2 b2); a waypoint with no yaw to face sends the yaw the vehicle has, so the bridge
// holds there too. A land leg does the same over its spot
// before it asks the landing check: it faces the drive leg that follows (so the tank starts aligned),
// or keeps its yaw. Not settled within ~yaw_settle_s (15 s): "yaw did not settle", the
// landing-not-clear path (QRLOITER, STOPPED). A leg's optional "yaw" (deg, fc frame) is a deliberate
// waypoint yaw at its end and overrides the direction of the next leg. Each new leg clears the waypoint
// yaw. A fly or land leg that starts with the bridge not streaming (after a takeoff, or a QRLOITER)
// first faces its goal the same way ("for": "leg start"). A scan leg ({type: scan, yaw_sweep_deg: 360}, optional scan_rate_deg, default ~scan_rate_deg 20)
// turns on the spot at a waypoint in the air, through the sweep, then settles (the occupancy-grid reason
// to yaw); it needs a fly leg before it.
//
// SUPER's re-seed (Gate L2 b7). SUPER takes a goal that comes while it follows a trajectory as a replan from
// that trajectory, and one that comes at rest with PlanFromRest. In b7, after the 3.55 s leg-start turn on
// leg 7, SUPER had long reached its post-takeoff hover point and sat at rest; its PlanFromRest toward leg 7
// (pole_b on the straight line 2.2 m ahead, in view once the nose faced it) found a path every time but its
// backup trajectory failed ("Pos violation", ~4600 times in 20 s), so it sent nothing, the vehicle held, the
// inputs never changed, and the leg went unreachable. b5, without the turn, gave SUPER leg 7's goal mid-climb
// and flew it. So after a leg-start turn the executive re-seeds SUPER (PLANNER_SEED, "why": "leg start"): a
// goal ~leg_start_seed_dz (0.1 m, owner 2026-10-29; was 0.3) straight over the vehicle: up from it when it is
// at or below the leg's planned altitude there, down toward that altitude when above it (mission.h
// seed_altitude). The container fly-through k1: 0.3 m seeds above the vehicle, each from where the last one
// took it, stacked 1.05 -> 1.35 -> 1.64 m under a 2.15 m roof; now repeats alternate within the plan +- 0.1 m
// and a seed is always a real 0.1 m move (SITL m2: a seed 5 mm from the vehicle left Gate L2 leg 7
// unreachable). At most ~seed_ceiling_clear (0.6 m: SUPER's 0.5 m inflation and a cell) below the bridge's
// "ceiling_above". Never more than 0.1 m below the leg's plan (b10's seeds below the vehicle at a low hover
// took it toward the inflated floor). Then,
// after ~leg_start_seed_s (0.5 s) with SUPER commanding again (the bridge's "planner_age" fresh) it sends the
// leg's goal, which SUPER then replans from its moving trajectory. The same on a flight leg whose planner has
// been silent ~planner_stuck_s (3 s; "planner_age") with the goal sent at least that long ago: re-seed, send
// the goal again, the no-progress clock restarted; at most ~planner_stuck_max (2) times a leg.
//
// Interactive mode (~interactive true, "click and run", 2026-10-30): no legs from the file (its defaults, name
// and takeoff_height are used), goals one at a time from /mission_executive/goal (std_msgs/String, JSON, fc
// frame): {"type": "drive", "goal": [x, y]} or {"type": "fly", "goal": [x, y, z], "yaw": deg (optional)},
// filled with the file's defaults and checked like a mission leg (a bad one: "goal_rejected" in the status).
// After /mission_executive/start it waits in GUIDED (IDLE) for a goal; each goal gets a number ("goal").
//   drive  on the ground: a drive leg. In the air: land first (face the goal, settle, the landing check on the
//          spot under the vehicle, else the landing-spot search below, NAV_LAND), then the drive leg
//   fly    on the ground: the bridge off, the takeoff (~takeoff_height of the file, the same standstill
//          rule), then the fly leg; at the goal the waypoint yaw (the goal's "yaw", or the yaw it has) and a
//          hover holding it until the next goal. From a hover the next fly leg first faces its goal
// A goal that comes while one runs replaces it (PREEMPTED) at the next check: during a fly / drive leg, a
// waypoint hover or the landing check; not during a takeoff or a landing descent (taken right after). Ends:
// REACHED, then IDLE. Unreachable (the same no-progress / timeout tests), a refused takeoff, a landing spot
// not reported clear, a yaw that did not settle: UNREACHABLE / REFUSED, in the air QRLOITER for
// ~unreachable_hold_s then GUIDED (bridge off), on the ground GUIDED with the bridge off; then IDLE. The FC
// leaving GUIDED, a dead planner, a frozen SLAM pose, a takeoff that did not complete and a bridge that
// did not switch stop for good (STOPPED), as in a mission. Nothing ends with a landing: the vehicle hovers
// or stands until the next goal.
//
// Landing to drive: the landing-spot search (owner request 2026-10-30). In the air, a drive (an interactive drive
// goal, or a mission drive leg that starts in the air; a mission's own land legs are unchanged) first lands:
//   1. over where the vehicle is: face the drive goal, settle, ask the landing check about the spot under the
//      vehicle for ~land_here_check_s (2 s). Clear: NAV_LAND (in GUIDED, as a land leg), then drive.
//   2. refused: LAND_SEARCH. /planner/landing_search (std_msgs/String JSON, planner frame: the vehicle, the drive
//      goal, ~land_search_radius (2 m), the spots already refused) to the landing check node, which evaluates
//      a grid of candidate spots within the radius with the landing check's own rules (l4_landing
//      landing_search.cpp: nearest and toward the goal first, ties to the most clear floor round the spot) and
//      answers on /planner/landing_search_result (and /planner/landing_search_markers). No answer within
//      ~land_search_timeout_s (10 s): as none found.
//   3. none found: LAND_SPIN, hovering in place (the bridge's waypoint hold), a 360 deg yaw at
//      ~land_spin_rate_deg (30 deg/s; the bridge caps 60) so the forward-facing lidar sees all round, then
//      face the goal and settle, and LAND_SEARCH again (the check node searches the last ~20 s of scans: the
//      whole spin). At most one spin a landing. Still none: "no landing spot found nearby" (REFUSED in
//      interactive mode: QRLOITER ~unreachable_hold_s, GUIDED, IDLE, as a refused spot; a mission stops in
//      QRLOITER as on a spot not reported clear).
//   4. found: LAND_MOVE, at the check height: a fly leg to the spot through SUPER, however short (owner
//      2026-11-02; ~land_move_direct_max, default 0, > 0 restores the former straight move through the bridge's
//      waypoint hold below that distance, land_move_direct) (go(): it faces the spot
//      first, so the nose-first approach keeps the spot in the lidar's view, and re-seeds SUPER, which then
//      plans a move this short from its moving trajectory), then the waypoint hold over the spot facing the
//      goal (the bridge holds the goal it was given when within its waypoint_goal_max 1 m, so the vehicle
//      eases onto the spot), and the landing check again on the spot under the vehicle (~landing_check_s).
//      Refused as "not observed" (the lidar looks forward: over the spot it no longer sees the floor under the
//      frame, and the 10 s window can lose the scans that chose it, SITL r5_c9): the same check once more as a
//      radius-0 search, i.e. with the search's ~20 s of scans; clear there counts.
//      The move's watchdog (2026-11-02): not 0.2 m closer within ~land_move_no_progress_s (4 s; SUPER's PlanFromRest
//      failing on it) or not there within ~land_move_timeout_s (30 s): LAND_MOVE "gave_up", a hold where the vehicle
//      is (also SUPER's goal), and back to 2 with that spot excluded (a move used up).
//      Clear: NAV_LAND, then the drive. Refused: back to 2 from there, the refused spots excluded, at most
//      ~land_search_tries (2) moves, then "landing spot not reported clear" (REFUSED / stop).
// ~land_search false: the former behaviour (step 1 with ~landing_check_s, refused ends it).
// "landing_check: false" on a mission land leg (owner 2026-11-03, SITL roof demo; default true): over the spot, faced
// and settled as any land leg, NAV_LAND at once: no check height, no landing check, no search (LAND_CHECK "skipped").
// A mission's land leg (owner 2026-11-02) lands the same way once over its spot: the check height, the check for
// ~land_here_check_s, else the search (the candidates facing the drive leg that follows, else the leg's own
// "yaw" or the yaw it has), the spin, the move, the check again; ~land_search false: its former check alone
// (~landing_check_s, refused ends the mission).
//
// The check height (owner 2026-11-02). The landing check drops lidar returns nearer than its near_range (1.2 m)
// to the sensor (on the real robot near floor reads 3-6 cm high); hovering at ~0.96 m that removed every floor
// return under the robot ("not observed" 22-29 %, "no ground plane"). So before the landing check (step 1, and a
// mission's land leg) the executive climbs to ~land_check_height (1.6 m) above the surface under the vehicle (the
// bridge's "floor_below": the nearest surface, a table top or the floor), at most up to ~land_check_ceiling_clear
// (0.6 m: SUPER's 0.5 m inflation and a cell, as the re-seed) below the bridge's "ceiling_above" (the lowest
// return over the vehicle: a ceiling, the underside of a table or a shelf); no room: it checks from the height it
// has. LAND_CLIMB: straight up through the bridge's waypoint hold (the goal straight over the vehicle, at most
// ~land_move_direct_max a step, as land_move_direct), facing the drive goal, until within 0.1 m of the target or
// ~yaw_settle_s; a ceiling that comes within ~land_check_ceiling_clear on the way stops it there. The search, the
// spin and the landing move then run at that height (they keep the height they have), the check after a landing
// move climbs again if the surface there is nearer (a table top), and NAV_LAND descends from there. Nothing to
// climb (less than ~land_check_climb_min, 0.1 m), no "floor_below" (an older bridge, too few returns under the
// vehicle) or ~land_check_height 0: no climb, the check from the height it has.
//
// The L2 fixes of 2026-11-02 evening (RESULTS_20261102.md, "Fixes"):
//   * the FC link (7c): /mavros/state silent ~fc_state_timeout_s (1.5 s) with the FC pose silent ~fc_pose_link_timeout_s,
//     or the state alone ~fc_state_max_s (3 s): "FC link lost", STOPPED at once, the bridge off, no resume;
//   * the bridge's heartbeat (1c, 1d, 10-28 note 9): enabled by us, its status (5 Hz) and its setpoints both silent
//     ~bridge_status_timeout_s (0.5 s): "planner bridge died". Driving: the executive streams the drive stop itself
//     (zero speed, zero turn rate, MASK_STOP 1479 on /mavros/setpoint_raw/local, a Guided setpoint, D5) until the
//     rover is at rest; flying: a hover target at the vehicle ~bridge_dead_hover_s, then QRLOITER; STOPPED;
//   * the drive stop after a stop on the ground (2d): every STOPPED and ground UNREACHABLE / REFUSED in GUIDED (not
//     with the FC link lost or the FC's own mode) disables the bridge and streams the drive stop until the FC's speed
//     stays under ~drive_stop_rest_speed for ~drive_stop_rest_s (at least ~drive_stop_min_s, at most
//     ~drive_stop_timeout_s);
//   * SLAM holds (3a, 10-28 note 5): a re-anchor hold ("reanchor" in the bridge's status) waits for a plan made after
//     it, and SUPER at rest plans only on a goal: with SLAM back the goal goes again after ~map_rebuild_s (then a
//     re-seed after ~planner_stuck_s silent), checked first (mission.h check_resent_goal: the executive's anchor
//     against the bridge's "planner_pos", the goal ~resend_floor_min above the surface under the vehicle). A SLAM hold
//     over ~hold_slam_timeout_s (20 s): QRLOITER ~unreachable_hold_s, then QRLAND (on the ground: the drive stop);
//     STOPPED "SLAM hold timed out";
//   * a fly goal inside an obstacle (4a): SUPER moved the goal (the bridge's "planner_goal") over ~goal_shift_max
//     (0.6 m: more than its 0.5 m inflation) up or down (sideways a plan also ends short when SUPER stops short of an
//     obstacle on the way: not judged; "planner_goal_clear" is reported only), with the vehicle within ~goal_end_near
//     of the plan's end and that end staying put (a long leg's plan ends at SUPER's horizon, a climb's first plans
//     part way up), for ~goal_clear_s: the leg ends, "goal in an obstacle" (on_unreachable applies). Any fly leg
//     given up in the air first backs off (BACK_OFF): up to
//     ~back_off_clear above the surface under the vehicle when nearer, through the bridge's waypoint hold, before
//     QRLOITER; and re-seeds stay ~seed_floor_clear above that surface.
//
// One difference from the Python node: a MAVROS service that cannot be called (MAVROS gone) counts as a
// refused command here (success false); the Python node raised ServiceException and died (RESULTS.md 7c).
#include <atomic>
#include <cmath>
#include <deque>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/StatusText.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <yaml-cpp/yaml.h>

#include <l4_common/params.h>
#include <l4_common/pyjson.h>

#include "l4_executive/mission.h"

namespace l4_executive {

using Vec3 = Eigen::Vector3d;
using Why = std::optional<std::string>;

// interactive: a newer goal replaced the running one
const char* const PREEMPTED = "preempted by a new goal";
const char* const LAND_NOT_CLEAR = "landing spot not reported clear";
// the landing-spot search found no clear spot within ~land_search_radius, after a 360 deg spin
const char* const NO_LAND_SPOT = "no landing spot found nearby";

bool starts_with(const std::string& s, const std::string& p) { return s.rfind(p, 0) == 0; }

l4::ojson vec_json(const Vec3& v) { return l4::ojson::array({v[0], v[1], v[2]}); }

// fc <- planner: p_fc = R p_planner + t
struct ExecAnchor {
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Vec3 t = Vec3::Zero();

  void set(const geometry_msgs::TransformStamped& msg) {
    const auto& q = msg.transform.rotation;
    const double x = q.x, y = q.y, z = q.z, w = q.w;
    R << 1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
         2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
         2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y);
    t = Vec3(msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z);
  }

  Vec3 fc_to_planner(const Vec3& p) const {
    const Vec3 d = p - t;
    Vec3 out;
    for (int i = 0; i < 3; ++i) out[i] = R(0, i) * d[0] + R(1, i) * d[1] + R(2, i) * d[2];
    return out;
  }

  Vec3 planner_to_fc(const Vec3& p) const { return R * p + t; }
};

class MissionExecutive {
 public:
  MissionExecutive() : pnh_("~") {
    std::string path;
    if (!pnh_.getParam("mission", path)) throw std::invalid_argument("~mission is required");
    mission_ = load_mission(path);
    mission_path_ = path;
    // interactive ("click and run"): goals from /mission_executive/goal, not the file's legs
    interactive_ = l4::param_bool(pnh_, "interactive", false);
    on_unreachable_ = l4::param_string(pnh_, "on_unreachable", "next");
    unreachable_hold_s_ = l4::param_double(pnh_, "unreachable_hold_s", 5.0);
    map_rebuild_s_ = l4::param_double(pnh_, "map_rebuild_s", 3.0);
    landing_check_s_ = l4::param_double(pnh_, "landing_check_s", 5.0);
    takeoff_timeout_s_ = l4::param_double(pnh_, "takeoff_timeout_s", 30.0);
    land_timeout_s_ = l4::param_double(pnh_, "land_timeout_s", 60.0);
    planner_frame_ = l4::param_string(pnh_, "planner_frame_id", "world");
    goal_resend_s_ = l4::param_double(pnh_, "goal_resend_s", 0.0);   // 0: send each goal once
    bridge_mode_timeout_s_ = l4::param_double(pnh_, "bridge_mode_timeout_s", 3.0);
    // after a drive Rover Guided runs on to its last target for a few seconds (cl32: ~6 s)
    takeoff_settle_s_ = l4::param_double(pnh_, "takeoff_settle_s", 15.0);
    // standing still before a takeoff (owner 2026-11-03, relaxed from 2 cm): the FC's horizontal pose moved less
    // than ~takeoff_still_m over the last ~takeoff_still_window_s, i.e. a mean speed under 0.05 m/s. Passes the
    // ~0.03 m/s creep Rover leaves under a zero-speed stop (it stops its speed loop under ATC_STOP_SPEED 0.1;
    // SITL: 2.4-3.6 cm per 1 s for 10 s, which failed 2 cm), refuses 0.1 m/s (10 cm) by a factor 2; QuadRover's own
    // guard refuses only above 0.2 m/s
    takeoff_still_m_ = l4::param_double(pnh_, "takeoff_still_m", 0.05);
    takeoff_still_window_s_ = l4::param_double(pnh_, "takeoff_still_window_s", 1.0);
    // drive legs: this far driven (not closer) also counts as progress; more than a pivot on the spot moves the FC
    drive_progress_m_ = l4::param_double(pnh_, "drive_progress_m", 0.5);
    // the first fly leg after a takeoff: SUPER gets its goal this long before the bridge streams, so its
    // first commands are from the new plan
    goal_settle_s_ = l4::param_double(pnh_, "goal_settle_s", 1.0);
    // waypoints (owner 2026-10-29): face the next leg, then hover with the yaw settled (rate under
    // ~yaw_settle_rate_deg and horizontal speed under ~yaw_settle_speed over 1 s, within ~yaw_tol_deg of
    // the waypoint yaw) before the next leg or the landing check; not within ~yaw_settle_s: stop
    yaw_settle_s_ = l4::param_double(pnh_, "yaw_settle_s", 15.0);
    yaw_settle_rate_ = l4::param_double(pnh_, "yaw_settle_rate_deg", 5.0) * (M_PI / 180.0);
    yaw_settle_speed_ = l4::param_double(pnh_, "yaw_settle_speed", 0.1);
    yaw_tol_ = l4::param_double(pnh_, "yaw_tol_deg", 10.0) * (M_PI / 180.0);
    waypoint_min_dist_ = l4::param_double(pnh_, "waypoint_min_dist", 0.3);   // nearer: no direction to face
    scan_rate_ = l4::param_double(pnh_, "scan_rate_deg", 20.0) * (M_PI / 180.0);
    // SUPER's re-seed (Gate L2 b7): a short vertical move from the vehicle, so that the leg's goal reaches
    // SUPER while it follows a trajectory, not at rest. After the leg start's turn, and when SUPER has been
    // silent ~planner_stuck_s on a flight leg (at most ~planner_stuck_max times a leg). 0 disables either
    seed_dz_ = l4::param_double(pnh_, "leg_start_seed_dz", 0.1);   // owner 2026-10-29 (was 0.3)
    seed_s_ = l4::param_double(pnh_, "leg_start_seed_s", 0.5);
    stuck_s_ = l4::param_double(pnh_, "planner_stuck_s", 3.0);
    stuck_max_ = l4::param_int(pnh_, "planner_stuck_max", 2);
    // the seed at most this far below the bridge's local ceiling ("ceiling_above"): SUPER's inflation (0.5 m)
    // and a cell
    seed_ceiling_clear_ = l4::param_double(pnh_, "seed_ceiling_clear", 0.6);
    // the mode for a frozen SLAM pose in the air (D5: QRLAND, QRLOITER, QRSMARTRTL or GUIDED)
    frozen_flight_mode_ = l4::param_string(pnh_, "slam_frozen_flight_mode", "QRLAND");
    if (!allowed_mode(frozen_flight_mode_))
      throw std::invalid_argument("~slam_frozen_flight_mode: D5 allows GUIDED, QRLOITER, QRLAND, QRSMARTRTL only");
    // landing to drive: the landing-spot search (header comment)
    land_search_ = l4::param_bool(pnh_, "land_search", true);
    land_search_radius_ = l4::param_double(pnh_, "land_search_radius", 2.0);
    land_here_check_s_ = l4::param_double(pnh_, "land_here_check_s", 2.0);
    land_search_timeout_s_ = l4::param_double(pnh_, "land_search_timeout_s", 10.0);
    land_search_tries_ = l4::param_int(pnh_, "land_search_tries", 2);
    land_spin_rate_ = l4::param_double(pnh_, "land_spin_rate_deg", 30.0) * (M_PI / 180.0);   // 0: no spin
    // a landing move this short goes straight through the bridge's waypoint hold, not SUPER (SITL c5, c7: SUPER
    // stayed silent on a 0.6 m goal from a hover, PlanFromRest as in Gate L2 b7); below the bridge's
    // ~waypoint_goal_max (1.0 m), within which it holds the goal it was given. 0: always SUPER
    // owner 2026-11-02: every landing move flies through SUPER (0); > 0 restores the direct hold move below it
    land_move_direct_max_ = l4::param_double(pnh_, "land_move_direct_max", 0.0);
    // the check height (header comment): above the surface under the vehicle, capped below the ceiling over it
    land_check_height_ = l4::param_double(pnh_, "land_check_height", 1.6);   // 0: no climb
    land_check_ceiling_clear_ = l4::param_double(pnh_, "land_check_ceiling_clear", 0.6);
    land_check_climb_min_ = l4::param_double(pnh_, "land_check_climb_min", 0.1);
    // a climb step through the bridge's waypoint hold: below its ~waypoint_goal_max (1.0 m)
    land_check_climb_step_ = l4::param_double(pnh_, "land_check_climb_step", 0.9);
    // a landing move's watchdog: not 0.2 m closer within this (SUPER not planning it), or not there within the
    // timeout: that spot is given up, the next candidate searched
    land_move_no_progress_s_ = l4::param_double(pnh_, "land_move_no_progress_s", 4.0);
    land_move_timeout_s_ = l4::param_double(pnh_, "land_move_timeout_s", 30.0);
    // the links (L2 7c, 1d, 2026-11-02 evening): /mavros/state silent ~fc_state_timeout_s with the FC pose silent
    // ~fc_pose_link_timeout_s too (MAVROS gone: everything stops), or the state alone ~fc_state_max_s: "FC link lost",
    // STOPPED at once, no resume. The state comes with the FC's 1 Hz heartbeat: SITL gaps up to 2.06 s, the FC pose's
    // up to 1.11 s (lf01, Shannon's load); 1.0 / 0.5 s gave a false loss in SITL fx00
    fc_state_timeout_s_ = l4::param_double(pnh_, "fc_state_timeout_s", 1.5);
    fc_pose_link_timeout_s_ = l4::param_double(pnh_, "fc_pose_link_timeout_s", 1.5);
    fc_state_max_s_ = l4::param_double(pnh_, "fc_state_max_s", 3.0);
    // the planner bridge's status (5 Hz) silent ~bridge_status_timeout_s while it streams for us, and none of its
    // setpoints for as long: the bridge died. Driving: the executive streams the drive stop itself until the rover
    // is at rest; flying: a hover target at the vehicle ~bridge_dead_hover_s, then QRLOITER; STOPPED
    bridge_status_timeout_s_ = l4::param_double(pnh_, "bridge_status_timeout_s", 0.5);
    bridge_dead_hover_s_ = l4::param_double(pnh_, "bridge_dead_hover_s", 1.0);
    // the drive stop on the ground (every STOPPED and ground UNREACHABLE; L2 1d, 2d): zero speed, zero turn rate
    // (MASK_STOP 1479, Rover Guided TurnRateAndSpeed) at 20 Hz, at least ~drive_stop_min_s and until the FC's speed
    // stays under ~drive_stop_rest_speed for ~drive_stop_rest_s, at most ~drive_stop_timeout_s
    drive_stop_min_s_ = l4::param_double(pnh_, "drive_stop_min_s", 1.0);
    drive_stop_rest_speed_ = l4::param_double(pnh_, "drive_stop_rest_speed", 0.05);
    drive_stop_rest_s_ = l4::param_double(pnh_, "drive_stop_rest_s", 1.0);
    drive_stop_timeout_s_ = l4::param_double(pnh_, "drive_stop_timeout_s", 10.0);
    // a SLAM hold (HOLD_SLAM) longer than this: in the air QRLOITER ~unreachable_hold_s, then QRLAND; on the ground
    // the drive stop; STOPPED "SLAM hold timed out" (10-28 note 5). 0: no timeout
    hold_slam_timeout_s_ = l4::param_double(pnh_, "hold_slam_timeout_s", 20.0);
    // a goal re-sent after a SLAM hold: the executive's anchor within ~resend_anchor_tol of the bridge's frame (its
    // "planner_pos"), the goal at least ~resend_floor_min above the surface under the vehicle (mission.h
    // check_resent_goal)
    resend_anchor_tol_ = l4::param_double(pnh_, "resend_anchor_tol", 0.3);
    resend_floor_min_ = l4::param_double(pnh_, "resend_floor_min", 0.3);
    // a fly goal inside an obstacle (L2 4a): with the vehicle within ~goal_end_near of SUPER's plan's end (the
    // bridge's "planner_goal"), the end staying within ~goal_end_steady, that end over ~goal_shift_max above or below our goal for ~goal_clear_s: the leg ends,
    // "goal in an obstacle"; 0: off. Then (any unreachable in the air) the back-off: up to ~back_off_clear above
    // the surface under the vehicle (SUPER's 0.5 m inflation and a margin) when nearer, under the ceiling
    goal_clear_s_ = l4::param_double(pnh_, "goal_clear_s", 1.0);
    goal_shift_max_ = l4::param_double(pnh_, "goal_shift_max", 0.6);   // moved more: in an obstacle, scans or not
    goal_end_near_ = l4::param_double(pnh_, "goal_end_near", 0.35);    // judged with the vehicle at the plan's end
    goal_end_steady_ = l4::param_double(pnh_, "goal_end_steady", 0.1); // ... and that end staying within this
    back_off_clear_ = l4::param_double(pnh_, "back_off_clear", 0.7);
    // a re-seed at least this far above the surface under the vehicle (mission.h seed_altitude)
    seed_floor_clear_ = l4::param_double(pnh_, "seed_floor_clear", 0.6);
    bridge_ = {{"state", "OK"}, {"slam_ok", true}};
    bridge_ok_since_ = now();

    subs_.push_back(nh_.subscribe("/mavros/state", 10, &MissionExecutive::on_state, this));
    subs_.push_back(nh_.subscribe("/mavros/local_position/pose", 10, &MissionExecutive::on_pose, this));
    subs_.push_back(nh_.subscribe("/mavros/statustext/recv", 50, &MissionExecutive::on_text, this));
    subs_.push_back(nh_.subscribe("/planner_bridge/status", 10, &MissionExecutive::on_bridge, this));
    subs_.push_back(nh_.subscribe("/planner_bridge/anchor", 10, &MissionExecutive::on_anchor, this));
    subs_.push_back(nh_.subscribe(l4::param_string(pnh_, "landing_check_topic", "/planner/landing_clear"), 10,
                                  &MissionExecutive::on_landing_clear, this));
    subs_.push_back(nh_.subscribe("/planner/landing_status", 10, &MissionExecutive::on_landing_status, this));
    subs_.push_back(nh_.subscribe("/track_planner/status", 10, &MissionExecutive::on_track_status, this));
    subs_.push_back(nh_.subscribe("/planner/landing_search_result", 10, &MissionExecutive::on_search_result, this));
    // the bridge's setpoints (not ours): the bridge-death check's second witness
    subs_.push_back(nh_.subscribe<mavros_msgs::PositionTarget>(
        "/mavros/setpoint_raw/local", 20,
        boost::function<void(const ros::MessageEvent<const mavros_msgs::PositionTarget>&)>(
            [this](const ros::MessageEvent<const mavros_msgs::PositionTarget>& e) {
              if (e.getPublisherName() == ros::this_node::getName()) return;
              std::lock_guard<std::mutex> g(lock_);
              t_sp_bridge_ = now();
            })));
    sp_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/planning/click_goal", 1, true);
    drive_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/planning/drive_goal", 1, true);
    leg_pub_ = nh_.advertise<std_msgs::String>("/mission_executive/leg", 1, true);
    spot_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/planner/landing_spot", 1);
    search_pub_ = nh_.advertise<std_msgs::String>("/planner/landing_search", 1);
    status_pub_ = nh_.advertise<std_msgs::String>("/mission_executive/status", 10, true);
    enable_pub_ = nh_.advertise<std_msgs::Bool>("/planner_bridge/enable", 1, true);
    publish_enable(false);
    mode_pub_ = nh_.advertise<std_msgs::String>("/planner_bridge/set_mode", 1, true);
    ground_pub_ = nh_.advertise<std_msgs::String>("/mission_executive/ground_spots", 1, true);
    yaw_pub_ = nh_.advertise<std_msgs::Float64>("/planner_bridge/yaw_cmd", 1, true);
    publish_yaw(std::nullopt);
    start_srv_ = nh_.advertiseService("/mission_executive/start", &MissionExecutive::on_start, this);
    if (interactive_) subs_.push_back(nh_.subscribe("/mission_executive/goal", 10, &MissionExecutive::on_goal, this));
    cmd_ = nh_.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    takeoff_srv_ = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    land_srv_ = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
  }

  void run();

 private:
  static double now() { return ros::Time::now().toSec(); }
  static void sleep(double s) {
    if (s > 0) ros::Duration(s).sleep();
  }
  static bool shutdown() { return !ros::ok(); }

  // ---- inputs ------------------------------------------------------------------------------------
  void on_state(const mavros_msgs::State::ConstPtr& m) {
    std::lock_guard<std::mutex> g(lock_);
    state_ = *m;
    t_state_ = now();
  }

  void on_pose(const geometry_msgs::PoseStamped::ConstPtr& m) {
    const auto& p = m->pose.position;
    const auto& q = m->pose.orientation;
    std::lock_guard<std::mutex> g(lock_);
    pose_ = Vec3(p.x, p.y, p.z);
    t_pose_ = now();
    yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  void on_text(const mavros_msgs::StatusText::ConstPtr& m) {
    std::lock_guard<std::mutex> g(lock_);
    texts_.push_back(m->text);
  }

  void on_bridge(const std_msgs::String::ConstPtr& m) {
    l4::ojson s = l4::loads(m->data);
    if (s.is_discarded() || !s.is_object()) return;
    // the bridge's own flag; without it (an older or fake sender) only HOLD counts
    s["slam_ok"] = !l4::truthy(l4::get(s, "slam_hold", l4::get(s, "state") == "HOLD"));
    std::lock_guard<std::mutex> g(lock_);
    if (s["slam_ok"].get<bool>() && !l4::truthy(l4::get(bridge_, "slam_ok", true))) bridge_ok_since_ = now();
    bridge_ = s;
    t_bridge_ = now();
  }

  void on_anchor(const geometry_msgs::TransformStamped::ConstPtr& m) {
    std::lock_guard<std::mutex> g(lock_);
    anchor_.set(*m);
  }

  void on_landing_clear(const std_msgs::Bool::ConstPtr& m) {
    std::lock_guard<std::mutex> g(lock_);
    landing_clear_ = static_cast<bool>(m->data);
  }

  void on_landing_status(const std_msgs::String::ConstPtr& m) {
    l4::ojson s = l4::loads(m->data);
    if (s.is_discarded()) return;
    std::lock_guard<std::mutex> g(lock_);
    landing_status_ = s;
  }

  void on_search_result(const std_msgs::String::ConstPtr& m) {
    l4::ojson s = l4::loads(m->data);
    if (s.is_discarded() || !s.is_object()) return;
    std::lock_guard<std::mutex> g(lock_);
    search_result_ = s;
  }

  void on_track_status(const std_msgs::String::ConstPtr& m) {
    l4::ojson s = l4::loads(m->data);
    if (s.is_discarded()) return;
    std::lock_guard<std::mutex> g(lock_);
    track_ = s;
    if (l4::get(s, "state") == "DRIVING" && l4::get(s, "reason") == "new goal") track_goals_ += 1;
  }

  // interactive: a goal (JSON, fc frame) checked and filled like a mission leg; it replaces any pending one
  void on_goal(const std_msgs::String::ConstPtr& m) {
    const l4::ojson g = l4::loads(m->data);
    std::string err;
    l4::ojson leg;
    if (g.is_discarded() || !g.is_object()) {
      err = "not a JSON object";
    } else if (!(l4::get(g, "type") == "fly" || l4::get(g, "type") == "drive")) {
      err = "an interactive goal is a fly or a drive";
    } else {
      try {
        // the file's defaults, the goal as its only leg: the mission file's own checks
        l4::ojson doc = yaml_to_json(YAML::LoadFile(mission_path_));
        doc["legs"] = l4::ojson::array({g});
        leg = load_mission_text(l4::dumps(doc), mission_path_).legs.at(0);
      } catch (const std::exception& e) {
        err = e.what();
      }
    }
    if (!err.empty()) {
      report({{"goal_rejected", err}});
      return;
    }
    int id;
    {
      std::lock_guard<std::mutex> lk(lock_);
      id = ++goal_seq_;
      leg["goal_id"] = id;
      pending_ = leg;
    }
    report({{"goal_received", id}, {"type", leg["type"]}, {"goal_fc", leg["goal"]}, {"yaw", l4::get(leg, "yaw")}});
  }

  // interactive: a newer goal is waiting (the running one ends PREEMPTED)
  bool preempted() {
    if (!interactive_) return false;
    std::lock_guard<std::mutex> lk(lock_);
    return pending_.has_value();
  }

  bool on_start(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    std::optional<mavros_msgs::State> st;
    {
      std::lock_guard<std::mutex> g(lock_);
      st = state_;
    }
    const std::string phase = get_phase();
    if (phase != "WAIT_ARMED") {
      res.success = false;
      res.message = "mission already " + phase;
      return true;
    }
    if (!st || !st->connected || !st->armed) {
      res.success = false;
      res.message = "the pilot arms first (the executive never arms)";
      return true;
    }
    started_ = true;
    res.success = true;
    const l4::ojson& name = mission_.name;
    res.message = "mission " + (name.is_string() ? name.get<std::string>() : l4::dumps(name)) + " started";
    return true;
  }

  // ---- helpers -----------------------------------------------------------------------------------
  std::string get_phase() {
    std::lock_guard<std::mutex> g(phase_lock_);
    return phase_;
  }

  // report(phase=..., key=value, ...): the keys in the order given; "phase" is the current phase when
  // not given (appended last, as Python's kw["phase"] = ...)
  void report(l4::ojson kw) {
    {
      std::lock_guard<std::mutex> g(phase_lock_);
      if (kw.contains("phase")) phase_ = kw["phase"].get<std::string>();
      kw["phase"] = phase_;
    }
    std_msgs::String m;
    m.data = l4::dumps(kw);
    status_pub_.publish(m);
    ROS_INFO("[mission] %s", m.data.c_str());
  }

  std::optional<int> mode() {
    std::lock_guard<std::mutex> g(lock_);
    if (!state_) return std::nullopt;
    return mode_number(state_->mode);
  }

  // why the FC has taken the vehicle (mode left what we set) or MAVROS lost it; none while ours
  Why fc_lost() {
    std::optional<mavros_msgs::State> st;
    std::optional<double> ts, tp;
    {
      std::lock_guard<std::mutex> g(lock_);
      st = state_;
      ts = t_state_;
      tp = t_pose_;
    }
    // MAVROS gone (L2 7c): its /mavros/state just stops ("connected" stays true in the last message)
    if (ts) {
      const double t = now(), as = t - *ts, ap = tp ? t - *tp : 1e9;
      if ((as > fc_state_timeout_s_ && ap > fc_pose_link_timeout_s_) || as > fc_state_max_s_) {
        if (!link_lost_reported_) {
          link_lost_reported_ = true;
          report({{"fc_link_lost", true}, {"state_age_s", l4::py_round(as, 2)}, {"pose_age_s", l4::py_round(std::min(ap, 1e3), 2)}});
        }
        return std::string(FC_LINK_LOST) + fmt_s(" (MAVROS silent %.1f s)", as);
      }
    }
    if (!st || !st->connected || !st->armed) return std::string("FC link lost or disarmed");
    const auto m = mode_number(st->mode);
    if (our_mode_ && m != modes().at(*our_mode_)) return "FC mode changed to " + st->mode;
    return std::nullopt;
  }

  // the bridge (enabled for this leg) reports a frozen SLAM pose: the mission ends (the caller lands or stops)
  Why slam_frozen(int idx) {
    l4::ojson bridge;
    {
      std::lock_guard<std::mutex> g(lock_);
      bridge = bridge_;
    }
    if (l4::get(bridge, "reason") != "slam_frozen" || !l4::truthy(l4::get(bridge, "enabled", true))) return std::nullopt;
    report({{"phase", "LEG"}, {"leg", idx}, {"slam_frozen", l4::get(bridge, "slam_frozen", true)}, {"flying", flying_}});
    return std::string(SLAM_FROZE);
  }

  static std::string fmt_s(const char* f, double v) {
    char b[96];
    std::snprintf(b, sizeof b, f, v);
    return b;
  }

  // the planner bridge died (L2 1c, 1d; 10-28 note 9): enabled by us, its status silent ~bridge_status_timeout_s and
  // none of its setpoints for as long
  Why bridge_dead(int idx) {
    if (!streaming_ || bridge_status_timeout_s_ <= 0) return std::nullopt;
    std::optional<double> tb, tsp;
    {
      std::lock_guard<std::mutex> g(lock_);
      tb = t_bridge_;
      tsp = t_sp_bridge_;
    }
    const double t = now();
    const double ab = tb ? t - *tb : 1e9, asp = tsp ? t - *tsp : 1e9;
    if (!(ab > bridge_status_timeout_s_ && asp > bridge_status_timeout_s_)) return std::nullopt;
    report({{"phase", "LEG"}, {"leg", idx}, {"bridge_dead", true}, {"status_age_s", l4::py_round(std::min(ab, 1e3), 2)},
            {"setpoint_age_s", l4::py_round(std::min(asp, 1e3), 2)}, {"flying", flying_}});
    return std::string(BRIDGE_DIED);
  }

  // what ends a leg or a hover at once: a frozen SLAM pose, a dead bridge
  Why watch(int idx) {
    if (auto fz = slam_frozen(idx)) return fz;
    return bridge_dead(idx);
  }

  // one setpoint of the executive's own (D5: Guided setpoints): the drive stop (zero speed, zero turn rate, MASK_STOP
  // 1479, as the bridge's) or a hover target (position, zero velocity and acceleration, yaw; MASK_FLIGHT 2048)
  void send_drive_stop() {
    mavros_msgs::PositionTarget sp;
    sp.header.stamp = ros::Time::now();
    sp.header.frame_id = "map";
    sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    sp.type_mask = 1 | 2 | 4 | 64 | 128 | 256 | 1024;   // no position, no acceleration, no yaw: velocity + yaw rate
    sp.yaw_rate = 0.0f;
    sp_pub_.publish(sp);
  }
  void send_hover(const Vec3& p, double yaw) {
    mavros_msgs::PositionTarget sp;
    sp.header.stamp = ros::Time::now();
    sp.header.frame_id = "map";
    sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    sp.type_mask = 2048;   // position + velocity + acceleration + yaw (QuadRover Guided flight)
    sp.position.x = p[0];
    sp.position.y = p[1];
    sp.position.z = p[2];
    sp.yaw = static_cast<float>(yaw);
    sp_pub_.publish(sp);
  }

  // on the ground in GUIDED: the bridge off and the drive stop streamed by the executive until the rover is at rest
  // (the FC's horizontal speed under ~drive_stop_rest_speed for ~drive_stop_rest_s), at least ~drive_stop_min_s, at
  // most ~drive_stop_timeout_s (L2 2d: the bridge's latched stop lasted 0.2 s after STOPPED, the rover coasted
  // 3.6 m; 1d: the bridge dead, Rover Guided crept round its last target). Not with the FC link lost, not outside
  // GUIDED (the FC's own mode owns the vehicle), not in the air
  void drive_stop_until_rest(const std::string& why) {
    if (flying_) return;
    {
      std::lock_guard<std::mutex> g(lock_);
      if (!state_ || state_->mode != "GUIDED") return;
    }
    if (fc_lost()) return;
    bridge_enable(false);
    report({{"drive_stop", "streaming"}, {"reason", why}});
    const double t0 = now();
    std::deque<std::pair<double, Vec3>> seen;
    double rest_since = -1.0, speed = 0.0;
    bool at_rest = false;
    while (!shutdown() && now() - t0 < drive_stop_timeout_s_) {
      if (fc_lost()) break;
      send_drive_stop();
      const auto p = pose_copy();
      const double t = now();
      if (p) {
        seen.emplace_back(t, *p);
        while (seen.size() > 2 && t - seen.front().first > 0.5) seen.pop_front();
        const double dt = t - seen.front().first;
        speed = dt > 0.2 ? (seen.back().second - seen.front().second).head<2>().norm() / dt : 1e9;
        if (speed < drive_stop_rest_speed_) {
          if (rest_since < 0) rest_since = t;
        } else {
          rest_since = -1.0;
        }
        // a frozen SLAM pose: the FC's speed follows it (reads ~0 while the rover rolls on, SITL fx10): the stop
        // streams for the whole ~drive_stop_timeout_s
        if (why != SLAM_FROZE && t - t0 >= drive_stop_min_s_ && rest_since >= 0 && t - rest_since >= drive_stop_rest_s_) {
          at_rest = true;
          break;
        }
      }
      sleep(0.05);
    }
    report({{"drive_stop", "done"}, {"reason", why}, {"at_rest", at_rest}, {"speed", l4::py_round(std::min(speed, 1e3), 3)},
            {"stop_s", l4::py_round(now() - t0, 2)}});
  }

  // the bridge died in the air: a hover target at the vehicle (where it is now) ~bridge_dead_hover_s, then QRLOITER
  void hover_then_loiter() {
    const auto p = pose_copy();
    std::optional<double> y;
    {
      std::lock_guard<std::mutex> g(lock_);
      y = yaw_;
    }
    if (p && y && !fc_lost()) {
      report({{"hover", vec_json(*p)}, {"why", BRIDGE_DIED}});
      const double t0 = now();
      while (!shutdown() && now() - t0 < bridge_dead_hover_s_ && !fc_lost()) {
        send_hover(*p, *y);
        sleep(0.05);
      }
    }
    set_mode("QRLOITER");
  }

  void publish_enable(bool on) {
    std_msgs::Bool b;
    b.data = on;
    enable_pub_.publish(b);
  }

  void bridge_enable(bool on) {
    streaming_ = on;
    publish_enable(on);
  }

  void publish_set_mode(const std::string& name) {
    std_msgs::String s;
    s.data = name;
    mode_pub_.publish(s);
  }

  // the bridge's mode for the next leg (flight | drive); true once the bridge reports it (an older
  // bridge that does not report a mode is taken at its word)
  bool bridge_mode(const std::string& name) {
    bridge_mode_now_ = name;
    publish_set_mode(name);
    const double t0 = now();
    while (!shutdown() && now() - t0 < bridge_mode_timeout_s_) {
      l4::ojson got;
      bool fresh;
      {
        std::lock_guard<std::mutex> g(lock_);
        got = l4::get(bridge_, "mode", name);
        // a live bridge (its 5 Hz status): a dead one's last status is not its word
        fresh = !t_bridge_ || now() - *t_bridge_ <= std::max(1.0, 2 * bridge_status_timeout_s_);
      }
      if (got == name && fresh) return true;
      sleep(0.05);
    }
    return false;
  }

  // wait until the vehicle has stopped (moved < ~takeoff_still_m over the last ~takeoff_still_window_s: QuadRover
  // refuses a takeoff only above 0.2 m/s, and Rover Guided pauses and moves on again while it settles on its last
  // target); false after timeout
  bool standstill(double timeout) {
    const double t0 = now();
    std::deque<std::pair<double, Vec3>> seen;
    while (!shutdown() && now() - t0 < timeout) {
      std::optional<Vec3> p;
      {
        std::lock_guard<std::mutex> g(lock_);
        p = pose_;
      }
      const double t = now();
      if (p) {
        std::deque<std::pair<double, Vec3>> keep;
        for (const auto& s : seen)
          if (t - s.first <= takeoff_still_window_s_ + 0.5) keep.push_back(s);
        keep.emplace_back(t, *p);
        seen.swap(keep);
        double maxd = 0.0;
        for (const auto& s : seen)
          if (t - s.first <= takeoff_still_window_s_) maxd = std::max(maxd, (s.second.head<2>() - p->head<2>()).norm());
        if (t - seen.front().first >= takeoff_still_window_s_ && maxd < takeoff_still_m_) return true;
      }
      sleep(0.1);
    }
    return false;
  }

  // the vehicle sits here on the ground: known floor for the landing check
  void record_ground_spot(const std::string& event) {
    l4::ojson spot;
    {
      std::lock_guard<std::mutex> g(lock_);
      if (!pose_ || !yaw_) return;
      spot["event"] = event;
      spot["xyz"] = {l4::py_round((*pose_)[0], 3), l4::py_round((*pose_)[1], 3), l4::py_round((*pose_)[2], 3)};
      spot["yaw"] = l4::py_round(*yaw_, 4);
      spot["t"] = l4::py_round(now(), 2);
    }
    ground_spots_.push_back(spot);
    l4::ojson doc;
    doc["frame"] = "fc";
    doc["spots"] = ground_spots_;
    std_msgs::String m;
    m.data = l4::dumps(doc);
    ground_pub_.publish(m);
  }

  bool set_mode(const std::string& name) {
    if (!allowed_mode(name)) throw std::runtime_error("D5: the executive may not set " + name);
    bridge_enable(false);
    our_mode_ = name;
    mavros_msgs::CommandLong srv;
    srv.request.broadcast = false;
    srv.request.command = MAV_CMD_DO_SET_MODE;
    srv.request.confirmation = 0;
    srv.request.param1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    srv.request.param2 = static_cast<float>(modes().at(name));
    srv.request.param3 = srv.request.param4 = srv.request.param5 = srv.request.param6 = srv.request.param7 = 0;
    if (!cmd_.call(srv)) {
      ROS_ERROR("[mission] /mavros/cmd/command (DO_SET_MODE %s) could not be called", name.c_str());
      srv.response.success = false;
    }
    const double t0 = now();
    while (!shutdown() && now() - t0 < 5.0) {
      if (mode() == modes().at(name)) return true;
      sleep(0.05);
    }
    ROS_WARN("[mission] mode %s not confirmed (command success %s)", name.c_str(),
             srv.response.success ? "True" : "False");
    return false;
  }

  bool wait_text(const std::string& text, double timeout) {
    const double t0 = now();
    while (!shutdown() && now() - t0 < timeout) {
      {
        std::lock_guard<std::mutex> g(lock_);
        for (const auto& x : texts_)
          if (x.find(text) != std::string::npos) return true;
      }
      if (fc_lost()) return false;
      sleep(0.05);
    }
    return false;
  }

  Vec3 publish_goal(const Vec3& p_fc, bool drive = false) {
    Vec3 p;
    {
      std::lock_guard<std::mutex> g(lock_);
      p = anchor_.fc_to_planner(p_fc);
    }
    geometry_msgs::PoseStamped m;
    m.header.stamp = ros::Time::now();
    m.header.frame_id = planner_frame_;
    m.pose.position.x = p[0];
    m.pose.position.y = p[1];
    m.pose.position.z = p[2];
    m.pose.orientation.w = 1.0;
    (drive ? drive_pub_ : goal_pub_).publish(m);
    return p;
  }

  std::optional<Vec3> pose_copy() {
    std::lock_guard<std::mutex> g(lock_);
    return pose_;
  }

  // ---- legs --------------------------------------------------------------------------------------
  Why takeoff(const l4::ojson& height);
  Why go(const l4::ojson& leg, int idx, const Vec3& target, bool drive);
  Why land(const l4::ojson& leg, int idx);
  Why land_check(double x, double y, int idx, double timeout, l4::ojson& check);
  Why land_descend(int idx, const l4::ojson& check);
  Why land_unchecked(int idx);
  Why land_to_drive(const l4::ojson& leg, int idx, double gx, double gy);
  Why land_search(int idx, double gx, double gy, const std::vector<Vec3>& tried, int n, std::optional<Vec3>& spot,
                  std::optional<double> radius = std::nullopt);
  Why land_spin(int idx, double gx, double gy);
  Why land_move_direct(int idx, const Vec3& spot, double gx, double gy);
  Why land_climb(int idx, std::optional<double> yaw, std::optional<double> height = std::nullopt,
                 const char* phase = "LAND_CLIMB");
  Why back_off(int idx, const std::string& why);
  Why land_here_or_nearby(const l4::ojson& leg, int idx, double gx, double gy);
  // the landing heading: a land leg's own yaw (land_yaw_), else facing the drive goal (gx, gy; NaN: none: the
  // yaw it has)
  std::optional<double> land_face(double gx, double gy) {
    if (land_yaw_) return land_yaw_;
    if (!std::isfinite(gx) || !std::isfinite(gy)) return std::nullopt;
    return heading_to(gx, gy);
  }
  std::optional<double> land_yaw_;
  Why waypoint(int idx, std::optional<double> yaw, const std::string& why);
  Why scan(const l4::ojson& leg, int idx);
  void run_interactive();
  Why interactive_goal(const l4::ojson& leg, int n, bool& hovering);
  // the mission stops for good. On the ground in GUIDED the drive stop goes on after STOPPED, streamed by the executive,
  // until the rover is at rest (drive_stop_until_rest)
  void stop(const std::string& why) {
    bridge_enable(false);
    report({{"phase", "STOPPED"}, {"reason", why}});
    if (!starts_with(why, "FC ")) drive_stop_until_rest(why);
  }

  // the waypoint yaw for the bridge (fc frame, rad), or none (NaN: the bridge holds the yaw it has)
  void publish_yaw(std::optional<double> yaw) {
    std_msgs::Float64 f;
    f.data = yaw ? *yaw : std::numeric_limits<double>::quiet_NaN();
    yaw_held_ = yaw.has_value();
    yaw_pub_.publish(f);
  }

  // the heading from the vehicle to p (fc frame), or none within waypoint_min_dist
  std::optional<double> heading_to(double x, double y) {
    const auto p = pose_copy();
    if (!p || std::hypot(x - (*p)[0], y - (*p)[1]) < waypoint_min_dist_) return std::nullopt;
    return std::atan2(y - (*p)[1], x - (*p)[0]);
  }

  // re-seed SUPER (Gate L2 b7): a goal straight over the vehicle, ~leg_start_seed_dz up from it (or down, when
  // it is above the leg's planned altitude; start: where the leg began; mission.h seed_altitude: repeats
  // never stack, k1), at most
  // ~seed_ceiling_clear below the bridge's local ceiling, then ~leg_start_seed_s with SUPER commanding before the caller sends the
  // leg's goal. SUPER handles a goal that comes while it follows a trajectory as a replan from that
  // trajectory; one that comes at rest goes to its PlanFromRest, whose backup trajectory failed every
  // attempt for 20 s in b7 (leg 7, pole_b on the line 2.2 m ahead: "Pos violation") with nothing sent, so
  // the vehicle never moved and nothing changed. A vertical move toward no obstacle is planned from rest
  // without that backup (all of it in view), and the leg's goal then comes mid-move (as in b5, mid-climb).
  // False when there is no seed to send (disabled, no pose).
  bool seed_planner(int idx, const Vec3& start, const Vec3& target, const std::string& why) {
    const auto here = pose_copy();
    if (seed_dz_ <= 0 || !here) return false;
    std::optional<double> ceiling;
    {
      std::lock_guard<std::mutex> g(lock_);
      const l4::ojson c = l4::get(bridge_, "ceiling_above");
      if (c.is_number()) ceiling = c.get<double>();
    }
    std::optional<double> floor;
    {
      std::lock_guard<std::mutex> g(lock_);
      const l4::ojson f = l4::get(bridge_, "floor_below");
      if (f.is_number()) floor = f.get<double>();
    }
    const SeedAltitude sa = seed_altitude(start, target, *here, seed_dz_, ceiling, seed_ceiling_clear_, floor, seed_floor_clear_);
    const Vec3 seed((*here)[0], (*here)[1], sa.z);
    const Vec3 planner = publish_goal(seed);
    report({{"phase", "PLANNER_SEED"}, {"leg", idx}, {"why", why}, {"seed_fc", vec_json(seed)}, {"seed_planner", vec_json(planner)},
            {"plan_z", l4::py_round(sa.plan_z, 3)}, {"ceiling_above", ceiling ? l4::ojson(*ceiling) : l4::ojson(nullptr)},
            {"capped", sa.capped}, {"floored", sa.floored}});
    // SUPER commanding again (the bridge's planner_age fresh; an older bridge without it: the time alone)
    const double t0 = now();
    while (!shutdown() && now() - t0 < seed_s_ + 2.0) {
      l4::ojson age;
      {
        std::lock_guard<std::mutex> g(lock_);
        age = l4::get(bridge_, "planner_age");
      }
      if (now() - t0 >= seed_s_ && (!age.is_number() || age.get<double>() <= 0.2)) break;
      sleep(0.05);
    }
    return true;
  }

  // a goal sent again after a SLAM hold, checked first (mission.h check_resent_goal, L2 3a): the executive's anchor
  // against the bridge's frame (its "planner_pos"), the goal's height above the surface under the vehicle. False
  // (not sent; asked again next time) while the anchor is not the bridge's
  bool resend_goal(int idx, const std::string& kind, const Vec3& target, bool drive, Vec3& planner, const char* why) {
    const auto here = pose_copy();
    if (!here) return false;
    std::optional<Vec3> ppos;
    std::optional<double> floor;
    Vec3 conv;
    {
      std::lock_guard<std::mutex> g(lock_);
      const l4::ojson pp = l4::get(bridge_, "planner_pos"), f = l4::get(bridge_, "floor_below");
      if (pp.is_array() && pp.size() == 3) ppos = Vec3(num(pp[0]), num(pp[1]), num(pp[2]));
      if (f.is_number()) floor = f.get<double>();
      conv = anchor_.fc_to_planner(*here);
    }
    const GoalCheck c = check_resent_goal(target, *here, conv, ppos, drive ? std::nullopt : floor, resend_anchor_tol_,
                                          resend_floor_min_);
    if (!c.ok) {
      if (now() - resend_wait_reported_ > 1.0) {
        resend_wait_reported_ = now();
        report({{"phase", "LEG"}, {"leg", idx}, {"type", kind}, {"resend_wait", "anchor"},
                {"anchor_err_m", l4::py_round(c.anchor_err_m, 3)}});
      }
      return false;
    }
    planner = publish_goal(c.goal_fc, drive);
    report({{"phase", "LEG"}, {"leg", idx}, {"type", kind}, {"resent", why}, {"goal_planner", vec_json(planner)},
            {"goal_fc", vec_json(c.goal_fc)}, {"anchor_err_m", l4::py_round(c.anchor_err_m, 3)},
            {"above_floor", c.above_floor ? l4::ojson(l4::py_round(*c.above_floor, 3)) : l4::ojson(nullptr)},
            {"raised_m", l4::py_round(c.raised_m, 3)}});
    return true;
  }
  double resend_wait_reported_ = -1e9;

  // a leg's deliberate waypoint yaw ("yaw", deg, fc frame)
  static std::optional<double> leg_yaw(const l4::ojson& leg) {
    if (!leg.contains("yaw")) return std::nullopt;
    return wrap_pi(num(leg["yaw"]) * (M_PI / 180.0));
  }

  ros::NodeHandle nh_, pnh_;
  Mission mission_;
  std::string on_unreachable_, planner_frame_;
  double unreachable_hold_s_, map_rebuild_s_, landing_check_s_, takeoff_timeout_s_, land_timeout_s_, goal_resend_s_,
      bridge_mode_timeout_s_, takeoff_settle_s_, drive_progress_m_, goal_settle_s_, takeoff_still_m_ = 0.05,
      takeoff_still_window_s_ = 1.0;
  double yaw_settle_s_, yaw_settle_rate_, yaw_settle_speed_, yaw_tol_, waypoint_min_dist_, scan_rate_;
  double seed_dz_, seed_s_, stuck_s_, seed_ceiling_clear_;
  int stuck_max_;
  std::string frozen_flight_mode_;
  bool land_search_ = true;
  double land_search_radius_, land_here_check_s_, land_search_timeout_s_, land_spin_rate_, land_move_direct_max_;
  double land_check_height_, land_check_ceiling_clear_, land_check_climb_min_, land_check_climb_step_;
  double land_move_no_progress_s_, land_move_timeout_s_;
  int land_search_tries_;
  int search_seq_ = 0;
  ros::Publisher yaw_pub_;
  // interactive ("click and run")
  std::string mission_path_;
  bool interactive_ = false;
  std::optional<l4::ojson> pending_;   // the newest goal not yet taken (under lock_)
  int goal_seq_ = 0;                   // goals received (under lock_)
  bool face_first_ = false;            // the next fly leg faces its goal first (from a hover)
  bool yaw_held_ = false;              // a waypoint yaw is out to the bridge

  double fc_state_timeout_s_, fc_pose_link_timeout_s_, fc_state_max_s_, bridge_status_timeout_s_, bridge_dead_hover_s_;
  double drive_stop_min_s_, drive_stop_rest_speed_, drive_stop_rest_s_, drive_stop_timeout_s_, hold_slam_timeout_s_;
  double resend_anchor_tol_, resend_floor_min_, goal_clear_s_, goal_shift_max_, goal_end_near_, goal_end_steady_, back_off_clear_, seed_floor_clear_;
  bool link_lost_reported_ = false;
  ros::Publisher sp_pub_;
  std::mutex lock_;   // the inputs, written by the callbacks
  std::optional<double> t_state_, t_pose_, t_bridge_, t_sp_bridge_;   // receive times
  std::optional<mavros_msgs::State> state_;
  std::optional<Vec3> pose_;
  std::optional<double> yaw_;   // the FC pose's yaw (fc frame)
  l4::ojson bridge_;
  double bridge_ok_since_ = 0.0;
  ExecAnchor anchor_;
  std::vector<std::string> texts_;
  std::optional<bool> landing_clear_;
  l4::ojson landing_status_ = l4::ojson::object();
  l4::ojson search_result_ = l4::ojson::object();   // the landing check's latest search answer
  l4::ojson track_ = l4::ojson::object();   // the track planner's latest status
  int track_goals_ = 0;                     // its "new goal" acknowledgements: a REACHED counts only after ours

  std::vector<l4::ojson> ground_spots_;   // where the vehicle sat on the ground (fc frame)
  bool flying_ = false;
  bool landed_once_ = false;   // a takeoff after a landing re-seeds SUPER at the hover point
  bool streaming_ = false;     // the bridge enabled by us
  std::string bridge_mode_now_ = "flight";
  std::atomic<bool> started_{false};
  std::optional<std::string> our_mode_;   // the last mode we commanded
  std::mutex phase_lock_;
  std::string phase_ = "WAIT_ARMED";

  std::vector<ros::Subscriber> subs_;
  ros::Publisher goal_pub_, drive_pub_, leg_pub_, spot_pub_, search_pub_, status_pub_, enable_pub_, mode_pub_, ground_pub_;
  ros::ServiceServer start_srv_;
  ros::ServiceClient cmd_, takeoff_srv_, land_srv_;
};

Why MissionExecutive::takeoff(const l4::ojson& height) {
  {
    std::lock_guard<std::mutex> g(lock_);
    texts_.clear();
  }
  if (streaming_ && bridge_mode_now_ == "drive") {
    // after a drive: keep the bridge on until the tracks stop. Once the track planner has reached its goal
    // the bridge holds (streams nothing) and Rover Guided stops at the last target
    report({{"phase", "TAKEOFF"}, {"height", height}, {"waiting", "the tracks to stop"}});
    const bool still = standstill(takeoff_settle_s_);
    if (!still) {
      bridge_enable(false);
      report({{"phase", "TAKEOFF"}, {"height", height}, {"refused", true}, {"reason", NOT_STATIONARY}});
      return std::string(NOT_STATIONARY);
    }
  }
  bridge_enable(false);
  bridge_mode_now_ = "flight";
  publish_set_mode("flight");   // not streaming: no need to wait for it here
  report({{"phase", "TAKEOFF"}, {"height", height}});
  // owner decision 2026-10-28: no takeoff unless standing still (QuadRover's own guard refuses above
  // 0.2 m/s, but a slow creep passes it); not still within takeoff_settle_s: stop here
  if (!standstill(takeoff_settle_s_)) {
    report({{"phase", "TAKEOFF"}, {"height", height}, {"refused", true}, {"reason", NOT_STATIONARY}});
    return std::string(NOT_STATIONARY);
  }
  record_ground_spot("takeoff");
  // standing still: ask, and again while QuadRover says "temporarily rejected"
  const double t0 = now();
  int tries = 0;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  while (true) {
    tries += 1;
    mavros_msgs::CommandTOL srv;
    srv.request.min_pitch = 0;
    srv.request.yaw = nan;
    srv.request.latitude = nan;
    srv.request.longitude = nan;
    srv.request.altitude = static_cast<float>(num(height));
    if (!takeoff_srv_.call(srv)) {
      ROS_ERROR("[mission] /mavros/cmd/takeoff could not be called");
      srv.response.success = false;
      srv.response.result = 0;
    }
    if (srv.response.success) break;
    if (srv.response.result != 1 || now() - t0 > takeoff_settle_s_ || shutdown()) {
      report({{"phase", "TAKEOFF"}, {"height", height}, {"refused", true}, {"result", static_cast<int>(srv.response.result)},
              {"tries", tries}});
      return std::string("takeoff refused");
    }
    sleep(1.0);
  }
  const auto here = pose_copy();
  if (landed_once_ && here) {
    // SUPER still holds the goal it had before the landing (and has been replanning toward it from the
    // ground): give it the hover point this takeoff ends at, so its reference is back at the vehicle when
    // the next fly leg's goal comes
    publish_goal(Vec3((*here)[0], (*here)[1], (*here)[2] + num(height)));
  }
  if (!wait_text("Guided takeoff complete", takeoff_timeout_s_)) {
    auto lost = fc_lost();
    return lost ? lost : std::string("takeoff did not complete");
  }
  flying_ = true;
  return std::nullopt;
}

// send the goal and watch it; none when reached, or why not
Why MissionExecutive::go(const l4::ojson& leg, int idx, const Vec3& target, bool drive) {
  const std::string kind = drive ? "drive" : "fly";
  const std::string mode = drive ? "drive" : "flight";
  if (!bridge_mode(mode)) return "bridge did not switch to " + mode + " mode";
  {
    l4::ojson l;
    l["index"] = idx;
    l["type"] = kind;
    l["speed"] = l4::get(leg, "speed");
    std_msgs::String m;
    m.data = l4::dumps(l);
    leg_pub_.publish(m);
  }
  int goals_before;
  {
    std::lock_guard<std::mutex> g(lock_);
    goals_before = track_goals_;
  }
  // where the leg began (fc): the re-seeds' planned altitude runs from here to the goal (seed_altitude)
  const auto here0 = pose_copy();
  const Vec3 leg_start = here0 ? *here0 : target;
  bool turned = false;
  // interactive: also from a hover (the bridge holding a waypoint yaw), whose next goal is new
  const bool face_first = face_first_;
  face_first_ = false;
  if (!drive && flying_ && (!streaming_ || face_first)) {
    // a flight leg with the bridge not streaming yet (after a takeoff, or after a QRLOITER): the waypoint at
    // its start (owner 2026-10-29: the robot yaws only at waypoints). Face the leg, hovering, before it
    // starts (Gate L2 b5, leg 7: off the ground facing the drive, the nose turned 100 deg in flight)
    const std::optional<double> y = heading_to(target[0], target[1]);
    if (y) {
      publish_yaw(y);
      bridge_enable(true);
      if (Why w = waypoint(idx, y, "leg start")) return w;
      turned = true;
    }
  }
  publish_yaw(std::nullopt);   // the last waypoint's yaw is over: the nose follows the travel direction
  // after the turn SUPER has long reached the hover point it was given and sits at rest (b7: the 3.55 s turn
  // on leg 7, then PlanFromRest failing for 20 s): re-seed it so the leg's goal comes while it moves
  if (turned) seed_planner(idx, leg_start, target, "leg start");
  Vec3 planner = publish_goal(target, drive);
  if (!drive && !streaming_ && goal_settle_s_ > 0) sleep(goal_settle_s_);   // SUPER plans the new goal before anything streams
  bridge_enable(true);
  {
    l4::ojson r = {{"phase", "LEG"}, {"leg", idx}, {"type", kind}, {"bridge_mode", mode}, {"goal_fc", vec_json(target)},
                   {"goal_planner", vec_json(planner)}};
    if (leg.contains("for")) r["for"] = leg["for"];   // a landing move: "landing spot"
    report(r);
  }
  double t0 = now(), last_progress_t = t0;
  std::optional<double> best;
  std::optional<Vec3> moved_from;   // drive legs: driving on counts as progress (a detour round an obstacle
  double last_send = t0;            // first takes the vehicle no closer to the goal)
  bool held = false;
  double held_t = t0;                    // the SLAM hold began
  bool resent_held = false;              // the goal went again during the hold (a re-anchor waiting for a plan)
  std::optional<double> reanchor_since;  // the bridge's re-anchor hold with SLAM fresh, since
  std::optional<double> not_clear_since; // SUPER's goal shifted and not clear, since (L2 4a)
  std::optional<Vec3> bad_end;           // the plan's end last seen (it must stay put)
  int reseeds = 0;   // SUPER re-seeded after falling silent on this leg
  const double accept_radius = num(leg["accept_radius"]), no_progress_s = num(leg["no_progress_s"]),
               leg_timeout_s = num(leg["leg_timeout_s"]);
  while (!shutdown()) {
    if (auto lost = fc_lost()) return lost;
    if (preempted()) return std::string(PREEMPTED);
    std::optional<Vec3> pos;
    l4::ojson bridge, track;
    double ok_since;
    {
      std::lock_guard<std::mutex> g(lock_);
      pos = pose_;
      bridge = bridge_;
      ok_since = bridge_ok_since_;
      track = track_goals_ > goals_before ? track_ : l4::ojson::object();
    }
    const double t = now();
    // the SLAM pose froze: the leg and mission end (land in the air, stop on the ground)
    if (auto fz = watch(idx)) return fz;
    // the planner died (a status from the bridge enabled for this leg): the leg and mission end
    if (l4::get(bridge, "reason") == "planner_dead" && l4::truthy(l4::get(bridge, "enabled", true))) {
      report({{"phase", "LEG"}, {"leg", idx}, {"type", kind}, {"planner_dead", true},
              {"planner_age", l4::get(bridge, "planner_age")}, {"planner_node", l4::get(bridge, "planner_node")}});
      return std::string(PLANNER_DIED);
    }
    // D7: the bridge holds on a SLAM reset or an anchor jump; we stop and wait
    if (!l4::truthy(l4::get(bridge, "slam_ok", true))) {
      if (!held) {
        held = true;
        held_t = t;
        resent_held = false;
        reanchor_since.reset();
        report({{"phase", "HOLD_SLAM"}, {"leg", idx}, {"reason", l4::get(bridge, "reason", "bridge hold")}});
      }
      // a re-anchor hold (flight) ends on a plan made after the re-anchor, and SUPER at rest (its goal reached, or
      // none since) plans only on a goal: with SLAM back, the goal again after ~map_rebuild_s (10-28 note 5: the
      // deadlock); still no plan after ~planner_stuck_s with SUPER silent: re-seed and the goal again
      const l4::ojson sa = l4::get(bridge, "slam_age"), age = l4::get(bridge, "planner_age");
      const bool slam_fresh = sa.is_number() && sa.get<double>() <= 0.3 && !l4::truthy(l4::get(bridge, "slam_pending", false));
      if (!drive && l4::get(bridge, "reanchor").is_object() && slam_fresh) {
        if (!reanchor_since) reanchor_since = t;
        if (!resent_held && t - *reanchor_since >= map_rebuild_s_) {
          if (resend_goal(idx, kind, target, drive, planner, "re-anchor")) {
            resent_held = true;
            last_send = t;
          }
        } else if (resent_held && stuck_s_ > 0 && reseeds < stuck_max_ && t - last_send >= stuck_s_ &&
                   age.is_number() && age.get<double>() >= stuck_s_) {
          reseeds += 1;
          report({{"phase", "HOLD_SLAM"}, {"leg", idx}, {"planner_silent_s", l4::py_round(age.get<double>(), 2)}, {"reseed", reseeds}});
          if (seed_planner(idx, leg_start, target, "re-anchor, planner silent")) resend_goal(idx, kind, target, drive, planner, "re-seed");
          last_send = now();
        }
      } else if (!slam_fresh) {
        reanchor_since.reset();
      }
      if (hold_slam_timeout_s_ > 0 && t - held_t > hold_slam_timeout_s_) {
        report({{"phase", "HOLD_SLAM"}, {"leg", idx}, {"timed_out", true}, {"held_s", l4::py_round(t - held_t, 2)},
                {"reason", l4::get(bridge, "reason", "bridge hold")}});
        return std::string(SLAM_HOLD_TIMEOUT);
      }
      last_progress_t = t;
      t0 += 0.1;   // the leg clock does not run while the bridge holds
    } else if (held) {
      if (resent_held) {
        held = false;
        last_progress_t = t;
        report({{"phase", "LEG"}, {"leg", idx}, {"type", kind}, {"resumed", true}, {"goal_resent_in_hold", true}});
      } else if (t - ok_since >= map_rebuild_s_) {
        if (resend_goal(idx, kind, target, drive, planner, "resumed")) {   // re-validated with the new anchor
          held = false;
          last_send = t;
          last_progress_t = t;
        }
      }
    } else if (pos) {
      const double d = drive ? std::hypot((*pos)[0] - target[0], (*pos)[1] - target[1]) : (*pos - target).norm();
      if (d <= accept_radius) return std::nullopt;
      if (drive && l4::get(track, "state") == "REACHED") {
        // the track planner's own test (its odometry point, ~footprint_offset ahead of the FC, within its
        // accept radius)
        report({{"phase", "LEG"}, {"leg", idx}, {"type", kind}, {"reached_by", "track planner"}, {"fc_dist", l4::py_round(d, 3)}});
        return std::nullopt;
      }
      if (!best || d < *best - 0.2) {
        best = d;
        last_progress_t = t;
        moved_from = *pos;
      } else if (drive && moved_from &&
                 std::hypot((*pos)[0] - (*moved_from)[0], (*pos)[1] - (*moved_from)[1]) > drive_progress_m_) {
        last_progress_t = t;
        moved_from = *pos;
      }
      if (t - last_progress_t > no_progress_s || t - t0 > leg_timeout_s) return std::string("unreachable");
      // a goal inside an obstacle (L2 4a): SUPER moved it (its plan's end, the bridge's "planner_goal", from a plan
      // made after our goal) and the bridge's scans have points round where it moved it to
      if (!drive && goal_clear_s_ > 0) {
        const l4::ojson pg = l4::get(bridge, "planner_goal"), pc = l4::get(bridge, "planner_goal_clear");
        bool bad = false;
        double shift = 0.0;
        // the plan's end is SUPER's goal only once the vehicle has got there and it stays put: a long leg's plan ends at
        // SUPER's planning horizon (SITL fx14b, leg 5: 1.2 m short of the goal) and a climb's first plans end part way
        // up (roof run hpr8, 2026-11-03: a 2.3 m climb given up 1.5 s in, the plans' ends 1.5 -> 1.3 m, the goal 3.04 m,
        // the vehicle at 0.72 m, 0.6 m from them)
        std::optional<Vec3> end;
        if (pg.is_array() && pg.size() == 3) end = Vec3(num(pg[0]), num(pg[1]), num(pg[2]));
        if (end && bad_end && (*end - *bad_end).norm() > goal_end_steady_) not_clear_since.reset();   // a new end
        bad_end = end;
        if (end && (*end - *pos).norm() <= goal_end_near_) {
          shift = (Vec3(num(pg[0]), num(pg[1]), num(pg[2])) - target).norm();
          const double dz = std::abs(num(pg[2]) - target[2]);
          // moved up or down more than SUPER's inflation (0.5 m): the goal was inside the obstacle itself, whatever
          // the scans show (SITL fx14: the forward lidar never saw the box top round the moved goal,
          // "planner_goal_clear" true, the goal moved 0.95 m up). Sideways it is not judged: a plan also ends short
          // of the goal when SUPER brakes or stops short of an obstacle on the way, scan points round it or not (fx16:
          // 1.2 m short of a pillar; Gate L2 fxg1: 2.8 m short in the course, a false "goal in an obstacle")
          bad = dz > goal_shift_max_;
          (void)pc;
        }
        if (!bad) {
          not_clear_since.reset();
        } else {
          if (!not_clear_since) not_clear_since = t;
          if (t - *not_clear_since >= goal_clear_s_) {
            report({{"phase", "LEG"}, {"leg", idx}, {"type", kind}, {"goal_not_clear", true}, {"planner_goal_fc", pg},
                    {"shift_m", l4::py_round(shift, 3)}});
            return std::string(GOAL_NOT_CLEAR);
          }
        }
      }
      // SUPER silent on a flight leg (the bridge's planner_age; its PlanFromRest failing at rest sends
      // nothing, so nothing moves and nothing changes, Gate L2 b7): re-seed it and send the goal again
      const l4::ojson age = l4::get(bridge, "planner_age");
      if (!drive && stuck_s_ > 0 && reseeds < stuck_max_ && age.is_number() && age.get<double>() >= stuck_s_ &&
          t - last_send >= stuck_s_ && l4::truthy(l4::get(bridge, "enabled", true))) {
        reseeds += 1;
        report({{"phase", "LEG"}, {"leg", idx}, {"type", kind}, {"planner_silent_s", l4::py_round(age.get<double>(), 2)},
                {"reseed", reseeds}});
        if (seed_planner(idx, leg_start, target, "planner silent")) {
          planner = publish_goal(target, drive);
          last_send = now();
          last_progress_t = last_send;   // a fresh start for the re-sent goal
          report({{"phase", "LEG"}, {"leg", idx}, {"type", kind}, {"resent", true}, {"goal_planner", vec_json(planner)}});
        }
        continue;
      }
      if (goal_resend_s_ > 0 && t - last_send > goal_resend_s_) {
        publish_goal(target, drive);
        last_send = t;
      }
    }
    sleep(0.1);
  }
  return std::string("shutdown");
}

Why MissionExecutive::land(const l4::ojson& leg, int idx) {
  l4::ojson check;
  // to the check height first, keeping the yaw it settled on (header comment)
  if (Why w = land_climb(idx, std::nullopt)) return w;
  if (Why w = land_check(num(leg["at"][0]), num(leg["at"][1]), idx, landing_check_s_, check)) return w;
  return land_descend(idx, check);
}

// a mission land leg with "landing_check: false" (owner 2026-11-03, the SITL roof demo: the landing check cannot
// clear the Hensel Phelps container roof on the rendered map): NAV_LAND from where the leg's hover settled, no check
// height, no landing check, no search. Opt-in per leg only; every other landing is checked as before
Why MissionExecutive::land_unchecked(int idx) {
  report({{"phase", "LAND_CHECK"}, {"leg", idx}, {"skipped", true}, {"reason", "landing_check false (mission leg)"}});
  return land_descend(idx, {{"check", "skipped (landing_check false)"}, {"check_s", 0.0}});
}

// ask the landing check about the spot at (x, y) (fc; z = the height we descend from: the vehicle's) and
// require it reported clear within `timeout` (a not-clear answer is asked again until then: a person walking
// off clears it); none when clear (check: its reason and time, for the LAND report)
Why MissionExecutive::land_check(double x, double y, int idx, double timeout, l4::ojson& check) {
  {
    std::lock_guard<std::mutex> g(lock_);
    landing_clear_.reset();
    landing_status_ = l4::ojson::object();
  }
  // ask the landing check about this spot (planner frame; z = the height we descend from)
  geometry_msgs::PointStamped spot;
  spot.header.frame_id = planner_frame_;
  Vec3 p;
  {
    std::lock_guard<std::mutex> g(lock_);
    const double z = pose_ ? (*pose_)[2] : 0.0;
    p = anchor_.fc_to_planner(Vec3(x, y, z));
  }
  spot.point.x = p[0];
  spot.point.y = p[1];
  spot.point.z = p[2];
  report({{"phase", "LAND_CHECK"}, {"leg", idx},
          {"spot_planner", {l4::py_round(p[0], 3), l4::py_round(p[1], 3), l4::py_round(p[2], 3)}}});
  const double t0 = now();
  double last_spot = -1.0;
  std::optional<bool> clear;
  int answers = 0;
  while (now() - t0 < timeout) {
    if (preempted()) return std::string(PREEMPTED);   // interactive: before NAV_LAND only
    if (now() - last_spot >= 0.2) {
      spot.header.stamp = ros::Time::now();
      spot_pub_.publish(spot);
      last_spot = now();
    }
    {
      std::lock_guard<std::mutex> g(lock_);
      clear = landing_clear_;
      landing_clear_.reset();
    }
    if (clear) answers += 1;
    if (clear && *clear) break;   // reported clear: land; not clear (yet): keep asking until the timeout
    sleep(0.05);
  }
  l4::ojson why;
  {
    std::lock_guard<std::mutex> g(lock_);
    why = l4::get(landing_status_, "reason", "no answer from the landing check");
  }
  if (!(clear && *clear)) {
    report({{"phase", "LAND_CHECK"}, {"leg", idx}, {"clear", false}, {"answers", answers}, {"reason", why},
            {"waited_s", l4::py_round(now() - t0, 2)}});
    return std::string(LAND_NOT_CLEAR);
  }
  check = {{"check", why}, {"check_s", l4::py_round(now() - t0, 2)}};
  return std::nullopt;
}

// the spot is clear: NAV_LAND (QuadRover lands and drives on in GUIDED)
Why MissionExecutive::land_descend(int idx, const l4::ojson& check) {
  {
    std::lock_guard<std::mutex> g(lock_);
    texts_.clear();
  }
  bridge_enable(false);
  // the bridge's last target must not reach the FC after NAV_LAND: a Guided target cancels QuadRover's Guided landing
  // (Gate L2 fxg2, 2026-11-02: one target 0.006 s after the disable, at the "Guided landing" text; it then held at
  // 1.41 m for 60 s, "landing did not complete"). Its 50 Hz tick sees the disable within 0.02 s
  sleep(0.2);
  report({{"phase", "LAND"}, {"leg", idx}, {"check", l4::get(check, "check")}, {"check_s", l4::get(check, "check_s")}});
  mavros_msgs::CommandTOL srv;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  srv.request.min_pitch = 0;
  srv.request.yaw = nan;
  srv.request.latitude = nan;
  srv.request.longitude = nan;
  srv.request.altitude = 0;
  if (!land_srv_.call(srv)) {
    ROS_ERROR("[mission] /mavros/cmd/land could not be called");
    srv.response.success = false;
  }
  if (!srv.response.success) return std::string("landing refused");
  if (!wait_text("Guided landing complete", land_timeout_s_)) {
    auto lost = fc_lost();
    return lost ? lost : std::string("landing did not complete");
  }
  flying_ = false;   // on the ground in GUIDED, driving (QuadRover's NAV_LAND)
  landed_once_ = true;
  record_ground_spot("landed");
  return std::nullopt;
}

// in the air, to drive to (gx, gy) (fc): land first, where the vehicle is or at a clear spot nearby (the
// landing-spot search, header comment); none once on the ground
Why MissionExecutive::land_to_drive(const l4::ojson& leg, int idx, double gx, double gy) {
  const auto p = pose_copy();
  if (!p) return std::string("no FC pose");
  l4::ojson land_leg = leg;
  land_leg["type"] = "land";
  land_leg["at"] = l4::ojson::array({l4::py_round((*p)[0], 3), l4::py_round((*p)[1], 3)});
  // over the spot (owner 2026-10-29): face the drive (the tank starts aligned), hover with the yaw settled
  Why why = go(land_leg, idx, *p, false);
  if (!why) why = waypoint(idx, heading_to(gx, gy), "landing");
  if (why) return why;
  if (!land_search_) return land(land_leg, idx);
  land_yaw_.reset();
  return land_here_or_nearby(leg, idx, gx, gy);
}

// hovering over the spot with the yaw settled: climb to the check height, the landing check on the spot under
// the vehicle, else the landing-spot search, spin, move (SUPER) and check again (header comment), then NAV_LAND.
// (gx, gy): the drive goal the candidates face (NaN: none; then land_yaw_, a land leg's own yaw, or the yaw it has)
Why MissionExecutive::land_here_or_nearby(const l4::ojson& leg, int idx, double gx, double gy) {
  const auto p = pose_copy();
  if (!p) return std::string("no FC pose");
  Why why;
  // to the check height (header comment): the check, the search, the spin and the moves run there
  if (Why w = land_climb(idx, land_face(gx, gy))) return w;
  l4::ojson check;
  const auto here = pose_copy();
  why = land_check((*here)[0], (*here)[1], idx, land_here_check_s_, check);
  if (!why) return land_descend(idx, check);
  if (*why != LAND_NOT_CLEAR) return why;
  std::vector<Vec3> tried = {*p};   // refused spots (fc)
  bool spun = false;
  int searches = 0;
  for (int moves = 0; moves < land_search_tries_;) {
    std::optional<Vec3> spot;
    if (Why w = land_search(idx, gx, gy, tried, ++searches, spot)) return w;
    if (!spot) {
      if (spun || land_spin_rate_ <= 0) return std::string(NO_LAND_SPOT);
      if (Why w = land_spin(idx, gx, gy)) return w;
      spun = true;
      continue;
    }
    moves += 1;
    const auto here = pose_copy();
    const double z = here ? (*here)[2] : (*p)[2];
    const double d = here ? std::hypot((*spot)[0] - (*here)[0], (*spot)[1] - (*here)[1]) : 0.0;
    report({{"phase", "LAND_MOVE"}, {"leg", idx}, {"spot_fc", vec_json(Vec3((*spot)[0], (*spot)[1], z))},
            {"dist_m", l4::py_round(d, 3)}, {"move", moves}});
    // a fly leg to the spot at this height (SUPER): facing the spot first (the nose-first approach keeps it in
    // the lidar's view), then the waypoint hold over it facing the goal, which eases the vehicle onto it
    l4::ojson mv = leg;
    mv["type"] = "fly";
    mv["for"] = "landing spot";
    if (land_move_direct_max_ > 0 && d <= land_move_direct_max_) {
      why = land_move_direct(idx, Vec3((*spot)[0], (*spot)[1], z), gx, gy);
    } else {
      // the move's own no-progress watchdog (owner 2026-11-02: SUPER's PlanFromRest failing ~90 times a second,
      // "GeneratePolytopeFromLine failed", on a 1.08 m move from a hover at 1.53 m; the leg's 20 s no-progress
      // and two re-seeds took 36 s): not 0.2 m closer within ~land_move_no_progress_s, or ~land_move_timeout_s
      mv["no_progress_s"] = land_move_no_progress_s_;
      mv["leg_timeout_s"] = land_move_timeout_s_;
      face_first_ = true;
      why = go(mv, idx, Vec3((*spot)[0], (*spot)[1], z), false);
      if (why && *why == "unreachable") {
        // give that spot up: hold where the vehicle is (a goal at it: the hold's point, and SUPER's goal off the
        // spot it could not plan to), settle, and the next candidate (the spot excluded; at most
        // ~land_search_tries moves, then REFUSED as before)
        const auto at = pose_copy();
        report({{"phase", "LAND_MOVE"}, {"leg", idx}, {"move", moves}, {"gave_up", true},
                {"reason", "no progress toward the spot (SUPER)"},
                {"off_m", at ? l4::ojson(l4::py_round(std::hypot((*at)[0] - (*spot)[0], (*at)[1] - (*spot)[1]), 3)) : l4::ojson(nullptr)}});
        publish_yaw(std::nullopt);
        if (at) publish_goal(*at);
        sleep(0.3);
        if (Why w = waypoint(idx, land_face(gx, gy), "landing")) return w;
        tried.push_back(*spot);
        if (at) tried.push_back(*at);
        continue;
      }
      if (!why) why = waypoint(idx, land_face(gx, gy), "landing");
    }
    if (why) return why;
    // over the spot: a nearer surface (a table top) is climbed away from too, as under the vehicle at first
    if (Why w = land_climb(idx, land_face(gx, gy))) return w;
    const auto at = pose_copy();
    if (!at) return std::string("no FC pose");
    report({{"phase", "LAND_MOVE"}, {"leg", idx}, {"arrived", true},
            {"off_m", l4::py_round(std::hypot((*at)[0] - (*spot)[0], (*at)[1] - (*spot)[1]), 3)},
            {"from_start_m", l4::py_round(std::hypot((*at)[0] - (*p)[0], (*at)[1] - (*p)[1]), 3)}});
    // the landing check on the spot under the vehicle, where it will come down
    why = land_check((*at)[0], (*at)[1], idx, landing_check_s_, check);
    if (!why) return land_descend(idx, check);
    if (*why != LAND_NOT_CLEAR) return why;
    std::string reason;
    {
      std::lock_guard<std::mutex> g(lock_);
      reason = l4::get(landing_status_, "reason", "").get<std::string>();
    }
    if (starts_with(reason, "not observed")) {
      // over the spot the forward-facing lidar no longer sees the floor under the frame and the check's 10 s
      // window may have lost the scans that chose it (SITL r5_c9): the same check on the spot under the vehicle
      // with the search's window (a radius-0 search: the same rules, the scans of the last ~20 s)
      std::optional<Vec3> same;
      if (Why w = land_search(idx, gx, gy, {}, ++searches, same, 0.0)) return w;
      if (same) return land_descend(idx, {{"check", "clear (the search window)"}, {"check_s", 0}});
    }
    tried.push_back(*spot);
    tried.push_back(*at);
  }
  return std::string(LAND_NOT_CLEAR);
}

// ask the landing check node for a clear spot within ~land_search_radius (n: this landing's search count); spot:
// the chosen one (fc, z = the vehicle's), none when none found or no answer within ~land_search_timeout_s
Why MissionExecutive::land_search(int idx, double gx, double gy, const std::vector<Vec3>& tried, int n,
                                  std::optional<Vec3>& spot, std::optional<double> radius) {
  const double r = radius ? *radius : land_search_radius_;
  spot.reset();
  const auto here = pose_copy();
  if (!here) return std::string("no FC pose");
  l4::ojson req;
  Vec3 vp, gp;
  {
    std::lock_guard<std::mutex> g(lock_);
    vp = anchor_.fc_to_planner(*here);
    if (std::isfinite(gx) && std::isfinite(gy)) gp = anchor_.fc_to_planner(Vec3(gx, gy, (*here)[2]));
    l4::ojson ex = l4::ojson::array();
    for (const auto& t : tried) {
      const Vec3 q = anchor_.fc_to_planner(Vec3(t[0], t[1], (*here)[2]));
      ex.push_back({l4::py_round(q[0], 3), l4::py_round(q[1], 3)});
    }
    req["id"] = ++search_seq_;
    req["vehicle"] = {l4::py_round(vp[0], 3), l4::py_round(vp[1], 3), l4::py_round(vp[2], 3)};
    if (std::isfinite(gx) && std::isfinite(gy)) req["goal"] = {l4::py_round(gp[0], 3), l4::py_round(gp[1], 3)};
    else req["goal"] = nullptr;   // a land leg with no drive after it: the candidates keep the vehicle's yaw
    req["radius"] = r;
    req["exclude"] = ex;
    search_result_ = l4::ojson::object();
  }
  const int id = req["id"].get<int>();
  report({{"phase", "LAND_SEARCH"}, {"leg", idx}, {"search", n}, {"radius", r},
          {"vehicle_fc", vec_json(*here)},
          {"goal_fc", std::isfinite(gx) ? l4::ojson::array({gx, gy}) : l4::ojson(nullptr)}, {"excluded", tried.size()}});
  std_msgs::String m;
  m.data = l4::dumps(req);
  const double t0 = now();
  double sent = -1e9;
  l4::ojson res;
  while (!shutdown() && now() - t0 < land_search_timeout_s_) {
    if (auto lost = fc_lost()) return lost;
    if (auto fz = watch(idx)) return fz;
    if (preempted()) return std::string(PREEMPTED);
    if (now() - sent >= 4.0) {   // once, again if unanswered (a lost message; the node answers a repeat from its cache)
      search_pub_.publish(m);
      sent = now();
    }
    {
      std::lock_guard<std::mutex> g(lock_);
      if (l4::get(search_result_, "id") == id) res = search_result_;
    }
    if (!res.is_null()) break;
    sleep(0.05);
  }
  if (shutdown()) return std::string("shutdown");
  if (res.is_null()) {
    report({{"phase", "LAND_SEARCH"}, {"leg", idx}, {"search", n}, {"found", false},
            {"reason", "no answer from the landing search"}, {"waited_s", l4::py_round(now() - t0, 2)}});
    return std::nullopt;
  }
  if (!l4::truthy(l4::get(res, "found", false)) || !l4::get(res, "spot").is_array()) {
    report({{"phase", "LAND_SEARCH"}, {"leg", idx}, {"search", n}, {"found", false},
            {"reasons", l4::get(res, "reasons")}, {"first_reason", l4::get(res, "first_reason")},
            {"candidates", l4::get(res, "candidates")}, {"evaluated", l4::get(res, "evaluated")},
            {"search_s", l4::get(res, "elapsed_s")}});
    return std::nullopt;
  }
  Vec3 sp;
  {
    std::lock_guard<std::mutex> g(lock_);
    sp = anchor_.planner_to_fc(Vec3(num(res["spot"][0]), num(res["spot"][1]), vp[2]));
  }
  spot = Vec3(sp[0], sp[1], (*here)[2]);
  report({{"phase", "LAND_SEARCH"}, {"leg", idx}, {"search", n}, {"found", true},
          {"spot_fc", {l4::py_round(sp[0], 3), l4::py_round(sp[1], 3)}}, {"dist_m", l4::get(res, "dist_m")},
          {"margin_m", l4::get(res, "margin_m")}, {"cost", l4::get(res, "cost")},
          {"candidates", l4::get(res, "candidates")}, {"evaluated", l4::get(res, "evaluated")},
          {"reasons", l4::get(res, "reasons")}, {"search_s", l4::get(res, "elapsed_s")}});
  return std::nullopt;
}

// a short landing move (within ~land_move_direct_max): the bridge's waypoint hold takes the executive's last goal
// as its hold point when that is within its ~waypoint_goal_max of the vehicle, and eases the vehicle onto it
// (capped like any target, zero velocity at the point). So: clear the waypoint yaw (the hold point is released),
// send the spot as the goal (SUPER gets it too, unused while the hold runs), set the waypoint yaw facing the
// drive goal (a new hold, at the spot), and wait until the vehicle is over the spot and settled. A straight move
// at the hover height: the spot's disc (0.54 m round the frame, up to that height) is clear, the rest of the
// <= 0.9 m line is what the vehicle has hovered next to
Why MissionExecutive::land_move_direct(int idx, const Vec3& spot, double gx, double gy) {
  publish_yaw(std::nullopt);
  const Vec3 planner = publish_goal(spot);
  report({{"phase", "LAND_MOVE"}, {"leg", idx}, {"direct", true}, {"goal_planner", vec_json(planner)}});
  sleep(0.3);   // the bridge has the goal before the new hold starts (its point is fixed then)
  const std::optional<double> y = land_face(gx, gy);
  const double t0 = now();
  while (true) {
    if (Why w = waypoint(idx, y ? y : land_face(gx, gy), "landing")) return w;
    const auto p = pose_copy();
    if (!p || std::hypot((*p)[0] - spot[0], (*p)[1] - spot[1]) <= 0.15 || now() - t0 > yaw_settle_s_) return std::nullopt;
    sleep(0.5);   // settled short of the spot (the hold eases on slowly near it): settle again
  }
}

// before the landing check: climb to ~land_check_height above the surface under the vehicle (the bridge's
// "floor_below"), at most up to ~land_check_ceiling_clear below its "ceiling_above" (header comment), facing yaw
// (none: the yaw it has). Through the bridge's waypoint hold, as land_move_direct: the goal straight over the
// vehicle (at most ~land_move_direct_max a step) and a waypoint yaw, which holds that goal. None when done, or
// nothing to do; the check runs from the height reached
Why MissionExecutive::land_climb(int idx, std::optional<double> yaw, std::optional<double> height_opt, const char* phase) {
  const double height = height_opt ? *height_opt : land_check_height_;
  if (height <= 0) return std::nullopt;
  const auto here = pose_copy();
  if (!here) return std::string("no FC pose");
  auto bridge_num = [this](const char* k) -> std::optional<double> {
    std::lock_guard<std::mutex> g(lock_);
    const l4::ojson v = l4::get(bridge_, k);
    if (v.is_number()) return v.get<double>();
    return std::nullopt;
  };
  const std::optional<double> floor = bridge_num("floor_below"), ceiling = bridge_num("ceiling_above");
  auto opt = [](std::optional<double> v) { return v ? l4::ojson(l4::py_round(*v, 3)) : l4::ojson(nullptr); };
  l4::ojson r = {{"phase", phase}, {"leg", idx}, {"from_z", l4::py_round((*here)[2], 3)},
                 {"floor_below", opt(floor)}, {"ceiling_above", opt(ceiling)}, {"check_height", height}};
  if (!floor) {
    r["climb"] = false;
    r["reason"] = "no floor_below from the bridge: the check from this height";
    report(r);
    return std::nullopt;
  }
  const double want = (*here)[2] + height - *floor;
  double target = want;
  bool capped = false;
  if (ceiling) {
    const double cap = (*here)[2] + *ceiling - land_check_ceiling_clear_;
    if (cap < target) {
      target = cap;
      capped = true;
    }
  }
  r["target_z"] = l4::py_round(target, 3);
  r["capped"] = capped;
  if (target - (*here)[2] < land_check_climb_min_) {
    r["climb"] = false;
    r["reason"] = capped ? "no room under the ceiling" : "at the check height";
    report(r);
    return std::nullopt;
  }
  r["climb"] = true;
  report(r);
  const double t0 = now();
  const double step_max = std::max(0.1, land_check_climb_step_);
  const double x = (*here)[0], y = (*here)[1];
  std::string stop_why = "reached";
  double z = (*here)[2];
  while (true) {
    const double step_z = std::min(target, z + step_max);
    // a new hold at the goal straight over the vehicle (land_move_direct)
    publish_yaw(std::nullopt);
    publish_goal(Vec3(x, y, step_z));
    sleep(0.3);
    if (Why w = waypoint(idx, yaw, "check height")) return w;
    const double ts = now();
    bool stepped = false;
    while (!shutdown() && now() - ts < yaw_settle_s_) {
      if (auto lost = fc_lost()) return lost;
      if (auto fz = watch(idx)) return fz;
      if (preempted()) return std::string(PREEMPTED);
      const auto p = pose_copy();
      if (p) z = (*p)[2];
      const std::optional<double> c = bridge_num("ceiling_above");
      if (c && *c < land_check_ceiling_clear_ - 0.1) {
        // a ceiling well within the clearance (seen on the way up, lower than at the start): hold here
        stop_why = "ceiling";
        break;
      }
      if (std::abs(z - step_z) <= 0.1) {
        stepped = true;
        break;
      }
      sleep(0.05);
    }
    if (shutdown()) return std::string("shutdown");
    if (stop_why == "ceiling") {
      publish_yaw(std::nullopt);
      const auto p = pose_copy();
      if (p) publish_goal(*p);
      sleep(0.3);
      if (Why w = waypoint(idx, yaw, "check height")) return w;
      break;
    }
    if (!stepped) {
      stop_why = "not reached";
      break;
    }
    if (step_z >= target - 1e-6) break;
  }
  const auto p = pose_copy();
  report({{"phase", phase}, {"leg", idx}, {"done", stop_why}, {"z", p ? l4::ojson(l4::py_round((*p)[2], 3)) : l4::ojson(nullptr)},
          {"target_z", l4::py_round(target, 3)}, {"floor_below", opt(bridge_num("floor_below"))},
          {"ceiling_above", opt(bridge_num("ceiling_above"))}, {"climb_s", l4::py_round(now() - t0, 2)}});
  return std::nullopt;
}

// a fly leg given up in the air (unreachable, a goal in an obstacle; L2 4a, SITL lf14: SUPER had flown its shifted
// goal over a box, 0.15 m over its top, and every PlanFromRest from there failed): before QRLOITER, up to
// ~back_off_clear above the surface under the vehicle when it is nearer (the bridge's "floor_below"; under its
// "ceiling_above" as the landing check climb), through the bridge's waypoint hold, so the next plan starts clear
// of SUPER's inflation. None when done or nothing to do
Why MissionExecutive::back_off(int idx, const std::string& why) {
  if (!flying_ || back_off_clear_ <= 0 || !streaming_ || bridge_mode_now_ != "flight") return std::nullopt;
  std::optional<double> floor;
  {
    std::lock_guard<std::mutex> g(lock_);
    const l4::ojson f = l4::get(bridge_, "floor_below");
    if (f.is_number()) floor = f.get<double>();
  }
  if (!floor || *floor >= back_off_clear_) return std::nullopt;
  report({{"phase", "BACK_OFF"}, {"leg", idx}, {"why", why}, {"floor_below", l4::py_round(*floor, 3)},
          {"clear", back_off_clear_}});
  return land_climb(idx, std::nullopt, back_off_clear_, "BACK_OFF");
}

// no spot found: hovering in place (the bridge's waypoint hold), turn a full 360 deg at ~land_spin_rate_deg so
// the forward-facing lidar sees all round, then face the goal and settle
Why MissionExecutive::land_spin(int idx, double gx, double gy) {
  std::optional<double> y0;
  {
    std::lock_guard<std::mutex> g(lock_);
    y0 = yaw_;
  }
  if (!y0) return std::string("no FC yaw");
  report({{"phase", "LAND_SPIN"}, {"leg", idx}, {"from_deg", l4::py_round(*y0 * (180.0 / M_PI), 1)},
          {"rate_deg", l4::py_round(land_spin_rate_ * (180.0 / M_PI), 1)}});
  const double t0 = now();
  double done = 0.0;
  while (!shutdown() && done < 2 * M_PI) {
    if (auto lost = fc_lost()) return lost;
    if (auto fz = watch(idx)) return fz;
    if (preempted()) return std::string(PREEMPTED);
    done = std::min(2 * M_PI, land_spin_rate_ * (now() - t0));
    publish_yaw(wrap_pi(*y0 + done));
    sleep(0.1);
  }
  if (shutdown()) return std::string("shutdown");
  report({{"phase", "LAND_SPIN"}, {"leg", idx}, {"done", true}, {"spin_s", l4::py_round(now() - t0, 2)}});
  return waypoint(idx, land_face(gx, gy), "landing");
}

// at a waypoint, in the air with the bridge streaming in flight mode (owner 2026-10-29): face `yaw` (none:
// keep the yaw it has) and hover with the yaw settled; none when settled, "yaw did not settle" after
// ~yaw_settle_s (the caller takes the landing-not-clear path: QRLOITER, STOPPED)
Why MissionExecutive::waypoint(int idx, std::optional<double> yaw, const std::string& why) {
  // the bridge holds a fixed point (not SUPER's hover round its goal) while a waypoint yaw is set (Gate L2
  // b2), so "keep the yaw it has" goes out as the yaw it has now
  std::optional<double> hold_yaw = yaw;
  if (!hold_yaw) {
    std::lock_guard<std::mutex> g(lock_);
    hold_yaw = yaw_;
  }
  publish_yaw(hold_yaw);
  l4::ojson r = {{"phase", "WAYPOINT_YAW"}, {"leg", idx}, {"for", why}};
  r["yaw_deg"] = yaw ? l4::ojson(l4::py_round(*yaw * (180.0 / M_PI), 1)) : l4::ojson(nullptr);
  report(r);
  SettleWindow w(1.0);
  const double t0 = now();
  while (!shutdown() && now() - t0 < yaw_settle_s_) {
    if (auto lost = fc_lost()) return lost;
    if (auto fz = watch(idx)) return fz;
    if (preempted()) return std::string(PREEMPTED);
    {
      std::lock_guard<std::mutex> g(lock_);
      if (pose_ && yaw_) w.add(now(), *pose_, *yaw_);
    }
    if (w.settled(yaw_settle_speed_, yaw_settle_rate_, yaw, yaw_tol_)) {
      std::optional<double> y;
      {
        std::lock_guard<std::mutex> g(lock_);
        y = yaw_;
      }
      report({{"phase", "WAYPOINT_YAW"}, {"leg", idx}, {"settled", true}, {"yaw_deg", l4::py_round(*y * (180.0 / M_PI), 1)},
              {"settle_s", l4::py_round(now() - t0, 2)}});
      return std::nullopt;
    }
    sleep(0.05);
  }
  if (shutdown()) return std::string("shutdown");
  report({{"phase", "WAYPOINT_YAW"}, {"leg", idx}, {"settled", false}, {"waited_s", l4::py_round(now() - t0, 2)}});
  return std::string("yaw did not settle");
}

// a deliberate yaw scan at a waypoint (the occupancy grid's reason to yaw): turn on the spot through
// yaw_sweep_deg (+ counter-clockwise) at ~scan_rate_deg (or the leg's scan_rate_deg), then settle
Why MissionExecutive::scan(const l4::ojson& leg, int idx) {
  if (!flying_ || !streaming_ || bridge_mode_now_ != "flight")
    return std::string("scan leg needs a fly leg before it (in the air, the bridge streaming)");
  std::optional<double> y0;
  {
    std::lock_guard<std::mutex> g(lock_);
    y0 = yaw_;
  }
  if (!y0) return std::string("scan leg: no FC yaw");
  const double sweep = num(leg["yaw_sweep_deg"]) * (M_PI / 180.0);
  const double rate = leg.contains("scan_rate_deg") ? num(leg["scan_rate_deg"]) * (M_PI / 180.0) : scan_rate_;
  report({{"phase", "SCAN"}, {"leg", idx}, {"yaw_sweep_deg", leg["yaw_sweep_deg"]}, {"from_deg", l4::py_round(*y0 * (180.0 / M_PI), 1)}});
  const double t0 = now();
  double done = 0.0;
  while (!shutdown() && done < std::abs(sweep)) {
    if (auto lost = fc_lost()) return lost;
    done = std::min(std::abs(sweep), rate * (now() - t0));
    publish_yaw(wrap_pi(*y0 + (sweep < 0 ? -done : done)));
    sleep(0.1);
  }
  return waypoint(idx, wrap_pi(*y0 + sweep), "scan end");
}

void MissionExecutive::run() {
  if (interactive_) return run_interactive();
  ros::Rate rate(10);
  report({{"phase", "WAIT_ARMED"}, {"mission", mission_.name}, {"legs", mission_.legs.size()}});
  while (!shutdown() && !started_) rate.sleep();
  if (shutdown()) return;
  if (!set_mode("GUIDED")) return stop("could not enter GUIDED");
  flying_ = false;
  std::vector<l4::ojson> legs = mission_.legs;
  if (legs.empty() || legs[0]["type"] == "fly" || legs[0]["type"] == "land")
    legs.insert(legs.begin(), l4::ojson{{"type", "takeoff"}, {"height", mission_.takeoff_height}});
  for (int idx = 0; idx < static_cast<int>(legs.size()); ++idx) {
    const l4::ojson& leg = legs[idx];
    if (auto lost = fc_lost()) return stop(*lost);
    Why why;
    const std::string type = leg["type"].get<std::string>();
    if (type == "takeoff") {
      why = takeoff(leg["height"]);
    } else if (type == "fly") {
      if (!flying_) why = takeoff(leg["goal"][2]);
      if (!why) why = go(leg, idx, Vec3(num(leg["goal"][0]), num(leg["goal"][1]), num(leg["goal"][2])), false);
      // the waypoint at its end (owner 2026-10-29): face the next fly / land leg (or the leg's own yaw),
      // hovering with the yaw settled, before it starts; the last leg (DONE) and a scan next just hover on
      if (!why) {
        const l4::ojson* next = idx + 1 < static_cast<int>(legs.size()) ? &legs[idx + 1] : nullptr;
        std::optional<double> y = leg_yaw(leg);
        if (!y && next && (*next)["type"] == "fly") y = heading_to(num((*next)["goal"][0]), num((*next)["goal"][1]));
        if (!y && next && (*next)["type"] == "land") y = heading_to(num((*next)["at"][0]), num((*next)["at"][1]));
        if (y || next) why = waypoint(idx, y, "next leg");
      }
    } else if (type == "scan") {
      why = scan(leg, idx);
    } else if (type == "drive") {
      // in the air: land first, facing the drive, here or at a clear spot nearby (the landing-spot search)
      if (flying_) why = land_to_drive(leg, idx, num(leg["goal"][0]), num(leg["goal"][1]));
      if (!why) {
        const auto p = pose_copy();
        const double z = p ? (*p)[2] : 0.0;
        why = go(leg, idx, Vec3(num(leg["goal"][0]), num(leg["goal"][1]), z), true);
      }
    } else if (type == "land") {
      if (!flying_) {
        why = std::string("land leg on the ground: the mission needs a takeoff first");
      } else {
        const auto p = pose_copy();
        const double z = p ? (*p)[2] : 0.0;
        why = go(leg, idx, Vec3(num(leg["at"][0]), num(leg["at"][1]), z), false);
        // over the spot (owner 2026-10-29): face the drive that follows (the tank starts aligned) or the
        // leg's own yaw, hover with the yaw settled, and only then ask the landing check and land
        if (!why) {
          std::optional<double> y = leg_yaw(leg);
          if (!y && idx + 1 < static_cast<int>(legs.size()) && legs[idx + 1]["type"] == "drive")
            y = heading_to(num(legs[idx + 1]["goal"][0]), num(legs[idx + 1]["goal"][1]));
          why = waypoint(idx, y, "landing");
        }
        // owner 2026-11-03 (SITL demo): "landing_check: false" lands here without the check height, the check or the
        // search (land_unchecked); the default (true) is unchanged
        if (!why && !l4::truthy(l4::get(leg, "landing_check", true))) {
          why = land_unchecked(idx);
        } else if (!why && land_search_) {
          // owner 2026-11-02: a land leg lands as a click landing does: the check height, the check, else the
          // search, the spin, the move (SUPER), the check again; the candidates face the drive that follows
          double gx = std::numeric_limits<double>::quiet_NaN(), gy = gx;
          if (idx + 1 < static_cast<int>(legs.size()) && legs[idx + 1]["type"] == "drive") {
            gx = num(legs[idx + 1]["goal"][0]);
            gy = num(legs[idx + 1]["goal"][1]);
          }
          land_yaw_ = leg_yaw(leg);
          why = land_here_or_nearby(leg, idx, gx, gy);
          land_yaw_.reset();
        } else if (!why) {
          why = land(leg, idx);
        }
      }
    }
    if (!why) {
      const double hold_s = num(l4::get(leg, "hold_s", 0));
      if (hold_s > 0) sleep(hold_s);
      continue;
    }
    const std::string& w = *why;
    const std::string policy = l4::get(leg, "on_unreachable", on_unreachable_).get<std::string>();
    if (starts_with(w, "FC ") || starts_with(w, "FC link")) return stop(w);   // the FC's failsafe owns the vehicle now
    if (w == "shutdown") return;
    if (w == "takeoff refused" || w == NOT_STATIONARY) {
      // still on the ground, driving in GUIDED: QRLOITER there would start the drive-to-fly transition;
      // stay in GUIDED with the bridge off (Rover Guided stops)
      return stop(w);   // nothing after a failed takeoff can fly
    }
    if (starts_with(w, "takeoff")) {
      set_mode("QRLOITER");
      return stop(w);
    }
    if (w == LAND_NOT_CLEAR || w == NO_LAND_SPOT || w == "yaw did not settle") {
      set_mode("QRLOITER");
      return stop(w);
    }
    if (w == SLAM_FROZE) {
      // the FC's position follows the frozen pose: in the air ~slam_frozen_flight_mode (QRLAND: down, and done
      // with the position soonest); on the ground GUIDED with the bridge off (the bridge's zero-speed stop is
      // Rover's last target; a QR mode there would spool the props)
      if (flying_) set_mode(frozen_flight_mode_);
      return stop(w);
    }
    if (w == PLANNER_DIED) {
      // owner 2026-10-29: just stop, never the next leg. In the air QRLOITER (the bridge has held since
      // the death; set_mode turns it off just before the mode command, as on every QRLOITER); on the
      // ground GUIDED with the bridge off (a QR mode there would spool the props)
      if (flying_) set_mode("QRLOITER");
      return stop(w);
    }
    if (w == BRIDGE_DIED) {
      // L2 1c, 1d: nothing streams any more. In the air a hover target at the vehicle, then QRLOITER; on the ground
      // the executive's own drive stop until the rover is at rest (stop()). Never the next leg
      if (flying_) hover_then_loiter();
      return stop(w);
    }
    if (w == SLAM_HOLD_TIMEOUT) {
      // 10-28 note 5: a SLAM hold that does not end. In the air QRLOITER, then QRLAND; on the ground the drive stop
      if (flying_) {
        set_mode("QRLOITER");
        sleep(unreachable_hold_s_);
        if (!fc_lost()) set_mode("QRLAND");
      }
      return stop(w);
    }
    if (!flying_) {
      // on the ground: an unreachable drive goal, a mission error. Hold in GUIDED, bridge off, the drive stop
      bridge_enable(false);
      report({{"phase", "UNREACHABLE"}, {"leg", idx}, {"reason", w}, {"on_ground", true}});
      if (starts_with(w, "bridge") || starts_with(w, "land leg") || starts_with(w, "drive leg") || starts_with(w, "scan leg"))
        return stop(w);
      const double t_hold = now();
      drive_stop_until_rest(w);
      sleep(unreachable_hold_s_ - (now() - t_hold));
      if (policy != "next" || fc_lost()) return stop(w);
      continue;
    }
    // in the air: unreachable goal, refused landing, a bridge that did not switch: QRLoiter
    report({{"phase", "UNREACHABLE"}, {"leg", idx}, {"reason", w}});
    if (w == "unreachable" || w == GOAL_NOT_CLEAR) {
      // L2 4a: clear of what is under the vehicle first
      if (Why b = back_off(idx, w)) {
        if (starts_with(*b, "FC ")) return stop(*b);
        if (*b == SLAM_FROZE) {
          set_mode(frozen_flight_mode_);
          return stop(*b);
        }
        if (*b == BRIDGE_DIED) {
          hover_then_loiter();
          return stop(*b);
        }
      }
    }
    set_mode("QRLOITER");
    if (starts_with(w, "bridge") || starts_with(w, "drive leg") || starts_with(w, "scan leg")) return stop(w);
    sleep(unreachable_hold_s_);
    if (policy != "next" || fc_lost()) return stop(w);
    if (!set_mode("GUIDED")) return stop("could not re-enter GUIDED after " + w);
  }
  bridge_enable(false);
  report({{"phase", "DONE"}});
}

// interactive: one goal (a filled leg, fc frame); none when reached. hovering: set when a fly goal ends in
// its waypoint hover
Why MissionExecutive::interactive_goal(const l4::ojson& leg, int n, bool& hovering) {
  const std::string type = leg["type"].get<std::string>();
  const double gx = num(leg["goal"][0]), gy = num(leg["goal"][1]);
  Why why;
  if (type == "fly") {
    if (!flying_) {
      // off the ground: the bridge off (Rover Guided brakes on its last target), then the takeoff's standstill
      // rule; a drive cut short by this goal is not left streaming toward its old goal
      bridge_enable(false);
      why = takeoff(mission_.takeoff_height);
    }
    if (!why) why = go(leg, n, Vec3(gx, gy, num(leg["goal"][2])), false);
    // at the goal: its yaw (or the yaw it has), hover holding it until the next goal
    if (!why) why = waypoint(n, leg_yaw(leg), "goal");
    if (!why) hovering = true;
    return why;
  }
  // drive: in the air, land first, facing the goal: where the vehicle is, or at a clear spot nearby (the
  // landing-spot search)
  if (flying_) {
    const auto p = pose_copy();
    if (!p) return std::string("no FC pose");
    report({{"phase", "GOAL"}, {"goal", n}, {"landing_first", {l4::py_round((*p)[0], 3), l4::py_round((*p)[1], 3)}}});
    if (Why w = land_to_drive(leg, n, gx, gy)) return w;
  }
  const auto p = pose_copy();
  return go(leg, n, Vec3(gx, gy, p ? (*p)[2] : 0.0), true);
}

void MissionExecutive::run_interactive() {
  ros::Rate rate(10);
  report({{"phase", "WAIT_ARMED"}, {"mission", mission_.name}, {"interactive", true}});
  while (!shutdown() && !started_) rate.sleep();
  if (shutdown()) return;
  if (!set_mode("GUIDED")) return stop("could not enter GUIDED");
  flying_ = false;
  bool hovering = false;   // a fly goal reached: hovering on its waypoint
  int last = 0;
  while (!shutdown()) {
    report({{"phase", "IDLE"}, {"flying", flying_}, {"hovering", hovering}});
    std::optional<l4::ojson> leg;
    while (!shutdown()) {
      if (auto lost = fc_lost()) return stop(*lost);
      if (hovering) {
        if (auto fz = watch(last)) {   // the hover's position is the EKF's: as in a leg
          if (*fz == BRIDGE_DIED) hover_then_loiter();
          else set_mode(frozen_flight_mode_);
          return stop(*fz);
        }
      }
      {
        std::lock_guard<std::mutex> g(lock_);
        if (pending_) {
          leg = *pending_;
          pending_.reset();
        }
      }
      if (leg) break;
      sleep(0.1);
    }
    if (!leg) return;
    const int n = (*leg)["goal_id"].get<int>();
    last = n;
    face_first_ = flying_ && (hovering || yaw_held_);
    hovering = false;
    report({{"phase", "GOAL"}, {"goal", n}, {"type", (*leg)["type"]}, {"goal_fc", (*leg)["goal"]}, {"flying", flying_}});
    const Why why = interactive_goal(*leg, n, hovering);
    if (!why) {
      report({{"phase", "REACHED"}, {"goal", n}, {"flying", flying_}});
      continue;
    }
    const std::string& w = *why;
    if (w == PREEMPTED) {
      report({{"phase", "PREEMPTED"}, {"goal", n}});
      continue;
    }
    if (starts_with(w, "FC ") || starts_with(w, "FC link")) return stop(w);   // the FC's failsafe owns the vehicle
    if (w == "shutdown") return;
    if (w == SLAM_FROZE) {
      if (flying_) set_mode(frozen_flight_mode_);
      return stop(w);
    }
    if (w == PLANNER_DIED) {
      if (flying_) set_mode("QRLOITER");
      return stop(w);
    }
    if (w == BRIDGE_DIED) {
      if (flying_) hover_then_loiter();
      return stop(w);
    }
    if (w == SLAM_HOLD_TIMEOUT) {
      if (flying_) {
        set_mode("QRLOITER");
        sleep(unreachable_hold_s_);
        if (!fc_lost()) set_mode("QRLAND");
      }
      return stop(w);
    }
    if (w == "takeoff refused" || w == NOT_STATIONARY) {
      // on the ground in GUIDED (a QR mode there starts the drive-to-fly transition): bridge off, next goal
      bridge_enable(false);
      report({{"phase", "REFUSED"}, {"goal", n}, {"reason", w}, {"on_ground", true}});
      drive_stop_until_rest(w);
      continue;
    }
    if (starts_with(w, "takeoff")) {   // started and did not complete
      set_mode("QRLOITER");
      return stop(w);
    }
    if (starts_with(w, "bridge")) {
      if (flying_) set_mode("QRLOITER");
      return stop(w);
    }
    const bool refused = w == LAND_NOT_CLEAR || w == NO_LAND_SPOT || w == "yaw did not settle" || w == "landing refused";
    const std::string phase = refused ? "REFUSED" : "UNREACHABLE";
    if (!flying_) {
      bridge_enable(false);
      report({{"phase", phase}, {"goal", n}, {"reason", w}, {"on_ground", true}});
      drive_stop_until_rest(w);
      continue;
    }
    // in the air: QRLOITER for ~unreachable_hold_s, then GUIDED (bridge off) and the next goal
    report({{"phase", phase}, {"goal", n}, {"reason", w}});
    if (w == "unreachable" || w == GOAL_NOT_CLEAR) {
      if (Why b = back_off(n, w)) {
        if (starts_with(*b, "FC ")) return stop(*b);
      }
    }
    set_mode("QRLOITER");
    sleep(unreachable_hold_s_);
    if (auto lost = fc_lost()) return stop(*lost);
    if (!set_mode("GUIDED")) return stop("could not re-enter GUIDED after " + w);
  }
}

}  // namespace l4_executive

int main(int argc, char** argv) {
  ros::init(argc, argv, "mission_executive");
  try {
    l4_executive::MissionExecutive ex;
    ros::AsyncSpinner spinner(4);   // the inputs and the start service, while run() works the legs
    spinner.start();
    ex.run();
    ros::waitForShutdown();
  } catch (const std::exception& e) {
    ROS_FATAL("mission executive: %s", e.what());
    return 1;
  }
  return 0;
}
