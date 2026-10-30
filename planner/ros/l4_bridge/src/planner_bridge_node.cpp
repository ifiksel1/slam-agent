// Planner bridge (L4 autonomy, L2): SUPER's commands to QuadRover Guided targets, and FAST-LIO's output to
// SUPER's frame. The core logic is bridge_core.cpp (ROS-free, unit-tested). C++ port of the former
// planner/planner_bridge.py: node name, topics, types, parameters, status JSON and behaviour unchanged.
//
// In:   /Odometry, /cloud_registered          FAST-LIO, raw frame
//       /mavros/vision_pose/pose              the SLAM-to-FC bridge's output (fc frame), for the anchor
//       /mavros/local_position/pose           the FC's EKF pose (fc frame): the vehicle position for the
//                                             caps, the drive target, the HOLD point and the geofence;
//                                             trusted only while fresh (~fc_pose_timeout, see below)
//       /mavros/state                         stream only in GUIDED
//       /planner_bridge/enable                std_msgs/Bool, latched, from the mission executive: stream
//                                             only while true (its fly and drive legs), never during the
//                                             FC's own takeoff or landing, which a target would override
//       /planner_bridge/set_mode              std_msgs/String, latched, from the mission executive:
//                                             "flight" or "drive" (~mode is only the start value)
//       /planning/pos_cmd                     SUPER (quadrotor_msgs/PositionCommand, planner frame), used
//                                             in flight mode
//       /planning/click_goal                  the executive's goal for SUPER (planner frame): the
//                                             waypoint hold point (below); ~goal_topic
//       /planner_bridge/yaw_cmd               std_msgs/Float64, latched, from the mission executive: the
//                                             waypoint yaw (fc frame, rad; NaN clears it), see below
//       ~drive_cmd_topic                      the track planner's carrot (/track_planner/pos_cmd,
//                                             PositionCommand, position + yaw), used in drive mode
//       /fastlio_health, ~range_topic         optional SLAM health (off when absent)
//       /mavros/imu/data                      the FC's IMU (angular rate): the frozen-pose check's evidence
// Out:  /planner/odom, /planner/cloud         planner frame, for SUPER and ROG-Map
//       /mavros/setpoint_raw/local            mavros_msgs/PositionTarget, 50 Hz, fc frame
//       /planner_bridge/anchor                geometry_msgs/TransformStamped, fc <- planner
//       /planner_bridge/status                std_msgs/String, JSON (its "mode" is the mode in use)
//       /planner_bridge/mode                  std_msgs/String, latched: the mode in use, on every change
//       /planner_bridge/mode_code             std_msgs/Int8, 5 Hz, for plots: 0 not streaming,
//                                             1 streaming flight targets, 2 streaming drive targets
//
// Modes. flight: SUPER's PositionCommand -> PosVelAccel + yaw, capped (Guided flight). drive: the track
// planner's target (its carrot run out along the carrot's bearing, frame-checked by the track planner)
// -> a position-only target within ~drive_lookahead of the vehicle (Rover Guided on the ground rejects
// any target with acceleration; its fast-update path asks ~0.5 m/s per metre to the target, hence 2.5 m).
// The two planners publish on separate topics (SUPER keeps publishing toward its last goal after a
// landing); the bridge listens only to the one of its mode. A mode change drops the last command and
// holds (latched) until a command from the new mode's planner arrives, so a target of one planner is
// never sent in the other mode.
//
// It never arms, never changes modes and never sets parameters (D5): mode changes belong to the mission
// executive. In HOLD in flight it streams a zero-velocity target at the current position, so a fault
// never leaves the FC coasting on the last velocity until Q_GUID_TIMEOUT (D4). In HOLD in drive mode it
// streams nothing (except for a dead planner or during a reverse, below): Rover Guided drives on to the
// last target it was sent (at most ~drive_lookahead ahead, in the direction of travel), counts it
// reached and brakes to a stop. A position hold at the vehicle does not stop a rover: taken while it
// moves, the point is soon behind it and Rover Guided orbits it (SITL cl36, ~0.28 m/s for 15 s); a
// zero-speed heading target let SITL's rover lurch into reverse and creep (cl37, cl38).
//
// Reversing (owner 2026-10-28, dead ends). A track planner command whose velocity points against its
// yaw means: back straight up, holding the yaw, until the FC is at the command's position
// (bridge_core reverse_speed). It goes out as a Rover Guided velocity + heading target (mask 2503: no
// position; MAVROS turns the ENU velocity (0, -speed) into NED vx -speed, which Rover reads as reverse:
// GCS_MAVLink_Rover speed_dir), speed tapering to 0 at the end point: a Guided setpoint, within D5 (no
// DO_SET_REVERSE, no mode change). Rover's velocity + heading submode keeps the last speed for 3 s
// without targets, so a HOLD after a reverse streams a zero-speed one.
//
// A stale FC pose (owner decision 2026-10-28). The FC pose is the bridge's "where the vehicle is now"
// (p_now): every target is capped or placed relative to it. Its age is measured on the receive time.
//   * none received yet: nothing is streamed (status "fc_pose": "never", reason "no FC pose yet");
//   * older than ~fc_pose_timeout (0.3 s): HOLD with reason "fc_pose_stale", whatever the planner says.
//     In flight the hold target is the last pose received while it was fresh (FcPoseGuard.good),
//     streamed at the full rate with zero velocity so Guided does not time out on Q_GUID_TIMEOUT; in
//     drive mode nothing is sent, except during a reverse: a zero-speed heading target. A hold that
//     began earlier (say, planner stale) keeps its point;
//   * fresh again: trusted after ~fc_pose_resume_n poses in a row, each within ~fc_pose_timeout of the
//     one before; then the usual resume (a planner command newer than the hold).
// "fc_pose_stale" is not a SLAM hold ("slam_hold" false): the executive's leg clock keeps running.
//
// A dead planner (owner decision 2026-10-29: if the planner dies, the robot just stops; no continuation).
// The bridge remembers which node sent each mode's commands (the connection header's callerid) and pings
// it (PlannerLiveness: its XML-RPC getPid, as `rosnode ping`, every ~planner_ping_s 0.1 s). No command
// from the mode's planner for ~planner_timeout (0.3 s) AND that process gone (connection refused or
// unregistered: at once; no answer for ~planner_hung_s 1.0 s: hung) -> HOLD with reason "planner_dead"
// (HoldLogic), latched:
//   * flight: the usual HOLD, a zero-velocity target at the FC position, at the full rate;
//   * drive: a zero-speed velocity + heading target (mask 2503, as the reverse HOLD) at the heading the
//     rover had at the death, streamed at the full rate: the rover stops where it is;
//   * no command clears it, even from a restarted planner: the leg is over. Only the executive enabling
//     the bridge again (false -> true, a new leg) does, and it latches again at once if that planner is
//     still gone. A planner that never sent a command in this mode, and none known from before, keeps the
//     wait before the first plan. A live planner with nothing to send is the ordinary "planner stale"
//     hold, resumable.
// Status: "planner_age", "planner_node", "planner_alive" (true | false | null). The mission executive
// stops the mission on "planner_dead" (STOPPED "planner died").
//
// Flight yaw (owner rule 2026-10-29: the heading points along the direction of travel; the robot yaws
// only at a waypoint, unless there is a reason). SUPER's yaw does not do this: with goal_yaw_en it turns
// toward each goal's orientation (the executive's goals carry none: yaw 0), and its yaw lags or leads the
// travel direction (Gate L2 b1: median 8 deg, p90 67 deg off the velocity heading above 0.5 m/s; 50-180
// deg in hovers, swings of 100 deg while hovering over a landing spot). So by default (~flight_yaw travel)
// the bridge computes the flight yaw itself: the heading of SUPER's commanded velocity while its
// horizontal speed is above ~yaw_speed_min (0.3 m/s); slower (arriving, hovering, any flight HOLD) it
// holds the yaw it has, or faces the executive's waypoint yaw (/planner_bridge/yaw_cmd, std_msgs/Float64,
// fc frame, rad; NaN clears). Either way at most ~yaw_rate_max_deg (60 deg/s), starting from the FC's yaw
// when the stream starts. ~flight_yaw planner passes SUPER's yaw through (the former Python bridge).
// Drive mode is unchanged (the track planner drives nose-first and pivots at path vertices; the reverse
// out of a dead end is the exception). Status: "flight_yaw", "yaw_cmd_deg".
// The yaw step is ~yaw_rate_max_deg times the time since the last tick (at most two ticks' worth), and
// each setpoint is stamped with its tick's time, so the commanded rate stays at the cap when the timer
// fires late and catches up (Gate L2 b2 read 69 deg/s: a fixed 1.2 deg step between setpoints 17 ms apart).
//
// The waypoint hold (Gate L2 b2: SUPER, hovering at its reached goal, kept commanding 0.2-0.4 m/s; above
// ~yaw_speed_min the travel yaw took over, the setpoint yaw swung -67..-98 deg round the -82 deg waypoint
// yaw, the vehicle never stood still and the executive stopped, "yaw did not settle"). In flight, while
// the executive's waypoint yaw is set and the bridge would stream, it does not follow SUPER: a fixed point,
// zero velocity and acceleration, the yaw turning to the waypoint yaw at ~yaw_rate_max_deg (in either
// ~flight_yaw). The point (bridge_core waypoint_hold_point): the reached goal (the executive's last
// ~goal_topic goal, anchored to fc) if within ~waypoint_goal_max (1.0 m) of the vehicle when the hold
// begins, else the vehicle's position then; the target is capped like any other (cap_command), so the
// vehicle eases onto it. Status: state "WAYPOINT_HOLD", reason "waypoint yaw", "waypoint_hold" {"point",
// "from": "goal" | "vehicle", "phase": "turn" | "depart"} (null otherwise). The executive clears the yaw
// (NaN) as the next leg starts, with the next goal. SUPER plans that leg from its own committed
// trajectory, not from the vehicle, and that kept hovering round the old goal (0.2-0.5 m from it, at up
// to 0.6 m/s) while the vehicle held; its first commands after the clear went backwards (Gate L2 b5, leg
// 2: 2 s at up to 0.6 m/s, the travel yaw turning the nose after them). So the hold goes on, the point and
// the yaw kept (phase "depart"), until SUPER sets off toward the new goal (bridge_core departed: its
// velocity along the hold point -> goal direction above ~yaw_speed_min, its position not behind the hold
// point), at most ~depart_timeout_s (5 s), then streaming resumes with SUPER's reference at the vehicle.
// A new waypoint yaw, a mode change or the stream stopping ends it. The faults keep their HOLD (a fault's HOLD wins: zero velocity
// at the vehicle, as before), and the planner-death latch, the stale-pose hold, the enable gating and drive
// mode are unchanged.
//
// The turn slowdown (owner 2026-10-29: the lidar faces forward, so the robot must not fly faster than its
// nose can turn toward the direction of travel; Gate L2 b5-b7: heading median 5-10 deg off the travel
// direction but p90 40-65 deg, the FC's yaw 20-35 deg behind the 60 deg/s ramp on SUPER's curves). In flight,
// while SUPER's command streams (not in the waypoint hold, the departure, any HOLD, nor in drive mode), the
// allowed speed is ~speed_cap times bridge_core turn_speed_frac of the heading error: 1 up to
// ~turn_slow_start_deg (15), ~turn_slow_min_frac (0.25) from ~turn_slow_full_deg (45) on, smooth between.
// The error is the FC's yaw against SUPER's commanded travel direction, and against that direction
// ~turn_slow_lookahead_s (0.5 s) ahead (v + a t: SUPER's acceleration turns its velocity before the corner,
// so the slowdown starts before the error builds), the larger (bridge_core turn_heading_error). The allowed
// speed changes at ~turn_slow_accel (1.5 m/s^2) at most. SUPER cannot be slowed, so the bridge replays
// SUPER's own reference (its commands by receive time, bridge_core RefBuffer) at a rate that keeps the
// replayed speed within the allowed one (bridge_core replay_rate), velocity and acceleration scaled with it,
// and caps the result at the allowed speed (cap_command with its fc_pos_p lead clamp, so the FC's total
// demand stays within it). The vehicle stays on the path SUPER flew, only later; behind, the replay catches
// up at up to ~turn_slow_catchup (1.5x) as the allowed speed permits; the lag levels off at
// ~turn_slow_max_lag_s (0.5 s: SUPER checks its backup trajectory from the vehicle, and in Gate L2 b9, with 2 s,
// SUPER stalled at the tower with the replay 0.77 s behind), the replay rate's floor rising to SUPER's pace over
// the last 0.5 s of it, here from no lag on (the
// cap rising with it). The replay restarts without lag whenever the stream of SUPER's commands resumes, and
// when SUPER's reference jumps (> REF_JUMP_M between two commands: a new plan from rest starts at the vehicle).
// Status "turn_slow": {"err_deg", "frac" (of the speed cap the heading allows), "allowed" (the cap applied,
// m/s), "rate", "lag_s"} (null when not replaying). ~turn_slow_enable false: SUPER's newest command capped
// at ~speed_cap, as before.
//
// The local ceiling (container fly-through k1, 2026-10-29: SUPER's re-seed goals, 0.3 m above the vehicle each,
// stacked to 1.64 m under a 2.15 m roof). Each /cloud_registered scan, in the planner frame, gives the lowest
// point within ~ceiling_radius (0.6 m: SUPER's 0.5 m inflation and a cell) of the vehicle horizontally and more
// than ~ceiling_min_up (0.15 m) above it (bridge_core CeilingFinder). Status "ceiling_above": from the vehicle up
// to that point (m), from a scan at most ~ceiling_max_age (1.0 s) old; null when no point is over the vehicle,
// or no recent scan. The executive keeps SUPER's re-seed goal SUPER's inflation below it.
// A flight HOLD holds the vehicle's position when the hold began (zero velocity): it never climbs, whatever
// SUPER's last command was (k1: 1.046 m and 1.334 m held flat through both of SUPER's silences).
//
// A frozen SLAM pose (L2 faults 2b, 2d, 2026-10-28: FAST-LIO's last pose repeated with fresh stamps; the
// receive-time SLAM check and the FC-pose check both passed, the EKF fused the frozen pose and the FC flew
// off at up to 8.4 m/s, 45 m, and drove off at 2.0 m/s, 12.7 m). bridge_core FrozenPoseCheck: the raw
// /Odometry pose repeated (within ~freeze_eps_m 1e-5 m and ~freeze_eps_q 1e-6 per quaternion component; a
// real FAST-LIO pose never repeats, SITL's does while parked) at least ~freeze_min_n (4) times in a row,
// while there is evidence that the vehicle moves or is made to move, without a break for ~freeze_hold_s
// (0.4 s; driving ~freeze_drive_hold_s 0.8 s: SITL's parked rover repeats its pose until it moves): this bridge
// streaming a flight target that asks the FC for at least ~freeze_cmd_speed (0.2 m/s: the feed-forward plus
// ~fc_pos_p times the lead to the target) or a drive target at least ~freeze_drive_dist (0.5 m) from the
// vehicle (a reverse at ~freeze_cmd_speed), the FC IMU's angular rate (/mavros/imu/data) at least ~freeze_gyro_min (0.2 rad/s), or the repeated /Odometry's own
// twist at least ~freeze_twist_min (0.2 m/s). Then HOLD with reason "slam_frozen", latched (HoldLogic):
//   * flight: the usual HOLD at the FC position (zero velocity); the mission executive switches to QRLAND
//     (D5) and stops the mission: nothing that holds a position can be trusted with a frozen pose;
//   * drive: the zero-speed velocity + heading stop of a dead planner, at the heading the rover has; the
//     executive stops the mission in GUIDED;
//   * only a new leg (the executive enabling the bridge again) clears it, and it latches again at once while
//     the pose is still frozen. ~freeze_enable false turns the check off.
// Status: "slam_frozen" {"repeats", "for_s", "evidence"} while a repeat run lasts (null otherwise); "slam_hold"
// stays false (the executive does not wait it out as a SLAM reset: it lands or stops).
#include <algorithm>
#include <cmath>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>

#include <l4_common/params.h>
#include <l4_common/pyjson.h>

#include "l4_bridge/bridge_core.h"
#include "l4_bridge/planner_liveness.h"

namespace l4_bridge {

// MAVROS PositionTarget type_mask bits (MAVLink POSITION_TARGET_TYPEMASK)
constexpr uint16_t IGNORE_PX = 1, IGNORE_PY = 2, IGNORE_PZ = 4;
constexpr uint16_t IGNORE_VX = 8, IGNORE_VY = 16, IGNORE_VZ = 32;
constexpr uint16_t IGNORE_AFX = 64, IGNORE_AFY = 128, IGNORE_AFZ = 256;
constexpr uint16_t IGNORE_YAW_RATE = 2048;
// flight: position + velocity + acceleration + yaw (QuadRover Guided flight -> set_pos_vel_accel_NED_m)
constexpr uint16_t MASK_FLIGHT = IGNORE_YAW_RATE;
// driving: position + yaw only (Rover Guided rejects any target with acceleration set)
constexpr uint16_t MASK_DRIVE = IGNORE_VX | IGNORE_VY | IGNORE_VZ | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ | IGNORE_YAW_RATE;
// reversing (dead ends): velocity + yaw, no position (Rover Guided HeadingAndSpeed; a negative NED vx
// backs up holding the yaw; stops by itself 3 s after the last target)
constexpr uint16_t MASK_REVERSE = IGNORE_PX | IGNORE_PY | IGNORE_PZ | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ | IGNORE_YAW_RATE;

// a step in SUPER's reference between two commands (~10 ms apart) bigger than this is a jump: the turn
// slowdown's replay restarts at the new reference
constexpr double REF_JUMP_M = 0.3;

const char* const SLAM_HOLD_REASONS[] ={"SLAM", "no SLAM", "frame re-anchored", "no frame anchor"};
const char* const MODES[] = {"flight", "drive"};

template <class M>
Vec3 vec(const M& m) {
  return Vec3(m.x, m.y, m.z);
}
template <class M>
Quat quat(const M& m) {
  return {m.x, m.y, m.z, m.w};
}

bool starts_with(const std::string& s, const char* p) { return s.rfind(p, 0) == 0; }

std::string lower_strip(const std::string& s) {
  const char* ws = " \t\n\r\f\v";
  size_t a = s.find_first_not_of(ws), b = s.find_last_not_of(ws);
  std::string o = a == std::string::npos ? "" : s.substr(a, b - a + 1);
  for (auto& c : o) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return o;
}

class PlannerBridge {
 public:
  PlannerBridge()
      : pnh_("~"),
        fence_(Vec3::Zero(), Vec3::Zero()),
        liveness_(ros::this_node::getName(), l4::param_double(pnh_, "planner_ping_s", 0.1),
                  l4::param_double(pnh_, "planner_ping_timeout", 0.25), l4::param_double(pnh_, "planner_hung_s", 1.0)) {
    mode_ = l4::param_string(pnh_, "mode", "flight");   // flight | drive: the start value; the executive
    if (mode_ != "flight" && mode_ != "drive")           // sets it on /planner_bridge/set_mode
      throw std::invalid_argument("~mode must be one of ('flight', 'drive')");
    rate_ = l4::param_double(pnh_, "rate_hz", 50.0);
    speed_cap_ = l4::param_double(pnh_, "speed_cap", 0.5);      // 1.0 in sim, 0.5 robot (D10)
    max_lead_ = l4::param_double(pnh_, "max_lead", 1.0);
    fc_pos_p_ = l4::param_double(pnh_, "fc_pos_p", 1.0);        // the FC's Q_P_NE_POS_P: the cap bounds the FC's demand
    drive_lookahead_ = l4::param_double(pnh_, "drive_lookahead", 2.5);   // above the track planner's run-out (<= 2.1)
    guided_mode_ = l4::param_string(pnh_, "guided_mode", "GUIDED");
    auto lo = l4::param_vec(pnh_, "geofence_min", {-50.0, -50.0, -1.0});
    auto hi = l4::param_vec(pnh_, "geofence_max", {50.0, 50.0, 10.0});
    if (lo.size() != 3 || hi.size() != 3) throw std::invalid_argument("~geofence_min / ~geofence_max need 3 values");
    fence_ = Geofence(Vec3(lo[0], lo[1], lo[2]), Vec3(hi[0], hi[1], hi[2]));
    hold_ = HoldLogic(l4::param_double(pnh_, "planner_timeout", 0.3), l4::param_double(pnh_, "slam_timeout", 0.2));
    fc_guard_ = FcPoseGuard(l4::param_double(pnh_, "fc_pose_timeout", 0.3), l4::param_int(pnh_, "fc_pose_resume_n", 5));
    tracker_ = AnchorTracker(l4::param_double(pnh_, "anchor_jump_m", 0.1), l4::param_double(pnh_, "anchor_jump_deg", 2.0),
                             l4::param_int(pnh_, "anchor_persist", 3),
                             l4::param_double(pnh_, "fc_mount_yaw_deg", 0.0) * (M_PI / 180.0));
    health_ = HealthCheck(l4::param_double(pnh_, "health_matched_min", 150.0), l4::param_double(pnh_, "health_eig_min", 5000.0),
                          l4::param_double(pnh_, "health_latency_max_ms", 250.0), l4::param_double(pnh_, "height_tol", 0.5));
    health_on_ = l4::param_bool(pnh_, "health_enable", false);
    // a frozen SLAM pose (L2 faults 2b, 2d): repeated poses with evidence of motion -> HOLD "slam_frozen"
    freeze_on_ = l4::param_bool(pnh_, "freeze_enable", true);
    freeze_ = FrozenPoseCheck(l4::param_double(pnh_, "freeze_eps_m", 1e-5), l4::param_double(pnh_, "freeze_eps_q", 1e-6),
                              l4::param_double(pnh_, "freeze_hold_s", 0.4), l4::param_int(pnh_, "freeze_min_n", 4),
                              l4::param_double(pnh_, "freeze_cmd_speed", 0.2), l4::param_double(pnh_, "freeze_drive_dist", 0.5),
                              l4::param_double(pnh_, "freeze_gyro_min", 0.2),
                              l4::param_double(pnh_, "freeze_twist_min", 0.2), l4::param_double(pnh_, "freeze_evidence_age", 0.25),
                              0.5, l4::param_double(pnh_, "freeze_drive_hold_s", 0.8));
    // flight yaw (owner rule 2026-10-29): "travel" points the nose along the commanded velocity while it
    // is above ~yaw_speed_min and holds the yaw (or the executive's waypoint yaw) when slower, rate-limited
    // to ~yaw_rate_max_deg; "planner" passes SUPER's yaw through (the Python bridge's behaviour)
    flight_yaw_ = l4::param_string(pnh_, "flight_yaw", "travel");
    if (flight_yaw_ != "travel" && flight_yaw_ != "planner") throw std::invalid_argument("~flight_yaw must be travel or planner");
    yaw_speed_min_ = l4::param_double(pnh_, "yaw_speed_min", 0.3);
    yaw_rate_max_ = l4::param_double(pnh_, "yaw_rate_max_deg", 60.0) * (M_PI / 180.0);
    // the waypoint hold: at the reached goal when it is this close to the vehicle, else at the vehicle
    wp_goal_max_ = l4::param_double(pnh_, "waypoint_goal_max", 1.0);
    // after the waypoint yaw is cleared, hold on until SUPER sets off toward the goal, at most this long
    depart_timeout_ = l4::param_double(pnh_, "depart_timeout_s", 5.0);
    // the turn slowdown (owner 2026-10-29): flight speed by the heading error, SUPER's reference replayed
    turn_slow_ = l4::param_bool(pnh_, "turn_slow_enable", true);
    turn_start_ = l4::param_double(pnh_, "turn_slow_start_deg", 15.0) * (M_PI / 180.0);
    turn_full_ = l4::param_double(pnh_, "turn_slow_full_deg", 45.0) * (M_PI / 180.0);
    turn_min_frac_ = l4::param_double(pnh_, "turn_slow_min_frac", 0.25);
    turn_lookahead_ = l4::param_double(pnh_, "turn_slow_lookahead_s", 0.5);
    turn_accel_ = l4::param_double(pnh_, "turn_slow_accel", 1.5);
    turn_catchup_ = l4::param_double(pnh_, "turn_slow_catchup", 1.5);
    turn_max_lag_ = l4::param_double(pnh_, "turn_slow_max_lag_s", 0.5);
    // the local ceiling over the vehicle (status "ceiling_above"; the executive caps SUPER's re-seed with it)
    ceiling_radius_ = l4::param_double(pnh_, "ceiling_radius", 0.6);
    ceiling_min_up_ = l4::param_double(pnh_, "ceiling_min_up", 0.15);
    ceiling_max_age_ = l4::param_double(pnh_, "ceiling_max_age", 1.0);
    // the executive's go-ahead; ~require_enable false streams in every GUIDED (bench use only)
    enabled_ = !l4::param_bool(pnh_, "require_enable", true);
    liveness_.start();

    pub_odom_ = nh_.advertise<nav_msgs::Odometry>("/planner/odom", 20);
    pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/planner/cloud", 5);
    pub_sp_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    pub_anchor_ = nh_.advertise<geometry_msgs::TransformStamped>("/planner_bridge/anchor", 5, true);
    pub_status_ = nh_.advertise<std_msgs::String>("/planner_bridge/status", 5);
    pub_mode_ = nh_.advertise<std_msgs::String>("/planner_bridge/mode", 1, true);
    pub_mode_code_ = nh_.advertise<std_msgs::Int8>("/planner_bridge/mode_code", 5);
    publish_mode();

    subs_.push_back(nh_.subscribe("/Odometry", 50, &PlannerBridge::on_odom, this));
    subs_.push_back(nh_.subscribe("/cloud_registered", 5, &PlannerBridge::on_cloud, this));
    subs_.push_back(nh_.subscribe("/mavros/vision_pose/pose", 50, &PlannerBridge::on_vision, this));
    subs_.push_back(nh_.subscribe("/mavros/local_position/pose", 20, &PlannerBridge::on_fc_pose, this));
    subs_.push_back(nh_.subscribe("/mavros/state", 5, &PlannerBridge::on_state, this));
    subs_.push_back(nh_.subscribe("/planner_bridge/enable", 5, &PlannerBridge::on_enable, this));
    subs_.push_back(nh_.subscribe("/planner_bridge/set_mode", 5, &PlannerBridge::on_set_mode, this));
    subs_.push_back(nh_.subscribe("/planner_bridge/yaw_cmd", 5, &PlannerBridge::on_yaw_cmd, this));
    subs_.push_back(nh_.subscribe(l4::param_string(pnh_, "goal_topic", "/planning/click_goal"), 5, &PlannerBridge::on_goal, this));
    subs_.push_back(nh_.subscribe<quadrotor_msgs::PositionCommand>(
        "/planning/pos_cmd", 20,
        boost::function<void(const ros::MessageEvent<const quadrotor_msgs::PositionCommand>&)>(
            [this](const ros::MessageEvent<const quadrotor_msgs::PositionCommand>& e) { on_cmd(e, "flight"); })));
    subs_.push_back(nh_.subscribe<quadrotor_msgs::PositionCommand>(
        l4::param_string(pnh_, "drive_cmd_topic", "/track_planner/pos_cmd"), 20,
        boost::function<void(const ros::MessageEvent<const quadrotor_msgs::PositionCommand>&)>(
            [this](const ros::MessageEvent<const quadrotor_msgs::PositionCommand>& e) { on_cmd(e, "drive"); })));
    if (freeze_on_) subs_.push_back(nh_.subscribe("/mavros/imu/data", 20, &PlannerBridge::on_imu, this));
    if (health_on_) {
      subs_.push_back(nh_.subscribe("/fastlio_health", 10, &PlannerBridge::on_health, this));
      subs_.push_back(nh_.subscribe(l4::param_string(pnh_, "range_topic", "/mavros/rangefinder/rangefinder"), 10,
                                    &PlannerBridge::on_range, this));
    }
    tick_timer_ = nh_.createTimer(ros::Duration(1.0 / rate_), &PlannerBridge::tick, this);
    status_timer_ = nh_.createTimer(ros::Duration(0.2), &PlannerBridge::publish_status, this);
    ROS_INFO("planner bridge: mode %s, %.0f Hz, speed cap %.2f m/s, geofence [%g %g %g]..[%g %g %g]", mode_.c_str(), rate_,
             speed_cap_, fence_.lo[0], fence_.lo[1], fence_.lo[2], fence_.hi[0], fence_.hi[1], fence_.hi[2]);
  }

 private:
  static double now() { return ros::Time::now().toSec(); }

  void publish_mode() {
    std_msgs::String m;
    m.data = mode_;
    pub_mode_.publish(m);
  }

  // ---- inputs ------------------------------------------------------------------------------------
  void on_odom(const nav_msgs::Odometry::ConstPtr& m) {
    const Vec3 p = raw_to_planner_pos(vec(m->pose.pose.position));
    const Quat q = raw_to_planner_quat(quat(m->pose.pose.orientation));
    const Vec3 v = raw_to_planner_pos(vec(m->twist.twist.linear));   // FAST-LIO twist is world-frame
    last_planner_ = std::make_pair(p, q);
    t_slam_ = now();
    freeze_.on_pose(*t_slam_, p, q, v);
    nav_msgs::Odometry o;
    o.header.stamp = m->header.stamp;
    o.header.frame_id = "world";
    o.child_frame_id = "body";
    o.pose.pose.position.x = p[0];
    o.pose.pose.position.y = p[1];
    o.pose.pose.position.z = p[2];
    o.pose.pose.orientation.x = q.x;
    o.pose.pose.orientation.y = q.y;
    o.pose.pose.orientation.z = q.z;
    o.pose.pose.orientation.w = q.w;
    o.twist.twist.linear.x = v[0];
    o.twist.twist.linear.y = v[1];
    o.twist.twist.linear.z = v[2];
    pub_odom_.publish(o);
  }

  void on_cloud(const sensor_msgs::PointCloud2::ConstPtr& m) {
    // rotate x, y, z in place (fields are float32 at offsets 0, 4, 8 in FAST-LIO's clouds)
    sensor_msgs::PointCloud2 out(*m);
    const size_t step = m->point_step;
    // the local ceiling over the vehicle (its newest planner-frame position), from the same points
    std::optional<CeilingFinder> ceil;
    if (last_planner_) ceil.emplace(last_planner_->first, ceiling_radius_, ceiling_min_up_);
    if (step >= 12) {
      const size_t n = out.data.size() / step;
      for (size_t i = 0; i < n; ++i) {
        uint8_t* d = out.data.data() + i * step;
        float xyz[3];
        std::memcpy(xyz, d, 12);
        const float planner[3] = {xyz[2], xyz[0], xyz[1]};   // POSITION_MATRIX: (z, x, y)
        std::memcpy(d, planner, 12);
        if (ceil) ceil->add(planner[0], planner[1], planner[2]);
      }
    }
    if (ceil) {
      t_ceiling_ = now();
      ceiling_ = ceil->above();
    }
    out.header.frame_id = "world";
    pub_cloud_.publish(out);
  }

  void on_vision(const geometry_msgs::PoseStamped::ConstPtr& m) {
    // the bridge publishes one vision pose per /Odometry, right after it: pair with the newest
    if (!last_planner_) return;
    const auto [p, q] = *last_planner_;
    auto [anchor, jump] = tracker_.update(p, q, vec(m->pose.position), quat(m->pose.orientation));
    if (jump) {
      ROS_WARN("planner bridge: %s: HOLD until a fresh plan", jump->c_str());
      hold_.latch("frame re-anchored (" + *jump + ")", now());
    }
    geometry_msgs::TransformStamped t;
    t.header.stamp = m->header.stamp;
    t.header.frame_id = "map";
    t.child_frame_id = "planner";
    t.transform.translation.x = anchor.t[0];
    t.transform.translation.y = anchor.t[1];
    t.transform.translation.z = anchor.t[2];
    const Quat qa = quat_yaw(anchor.psi);
    t.transform.rotation.x = qa.x;
    t.transform.rotation.y = qa.y;
    t.transform.rotation.z = qa.z;
    t.transform.rotation.w = qa.w;
    pub_anchor_.publish(t);
  }

  void on_fc_pose(const geometry_msgs::PoseStamped::ConstPtr& m) {
    fc_pose_ = FcPose{vec(m->pose.position), yaw_of(quat(m->pose.orientation))};
    fc_guard_.on_pose(now(), *fc_pose_);
  }

  void on_state(const mavros_msgs::State::ConstPtr& m) { fc_mode_ = m->mode; }

  // the FC's IMU: its angular rate is the frozen-pose check's evidence (raw gyro, not the EKF's attitude)
  void on_imu(const sensor_msgs::Imu::ConstPtr& m) { freeze_.on_gyro(now(), vec(m->angular_velocity).norm()); }

  void on_enable(const std_msgs::Bool::ConstPtr& m) {
    if (m->data && !enabled_) {
      // the executive's new leg: a planner_dead latch from before is over (it latches again at once
      // while that planner is still gone)
      hold_.new_leg();
      stop_yaw_.reset();
    }
    enabled_ = m->data;
  }

  void on_set_mode(const std_msgs::String::ConstPtr& m) {
    const std::string mode = lower_strip(m->data);
    if (mode != "flight" && mode != "drive") {
      ROS_ERROR("planner bridge: unknown mode '%s' ignored (flight | drive)", m->data.c_str());
      return;
    }
    if (mode == mode_) return;
    // the other planner's last command must never be sent in this mode: drop it and hold until the new
    // mode's planner sends a command (HoldLogic.latch needs one newer than now)
    mode_ = mode;
    cmd_.reset();
    t_cmd_.reset();
    ref_.clear();
    replay_on_ = false;
    hold_point_.reset();
    reversing_.reset();
    stop_yaw_.reset();
    yaw_out_.reset();
    wp_point_.reset();
    depart_.reset();
    hold_.latch("mode changed to " + mode, now());
    publish_mode();
    ROS_INFO("planner bridge: mode %s", mode.c_str());
  }

  void on_cmd(const ros::MessageEvent<const quadrotor_msgs::PositionCommand>& e, const std::string& source) {
    if (source != mode_) return;   // the other mode's planner
    const auto& m = *e.getConstMessage();
    cmd_ = Cmd{vec(m.position), vec(m.velocity), vec(m.acceleration), m.yaw};
    t_cmd_ = now();
    if (source == "flight") {
      // SUPER's reference, for the turn slowdown's replay (planner frame: anchored when replayed). A jump in
      // it (SUPER planning again from rest starts at the vehicle, not where its old reference had got to):
      // what was buffered is no longer SUPER's path; the replay restarts at the new reference
      if (!ref_.empty() && (cmd_->p - ref_.last().p).norm() > REF_JUMP_M) {
        ref_.clear();
        replay_on_ = false;
      }
      ref_.push(RefBuffer::Sample{*t_cmd_, cmd_->p, cmd_->v, cmd_->a, cmd_->yaw});
      if (!replay_on_) ref_.trim(*t_cmd_ - 0.5);   // not replaying: nothing older is needed
    }
    const std::string sender = e.getPublisherName();
    if (!sender.empty()) liveness_.set_node(source, sender);   // the planner process whose liveness counts
  }

  // the executive's waypoint yaw (fc frame, rad; NaN clears it): while set, the waypoint hold (flight);
  // a new value while set (a scan's sweep) keeps the hold point, NaN ends the hold
  void on_yaw_cmd(const std_msgs::Float64::ConstPtr& m) {
    if (std::isfinite(m->data)) {
      yaw_cmd_ = wrap_pi(m->data);
      depart_.reset();
    } else {
      // the next leg: hold on at the point until SUPER sets off toward its goal
      if (yaw_cmd_ && wp_point_) depart_ = Depart{*wp_point_, now()};
      yaw_cmd_.reset();
      wp_point_.reset();
    }
  }

  // the executive's goal for SUPER (planner frame): the waypoint hold's point once reached, the
  // departure's direction
  void on_goal(const geometry_msgs::PoseStamped::ConstPtr& m) {
    goal_planner_ = vec(m->pose.position);
    t_goal_ = now();
  }

  // the flight yaw to send. planner: SUPER's (anchored) yaw. travel (owner rule 2026-10-29): the travel
  // heading while moving; slow or hovering, the executive's waypoint yaw, else the yaw it has. waypoint
  // (the waypoint hold, either ~flight_yaw): the waypoint yaw. One step of at most yaw_rate_max per
  // second since the last tick from the yaw last sent (the FC's yaw when the stream starts)
  double flight_yaw(double planner_yaw, std::optional<double> travel, double yaw_now, bool waypoint = false) {
    if (flight_yaw_ != "travel" && !waypoint) {
      yaw_out_ = planner_yaw;   // where a waypoint hold's turn starts from
      return planner_yaw;
    }
    if (!yaw_out_) yaw_out_ = yaw_now;
    const double desired = travel ? *travel : yaw_cmd_ ? *yaw_cmd_ : *yaw_out_;
    yaw_out_ = yaw_step(*yaw_out_, desired, yaw_max_step(yaw_rate_max_, tick_dt_, 2.0 / rate_));
    return *yaw_out_;
  }

  void on_health(const std_msgs::Float64MultiArray::ConstPtr& m) { health_.on_health(now(), m->data); }

  void on_range(const sensor_msgs::Range::ConstPtr& m) {
    if (fc_pose_) health_.on_range(now(), m->range, fc_pose_->p[2]);
  }

  // ---- output ------------------------------------------------------------------------------------
  void tick(const ros::TimerEvent&) {
    stamp_ = ros::Time::now();
    const double t_now = stamp_.toSec();
    tick_dt_ = t_tick_ ? t_now - *t_tick_ : 1.0 / rate_;   // the yaw step's time (yaw_max_step clamps it)
    t_tick_ = t_now;
    wp_active_ = false;
    // the replay runs only while SUPER's command streams as such (not in a hold, a waypoint hold, a departure,
    // drive mode or with the stream off); it restarts without lag
    if (!replay_tick_) {
      replay_on_ = false;
      turn_.reset();
    }
    replay_tick_ = false;
    std::vector<std::string> faults;
    if (health_on_) faults = health_.faults(t_now);
    const auto anchor = tracker_.anchor;
    const std::string mode = mode_;
    const auto cmd = cmd_;
    struct Target { Vec3 p, v, a; double yaw; };
    std::optional<Target> target;
    if (cmd && anchor) {
      target = Target{anchor->pos(cmd->p), anchor->vec(cmd->v), anchor->vec(cmd->a), anchor->yaw(cmd->yaw)};
      if (!fence_.inside(target->p)) faults.push_back("target outside the geofence");
    }
    if (fc_pose_ && !fence_.inside(fc_pose_->p)) faults.push_back("vehicle outside the geofence");
    if (!anchor) faults.push_back("no frame anchor yet (no vision pose)");
    const std::string fc = fc_guard_.check(t_now);
    if (fc == "never") faults.insert(faults.begin(), "no FC pose yet");
    else if (fc == "stale") faults.insert(faults.begin(), "fc_pose_stale");   // the reason to report: first
    const auto node = liveness_.node(mode);
    liveness_.want(node);
    const auto frozen = freeze_on_ ? freeze_.check(t_now, mode == "drive") : std::nullopt;
    if (frozen && !hold_.frozen)
      ROS_ERROR("planner bridge: SLAM pose frozen (%d repeats over %.2f s, evidence %s): HOLD slam_frozen", freeze_.repeats,
                t_now - *freeze_.run_start, frozen->c_str());
    const std::string state = hold_.step(t_now, t_cmd_, t_slam_, faults, liveness_.alive(node), frozen.has_value());
    if (fc_mode_ != guided_mode_ || !enabled_ || fc == "never") {
      hold_point_.reset();
      yaw_out_.reset();
      wp_point_.reset();
      depart_.reset();
      return;   // never stream outside Guided (the executive owns modes), nor during the FC's own takeoff
                // or landing (not enabled), nor without ever having had the vehicle's position
    }
    // stale: the last pose received while fresh (the state is HOLD then: nothing is placed on it but
    // the hold point)
    const FcPose now_pose = fc == "stale" ? *fc_guard_.good : *fc_pose_;
    const Vec3& p_now = now_pose.p;
    const double yaw_now = now_pose.yaw;
    if (state == "STREAM" && target) {
      hold_point_.reset();
      if (mode == "drive") {
        auto rs = reverse_speed(target->p, target->v, target->yaw, p_now);
        if (rs) {
          freeze_.on_command(t_now, *rs);
          send_reverse(*rs, target->yaw);
          return;
        }
        reversing_.reset();
        const Vec3 goal = drive_target(target->p, target->v, p_now, drive_lookahead_);
        freeze_.on_drive_target(t_now, (goal - p_now).head<2>().norm());
        send(goal, nullptr, nullptr, target->yaw, MASK_DRIVE);
      } else if (yaw_cmd_) {
        // the waypoint hold: not SUPER's command, a fixed point (the reached goal, or the vehicle when the
        // hold began), capped like any target, zero velocity, turning to the waypoint yaw
        if (!wp_point_) {
          std::optional<Vec3> goal_fc;
          if (goal_planner_) goal_fc = anchor->pos(*goal_planner_);
          wp_point_ = fence_.clamp(waypoint_hold_point(goal_fc, p_now, wp_goal_max_));
          wp_from_goal_ = goal_fc && (*goal_fc - p_now).norm() <= wp_goal_max_;
          ROS_INFO("planner bridge: waypoint hold at %s (%.2f %.2f %.2f), %.2f m from the vehicle, yaw %.1f deg",
                   wp_from_goal_ ? "the goal" : "the vehicle", (*wp_point_)[0], (*wp_point_)[1], (*wp_point_)[2],
                   (*wp_point_ - p_now).norm(), *yaw_cmd_ * (180.0 / M_PI));
        }
        const Vec3 zero = Vec3::Zero();
        auto c = cap_command(*wp_point_, zero, zero, p_now, speed_cap_, max_lead_, fc_pos_p_);
        send(c.p, &zero, &zero, flight_yaw(target->yaw, std::nullopt, yaw_now, true), MASK_FLIGHT);
        wp_active_ = true;
      } else if (depart_ && !departing_done(*anchor, *target, t_now)) {
        // the departure: hold on, point and yaw, until SUPER sets off toward the next goal
        const Vec3 zero = Vec3::Zero();
        auto c = cap_command(depart_->point, zero, zero, p_now, speed_cap_, max_lead_, fc_pos_p_);
        send(c.p, &zero, &zero, flight_yaw(target->yaw, std::nullopt, yaw_now, true), MASK_FLIGHT);
        wp_active_ = true;
      } else if (turn_slow_ && !ref_.empty()) {
        // the turn slowdown: SUPER's reference replayed at the speed the heading error allows, capped at it
        const Replay r = replay(*anchor, yaw_now);
        auto c = cap_command(r.p, r.v, r.a, p_now, r.cap, max_lead_, fc_pos_p_);
        freeze_.on_command(t_now, fc_demand(c.p, c.v, p_now));
        send(c.p, &c.v, &c.a, flight_yaw(r.yaw, travel_yaw(r.v_ref, yaw_speed_min_), yaw_now), MASK_FLIGHT);
      } else {
        auto c = cap_command(target->p, target->v, target->a, p_now, speed_cap_, max_lead_, fc_pos_p_);
        freeze_.on_command(t_now, fc_demand(c.p, c.v, p_now));
        send(c.p, &c.v, &c.a, flight_yaw(target->yaw, travel_yaw(target->v, yaw_speed_min_), yaw_now), MASK_FLIGHT);
      }
      return;
    }
    if (mode == "drive") {
      if (hold_.dead || hold_.frozen) {
        // the planner died, or the SLAM pose froze: stop where the rover is, now. A zero-speed velocity +
        // heading target (the reverse HOLD's), the heading the rover has at the fault, so it neither drives
        // on to the last (run-out) target nor pivots
        if (!stop_yaw_) stop_yaw_ = yaw_now;
        send_reverse(0.0, *stop_yaw_);
        return;
      }
      // a reverse under way: Rover's velocity + heading target would back up on for 3 s
      if (reversing_) send_reverse(0.0, *reversing_);
      return;   // HOLD on the ground: Rover Guided stops at the last target
    }
    // HOLD (or WAIT) in flight: stay where the vehicle is, zero velocity and acceleration
    // (travel yaw: the heading it was flying, not a new one)
    if (!hold_point_) hold_point_ = FcPose{fence_.clamp(p_now), flight_yaw_ == "travel" && yaw_out_ ? *yaw_out_ : yaw_now};
    const Vec3 zero = Vec3::Zero();
    const double hy = flight_yaw_ == "travel" ? flight_yaw(hold_point_->yaw, std::nullopt, hold_point_->yaw) : hold_point_->yaw;
    send(hold_point_->p, &zero, &zero, hy, MASK_FLIGHT);
  }

  // the departure is over (and cleared): SUPER sets off toward the goal, or ~depart_timeout_s passed
  template <class T>
  bool departing_done(const Anchor& anchor, const T& target, double t_now) {
    std::optional<Vec3> goal_fc;
    if (goal_planner_) goal_fc = anchor.pos(*goal_planner_);
    const bool goal_new = t_goal_ && *t_goal_ > depart_->t0;
    const bool off = departed(depart_->point, goal_fc, goal_new, target.p, target.v, yaw_speed_min_);
    const bool late = t_now - depart_->t0 >= depart_timeout_;
    if (!off && !late) return false;
    if (off)
      ROS_INFO("planner bridge: departure after %.2f s: the planner sets off toward the goal", t_now - depart_->t0);
    else
      ROS_WARN("planner bridge: departure: the planner did not set off toward the goal within %.1f s: streaming",
               depart_timeout_);
    depart_.reset();
    return true;
  }

  // the turn slowdown (owner 2026-10-29): one tick of SUPER's reference, replayed (bridge_core replay_rate)
  // at the speed the heading error allows (bridge_core turn_speed_frac of turn_heading_error: the FC's yaw
  // against SUPER's commanded travel direction at the replay point, now and ~turn_slow_lookahead_s ahead).
  // The allowed speed moves toward speed_cap * frac at ~turn_slow_accel m/s^2 at most, and is the cap the
  // target is then held to (cap_command, with its fc_pos_p lead clamp): the FC's total demand stays within it.
  // Flight streaming only: the waypoint hold, the departure, the holds and drive mode never come here.
  struct Replay {
    Vec3 p, v, a, v_ref;   // fc frame; v, a: as replayed (rate-scaled); v_ref: SUPER's at the replay point
    double yaw, cap;
  };
  Replay replay(const Anchor& anchor, double yaw_now) {
    const double dt = std::max(0.0, std::min(tick_dt_, 2.0 / rate_));
    auto err_at = [&](const RefBuffer::Sample& s) {
      return turn_heading_error(yaw_now, anchor.vec(s.v), anchor.vec(s.a), turn_lookahead_, yaw_speed_min_);
    };
    auto frac_of = [&](std::optional<double> e) {
      return e ? turn_speed_frac(*e, turn_start_, turn_full_, turn_min_frac_) : 1.0;
    };
    double rate = 1.0;
    std::optional<double> err;
    if (!replay_on_) {
      // (re)start at SUPER's newest command, no lag, the allowed speed straight at its value
      tau_ = ref_.newest();
      err = err_at(ref_.at(tau_));
      allowed_ = speed_cap_ * frac_of(err);
      replay_on_ = true;
    } else {
      const RefBuffer::Sample s0 = ref_.at(tau_);
      err = err_at(s0);
      const double want = speed_cap_ * frac_of(err);
      const double dv = turn_accel_ * dt;
      allowed_ = std::min(speed_cap_, std::clamp(want, allowed_ - dv, allowed_ + dv));
      rate = replay_rate(allowed_, s0.v.norm(), ref_.newest() - tau_, turn_catchup_, turn_max_lag_);
      tau_ = std::min(tau_ + rate * dt, ref_.newest());
    }
    ref_.trim(tau_ - 0.1);
    const RefBuffer::Sample s = ref_.at(tau_);
    // the cap: the allowed speed, or the replayed speed where the lag limit makes it faster (else the cap
    // would hold the vehicle back from the replay it follows)
    const double cap = std::min(speed_cap_, std::max(allowed_, rate * s.v.norm()));
    replay_tick_ = true;
    turn_ = TurnState{err, allowed_ / std::max(speed_cap_, 1e-9), cap, rate, ref_.newest() - tau_};
    return Replay{anchor.pos(s.p), anchor.vec(s.v) * rate, anchor.vec(s.a) * (rate * rate), anchor.vec(s.v),
                  anchor.yaw(s.yaw), cap};
  }

  // the horizontal speed a flight target asks of the FC: its feed-forward plus ~fc_pos_p times the lead (a
  // frozen pose keeps the target ahead of the frozen FC position: the cap then trades the feed-forward for
  // the lead, SITL fz5: velocity 0, lead 1 m, the FC still flying 1-1.9 m/s)
  double fc_demand(const Vec3& p, const Vec3& v, const Vec3& p_now) const {
    return (v + fc_pos_p_ * (p - p_now)).head<2>().norm();
  }

  // back up at speed (m/s, >= 0) holding yaw (fc frame, ENU): MAVROS turns the ENU velocity (0, -speed)
  // into NED vx = -speed, which Rover Guided reads as reverse (speed_dir -1); at 0 a zero-speed heading hold
  void send_reverse(double speed, double yaw) {
    reversing_ = yaw;
    const Vec3 v(0.0, -std::abs(speed), 0.0);
    send(Vec3::Zero(), &v, nullptr, yaw, MASK_REVERSE);
  }

  void send(const Vec3& p, const Vec3* v, const Vec3* a, double yaw, uint16_t mask) {
    mavros_msgs::PositionTarget sp;
    sp.header.stamp = stamp_;   // the tick's time: the yaw step was sized on it
    sp.header.frame_id = "map";
    // MAVROS takes ENU values here and converts them to the FC's NED
    sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    sp.type_mask = mask;
    sp.position.x = p[0];
    sp.position.y = p[1];
    sp.position.z = p[2];
    if (v) {
      sp.velocity.x = (*v)[0];
      sp.velocity.y = (*v)[1];
      sp.velocity.z = (*v)[2];
    }
    if (a) {
      sp.acceleration_or_force.x = (*a)[0];
      sp.acceleration_or_force.y = (*a)[1];
      sp.acceleration_or_force.z = (*a)[2];
    }
    sp.yaw = static_cast<float>(yaw);
    pub_sp_.publish(sp);
    n_out_ += 1;
  }

  void publish_status(const ros::TimerEvent&) {
    const double t_now = now();
    auto age = [&](const std::optional<double>& t) -> l4::ojson {
      if (!t) return nullptr;
      return l4::py_round(t_now - *t, 3);
    };
    const bool streaming = fc_mode_ == guided_mode_ && enabled_;
    std_msgs::Int8 code;
    code.data = streaming ? (mode_ == "flight" ? 1 : 2) : 0;
    pub_mode_code_.publish(code);
    const auto node = liveness_.node(mode_);
    const auto alive = liveness_.alive(node);
    bool slam_hold = false;
    if (hold_.state == "HOLD")
      for (const char* r : SLAM_HOLD_REASONS)
        if (starts_with(hold_.reason, r)) slam_hold = true;
    l4::ojson s;
    // the waypoint hold streams (HoldLogic STREAM) but does not follow the planner: its own state
    const bool wp = wp_active_ && hold_.state == "STREAM" && streaming && mode_ == "flight";
    s["state"] = wp ? std::string("WAYPOINT_HOLD") : hold_.state;
    s["reason"] = wp ? std::string(depart_ ? "waiting for the planner to set off" : "waypoint yaw") : hold_.reason;
    s["mode"] = mode_;
    s["fc_mode"] = fc_mode_;
    // the mission executive pauses its leg only for SLAM-side holds (D7), not for a planner that has not
    // planned yet or a geofence stop
    s["slam_hold"] = slam_hold;
    s["enabled"] = enabled_;
    s["streaming"] = streaming;
    s["cmd_age"] = age(t_cmd_);
    s["slam_age"] = age(t_slam_);
    // the mode's planner: its last command's age, its node and whether that process is there
    s["planner_age"] = age(t_cmd_);
    s["planner_node"] = node ? l4::ojson(*node) : l4::ojson(nullptr);
    s["planner_alive"] = alive ? l4::ojson(*alive) : l4::ojson(nullptr);
    s["fc_pose"] = fc_guard_.check(t_now);
    // a repeated SLAM pose (the frozen-pose check): how many in a row, for how long, the evidence now
    if (freeze_on_ && freeze_.repeats > 0 && freeze_.run_start) {
      l4::ojson fz;
      fz["repeats"] = freeze_.repeats;
      fz["for_s"] = l4::py_round(t_now - *freeze_.run_start, 3);
      fz["evidence"] = freeze_.evidence(t_now);
      fz["latched"] = hold_.frozen;
      s["slam_frozen"] = fz;
    } else {
      s["slam_frozen"] = hold_.frozen ? l4::ojson{{"latched", true}} : l4::ojson(nullptr);
    }
    s["fc_pose_age"] = age(fc_guard_.t);
    s["speed_cap"] = speed_cap_;
    const auto& a = tracker_.anchor;
    if (a) {
      l4::ojson an;
      an["psi_deg"] = l4::py_round(a->psi * (180.0 / M_PI), 2);
      an["t"] = {l4::py_round(a->t[0], 3), l4::py_round(a->t[1], 3), l4::py_round(a->t[2], 3)};
      s["anchor"] = an;
    } else {
      s["anchor"] = nullptr;
    }
    s["anchor_jumps"] = tracker_.jumps;
    s["targets_sent"] = n_out_;
    s["flight_yaw"] = flight_yaw_;
    s["yaw_cmd_deg"] = yaw_cmd_ ? l4::ojson(l4::py_round(*yaw_cmd_ * (180.0 / M_PI), 2)) : l4::ojson(nullptr);
    if (wp && (wp_point_ || depart_)) {
      const Vec3 hp = wp_point_ ? *wp_point_ : depart_->point;
      l4::ojson h;
      h["point"] = {l4::py_round(hp[0], 3), l4::py_round(hp[1], 3), l4::py_round(hp[2], 3)};
      h["from"] = wp_from_goal_ ? "goal" : "vehicle";
      h["phase"] = wp_point_ ? "turn" : "depart";
      s["waypoint_hold"] = h;
    } else {
      s["waypoint_hold"] = nullptr;
    }
    // the turn slowdown while it replays SUPER's reference (null otherwise): the heading error it acts on
    // (null: no travel direction), the allowed fraction of the speed cap and speed, the replay rate, the lag
    if (turn_ && streaming && mode_ == "flight") {
      l4::ojson ts;
      ts["err_deg"] = turn_->err ? l4::ojson(l4::py_round(*turn_->err * (180.0 / M_PI), 1)) : l4::ojson(nullptr);
      ts["frac"] = l4::py_round(turn_->frac, 3);
      ts["allowed"] = l4::py_round(turn_->allowed, 3);
      ts["rate"] = l4::py_round(turn_->rate, 3);
      ts["lag_s"] = l4::py_round(turn_->lag, 3);
      s["turn_slow"] = ts;
    } else {
      s["turn_slow"] = nullptr;
    }
    // the local ceiling over the vehicle (m above it; null: none seen over it, or no recent scan)
    const bool ceil_fresh = t_ceiling_ && t_now - *t_ceiling_ <= ceiling_max_age_;
    s["ceiling_above"] = ceil_fresh && ceiling_ ? l4::ojson(l4::py_round(*ceiling_, 3)) : l4::ojson(nullptr);
    std_msgs::String out;
    out.data = l4::dumps(s);
    pub_status_.publish(out);
  }

  struct Cmd {
    Vec3 p, v, a;
    double yaw;
  };

  ros::NodeHandle nh_, pnh_;
  std::string mode_, guided_mode_;
  double rate_ = 50.0, speed_cap_ = 0.5, max_lead_ = 1.0, fc_pos_p_ = 1.0, drive_lookahead_ = 2.5;
  Geofence fence_;
  HoldLogic hold_;
  FcPoseGuard fc_guard_;
  AnchorTracker tracker_;
  HealthCheck health_;
  FrozenPoseCheck freeze_;
  bool health_on_ = false, enabled_ = false, freeze_on_ = true;

  std::optional<double> t_cmd_;
  std::optional<Cmd> cmd_;                              // planner frame
  std::optional<double> t_slam_;
  std::optional<std::pair<Vec3, Quat>> last_planner_;   // (p, q) from the newest /Odometry, planner frame
  std::optional<FcPose> fc_pose_;                       // the newest, whatever its age
  std::string fc_mode_;
  std::optional<FcPose> hold_point_;
  std::optional<double> reversing_;                     // the yaw of a reverse under way (drive mode)
  std::optional<double> stop_yaw_;                      // drive mode, planner dead: the heading the stop holds
  std::string flight_yaw_ = "travel";
  double yaw_speed_min_ = 0.3, yaw_rate_max_ = 60.0 * M_PI / 180.0;
  std::optional<double> yaw_cmd_;                       // the executive's waypoint yaw (fc frame), /planner_bridge/yaw_cmd
  std::optional<double> yaw_out_;                       // the flight yaw last sent (travel mode, a waypoint hold)
  std::optional<Vec3> goal_planner_;                    // the executive's last goal for SUPER (planner frame)
  std::optional<Vec3> wp_point_;                        // the waypoint hold's point (fc), fixed when it begins
  bool wp_from_goal_ = false, wp_active_ = false;       // the point is the goal; the last tick held it
  double wp_goal_max_ = 1.0;
  struct Depart {
    Vec3 point;   // the hold point (fc)
    double t0;    // when the waypoint yaw was cleared
  };
  std::optional<Depart> depart_;                        // after a waypoint hold, until SUPER sets off
  std::optional<double> t_goal_;                        // when the last goal came
  double depart_timeout_ = 5.0;
  // the turn slowdown
  bool turn_slow_ = true;
  double turn_start_ = 0.0, turn_full_ = 0.0, turn_min_frac_ = 0.25, turn_lookahead_ = 0.5, turn_accel_ = 1.5,
         turn_catchup_ = 1.5, turn_max_lag_ = 0.5;
  double ceiling_radius_ = 0.6, ceiling_min_up_ = 0.15, ceiling_max_age_ = 1.0;
  std::optional<double> t_ceiling_, ceiling_;   // the newest scan's time and its ceiling over the vehicle
  RefBuffer ref_;                                       // SUPER's recent commands (planner frame, receive time)
  bool replay_on_ = false, replay_tick_ = false;        // replaying; this tick replayed
  double tau_ = 0.0;                                    // the replay time (receive time of SUPER's commands)
  double allowed_ = 0.0;                                // the speed allowed now, m/s
  struct TurnState {
    std::optional<double> err;
    double frac, allowed, rate, lag;
  };
  std::optional<TurnState> turn_;                       // the last replayed tick's numbers, for the status
  ros::Time stamp_;                                    // this tick's time (the setpoints' stamp)
  std::optional<double> t_tick_;                        // the last tick's time
  double tick_dt_ = 0.02;                               // the time since the tick before
  long long n_out_ = 0;
  PlannerLiveness liveness_;

  ros::Publisher pub_odom_, pub_cloud_, pub_sp_, pub_anchor_, pub_status_, pub_mode_, pub_mode_code_;
  std::vector<ros::Subscriber> subs_;
  ros::Timer tick_timer_, status_timer_;
};

}  // namespace l4_bridge

int main(int argc, char** argv) {
  ros::init(argc, argv, "planner_bridge");
  try {
    l4_bridge::PlannerBridge bridge;
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL("planner bridge: %s", e.what());
    return 1;
  }
  return 0;
}
