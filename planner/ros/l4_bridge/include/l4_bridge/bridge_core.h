// Planner bridge core: frames, the frame anchor, caps, the geofence and the HOLD state machine.
//
// ROS-free (Eigen only) so the unit tests run this exact code; planner_bridge_node.cpp wraps it in roscpp.
// C++ port of the former planner/bridge_core.py (same names, same arithmetic).
//
// Frames (L4 plan, D7):
//   raw      FAST-LIO's world frame: +x left, +y up, +z forward (raw /Odometry, /cloud_registered).
//   planner  raw rotated z-up by POSITION_MATRIX, the transform fastlio_mavros_bridge.py applies
//            before its frame anchor. SUPER, ROG-Map, the track planner and the landing check work here.
//            It does NOT jump with a FAST-LIO frame reset (2026-11-02 evening): after one, the bridge keeps
//            publishing the planner frame it had, carried over the reset (FrameCarry below): planner =
//            carry(raw rotated z-up); carry is the identity until the first reset.
//   fc       MAVROS local ENU ("map"): the frame of /mavros/vision_pose/pose, i.e. the planner frame
//            after the bridge's frame anchor  fc = Rz(psi) * planner + t  (frame_anchor.Anchor). Mission
//            goals are in fc (the EKF's frame survives a SLAM reset: the SLAM-to-FC bridge re-anchors onto the
//            EKF); /planner_bridge/anchor is fc <- planner as published, unchanged by a carried reset.
// Quaternions are [x, y, z, w]; angles in radians.
#pragma once

#include <cstdint>
#include <deque>
#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

namespace l4_bridge {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

struct Quat {
  double x = 0, y = 0, z = 0, w = 1;
};

// raw -> planner, the default ~position_matrix / ~orientation_matrix of fastlio_mavros_bridge.py
const Mat3& position_matrix();

double wrap_pi(double a);
Quat quat_mul(const Quat& a, const Quat& b);
Quat quat_conj(const Quat& q);
Quat quat_from_matrix(const Mat3& m);   // rotation matrix -> [x, y, z, w]
double yaw_of(const Quat& q);
Quat quat_yaw(double psi);
const Quat& q_perm();

// ---- raw -> planner (the bridge's transform before its anchor) ----------------------------------
Vec3 raw_to_planner_pos(const Vec3& p);
Quat raw_to_planner_quat(const Quat& q);   // the bridge's similarity transform q_enu = qR * q * qR^-1

// ---- the anchor: fc <- planner ------------------------------------------------------------------
// fc = Rz(psi) * planner + t, the bridge's frame anchor recovered from its output
struct Anchor {
  double psi = 0.0;
  Vec3 t = Vec3::Zero();

  Anchor() = default;
  Anchor(double psi_, const Vec3& t_) : psi(psi_), t(t_) {}

  // the anchor that maps one planner pose onto the bridge's vision pose made from it. mount_yaw: the
  // bridge's ~fc_mount_yaw_deg in radians (a body-side yaw on the orientation only, before the
  // anchor), so psi is not biased by it
  static Anchor from_pair(const Vec3& p_planner, const Quat& q_planner, const Vec3& p_fc, const Quat& q_fc,
                          double mount_yaw = 0.0);
  Vec3 pos(const Vec3& p) const;
  Vec3 vec(const Vec3& v) const;
  double yaw(double yaw) const;
  Vec3 inverse_pos(const Vec3& p) const;
  // the change between two anchors as a reason, or none
  std::optional<std::string> differs(const Anchor& other, double jump_m, double jump_rad) const;
};

// the latest anchor from (planner, vision) pose pairs; a change that persists for `persist` pairs is a
// re-anchor or SLAM reset (one odd pair is a pairing glitch)
class AnchorTracker {
 public:
  AnchorTracker(double jump_m = 0.1, double jump_deg = 2.0, int persist = 3, double mount_yaw = 0.0);
  // -> (anchor, jump reason or none)
  std::pair<Anchor, std::optional<std::string>> update(const Vec3& p_planner, const Quat& q_planner, const Vec3& p_fc,
                                                       const Quat& q_fc);
  // take this anchor now (a SLAM frame reset classified on its first pair; counts as a jump)
  void force(const Anchor& a) {
    anchor = a;
    candidate_.clear();
    jumps += 1;
  }
  std::optional<Anchor> anchor;
  int jumps = 0;
  double jump_m, jump_rad;
  int persist;
  double mount_yaw;

 private:
  std::vector<Anchor> candidate_;
};

// ---- the output target --------------------------------------------------------------------------
struct CapResult {
  Vec3 p, v, a;
  bool capped;
};

// scale velocity (and acceleration with it) to the speed cap, and keep the position target within
// max_lead of the current position so a capped vehicle is not dragged after a target that runs ahead.
// With fc_pos_p (the FC's horizontal position gain, Q_P_NE_POS_P) the cap bounds what the FC actually
// flies: its velocity demand is the feed-forward plus fc_pos_p times the position error. The position
// correction is kept, and the feed-forward is scaled down until |v + fc_pos_p * lead| <= speed_cap.
CapResult cap_command(Vec3 p, Vec3 v, Vec3 a, const Vec3& p_now, double speed_cap, double max_lead,
                      double fc_pos_p = 0.0);

// a position-only driving target at most `lookahead` ahead of the vehicle, towards where the command is
// heading (D4: a dead planner then stops the rover within the lookahead)
Vec3 drive_target(const Vec3& p_cmd, const Vec3& v_cmd, const Vec3& p_now, double lookahead, double horizon_s = 0.5);

// a reverse from the track planner (owner 2026-10-28, dead ends): a command whose velocity points against
// its yaw means "back straight up, holding yaw, until the vehicle (the FC) is at p_cmd". -> the speed to
// back up at (>= 0: |v_cmd| at most, tapering at `gain` per metre left, 0 within stop_tol of the end or
// past it), or none for an ordinary (forward) command.
std::optional<double> reverse_speed(const Vec3& p_cmd, const Vec3& v_cmd, double yaw, const Vec3& p_now,
                                    double gain = 1.0, double stop_tol = 0.02);

struct Geofence {
  Vec3 lo, hi;
  Geofence(const Vec3& lo_, const Vec3& hi_) : lo(lo_), hi(hi_) {}
  bool inside(const Vec3& p) const;
  Vec3 clamp(const Vec3& p) const;
};

// ---- the FC pose's freshness --------------------------------------------------------------------
struct FcPose {
  Vec3 p = Vec3::Zero();
  double yaw = 0.0;
};

// the FC's EKF pose (/mavros/local_position/pose, ~50 Hz on the robot) is the bridge's p_now (cap, drive
// target) and its HOLD point. It is trusted while the newest message is younger than `timeout` (its
// receive time). Stale: the bridge holds at `good`, the last pose received while it was trusted. Fresh
// again: trusted after `resume_n` poses in a row, each within `timeout` of the one before.
// check(now) -> "never" (none yet: stream nothing), "stale" or "ok".
class FcPoseGuard {
 public:
  explicit FcPoseGuard(double timeout = 0.3, int resume_n = 5);
  void on_pose(double now, const FcPose& pose);
  std::optional<double> age(double now) const;
  std::string check(double now);

  double timeout;
  int resume_n;
  std::optional<FcPose> pose;   // the newest pose, whatever its age
  std::optional<double> t;      // its receive time
  std::optional<FcPose> good;   // the last pose received while trusted
  bool stale = false;
  int fresh_n = 0;
};

// ---- a frozen SLAM pose (L2 faults 2b, 2d: FAST-LIO stuck, its last pose repeated with fresh stamps) ----
// The receive-time SLAM check and the FC-pose check both pass on a frozen pose (fresh stamps, and the
// EKF follows the frozen vision pose), and the FC then flies or drives away (f22: 8.4 m/s, 45 m; f24:
// 2.0 m/s, 12.7 m). A real FAST-LIO pose is never repeated frame to frame (4 recordings, ~9200 pairs:
// bench parked with the motors off, flight replays; the smallest step 0.075 mm / 1.5e-5 in a quaternion
// component), so a repeat is the stuck output itself. SITL's pose repeats exactly while the vehicle is
// parked (f2: runs up to 47 s), so a repeat alone is not a fault: it must come with evidence that the
// vehicle moves or is made to move:
//   * command: the bridge streaming a flight target that asks the FC for at least cmd_speed (horizontal,
//     m/s: the caller passes the feed-forward plus the FC's position gain times the lead), or a drive
//     target at least drive_dist ahead of the vehicle (the track planner's carrot carries no velocity:
//     Rover Guided drives toward its position);
//   * gyro: the FC's own IMU (/mavros/imu/data angular velocity, not an EKF output) turning at gyro_min;
//   * twist: the repeated /Odometry carrying a velocity of at least twist_min (a pose that says it moves
//     but does not).
// A pose repeats when it is within eps_m of the one before and every quaternion component within eps_q
// (bit-identical in the faults; 7x / 15x under the smallest real step). Frozen: at least min_n repeats in
// a row, and the evidence active (each kind no older than evidence_age) without a break for hold_s in
// flight, drive_hold_s driving (SITL's rover from rest: its pose repeats until it moves; SITL f2/fz1: a
// flying vehicle's pose never repeats, and every 0.1 s of a frozen pose in flight lets the FC accelerate,
// fz1: 1.06 -> 1.79 m/s in the 1.05 s to the detection).
// A pose that moves ends the run. check() is none while the newest pose is older than max_age (that is
// the SLAM stale hold's). The FC must be alive: once an IMU sample has been seen, no evidence counts (and
// its clock restarts) while the newest is older than evidence_age. SITL stalls under host load stop
// everything, the FC and its MAVLink stream with the render node's pose (fz13: MAVROS silent 0.6 s, the
// pose repeated 11 times with the last twist 0.31 m/s and commands out; f2/f3: repeat runs up to 1.7 s);
// a stuck FAST-LIO leaves the FC running.
extern const char* const SLAM_FROZEN;    // "slam_frozen": the HOLD reason (the executive acts on it)

class FrozenPoseCheck {
 public:
  FrozenPoseCheck(double eps_m = 1e-5, double eps_q = 1e-6, double hold_s = 0.4, int min_n = 4, double cmd_speed = 0.2,
                  double drive_dist = 0.5, double gyro_min = 0.2, double twist_min = 0.2, double evidence_age = 0.25,
                  double max_age = 0.5, double drive_hold_s = 0.8);
  // a raw /Odometry pose (any fixed frame: only the change counts) and its twist, at receive time t
  void on_pose(double t, const Vec3& p, const Quat& q, const Vec3& twist);
  // a streamed flight target's horizontal speed (m/s) at time t
  void on_command(double t, double speed);
  // a streamed drive target's horizontal distance from the vehicle (m) at time t
  void on_drive_target(double t, double dist);
  // the FC IMU's angular rate magnitude (rad/s) at time t
  void on_gyro(double t, double rate);
  // -> the evidence ("command", "gyro", "twist", joined by "+") when frozen, else none. Call it every tick:
  // it runs the evidence clock. driving: drive_hold_s instead of hold_s
  std::optional<std::string> check(double now, bool driving = false);
  // the evidence active now ("" when none)
  std::string evidence(double now) const;

  double eps_m, eps_q, hold_s;
  int min_n;
  double cmd_speed, drive_dist, gyro_min, twist_min, evidence_age, max_age, drive_hold_s;
  int repeats = 0;                     // repeated poses in a row
  std::optional<double> run_start;     // receive time of the pose the run repeats
  std::optional<double> evidence_from; // the evidence active since (within this run)
  std::optional<double> t_pose;        // the newest pose's receive time

 private:
  std::optional<std::pair<Vec3, Quat>> last_;
  double twist_ = 0.0;                 // the newest pose's twist (m/s)
  std::optional<double> t_cmd_move_;   // the last streamed target at cmd_speed or more
  std::optional<double> t_gyro_;       // the last IMU sample
  double gyro_ = 0.0;
};

// ---- HOLD ---------------------------------------------------------------------------------------
extern const char* const PLANNER_DEAD;   // "planner_dead"

// STREAM while every input is fresh and healthy; HOLD (zero velocity at the current position) on the
// first fault. Leaving HOLD needs the cause cleared AND a planner command newer than the moment the
// hold began, so a stale trajectory is never resumed.
//
// A dead planner (owner decision 2026-10-29: the robot just stops, no continuation): the mode's planner
// has sent no command for planner_timeout (or none since the mode changed) AND its process is gone
// (planner_alive false). That latches HOLD with reason PLANNER_DEAD, which no command clears: only
// new_leg() (the executive enabling the bridge for a new leg) does. A planner alive but silent is the
// ordinary, resumable "planner stale" hold; one that never sent a command (planner_alive none) keeps
// the WAIT before the first plan.
//
// A frozen SLAM pose (slam_frozen true, FrozenPoseCheck) latches HOLD with reason SLAM_FROZEN the same
// way, ahead of everything else (the vehicle's position is unknown: the executive lands it in the air and
// stops it on the ground). Only new_leg() clears it, and it latches again at once while the pose is
// still frozen.
class HoldLogic {
 public:
  explicit HoldLogic(double planner_timeout = 0.3, double slam_timeout = 0.2);
  void latch(const std::string& reason, double now);
  void new_leg();
  // faults: reasons from the caller (geofence, SLAM health). planner_alive: the liveness of the mode's
  // planner process (true, false: gone, none: unknown or no planner known). slam_frozen: FrozenPoseCheck
  // says frozen. -> state
  std::string step(double now, std::optional<double> t_cmd, std::optional<double> t_slam,
                   const std::vector<std::string>& faults = {}, std::optional<bool> planner_alive = std::nullopt,
                   bool slam_frozen = false);

  double planner_timeout, slam_timeout;
  std::string state = "WAIT";   // WAIT (nothing yet), STREAM, HOLD
  std::string reason = "no planner command yet";
  std::optional<double> hold_since;
  std::optional<std::string> latched;   // a latched cause (anchor jump) until a fresh command
  bool dead = false;                    // the planner died: HOLD until new_leg()
  bool frozen = false;                  // the SLAM pose froze: HOLD until new_leg()

 private:
  void hold(const std::string& reason, double now);
};

// ---- SLAM health (pluggable, off unless configured and its topic is seen) -----------------------
// FAST-LIO health (/fastlio_health: data[0] matched points, data[4] observability eigenvalue, data[7]
// latency ms) and a height cross-check (SLAM height vs a downward rangefinder, the log 233 case)
class HealthCheck {
 public:
  HealthCheck(double matched_min = 150.0, double eig_min = 5000.0, double latency_max_ms = 250.0,
              double height_tol = 0.5, double max_age = 0.5);
  void on_health(double t, const std::vector<double>& data);
  void on_range(double t, double r, double fc_z);
  std::vector<std::string> faults(double now) const;

  double matched_min, eig_min, latency_max_ms, height_tol, max_age;
  struct Health { double t, matched, eig, latency; };
  struct RangeSample { double t, r, fc_z; };
  std::optional<Health> health;
  std::optional<RangeSample> range;
  std::optional<double> height_offset;   // fc z minus range, learnt when first seen
};

// ---- flight yaw (owner rule 2026-10-29: the heading points along the direction of travel; the robot
// yaws only at a waypoint) ----------------------------------------------------------------------------
// the travel heading of a commanded velocity (fc frame), or none while its horizontal speed is at most
// speed_min (slow, hovering: the caller holds the yaw it has, or the executive's waypoint yaw)
std::optional<double> travel_yaw(const Vec3& v, double speed_min);
// one rate-limited step from `current` toward `desired` (shortest way round), at most max_step rad
double yaw_step(double current, double desired, double max_step);
// the largest yaw step for a tick that came dt s after the one before: rate_max * dt, dt clamped to
// [0, dt_max]. A step per elapsed time, not per tick, keeps the commanded rate at rate_max when the timer
// fires late and catches up (Gate L2 b2: a fixed 1.2 deg step per nominal 20 ms tick read 69 deg/s
// between two setpoints 17 ms apart)
double yaw_max_step(double rate_max, double dt, double dt_max);

// ---- the waypoint hold (Gate L2 b2) -------------------------------------------------------------
// While the executive's waypoint yaw is set (/planner_bridge/yaw_cmd, at the end of a fly leg or over a
// landing spot) the bridge does not follow SUPER: SUPER, hovering at its reached goal, keeps replanning
// round it (0.2-0.4 m/s commanded, its position wandering 0.2 m), and the vehicle never stood still for
// the executive's yaw settle (b2: "yaw did not settle"). It holds a fixed point instead, zero velocity,
// facing the waypoint yaw. The point, fixed when the hold begins: the reached goal (the executive's last
// goal, /planning/click_goal, anchored to the fc frame) when it is within goal_max of the vehicle, else
// (no goal known, or a goal farther than that) the vehicle's position.
// Why the goal and not where the turn began: the executive ends a leg at accept_radius (0.5 m) from the
// goal, often still moving; the goal is the point the mission named (the landing check is asked about
// it, the next drive leg starts from it) and SUPER's reference hovers at it, and SUPER replans the next
// leg from its committed trajectory, not from the odometry, so a hold at the goal starts the next plan
// where the vehicle is (a hold 0.5 m short would make the next plan start 0.5 m away from it).
Vec3 waypoint_hold_point(const std::optional<Vec3>& goal_fc, const Vec3& p_now, double goal_max);

// The waypoint hold's point, clearance-checked (L2 5a, SITL lf16, 2026-11-02: a pillar that appeared 0.19 m from
// the leg's goal; SUPER shifted its goal away from it and stopped clear, but the hold flew to the executive's raw
// goal and the FC centre came to 0.08 m from the pillar). In order, the first candidate within goal_max of the
// vehicle that `clear` accepts: the executive's goal ("goal"), the end of SUPER's committed trajectory, its
// reached or shifted goal ("planner_goal"); else the vehicle ("vehicle"). clear: none checks nothing (the old
// waypoint_hold_point). The pillar fix (2026-11-03): the goal's short point (bridge_core goal_short; none: left out)
// comes after the goal and before SUPER's end ("goal_short"; SITL pf1: SUPER's shifted end, 0.44 m clear in the
// planner frame, was 0.21 m from the real pillar and the move there from the short point passed it at 0.04 m).
struct HoldPoint {
  Vec3 p;
  std::string from;   // "goal" | "goal_short" | "planner_goal" | "vehicle"
};
HoldPoint waypoint_hold_point(const std::optional<Vec3>& goal_fc, const std::optional<Vec3>& planner_goal_fc,
                              const Vec3& p_now, double goal_max, const std::function<bool(const Vec3&)>& clear,
                              const std::optional<Vec3>& goal_short_fc = std::nullopt);

// The point nearest `from` on the segment from -> to (every step m, `to` last) that `clear` accepts; none: none is.
// The hold short of a goal that is not clear (L2 5a pillar fix, 2026-11-03: the pillar 0.09 m from leg 3's goal,
// SUPER's own end beside it too): the nearest clear point on the way back to the vehicle.
std::optional<Vec3> first_clear_toward(const Vec3& from, const Vec3& to, double step,
                                       const std::function<bool(const Vec3&)>& clear);

// ---- obstacles on the path ahead (L2 5a, SITL f25r, 2026-11-02) ---------------------------------------------
// SUPER committed a trajectory 0.01 s before a pillar reached its map; every replan after that failed, and it
// played the committed trajectory straight through the pillar (it never checks that trajectory against the map:
// its EMER_STOP is never reached), with the bridge streaming it. The bridge's own check, defence in depth:

// The occupied voxels of the latest `scans` scans (/planner/cloud, planner frame), each scan's points farther
// than self_r from the vehicle at that scan only (the robot's own body; FAST-LIO's blind is 0.4 m on the robot).
class ObstacleVoxels {
 public:
  explicit ObstacleVoxels(double voxel = 0.1, int scans = 3, double self_r = 0.4);
  // a scan at time t, the vehicle (planner frame) then (none: no self filter); add() its points; end_scan()
  void begin_scan(double t, const std::optional<Vec3>& vehicle);
  void add(double x, double y, double z);
  void end_scan();
  // the occupied voxels whose centre lies within r of p
  int count_within(const Vec3& p, double r) const;
  bool has(int64_t k) const { return occ_.count(k) > 0; }
  std::optional<double> newest() const { return scans_.empty() ? std::nullopt : std::optional<double>(scans_.back().first); }
  size_t size() const { return occ_.size(); }
  double voxel, self_r;
  int scans;
  static int64_t key(int64_t i, int64_t j, int64_t k);

 private:
  std::deque<std::pair<double, std::vector<int64_t>>> scans_;   // each kept scan's voxels
  std::unordered_map<int64_t, int> occ_;                         // voxel -> kept scans with it
  std::vector<int64_t> cur_;
  std::optional<Vec3> veh_;
  double t_cur_ = 0.0;
  bool open_ = false;
};

// The voxels (size `voxel`, keyed as ObstacleVoxels::key) that `occupied` accepts whose centre lies within the
// upright ellipsoid round p: horizontal radius rh, vertical half-axis rz (rz = rh: a sphere).
int count_voxels_near(const Vec3& p, double rh, double rz, double voxel, const std::function<bool(int64_t)>& occupied);

// The leg's memory (L2 5a pillar fix, 2026-11-03). The JT128 sees a forward dome only and FAST-LIO drops returns within
// 0.4 m (its blind): an obstacle beside, behind or right at the vehicle is in none of the latest scans. SITL fx16c /
// fx16dr: a pillar 0.09 m from leg 3's goal; SUPER curled round it and flew back onto the goal toward it at 0.9 m/s, the
// path check (3 scans, self filter 0.4 m) and the waypoint hold's (10 scans) no longer saw it, the hold point was the
// goal, the FC centre reached the pillar (0.00 m). The memory keeps every return within `radius` of the leg's goal (the
// executive's, planner frame) since the goal was sent, less the robot's own (within self_r of the vehicle at that
// scan); a voxel counts once `min_hits` scans had it (one stray scan does not). It is cleared on a new goal (reset).
// Static world assumed for one leg: a person who walked through the goal's surroundings stays in it until the leg ends.
class LegMemory {
 public:
  explicit LegMemory(double voxel = 0.1, double radius = 2.5, double self_r = 0.4, int min_hits = 2);
  // the leg's goal (planner frame), the memory emptied; none: off (nothing kept)
  void reset(const std::optional<Vec3>& centre);
  void begin_scan(const std::optional<Vec3>& vehicle);
  void add(double x, double y, double z);
  void end_scan();
  bool has(int64_t k) const;
  int count_within(const Vec3& p, double r) const;
  size_t size() const;   // the voxels that count
  size_t scans() const { return n_scans_; }
  const std::optional<Vec3>& centre() const { return centre_; }
  double voxel, radius, self_r;
  int min_hits;

 private:
  std::optional<Vec3> centre_;
  std::unordered_map<int64_t, int> hits_;   // voxel -> scans with it
  std::vector<int64_t> cur_;
  std::optional<Vec3> veh_;
  bool open_ = false;
  size_t n_scans_ = 0, n_counted_ = 0;
};

// A goal's clearance in the scans and the leg's memory (L2 5a pillar fix): fewer than min_voxels occupied voxels
// inside the upright ellipsoid round it (horizontal rh: SUPER's robot_r 0.44; vertical rz: the frame's height, so a
// hover 0.4-0.6 m over a roof or a landing spot stays clear, HP container R low). `count(p, rh, rz)` counts them.
using VoxelCount = std::function<int(const Vec3&, double, double)>;

// The goal approach (L2 5a pillar fix, SITL fx16dr): a goal that is not clear (min_voxels inside the rh x rz
// ellipsoid) is not flown onto. The vehicle comes in at up to the speed cap behind SUPER's reference and overshoots it
// when it stops (fx16dr: 0.9 m/s at the accept radius, 0.4 m past the goal into the pillar), and the scans may sit
// 0.15-0.2 m off the real obstacle (SITL's SLAM). Within approach_dist of the goal the bridge stops streaming SUPER
// and flies (capped, zero velocity) to the goal's short point: the point nearest the goal (rings every `step` m out to
// short_max, 16 directions and the one toward the vehicle, at the goal's height; the executive's accept radius: the leg
// still ends there) that is clear by the larger short_r x rz, with the line from the vehicle to it clear by line_r
// (path_block_r; beyond line_skip); of a ring's candidates the one nearest the vehicle. None (the goal clear, no such
// point): SUPER is streamed as before.
struct GoalShort {
  std::optional<Vec3> point;   // the short point, when the approach is to be held there
  bool goal_clear = true;
  std::string why;             // "goal clear" | "far" | "no clear point within short_max" | "short"
};
GoalShort goal_short(const Vec3& goal, const Vec3& vehicle, double approach_dist, double short_max, double rh,
                     double short_r, double rz, int min_voxels, double line_r, double line_skip, double step,
                     const VoxelCount& count);
// the line from a to b clear: fewer than min_voxels within r of every point on it (every step m) farther than skip from a
bool line_clear(const Vec3& a, const Vec3& b, double r, int min_voxels, double skip, double step, const VoxelCount& count);

// SUPER's committed position trajectory (quadrotor_msgs/PolynomialTrajectory on /planning_cmd/poly_traj, which
// SUPER publishes with each successful plan): pieces of `order`, coefficients highest power first (SUPER's
// Piece::getCoeffMat rows), the start time t0 (s, ROS time, SUPER's clock).
struct PolyTraj {
  double t0 = 0.0;
  int order = 7;
  std::vector<double> dur;
  std::vector<double> cx, cy, cz;   // (order + 1) per piece
  bool empty() const { return dur.empty(); }
  double duration() const;
  Vec3 pos(double t) const;         // at absolute time t, clamped to the trajectory
  Vec3 end() const { return pos(t0 + duration()); }
};

// The positions the vehicle is about to be sent toward: SUPER's committed trajectory from t_from to t_to
// (absolute, every dt; none: left out) and the straight segment from the vehicle to the target being streamed
// (every step m), less those within skip of the vehicle: the points round where the vehicle already is (just
// beyond the self filter) are not a path ahead to stop for (Gate L2 oxg3: a single tick, 2 voxels at the vehicle,
// held a tight pass 4.2 s).
std::vector<Vec3> path_samples(const std::optional<PolyTraj>& traj, double t_from, double t_to, double dt,
                               const Vec3& vehicle, const Vec3& target, double step, double skip = 0.0);

// HOLD "path_blocked" (latched): the first sample with at least min_voxels occupied voxels within r is blocked.
// It clears when the path checks clear for release_s without a break and, once SUPER's trajectories are seen,
// on a trajectory received after the block began (SUPER's new plan; the blocked one is never resumed).
class PathGuard {
 public:
  explicit PathGuard(double r = 0.30, int min_voxels = 2, double release_s = 0.3);
  // -> blocked. traj_rcv: when the trajectory in use arrived (none: no trajectory seen)
  bool update(double now, const ObstacleVoxels& obs, const std::vector<Vec3>& samples, std::optional<double> traj_rcv);
  // the same against any voxel count (count(p, r): occupied voxels within r of p; the scans and the leg's memory)
  bool update(double now, const std::function<int(const Vec3&, double)>& count, const std::vector<Vec3>& samples,
              std::optional<double> traj_rcv);
  void reset();
  double r, release_s;
  int min_voxels;
  bool blocked = false;
  std::optional<double> since;      // the block began
  std::optional<Vec3> at;           // the first blocked sample (planner frame), from the last check that found one
  int voxels = 0;                   // occupied voxels within r of it
  std::optional<double> clear_since;
};

// The departure from a waypoint hold (Gate L2 b5). When the executive clears the waypoint yaw and sends
// the next goal, SUPER's command is first still its hover round the old goal, then a new plan that
// starts from SUPER's own committed trajectory, which kept wandering while the vehicle held (b5: SUPER
// commanded 0.6 m/s backwards for 2 s at the start of leg 2, its reference 0.3 m ahead of the held vehicle
// then 0.5 m behind it; the travel yaw turned the nose after it). So the bridge keeps holding (point and
// yaw) until SUPER sets off toward the goal: its commanded horizontal velocity along the direction from
// the hold point to the goal above speed_min, and its commanded position no more than behind_tol behind
// the hold point along it, so the stream resumes with SUPER's reference at the vehicle. A goal within
// min_dist of the point gives no direction: departed once a goal came after the clear (goal_new; the old
// goal is the hold point). No goal known at all: departed (nothing to wait for). The caller streams
// anyway after a timeout.
bool departed(const Vec3& point, const std::optional<Vec3>& goal_fc, bool goal_new, const Vec3& p_cmd, const Vec3& v_cmd,
              double speed_min, double behind_tol = 0.05, double min_dist = 0.2);

// ---- the turn slowdown (owner 2026-10-29: the lidar faces forward, so the robot must not fly faster than its
// nose can turn toward the direction of travel) ---------------------------------------------------------
// The fraction of the speed cap allowed at a heading error `err` (rad, >= 0): 1 up to `start`, `min_frac`
// from `full` on, a smoothstep between (no corner in the speed as the error grows).
double turn_speed_frac(double err, double start, double full, double min_frac);

// The heading error the slowdown acts on (rad, >= 0): the vehicle's yaw against the commanded travel direction
// (the horizontal heading of v), and against where that direction is going (v + a * lookahead: SUPER's
// commanded acceleration turns the velocity before the corner, so the slowdown starts before the error
// builds), the larger of the two. None while the commanded horizontal speed is at most speed_min (no travel
// direction: hovering, arriving; the ahead term is left out when v + a * lookahead is that slow).
std::optional<double> turn_heading_error(double yaw, const Vec3& v, const Vec3& a, double lookahead, double speed_min);

// The rate the bridge replays SUPER's reference at (1: SUPER's own pace). SUPER cannot be slowed (its
// max_vel is fixed), so a slowed vehicle falls behind SUPER's reference; chasing the newest reference (as the
// speed cap alone does) would cut the corner on the straight line to it, away from SUPER's collision-checked
// path. The bridge instead plays back SUPER's own commands (RefBuffer) at this rate: the vehicle stays on the
// path SUPER flew, only later. allowed: the speed allowed now (m/s); speed: SUPER's commanded speed at the
// replay point; lag: how far (s) the replay is behind SUPER's newest command. Behind: catch up at up to
// `catchup` as the allowed speed permits; near max_lag never much slower than SUPER: the rate's floor rises
// linearly from 0 at max_lag - REPLAY_LAG_BAND (0.5 s) to 1 (SUPER's pace) at max_lag, so the lag levels off
// there. Within REPLAY_LAG_MIN (0.05 s) of the newest command the replay counts as at it (SUPER's pace).
constexpr double REPLAY_LAG_MIN = 0.05;
constexpr double REPLAY_LAG_BAND = 0.5;
double replay_rate(double allowed, double speed, double lag, double catchup, double max_lag);

// SUPER's recent commands by receive time (planner frame), for the replay. at(t): the command at time t,
// interpolated (p, v, a linearly; yaw: the earlier sample's), clamped to the oldest/newest sample.
class RefBuffer {
 public:
  struct Sample {
    double t;
    Vec3 p, v, a;
    double yaw;
  };
  void push(const Sample& s);   // a sample older than the newest is dropped (out of order)
  void clear() { s_.clear(); }
  bool empty() const { return s_.empty(); }
  size_t size() const { return s_.size(); }
  double newest() const { return s_.back().t; }
  const Sample& last() const { return s_.back(); }
  double oldest() const { return s_.front().t; }
  Sample at(double t) const;
  void trim(double before);   // drop samples older than `before`, keeping the last one at or before it
 private:
  std::vector<Sample> s_;
};

// ---- the local ceiling (container fly-through k1, 2026-10-29) -------------------------------------------
// The lowest cloud point over the vehicle: the points within `radius` of it horizontally and more than
// `min_up` above it (the vehicle's own body, the level it flies at), planner frame. above(): from the vehicle
// up to that point (m), none when no point is over it. The bridge reports it in its status ("ceiling_above"),
// and the executive keeps SUPER's re-seed goal SUPER's inflation below it.
class CeilingFinder {
 public:
  CeilingFinder(const Vec3& at, double radius, double min_up) : at_(at), r2_(radius * radius), min_up_(min_up) {}
  void add(double x, double y, double z) {
    const double up = z - at_[2];
    if (!(up > min_up_)) return;   // also NaN
    const double dx = x - at_[0], dy = y - at_[1];
    if (!(dx * dx + dy * dy <= r2_)) return;   // also NaN
    if (!best_ || up < *best_) best_ = up;
  }
  std::optional<double> above() const { return best_; }

 private:
  Vec3 at_;
  double r2_, min_up_;
  std::optional<double> best_;
};

// ---- the surface under the vehicle (landing check height, owner 2026-11-02) -----------------------------
// The nearest surface below the vehicle: the points within `radius` of it horizontally and more than
// `min_down` below it (the vehicle's own body), planner frame. below(): from the vehicle down to that surface
// (m): the pct-th percentile of the points' depths (the nearest surface, not one stray return), none with
// fewer than min_points such points. The bridge reports it in its status ("floor_below"); the executive climbs
// to its landing check height above it.
class FloorFinder {
 public:
  FloorFinder(const Vec3& at, double radius, double min_down, double pct = 10.0, int min_points = 10)
      : at_(at), r2_(radius * radius), min_down_(min_down), pct_(pct), min_points_(min_points) {}
  void add(double x, double y, double z) {
    const double down = at_[2] - z;
    if (!(down > min_down_)) return;   // also NaN
    const double dx = x - at_[0], dy = y - at_[1];
    if (!(dx * dx + dy * dy <= r2_)) return;   // also NaN
    d_.push_back(down);
  }
  std::optional<double> below() const;

 private:
  Vec3 at_;
  double r2_, min_down_, pct_;
  int min_points_;
  std::vector<double> d_;
};

// ---- a SLAM frame reset (L2 3a / 3b, 2026-11-02 evening) ---------------------------------------------------
// A FAST-LIO restart (laserMapping restarted: a new origin at the lidar, level, at the current heading) or a
// pose jump changes the raw frame. The SLAM-to-FC bridge re-anchors onto the EKF, so the FC's frame carries on;
// but SUPER's ROG-map and committed trajectory (and the track planner's and landing check's scan memory) are in
// the planner frame, and a planner frame that jumps with the raw frame left them in the old one (SITL lf12: the
// old map's floor slab at the vehicle's new height, SUPER climbed over it, 0.96 -> 3.14 m, 1.45 m/s; lf11 the
// same after a 2 s gap). SUPER has no map reset (only its click goal topic; ROG-map clears only when the pose
// leaves its local map). So the bridge carries the planner frame over the reset: it keeps publishing the planner
// frame it had (FrameCarry, a yaw + offset from the new raw-derived frame to the old one), and SUPER's map and
// trajectory stay valid (up to the EKF's dead-reckoning over the gap).
//
// Anchor algebra (fc = A(p) = Rz(psi) p + t).
Anchor anchor_compose(const Anchor& a, const Anchor& b);   // a after b: p -> a(b(p))
Anchor anchor_inverse(const Anchor& a);

// Which change is it? The raw pose came after a gap (gap_s since the one before) or with a step from it. Under
// "the same raw frame" (FAST-LIO stalled and resumed: the SLAM-to-FC bridge re-anchors onto the EKF, whose drift
// over the gap the fc frame then carries, L2 2a), the anchor in use maps the new raw pose onto the vision pose to
// within that drift. Under "a new raw frame" it misses by as much as the frames differ. residual_m / residual_deg:
// |A_old(p_new) - p_vision| and the yaw difference. A reset when residual_m > max(min_m, rate * gap_s) or
// residual_deg > min_deg. Near the old origin, at the old heading, the two are alike, and so are the frames.
struct FrameChange {
  bool reset = false;
  double residual_m = 0.0, residual_deg = 0.0;
};
FrameChange classify_frame_change(const Anchor& old_anchor, const Vec3& p_raw_planner, const Quat& q_raw_planner,
                                  const Vec3& p_vision, const Quat& q_vision, double gap_s, double min_m, double rate,
                                  double min_deg, double mount_yaw = 0.0);

// The planner frame carried over SLAM frame resets: published = carry(raw-derived planner frame). Identity until
// the first reset. On a reset with the anchor before it (raw-derived, A_old) and the one the new raw frame pairs
// with (A_new): carry = A_pub^-1 after A_new, A_pub = A_old after carry^-1 (the published frame's anchor) kept.
class FrameCarry {
 public:
  void reset_to(const Anchor& a_old_raw, const Anchor& a_new_raw);
  Vec3 pos(const Vec3& p) const { return c_.pos(p); }
  Vec3 vec(const Vec3& v) const { return c_.vec(v); }
  Quat quat(const Quat& q) const;
  // the published frame's anchor (fc <- published planner) for a raw-derived one
  Anchor published(const Anchor& a_raw) const { return anchor_compose(a_raw, anchor_inverse(c_)); }
  bool active() const { return resets > 0; }
  const Anchor& carry() const { return c_; }
  int resets = 0;

 private:
  Anchor c_;   // raw-derived planner -> published planner
};

// After a re-anchor in flight the bridge holds until SUPER has planned in the frame now in use (L2 3a, 3b): a
// committed trajectory that started after the re-anchor (start_WT, SUPER's clock: the bridge's ROS time; less
// `slack` for SUPER planning from odometry received just before) and from the vehicle (its start within
// `start_tol` of the vehicle, planner frame; a replan spliced onto the old committed trajectory starts where that
// one was).
bool plan_after(const std::optional<PolyTraj>& traj, double t_reanchor, const Vec3& vehicle, double start_tol,
                double slack = 0.05);

// Defence in depth on a flight target's height (L2 3b: targets in the old frame climbed 2.2 m): the target's z
// moves at most vz_max * dt from the last one sent (none: no limit), lies within z_lead of the vehicle's, and the
// vertical feed-forward is at most vz_max (acceleration zeroed where it is clipped). -> true when it clipped.
bool clamp_vertical(Vec3& p, Vec3& v, Vec3& a, const Vec3& p_now, const std::optional<double>& z_last, double dt,
                    double vz_max, double z_lead);

// printf-style formatting into a std::string (the Python "%" reasons)
std::string fmt(const char* f, ...) __attribute__((format(printf, 1, 2)));

}  // namespace l4_bridge
