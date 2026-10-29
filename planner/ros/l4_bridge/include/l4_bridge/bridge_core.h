// Planner bridge core: frames, the frame anchor, caps, the geofence and the HOLD state machine.
//
// ROS-free (Eigen only) so the unit tests run this exact code; planner_bridge_node.cpp wraps it in roscpp.
// C++ port of the former planner/bridge_core.py (same names, same arithmetic).
//
// Frames (L4 plan, D7):
//   raw      FAST-LIO's world frame: +x left, +y up, +z forward (raw /Odometry, /cloud_registered).
//   planner  raw rotated z-up by POSITION_MATRIX, the transform fastlio_mavros_bridge.py applies
//            before its frame anchor. SUPER and ROG-Map work here.
//   fc       MAVROS local ENU ("map"): the frame of /mavros/vision_pose/pose, i.e. the planner frame
//            after the bridge's frame anchor  fc = Rz(psi) * planner + t  (frame_anchor.Anchor).
// Quaternions are [x, y, z, w]; angles in radians.
#pragma once

#include <optional>
#include <string>
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
class HoldLogic {
 public:
  explicit HoldLogic(double planner_timeout = 0.3, double slam_timeout = 0.2);
  void latch(const std::string& reason, double now);
  void new_leg();
  // faults: reasons from the caller (geofence, SLAM health). planner_alive: the liveness of the mode's
  // planner process (true, false: gone, none: unknown or no planner known). -> state
  std::string step(double now, std::optional<double> t_cmd, std::optional<double> t_slam,
                   const std::vector<std::string>& faults = {}, std::optional<bool> planner_alive = std::nullopt);

  double planner_timeout, slam_timeout;
  std::string state = "WAIT";   // WAIT (nothing yet), STREAM, HOLD
  std::string reason = "no planner command yet";
  std::optional<double> hold_since;
  std::optional<std::string> latched;   // a latched cause (anchor jump) until a fresh command
  bool dead = false;                    // the planner died: HOLD until new_leg()

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

// printf-style formatting into a std::string (the Python "%" reasons)
std::string fmt(const char* f, ...) __attribute__((format(printf, 1, 2)));

}  // namespace l4_bridge
