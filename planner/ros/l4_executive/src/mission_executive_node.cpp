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
//            in drive mode (Rover Guided position targets); only after a land leg, or as the first legs
//            of a mission that starts on the ground. Done when the FC is within accept_radius or the
//            track planner reports REACHED for this goal
//   land     at [x, y]: fly there, ask the landing check (/planner/landing_spot) and require it to report
//            the spot clear within ~landing_check_s (a not-clear answer is asked again until then: a
//            person walking off clears it), then NAV_LAND (lands and drives on in GUIDED)
//   takeoff  height above the liftoff point (NAV_TAKEOFF in GUIDED). The executive first waits for the
//            vehicle to stand still (< 2 cm over 1 s) for up to ~takeoff_settle_s (15 s). Not still by
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
// metres), which survives SLAM restarts (D7). SUPER plans in "planner"; the planner bridge publishes the
// fc<-planner transform on /planner_bridge/anchor and its state on /planner_bridge/status (JSON). Every
// goal is converted fc->planner with the latest anchor.
//
// Failures:
//   * no progress (not 0.2 m closer within no_progress_s; on a drive leg also not ~drive_progress_m
//     driven) or leg timeout (goal unreachable) -> QRLOITER (in the air), hold, then the next leg
//     (~on_unreachable: next) or stop there (stop)
//   * the planner died (owner decision 2026-10-29): the bridge reports HOLD "planner_dead" -> the mission
//     stops at once, STOPPED "planner died", whatever ~on_unreachable says. In the air: QRLOITER. On the
//     ground: GUIDED, bridge off (its last target, the zero-speed stop, stays Rover's)
//   * bridge HOLD for a SLAM reset or an anchor jump -> stop sending goals, wait for the bridge to report
//     OK plus ~map_rebuild_s, re-convert the goal with the new anchor and resend it (D7)
//   * on the ground (driving) nothing is held in QRLOITER: entering a QR mode armed on the ground starts
//     QuadRover's drive-to-fly transition (props spool up). There an unreachable drive goal, a refused
//     takeoff or a stop leaves the vehicle in GUIDED with the bridge disabled
//   * the FC leaves GUIDED on its own (failsafe, the pilot) or MAVROS loses the FC -> stop commanding for
//     good; the FC's failsafe owns the vehicle
//   * landing spot not reported clear within ~landing_check_s -> QRLOITER, mission stopped
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
#include <mavros_msgs/State.h>
#include <mavros_msgs/StatusText.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include <l4_common/params.h>
#include <l4_common/pyjson.h>

#include "l4_executive/mission.h"

namespace l4_executive {

using Vec3 = Eigen::Vector3d;
using Why = std::optional<std::string>;

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
};

class MissionExecutive {
 public:
  MissionExecutive() : pnh_("~") {
    std::string path;
    if (!pnh_.getParam("mission", path)) throw std::invalid_argument("~mission is required");
    mission_ = load_mission(path);
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
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/planning/click_goal", 1, true);
    drive_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/planning/drive_goal", 1, true);
    leg_pub_ = nh_.advertise<std_msgs::String>("/mission_executive/leg", 1, true);
    spot_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/planner/landing_spot", 1);
    status_pub_ = nh_.advertise<std_msgs::String>("/mission_executive/status", 10, true);
    enable_pub_ = nh_.advertise<std_msgs::Bool>("/planner_bridge/enable", 1, true);
    publish_enable(false);
    mode_pub_ = nh_.advertise<std_msgs::String>("/planner_bridge/set_mode", 1, true);
    ground_pub_ = nh_.advertise<std_msgs::String>("/mission_executive/ground_spots", 1, true);
    yaw_pub_ = nh_.advertise<std_msgs::Float64>("/planner_bridge/yaw_cmd", 1, true);
    publish_yaw(std::nullopt);
    start_srv_ = nh_.advertiseService("/mission_executive/start", &MissionExecutive::on_start, this);
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
  }

  void on_pose(const geometry_msgs::PoseStamped::ConstPtr& m) {
    const auto& p = m->pose.position;
    const auto& q = m->pose.orientation;
    std::lock_guard<std::mutex> g(lock_);
    pose_ = Vec3(p.x, p.y, p.z);
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

  void on_track_status(const std_msgs::String::ConstPtr& m) {
    l4::ojson s = l4::loads(m->data);
    if (s.is_discarded()) return;
    std::lock_guard<std::mutex> g(lock_);
    track_ = s;
    if (l4::get(s, "state") == "DRIVING" && l4::get(s, "reason") == "new goal") track_goals_ += 1;
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
    {
      std::lock_guard<std::mutex> g(lock_);
      st = state_;
    }
    if (!st || !st->connected || !st->armed) return std::string("FC link lost or disarmed");
    const auto m = mode_number(st->mode);
    if (our_mode_ && m != modes().at(*our_mode_)) return "FC mode changed to " + st->mode;
    return std::nullopt;
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
      {
        std::lock_guard<std::mutex> g(lock_);
        got = l4::get(bridge_, "mode", name);
      }
      if (got == name) return true;
      sleep(0.05);
    }
    return false;
  }

  // wait until the vehicle has stopped (< 2 cm over 1 s: QuadRover refuses a takeoff above 0.2 m/s, and
  // Rover Guided pauses and moves on again while it settles on its last target); false after timeout
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
          if (t - s.first <= 1.5) keep.push_back(s);
        keep.emplace_back(t, *p);
        seen.swap(keep);
        double maxd = 0.0;
        for (const auto& s : seen)
          if (t - s.first <= 1.0) maxd = std::max(maxd, (s.second.head<2>() - p->head<2>()).norm());
        if (t - seen.front().first >= 1.0 && maxd < 0.02) return true;
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
  Why waypoint(int idx, std::optional<double> yaw, const std::string& why);
  Why scan(const l4::ojson& leg, int idx);
  void stop(const std::string& why) {
    bridge_enable(false);
    report({{"phase", "STOPPED"}, {"reason", why}});
  }

  // the waypoint yaw for the bridge (fc frame, rad), or none (NaN: the bridge holds the yaw it has)
  void publish_yaw(std::optional<double> yaw) {
    std_msgs::Float64 f;
    f.data = yaw ? *yaw : std::numeric_limits<double>::quiet_NaN();
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
    const SeedAltitude sa = seed_altitude(start, target, *here, seed_dz_, ceiling, seed_ceiling_clear_);
    const Vec3 seed((*here)[0], (*here)[1], sa.z);
    const Vec3 planner = publish_goal(seed);
    report({{"phase", "PLANNER_SEED"}, {"leg", idx}, {"why", why}, {"seed_fc", vec_json(seed)}, {"seed_planner", vec_json(planner)},
            {"plan_z", l4::py_round(sa.plan_z, 3)}, {"ceiling_above", ceiling ? l4::ojson(*ceiling) : l4::ojson(nullptr)},
            {"capped", sa.capped}});
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

  // a leg's deliberate waypoint yaw ("yaw", deg, fc frame)
  static std::optional<double> leg_yaw(const l4::ojson& leg) {
    if (!leg.contains("yaw")) return std::nullopt;
    return wrap_pi(num(leg["yaw"]) * (M_PI / 180.0));
  }

  ros::NodeHandle nh_, pnh_;
  Mission mission_;
  std::string on_unreachable_, planner_frame_;
  double unreachable_hold_s_, map_rebuild_s_, landing_check_s_, takeoff_timeout_s_, land_timeout_s_, goal_resend_s_,
      bridge_mode_timeout_s_, takeoff_settle_s_, drive_progress_m_, goal_settle_s_;
  double yaw_settle_s_, yaw_settle_rate_, yaw_settle_speed_, yaw_tol_, waypoint_min_dist_, scan_rate_;
  double seed_dz_, seed_s_, stuck_s_, seed_ceiling_clear_;
  int stuck_max_;
  ros::Publisher yaw_pub_;

  std::mutex lock_;   // the inputs, written by the callbacks
  std::optional<mavros_msgs::State> state_;
  std::optional<Vec3> pose_;
  std::optional<double> yaw_;   // the FC pose's yaw (fc frame)
  l4::ojson bridge_;
  double bridge_ok_since_ = 0.0;
  ExecAnchor anchor_;
  std::vector<std::string> texts_;
  std::optional<bool> landing_clear_;
  l4::ojson landing_status_ = l4::ojson::object();
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
  ros::Publisher goal_pub_, drive_pub_, leg_pub_, spot_pub_, status_pub_, enable_pub_, mode_pub_, ground_pub_;
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
  if (!drive && flying_ && !streaming_) {
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
  report({{"phase", "LEG"}, {"leg", idx}, {"type", kind}, {"bridge_mode", mode}, {"goal_fc", vec_json(target)},
          {"goal_planner", vec_json(planner)}});
  double t0 = now(), last_progress_t = t0;
  std::optional<double> best;
  std::optional<Vec3> moved_from;   // drive legs: driving on counts as progress (a detour round an obstacle
  double last_send = t0;            // first takes the vehicle no closer to the goal)
  bool held = false;
  int reseeds = 0;   // SUPER re-seeded after falling silent on this leg
  const double accept_radius = num(leg["accept_radius"]), no_progress_s = num(leg["no_progress_s"]),
               leg_timeout_s = num(leg["leg_timeout_s"]);
  while (!shutdown()) {
    if (auto lost = fc_lost()) return lost;
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
        report({{"phase", "HOLD_SLAM"}, {"leg", idx}, {"reason", l4::get(bridge, "reason", "bridge hold")}});
      }
      last_progress_t = t;
      t0 += 0.1;   // the leg clock does not run while the bridge holds
    } else if (held) {
      if (t - ok_since >= map_rebuild_s_) {
        held = false;
        planner = publish_goal(target, drive);   // re-validated with the new anchor
        last_send = t;
        report({{"phase", "LEG"}, {"leg", idx}, {"type", kind}, {"resumed", true}, {"goal_planner", vec_json(planner)}});
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
  {
    std::lock_guard<std::mutex> g(lock_);
    landing_clear_.reset();
    landing_status_ = l4::ojson::object();
    texts_.clear();
  }
  // ask the landing check about this spot (planner frame; z = the height we descend from)
  geometry_msgs::PointStamped spot;
  spot.header.frame_id = planner_frame_;
  Vec3 p;
  {
    std::lock_guard<std::mutex> g(lock_);
    const double z = pose_ ? (*pose_)[2] : 0.0;
    p = anchor_.fc_to_planner(Vec3(num(leg["at"][0]), num(leg["at"][1]), z));
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
  while (now() - t0 < landing_check_s_) {
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
    if (clear && *clear) break;   // reported clear: land; not clear (yet): keep asking until landing_check_s
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
    return std::string("landing spot not reported clear");
  }
  bridge_enable(false);
  report({{"phase", "LAND"}, {"leg", idx}, {"check", why}, {"check_s", l4::py_round(now() - t0, 2)}});
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
      if (flying_) {
        why = std::string("drive leg while flying: the mission needs a land leg first");
      } else {
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
        if (!why) why = land(leg, idx);
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
    if (w == "landing spot not reported clear" || w == "yaw did not settle") {
      set_mode("QRLOITER");
      return stop(w);
    }
    if (w == PLANNER_DIED) {
      // owner 2026-10-29: just stop, never the next leg. In the air QRLOITER (the bridge has held since
      // the death; set_mode turns it off just before the mode command, as on every QRLOITER); on the
      // ground GUIDED with the bridge off (a QR mode there would spool the props)
      if (flying_) set_mode("QRLOITER");
      return stop(w);
    }
    if (!flying_) {
      // on the ground: an unreachable drive goal, a mission error. Hold in GUIDED, bridge off
      bridge_enable(false);
      report({{"phase", "UNREACHABLE"}, {"leg", idx}, {"reason", w}, {"on_ground", true}});
      if (starts_with(w, "bridge") || starts_with(w, "land leg") || starts_with(w, "drive leg") || starts_with(w, "scan leg"))
        return stop(w);
      sleep(unreachable_hold_s_);
      if (policy != "next" || fc_lost()) return stop(w);
      continue;
    }
    // in the air: unreachable goal, refused landing, a bridge that did not switch: QRLoiter
    report({{"phase", "UNREACHABLE"}, {"leg", idx}, {"reason", w}});
    set_mode("QRLOITER");
    if (starts_with(w, "bridge") || starts_with(w, "drive leg") || starts_with(w, "scan leg")) return stop(w);
    sleep(unreachable_hold_s_);
    if (policy != "next" || fc_lost()) return stop(w);
    if (!set_mode("GUIDED")) return stop("could not re-enter GUIDED after " + w);
  }
  bridge_enable(false);
  report({{"phase", "DONE"}});
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
