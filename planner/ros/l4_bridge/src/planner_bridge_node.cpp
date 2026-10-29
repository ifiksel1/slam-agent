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
//       ~drive_cmd_topic                      the track planner's carrot (/track_planner/pos_cmd,
//                                             PositionCommand, position + yaw), used in drive mode
//       /fastlio_health, ~range_topic         optional SLAM health (off when absent)
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

const char* const SLAM_HOLD_REASONS[] = {"SLAM", "no SLAM", "frame re-anchored", "no frame anchor"};
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
    // flight yaw (owner rule 2026-10-29): "travel" points the nose along the commanded velocity while it
    // is above ~yaw_speed_min and holds the yaw (or the executive's waypoint yaw) when slower, rate-limited
    // to ~yaw_rate_max_deg; "planner" passes SUPER's yaw through (the Python bridge's behaviour)
    flight_yaw_ = l4::param_string(pnh_, "flight_yaw", "travel");
    if (flight_yaw_ != "travel" && flight_yaw_ != "planner") throw std::invalid_argument("~flight_yaw must be travel or planner");
    yaw_speed_min_ = l4::param_double(pnh_, "yaw_speed_min", 0.3);
    yaw_rate_max_ = l4::param_double(pnh_, "yaw_rate_max_deg", 60.0) * (M_PI / 180.0);
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
    subs_.push_back(nh_.subscribe<quadrotor_msgs::PositionCommand>(
        "/planning/pos_cmd", 20,
        boost::function<void(const ros::MessageEvent<const quadrotor_msgs::PositionCommand>&)>(
            [this](const ros::MessageEvent<const quadrotor_msgs::PositionCommand>& e) { on_cmd(e, "flight"); })));
    subs_.push_back(nh_.subscribe<quadrotor_msgs::PositionCommand>(
        l4::param_string(pnh_, "drive_cmd_topic", "/track_planner/pos_cmd"), 20,
        boost::function<void(const ros::MessageEvent<const quadrotor_msgs::PositionCommand>&)>(
            [this](const ros::MessageEvent<const quadrotor_msgs::PositionCommand>& e) { on_cmd(e, "drive"); })));
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
    if (step >= 12) {
      const size_t n = out.data.size() / step;
      for (size_t i = 0; i < n; ++i) {
        uint8_t* d = out.data.data() + i * step;
        float xyz[3];
        std::memcpy(xyz, d, 12);
        const float planner[3] = {xyz[2], xyz[0], xyz[1]};   // POSITION_MATRIX: (z, x, y)
        std::memcpy(d, planner, 12);
      }
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
    hold_point_.reset();
    reversing_.reset();
    stop_yaw_.reset();
    yaw_out_.reset();
    hold_.latch("mode changed to " + mode, now());
    publish_mode();
    ROS_INFO("planner bridge: mode %s", mode.c_str());
  }

  void on_cmd(const ros::MessageEvent<const quadrotor_msgs::PositionCommand>& e, const std::string& source) {
    if (source != mode_) return;   // the other mode's planner
    const auto& m = *e.getConstMessage();
    cmd_ = Cmd{vec(m.position), vec(m.velocity), vec(m.acceleration), m.yaw};
    t_cmd_ = now();
    const std::string sender = e.getPublisherName();
    if (!sender.empty()) liveness_.set_node(source, sender);   // the planner process whose liveness counts
  }

  // the executive's waypoint yaw (fc frame, rad; NaN clears it): faced while the commanded speed is slow
  void on_yaw_cmd(const std_msgs::Float64::ConstPtr& m) {
    if (std::isfinite(m->data)) yaw_cmd_ = wrap_pi(m->data);
    else yaw_cmd_.reset();
  }

  // the flight yaw to send. planner: SUPER's (anchored) yaw. travel (owner rule 2026-10-29): the travel
  // heading while moving; slow or hovering, the executive's waypoint yaw, else the yaw it has; one step
  // of at most yaw_rate_max per tick from the yaw last sent (the FC's yaw when the stream starts)
  double flight_yaw(double planner_yaw, std::optional<double> travel, double yaw_now) {
    if (flight_yaw_ != "travel") return planner_yaw;
    if (!yaw_out_) yaw_out_ = yaw_now;
    const double desired = travel ? *travel : yaw_cmd_ ? *yaw_cmd_ : *yaw_out_;
    yaw_out_ = yaw_step(*yaw_out_, desired, yaw_rate_max_ / rate_);
    return *yaw_out_;
  }

  void on_health(const std_msgs::Float64MultiArray::ConstPtr& m) { health_.on_health(now(), m->data); }

  void on_range(const sensor_msgs::Range::ConstPtr& m) {
    if (fc_pose_) health_.on_range(now(), m->range, fc_pose_->p[2]);
  }

  // ---- output ------------------------------------------------------------------------------------
  void tick(const ros::TimerEvent&) {
    const double t_now = now();
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
    const std::string state = hold_.step(t_now, t_cmd_, t_slam_, faults, liveness_.alive(node));
    if (fc_mode_ != guided_mode_ || !enabled_ || fc == "never") {
      hold_point_.reset();
      yaw_out_.reset();
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
          send_reverse(*rs, target->yaw);
          return;
        }
        reversing_.reset();
        send(drive_target(target->p, target->v, p_now, drive_lookahead_), nullptr, nullptr, target->yaw, MASK_DRIVE);
      } else {
        auto c = cap_command(target->p, target->v, target->a, p_now, speed_cap_, max_lead_, fc_pos_p_);
        send(c.p, &c.v, &c.a, flight_yaw(target->yaw, travel_yaw(target->v, yaw_speed_min_), yaw_now), MASK_FLIGHT);
      }
      return;
    }
    if (mode == "drive") {
      if (hold_.dead) {
        // the planner died: stop where the rover is, now. A zero-speed velocity + heading target (the
        // reverse HOLD's), the heading the rover has at the death, so it neither drives on to the last
        // (run-out) target nor pivots
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

  // back up at speed (m/s, >= 0) holding yaw (fc frame, ENU): MAVROS turns the ENU velocity (0, -speed)
  // into NED vx = -speed, which Rover Guided reads as reverse (speed_dir -1); at 0 a zero-speed heading hold
  void send_reverse(double speed, double yaw) {
    reversing_ = yaw;
    const Vec3 v(0.0, -std::abs(speed), 0.0);
    send(Vec3::Zero(), &v, nullptr, yaw, MASK_REVERSE);
  }

  void send(const Vec3& p, const Vec3* v, const Vec3* a, double yaw, uint16_t mask) {
    mavros_msgs::PositionTarget sp;
    sp.header.stamp = ros::Time::now();
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
    s["state"] = hold_.state;
    s["reason"] = hold_.reason;
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
  bool health_on_ = false, enabled_ = false;

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
  std::optional<double> yaw_out_;                       // the flight yaw last sent (travel mode)
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
