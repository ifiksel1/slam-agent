// Track planner node: drives the tracked QuadRover to /planning/drive_goal on the live scans.
// C++ port of planner/track_planner.py (2026-10-29); same node name, topics, params and status JSON.
//
//   in   /planner/cloud         sensor_msgs/PointCloud2, planner frame (z up), world-registered scans
//                               (FAST-LIO's dense /cloud_registered through the planner bridge)
//        /planner/odom          nav_msgs/Odometry, planner frame (FAST-LIO's IMU: the odometry point,
//                               ~footprint_offset 0.193 m ahead of the frame centre, l4_track::ODOM_AHEAD)
//        /planning/drive_goal   geometry_msgs/PoseStamped, planner frame (from the mission executive)
//   out  ~cmd_topic             quadrotor_msgs/PositionCommand (default /track_planner/pos_cmd), streamed
//                               by the planner bridge in its drive mode. Driving: the carrot ~lookahead
//                               ahead on the path run out along its bearing (l4_track::run_out: up to
//                               ~runout_s x ~drive_speed from the FC, frame-checked, none at stop points,
//                               while pivoting or turning), position + yaw, zero velocity (D4: position
//                               + yaw, mask 2552). Reversing out of a dead end (l4_track::plan_reverse):
//                               the FC's end point, a velocity of ~reverse_speed against the yaw, the
//                               heading held (the bridge's reverse: bridge_core.reverse_speed). Not
//                               SUPER's /planning/pos_cmd: SUPER keeps publishing toward its last goal
//                               after a landing, and the bridge takes each mode's commands from its own
//                               planner's topic only
//        /track_planner/path    nav_msgs/Path (each pose's orientation: the heading driven or, at a
//                               pivot's arrival and leaving points, the heading before and after it)
//        /track_planner/status  std_msgs/String JSON: state (IDLE, DRIVING, REACHED, NO_PATH), reason,
//                               plan_ms (the last replan's wall time), phase (drive, approach: the
//                               carrot holds at the next pivot, pivot: turning in place there,
//                               reverse: backing out of a dead end),
//                               pivots: [{x, y (the frame centre), turn_deg, yaw_in_deg, yaw_out_deg,
//                               start}] of the current path
//
// Pivots (l4_track::pivot_pass, follow_plan). A turn over ~pivot_angle_deg (Rover's WP_PIVOT_ANGLE)
// is a pivot in place about the frame centre, planned as its own step where the pivot circle is
// clear; the carrot holds ~pivot_lead past the centre (half the robot's WP_RADIUS 0.3: Rover stops
// within WP_RADIUS of a held target) until the robot has arrived, then sits on the leaving ray until
// the heading is within ~pivot_done_deg, so Rover pivots there rather than arcing. The robot's
// heading (/planner/odom orientation, the nose in the planner frame) is the start heading of each
// replan: a turn on the spot is a start pivot and is checked where the robot stands. The pivot
// circle's margin is ~pivot_margin (its own, owner 2026-10-28; the frame's ~margin stays for driving).
//
// Dead ends (owner 2026-10-28; l4_track::plan_reverse). With the goal behind (beyond pivot_angle) and
// no room for the start pivot where the robot stands, it backs straight up the way it came, at most
// ~reverse_max (2 m), over cells observed free (floor seen in the live window, or seen through in the
// evidence memory: never unknown space, the JT128 dome sees nothing behind) with the oriented frame
// clear, to the nearest spot the start pivot fits; then pivots and drives on.
//
// The arc onto the path (owner request 2026-10-29, after the Gate L2 door-jamb clips c1/c3). A turn
// under pivot_angle is driven: the robot arcs onto the path with its heading lagging (SITL: 7-10 deg
// all the way to a 0.90 m door). Each replan simulates that arc onto every driving leg from the
// robot's heading (Rover's steering lag ~arc_steer 3.0 m, fit to SITL; ~arc_check false: off) and checks its frame;
// at the start a turn whose arc is not clear becomes a start pivot (its hard check applies; the
// carrot leads the heading past WP_PIVOT_ANGLE so Rover pivots); any other arc failure is NO_PATH (the
// last path is kept while it still checks). ~arc_margin 0 (the bare frame), ~arc_commit 1.0 m (a robot
// already heading along its leg is committed to that much), ~arc_vertices false (a pivot at a gentle
// turn whose arc fails instead of NO_PATH; off: see track_core.cpp LIMITS).
//
// Speed (l4_track::run_out). Rover Guided's fast-update path asks ~0.5 m/s per metre to its target
// and steers like pure pursuit on it: the carrot alone held it to ~0.3 m/s; the run-out gives it
// ~runout_s (2.1) x ~drive_speed (the FC's WP_SPEED) of target on straights, and none at stop points
// or while it turns. ~drive_lookahead on the bridge must cover it. ~drive_speed defaults to 1.0 and
// is capped at 1.0 m/s, the robot's driving speed cap (owner 2026-10-29: WP_SPEED 1.0 on the robot);
// a higher value is clamped with a warning.
//
// The map is the last ~window_s seconds of scans (l4_track::LiveWindow), not an ever-growing union:
// z drift between visits and moved objects (people, doors) made whole maps block the robot's own
// path. Each scan is deduplicated as it arrives; each replan (~replan_hz, default 10) builds a height
// grid over the start-goal box with a local ground model, corrects it with a decaying per-cell hit /
// see-through memory (moved people cleared, walls seen earlier or sparsely held) and plans with the
// oriented frame. Unknown space is driven through at a cost and replanned as it is seen. A failed
// replan keeps the last path while what is left of it still checks in the new window; otherwise it
// clears the path and the bridge holds.
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <cstring>
#include <mutex>
#include <nlohmann/json.hpp>

#include "l4_track/track_core.h"

namespace tc = l4_track;
using tc::V2;

namespace {
constexpr double MAX_DRIVE_SPEED = 1.0;   // the robot's driving speed cap, m/s (owner 2026-10-29)

double getd(ros::NodeHandle& nh, const std::string& name, double def) {
  double v = def;
  if (!nh.getParam(name, v)) {
    int i;
    if (nh.getParam(name, i)) v = i;
  }
  return v;
}

// PointCloud2 -> xyz (FLOAT32 or FLOAT64 fields), non-finite points dropped
tc::CloudF cloud_xyz(const sensor_msgs::PointCloud2& m) {
  int off[3] = {-1, -1, -1}, typ[3] = {0, 0, 0};
  const char* names[3] = {"x", "y", "z"};
  for (auto& f : m.fields)
    for (int k = 0; k < 3; ++k)
      if (f.name == names[k]) {
        off[k] = int(f.offset);
        typ[k] = f.datatype;
      }
  tc::CloudF out;
  if (off[0] < 0 || off[1] < 0 || off[2] < 0) return out;
  size_t n = size_t(m.width) * m.height;
  out.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    const uint8_t* p = m.data.data() + i * m.point_step;
    float v[3];
    for (int k = 0; k < 3; ++k) {
      if (typ[k] == sensor_msgs::PointField::FLOAT64) {
        double d;
        std::memcpy(&d, p + off[k], 8);
        v[k] = float(d);
      } else {
        std::memcpy(&v[k], p + off[k], 4);
      }
    }
    if (std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2])) out.push_back({v[0], v[1], v[2]});
  }
  return out;
}

double round_(double x, int nd) { return tc::pyround(x, nd); }
}  // namespace

class TrackPlanner {
 public:
  TrackPlanner() : pnh_("~") {
    tc::Config c = tc::live_config();
    c.res = getd(pnh_, "res", 0.1);
    c.margin = getd(pnh_, "margin", 0.05);
    c.z_lo = getd(pnh_, "z_lo", 0.15);
    c.z_hi = getd(pnh_, "z_hi", 0.60);
    c.min_pts = int(getd(pnh_, "min_pts", 2));
    c.lookahead = getd(pnh_, "lookahead", 0.5);
    c.pad = getd(pnh_, "pad", 3.0);
    c.off = getd(pnh_, "footprint_offset", tc::ODOM_AHEAD);
    c.min_tall_only = int(getd(pnh_, "min_tall_only", 2));
    c.mem_min_range = getd(pnh_, "mem_min_range", 1.5);
    c.pivot_angle_deg = getd(pnh_, "pivot_angle_deg", 60.0);
    c.pivot_done_deg = getd(pnh_, "pivot_done_deg", 10.0);
    c.pivot_tol = getd(pnh_, "pivot_tol", 0.2);
    c.pivot_search = getd(pnh_, "pivot_search", 0.6);
    c.pivot_margin = getd(pnh_, "pivot_margin", tc::Config().pivot_margin);
    c.reverse_max = getd(pnh_, "reverse_max", 2.0);
    // the arc check (track_core: arc_fail): the robot's own arc onto each leg, Rover's steering lag fit
    // to SITL (2 sin(error) / max(distance to its target, arc_steer))
    bool arc = true;
    pnh_.param("arc_check", arc, true);
    c.arc_check = arc;
    c.arc_steer = getd(pnh_, "arc_steer", 3.0);
    c.arc_margin = getd(pnh_, "arc_margin", 0.0);
    c.arc_commit = getd(pnh_, "arc_commit", 1.0);
    bool av = false;
    pnh_.param("arc_vertices", av, false);
    c.arc_vertices = av;
    c.update();
    cfg_ = c;
    pivot_lead_ = getd(pnh_, "pivot_lead", 0.15);
    // the run-out (l4_track::run_out): the target the FC sees, up to runout_s x drive_speed ahead
    // along the carrot's bearing. Rover Guided's fast-update path asks ~0.5 m/s per metre to its
    // target (SITL d10: 0.58 m/s at 1.15 m), so WP_SPEED needs ~2 s of it. drive_speed: the FC's
    // WP_SPEED, at most the robot's 1 m/s cap. runout_s 0: the bare carrot
    drive_speed_ = getd(pnh_, "drive_speed", 1.0);
    if (drive_speed_ > MAX_DRIVE_SPEED) {
      ROS_WARN("track planner: drive_speed %.2f m/s is over the robot's %.1f m/s driving cap: using %.1f m/s",
               drive_speed_, MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
      drive_speed_ = MAX_DRIVE_SPEED;
    }
    runout_ = getd(pnh_, "runout_s", 2.1) * drive_speed_;
    cfg_.arc_reach = runout_;   // the arc check steers toward the node's own target
    // the FC (Rover's position) behind the odometry point, m: the FC sits at the frame centre (owner
    // 2026-10-29), so footprint_offset (the Python node had 0.178, an FC 0.015 m ahead of the centre)
    fc_behind_ = getd(pnh_, "fc_behind", c.off);
    // dead ends (l4_track::plan_reverse): back up at this speed, straight, holding the heading
    reverse_speed_ = getd(pnh_, "reverse_speed", 0.25);
    win_ = std::make_unique<tc::LiveWindow>(cfg_, getd(pnh_, "window_s", 3.0), getd(pnh_, "chunk_s", 0.5), 2,
                                            std::nullopt, getd(pnh_, "mem_tau", 10.0), getd(pnh_, "mem_hits", 1.5),
                                            getd(pnh_, "mem_clear_ratio", 1.5));
    accept_ = getd(pnh_, "accept_radius", 0.3);
    std::string cmd_topic;
    pnh_.param<std::string>("cmd_topic", cmd_topic, "/track_planner/pos_cmd");
    pub_cmd_ = nh_.advertise<quadrotor_msgs::PositionCommand>(cmd_topic, 10);
    pub_path_ = nh_.advertise<nav_msgs::Path>("/track_planner/path", 1, true);
    pub_status_ = nh_.advertise<std_msgs::String>("/track_planner/status", 1, true);
    sub_cloud_ = nh_.subscribe("/planner/cloud", 2, &TrackPlanner::on_cloud, this);
    sub_odom_ = nh_.subscribe("/planner/odom", 20, &TrackPlanner::on_odom, this);
    sub_goal_ = nh_.subscribe("/planning/drive_goal", 1, &TrackPlanner::on_goal, this);
    t_replan_ = nh_.createTimer(ros::Duration(1.0 / getd(pnh_, "replan_hz", 10.0)), [this](const ros::TimerEvent&) { replan(); });
    t_stream_ = nh_.createTimer(ros::Duration(0.1), [this](const ros::TimerEvent&) { stream(); });
  }

 private:
  ros::NodeHandle nh_, pnh_;
  tc::Config cfg_;
  double pivot_lead_, drive_speed_, runout_, fc_behind_, reverse_speed_, accept_;
  std::unique_ptr<tc::LiveWindow> win_;
  ros::Publisher pub_cmd_, pub_path_, pub_status_;
  ros::Subscriber sub_cloud_, sub_odom_, sub_goal_;
  ros::Timer t_replan_, t_stream_;
  std::mutex win_mx_, st_mx_, replan_mx_;
  // state (st_mx_)
  std::optional<Eigen::Vector3d> odom_;
  std::optional<double> yaw_;
  std::optional<V2> goal_;
  std::optional<tc::Path> path_, route_;   // route: the path before the pivot steps
  std::vector<tc::Pivot> pivots_;
  std::string phase_ = "drive";
  std::optional<double> plan_ms_;
  std::string state_ = "IDLE", reason_;
  struct Cur {
    std::optional<tc::Path> path;
    std::vector<tc::Pivot> pivots;
    std::shared_ptr<tc::Window> grid;
    std::optional<tc::Reverse> rev;
  } cur_;

  void on_odom(const nav_msgs::Odometry::ConstPtr& m) {
    auto& p = m->pose.pose.position;
    auto& q = m->pose.pose.orientation;
    std::lock_guard<std::mutex> l(st_mx_);
    odom_ = Eigen::Vector3d(p.x, p.y, p.z);
    yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  void on_cloud(const sensor_msgs::PointCloud2::ConstPtr& m) {
    std::optional<Eigen::Vector3d> od;
    {
      std::lock_guard<std::mutex> l(st_mx_);
      od = odom_;
    }
    if (!od) return;
    tc::CloudF pts = cloud_xyz(*m);
    if (!pts.empty()) {
      std::lock_guard<std::mutex> l(win_mx_);
      win_->add_scan(m->header.stamp.toSec(), pts, *od);
    }
  }

  void on_goal(const geometry_msgs::PoseStamped::ConstPtr& m) {
    {
      std::lock_guard<std::mutex> l(st_mx_);
      goal_ = V2(m->pose.position.x, m->pose.position.y);
      set_path(std::nullopt);
      phase_ = "drive";   // nothing under way from an earlier leg
      set_state("DRIVING", "new goal");
    }
    replan();
  }

  // (st_mx_ held) grid: the window the path was checked in, for the run-out check; reverse: a dead
  // end's reverse before the path (plan_reverse: the path starts where it ends)
  void set_path(std::optional<tc::Path> path, const std::vector<tc::Pivot>& pivots = {},
                std::optional<tc::Path> route = std::nullopt, std::shared_ptr<tc::Window> grid = nullptr,
                std::optional<tc::Reverse> reverse = std::nullopt) {
    if (path) {
      path_ = path;
      pivots_ = pivots;
      route_ = route;
      cur_ = {path, pivots, grid, reverse};
    } else {
      path_.reset();
      pivots_.clear();
      route_.reset();
      cur_ = Cur{};
    }
  }

  nlohmann::ordered_json status() const {
    nlohmann::ordered_json pv = nlohmann::ordered_json::array();
    for (auto& p : pivots_)
      pv.push_back({{"x", round_(p.centre.x(), 3)},
                    {"y", round_(p.centre.y(), 3)},
                    {"turn_deg", round_(tc::degrees(p.turn), 1)},
                    {"yaw_in_deg", round_(tc::degrees(p.yaw_in), 1)},
                    {"yaw_out_deg", round_(tc::degrees(p.yaw_out), 1)},
                    {"start", p.start}});
    nlohmann::ordered_json j;
    j["state"] = state_;
    j["reason"] = reason_;
    j["plan_ms"] = plan_ms_ ? nlohmann::ordered_json(*plan_ms_) : nlohmann::ordered_json(nullptr);
    j["phase"] = phase_;
    j["pivots"] = pv;
    return j;
  }

  void publish_status() {
    std_msgs::String s;
    s.data = status().dump();
    pub_status_.publish(s);
  }

  void set_state(const std::string& s, const std::string& why = "") {
    state_ = s;
    reason_ = why;
    publish_status();
  }

  static std::vector<std::pair<double, double>> centres(const std::vector<tc::Pivot>& pv) {
    std::vector<std::pair<double, double>> out;
    for (auto& p : pv) out.push_back({round_(p.centre.x(), 2), round_(p.centre.y(), 2)});
    return out;
  }

  void replan() {
    std::lock_guard<std::mutex> rl(replan_mx_);
    Eigen::Vector3d pos;
    double yaw;
    V2 goal;
    bool pivoting;
    std::vector<std::pair<double, double>> before;
    std::optional<tc::Path> route;
    {
      std::lock_guard<std::mutex> l(st_mx_);
      if (!goal_ || !odom_ || state_ == "REACHED") return;
      pos = *odom_;
      yaw = *yaw_;
      goal = *goal_;
      if (tc::norm2(pos.head<2>() - goal) < accept_) {
        set_path(std::nullopt);
        set_state("REACHED");
        return;
      }
      pivoting = phase_ == "pivot";
      before = centres(pivots_);
      route = route_;
    }
    V2 p2 = pos.head<2>();
    auto t0 = ros::WallTime::now();
    tc::PlanResult out;
    {
      std::lock_guard<std::mutex> l(win_mx_);
      out = win_->plan(p2, goal, yaw, pivoting);
    }
    double ms = round_(1000.0 * (ros::WallTime::now() - t0).toSec(), 1);
    std::lock_guard<std::mutex> l(st_mx_);
    plan_ms_ = ms;
    if (!out.ok) {
      // keep the last path while what is left of it still checks in the new window (the frame along
      // it and its pivots, from where the robot is now), so one failed replan does not stop the
      // robot; otherwise clear it and the bridge holds
      if (route && out.grid) {
        tc::Path rest = tc::remaining(*route, p2);
        auto kept = tc::check_route(*out.grid, rest, yaw, pivoting);
        if (kept.ok) {
          set_path(kept.path, kept.pivots, rest, out.grid);
          if (reason_ != "kept the last path") set_state("DRIVING", "kept the last path");
          return;
        }
      }
      set_path(std::nullopt);
      // no command streams now (the bridge holds; Rover finishes turning toward its last target and
      // stops): a 'pivot' phase left over would make every replan from here a pivot under way (any
      // turn over pivot_done needs the circle where it stands: d7 sat 21 s in NO_PATH)
      phase_ = "drive";
      std::string why = out.reason + (out.reverse_reason.empty() ? "" : "; reverse: " + out.reverse_reason);
      set_state("NO_PATH", why);
      return;
    }
    if (pivoting && route) {
      // a pivot under way: finish it on the route it began on while that still checks from here (a
      // new route mid-turn can send the robot back the other way)
      tc::Path rest = tc::remaining(*route, p2);
      auto kept = tc::check_route(*out.grid, rest, yaw, true);
      if (kept.ok) {
        out.path = kept.path;
        out.pivots = kept.pivots;
        out.route = rest;
        out.reverse.reset();
      }
    }
    set_path(out.path, out.pivots, out.route, out.grid, out.reverse);
    if (state_ != "DRIVING" || reason_ != "path found" || centres(pivots_) != before) set_state("DRIVING", "path found");
    nav_msgs::Path msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    auto heads = headings(*path_, pivots_);
    for (size_t k = 0; k < path_->size(); ++k) {
      geometry_msgs::PoseStamped ps;
      ps.header = msg.header;
      ps.pose.position.x = (*path_)[k].x();
      ps.pose.position.y = (*path_)[k].y();
      ps.pose.position.z = pos.z();
      ps.pose.orientation.z = std::sin(heads[k] / 2.0);
      ps.pose.orientation.w = std::cos(heads[k] / 2.0);
      msg.poses.push_back(ps);
    }
    pub_path_.publish(msg);
  }

  // per path point: the heading of the segment leaving it (the last: arriving); a pivot's arrival
  // and leaving points: its headings before and after
  static std::vector<double> headings(const tc::Path& P, const std::vector<tc::Pivot>& pivots) {
    std::vector<double> h;
    for (size_t k = 0; k + 1 < P.size(); ++k) h.push_back(tc::hd(P[k + 1] - P[k]));
    h.push_back(h.empty() ? 0.0 : h.back());
    for (auto& p : pivots) {
      h[p.i] = p.yaw_in;
      h[p.i + 1] = p.yaw_out;
    }
    return h;
  }

  void stream() {
    // no command when idle, reached or without a path: the bridge then holds (zero velocity)
    std::lock_guard<std::mutex> l(st_mx_);
    if (state_ != "DRIVING" || !cur_.path || !odom_) return;
    V2 p2 = odom_->head<2>();
    V2 c, vel(0, 0);
    double yaw;
    std::string phase;
    if (cur_.rev && tc::reverse_left(*cur_.rev, p2) > 0.03) {
      // a dead end: back straight up to where the pivot fits: the FC's end point, a velocity against
      // the heading (the bridge's reverse: bridge_core.reverse_speed), the heading held
      yaw = cur_.rev->yaw;
      phase = "reverse";
      V2 u = tc::unit(yaw);
      c = cur_.rev->to - fc_behind_ * u;
      vel = -reverse_speed_ * u;
    } else {
      auto f = tc::follow_plan(*cur_.path, cur_.pivots, p2, yaw_, cfg_, pivot_lead_);
      yaw = f.yaw;
      phase = f.phase;
      c = tc::run_out(*cur_.path, cur_.pivots, p2, yaw_, f.c, f.phase, f.j, cfg_, runout_, fc_behind_, pivot_lead_,
                      cur_.grid.get())
              .first;
    }
    if (phase != phase_) {
      phase_ = phase;
      publish_status();
    }
    quadrotor_msgs::PositionCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "world";
    cmd.position.x = c.x();
    cmd.position.y = c.y();
    cmd.position.z = odom_->z();
    cmd.velocity.x = vel.x();
    cmd.velocity.y = vel.y();
    cmd.yaw = yaw;
    cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    pub_cmd_.publish(cmd);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "track_planner");
  TrackPlanner tp;
  ros::AsyncSpinner spinner(4);   // as rospy: the cloud, odometry, replan and stream run side by side
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
