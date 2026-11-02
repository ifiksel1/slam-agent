// Landing-spot check node (D3): answers the mission executive's "is this spot clear to land?"
// C++ port of planner/landing_check.py (2026-10-29); same node name, topics, params and status JSON.
//
//   in:  /planner/cloud          sensor_msgs/PointCloud2, planner frame (z up), from the planner bridge;
//                                each scan kept on a 5 cm voxel grid for the last ~window_s seconds,
//                                with the sensor position at its arrival (the latest /planner/odom)
//        /planner/odom           nav_msgs/Odometry, planner frame: the lidar (returns nearer than
//                                ~near_range read high on recorded data and are dropped) and the
//                                heading: the disc is centred on the frame, ~footprint_offset
//                                (l4_track::ODOM_AHEAD, 0.193 m) behind the spot; no odom yet ->
//                                not clear, "no heading"
//        /planner/landing_spot   geometry_msgs/PointStamped, planner frame: the spot the executive
//                                wants to land on (z = the height it descends from)
//        /mission_executive/ground_spots  std_msgs/String, latched JSON {"frame": "fc", "spots":
//                                [{"event", "xyz", "yaw", "t"}]}: where the robot sat on the ground this
//                                mission (each takeoff spot, each landing), the FC pose (odometry point)
//                                in the fc frame. Their frame footprints are known floor (landing_core:
//                                unobserved cells there count as observed; occupied ones stay occupied)
//        /planner_bridge/anchor  geometry_msgs/TransformStamped, latched, fc <- planner: converts the
//                                ground spots to the planner frame at every evaluation, so a SLAM
//                                re-anchor moves them with the map (no anchor yet: they are not used,
//                                status "known_floor": "no anchor")
//   out: /planner/landing_clear  std_msgs/Bool, every tick_s while a spot request is fresh (< spot_timeout_s)
//        /planner/landing_status std_msgs/String, JSON: the reason and the numbers behind it
//                                ("spot", and "centre": the disc's)
//        /planner/landing_markers visualization_msgs/MarkerArray, in /planner/cloud's frame (world),
//                                on every evaluation; DELETEALL when the spot request goes stale
//                                (> spot_timeout_s) or the robot is more than ~marker_max_dist away.
//                                Namespaces (one marker each, id 0):
//                                  landing_disc        CYLINDER at the floor, alpha 0.35: green clear,
//                                                      red occupied / not flat, amber uncertain /
//                                                      not observed / no heading
//                                  landing_footprint   LINE_STRIP, the frame's rectangle (white)
//                                  landing_occupied    CUBE_LIST, 10 cm cells, top at their height
//                                                      above the floor (red)
//                                  landing_unobserved  CUBE_LIST, flat grey cells at the floor
//                                  landing_known_floor LINE_LIST, light blue: the footprints where
//                                                      the robot sat (known floor) that reach the disc
//                                  landing_label       TEXT_VIEW_FACING, decision + reason + layer
//
// The landing-spot search (owner request 2026-10-30; landing_search.cpp): the executive, in the air and about
// to land to drive, asks for a spot near the vehicle when the spot under it is refused.
//   in:  /planner/landing_search  std_msgs/String, JSON, planner frame: {"id": n, "vehicle": [x, y, z] (the
//                                spot under the vehicle, z = the height it descends from, as landing_spot),
//                                "goal": [x, y] or null (the drive goal: candidates face it), "radius": m,
//                                "exclude": [[x, y], ...] (spots already refused)}
//   out: /planner/landing_search_result  std_msgs/String, JSON: {"id", "found", "spot": [x, y] (planner),
//                                "yaw_deg", "dist_m", "cost", "margin_m", "candidates", "evaluated",
//                                "reasons": {word: count}, "spots": [[x, y, word, number, los_open], ...] (each
//                                evaluated candidate, landing_search.cpp), "los": {"z", "radius", "penalty",
//                                "blocked"} or null, "los_open", "los_points" (the chosen spot's line of sight from
//                                the vehicle), "budget_hit", "elapsed_s", "check": the chosen
//                                spot's status, "scans", "window_s"} or, none found, "first_reason"
//        /planner/landing_search_markers  visualization_msgs/MarkerArray (DELETEALL, then):
//                                  land_search_candidates  CUBE_LIST, 8 cm squares at the floor per candidate:
//                                                      green clear, red occupied / not flat, amber not
//                                                      observed / uncertain, grey excluded / not evaluated
//                                  land_search_radius  LINE_STRIP, the search circle (white)
//                                  land_search_choice  CYLINDER, the chosen spot's disc (green)
//                                  land_search_choice_footprint  LINE_STRIP, its frame and heading (white)
//                                  land_search_los     LINE_STRIP, vehicle -> chosen spot at the check height:
//                                                      green open, amber blocked (SUPER detours)
//                                  land_search_label   TEXT_VIEW_FACING, the distance and margin, or "NO
//                                                      LANDING SPOT"
//                                cleared (DELETEALL) when the vehicle is more than ~marker_max_dist from it
// The search uses the scans of the last ~search_window_s (20 s, longer than window_s so that a 360 deg yaw
// spin's scans all count; the node keeps that much), the same check rules, ~search_step (0.2 m) grid,
// ~search_goal_weight (0.5), ~search_tie_m (0.15), ~search_margin_max (0.6), ~search_budget_s (6 s): see
// landing_search.cpp. The search runs on its own (a separate check object), so the spot check keeps ticking.
//
// Only surfaces below the vehicle are floor (owner 2026-11-02): at least ~below_min (0.3 m) under the spot's z
// (the height it descends from; z <= 0: the latest /planner/odom's z), landing_core.cpp; the spot check and every
// search candidate alike.
//
// Rules in landing_core.cpp (scan by scan: each scan's floor is fitted to the track planner's local
// ground model, and the latest scans decide whether a cell is occupied). Anything uncertain is not clear.
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <algorithm>
#include <chrono>
#include <tuple>
#include <cstring>
#include <mutex>

#include "l4_landing/landing_core.h"
#include "l4_landing/landing_markers.h"
#include "l4_landing/landing_search.h"

using namespace l4_landing;
namespace tc = l4_track;

namespace {
double mono() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
double getd(ros::NodeHandle& nh, const std::string& n, double def) {
  double v = def;
  if (!nh.getParam(n, v)) {
    int i;
    if (nh.getParam(n, i)) v = i;
  }
  return v;
}
double yaw_of(const geometry_msgs::Quaternion& q) {
  return std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}
}  // namespace

class LandingCheckNode {
 public:
  LandingCheckNode() : pnh_("~") {
    Params p;
    p.margin = getd(pnh_, "margin", 0.2);
    p.observed_min = getd(pnh_, "observed_min", 0.7);
    p.slope_max_deg = getd(pnh_, "slope_max_deg", 5.0);
    p.residual_max = getd(pnh_, "residual_max", 0.03);
    p.obstacle_min_h = getd(pnh_, "obstacle_min_h", 0.05);
    p.floor_layer_max = getd(pnh_, "floor_layer_max", 0.20);
    p.near_range = getd(pnh_, "near_range", 1.2);
    p.footprint_offset = getd(pnh_, "footprint_offset", tc::ODOM_AHEAD);
    p.below_min = getd(pnh_, "below_min", 0.3);   // a floor lies at least this far under the vehicle
    check_ = LandingCheck(p);
    search_check_ = LandingCheck(p);
    window_s_ = getd(pnh_, "window_s", 10.0);
    search_window_s_ = getd(pnh_, "search_window_s", 20.0);
    sp_.step = getd(pnh_, "search_step", sp_.step);
    sp_.goal_weight = getd(pnh_, "search_goal_weight", sp_.goal_weight);
    sp_.tie_m = getd(pnh_, "search_tie_m", sp_.tie_m);
    sp_.margin_max = getd(pnh_, "search_margin_max", sp_.margin_max);
    sp_.heading_min_dist = getd(pnh_, "search_heading_min_dist", sp_.heading_min_dist);
    sp_.exclude_r = getd(pnh_, "search_exclude_r", sp_.exclude_r);
    sp_.budget_s = getd(pnh_, "search_budget_s", sp_.budget_s);
    // line of sight (owner 2026-11-02, landing_search.cpp): a blocked candidate's cost + ~search_los_penalty
    sp_.los_penalty = getd(pnh_, "search_los_penalty", sp_.los_penalty);
    sp_.los_radius = getd(pnh_, "search_los_radius", sp_.los_radius);
    sp_.los_min_points = int(getd(pnh_, "search_los_min_points", sp_.los_min_points));
    search_max_radius_ = getd(pnh_, "search_max_radius", 4.0);   // a request's radius is capped here
    voxel_ = getd(pnh_, "voxel", 0.05);
    spot_timeout_s_ = getd(pnh_, "spot_timeout_s", 2.0);
    odom_timeout_s_ = getd(pnh_, "odom_timeout_s", 0.5);
    marker_max_dist_ = getd(pnh_, "marker_max_dist", 5.0);   // the robot left: markers deleted
    pnh_.param<std::string>("frame_id", frame_, "world");   // replaced by /planner/cloud's frame_id
    std::string gs_topic;
    pnh_.param<std::string>("ground_spots_topic", gs_topic, "/mission_executive/ground_spots");
    pub_clear_ = nh_.advertise<std_msgs::Bool>("/planner/landing_clear", 10);
    pub_status_ = nh_.advertise<std_msgs::String>("/planner/landing_status", 10);
    pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/planner/landing_markers", 2);
    pub_search_ = nh_.advertise<std_msgs::String>("/planner/landing_search_result", 10);
    pub_search_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/planner/landing_search_markers", 2, true);
    subs_.push_back(nh_.subscribe("/planner/odom", 20, &LandingCheckNode::on_odom, this));
    subs_.push_back(nh_.subscribe("/planner/cloud", 5, &LandingCheckNode::on_cloud, this));
    subs_.push_back(nh_.subscribe("/planner/landing_spot", 5, &LandingCheckNode::on_spot, this));
    subs_.push_back(nh_.subscribe(gs_topic, 5, &LandingCheckNode::on_ground_spots, this));
    subs_.push_back(nh_.subscribe("/planner_bridge/anchor", 5, &LandingCheckNode::on_anchor, this));
    subs_.push_back(nh_.subscribe("/planner/landing_search", 5, &LandingCheckNode::on_search, this));
    timer_ = nh_.createTimer(ros::Duration(getd(pnh_, "tick_s", 0.25)), [this](const ros::TimerEvent&) { tick(); });
  }

 private:
  ros::NodeHandle nh_, pnh_;
  LandingCheck check_, search_check_;   // the search's own: it runs beside the spot check (viz is per object)
  SearchParams sp_;
  double window_s_, search_window_s_, search_max_radius_, voxel_, spot_timeout_s_, odom_timeout_s_, marker_max_dist_;
  std::optional<V2> search_at_;   // where the last search's markers are (deleted once the vehicle is far)
  std::optional<std::pair<std::string, std::string>> last_search_;   // (request, answer): a repeat is answered at once
  std::string frame_;
  ros::Publisher pub_clear_, pub_status_, pub_markers_, pub_search_, pub_search_markers_;
  std::vector<ros::Subscriber> subs_;
  ros::Timer timer_;
  std::mutex mx_, busy_;
  std::optional<double> yaw_;
  bool markers_up_ = false;
  struct Win {
    double t;
    tc::Cloud pts;
    std::optional<Eigen::Vector3d> sensor;
  };
  std::vector<Win> scans_;
  struct Spot {
    double t, x, y, z;
  };
  std::optional<Spot> spot_;
  std::optional<std::pair<double, Eigen::Vector3d>> odom_;   // (time, xyz)
  std::vector<GroundSpot> ground_spots_;                     // the executive's (fc frame)
  std::optional<std::array<double, 3>> anchor_;              // (psi, tx, ty): fc <- planner

  void on_ground_spots(const std_msgs::String::ConstPtr& m) {
    std::vector<GroundSpot> spots;
    std::string frame = "fc";
    try {
      auto d = nlohmann::json::parse(m->data);
      for (auto& s : d.value("spots", nlohmann::json::array())) {
        auto& xyz = s.at("xyz");
        if (xyz.size() < 2 || !s.contains("yaw") || s["yaw"].is_null()) continue;
        spots.push_back({xyz[0].get<double>(), xyz[1].get<double>(), s["yaw"].get<double>()});
      }
      if (d.contains("frame")) frame = d["frame"].get<std::string>();
    } catch (const std::exception&) {
      ROS_WARN("landing check: malformed ground spots ignored");
      return;
    }
    if (frame != "fc") {
      ROS_WARN("landing check: ground spots must be in the fc frame, ignored");
      return;
    }
    std::lock_guard<std::mutex> l(mx_);
    ground_spots_ = spots;
  }

  void on_anchor(const geometry_msgs::TransformStamped::ConstPtr& m) {
    std::lock_guard<std::mutex> l(mx_);
    anchor_ = std::array<double, 3>{yaw_of(m->transform.rotation), m->transform.translation.x, m->transform.translation.y};
  }

  void on_odom(const nav_msgs::Odometry::ConstPtr& m) {
    std::lock_guard<std::mutex> l(mx_);
    yaw_ = yaw_of(m->pose.pose.orientation);
    auto& p = m->pose.pose.position;
    odom_ = std::make_pair(mono(), Eigen::Vector3d(p.x, p.y, p.z));
    if (search_at_ && std::hypot(p.x - search_at_->x(), p.y - search_at_->y()) > marker_max_dist_) {
      // the robot left the search area: its markers go
      visualization_msgs::Marker d;
      d.action = visualization_msgs::Marker::DELETEALL;
      d.header.frame_id = frame_;
      d.header.stamp = ros::Time::now();
      visualization_msgs::MarkerArray a;
      a.markers.push_back(d);
      pub_search_markers_.publish(a);
      search_at_.reset();
    }
  }

  // the landing-spot search: one request, one answer (and its markers)
  void on_search(const std_msgs::String::ConstPtr& m) {
    double t0 = mono();
    {
      // the executive repeats an unanswered request (a lost message); one already answered gets the same answer
      // (a search takes up to ~search_budget_s; recomputing each repeat queued them up, SITL run 5)
      std::lock_guard<std::mutex> l(mx_);
      if (last_search_ && last_search_->first == m->data) {
        std_msgs::String s;
        s.data = last_search_->second;
        pub_search_.publish(s);
        return;
      }
    }
    Json res;
    V2 vehicle;
    std::optional<V2> goal;
    std::optional<double> descend;
    double radius = 2.0;
    std::vector<V2> exclude;
    try {
      auto d = nlohmann::json::parse(m->data);
      res["id"] = d.value("id", 0);
      auto& v = d.at("vehicle");
      vehicle = V2(v.at(0).get<double>(), v.at(1).get<double>());
      if (v.size() > 2 && v[2].get<double>() > 0) descend = v[2].get<double>();
      if (d.contains("goal") && d["goal"].is_array() && d["goal"].size() >= 2)
        goal = V2(d["goal"][0].get<double>(), d["goal"][1].get<double>());
      radius = std::min(search_max_radius_, std::max(0.0, d.value("radius", 2.0)));
      for (auto& e : d.value("exclude", nlohmann::json::array()))
        if (e.is_array() && e.size() >= 2) exclude.push_back(V2(e[0].get<double>(), e[1].get<double>()));
    } catch (const std::exception& e) {
      ROS_WARN("landing check: malformed search request ignored: %s", e.what());
      return;
    }
    std::vector<Scan> scans;
    std::optional<double> yaw;
    std::vector<GroundSpot> spots;
    std::optional<std::array<double, 3>> anchor;
    std::optional<std::pair<double, Eigen::Vector3d>> od;
    std::string frame;
    {
      std::lock_guard<std::mutex> l(mx_);
      const double now = mono();
      for (auto& s : scans_)
        if (now - s.t <= search_window_s_) scans.push_back({s.pts, s.sensor});
      yaw = yaw_;
      spots = ground_spots_;
      anchor = anchor_;
      od = odom_;
      frame = frame_;
    }
    res["scans"] = int(scans.size());
    res["window_s"] = search_window_s_;
    res["radius"] = radius;
    if (!yaw) {
      res["found"] = false;
      res["first_reason"] = "no heading: no /planner/odom yet";
      std_msgs::String s;
      s.data = res.dump();
      pub_search_.publish(s);
      return;
    }
    std::vector<KnownPose> known;
    if (!spots.empty() && anchor) known = ground_spots_to_planner(spots, (*anchor)[0], (*anchor)[1], (*anchor)[2]);
    SearchParams sp = sp_;
    sp.radius = radius;
    SearchResult r = search_landing_spot(search_check_, scans, vehicle, *yaw, goal, descend, known, sp, exclude);
    for (auto it = r.summary.begin(); it != r.summary.end(); ++it) res[it.key()] = it.value();
    res["elapsed_s"] = tc::pyround(mono() - t0, 3);
    std_msgs::String s;
    s.data = res.dump();
    pub_search_.publish(s);
    {
      std::lock_guard<std::mutex> l(mx_);
      last_search_ = std::make_pair(m->data, s.data);
    }
    ROS_INFO("landing check: search %s: %s, %d of %d candidates evaluated in %.2f s", res["id"].dump().c_str(),
             r.best ? "found" : "none", r.evaluated, int(r.cands.size()), mono() - t0);
    try {
      double fallback = od ? od->second.z() - 1.0 : (descend ? *descend - 1.0 : 0.0);
      pub_search_markers_.publish(build_search_markers(r, search_check_, vehicle, radius, frame, ros::Time::now(), fallback));
      std::lock_guard<std::mutex> l(mx_);
      search_at_ = vehicle;
    } catch (const std::exception& e) {   // the markers never stop the answer
      ROS_WARN_THROTTLE(10.0, "landing check: search markers failed: %s", e.what());
    }
  }

  void on_cloud(const sensor_msgs::PointCloud2::ConstPtr& m) {
    int off[3] = {-1, -1, -1};
    for (auto& f : m->fields) {
      if (f.name == "x") off[0] = int(f.offset);
      if (f.name == "y") off[1] = int(f.offset);
      if (f.name == "z") off[2] = int(f.offset);
    }
    if (off[0] < 0 || off[1] < 0 || off[2] < 0) return;
    size_t n = m->point_step ? m->data.size() / m->point_step : 0;
    // one point per voxel (its first), in voxel order
    struct K {
      int64_t a, b, c;
      size_t i;
      double x, y, z;
    };
    std::vector<K> ks;
    ks.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      float v[3];
      for (int k = 0; k < 3; ++k) std::memcpy(&v[k], m->data.data() + i * m->point_step + off[k], 4);
      double x = v[0], y = v[1], z = v[2];
      if (!(std::isfinite(x) && std::isfinite(y) && std::isfinite(z))) continue;
      ks.push_back({int64_t(std::floor(x / voxel_)), int64_t(std::floor(y / voxel_)), int64_t(std::floor(z / voxel_)), i, x, y, z});
    }
    std::sort(ks.begin(), ks.end(), [](const K& p, const K& q) {
      return std::tie(p.a, p.b, p.c, p.i) < std::tie(q.a, q.b, q.c, q.i);
    });
    tc::Cloud P;
    for (size_t q = 0; q < ks.size(); ++q)
      if (q == 0 || ks[q].a != ks[q - 1].a || ks[q].b != ks[q - 1].b || ks[q].c != ks[q - 1].c)
        P.push_back({ks[q].x, ks[q].y, ks[q].z});
    double now = mono();
    std::lock_guard<std::mutex> l(mx_);
    if (!m->header.frame_id.empty()) frame_ = m->header.frame_id;
    std::optional<Eigen::Vector3d> sensor;
    if (odom_ && now - odom_->first <= odom_timeout_s_) sensor = odom_->second;
    std::vector<Win> keep;
    const double keep_s = std::max(window_s_, search_window_s_);   // the spot check uses the last window_s of it
    for (auto& s : scans_)
      if (now - s.t <= keep_s) keep.push_back(std::move(s));
    keep.push_back({now, std::move(P), sensor});
    scans_.swap(keep);
  }

  void on_spot(const geometry_msgs::PointStamped::ConstPtr& m) {
    bool fresh;
    {
      std::lock_guard<std::mutex> l(mx_);
      double now = mono();
      fresh = !spot_ || now - spot_->t > spot_timeout_s_;
      spot_ = Spot{now, m->point.x, m->point.y, m->point.z};
    }
    if (fresh) tick();   // a new request: answer at once
  }

  void delete_markers() {
    if (!markers_up_) return;
    visualization_msgs::Marker m;
    m.action = visualization_msgs::Marker::DELETEALL;
    m.header.frame_id = frame_;
    m.header.stamp = ros::Time::now();
    visualization_msgs::MarkerArray a;
    a.markers.push_back(m);
    pub_markers_.publish(a);
    markers_up_ = false;
  }

  void tick() {
    std::unique_lock<std::mutex> busy(busy_, std::try_to_lock);
    if (!busy.owns_lock()) return;
    Spot sp;
    std::optional<double> yaw;
    std::optional<std::pair<double, Eigen::Vector3d>> od;
    std::vector<Scan> scans;
    std::vector<GroundSpot> spots;
    std::optional<std::array<double, 3>> anchor;
    size_t nscans;
    std::string frame;
    {
      std::lock_guard<std::mutex> l(mx_);
      if (!spot_ || mono() - spot_->t > spot_timeout_s_) {
        delete_markers();
        return;
      }
      sp = *spot_;
      yaw = yaw_;
      od = odom_;
      const double now = mono();
      for (auto& s : scans_)
        if (now - s.t <= window_s_) scans.push_back({s.pts, s.sensor});
      nscans = scans.size();
      spots = ground_spots_;
      anchor = anchor_;
      frame = frame_;
    }
    double t0 = mono();
    bool clear;
    Json st;
    if (!yaw) {
      clear = false;
      st["spot"] = {tc::pyround(sp.x, 2), tc::pyround(sp.y, 2)};
      st["centre"] = {tc::pyround(sp.x, 2), tc::pyround(sp.y, 2)};
      st["radius_m"] = tc::pyround(check_.radius, 3);
      st["reason"] = "no heading: no /planner/odom yet";
      check_.viz = Viz();
    } else {
      if (scans.empty()) scans.push_back(Scan{});
      std::vector<KnownPose> known;
      if (!spots.empty() && anchor) known = ground_spots_to_planner(spots, (*anchor)[0], (*anchor)[1], (*anchor)[2]);
      std::optional<double> descend;
      if (sp.z > 0) descend = sp.z;
      std::tie(clear, st) = check_.evaluate_scans(scans, V2(sp.x, sp.y), descend, yaw, known);
      st["scans"] = int(nscans);
      if (!spots.empty())
        st["known_floor"] = anchor ? std::to_string(spots.size()) + " ground spots" : std::string("no anchor");
    }
    st["eval_ms"] = tc::pyround(1000 * (mono() - t0), 1);
    std_msgs::Bool b;
    b.data = clear;
    pub_clear_.publish(b);
    std_msgs::String s;
    s.data = st.dump();
    pub_status_.publish(s);
    std::lock_guard<std::mutex> l(mx_);
    if (od && std::hypot(od->second.x() - sp.x, od->second.y() - sp.y) > marker_max_dist_) {
      delete_markers();
      return;
    }
    try {
      double fallback = od ? od->second.z() - 1.0 : sp.z;   // no floor known: ~1 m under the robot
      pub_markers_.publish(build_markers(clear, st, check_.viz, frame, ros::Time::now(), fallback));
      markers_up_ = true;
    } catch (const std::exception& e) {   // the markers never stop the answer
      ROS_WARN_THROTTLE(10.0, "landing check: markers failed: %s", e.what());
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "landing_check");
  LandingCheckNode node;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
