// click_goal (click and run, 2026-10-30): clicks in a 3D view (Foxglove's 3D panel publish tool, RViz's tools)
// -> goals for the mission executive's interactive mode, which runs them with all its checks (the
// executive decides takeoffs, landings and modes; this node commands nothing on the vehicle).
//
// In:   ~drive_topic  (geometry_msgs/PointStamped, default /click_and_run/drive)        a point click: drive there
//       ~fly_topic    (geometry_msgs/PoseStamped, default /click_and_run/fly)  a pose click: fly there,
//                     ~fly_height above the floor, facing the pose's heading (~fly_use_yaw)
//       /planner_bridge/anchor (fc <- planner), /mavros/local_position/pose, /mission_executive/ground_spots,
//       /mission_executive/status
// Out:  /mission_executive/goal (std_msgs/String JSON, fc frame; see l4_click/click_goal.h)
//       ~marker_topic (visualization_msgs/MarkerArray, default /click_and_run/goal_marker, planner frame): the
//                     goal and its state (yellow sent, orange running, green reached, red failed / refused /
//                     stopped, grey replaced), 2 Hz
//       /click_and_run/status (std_msgs/String JSON, latched): the last click and what became of it
//
// A click's frame: the planner frame (~planner_frame, "world") as is; any other frame through TF into it
// (none: refused). The floor (for ~fly_height): the newest ground spot the executive recorded (where the
// vehicle stood before a takeoff / after a landing), else the FC's height when this node first heard it (the
// vehicle starts on the ground). A new click replaces the running goal (the executive's rule).
//
// The topics are this node's own, not RViz's /clicked_point and /move_base_simple/goal: MAVROS publishes
// on /move_base_simple/goal (frame map; seen in SITL 2026-10-30), which must never become a goal.
#include <cmath>
#include <mutex>
#include <optional>
#include <string>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <l4_common/params.h>
#include <l4_common/pyjson.h>

#include "l4_click/click_goal.h"

namespace l4_click {

class ClickGoal {
 public:
  ClickGoal() : pnh_("~"), tf_listener_(tf_buffer_) {
    planner_frame_ = l4::param_string(pnh_, "planner_frame", "world");
    fly_height_ = l4::param_double(pnh_, "fly_height", 1.0);
    fly_use_yaw_ = l4::param_bool(pnh_, "fly_use_yaw", true);
    subs_.push_back(nh_.subscribe(l4::param_string(pnh_, "drive_topic", "/click_and_run/drive"), 5, &ClickGoal::on_point, this));
    subs_.push_back(nh_.subscribe(l4::param_string(pnh_, "fly_topic", "/click_and_run/fly"), 5, &ClickGoal::on_pose, this));
    subs_.push_back(nh_.subscribe("/planner_bridge/anchor", 5, &ClickGoal::on_anchor, this));
    subs_.push_back(nh_.subscribe("/mavros/local_position/pose", 5, &ClickGoal::on_fc_pose, this));
    subs_.push_back(nh_.subscribe("/mission_executive/ground_spots", 5, &ClickGoal::on_ground, this));
    subs_.push_back(nh_.subscribe("/mission_executive/status", 50, &ClickGoal::on_status, this));
    goal_pub_ = nh_.advertise<std_msgs::String>("/mission_executive/goal", 5);
    status_pub_ = nh_.advertise<std_msgs::String>("/click_and_run/status", 5, true);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        l4::param_string(pnh_, "marker_topic", "/click_and_run/goal_marker"), 2, true);
    timer_ = nh_.createTimer(ros::Duration(0.5), [this](const ros::TimerEvent&) { publish_marker(); });
    ROS_INFO("click_goal: point clicks drive, pose clicks fly %.2f m above the floor (frame %s)", fly_height_,
             planner_frame_.c_str());
  }

 private:
  void on_anchor(const geometry_msgs::TransformStamped::ConstPtr& m) {
    const auto& q = m->transform.rotation;
    const auto& t = m->transform.translation;
    std::lock_guard<std::mutex> g(lock_);
    anchor_ = Anchor::from(q.x, q.y, q.z, q.w, Vec3(t.x, t.y, t.z));
  }

  void on_fc_pose(const geometry_msgs::PoseStamped::ConstPtr& m) {
    std::lock_guard<std::mutex> g(lock_);
    if (!start_z_) start_z_ = m->pose.position.z;
  }

  void on_ground(const std_msgs::String::ConstPtr& m) {
    const l4::ojson d = l4::loads(m->data);
    const l4::ojson spots = d.is_object() ? l4::get(d, "spots") : l4::ojson();
    if (!spots.is_array() || spots.empty()) return;
    const l4::ojson xyz = l4::get(spots.back(), "xyz");
    if (!xyz.is_array() || xyz.size() < 3 || !xyz[2].is_number()) return;
    std::lock_guard<std::mutex> g(lock_);
    ground_z_ = xyz[2].get<double>();
  }

  void on_status(const std_msgs::String::ConstPtr& m) {
    const l4::ojson s = l4::loads(m->data);
    if (s.is_discarded() || !s.is_object()) return;
    std::lock_guard<std::mutex> g(lock_);
    if (!goal_) return;
    if (state_ == "sent") {
      // the executive's answer to the goal we sent last (it answers each goal at once, in order)
      if (l4::get(s, "goal_received").is_number_integer()) {
        id_ = l4::get(s, "goal_received").get<int>();
        state_ = "taken";
      } else if (l4::get(s, "goal_rejected").is_string()) {
        state_ = "rejected";
        detail_ = l4::get(s, "goal_rejected").get<std::string>();
      }
    } else if (id_) {
      if (auto st = goal_state(s, *id_)) {
        state_ = *st;
        detail_ = l4::get(s, "phase").get<std::string>();
        const l4::ojson r = l4::get(s, "reason");
        if (r.is_string()) detail_ += ": " + r.get<std::string>();
      }
    }
  }

  // a click's position (and heading) in the planner frame, or none
  std::optional<std::pair<Vec3, double>> to_planner(const geometry_msgs::PoseStamped& in) {
    if (in.header.frame_id.empty() || in.header.frame_id == planner_frame_) {
      const auto& p = in.pose.position;
      const auto& q = in.pose.orientation;
      return std::make_pair(Vec3(p.x, p.y, p.z), quat_yaw(q.x, q.y, q.z, q.w));
    }
    try {
      geometry_msgs::PoseStamped out;
      geometry_msgs::PoseStamped latest = in;
      latest.header.stamp = ros::Time(0);   // the newest transform: a click is a place, not a moment
      tf_buffer_.transform(latest, out, planner_frame_, ros::Duration(0.2));
      const auto& p = out.pose.position;
      const auto& q = out.pose.orientation;
      return std::make_pair(Vec3(p.x, p.y, p.z), quat_yaw(q.x, q.y, q.z, q.w));
    } catch (const tf2::TransformException& e) {
      refuse("no transform from " + in.header.frame_id + " to " + planner_frame_ + ": " + e.what());
      return std::nullopt;
    }
  }

  void on_point(const geometry_msgs::PointStamped::ConstPtr& m) {
    geometry_msgs::PoseStamped p;
    p.header = m->header;
    p.pose.position = m->point;
    p.pose.orientation.w = 1.0;
    const auto c = to_planner(p);
    if (!c) return;
    std::lock_guard<std::mutex> g(lock_);
    if (!anchor_) return refuse_locked("no /planner_bridge/anchor yet (the planner bridge is not up)");
    send_locked(drive_goal(c->first, *anchor_), c->first);
  }

  void on_pose(const geometry_msgs::PoseStamped::ConstPtr& m) {
    const auto c = to_planner(*m);
    if (!c) return;
    std::lock_guard<std::mutex> g(lock_);
    if (!anchor_) return refuse_locked("no /planner_bridge/anchor yet (the planner bridge is not up)");
    const std::optional<double> floor = ground_z_ ? ground_z_ : start_z_;
    if (!floor) return refuse_locked("no floor height yet (no FC pose)");
    send_locked(fly_goal(c->first, c->second, *anchor_, *floor, fly_height_, fly_use_yaw_), c->first);
  }

  void send_locked(const l4::ojson& goal, const Vec3& click) {
    goal_ = goal;
    click_ = click;
    state_ = "sent";
    detail_.clear();
    id_.reset();
    std_msgs::String s;
    s.data = l4::dumps(goal);
    goal_pub_.publish(s);
    ROS_INFO("click_goal: %s", s.data.c_str());
    publish_status_locked();
  }

  void refuse(const std::string& why) {
    std::lock_guard<std::mutex> g(lock_);
    refuse_locked(why);
  }
  void refuse_locked(const std::string& why) {
    ROS_WARN("click_goal: click refused: %s", why.c_str());
    std_msgs::String s;
    s.data = l4::dumps(l4::ojson{{"refused", why}});
    status_pub_.publish(s);
  }

  void publish_status_locked() {
    std_msgs::String s;
    s.data = l4::dumps(l4::ojson{{"goal", *goal_}, {"state", state_}, {"detail", detail_},
                                 {"id", id_ ? l4::ojson(*id_) : l4::ojson(nullptr)}});
    status_pub_.publish(s);
  }

  void publish_marker() {
    std::lock_guard<std::mutex> g(lock_);
    if (!goal_ || !anchor_) return;
    if (state_ != last_state_ || detail_ != last_detail_) {
      publish_status_locked();
      last_state_ = state_;
      last_detail_ = detail_;
    }
    // the goal in the planner frame now (the anchor may have moved since the click)
    const l4::ojson& gg = (*goal_)["goal"];
    const bool fly = (*goal_)["type"] == "fly";
    const double floor_fc = ground_z_ ? *ground_z_ : (start_z_ ? *start_z_ : 0.0);
    const Vec3 fc(gg[0].get<double>(), gg[1].get<double>(), fly ? gg[2].get<double>() : floor_fc);
    const Vec3 p = anchor_->to_planner(fc);
    const Vec3 floor_p = anchor_->to_planner(Vec3(fc[0], fc[1], floor_fc));
    float r = 1.0f, gr = 0.85f, b = 0.1f;   // sent / taken: yellow
    if (state_ == "active") r = 1.0f, gr = 0.5f, b = 0.0f;
    else if (state_ == "reached") r = 0.2f, gr = 0.85f, b = 0.3f;
    else if (state_ == "failed" || state_ == "rejected" || state_ == "stopped") r = 0.95f, gr = 0.15f, b = 0.15f;
    else if (state_ == "replaced") r = gr = b = 0.55f;
    visualization_msgs::MarkerArray arr;
    auto base = [&](int id, int type) {
      visualization_msgs::Marker m;
      m.header.frame_id = planner_frame_;
      m.header.stamp = ros::Time::now();
      m.ns = "click_goal";
      m.id = id;
      m.type = type;
      m.action = visualization_msgs::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.color.r = r;
      m.color.g = gr;
      m.color.b = b;
      m.color.a = 0.9f;
      return m;
    };
    // the goal: a disc on the floor (drive) or a ball at the flight height over a post (fly)
    auto spot = base(0, fly ? visualization_msgs::Marker::SPHERE : visualization_msgs::Marker::CYLINDER);
    spot.pose.position.x = p[0];
    spot.pose.position.y = p[1];
    spot.pose.position.z = fly ? p[2] : floor_p[2] + 0.03;
    spot.scale.x = spot.scale.y = fly ? 0.35 : 0.6;
    spot.scale.z = fly ? 0.35 : 0.06;
    arr.markers.push_back(spot);
    auto post = base(1, visualization_msgs::Marker::LINE_LIST);
    post.scale.x = 0.03;
    if (fly) {
      geometry_msgs::Point a, c;
      a.x = c.x = p[0];
      a.y = c.y = p[1];
      a.z = floor_p[2];
      c.z = p[2];
      post.points = {a, c};
    } else {
      post.action = visualization_msgs::Marker::DELETE;
    }
    arr.markers.push_back(post);
    // the heading a fly goal ends with
    auto arrow = base(2, visualization_msgs::Marker::ARROW);
    const l4::ojson yaw = l4::get(*goal_, "yaw");
    if (fly && yaw.is_number()) {
      const double y = yaw.get<double>() * M_PI / 180.0 - anchor_->yaw();   // fc heading -> planner heading
      arrow.pose.position.x = p[0];
      arrow.pose.position.y = p[1];
      arrow.pose.position.z = p[2];
      arrow.pose.orientation.z = std::sin(y / 2);
      arrow.pose.orientation.w = std::cos(y / 2);
      arrow.scale.x = 0.7;
      arrow.scale.y = arrow.scale.z = 0.08;
    } else {
      arrow.action = visualization_msgs::Marker::DELETE;
    }
    arr.markers.push_back(arrow);
    auto text = base(3, visualization_msgs::Marker::TEXT_VIEW_FACING);
    text.pose.position.x = p[0];
    text.pose.position.y = p[1];
    text.pose.position.z = p[2] + 0.45;
    text.scale.z = 0.22;
    text.color.r = text.color.g = text.color.b = 1.0f;
    text.text = std::string(fly ? "fly" : "drive") + (id_ ? " #" + std::to_string(*id_) : "") + ": " + state_ +
                (detail_.empty() ? "" : " (" + detail_ + ")");
    arr.markers.push_back(text);
    marker_pub_.publish(arr);
  }

  ros::NodeHandle nh_, pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string planner_frame_;
  double fly_height_;
  bool fly_use_yaw_;
  std::vector<ros::Subscriber> subs_;
  ros::Publisher goal_pub_, status_pub_, marker_pub_;
  ros::Timer timer_;

  std::mutex lock_;
  std::optional<Anchor> anchor_;
  std::optional<double> start_z_, ground_z_;
  std::optional<l4::ojson> goal_;   // the last goal sent (fc)
  Vec3 click_ = Vec3::Zero();
  std::string state_, detail_, last_state_, last_detail_;
  std::optional<int> id_;           // the executive's number for it
};

}  // namespace l4_click

int main(int argc, char** argv) {
  ros::init(argc, argv, "click_goal");
  l4_click::ClickGoal node;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
