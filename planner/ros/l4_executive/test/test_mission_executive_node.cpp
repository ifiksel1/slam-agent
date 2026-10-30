// Scenario tests of the mission executive against a fake MAVROS, a fake bridge and a fake SUPER / track
// planner (formerly planner/test/test_mission_executive.py: the same world, scenarios and 44 checks).
//
// No SITL: this process is the vehicle. It serves /mavros/cmd/command (DO_SET_MODE), /mavros/cmd/takeoff
// and /mavros/cmd/land, and also the arming and parameter services so that any call to them is caught. It
// publishes /mavros/state, /mavros/local_position/pose and the takeoff/landing status texts, plays the
// bridge (/planner_bridge/anchor, /planner_bridge/status; it moves only while /planner_bridge/enable, and
// only toward the target of the planner its mode (/planner_bridge/set_mode, reported back in its status)
// listens to), SUPER (flies toward each /planning/click_goal converted planner -> fc with the current
// anchor) and the track planner (drives toward /planning/drive_goal on the ground, /track_planner/status).
// Like QuadRover it refuses a takeoff while the tracks move faster than 0.2 m/s
// (MAV_RESULT_TEMPORARILY_REJECTED); a slower creep (creep) passes that guard, and the executive must not
// take off then. It records the executive's /mission_executive/ground_spots. Each scenario runs the real
// executive (the mission_executive binary) as a child process.
#include <gtest/gtest.h>

#include <fcntl.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/StatusText.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include <l4_common/pyjson.h>

using l4::ojson;
using Vec3 = Eigen::Vector3d;

// a subscription with a lambda callback
template <class M, class F>
ros::Subscriber sub(ros::NodeHandle& nh, const std::string& topic, int q, F f) {
  return nh.subscribe<M>(topic, q, boost::function<void(const boost::shared_ptr<M const>&)>(f));
}

namespace {

double wall() { return ros::WallTime::now().toSec(); }
void wsleep(double s) { std::this_thread::sleep_for(std::chrono::duration<double>(s)); }

std::string vstr(const Vec3& v) {
  char b[96];
  std::snprintf(b, sizeof b, "[%.2f, %.2f, %.2f]", v[0], v[1], v[2]);
  return b;
}

bool allclose(double a, double b, double atol) { return std::abs(a - b) <= atol + 1e-5 * std::abs(b); }

struct World {
  std::mutex m;
  // records
  std::vector<std::string> forbidden;             // arming, set_mode by name, parameter calls
  std::vector<std::pair<double, int>> mode_cmds;  // (time, mode number)
  std::vector<std::pair<double, Vec3>> goals;     // (time, planner xyz)
  std::vector<std::pair<double, Vec3>> drive_goals;
  std::vector<ojson> status;                      // executive status JSON
  std::vector<double> status_t;                   // its receive times
  std::optional<std_srvs::Trigger::Response> start_reply;
  // state
  Vec3 pos = Vec3::Zero();
  std::optional<Vec3> target;
  int mode = 0;
  bool armed = true, connected = true;
  std::optional<double> climb_to;
  bool landing = false;
  std::set<int> block;   // goal indices SUPER cannot reach
  int goal_count = 0;
  Vec3 anchor_t = Vec3::Zero();
  double anchor_yaw = 0.0;
  ojson bridge_state = {{"state", "OK"}};
  bool landing_clear = true;
  bool refuse_takeoff = false;
  bool enabled = false;
  int enabled_during_fc_move = 0;              // steps the bridge was enabled during the FC's takeoff/landing
  std::optional<std::string> bridge_mode = std::string("flight");   // none: a bridge that does not report its mode
  bool bridge_stuck = false;                   // never switches mode
  std::optional<std::string> target_kind;      // "flight" (SUPER) or "drive" (the track planner)
  int mode_mismatch = 0;                       // steps enabled with the bridge in the wrong mode for the vehicle
  int drive_steps = 0;                         // steps the fake moved on the ground (bridge in drive mode)
  double drive_short = 0.0;                    // the track planner stops this short of the goal and says REACHED
  int refuse_temp = 0;                         // refuse this many takeoffs "temporarily" (result 1)
  double speed = 0.0;
  int coast = 0;                               // steps the tracks keep moving after the stream stops
  std::vector<std::pair<double, int>> takeoff_calls;   // (speed at the call, result)
  double creep = 0.0;                          // m/s the tracks creep on the ground whatever is commanded
  ojson ground = nullptr;                      // the executive's latest ground spots
  double yaw = 0.0;                            // the vehicle's yaw (fc frame)
  std::optional<double> yaw_cmd;               // the executive's waypoint yaw (/planner_bridge/yaw_cmd)
  std::vector<double> yaw_cmds;                // every yaw_cmd received (NaN: cleared)
  bool yaw_wobble = false;                     // the yaw swings +-20 deg at 0.5 Hz in the air: never settles
  double yaw_travel = 0.0;                     // total yaw turned, rad
  std::optional<double> yaw_at_land;           // the yaw when NAV_LAND came
  std::vector<double> yaw_at_goal;             // the yaw when each click_goal came
  double max_z = 0.0;                          // the highest the vehicle has been (m)

  ros::NodeHandle nh;
  std::vector<ros::ServiceServer> srvs;
  std::vector<ros::Subscriber> subs;
  ros::Publisher state_pub, pose_pub, text_pub, status_pub, anchor_pub, clear_pub, track_pub;
  ros::WallTimer timer;

  void reset() {
    std::lock_guard<std::mutex> g(m);
    pos = Vec3::Zero();
    max_z = 0.0;
    target.reset();
    mode = 0;
    armed = connected = true;
    climb_to.reset();
    landing = false;
    block.clear();
    goal_count = 0;
    anchor_t = Vec3::Zero();
    anchor_yaw = 0.0;
    bridge_state = {{"state", "OK"}};
    landing_clear = true;
    refuse_takeoff = false;
    enabled = false;
    enabled_during_fc_move = 0;
    bridge_mode = std::string("flight");
    bridge_stuck = false;
    target_kind.reset();
    mode_mismatch = 0;
    drive_steps = 0;
    drive_short = 0.0;
    refuse_temp = 0;
    speed = 0.0;
    coast = 0;
    takeoff_calls.clear();
    creep = 0.0;
    ground = nullptr;
    yaw = 0.0;
    yaw_cmd.reset();
    yaw_cmds.clear();
    yaw_wobble = false;
    yaw_travel = 0.0;
    yaw_at_land.reset();
    yaw_at_goal.clear();
    forbidden.clear();
    mode_cmds.clear();
    goals.clear();
    drive_goals.clear();
    status.clear();
    status_t.clear();
  }

  template <class Srv>
  void forbid(const std::string& topic, const std::string& name) {
    srvs.push_back(nh.advertiseService<typename Srv::Request, typename Srv::Response>(
        topic, [this, name](typename Srv::Request&, typename Srv::Response&) {
          std::lock_guard<std::mutex> g(m);
          forbidden.push_back(name);
          return true;
        }));
  }

  void start() {
    srvs.push_back(nh.advertiseService("/mavros/cmd/command", &World::on_command, this));
    srvs.push_back(nh.advertiseService("/mavros/cmd/takeoff", &World::on_takeoff, this));
    srvs.push_back(nh.advertiseService("/mavros/cmd/land", &World::on_land, this));
    forbid<mavros_msgs::CommandBool>("/mavros/cmd/arming", "arming");
    forbid<mavros_msgs::SetMode>("/mavros/set_mode", "set_mode");
    forbid<mavros_msgs::ParamSet>("/mavros/param/set", "param/set");
    forbid<mavros_msgs::ParamGet>("/mavros/param/get", "param/get");
    state_pub = nh.advertise<mavros_msgs::State>("/mavros/state", 10);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10);
    text_pub = nh.advertise<mavros_msgs::StatusText>("/mavros/statustext/recv", 10);
    status_pub = nh.advertise<std_msgs::String>("/planner_bridge/status", 10);
    anchor_pub = nh.advertise<geometry_msgs::TransformStamped>("/planner_bridge/anchor", 10);
    clear_pub = nh.advertise<std_msgs::Bool>("/planner/landing_clear", 10);
    track_pub = nh.advertise<std_msgs::String>("/track_planner/status", 10);
    subs.push_back(nh.subscribe("/planning/click_goal", 10, &World::on_goal, this));
    subs.push_back(nh.subscribe("/planning/drive_goal", 10, &World::on_drive_goal, this));
    subs.push_back(sub<std_msgs::String>(nh, "/mission_executive/status", 100, [this](const std_msgs::String::ConstPtr& s) {
      std::lock_guard<std::mutex> g(m);
      status.push_back(l4::loads(s->data));
      status_t.push_back(wall());
    }));
    subs.push_back(sub<std_msgs::Bool>(nh, "/planner_bridge/enable", 10, [this](const std_msgs::Bool::ConstPtr& b) {
      std::lock_guard<std::mutex> g(m);
      enabled = b->data;
    }));
    subs.push_back(sub<std_msgs::String>(nh, "/planner_bridge/set_mode", 10, [this](const std_msgs::String::ConstPtr& s) {
      std::lock_guard<std::mutex> g(m);
      if (!bridge_stuck) bridge_mode = s->data;
    }));
    subs.push_back(sub<std_msgs::Float64>(nh, "/planner_bridge/yaw_cmd", 10, [this](const std_msgs::Float64::ConstPtr& y) {
      std::lock_guard<std::mutex> g(m);
      if (std::isfinite(y->data)) yaw_cmd = y->data;
      else yaw_cmd.reset();
      yaw_cmds.push_back(y->data);
    }));
    subs.push_back(sub<std_msgs::String>(nh, "/mission_executive/ground_spots", 10, [this](const std_msgs::String::ConstPtr& s) {
      std::lock_guard<std::mutex> g(m);
      ground = l4::loads(s->data);
    }));
    timer = nh.createWallTimer(ros::WallDuration(0.05), [this](const ros::WallTimerEvent&) { step(); });
  }

  // ---- services --------------------------------------------------------------------------------
  bool on_command(mavros_msgs::CommandLong::Request& req, mavros_msgs::CommandLong::Response& res) {
    std::lock_guard<std::mutex> g(m);
    if (static_cast<int>(req.command) != 176) {
      forbidden.push_back("command " + std::to_string(req.command));
      res.success = false;
      res.result = 4;
      return true;
    }
    mode = static_cast<int>(req.param2);
    mode_cmds.emplace_back(wall(), mode);
    res.success = true;
    res.result = 0;
    return true;
  }

  bool on_takeoff(mavros_msgs::CommandTOL::Request& req, mavros_msgs::CommandTOL::Response& res) {
    std::lock_guard<std::mutex> g(m);
    const bool moving = speed > 0.2;
    if (mode != 15 || !armed || refuse_takeoff) {
      takeoff_calls.emplace_back(speed, 4);
      res.success = false;
      res.result = 4;
      return true;
    }
    if (moving || refuse_temp > 0) {
      // QuadRover: "Guided takeoff refused: moving" (MAV_RESULT_TEMPORARILY_REJECTED)
      refuse_temp = std::max(0, refuse_temp - 1);
      takeoff_calls.emplace_back(speed, 1);
      res.success = false;
      res.result = 1;
      return true;
    }
    takeoff_calls.emplace_back(speed, 0);
    target.reset();
    climb_to = pos[2] + req.altitude;
    res.success = true;
    res.result = 0;
    return true;
  }

  bool on_land(mavros_msgs::CommandTOL::Request&, mavros_msgs::CommandTOL::Response& res) {
    std::lock_guard<std::mutex> g(m);
    if (mode != 15) {
      res.success = false;
      res.result = 4;
      return true;
    }
    landing = true;
    yaw_at_land = yaw;
    target.reset();
    res.success = true;
    res.result = 0;
    return true;
  }

  // ---- fake SUPER: fly toward the goal, converted planner -> fc with the anchor ------------------
  Vec3 planner_to_fc(const Vec3& p) const {
    const double c = std::cos(anchor_yaw), s = std::sin(anchor_yaw);
    return Vec3(c * p[0] - s * p[1], s * p[0] + c * p[1], p[2]) + anchor_t;
  }

  void on_goal(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> g(m);
    const Vec3 p(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    goals.emplace_back(wall(), p);
    yaw_at_goal.push_back(yaw);
    const int idx = goal_count++;
    if (block.count(idx)) target.reset();
    else target = planner_to_fc(p);
    target_kind = std::string("flight");
  }

  void on_drive_goal(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> g(m);
    const Vec3 p(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    drive_goals.emplace_back(wall(), p);
    const Vec3 q = planner_to_fc(p);
    const int idx = goal_count++;
    if (block.count(idx)) target.reset();
    else target = Vec3(q[0], q[1], pos[2]);
    target_kind = std::string("drive");
    publish_track("DRIVING", "new goal");
  }

  void publish_track(const std::string& state, const std::string& reason) {
    std_msgs::String s;
    s.data = l4::dumps(ojson{{"state", state}, {"reason", reason}});
    track_pub.publish(s);
  }

  void publish_text(const std::string& t) {
    mavros_msgs::StatusText st;
    st.severity = 6;
    st.text = t;
    text_pub.publish(st);
  }

  void step() {
    std::lock_guard<std::mutex> g(m);
    const double dt = 0.05;
    std::optional<double> step_dir;   // this step's travel direction
    if ((climb_to || landing) && enabled) enabled_during_fc_move += 1;
    const bool kind_ok = !bridge_mode || (target_kind && *bridge_mode == *target_kind);
    // the bridge's waypoint hold: in the air with a waypoint yaw set it holds, it does not fly on to SUPER's goal
    const bool waypoint_hold = pos[2] >= 0.05 && yaw_cmd && target_kind && *target_kind == "flight";
    if (climb_to) {
      pos[2] = std::min(*climb_to, pos[2] + 1.5 * dt);
      if (pos[2] >= *climb_to - 1e-6) {
        climb_to.reset();
        publish_text("Guided takeoff complete");
      }
    } else if (landing) {
      pos[2] = std::max(0.0, pos[2] - 1.0 * dt);
      if (pos[2] <= 0.0) {
        landing = false;
        publish_text("Guided landing complete");
      }
    } else if (target && mode == 15 && enabled && l4::get(bridge_state, "state") == "OK" && kind_ok && !waypoint_hold) {
      const Vec3 d = *target - pos;
      double n = d.norm();
      const bool drive = target_kind && *target_kind == "drive";
      if (drive) n -= drive_short;
      if (n > 1e-6) {
        const Vec3 st = d / d.norm() * std::min(n, 2.0 * dt);
        if (std::hypot(st[0], st[1]) > 1e-6) step_dir = std::atan2(st[1], st[0]);   // a vertical move: none
        pos += st;
        speed = st.norm() / dt;
        coast = drive ? 20 : 0;
        if (drive) drive_steps += 1;
      } else if (drive) {
        speed = 0.0;
        publish_track("REACHED", "");
      }
    } else if (creep > 0 && pos[2] < 0.05) {
      pos[0] += creep * dt;
      speed = creep;
    } else if (coast > 0 && pos[2] < 0.05) {
      coast -= 1;   // the tracks run on for a second after the stream stops
      pos[0] += 0.5 * dt;
      speed = 0.5;
    } else {
      speed = 0.0;
    }
    // the fake bridge's flight yaw: the nose along the travel direction while moving (> 0.3 m/s), else the
    // executive's waypoint yaw; 60 deg/s; driving: nose-first
    if (enabled && mode == 15) {
      std::optional<double> want;
      if (speed > 0.3 && step_dir) want = *step_dir;
      else if (pos[2] >= 0.05 && yaw_cmd) want = *yaw_cmd;
      if (want) {
        const double d = std::atan2(std::sin(*want - yaw), std::cos(*want - yaw));
        const double s = std::max(-60.0 * M_PI / 180.0 * dt, std::min(60.0 * M_PI / 180.0 * dt, d));
        yaw = std::atan2(std::sin(yaw + s), std::cos(yaw + s));
        yaw_travel += std::abs(s);
      }
    }
    if (enabled && bridge_mode) {
      const bool on_ground = pos[2] < 0.05;
      if (*bridge_mode != (on_ground ? "drive" : "flight")) mode_mismatch += 1;
    }
    mavros_msgs::State st;
    st.connected = connected;
    st.armed = armed;
    st.guided = true;
    st.mode = mode == 0 ? "MANUAL" : mode == 4 ? "HOLD" : mode == 15 ? "GUIDED" : "CMODE(" + std::to_string(mode) + ")";
    state_pub.publish(st);
    geometry_msgs::PoseStamped ps;
    ps.header.stamp = ros::Time::now();
    ps.header.frame_id = "map";
    ps.pose.position.x = pos[0];
    ps.pose.position.y = pos[1];
    ps.pose.position.z = pos[2];
    max_z = std::max(max_z, pos[2]);
    const double y_pub = yaw + (yaw_wobble && pos[2] >= 0.05 ? 0.35 * std::sin(2 * M_PI * 0.5 * wall()) : 0.0);
    ps.pose.orientation.z = std::sin(y_pub / 2);
    ps.pose.orientation.w = std::cos(y_pub / 2);
    pose_pub.publish(ps);
    ojson bs = bridge_state;
    if (bridge_mode) bs["mode"] = *bridge_mode;
    std_msgs::String s;
    s.data = l4::dumps(bs);
    status_pub.publish(s);
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = ros::Time::now();
    tf.header.frame_id = "fc";
    tf.child_frame_id = "planner";
    tf.transform.translation.x = anchor_t[0];
    tf.transform.translation.y = anchor_t[1];
    tf.transform.translation.z = anchor_t[2];
    tf.transform.rotation.z = std::sin(anchor_yaw / 2);
    tf.transform.rotation.w = std::cos(anchor_yaw / 2);
    anchor_pub.publish(tf);
    std_msgs::Bool b;
    b.data = landing_clear;
    clear_pub.publish(b);
  }

  std::vector<std::string> phases() {
    std::lock_guard<std::mutex> g(m);
    std::vector<std::string> out;
    for (const auto& s : status) out.push_back(l4::get(s, "phase", "").get<std::string>());
    return out;
  }
  std::vector<int> modes() {
    std::lock_guard<std::mutex> g(m);
    std::vector<int> out;
    for (const auto& x : mode_cmds) out.push_back(x.second);
    return out;
  }
  ojson last_status() {
    std::lock_guard<std::mutex> g(m);
    return status.empty() ? ojson::object() : status.back();
  }
};

World* W = nullptr;
int CHECKS = 0, FAILED = 0;

std::string join(const std::vector<std::string>& v) {
  std::string o;
  for (size_t i = 0; i < v.size(); ++i) o += (i ? "," : "") + v[i];
  return o;
}
std::string join(const std::vector<int>& v) {
  std::string o = "[";
  for (size_t i = 0; i < v.size(); ++i) o += (i ? ", " : "") + std::to_string(v[i]);
  return o + "]";
}
bool last_is(const std::vector<std::string>& ph, const char* p) { return !ph.empty() && ph.back() == p; }
bool contains(const std::vector<std::string>& ph, const char* p) { return std::find(ph.begin(), ph.end(), p) != ph.end(); }

void check(const std::string& name, bool ok, const std::string& detail = "") {
  ++CHECKS;
  if (!ok) ++FAILED;
  std::printf("%s  %-58s %s\n", ok ? "PASS" : "FAIL", name.c_str(), detail.c_str());
  std::fflush(stdout);
  EXPECT_TRUE(ok) << name << ": " << detail;
}

std::string mission_file(const std::vector<ojson>& legs, ojson defaults = ojson::object()) {
  ojson d = {{"speed", 1.0}, {"accept_radius", 0.3}, {"leg_timeout_s", 30}, {"no_progress_s", 3}};
  for (auto it = defaults.begin(); it != defaults.end(); ++it) d[it.key()] = it.value();
  std::string text = "name: test\nframe: fc\ntakeoff_height: 1.0\ndefaults: " + l4::dumps(d) + "\nlegs:" + std::string(legs.empty() ? " []\n" : "\n");   // none: interactive
  for (const auto& l : legs) text += "  - " + l4::dumps(l) + "\n";
  char path[] = "/tmp/l4_mission_XXXXXX.yaml";
  int fd = mkstemps(path, 5);
  if (fd < 0) return "";
  if (write(fd, text.data(), text.size()) < 0) {}
  close(fd);
  return path;
}

std::vector<std::string> run_executive(const std::vector<ojson>& legs, std::function<void()> setup = nullptr,
                                       std::function<void(double)> during = nullptr, double timeout = 40,
                                       bool start = true, ojson defaults = ojson::object(),
                                       const std::vector<std::string>& extra_args = {}) {
  const std::string path = mission_file(legs, defaults);
  std::vector<std::string> args = {MISSION_EXECUTIVE, "__name:=mission_executive", "_mission:=" + path,
                                   "_unreachable_hold_s:=1.0", "_map_rebuild_s:=1.0", "_landing_check_s:=2.0",
                                   "_goal_settle_s:=0.2", "_takeoff_settle_s:=4.0", "_bridge_mode_timeout_s:=1.0"};
  // no re-seed goals in between (the scenarios count goals by index) unless a scenario asks (S27-S30: any
  // _leg_start_seed_* or _planner_stuck_* argument; without _leg_start_seed_dz the node's default, 0.1 m)
  bool seed_arg = false, dz_arg = false;
  for (const auto& a : extra_args) {
    seed_arg = seed_arg || a.rfind("_leg_start_seed_", 0) == 0 || a.rfind("_planner_stuck_", 0) == 0;
    dz_arg = dz_arg || a.rfind("_leg_start_seed_dz:=", 0) == 0;
  }
  if (!seed_arg) args.push_back("_leg_start_seed_dz:=0.0");
  // a private _x:= argument stays on the parameter server after its run: the node's default needs it gone
  else if (!dz_arg) ros::param::del("/mission_executive/leg_start_seed_dz");
  args.insert(args.end(), extra_args.begin(), extra_args.end());
  pid_t pid = fork();
  if (pid == 0) {
    int devnull = open("/dev/null", O_WRONLY);
    dup2(devnull, 1);
    dup2(devnull, 2);
    std::vector<char*> argv;
    for (auto& a : args) argv.push_back(const_cast<char*>(a.c_str()));
    argv.push_back(nullptr);
    execv(argv[0], argv.data());
    _exit(127);
  }
  if (ros::service::waitForService("/mission_executive/start", ros::Duration(20))) {
    if (setup) setup();
    wsleep(0.5);
    if (start) {
      std_srvs::Trigger t;
      if (ros::service::call("/mission_executive/start", t)) {
        std::lock_guard<std::mutex> g(W->m);
        W->start_reply = t.response;
      }
    }
    const double t0 = wall();
    while (wall() - t0 < timeout) {
      auto ph = W->phases();
      if (!ph.empty() && (ph.back() == "DONE" || ph.back() == "STOPPED")) break;
      if (during) during(wall() - t0);
      wsleep(0.05);
    }
  }
  wsleep(0.5);
  kill(pid, SIGTERM);
  int st = 0;
  const double tk = wall();
  while (waitpid(pid, &st, WNOHANG) == 0) {
    if (wall() - tk > 10) {
      kill(pid, SIGKILL);
      waitpid(pid, &st, 0);
      break;
    }
    wsleep(0.05);
  }
  wsleep(1.0);
  unlink(path.c_str());
  return W->phases();
}

const ojson dead_state = {{"state", "HOLD"}, {"reason", "planner_dead"}, {"slam_hold", false}, {"enabled", true},
                          {"planner_age", 0.41}, {"planner_node", "/fsm_node"}, {"planner_alive", false}};

// the bridge's status after a frozen SLAM pose (FrozenPoseCheck): HOLD "slam_frozen", latched
const ojson frozen_state = {{"state", "HOLD"}, {"reason", "slam_frozen"}, {"slam_hold", false}, {"enabled", true},
                            {"slam_frozen", {{"repeats", 9}, {"for_s", 0.91}, {"evidence", "command"}, {"latched", true}}}};

ojson fly(double x, double y, double z) { return {{"type", "fly"}, {"goal", {x, y, z}}}; }
ojson drive(double x, double y) { return {{"type", "drive"}, {"goal", {x, y}}}; }
ojson land(double x, double y) { return {{"type", "land"}, {"at", {x, y}}}; }
ojson takeoff(double h) { return {{"type", "takeoff"}, {"height", h}}; }

}  // namespace

class Executive : public ::testing::Test {
 protected:
  void SetUp() override { W->reset(); }
};

// 1. a normal mission with an offset anchor: every leg type, goals converted fc -> planner
TEST_F(Executive, S01_NormalMission) {
  {
    std::lock_guard<std::mutex> g(W->m);
    W->anchor_t = Vec3(1.0, 2.0, 0.0);
    W->anchor_yaw = M_PI / 2;
  }
  auto ph = run_executive({takeoff(1.0), fly(3.0, 0.0, 1.0), fly(3.0, 3.0, 1.5), land(2.0, 3.0), drive(0.0, 3.0)});
  check("normal mission completes", last_is(ph, "DONE"), join(ph));
  std::lock_guard<std::mutex> g(W->m);
  const Vec3 d = Vec3(3.0, 0.0, 1.0) - W->anchor_t;
  const Vec3 exp(d[1], -d[0], d[2]);   // [[0, 1, 0], [-1, 0, 0], [0, 0, 1]] @ d
  bool ok = !W->goals.empty() && (W->goals[0].second - exp).cwiseAbs().maxCoeff() <= 1e-6 + 1e-5;
  check("goal converted fc -> planner with the anchor", ok,
        "got " + (W->goals.empty() ? std::string("None") : vstr(W->goals[0].second)) + " want " + vstr(exp));
  check("drive leg sent to the track planner", W->drive_goals.size() == 1);
  check("drive leg driven with the bridge in drive mode, never in the wrong mode", W->drive_steps > 0 && W->mode_mismatch == 0,
        "drive steps " + std::to_string(W->drive_steps) + ", mismatched steps " + std::to_string(W->mode_mismatch));
  std::vector<int> modes;
  for (auto& x : W->mode_cmds) modes.push_back(x.second);
  check("modes commanded: GUIDED only", modes == std::vector<int>{15}, join(modes));
  check("never arms or touches parameters", W->forbidden.empty(), join(W->forbidden));
  check("bridge not enabled during the FC's takeoff and landing", W->enabled_during_fc_move == 0,
        std::to_string(W->enabled_during_fc_move) + " steps");
  check("bridge disabled at the end", !W->enabled);
}

// 2. an unreachable goal: QRLOITER, hold, back to GUIDED, next leg
TEST_F(Executive, S02_UnreachableGoal) {
  {
    std::lock_guard<std::mutex> g(W->m);
    W->block = {0};
  }
  auto ph = run_executive({fly(4.0, 0.0, 1.0), fly(0.0, 2.0, 1.0)});
  auto modes = W->modes();
  check("unreachable goal -> UNREACHABLE, then the next leg", contains(ph, "UNREACHABLE") && last_is(ph, "DONE"), join(ph));
  check("unreachable goal -> QRLOITER (19) then GUIDED", modes == std::vector<int>{15, 19, 15}, join(modes));
  std::lock_guard<std::mutex> g(W->m);
  check("never arms or touches parameters (unreachable)", W->forbidden.empty(), join(W->forbidden));
}

// 3. an anchor jump: the bridge holds, the executive waits, then resends with the new anchor
TEST_F(Executive, S03_AnchorJump) {
  double done = 0.0;
  auto jump = [&](double t) {
    std::lock_guard<std::mutex> g(W->m);
    // mid-leg, once the robot is 2 m along: the bridge re-anchors and holds for 1.5 s
    if (W->pos[0] > 2.0 && done == 0.0) {
      done = t;
      W->bridge_state = {{"state", "HOLD"}, {"reason", "anchor_jump"}};
      W->anchor_t = Vec3(0.0, 0.5, 0.0);
    }
    if (done != 0.0 && t > done + 1.5 && l4::get(W->bridge_state, "state") != "OK") W->bridge_state = {{"state", "OK"}};
  };
  auto ph = run_executive({fly(6.0, 0.0, 1.0)}, nullptr, jump);
  check("anchor jump -> HOLD_SLAM, resume, mission completes", contains(ph, "HOLD_SLAM") && last_is(ph, "DONE"), join(ph));
  std::lock_guard<std::mutex> g(W->m);
  const Vec3 want = Vec3(6.0, 0.0, 1.0) - Vec3(0.0, 0.5, 0.0);
  bool resent = W->goals.size() >= 2;
  if (resent)
    for (int i = 0; i < 3; ++i) resent = resent && allclose(W->goals.back().second[i], want[i], 1e-8);
  std::string gs;
  for (auto& x : W->goals) gs += vstr(x.second) + " ";
  check("goal resent after the hold, converted with the new anchor", resent, gs);
}

// 4. the FC takes the vehicle (QRSmartRTL failsafe) mid-leg: the executive stops for good
TEST_F(Executive, S04_FcFailsafe) {
  std::optional<double> taken;
  auto failsafe = [&](double t) {
    if (t > 4.0 && !taken) {
      taken = wall();
      std::lock_guard<std::mutex> g(W->m);
      W->mode = 23;
    }
  };
  auto ph = run_executive({fly(8.0, 0.0, 1.0), fly(0.0, 0.0, 1.0)}, nullptr, failsafe);
  auto last = W->last_status();
  check("FC failsafe -> STOPPED",
        last_is(ph, "STOPPED") && l4::get(last, "reason", "").get<std::string>().find("FC mode") != std::string::npos,
        l4::dumps(last));
  std::lock_guard<std::mutex> g(W->m);
  int after_goals = 0;
  std::vector<int> after_modes;
  for (auto& x : W->goals)
    if (taken && x.first > *taken) ++after_goals;
  for (auto& x : W->mode_cmds)
    if (taken && x.first > *taken) after_modes.push_back(x.second);
  check("no goal or mode command after the failsafe", after_goals == 0 && after_modes.empty(),
        "goals " + std::to_string(after_goals) + " modes " + join(after_modes));
}

// 5. landing spot not reported clear: QRLOITER and stop
TEST_F(Executive, S05_LandingSpotNotClear) {
  {
    std::lock_guard<std::mutex> g(W->m);
    W->landing_clear = false;
  }
  auto ph = run_executive({land(1.0, 0.0)});
  auto modes = W->modes();
  check("landing spot not clear -> QRLOITER, STOPPED",
        last_is(ph, "STOPPED") && std::find(modes.begin(), modes.end(), 19) != modes.end(), join(ph));
  check("no landing on a spot not reported clear", !contains(ph, "LAND"), join(ph));
}

// 5b. not clear at first, clear within landing_check_s (a person walked off): lands
TEST_F(Executive, S05b_SpotClearsWithinTheCheck) {
  {
    std::lock_guard<std::mutex> g(W->m);
    W->landing_clear = false;
  }
  std::optional<double> asked;
  auto walk_off = [&](double) {
    if (contains(W->phases(), "LAND_CHECK")) {
      if (!asked) asked = wall();
      if (wall() - *asked > 1.0) {
        std::lock_guard<std::mutex> g(W->m);
        W->landing_clear = true;
      }
    }
  };
  auto ph = run_executive({land(1.0, 0.0)}, nullptr, walk_off);
  check("spot clear within landing_check_s -> lands, DONE", last_is(ph, "DONE") && contains(ph, "LAND"), join(ph));
}

// 6. start refused while disarmed; nothing commanded
TEST_F(Executive, S06_StartRefusedWhileDisarmed) {
  {
    std::lock_guard<std::mutex> g(W->m);
    W->armed = false;
    W->start_reply.reset();
  }
  run_executive({fly(1.0, 0.0, 1.0)}, nullptr, nullptr, 3);
  std::lock_guard<std::mutex> g(W->m);
  check("start refused while disarmed (the pilot arms)", W->start_reply && !W->start_reply->success,
        W->start_reply ? W->start_reply->message : "");
  check("nothing commanded while disarmed", W->mode_cmds.empty() && W->goals.empty() && W->forbidden.empty());
}

// 7. takeoff refused: stop, no goal sent
TEST_F(Executive, S07_TakeoffRefused) {
  {
    std::lock_guard<std::mutex> g(W->m);
    W->refuse_takeoff = true;
  }
  auto ph = run_executive({takeoff(1.0), fly(1.0, 0.0, 1.0)});
  std::lock_guard<std::mutex> g(W->m);
  check("takeoff refused -> STOPPED, no goal sent", last_is(ph, "STOPPED") && W->goals.empty(), join(ph));
}

// 8. land, drive, take off again (the owner's drive-and-fly mission): the bridge switches modes, the
// takeoff waits for the tracks to stop, SUPER is re-seeded at the hover point
TEST_F(Executive, S08_LandDriveTakeOff) {
  auto ph = run_executive({takeoff(1.0), fly(2.0, 0.0, 1.0), land(2.0, 1.0), drive(4.0, 1.0), drive(4.0, -1.0), takeoff(1.2),
                           fly(0.0, 0.0, 1.2), land(0.0, 0.5)},
                          nullptr, nullptr, 60);
  auto modes = W->modes();
  check("land, drive, take off, fly, land: DONE", last_is(ph, "DONE"), join(ph));
  std::lock_guard<std::mutex> g(W->m);
  check("bridge mode follows the vehicle (drive on the ground, flight in the air)", W->mode_mismatch == 0 && W->drive_steps > 0,
        "mismatched " + std::to_string(W->mode_mismatch) + ", drive steps " + std::to_string(W->drive_steps));
  bool calls_ok = W->takeoff_calls.size() == 2;
  std::string calls;
  for (auto& c : W->takeoff_calls) {
    calls_ok = calls_ok && c.second == 0 && c.first <= 0.2;
    calls += "(" + std::to_string(c.first) + ", " + std::to_string(c.second) + ") ";
  }
  check("takeoff after a drive only once stopped (never refused for moving)", calls_ok, calls);
  int hover = 0;
  std::string gs;
  if (!W->drive_goals.empty())
    for (auto& x : W->goals) {
      gs += vstr(x.second) + " ";
      if (std::abs(x.second[2] - 1.2) < 0.05 && (x.second.head<2>() - W->drive_goals.back().second.head<2>()).norm() < 1.0)
        ++hover;
    }
  check("second takeoff re-seeds SUPER at its hover point", hover >= 1, gs);
  check("modes commanded: GUIDED only (drive and fly)", modes == std::vector<int>{15}, join(modes));
  check("never arms or touches parameters (drive and fly)", W->forbidden.empty(), join(W->forbidden));
  const ojson spots = W->ground.is_object() ? l4::get(W->ground, "spots", ojson::array()) : ojson::array();
  std::vector<std::string> ev;
  for (const auto& s : spots) ev.push_back(s["event"].get<std::string>());
  bool ok = ev == std::vector<std::string>{"takeoff", "landed", "takeoff", "landed"} && l4::get(W->ground, "frame") == "fc";
  if (ok) {
    for (const auto& s : spots) ok = ok && std::abs(s["xyz"][2].get<double>()) < 0.05;
    ok = ok && allclose(spots[0]["xyz"][0].get<double>(), 0, 0.05) && allclose(spots[0]["xyz"][1].get<double>(), 0, 0.05);
    ok = ok && allclose(spots[2]["xyz"][0].get<double>(), 4.0, 0.6) && allclose(spots[2]["xyz"][1].get<double>(), -1.0, 0.6);   // the fake's tracks coast 0.5 m
  }
  check("ground spots: each takeoff spot and each landing, on the ground, fc frame", ok, l4::dumps(spots));
  check("bridge not enabled during the second takeoff and landing", W->enabled_during_fc_move == 0,
        std::to_string(W->enabled_during_fc_move) + " steps");
}

// 9. QuadRover refuses the takeoff "temporarily" twice: asked again, flies
TEST_F(Executive, S09_TakeoffTemporarilyRejected) {
  {
    std::lock_guard<std::mutex> g(W->m);
    W->refuse_temp = 2;
  }
  auto ph = run_executive({takeoff(1.0), fly(1.0, 0.0, 1.0)});
  std::lock_guard<std::mutex> g(W->m);
  std::vector<int> res;
  for (auto& c : W->takeoff_calls) res.push_back(c.second);
  check("takeoff temporarily rejected twice -> asked again, DONE", last_is(ph, "DONE") && res == std::vector<int>{1, 1, 0},
        join(ph) + " " + join(res));
}

// 10. a mission that starts on the ground with a drive: no takeoff first; the track planner's REACHED
// ends the leg (the FC 0.2 m behind its odometry point, outside a 0.1 m accept radius)
TEST_F(Executive, S10_DriveFirstReached) {
  {
    std::lock_guard<std::mutex> g(W->m);
    W->drive_short = 0.2;
  }
  ojson leg = drive(2.0, 0.0);
  leg["accept_radius"] = 0.1;
  auto ph = run_executive({leg});
  std::lock_guard<std::mutex> g(W->m);
  bool reached = false;
  for (const auto& s : W->status)
    if (l4::get(s, "reached_by") == "track planner") reached = true;
  check("drive-first mission: no takeoff, REACHED ends the leg", last_is(ph, "DONE") && W->takeoff_calls.empty() && reached,
        join(ph) + " takeoffs " + std::to_string(W->takeoff_calls.size()));
}

// 11. an unreachable drive goal on the ground: never QRLOITER (it would spool the props), next leg
TEST_F(Executive, S11_UnreachableDriveGoal) {
  {
    std::lock_guard<std::mutex> g(W->m);
    W->block = {0};
  }
  auto ph = run_executive({drive(3.0, 0.0), drive(0.0, 1.0)});
  auto modes = W->modes();
  check("unreachable drive goal -> hold in GUIDED (no QR mode), next leg",
        contains(ph, "UNREACHABLE") && last_is(ph, "DONE") && modes == std::vector<int>{15}, join(ph) + " modes " + join(modes));
}

// 12. the bridge does not switch to drive mode: no drive goal streamed, STOPPED on the ground
TEST_F(Executive, S12_BridgeStuck) {
  {
    std::lock_guard<std::mutex> g(W->m);
    W->bridge_stuck = true;
  }
  auto ph = run_executive({drive(2.0, 0.0)});
  auto last = W->last_status();
  auto modes = W->modes();
  std::lock_guard<std::mutex> g(W->m);
  check("bridge stuck in flight mode -> STOPPED, never enabled, GUIDED only",
        last_is(ph, "STOPPED") &&
            l4::get(last, "reason", "").get<std::string>().find("bridge did not switch") != std::string::npos &&
            W->drive_steps == 0 && modes == std::vector<int>{15},
        l4::dumps(last));
}

// 13. takeoff refused on the ground: no QRLOITER (QuadRover's drive-to-fly transition)
TEST_F(Executive, S13_TakeoffRefusedOnTheGround) {
  {
    std::lock_guard<std::mutex> g(W->m);
    W->refuse_takeoff = true;
  }
  auto ph = run_executive({takeoff(1.0)});
  auto modes = W->modes();
  check("takeoff refused on the ground -> stays GUIDED", last_is(ph, "STOPPED") && modes == std::vector<int>{15}, join(modes));
}

// 14. the tracks creep at 0.1 m/s (under QuadRover's 0.2 m/s guard) and never stand still: no takeoff,
// STOPPED "not stationary before takeoff" on the ground in GUIDED, bridge off
TEST_F(Executive, S14_CreepNoTakeoff) {
  {
    std::lock_guard<std::mutex> g(W->m);
    W->creep = 0.1;
  }
  auto ph = run_executive({takeoff(1.0), fly(1.0, 0.0, 1.0)});
  auto last = W->last_status();
  auto modes = W->modes();
  std::lock_guard<std::mutex> g(W->m);
  check("not standing still before a takeoff -> STOPPED, no takeoff sent",
        last_is(ph, "STOPPED") && l4::get(last, "reason") == "not stationary before takeoff" && W->takeoff_calls.empty(),
        l4::dumps(last) + " takeoffs " + std::to_string(W->takeoff_calls.size()));
  check("not stationary: stays GUIDED (no QR mode on the ground), bridge off, no goal",
        modes == std::vector<int>{15} && !W->enabled && W->goals.empty() && W->forbidden.empty(),
        "modes " + join(modes) + " goals " + std::to_string(W->goals.size()));
  const ojson spots = W->ground.is_object() ? l4::get(W->ground, "spots", ojson::array()) : ojson::array();
  check("not stationary: no ground spot recorded", spots.empty(), l4::dumps(W->ground));
}

// 15. a drive, then the tracks creep on once the bridge is off: no takeoff, STOPPED on the ground
TEST_F(Executive, S15_CreepAfterADrive) {
  auto creep_after_drive = [&](double) {
    std::lock_guard<std::mutex> g(W->m);
    bool drove = false;
    for (const auto& s : W->status)
      if (l4::get(s, "type") == "drive") drove = true;
    if (drove && W->drive_steps > 5) W->creep = 0.1;
  };
  auto ph = run_executive({drive(1.0, 0.0), takeoff(1.0)}, nullptr, creep_after_drive);
  auto last = W->last_status();
  auto modes = W->modes();
  std::lock_guard<std::mutex> g(W->m);
  check("creeping after a drive -> STOPPED not stationary, GUIDED only, no takeoff",
        last_is(ph, "STOPPED") && l4::get(last, "reason") == "not stationary before takeoff" && W->takeoff_calls.empty() &&
            modes == std::vector<int>{15} && !W->enabled,
        l4::dumps(last) + " modes " + join(modes));
}

// 16-17. the planner dies mid-leg (owner 2026-10-29: the robot just stops, no continuation): the bridge
// reports HOLD "planner_dead"; STOPPED "planner died" at once (not after no_progress_s), no next leg even
// with on_unreachable: next. In the air QRLOITER; on the ground GUIDED, bridge off
static std::function<void(double)> planner_dies(double at_x, std::optional<double>& died) {
  return [at_x, &died](double) {
    std::lock_guard<std::mutex> g(W->m);
    if (!died && W->enabled && W->pos[0] > at_x) {
      died = wall();
      W->bridge_state = dead_state;
    }
  };
}

static std::optional<double> stopped_after(const std::optional<double>& t_dead) {
  std::lock_guard<std::mutex> g(W->m);
  std::optional<double> ts;
  for (size_t i = 0; i < W->status.size(); ++i)
    if (l4::get(W->status[i], "phase") == "STOPPED") ts = W->status_t[i];
  if (!ts || !t_dead) return std::nullopt;
  return *ts - *t_dead;
}

TEST_F(Executive, S16_PlannerDiesFlying) {
  std::optional<double> died;
  auto ph = run_executive({fly(4.0, 0.0, 1.0), fly(0.0, 2.0, 1.0)}, nullptr, planner_dies(1.0, died), 40, true,
                          {{"no_progress_s", 20}, {"on_unreachable", "next"}});
  auto last = W->last_status();
  auto dt = stopped_after(died);
  auto modes = W->modes();
  check("planner dies flying -> STOPPED 'planner died' at once",
        last_is(ph, "STOPPED") && l4::get(last, "reason") == "planner died" && dt && *dt < 2.0,
        l4::dumps(last) + ", " + (dt ? std::to_string(*dt) : "None") + " s after");
  std::lock_guard<std::mutex> g(W->m);
  check("planner dies flying -> no next leg, no UNREACHABLE, no retry", W->goals.size() == 1 && !contains(ph, "UNREACHABLE"),
        "goals " + std::to_string(W->goals.size()) + " phases " + join(ph));
  check("planner dies flying -> QRLOITER (19), bridge off, never arms",
        modes == std::vector<int>{15, 19} && !W->enabled && W->forbidden.empty(), "modes " + join(modes));
}

TEST_F(Executive, S17_PlannerDiesDriving) {
  std::optional<double> died;
  auto ph = run_executive({drive(4.0, 0.0), drive(0.0, 1.0)}, nullptr, planner_dies(1.0, died), 40, true,
                          {{"no_progress_s", 20}, {"on_unreachable", "next"}});
  auto last = W->last_status();
  auto dt = stopped_after(died);
  auto modes = W->modes();
  check("planner dies driving -> STOPPED 'planner died' at once",
        last_is(ph, "STOPPED") && l4::get(last, "reason") == "planner died" && dt && *dt < 2.0,
        l4::dumps(last) + ", " + (dt ? std::to_string(*dt) : "None") + " s after");
  std::lock_guard<std::mutex> g(W->m);
  check("planner dies driving -> GUIDED only (no QR mode on the ground), bridge off, no next leg",
        modes == std::vector<int>{15} && !W->enabled && W->drive_goals.size() == 1 && !contains(ph, "UNREACHABLE") &&
            W->takeoff_calls.empty(),
        "modes " + join(modes) + " drive goals " + std::to_string(W->drive_goals.size()) + " phases " + join(ph));
}

// 16b-17b. a frozen SLAM pose mid-leg (L2 faults 2b, 2d): the bridge reports HOLD "slam_frozen"; STOPPED "SLAM
// pose frozen" at once, no next leg. In the air QRLAND (21: the FC's position follows the frozen pose, nothing
// that holds a position can be trusted); on the ground GUIDED, bridge off. Never arms
static std::function<void(double)> slam_freezes(double at_x, std::optional<double>& froze) {
  return [at_x, &froze](double) {
    std::lock_guard<std::mutex> g(W->m);
    if (!froze && W->enabled && W->pos[0] > at_x) {
      froze = wall();
      W->bridge_state = frozen_state;
    }
  };
}

TEST_F(Executive, S16b_SlamFreezesFlying) {
  std::optional<double> froze;
  auto ph = run_executive({fly(4.0, 0.0, 1.0), fly(0.0, 2.0, 1.0)}, nullptr, slam_freezes(1.0, froze), 40, true,
                          {{"no_progress_s", 20}, {"on_unreachable", "next"}});
  auto last = W->last_status();
  auto dt = stopped_after(froze);
  auto modes = W->modes();
  check("SLAM freezes flying -> STOPPED 'SLAM pose frozen' at once",
        last_is(ph, "STOPPED") && l4::get(last, "reason") == "SLAM pose frozen" && dt && *dt < 2.0,
        l4::dumps(last) + ", " + (dt ? std::to_string(*dt) : "None") + " s after");
  std::lock_guard<std::mutex> g(W->m);
  check("SLAM freezes flying -> no next leg, no UNREACHABLE", W->goals.size() == 1 && !contains(ph, "UNREACHABLE"),
        "goals " + std::to_string(W->goals.size()) + " phases " + join(ph));
  check("SLAM freezes flying -> QRLAND (21), bridge off, never arms",
        modes == std::vector<int>{15, 21} && !W->enabled && W->forbidden.empty(), "modes " + join(modes));
}

TEST_F(Executive, S17b_SlamFreezesDriving) {
  std::optional<double> froze;
  auto ph = run_executive({drive(4.0, 0.0), drive(0.0, 1.0)}, nullptr, slam_freezes(1.0, froze), 40, true,
                          {{"no_progress_s", 20}, {"on_unreachable", "next"}});
  auto last = W->last_status();
  auto dt = stopped_after(froze);
  auto modes = W->modes();
  check("SLAM freezes driving -> STOPPED 'SLAM pose frozen' at once",
        last_is(ph, "STOPPED") && l4::get(last, "reason") == "SLAM pose frozen" && dt && *dt < 2.0,
        l4::dumps(last) + ", " + (dt ? std::to_string(*dt) : "None") + " s after");
  std::lock_guard<std::mutex> g(W->m);
  check("SLAM freezes driving -> GUIDED only (no QR mode on the ground), bridge off, no next leg, no takeoff",
        modes == std::vector<int>{15} && !W->enabled && W->drive_goals.size() == 1 && !contains(ph, "UNREACHABLE") &&
            W->takeoff_calls.empty(),
        "modes " + join(modes) + " drive goals " + std::to_string(W->drive_goals.size()) + " phases " + join(ph));
}

// 17c. a frozen pose reported while hovering at a waypoint (the executive's yaw settle): QRLAND too
TEST_F(Executive, S17c_SlamFreezesAtAWaypoint) {
  std::optional<double> froze;
  bool armed_freeze = false;
  auto ph = run_executive({fly(3.0, 0.0, 1.0), fly(3.0, 3.0, 1.0)}, nullptr,
                          [&](double) {
                            std::lock_guard<std::mutex> g(W->m);
                            bool at_wp = false;
                            for (const auto& s : W->status)
                              if (l4::get(s, "phase") == "WAYPOINT_YAW") at_wp = true;
                            if (!armed_freeze && at_wp) {
                              armed_freeze = true;
                              froze = wall();
                              W->bridge_state = frozen_state;
                            }
                          },
                          40);
  auto last = W->last_status();
  auto modes = W->modes();
  check("SLAM freezes at a waypoint -> STOPPED 'SLAM pose frozen', QRLAND",
        last_is(ph, "STOPPED") && l4::get(last, "reason") == "SLAM pose frozen" && modes == std::vector<int>{15, 21},
        l4::dumps(last) + " modes " + join(modes));
}

// 18. a planner_dead status from a bridge not enabled (left over from before the leg) is not acted on
TEST_F(Executive, S18_PlannerDeadWhileBridgeOff) {
  {
    std::lock_guard<std::mutex> g(W->m);
    W->bridge_state = dead_state;
    W->bridge_state["enabled"] = false;
    W->bridge_state["state"] = "OK";
  }
  auto ph = run_executive({drive(2.0, 0.0)});
  check("a planner_dead status while the bridge is off does not stop the leg", last_is(ph, "DONE"), join(ph));
}


// ---- the heading at waypoints (owner rule 2026-10-29) ------------------------------------------------

static double deg(double r) { return r * 180.0 / M_PI; }
static double ang(double a, double b) { return std::abs(std::atan2(std::sin(a - b), std::cos(a - b))); }
static int index_of(const std::vector<std::string>& ph, const char* p, int from = 0) {
  for (int i = from; i < static_cast<int>(ph.size()); ++i)
    if (ph[i] == p) return i;
  return -1;
}

// 19. land, then drive: over the spot the executive turns to face the drive, waits for the yaw to settle,
// and only then asks the landing check and lands
TEST_F(Executive, S19_LandFacingTheDrive) {
  auto ph = run_executive({takeoff(1.0), fly(2.0, 0.0, 1.0), land(2.0, 0.0), drive(2.0, 3.0)});
  check("land then drive: DONE", last_is(ph, "DONE"), join(ph));
  const int wy = index_of(ph, "WAYPOINT_YAW"), lc = index_of(ph, "LAND_CHECK");
  check("waypoint yaw settled before the landing check", wy >= 0 && lc > wy, join(ph));
  std::lock_guard<std::mutex> g(W->m);
  check("landed facing the drive leg (+90 deg) within 10 deg",
        W->yaw_at_land && ang(*W->yaw_at_land, M_PI / 2) < 10.0 * M_PI / 180.0,
        W->yaw_at_land ? std::to_string(deg(*W->yaw_at_land)) : "no landing");
  bool settled_report = false;
  for (const auto& s : W->status)
    if (l4::get(s, "phase") == "WAYPOINT_YAW" && l4::get(s, "settled") == true) settled_report = true;
  check("WAYPOINT_YAW reported settled", settled_report);
}

// 20. the yaw keeps swinging over the spot: no landing check, no landing; QRLOITER, STOPPED
TEST_F(Executive, S20_YawNeverSettles) {
  {
    std::lock_guard<std::mutex> g(W->m);
    W->yaw_wobble = true;
  }
  const std::vector<std::string> extra = {"_yaw_settle_s:=4.0"};
  auto ph = run_executive({takeoff(1.0), fly(2.0, 0.0, 1.0), land(2.0, 0.0)}, nullptr, nullptr, 40, true, ojson::object(), extra);
  auto last = W->last_status();
  auto modes = W->modes();
  check("yaw not settled -> STOPPED 'yaw did not settle'", last_is(ph, "STOPPED") && l4::get(last, "reason") == "yaw did not settle",
        l4::dumps(last));
  std::lock_guard<std::mutex> g(W->m);
  check("yaw not settled -> QRLOITER, no landing check, no landing",
        modes == std::vector<int>{15, 19} && !contains(ph, "LAND_CHECK") && !W->yaw_at_land, join(ph) + " modes " + join(modes));
}

// 21. fly, fly: at the first goal it faces the second leg before it starts
TEST_F(Executive, S21_FacesTheNextFlyLeg) {
  auto ph = run_executive({fly(3.0, 0.0, 1.0), fly(3.0, 3.0, 1.0)});
  check("fly, fly: DONE", last_is(ph, "DONE"), join(ph));
  std::lock_guard<std::mutex> g(W->m);
  check("second goal sent facing it (+90 deg) within 10 deg",
        W->yaw_at_goal.size() == 2 && ang(W->yaw_at_goal[1], M_PI / 2) < 10.0 * M_PI / 180.0,
        W->yaw_at_goal.size() == 2 ? std::to_string(deg(W->yaw_at_goal[1])) : "goals " + std::to_string(W->yaw_at_goal.size()));
  bool cleared = !W->yaw_cmds.empty() && std::isnan(W->yaw_cmds.back());
  check("the waypoint yaw is cleared when the next leg starts", cleared);
}

// 22. a leg's own "yaw" overrides the next leg's direction
TEST_F(Executive, S22_LegYawOverrides) {
  ojson l1 = fly(3.0, 0.0, 1.0);
  l1["yaw"] = -90;
  auto ph = run_executive({l1, fly(3.0, 3.0, 1.0)});
  check("leg yaw: DONE", last_is(ph, "DONE"), join(ph));
  std::lock_guard<std::mutex> g(W->m);
  check("faced the leg's yaw (-90 deg), not the next leg (+90)",
        W->yaw_at_goal.size() == 2 && ang(W->yaw_at_goal[1], -M_PI / 2) < 10.0 * M_PI / 180.0,
        W->yaw_at_goal.size() == 2 ? std::to_string(deg(W->yaw_at_goal[1])) : "");
}

// 23. a scan leg turns a full circle on the spot, then the mission goes on
TEST_F(Executive, S23_Scan) {
  auto ph = run_executive({fly(2.0, 0.0, 1.0), {{"type", "scan"}, {"yaw_sweep_deg", 360}, {"scan_rate_deg", 60}}, fly(2.0, 2.0, 1.0)},
                          nullptr, nullptr, 60);
  check("scan: SCAN then DONE", contains(ph, "SCAN") && last_is(ph, "DONE"), join(ph));
  std::lock_guard<std::mutex> g(W->m);
  check("scan turned through ~360 deg", W->yaw_travel > 340.0 * M_PI / 180.0, std::to_string(deg(W->yaw_travel)));
}

// 24. a scan leg on the ground (no fly leg before it): STOPPED, GUIDED only
TEST_F(Executive, S24_ScanOnTheGround) {
  auto ph = run_executive({drive(1.0, 0.0), {{"type", "scan"}, {"yaw_sweep_deg", 90}}});
  auto modes = W->modes();
  check("scan on the ground -> STOPPED, GUIDED only", last_is(ph, "STOPPED") && modes == std::vector<int>{15}, join(ph) + " " + join(modes));
}
// 25. a waypoint with no yaw to face (the last landing, no drive after it) sends the yaw the vehicle has,
// not none: the bridge holds a fixed point only while a waypoint yaw is set (Gate L2 b2)
TEST_F(Executive, S25_WaypointWithoutAYawSendsTheYawItHas) {
  auto ph = run_executive({takeoff(1.0), fly(2.0, 0.0, 1.0), land(2.0, 2.0)});
  check("fly, land: DONE", last_is(ph, "DONE"), join(ph));
  std::lock_guard<std::mutex> g(W->m);
  const bool finite = !W->yaw_cmds.empty() && std::isfinite(W->yaw_cmds.back());
  check("the landing's waypoint yaw is set (not NaN)", finite,
        W->yaw_cmds.empty() ? "no yaw_cmd" : std::to_string(W->yaw_cmds.back()));
  check("it is the yaw the vehicle had (the land leg's travel, +90 deg) and landed with, within 10 deg",
        finite && W->yaw_at_land && ang(W->yaw_cmds.back(), *W->yaw_at_land) < 10.0 * M_PI / 180.0 &&
            ang(W->yaw_cmds.back(), M_PI / 2) < 10.0 * M_PI / 180.0,
        finite ? std::to_string(deg(W->yaw_cmds.back())) : "");
}

// 26. off the ground, the first fly leg starts with a waypoint: face its goal, settle, then go (Gate L2 b5)
TEST_F(Executive, S26_FacesTheFirstLegAfterTakeoff) {
  auto ph = run_executive({takeoff(1.0), fly(0.0, 3.0, 1.0)});
  check("takeoff, fly: DONE", last_is(ph, "DONE"), join(ph));
  const int wy = index_of(ph, "WAYPOINT_YAW"), lg = index_of(ph, "LEG");
  check("a waypoint yaw before the leg", wy >= 0 && lg > wy, join(ph));
  std::lock_guard<std::mutex> g(W->m);
  check("the goal sent facing it (+90 deg) within 10 deg",
        W->yaw_at_goal.size() == 1 && ang(W->yaw_at_goal[0], M_PI / 2) < 10.0 * M_PI / 180.0,
        W->yaw_at_goal.size() == 1 ? std::to_string(deg(W->yaw_at_goal[0])) : "goals " + std::to_string(W->yaw_at_goal.size()));
  bool leg_start = false;
  for (const auto& s : W->status)
    if (l4::get(s, "phase") == "WAYPOINT_YAW" && l4::get(s, "for") == "leg start") leg_start = true;
  check("reported as the leg start's waypoint", leg_start);
}

// 27. the leg start's turn, then SUPER re-seeded with a short climb before the leg's goal (Gate L2 b7: at rest
// after the turn, SUPER's PlanFromRest never produced a plan)
TEST_F(Executive, S27_LegStartReseedsThePlanner) {
  // ~leg_start_seed_dz at its default (0.1 m, owner 2026-10-29)
  const std::vector<std::string> extra = {"_leg_start_seed_s:=0.5"};
  auto ph = run_executive({takeoff(1.0), fly(0.0, 3.0, 1.0)}, nullptr, nullptr, 40, true, ojson::object(), extra);
  check("takeoff, fly: DONE", last_is(ph, "DONE"), join(ph));
  const int wy = index_of(ph, "WAYPOINT_YAW"), sd = index_of(ph, "PLANNER_SEED"), lg = index_of(ph, "LEG");
  check("the turn, then the re-seed, then the leg", wy >= 0 && sd > wy && lg > sd, join(ph));
  std::lock_guard<std::mutex> g(W->m);
  check("two goals: the seed, then the leg's", W->goals.size() == 2, "goals " + std::to_string(W->goals.size()));
  if (W->goals.size() != 2) return;
  check("the seed 0.1 m (the default) straight above the vehicle (it hovered at (0, 0, 1), the leg's altitude)",
        (W->goals[0].second - Vec3(0.0, 0.0, 1.1)).norm() < 0.02, vstr(W->goals[0].second));
  check("the leg's goal", (W->goals[1].second - Vec3(0.0, 3.0, 1.0)).norm() < 1e-6, vstr(W->goals[1].second));
  check("the leg's goal ~leg_start_seed_s after the seed", W->goals[1].first - W->goals[0].first >= 0.45,
        std::to_string(W->goals[1].first - W->goals[0].first));
  check("the seed sent facing the leg (+90 deg), the leg's goal too (a vertical move keeps the yaw)",
        W->yaw_at_goal.size() == 2 && ang(W->yaw_at_goal[0], M_PI / 2) < 10.0 * M_PI / 180.0 &&
            ang(W->yaw_at_goal[1], M_PI / 2) < 10.0 * M_PI / 180.0);
  bool why = false;
  for (const auto& s : W->status)
    if (l4::get(s, "phase") == "PLANNER_SEED" && l4::get(s, "why") == "leg start") why = true;
  check("reported as the leg start's re-seed", why);
}

// 28. SUPER silent on a flight leg (the bridge's planner_age): re-seeded, the goal sent again, and it flies
TEST_F(Executive, S28_SilentPlannerIsReseeded) {
  {
    std::lock_guard<std::mutex> g(W->m);
    W->block = {1};   // the leg's first goal (after the leg start's seed): SUPER finds no plan
    W->bridge_state = {{"state", "OK"}, {"planner_age", 0.01}};
  }
  const std::vector<std::string> extra = {"_planner_stuck_s:=1.0"};
  auto during = [](double) {
    std::lock_guard<std::mutex> g(W->m);
    // silent from the leg's goal until its re-sent copy
    const double age = W->goals.size() == 2 ? 5.0 : 0.01;
    W->bridge_state = {{"state", "OK"}, {"planner_age", age}};
  };
  auto ph = run_executive({takeoff(1.0), fly(0.0, 3.0, 1.0)}, nullptr, during, 40, true, ojson::object(), extra);
  check("silent planner re-seeded: DONE", last_is(ph, "DONE"), join(ph));
  std::lock_guard<std::mutex> g(W->m);
  check("goals: seed, leg (no plan), seed, leg", W->goals.size() == 4, "goals " + std::to_string(W->goals.size()));
  if (W->goals.size() != 4) return;
  // the vehicle flew to the first seed (1.1 m, 0.1 m above the leg's 1.0 m): the second goes back down to
  // the leg's altitude, not 0.1 m above the vehicle (k1: 1.05 -> 1.35 -> 1.64 m)
  check("the second seed vertical over where it held, down to 1.0 m (no stacking), the goal re-sent unchanged",
        std::hypot(W->goals[2].second[0], W->goals[2].second[1]) < 0.4 && std::abs(W->goals[2].second[2] - 1.0) < 0.02 &&
            std::abs(W->goals[0].second[2] - 1.1) < 0.02 && (W->goals[3].second - Vec3(0.0, 3.0, 1.0)).norm() < 1e-6,
        vstr(W->goals[0].second) + " " + vstr(W->goals[2].second) + " " + vstr(W->goals[3].second));
  bool why = false, resent = false;
  for (const auto& s : W->status) {
    if (l4::get(s, "phase") == "PLANNER_SEED" && l4::get(s, "why") == "planner silent") why = true;
    if (l4::get(s, "phase") == "LEG" && l4::get(s, "resent") == true) resent = true;
  }
  check("reported: the re-seed and the goal re-sent", why && resent);
  check("no UNREACHABLE", !contains(ph, "UNREACHABLE"), join(ph));
}

// 29. SUPER stays silent: at most ~planner_stuck_max re-seeds a leg, then the usual unreachable
TEST_F(Executive, S29_ReseedsAreBounded) {
  {
    std::lock_guard<std::mutex> g(W->m);
    W->block = {1, 3, 5, 7};
    W->bridge_state = {{"state", "OK"}, {"planner_age", 0.01}};
  }
  const std::vector<std::string> extra = {"_planner_stuck_s:=1.0", "_planner_stuck_max:=2", "_on_unreachable:=stop"};
  auto during = [](double) {
    std::lock_guard<std::mutex> g(W->m);
    const double age = W->goals.size() >= 2 && W->goals.size() % 2 == 0 ? 5.0 : 0.01;
    W->bridge_state = {{"state", "OK"}, {"planner_age", age}};
  };
  auto ph = run_executive({takeoff(1.0), fly(0.0, 3.0, 1.0)}, nullptr, during, 40, true, ojson::object(), extra);
  check("never planned: UNREACHABLE, STOPPED", contains(ph, "UNREACHABLE") && last_is(ph, "STOPPED"), join(ph));
  std::lock_guard<std::mutex> g(W->m);
  check("goals: the leg start's seed and goal, then two re-seeds with the goal each", W->goals.size() == 6,
        "goals " + std::to_string(W->goals.size()));
  if (W->goals.size() != 6) return;
  // the fake SUPER flies to every seed: three seeds, each a 0.1 m move, alternating 1.1, 1.0, 1.1 round the
  // leg's 1.0 m, never one above the last
  check("repeated seeds do not stack: 1.1, 1.0, 1.1 m",
        std::abs(W->goals[0].second[2] - 1.1) < 0.02 && std::abs(W->goals[2].second[2] - 1.0) < 0.02 &&
            std::abs(W->goals[4].second[2] - 1.1) < 0.02,
        vstr(W->goals[0].second) + " " + vstr(W->goals[2].second) + " " + vstr(W->goals[4].second));
  check("the vehicle never above 1.1 m (the leg's altitude + one 0.1 m seed)", W->max_z <= 1.1 + 0.02,
        std::to_string(W->max_z));
}

// 30. the bridge reports a ceiling over the vehicle (its "ceiling_above"): the seed stays ~seed_ceiling_clear
// (0.6 m, SUPER's inflation and a cell) below it (container fly-through k1: 2.15 m roof)
TEST_F(Executive, S30_SeedCappedBelowTheCeiling) {
  auto setup = [] {
    std::lock_guard<std::mutex> g(W->m);
    W->bridge_state = {{"state", "OK"}, {"planner_age", 0.01}, {"ceiling_above", 0.65}};
  };
  const std::vector<std::string> extra = {"_leg_start_seed_s:=0.5"};
  auto ph = run_executive({takeoff(1.0), fly(0.0, 3.0, 1.0)}, setup, nullptr, 40, true, ojson::object(), extra);
  check("takeoff, fly: DONE", last_is(ph, "DONE"), join(ph));
  std::lock_guard<std::mutex> g(W->m);
  check("two goals: the seed, then the leg's", W->goals.size() == 2, "goals " + std::to_string(W->goals.size()));
  if (W->goals.size() != 2) return;
  check("the seed 0.6 m below the ceiling 0.65 m over the vehicle (1.05 m, not 1.1 m)",
        std::abs(W->goals[0].second[2] - 1.05) < 0.02, vstr(W->goals[0].second));
  bool capped = false;
  for (const auto& s : W->status)
    if (l4::get(s, "phase") == "PLANNER_SEED" && l4::get(s, "capped") == true && l4::get(s, "ceiling_above") == 0.65)
      capped = true;
  check("reported: capped by the ceiling", capped);
}

// ---- interactive mode ("click and run", ~interactive true): goals on /mission_executive/goal -------------
static ros::Publisher& goal_pub() {
  static ros::NodeHandle nh;
  static ros::Publisher p = nh.advertise<std_msgs::String>("/mission_executive/goal", 10);
  return p;
}
static void send_goal(const ojson& g) {
  std_msgs::String s;
  s.data = l4::dumps(g);
  goal_pub().publish(s);
}
// the executive's statuses with this phase (and goal number, when given)
static int count_status(const char* phase, int goal = -1) {
  std::lock_guard<std::mutex> g(W->m);
  int n = 0;
  for (const auto& s : W->status)
    if (l4::get(s, "phase") == phase && (goal < 0 || l4::get(s, "goal") == goal)) ++n;
  return n;
}
static const std::vector<std::string> INTERACTIVE = {"_interactive:=true"};

// I1. drive, fly (with a yaw), drive again: the takeoff and the landing come from the goals; no DONE, no landing
// at the end; GUIDED only; never arms
TEST_F(Executive, I1_InteractiveDriveFlyDrive) {
  goal_pub();
  int sent = 0;
  auto during = [&](double t) {
    if (sent == 0 && t > 1.0) {
      send_goal({{"type", "drive"}, {"goal", {2.0, 0.0}}});
      sent = 1;
    } else if (sent == 1 && count_status("REACHED", 1)) {
      send_goal({{"type", "fly"}, {"goal", {2.0, 2.0, 1.0}}, {"yaw", 90}});
      sent = 2;
    } else if (sent == 2 && count_status("REACHED", 2)) {
      send_goal({{"type", "drive"}, {"goal", {0.0, 2.0}}});
      sent = 3;
    }
  };
  auto ph = run_executive({}, nullptr, during, 45, true, ojson::object(), INTERACTIVE);
  auto modes = W->modes();
  check("interactive: three goals reached in turn", count_status("REACHED", 1) && count_status("REACHED", 2) && count_status("REACHED", 3),
        join(ph));
  check("interactive: never DONE or STOPPED, ends IDLE", !contains(ph, "DONE") && !contains(ph, "STOPPED") && last_is(ph, "IDLE"), join(ph));
  check("interactive: the drive in the air lands first (landing check, NAV_LAND)", contains(ph, "LAND_CHECK") && contains(ph, "LAND"), join(ph));
  std::lock_guard<std::mutex> g(W->m);
  check("interactive: one takeoff, for the fly goal", W->takeoff_calls.size() == 1, std::to_string(W->takeoff_calls.size()));
  check("interactive: the fly goal's yaw held at its end", W->yaw_at_land && ang(*W->yaw_at_land, M_PI) < 0.2,
        W->yaw_at_land ? std::to_string(deg(*W->yaw_at_land)) : "no landing");   // over the spot it faces the drive west
  check("interactive: on the ground at the end", W->pos[2] < 0.05 && (W->pos.head<2>() - Eigen::Vector2d(0.0, 2.0)).norm() < 0.5,
        vstr(W->pos));
  check("interactive: GUIDED only", modes == std::vector<int>{15}, join(modes));
  check("interactive: never arms or touches parameters", W->forbidden.empty(), join(W->forbidden));
  check("interactive: bridge never in the wrong mode", W->mode_mismatch == 0, std::to_string(W->mode_mismatch));
}

// I2. a new goal mid-leg replaces the running one, in the air and on the ground
TEST_F(Executive, I2_InteractiveGoalReplaced) {
  goal_pub();
  int sent = 0;
  auto during = [&](double t) {
    Vec3 pos;
    {
      std::lock_guard<std::mutex> g(W->m);
      pos = W->pos;
    }
    if (sent == 0 && t > 1.0) {
      send_goal({{"type", "drive"}, {"goal", {5.0, 0.0}}});
      sent = 1;
    } else if (sent == 1 && pos[0] > 1.0) {
      send_goal({{"type", "drive"}, {"goal", {1.5, 1.5}}});   // replaces the drive mid-leg
      sent = 2;
    } else if (sent == 2 && count_status("REACHED", 2)) {
      send_goal({{"type", "fly"}, {"goal", {5.0, 1.5, 1.0}}});
      sent = 3;
    } else if (sent == 3 && pos[0] > 2.8 && pos[2] > 0.5) {
      send_goal({{"type", "fly"}, {"goal", {3.0, 3.5, 1.0}}});   // replaces the fly goal mid-leg
      sent = 4;
    }
  };
  auto ph = run_executive({}, nullptr, during, 45, true, ojson::object(), INTERACTIVE);
  check("interactive: the replaced drive and fly goals end PREEMPTED", count_status("PREEMPTED", 1) == 1 && count_status("PREEMPTED", 3) == 1,
        join(ph));
  check("interactive: the replacing goals are reached", count_status("REACHED", 2) && count_status("REACHED", 4), join(ph));
  std::lock_guard<std::mutex> g(W->m);
  check("interactive: never went on to the replaced goals", W->pos[0] < 4.0 && (W->pos - Vec3(3.0, 3.5, 1.0)).norm() < 0.5, vstr(W->pos));
  check("interactive: replaced: never arms", W->forbidden.empty(), join(W->forbidden));
}

// I3. unreachable goals: on the ground no QR mode, in the air QRLOITER then GUIDED; the next goal still runs;
// a malformed goal is rejected
TEST_F(Executive, I3_InteractiveUnreachableThenNext) {
  {
    std::lock_guard<std::mutex> g(W->m);
    W->block = {0, 2};   // the first drive goal and the first fly goal (the track planner and SUPER count together)
  }
  goal_pub();
  int sent = 0;
  auto during = [&](double t) {
    if (sent == 0 && t > 1.0) {
      send_goal({{"type", "land"}, {"at", {1.0, 1.0}}});   // not an interactive goal
      send_goal({{"type", "drive"}, {"goal", {3.0, 0.0}}});
      sent = 1;
    } else if (sent == 1 && count_status("UNREACHABLE", 1)) {
      send_goal({{"type", "drive"}, {"goal", {0.0, 1.0}}});
      sent = 2;
    } else if (sent == 2 && count_status("REACHED", 2)) {
      send_goal({{"type", "fly"}, {"goal", {0.0, 4.0, 1.0}}});
      sent = 3;
    } else if (sent == 3 && count_status("UNREACHABLE", 3) && count_status("IDLE") >= 4) {
      send_goal({{"type", "fly"}, {"goal", {1.0, 1.0, 1.0}}});
      sent = 4;
    }
  };
  auto ph = run_executive({}, nullptr, during, 50, true, ojson::object(), INTERACTIVE);
  bool rejected = false;
  {
    std::lock_guard<std::mutex> g(W->m);
    for (const auto& s : W->status)
      if (l4::get(s, "goal_rejected").is_string()) rejected = true;
  }
  check("interactive: a land goal is rejected", rejected);
  check("interactive: unreachable drive, then the next goal reached", count_status("UNREACHABLE", 1) && count_status("REACHED", 2), join(ph));
  check("interactive: unreachable fly, then the next goal reached", count_status("UNREACHABLE", 3) && count_status("REACHED", 4), join(ph));
  auto modes = W->modes();
  check("interactive: QRLOITER only in the air (GUIDED, QRLOITER, GUIDED)", modes == std::vector<int>{15, 19, 15}, join(modes));
  check("interactive: unreachable: never STOPPED", !contains(ph, "STOPPED"), join(ph));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "fake_mavros_world");
  ros::AsyncSpinner spinner(4);
  spinner.start();
  W = new World;
  W->start();
  wsleep(1.0);
  int r = RUN_ALL_TESTS();
  std::printf("\n%d of %d checks passed\n", CHECKS - FAILED, CHECKS);
  return r;
}
