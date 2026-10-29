// Node tests of the planner bridge (rostest; formerly test_planner_bridge.py --ros, class Node): the bridge
// against synthetic inputs (FAST-LIO odometry, the SLAM bridge's vision pose with a known anchor, the FC
// pose and mode, SUPER and the track planner), and planners in their own processes (fake_planner) killed
// -9. One TEST per Python case, in the same order (they share the bridge's state, as the Python ones did).
#include <gtest/gtest.h>

#include <fcntl.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cmath>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>

#include <l4_common/pyjson.h>

#include "l4_bridge/bridge_core.h"

// a subscription with a lambda callback
template <class M, class F>
ros::Subscriber sub(ros::NodeHandle& nh, const std::string& topic, int q, F f) {
  return nh.subscribe<M>(topic, q, boost::function<void(const boost::shared_ptr<M const>&)>(f));
}

using namespace l4_bridge;
using mavros_msgs::PositionTarget;

namespace {

double wall() { return ros::WallTime::now().toSec(); }
void wsleep(double s) { std::this_thread::sleep_for(std::chrono::duration<double>(s)); }

pid_t start_planner(const std::string& name, const std::string& topic, const Vec3& pos, const Vec3& vel, double hz) {
  char p[128], v[128], h[32];
  std::snprintf(p, sizeof p, "%.17g,%.17g,%.17g", pos[0], pos[1], pos[2]);
  std::snprintf(v, sizeof v, "%.17g,%.17g,%.17g", vel[0], vel[1], vel[2]);
  std::snprintf(h, sizeof h, "%g", hz);
  pid_t pid = fork();
  if (pid == 0) {
    int devnull = open("/dev/null", O_WRONLY);
    dup2(devnull, 1);
    dup2(devnull, 2);
    execl(FAKE_PLANNER, FAKE_PLANNER, name.c_str(), topic.c_str(), p, v, h, (char*)nullptr);
    _exit(127);
  }
  return pid;
}

void kill9(pid_t pid) {   // SIGKILL, as the SITL fault cases (pkill -9)
  kill(pid, SIGKILL);
  waitpid(pid, nullptr, 0);
}

struct Feed {
  std::optional<Anchor> anchor;
  bool cmd = true, slam = true, fc = true;
  std::string mode = "GUIDED";
  Vec3 p_planner = Vec3(1.0, 0.5, 1.2);
  std::optional<Vec3> cmd_pos;
  Vec3 cmd_vel = Vec3(0.4, 0.0, 0.0);
  ros::Publisher* cmd_pub = nullptr;
};

}  // namespace

class Node : public ::testing::Test {
 protected:
  static inline Anchor ANCHOR{30.0 * M_PI / 180.0, Vec3(2.0, -1.0, 0.1)};
  static inline ros::NodeHandle* nh = nullptr;
  static inline ros::AsyncSpinner* spinner = nullptr;
  static inline std::mutex mtx;
  static inline std::vector<std::pair<double, PositionTarget>> sp;
  static inline std::string status;
  static inline std::optional<int> mode_code;
  static inline std::optional<std::string> mode_out;
  static inline ros::Publisher pub_odom, pub_vis, pub_fc, pub_state, pub_cmd, pub_drive, pub_enable, pub_mode;
  static inline std::vector<ros::Subscriber> subs;

  static void SetUpTestSuite() {
    nh = new ros::NodeHandle;
    subs.push_back(sub<PositionTarget>(*nh, "/mavros/setpoint_raw/local", 1000, [](const PositionTarget::ConstPtr& m) {
      std::lock_guard<std::mutex> g(mtx);
      sp.emplace_back(wall(), *m);
    }));
    subs.push_back(sub<std_msgs::String>(*nh, "/planner_bridge/status", 10, [](const std_msgs::String::ConstPtr& m) {
      std::lock_guard<std::mutex> g(mtx);
      status = m->data;
    }));
    subs.push_back(sub<std_msgs::Int8>(*nh, "/planner_bridge/mode_code", 10, [](const std_msgs::Int8::ConstPtr& m) {
      std::lock_guard<std::mutex> g(mtx);
      mode_code = m->data;
    }));
    subs.push_back(sub<std_msgs::String>(*nh, "/planner_bridge/mode", 10, [](const std_msgs::String::ConstPtr& m) {
      std::lock_guard<std::mutex> g(mtx);
      mode_out = m->data;
    }));
    pub_odom = nh->advertise<nav_msgs::Odometry>("/Odometry", 50);
    pub_vis = nh->advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 50);
    pub_fc = nh->advertise<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 50);
    pub_state = nh->advertise<mavros_msgs::State>("/mavros/state", 5, true);
    pub_cmd = nh->advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 50);
    pub_drive = nh->advertise<quadrotor_msgs::PositionCommand>("/track_planner/pos_cmd", 50);
    pub_enable = nh->advertise<std_msgs::Bool>("/planner_bridge/enable", 1, true);
    pub_mode = nh->advertise<std_msgs::String>("/planner_bridge/set_mode", 1, true);
    spinner = new ros::AsyncSpinner(2);
    spinner->start();
    wsleep(3.0);
    enable(true);
  }

  static void enable(bool on) {
    std_msgs::Bool b;
    b.data = on;
    pub_enable.publish(b);
  }
  static void set_mode(const std::string& m) {
    std_msgs::String s;
    s.data = m;
    pub_mode.publish(s);
  }

  // publish every input at 20 Hz (FAST-LIO) / 50 Hz (commands) for `seconds`; fc false: no FC pose
  void feed(double seconds, Feed f = Feed()) {
    const Anchor anchor = f.anchor ? *f.anchor : ANCHOR;
    mavros_msgs::State st;
    st.mode = f.mode;
    st.armed = true;
    pub_state.publish(st);
    const Vec3 raw = position_matrix().transpose() * f.p_planner;   // planner -> raw
    const Quat q_pl = raw_to_planner_quat(Quat{0, 0, 0, 1});
    const double t_end = wall() + seconds;
    int k = 0;
    while (wall() < t_end && ros::ok()) {
      if ((f.slam || f.fc) && k % 5 == 0) {
        if (f.slam) {
          nav_msgs::Odometry o;
          o.header.stamp = ros::Time::now();
          o.pose.pose.position.x = raw[0];
          o.pose.pose.position.y = raw[1];
          o.pose.pose.position.z = raw[2];
          o.pose.pose.orientation.w = 1.0;
          pub_odom.publish(o);
          wsleep(0.002);
        }
        // the FC's EKF pose goes on without SLAM (it then dead-reckons)
        std::vector<ros::Publisher*> pubs;
        if (f.slam) pubs.push_back(&pub_vis);
        if (f.fc) pubs.push_back(&pub_fc);
        for (auto* pub : pubs) {
          geometry_msgs::PoseStamped v;
          v.header.stamp = ros::Time::now();
          const Vec3 pf = anchor.pos(f.p_planner);
          v.pose.position.x = pf[0];
          v.pose.position.y = pf[1];
          v.pose.position.z = pf[2];
          const Quat qf = quat_mul(quat_yaw(anchor.psi), q_pl);
          v.pose.orientation.x = qf.x;
          v.pose.orientation.y = qf.y;
          v.pose.orientation.z = qf.z;
          v.pose.orientation.w = qf.w;
          pub->publish(v);
        }
      }
      if (f.cmd && k % 2 == 0) {
        quadrotor_msgs::PositionCommand c;
        c.header.stamp = ros::Time::now();
        const Vec3 cp = f.cmd_pos ? *f.cmd_pos : Vec3(f.p_planner + Vec3(0.3, 0.0, 0.0));
        c.position.x = cp[0];
        c.position.y = cp[1];
        c.position.z = cp[2];
        c.velocity.x = f.cmd_vel[0];
        c.velocity.y = f.cmd_vel[1];
        c.velocity.z = f.cmd_vel[2];
        c.acceleration.x = 0.2;
        c.yaw = 0.1;
        (f.cmd_pub ? f.cmd_pub : &pub_cmd)->publish(c);
      }
      ++k;
      wsleep(0.01);
    }
  }

  static std::vector<PositionTarget> last(double since) {
    std::lock_guard<std::mutex> g(mtx);
    std::vector<PositionTarget> out;
    for (const auto& s : sp)
      if (s.first >= since) out.push_back(s.second);
    return out;
  }
  static std::vector<std::pair<double, PositionTarget>> since_t(double since) {
    std::lock_guard<std::mutex> g(mtx);
    std::vector<std::pair<double, PositionTarget>> out;
    for (const auto& s : sp)
      if (s.first >= since) out.push_back(s);
    return out;
  }
  static std::string status_str() {
    std::lock_guard<std::mutex> g(mtx);
    return status;
  }
  static l4::ojson status_json() { return l4::loads(status_str()); }
  static bool has(const std::string& s) { return status_str().find(s) != std::string::npos; }

  void new_leg_with_this_node_as_planner(Feed f = Feed()) {
    // the executive's next leg: disable, this test node commanding (its node becomes the mode's planner), enable
    enable(false);
    feed(0.5, f);
    enable(true);
    feed(0.6, f);
  }
};

static void near_xyz(const geometry_msgs::Point& p, const Vec3& want, double tol = 1e-3) {
  EXPECT_NEAR(p.x, want[0], tol);
  EXPECT_NEAR(p.y, want[1], tol);
  EXPECT_NEAR(p.z, want[2], tol);
}
static bool zero_vel(const PositionTarget& m) { return m.velocity.x == 0.0 && m.velocity.y == 0.0 && m.velocity.z == 0.0; }

TEST_F(Node, T0_NothingBeforeTheFirstFcPose) {
  // runs first: the bridge has never had an FC pose; SLAM, anchor and SUPER are all there
  const double t0 = wall();
  Feed f;
  f.fc = false;
  feed(1.0, f);
  EXPECT_EQ(last(t0).size(), 0u);
  wsleep(0.25);
  auto st = status_json();
  EXPECT_EQ(st["fc_pose"], "never");
  EXPECT_TRUE(st["fc_pose_age"].is_null());
  EXPECT_EQ(st["reason"], "no FC pose yet");
}

TEST_F(Node, T1_StreamsAt50HzInFcFrame) {
  feed(1.0);
  const double t0 = wall();
  feed(2.0);
  auto out = last(t0);
  EXPECT_GT(out.size() / 2.0, 45.0);
  ASSERT_FALSE(out.empty());
  const auto& m = out.back();
  EXPECT_EQ(m.type_mask, 2048);   // position + velocity + acceleration + yaw
  EXPECT_EQ(m.coordinate_frame, PositionTarget::FRAME_LOCAL_NED);
  near_xyz(m.position, ANCHOR.pos(Vec3(1.3, 0.5, 1.2)));
  const Vec3 wv = ANCHOR.vec(Vec3(0.4, 0, 0));
  EXPECT_NEAR(m.velocity.x, wv[0], 1e-3);
  EXPECT_NEAR(m.velocity.y, wv[1], 1e-3);
  EXPECT_NEAR(m.yaw, 0.1 + ANCHOR.psi, 1e-3);
  EXPECT_TRUE(has("\"state\": \"STREAM\""));
}

TEST_F(Node, T2_SpeedCap) {
  Feed f;
  f.cmd_vel = Vec3(3.0, 0.0, 0.0);
  feed(0.5, f);
  const double t0 = wall();
  feed(0.5, f);
  auto out = last(t0);
  ASSERT_FALSE(out.empty());
  const auto& m = out.back();
  // the FC's demand, feed-forward + Q_P_NE_POS_P (1.0) x the 0.3 m lead, is held to the 1.0 cap
  const Vec3 c = ANCHOR.pos(Vec3(1.0, 0.5, 1.2));
  const double lx = m.position.x - c[0], ly = m.position.y - c[1];
  EXPECT_NEAR(std::hypot(m.velocity.x + lx, m.velocity.y + ly), 1.0, 1e-3);
}

TEST_F(Node, T3_PlannerStaleHoldsWithZeroVelocity) {
  feed(0.5);
  const double t0 = wall();
  Feed f;
  f.cmd = false;
  feed(1.0, f);
  auto out = last(t0 + 0.5);
  EXPECT_GT(out.size(), 20u);   // still streaming: a hold target
  ASSERT_FALSE(out.empty());
  const auto& m = out.back();
  EXPECT_TRUE(zero_vel(m));
  const Vec3 w = ANCHOR.pos(Vec3(1.0, 0.5, 1.2));
  EXPECT_NEAR(m.position.x, w[0], 1e-3);
  EXPECT_NEAR(m.position.y, w[1], 1e-3);
  EXPECT_TRUE(has("planner stale"));
}

TEST_F(Node, T4_SlamStaleHolds) {
  feed(0.5);
  Feed f;
  f.slam = false;
  feed(0.8, f);
  EXPECT_TRUE(has("SLAM stale"));
}

TEST_F(Node, T5_AnchorJumpHoldsUntilAFreshPlan) {
  feed(0.5);
  const Anchor moved(ANCHOR.psi, ANCHOR.t + Vec3(0.5, 0, 0));
  Feed f;
  f.anchor = moved;
  f.cmd = false;
  feed(0.6, f);
  wsleep(0.25);
  EXPECT_TRUE(has("re-anchored"));
  Feed g;
  g.anchor = moved;
  feed(0.8, g);   // fresh commands after the jump
  EXPECT_TRUE(has("\"state\": \"STREAM\""));
  ANCHOR = moved;
}

TEST_F(Node, T6_Geofence) {
  Feed f;
  f.cmd_pos = Vec3(50.0, 0.0, 1.2);
  feed(0.5, f);
  wsleep(0.25);
  EXPECT_TRUE(has("geofence"));
}

TEST_F(Node, T7_NothingOutsideGuided) {
  feed(0.3);
  const double t0 = wall();
  Feed f;
  f.mode = "QRLOITER";
  feed(0.8, f);
  EXPECT_EQ(last(t0 + 0.2).size(), 0u);
}

TEST_F(Node, T8_NothingUnlessEnabled) {
  // the FC's own takeoff and landing: a target, even a hold, would override them
  enable(false);
  feed(0.3);
  const double t0 = wall();
  feed(0.8);
  EXPECT_EQ(last(t0 + 0.2).size(), 0u);
  EXPECT_TRUE(has("\"enabled\": false"));
  enable(true);
  const double t1 = wall();
  feed(0.8);
  EXPECT_GT(last(t1 + 0.3).size(), 15u);
}

TEST_F(Node, T9a_DriveModeTakesOnlyTheTrackPlanner) {
  // the executive switches the bridge to drive for a drive leg; SUPER keeps publishing (toward its last
  // goal) and must be ignored
  const Vec3 ground(1.0, 0.5, 0.2);
  set_mode("drive");
  Feed f;
  f.p_planner = ground;
  feed(0.6, f);   // SUPER only
  wsleep(0.25);
  EXPECT_TRUE(has("\"mode\": \"drive\""));
  EXPECT_TRUE(has("mode changed to drive"));
  {
    std::lock_guard<std::mutex> g(mtx);
    EXPECT_EQ(mode_out.value_or(""), "drive");
  }
  const double t0 = wall();
  feed(0.4, f);
  // a hold on the ground sends nothing: Rover Guided stops at the last target it had
  EXPECT_EQ(last(t0 + 0.1).size(), 0u);
  // the track planner's target 3 m ahead: a target drive_lookahead (2.5 m) toward it
  const double t1 = wall();
  Feed d;
  d.p_planner = ground;
  d.cmd_pos = Vec3(ground + Vec3(3.0, 0.0, 0.0));
  d.cmd_vel = Vec3::Zero();
  d.cmd_pub = &pub_drive;
  feed(0.8, d);
  auto out = last(t1 + 0.4);
  EXPECT_GT(out.size(), 10u);
  ASSERT_FALSE(out.empty());
  const auto& m = out.back();
  EXPECT_EQ(m.type_mask, 2552);
  EXPECT_EQ(m.acceleration_or_force.x, 0.0);
  EXPECT_EQ(m.velocity.x, 0.0);
  near_xyz(m.position, ANCHOR.pos(ground) + ANCHOR.vec(Vec3(2.5, 0.0, 0.0)));
  EXPECT_TRUE(has("\"state\": \"STREAM\""));   // checked while fed (SLAM goes stale in 0.2 s)
  std::lock_guard<std::mutex> g(mtx);
  EXPECT_EQ(mode_code.value_or(-1), 2);
}

TEST_F(Node, T9b_BackToFlightIgnoresTheTrackPlanner) {
  set_mode("flight");
  Feed d;
  d.cmd_pos = Vec3(3.0, 0.5, 1.2);
  d.cmd_vel = Vec3::Zero();
  d.cmd_pub = &pub_drive;   // carrots only
  feed(0.6, d);
  wsleep(0.25);
  EXPECT_TRUE(has("\"mode\": \"flight\""));
  const double t0 = wall();
  feed(0.4, d);
  auto out = last(t0);
  ASSERT_FALSE(out.empty());
  const auto& m = out.back();
  EXPECT_EQ(m.type_mask, 2048);   // a flight hold at the vehicle
  EXPECT_TRUE(zero_vel(m));
  const Vec3 w = ANCHOR.pos(Vec3(1.0, 0.5, 1.2));
  EXPECT_NEAR(m.position.x, w[0], 1e-3);
  EXPECT_NEAR(m.position.y, w[1], 1e-3);
  feed(0.6);   // SUPER again
  EXPECT_TRUE(has("\"state\": \"STREAM\""));
  std::lock_guard<std::mutex> g(mtx);
  EXPECT_EQ(mode_code.value_or(-1), 1);
}

TEST_F(Node, T9c_UnknownModeIgnoredAndCodeZeroWhenDisabled) {
  set_mode("hover");
  feed(0.5);
  wsleep(0.25);
  EXPECT_TRUE(has("\"mode\": \"flight\""));
  enable(false);
  feed(0.5);
  wsleep(0.25);
  {
    std::lock_guard<std::mutex> g(mtx);
    EXPECT_EQ(mode_code.value_or(-1), 0);
  }
  enable(true);
}

TEST_F(Node, T9d_FcPoseStaleHoldsAtTheLastGoodPosition) {
  // flight: the FC pose stops while SLAM and SUPER go on (SLAM says the vehicle moved 1.5 m): HOLD at the
  // last fresh FC position, zero velocity, still at the full rate (Q_GUID_TIMEOUT)
  const Vec3 a(1.0, 0.5, 1.2), b(2.5, 0.5, 1.2);
  Feed fa;
  fa.p_planner = a;
  feed(0.6, fa);
  EXPECT_TRUE(has("\"state\": \"STREAM\""));
  const double t0 = wall();
  Feed fb;
  fb.p_planner = b;
  fb.fc = false;
  feed(1.0, fb);
  auto st = status_json();
  EXPECT_EQ(st["state"], "HOLD");
  EXPECT_EQ(st["reason"], "fc_pose_stale");
  EXPECT_EQ(st["fc_pose"], "stale");
  EXPECT_GT(st["fc_pose_age"].get<double>(), 0.3);
  EXPECT_FALSE(st["slam_hold"].get<bool>());
  auto out = last(t0 + 0.4);
  EXPECT_GT(out.size() / 0.6, 40.0);   // still ~50 Hz
  for (const auto& m : out) {
    EXPECT_EQ(m.type_mask, 2048);
    EXPECT_TRUE(zero_vel(m));
    near_xyz(m.position, ANCHOR.pos(a));
  }
  // fresh again: streams SUPER's command relative to the new position
  const double t1 = wall();
  Feed fb2;
  fb2.p_planner = b;
  feed(1.0, fb2);
  st = status_json();
  EXPECT_EQ(st["state"], "STREAM");
  EXPECT_EQ(st["fc_pose"], "ok");
  auto o2 = last(t1);
  ASSERT_FALSE(o2.empty());
  near_xyz(o2.back().position, ANCHOR.pos(b + Vec3(0.3, 0.0, 0.0)));
  EXPECT_FALSE(o2.back().velocity.x == 0.0 && o2.back().velocity.y == 0.0);
}

TEST_F(Node, T9e_FcPoseStaleInDriveModeSendsNothing) {
  const Vec3 ground(1.0, 0.5, 0.2);
  Feed carrot;
  carrot.p_planner = ground;
  carrot.cmd_pos = Vec3(ground + Vec3(2.0, 0.0, 0.0));
  carrot.cmd_vel = Vec3::Zero();
  carrot.cmd_pub = &pub_drive;
  set_mode("drive");
  feed(0.8, carrot);
  EXPECT_TRUE(has("\"state\": \"STREAM\""));
  const double t0 = wall();
  Feed nofc = carrot;
  nofc.fc = false;
  feed(0.8, nofc);
  EXPECT_EQ(last(t0 + 0.4).size(), 0u);   // Rover Guided brakes at its last target
  auto st = status_json();
  EXPECT_EQ(st["state"], "HOLD");
  EXPECT_EQ(st["reason"], "fc_pose_stale");
  EXPECT_EQ(st["mode"], "drive");
  const double t1 = wall();
  feed(0.8, carrot);   // fresh again: drive targets resume
  EXPECT_GT(last(t1 + 0.4).size(), 10u);
  auto o = last(t1);
  ASSERT_FALSE(o.empty());
  EXPECT_EQ(o.back().type_mask, 2552);
  set_mode("flight");
  feed(0.6);
}

TEST_F(Node, T9f_DriveReverseThenHoldStops) {
  // a dead end: the track planner's command with its velocity against its yaw backs up to the end point:
  // a Rover Guided velocity + heading target (mask 2503: no position), ENU velocity (0, -speed) = NED vx
  // -speed (Rover: reverse), the yaw held; a HOLD sends a zero-speed one
  const Vec3 ground(1.0, 0.5, 0.2);
  Feed back;
  back.p_planner = ground;
  back.cmd_pos = Vec3(ground + Vec3(-1.0, 0.0, 0.0));
  back.cmd_vel = Vec3(-0.25, 0.0, 0.0);
  back.cmd_pub = &pub_drive;
  set_mode("drive");
  feed(0.6, back);
  const double t0 = wall();
  feed(0.5, back);
  auto out = last(t0 + 0.1);
  EXPECT_GT(out.size(), 10u);
  ASSERT_FALSE(out.empty());
  const auto& m = out.back();
  EXPECT_EQ(m.type_mask, 2503);
  EXPECT_NEAR(m.velocity.x, 0.0, 1e-6);
  EXPECT_NEAR(m.velocity.y, -0.25, 1e-3);
  EXPECT_NEAR(m.yaw, ANCHOR.yaw(0.1), 1e-3);
  const double t1 = wall();
  Feed stop;
  stop.cmd = false;
  stop.p_planner = ground;
  feed(0.8, stop);   // the planner stops: HOLD
  auto held = last(t1 + 0.5);
  EXPECT_GT(held.size(), 5u);
  for (const auto& h : held) EXPECT_TRUE(h.type_mask == 2503 && h.velocity.y == 0.0);
  EXPECT_EQ(status_json()["state"], "HOLD");
  // a forward target afterwards: position targets again, no more reverse stops
  const double t2 = wall();
  Feed fwd;
  fwd.p_planner = ground;
  fwd.cmd_pos = Vec3(ground + Vec3(2.0, 0.0, 0.0));
  fwd.cmd_vel = Vec3::Zero();
  fwd.cmd_pub = &pub_drive;
  feed(0.8, fwd);
  auto o = last(t2 + 0.4);
  ASSERT_FALSE(o.empty());
  EXPECT_EQ(o.back().type_mask, 2552);
  set_mode("flight");
  feed(0.6);
}

TEST_F(Node, T9g_PlannerDiesInFlightHoldsAndNeverResumes) {
  // SUPER in its own process, killed -9 mid-flight: HOLD "planner_dead" at once, a zero-velocity target at
  // the vehicle at the full rate; it stays when SUPER comes back; a new leg ends it
  const Vec3 a(1.0, 0.5, 1.2);
  Feed quiet;
  quiet.p_planner = a;
  quiet.cmd = false;
  pid_t sup = start_planner("fake_super", "/planning/pos_cmd", Vec3(1.3, 0.5, 1.2), Vec3(0.4, 0.0, 0.0), 100);
  feed(2.5, quiet);
  auto st = status_json();
  EXPECT_EQ(st["state"], "STREAM");
  EXPECT_EQ(st["planner_node"], "/fake_super");
  EXPECT_EQ(st["planner_alive"], true);
  {
    auto o = last(wall() - 0.3);
    ASSERT_FALSE(o.empty());
    EXPECT_NE(o.back().velocity.x, 0.0);
  }
  kill9(sup);
  const double t_kill = wall();
  feed(1.2, quiet);
  st = status_json();
  EXPECT_EQ(st["state"], "HOLD");
  EXPECT_EQ(st["reason"], "planner_dead");
  EXPECT_EQ(st["planner_alive"], false);
  EXPECT_EQ(st["slam_hold"], false);
  EXPECT_GT(st["planner_age"].get<double>(), 0.3);
  std::optional<double> first_hold;
  for (const auto& [t, m] : since_t(t_kill))
    if (m.velocity.x == 0.0 && m.velocity.y == 0.0) {
      first_hold = t;
      break;
    }
  ASSERT_TRUE(first_hold);
  EXPECT_LT(*first_hold - t_kill, 0.6);   // planner_timeout 0.3 + the 50 Hz tick + pings
  auto held = last(t_kill + 0.6);
  EXPECT_GT(held.size() / 0.6, 40.0);     // still ~50 Hz: Guided does not time out
  for (const auto& m : held) {
    EXPECT_EQ(m.type_mask, 2048);
    EXPECT_TRUE(zero_vel(m));
    near_xyz(m.position, ANCHOR.pos(a));
  }
  // SUPER comes back (same name) and plans again: no resume, the leg is over
  sup = start_planner("fake_super", "/planning/pos_cmd", Vec3(1.3, 0.5, 1.2), Vec3(0.4, 0.0, 0.0), 100);
  feed(2.5, quiet);
  st = status_json();
  EXPECT_EQ(st["state"], "HOLD");
  EXPECT_EQ(st["reason"], "planner_dead");
  EXPECT_EQ(st["planner_alive"], true);
  EXPECT_LT(st["planner_age"].get<double>(), 0.1);
  for (const auto& m : last(wall() - 1.0)) EXPECT_EQ(m.velocity.x, 0.0);
  // the executive's next leg (enable false -> true) ends the latch: streams the live SUPER
  enable(false);
  feed(0.4, quiet);
  enable(true);
  feed(0.8, quiet);
  EXPECT_EQ(status_json()["state"], "STREAM");
  kill9(sup);
  // dead again; then this node as SUPER for the tests after this one
  feed(0.8, quiet);
  EXPECT_EQ(status_json()["reason"], "planner_dead");
  new_leg_with_this_node_as_planner();
  EXPECT_EQ(status_json()["state"], "STREAM");
}

TEST_F(Node, T9h_PlannerDiesWhileDrivingStopsAtOnce) {
  // the track planner (10 Hz) in its own process, killed -9 mid-drive: a zero-speed velocity + heading
  // target (mask 2503) at the rover's own heading, streamed at the full rate, never the run-out position
  // target again
  const Vec3 ground(1.0, 0.5, 0.2);
  Feed quiet;
  quiet.p_planner = ground;
  quiet.cmd = false;
  set_mode("drive");
  pid_t trk = start_planner("fake_track", "/track_planner/pos_cmd", ground + Vec3(2.0, 0.0, 0.0), Vec3::Zero(), 10);
  feed(2.5, quiet);
  auto st = status_json();
  EXPECT_EQ(st["state"], "STREAM");
  EXPECT_EQ(st["mode"], "drive");
  EXPECT_EQ(st["planner_node"], "/fake_track");
  {
    auto o = last(wall() - 0.3);
    ASSERT_FALSE(o.empty());
    EXPECT_EQ(o.back().type_mask, 2552);
  }
  kill9(trk);
  const double t_kill = wall();
  feed(1.2, quiet);
  st = status_json();
  EXPECT_EQ(st["state"], "HOLD");
  EXPECT_EQ(st["reason"], "planner_dead");
  EXPECT_EQ(st["mode"], "drive");
  std::optional<double> first_stop;
  for (const auto& [t, m] : since_t(t_kill))
    if (m.type_mask == 2503) {
      first_stop = t;
      break;
    }
  ASSERT_TRUE(first_stop);
  EXPECT_LT(*first_stop - t_kill, 0.7);   // 0.3 s timeout + up to 0.1 s of the 10 Hz period
  auto after = last(*first_stop);
  EXPECT_GT(after.size(), 30u);           // streamed, not a single message
  for (const auto& m : after) {
    EXPECT_EQ(m.type_mask, 2503);         // never the run-out position target again
    EXPECT_EQ(m.velocity.x, 0.0);
    EXPECT_EQ(m.velocity.y, 0.0);
    EXPECT_NEAR(m.yaw, ANCHOR.psi, 1e-3); // the rover's heading, not the carrot's
  }
  // the track planner comes back: still stopped
  trk = start_planner("fake_track", "/track_planner/pos_cmd", ground + Vec3(2.0, 0.0, 0.0), Vec3::Zero(), 10);
  const double t1 = wall();
  feed(2.0, quiet);
  EXPECT_EQ(status_json()["reason"], "planner_dead");
  for (const auto& m : last(t1)) EXPECT_TRUE(m.type_mask == 2503 && m.velocity.y == 0.0);
  kill9(trk);
  set_mode("flight");
  new_leg_with_this_node_as_planner();
  EXPECT_EQ(status_json()["state"], "STREAM");
}

TEST_F(Node, T9i_LivePlannerSilentIsNotDead) {
  // this node (the planner, alive) stops commanding: the resumable "planner stale" hold, and it streams
  // again on the next command without a new leg
  feed(0.6);
  Feed quiet;
  quiet.cmd = false;
  feed(1.0, quiet);
  auto st = status_json();
  EXPECT_EQ(st["state"], "HOLD");
  EXPECT_NE(st["reason"].get<std::string>().find("planner stale"), std::string::npos);
  EXPECT_EQ(st["planner_alive"], true);
  feed(0.6);
  EXPECT_EQ(status_json()["state"], "STREAM");
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_planner_bridge", ros::init_options::AnonymousName);
  return RUN_ALL_TESTS();
}
