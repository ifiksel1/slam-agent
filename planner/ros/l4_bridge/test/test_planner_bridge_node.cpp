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
#include <cstring>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <limits>
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
  uint8_t flag = 0;                                  // the command's trajectory_flag (3, 4: the track planner's stop)
  const sensor_msgs::PointCloud2* cloud = nullptr;   // published at 10 Hz (raw frame, /cloud_registered)
};

// a cloud of planner-frame points as FAST-LIO's /cloud_registered (raw frame, float32 x y z)
sensor_msgs::PointCloud2 raw_cloud(const std::vector<Vec3>& planner_pts) {
  sensor_msgs::PointCloud2 c;
  c.header.frame_id = "camera_init";
  c.height = 1;
  c.width = uint32_t(planner_pts.size());
  const char* names[3] = {"x", "y", "z"};
  for (int k = 0; k < 3; ++k) {
    sensor_msgs::PointField f;
    f.name = names[k];
    f.offset = uint32_t(4 * k);
    f.datatype = sensor_msgs::PointField::FLOAT32;
    f.count = 1;
    c.fields.push_back(f);
  }
  c.point_step = 12;
  c.row_step = 12 * c.width;
  c.is_dense = true;
  c.data.resize(c.row_step);
  for (size_t i = 0; i < planner_pts.size(); ++i) {
    const Vec3 r = position_matrix().transpose() * planner_pts[i];
    const float xyz[3] = {float(r[0]), float(r[1]), float(r[2])};
    std::memcpy(c.data.data() + 12 * i, xyz, 12);
  }
  return c;
}

// SUPER's committed trajectory: one straight piece from p0 at velocity v for dur s, starting now
quadrotor_msgs::PolynomialTrajectory line_traj(const Vec3& p0, const Vec3& v, double dur) {
  quadrotor_msgs::PolynomialTrajectory t;
  t.header.stamp = ros::Time::now();
  t.type = quadrotor_msgs::PolynomialTrajectory::POSITION_TRAJ | quadrotor_msgs::PolynomialTrajectory::HEART_BEAT;
  t.piece_num_pos = 1;
  t.order_pos = 7;
  t.start_WT_pos = ros::Time::now();
  t.time_pos = {dur};
  for (int k = 0; k < 8; ++k) {
    t.coef_pos_x.push_back(k == 6 ? v[0] : k == 7 ? p0[0] : 0.0);
    t.coef_pos_y.push_back(k == 6 ? v[1] : k == 7 ? p0[1] : 0.0);
    t.coef_pos_z.push_back(k == 6 ? v[2] : k == 7 ? p0[2] : 0.0);
  }
  return t;
}

}  // namespace

class Node : public ::testing::Test {
 protected:
  static inline Anchor ANCHOR{30.0 * M_PI / 180.0, Vec3(2.0, -1.0, 0.1)};
  static inline ros::NodeHandle* nh = nullptr;
  static inline ros::AsyncSpinner* spinner = nullptr;
  static inline std::mutex mtx;
  static inline std::vector<std::pair<double, PositionTarget>> sp;
  static inline std::string status;
  static inline std::optional<Vec3> planner_odom;   // the newest /planner/odom position
  static inline std::optional<int> mode_code;
  static inline std::optional<std::string> mode_out;
  static inline ros::Publisher pub_cloud, pub_odom, pub_vis, pub_fc, pub_state, pub_cmd, pub_drive, pub_enable, pub_mode,
      pub_traj, pub_yaw, pub_goal;
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
    subs.push_back(sub<nav_msgs::Odometry>(*nh, "/planner/odom", 50, [](const nav_msgs::Odometry::ConstPtr& m) {
      std::lock_guard<std::mutex> g(mtx);
      planner_odom = Vec3(m->pose.pose.position.x, m->pose.pose.position.y, m->pose.pose.position.z);
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
    pub_cloud = nh->advertise<sensor_msgs::PointCloud2>("/cloud_registered", 5);
    pub_vis = nh->advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 50);
    pub_fc = nh->advertise<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 50);
    pub_state = nh->advertise<mavros_msgs::State>("/mavros/state", 5, true);
    pub_cmd = nh->advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 50);
    pub_drive = nh->advertise<quadrotor_msgs::PositionCommand>("/track_planner/pos_cmd", 50);
    pub_enable = nh->advertise<std_msgs::Bool>("/planner_bridge/enable", 1, true);
    pub_mode = nh->advertise<std_msgs::String>("/planner_bridge/set_mode", 1, true);
    pub_traj = nh->advertise<quadrotor_msgs::PolynomialTrajectory>("/planning_cmd/poly_traj", 5);
    pub_yaw = nh->advertise<std_msgs::Float64>("/planner_bridge/yaw_cmd", 1, true);
    pub_goal = nh->advertise<geometry_msgs::PoseStamped>("/planning/click_goal", 1, true);
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
        c.trajectory_flag = f.flag;
        (f.cmd_pub ? f.cmd_pub : &pub_cmd)->publish(c);
      }
      if (f.cloud && k % 10 == 0) pub_cloud.publish(*f.cloud);
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
static void near3(const Vec3& a, const Vec3& b, double tol) {
  for (int i = 0; i < 3; ++i) EXPECT_NEAR(a[i], b[i], tol) << "component " << i;
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
  feed(0.8, g);   // fresh commands after the jump, but no plan after it (L2 3a, 3b): still held
  EXPECT_TRUE(has("re-anchored"));
  EXPECT_TRUE(status_json()["reanchor"].is_object());
  EXPECT_TRUE(status_json()["slam_hold"].get<bool>());
  // a plan that does not start at the vehicle (a splice onto the old committed trajectory): still held
  pub_traj.publish(line_traj(g.p_planner + Vec3(2.0, 0.0, 0.0), Vec3(0.4, 0.0, 0.0), 3.0));
  feed(0.5, g);
  EXPECT_TRUE(has("re-anchored"));
  // a plan from the vehicle, made after the re-anchor: released
  pub_traj.publish(line_traj(g.p_planner, Vec3(0.4, 0.0, 0.0), 3.0));
  feed(0.8, g);
  EXPECT_TRUE(has("\"state\": \"STREAM\""));
  EXPECT_TRUE(status_json()["reanchor"].is_null());
  ANCHOR = moved;
}

TEST_F(Node, T5b_SlamFrameResetIsCarriedOver) {
  // L2 3b: FAST-LIO restarts at its own origin, 8 deg off (an instant jump); the SLAM-to-FC bridge re-anchors so the
  // vision pose carries on. The bridge carries the planner frame over it: /planner/odom, the anchor published and
  // SUPER's targets stay as they were; HOLD (re-anchored) until a plan after it
  const Vec3 p_old(1.0, 0.5, 1.2);
  Feed f;
  f.p_planner = p_old;
  feed(0.6, f);
  wsleep(0.25);
  ASSERT_TRUE(planner_odom.has_value());
  near3(*planner_odom, p_old, 1e-4);
  const int resets0 = status_json()["frame_resets"].get<int>();
  const auto an0 = status_json()["anchor"];
  // the new raw frame: the vehicle at its origin, the frame's yaw 8 deg on; the vision pose (fc) unchanged
  const Vec3 p_new(0.0, 0.0, 0.0);
  const double psi_new = ANCHOR.psi + 8.0 * M_PI / 180.0;
  const Anchor a_new(psi_new, ANCHOR.pos(p_old) - Anchor(psi_new, Vec3::Zero()).pos(p_new));
  Feed g;
  g.p_planner = p_new;
  g.anchor = a_new;
  g.cmd_pos = p_old + Vec3(0.3, 0.0, 0.0);   // SUPER, in the carried frame, commands as before
  const double t0 = wall();
  feed(0.6, g);
  wsleep(0.25);
  auto st = status_json();
  EXPECT_EQ(st["frame_resets"].get<int>(), resets0 + 1);
  EXPECT_EQ(st["state"], "HOLD");
  EXPECT_TRUE(st["slam_hold"].get<bool>());
  EXPECT_TRUE(st["reanchor"].is_object());
  EXPECT_NEAR(st["anchor"]["psi_deg"].get<double>(), an0["psi_deg"].get<double>(), 0.05);
  for (int i = 0; i < 3; ++i) EXPECT_NEAR(st["anchor"]["t"][i].get<double>(), an0["t"][i].get<double>(), 0.01);
  near3(*planner_odom, p_old, 1e-3);   // SUPER's odometry did not jump
  for (const auto& m : last(t0)) {      // nothing but the hold at the vehicle meanwhile
    EXPECT_TRUE(zero_vel(m));
    near_xyz(m.position, ANCHOR.pos(p_old), 0.02);
  }
  // SUPER plans again from the vehicle (carried frame): streaming, its targets where they were before the reset
  pub_traj.publish(line_traj(p_old, Vec3(0.4, 0.0, 0.0), 3.0));
  const double t1 = wall();
  feed(0.8, g);
  st = status_json();
  EXPECT_EQ(st["state"], "STREAM");
  auto out = last(t1 + 0.4);
  ASSERT_FALSE(out.empty());
  const Vec3 want = ANCHOR.pos(p_old + Vec3(0.3, 0.0, 0.0));
  EXPECT_NEAR(out.back().position.x, want[0], 0.02);
  EXPECT_NEAR(out.back().position.y, want[1], 0.02);
  EXPECT_NEAR(out.back().position.z, want[2], 0.02);
  // FAST-LIO back in the first raw frame (another reset): the carry undone, the published frame still the same
  Feed h;
  h.p_planner = p_old;
  feed(0.6, h);
  wsleep(0.25);
  st = status_json();
  EXPECT_EQ(st["frame_resets"].get<int>(), resets0 + 2);
  EXPECT_NEAR(st["anchor"]["psi_deg"].get<double>(), an0["psi_deg"].get<double>(), 0.05);
  near3(*planner_odom, p_old, 1e-3);
  pub_traj.publish(line_traj(p_old, Vec3(0.4, 0.0, 0.0), 3.0));
  feed(0.8, h);
  EXPECT_EQ(status_json()["state"], "STREAM");
}

TEST_F(Node, T5c_TargetHeightClamped) {
  // defence in depth (L2 3b): SUPER's target 2 m above the vehicle: the target's z within flight_z_lead (0.5 m) of
  // the vehicle's, the vertical feed-forward within the speed cap
  Feed f;
  f.cmd_pos = Vec3(1.2, 0.5, 3.2);
  f.cmd_vel = Vec3(0.0, 0.0, 2.0);
  const double t0 = wall();
  feed(1.0, f);
  auto out = last(t0 + 0.5);
  ASSERT_FALSE(out.empty());
  const double z_now = ANCHOR.pos(Vec3(1.0, 0.5, 1.2))[2];
  for (const auto& m : out) {
    EXPECT_LE(m.position.z, z_now + 0.5 + 1e-6);
    EXPECT_LE(std::abs(m.velocity.z), 1.0 + 1e-6);
  }
  EXPECT_GT(status_json()["z_clamped"].get<long long>(), 0);
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
  // a hold on the ground stops the rover: a zero-speed, zero-turn-rate target (mask 1479, no position) at the
  // full rate (owner task 2026-11-02; before, nothing, and Rover drove on to its last target)
  {
    auto held = last(t0 + 0.1);
    EXPECT_GT(held.size(), 10u);
    for (const auto& h : held) EXPECT_TRUE(h.type_mask == 1479 && h.velocity.x == 0.0 && h.velocity.y == 0.0);
  }
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

TEST_F(Node, T9e_FcPoseStaleInDriveModeStops) {
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
  {
    // the drive stop (no position in it: the stale FC pose is not needed), at the heading the rover had
    auto held = last(t0 + 0.4);
    EXPECT_GT(held.size(), 10u);
    for (const auto& h : held) EXPECT_TRUE(h.type_mask == 1479 && h.velocity.x == 0.0 && h.velocity.y == 0.0);
    for (const auto& h : held) EXPECT_EQ(h.yaw_rate, 0.0f);
  }
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
  // -speed (Rover: reverse), the yaw held; a HOLD sends the drive stop (mask 1479)
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
  for (const auto& h : held) EXPECT_TRUE(h.type_mask == 1479 && h.velocity.y == 0.0);
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

TEST_F(Node, T9f2_TrackPlannerStopStopsAtOnceAndResumes) {
  // the track planner's stop (NO_PATH: trajectory_flag 4; REACHED: 3) stops the rover within a tick or two, not
  // after the ~0.3 s planner_timeout; its next ordinary command resumes the drive targets
  const Vec3 ground(1.0, 0.5, 0.2);
  Feed carrot;
  carrot.p_planner = ground;
  carrot.cmd_pos = Vec3(ground + Vec3(2.0, 0.0, 0.0));
  carrot.cmd_vel = Vec3::Zero();
  carrot.cmd_pub = &pub_drive;
  set_mode("drive");
  feed(0.8, carrot);
  EXPECT_TRUE(has("\"state\": \"STREAM\""));
  for (uint8_t flag : {uint8_t(4), uint8_t(3)}) {
    Feed stop = carrot;
    stop.flag = flag;
    stop.cmd_pos = ground;
    const double t0 = wall();
    feed(0.6, stop);
    auto out = since_t(t0);
    ASSERT_FALSE(out.empty());
    double t_first = -1;
    for (const auto& [t, m] : out)
      if (m.type_mask == 1479) {
        t_first = t;
        break;
      }
    EXPECT_GE(t_first, t0);
    EXPECT_LT(t_first - t0, 0.12) << "stop after " << t_first - t0 << " s";
    for (const auto& [t, m] : out)
      if (t > t_first) EXPECT_TRUE(m.type_mask == 1479 && m.velocity.x == 0.0 && m.velocity.y == 0.0);
    auto st = status_json();
    EXPECT_EQ(st["state"], "HOLD");
    EXPECT_EQ(st["reason"], flag == 4 ? "track planner stop (NO_PATH)" : "track planner stop (REACHED)");
    const double t1 = wall();
    feed(0.5, carrot);   // a path again
    auto o = last(t1 + 0.2);
    ASSERT_FALSE(o.empty());
    EXPECT_EQ(o.back().type_mask, 2552);
  }
  set_mode("flight");
  feed(0.6);
}

TEST_F(Node, T9f3_PathBlockedHoldsUntilANewClearPlan) {
  // flight (L2 5a, f25r): SUPER's committed trajectory runs through a wall that the latest scans show 1.5 m ahead:
  // HOLD path_blocked, a zero-velocity target at the vehicle; a new trajectory away from it releases the hold
  const Vec3 a(1.0, 0.5, 1.2);
  std::vector<Vec3> pts;
  for (double y = -0.5; y <= 1.5; y += 0.05)
    for (double z = 0.2; z <= 2.2; z += 0.05) pts.push_back(Vec3(a[0] + 1.5, y, z));
  const auto cloud = raw_cloud(pts);
  Feed f;
  f.p_planner = a;
  f.cloud = &cloud;
  feed(0.6, f);
  EXPECT_TRUE(has("\"state\": \"STREAM\""));   // the wall alone: SUPER's command 0.3 m ahead is clear
  pub_traj.publish(line_traj(a, Vec3(1.0, 0.0, 0.0), 3.0));   // through the wall
  const double t0 = wall();
  feed(0.6, f);
  auto st = status_json();
  EXPECT_EQ(st["state"], "HOLD");
  EXPECT_EQ(st["reason"], "path_blocked");
  EXPECT_TRUE(st["path_block"]["blocked"].get<bool>());
  EXPECT_FALSE(st["slam_hold"].get<bool>());
  auto held = last(t0 + 0.2);
  EXPECT_GT(held.size(), 10u);
  for (const auto& m : held) {
    EXPECT_EQ(m.type_mask, 2048);
    EXPECT_TRUE(zero_vel(m));
    near_xyz(m.position, ANCHOR.pos(a));
  }
  // the same trajectory sent again (SUPER's replans fail, its committed one stays): still blocked
  pub_traj.publish(line_traj(a, Vec3(1.0, 0.0, 0.0), 3.0));
  feed(0.6, f);
  EXPECT_EQ(status_json()["reason"], "path_blocked");
  // a new plan sideways, clear of the wall: released after path_block_release_s
  pub_traj.publish(line_traj(a, Vec3(0.0, -1.0, 0.0), 3.0));
  feed(0.8, f);
  st = status_json();
  EXPECT_EQ(st["state"], "STREAM");
  EXPECT_FALSE(st["path_block"]["blocked"].get<bool>());
  EXPECT_EQ(st["path_block"]["holds"], 1);
  EXPECT_LT(st["path_block"]["check_ms_max"].get<double>(), 5.0);
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
  // the track planner (10 Hz) in its own process, killed -9 mid-drive: the drive stop (zero speed, zero turn
  // rate, mask 1479), streamed at the full rate, never the run-out position
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
    if (m.type_mask == 1479) {
      first_stop = t;
      break;
    }
  ASSERT_TRUE(first_stop);
  EXPECT_LT(*first_stop - t_kill, 0.7);   // 0.3 s timeout + up to 0.1 s of the 10 Hz period
  auto after = last(*first_stop);
  EXPECT_GT(after.size(), 30u);           // streamed, not a single message
  for (const auto& m : after) {
    EXPECT_EQ(m.type_mask, 1479);         // never the run-out position target again
    EXPECT_EQ(m.velocity.x, 0.0);
    EXPECT_EQ(m.velocity.y, 0.0);
    EXPECT_EQ(m.yaw_rate, 0.0f);          // no turn: not toward the carrot
  }
  // the track planner comes back: still stopped
  trk = start_planner("fake_track", "/track_planner/pos_cmd", ground + Vec3(2.0, 0.0, 0.0), Vec3::Zero(), 10);
  const double t1 = wall();
  feed(2.0, quiet);
  EXPECT_EQ(status_json()["reason"], "planner_dead");
  for (const auto& m : last(t1)) EXPECT_TRUE(m.type_mask == 1479 && m.velocity.y == 0.0);
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

TEST_F(Node, T10_SilentPlannerHoldNeverClimbs) {
  // container fly-through k1 (2026-10-29): SUPER climbing toward a re-seed goal, then silent. The hold is
  // the vehicle's position when it began, zero velocity: not SUPER's last (higher) point nor its climb, and
  // it does not follow the vehicle up while SUPER stays silent
  Feed climb;
  climb.cmd_pos = Feed().p_planner + Vec3(0.0, 0.0, 0.3);
  climb.cmd_vel = Vec3(0.0, 0.0, 0.4);
  feed(0.6, climb);
  const double t0 = wall();
  Feed quiet;
  quiet.cmd = false;
  feed(1.0, quiet);
  Feed drift = quiet;   // the vehicle carried 0.2 m up while SUPER is silent
  drift.p_planner = Feed().p_planner + Vec3(0.0, 0.0, 0.2);
  feed(0.6, drift);
  const auto out = since_t(t0 + 0.4);   // planner_timeout 0.3: the hold from here on
  ASSERT_GT(out.size(), 40u);
  const double z_vehicle = ANCHOR.pos(Feed().p_planner)[2];
  double z_max = -1e9;
  for (const auto& s : out) {
    z_max = std::max(z_max, static_cast<double>(s.second.position.z));
    EXPECT_TRUE(zero_vel(s.second));
    EXPECT_EQ(s.second.velocity.z, 0.0);
  }
  EXPECT_NEAR(z_max, z_vehicle, 1e-3);   // held where it was, never above
  EXPECT_NEAR(out.back().second.position.z, z_vehicle, 1e-3);
  EXPECT_TRUE(has("planner stale"));
}

TEST_F(Node, T11_CeilingOverTheVehicle) {
  // a /cloud_registered scan (raw frame): the lowest point within ~ceiling_radius (0.6 m) over the vehicle,
  // more than ~ceiling_min_up (0.15 m) above it, in the status as "ceiling_above"
  const Vec3 v = Feed().p_planner;   // (1.0, 0.5, 1.2) planner frame
  std::vector<Vec3> pts = {v + Vec3(0.2, 0.1, 1.1), v + Vec3(-0.3, 0.2, 1.3), v + Vec3(1.0, 0.0, 0.4),
                           v + Vec3(0.1, 0.0, 0.1), v + Vec3(0.0, 0.0, -1.2)};
  // and the floor 1.2 m under it (status "floor_below": at least ~floor_min_points 10 points under it)
  for (int i = 0; i < 12; ++i) pts.push_back(v + Vec3(-0.3 + 0.05 * i, 0.1, -1.2));
  sensor_msgs::PointCloud2 c;
  c.header.frame_id = "camera_init";
  c.height = 1;
  c.width = pts.size();
  for (int i = 0; i < 3; ++i) {
    sensor_msgs::PointField f;
    f.name = std::string(1, "xyz"[i]);
    f.offset = 4 * i;
    f.datatype = sensor_msgs::PointField::FLOAT32;
    f.count = 1;
    c.fields.push_back(f);
  }
  c.point_step = 16;
  c.row_step = c.point_step * c.width;
  c.is_dense = true;
  c.data.assign(c.row_step, 0);
  for (size_t i = 0; i < pts.size(); ++i) {
    const Vec3 raw = position_matrix().transpose() * pts[i];   // planner -> raw
    const float xyz[3] = {static_cast<float>(raw[0]), static_cast<float>(raw[1]), static_cast<float>(raw[2])};
    std::memcpy(c.data.data() + i * c.point_step, xyz, 12);
  }
  feed(0.3);
  c.header.stamp = ros::Time::now();
  pub_cloud.publish(c);
  feed(0.5);
  auto st = status_json();
  ASSERT_TRUE(st["ceiling_above"].is_number()) << status_str();
  EXPECT_NEAR(st["ceiling_above"].get<double>(), 1.1, 1e-3);
  ASSERT_TRUE(st["floor_below"].is_number()) << status_str();
  EXPECT_NEAR(st["floor_below"].get<double>(), 1.2, 1e-3);
  // no scan for longer than ~ceiling_max_age (1.0 s): not reported
  feed(1.3);
  EXPECT_TRUE(status_json()["ceiling_above"].is_null()) << status_str();
  EXPECT_TRUE(status_json()["floor_below"].is_null()) << status_str();
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_planner_bridge", ros::init_options::AnonymousName);
  return RUN_ALL_TESTS();
}

TEST_F(Node, T12_WaypointHoldWhileThePlannerIsQuiet) {
  // L2 4a (SITL fx14e): the executive climbs away from a box through the waypoint hold (a goal straight over the
  // vehicle, a waypoint yaw) while SUPER is silent (alive, its plans failing): the planner-stale HOLD must not keep the
  // vehicle where it is; the waypoint hold takes the goal (within waypoint_goal_max) and eases the vehicle up to it
  const Vec3 a(1.0, 0.5, 1.2);
  Feed f;
  f.p_planner = a;
  feed(0.5, f);
  Feed quiet = f;
  quiet.cmd = false;
  feed(0.6, quiet);
  EXPECT_TRUE(has("planner stale"));
  geometry_msgs::PoseStamped g;
  g.header.stamp = ros::Time::now();
  g.pose.position.x = a[0];
  g.pose.position.y = a[1];
  g.pose.position.z = a[2] + 0.6;
  g.pose.orientation.w = 1.0;
  pub_goal.publish(g);
  wsleep(0.2);
  std_msgs::Float64 y;
  y.data = 0.3;
  pub_yaw.publish(y);
  const double t0 = wall();
  feed(1.5, quiet);
  auto out = last(t0 + 0.3);
  ASSERT_FALSE(out.empty());
  const Vec3 v = ANCHOR.pos(a);
  EXPECT_EQ(out.back().type_mask, 2048);
  EXPECT_GT(out.back().position.z, v[2] + 0.45);   // toward the goal 0.6 m up (the target within flight_z_lead)
  EXPECT_LE(out.back().position.z, v[2] + 0.5 + 1e-6);
  auto st = status_json();
  EXPECT_EQ(st["state"], "HOLD");
  EXPECT_TRUE(st["waypoint_hold"].is_object());
  // a fault still wins: SLAM stale -> the hold at the vehicle, not the goal
  Feed stale = quiet;
  stale.slam = false;
  const double t1 = wall();
  feed(0.6, stale);
  auto held = last(t1 + 0.4);
  ASSERT_FALSE(held.empty());
  EXPECT_LT(held.back().position.z, v[2] + 0.1);
  y.data = std::numeric_limits<double>::quiet_NaN();
  pub_yaw.publish(y);
  feed(0.5, f);
}

TEST_F(Node, T13_GoalBesideAPillarOutOfViewIsHeldShortOf) {
  // L2 5a pillar fix (SITL fx16dr): a pillar 0.28 m beyond the goal (planner frame), seen as the vehicle came in, then
  // out of every scan; the vehicle 1.5 m from the goal, SUPER commanding it on toward the goal at 0.9 m/s. The leg's
  // memory keeps the pillar: the bridge holds short of the goal on the vehicle's side (zero velocity, capped), and the
  // waypoint hold holds there too
  const Vec3 a(1.0, 0.5, 1.2), goal = a + Vec3(1.5, 0.0, 0.0);
  std::vector<Vec3> pillar;
  for (double x = goal[0] + 0.28; x <= goal[0] + 0.88 + 1e-9; x += 0.05)
    for (double y = goal[1] - 0.3; y <= goal[1] + 0.3 + 1e-9; y += 0.05)
      for (double z = 0.2; z <= 2.2 + 1e-9; z += 0.1) pillar.push_back(Vec3(x, y, z));
  const auto seen = raw_cloud(pillar);
  const auto empty = raw_cloud({Vec3(a[0] - 6.0, a[1] + 6.0, 1.0)});   // the dome turned away: fresh scans, no pillar
  geometry_msgs::PoseStamped g;
  g.header.stamp = ros::Time::now();
  g.pose.position.x = goal[0];
  g.pose.position.y = goal[1];
  g.pose.position.z = goal[2];
  g.pose.orientation.w = 1.0;
  Feed far;
  far.p_planner = a - Vec3(1.0, 0.0, 0.0);   // 2.5 m out: no approach yet
  far.cloud = &seen;
  feed(0.3, far);
  pub_goal.publish(g);
  wsleep(0.1);
  feed(1.2, far);   // > waypoint_clear_scans scans: the earlier tests' scans are gone
  auto st = status_json();
  EXPECT_FALSE(st["goal_clear"].get<bool>());
  EXPECT_TRUE(st["goal_short"].is_null());
  EXPECT_GT(st["leg_memory"]["voxels"].get<int>(), 20);
  // 1.5 m out, the pillar out of view: held short
  Feed f;
  f.p_planner = a;
  f.cloud = &empty;
  f.cmd_pos = a + Vec3(0.5, 0.0, 0.0);
  f.cmd_vel = Vec3(0.9, 0.0, 0.0);
  const double t0 = wall();
  feed(0.8, f);
  st = status_json();
  ASSERT_TRUE(st["goal_short"].is_object()) << status_str();
  EXPECT_EQ(st["state"], "STREAM");
  EXPECT_GE(st["goal_shorts"].get<int>(), 1);
  // the nearest point to the goal clear by goal_short_r 0.6 (the pillar's voxel centres 0.25 m beyond it): 0.35 m back
  EXPECT_NEAR(st["goal_short"]["from_goal"].get<double>(), 0.35, 0.03);
  const Anchor br(st["anchor"]["psi_deg"].get<double>() * M_PI / 180.0,
                  Vec3(st["anchor"]["t"][0].get<double>(), st["anchor"]["t"][1].get<double>(), st["anchor"]["t"][2].get<double>()));
  const Vec3 spt(st["goal_short"]["point"][0].get<double>(), st["goal_short"]["point"][1].get<double>(),
                 st["goal_short"]["point"][2].get<double>());
  near3(spt, br.pos(goal - Vec3(st["goal_short"]["from_goal"].get<double>(), 0.0, 0.0)), 0.01);
  const Vec3 v = ANCHOR.pos(a);   // the FC pose fed
  const Vec3 dir = (spt - v).normalized();
  auto out = last(t0 + 0.3);
  ASSERT_GT(out.size(), 10u);
  for (const auto& m : out) {
    EXPECT_EQ(m.type_mask, 2048);
    EXPECT_TRUE(zero_vel(m));
    // toward the short point, never past it (not SUPER's command 0.5 m ahead at 0.9 m/s)
    const Vec3 p(m.position.x, m.position.y, m.position.z);
    EXPECT_LE((p - v).dot(dir), (spt - v).norm() + 0.02);
    EXPECT_LE((p - spt).norm(), (v - spt).norm() + 0.02);
  }
  // the vehicle at the short point's side, the leg ends (the waypoint yaw): the waypoint hold holds the short point
  Feed at = f;
  at.p_planner = goal - Vec3(0.45, 0.0, 0.0);
  feed(0.3, at);
  std_msgs::Float64 y;
  y.data = 0.3;
  pub_yaw.publish(y);
  feed(0.5, at);
  st = status_json();
  ASSERT_TRUE(st["waypoint_hold"].is_object()) << status_str();
  EXPECT_EQ(st["waypoint_hold"]["from"], "goal_short");
  const Vec3 hp(st["waypoint_hold"]["point"][0].get<double>(), st["waypoint_hold"]["point"][1].get<double>(),
                st["waypoint_hold"]["point"][2].get<double>());
  EXPECT_NEAR((hp - br.pos(goal)).norm(), 0.35, 0.03);
  y.data = std::numeric_limits<double>::quiet_NaN();
  pub_yaw.publish(y);
  // the next leg's goal far from the pillar: a new memory, the goal clear, SUPER streamed
  g.pose.position.x = a[0] - 3.0;
  pub_goal.publish(g);
  feed(1.0, f);
  st = status_json();
  EXPECT_TRUE(st["goal_clear"].get<bool>());
  EXPECT_TRUE(st["goal_short"].is_null());
}
