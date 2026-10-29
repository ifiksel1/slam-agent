// The turn slowdown of the planner bridge (owner 2026-10-29: the lidar faces forward, so the robot must not
// fly faster than its nose can turn toward the direction of travel). The bridge (turn_slow_node.test: fc_pos_p
// 0, max_lead 100, so each setpoint is the replayed reference itself) against synthetic inputs: FAST-LIO and
// the vision pose at 20 Hz (a fixed vehicle, the anchor), the FC pose at 20 Hz with a yaw the test sets, and
// SUPER at 100 Hz flying a straight line at the velocity the test sets (its acceleration reported as set).
#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <limits>
#include <mutex>
#include <thread>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <l4_common/pyjson.h>

#include "l4_bridge/bridge_core.h"

using namespace l4_bridge;
using mavros_msgs::PositionTarget;

namespace {
double wall() { return ros::WallTime::now().toSec(); }
void wsleep(double s) { std::this_thread::sleep_for(std::chrono::duration<double>(s)); }
const Anchor ANCHOR(0.5, Vec3(1.0, -2.0, 0.0));
const Vec3 P(1.0, 0.5, 1.2);   // the vehicle, planner frame
const double D = M_PI / 180.0;

std::mutex mtx;
std::vector<std::pair<double, PositionTarget>> sp;
std::string status;
Vec3 cmd_p = P, cmd_v(0, 0, 0), cmd_a(0, 0, 0);   // SUPER (planner frame)
bool cmd_on = true;
double fc_yaw = ANCHOR.psi;                      // the FC's yaw (fc frame)
}  // namespace

class TurnSlow : public ::testing::Test {
 protected:
  static inline ros::NodeHandle* nh;
  static inline ros::Publisher pub_yaw, pub_mode, pub_track;
  static inline std::thread* feeder;
  static inline std::atomic<bool> stop{false};

  static void SetUpTestSuite() {
    nh = new ros::NodeHandle;
    static ros::Subscriber s1 = nh->subscribe<PositionTarget>("/mavros/setpoint_raw/local", 1000,
        boost::function<void(const PositionTarget::ConstPtr&)>([](const PositionTarget::ConstPtr& m) {
          std::lock_guard<std::mutex> g(mtx);
          sp.emplace_back(wall(), *m);
        }));
    static ros::Subscriber s2 = nh->subscribe<std_msgs::String>("/planner_bridge/status", 10,
        boost::function<void(const std_msgs::String::ConstPtr&)>([](const std_msgs::String::ConstPtr& m) {
          std::lock_guard<std::mutex> g(mtx);
          status = m->data;
        }));
    static ros::Publisher odom = nh->advertise<nav_msgs::Odometry>("/Odometry", 50);
    static ros::Publisher vis = nh->advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 50);
    static ros::Publisher fc = nh->advertise<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 50);
    static ros::Publisher state = nh->advertise<mavros_msgs::State>("/mavros/state", 5, true);
    static ros::Publisher cmd = nh->advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 200);
    static ros::Publisher enable = nh->advertise<std_msgs::Bool>("/planner_bridge/enable", 1, true);
    pub_yaw = nh->advertise<std_msgs::Float64>("/planner_bridge/yaw_cmd", 1, true);
    pub_mode = nh->advertise<std_msgs::String>("/planner_bridge/set_mode", 1, true);
    pub_track = nh->advertise<quadrotor_msgs::PositionCommand>("/track_planner/pos_cmd", 50);
    static ros::AsyncSpinner spinner(2);
    spinner.start();
    wsleep(2.0);
    mavros_msgs::State st;
    st.mode = "GUIDED";
    st.armed = true;
    state.publish(st);
    std_msgs::Bool b;
    b.data = true;
    enable.publish(b);
    feeder = new std::thread([] {
      const Vec3 raw = position_matrix().transpose() * P;
      const Quat q_pl = raw_to_planner_quat(Quat{0, 0, 0, 1});
      double t_last = wall();
      for (int k = 0; !stop && ros::ok(); ++k) {
        const double t = wall();
        Vec3 p, v, a;
        bool on;
        double y;
        {
          std::lock_guard<std::mutex> g(mtx);
          cmd_p += cmd_v * (t - t_last);   // SUPER's reference flies on at its velocity
          p = cmd_p;
          v = cmd_v;
          a = cmd_a;
          on = cmd_on;
          y = fc_yaw;
        }
        t_last = t;
        if (k % 5 == 0) {
          nav_msgs::Odometry o;
          o.header.stamp = ros::Time::now();
          o.pose.pose.position.x = raw[0];
          o.pose.pose.position.y = raw[1];
          o.pose.pose.position.z = raw[2];
          o.pose.pose.orientation.w = 1.0;
          odom.publish(o);
          wsleep(0.002);
          geometry_msgs::PoseStamped vp;
          vp.header.stamp = ros::Time::now();
          const Vec3 pf = ANCHOR.pos(P);
          vp.pose.position.x = pf[0];
          vp.pose.position.y = pf[1];
          vp.pose.position.z = pf[2];
          const Quat qf = quat_mul(quat_yaw(ANCHOR.psi), q_pl);
          vp.pose.orientation.x = qf.x;
          vp.pose.orientation.y = qf.y;
          vp.pose.orientation.z = qf.z;
          vp.pose.orientation.w = qf.w;
          vis.publish(vp);
          // the FC pose: the same position, the yaw the test sets
          geometry_msgs::PoseStamped fp = vp;
          const Quat qy = quat_yaw(y);
          fp.pose.orientation.x = qy.x;
          fp.pose.orientation.y = qy.y;
          fp.pose.orientation.z = qy.z;
          fp.pose.orientation.w = qy.w;
          fc.publish(fp);
        }
        if (on) {
          quadrotor_msgs::PositionCommand c;
          c.header.stamp = ros::Time::now();
          c.position.x = p[0];
          c.position.y = p[1];
          c.position.z = p[2];
          c.velocity.x = v[0];
          c.velocity.y = v[1];
          c.velocity.z = v[2];
          c.acceleration.x = a[0];
          c.acceleration.y = a[1];
          c.acceleration.z = a[2];
          c.yaw = 1.3;
          cmd.publish(c);
        }
        wsleep(0.01);
      }
    });
  }
  static void TearDownTestSuite() {
    stop = true;
    feeder->join();
  }

  // SUPER flies on from where its reference is, at v (planner frame), reporting acceleration a
  static void set_cmd(const Vec3& v, const Vec3& a = Vec3::Zero(), bool on = true) {
    std::lock_guard<std::mutex> g(mtx);
    cmd_v = v;
    cmd_a = a;
    cmd_on = on;
  }
  static void set_fc_yaw(double y) {
    std::lock_guard<std::mutex> g(mtx);
    fc_yaw = y;
  }
  static Vec3 super_p() {
    std::lock_guard<std::mutex> g(mtx);
    return cmd_p;
  }
  static l4::ojson status_now() {
    std::lock_guard<std::mutex> g(mtx);
    return l4::loads(status);
  }
  static std::vector<PositionTarget> since(double t) {
    std::lock_guard<std::mutex> g(mtx);
    std::vector<PositionTarget> out;
    for (auto& s : sp)
      if (s.first >= t) out.push_back(s.second);
    return out;
  }
  static double hspeed(const PositionTarget& m) { return std::hypot(m.velocity.x, m.velocity.y); }
  // the setpoint's distance from SUPER's line through `p0` (planner frame) along planner +x, in fc
  static double off_line(const PositionTarget& m, const Vec3& p0) {
    const Vec3 q = ANCHOR.inverse_pos(Vec3(m.position.x, m.position.y, m.position.z));
    return std::hypot(q[1] - p0[1], q[2] - p0[2]);
  }
};

TEST_F(TurnSlow, T1_NoseAlongTheTravelFullSpeed) {
  // SUPER flies planner +x at 0.8 m/s, the FC's nose along it (fc yaw psi): SUPER's command as it is
  set_fc_yaw(ANCHOR.psi);
  set_cmd(Vec3(0.8, 0, 0));
  const double t0 = wall();
  wsleep(1.5);
  auto out = since(t0 + 0.5);
  ASSERT_GT(out.size(), 30u);
  for (const auto& m : out) EXPECT_NEAR(hspeed(m), 0.8, 1e-3);
  auto st = status_now();
  ASSERT_TRUE(st["turn_slow"].is_object()) << l4::dumps(st);
  EXPECT_LT(st["turn_slow"]["err_deg"].get<double>(), 1.0);
  EXPECT_EQ(st["turn_slow"]["frac"].get<double>(), 1.0);
  EXPECT_LT(st["turn_slow"]["lag_s"].get<double>(), 0.05);
  EXPECT_EQ(st["state"], "STREAM");
}

TEST_F(TurnSlow, T2_NoseOffTheTravelSlowsToTheFloorOnSUPERsPath) {
  // the nose 90 deg off the travel direction: the speed falls to the floor (0.25 of the 1.0 m/s cap) at no
  // more than ~turn_slow_accel (1.5 m/s^2), and every setpoint stays on SUPER's line, behind its reference
  const Vec3 line = super_p();
  set_fc_yaw(ANCHOR.psi + M_PI / 2);
  const double t0 = wall();
  wsleep(1.6);
  auto out = since(t0);
  ASSERT_GT(out.size(), 60u);
  auto late = since(t0 + 0.8);
  ASSERT_GT(late.size(), 30u);
  for (const auto& m : late) {
    EXPECT_LE(hspeed(m), 0.25 + 1e-3);
    EXPECT_GE(hspeed(m), 0.24);
  }
  double worst = 0.0, decel = 0.0;
  for (size_t i = 0; i < out.size(); ++i) {
    worst = std::max(worst, off_line(out[i], line));
    if (i > 0) {
      const double dt = (out[i].header.stamp - out[i - 1].header.stamp).toSec();
      if (dt > 0.005) decel = std::max(decel, (hspeed(out[i - 1]) - hspeed(out[i])) / dt);
    }
  }
  EXPECT_LT(worst, 1e-3);
  EXPECT_LE(decel, 1.5 * 1.2);   // the allowed speed's rate limit (a tick's timing jitter allowed for)
  auto st = status_now();
  ASSERT_TRUE(st["turn_slow"].is_object());
  EXPECT_NEAR(st["turn_slow"]["err_deg"].get<double>(), 90.0, 1.0);
  EXPECT_NEAR(st["turn_slow"]["frac"].get<double>(), 0.25, 1e-3);
  EXPECT_GT(st["turn_slow"]["lag_s"].get<double>(), 0.3);
  EXPECT_LT(st["turn_slow"]["lag_s"].get<double>(), 1.5);
}

TEST_F(TurnSlow, T3_NoseBackAlongTheTravelCatchesUp) {
  // the nose back along the travel: up to the cap (1.0 m/s, the replay at 1.25x SUPER's 0.8 m/s) until the
  // replay is back at SUPER's reference, then SUPER's own speed
  set_fc_yaw(ANCHOR.psi);
  const double t0 = wall();
  wsleep(6.0);
  auto out = since(t0);
  ASSERT_GT(out.size(), 200u);
  double top = 0.0;
  for (const auto& m : out) top = std::max(top, hspeed(m));
  EXPECT_LE(top, 1.0 + 1e-3);
  EXPECT_GT(top, 0.95);
  auto st = status_now();
  EXPECT_LT(st["turn_slow"]["lag_s"].get<double>(), 0.05);
  auto end = since(wall() - 0.5);
  ASSERT_FALSE(end.empty());
  for (const auto& m : end) EXPECT_NEAR(hspeed(m), 0.8, 1e-3);
}

TEST_F(TurnSlow, T4_TheLagStopsGrowingAtTheLimit) {
  // the nose stays off: the lag levels off at ~turn_slow_max_lag_s (2 s), the replay (and its cap) back at
  // SUPER's pace
  set_fc_yaw(ANCHOR.psi + M_PI / 2);
  wsleep(6.0);
  auto st = status_now();
  EXPECT_LE(st["turn_slow"]["lag_s"].get<double>(), 2.05);
  EXPECT_GE(st["turn_slow"]["lag_s"].get<double>(), 1.9);
  auto end = since(wall() - 0.5);
  ASSERT_FALSE(end.empty());
  for (const auto& m : end) EXPECT_NEAR(hspeed(m), 0.8, 0.03);
  set_fc_yaw(ANCHOR.psi);
  wsleep(10.0);   // caught up again (2 s at 0.25 s/s, after the 0.5 s ramp back up)
  EXPECT_LT(status_now()["turn_slow"]["lag_s"].get<double>(), 0.05);
}

TEST_F(TurnSlow, T5_ACornerAheadSlowsBeforeTheError) {
  // the nose along the velocity (no error now), SUPER's acceleration turning it (planner +y, 2 m/s^2):
  // 0.5 s ahead the travel is atan2(1.0, 0.8) = 51 deg off the nose -> slowed to the floor already
  set_fc_yaw(ANCHOR.psi);
  set_cmd(Vec3(0.8, 0, 0), Vec3(0, 2.0, 0));
  wsleep(1.5);
  auto st = status_now();
  ASSERT_TRUE(st["turn_slow"].is_object());
  EXPECT_NEAR(st["turn_slow"]["err_deg"].get<double>(), std::atan2(1.0, 0.8) / D, 1.0);
  EXPECT_NEAR(st["turn_slow"]["frac"].get<double>(), 0.25, 1e-3);
  auto end = since(wall() - 0.3);
  ASSERT_FALSE(end.empty());
  for (const auto& m : end) EXPECT_LE(hspeed(m), 0.25 + 1e-3);
  // the corner is in the replayed path: it is flown slowly where it is, then the replay catches up
  set_cmd(Vec3(0.8, 0, 0));
  wsleep(14.0);
  EXPECT_LT(status_now()["turn_slow"]["lag_s"].get<double>(), 0.05);
}

TEST_F(TurnSlow, T6_SlowCommandsAreNotSlowed) {
  // below ~yaw_speed_min (0.3 m/s) there is no travel direction: no slowdown, whatever the nose
  set_fc_yaw(ANCHOR.psi + M_PI / 2);
  set_cmd(Vec3(0.2, 0, 0));
  wsleep(1.5);
  auto st = status_now();
  ASSERT_TRUE(st["turn_slow"].is_object());
  EXPECT_TRUE(st["turn_slow"]["err_deg"].is_null());
  EXPECT_EQ(st["turn_slow"]["frac"].get<double>(), 1.0);
  auto end = since(wall() - 0.5);
  ASSERT_FALSE(end.empty());
  for (const auto& m : end) EXPECT_NEAR(hspeed(m), 0.2, 1e-3);
}

TEST_F(TurnSlow, T6b_AJumpInSUPERsReferenceRestartsTheReplay) {
  // slowed and behind SUPER, then SUPER's reference jumps 1 m back (a new plan from rest starts at the
  // vehicle): the buffered path is dropped and the replay restarts at the new reference, no lag
  set_cmd(Vec3(0.8, 0, 0));
  set_fc_yaw(ANCHOR.psi + M_PI / 2);
  wsleep(1.5);
  ASSERT_GT(status_now()["turn_slow"]["lag_s"].get<double>(), 0.5);
  Vec3 jumped;
  {
    std::lock_guard<std::mutex> g(mtx);
    cmd_p -= Vec3(1.0, 0, 0);
    jumped = cmd_p;
  }
  const double t0 = wall();
  wsleep(0.3);
  EXPECT_LT(status_now()["turn_slow"]["lag_s"].get<double>(), 0.25);   // (still slowed: it grows again)
  wsleep(0.3);
  auto out = since(t0 + 0.1);
  ASSERT_FALSE(out.empty());
  // every setpoint from the new reference on: within 0.5 s of travel (at most 0.8 m/s) of where it jumped to
  for (const auto& m : out) {
    const Vec3 q = ANCHOR.inverse_pos(Vec3(m.position.x, m.position.y, m.position.z));
    EXPECT_LT(std::abs(q[0] - jumped[0]), 0.5) << q.transpose() << " vs " << jumped.transpose();
  }
  set_fc_yaw(ANCHOR.psi);
  wsleep(1.0);
}

TEST_F(TurnSlow, T7_WaypointHoldAndFaultHoldAreNotTouched) {
  // the waypoint hold: zero velocity, no replay; SUPER silent (planner stale HOLD): the same
  set_cmd(Vec3(0.8, 0, 0));
  std_msgs::Float64 f;
  f.data = ANCHOR.psi;
  pub_yaw.publish(f);
  double t0 = wall();
  wsleep(1.0);
  for (const auto& m : since(t0 + 0.1)) EXPECT_EQ(hspeed(m), 0.0);
  auto st = status_now();
  EXPECT_EQ(st["state"], "WAYPOINT_HOLD");
  EXPECT_TRUE(st["turn_slow"].is_null());
  f.data = std::numeric_limits<double>::quiet_NaN();
  pub_yaw.publish(f);
  set_cmd(Vec3::Zero(), Vec3::Zero(), false);
  t0 = wall();
  wsleep(1.0);
  for (const auto& m : since(t0 + 0.5)) EXPECT_EQ(hspeed(m), 0.0);
  st = status_now();
  EXPECT_EQ(st["state"], "HOLD");
  EXPECT_TRUE(st["turn_slow"].is_null());
}

TEST_F(TurnSlow, T8_DriveModeIsNotTouched) {
  // drive mode: the track planner's target, position-only; no replay
  std_msgs::String m;
  m.data = "drive";
  pub_mode.publish(m);
  wsleep(0.3);
  const double t0 = wall();
  for (int i = 0; i < 100; ++i) {
    quadrotor_msgs::PositionCommand c;
    c.header.stamp = ros::Time::now();
    c.position.x = P[0] + 1.0;
    c.position.y = P[1];
    c.position.z = P[2];
    c.velocity.x = 0.8;
    c.yaw = 0.0;
    pub_track.publish(c);
    wsleep(0.02);
  }
  auto out = since(t0 + 0.5);
  ASSERT_FALSE(out.empty());
  for (const auto& s : out) EXPECT_EQ(s.type_mask, 8 | 16 | 32 | 64 | 128 | 256 | 2048);
  auto st = status_now();
  EXPECT_EQ(st["mode"], "drive");
  EXPECT_TRUE(st["turn_slow"].is_null());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_bridge_turn_slow", ros::init_options::AnonymousName);
  return RUN_ALL_TESTS();
}
