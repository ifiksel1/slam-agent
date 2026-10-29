// Flight yaw of the planner bridge (owner rule 2026-10-29: the heading points along the direction of
// travel; the robot yaws only at a waypoint). The bridge with ~flight_yaw travel against synthetic inputs
// (as test_planner_bridge_node, published by a background thread at FAST-LIO's 20 Hz / SUPER's 50 Hz).
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

std::mutex mtx;
std::vector<std::pair<double, PositionTarget>> sp;
std::string status;
Vec3 cmd_vel(0, 0, 0);
bool cmd_on = true;
bool dither = false;   // SUPER hovering at its reached goal (Gate L2 b2): 0.2-0.4 m/s, its position wandering
}  // namespace

class Yaw : public ::testing::Test {
 protected:
  static inline ros::NodeHandle* nh;
  static inline ros::Publisher pub_yaw, pub_goal;
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
    static ros::Publisher cmd = nh->advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 50);
    static ros::Publisher enable = nh->advertise<std_msgs::Bool>("/planner_bridge/enable", 1, true);
    pub_yaw = nh->advertise<std_msgs::Float64>("/planner_bridge/yaw_cmd", 1, true);
    pub_goal = nh->advertise<geometry_msgs::PoseStamped>("/planning/click_goal", 1, true);
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
      for (int k = 0; !stop && ros::ok(); ++k) {
        if (k % 5 == 0) {
          nav_msgs::Odometry o;
          o.header.stamp = ros::Time::now();
          o.pose.pose.position.x = raw[0];
          o.pose.pose.position.y = raw[1];
          o.pose.pose.position.z = raw[2];
          o.pose.pose.orientation.w = 1.0;
          odom.publish(o);
          wsleep(0.002);
          geometry_msgs::PoseStamped v;
          v.header.stamp = ros::Time::now();
          const Vec3 pf = ANCHOR.pos(P);
          v.pose.position.x = pf[0];
          v.pose.position.y = pf[1];
          v.pose.position.z = pf[2];
          const Quat qf = quat_mul(quat_yaw(ANCHOR.psi), q_pl);   // the FC's yaw: ANCHOR.psi
          v.pose.orientation.x = qf.x;
          v.pose.orientation.y = qf.y;
          v.pose.orientation.z = qf.z;
          v.pose.orientation.w = qf.w;
          vis.publish(v);
          fc.publish(v);
        }
        Vec3 cv, cp = P;
        bool on;
        {
          std::lock_guard<std::mutex> g(mtx);
          cv = cmd_vel;
          on = cmd_on;
          if (dither) {
            // a new direction and speed (0.2, 0.3, 0.4 m/s) every 0.2 s, the position 0.1 m round P
            const int ph = k / 20;
            const double sp = 0.2 + 0.1 * (ph % 3), a = 2.0 * ph;
            cv = Vec3(sp * std::cos(a), sp * std::sin(a), 0.0);
            cp = P + Vec3(0.1 * std::cos(a), 0.1 * std::sin(a), 0.0);
          }
        }
        if (on && k % 2 == 0) {
          quadrotor_msgs::PositionCommand c;
          c.header.stamp = ros::Time::now();
          c.position.x = cp[0];
          c.position.y = cp[1];
          c.position.z = cp[2];
          c.velocity.x = cv[0];
          c.velocity.y = cv[1];
          c.velocity.z = cv[2];
          c.yaw = 1.3;   // SUPER's own yaw: never followed in travel mode
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

  static void set_cmd(const Vec3& v, bool on = true, bool dith = false) {
    std::lock_guard<std::mutex> g(mtx);
    cmd_vel = v;
    cmd_on = on;
    dither = dith;
  }
  static void goal(const Vec3& p_planner) {
    geometry_msgs::PoseStamped m;
    m.header.stamp = ros::Time::now();
    m.header.frame_id = "world";
    m.pose.position.x = p_planner[0];
    m.pose.position.y = p_planner[1];
    m.pose.position.z = p_planner[2];
    m.pose.orientation.w = 1.0;
    pub_goal.publish(m);
  }
  static l4::ojson status_now() {
    std::lock_guard<std::mutex> g(mtx);
    return l4::loads(status);
  }
  static void yaw_cmd(double y) {
    std_msgs::Float64 f;
    f.data = y;
    pub_yaw.publish(f);
  }
  static std::vector<PositionTarget> since(double t) {
    std::lock_guard<std::mutex> g(mtx);
    std::vector<PositionTarget> out;
    for (auto& s : sp)
      if (s.first >= t) out.push_back(s.second);
    return out;
  }
  static double max_step(const std::vector<PositionTarget>& v) {
    double m = 0;
    for (size_t i = 1; i < v.size(); ++i) m = std::max(m, std::abs(wrap_pi(v[i].yaw - v[i - 1].yaw)));
    return m;
  }
  // the commanded yaw rate, max, rad/s: between setpoints by their stamps (the tick times the bridge
  // sized each step on); the step is rate * elapsed time, so a late tick takes a bigger step
  static double max_rate(const std::vector<PositionTarget>& v) {
    double m = 0;
    for (size_t i = 1; i < v.size(); ++i) {
      const double dt = (v[i].header.stamp - v[i - 1].header.stamp).toSec();
      const double d = std::abs(wrap_pi(v[i].yaw - v[i - 1].yaw));
      if (dt > 0) m = std::max(m, d / dt);
      else if (d > 1e-6) m = 1e9;   // a step with no time
    }
    return m;
  }
};

TEST_F(Yaw, Y1_MovingTheNosePointsAlongTheTravelDirection) {
  // SUPER flies planner +y (fc: psi + 90 deg) with its own yaw 1.3: the nose turns to the travel heading at
  // <= 60 deg/s from the FC's yaw (psi), never to SUPER's
  set_cmd(Vec3(0.0, 0.6, 0.0));
  const double t0 = wall();
  wsleep(3.0);
  auto out = since(t0);
  ASSERT_GT(out.size(), 100u);
  EXPECT_NEAR(wrap_pi(out.back().yaw - (ANCHOR.psi + M_PI / 2)), 0.0, 1e-3);
  EXPECT_LE(max_rate(out), 60.0 * M_PI / 180.0 * 1.001);           // 60 deg/s by the stamps
  EXPECT_LE(max_step(out), 2.0 * 60.0 * M_PI / 180.0 / 50.0 + 1e-4);   // a late tick: two ticks' worth at most
  EXPECT_NEAR(wrap_pi(out.front().yaw - ANCHOR.psi), 0.0, 0.05);  // starts from the FC's yaw
  auto st = l4::loads(status);
  EXPECT_EQ(st["flight_yaw"], "travel");
}

TEST_F(Yaw, Y2_SlowHoldsTheYawItHas) {
  // slowing below yaw_speed_min (0.3 m/s): the yaw stays where it is, whatever SUPER's yaw or the velocity
  set_cmd(Vec3(0.2, 0.0, 0.0));
  const double t0 = wall();
  wsleep(1.5);
  for (const auto& m : since(t0)) EXPECT_NEAR(wrap_pi(m.yaw - (ANCHOR.psi + M_PI / 2)), 0.0, 1e-3);
}

TEST_F(Yaw, Y3_WaypointYawFacedWhenSlow) {
  // the executive's waypoint yaw: faced at <= 60 deg/s while slow (hovering at the goal)
  set_cmd(Vec3::Zero());
  const double want = wrap_pi(ANCHOR.psi - 1.0);
  yaw_cmd(want);
  const double t0 = wall();
  wsleep(3.0);
  auto out = since(t0);
  ASSERT_FALSE(out.empty());
  EXPECT_NEAR(wrap_pi(out.back().yaw - want), 0.0, 1e-3);
  EXPECT_LE(max_rate(out), 60.0 * M_PI / 180.0 * 1.001);
  // the waypoint hold: no goal known yet, so at the vehicle, zero velocity
  const Vec3 p_fc = ANCHOR.pos(P);
  for (const auto& m : out) {
    EXPECT_EQ(m.velocity.x, 0.0);
    EXPECT_EQ(m.velocity.y, 0.0);
    EXPECT_NEAR(m.position.x, p_fc[0], 1e-6);
    EXPECT_NEAR(m.position.y, p_fc[1], 1e-6);
  }
  wsleep(0.3);
  auto st = status_now();
  EXPECT_NEAR(st["yaw_cmd_deg"].get<double>(), want * 180.0 / M_PI, 0.01);
  EXPECT_EQ(st["state"], "WAYPOINT_HOLD");
  EXPECT_EQ(st["waypoint_hold"]["from"], "vehicle");
}

TEST_F(Yaw, Y4_TravelOnceTheWaypointYawIsCleared) {
  // the executive clears the waypoint yaw as the next leg starts; until then the waypoint hold keeps the
  // waypoint yaw whatever SUPER commands (Y6)
  yaw_cmd(std::numeric_limits<double>::quiet_NaN());
  set_cmd(Vec3(0.6, 0.0, 0.0));   // fc heading psi
  const double t0 = wall();
  wsleep(2.5);
  auto out = since(t0);
  ASSERT_FALSE(out.empty());
  EXPECT_NEAR(wrap_pi(out.back().yaw - ANCHOR.psi), 0.0, 1e-3);
}

TEST_F(Yaw, Y5_HoldFacesTheWaypointYawThenKeepsIt) {
  // SUPER silent (planner stale HOLD): a zero-velocity target at the vehicle facing the waypoint yaw;
  // NaN clears it and the yaw stays where it got to
  set_cmd(Vec3::Zero(), false);
  const double want = wrap_pi(ANCHOR.psi + 2.0);
  yaw_cmd(want);
  const double t0 = wall();
  wsleep(3.0);
  auto out = since(t0 + 0.6);
  ASSERT_FALSE(out.empty());
  for (const auto& m : out) EXPECT_EQ(m.velocity.x, 0.0);
  EXPECT_NEAR(wrap_pi(out.back().yaw - want), 0.0, 1e-3);
  EXPECT_NE(l4::loads(status)["reason"].get<std::string>().find("planner stale"), std::string::npos);
  yaw_cmd(std::numeric_limits<double>::quiet_NaN());
  const double t1 = wall();
  wsleep(1.0);
  for (const auto& m : since(t1)) EXPECT_NEAR(wrap_pi(m.yaw - want), 0.0, 1e-3);
  EXPECT_TRUE(l4::loads(status)["yaw_cmd_deg"].is_null());
}

TEST_F(Yaw, Y6_WaypointHoldIgnoresSuperDithering) {
  // Gate L2 b2: SUPER hovering at its reached goal commands 0.2-0.4 m/s in changing directions. With the
  // waypoint yaw set the bridge holds the reached goal (0.3 m from the vehicle), zero velocity, and turns
  // to the waypoint yaw at <= 60 deg/s and stays there: no travel yaw from the dithering
  const Vec3 g_planner = P + Vec3(0.3, 0.0, 0.0);
  goal(g_planner);
  set_cmd(Vec3::Zero(), true, true);
  wsleep(0.5);
  const double want = wrap_pi(ANCHOR.psi + 1.0);
  yaw_cmd(want);
  const double t0 = wall();
  wsleep(3.0);
  auto out = since(t0 + 0.1);
  ASSERT_GT(out.size(), 100u);
  const Vec3 g_fc = ANCHOR.pos(g_planner);
  for (const auto& m : out) {
    EXPECT_EQ(m.type_mask, 2048);
    EXPECT_EQ(m.velocity.x, 0.0);
    EXPECT_EQ(m.velocity.y, 0.0);
    EXPECT_EQ(m.velocity.z, 0.0);
    EXPECT_EQ(m.acceleration_or_force.x, 0.0);
    EXPECT_NEAR(m.position.x, g_fc[0], 1e-6);
    EXPECT_NEAR(m.position.y, g_fc[1], 1e-6);
    EXPECT_NEAR(m.position.z, g_fc[2], 1e-6);
  }
  EXPECT_LE(max_rate(out), 60.0 * M_PI / 180.0 * 1.001);
  // the yaw only ever closes on the waypoint yaw, and holds it once there
  double prev = std::abs(wrap_pi(out.front().yaw - want));
  for (const auto& m : out) {
    const double e = std::abs(wrap_pi(m.yaw - want));
    EXPECT_LE(e, prev + 1e-6);
    prev = e;
  }
  for (const auto& m : since(t0 + 2.0)) EXPECT_NEAR(wrap_pi(m.yaw - want), 0.0, 1e-3);
  auto st = status_now();
  EXPECT_EQ(st["state"], "WAYPOINT_HOLD");
  EXPECT_EQ(st["reason"], "waypoint yaw");
  EXPECT_EQ(st["waypoint_hold"]["from"], "goal");
  EXPECT_NEAR(st["waypoint_hold"]["point"][0].get<double>(), g_fc[0], 1e-3);
}

TEST_F(Yaw, Y7_ClearingTheWaypointYawResumesStreaming) {
  // NaN and the next goal (the executive's next leg, planner +y): once SUPER sets off toward it, its
  // command streams again, velocity and all, the nose along it
  yaw_cmd(std::numeric_limits<double>::quiet_NaN());
  goal(P + Vec3(0.0, 3.0, 0.0));
  set_cmd(Vec3(0.0, 0.6, 0.0));
  wsleep(0.3);
  const double t0 = wall();
  wsleep(2.5);
  auto out = since(t0);
  ASSERT_GT(out.size(), 50u);
  const Vec3 v_fc = ANCHOR.vec(Vec3(0.0, 0.6, 0.0)), p_fc = ANCHOR.pos(P);
  for (const auto& m : out) {
    EXPECT_NEAR(m.velocity.x, v_fc[0], 1e-6);
    EXPECT_NEAR(m.velocity.y, v_fc[1], 1e-6);
    EXPECT_NEAR(m.position.x, p_fc[0], 1e-6);
    EXPECT_NEAR(m.position.y, p_fc[1], 1e-6);
  }
  EXPECT_NEAR(wrap_pi(out.back().yaw - (ANCHOR.psi + M_PI / 2)), 0.0, 1e-3);
  auto st = status_now();
  EXPECT_EQ(st["state"], "STREAM");
  EXPECT_TRUE(st["waypoint_hold"].is_null());
}

TEST_F(Yaw, Y8_FarGoalHoldsAtTheVehicle) {
  // a goal farther than ~waypoint_goal_max (1.0 m) from the vehicle when the hold begins (not reached):
  // the hold is at the vehicle
  goal(P + Vec3(3.0, 0.0, 0.0));
  set_cmd(Vec3::Zero(), true, true);
  wsleep(0.3);
  yaw_cmd(ANCHOR.psi);
  const double t0 = wall();
  wsleep(1.0);
  const Vec3 p_fc = ANCHOR.pos(P);
  auto out = since(t0 + 0.1);
  ASSERT_FALSE(out.empty());
  for (const auto& m : out) {
    EXPECT_EQ(m.velocity.x, 0.0);
    EXPECT_NEAR(m.position.x, p_fc[0], 1e-6);
    EXPECT_NEAR(m.position.y, p_fc[1], 1e-6);
  }
  EXPECT_EQ(status_now()["waypoint_hold"]["from"], "vehicle");
}

TEST_F(Yaw, Y9_DepartureWaitsForThePlannerToSetOff) {
  // Gate L2 b5: after the clear SUPER first flies on round the old goal, backwards. Held at the vehicle
  // (Y8), next goal planner +x 3 m: SUPER commanding -x 0.5 m/s -> the bridge holds on, point and yaw;
  // SUPER commanding +x 0.6 m/s from the vehicle -> streaming
  set_cmd(Vec3(-0.5, 0.0, 0.0));
  goal(P + Vec3(3.0, 0.0, 0.0));
  yaw_cmd(std::numeric_limits<double>::quiet_NaN());
  const double t0 = wall();
  wsleep(1.5);
  const Vec3 p_fc = ANCHOR.pos(P);
  auto out = since(t0 + 0.2);
  ASSERT_GT(out.size(), 40u);
  const double yaw_held = out.front().yaw;
  for (const auto& m : out) {
    EXPECT_EQ(m.velocity.x, 0.0);
    EXPECT_NEAR(m.position.x, p_fc[0], 1e-6);
    EXPECT_NEAR(m.position.y, p_fc[1], 1e-6);
    EXPECT_NEAR(wrap_pi(m.yaw - yaw_held), 0.0, 1e-6);   // the nose does not follow the backward command
  }
  auto st = status_now();
  EXPECT_EQ(st["state"], "WAYPOINT_HOLD");
  EXPECT_EQ(st["waypoint_hold"]["phase"], "depart");
  set_cmd(Vec3(0.6, 0.0, 0.0));
  wsleep(0.3);
  const double t1 = wall();
  wsleep(0.5);
  const Vec3 v_fc = ANCHOR.vec(Vec3(0.6, 0.0, 0.0));
  auto out2 = since(t1);
  ASSERT_FALSE(out2.empty());
  for (const auto& m : out2) EXPECT_NEAR(m.velocity.x, v_fc[0], 1e-6);
  EXPECT_EQ(status_now()["state"], "STREAM");
}

TEST_F(Yaw, Y10_DepartureTimesOut) {
  // SUPER never sets off toward the goal: streaming resumes after ~depart_timeout_s (5 s)
  goal(P + Vec3(0.3, 0.0, 0.0));
  set_cmd(Vec3::Zero());
  wsleep(0.3);
  yaw_cmd(ANCHOR.psi);
  wsleep(0.5);
  goal(P + Vec3(-3.0, 0.0, 0.0));
  set_cmd(Vec3(0.6, 0.0, 0.0));   // away from the goal
  yaw_cmd(std::numeric_limits<double>::quiet_NaN());
  const double t0 = wall();
  wsleep(4.0);
  EXPECT_EQ(status_now()["state"], "WAYPOINT_HOLD");
  wsleep(1.7);
  EXPECT_EQ(status_now()["state"], "STREAM");
  const Vec3 v_fc = ANCHOR.vec(Vec3(0.6, 0.0, 0.0));
  auto out = since(t0 + 5.3);
  ASSERT_FALSE(out.empty());
  EXPECT_NEAR(out.back().velocity.x, v_fc[0], 1e-6);
  set_cmd(Vec3::Zero());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_bridge_yaw", ros::init_options::AnonymousName);
  return RUN_ALL_TESTS();
}
