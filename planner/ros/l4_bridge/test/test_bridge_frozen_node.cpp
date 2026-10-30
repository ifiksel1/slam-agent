// A frozen SLAM pose (L2 faults 2b, 2d): the planner bridge with its defaults (~freeze_enable true) against
// synthetic inputs published by a background thread (FAST-LIO 20 Hz, SUPER 50 Hz, the track planner 10 Hz,
// the FC IMU 50 Hz). A moving pose carries a 0.1 mm jitter (a real FAST-LIO pose never repeats); a frozen one
// repeats bit for bit, with the vision and FC poses following it (the EKF fuses the frozen pose).
#include <gtest/gtest.h>

#include <atomic>
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
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
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
// the feeder's inputs
bool frozen = false;           // the SLAM pose repeats
Vec3 cmd_vel(0, 0, 0);         // SUPER's commanded velocity (flight)
bool drive = false;            // the track planner's carrot (2 m ahead) instead of SUPER
double gyro = 0.0;             // the FC IMU's angular rate, rad/s
}  // namespace

class Frozen : public ::testing::Test {
 protected:
  static inline ros::NodeHandle* nh;
  static inline ros::Publisher pub_enable, pub_mode;
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
    static ros::Publisher imu = nh->advertise<sensor_msgs::Imu>("/mavros/imu/data", 50);
    static ros::Publisher state = nh->advertise<mavros_msgs::State>("/mavros/state", 5, true);
    static ros::Publisher cmd = nh->advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 50);
    static ros::Publisher trk = nh->advertise<quadrotor_msgs::PositionCommand>("/track_planner/pos_cmd", 50);
    pub_enable = nh->advertise<std_msgs::Bool>("/planner_bridge/enable", 1, true);
    pub_mode = nh->advertise<std_msgs::String>("/planner_bridge/set_mode", 1, true);
    static ros::AsyncSpinner spinner(2);
    spinner.start();
    wsleep(2.0);
    mavros_msgs::State st;
    st.mode = "GUIDED";
    st.armed = true;
    state.publish(st);
    enable(true);
    feeder = new std::thread([] {
      const Quat q_pl = raw_to_planner_quat(Quat{0, 0, 0, 1});
      Vec3 p = P;   // the pose last published (a frozen one repeats it)
      for (int k = 0; !stop && ros::ok(); ++k) {
        bool fz, drv;
        Vec3 cv;
        double w;
        {
          std::lock_guard<std::mutex> g(mtx);
          fz = frozen;
          drv = drive;
          cv = cmd_vel;
          w = gyro;
        }
        if (k % 5 == 0) {
          if (!fz) p = P + Vec3(1e-4 * std::sin(0.7 * k), 1e-4 * std::cos(0.3 * k), 1e-4 * std::sin(0.1 * k));
          const Vec3 raw = position_matrix().transpose() * p;
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
          const Vec3 pf = ANCHOR.pos(p);
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
        if (k % 2 == 0) {
          sensor_msgs::Imu m;
          m.header.stamp = ros::Time::now();
          m.angular_velocity.z = w;
          imu.publish(m);
        }
        quadrotor_msgs::PositionCommand c;
        c.header.stamp = ros::Time::now();
        if (drv && k % 10 == 0) {
          // the track planner: a carrot 2 m ahead, position + yaw, zero velocity
          const Vec3 cp = p + Vec3(2.0, 0.0, 0.0);
          c.position.x = cp[0];
          c.position.y = cp[1];
          c.position.z = cp[2];
          c.yaw = 0.0;
          trk.publish(c);
        } else if (!drv && k % 2 == 0) {
          const Vec3 cp = p + cv * 0.3;
          c.position.x = cp[0];
          c.position.y = cp[1];
          c.position.z = cp[2];
          c.velocity.x = cv[0];
          c.velocity.y = cv[1];
          c.velocity.z = cv[2];
          cmd.publish(c);
        }
        wsleep(0.01);
      }
    });
    wsleep(2.0);
  }
  static void TearDownTestSuite() {
    stop = true;
    feeder->join();
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
  static void set(bool fz, const Vec3& cv, double w = 0.0, bool drv = false) {
    std::lock_guard<std::mutex> g(mtx);
    frozen = fz;
    cmd_vel = cv;
    gyro = w;
    drive = drv;
  }
  static l4::ojson status_now() {
    std::lock_guard<std::mutex> g(mtx);
    return l4::loads(status);
  }
  static std::vector<std::pair<double, PositionTarget>> since(double t) {
    std::lock_guard<std::mutex> g(mtx);
    std::vector<std::pair<double, PositionTarget>> out;
    for (const auto& s : sp)
      if (s.first >= t) out.push_back(s);
    return out;
  }
  // the executive's next leg, with the pose moving again: the latch is over
  void new_leg() {
    set(false, Vec3::Zero());
    enable(false);
    wsleep(0.4);
    enable(true);
    wsleep(1.0);
  }
};

TEST_F(Frozen, F1_AMovingPoseStreams) {
  set(false, Vec3(0.6, 0.0, 0.0));
  wsleep(2.5);
  auto st = status_now();
  EXPECT_EQ(st["state"], "STREAM");
  EXPECT_TRUE(st["slam_frozen"].is_null()) << st.dump();
}

TEST_F(Frozen, F2_ARepeatedPoseWithNoEvidenceOfMotionIsNotFrozen) {
  // SITL's pose while hovering on a zero command or parked: repeats, nothing says the vehicle moves (the
  // command at rest first: a freeze while the last motion command is still out is a freeze in motion)
  set(false, Vec3::Zero());
  wsleep(1.0);
  set(true, Vec3::Zero());
  wsleep(3.0);
  auto st = status_now();
  EXPECT_EQ(st["state"], "STREAM") << st.dump();
  ASSERT_TRUE(st["slam_frozen"].is_object()) << st.dump();
  EXPECT_GT(st["slam_frozen"]["repeats"].get<int>(), 20);
  EXPECT_EQ(st["slam_frozen"]["evidence"], "");
  EXPECT_EQ(st["slam_frozen"]["latched"], false);
  // (a burst of motion shorter than freeze_hold_s: test_bridge_core Frozen.EvidenceMustLastWithoutABreak)
  set(false, Vec3::Zero());
  wsleep(0.5);
}

TEST_F(Frozen, F3_RepeatedWhileCommandedLatchesAHoldAtTheVehicle) {
  set(false, Vec3(0.6, 0.0, 0.0));
  wsleep(1.5);
  ASSERT_EQ(status_now()["state"], "STREAM");
  set(true, Vec3(0.6, 0.0, 0.0));
  const double t_freeze = wall();
  wsleep(2.0);
  auto st = status_now();
  EXPECT_EQ(st["state"], "HOLD") << st.dump();
  EXPECT_EQ(st["reason"], SLAM_FROZEN);
  EXPECT_EQ(st["slam_hold"], false);   // not a SLAM reset to wait out: the executive lands or stops
  EXPECT_EQ(st["slam_frozen"]["latched"], true);
  std::optional<double> first_hold;
  for (const auto& [t, m] : since(t_freeze))
    if (m.velocity.x == 0.0 && m.velocity.y == 0.0) {
      first_hold = t;
      break;
    }
  ASSERT_TRUE(first_hold);
  // freeze_hold_s 0.4 after the first repeat (up to 0.05 s after the freeze), at least freeze_min_n 4 repeats
  // (0.2 s at 20 Hz), + the 50 Hz tick
  EXPECT_GT(*first_hold - t_freeze, 0.35);
  EXPECT_LT(*first_hold - t_freeze, 0.8);
  const auto held = since(*first_hold);
  EXPECT_GT(held.size(), 30u);   // streamed at the full rate (Guided does not time out)
  for (const auto& [t, m] : held) {
    EXPECT_EQ(m.type_mask, 2048);
    EXPECT_EQ(m.velocity.x, 0.0);
    EXPECT_EQ(m.velocity.y, 0.0);
  }
  // the pose moves again, SUPER plans on: still HOLD, latched (the leg is over)
  set(false, Vec3(0.6, 0.0, 0.0));
  wsleep(1.5);
  st = status_now();
  EXPECT_EQ(st["state"], "HOLD");
  EXPECT_EQ(st["reason"], SLAM_FROZEN);
  for (const auto& [t, m] : since(wall() - 0.5)) EXPECT_EQ(m.velocity.x, 0.0);
  // a new leg clears it
  new_leg();
  set(false, Vec3(0.6, 0.0, 0.0));
  wsleep(1.0);
  EXPECT_EQ(status_now()["state"], "STREAM") << status_now().dump();
}

TEST_F(Frozen, F4_TheFcGyroIsEvidence) {
  // a hover on a zero command, the pose repeating while the FC's IMU turns at 0.5 rad/s
  set(true, Vec3::Zero(), 0.5);
  wsleep(2.0);
  auto st = status_now();
  EXPECT_EQ(st["state"], "HOLD") << st.dump();
  EXPECT_EQ(st["reason"], SLAM_FROZEN);
  // a new leg while still frozen latches again at once
  enable(false);
  wsleep(0.3);
  enable(true);
  wsleep(0.5);
  EXPECT_EQ(status_now()["reason"], SLAM_FROZEN);
  new_leg();
  EXPECT_NE(status_now()["reason"], SLAM_FROZEN);
}

TEST_F(Frozen, F5_DrivingStopsAtOnce) {
  set_mode("drive");
  set(false, Vec3::Zero(), 0.0, true);
  wsleep(2.0);
  auto st = status_now();
  ASSERT_EQ(st["mode"], "drive");
  ASSERT_EQ(st["state"], "STREAM") << st.dump();
  set(true, Vec3::Zero(), 0.0, true);   // the carrot stays 2 m ahead of the frozen pose (f24)
  const double t_freeze = wall();
  wsleep(2.0);
  st = status_now();
  EXPECT_EQ(st["state"], "HOLD") << st.dump();
  EXPECT_EQ(st["reason"], SLAM_FROZEN);
  std::optional<double> first_stop;
  for (const auto& [t, m] : since(t_freeze))
    if (m.type_mask == 2503) {
      first_stop = t;
      break;
    }
  ASSERT_TRUE(first_stop);
  EXPECT_GT(*first_stop - t_freeze, 0.75);   // freeze_drive_hold_s 0.8
  EXPECT_LT(*first_stop - t_freeze, 1.3);
  const auto after = since(*first_stop);
  EXPECT_GT(after.size(), 30u);
  for (const auto& [t, m] : after) {
    EXPECT_EQ(m.type_mask, 2503);   // the zero-speed stop, never the carrot again
    EXPECT_EQ(m.velocity.x, 0.0);
    EXPECT_EQ(m.velocity.y, 0.0);
    EXPECT_NEAR(m.yaw, ANCHOR.psi, 1e-3);
  }
  new_leg();
}

TEST_F(Frozen, F6_DrivingOffFromRestIsNotFrozen) {
  // SITL's parked rover: the pose repeats until the rover moves; the carrot comes first. Moving within
  // freeze_drive_hold_s (0.8 s): no hold
  set(true, Vec3::Zero(), 0.0, false);   // parked, no carrot (the bridge holds: planner stale)
  wsleep(1.0);
  set(true, Vec3::Zero(), 0.0, true);
  wsleep(0.5);   // 0.5 s commanded with the pose repeating
  set(false, Vec3::Zero(), 0.0, true);
  wsleep(1.5);
  auto st = status_now();
  EXPECT_EQ(st["state"], "STREAM") << st.dump();
  EXPECT_TRUE(st["slam_frozen"].is_null()) << st.dump();
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_bridge_frozen", ros::init_options::AnonymousName);
  return RUN_ALL_TESTS();
}
