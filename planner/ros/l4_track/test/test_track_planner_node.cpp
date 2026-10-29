// rostest: the track_planner node on synthetic topics: a floor scan, odometry, a drive goal ->
// /track_planner/status (JSON fields as the Python node's), /track_planner/path, the carrot on
// /track_planner/pos_cmd; then the robot at the goal -> REACHED and no more commands.
#include <geometry_msgs/PoseStamped.h>
#include <gtest/gtest.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/String.h>

#include <nlohmann/json.hpp>

namespace {
std::vector<nlohmann::json> statuses;
std::vector<quadrotor_msgs::PositionCommand> cmds;
nav_msgs::Path last_path;
int n_path = 0;

sensor_msgs::PointCloud2 floor_cloud() {
  sensor_msgs::PointCloud2 c;
  c.header.frame_id = "world";
  c.header.stamp = ros::Time::now();
  sensor_msgs::PointCloud2Modifier mod(c);
  mod.setPointCloud2FieldsByString(1, "xyz");
  std::vector<std::array<float, 3>> pts;
  for (float x = -4; x < 6; x += 0.05f)
    for (float y = -3; y < 3; y += 0.05f) pts.push_back({x, y, 0.0f});
  mod.resize(pts.size());
  sensor_msgs::PointCloud2Iterator<float> ix(c, "x"), iy(c, "y"), iz(c, "z");
  for (auto& p : pts) {
    *ix = p[0];
    *iy = p[1];
    *iz = p[2];
    ++ix;
    ++iy;
    ++iz;
  }
  return c;
}

nav_msgs::Odometry odom(double x, double y) {
  nav_msgs::Odometry o;
  o.header.frame_id = "world";
  o.header.stamp = ros::Time::now();
  o.pose.pose.position.x = x;
  o.pose.pose.position.y = y;
  o.pose.pose.position.z = 0.1;
  o.pose.pose.orientation.w = 1.0;
  return o;
}

template <class F>
bool wait_for(F f, double s) {
  ros::WallTime end = ros::WallTime::now() + ros::WallDuration(s);
  while (ros::WallTime::now() < end) {
    ros::spinOnce();
    if (f()) return true;
    ros::WallDuration(0.02).sleep();
  }
  return false;
}
}  // namespace

TEST(TrackPlannerNode, PlansStreamsAndReaches) {
  ros::NodeHandle nh;
  auto s1 = nh.subscribe<std_msgs::String>("/track_planner/status", 10, (boost::function<void(const std_msgs::String::ConstPtr&)>)[](const std_msgs::String::ConstPtr& m) { statuses.push_back(nlohmann::json::parse(m->data)); });
  auto s2 = nh.subscribe<quadrotor_msgs::PositionCommand>("/track_planner/pos_cmd", 10, (boost::function<void(const quadrotor_msgs::PositionCommand::ConstPtr&)>)[](const quadrotor_msgs::PositionCommand::ConstPtr& m) { cmds.push_back(*m); });
  auto s3 = nh.subscribe<nav_msgs::Path>("/track_planner/path", 10, (boost::function<void(const nav_msgs::Path::ConstPtr&)>)[](const nav_msgs::Path::ConstPtr& m) {
    last_path = *m;
    n_path++;
  });
  auto po = nh.advertise<nav_msgs::Odometry>("/planner/odom", 10);
  auto pc = nh.advertise<sensor_msgs::PointCloud2>("/planner/cloud", 10);
  auto pg = nh.advertise<geometry_msgs::PoseStamped>("/planning/drive_goal", 1);
  ASSERT_TRUE(wait_for([&] { return po.getNumSubscribers() && pc.getNumSubscribers() && pg.getNumSubscribers(); }, 20.0));
  for (int k = 0; k < 5; ++k) {
    po.publish(odom(0, 0));
    ros::WallDuration(0.05).sleep();
    pc.publish(floor_cloud());
    ros::WallDuration(0.1).sleep();
  }
  geometry_msgs::PoseStamped g;
  g.header.frame_id = "world";
  g.pose.position.x = 3.0;
  g.pose.orientation.w = 1.0;
  pg.publish(g);
  ASSERT_TRUE(wait_for([] { return !statuses.empty() && statuses.back()["reason"] == "path found"; }, 10.0));
  auto& st = statuses.back();
  EXPECT_EQ(st["state"], "DRIVING");
  for (auto k : {"state", "reason", "plan_ms", "phase", "pivots"}) EXPECT_TRUE(st.contains(k)) << k;
  EXPECT_TRUE(st["plan_ms"].is_number());
  ASSERT_TRUE(wait_for([] { return n_path > 0 && cmds.size() >= 3; }, 5.0));
  EXPECT_NEAR(last_path.poses.back().pose.position.x, 3.0, 1e-6);
  auto& c = cmds.back();
  EXPECT_EQ(c.header.frame_id, "world");
  EXPECT_EQ(c.trajectory_flag, quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY);
  // the carrot run out along the bearing: up to runout_s x 1.0 m/s (the clamped drive_speed) from the
  // FC at the frame centre, 0.193 m behind the odometry point; never past the goal
  EXPECT_GT(c.position.x, 0.5);
  EXPECT_LE(c.position.x, 3.0 + 1e-6);
  EXPECT_LE(c.position.x - (-0.193), 2.1 * 1.0 + 1e-6);
  EXPECT_NEAR(c.yaw, 0.0, 1e-6);
  // at the goal: REACHED, no more commands
  po.publish(odom(2.9, 0.0));
  ASSERT_TRUE(wait_for([] { return statuses.back()["state"] == "REACHED"; }, 5.0));
  ros::WallDuration(0.3).sleep();
  ros::spinOnce();
  size_t n = cmds.size();
  wait_for([] { return false; }, 0.5);
  EXPECT_EQ(cmds.size(), n);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_track_planner_node");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
