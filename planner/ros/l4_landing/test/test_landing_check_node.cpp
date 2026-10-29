// rostest: the landing_check node on synthetic topics: a spot without odometry -> "no heading"; a floor
// seen from 2 m with odometry -> clear, the status JSON's fields, /planner/landing_markers; a box on
// the spot -> occupied; the ground spots and anchor -> "known_floor".
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <gtest/gtest.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>

#include <nlohmann/json.hpp>
#include <random>

namespace {
std::vector<nlohmann::json> statuses;
std::vector<bool> clears;
visualization_msgs::MarkerArray markers;
int n_markers = 0;

sensor_msgs::PointCloud2 cloud(bool box) {
  static std::mt19937 rng(2);
  std::normal_distribution<float> N(0.f, 0.005f);
  std::vector<std::array<float, 3>> pts;
  for (float x = -2; x <= 2; x += 0.04f)
    for (float y = -2; y <= 2; y += 0.04f) {
      bool in_box = std::abs(x + 0.19f) <= 0.15f && std::abs(y) <= 0.15f;
      if (box && in_box) continue;
      pts.push_back({x, y, N(rng)});
    }
  if (box)
    for (float x = -0.34f; x <= 0.0f; x += 0.03f)
      for (float y = -0.15f; y <= 0.15f; y += 0.03f)
        for (float z = 0; z <= 0.3f; z += 0.03f) pts.push_back({x, y, z});
  sensor_msgs::PointCloud2 c;
  c.header.frame_id = "world";
  c.header.stamp = ros::Time::now();
  sensor_msgs::PointCloud2Modifier mod(c);
  mod.setPointCloud2FieldsByString(1, "xyz");
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

TEST(LandingCheckNode, AnswersTheExecutive) {
  ros::NodeHandle nh;
  auto s1 = nh.subscribe<std_msgs::String>("/planner/landing_status", 50, (boost::function<void(const std_msgs::String::ConstPtr&)>)[](const std_msgs::String::ConstPtr& m) { statuses.push_back(nlohmann::json::parse(m->data)); });
  auto s2 = nh.subscribe<std_msgs::Bool>("/planner/landing_clear", 50, (boost::function<void(const std_msgs::Bool::ConstPtr&)>)[](const std_msgs::Bool::ConstPtr& m) { clears.push_back(m->data); });
  auto s3 = nh.subscribe<visualization_msgs::MarkerArray>("/planner/landing_markers", 10, (boost::function<void(const visualization_msgs::MarkerArray::ConstPtr&)>)[](const visualization_msgs::MarkerArray::ConstPtr& m) {
                                                            markers = *m;
                                                            n_markers++;
                                                          });
  auto po = nh.advertise<nav_msgs::Odometry>("/planner/odom", 10);
  auto pc = nh.advertise<sensor_msgs::PointCloud2>("/planner/cloud", 10);
  auto ps = nh.advertise<geometry_msgs::PointStamped>("/planner/landing_spot", 10);
  auto pgs = nh.advertise<std_msgs::String>("/mission_executive/ground_spots", 1, true);
  auto pa = nh.advertise<geometry_msgs::TransformStamped>("/planner_bridge/anchor", 1, true);
  ASSERT_TRUE(wait_for([&] { return po.getNumSubscribers() && pc.getNumSubscribers() && ps.getNumSubscribers(); }, 20.0));
  geometry_msgs::PointStamped spot;
  spot.header.frame_id = "world";
  spot.point.z = 2.0;
  ps.publish(spot);
  ASSERT_TRUE(wait_for([] { return !statuses.empty(); }, 5.0));
  EXPECT_EQ(statuses.back()["reason"], "no heading: no /planner/odom yet");
  EXPECT_FALSE(clears.back());
  // the robot hovering 2 m up, 1.5 m off the spot, heading +x
  nav_msgs::Odometry o;
  o.header.frame_id = "world";
  o.pose.pose.position.x = -1.5;
  o.pose.pose.position.z = 2.0;
  o.pose.pose.orientation.w = 1.0;
  auto feed = [&](bool box) {
    for (int k = 0; k < 4; ++k) {
      o.header.stamp = ros::Time::now();
      po.publish(o);
      ros::WallDuration(0.05).sleep();
      pc.publish(cloud(box));
      ros::WallDuration(0.1).sleep();
      ps.publish(spot);
    }
  };
  feed(false);
  ASSERT_TRUE(wait_for([] { return statuses.back().value("reason", "") == "clear"; }, 5.0)) << statuses.back().dump();
  auto st = statuses.back();
  EXPECT_TRUE(clears.back());
  for (auto k : {"spot", "centre", "radius_m", "yaw_deg", "footprint_offset_m", "points", "model_z", "ground_z", "slope_deg",
                 "residual_m", "observed", "floor_layer_m", "obstacle_threshold_m", "obstacle_points", "obstacle_cells",
                 "tall_points", "hit_cells", "max_obstacle_h", "obstacle_xy", "reason", "scans", "eval_ms"})
    EXPECT_TRUE(st.contains(k)) << k;
  EXPECT_EQ(st["centre"][0].get<double>(), -0.19);
  ASSERT_TRUE(wait_for([] { return n_markers > 0; }, 3.0));
  EXPECT_EQ(markers.markers.size(), 6u);
  EXPECT_EQ(markers.markers[0].ns, "landing_disc");
  // known floor: the ground spots and the anchor
  std_msgs::String gs;
  gs.data = "{\"frame\": \"fc\", \"spots\": [{\"event\": \"takeoff\", \"xyz\": [0.0, 0.0, 0.0], \"yaw\": 0.0, \"t\": 1.0}]}";
  pgs.publish(gs);
  wait_for([] { return false; }, 0.5);
  ps.publish(spot);
  ASSERT_TRUE(wait_for([] { return statuses.back().value("known_floor", "") == "no anchor"; }, 3.0)) << statuses.back().dump();
  geometry_msgs::TransformStamped a;
  a.transform.rotation.w = 1.0;
  pa.publish(a);
  ASSERT_TRUE(wait_for([] {
    return statuses.back().value("known_floor", "") == "1 ground spots" && statuses.back().contains("known_footprints");
  }, 3.0)) << statuses.back().dump();
  // a box on the spot, under the frame
  feed(true);
  feed(true);
  ASSERT_TRUE(wait_for([] { return statuses.back().value("reason", "").rfind("occupied", 0) == 0; }, 5.0))
      << statuses.back().dump();
  EXPECT_FALSE(clears.back());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_landing_check_node");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
