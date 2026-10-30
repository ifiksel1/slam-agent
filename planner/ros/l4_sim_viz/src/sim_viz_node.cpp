// sim_viz (SIM ONLY, click and run 2026-10-30): what a live 3D view of an L2 closed-loop run needs and the
// run does not publish. The robot has its own (FAST-LIO's map); nothing here runs on it.
//
//   /map, /map_low  sensor_msgs/PointCloud2, latched, "world" (the planner frame): the run's map (~pcd, the
//                   PCD the render node raycasts), moved into the planner frame with /sim/map_to_planner
//                   (planner = Rz(-yaw_deg) (map - start), as foxglove_bag.py), on a ~grid (0.15 m) grid;
//                   /map_low only between ~low_z0 and ~low_z1 above the map's z = 0 (SITL's ground; 0.3 .. 1.8 m:
//                   walls and obstacles without the floor's smear or the ceilings, for top-down views)
//   /tf             world -> drone from /planner/odom (the 3D panel can follow the robot)
//   /robot/body     visualization_msgs/Marker: the frame (0.508 x 0.457 x 0.203 m box) at the odometry
//
// The PCD: binary or ascii, fields x y z (float32) among any others.
#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

namespace {

struct P3 {
  float x, y, z;
};

// x y z of a PCD file (binary or ascii)
std::vector<P3> read_pcd(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot read " + path);
  std::vector<std::string> fields;
  std::vector<int> sizes, counts;
  std::string types_line, data, line;
  size_t points = 0;
  while (std::getline(f, line)) {
    std::istringstream ss(line);
    std::string key;
    ss >> key;
    if (key == "FIELDS") {
      for (std::string v; ss >> v;) fields.push_back(v);
    } else if (key == "SIZE") {
      for (int v; ss >> v;) sizes.push_back(v);
    } else if (key == "COUNT") {
      for (int v; ss >> v;) counts.push_back(v);
    } else if (key == "POINTS") {
      ss >> points;
    } else if (key == "DATA") {
      ss >> data;
      break;
    }
  }
  if (counts.empty()) counts.assign(fields.size(), 1);
  int ix = -1, iy = -1, iz = -1, step = 0;
  std::vector<int> offs;
  for (size_t i = 0; i < fields.size(); ++i) {
    offs.push_back(step);
    if (fields[i] == "x") ix = static_cast<int>(i);
    if (fields[i] == "y") iy = static_cast<int>(i);
    if (fields[i] == "z") iz = static_cast<int>(i);
    step += sizes.at(i) * counts.at(i);
  }
  if (ix < 0 || iy < 0 || iz < 0) throw std::runtime_error(path + ": no x y z fields");
  std::vector<P3> out;
  out.reserve(points);
  if (data == "binary") {
    std::vector<char> buf(static_cast<size_t>(step) * points);
    f.read(buf.data(), static_cast<std::streamsize>(buf.size()));
    const size_t n = static_cast<size_t>(f.gcount()) / step;
    for (size_t k = 0; k < n; ++k) {
      const char* r = buf.data() + k * step;
      P3 p;
      std::memcpy(&p.x, r + offs[ix], 4);
      std::memcpy(&p.y, r + offs[iy], 4);
      std::memcpy(&p.z, r + offs[iz], 4);
      out.push_back(p);
    }
  } else if (data == "ascii") {
    while (std::getline(f, line)) {
      std::istringstream ss(line);
      std::vector<double> v;
      for (double d; ss >> d;) v.push_back(d);
      if (static_cast<int>(v.size()) > std::max({ix, iy, iz}))
        out.push_back({static_cast<float>(v[ix]), static_cast<float>(v[iy]), static_cast<float>(v[iz])});
    }
  } else {
    throw std::runtime_error(path + ": DATA " + data + " not supported (binary or ascii)");
  }
  return out;
}

sensor_msgs::PointCloud2 cloud(const std::vector<P3>& pts, const std::string& frame) {
  sensor_msgs::PointCloud2 c;
  c.header.frame_id = frame;
  c.header.stamp = ros::Time::now();
  c.height = 1;
  c.width = static_cast<uint32_t>(pts.size());
  const char* names[3] = {"x", "y", "z"};
  for (int i = 0; i < 3; ++i) {
    sensor_msgs::PointField pf;
    pf.name = names[i];
    pf.offset = 4 * i;
    pf.datatype = sensor_msgs::PointField::FLOAT32;
    pf.count = 1;
    c.fields.push_back(pf);
  }
  c.is_bigendian = false;
  c.point_step = 12;
  c.row_step = 12 * c.width;
  c.is_dense = true;
  c.data.resize(c.row_step);
  std::memcpy(c.data.data(), pts.data(), c.data.size());
  return c;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "sim_viz");
  ros::NodeHandle nh, pnh("~");
  std::string pcd;
  if (!pnh.getParam("pcd", pcd)) {
    const char* e = std::getenv("MAP_PCD");   // the run's map (sitl_harness.sh sets it in the ROS container)
    if (e) pcd = e;
  }
  double grid = pnh.param("grid", 0.15), low0 = pnh.param("low_z0", 0.3), low1 = pnh.param("low_z1", 1.8);
  std::string frame = pnh.param<std::string>("frame", "world");

  ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map", 1, true);
  ros::Publisher low_pub = nh.advertise<sensor_msgs::PointCloud2>("/map_low", 1, true);
  ros::Publisher body_pub = nh.advertise<visualization_msgs::Marker>("/robot/body", 5);
  tf2_ros::TransformBroadcaster tf;

  // the map, in the planner frame, once /sim/map_to_planner is set (the render node sets it at its start)
  if (!pcd.empty()) {
    XmlRpc::XmlRpcValue m2p;
    while (ros::ok() && !ros::param::get("/sim/map_to_planner", m2p)) ros::Duration(0.5).sleep();
    double yaw = 0, sx = 0, sy = 0, sz = 0;
    try {
      yaw = static_cast<double>(m2p["yaw_deg"]) * M_PI / 180.0;
      XmlRpc::XmlRpcValue& s = m2p["start"];
      auto d = [](XmlRpc::XmlRpcValue& v) {
        return v.getType() == XmlRpc::XmlRpcValue::TypeInt ? static_cast<double>(static_cast<int>(v)) : static_cast<double>(v);
      };
      sx = d(s[0]);
      sy = d(s[1]);
      sz = d(s[2]);
    } catch (...) {
      ROS_ERROR("sim_viz: /sim/map_to_planner: not {yaw_deg, start: [x, y, z]}");
    }
    try {
      const std::vector<P3> raw = read_pcd(pcd);
      const double c = std::cos(-yaw), s = std::sin(-yaw);
      std::unordered_set<long long> seen;
      std::vector<P3> all, low;
      for (const P3& p : raw) {
        const double dx = p.x - sx, dy = p.y - sy, dz = p.z - sz;
        const P3 q{static_cast<float>(c * dx - s * dy), static_cast<float>(s * dx + c * dy), static_cast<float>(dz)};
        const long long kx = static_cast<long long>(std::floor(q.x / grid)) & 0x1FFFFF;
        const long long ky = static_cast<long long>(std::floor(q.y / grid)) & 0x1FFFFF;
        const long long kz = static_cast<long long>(std::floor(q.z / grid)) & 0x1FFFFF;
        if (!seen.insert((kx << 42) | (ky << 21) | kz).second) continue;
        all.push_back(q);
        if (p.z >= low0 && p.z <= low1) low.push_back(q);   // map z: above SITL's ground
      }
      map_pub.publish(cloud(all, frame));
      low_pub.publish(cloud(low, frame));
      ROS_INFO("sim_viz: %s: %zu points, %zu on the %.2f m grid, %zu in /map_low", pcd.c_str(), raw.size(), all.size(),
               grid, low.size());
    } catch (const std::exception& e) {
      ROS_ERROR("sim_viz: %s", e.what());
    }
  } else {
    ROS_WARN("sim_viz: no ~pcd and no MAP_PCD: no /map");
  }

  ros::Time last_body;
  auto on_odom = [&](const nav_msgs::Odometry::ConstPtr& o) {
    geometry_msgs::TransformStamped t;
    t.header.stamp = o->header.stamp;
    t.header.frame_id = o->header.frame_id.empty() ? frame : o->header.frame_id;
    t.child_frame_id = "drone";
    t.transform.translation.x = o->pose.pose.position.x;
    t.transform.translation.y = o->pose.pose.position.y;
    t.transform.translation.z = o->pose.pose.position.z;
    t.transform.rotation = o->pose.pose.orientation;
    tf.sendTransform(t);
    if ((o->header.stamp - last_body).toSec() < 0.05) return;   // 20 Hz
    last_body = o->header.stamp;
    visualization_msgs::Marker m;
    m.header = o->header;
    m.header.frame_id = t.header.frame_id;
    m.ns = "body";
    m.type = visualization_msgs::Marker::CUBE;
    m.pose = o->pose.pose;
    m.scale.x = 0.508;
    m.scale.y = 0.457;
    m.scale.z = 0.203;
    m.color.r = 0.2f;
    m.color.g = 0.8f;
    m.color.b = 0.35f;
    m.color.a = 0.9f;
    m.lifetime = ros::Duration(0.5);
    body_pub.publish(m);
  };
  ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>("/planner/odom", 20, on_odom);
  ros::spin();
  return 0;
}
