// A planner in its own process, to kill (the node tests): publishes a fixed PositionCommand at a rate.
//   fake_planner <node name> <topic> <px,py,pz> <vx,vy,vz> <hz>
#include <cstdio>
#include <string>

#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>

static void parse3(const char* s, double* v) { std::sscanf(s, "%lf,%lf,%lf", &v[0], &v[1], &v[2]); }

int main(int argc, char** argv) {
  if (argc < 6) return 2;
  const std::string name = argv[1], topic = argv[2];
  double p[3], v[3];
  parse3(argv[3], p);
  parse3(argv[4], v);
  const double hz = std::stod(argv[5]);
  ros::init(argc, argv, name, ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<quadrotor_msgs::PositionCommand>(topic, 10);
  ros::Rate r(hz);
  while (ros::ok()) {
    quadrotor_msgs::PositionCommand c;
    c.header.stamp = ros::Time::now();
    c.position.x = p[0]; c.position.y = p[1]; c.position.z = p[2];
    c.velocity.x = v[0]; c.velocity.y = v[1]; c.velocity.z = v[2];
    c.yaw = 0.1;
    pub.publish(c);
    r.sleep();
  }
  return 0;
}
