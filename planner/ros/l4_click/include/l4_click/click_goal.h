// Click and run (2026-10-30): a click in the planner frame -> a goal for the mission executive's interactive
// mode (/mission_executive/goal, JSON, fc frame). ROS-free.
//
//   a point click  -> {"type": "drive", "goal": [x, y]}                 drive there on the ground
//   a pose click   -> {"type": "fly", "goal": [x, y, floor + height],   fly there, `height` above the floor,
//                      "yaw": deg}                                       facing the pose's heading
//
// Frames (the shared contract): the planner frame ("world": SUPER's, the bridge's /planner/cloud) is where
// the 3D panel clicks; the fc frame (the FC's EKF frame, MAVROS local ENU) is where the executive takes
// goals (D7). The bridge's /planner_bridge/anchor is fc <- planner: p_fc = R p_planner + t.
#pragma once

#include <optional>
#include <string>

#include <Eigen/Core>
#include <l4_common/pyjson.h>

namespace l4_click {

using Vec3 = Eigen::Vector3d;

struct Anchor {   // fc <- planner
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Vec3 t = Vec3::Zero();
  // from a quaternion (x, y, z, w) and a translation
  static Anchor from(double qx, double qy, double qz, double qw, const Vec3& t);
  Vec3 to_fc(const Vec3& p_planner) const { return R * p_planner + t; }
  Vec3 to_planner(const Vec3& p_fc) const { return R.transpose() * (p_fc - t); }
  double yaw() const;   // the rotation's heading (rad)
};

// a yaw (rad) from a quaternion (x, y, z, w)
double quat_yaw(double qx, double qy, double qz, double qw);

// a point click (planner frame) -> a drive goal (fc)
l4::ojson drive_goal(const Vec3& click_planner, const Anchor& a);

// a pose click (planner frame, its heading in the planner frame) -> a fly goal (fc) `height` above floor_fc_z;
// with_yaw false: no "yaw" (the executive holds the yaw it arrives with)
l4::ojson fly_goal(const Vec3& click_planner, double yaw_planner, const Anchor& a, double floor_fc_z, double height,
                   bool with_yaw = true);

// the executive's status for our goal (`id`: its number, from "goal_received") -> the marker's state:
// "sent" (not yet taken), "active", "reached", "failed", "replaced", "rejected", "stopped"; none when the
// status is not about this goal
std::optional<std::string> goal_state(const l4::ojson& status, int id);

}  // namespace l4_click
