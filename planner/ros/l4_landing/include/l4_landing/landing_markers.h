// The landing check's ROS-side helpers (no node, no master needed): /planner/landing_markers from an
// evaluation, the decision word and colour, and the executive's ground spots in the planner frame.
// C++ port of the helpers in planner/landing_check.py (2026-10-29).
#pragma once

#include <ros/time.h>
#include <visualization_msgs/MarkerArray.h>

#include <array>
#include <string>
#include <utility>
#include <vector>

#include "l4_landing/landing_core.h"

namespace l4_landing {

using RGB = std::array<double, 3>;
constexpr RGB GREEN{0.1, 0.8, 0.2}, RED{0.9, 0.1, 0.1}, AMBER{1.0, 0.65, 0.0}, LIGHT_BLUE{0.45, 0.75, 1.0};

struct GroundSpot {
  double x, y, yaw;   // the FC pose (odometry point) in the fc frame
};

// the executive's ground spots (fc frame) -> the odometry point's poses in the planner frame, with the
// bridge's anchor fc = Rz(psi) * planner + t
std::vector<KnownPose> ground_spots_to_planner(const std::vector<GroundSpot>& spots, double psi, double tx, double ty);
// -> (label word, rgb)
std::pair<std::string, RGB> decision(bool clear, const std::string& reason);
std::string label_text(bool clear, const Json& st);
visualization_msgs::MarkerArray build_markers(bool clear, const Json& st, const Viz& viz, const std::string& frame,
                                              const ros::Time& stamp, double fallback_z);

}  // namespace l4_landing
