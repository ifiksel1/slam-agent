// Landing-spot check (D3), ROS-free: is a planned landing spot safe to land on?
// C++ port of planner/landing_core.py (2026-10-29): the same rules, defaults and status fields; the
// description of the model (disc on the frame, scan-by-scan floor alignment, known floor, near range)
// and its LIMITS are in landing_core.cpp's header comment.
#pragma once

#include <Eigen/Core>
#include <array>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "l4_track/track_core.h"

namespace l4_landing {

using l4_track::V2;
using Json = nlohmann::ordered_json;

constexpr double FOOTPRINT_LEN = 0.508;   // length, width (m)
constexpr double FOOTPRINT_WID = 0.457;

struct Scan {
  l4_track::Cloud pts;
  std::optional<Eigen::Vector3d> sensor;   // returns nearer than near_range to it are dropped
};

struct KnownPose {
  double x, y, yaw;   // the odometry point where the robot sat on the ground (planner frame)
};

// the last evaluation's geometry, for the markers (plain numbers, no ROS)
struct Viz {
  bool valid = false;   // false: no evaluation (no heading)
  V2 centre = V2::Zero();
  std::optional<double> yaw;
  double radius = 0;
  std::optional<double> floor_z;
  std::optional<std::pair<double, double>> plane;   // floor z = floor_z + a (x - cx) + b (y - cy)
  std::vector<std::array<double, 3>> occupied;      // cell centre x, y, top above the floor
  std::vector<V2> unobserved;
  double cell = 0.1;
  std::vector<std::array<V2, 4>> known_floor;        // footprint corners that reach the disc
  std::vector<V2> known_cells;
};

struct Params {
  double margin = 0.2, cell = 0.10, observed_min = 0.7, slope_max_deg = 5.0, residual_max = 0.03;
  double obstacle_min_h = 0.05;
  int obstacle_min_points = 5;
  double descent_top = 3.0, ground_band = 0.08, floor_layer_max = 0.20;
  int obstacle_min_cells = 2;
  double align_radius = 1.5, align_band = 0.3;
  int align_min_cells = 30;
  double seed_band = 0.15;
  int evidence_last = 10;
  double evidence_frac = 0.5;
  int evidence_latest = 3;
  double near_range = 1.2, unaligned_min_h = 0.3;
  double footprint_offset = l4_track::ODOM_AHEAD;
};

class LandingCheck {
 public:
  explicit LandingCheck(const Params& p = Params());
  // the disc centre (the frame centre) for the odometry point at spot heading yaw (none: the spot)
  V2 centre(const V2& spot, std::optional<double> yaw) const;
  std::array<V2, 4> footprint_corners(const V2& odom, double yaw) const;
  bool in_footprint(const V2& xy, const V2& odom, double yaw) const;
  // one merged cloud (one scan, no alignment)
  std::pair<bool, Json> evaluate(const l4_track::Cloud& points, const V2& spot, std::optional<double> descend_from = std::nullopt,
                                 std::optional<double> yaw = std::nullopt, const std::vector<KnownPose>& known = {});
  // scans, oldest first, each fitted to the local ground model
  std::pair<bool, Json> evaluate_scans(const std::vector<Scan>& scans, const V2& spot,
                                       std::optional<double> descend_from = std::nullopt,
                                       std::optional<double> yaw = std::nullopt, const std::vector<KnownPose>& known = {});
  Params p;
  double radius, align_radius, slope_max;
  l4_track::Config gcfg;   // the track planner's ground model (tile 0.5 m, 20th percentile)
  Viz viz;
};

}  // namespace l4_landing
