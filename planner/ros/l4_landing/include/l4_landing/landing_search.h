// Landing-spot search (owner request 2026-10-30), ROS-free: in the air, about to land to drive, the spot under
// the vehicle is refused: find a spot within a radius that the landing check (LandingCheck, the same rules,
// footprint, flatness and observed-ground tests, unchanged) reports clear. See landing_search.cpp.
#pragma once

#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "l4_landing/landing_core.h"

namespace l4_landing {

struct SearchParams {
  double radius = 2.0;           // candidates within this of the vehicle (m)
  double step = 0.2;             // the candidate grid (m), centred on the vehicle
  double goal_weight = 0.5;      // cost = distance from the vehicle + goal_weight * (distance to the goal - the
                                 // vehicle's distance to the goal): 0 ignores the goal, 1 counts only the goal
  double tie_m = 0.15;           // clear candidates within this of the best cost: the one with the most margin
  double margin_max = 0.6;       // margin: distance to the nearest evaluated not-clear candidate, capped here
  double heading_min_dist = 0.3; // a candidate nearer the goal than this keeps the vehicle's yaw (as the executive's
                                 // waypoint_min_dist: no direction to face)
  double exclude_r = 0.3;        // candidates within this of an excluded point (spots already refused) are skipped
  double budget_s = 6.0;         // evaluation time cap (s): the candidates not evaluated by then stay unevaluated
  // line of sight (owner 2026-11-02): the straight line from the vehicle to a candidate at the check height (the
  // descent height; else the latest sensor's z), the robot's radius round it, against the window's returns
  double los_penalty = 10.0;     // added to a blocked candidate's cost (m): above the cost spread within the radius
                                 // (radius x (1 + goal_weight) x 2), so every open candidate ranks first; 0: off
  double los_radius = 0.35;      // the robot's radius (half-diagonal 0.34 m): returns within this of the line and
                                 // within this of the check height block it
  int los_min_points = 3;        // at least this many such returns block it (not one speck)
};

struct Candidate {
  V2 spot = V2::Zero();     // the odometry point's landing position (planner frame), as LandingCheck's spot
  double yaw = 0.0;         // the heading it would land with: facing the goal (the executive's waypoint yaw)
  double dist = 0.0;        // from the vehicle (m)
  double cost = 0.0;
  bool coarse = false;      // on the coarse grid (every other step): evaluated first, so the budget covers the radius
  bool excluded = false;
  bool evaluated = false;
  bool clear = false;
  double margin = 0.0;      // clear candidates: distance to the nearest evaluated not-clear one (<= margin_max)
  std::string reason;       // the check's reason ("clear", "occupied: ...", ...), "excluded", "" (not evaluated)
  std::optional<double> floor_z;   // the check's floor at the disc (for the markers)
  std::optional<bool> los_open;    // the line from the vehicle open at the check height (none: not checked)
  int los_points = 0;              // the returns that block it
  Json status;              // the check's status JSON (evaluated candidates)
};

struct SearchResult {
  std::vector<Candidate> cands;   // in cost order
  std::optional<size_t> best;     // index into cands
  int evaluated = 0;
  double elapsed_s = 0.0;
  bool budget_hit = false;
  std::optional<double> los_z;    // the height the lines were checked at (none: not checked)
  Json summary;   // found, counts per reason word, the best's numbers
};

// The search. scans: the landing check's window (oldest first, each with its sensor position); vehicle: the
// spot under the vehicle (planner frame, the executive's FC position as for /planner/landing_spot); vehicle_yaw:
// the yaw it has (planner frame); goal: the drive goal (planner frame) the candidates face, if any;
// descend_from: as LandingCheck (the hover height); known: the ground spots (known floor); exclude: spots already
// refused. clock: seconds (for the budget; default steady clock).
SearchResult search_landing_spot(LandingCheck& check, const std::vector<Scan>& scans, const V2& vehicle, double vehicle_yaw,
                                 std::optional<V2> goal, std::optional<double> descend_from,
                                 const std::vector<KnownPose>& known, const SearchParams& sp,
                                 const std::vector<V2>& exclude = {}, std::function<double()> clock = nullptr);

// the first word of a check reason ("occupied", "not observed", "not flat", "uncertain floor", "no ground plane",
// "clear", ...): the summary's counts
std::string reason_word(const std::string& reason);

}  // namespace l4_landing
