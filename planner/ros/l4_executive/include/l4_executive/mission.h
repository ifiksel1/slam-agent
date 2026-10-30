// The D6 mission file and QuadRover's mode numbers (ROS-free; yaml-cpp). Port of load_mission() and
// mode_number() of the former planner/mission_executive.py.
//
// A mission is read into JSON values with PyYAML's safe_load rules (YAML 1.1 scalars: 1 is an int, 1.0 a
// float, yes/no booleans, ~ null), so every value the executive echoes in its status JSON (a leg's speed,
// a takeoff height) is written exactly as the Python executive wrote it.
#pragma once

#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <l4_common/pyjson.h>

namespace YAML {
class Node;
}

namespace l4_executive {

using l4::ojson;

// QuadRover custom modes (Rover/mode.h)
const std::map<std::string, int>& modes();
// D5: the only modes the executive may set
bool allowed_mode(const std::string& name);
extern const char* const NOT_STATIONARY;   // "not stationary before takeoff"
extern const char* const PLANNER_DIED;     // "planner died"
extern const char* const SLAM_FROZE;       // "SLAM pose frozen"
constexpr int MAV_CMD_DO_SET_MODE = 176;
constexpr int MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1;

// MAVROS's mode string to QuadRover's number: a known name, or CMODE(n) for the QR modes
std::optional<int> mode_number(const std::string& name);

struct Mission {
  ojson name;              // the file's name, or its path
  double takeoff_height;   // the takeoff a mission starting with fly / land gets
  std::vector<ojson> legs; // each leg with the defaults filled in (a JSON object, keys as in the file)
};

// what an unreachable goal does (owner 2026-10-27: next, may change)
bool unreachable_policy_ok(const std::string& p);   // next | stop

// the D6 mission: defaults filled in per leg; throws std::invalid_argument on a malformed file
Mission load_mission(const std::string& path);
Mission load_mission_text(const std::string& text, const std::string& path);

// a YAML node as PyYAML's safe_load would give it, as JSON
ojson yaml_to_json(const YAML::Node& n);

// a JSON number as a float (Python's float(x) on a number or a bool); throws on anything else
double num(const ojson& v);

}  // namespace l4_executive

namespace l4_executive {

// Hover and yaw settle (owner 2026-10-29: land only once the yaw has settled): FC poses (time, position,
// yaw) over the last `window` s; settled when they cover the window, the horizontal position stayed within
// speed_max * window of the newest, the yaw within rate_max * window of the newest (a rate under rate_max
// over the window) and, with a target, the yaw within tol of it.
class SettleWindow {
 public:
  explicit SettleWindow(double window = 1.0) : window_(window) {}
  void add(double t, const Eigen::Vector3d& p, double yaw);
  bool settled(double speed_max, double rate_max, std::optional<double> target = std::nullopt, double tol = 0.0) const;
  void clear() { s_.clear(); }

 private:
  struct S { double t; Eigen::Vector3d p; double yaw; };
  double window_;
  std::vector<S> s_;
};

double wrap_pi(double a);

// SUPER's re-seed goal altitude (fc frame, m; owner 2026-10-29). Container fly-through k1: each seed went
// 0.3 m above where the vehicle was, SUPER flew to it, and the next seed went 0.3 m above that (1.05 ->
// 1.35 -> 1.64 m under a 2.15 m roof). Now the seed is `dz` from the vehicle, up when the vehicle is at or
// below the leg's planned altitude there, down when above it. The planned altitude: the straight line from
// the leg's start to its goal, at the vehicle's horizontal progress along it (clamped to the leg; a leg with
// no horizontal length: the vehicle's altitude clamped between the start's and the goal's); a seed (a
// vertical move) does not change it. So a seed is always a real dz move (SUPER needs one from rest, Gate L2
// b7; a seed at plan + dz with the vehicle already there was a 5 mm move in SITL m2 and leg 7 went
// unreachable), never higher than plan + dz unless the vehicle is already higher (then it goes down), and
// repeats do not stack: they alternate within plan +- dz. ceiling_above (the bridge's "ceiling_above": from
// the vehicle up to the lowest point over it, m): the seed at most `ceiling_clear` (SUPER's inflation and a
// cell) below that point; capped then (it may be below the vehicle: a move away from the ceiling).
struct SeedAltitude {
  double z;        // the seed's altitude
  double plan_z;   // the leg's planned altitude at the vehicle
  bool capped;     // held down by the ceiling
};
SeedAltitude seed_altitude(const Eigen::Vector3d& start, const Eigen::Vector3d& goal, const Eigen::Vector3d& here,
                           double dz, std::optional<double> ceiling_above, double ceiling_clear);

}  // namespace l4_executive
