// Click and run: clicks -> interactive goals (see click_goal.h).
#include "l4_click/click_goal.h"

#include <cmath>

namespace l4_click {

Anchor Anchor::from(double x, double y, double z, double w, const Vec3& t) {
  Anchor a;
  a.R << 1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
         2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
         2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y);
  a.t = t;
  return a;
}

double Anchor::yaw() const { return std::atan2(R(1, 0), R(0, 0)); }

double quat_yaw(double x, double y, double z, double w) {
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

l4::ojson drive_goal(const Vec3& click_planner, const Anchor& a) {
  const Vec3 fc = a.to_fc(click_planner);
  return {{"type", "drive"}, {"goal", {l4::py_round(fc[0], 3), l4::py_round(fc[1], 3)}}};
}

l4::ojson fly_goal(const Vec3& click_planner, double yaw_planner, const Anchor& a, double floor_fc_z, double height,
                   bool with_yaw) {
  const Vec3 fc = a.to_fc(click_planner);
  l4::ojson g = {{"type", "fly"},
                 {"goal", {l4::py_round(fc[0], 3), l4::py_round(fc[1], 3), l4::py_round(floor_fc_z + height, 3)}}};
  if (with_yaw) {
    const double y = std::atan2(std::sin(yaw_planner + a.yaw()), std::cos(yaw_planner + a.yaw()));
    g["yaw"] = l4::py_round(y * 180.0 / M_PI, 1);
  }
  return g;
}

std::optional<std::string> goal_state(const l4::ojson& s, int id) {
  if (!s.is_object()) return std::nullopt;
  const l4::ojson phase = l4::get(s, "phase");
  if (phase == "STOPPED") return std::string("stopped");   // the executive is done for good
  // a status names its goal as "goal" (interactive phases) or "leg" (the legs' own reports)
  l4::ojson g = l4::get(s, "goal", l4::get(s, "leg"));
  if (!g.is_number_integer() || g.get<int>() != id) return std::nullopt;
  if (phase == "REACHED") return std::string("reached");
  if (phase == "UNREACHABLE" || phase == "REFUSED") return std::string("failed");
  if (phase == "PREEMPTED") return std::string("replaced");
  return std::string("active");
}

}  // namespace l4_click
