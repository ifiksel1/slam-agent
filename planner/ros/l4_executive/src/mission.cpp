// The D6 mission file (see mission.h).
#include "l4_executive/mission.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <regex>
#include <sstream>

#include <yaml-cpp/yaml.h>

namespace l4_executive {

const char* const NOT_STATIONARY = "not stationary before takeoff";
const char* const PLANNER_DIED = "planner died";

const std::map<std::string, int>& modes() {
  static const std::map<std::string, int> m = {
      {"MANUAL", 0},       {"HOLD", 4},      {"AUTO", 10},     {"RTL", 11},     {"SMART_RTL", 12},
      {"GUIDED", 15},      {"QRSTABILIZE", 17}, {"QRALTHOLD", 18}, {"QRLOITER", 19}, {"QRRTL", 20},
      {"QRLAND", 21},      {"QRCIRCLE", 22}, {"QRSMARTRTL", 23}};
  return m;
}

bool allowed_mode(const std::string& name) {
  return name == "GUIDED" || name == "QRLOITER" || name == "QRLAND" || name == "QRSMARTRTL";
}

std::optional<int> mode_number(const std::string& name) {
  static const std::regex cmode(R"(CMODE\((\d+)\))");
  std::smatch m;
  if (std::regex_match(name, m, cmode)) return std::stoi(m[1].str());
  auto it = modes().find(name);
  if (it == modes().end()) return std::nullopt;
  return it->second;
}

bool unreachable_policy_ok(const std::string& p) { return p == "next" || p == "stop"; }

double num(const ojson& v) {
  if (v.is_number()) return v.get<double>();
  if (v.is_boolean()) return v.get<bool>() ? 1.0 : 0.0;
  throw std::invalid_argument("not a number: " + l4::dumps(v));
}

namespace {

// PyYAML's implicit resolvers (YAML 1.1), for plain scalars
ojson resolve_plain(const std::string& s) {
  static const std::regex re_null(R"(^(?:~|null|Null|NULL|)$)");
  static const std::regex re_true(R"(^(?:yes|Yes|YES|true|True|TRUE|on|On|ON)$)");
  static const std::regex re_false(R"(^(?:no|No|NO|false|False|FALSE|off|Off|OFF)$)");
  static const std::regex re_int(
      R"(^(?:[-+]?0b[0-1_]+|[-+]?0[0-7_]+|[-+]?(?:0|[1-9][0-9_]*)|[-+]?0x[0-9a-fA-F_]+|[-+]?[1-9][0-9_]*(?::[0-5]?[0-9])+)$)");
  static const std::regex re_float(
      R"(^(?:[-+]?(?:[0-9][0-9_]*)\.[0-9_]*(?:[eE][-+][0-9]+)?|\.[0-9_]+(?:[eE][-+][0-9]+)?|[-+]?[0-9][0-9_]*(?::[0-5]?[0-9])+\.[0-9_]*|[-+]?\.(?:inf|Inf|INF)|\.(?:nan|NaN|NAN))$)");
  if (std::regex_match(s, re_null)) return nullptr;
  if (std::regex_match(s, re_true)) return true;
  if (std::regex_match(s, re_false)) return false;
  auto strip_us = [](std::string x) {
    x.erase(std::remove(x.begin(), x.end(), '_'), x.end());
    return x;
  };
  if (std::regex_match(s, re_int)) {
    std::string v = strip_us(s);
    int sign = 1;
    if (!v.empty() && (v[0] == '-' || v[0] == '+')) {
      if (v[0] == '-') sign = -1;
      v = v.substr(1);
    }
    long long out = 0;
    if (v == "0") out = 0;
    else if (v.rfind("0b", 0) == 0) out = std::stoll(v.substr(2), nullptr, 2);
    else if (v.rfind("0x", 0) == 0) out = std::stoll(v.substr(2), nullptr, 16);
    else if (v[0] == '0') out = std::stoll(v, nullptr, 8);
    else if (v.find(':') != std::string::npos) {
      std::stringstream ss(v);
      std::string part;
      while (std::getline(ss, part, ':')) out = out * 60 + std::stoll(part);
    } else out = std::stoll(v);
    return sign * out;
  }
  if (std::regex_match(s, re_float)) {
    std::string v = strip_us(s);
    std::string low = v;
    for (auto& c : low) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (low.find(".inf") != std::string::npos) return low[0] == '-' ? -HUGE_VAL : HUGE_VAL;
    if (low.find(".nan") != std::string::npos) return std::nan("");
    if (v.find(':') != std::string::npos) {
      int sign = 1;
      if (v[0] == '-' || v[0] == '+') {
        if (v[0] == '-') sign = -1;
        v = v.substr(1);
      }
      double out = 0;
      std::stringstream ss(v);
      std::string part;
      while (std::getline(ss, part, ':')) out = out * 60 + std::stod(part);
      return sign * out;
    }
    return std::stod(v);
  }
  return s;
}

}  // namespace

ojson yaml_to_json(const YAML::Node& n) {
  switch (n.Type()) {
    case YAML::NodeType::Null:
    case YAML::NodeType::Undefined:
      return nullptr;
    case YAML::NodeType::Sequence: {
      ojson a = ojson::array();
      for (const auto& e : n) a.push_back(yaml_to_json(e));
      return a;
    }
    case YAML::NodeType::Map: {
      ojson o = ojson::object();
      for (const auto& kv : n) {
        ojson k = yaml_to_json(kv.first);
        o[k.is_string() ? k.get<std::string>() : l4::dumps(k)] = yaml_to_json(kv.second);
      }
      return o;
    }
    case YAML::NodeType::Scalar: {
      const std::string& tag = n.Tag();
      const std::string& s = n.Scalar();
      if (tag == "!" || tag == "tag:yaml.org,2002:str") return s;   // quoted, or !!str
      if (tag == "tag:yaml.org,2002:float") return std::stod(s);
      if (tag == "tag:yaml.org,2002:int") return std::stoll(s);
      return resolve_plain(s);
    }
  }
  return nullptr;
}

Mission load_mission_text(const std::string& text, const std::string& path) {
  YAML::Node root;
  try {
    root = YAML::Load(text);
  } catch (const YAML::Exception& e) {
    throw std::invalid_argument(std::string("mission YAML: ") + e.what());
  }
  ojson doc = yaml_to_json(root);
  if (!doc.is_object()) throw std::invalid_argument("mission: not a mapping");
  if (l4::get(doc, "frame", "fc") != ojson("fc")) throw std::invalid_argument("mission waypoints must be in the fc frame (D7)");
  ojson d = {{"speed", 0.5}, {"accept_radius", 0.5}, {"leg_timeout_s", 90.0}, {"no_progress_s", 20.0}, {"hold_s", 0.0}};
  const ojson defaults = l4::get(doc, "defaults", ojson::object());
  if (defaults.is_object())
    for (auto it = defaults.begin(); it != defaults.end(); ++it) d[it.key()] = it.value();
  if (!doc.contains("legs") || !doc["legs"].is_array()) throw std::invalid_argument("mission: no legs");
  Mission m;
  int i = 0;
  for (const auto& leg : doc["legs"]) {
    if (!leg.is_object()) throw std::invalid_argument("leg " + std::to_string(i) + ": not a mapping");
    const ojson kind = l4::get(leg, "type");
    if (!(kind == "fly" || kind == "drive" || kind == "land" || kind == "takeoff" || kind == "scan"))
      throw std::invalid_argument("leg " + std::to_string(i) + ": unknown type " + l4::dumps(kind));
    ojson full = d;
    for (auto it = leg.begin(); it != leg.end(); ++it) full[it.key()] = it.value();
    auto len_of = [&](const char* k) -> size_t {
      const ojson v = l4::get(full, k, ojson::array());
      if (!v.is_array()) throw std::invalid_argument("leg " + std::to_string(i) + ": " + k + " must be a list");
      for (const auto& x : v) num(x);
      return v.size();
    };
    if (kind == "fly" && len_of("goal") != 3) throw std::invalid_argument("leg " + std::to_string(i) + ": fly needs goal [x, y, z]");
    if (kind == "drive" && len_of("goal") < 2) throw std::invalid_argument("leg " + std::to_string(i) + ": drive needs goal [x, y]");
    if (kind == "land" && len_of("at") < 2) throw std::invalid_argument("leg " + std::to_string(i) + ": land needs at [x, y]");
    if (kind == "takeoff" && !full.contains("height")) throw std::invalid_argument("leg " + std::to_string(i) + ": takeoff needs height");
    if (kind == "scan" && !full.contains("yaw_sweep_deg")) throw std::invalid_argument("leg " + std::to_string(i) + ": scan needs yaw_sweep_deg");
    if (kind == "scan") num(full["yaw_sweep_deg"]);
    if (full.contains("yaw")) num(full["yaw"]);   // a deliberate waypoint yaw, deg, fc frame (owner 2026-10-29)
    if (full.contains("scan_rate_deg")) num(full["scan_rate_deg"]);
    const ojson pol = l4::get(full, "on_unreachable", "next");
    if (!pol.is_string() || !unreachable_policy_ok(pol.get<std::string>()))
      throw std::invalid_argument("leg " + std::to_string(i) + ": on_unreachable must be one of next, stop");
    for (const char* k : {"accept_radius", "leg_timeout_s", "no_progress_s", "hold_s"}) num(full[k]);
    if (kind == "takeoff") num(full["height"]);
    m.legs.push_back(full);
    ++i;
  }
  m.name = l4::get(doc, "name", path);
  m.takeoff_height = num(l4::get(doc, "takeoff_height", 1.0));
  return m;
}

Mission load_mission(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::invalid_argument("cannot read the mission " + path);
  std::stringstream ss;
  ss << f.rdbuf();
  return load_mission_text(ss.str(), path);
}

}  // namespace l4_executive

namespace l4_executive {

double wrap_pi(double a) { return std::atan2(std::sin(a), std::cos(a)); }

void SettleWindow::add(double t, const Eigen::Vector3d& p, double yaw) {
  s_.push_back({t, p, yaw});
  // keep a little more than the window: the oldest kept sample is at least `window` old
  while (s_.size() > 2 && t - s_[1].t >= window_) s_.erase(s_.begin());
}

bool SettleWindow::settled(double speed_max, double rate_max, std::optional<double> target, double tol) const {
  if (s_.size() < 2) return false;
  const S& now = s_.back();
  if (now.t - s_.front().t < window_) return false;
  for (const S& s : s_) {
    if ((s.p.head<2>() - now.p.head<2>()).norm() > speed_max * window_) return false;
    if (std::abs(wrap_pi(s.yaw - now.yaw)) > rate_max * window_) return false;
  }
  if (target && std::abs(wrap_pi(now.yaw - *target)) > tol) return false;
  return true;
}

}  // namespace l4_executive
