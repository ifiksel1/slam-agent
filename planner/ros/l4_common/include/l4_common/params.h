// Parameter reads with Python's coercions (rospy.get_param + float()/int()/bool()): an int given for a
// float parameter, a float for an int one, 0/1 for a bool, int lists for float lists.
//
// roscpp keeps a "_name:=value" argument it cannot read as an int, a double or a bool as a string, where
// rospy YAML-parses it: "_geofence_min:=[-20,-20,-1]", "_x:=1e-3" or "_flag:=True" arrive as text here
// and are read as rospy would.
#pragma once

#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

#include <ros/ros.h>

namespace l4 {

inline bool text_number(const std::string& s0, double& out) {
  std::string s;
  for (char c : s0)
    if (c != ' ' && c != '\t') s += c;
  if (s == "true" || s == "True" || s == "TRUE" || s == "yes" || s == "Yes" || s == "on") {
    out = 1.0;
    return true;
  }
  if (s == "false" || s == "False" || s == "FALSE" || s == "no" || s == "No" || s == "off") {
    out = 0.0;
    return true;
  }
  if (s.empty()) return false;
  char* end = nullptr;
  out = std::strtod(s.c_str(), &end);
  return end && *end == '\0';
}

inline bool xml_number(const XmlRpc::XmlRpcValue& v0, double& out) {
  XmlRpc::XmlRpcValue& v = const_cast<XmlRpc::XmlRpcValue&>(v0);
  switch (v.getType()) {
    case XmlRpc::XmlRpcValue::TypeDouble: out = static_cast<double>(v); return true;
    case XmlRpc::XmlRpcValue::TypeInt: out = static_cast<int>(v); return true;
    case XmlRpc::XmlRpcValue::TypeBoolean: out = static_cast<bool>(v) ? 1.0 : 0.0; return true;
    case XmlRpc::XmlRpcValue::TypeString: return text_number(static_cast<std::string&>(v), out);
    default: return false;
  }
}

inline double param_double(const ros::NodeHandle& nh, const std::string& key, double def) {
  XmlRpc::XmlRpcValue v;
  double out;
  if (nh.getParam(key, v) && xml_number(v, out)) return out;
  return def;
}

inline int param_int(const ros::NodeHandle& nh, const std::string& key, int def) {
  XmlRpc::XmlRpcValue v;
  double out;
  if (nh.getParam(key, v) && xml_number(v, out)) return static_cast<int>(out);   // int(): truncation
  return def;
}

inline bool param_bool(const ros::NodeHandle& nh, const std::string& key, bool def) {
  XmlRpc::XmlRpcValue v;
  double out;
  if (!nh.getParam(key, v)) return def;
  if (xml_number(v, out)) return out != 0.0;
  if (v.getType() == XmlRpc::XmlRpcValue::TypeString) return !static_cast<std::string&>(v).empty();
  return def;
}

inline std::string param_string(const ros::NodeHandle& nh, const std::string& key, const std::string& def) {
  XmlRpc::XmlRpcValue v;
  if (!nh.getParam(key, v)) return def;
  if (v.getType() == XmlRpc::XmlRpcValue::TypeString) return static_cast<std::string&>(v);
  return def;
}

inline std::vector<double> param_vec(const ros::NodeHandle& nh, const std::string& key, const std::vector<double>& def) {
  XmlRpc::XmlRpcValue v;
  if (!nh.getParam(key, v)) return def;
  std::vector<double> out;
  if (v.getType() == XmlRpc::XmlRpcValue::TypeString) {
    const std::string s = static_cast<std::string&>(v);
    const size_t a = s.find('['), b = s.rfind(']');
    if (a == std::string::npos || b == std::string::npos || b < a) return def;
    std::stringstream ss(s.substr(a + 1, b - a - 1));
    std::string item;
    while (std::getline(ss, item, ',')) {
      double x;
      if (!text_number(item, x)) return def;
      out.push_back(x);
    }
    return out;
  }
  if (v.getType() != XmlRpc::XmlRpcValue::TypeArray) return def;
  for (int i = 0; i < v.size(); ++i) {
    double x;
    if (!xml_number(v[i], x)) return def;
    out.push_back(x);
  }
  return out;
}

}  // namespace l4
