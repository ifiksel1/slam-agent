// Python-compatible JSON and number formatting for the L4 nodes' status topics.
//
// The former Python nodes published json.dumps(...) strings; their consumers (the executive, the SITL
// tools, the tests) parse them, and some tests match substrings such as '"state": "STREAM"'. dumps()
// here writes exactly what Python's json.dumps writes for the same values: ", " and ": " separators,
// keys in insertion order (ordered_json), ensure_ascii escaping, floats as Python's repr (the shortest
// round-trip digits; "1.0", "0.123", "1e-05", "1e+16"), NaN / Infinity.
// py_round() is Python's round(x, n) on a float (correctly rounded, ties to even on the exact value).
#pragma once

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <string>

#include <nlohmann/json.hpp>

namespace l4 {

using ojson = nlohmann::ordered_json;

// Python's repr(float)
inline std::string py_float_repr(double x) {
  if (std::isnan(x)) return "NaN";
  if (std::isinf(x)) return x > 0 ? "Infinity" : "-Infinity";
  if (x == 0.0) return std::signbit(x) ? "-0.0" : "0.0";
  char buf[64];
  int prec = 1;
  for (; prec <= 17; ++prec) {
    std::snprintf(buf, sizeof buf, "%.*e", prec - 1, x);
    if (std::strtod(buf, nullptr) == x) break;
  }
  // buf: [-]d[.ddd]e(+|-)XX
  std::string s(buf), digits, out;
  size_t i = 0;
  if (s[0] == '-') { out = "-"; i = 1; }
  size_t epos = s.find('e');
  for (size_t k = i; k < epos; ++k)
    if (s[k] != '.') digits += s[k];
  while (digits.size() > 1 && digits.back() == '0') digits.pop_back();
  int e = std::atoi(s.c_str() + epos + 1);
  int decpt = e + 1;
  int n = static_cast<int>(digits.size());
  if (decpt > 16 || decpt < -3) {
    out += digits[0];
    if (n > 1) out += "." + digits.substr(1);
    char eb[16];
    std::snprintf(eb, sizeof eb, "e%c%02d", e < 0 ? '-' : '+', e < 0 ? -e : e);
    return out + eb;
  }
  if (decpt <= 0) return out + "0." + std::string(-decpt, '0') + digits;
  if (decpt >= n) return out + digits + std::string(decpt - n, '0') + ".0";
  return out + digits.substr(0, decpt) + "." + digits.substr(decpt);
}

// Python's round(x, nd) for a float
inline double py_round(double x, int nd) {
  if (!std::isfinite(x)) return x;
  char buf[512];
  std::snprintf(buf, sizeof buf, "%.*f", nd, x);
  return std::strtod(buf, nullptr);
}

namespace detail {
inline void dump_string(const std::string& s, std::string& out) {
  out += '"';
  for (size_t i = 0; i < s.size(); ++i) {
    unsigned char c = static_cast<unsigned char>(s[i]);
    char b[16];
    switch (c) {
      case '"': out += "\\\""; continue;
      case '\\': out += "\\\\"; continue;
      case '\n': out += "\\n"; continue;
      case '\r': out += "\\r"; continue;
      case '\t': out += "\\t"; continue;
      case '\b': out += "\\b"; continue;
      case '\f': out += "\\f"; continue;
      default: break;
    }
    if (c < 0x20 || c == 0x7f) {
      std::snprintf(b, sizeof b, "\\u%04x", c);
      out += b;
      continue;
    }
    if (c < 0x80) { out += static_cast<char>(c); continue; }
    // UTF-8 -> \uXXXX (ensure_ascii), surrogate pairs above the BMP
    unsigned cp = 0;
    int extra = 0;
    if ((c & 0xE0) == 0xC0) { cp = c & 0x1F; extra = 1; }
    else if ((c & 0xF0) == 0xE0) { cp = c & 0x0F; extra = 2; }
    else if ((c & 0xF8) == 0xF0) { cp = c & 0x07; extra = 3; }
    else { cp = 0xFFFD; extra = 0; }
    for (int k = 0; k < extra && i + 1 < s.size(); ++k) cp = (cp << 6) | (static_cast<unsigned char>(s[++i]) & 0x3F);
    if (cp >= 0x10000) {
      cp -= 0x10000;
      std::snprintf(b, sizeof b, "\\u%04x\\u%04x", 0xD800 + (cp >> 10), 0xDC00 + (cp & 0x3FF));
    } else {
      std::snprintf(b, sizeof b, "\\u%04x", cp);
    }
    out += b;
  }
  out += '"';
}

template <class J>
inline void dump(const J& j, std::string& out) {
  switch (j.type()) {
    case nlohmann::json::value_t::null: out += "null"; break;
    case nlohmann::json::value_t::boolean: out += j.template get<bool>() ? "true" : "false"; break;
    case nlohmann::json::value_t::number_integer: out += std::to_string(j.template get<long long>()); break;
    case nlohmann::json::value_t::number_unsigned: out += std::to_string(j.template get<unsigned long long>()); break;
    case nlohmann::json::value_t::number_float: out += py_float_repr(j.template get<double>()); break;
    case nlohmann::json::value_t::string: dump_string(j.template get_ref<const std::string&>(), out); break;
    case nlohmann::json::value_t::array: {
      out += '[';
      bool first = true;
      for (const auto& v : j) {
        if (!first) out += ", ";
        first = false;
        dump(v, out);
      }
      out += ']';
      break;
    }
    case nlohmann::json::value_t::object: {
      out += '{';
      bool first = true;
      for (auto it = j.begin(); it != j.end(); ++it) {
        if (!first) out += ", ";
        first = false;
        dump_string(it.key(), out);
        out += ": ";
        dump(it.value(), out);
      }
      out += '}';
      break;
    }
    default: out += "null"; break;
  }
}
}  // namespace detail

// Python's json.dumps(value)
template <class J>
inline std::string dumps(const J& j) {
  std::string out;
  detail::dump(j, out);
  return out;
}

// json.loads: an object, or a discarded value on a parse error (the caller ignores that message, as the
// Python nodes did on ValueError). NaN / Infinity are accepted as Python's json does.
inline ojson loads(const std::string& s) {
  ojson j = ojson::parse(s, nullptr, false);
  if (!j.is_discarded()) return j;
  // Python's json also reads NaN / Infinity / -Infinity: retry with them as null-free numbers
  std::string t;
  t.reserve(s.size());
  bool in_str = false;
  for (size_t i = 0; i < s.size(); ++i) {
    char c = s[i];
    if (in_str) {
      t += c;
      if (c == '\\' && i + 1 < s.size()) t += s[++i];
      else if (c == '"') in_str = false;
      continue;
    }
    if (c == '"') { in_str = true; t += c; continue; }
    if (s.compare(i, 3, "NaN") == 0) { t += "\"__NaN__\""; i += 2; continue; }
    if (s.compare(i, 9, "-Infinity") == 0) { t += "\"__-Inf__\""; i += 8; continue; }
    if (s.compare(i, 8, "Infinity") == 0) { t += "\"__Inf__\""; i += 7; continue; }
    t += c;
  }
  ojson k = ojson::parse(t, nullptr, false);
  if (k.is_discarded()) return k;
  std::function<void(ojson&)> fix = [&](ojson& v) {
    if (v.is_string()) {
      const auto& x = v.get_ref<const std::string&>();
      if (x == "__NaN__") v = std::nan("");
      else if (x == "__Inf__") v = HUGE_VAL;
      else if (x == "__-Inf__") v = -HUGE_VAL;
    } else if (v.is_structured()) {
      for (auto& e : v) fix(e);
    }
  };
  fix(k);
  return k;
}

// Python truthiness of a JSON value (bool(x))
inline bool truthy(const ojson& v) {
  switch (v.type()) {
    case nlohmann::json::value_t::null: return false;
    case nlohmann::json::value_t::boolean: return v.get<bool>();
    case nlohmann::json::value_t::number_integer: return v.get<long long>() != 0;
    case nlohmann::json::value_t::number_unsigned: return v.get<unsigned long long>() != 0;
    case nlohmann::json::value_t::number_float: return v.get<double>() != 0.0;
    case nlohmann::json::value_t::string: return !v.get_ref<const std::string&>().empty();
    case nlohmann::json::value_t::array:
    case nlohmann::json::value_t::object: return !v.empty();
    default: return false;
  }
}

// dict.get(key, default) on a JSON object (a non-object gives the default)
inline ojson get(const ojson& o, const std::string& key, const ojson& def = nullptr) {
  if (!o.is_object()) return def;
  auto it = o.find(key);
  return it == o.end() ? def : *it;
}

}  // namespace l4
