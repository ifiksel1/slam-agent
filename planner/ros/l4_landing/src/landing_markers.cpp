// The landing check's markers and helpers (see landing_markers.h and the node's header comment).
#include "l4_landing/landing_markers.h"

#include <geometry_msgs/Point.h>

#include <cmath>
#include <cstdarg>
#include <cstdio>

namespace l4_landing {

namespace {
std::string fmt(const char* f, ...) __attribute__((format(printf, 1, 2)));
std::string fmt(const char* f, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, f);
  vsnprintf(buf, sizeof buf, f, ap);
  va_end(ap);
  return buf;
}
bool starts(const std::string& s, const char* p) { return s.rfind(p, 0) == 0; }
geometry_msgs::Point pt(double x, double y, double z) {
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}
std_msgs::ColorRGBA col(double r, double g, double b, double a) {
  std_msgs::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}
double num(const Json& st, const char* k, double def) { return st.contains(k) && st[k].is_number() ? st[k].get<double>() : def; }
}  // namespace

std::vector<KnownPose> ground_spots_to_planner(const std::vector<GroundSpot>& spots, double psi, double tx, double ty) {
  double c = std::cos(psi), s = std::sin(psi);
  std::vector<KnownPose> out;
  for (auto& sp : spots) {
    double dx = sp.x - tx, dy = sp.y - ty, yaw = sp.yaw - psi;
    out.push_back({c * dx + s * dy, -s * dx + c * dy, std::atan2(std::sin(yaw), std::cos(yaw))});
  }
  return out;
}

std::pair<std::string, RGB> decision(bool clear, const std::string& reason) {
  if (clear) return {"CLEAR", GREEN};
  if (starts(reason, "occupied")) return {"OCCUPIED", RED};
  if (starts(reason, "not flat")) return {"NOT FLAT", RED};
  if (starts(reason, "not observed")) return {"NOT OBSERVED", AMBER};
  if (starts(reason, "no heading")) return {"NO HEADING", AMBER};
  return {"UNCERTAIN", AMBER};
}

std::string label_text(bool clear, const Json& st) {
  std::string reason = st.value("reason", "");
  std::string word = decision(clear, reason).first;
  bool has_layer = st.contains("floor_layer_m") && st["floor_layer_m"].is_number();
  double layer = num(st, "floor_layer_m", 0.0);
  if (word == "CLEAR") return fmt("CLEAR layer %.2f m", layer);
  if (word == "OCCUPIED")
    return fmt("OCCUPIED %d cells up to %.2f m", int(num(st, "obstacle_cells", 0)), num(st, "max_obstacle_h", 0.0));
  if (word == "NOT OBSERVED" && st.contains("observed")) return fmt("NOT OBSERVED %.0f%%", 100 * num(st, "observed", 0));
  if (word == "NOT FLAT")
    return fmt("NOT FLAT slope %.1f deg, resid %.3f m", num(st, "slope_deg", 0.0), num(st, "residual_m", 0.0));
  if (word == "UNCERTAIN" && has_layer) return fmt("UNCERTAIN layer %.2f m", layer);
  return word + " " + reason;
}

visualization_msgs::MarkerArray build_markers(bool clear, const Json& st, const Viz& viz, const std::string& frame,
                                              const ros::Time& stamp, double fallback_z) {
  using visualization_msgs::Marker;
  RGB rgb = decision(clear, st.value("reason", "")).second;
  double cx, cy;
  if (viz.valid) {
    cx = viz.centre.x();
    cy = viz.centre.y();
  } else {
    cx = st["centre"][0].get<double>();
    cy = st["centre"][1].get<double>();
  }
  double fz = viz.floor_z ? *viz.floor_z : fallback_z;
  double a = viz.plane ? viz.plane->first : 0.0, b = viz.plane ? viz.plane->second : 0.0;
  double cell = viz.valid ? viz.cell : 0.1;
  auto mk = [&](const std::string& ns, int typ) {
    Marker m;
    m.header.frame_id = frame;
    m.header.stamp = stamp;
    m.ns = ns;
    m.id = 0;
    m.type = typ;
    m.action = Marker::ADD;
    m.pose.orientation.w = 1.0;
    return m;
  };
  visualization_msgs::MarkerArray out;
  Marker d = mk("landing_disc", Marker::CYLINDER);
  double r = viz.valid ? viz.radius : st["radius_m"].get<double>();
  d.pose.position.x = cx;
  d.pose.position.y = cy;
  d.pose.position.z = fz + 0.005;
  d.scale.x = d.scale.y = 2 * r;
  d.scale.z = 0.01;
  d.color = col(rgb[0], rgb[1], rgb[2], 0.35);
  out.markers.push_back(d);

  Marker f = mk("landing_footprint", Marker::LINE_STRIP);
  f.scale.x = 0.02;
  f.color = col(1.0, 1.0, 1.0, 0.9);
  if (viz.valid && viz.yaw) {
    double c = std::cos(*viz.yaw), s = std::sin(*viz.yaw), hl = FOOTPRINT_LEN / 2, hw = FOOTPRINT_WID / 2;
    const double uv[5][2] = {{hl, hw}, {-hl, hw}, {-hl, -hw}, {hl, -hw}, {hl, hw}};
    for (auto& q : uv) f.points.push_back(pt(cx + c * q[0] - s * q[1], cy + s * q[0] + c * q[1], fz + 0.02));
    f.points.push_back(pt(cx, cy, fz + 0.02));   // a line to the front edge's middle: the heading
    f.points.push_back(pt(cx + c * hl, cy + s * hl, fz + 0.02));
  } else {
    f.action = Marker::DELETE;
  }
  out.markers.push_back(f);

  Marker o = mk("landing_occupied", Marker::CUBE_LIST);
  o.scale.x = o.scale.y = cell;
  o.scale.z = 0.05;
  o.color = col(RED[0], RED[1], RED[2], 0.9);
  for (auto& q : viz.occupied) o.points.push_back(pt(q[0], q[1], fz + a * (q[0] - cx) + b * (q[1] - cy) + q[2] - 0.025));
  if (o.points.empty()) o.action = Marker::DELETE;
  out.markers.push_back(o);

  Marker k = mk("landing_known_floor", Marker::LINE_LIST);
  k.scale.x = 0.025;
  k.color = col(LIGHT_BLUE[0], LIGHT_BLUE[1], LIGHT_BLUE[2], 0.9);
  for (auto& corners : viz.known_floor)
    for (int q = 0; q < 4; ++q) {
      const V2 &p0 = corners[q], &p1 = corners[(q + 1) % 4];
      k.points.push_back(pt(p0.x(), p0.y(), fz + a * (p0.x() - cx) + b * (p0.y() - cy) + 0.015));
      k.points.push_back(pt(p1.x(), p1.y(), fz + a * (p1.x() - cx) + b * (p1.y() - cy) + 0.015));
    }
  if (k.points.empty()) k.action = Marker::DELETE;
  out.markers.push_back(k);

  Marker u = mk("landing_unobserved", Marker::CUBE_LIST);
  u.scale.x = u.scale.y = cell * 0.9;
  u.scale.z = 0.01;
  u.color = col(0.5, 0.5, 0.5, 0.6);
  for (auto& q : viz.unobserved) u.points.push_back(pt(q.x(), q.y(), fz + a * (q.x() - cx) + b * (q.y() - cy) + 0.01));
  if (u.points.empty()) u.action = Marker::DELETE;
  out.markers.push_back(u);

  Marker t = mk("landing_label", Marker::TEXT_VIEW_FACING);
  t.pose.position.x = cx;
  t.pose.position.y = cy;
  t.pose.position.z = fz + 0.6;
  t.scale.z = 0.15;
  t.color = col(rgb[0], rgb[1], rgb[2], 1.0);
  t.text = label_text(clear, st);
  out.markers.push_back(t);
  return out;
}

visualization_msgs::MarkerArray build_search_markers(const SearchResult& r, const LandingCheck& check, const V2& vehicle, double radius,
                                                     const std::string& frame, const ros::Time& stamp, double fallback_z) {
  using visualization_msgs::Marker;
  auto mk = [&](const std::string& ns, int typ) {
    Marker m;
    m.header.frame_id = frame;
    m.header.stamp = stamp;
    m.ns = ns;
    m.id = 0;
    m.type = typ;
    m.action = Marker::ADD;
    m.pose.orientation.w = 1.0;
    return m;
  };
  visualization_msgs::MarkerArray out;
  Marker clear_all = mk("", Marker::CUBE);
  clear_all.action = Marker::DELETEALL;
  out.markers.push_back(clear_all);
  // the floor: the median of the evaluated candidates' floors (no floor known: fallback_z)
  std::vector<double> fl;
  for (const auto& c : r.cands)
    if (c.floor_z) fl.push_back(*c.floor_z);
  const double fz = fl.empty() ? fallback_z : l4_track::median(fl);

  // the candidates: green clear, red occupied / not flat, amber not observed / uncertain, grey excluded or
  // not evaluated
  Marker cm = mk("land_search_candidates", Marker::CUBE_LIST);
  cm.scale.x = cm.scale.y = 0.08;
  cm.scale.z = 0.02;
  for (const auto& c : r.cands) {
    RGB rgb{0.55, 0.55, 0.55};
    double alpha = 0.5;
    if (c.evaluated) {
      rgb = decision(c.clear, c.reason).second;
      alpha = 0.9;
    }
    cm.points.push_back(pt(c.spot.x(), c.spot.y(), (c.floor_z ? *c.floor_z : fz) + 0.02));
    cm.colors.push_back(col(rgb[0], rgb[1], rgb[2], alpha));
  }
  if (cm.points.empty()) cm.action = Marker::DELETE;
  out.markers.push_back(cm);

  Marker ring = mk("land_search_radius", Marker::LINE_STRIP);
  ring.scale.x = 0.02;
  ring.color = col(1.0, 1.0, 1.0, 0.6);
  for (int k = 0; k <= 72; ++k) {
    const double a = 2 * M_PI * k / 72;
    ring.points.push_back(pt(vehicle.x() + radius * std::cos(a), vehicle.y() + radius * std::sin(a), fz + 0.01));
  }
  out.markers.push_back(ring);

  Marker d = mk("land_search_choice", Marker::CYLINDER);
  Marker f = mk("land_search_choice_footprint", Marker::LINE_STRIP);
  Marker t = mk("land_search_label", Marker::TEXT_VIEW_FACING);
  Marker los = mk("land_search_los", Marker::LINE_STRIP);
  t.scale.z = 0.15;
  if (r.best) {
    const Candidate& b = r.cands[*r.best];
    if (b.los_open && r.los_z) {
      los.scale.x = 0.03;
      los.color = *b.los_open ? col(GREEN[0], GREEN[1], GREEN[2], 0.9) : col(AMBER[0], AMBER[1], AMBER[2], 0.9);
      los.points.push_back(pt(vehicle.x(), vehicle.y(), *r.los_z));
      los.points.push_back(pt(b.spot.x(), b.spot.y(), *r.los_z));
    } else {
      los.action = Marker::DELETE;
    }
    const double z = b.floor_z ? *b.floor_z : fz;
    const V2 ctr = check.centre(b.spot, b.yaw);
    d.pose.position.x = ctr.x();
    d.pose.position.y = ctr.y();
    d.pose.position.z = z + 0.006;
    d.scale.x = d.scale.y = 2 * check.radius;
    d.scale.z = 0.012;
    d.color = col(GREEN[0], GREEN[1], GREEN[2], 0.5);
    f.scale.x = 0.025;
    f.color = col(1.0, 1.0, 1.0, 0.95);
    const auto corners = check.footprint_corners(b.spot, b.yaw);
    for (int q = 0; q <= 4; ++q) f.points.push_back(pt(corners[q % 4].x(), corners[q % 4].y(), z + 0.03));
    f.points.push_back(pt(ctr.x(), ctr.y(), z + 0.03));   // the heading
    f.points.push_back(pt(ctr.x() + std::cos(b.yaw) * FOOTPRINT_LEN / 2, ctr.y() + std::sin(b.yaw) * FOOTPRINT_LEN / 2, z + 0.03));
    t.pose.position.x = b.spot.x();
    t.pose.position.y = b.spot.y();
    t.pose.position.z = z + 0.7;
    t.color = col(GREEN[0], GREEN[1], GREEN[2], 1.0);
    t.text = fmt("LANDING SPOT %.2f m away, margin %.2f m%s (%d/%zu checked)", b.dist, b.margin,
                 b.los_open ? (*b.los_open ? ", line open" : ", line blocked: detour") : "", r.evaluated, r.cands.size());
  } else {
    los.action = Marker::DELETE;
    d.action = Marker::DELETE;
    f.action = Marker::DELETE;
    t.pose.position.x = vehicle.x();
    t.pose.position.y = vehicle.y();
    t.pose.position.z = fz + 0.7;
    t.color = col(AMBER[0], AMBER[1], AMBER[2], 1.0);
    t.text = fmt("NO LANDING SPOT within %.1f m (%d/%zu checked)", radius, r.evaluated, r.cands.size());
  }
  out.markers.push_back(d);
  out.markers.push_back(f);
  out.markers.push_back(t);
  out.markers.push_back(los);
  return out;
}

}  // namespace l4_landing
