// Landing-spot check tests (gtest): the cases of the former planner/test/test_landing_check.py, one
// TEST per Python test: synthetic patches and scan-by-scan windows, the markers (no master needed), and
// real data when present: merged maps (LANDING_MAPS=dir with the map PCDs) and recorded keyframe scans
// (LANDING_SIM=Shannon's ~/sim: track/maps_tfix, flights/<run>/{out,view}, tfix/maps/<bag>/tfix/out).
//
//   LANDING_MAPS=~/sim/marsim/maps LANDING_SIM=~/sim devel/lib/l4_landing/test_landing_core   (on Shannon)
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <map>
#include <set>
#include <tuple>
#include <random>
#include <sstream>

#include "l4_landing/landing_core.h"
#include "l4_landing/landing_markers.h"

using namespace l4_landing;
namespace tc = l4_track;
using tc::Cloud;
using visualization_msgs::Marker;

namespace {
std::mt19937 rng(1);

Cloud floor_(double cx = 0.0, double cy = 0.0, double r = 1.0, double z = 0.0, double slope_deg = 0.0,
             double noise = 0.01, double step = 0.04) {
  std::normal_distribution<double> N(0, noise);
  auto g = tc::arange(-r, r + 1e-9, step);
  Cloud out;
  for (double y : g)
    for (double x : g) out.push_back({x + cx, y + cy, z + std::tan(tc::radians(slope_deg)) * x + N(rng)});
  return out;
}

Cloud box(double cx, double cy, double h, double size = 0.3, double z0 = 0.0, double step = 0.04) {
  auto g = tc::arange(-size / 2, size / 2 + 1e-9, step);
  auto zz = tc::arange(0, h + 1e-9, step);
  Cloud out;
  for (double x : g)
    for (double y : g)
      for (double z : zz) out.push_back({x + cx, y + cy, z + z0});
  return out;
}

Cloud cat(std::initializer_list<Cloud> cs) {
  Cloud out;
  for (auto& c : cs) out.insert(out.end(), c.begin(), c.end());
  return out;
}
Cloud shift(Cloud c, double dz) {
  for (auto& p : c) p.z += dz;
  return c;
}
template <class F>
Cloud filter(const Cloud& c, F f) {
  Cloud o;
  for (auto& p : c)
    if (f(p)) o.push_back(p);
  return o;
}
std::string reason(const Json& st) { return st.value("reason", ""); }
bool has(const Json& st, const char* s) { return reason(st).find(s) != std::string::npos; }
const V2 O(0, 0);

// P without the floor the robot's body hid while it sat with its odometry point at odom, heading yaw:
// the frame's footprint (centred 0.193 m behind the odometry point) and a shadow behind its rear edge
Cloud hide_where_it_sat(const Cloud& P, V2 odom = V2(0, 0), double yaw = 0.0, double shadow = 0.2) {
  double c = std::cos(yaw), s = std::sin(yaw);
  return filter(P, [&](const tc::P3& p) {
    double dx = p.x - (odom.x() - 0.193 * c), dy = p.y - (odom.y() - 0.193 * s);
    double u = dx * c + dy * s, v = -dx * s + dy * c;
    return !(u >= -0.254 - shadow && u <= 0.254 && std::abs(v) <= 0.2285);
  });
}

// n scans of one flat floor, each at its own height (z drift 0..drift), each 1 cm thick; extra(k) ->
// points added to scan k (in that scan's own frame)
std::vector<Scan> drifting_scans(std::function<Cloud(int)> extra = nullptr, int n = 20, double drift = 0.25, double r = 2.0) {
  std::vector<Scan> out;
  for (int k = 0; k < n; ++k) {
    double dz = drift * k / std::max(1, n - 1);
    Cloud S = shift(floor_(0, 0, r, 0, 0, 0.01, 0.05), dz);
    if (extra) {
      Cloud e = extra(k);
      if (!e.empty()) {
        double mz = 1e9, mx = 0, my = 0;
        for (auto& p : e) {
          mz = std::min(mz, p.z);
          mx += p.x;
          my += p.y;
        }
        mx /= e.size();
        my /= e.size();
        if (mz < 0.05)   // stands on the floor: hides it
          S = filter(S, [&](const tc::P3& p) { return !(std::abs(p.x - mx) < 0.16 && std::abs(p.y - my) < 0.16); });
        S = cat({S, shift(e, dz)});
      }
    }
    out.push_back({S, std::nullopt});
  }
  return out;
}
std::vector<Scan> scans_of(const std::vector<Cloud>& cs) {
  std::vector<Scan> o;
  for (auto& c : cs) o.push_back({c, std::nullopt});
  return o;
}
}  // namespace

// ---- Synthetic
TEST(Synthetic, FlatFloorIsClear) {
  LandingCheck lc;
  auto [ok, st] = lc.evaluate(floor_(), O);
  EXPECT_TRUE(ok) << st.dump();
}

TEST(Synthetic, BoxIsOccupied) {
  LandingCheck lc;
  auto [ok, st] = lc.evaluate(cat({floor_(), box(0.1, 0.0, 0.3)}), O);
  EXPECT_FALSE(ok);
  EXPECT_TRUE(has(st, "occupied")) << st.dump();
}

namespace {
Cloud beam(double z) {
  Cloud b;
  for (double x : tc::linspace(-0.6, 0.6, 60)) b.push_back({x, 0.0, z});
  return b;
}
}  // namespace

TEST(Synthetic, OverheadBeamIsOccupied) {
  LandingCheck lc;
  auto [ok, st] = lc.evaluate(cat({floor_(), beam(1.5)}), O, 2.5);
  EXPECT_FALSE(ok);
  EXPECT_TRUE(has(st, "occupied")) << st.dump();
}

TEST(Synthetic, BeamAboveTheDescentHeightIsIgnored) {
  LandingCheck lc;
  auto [ok, st] = lc.evaluate(cat({floor_(), beam(3.0)}), O, 2.0);
  EXPECT_TRUE(ok) << st.dump();
}

TEST(Synthetic, SlopeIsNotFlat) {
  LandingCheck lc;
  auto [ok, st] = lc.evaluate(floor_(0, 0, 1.0, 0, 10), O);
  EXPECT_FALSE(ok);
  EXPECT_TRUE(has(st, "not flat")) << st.dump();
}

TEST(Synthetic, NothingSeenIsNotObserved) {
  LandingCheck lc;
  auto [ok, st] = lc.evaluate(floor_(5.0), O);
  EXPECT_FALSE(ok);
  EXPECT_TRUE(has(st, "not observed")) << st.dump();
}

TEST(Synthetic, HalfSeenIsNotObserved) {
  LandingCheck lc;
  auto [ok, st] = lc.evaluate(filter(floor_(), [](const tc::P3& p) { return p.x > 0; }), O);
  EXPECT_FALSE(ok);
  EXPECT_TRUE(has(st, "not observed")) << st.dump();
}

TEST(Synthetic, ThickFloorLayerIsUncertain) {
  LandingCheck lc;
  std::uniform_real_distribution<double> U(0, 0.28);
  Cloud P = floor_();
  for (auto& p : P) p.z += U(rng);
  EXPECT_FALSE(lc.evaluate(P, O).first);
}

// the odometry point is 0.193 m ahead of the frame centre: a post 0.57-0.67 m behind the spot lies
// outside a disc centred on the spot (r 0.542 m) but inside the 0.2 m margin of the frame
TEST(Synthetic, ObstacleJustBehindTheFrameIsCaught) {
  LandingCheck lc;
  Cloud P = cat({floor_(), box(-0.62, 0.0, 0.5, 0.1, 0.0, 0.02)});
  auto [ok0, st0] = lc.evaluate(P, O);   // no yaw: centred on the spot (the old way)
  EXPECT_TRUE(ok0) << st0.dump();
  auto [ok, st] = lc.evaluate(P, O, std::nullopt, 0.0);
  EXPECT_FALSE(ok);
  EXPECT_TRUE(has(st, "occupied")) << st.dump();
  EXPECT_EQ(st["spot"], Json::array({0.0, 0.0}));
  EXPECT_EQ(st["centre"], Json::array({-0.19, 0.0}));
  // heading the other way, the post is in front, beyond the margin
  auto [ok2, st2] = lc.evaluate(P, O, std::nullopt, M_PI);
  EXPECT_TRUE(ok2) << st2.dump();
  EXPECT_EQ(st2["centre"], Json::array({0.19, 0.0}));
}

TEST(Synthetic, CentreFollowsTheHeadingAndTheOffset) {
  LandingCheck lc;
  EXPECT_LT((lc.centre(V2(1, 2), M_PI / 2) - V2(1.0, 2.0 - 0.193)).norm(), 1e-9);
  Params p;
  p.footprint_offset = 0.0;
  EXPECT_LT((LandingCheck(p).centre(V2(1, 2), 0.3) - V2(1, 2)).norm(), 1e-12);
  EXPECT_LT((lc.centre(V2(1, 2), std::nullopt) - V2(1, 2)).norm(), 1e-12);
}

TEST(Synthetic, VizHoldsTheOccupiedCellsAndTheFloor) {
  LandingCheck lc;
  auto [ok, st] = lc.evaluate(cat({floor_(0, 0, 1.0, 0.4), box(0.1, 0.0, 0.3, 0.3, 0.4)}), O, std::nullopt, 0.0);
  EXPECT_FALSE(ok);
  EXPECT_NEAR(*lc.viz.floor_z, 0.4, 0.03);
  EXPECT_EQ(int(lc.viz.occupied.size()), st["obstacle_cells"].get<int>());
  for (auto& q : lc.viz.occupied) {
    EXPECT_LT(std::abs(q[0] - 0.1), 0.25);
    EXPECT_LT(std::abs(q[1]), 0.25);
    EXPECT_GT(q[2], 0.1);
  }
  EXPECT_TRUE(lc.viz.unobserved.empty());
}

TEST(Synthetic, VizHoldsTheUnobservedCells) {
  LandingCheck lc;
  auto [ok, st] = lc.evaluate(filter(floor_(), [](const tc::P3& p) { return p.x > 0; }), O, std::nullopt, 0.0);
  EXPECT_FALSE(ok);
  EXPECT_GT(lc.viz.unobserved.size(), 20u);
  for (auto& u : lc.viz.unobserved) EXPECT_LT(u.x(), 0.05);
}

TEST(Synthetic, BoxOverMostOfTheDiscIsNotTakenForTheFloor) {
  LandingCheck lc;
  Cloud P = filter(floor_(0, 0, 2.0), [](const tc::P3& p) { return !(std::abs(p.x) < 0.45 && std::abs(p.y) < 0.45); });
  auto [ok, st] = lc.evaluate(cat({P, box(0, 0, 0.4, 0.9)}), O);
  EXPECT_FALSE(ok) << st.dump();
}

// ---- KnownFloor (owner decision 2026-10-28: where the robot sat on the ground is known floor)
TEST(KnownFloor, HiddenFootprintNotObservedWithoutClearWithTheTakeoffSpot) {
  LandingCheck lc;
  Cloud P = hide_where_it_sat(floor_(0, 0, 1.5));
  auto [ok, st] = lc.evaluate(P, O, std::nullopt, 0.0);
  EXPECT_FALSE(ok);
  EXPECT_TRUE(has(st, "not observed")) << st.dump();
  auto [ok2, st2] = lc.evaluate(P, O, std::nullopt, 0.0, {{0.0, 0.0, 0.0}});
  EXPECT_TRUE(ok2) << st2.dump();
  EXPECT_LT(st2["observed_measured"].get<double>(), 0.7);
  EXPECT_EQ(st2["known_floor_cells"].get<int>(), 25);   // the 5 x 5 cells under the frame
  EXPECT_GE(st2["observed"].get<double>(), 0.8);
  EXPECT_EQ(lc.viz.known_cells.size(), 25u);
  EXPECT_EQ(lc.viz.known_floor.size(), 1u);
  for (auto& u : lc.viz.unobserved) EXPECT_LT(u.x(), -0.44);   // the shadow behind the frame: still unobserved
}

TEST(KnownFloor, RotatedSpotAndALandingHeading) {
  LandingCheck lc;
  Cloud P = hide_where_it_sat(floor_(2.0, 1.0, 1.5), V2(2, 1), M_PI / 2);
  EXPECT_FALSE(lc.evaluate(P, V2(2, 1), std::nullopt, M_PI / 2).first);
  auto [ok, st] = lc.evaluate(P, V2(2, 1), std::nullopt, M_PI / 2, {{2.0, 1.0, M_PI / 2}});
  EXPECT_TRUE(ok) << st.dump();
}

TEST(KnownFloor, BoxOnTheTakeoffSpotIsOccupied) {
  LandingCheck lc;
  Cloud P = hide_where_it_sat(floor_(0, 0, 1.5));
  auto [ok, st] = lc.evaluate(cat({P, box(-0.19, 0.0, 0.3)}), O, std::nullopt, 0.0, {{0, 0, 0}});
  EXPECT_FALSE(ok);
  EXPECT_TRUE(has(st, "occupied")) << st.dump();
  auto [ok2, st2] = lc.evaluate(cat({P, box(-0.1, 0.05, 0.12)}), O, std::nullopt, 0.0, {{0, 0, 0}});
  EXPECT_FALSE(ok2);
  EXPECT_TRUE(has(st2, "occupied")) << st2.dump();
}

TEST(KnownFloor, ASpotElsewhereDoesNotHelp) {
  LandingCheck lc;
  auto [ok, st] = lc.evaluate(hide_where_it_sat(floor_(0, 0, 1.5)), O, std::nullopt, 0.0, {{3.0, 0.0, 0.0}});
  EXPECT_FALSE(ok);
  EXPECT_FALSE(st.contains("known_footprints"));
  EXPECT_TRUE(lc.viz.known_floor.empty());
}

TEST(KnownFloor, KnownFloorDoesNotClearASpotNeverSeen) {
  LandingCheck lc;
  EXPECT_FALSE(lc.evaluate(floor_(5.0), O, std::nullopt, 0.0, {{0, 0, 0}}).first);
}

TEST(KnownFloor, ScanByScan) {
  LandingCheck lc;
  std::vector<Scan> scans;
  for (auto& s : drifting_scans()) scans.push_back({hide_where_it_sat(s.pts), std::nullopt});
  EXPECT_FALSE(lc.evaluate_scans(scans, O, std::nullopt, 0.0).first);
  auto [ok, st] = lc.evaluate_scans(scans, O, std::nullopt, 0.0, {{0, 0, 0}});
  EXPECT_TRUE(ok) << st.dump();
  // a box put on the spot: seen by the last 5 scans (on each scan's own drifted floor)
  for (int k = 15; k < 20; ++k) scans[k].pts = cat({scans[k].pts, shift(box(-0.19, 0.0, 0.3), 0.25 * k / 19)});
  auto [ok2, st2] = lc.evaluate_scans(scans, O, std::nullopt, 0.0, {{0, 0, 0}});
  EXPECT_FALSE(ok2);
  EXPECT_TRUE(has(st2, "occupied")) << st2.dump();
}

// ---- ScanByScan: the node's case, the window's scans one by one, each fitted to the local ground model
TEST(ScanByScan, DriftingFloorMergedIsUncertainButScanByScanIsClear) {
  LandingCheck lc;
  auto scans = drifting_scans(nullptr, 20, 0.3);
  Cloud merged;
  for (auto& s : scans) merged.insert(merged.end(), s.pts.begin(), s.pts.end());
  EXPECT_FALSE(lc.evaluate(merged, O).first);
  auto [ok, st] = lc.evaluate_scans(scans, O);
  EXPECT_TRUE(ok) << st.dump();
  EXPECT_LT(st["floor_layer_m"].get<double>(), 0.05);
}

TEST(ScanByScan, LowBoxOnADriftingFloorIsOccupied) {
  LandingCheck lc;
  auto [ok, st] = lc.evaluate_scans(drifting_scans([](int) { return box(0.1, 0.0, 0.10); }), O);
  EXPECT_FALSE(ok);
  EXPECT_TRUE(has(st, "occupied")) << st.dump();
}

TEST(ScanByScan, ObjectThatLeftIsClearedByTheLatestScans) {
  LandingCheck lc;
  auto [ok, st] = lc.evaluate_scans(drifting_scans([](int k) { return k < 8 ? box(0.1, 0.0, 0.5) : Cloud{}; }), O);
  EXPECT_TRUE(ok) << st.dump();
}

TEST(ScanByScan, ObjectThatArrivedIsSeenByTheLatestScans) {
  LandingCheck lc;
  auto [ok, st] = lc.evaluate_scans(drifting_scans([](int k) { return k >= 16 ? box(0.1, 0.0, 0.5) : Cloud{}; }), O);
  EXPECT_FALSE(ok);
  EXPECT_TRUE(has(st, "occupied")) << st.dump();
}

TEST(ScanByScan, OverheadBeamSeenScanByScanIsOccupied) {
  LandingCheck lc;
  EXPECT_FALSE(lc.evaluate_scans(drifting_scans([](int) { return beam(1.5); }, 20, 0.1), O, 2.5).first);
}

// ---- Markers: /planner/landing_markers built from an evaluation (no ROS master needed)
namespace {
std::map<std::string, Marker> ns(const visualization_msgs::MarkerArray& a) {
  std::map<std::string, Marker> m;
  for (auto& x : a.markers) m[x.ns] = x;
  return m;
}
}  // namespace

TEST(Markers, OccupiedMarkers) {
  LandingCheck lc;
  auto [ok, st] = lc.evaluate(cat({floor_(0, 0, 1.0, 0.4), box(0.1, 0.0, 0.3, 0.3, 0.4)}), O, std::nullopt, 0.5);
  auto arr = build_markers(ok, st, lc.viz, "world", ros::Time(0), 0.0);
  auto m = ns(arr);
  std::set<std::string> want{"landing_disc", "landing_footprint", "landing_occupied", "landing_unobserved",
                             "landing_known_floor", "landing_label"}, got;
  for (auto& kv : m) got.insert(kv.first);
  EXPECT_EQ(got, want);
  EXPECT_EQ(m["landing_known_floor"].action, Marker::DELETE);
  for (auto& x : arr.markers) EXPECT_EQ(x.header.frame_id, "world");
  auto& d = m["landing_disc"];
  EXPECT_FLOAT_EQ(d.color.r, RED[0]);
  EXPECT_FLOAT_EQ(d.color.g, RED[1]);
  EXPECT_LT(d.color.a, 1.0);
  EXPECT_NEAR(d.pose.position.z, 0.4, 0.04);
  EXPECT_NEAR(d.pose.position.x, -0.193 * std::cos(0.5), 0.01);
  EXPECT_DOUBLE_EQ(d.scale.x, 2 * lc.radius);
  auto& o = m["landing_occupied"];
  EXPECT_EQ(int(o.points.size()), st["obstacle_cells"].get<int>());
  for (auto& p : o.points) EXPECT_TRUE(0.5 < p.z && p.z < 0.75) << p.z;
  EXPECT_EQ(m["landing_unobserved"].action, Marker::DELETE);
  EXPECT_EQ(m["landing_footprint"].points.size(), 7u);
  EXPECT_EQ(m["landing_label"].text.rfind("OCCUPIED ", 0), 0u) << m["landing_label"].text;
}

TEST(Markers, ClearAndNotObservedAndNoHeading) {
  LandingCheck lc;
  auto [ok, st] = lc.evaluate(floor_(), O, std::nullopt, 0.0);
  auto m = ns(build_markers(ok, st, lc.viz, "world", ros::Time(0), 0.0));
  EXPECT_FLOAT_EQ(m["landing_disc"].color.r, GREEN[0]);
  EXPECT_FLOAT_EQ(m["landing_disc"].color.g, GREEN[1]);
  EXPECT_EQ(m["landing_label"].text.rfind("CLEAR layer ", 0), 0u) << m["landing_label"].text;
  EXPECT_EQ(m["landing_occupied"].action, Marker::DELETE);
  auto [ok2, st2] = lc.evaluate(filter(floor_(), [](const tc::P3& p) { return p.x > 0; }), O, std::nullopt, 0.0);
  m = ns(build_markers(ok2, st2, lc.viz, "world", ros::Time(0), 0.0));
  EXPECT_FLOAT_EQ(m["landing_disc"].color.r, AMBER[0]);
  EXPECT_FLOAT_EQ(m["landing_disc"].color.g, AMBER[1]);
  EXPECT_GT(m["landing_unobserved"].points.size(), 20u);
  EXPECT_EQ(m["landing_label"].text.rfind("NOT OBSERVED", 0), 0u);
  Json st3;
  st3["spot"] = {1.0, 2.0};
  st3["centre"] = {1.0, 2.0};
  st3["radius_m"] = 0.542;
  st3["reason"] = "no heading: no /planner/odom yet";
  m = ns(build_markers(false, st3, Viz(), "world", ros::Time(0), 0.3));
  EXPECT_FLOAT_EQ(m["landing_disc"].color.r, AMBER[0]);
  EXPECT_FLOAT_EQ(m["landing_disc"].color.g, AMBER[1]);
  EXPECT_NEAR(m["landing_disc"].pose.position.z, 0.305, 1e-9);
  EXPECT_EQ(m["landing_footprint"].action, Marker::DELETE);
  EXPECT_EQ(m["landing_label"].text.rfind("NO HEADING", 0), 0u);
}

TEST(Markers, KnownFloorMarker) {
  LandingCheck lc;
  auto [ok, st] = lc.evaluate(hide_where_it_sat(floor_(0, 0, 1.5, 0.4)), O, std::nullopt, 0.0, {{0, 0, 0}});
  ASSERT_TRUE(ok) << st.dump();
  auto m = ns(build_markers(ok, st, lc.viz, "world", ros::Time(0), 0.0));
  auto& k = m["landing_known_floor"];
  EXPECT_EQ(k.type, Marker::LINE_LIST);
  EXPECT_EQ(k.action, Marker::ADD);
  EXPECT_EQ(k.points.size(), 8u);   // one rectangle
  EXPECT_FLOAT_EQ(k.color.r, LIGHT_BLUE[0]);
  EXPECT_FLOAT_EQ(k.color.g, LIGHT_BLUE[1]);
  EXPECT_FLOAT_EQ(k.color.b, LIGHT_BLUE[2]);
  double lo = 1e9, hi = -1e9;
  for (auto& p : k.points) {
    lo = std::min(lo, p.x);
    hi = std::max(hi, p.x);
    EXPECT_LT(std::abs(p.z - 0.415), 0.04);
  }
  EXPECT_NEAR(lo, -0.193 - 0.254, 1e-3);
  EXPECT_NEAR(hi, -0.193 + 0.254, 1e-3);
  EXPECT_EQ(m["landing_unobserved"].points.size(), lc.viz.unobserved.size());
}

TEST(Markers, GroundSpotsToThePlannerFrame) {
  // the bridge's anchor fc = Rz(psi) planner + t; a spot recorded in fc comes back in planner
  double psi = 0.6, tx = 2.0, ty = -1.0, c = std::cos(psi), s = std::sin(psi);
  double px = 1.5, py = 0.5, pyaw = 0.3;
  GroundSpot fc{c * px - s * py + tx, s * px + c * py + ty, pyaw + psi};
  auto out = ground_spots_to_planner({fc}, psi, tx, ty);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_NEAR(out[0].x, px, 1e-9);
  EXPECT_NEAR(out[0].y, py, 1e-9);
  EXPECT_NEAR(out[0].yaw, pyaw, 1e-9);
}

// ---- real data
namespace {
std::string slurp(const std::string& p) {
  std::ifstream f(p, std::ios::binary);
  if (!f) throw std::runtime_error("cannot read " + p);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}
Cloud load_pcd(const std::string& p) {
  std::string raw = slurp(p);
  const std::string tag = "DATA binary\n";
  size_t k = raw.find(tag);
  std::istringstream hdr(raw.substr(0, k));
  std::string line;
  long n = 0;
  int nf = 0;
  while (std::getline(hdr, line)) {
    std::istringstream ls(line);
    std::string key;
    ls >> key;
    if (key == "POINTS") ls >> n;
    if (key == "FIELDS") {
      std::string f;
      while (ls >> f) nf++;
    }
  }
  Cloud out(n);
  for (long i = 0; i < n; ++i) {
    float v[3];
    std::memcpy(v, raw.data() + k + tag.size() + size_t(i) * nf * 4, 12);
    out[i] = {v[0], v[1], v[2]};
  }
  return out;
}
std::vector<std::vector<double>> load_txt(const std::string& p, char delim = 0, int skip = 0) {
  std::ifstream f(p);
  std::string line;
  std::vector<std::vector<double>> rows;
  int ln = 0;
  while (std::getline(f, line)) {
    if (ln++ < skip) continue;
    if (delim) std::replace(line.begin(), line.end(), delim, ' ');
    std::istringstream ls(line);
    std::vector<double> r;
    double v;
    while (ls >> v) r.push_back(v);
    if (!r.empty()) rows.push_back(r);
  }
  return rows;
}
using Traj = std::vector<Eigen::Vector3d>;
Traj traj_csv(const std::string& p) {
  Traj t;
  for (auto& r : load_txt(p, ',', 1)) t.push_back(Eigen::Vector3d(r[1], r[2], r[3]));
  return t;
}
Traj traj_meta(const std::string& p) {
  auto m = nlohmann::json::parse(slurp(p));
  Traj t;
  for (auto& r : m["traj"]) t.push_back(Eigen::Vector3d(r[0].get<double>(), r[1].get<double>(), r[2].get<double>()));
  return t;
}

// an SC-PGO run's keyframe scans in a map's frame (rigid fit of the keyframe positions to the map's
// path), FAST-LIO's 0.4 m blind applied
struct Keyframes {
  std::vector<Eigen::Matrix3d> Rk;
  std::vector<Eigen::Vector3d> pk;
  std::vector<double> t;
  std::string out;
  Keyframes(const std::string& dir, const Traj& traj) : out(dir) {
    auto T = load_txt(dir + "/optimized_poses.txt");
    std::vector<double> ts;
    for (auto& r : load_txt(dir + "/times.txt"))
      for (double v : r) ts.push_back(v);
    size_t n = std::min({T.size(), ts.size(), traj.size()});
    t.assign(ts.begin(), ts.begin() + n);
    Eigen::MatrixXd A(n, 3), B(n, 3);
    for (size_t i = 0; i < n; ++i) {
      A.row(i) << T[i][3], T[i][7], T[i][11];
      B.row(i) = traj[i].transpose();
    }
    Eigen::RowVector3d ca = A.colwise().mean(), cb = B.colwise().mean();
    Eigen::Matrix3d H = (A.rowwise() - ca).transpose() * (B.rowwise() - cb);
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU(), V = svd.matrixV();
    double d = (V * U.transpose()).determinant() > 0 ? 1.0 : -1.0;
    Eigen::Matrix3d R = V * Eigen::Vector3d(1, 1, d).asDiagonal() * U.transpose();
    for (size_t i = 0; i < n; ++i) {
      Eigen::Matrix3d Ri;
      Ri << T[i][0], T[i][1], T[i][2], T[i][4], T[i][5], T[i][6], T[i][8], T[i][9], T[i][10];
      Rk.push_back(R * Ri);
      pk.push_back(R * A.row(i).transpose() + (cb.transpose() - R * ca.transpose()));
    }
  }
  // [(scan, sensor)] of the keyframes within r of xy, in time order
  std::vector<Scan> scans_near(const V2& xy, double r = 10.0, std::optional<double> t_max = std::nullopt) const {
    std::vector<Scan> o;
    for (size_t k = 0; k < t.size(); ++k) {
      if (std::hypot(pk[k].x() - xy.x(), pk[k].y() - xy.y()) >= r || (t_max && t[k] - t[0] > *t_max)) continue;
      char name[64];
      snprintf(name, sizeof name, "/Scans/%06zu.pcd", k);
      Cloud S;
      for (auto& p : load_pcd(out + name)) {
        Eigen::Vector3d b(p.x, p.y, p.z);
        if (!b.allFinite() || b.norm() < 0.4) continue;
        Eigen::Vector3d q = Rk[k] * b + pk[k];
        S.push_back({q.x(), q.y(), q.z()});
      }
      o.push_back({S, pk[k]});
    }
    return o;
  }
  double heading(int k) const {   // the sensor's +z (raw FAST-LIO body: forward) in the map frame
    if (k < 0) k += int(Rk.size());
    Eigen::Vector3d f = Rk[k].col(2);
    return std::atan2(f.y(), f.x());
  }
};

const char* env(const char* n) {
  const char* v = std::getenv(n);
  return v && *v ? v : nullptr;
}
double pct_z_near(const Cloud& S, const V2& c, double r, double q, int* n = nullptr) {
  std::vector<double> z;
  for (auto& p : S)
    if (std::hypot(p.x - c.x(), p.y - c.y()) < r) z.push_back(p.z);
  if (n) *n = int(z.size());
  return z.empty() ? 0.0 : tc::percentile(z, q);
}
}  // namespace

// Hensel Phelps 2026-10-15 and CAN USA 2026-07-15 20:34:50 (Shannon: ~/sim/marsim/maps)
class RealMaps : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!env("LANDING_MAPS")) return;
    std::string d = env("LANDING_MAPS");
    hp = new Cloud(load_pcd(d + "/hp_1015_carved.pcd"));
    cu = new Cloud(load_pcd(d + "/canusa_2034.pcd"));
  }
  void SetUp() override {
    if (!env("LANDING_MAPS")) GTEST_SKIP() << "set LANDING_MAPS to the directory with the map PCDs";
  }
  static Cloud *hp, *cu;
  LandingCheck lc;
};
Cloud *RealMaps::hp = nullptr, *RealMaps::cu = nullptr;

TEST_F(RealMaps, OpenFloorIsClear) { EXPECT_TRUE(lc.evaluate(*hp, V2(-5.7, -8.4)).first); }
TEST_F(RealMaps, WhereTheRobotLandedIsClear) { EXPECT_TRUE(lc.evaluate(*cu, V2(-0.42, 1.45)).first); }
TEST_F(RealMaps, FurnitureAndRacksAreNotClear) {
  EXPECT_FALSE(lc.evaluate(*hp, V2(-3.75, -5.25)).first);
  EXPECT_FALSE(lc.evaluate(*hp, V2(2.25, -1.25)).first);
  EXPECT_FALSE(lc.evaluate(*cu, V2(2.25, -6.25)).first);
  EXPECT_FALSE(lc.evaluate(*cu, V2(-0.75, -6.75)).first);
}
TEST_F(RealMaps, BoxOnOpenFloorIsNotClear) {
  double z0 = pct_z_near(*hp, V2(-5.7, -8.4), 0.5, 5);
  EXPECT_FALSE(lc.evaluate(cat({*hp, box(-5.7, -8.4, 0.3, 0.3, z0)}), V2(-5.7, -8.4)).first);
}
TEST_F(RealMaps, UnobservedIsNotClear) { EXPECT_FALSE(lc.evaluate(*hp, V2(30.0, 30.0)).first); }

// Recorded keyframe scans, scan by scan, descending from 1.5 m: the takeoff and landing spots of the
// 2026-07-15 flights and Hensel Phelps 2026-10-15; furniture; a box put on a spot
class RealScans : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!env("LANDING_SIM")) GTEST_SKIP() << "set LANDING_SIM to Shannon's ~/sim";
    H = env("LANDING_SIM");
  }
  std::string H;
  LandingCheck lc;
  std::pair<Traj, Keyframes> run(const std::string& name) {
    if (name == "20_21_51") {
      Traj tr = traj_csv(H + "/track/maps_tfix/20_21_51_tfix_path.csv");
      return {tr, Keyframes(H + "/tfix/maps/20_21_51/tfix/out", tr)};
    }
    if (name == "house") {
      Traj tr = traj_csv(H + "/track/maps_tfix/house_tfix_path.csv");
      return {tr, Keyframes(H + "/tfix/maps/house/tfix/out", tr)};
    }
    Traj tr = traj_meta(H + "/flights/" + name + "/view/map_meta.json");
    return {tr, Keyframes(H + "/flights/" + name + "/out", tr)};
  }
  std::pair<bool, Json> judge(const Traj& tr, const Keyframes& kf, const V2& xy, double r = 10.0,
                              std::optional<double> t_max = std::nullopt) {
    return lc.evaluate_scans(kf.scans_near(xy, r, t_max), xy, tr[0].z() + 1.5);
  }
};

TEST_F(RealScans, FlightTakeoffAndLandingSpotsAreClear) {
  for (auto [name, ends] : std::vector<std::pair<std::string, std::vector<int>>>{
           {"20_21_51", {0, -1}}, {"20_34_50", {0, -1}}, {"20_18_07", {0, -1}}, {"hp_1015", {0}}}) {
    auto [tr, kf] = run(name);
    for (int e : ends) {
      auto& p = tr[e < 0 ? tr.size() - 1 : e];
      auto [ok, st] = judge(tr, kf, p.head<2>());
      EXPECT_TRUE(ok) << name << " " << e << " " << st.dump();
    }
  }
}

TEST_F(RealScans, Merged202151ScansRefuseTheTakeoffSpotButScanByScanIsClear) {
  auto [tr, kf] = run("20_21_51");
  V2 xy = tr[0].head<2>();
  Cloud P;
  for (auto& s : kf.scans_near(xy, 3.0)) P.insert(P.end(), s.pts.begin(), s.pts.end());
  auto [ok, st] = lc.evaluate(P, xy, tr[0].z() + 1.5);
  EXPECT_FALSE(ok) << st.dump();   // merged: floor drift reads as obstacles up to ~0.4 m (or a thick layer)
  EXPECT_TRUE(judge(tr, kf, xy, 3.0).first);
}

TEST_F(RealScans, LowBoxOnALandingSpotIsCaught) {
  auto [tr, kf] = run("20_34_50");
  V2 end = tr.back().head<2>(), c = end + V2(0.15, 0.0);
  std::vector<Scan> scans;
  for (auto& s : kf.scans_near(end)) {
    int n;
    double z0 = pct_z_near(s.pts, c, 1.0, 30, &n);
    Cloud S = s.pts;
    if (n >= 20) {   // a 0.3 m box 0.10 m high on this scan's own floor
      S = filter(S, [&](const tc::P3& p) {
        return !(std::abs(p.x - c.x()) <= 0.15 && std::abs(p.y - c.y()) <= 0.15 && p.z < z0 + 0.12);
      });
      S = cat({S, box(c.x(), c.y(), 0.10, 0.3, z0, 0.03)});
    }
    scans.push_back({S, s.sensor});
  }
  auto [ok, st] = lc.evaluate_scans(scans, end, tr[0].z() + 1.5);
  EXPECT_FALSE(ok);
  EXPECT_TRUE(has(st, "occupied")) << st.dump();
}

TEST_F(RealScans, FurnitureIsNotClear) {
  auto [tr, kf] = run("hp_1015");
  auto [ok, st] = judge(tr, kf, V2(-3.75, -5.25));
  EXPECT_FALSE(ok);
  EXPECT_TRUE(has(st, "occupied")) << st.dump();
}

// owner decision 2026-10-28: with the disc on the frame the Hensel Phelps takeoff spot and the 20:18:07
// landing spot read "not observed" (62%); as known floor they are clear, and a box put there is still
// occupied
TEST_F(RealScans, SpotsWhereItSatAreKnownFloor) {
  for (auto [name, e, k] : std::vector<std::tuple<std::string, int, int>>{{"hp_1015", 0, 0}, {"20_18_07", -1, -1}}) {
    auto [tr, kf] = run(name);
    V2 xy = tr[e < 0 ? tr.size() - 1 : e].head<2>();
    double yaw = kf.heading(k), top = tr[0].z() + 1.5;
    auto scans = kf.scans_near(xy);
    auto [ok, st] = lc.evaluate_scans(scans, xy, top, yaw);
    EXPECT_FALSE(ok) << name << " " << st.dump();
    EXPECT_TRUE(has(st, "not observed")) << st.dump();
    auto [ok2, st2] = lc.evaluate_scans(scans, xy, top, yaw, {{xy.x(), xy.y(), yaw}});
    printf("  %s %s: measured %.0f%%, with the footprint %.0f%%, %s\n", name.c_str(), e == 0 ? "takeoff" : "landing",
           100 * st2.value("observed_measured", 0.0), 100 * st2.value("observed", 0.0), reason(st2).c_str());
    EXPECT_TRUE(ok2) << name << " " << st2.dump();
    // a 0.3 m box on the spot, under the frame (on each scan's own floor)
    V2 c = lc.centre(xy, yaw);
    std::vector<Scan> boxed;
    for (auto& s : scans) {
      int n;
      double z0 = pct_z_near(s.pts, c, 1.0, 30, &n);
      boxed.push_back({n >= 20 ? cat({s.pts, box(c.x(), c.y(), 0.30, 0.3, z0, 0.03)}) : s.pts, s.sensor});
    }
    auto [ok3, st3] = lc.evaluate_scans(boxed, xy, top, yaw, {{xy.x(), xy.y(), yaw}});
    EXPECT_FALSE(ok3) << name;
    EXPECT_TRUE(has(st3, "occupied")) << st3.dump();
  }
}

// the house drive's start: clear in the first 160 s of scans; later scans see something 1.1-1.55 m
// tall over the robot's rear edge for a few seconds at a time (a person), and those were the last
// looks at it, so the whole drive reads occupied (the latest wins)
TEST_F(RealScans, HouseStartClearUntilSomeoneStandsThere) {
  auto [tr, kf] = run("house");
  V2 xy = tr[0].head<2>();
  auto [ok, st] = judge(tr, kf, xy, 10.0, 160.0);
  EXPECT_TRUE(ok) << st.dump();
  EXPECT_FALSE(judge(tr, kf, xy).first);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
