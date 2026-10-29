// Landing-spot check (D3), ROS-free: is a planned landing spot safe to land on?
// C++ port of planner/landing_core.py (2026-10-29).
//
// The robot's footprint (0.508 x 0.457 m) plus a margin, taken as a yaw-independent disc of radius
// half-diagonal + margin (0.542 m) around the frame centre, must be all of:
//   (a) observed: enough of the disc has ground returns (unobserved = not clear),
//   (b) flat: a ground plane fits with a small slope and residual,
//   (c) free of obstacles: no points higher than the floor layer + obstacle_min_h above that plane, up
//       to the height the robot descends from (which also covers overhead obstacles).
// Anything uncertain is not clear. Points are in the planner frame (z up), metres.
//
// The disc's centre (owner decision 2026-10-28). The spot is where the odometry point (FAST-LIO's IMU
// frame) comes down, footprint_offset = l4_track::ODOM_AHEAD = 0.193 m ahead of the frame centre along
// the heading. Given the yaw (the node takes it from /planner/odom: QuadRover descends without
// yawing), the disc is centred on the frame: spot - footprint_offset * (cos yaw, sin yaw), so the
// margin is 0.2 m all round the frame. Without a yaw the disc is centred on the spot itself (the node
// answers "no heading" instead). The status gives both ("spot", "centre"). The FC (and its IMU) sits
// at the frame centre (owner 2026-10-29): the disc is centred on it.
//
// The floor. Recorded maps smear the floor to 0.2-0.26 m, but one scan's floor is only 0.03-0.05 m
// thick: the smear is scans disagreeing on the floor height (slow z drift between visits) plus things
// that moved (people, doors). So the check takes the scans one by one (evaluate_scans: the node's
// window, or a map's keyframes) and:
//   1. builds the track planner's local ground model (l4_track::local_ground: per 0.5 m tile the 20th
//      percentile of the cells' lowest voxels, a sofa-like tile replaced by its neighbours' median, a
//      ceiling-only tile by the nearest good tile) over the merged scans around the spot;
//   2. fits each scan's own floor offset from that model (the median, over the 10 cm cells within
//      align_radius whose lowest point lies within align_band of the model, of lowest point - model)
//      and shifts the scan by it, so every scan's floor lands on the model: the merged floor layer is
//      one scan's thickness again (~0.01-0.05 m) and objects from ~0.06-0.10 m up are seen (a scan that
//      sees too little floor near the spot keeps only its returns more than unaligned_min_h up);
//   3. seeds the plane fit with the cells near the model (a box covering most of the disc is not taken
//      for the floor);
//   4. counts a 10 cm cell as occupied by the latest observations: of the last evidence_last scans that
//      returned anything in the cell (below the descent height), at least evidence_frac have a point
//      above the obstacle threshold there, or the last evidence_latest all do (and 2 points in all).
//      An object that left is cleared by the scans that saw the floor since; one that arrived is seen
//      by the latest ones.
// A single merged cloud (evaluate(points)) is one scan: no alignment, the floor layer is the map's own
// smear, and a layer thicker than floor_layer_max is not clear (uncertain).
//
// Near range. On the 07/15 20:21:51 flight (hovering 0.5 m over the takeoff spot) floor returns nearer
// than ~1.2 m to the lidar read 3-6 cm high (median; p95 10-13 cm), enough to make the floor under the
// robot an "obstacle"; beyond 1.2 m they agree to ~1 cm. A scan given with its sensor position drops
// returns nearer than near_range (1.2 m). So the spot must have been seen from at least that far: hover
// at 1.2 m or more, or approach nose-first so the scans of the approach see it.
//
// Known floor (owner decision 2026-10-28). Where the robot sat on the ground (its takeoff spot, the
// spots it landed on earlier in the mission) its own body hid the floor from the lidar: with the disc on
// the frame such a spot read "not observed" (Hensel Phelps 10/15 takeoff spot and the 07/15 20:18:07
// landing spot: 62%). known = [(x, y, yaw), ...] takes those poses of the odometry point (planner
// frame) and their frame footprints (the 0.508 x 0.457 m rectangle, centred footprint_offset behind the
// odometry point): a disc cell with no ground returns whose centre lies inside one of them counts as
// observed floor, at the floor plane fitted to the floor seen around it (the floor it sat on). Only the
// observed share changes: points in those cells still count, so something put there later is still
// occupied, and the plane, flatness and floor layer come from real returns only. The status gives
// "observed_measured" (without them), "known_floor_cells" and "known_footprints".
//
// For the markers the last evaluation's geometry is kept in viz (plain numbers, no ROS).
//
// LIMITS: the disc is centred on the frame for the heading at the time of the check: a yaw during the
// descent (QuadRover does not yaw) or a footprint_offset that is not the real IMU offset moves the frame
// off it (the 0.2 m margin absorbs ~0.2 m); objects lower than about the floor layer + 5 cm (~0.06-0.13
// m scan by scan, the map's smear + 5 cm on one merged cloud) are not seen; glass, water and dark
// surfaces that return nothing read as unobserved (not clear), not as floor.
#include "l4_landing/landing_core.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <map>
#include <set>

namespace l4_landing {

namespace tc = l4_track;

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
double r_(double x, int n) { return tc::pyround(x, n); }          // Python round(float, n)
double nr_(double x, int n) {                                       // numpy round (np.float64)
  double s = std::pow(10.0, n);
  return std::nearbyint(x * s) / s;
}
int64_t cell_key(double x, double y, const V2& o, double cell) {
  int64_t a = int64_t(std::floor((x - o.x()) / cell)), b = int64_t(std::floor((y - o.y()) / cell));
  return (a + 100000) * 200001 + (b + 100000);
}
double mean(const std::vector<double>& v) {
  double s = 0;
  for (double x : v) s += x;
  return s / v.size();
}
double pstd(const std::vector<double>& v) {
  double m = mean(v), s = 0;
  for (double x : v) s += (x - m) * (x - m);
  return std::sqrt(s / v.size());
}

struct PlaneFit {
  Eigen::Vector3d coef;
};
// least squares z = a (x - sx) + b (y - sy) + c
Eigen::Vector3d lstsq(const std::vector<tc::P3>& pts, const std::vector<size_t>& idx, double sx, double sy) {
  Eigen::MatrixXd A(idx.size(), 3);
  Eigen::VectorXd z(idx.size());
  for (size_t q = 0; q < idx.size(); ++q) {
    auto& p = pts[idx[q]];
    A.row(q) << p.x - sx, p.y - sy, 1.0;
    z(q) = p.z;
  }
  return A.colPivHouseholderQr().solve(z);
}
}  // namespace

LandingCheck::LandingCheck(const Params& p_) : p(p_) {
  radius = std::hypot(FOOTPRINT_LEN, FOOTPRINT_WID) / 2 + p.margin;
  align_radius = std::max(p.align_radius, radius);   // per-scan floor offset fitted within this
  slope_max = tc::radians(p.slope_max_deg);
  gcfg = tc::Config();
  gcfg.res = p.cell;
  gcfg.update();
}

V2 LandingCheck::centre(const V2& s, std::optional<double> yaw) const {
  if (!yaw) return s;
  return V2(s.x() - p.footprint_offset * std::cos(*yaw), s.y() - p.footprint_offset * std::sin(*yaw));
}

std::array<V2, 4> LandingCheck::footprint_corners(const V2& odom, double yaw) const {
  V2 c = centre(odom, yaw);
  double cs = std::cos(yaw), sn = std::sin(yaw), hl = FOOTPRINT_LEN / 2, hw = FOOTPRINT_WID / 2;
  const double uv[4][2] = {{hl, hw}, {-hl, hw}, {-hl, -hw}, {hl, -hw}};
  std::array<V2, 4> out;
  for (int k = 0; k < 4; ++k)
    out[k] = V2(c.x() + cs * uv[k][0] - sn * uv[k][1], c.y() + sn * uv[k][0] + cs * uv[k][1]);
  return out;
}

bool LandingCheck::in_footprint(const V2& xy, const V2& odom, double yaw) const {
  V2 c = centre(odom, yaw);
  double dx = xy.x() - c.x(), dy = xy.y() - c.y(), cs = std::cos(yaw), sn = std::sin(yaw);
  double u = dx * cs + dy * sn, v = -dx * sn + dy * cs;
  return std::abs(u) <= FOOTPRINT_LEN / 2 && std::abs(v) <= FOOTPRINT_WID / 2;
}

std::pair<bool, Json> LandingCheck::evaluate(const tc::Cloud& points, const V2& spot, std::optional<double> descend_from,
                                             std::optional<double> yaw, const std::vector<KnownPose>& known) {
  return evaluate_scans({Scan{points, std::nullopt}}, spot, descend_from, yaw, known);
}

std::pair<bool, Json> LandingCheck::evaluate_scans(const std::vector<Scan>& raw, const V2& spot_xy,
                                                   std::optional<double> descend_from, std::optional<double> yaw,
                                                   const std::vector<KnownPose>& known_in) {
  const V2 spot = centre(spot_xy, yaw);
  const double sx = spot.x(), sy = spot.y(), cell = p.cell;
  Json st;
  st["spot"] = {r_(spot_xy.x(), 2), r_(spot_xy.y(), 2)};
  st["centre"] = {nr_(sx, 2), nr_(sy, 2)};
  st["radius_m"] = r_(radius, 3);
  if (yaw) {
    st["yaw_deg"] = r_(tc::degrees(*yaw), 1);
    st["footprint_offset_m"] = r_(p.footprint_offset, 3);
  }
  viz = Viz();
  viz.valid = true;
  viz.centre = spot;
  viz.yaw = yaw;
  viz.radius = radius;
  viz.cell = cell;
  // the known-floor footprints that reach the disc
  double reach = radius + std::hypot(FOOTPRINT_LEN, FOOTPRINT_WID) / 2 + p.footprint_offset;
  std::vector<KnownPose> known;
  for (auto& k : known_in)
    if (std::hypot(k.x - sx, k.y - sy) <= reach) known.push_back(k);
  for (auto& k : known) viz.known_floor.push_back(footprint_corners(V2(k.x, k.y), k.yaw));
  if (!known.empty()) st["known_footprints"] = int(known.size());
  const double box = align_radius + gcfg.tile;
  std::vector<tc::Cloud> crop;
  for (auto& S : raw) {
    tc::Cloud c;
    for (auto& q : S.pts) {
      if (!(std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z))) continue;
      if (!(std::abs(q.x - sx) <= box && std::abs(q.y - sy) <= box)) continue;
      if (S.sensor && p.near_range > 0) {
        Eigen::Vector3d d(q.x - S.sensor->x(), q.y - S.sensor->y(), q.z - S.sensor->z());
        if (!(d.norm() >= p.near_range)) continue;
      }
      c.push_back(q);
    }
    crop.push_back(std::move(c));
  }
  tc::Cloud allp;
  for (auto& c : crop) allp.insert(allp.end(), c.begin(), c.end());
  int npts = 0;
  for (auto& q : allp) npts += std::hypot(q.x - sx, q.y - sy) <= radius;
  st["points"] = npts;
  if (npts < 20) {
    st["reason"] = fmt("not observed: %d points in the patch", npts);
    return {false, st};
  }

  // 1. the local ground model; 2. each scan's floor onto it
  double m = align_radius + gcfg.tile;
  tc::Ground G = tc::local_ground(allp, gcfg, V2(sx - m, sy - m), V2(sx + m, sy + m));
  st["model_z"] = r_(G.at(sx, sy), 3);
  viz.floor_z = r_(G.at(sx, sy), 3);
  const bool multi = crop.size() > 1;
  std::vector<std::optional<double>> offs;
  std::vector<double> good;
  for (auto& S : crop) {
    std::optional<double> o;
    if (multi) {
      std::map<int64_t, double> low;
      for (auto& q : S) {
        if (!(std::hypot(q.x - sx, q.y - sy) <= align_radius)) continue;
        double h = q.z - G.at(q.x, q.y);
        int64_t k = cell_key(q.x, q.y, spot, cell);
        auto it = low.find(k);
        if (it == low.end() || h < it->second) low[k] = h;
      }
      std::vector<double> lo;
      for (auto& kv : low)
        if (std::abs(kv.second) < p.align_band) lo.push_back(kv.second);
      if (int(lo.size()) >= p.align_min_cells) o = tc::median(lo);
    }
    offs.push_back(o);
    if (o) good.push_back(*o);
  }
  double fill = good.empty() ? 0.0 : tc::median(good);
  if (multi) {
    st["scans"] = int(crop.size());
    st["scans_aligned"] = int(good.size());
    if (!good.empty()) st["scan_offset_p5_p95_m"] = {r_(tc::percentile(good, 5), 3), r_(tc::percentile(good, 95), 3)};
  }
  std::vector<tc::P3> patch;
  std::vector<int> sid;
  for (size_t k = 0; k < crop.size(); ++k) {
    for (auto& q : crop[k]) {
      if (!(std::hypot(q.x - sx, q.y - sy) <= radius)) continue;
      // too little floor to place this scan's floor: only its tall returns count (a drift of a few cm
      // cannot make them, and cannot hide them)
      if (!offs[k] && multi && !(q.z - G.at(q.x, q.y) - fill > p.unaligned_min_h)) continue;
      patch.push_back({q.x, q.y, q.z - (offs[k] ? *offs[k] : fill)});
      sid.push_back(int(k));
    }
  }
  if (patch.size() < 20) {
    st["reason"] = "not observed: no scan places its floor here";
    return {false, st};
  }
  const size_t N = patch.size();

  // 3. ground: a robust plane through the lowest point of each 10 cm cell, seeded with the cells near
  // the ground model, then the ground points within ground_band of it
  std::vector<int64_t> kk(N);
  for (size_t q = 0; q < N; ++q) kk[q] = cell_key(patch[q].x, patch[q].y, spot, cell);
  std::map<int64_t, size_t> lowi;   // per cell (sorted) its lowest point, the first of equal ones
  for (size_t q = 0; q < N; ++q) {
    auto it = lowi.find(kk[q]);
    if (it == lowi.end() || patch[q].z < patch[it->second].z) lowi[kk[q]] = q;
  }
  std::vector<size_t> lows;
  for (auto& kv : lowi) lows.push_back(kv.second);
  std::vector<uint8_t> keep(lows.size());
  for (size_t q = 0; q < lows.size(); ++q) {
    auto& l = patch[lows[q]];
    keep[q] = std::abs(l.z - G.at(l.x, l.y)) < p.seed_band;
  }
  std::optional<Eigen::Vector3d> coef;
  for (int it = 0; it < 5; ++it) {
    std::vector<size_t> sel;
    for (size_t q = 0; q < lows.size(); ++q)
      if (keep[q]) sel.push_back(lows[q]);
    if (sel.size() < 10) {
      coef.reset();
      break;
    }
    coef = lstsq(patch, sel, sx, sy);
    std::vector<double> r(lows.size()), rk;
    for (size_t q = 0; q < lows.size(); ++q) {
      auto& l = patch[lows[q]];
      r[q] = l.z - ((l.x - sx) * (*coef)(0) + (l.y - sy) * (*coef)(1) + (*coef)(2));
      if (keep[q]) rk.push_back(r[q]);
    }
    double lim = std::max(0.03, 2.5 * pstd(rk));
    std::vector<uint8_t> nw(lows.size());
    for (size_t q = 0; q < lows.size(); ++q) nw[q] = std::abs(r[q]) < lim;
    if (nw == keep) break;
    keep = nw;
  }
  if (!coef) {
    st["reason"] = "no ground plane in the patch";
    return {false, st};
  }
  std::vector<size_t> gi;
  for (size_t q = 0; q < N; ++q) {
    double r = patch[q].z - ((patch[q].x - sx) * (*coef)(0) + (patch[q].y - sy) * (*coef)(1) + (*coef)(2));
    if (std::abs(r) < p.ground_band) gi.push_back(q);
  }
  if (gi.size() < 10) {
    st["reason"] = "no ground plane in the patch";
    return {false, st};
  }
  Eigen::Vector3d c = lstsq(patch, gi, sx, sy);   // refit on the band: the floor's middle, not its bottom
  double ss = 0;
  for (size_t q : gi) {
    double r = patch[q].z - ((patch[q].x - sx) * c(0) + (patch[q].y - sy) * c(1) + c(2));
    ss += r * r;
  }
  double resid = std::sqrt(ss / gi.size());
  double slope = std::atan(std::hypot(c(0), c(1)));
  double ground_z = c(2) + (good.empty() ? 0.0 : good.back());   // back to the latest scan's level
  st["ground_z"] = r_(ground_z, 3);
  st["slope_deg"] = r_(tc::degrees(slope), 2);
  st["residual_m"] = r_(resid, 4);
  viz.floor_z = ground_z;
  viz.plane = std::make_pair(c(0), c(1));

  // (a) observed: share of 10 cm cells of the disc that hold ground returns
  int n = int(std::ceil(radius / cell));
  std::set<std::pair<long, long>> cells;
  for (size_t q : gi)
    cells.insert({long(std::nearbyint((patch[q].x - sx) / cell)), long(std::nearbyint((patch[q].y - sy) / cell))});
  std::vector<std::pair<int, int>> want, miss;
  for (int j = 0; j < 2 * n + 1; ++j)
    for (int i = 0; i < 2 * n + 1; ++i) {
      double gx = (i - n) * cell, gy = (j - n) * cell;
      if (std::hypot(gx, gy) <= radius - cell / 2) want.push_back({i - n, j - n});
    }
  for (auto& w : want)
    if (!cells.count({w.first, w.second})) miss.push_back(w);
  if (!known.empty() && !miss.empty()) {
    // cells under a footprint where the robot sat: floor its own body hid
    std::vector<std::pair<int, int>> still;
    int under = 0;
    for (auto& w : miss) {
      V2 xy(sx + w.first * cell, sy + w.second * cell);
      bool u = false;
      for (auto& k : known) u = u || in_footprint(xy, V2(k.x, k.y), k.yaw);
      if (u) {
        viz.known_cells.push_back(xy);
        under++;
      } else {
        still.push_back(w);
      }
    }
    st["observed_measured"] = r_(1.0 - double(miss.size()) / std::max<size_t>(1, want.size()), 3);
    st["known_floor_cells"] = under;
    miss = still;
  }
  double observed = 1.0 - double(miss.size()) / std::max<size_t>(1, want.size());
  for (auto& w : miss) viz.unobserved.push_back(V2(sx + w.first * cell, sy + w.second * cell));
  st["observed"] = r_(observed, 3);
  if (observed < p.observed_min) {
    st["reason"] = fmt("not observed: %.0f%% of the patch has ground returns", 100 * observed);
    return {false, st};
  }

  // (b) flat
  if (slope > slope_max) {
    st["reason"] = fmt("not flat: slope %.1f deg", tc::degrees(slope));
    return {false, st};
  }
  if (resid > p.residual_max) {
    st["reason"] = fmt("not flat: ground residual %.3f m", resid);
    return {false, st};
  }

  // (c) obstacles above the plane, up to the descent height (covers overhead obstacles too)
  std::vector<double> h(N);
  for (size_t q = 0; q < N; ++q) h[q] = patch[q].z - ((patch[q].x - sx) * c(0) + (patch[q].y - sy) * c(1) + c(2));
  // the floor layer per 10 cm cell (90th percentile of the points below 0.3 m), its median over the
  // cells: a box covers a few cells and cannot raise it, an evenly thick floor does
  std::map<int64_t, std::vector<size_t>> by_cell;
  for (size_t q = 0; q < N; ++q) by_cell[kk[q]].push_back(q);
  std::vector<double> tops;
  for (auto& kv : by_cell) {
    std::vector<double> hc;
    for (size_t q : kv.second)
      if (h[q] < 0.3) hc.push_back(h[q]);
    if (hc.size() >= 3) tops.push_back(tc::percentile(hc, 90));
  }
  double layer = tops.empty() ? 0.0 : tc::median(tops);
  st["floor_layer_m"] = r_(layer, 3);
  if (layer > p.floor_layer_max) {
    st["reason"] = fmt("uncertain floor: layer %.2f m thick", layer);
    return {false, st};
  }
  double thr = std::max(p.obstacle_min_h, layer + p.obstacle_min_h);
  st["obstacle_threshold_m"] = r_(thr, 3);
  double top = descend_from ? *descend_from - ground_z : p.descent_top;
  // 4. per cell, the latest evidence_last scans that returned anything there below the top
  std::vector<int64_t> occ_cells;
  int tall = 0, hit_cells = 0, occ_points = 0;
  double max_h = -1e300;
  Json oxy = Json::array();
  for (auto& kv : by_cell) {
    bool any_hit = false;
    for (size_t q : kv.second) any_hit = any_hit || (h[q] > thr && h[q] < top);
    if (!any_hit) continue;
    hit_cells++;
    std::set<int> seen_set;
    for (size_t q : kv.second)
      if (h[q] < top) seen_set.insert(sid[q]);
    std::vector<int> seen(seen_set.begin(), seen_set.end());
    if (int(seen.size()) > p.evidence_last) seen.erase(seen.begin(), seen.end() - p.evidence_last);
    std::set<int> hit_s;
    int hc = 0, tall_c = 0;
    for (size_t q : kv.second)
      if (h[q] < top && h[q] > thr && sid[q] >= seen[0]) {
        hit_s.insert(sid[q]);
        hc++;
        tall_c += h[q] >= 0.3;
      }
    bool latest = int(seen.size()) >= p.evidence_latest;
    for (int k = int(seen.size()) - p.evidence_latest; latest && k < int(seen.size()); ++k)
      latest = hit_s.count(seen[k]) > 0;
    if ((int(hit_s.size()) >= int(std::ceil(p.evidence_frac * seen.size())) || latest) && hc >= 2) {
      occ_cells.push_back(kv.first);
      tall += tall_c;
      double cm = -1e300, mx = 0, my = 0;
      int cn = 0;
      for (size_t q : kv.second)
        if (h[q] > thr && h[q] < top) {
          cm = std::max(cm, h[q]);
          mx += patch[q].x;
          my += patch[q].y;
          cn++;
        }
      occ_points += cn;
      max_h = std::max(max_h, cm);
      int64_t c0 = kv.first;
      viz.occupied.push_back({sx + (double(c0 / 200001) - 100000 + 0.5) * cell,
                              sy + (double(c0 % 200001) - 100000 + 0.5) * cell, cm});
      if (oxy.size() < 12) oxy.push_back({r_(mx / cn, 2), r_(my / cn, 2), r_(cm, 2)});
    }
  }
  int n_cells = int(occ_cells.size());
  st["obstacle_points"] = occ_points;
  st["obstacle_cells"] = n_cells;
  st["tall_points"] = tall;
  st["hit_cells"] = hit_cells;
  st["max_obstacle_h"] = occ_points ? r_(max_h, 3) : 0.0;
  st["obstacle_xy"] = oxy;
  if (n_cells >= p.obstacle_min_cells || tall >= p.obstacle_min_points) {
    st["reason"] = fmt("occupied: %d cells, %d points up to %.2f m above the floor", n_cells, occ_points,
                       st["max_obstacle_h"].get<double>());
    return {false, st};
  }
  st["reason"] = "clear";
  return {true, st};
}

}  // namespace l4_landing
