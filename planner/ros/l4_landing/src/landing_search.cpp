// Landing-spot search (owner request 2026-10-30), ROS-free.
//
// In the air, about to land to drive (a drive click from a hover, a mission drive leg that starts in the air),
// the executive first asks the landing check about the spot under the vehicle. When that is refused it asks
// for a search: candidate spots on a `step` grid centred on the vehicle, within `radius`, each evaluated by
// the landing check itself (LandingCheck::evaluate_scans: the same disc on the frame, the same observed /
// flat / free rules and thresholds, the same scans and known floor). Nothing about the check is loosened: a
// candidate is a spot the check would report clear if the vehicle hovered over it now.
//
// Each candidate's heading is the one it would land with: facing the drive goal from the candidate (the
// executive's waypoint yaw over the spot), or the vehicle's yaw when the goal is nearer than
// heading_min_dist. The disc follows the heading (LandingCheck::centre), so the heading matters.
//
// Cost (lower is better): the distance from the vehicle (a short move) plus goal_weight times how much
// farther from the goal the candidate is than the vehicle (negative toward the goal). With goal_weight 0.5
// a spot 1 m toward the goal costs 0.5, one 1 m away from it 1.5. The tie-break is the margin: among clear
// candidates within tie_m of the best cost, the one farthest from any evaluated not-clear candidate (the
// spot with the most clear floor round it), capped at margin_max.
//
// Evaluation order and the budget. The candidates on the coarse grid (every other step, 0.4 m) are evaluated
// first, in cost order, then the rest in cost order, so a budget that runs out still has covered the whole
// radius (SITL run 4: in plain cost order the 4 s budget never reached past ~0.9 m). Once one is clear, only the
// candidates that can still matter are evaluated: a candidate within tie_m of the best cost, or one within
// margin_max of such a candidate (its margin's neighbours). |cost(a) - cost(b)| <= (1 + goal_weight) |a - b|,
// so everything costing more than best + tie_m + (1 + goal_weight) margin_max is skipped. budget_s caps the
// time (6 s): candidates not evaluated by then stay unevaluated (grey in the markers) and the best among the
// evaluated ones is taken. Each evaluation gets the scans' points from the 0.5 m buckets that reach its own crop
// box (the check crops exactly), not the whole search area.
//
// The scans are cropped once to the search area and their near-range returns (closer than the check's
// near_range to the sensor) dropped once, which is what each evaluation would do on its own: the result is
// the same as calling evaluate_scans on the raw window, only faster.
#include "l4_landing/landing_search.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>

namespace l4_landing {

namespace tc = l4_track;

namespace {
double steady() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

std::string reason_word(const std::string& reason) {
  for (const char* w : {"clear", "occupied", "not observed", "not flat", "uncertain floor", "no ground plane",
                        "no heading", "excluded"})
    if (reason.rfind(w, 0) == 0) return w;
  return reason.empty() ? "not evaluated" : reason.substr(0, reason.find(':'));
}

SearchResult search_landing_spot(LandingCheck& check, const std::vector<Scan>& scans, const V2& vehicle, double vehicle_yaw,
                                 std::optional<V2> goal, std::optional<double> descend_from,
                                 const std::vector<KnownPose>& known, const SearchParams& sp,
                                 const std::vector<V2>& exclude, std::function<double()> clock) {
  if (!clock) clock = steady;
  const double t0 = clock();
  SearchResult R;
  // the candidates: the grid within the radius, each with its landing heading and cost
  const double step = std::max(0.05, sp.step);
  const int n = int(std::floor(sp.radius / step + 1e-9));
  const double gv = goal ? (*goal - vehicle).norm() : 0.0;
  for (int j = -n; j <= n; ++j)
    for (int i = -n; i <= n; ++i) {
      Candidate c;
      c.spot = vehicle + V2(i * step, j * step);
      c.dist = (c.spot - vehicle).norm();
      c.coarse = i % 2 == 0 && j % 2 == 0;
      if (c.dist > sp.radius + 1e-9) continue;
      c.yaw = vehicle_yaw;
      c.cost = c.dist;
      if (goal) {
        const V2 d = *goal - c.spot;
        if (d.norm() >= sp.heading_min_dist) c.yaw = std::atan2(d.y(), d.x());
        c.cost += sp.goal_weight * (d.norm() - gv);
      }
      for (const auto& e : exclude) c.excluded = c.excluded || (c.spot - e).norm() <= sp.exclude_r;
      if (c.excluded) c.reason = "excluded";
      R.cands.push_back(c);
    }
  std::stable_sort(R.cands.begin(), R.cands.end(), [](const Candidate& a, const Candidate& b) {
    return a.cost < b.cost || (a.cost == b.cost && a.dist < b.dist);
  });

  // the window cropped to the search area once, the near-range returns dropped once (as each evaluation would)
  const double half = sp.radius + check.p.footprint_offset + check.align_radius + check.gcfg.tile + 0.05;
  std::vector<Scan> crop;
  crop.reserve(scans.size());
  for (const auto& S : scans) {
    Scan c;
    for (const auto& q : S.pts) {
      if (!(std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z))) continue;
      if (!(std::abs(q.x - vehicle.x()) <= half && std::abs(q.y - vehicle.y()) <= half)) continue;
      if (S.sensor && check.p.near_range > 0) {
        Eigen::Vector3d d(q.x - S.sensor->x(), q.y - S.sensor->y(), q.z - S.sensor->z());
        if (!(d.norm() >= check.p.near_range)) continue;
      }
      c.pts.push_back(q);
    }
    crop.push_back(std::move(c));
  }
  // each scan's points in B m buckets over the search area: a candidate takes the buckets that reach its
  // evaluation box (the check crops to the box itself), not the whole area
  const double B = 0.5;
  const int nb = int(std::ceil(2 * half / B)) + 1;
  const V2 lo = vehicle - V2(half, half);
  std::vector<std::vector<tc::Cloud>> buckets(crop.size(), std::vector<tc::Cloud>(size_t(nb) * nb));
  for (size_t k = 0; k < crop.size(); ++k)
    for (const auto& q : crop[k].pts) {
      const int bx = std::min(nb - 1, std::max(0, int((q.x - lo.x()) / B)));
      const int by = std::min(nb - 1, std::max(0, int((q.y - lo.y()) / B)));
      buckets[k][size_t(bx) * nb + by].push_back(q);
    }
  const double box = check.align_radius + check.gcfg.tile;   // LandingCheck::evaluate_scans' crop
  auto window_for = [&](const Candidate& c) {
    const V2 ctr = check.centre(c.spot, c.yaw);
    const int x0 = std::max(0, int((ctr.x() - box - lo.x()) / B)), x1 = std::min(nb - 1, int((ctr.x() + box - lo.x()) / B));
    const int y0 = std::max(0, int((ctr.y() - box - lo.y()) / B)), y1 = std::min(nb - 1, int((ctr.y() + box - lo.y()) / B));
    std::vector<Scan> w(crop.size());
    for (size_t k = 0; k < crop.size(); ++k)
      for (int bx = x0; bx <= x1; ++bx)
        for (int by = y0; by <= y1; ++by) {
          const auto& b = buckets[k][size_t(bx) * nb + by];
          w[k].pts.insert(w[k].pts.end(), b.begin(), b.end());
        }
    return w;
  };

  // evaluate the coarse grid in cost order, then the rest in cost order; once one is clear, only what can still
  // change the choice
  std::optional<double> best_cost;
  const double reach = sp.tie_m + (1.0 + std::abs(sp.goal_weight)) * sp.margin_max;
  std::vector<size_t> order;
  for (int pass = 0; pass < 2; ++pass)
    for (size_t k = 0; k < R.cands.size(); ++k)
      if (R.cands[k].coarse == (pass == 0)) order.push_back(k);
  for (size_t k : order) {
    auto& c = R.cands[k];
    if (c.excluded) continue;
    if (best_cost && c.cost > *best_cost + reach) continue;
    if (clock() - t0 > sp.budget_s) {
      R.budget_hit = true;
      break;
    }
    auto [ok, st] = check.evaluate_scans(window_for(c), c.spot, descend_from, c.yaw, known);
    c.evaluated = true;
    c.clear = ok;
    c.reason = st.value("reason", "");
    if (check.viz.floor_z) c.floor_z = *check.viz.floor_z;
    c.status = st;
    R.evaluated += 1;
    if (ok && (!best_cost || c.cost < *best_cost)) best_cost = c.cost;
  }

  // margins, then the choice: the best cost, the most margin within tie_m of it
  for (auto& c : R.cands) {
    if (!c.clear) continue;
    c.margin = sp.margin_max;
    for (const auto& o : R.cands)
      if (o.evaluated && !o.clear) c.margin = std::min(c.margin, (o.spot - c.spot).norm());
  }
  if (best_cost) {
    for (size_t k = 0; k < R.cands.size(); ++k) {
      const auto& c = R.cands[k];
      if (!c.clear || c.cost > *best_cost + sp.tie_m + 1e-9) continue;
      if (!R.best || c.margin > R.cands[*R.best].margin + 1e-9) R.best = k;
    }
  }
  R.elapsed_s = clock() - t0;

  Json s;
  s["found"] = R.best.has_value();
  s["candidates"] = int(R.cands.size());
  s["evaluated"] = R.evaluated;
  std::map<std::string, int> words;
  for (const auto& c : R.cands)
    if (c.evaluated || c.excluded) words[reason_word(c.reason)] += 1;
  Json w = Json::object();
  for (const auto& kv : words) w[kv.first] = kv.second;
  s["reasons"] = w;
  // every evaluated candidate, compact: [x, y, reason word, its number] (occupied: the obstacle height; not
  // observed: the observed share; not flat: the residual; uncertain floor: the layer; clear: the margin)
  Json ev = Json::array();
  for (const auto& c : R.cands) {
    if (!c.evaluated) continue;
    const std::string w = reason_word(c.reason);
    const char* key = w == "occupied" ? "max_obstacle_h" : w == "not observed" ? "observed" : w == "not flat" ? "residual_m"
                    : w == "uncertain floor" ? "floor_layer_m" : nullptr;
    Json v = nullptr;
    if (c.clear) v = tc::pyround(c.margin, 2);
    else if (key && c.status.contains(key) && c.status[key].is_number()) v = c.status[key];
    ev.push_back({tc::pyround(c.spot.x(), 2), tc::pyround(c.spot.y(), 2), w, v});
  }
  s["spots"] = ev;
  s["budget_hit"] = R.budget_hit;
  s["elapsed_s"] = tc::pyround(R.elapsed_s, 3);
  if (R.best) {
    const auto& b = R.cands[*R.best];
    s["spot"] = {tc::pyround(b.spot.x(), 3), tc::pyround(b.spot.y(), 3)};
    s["yaw_deg"] = tc::pyround(tc::degrees(b.yaw), 1);
    s["dist_m"] = tc::pyround(b.dist, 3);
    s["cost"] = tc::pyround(b.cost, 3);
    s["margin_m"] = tc::pyround(b.margin, 3);
    s["check"] = b.status;
  } else {
    // the cheapest evaluated candidate's reason (the counts say what the search mostly saw)
    for (const auto& c : R.cands)
      if (c.evaluated) {
        s["first_reason"] = c.reason;
        break;
      }
  }
  R.summary = s;
  return R;
}

}  // namespace l4_landing
