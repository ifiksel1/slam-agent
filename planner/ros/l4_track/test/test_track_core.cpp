// Track planner tests (gtest): the synthetic cases of the former planner/test/test_track_planner.py,
// one TEST per Python check (35), same scenes and thresholds. The real-map gate is the track_gate tool.
//
// The drive chains model the FC (Rover's position, the point Rover steers and turns about) at the
// frame centre, footprint_offset behind the odometry point (owner 2026-10-29: the FC sits at the
// frame centre; the Python tests had it 0.178 m behind the odometry point).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>

#include "l4_track/track_core.h"

namespace tc = l4_track;
using tc::Cloud;
using tc::Path;
using tc::V2;

namespace {
std::mt19937 RNG(0);
const double FCB = tc::ODOM_AHEAD;   // the FC behind the odometry point (at the frame centre)

Cloud floor_(double x0, double x1, double y0, double y1, double thick = 0.06, double step = 0.05) {
  std::uniform_real_distribution<double> U(-0.02, thick - 0.02);
  Cloud out;
  for (double y : tc::arange(y0, y1, step))
    for (double x : tc::arange(x0, x1, step)) out.push_back({x, y, U(RNG)});
  return out;
}

Cloud wall(double x0, double y0, double x1, double y1, double z0 = 0.0, double z1 = 1.0, double step = 0.03) {
  int n = std::max(2, int(std::hypot(x1 - x0, y1 - y0) / step));
  auto xs = tc::linspace(x0, x1, n), ys = tc::linspace(y0, y1, n);
  Cloud out;
  for (double z : tc::arange(z0, z1, 0.05))
    for (int i = 0; i < n; ++i) out.push_back({xs[i], ys[i], z});
  return out;
}

Cloud cat(std::initializer_list<Cloud> cs) {
  Cloud out;
  for (auto& c : cs) out.insert(out.end(), c.begin(), c.end());
  return out;
}

std::vector<V2> band_xy(const Cloud& pts, const tc::Config& cfg, double floor_z = 0.0) {
  std::vector<V2> out;
  for (auto& p : pts)
    if (p.z - floor_z > cfg.z_lo && p.z - floor_z < cfg.z_hi) out.push_back(V2(p.x, p.y));
  return out;
}

// the bare frame (no margin) at odometry pose (p, yaw) against obstacle points -> worst intrusion
double intrusion(const V2& p, double yaw, const std::vector<V2>& obs, double off) {
  double c = std::cos(yaw), s = std::sin(yaw), worst = 0.0;
  V2 f = p - off * V2(c, s);
  for (auto& o : obs) {
    double dx = o.x() - f.x(), dy = o.y() - f.y();
    double lon = std::abs(dx * c + dy * s), lat = std::abs(-dx * s + dy * c);
    if (lon < tc::FOOTPRINT_LEN / 2 && lat < tc::FOOTPRINT_WID / 2)
      worst = std::max(worst, std::min(tc::FOOTPRINT_LEN / 2 - lon, tc::FOOTPRINT_WID / 2 - lat));
  }
  return worst;
}

// independent check: the bare frame, centre footprint_offset behind each follower pose, heading to the
// point lookahead ahead, holds no obstacle point -> (ok, worst intrusion m)
std::pair<bool, double> frame_clear(const Path& path, const std::vector<V2>& obs, const tc::Config& cfg) {
  Path P = tc::densify(path, 0.02);
  std::vector<double> cum{0.0};
  for (size_t k = 1; k < P.size(); ++k) cum.push_back(cum.back() + tc::norm2(P[k] - P[k - 1]));
  double worst = 0.0;
  for (size_t k = 0; k < P.size(); ++k) {
    if (tc::norm2(P[k] - P[0]) < cfg.start_skip) continue;
    size_t qi = std::lower_bound(cum.begin(), cum.end(), cum[k] + cfg.lookahead) - cum.begin();
    V2 q = P[std::min(qi, P.size() - 1)];
    V2 d = tc::norm2(q - P[k]) > 1e-6 ? V2(q - P[k]) : V2(P.back() - P[P.size() - 2]);
    worst = std::max(worst, intrusion(P[k], std::atan2(d.y(), d.x()), obs, cfg.off));
  }
  return {worst == 0.0, worst};
}

// no obstacle point within the bare frame's half-diagonal of a pivot centre -> (ok, nearest m)
std::pair<bool, double> disc_clear(const V2& c, const std::vector<V2>& obs, double r = tc::FOOTPRINT_HALF_DIAGONAL) {
  double d = 1e3;
  for (auto& o : obs) d = std::min(d, std::hypot(o.x() - c.x(), o.y() - c.y()));
  return {d >= r, d};
}

struct Legs {
  bool ok;
  double worst, near;
};
// each driving leg on its own (frame_clear), each pivot circle (disc_clear)
Legs legs_clear(const tc::PlanResult& out, const std::vector<V2>& obs, tc::Config cfg) {
  std::vector<int> starts{0}, ends;
  for (auto& p : out.pivots) {
    starts.push_back(p.i + 1);
    ends.push_back(p.i);
  }
  ends.push_back(int(out.path.size()) - 1);
  Legs L{true, 0.0, 1e3};
  for (size_t k = 0; k < starts.size(); ++k) {
    int a = starts[k], b = ends[k];
    if (b - a < 1) continue;
    Path leg(out.path.begin() + a, out.path.begin() + b + 1);
    tc::Config c = cfg;
    if (k > 0) c.start_skip = 0.0;   // the start skip of frame_clear is for the plan's start only
    auto [o, w] = frame_clear(leg, obs, c);
    L.ok = L.ok && o;
    L.worst = std::max(L.worst, w);
  }
  for (auto& p : out.pivots) {
    auto [o, d] = disc_clear(p.centre, obs);
    L.ok = L.ok && o;
    L.near = std::min(L.near, d);
  }
  return L;
}

// the planner bridge's drive target (bridge_core.drive_target, zero velocity)
V2 drive_target(const V2& c, const V2& now, double lookahead) {
  V2 d = c - now;
  double n = d.norm();
  if (n < 1e-6) return now;
  return now + d / n * std::min(lookahead, n);
}
double wrap_pi(double a) { return std::atan2(std::sin(a), std::cos(a)); }

struct Chain {
  bool clear, reached;
  double worst, t;
};
// replan every second, carrot, the bridge's target from the FC, a skid-steer driving at it
Chain drive_chain(const tc::Config& cfg, const Cloud& obs_pts, const Cloud& cloud, V2 odom, V2 goal,
                  double fc_behind = FCB, double lookahead = 0.5, double speed = 0.5, double turn_rate = 1.0,
                  double dt = 0.1) {
  auto obs = band_xy(obs_pts, cfg);
  auto hg = std::make_shared<tc::HeightGrid>(cloud, std::make_shared<const tc::Config>(cfg));
  double yaw = 0.0, worst = 0.0, t = 0.0;
  Path path;
  bool have = false;
  while (t < 120.0) {
    if (!have || int64_t(std::nearbyint(t / dt)) % 10 == 0) {
      auto out = tc::plan(hg, odom, goal);
      if (!out.ok) return {false, false, worst, t};
      path = out.path;
      have = true;
    }
    if ((odom - goal).norm() < 0.3) return {worst == 0.0, true, worst, t};
    V2 c = tc::carrot(path, odom, cfg.lookahead).first;
    V2 fc = odom - fc_behind * tc::unit(yaw);
    V2 d = drive_target(c, fc, lookahead) - fc;
    double err = wrap_pi(std::atan2(d.y(), d.x()) - yaw);
    yaw += std::clamp(err, -turn_rate * dt, turn_rate * dt);
    if (std::abs(err) < tc::radians(60)) fc += std::min(speed, d.norm() / dt) * dt * tc::unit(yaw);
    odom = fc + fc_behind * tc::unit(yaw);
    worst = std::max(worst, intrusion(odom, yaw, obs, cfg.off));
    t += dt;
  }
  return {worst == 0.0, false, worst, t};
}

struct ChainP {
  bool clear, reached;
  double worst, t;
  std::vector<double> seen;   // nearest obstacle to the frame centre at each pivot driven
};
// the drive leg with pivots: replan every second with the heading, follow_plan, the bridge's target,
// Rover Guided as a skid-steer about the FC (pivot over 60 deg until within 5, stop within wp_radius)
ChainP drive_chain_pivots(const tc::Config& cfg, const Cloud& obs_pts, const Cloud& cloud, V2 odom, double yaw, V2 goal,
                          double fc_behind = FCB, double lookahead = 0.5, double speed = 0.5, double turn_rate = 1.0,
                          double dt = 0.1, double wp_radius = 0.3, double lead = 0.15) {
  auto obs = band_xy(obs_pts, cfg);
  auto hg = std::make_shared<tc::HeightGrid>(cloud, std::make_shared<const tc::Config>(cfg));
  double worst = 0.0, t = 0.0;
  std::string phase = "drive";
  bool piv = false, have = false;
  tc::PlanResult out;
  std::vector<double> seen;
  while (t < 180.0) {
    if (!have || int64_t(std::nearbyint(t / dt)) % 10 == 0) {
      auto nw = tc::plan(hg, odom, goal, yaw, phase == "pivot");
      if (!nw.ok) {
        printf("  [chain] replan failed at odom (%.3f, %.3f) yaw %.4f pivoting %d: %s\n", odom.x(), odom.y(), yaw, phase == "pivot", nw.reason.c_str());
        return {false, false, worst, t, seen};
      }
      if (phase == "pivot" && have) {
        // as the node: a pivot under way finishes on its route while that still checks
        Path rest = tc::remaining(out.route, odom);
        auto kept = tc::check_route(*nw.grid, rest, yaw, true);
        if (kept.ok) {
          nw.path = kept.path;
          nw.pivots = kept.pivots;
          nw.route = rest;
        }
      }
      out = nw;
      have = true;
    }
    if ((odom - goal).norm() < 0.3) return {worst == 0.0, true, worst, t, seen};
    auto f = tc::follow_plan(out.path, out.pivots, odom, yaw, cfg, lead);
    phase = f.phase;
    V2 h = tc::unit(yaw);
    V2 fc = odom - fc_behind * h;
    V2 d = drive_target(f.c, fc, lookahead) - fc;
    double err = d.norm() > 1e-6 ? wrap_pi(std::atan2(d.y(), d.x()) - yaw) : 0.0;
    if (!piv && std::abs(err) > tc::radians(60)) {
      piv = true;
      seen.push_back(disc_clear(odom - cfg.off * h, obs).second);
    } else if (piv && std::abs(err) < tc::radians(5)) {
      piv = false;
    }
    yaw += std::clamp(err, -turn_rate * dt, turn_rate * dt);
    if (!piv && d.norm() >= wp_radius) fc += std::min(speed, d.norm() / dt) * dt * tc::unit(yaw);
    odom = fc + fc_behind * tc::unit(yaw);
    worst = std::max(worst, intrusion(odom, yaw, obs, cfg.off));
    t += dt;
  }
  return {worst == 0.0, false, worst, t, seen};
}

struct ChainR {
  bool reached;
  double worst, vmax, t;
  V2 final_;
  double v;
  std::string why;
};
// the drive leg with Rover Guided's fast-update speed law (see the Python docstring): the track
// planner (replans every second, follow_plan, run_out), the bridge (drive_target), a skid-steer about
// the FC steering by pure pursuit on its target at min(WP_SPEED, kv x distance)
ChainR drive_chain_rover(const tc::Config& cfg, const Cloud& obs_pts, const Cloud& cloud, V2 odom, double yaw, V2 goal,
                         double runout = 2.1, double wp_speed = 1.0, double kv = 0.5, double accel = 1.0,
                         double fc_behind = FCB, double lookahead = 2.5, double turn_rate = 1.0, double dt = 0.05,
                         double wp_radius = 0.3, double lead = 0.15, double settle = 4.0,
                         double steer_dist = 0.0, int* pivots_driven = nullptr) {
  auto obs = band_xy(obs_pts, cfg);
  auto hg = std::make_shared<tc::HeightGrid>(cloud, std::make_shared<const tc::Config>(cfg));
  double worst = 0, t = 0, v = 0, vmax = 0;
  std::optional<double> t_reached;
  std::string phase = "drive";
  bool piv = false, have = false;
  tc::PlanResult out;
  V2 tgt(0, 0);
  long k = 0;
  while (t < 150.0) {
    V2 h = tc::unit(yaw);
    V2 fc = odom - fc_behind * h;
    if (!t_reached) {
      if ((odom - goal).norm() < 0.3) {
        t_reached = t;
      } else {
        if (!have || k % long(std::nearbyint(1.0 / dt)) == 0) {
          auto nw = tc::plan(hg, odom, goal, yaw, phase == "pivot");
          if (!nw.ok) {
            printf("  [chain] replan failed at odom (%.3f, %.3f) yaw %.4f pivoting %d: %s\n", odom.x(), odom.y(), yaw, phase == "pivot", nw.reason.c_str());
            return {false, worst, vmax, t, odom, v, nw.reason};
          }
          if (phase == "pivot" && have) {
            Path rest = tc::remaining(out.route, odom);
            auto kept = tc::check_route(*nw.grid, rest, yaw, true);
            if (kept.ok) {
              nw.path = kept.path;
              nw.pivots = kept.pivots;
              nw.route = rest;
            }
          }
          out = nw;
          have = true;
        }
        auto f = tc::follow_plan(out.path, out.pivots, odom, yaw, cfg, lead);
        phase = f.phase;
        V2 c = tc::run_out(out.path, out.pivots, odom, yaw, f.c, f.phase, f.j, cfg, runout, fc_behind, lead, out.grid.get())
                   .first;
        tgt = drive_target(c, fc, lookahead);
      }
    } else if (t - *t_reached > settle) {
      break;
    }
    V2 d = tgt - fc;
    double dist = d.norm();
    double err = dist > 1e-6 ? wrap_pi(std::atan2(d.y(), d.x()) - yaw) : 0.0;
    if (!piv && std::abs(err) > tc::radians(60) && dist >= wp_radius) {
      piv = true;
      if (pivots_driven) ++*pivots_driven;
    }
    else if (piv && std::abs(err) < tc::radians(5))
      piv = false;
    double v_des = (piv || dist < wp_radius) ? 0.0 : std::min(wp_speed, kv * dist);
    v += std::clamp(v_des - v, -accel * dt, accel * dt);
    if (piv && std::abs(v) < 0.05) {
      yaw += std::clamp(err, -turn_rate * dt, turn_rate * dt);
    } else if (!piv && dist >= wp_radius) {
      // pure pursuit on the target; steer_dist: at least this far (SITL's heading lag, fit to c1/c3)
      double w = 2.0 * std::max(std::abs(v), 0.1) * std::sin(err) / std::max(dist, steer_dist);
      yaw += std::clamp(w * dt, -turn_rate * dt, turn_rate * dt);
    }
    fc += v * dt * tc::unit(yaw);
    odom = fc + fc_behind * tc::unit(yaw);
    vmax = std::max(vmax, std::abs(v));
    worst = std::max(worst, intrusion(odom, yaw, obs, cfg.off));
    t += dt;
    k++;
  }
  return {bool(t_reached), worst, vmax, t_reached ? *t_reached : t, odom, v, ""};
}

const tc::Config CFG;
const double E = 0.025;   // walls and points 0.025 m off the 0.1 m cell boundaries
Cloud w(double x0, double y0, double x1, double y1) { return wall(x0 + E, y0 + E, x1 + E, y1 + E); }
V2 at(double x, double y) { return V2(x + E, y + E); }

// a wall across the middle with a 1.6 m gap at the top, on a floor
Cloud WALLS() { return cat({wall(0, -5, 0, 1.2), wall(-6, -5, 6, -5), wall(-6, 3.0, 6, 3.0)}); }
}  // namespace

TEST(Synthetic, AroundAWallThroughTheGap) {
  Cloud walls = WALLS(), room = cat({floor_(-6, 6, -5, 3), walls});
  auto out = tc::plan(room, std::nullopt, V2(-3, 0), V2(3, 0), CFG);
  EXPECT_TRUE(out.ok && out.length > 6.0) << "length " << out.length << " " << out.reason;
}

TEST(Synthetic, BareFrameNeverTouchesTheWall) {
  Cloud walls = WALLS(), room = cat({floor_(-6, 6, -5, 3), walls});
  auto out = tc::plan(room, std::nullopt, V2(-3, 0), V2(3, 0), CFG);
  ASSERT_TRUE(out.ok);
  auto [ok, worst] = frame_clear(out.path, band_xy(walls, CFG), CFG);
  EXPECT_TRUE(ok) << "worst intrusion " << worst;
}

TEST(Synthetic, FloorSmearAndHighShelfDoNotBlock) {
  auto out = tc::plan(cat({floor_(-3, 3, -2, 2, 0.25), wall(-3, 0.0, 3, 0.0, 1.4, 1.6)}), std::nullopt, V2(-2, -1),
                      V2(2, 1), CFG);
  EXPECT_TRUE(out.ok && out.length < 4.7) << out.length;
}

TEST(Synthetic, EnclosedStartReportsNoPath) {
  Cloud box = cat({floor_(-3, 6, -3, 3), wall(-1, -1, 1, -1), wall(1, -1, 1, 1), wall(1, 1, -1, 1), wall(-1, 1, -1, -1)});
  EXPECT_FALSE(tc::plan(box, std::nullopt, V2(0, 0), V2(4, 0), CFG).ok);
}

namespace {
Cloud door(double wd) {
  double h = wd / 2;
  return cat({floor_(-4, 4, -4, 4), wall(0, -4, 0, -h), wall(0, h, 0, 4), wall(-4, -4, 4, -4), wall(-4, 4, 4, 4)});
}
}  // namespace

TEST(Synthetic, Doorway050Refused) {
  EXPECT_FALSE(tc::plan(door(0.50), std::nullopt, V2(-2, 0), V2(2, 0), CFG).ok);
}

TEST(Synthetic, Doorway070PassedStraightThrough) {
  Cloud d = door(0.70);
  auto out = tc::plan(d, std::nullopt, V2(-2, 0), V2(2, 0), CFG);
  ASSERT_TRUE(out.ok) << out.reason;
  auto [ok, worst] = frame_clear(out.path, band_xy(d, CFG), CFG);
  EXPECT_TRUE(ok) << worst;
}

TEST(Synthetic, Doorway080PassedStraightThrough) {
  Cloud d = door(0.80);
  auto out = tc::plan(d, std::nullopt, V2(-2, 0), V2(2, 0), CFG);
  ASSERT_TRUE(out.ok) << out.reason;
  auto [ok, worst] = frame_clear(out.path, band_xy(d, CFG), CFG);
  EXPECT_TRUE(ok) << worst;
}

TEST(Synthetic, CorridorFrameFreeAlongBlockedAcross) {
  Cloud corr = cat({floor_(-3, 3, -2, 2), wall(-3, -0.35, 3, -0.35), wall(-3, 0.35, 3, 0.35)});
  tc::HeightGrid hg(corr, std::make_shared<const tc::Config>(CFG));
  auto win = hg.crop(V2(-2, -1), V2(2, 1));
  EXPECT_TRUE(win->pose_free(V2(0, 0), 0.0) && win->pose_free(V2(0, 0), M_PI) && !win->pose_free(V2(0, 0), M_PI / 2));
}

TEST(Synthetic, MovedPersonClearedWallBehindKept) {
  auto lcfg = std::make_shared<const tc::Config>(tc::live_config());
  Cloud back = wall(-1, 3, 1, 3), person = wall(-0.1, 1.5, 0.1, 1.5, 0.0, 1.7), fl = floor_(-3, 3, -1, 4);
  auto hg = std::make_shared<tc::HeightGrid>(cat({fl, back, person}), lcfg, V2(-3, -1), V2(3, 4));
  auto blocked = tc::plan(hg, V2(0, 0), V2(0, 2.5));
  std::vector<int> hits, miss;
  tc::evidence(*hg, {{cat({fl, back}), Eigen::Vector3d(0, 0, 0.25)}}, hits, miss);
  int dropped = hg->apply_evidence(hits, miss);
  auto out = tc::plan(hg, V2(0, 0), V2(0, 2.5));
  bool back_kept = true;
  for (double x : {-0.5, 0.0, 0.5}) {
    auto c = hg->cell(V2(x, 3.0));
    back_kept = back_kept && hg->occ[hg->idx(c.first, c.second)];
  }
  EXPECT_TRUE(dropped > 0 && out.ok && out.length < 2.6 && back_kept && (!blocked.ok || blocked.length > 2.6))
      << "dropped " << dropped << " after " << out.length << " before " << blocked.length;
}

TEST(Synthetic, LowLidarClearsPersonFloorRaysDoNotClearWall) {
  auto lcfg = std::make_shared<const tc::Config>(tc::live_config());
  Cloud back = wall(-1, 3, 1, 3), person = wall(-0.1, 1.5, 0.1, 1.5, 0.0, 1.7), fl = floor_(-3, 3, -1, 4);
  tc::HeightGrid hg(cat({fl, back, person}), lcfg, V2(-3, -1), V2(3, 4));
  std::vector<int> hits, miss;
  tc::evidence(hg, {{cat({fl, back}), Eigen::Vector3d(0, 0, 0.08)}}, hits, miss, true);
  int dropped_low = hg.apply_evidence(hits, miss);
  auto pc = hg.cell(V2(0.0, 1.5));
  bool person_gone = !hg.occ[hg.idx(pc.first, pc.second)];
  Cloud shelf = cat({floor_(-3, 3, -1, 5), wall(-1, 2.0, 1, 2.0, 0.2, 1.0)});   // a wall standing on legs
  tc::HeightGrid hs(shelf, lcfg, V2(-3, -1), V2(3, 5));
  tc::evidence(hs, {{floor_(-3, 3, -1, 5), Eigen::Vector3d(0, 0, 0.08)}}, hits, miss, true);   // a later scan: floor only
  hs.apply_evidence(hits, miss);
  bool kept = true;
  for (double x : {-0.5, 0.0, 0.5}) {
    auto c = hs.cell(V2(x, 2.0));
    kept = kept && hs.occ[hs.idx(c.first, c.second)];
  }
  EXPECT_TRUE(person_gone && dropped_low > 0 && kept) << person_gone << " " << kept;
}

namespace {
Cloud ceiling_scene() {
  std::uniform_real_distribution<double> U(1, 2);
  Cloud ceil;
  for (int i = 0; i < 400; ++i) {
    double x = U(RNG), y = U(RNG);
    ceil.push_back({x, y, 2.4});
  }
  Cloud fl;
  for (auto& p : floor_(-2, 4, -2, 4))
    if (!(p.x > 1 && p.x < 2 && p.y > 1 && p.y < 2)) fl.push_back(p);
  return cat({fl, ceil});
}
}  // namespace

TEST(Synthetic, CeilingOnlyTileMakesNoObstacle) {
  tc::HeightGrid hg(ceiling_scene(), std::make_shared<const tc::Config>(CFG));
  int n = 0;
  for (auto v : hg.occ) n += v;
  EXPECT_EQ(n, 0);
}

TEST(Synthetic, LocalGroundFloorEverywhere) {
  auto g = tc::local_ground(ceiling_scene(), tc::Config());
  double mx = 0;
  for (double v : g.g) mx = std::max(mx, std::abs(v));
  EXPECT_TRUE(mx < 0.05 && g.res == 0.1) << mx;
}

TEST(Synthetic, SofaDrivenAround) {
  Cloud sofa;
  for (double y : tc::arange(0, 0.8, 0.04))
    for (double x : tc::arange(0, 1.2, 0.04)) sofa.push_back({x, y, 0.45});
  sofa = cat({sofa, wall(0, 0, 1.2, 0, 0, 0.45), wall(1.2, 0, 1.2, 0.8, 0, 0.45), wall(1.2, 0.8, 0, 0.8, 0, 0.45),
              wall(0, 0.8, 0, 0, 0, 0.45)});
  Cloud fl;
  for (auto& p : floor_(-3, 4, -3, 4))
    if (!(p.x > 0 && p.x < 1.2 && p.y > 0 && p.y < 0.8)) fl.push_back(p);
  auto out = tc::plan(cat({fl, sofa}), std::nullopt, V2(-2, 0.4), V2(3, 0.4), CFG);
  double near = 1e9;
  if (out.ok)
    for (auto& p : tc::densify(out.path, 0.05)) near = std::min(near, std::hypot(p.x() - 0.6, p.y() - 0.4));
  EXPECT_TRUE(out.ok && near > 0.6) << out.length << " nearest " << near;
}

namespace {
tc::CloudF F(const Cloud& c) {
  tc::CloudF o;
  for (auto& p : c) o.push_back({float(p.x), float(p.y), float(p.z)});
  return o;
}
}  // namespace

TEST(Synthetic, LiveWindowForgetsAWallSeen4sAgo) {
  tc::LiveWindow lw(tc::live_config(), 3.0);
  lw.add_scan(0.0, F(cat({floor_(-3, 3, -2, 4), wall(-2, 1.5, 2, 1.5)})), Eigen::Vector3d(0, 0, 0.25));
  auto a = lw.plan(V2(0, 0), V2(0, 3));
  for (double t : {3.5, 4.0, 4.5}) lw.add_scan(t, F(floor_(-3, 3, -2, 4)), Eigen::Vector3d(0, 0, 0.25));
  auto b = lw.plan(V2(0, 0), V2(0, 3));
  EXPECT_TRUE((!a.ok || a.length > 3.5) && b.ok && b.length < 3.2) << a.length << " " << b.length;
}

TEST(Synthetic, MemoryHoldsWallFromAfarNotPersonBeside) {
  tc::LiveWindow lw(tc::live_config(), 3.0);
  Cloud fl = floor_(-3, 3, -1, 5);
  Cloud seen = cat({fl, wall(-1.5, 3.0, 1.5, 3.0), wall(0.5, 0.4, 0.7, 0.4, 0.0, 1.7)});
  for (double t : tc::arange(0.0, 2.5, 0.5)) {
    lw.add_scan(t, F(seen), Eigen::Vector3d(0, 0, 0.08));
    lw.plan(V2(0, 0), V2(0, 4.5));
  }
  Cloud away;
  for (auto& p : fl)
    if (p.y < 0.0) away.push_back(p);
  tc::PlanResult out;
  for (double t : tc::arange(4.0, 6.5, 0.5)) {
    lw.add_scan(t, F(away), Eigen::Vector3d(0, 0, 0.08));   // looking away: no evidence either way
    out = lw.plan(V2(0, 0), V2(0, 4.5));
  }
  auto& hg = *out.hg;
  bool held = true;
  for (double x : {-0.5, 0.0, 0.5}) {
    auto c = hg.cell(V2(x, 3.0));
    held = held && hg.occ[hg.idx(c.first, c.second)];
  }
  auto pc = hg.cell(V2(0.6, 0.4));
  bool person_gone = !hg.occ[hg.idx(pc.first, pc.second)];
  EXPECT_TRUE(held && person_gone && out.ok && out.length > 4.8) << held << person_gone << " " << out.length;
}

TEST(Synthetic, Carrot05mAhead) {
  Path path{V2(0, 0), V2(2, 0), V2(2, 2)};
  auto [c, yaw] = tc::carrot(path, V2(1.0, 0.1), 0.5);
  EXPECT_NEAR(c.x(), 1.5, 1e-6);
  EXPECT_NEAR(c.y(), 0.0, 1e-6);
}

TEST(Synthetic, CarrotTurnsTheCorner) {
  auto c = tc::carrot({V2(0, 0), V2(2, 0), V2(2, 2)}, V2(1.9, 0.0), 0.5).first;
  EXPECT_NEAR(c.x(), 2.0, 1e-6);
  EXPECT_NEAR(c.y(), 0.4, 1e-6);
}

TEST(Synthetic, CarrotStopsAtTheGoal) {
  auto c = tc::carrot({V2(0, 0), V2(2, 0), V2(2, 2)}, V2(2.0, 1.9), 0.5).first;
  EXPECT_TRUE((c - V2(2, 2)).norm() < 1e-8);
}

TEST(Synthetic, RemainingPathFromTheRobot) {
  auto rem = tc::remaining({V2(0, 0), V2(2, 0), V2(2, 2)}, V2(1.0, 0.1));
  EXPECT_TRUE(rem.size() == 3 && (rem[1] - V2(2, 0)).norm() < 1e-8);
}

TEST(Synthetic, DriveChainAroundTheWallFrameClear) {
  Cloud walls = WALLS(), room = cat({floor_(-6, 6, -5, 3), walls});
  auto r = drive_chain(CFG, walls, room, V2(-3, 0), V2(3, 0));
  EXPECT_TRUE(r.clear && r.reached) << "reached " << r.reached << " in " << r.t << " s, worst " << r.worst;
}

// ---- pivots
TEST(Pivots, CornerOf07mCorridorsRefused) {
  Cloud L = cat({w(-4, -0.35, 0, -0.35), w(-4, 0.35, -0.7, 0.35), w(0, -0.35, 0, 4), w(-0.7, 0.35, -0.7, 4),
                 w(-4, -0.35, -4, 0.35), w(-0.7, 4, 0, 4)});
  auto out = tc::plan(cat({floor_(-5, 5, -5, 5), L}), std::nullopt, at(-3, 0), at(-0.35, 3), CFG, 0.0);
  EXPECT_FALSE(out.ok) << "planned";
}

TEST(Pivots, NoRoomWhereItStandsMovedAheadIntoTheRoom) {
  Cloud cor = cat({w(-4, -0.35, 0.5, -0.35), w(-4, 0.35, 0.5, 0.35), w(-4, -0.35, -4, 0.35), w(0.5, 0.35, 0.5, 1.0),
                   w(0.5, -0.35, 0.5, -1.0), w(0.5, 1.0, 2.0, 1.0), w(0.5, -1.0, 2.0, -1.0), w(2.0, -1.0, 2.0, 1.0)});
  auto out = tc::plan(cat({floor_(-5, 5, -5, 5), cor}), std::nullopt, at(0.7, 0), at(-2.5, 0), CFG, 0.0);
  ASSERT_TRUE(out.ok) << out.reason;
  auto L = legs_clear(out, band_xy(cor, CFG), CFG);
  bool moved = out.pivots.size() == 1 && !out.pivots[0].start && out.pivots[0].centre.x() > 0.6;
  EXPECT_TRUE(moved && L.ok) << "worst " << L.worst << " near " << L.near;
}

TEST(Pivots, UTurnIn07mCorridorRefusedFacingTheGoalDrives) {
  Cloud dead = cat({w(-4, -0.35, 1.0, -0.35), w(-4, 0.35, 1.0, 0.35), w(1.0, -0.35, 1.0, 0.35), w(-4, -0.35, -4, 0.35)});
  Cloud dc = cat({floor_(-5, 5, -5, 5), dead});
  auto out = tc::plan(dc, std::nullopt, at(0, 0), at(-2.5, 0), CFG, 0.0);
  auto back = tc::plan(dc, std::nullopt, at(0, 0), at(-2.5, 0), CFG, M_PI);
  auto blind = tc::plan(dc, std::nullopt, at(0, 0), at(-2.5, 0), CFG);
  EXPECT_TRUE(!out.ok && back.ok && back.pivots.empty() && blind.ok) << out.reason << " | " << back.reason;
}

namespace {
Cloud hairpin() {
  return cat({w(-4, -0.35, 0, -0.35), w(-4, 0.35, 0, 0.35), w(-4, 0.95, 0, 0.95), w(-4, 1.85, 0, 1.85), w(0, -0.35, 0, -0.5),
              w(0, 1.85, 0, 2.0), w(0, -0.5, 1.2, -0.5), w(0, 2.0, 1.2, 2.0), w(1.2, -0.5, 1.2, 2.0), w(0, 0.35, 0, 0.95),
              w(-4, -0.35, -4, 0.35), w(-4, 0.95, -4, 1.85)});
}
}  // namespace

TEST(Pivots, HairpinPlannedWithPivotsInTheRoom) {
  Cloud hp = hairpin();
  auto out = tc::plan(cat({floor_(-5, 5, -5, 5), hp}), std::nullopt, at(-1.5, 0), at(-1.5, 1.4), CFG, 0.0);
  ASSERT_TRUE(out.ok) << out.reason;
  auto L = legs_clear(out, band_xy(hp, CFG), CFG);
  bool in_room = !out.pivots.empty();
  for (auto& p : out.pivots) in_room = in_room && p.centre.x() > 0.0;
  EXPECT_TRUE(in_room && L.ok) << out.pivots.size() << " pivots, worst " << L.worst << " near " << L.near;
}

TEST(Pivots, DriveChainWithPivotsThroughTheHairpin) {
  Cloud hp = hairpin(), hc = cat({floor_(-5, 5, -5, 5), hp});
  auto r = drive_chain_pivots(CFG, hp, hc, at(-1.5, 0.0), 0.0, at(-1.5, 1.4));
  bool circles = !r.seen.empty();
  for (double d : r.seen) circles = circles && d >= tc::FOOTPRINT_HALF_DIAGONAL;
  EXPECT_TRUE(r.clear && r.reached && circles) << "reached " << r.reached << " in " << r.t << " s, worst " << r.worst
                                               << ", " << r.seen.size() << " pivots";
}

TEST(Pivots, StartPivotAboutTheFrameCentre) {
  Cloud fl = floor_(-5, 5, -5, 5);
  auto out = tc::plan(fl, std::nullopt, V2(0, 0), V2(-3, 0), CFG, 0.0);
  ASSERT_TRUE(out.ok) << out.reason;
  bool ok = out.pivots.size() == 1 && out.pivots[0].start && (out.pivots[0].centre - V2(-CFG.off, 0)).norm() < 1e-6 &&
            (out.path[1] - V2(-2 * CFG.off, 0)).norm() < 1e-6 && std::abs(std::abs(out.pivots[0].yaw_out) - M_PI) < 0.05;
  auto same = tc::plan(fl, std::nullopt, V2(0, 0), V2(-3, 0), CFG, M_PI);
  auto small = tc::plan(fl, std::nullopt, V2(0, 0), V2(3, 1.5), CFG, 0.0);   // 27 deg: driven, no pivot
  EXPECT_TRUE(ok && same.ok && same.pivots.empty() && small.ok && small.pivots.empty());
}

namespace {
Path PIV_PATH() { return {V2(0, 0), V2(2, 0), V2(2.0 - CFG.off, CFG.off), V2(2.0 - CFG.off, 2.0)}; }
std::vector<tc::Pivot> PIVS() {
  tc::Pivot p;
  p.i = 1;
  p.centre = V2(2.0 - CFG.off, 0.0);
  p.yaw_in = 0.0;
  p.yaw_out = M_PI / 2;
  p.turn = M_PI / 2;
  p.start = false;
  return {p};
}
}  // namespace

TEST(Pivots, CarrotDrivesHoldsTurnsDrivesOn) {
  auto P = PIV_PATH();
  auto pv = PIVS();
  auto f1 = tc::follow_plan(P, pv, V2(1.0, 0.0), 0.0, CFG, 0.15);
  auto f2 = tc::follow_plan(P, pv, V2(1.7, 0.0), 0.0, CFG, 0.15);
  auto f3 = tc::follow_plan(P, pv, V2(2.0, 0.0), 0.0, CFG, 0.15);
  auto f4 = tc::follow_plan(P, pv, V2(2.0 - CFG.off, CFG.off), M_PI / 2, CFG, 0.15);
  EXPECT_TRUE(f1.phase == "approach" && (f1.c - V2(1.5, 0)).norm() < 1e-8 && f2.phase == "approach" &&
              (f2.c - V2(2.0 - CFG.off + 0.15, 0)).norm() < 1e-8 && f3.phase == "pivot" &&
              (f3.c - V2(2.0 - CFG.off, CFG.off + CFG.lookahead)).norm() < 1e-8 && f4.phase == "drive" && f4.c.y() > 0.6)
      << f1.phase << " " << f2.phase << " " << f3.phase << " " << f4.phase;
}

// ---- the run-out
TEST(RunOut, TargetOnTheCarrotBearingNeverPastTheGoalNoneWithoutAWindow) {
  Cloud fl = floor_(-5, 5, -5, 5);
  auto win = tc::plan(fl, std::nullopt, V2(0 + E, 0 + E), V2(4 + E, 0 + E), CFG).grid;
  Path path{V2(0, 0), V2(4, 0)};
  auto f = tc::follow_plan(path, {}, V2(1, 0), 0.0, CFG);
  auto [t1, e1] = tc::run_out(path, {}, V2(1, 0), 0.0, f.c, f.phase, f.j, CFG, 1.2, FCB, 0.15, win.get());
  auto f2 = tc::follow_plan(path, {}, V2(3.5, 0), 0.0, CFG);
  auto [t2, e2] = tc::run_out(path, {}, V2(3.5, 0), 0.0, f2.c, f2.phase, f2.j, CFG, 1.2, FCB, 0.15, win.get());
  auto [t3, e3] = tc::run_out(path, {}, V2(1, 0), 0.0, f.c, f.phase, f.j, CFG, 1.2, FCB, 0.15, nullptr);
  EXPECT_TRUE(std::abs((t1 - V2(1.0 - FCB, 0.0)).norm() - 1.2) < 1e-6 && std::abs(t1.y()) < 1e-9 && e1 > 0.5 &&
              (t2 - V2(4, 0)).norm() < 1e-8 && e2 == 0.0 && (t3 - f.c).norm() < 1e-12 && e3 == 0.0)
      << t1.transpose() << " +" << e1 << " | " << t2.transpose() << " | " << t3.transpose();
}

TEST(RunOut, ApproachingAPivotNeverPastItsHoldPointNoneWhilePivoting) {
  Cloud fl = floor_(-5, 5, -5, 5);
  auto win = tc::plan(fl, std::nullopt, V2(0 + E, 0 + E), V2(4 + E, 0 + E), CFG).grid;
  auto P = PIV_PATH();
  auto pv = PIVS();
  double H = 2.0 - CFG.off + 0.15;
  bool ok = true;
  for (double x : {0.5, 1.0, 1.5}) {
    auto f = tc::follow_plan(P, pv, V2(x, 0), 0.0, CFG, 0.15);
    auto tt = tc::run_out(P, pv, V2(x, 0), 0.0, f.c, f.phase, f.j, CFG, 1.2, FCB, 0.15, win.get()).first;
    ok = ok && f.phase == "approach" && tt.x() <= H + 1e-9;
  }
  auto f = tc::follow_plan(P, pv, V2(2.0, 0), 0.0, CFG, 0.15);
  auto [tp, ep] = tc::run_out(P, pv, V2(2.0, 0), 0.0, f.c, f.phase, f.j, CFG, 1.2, FCB, 0.15, win.get());
  EXPECT_TRUE(ok && f.phase == "pivot" && ep == 0.0 && (tp - f.c).norm() < 1e-12);
}

TEST(RunOut, CutShortOfAWallAcrossTheBearing) {
  Cloud fl = floor_(-5, 5, -5, 5);
  auto win = tc::plan(cat({fl, w(2.3, -2.0, 2.3, 1.0)}), std::nullopt, V2(0 + E, 0 + E), V2(-1 + E, 1 + E), CFG).grid;
  Path path{V2(0, 0), V2(4, 0)};   // (the run-out checks the frame, not the path)
  auto f = tc::follow_plan(path, {}, V2(1.2, 0), 0.0, CFG);
  auto [tw, ew] = tc::run_out(path, {}, V2(1.2, 0), 0.0, f.c, f.phase, f.j, CFG, 1.2, FCB, 0.15, win.get());
  double front = tw.x() + FCB - CFG.off + tc::FOOTPRINT_LEN / 2;   // the bare frame's nose with the FC at the target
  // (the grid checks obstacle cell centres against the frame + margin: the bare frame is clear, the
  // margin absorbs the half-cell rounding)
  EXPECT_TRUE(ew < 1.2 - (0.5 + FCB) && 2.3 + E - CFG.margin - CFG.res <= front && front < 2.3 + E && tw.x() >= f.c.x())
      << tw.transpose() << " +" << ew << " nose " << front;
}

TEST(RunOut, DriveChainThroughA090mDoorAt1mps) {
  Cloud fl = floor_(-5, 5, -5, 5);
  Cloud dr = cat({w(0.0, -5.0, 0.0, -0.45), w(0.0, 0.45, 0.0, 5.0)});
  Cloud cloud = cat({fl, dr});
  // (the bare carrot only for its speed: the model's low-speed steering is cruder than Rover's)
  auto bare = drive_chain_rover(CFG, dr, cloud, V2(-2.5 + E, 1.2 + E), 0.0, V2(2.5 + E, -0.8 + E), 0.0);
  auto ro = drive_chain_rover(CFG, dr, cloud, V2(-2.5 + E, 1.2 + E), 0.0, V2(2.5 + E, -0.8 + E), 2.1);
  double past = (ro.final_ - V2(2.5 + E, -0.8 + E)).norm();
  EXPECT_TRUE(ro.reached && ro.worst == 0.0 && ro.vmax >= 0.9 && bare.vmax < 0.45 && past < 0.4 && std::abs(ro.v) < 0.02)
      << "run-out " << ro.t << " s vmax " << ro.vmax << " worst " << ro.worst << " past " << past << " v " << ro.v
      << " " << ro.why << "; bare vmax " << bare.vmax << " " << bare.why;
}

// ---- dead ends
namespace {
Cloud dead_corr() {
  return cat({w(-2.0, -0.35, 0.0, -0.35), w(-2.0, 0.35, 0.0, 0.35), w(0.0, -0.35, 0.0, 0.35), w(-2.0, -3.0, -2.0, -0.35),
              w(-2.0, 0.35, -2.0, 3.0)});
}
const V2 RS = at(-0.6, 0.0), RG = at(-3.5, 1.5);
}  // namespace

TEST(Reverse, DeadEndCorridorBackUpPivotDrive) {
  Cloud corr = dead_corr(), fl = floor_(-5, 0.5, -3, 3);
  auto out = tc::plan(cat({fl, corr}), std::nullopt, RS, RG, CFG, 0.0);
  ASSERT_TRUE(out.ok && out.reverse && !out.pivots.empty() && out.pivots[0].start) << out.reason << " | " << out.reverse_reason;
  // independent: the bare frame along the reverse (heading kept) and the pivot circle at its end
  auto obs = band_xy(corr, CFG);
  double worst = 0.0;
  for (double r : tc::arange(0.0, out.reverse->dist + 1e-9, 0.02)) worst = std::max(worst, intrusion(RS - r * V2(1, 0), 0.0, obs, CFG.off));
  bool circ = disc_clear(out.pivots[0].centre, obs).first;
  auto L = legs_clear(out, obs, CFG);
  EXPECT_TRUE(worst == 0.0 && circ && L.ok && out.reverse->dist <= CFG.reverse_max)
      << "reverse " << out.reverse->dist << " worst " << worst << " legs worst " << L.worst;
}

TEST(Reverse, ObstacleBehindRefused) {
  Cloud corr = dead_corr(), fl = floor_(-5, 0.5, -3, 3);
  Cloud cl = cat({fl, corr, w(-1.6, -0.3, -1.6, 0.3), w(-1.4, -0.3, -1.4, 0.3)});
  auto out = tc::plan(cl, std::nullopt, RS, RG, CFG, 0.0);
  double m = CFG.r_out + CFG.res * 2 + CFG.pad;
  auto hg = std::make_shared<tc::HeightGrid>(cl, std::make_shared<const tc::Config>(CFG), V2(RS.cwiseMin(RG).array() - m),
                                             V2(RS.cwiseMax(RG).array() + m));
  auto rb = tc::plan_reverse(hg, RS, RG, 0.0);   // the reverse's own check (plan() fails the search first here)
  EXPECT_TRUE(!out.ok && !out.reverse && !rb.ok && rb.reason.find("blocked") != std::string::npos)
      << out.reason << " | plan_reverse: " << rb.reason;
}

TEST(Reverse, UnobservedFloorBehindRefused) {
  Cloud corr = dead_corr(), seen;
  for (auto& p : floor_(-5, 0.5, -3, 3))
    if (!(p.x > -1.6 && p.x < -1.0 && std::abs(p.y) < 0.6)) seen.push_back(p);
  auto out = tc::plan(cat({seen, corr}), std::nullopt, RS, RG, CFG, 0.0);
  EXPECT_TRUE(!out.ok && out.reverse_reason.find("unobserved") != std::string::npos) << out.reason << " | " << out.reverse_reason;
}

TEST(Reverse, NoRoomWithinReverseMaxRefused) {
  tc::Config c2;
  c2.reverse_max = 0.8;
  c2.update();
  auto out = tc::plan(cat({floor_(-5, 0.5, -3, 3), dead_corr()}), std::nullopt, RS, RG, c2, 0.0);
  EXPECT_TRUE(!out.ok && !out.reverse) << out.reason << " | " << out.reverse_reason;
}

// ---- the arc onto the path (owner request 2026-10-29: Gate L2 c1/c3, the robot landed 33-35 deg off
// the drive leg's first bearing, arced onto the path with its heading lagging and clipped the 0.90 m
// door's jamb by 7.5 cm). The drive chain with Rover's speed law steers with SITL's lag (pure pursuit
// a curvature of 2 sin(error) / max(distance, 3.0 m), fit to SITL c1, c2, c3, d18) from a start heading 30-50 deg off near the door.
namespace {
const double STEER = 3.0;   // SITL's steering lag (arc_steer, fit to c1, c2, c3, d18)
Cloud door_room() {
  return cat({floor_(-4, 4, -3, 3), w(0.0, -3.0, 0.0, -0.45), w(0.0, 0.45, 0.0, 3.0)});   // a 0.90 m door at x = 0
}
Cloud door_walls() { return cat({w(0.0, -3.0, 0.0, -0.45), w(0.0, 0.45, 0.0, 3.0)}); }
const V2 DS = at(-1.3, 0.55), DG = at(2.0, 0.0);
}  // namespace

TEST(ArcCheck, StartHeadingOffNearANarrowDoorNoClip) {
  Cloud room = door_room(), walls = door_walls();
  double first = std::atan2(DG.y() - DS.y(), DG.x() - DS.x());   // roughly the path's first bearing
  for (double off_deg : {30.0, 40.0, 50.0, -30.0, -40.0, -50.0}) {
    double yaw = first + tc::radians(off_deg);
    int piv = 0;
    auto r = drive_chain_rover(CFG, walls, room, DS, yaw, DG, 2.1, 1.0, 0.5, 1.0, FCB, 2.5, 1.0, 0.05, 0.3, 0.15, 4.0, STEER, &piv);
    EXPECT_TRUE(r.reached && r.worst == 0.0) << "start " << off_deg << " deg off: reached " << r.reached << " worst intrusion "
                                             << r.worst << " pivots " << piv << " " << r.why;
  }
}

TEST(ArcCheck, WithoutItTheLaggingArcClipsTheJamb) {
  // the same scene without the arc check: some start heading clips (the test has teeth)
  tc::Config off = CFG;
  off.arc_check = false;
  off.update();
  Cloud room = door_room(), walls = door_walls();
  double first = std::atan2(DG.y() - DS.y(), DG.x() - DS.x()), worst = 0.0;
  for (double off_deg : {30.0, 40.0, 50.0, -30.0, -40.0, -50.0}) {
    auto r = drive_chain_rover(off, walls, room, DS, first + tc::radians(off_deg), DG, 2.1, 1.0, 0.5, 1.0, FCB, 2.5, 1.0, 0.05,
                               0.3, 0.15, 4.0, STEER);
    worst = std::max(worst, r.worst);
  }
  EXPECT_GT(worst, 0.0);
}

TEST(ArcCheck, ArcNotClearMakesAStartPivotUnderPivotAngle) {
  Cloud room = door_room();
  double first = std::atan2(DG.y() - DS.y(), DG.x() - DS.x());
  auto out = tc::plan(room, std::nullopt, DS, DG, CFG, first + tc::radians(40.0));
  ASSERT_TRUE(out.ok) << out.reason;
  ASSERT_FALSE(out.pivots.empty());
  EXPECT_TRUE(out.pivots[0].start && out.pivots[0].turn <= CFG.pivot_angle) << out.pivots[0].turn;
  // the carrot leads the heading past pivot_angle so Rover pivots, until within pivot_done
  auto f = tc::follow_plan(out.path, out.pivots, DS, first + tc::radians(40.0), CFG);
  double e = std::abs(tc::wrap(tc::hd(f.c - out.pivots[0].centre) - (first + tc::radians(40.0))));
  EXPECT_EQ(f.phase, "pivot");
  EXPECT_GT(e, CFG.pivot_angle);
  // in the open the same heading is driven (no pivot)
  auto open = tc::plan(floor_(-4, 4, -3, 3), std::nullopt, DS, DG, CFG, first + tc::radians(40.0));
  EXPECT_TRUE(open.ok && open.pivots.empty());
}

TEST(ArcCheck, LagOnAGentleTurnNextToAWallIsRefusedOrRoutedClear) {
  // a gentle (under pivot_angle) turn into a 0.75 m gap right after a bend: the lagging heading on the
  // bend; whatever the plan, the chain with SITL's lag stays clear
  Cloud walls = cat({w(1.5, -3.0, 1.5, -0.375), w(1.5, 0.375, 1.5, 3.0)});
  Cloud room = cat({floor_(-3, 5, -3, 3), walls});
  for (double y0 : {0.0, 0.35, -0.35}) {
    auto r = drive_chain_rover(CFG, walls, room, V2(-1.5 + E, 1.2 + E), y0, V2(3.5 + E, -0.6 + E), 2.1, 1.0, 0.5, 1.0, FCB, 2.5,
                               1.0, 0.05, 0.3, 0.15, 4.0, STEER);
    EXPECT_EQ(r.worst, 0.0) << "yaw " << y0 << ": reached " << r.reached << " " << r.why;
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
