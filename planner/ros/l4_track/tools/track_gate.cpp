// The track planner's real-map gate (C++ port of planner/test/real_maps.py and the --map/--gate part
// of test_track_planner.py, 2026-10-29). It times the C++ planner.
//
//   track_gate [--map LABEL:SPEC ...] [--live-step N] [--gate] [--dump FILE.jsonl]
//
// SPEC is view=DIR[,kf=OUT] (map_points.bin + map_meta.json, ground at 0) or
// pcd=FILE,path=CSV|meta=VIEWDIR[,kf=OUT]; OUT is the mapping run's SC-PGO output (optimized_poses.txt,
// times.txt, Scans/*.pcd: the keyframe scans, sensor frame), aligned to the path by a rigid fit of the
// keyframe positions, which adds free-space evidence to the map and the live-window replay.
//
//   map mode   the whole map, built once (HeightGrid), with the keyframes' free-space evidence when
//              given; goals sampled every 2 m along the driven path, 2/5/10/20 m ahead of it
//   live mode  the keyframes replayed in order through LiveWindow (the node's map), a replan from
//              every live_step-th keyframe to a goal 3 and 6 m ahead along the driven path, with the
//              keyframe's heading (the start pivot); timed
//   walls      reference walls (points in each 0.3 m layer from 0.3 to 1.5 m above the ground and,
//              with keyframes, hit by 3+ of them and never mostly seen through: people are tall too);
//              driven-path pairs 1-4 m apart with a wall between them but far apart along the path,
//              and goals 1 m behind the nearest wall: each must be refused, or routed with the whole
//              path at least 0.2 m from every reference wall cell
//
// Real maps pass with >= 90% of the goals along the driven path planned on the whole map, >= 85% on
// the live window, and every wall check passed; --gate also requires the live replan p95 <= 100 ms
// (Gate L1: >= 10 Hz; run it pinned to 4 cores: taskset -c <4 cpus>).
//
// The pair and behind goals are drawn as real_maps.py drew them (numpy's RandomState(0) / (1)
// permutation, reproduced); the neighbours of a pair's first point are tried in index order
// (scipy's cKDTree returned them in its tree order).
#include <Eigen/Dense>

#include <algorithm>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <numeric>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <nlohmann/json.hpp>
#include <random>
#include <sstream>

#include "l4_track/track_core.h"

namespace tc = l4_track;
using tc::V2;
using json = nlohmann::json;

namespace {
double now_s() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// A/B switches for the preferred clearance and the start check (the node's ~prefer_clearance,
// ~start_frame, ~start_soft, ~overhang_clear): L4_PREFER, L4_START_FRAME, L4_START_SOFT, L4_OVERHANG (0: off)
void ab_env(tc::Config& c) {
  if (std::getenv("L4_OVERHANG")) c.overhang_clear = std::atof(std::getenv("L4_OVERHANG"));
  if (std::getenv("L4_OVERHANG_SEEN")) c.overhang_seen = std::atof(std::getenv("L4_OVERHANG_SEEN"));
  if (std::getenv("L4_OVERHANG_RING")) c.overhang_ring = std::atoi(std::getenv("L4_OVERHANG_RING"));
  if (std::getenv("L4_PREFER")) c.prefer_clearance = std::atof(std::getenv("L4_PREFER"));
  if (std::getenv("L4_START_FRAME")) c.start_frame = std::atoi(std::getenv("L4_START_FRAME")) != 0;
  if (std::getenv("L4_START_SOFT")) c.start_soft = std::atoi(std::getenv("L4_START_SOFT")) != 0;
  if (std::getenv("L4_CUT")) c.cut_check = std::atoi(std::getenv("L4_CUT")) != 0;
  if (std::getenv("L4_CUT_LEAD")) c.cut_lead = std::atof(std::getenv("L4_CUT_LEAD"));
  if (std::getenv("L4_MERGE")) c.bend_merge = std::atof(std::getenv("L4_MERGE"));
}

std::vector<bool> results;
void check(const std::string& name, bool ok, const std::string& detail = "") {
  results.push_back(ok);
  printf("%s  %-62s %s\n", ok ? "PASS" : "FAIL", name.c_str(), detail.c_str());
  fflush(stdout);
}

std::string fmt(const char* f, ...) __attribute__((format(printf, 1, 2)));
std::string fmt(const char* f, ...) {
  char buf[2048];
  va_list ap;
  va_start(ap, f);
  vsnprintf(buf, sizeof buf, f, ap);
  va_end(ap);
  return buf;
}

std::string slurp(const std::string& p) {
  std::ifstream f(p, std::ios::binary);
  if (!f) throw std::runtime_error("cannot read " + p);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// ---- loaders
tc::Cloud load_pcd(const std::string& p) {
  std::string raw = slurp(p);
  const std::string tag = "DATA binary\n";
  size_t k = raw.find(tag);
  if (k == std::string::npos) throw std::runtime_error("not a binary PCD: " + p);
  std::istringstream hdr(raw.substr(0, k));
  std::string line;
  long n = 0;
  int nf = 0;
  while (std::getline(hdr, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ls(line);
    std::string key;
    ls >> key;
    if (key == "POINTS") ls >> n;
    if (key == "FIELDS") {
      std::string f;
      while (ls >> f) nf++;
    }
  }
  const char* d = raw.data() + k + tag.size();
  tc::Cloud out(n);
  for (long i = 0; i < n; ++i) {
    float v[3];
    std::memcpy(v, d + (size_t(i) * nf) * 4, 12);
    out[i] = {v[0], v[1], v[2]};
  }
  return out;
}

std::vector<Eigen::Vector3d> load_traj_json(const json& m) {
  std::vector<Eigen::Vector3d> t;
  for (auto& r : m["traj"]) t.push_back(Eigen::Vector3d(r[0].get<double>(), r[1].get<double>(), r[2].get<double>()));
  return t;
}

void load_view(const std::string& d, tc::Cloud* pts, std::vector<Eigen::Vector3d>* traj) {
  json m = json::parse(slurp(d + "/map_meta.json"));
  if (pts) {
    std::string raw = slurp(d + "/map_points.bin");
    long n = m["points"].get<long>();
    pts->resize(n);
    for (long i = 0; i < n; ++i) {
      int16_t v[3];
      std::memcpy(v, raw.data() + i * 6, 6);
      (*pts)[i] = {v[0] / 100.0, v[1] / 100.0, v[2] / 100.0};
    }
  }
  if (traj) *traj = load_traj_json(m);
}

std::vector<std::vector<double>> load_txt(const std::string& p, char delim = 0, int skip = 0) {
  std::ifstream f(p);
  if (!f) throw std::runtime_error("cannot read " + p);
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

// ---- numpy RandomState(seed).permutation(arr) (legacy MT19937 + random_interval)
std::vector<long> permutation(uint32_t seed, std::vector<long> a) {
  std::mt19937 mt(seed);
  for (long i = long(a.size()) - 1; i >= 1; --i) {
    uint64_t max = uint64_t(i), mask = max, v;
    mask |= mask >> 1;
    mask |= mask >> 2;
    mask |= mask >> 4;
    mask |= mask >> 8;
    mask |= mask >> 16;
    mask |= mask >> 32;
    while ((v = (uint64_t(mt()) & mask)) > max) {
    }
    std::swap(a[i], a[v]);
  }
  return a;
}

// ---- a 2-D k-d tree (nearest, radius)
class KD {
 public:
  explicit KD(const std::vector<V2>& p) : p_(p), idx_(p.size()) {
    std::iota(idx_.begin(), idx_.end(), 0);
    build(0, int(p.size()), 0);
  }
  size_t size() const { return p_.size(); }
  std::pair<double, int> nearest(const V2& q) const {
    double bd = 1e300;
    int bi = -1;
    nn(0, int(p_.size()), 0, q, bd, bi);
    return {std::sqrt(bd), bi};
  }
  std::vector<int> ball(const V2& q, double r) const {
    std::vector<int> out;
    rb(0, int(p_.size()), 0, q, r * r, out);
    std::sort(out.begin(), out.end());
    return out;
  }

 private:
  std::vector<V2> p_;
  std::vector<int> idx_;
  void build(int a, int b, int d) {
    if (b - a <= 1) return;
    int m = (a + b) / 2;
    std::nth_element(idx_.begin() + a, idx_.begin() + m, idx_.begin() + b,
                     [&](int x, int y) { return p_[x][d] < p_[y][d]; });
    build(a, m, 1 - d);
    build(m + 1, b, 1 - d);
  }
  void nn(int a, int b, int d, const V2& q, double& bd, int& bi) const {
    if (a >= b) return;
    int m = (a + b) / 2;
    const V2& p = p_[idx_[m]];
    double dd = (p - q).squaredNorm();
    if (dd < bd || (dd == bd && idx_[m] < bi)) {
      bd = dd;
      bi = idx_[m];
    }
    double diff = q[d] - p[d];
    int f1a = diff < 0 ? a : m + 1, f1b = diff < 0 ? m : b;
    int f2a = diff < 0 ? m + 1 : a, f2b = diff < 0 ? b : m;
    nn(f1a, f1b, 1 - d, q, bd, bi);
    if (diff * diff <= bd) nn(f2a, f2b, 1 - d, q, bd, bi);
  }
  void rb(int a, int b, int d, const V2& q, double r2, std::vector<int>& out) const {
    if (a >= b) return;
    int m = (a + b) / 2;
    const V2& p = p_[idx_[m]];
    if ((p - q).squaredNorm() <= r2) out.push_back(idx_[m]);
    double diff = q[d] - p[d];
    if (diff <= 0 || diff * diff <= r2) rb(a, m, 1 - d, q, r2, out);
    if (diff >= 0 || diff * diff <= r2) rb(m + 1, b, 1 - d, q, r2, out);
  }
};

struct Stats {
  double median = 0, p95 = 0, max = 0;
  int n = 0;
};
Stats stats_ms(const std::vector<double>& x) {
  Stats s;
  if (x.empty()) return s;
  std::vector<double> v;
  for (double t : x) v.push_back(t * 1000);
  s.median = tc::median(v);
  s.p95 = tc::percentile(v, 95);
  s.max = *std::max_element(v.begin(), v.end());
  s.n = int(v.size());
  return s;
}

// SC-PGO keyframes in the map frame: a rigid fit of the keyframe positions to the path
struct Keyframes {
  std::vector<Eigen::Matrix3d> Rk;
  std::vector<Eigen::Vector3d> pk;
  std::vector<double> t;
  double rms = 0;
  std::string out;
  Keyframes(const std::string& dir, const std::vector<Eigen::Vector3d>& path) : out(dir) {
    auto T = load_txt(dir + "/optimized_poses.txt");
    auto tt = load_txt(dir + "/times.txt");
    std::vector<double> ts;
    for (auto& r : tt)
      for (double v : r) ts.push_back(v);
    size_t n = std::min({T.size(), ts.size(), path.size()});
    t.assign(ts.begin(), ts.begin() + n);
    Eigen::MatrixXd A(n, 3), B(n, 3);
    for (size_t i = 0; i < n; ++i) {
      A.row(i) << T[i][3], T[i][7], T[i][11];
      B.row(i) = path[i].transpose();
    }
    Eigen::RowVector3d ca = A.colwise().mean(), cb = B.colwise().mean();
    Eigen::Matrix3d H = (A.rowwise() - ca).transpose() * (B.rowwise() - cb);
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU(), V = svd.matrixV();
    double d = (V * U.transpose()).determinant() > 0 ? 1.0 : -1.0;
    Eigen::Matrix3d R = V * Eigen::Vector3d(1, 1, d).asDiagonal() * U.transpose();
    Eigen::Vector3d off = cb.transpose() - R * ca.transpose();
    double ss = 0;
    for (size_t i = 0; i < n; ++i) {
      Eigen::Matrix3d Ri;
      Ri << T[i][0], T[i][1], T[i][2], T[i][4], T[i][5], T[i][6], T[i][8], T[i][9], T[i][10];
      Rk.push_back(R * Ri);
      Eigen::Vector3d p = R * A.row(i).transpose() + off;
      pk.push_back(p);
      ss += (p - path[i]).squaredNorm();
    }
    rms = std::sqrt(ss / n);
  }
  size_t size() const { return t.size(); }
  tc::Cloud scan(size_t k) const {
    char name[64];
    snprintf(name, sizeof name, "/Scans/%06zu.pcd", k);
    tc::Cloud b = load_pcd(out + name), o;
    o.reserve(b.size());
    for (auto& p : b) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      Eigen::Vector3d q = Rk[k] * Eigen::Vector3d(p.x, p.y, p.z) + pk[k];
      o.push_back({q.x(), q.y(), q.z()});
    }
    return o;
  }
};

FILE* dumpf = nullptr;
json pjson(const tc::Path& p) {
  json a = json::array();
  for (auto& v : p) a.push_back({v.x(), v.y()});
  return a;
}
void dump(const std::string& label, const std::string& mode, long k, const V2& s, const V2& g,
          std::optional<double> yaw, const tc::PlanResult& out) {
  if (!dumpf) return;
  json j;
  j["label"] = label;
  j["mode"] = mode;
  j["k"] = k;
  j["s"] = {s.x(), s.y()};
  j["g"] = {g.x(), g.y()};
  j["yaw"] = yaw ? json(*yaw) : json(nullptr);
  j["ok"] = out.ok;
  j["reason"] = out.reason;
  if (out.ok) {
    j["path"] = pjson(out.path);
    json pv = json::array();
    for (auto& p : out.pivots) pv.push_back({p.i, p.centre.x(), p.centre.y(), p.yaw_in, p.yaw_out, p.start});
    j["pivots"] = pv;
    if (out.reverse) j["reverse"] = {out.reverse->to.x(), out.reverse->to.y(), out.reverse->dist};
  }
  fprintf(dumpf, "%s\n", j.dump().c_str());
}

// the live grid (raw and corrected) at keyframe L4_GRID_K of map L4_GRID_LABEL, as JSON (A/B diagnosis)
void grid_dump(const std::string& label, long k, double dist, const tc::LiveWindow& lw, const V2& s, const V2& g,
               const tc::PlanResult& out) {
  const char* L = std::getenv("L4_GRID_LABEL");
  const char* K = std::getenv("L4_GRID_K");
  if (!L || !K || label != L || k != std::atol(K) || !out.hg) return;
  const tc::Config& c = *lw.cfg;
  double m = c.r_out + c.res * 2 + c.pad;
  V2 lo = s.cwiseMin(g).array() - m, hi = s.cwiseMax(g).array() + m;
  tc::Cloud pts;
  for (auto& q : lw.points())
    if (q.x >= lo.x() && q.y >= lo.y() && q.x <= hi.x() && q.y <= hi.y()) pts.push_back({q.x, q.y, q.z});
  tc::HeightGrid raw(pts, lw.cfg, lo, hi);
  auto& hg = *out.hg;
  json j;
  j["lo"] = {hg.lo.x(), hg.lo.y()};
  j["shape"] = {hg.ny, hg.nx};
  j["npts"] = pts.size();
  j["raw_occ"] = raw.occ;
  j["ground"] = hg.ground;
  j["occ"] = hg.occ;
  j["known"] = hg.known;
  j["seen_free"] = hg.seen_free;
  j["clear"] = hg.clear;
  // the overhang inputs: low voxels per cell, the evidence memory's channels now
  j["low"] = hg.low;
  j["seen_up"] = hg.seen_up;
  if (lw.mem && !lw.recent.empty()) {
    std::vector<double> h, mm, w, u, l;
    lw.mem->read(hg, lw.recent.back().t, h, mm, w, u, l);
    j["mem_h"] = h;
    j["mem_m"] = mm;
    j["mem_w"] = w;
    j["mem_u"] = u;
    j["mem_l"] = l;
  }
  std::ofstream(fmt("grid_cpp_%s_%ld_%g.json", label.c_str(), k, dist)) << j.dump();
}

tc::Path past_start(const tc::Path& path, const tc::Config& cfg) {
  tc::Path P = tc::densify(path, 0.05), far;
  for (auto& p : P)
    if (std::hypot(p.x() - P[0].x(), p.y() - P[0].y()) >= cfg.start_skip) far.push_back(p);
  return far.empty() ? tc::Path{P.back()} : far;
}

class RealMap {
 public:
  std::string label;
  tc::Cloud P;
  std::unique_ptr<Keyframes> kf;
  tc::Path D;
  std::vector<double> Dz, S;
  std::vector<V2> ref_xy;

  RealMap(const std::string& label, const std::string& spec) : label(label) {
    std::map<std::string, std::string> kv;
    std::istringstream ss(spec);
    std::string item;
    while (std::getline(ss, item, ',')) {
      auto e = item.find('=');
      kv[item.substr(0, e)] = item.substr(e + 1);
    }
    std::vector<Eigen::Vector3d> path;
    if (kv.count("view")) {
      load_view(kv["view"], &P, &path);
    } else {
      P = load_pcd(kv["pcd"]);
      if (kv.count("path")) {
        for (auto& r : load_txt(kv["path"], ',', 1)) path.push_back(Eigen::Vector3d(r[1], r[2], r[3]));
      } else {
        load_view(kv["meta"], nullptr, &path);
      }
    }
    if (kv.count("kf")) kf = std::make_unique<Keyframes>(kv["kf"], path);
    // A/B: the keyframe poses as real_maps.py computed them (numpy's SVD differs from Eigen's at 1e-15)
    if (kf && std::getenv("L4_KF_POSES")) {
      json j = json::parse(slurp(std::string(std::getenv("L4_KF_POSES")) + "/" + label + ".json"));
      for (size_t k = 0; k < kf->size(); ++k) {
        for (int a = 0; a < 3; ++a) {
          kf->pk[k](a) = j["pk"][k][a].get<double>();
          for (int b = 0; b < 3; ++b) kf->Rk[k](a, b) = j["Rk"][k][a][b].get<double>();
        }
      }
    }
    tc::Path p2;
    for (auto& p : path) p2.push_back(p.head<2>());
    D = tc::densify(p2, 0.05);
    // np.interp(np.linspace(0, len(path) - 1, len(D)), np.arange(len(path)), path z)
    auto x = tc::linspace(0, double(path.size() - 1), int(D.size()));
    for (double v : x) {
      int i = std::min(int(std::floor(v)), int(path.size()) - 2);
      if (path.size() < 2) {
        Dz.push_back(path[0].z());
        continue;
      }
      double f = v - i;
      Dz.push_back(f == 0 ? path[i].z() : path[i].z() + (path[i + 1].z() - path[i].z()) * f);
    }
    S.push_back(0);
    for (size_t i = 1; i < D.size(); ++i) S.push_back(S.back() + tc::norm2(D[i] - D[i - 1]));
  }

  int search_left(double v) const { return int(std::lower_bound(S.begin(), S.end(), v) - S.begin()); }

  std::vector<uint8_t> ref_walls(const tc::HeightGrid& hg, const std::vector<int>* hits, const std::vector<int>* miss) {
    double r = hg.cfg->res;
    std::vector<uint8_t> m(size_t(hg.ny) * hg.nx, 1);
    std::vector<std::vector<uint8_t>> g(4, std::vector<uint8_t>(m.size(), 0));
    const double los[4] = {0.3, 0.6, 0.9, 1.2};
    for (auto& p : P) {
      long j = long(std::floor((p.x - hg.lo.x()) / r)), i = long(std::floor((p.y - hg.lo.y()) / r));
      if (j < 0 || i < 0 || j >= hg.nx || i >= hg.ny) continue;
      double h = p.z - hg.ground[i * hg.nx + j];
      for (int b = 0; b < 4; ++b)
        if (h >= los[b] && h < los[b] + 0.3) g[b][i * hg.nx + j] = 1;
    }
    for (size_t q = 0; q < m.size(); ++q) {
      m[q] = g[0][q] && g[1][q] && g[2][q] && g[3][q];
      if (hits) m[q] = m[q] && (*hits)[q] >= 3 && (*miss)[q] <= (*hits)[q];
    }
    return m;
  }

  struct Walls {
    int pairs_ok = 0, pairs_n = 0, behind_ok = 0, behind_n = 0, routed = 0;
    std::vector<std::string> fails;
  };

  Walls walls(std::shared_ptr<tc::HeightGrid> hg, const std::vector<int>* hits, const std::vector<int>* miss, double low,
              size_t n = 40) {
    Walls w;
    auto wm = ref_walls(*hg, hits, miss);
    std::vector<V2> wxy;
    for (int i = 0; i < hg->ny; ++i)
      for (int j = 0; j < hg->nx; ++j)
        if (wm[size_t(i) * hg->nx + j]) wxy.push_back(hg->xy({i, j}));
    if (wxy.empty()) return w;
    KD tw(wxy);
    auto judge = [&](const V2& s, const V2& g, bool& routed) {
      auto out = tc::plan(hg, s, g);
      dump(label, "wall", -1, s, g, std::nullopt, out);
      routed = false;
      if (!out.ok) return std::make_pair(true, "refused: " + out.reason);
      double dmin = 1e300;
      for (auto& p : past_start(out.path, *hg->cfg)) dmin = std::min(dmin, tw.nearest(p).first);
      routed = true;
      return std::make_pair(dmin >= 0.2, fmt("routed %.1f m, %.2f m from a wall", out.length, dmin));
    };
    std::vector<long> ar;
    for (long i = 0; i < long(D.size()); i += 10) ar.push_back(i);
    KD tree(D);
    std::vector<std::pair<V2, V2>> pairs;
    for (long i : permutation(0, ar)) {
      for (int j : tree.ball(D[i], 4.0)) {
        double e = tc::norm2(D[j] - D[i]);
        if (e < 1.0 || std::abs(S[j] - S[i]) < 3 * e + 2 || Dz[i] > low || Dz[j] > low) continue;
        bool hit = false;
        for (double t : tc::linspace(0, 1, int(e / 0.05) + 2)) {
          V2 p = D[i] + (D[j] - D[i]) * t;
          auto c = hg->cell(p);
          if (hg->inside(c) && wm[size_t(c.first) * hg->nx + c.second]) {
            hit = true;
            break;
          }
        }
        if (hit) {
          pairs.push_back({D[i], D[j]});
          break;
        }
      }
      if (pairs.size() >= n) break;
    }
    std::vector<std::pair<V2, V2>> behind;
    for (long i : permutation(1, ar)) {
      if (Dz[i] > low) continue;
      auto [d, k] = tw.nearest(D[i]);
      if (d >= 0.4 && d <= 2.5) behind.push_back({D[i], wxy[k] + (wxy[k] - D[i]) / d * 1.0});
      if (behind.size() >= n) break;
    }
    for (auto& [s, g] : pairs) {
      bool r;
      auto [ok, why] = judge(s, g, r);
      w.pairs_ok += ok;
      if (!ok) w.fails.push_back(fmt("('pair', [%.3f, %.3f], [%.3f, %.3f], '%s')", s.x(), s.y(), g.x(), g.y(), why.c_str()));
    }
    w.pairs_n = int(pairs.size());
    for (auto& [s, g] : behind) {
      bool r;
      auto [ok, why] = judge(s, g, r);
      w.behind_ok += ok;
      w.routed += ok && r;
      if (!ok)
        w.fails.push_back(fmt("('behind', [%.3f, %.3f], [%.3f, %.3f], '%s')", s.x(), s.y(), g.x(), g.y(), why.c_str()));
    }
    w.behind_n = int(behind.size());
    return w;
  }

  struct MapRes {
    std::vector<std::pair<double, std::pair<int, int>>> plans;
    std::vector<std::string> fails;
    int dropped = 0;
    Stats time;
    Walls walls;
  };

  MapRes map_mode(double low = 9.0) {
    const double dists[4] = {2, 5, 10, 20};
    tc::Config mc;
    ab_env(mc);
    auto cfg = std::make_shared<const tc::Config>(mc);
    MapRes res;
    auto hg = std::make_shared<tc::HeightGrid>(P, cfg);
    std::vector<int> hits, miss;
    bool ev = false;
    if (kf) {
      // the keyframes' free-space evidence, the band-level model (evidence(), whole maps), one scan
      // at a time (a whole run's scans do not fit in memory at once)
      hits.assign(size_t(hg->ny) * hg->nx, 0);
      miss.assign(hits.size(), 0);
      for (size_t k = 0; k < kf->size(); ++k) {
        tc::Evidence e = tc::scan_evidence(*hg, kf->scan(k), kf->pk[k], false, false);
        for (int c : e.hit) hits[c]++;
        for (int c : e.free) miss[c]++;
      }
      res.dropped = hg->apply_evidence(hits, miss);
      ev = true;
    }
    std::vector<double> times;
    int stepi = int(2.0 / 0.05);
    for (double dist : dists) {
      int ok = 0, n = 0;
      for (long i = 0; i < long(D.size()); i += stepi) {
        int j = search_left(S[i] + dist);
        if (j >= int(D.size()) || Dz[i] > low || Dz[j] > low) continue;
        double t0 = now_s();
        auto out = tc::plan(hg, D[i], D[j]);
        times.push_back(now_s() - t0);
        dump(label, "map", i, D[i], D[j], std::nullopt, out);
        n++;
        ok += out.ok;
        if (!out.ok) res.fails.push_back(fmt("fail at %.0f m along, goal %g m ahead: %s", S[i], dist, out.reason.c_str()));
      }
      res.plans.push_back({dist, {ok, n}});
    }
    res.time = stats_ms(times);
    res.walls = walls(hg, ev ? &hits : nullptr, ev ? &miss : nullptr, low);
    auto ref = ref_walls(*hg, ev ? &hits : nullptr, ev ? &miss : nullptr);
    ref_xy.clear();
    for (int i = 0; i < hg->ny; ++i)
      for (int j = 0; j < hg->nx; ++j)
        if (ref[size_t(i) * hg->nx + j])
          ref_xy.push_back(V2(hg->lo.x() + (j + 0.5) * cfg->res, hg->lo.y() + (i + 0.5) * cfg->res));
    return res;
  }

  struct LiveRes {
    std::map<double, std::pair<int, int>> plans, free_now;
    Stats time, ingest;
    std::vector<std::string> fails, wall_fails;
    int wall_ok = 0, wall_n = 0, wall_routed = 0, wall_unseen = 0;
    int npiv[3] = {0, 0, 0};
  };

  std::optional<LiveRes> live_mode(int live_step = 2, double window_s = 3.0, double low = 9.0) {
    if (!kf) return std::nullopt;
    const double goals[2] = {3.0, 6.0};
    tc::Config cfg = tc::live_config();
    // A/B switches for the arc check (the node's ~arc_check, ~arc_vertices)
    if (std::getenv("L4_ARC")) cfg.arc_check = std::atoi(std::getenv("L4_ARC")) != 0;
    if (std::getenv("L4_ARC_VERTICES")) cfg.arc_vertices = std::atoi(std::getenv("L4_ARC_VERTICES")) != 0;
    ab_env(cfg);
    LiveRes res;
    KD Dk(D);
    tc::LiveWindow lw(cfg, window_s);
    std::vector<double> ing, times;
    std::unique_ptr<KD> tw;
    if (!ref_xy.empty()) tw = std::make_unique<KD>(ref_xy);
    for (double d : goals) {
      res.plans[d] = {0, 0};
      res.free_now[d] = {0, 0};
    }
    for (size_t k = 0; k < kf->size(); ++k) {
      tc::Cloud scan = kf->scan(k);
      tc::CloudF sf(scan.size());
      for (size_t q = 0; q < scan.size(); ++q) sf[q] = {float(scan[q].x), float(scan[q].y), float(scan[q].z)};
      double t0 = now_s();
      lw.add_scan(kf->t[k] - kf->t[0], sf, kf->pk[k]);
      ing.push_back(now_s() - t0);
      if (k % live_step) continue;
      V2 pk2 = kf->pk[k].head<2>();
      int i = Dk.nearest(pk2).second;
      // the robot's heading: the nose (FAST-LIO body +z) in the map frame, for the start pivot
      Eigen::Vector3d nose3 = kf->Rk[k] * Eigen::Vector3d(0, 0, 1);
      V2 nose = nose3.head<2>();
      double yaw = std::atan2(nose.y(), nose.x());
      for (double dist : goals) {
        int j = search_left(S[i] + dist);
        if (j >= int(D.size()) || Dz[i] > low || Dz[j] > low) continue;
        t0 = now_s();
        auto out = lw.plan(pk2, D[j], yaw);
        times.push_back(now_s() - t0);
        grid_dump(label, long(k), dist, lw, pk2, D[j], out);
        dump(label, "live", long(k), pk2, D[j], yaw, out);
        res.plans[dist].second++;
        res.plans[dist].first += out.ok;
        if (out.ok && !out.pivots.empty()) {
          res.npiv[0]++;
          for (auto& p : out.pivots) res.npiv[1] += p.start;
          res.npiv[2] += int(out.pivots.size());
        }
        // free now: no band return of this very scan within the frame's half-width of the driven
        // stretch (the path is known free when the robot drove it, not before)
        if (out.hg) {
          auto& hg = *out.hg;
          std::vector<V2> bq;
          for (auto& q : lw.recent.back().p) {
            long cj = long(std::floor((double(q.x) - hg.lo.x()) / hg.cfg->res)),
                 ci = long(std::floor((double(q.y) - hg.lo.y()) / hg.cfg->res));
            if (cj < 0 || ci < 0 || cj >= hg.nx || ci >= hg.ny) continue;
            double hq = double(q.z) - hg.ground[ci * hg.nx + cj];
            if (hq > hg.cfg->z_lo && hq < hg.cfg->z_hi) bq.push_back(V2(q.x, q.y));
          }
          std::vector<V2> seg;
          for (int q = i; q < std::min(j + 1, int(D.size())); q += 2)
            if (std::hypot(D[q].x() - pk2.x(), D[q].y() - pk2.y()) > hg.cfg->start_skip) seg.push_back(D[q]);
          bool busy = false;
          if (!bq.empty() && !seg.empty()) {
            KD tb(bq);
            double dm = 1e300;
            for (auto& p : seg) dm = std::min(dm, tb.nearest(p).first);
            busy = dm < tc::FOOTPRINT_WID / 2;
          }
          if (!busy) {
            res.free_now[dist].second++;
            res.free_now[dist].first += out.ok;
          }
        }
        if (!out.ok) res.fails.push_back(fmt("fail at keyframe %zu, goal %g m ahead: %s", k, dist, out.reason.c_str()));
      }
      // a goal 1 m behind the nearest reference wall ahead of the nose (0.4-2.5 m away)
      if (tw) {
        int best = -1;
        double bd = 1e300;
        for (int c : tw->ball(pk2, 2.5)) {
          V2 v = ref_xy[c] - pk2;
          double nv = tc::norm2(v);
          if (nv >= 0.4 && v.dot(nose) >= 0.5 * nv * tc::norm2(nose) && nv < bd) {
            bd = nv;
            best = c;
          }
        }
        if (best >= 0) {
          V2 u = (ref_xy[best] - pk2) / bd;
          auto out = lw.plan(pk2, ref_xy[best] + u, yaw);
          grid_dump(label, long(k), -1.0, lw, pk2, ref_xy[best] + u, out);
          dump(label, "live_wall", long(k), pk2, ref_xy[best] + u, yaw, out);
          res.wall_n++;
          if (!out.ok) {
            res.wall_ok++;
          } else {
            // only walls the window has seen count: unknown space is driven through by design
            auto& hl = *out.hg;
            bool hit_seen = false, near_unseen = false;
            double dseen = 1e300;
            for (auto& p : past_start(out.path, cfg)) {
              auto [d, ri] = tw->nearest(p);
              auto c = hl.cell(ref_xy[ri]);
              bool seen = hl.inside(c) && hl.seen_up[size_t(c.first) * hl.nx + c.second];
              if (d < 0.2 && seen) {
                hit_seen = true;
                dseen = std::min(dseen, d);
              }
              if (d < 0.2 && !seen) near_unseen = true;
            }
            res.wall_ok += !hit_seen;
            res.wall_routed += !hit_seen;
            res.wall_unseen += near_unseen && !hit_seen;
            if (hit_seen) {
              // real_maps.py: d[seen].min() over all path points whose nearest wall the window had seen
              double ds = 1e300;
              for (auto& p : past_start(out.path, cfg)) {
                auto [d, ri] = tw->nearest(p);
                auto c = hl.cell(ref_xy[ri]);
                if (hl.inside(c) && hl.seen_up[size_t(c.first) * hl.nx + c.second]) ds = std::min(ds, d);
              }
              res.wall_fails.push_back(fmt("wall fail at keyframe %zu: path %.2f m from a seen wall", k, ds));
            }
          }
        }
      }
    }
    res.time = stats_ms(times);
    res.ingest = stats_ms(ing);
    return res;
  }
};

void real(const std::string& label, const std::string& spec, int live_step, bool gate) {
  RealMap m(label, spec);
  if (m.kf) printf("%s: %zu keyframes, fit to the path rms %.3f m\n", label.c_str(), m.kf->size(), m.kf->rms);
  auto r = m.map_mode();
  int planned = 0, total = 0;
  std::string per;
  for (auto& [d, v] : r.plans) {
    planned += v.first;
    total += v.second;
    per += (per.empty() ? "" : ", ") + fmt("%g m %d/%d", d, v.first, v.second);
  }
  check(label + " map: goals along the driven path planned", total && planned >= 0.9 * total,
        fmt("%d/%d (%s); %d moved cells dropped; plan median %.0f ms p95 %.0f ms", planned, total, per.c_str(), r.dropped,
            r.time.median, r.time.p95));
  for (size_t q = 0; q < std::min<size_t>(5, r.fails.size()); ++q) printf("        %s\n", r.fails[q].c_str());
  auto& w = r.walls;
  check(label + " map: walls refused or routed around", w.pairs_ok == w.pairs_n && w.behind_ok == w.behind_n,
        fmt("pairs %d/%d; goals behind a wall %d/%d (%d routed around)", w.pairs_ok, w.pairs_n, w.behind_ok, w.behind_n,
            w.routed));
  for (size_t q = 0; q < std::min<size_t>(5, w.fails.size()); ++q) printf("        wall fail: %s\n", w.fails[q].c_str());
  auto lv = m.live_mode(live_step);
  if (!lv) return;
  planned = total = 0;
  per.clear();
  int pf = 0, tf = 0;
  for (auto& [d, v] : lv->plans) {
    planned += v.first;
    total += v.second;
    per += (per.empty() ? "" : ", ") + fmt("%g m %d/%d", d, v.first, v.second);
  }
  for (auto& [d, v] : lv->free_now) {
    pf += v.first;
    tf += v.second;
  }
  check(label + " live window: goals along the driven path planned", total && planned >= 0.85 * total,
        fmt("%d/%d (%s); where the scan of the moment shows the driven stretch free: %d/%d; plans with a pivot %d (start "
            "pivots %d, pivots %d)",
            planned, total, per.c_str(), pf, tf, lv->npiv[0], lv->npiv[1], lv->npiv[2]));
  for (size_t q = 0; q < std::min<size_t>(8, lv->fails.size()); ++q) printf("        %s\n", lv->fails[q].c_str());
  check(label + " live window: goal behind a seen wall ahead refused or routed around", lv->wall_ok == lv->wall_n,
        fmt("%d/%d (%d routed around; %d of those through space the window had not seen, nothing seen there from z_lo to "
            "2 m, where the map has a wall)",
            lv->wall_ok, lv->wall_n, lv->wall_routed, lv->wall_unseen));
  for (size_t q = 0; q < std::min<size_t>(5, lv->wall_fails.size()); ++q) printf("        %s\n", lv->wall_fails[q].c_str());
  auto& t = lv->time;
  auto& ig = lv->ingest;
  std::string detail = fmt("replan median %.0f ms, p95 %.0f ms, max %.0f ms (n=%d); scan ingest median %.0f ms, p95 %.0f ms",
                           t.median, t.p95, t.max, t.n, ig.median, ig.p95);
  if (gate)
    check(label + " live window: replan p95 <= 100 ms (Gate L1, 10 Hz)", t.p95 <= 100.0, detail);
  else
    printf("      %s\n", detail.c_str());
  // the replan times themselves (for the report's median / p95 over all maps)
  printf("      %s replan times ms: median %.1f p95 %.1f; map plans median %.1f p95 %.1f\n", label.c_str(), t.median, t.p95,
         r.time.median, r.time.p95);
}
}  // namespace

int main(int argc, char** argv) {
  std::vector<std::string> args(argv + 1, argv + argc);
  int step = 2;
  bool gate = false;
  for (size_t i = 0; i < args.size(); ++i) {
    if (args[i] == "--live-step") step = std::stoi(args[i + 1]);
    if (args[i] == "--gate") gate = true;
    if (args[i] == "--dump") dumpf = fopen(args[i + 1].c_str(), "w");
    if (args[i] == "--grid-plan") {   // --grid-plan GRID.json sx sy gx gy yaw: one live plan on a dumped grid (A/B)
      json j = json::parse(slurp(args[i + 1]));
      tc::Config lc = tc::live_config();
      if (std::getenv("L4_ARC")) lc.arc_check = std::atoi(std::getenv("L4_ARC")) != 0;
      auto cfg = std::make_shared<const tc::Config>(lc);
      auto hg = std::make_shared<tc::HeightGrid>(tc::Cloud{}, cfg, V2(0, 0), V2(1, 1), 0.0);
      hg->lo = V2(j["lo"][0].get<double>(), j["lo"][1].get<double>());
      hg->ny = j["shape"][0];
      hg->nx = j["shape"][1];
      hg->ground = j["ground"].get<std::vector<double>>();
      hg->occ = j["occ"].get<std::vector<uint8_t>>();
      hg->known = j["known"].get<std::vector<uint8_t>>();
      hg->seen_free = j["seen_free"].get<std::vector<uint8_t>>();
      hg->floor_seen.assign(hg->occ.size(), 0);
      hg->edt();
      V2 s(std::stod(args[i + 2]), std::stod(args[i + 3])), g(std::stod(args[i + 4]), std::stod(args[i + 5]));
      double yaw = std::stod(args[i + 6]);
      auto win = hg->crop(s.cwiseMin(g).array() - cfg->pad, s.cwiseMax(g).array() + cfg->pad);
      auto sc = win->nearest_free(win->cell(s), cfg->snap_radius), gc = win->nearest_free(win->cell(g), cfg->snap_radius);
      if (sc && gc) {
        auto cells = tc::search(*win, *sc, *gc);
        printf("search:");
        if (cells)
          for (auto& c : *cells) printf(" %d,%d", c.first, c.second);
        printf("\n");
      }
      auto out = tc::plan(hg, s, g, yaw);
      printf("ok %d reason %s attempts %d\npath:", out.ok, out.reason.c_str(), out.attempts);
      for (auto& p : out.path) printf(" %.6f,%.6f", p.x(), p.y());
      printf("\nroute:");
      for (auto& p : out.route) printf(" %.6f,%.6f", p.x(), p.y());
      printf("\n");
      return 0;
    }
    if (args[i] == "--perm-check") {   // numpy RandomState(seed).permutation(arange(n)) for verification
      std::vector<long> a(std::stol(args[i + 2]));
      std::iota(a.begin(), a.end(), 0);
      for (long v : permutation(uint32_t(std::stol(args[i + 1])), a)) printf("%ld ", v);
      printf("\n");
      return 0;
    }
  }
  for (size_t i = 0; i < args.size(); ++i)
    if (args[i] == "--map") {
      auto e = args[i + 1].find(':');
      real(args[i + 1].substr(0, e), args[i + 1].substr(e + 1), step, gate);
    }
  if (dumpf) fclose(dumpf);
  int passed = int(std::count(results.begin(), results.end(), true));
  printf("%d of %zu checks passed\n", passed, results.size());
  return std::all_of(results.begin(), results.end(), [](bool b) { return b; }) ? 0 : 1;
}
