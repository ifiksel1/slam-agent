// Track planner core (ROS-free): plan driving legs for the tracked QuadRover on a 2.5-D height grid.
//
// C++ port of planner/track_core.py (2026-10-29, owner decision: no Python in the robot software
// unless required). Behaviour, names and defaults are the Python module's; the long description of
// the model (height grid, moved objects, footprint, pivots, dead ends, the run-out) and its LIMITS
// are in track_core.cpp's header comment.
//
// Conventions: a grid cell (i, j) is row i (y), column j (x); a cell's lower-left corner is
// lo + (j, i) * res. Angles in radians unless a name ends in _deg.
#pragma once

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace l4_track {

using V2 = Eigen::Vector2d;
using Path = std::vector<V2>;

struct P3 {
  double x, y, z;
};
struct P3f {
  float x, y, z;
};
using Cloud = std::vector<P3>;
using CloudF = std::vector<P3f>;

constexpr double FOOTPRINT_LEN = 0.508;  // length, width (m), as landing_core
constexpr double FOOTPRINT_WID = 0.457;
extern const double FOOTPRINT_HALF_DIAGONAL;  // 0.342 m
// The odometry point (FAST-LIO /Odometry = the JT128's built-in IMU) ahead of the frame centre
// (owner-measured 0.184 m + the IMU origin 8.68 mm above the lidar mounting face) = 0.193 m.
constexpr double ODOM_AHEAD = 0.193;
constexpr double ROBOT_HEIGHT = 0.203;
constexpr int N_HEADINGS = 64;

// numpy / Python semantics used by the port
double pymod(double a, double b);             // Python float % (sign of the divisor)
double wrap(double a);                        // (a + pi) % 2pi - pi
double radians(double deg);                   // math.radians
double degrees(double rad);                   // math.degrees
double pyround(double x, int nd);             // round(x, nd): correctly rounded, half to even
std::vector<double> arange(double start, double stop, double step);   // np.arange
std::vector<double> linspace(double a, double b, int n);              // np.linspace
double percentile(std::vector<double> v, double q_pct);               // np.percentile, linear
double median(std::vector<double> v);                                 // np.median
inline double hd(const V2& v) { return std::atan2(v.y(), v.x()); }
inline V2 unit(double a) { return V2(std::cos(a), std::sin(a)); }
inline double norm2(const V2& v) { return std::sqrt(v.x() * v.x() + v.y() * v.y()); }   // np.linalg.norm

struct Kernels {
  int R = 0;
  std::array<std::vector<std::pair<int, int>>, N_HEADINGS> ks;
};

// Config: the base parameters (as track_core.Config's arguments) and the derived ones; call
// update() after changing a base parameter (the constructor does).
struct Config {
  double res = 0.1, margin = 0.05, off = ODOM_AHEAD, z_lo = 0.30, z_hi = 0.60;
  int min_pts = 3;
  double z_tall = 2.0;
  int min_tall = 4, min_tall_only = 8;
  double mem_min_range = 1.0, tile = 0.5, furniture_step = 0.15, max_ground_step = 0.5, ground_pct = 20.0;
  double prefer_clear = 0.8, clear_weight = 2.0, pivot_weight = 1.0;
  bool unknown_free = false;
  double unknown_cost = 1.0, known_radius = 0.35, pad = 3.0, lookahead = 0.5, snap_radius = 0.5;
  int ray_bins = 720;
  double ray_range = 8.0;
  int clear_misses = 2;
  double clear_ratio = 1.0;
  int retries = 2;
  double start_skip = 0.5, start_skip_heading = 0.5;
  double pivot_angle_deg = 60.0, pivot_done_deg = 10.0;
  double pivot_tol = 0.2, pivot_tol_start = 0.2, pivot_search = 0.6;
  int pivot_tries = 12, pivot_retries = 2;
  double pivot_margin = 0.05, reverse_max = 2.0;
  int reverse_tries = 4;
  // the arc check (owner request 2026-10-29, after the Gate L2 door-jamb clips c1/c3): the robot's own
  // arc onto each driving leg from its actual heading, simulated up to arc_max along the leg; its frame
  // (plus margin) must stay clear (see arc_fail, pivot_pass). Rover steers toward the node's target
  // (the carrot run out by up to arc_reach: the node's runout_s x drive_speed) with a curvature of
  // 2 sin(error) / max(distance, arc_steer): arc_steer 3.0 m fits SITL's c1, c2, c3 and d18 drives
  bool arc_check = true;
  double arc_steer = 3.0, arc_reach = 2.1, arc_max = 6.0;
  // the arc check's frame margin: the bare frame by default (the arc check guards against contact; the
  // path keeps the driving margin through the follower check)
  double arc_margin = 0.0;
  // a robot already heading along its first leg (within pivot_done) is committed to the next
  // arc_commit metres of it: failures there are not acted on (earlier replans checked that stretch)
  double arc_commit = 1.0;
  // a leg after a pivot whose arc fails on a gentle turn: a pivot at that vertex (else refused). Off:
  // on the house drive it made routes the old planner rejected viable, one 0.18 m from a wall cell
  // (the gate's 0.2 m wall check failed, house_current keyframe 62)
  bool arc_vertices = false;
  // derived (update())
  double half_len = 0, half_wid = 0, r_in = 0, r_hard = 0, r_out = 0, inflate = 0;
  double pivot_angle = 0, pivot_done = 0, pivot_radius = 0;
  std::shared_ptr<const Kernels> kern;

  Config() { update(); }
  void update();
};
using ConfigPtr = std::shared_ptr<const Config>;

// the live window's defaults (live_config()): z_lo 0.15, min_pts 2, unknown space allowed, ...
Config live_config();

CloudF voxel_down(const CloudF& p, double vox);

class Window;

// ground, obstacles, clearance and known space on a res grid over [lo, hi] (xy)
class HeightGrid {
 public:
  HeightGrid(const Cloud& points, ConfigPtr cfg, std::optional<V2> lo = std::nullopt,
             std::optional<V2> hi = std::nullopt, std::optional<double> floor_z = std::nullopt,
             bool ground_only = false);
  ConfigPtr cfg;
  V2 lo;
  int ny = 0, nx = 0;
  std::vector<double> cellmin, ground, clear;
  std::vector<int> count;
  std::vector<uint8_t> occ, seen_up, floor_seen, known, seen_free;   // seen_free empty: none
  // per cell: its tile has a floor level of its own; 0: no floor seen there (no points, or only a ceiling
  // or wall top), its ground is the nearest such tile's, a stand-in (empty: all seen, a fixed floor_z)
  std::vector<uint8_t> ground_seen;
  int idx(int i, int j) const { return i * nx + j; }
  std::pair<int, int> cell(const V2& xy) const;
  V2 xy(std::pair<int, int> c) const;
  bool inside(std::pair<int, int> c) const { return c.first >= 0 && c.first < ny && c.second >= 0 && c.second < nx; }
  double clearance_at(const V2& xy) const;
  double ground_at(const V2& xy) const;
  int apply_evidence(const std::vector<int>& hits, const std::vector<int>& misses);
  void edt();
  std::shared_ptr<Window> crop(const V2& lo, const V2& hi) const;

 private:
  void ground_model(int m);
};

// the local ground model alone (stable API, also used by the landing-spot check): floor height per
// cell, row = y, column = x, cell (i, j)'s lower-left corner at lo + (j, i) * res
struct Ground {
  std::vector<double> g;
  int ny = 0, nx = 0;
  V2 lo;
  double res = 0.1;
  double at(double x, double y) const;   // clamped to the grid (landing_core's lookup)
};
Ground local_ground(const Cloud& points, const Config& cfg, std::optional<V2> lo = std::nullopt,
                    std::optional<V2> hi = std::nullopt);

// the search box, a slice of a HeightGrid
class Window {
 public:
  Window(const HeightGrid& hg, int i0, int i1, int j0, int j1);
  ConfigPtr cfg;
  V2 lo;
  int ny = 0, nx = 0;
  std::vector<uint8_t> occ, known, free;
  std::vector<double> clear;
  std::vector<uint8_t> occ_big;
  int pad = 0, bny = 0, bnx = 0;
  std::optional<V2> start;
  double skip = 0.5;
  std::optional<V2> fail_at;

  std::pair<int, int> cell(const V2& xy) const;
  V2 xy(std::pair<int, int> c) const;
  bool inside(std::pair<int, int> c) const { return c.first >= 0 && c.first < ny && c.second >= 0 && c.second < nx; }
  double clearance_at(const V2& xy) const;
  bool is_free(std::pair<int, int> c) const { return free[c.first * nx + c.second] != 0; }
  std::optional<std::pair<int, int>> nearest_free(std::pair<int, int> c, double radius_m) const;
  std::vector<uint8_t> poses_free(const std::vector<V2>& xy, const std::vector<double>& yaw) const;
  bool pose_free(const V2& xy, double yaw) const;
  // the obstacle cells (indices into occ_big) inside the frame plus margin at that pose, sorted
  std::vector<int> pose_hits(const V2& xy, double yaw, double margin = -1.0) const;   // margin < 0: cfg.margin
  std::vector<uint8_t> discs_free(const std::vector<V2>& xy, double radius = -1.0) const;
  bool pivot_free(const V2& centre, double yaw_in, double tol = -1.0, bool back = true) const;
  std::optional<int> follow_free(const Path& pts, int k0 = 0);
};

struct Evidence {
  std::vector<int> hit, free, wall;   // flat cell indices, sorted, unique
};
Evidence scan_evidence(const HeightGrid& hg, const Cloud& pts, const Eigen::Vector3d& sensor, bool walls = false,
                       bool low_sensor = true);
// hit and seen-free counts per cell over scans (whole maps: the band-level model by default)
void evidence(const HeightGrid& hg, const std::vector<std::pair<Cloud, Eigen::Vector3d>>& scans,
              std::vector<int>& hits, std::vector<int>& miss, bool low_sensor = false);

std::optional<std::vector<std::pair<int, int>>> search(const Window& win, std::pair<int, int> start,
                                                       std::pair<int, int> goal);
std::pair<Path, std::optional<int>> prune(Window& win, const Path& pts);

struct Pivot {
  int i = 0;
  V2 centre = V2::Zero();
  double yaw_in = 0, yaw_out = 0, turn = 0;
  bool start = false;
};

struct Block {
  enum Kind { SEGMENT, VERTEX, START } kind = SEGMENT;
  Path seg;                 // SEGMENT: the segment
  std::optional<V2> at;     // SEGMENT: its first blocked pose
  V2 p = V2::Zero();        // VERTEX / START: the point
  double yaw = 0;           // START: the heading
};

struct PivotResult {
  bool ok = true;
  Path path;
  std::vector<Pivot> pivots;
  std::string reason;
  Block block;
  bool retry = true;   // false: no other search (an arc onto the path that no pivot can fix)
};

V2 point_along(const Path& pts, double dist);
std::vector<int> pivot_vertices(const Path& P, const Config& cfg);
// the robot's arc onto leg from (p0, yaw0) (see Config::arc_check): the first pose whose frame plus margin
// holds an obstacle cell the start pose's frame does not -> (that odometry point, the leg segment index)
struct ArcFail {
  V2 at;
  int seg = 0;
  double s = 0;   // distance the FC travelled to it, m
};
std::optional<ArcFail> arc_fail(const Window& win, const Path& leg, const V2& p0, double yaw0);

PivotResult pivot_pass(Window& win, const Path& path, std::optional<double> start_yaw = std::nullopt,
                       bool start_pivoting = false);
std::optional<Path> sharpen(Window& win, const Path& route, int bad, int max_skip = 4);

struct Reverse {
  V2 from, to;
  double dist = 0, yaw = 0;
};

struct PlanResult {
  bool ok = false;
  std::string reason;
  Path path, route;
  std::vector<Pivot> pivots;
  std::shared_ptr<Window> grid;
  std::shared_ptr<HeightGrid> hg;   // LiveWindow::plan: the corrected grid
  double length = 0, min_clearance = 1e3;
  int attempts = 0;
  std::optional<Reverse> reverse;
  std::string reverse_reason;
  double t_search = 0, t_prune = 0, t_pivot = 0, t_grid = 0, t_evidence = 0;   // s
};

extern const char* const REVERSE_REASONS[2];

// plan on a grid (a prior map, built once, or LiveWindow's)
PlanResult plan(std::shared_ptr<HeightGrid> hg, const V2& start, const V2& goal,
                std::optional<double> start_yaw = std::nullopt, bool start_pivoting = false);
// plan on a cloud: a grid over the search box, the ground at floor_z or estimated locally
PlanResult plan(const Cloud& cloud, std::optional<double> floor_z, const V2& start, const V2& goal, const Config& cfg,
                std::optional<double> start_yaw = std::nullopt, bool start_pivoting = false);
PlanResult plan_grid(std::shared_ptr<HeightGrid> hg, const V2& s, const V2& g, std::optional<double> start_yaw,
                     bool start_pivoting);
PlanResult plan_reverse(std::shared_ptr<HeightGrid> hg, const V2& s, const V2& g, double yaw);
double reverse_left(const Reverse& rev, const V2& pos);
std::vector<uint8_t> observed_free(const HeightGrid& hg);

PivotResult check_route(Window& win, const Path& route, std::optional<double> start_yaw = std::nullopt,
                        bool start_pivoting = false);

struct Follow {
  V2 c;
  double yaw = 0;
  std::string phase;
  std::optional<int> j;
};
Follow follow_plan(const Path& path, const std::vector<Pivot>& pivots, const V2& pos, std::optional<double> yaw,
                   const Config& cfg, double lead = 0.15);
std::pair<V2, double> run_out(const Path& path, const std::vector<Pivot>& pivots, const V2& pos,
                              std::optional<double> yaw, const V2& carrot_xy, const std::string& phase,
                              std::optional<int> j, const Config& cfg, double reach = 1.2, double fc_behind = ODOM_AHEAD,
                              double lead = 0.15, const Window* win = nullptr, double aligned_deg = 10.0,
                              double turning_deg = 30.0, double corner_deg = 20.0);

Path densify(const Path& path, double step);
Path remaining(const Path& path, const V2& pos);
std::pair<V2, double> carrot(const Path& path, const V2& pos, double lookahead);

// decaying per-cell hit / see-through / hit-from-afar counts on a world-fixed, growing grid
class EvidenceMemory {
 public:
  EvidenceMemory(double res, double tau) : res(res), tau(tau) {}
  void add(const HeightGrid& hg, const Evidence& ev, double now);
  void read(const HeightGrid& hg, double now, std::vector<double>& h, std::vector<double>& m,
            std::vector<double>& w);
  double res, tau;

 private:
  bool have = false;
  V2 lo = V2::Zero();
  int ny = 0, nx = 0;
  std::vector<float> h_, m_, w_;
  std::vector<double> t_;
  void ensure(const V2& lo, const V2& hi);
  std::pair<int, int> view(const HeightGrid& hg);
};

// the node's map: the last window_s seconds of registered scans (see track_core.cpp)
class LiveWindow {
 public:
  explicit LiveWindow(const Config& cfg = live_config(), double window_s = 3.0, double chunk_s = 0.5,
                      int n_clear = 2, std::optional<double> keep_radius = std::nullopt, double mem_tau = 10.0,
                      double mem_hits = 1.5, double mem_clear_ratio = 1.5, int max_new = 4);
  void add_scan(double t, const CloudF& points, const Eigen::Vector3d& sensor);
  CloudF points() const;
  PlanResult plan(const V2& start, const V2& goal, std::optional<double> start_yaw = std::nullopt,
                  bool start_pivoting = false);
  ConfigPtr cfg;
  double window_s, chunk_s;
  int n_clear;
  double keep_radius;
  std::unique_ptr<EvidenceMemory> mem;
  double mem_hits, mem_clear_ratio;
  int max_new;
  struct Scan {
    CloudF p;
    Eigen::Vector3d s;
    double t;
    long seq;
  };
  std::vector<std::pair<double, CloudF>> chunks, pending;
  std::vector<Scan> recent;
  long seq = 0, done = 0;

 private:
  void correct(HeightGrid& hg);
};

}  // namespace l4_track
