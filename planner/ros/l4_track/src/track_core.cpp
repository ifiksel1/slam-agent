// Track planner core (ROS-free): plan driving legs for the tracked QuadRover on a 2.5-D height grid.
// C++ port of planner/track_core.py (2026-10-29); the same model, names and defaults.
//
// Height grid (HeightGrid). The cloud is cut into res cells and deduplicated into res/2 voxels. The
// ground is a robust local model, not a fixed height band: each cell's lowest voxel; a low percentile
// of those per tile (tile, 0.5 m); a tile well above the median of its neighbours (a sofa seen only
// from above) takes that median; a tile far above the cloud's floor level (only a ceiling or the top
// of a wall seen there) or with no points takes the nearest good tile's value. A cell is an obstacle
// when at least min_pts voxels sit between z_lo and z_hi above that ground, or one does under a column
// of min_tall voxels up to z_tall (a pole or rail sparse in the band), or under a column of
// 2 x min_tall voxels with nothing in the band (a wall whose base is hidden). The obstacle band starts
// above the floor smear: 0.30 m on whole maps (their floor is ~0.22-0.26 m thick from floor height
// disagreeing between keyframes, z drift; one keyframe's floor is 0.03-0.05 m and the deskew time-unit
// fix does not thin it), 0.15 m on the live window (a few seconds of scans hardly drift).
//
// Overhangs (owner 2026-10-30: "the robot can drive under openings with a ceiling of 15 inches"; the
// frame is 0.203 m high). overhang_clear (0.381 m) is the top of what blocks on its own: a cell with any
// voxel in (z_lo, overhang_clear] above its local ground ("low") is judged by the rules above and never
// as an overhang. A cell whose returns all lie above overhang_clear (a table top, a shelf, a beam, a wall
// whose base is hidden) is still an obstacle by the same rules (the live window's sparse-wall rule
// included) unless the live evidence saw through the space beneath it: scan_evidence() casts the rays
// a second time in the slab (z_lo, overhang_clear] (Evidence::under), cut at the nearest return in that
// slab (table legs, clutter under the shelf, the base of a wall, the wall behind), not by returns above
// it (the table top), and counts the scans that hit the cell in the slab (Evidence::low). On the live
// window a cell with no low voxel (overhang_ok) whose beneath was seen through in at least overhang_seen
// (3) recent scans (the memory's decayed count) and more than mem_clear_ratio x the low hits is passable,
// and the memory does not hold it as a wall from afar. A wall seen only above the band has nothing seen
// beyond it, so no ray passes beneath it and it stays a wall; a table whose underside is lower than
// overhang_clear has low voxels and stays an obstacle. A wall's top that registers a cell off its base
// (tilt, drift) can be cleared as an overhang; its base cells keep their low voxels and stay obstacles, so
// the margin is held to the base (overhang_ring 1 would keep a cell round every low column, but narrows
// every opening under an overhang by a cell each side). On the house drive's live replay (2026-10-30), with
// overhang_seen 1 a wall end seen through beneath by 2 grazing scans (0.5 deg bins) was driven past at
// 0.02-0.04 m; with 3 the house wall checks are unchanged (255/255, 259/259, 262/263 as without). The
// lidar sits ~0.08 m up on the robot (0.25 m in SITL), well under a 0.38 m underside, so it sees under
// tables to whatever stands behind them. Whole maps (prior maps, the band-level evidence) have no
// beneath evidence: unchanged there.
//
// Moved objects. A merged map keeps everything that was ever there: on the house drive the cells
// blocking the robot's own driven path were hit by a median of 3 of 524 keyframes, far in time from
// when the robot drove there (people, doors). scan_evidence() casts each scan as a 2-D virtual scan
// of the obstacle band and counts, per cell, the scans that hit it and the scans that saw through it;
// apply_evidence() drops obstacle cells seen through more often than hit. With a prior map this uses
// the mapping run's keyframes; on the live window the latest scans (the latest observation wins).
// Every return is judged against its own tile's local ground (on the live window each scan gets a
// ground-only grid of the window's points around the sensor), never one flat level at the robot's
// floor (2026-10-29: from a floor gap beside lower ground, SITL k5/k6, the flat level read a container
// floor 0.15 m up the sill as 0.36 m up, walls from afar). A tile with no floor seen (ground_seen 0:
// no points, or only a ceiling) has a stand-in level, the nearest seen tile's; its returns are judged
// against the sensor's floor instead, as before, and the sensor's own floor, if its tile is one, is the
// highest floor seen within a tile of it (none: the scan's hits only, no see-throughs).
//
// Footprint. The frame is 0.508 m long and 0.457 m wide (landing FOOTPRINT) plus margin each side,
// and the planned point is the odometry point (the IMU, footprint_offset 0.193 m ahead of the frame
// centre; the bridge applies no lever arm). The search runs on cells whose clearance exceeds the
// half-width plus margin, preferring the middle of a passage and room to turn. The path is then
// shortened by line of sight with an oriented-rectangle check of the follower's poses: along each
// segment the heading is the direction to the carrot lookahead ahead, as the carrot follower drives
// (so corners are checked with the robot turning onto the next segment). A doorway narrower than the
// frame's diagonal but wider than the frame passes when the robot goes straight through it. The
// shortening prefers clearance (owner decision 2026-10-30, option 1): a shortcut (and a sharpen()
// corner) is kept only if its frame keeps prefer_clearance (0.15 m, bare frame to obstacle cell
// centres) wherever the search's cell path had that much (each pose against the nearest vertex it
// replaces, owed up to prefer_slack less than the cell path had); otherwise the cell path's vertex
// stays. Where the cell path itself is tighter (doors, the house drive) the requirement falls with it,
// down to the margin: the margin stays the hard limit. Soft throughout: a preferred route the frame
// cannot follow, or whose pivots or arcs are refused, gives way to the old straightened route before
// any refusal or new search (house drive live replay: 315/337/342 of 490/490/506 planned, was
// 298/315/344; without that fallback 297/321/316, the arc onto a route bent off the heading). A carrot
// a fixed distance ahead is the driving target (D4: position + yaw only, so a dead planner stops
// within the lookahead).
//
// Pivots (owner-approved 2026-10-28). The robot is a skid-steer: Rover Guided pivots in place when
// the bearing to its target leaves the heading by more than WP_PIVOT_ANGLE (60 on the robot) and
// pivots on until within 5 deg (AR_PivotTurn; the streamed Guided targets take the fast-update path,
// which does not cancel a pivot). Tracks turn about the middle of their contact patches, the frame
// centre, where the FC sits (owner 2026-10-29: the FC and its IMU are at the frame centre, the
// odometry point 0.193 m ahead of it; SITL's QuadRover turns about its vehicle origin, the FC). A pivot sweeps the circle of the frame's
// half-diagonal, 0.342 m, plus pivot_margin (its own parameter, owner 2026-10-28; the frame's margin
// stays for driving): 0.392 m, 0.78 m across, at 0.05. pivot_pass() makes every turn of more than
// pivot_angle (chord to the point lookahead further on, and the bearing from the frame centre) its
// own step: the carrot holds just past the pivot (follow_plan) so the robot drives straight in and
// stops with its frame centre there (within pivot_tol), the circle about that centre and up to
// pivot_tol either side along the incoming heading must be clear, the robot pivots, and the path goes
// on from where the odometry point is after the pivot. A circle that is not clear moves to the
// nearest centre within pivot_search where it is (reached by driving, frame clear), or the cells
// there leave the search and it runs again (pivot_retries). With the robot's heading the start is
// checked too: a start pivot needs the circle where the robot stands and up to pivot_tol ahead (a
// replan may turn a robot that is still rolling), or the robot drives ahead (within pivot_angle of
// its heading) to a centre where it fits. Turns under pivot_angle are driven and checked by the
// follower's oriented frame as before.
//
// Dead ends (owner 2026-10-28, plan_reverse). With the robot's heading known, the goal behind it
// (beyond pivot_angle) and the start pivot refused where it stands, plan() backs the robot straight
// up along its heading, at most reverse_max, to the nearest spot the start pivot fits and plans on
// from there (the result's reverse). Every frame pose of the reverse must be clear and every cell
// it sweeps observed free (observed_free: floor seen in the cell, or seen through more than hit in
// the live evidence memory): the robot backs up blind. The node streams it as a velocity against the
// heading, which the bridge turns into Rover Guided's velocity + heading target (reverse).
//
// The driving target (run_out). Rover Guided's fast-update path asks ~0.5 m/s per metre to its
// target and steers like pure pursuit on it; the node runs the carrot out along its bearing (up to
// ~2 s of WP_SPEED from the FC, frame-checked) on straights, and not at stop points or while the robot
// turns. Near obstacles the node shortens it (slow_reach, 2026-10-30): ~0.4 m/s where the frame comes
// within 0.10 m of an obstacle cell, here or on the next metre of the path, full speed from 0.35 m.
//
// LIMITS (recorded maps, 2026-10-28):
// - obstacles lower than z_lo above the local ground are invisible: 0.30 m on whole maps, 0.15 m on
//   the live window (kerbs, steps down, cables, low boxes, a pet); above z_hi (0.6 m), on whole maps
//   only a tall column counts (overhangs are driven under), on the live window 2+ voxels up to 2 m do
//   unless the space beneath (to overhang_clear, 0.381 m) was seen through (see Overhangs);
// - overhangs: heights are voxel centres (res/2 = 0.05 m) over the ground model, which sits at the
//   bottom of the floor smear (the lowest voxel's centre), so a voxel counts as low when its bottom edge
//   (centre - res/4) is at or under overhang_clear: an underside up to ~0.05 m above overhang_clear can
//   read low and be refused (conservative); on the gtest floor (0.06 m smear) an underside at 0.40 m
//   passes and 0.35 m does not. A thicker floor smear reads undersides higher. The robot drives under only
//   what it has seen through lately (decay 10 s): an overhang seen only from above or behind, or whose
//   beneath is hidden, is a wall. The memory's under-rays are the same 0.5 deg bins as the band's; a
//   wall whose base is hidden but which rays pass beside at a grazing angle could read as an overhang
//   (house wall checks: see the 2026-10-30 overhang report). Whole maps never drive under 0.38-0.6 m;
// - the lidar sits ~0.08 m above the floor: wall bases behind furniture go unseen. On the live window
//   any column seen above the band (2+ voxels) is a wall unless seen through beneath, and the evidence
//   memory holds walls hit from 1.5 m or more in earlier scans (decay 10 s; not overhangs seen through
//   beneath); a wall seen by one voxel and never from afar is still missed (house drive tfix: 1 of 263
//   goals behind a seen wall);
// - people standing still are obstacles until rays at band height see through them (1.5 x more often
//   than hit, decayed): on the house drive about 13% of the live goals ahead are refused, about 6-8%
//   of those whose driven stretch the scan of the moment shows free;
// - unknown space is driven through on the live window (optimistic, replanned as it is seen);
// - whole maps use the first, band-level evidence model (rays to the farthest return), which
//   overstates see-throughs; it works over hundreds of keyframes, not for a few scans;
// - doors seen open and closed equally often stay obstacles in a prior map;
// - the ground model is one level per 0.5 m tile: no steps or ramps, one storey; glass is untested.
//   A tile across a step takes its lower level (20th percentile), so the upper side's floor next to
//   the edge can read up to the step's height above it (SITL container: floor -0.12 in a tile reaching
//   the lower floor outside, -0.43, counted at z_lo 0.30 within ~0.3 m of the south wall). In the
//   per-scan evidence a tile with no floor seen is judged at the sensor's floor (the old flat
//   assumption, kept there: leaving those returns out, unknown, lost a wall check on house_current,
//   258/259, and changed the map-mode evidence); a robot standing where no floor is seen takes the
//   highest floor seen within a tile, never the nearest (that was a lower floor in k5);
// - pivots are checked where the plan puts them; Rover may also pivot elsewhere if its heading lags
//   the carrot by more than 60 deg on a turn under pivot_angle (not seen in the drive-chain model);
//   within start_skip (0.5 m) of the start the follower's frame was not checked until 2026-10-30, only
//   a clearance (a start pivot's circle is). With replans at 10 Hz that region is always just ahead of
//   the robot: a replan could cut a corner there (Gate L2 s8: 0.046-0.056 m from the crate, only the
//   bare-frame arc check applied). Since 2026-10-30 (start_frame) the frame there is held to the margin
//   and the preferred clearance where the start pose has that much, and otherwise (start_soft) to the
//   start pose's own clearance: no nearer anything than the robot already is. With the heading known the
//   poses there turn from the robot's heading to the path's over the stretch (unless a start pivot): at
//   the path's heading the start pose was fictitious (Gate L2 w1: 12 deg off the robot's, already at a
//   door jamb), which made the limit lax and the robot passed the jamb at 0.044 m. The full margin from the start would refuse every replan of a robot standing
//   within it (after a pivot near a wall); a clearance that has to grow back over the stretch would
//   refuse driving straight on along the wall, since turning away swings the rear corner into it. The
//   robot's own heading there is still only the arc check's (bare frame). start_skip_heading 0.15
//   (instead, a frame check from 0.15 m at the full margin) refused 29 more house replans in 2026-10-28;
// - the arc onto the path (2026-10-29, after the Gate L2 door-jamb clips c1/c3): with the heading known
//   each driving leg's arc is simulated from the robot's heading with Rover's steering lag fit to SITL
//   (2 sin(error) / max(distance to its target, arc_steer 3.0 m)) and its bare frame checked
//   (arc_margin 0). A start off by more than pivot_done whose arc is not clear becomes a start pivot,
//   even under pivot_angle (the carrot leads it past WP_PIVOT_ANGLE so Rover pivots); any other arc
//   failure is a refusal (NO_PATH; the node keeps its last path while that still checks), not another
//   search (blocking cells there sent the search on detours); with arc_vertices (off by default) a leg
//   after a pivot gets a pivot at its largest gentle turn before the failure first (up to 3). On the
//   house drive's live replay the arc check refuses about 20% of the replans that planned before
//   (296/316/341 instead of 376/385/405 of 490/490/506), mostly start pivots with no room: the
//   robot stops rather than arc into a wall on SITL's steering. A robot already heading along its first leg is committed to its next
//   arc_commit (1 m): a failure there is not acted on (it flip-flopped with the model's error). The
//   steering fit is SITL's (QuadRover, ATC_STR_RAT_FF 0.3, CRUISE_THROTTLE 5); the robot's own lag is
//   not measured yet. A pivot under way is checked where the robot stands only (it turns in place);
// - the start pivot is a hard check where the robot stands: on the house drive (tight, the robot
//   pivoted near walls) about 10% of live replans are refused for it (see the 2026-10-28 report);
// - a pivot that cannot be moved within pivot_search blocks the cells within the pivot radius of its
//   vertex for the next search, which also bars driving straight through them;
// - margin 0.05 m: the house drive passes walls with ~5 cm to spare, so 0.10 m refuses it. The
//   preferred clearance (0.15 m) is soft for that reason; it is measured to obstacle cell centres (the
//   obstacle's surface may be up to res/2 nearer or farther), and it binds the plan, not the robot:
//   heading lag and tracking error come off it (the slow-down near obstacles limits those).
// - driven bends (owner request 2026-10-30, after the hairpin drive chain went 0.016 m into the
//   divider: keeping the cell path's vertices split a sharp corner into two 43-45 deg bends 0.42 m
//   apart, each under pivot_angle, so driven, and the carrot chase cut inside; the old straightening
//   plans a driven 69 deg corner there from the same pose). Two guards: (1) follow_free checks the
//   chase's inside cut at every driven bend: the frame along the Bezier curve on the path from
//   cut_lead (0.5 m) before the bend to cut_lead after it (heading along the curve), at the margin; a
//   straight chord there turned the frame onto the chord at its start and refused leaving a 0.75 m
//   dead-end corridor; (2) merge_bends (in pivot_pass) makes driven bends within bend_merge (0.5 m) of
//   each other that turn by more than pivot_angle in all one corner where the incoming and outgoing
//   lines meet, which pivot_pass then pivots (the pivot's hard check), when the frame can follow
//   it. The curve is a model of the chase, not of Rover's lag (the arc check covers the outside swing);
// - the reverse only backs straight up the way the robot faces, over cells seen free in the last
//   ~window_s (floor) or the evidence memory (see-throughs, decay mem_tau 10 s): a robot that waited
//   long in a dead end may have forgotten the way out and is refused (never into unknown space).
//
// Port notes: numpy's arange / linspace / percentile, scipy's exact Euclidean feature transform
// (ndimage.distance_transform_edt, its tie-breaking included) and scipy.sparse.csgraph's Dijkstra
// (its Fibonacci heap, so equal-cost paths come out the same) are reproduced so the C++ planner takes
// the same decisions as the Python one did (see the 2026-10-29 A/B report).
#include "l4_track/track_core.h"

#include <algorithm>
#include <cstdarg>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <numeric>
#include <set>
#include <stdexcept>

namespace l4_track {

namespace {
constexpr double PI = M_PI;
constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
constexpr double INF = std::numeric_limits<double>::infinity();
using Cell = std::pair<int, int>;

double now_s() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// np.minimum / np.maximum: NaN propagates
inline double np_min(double a, double b) { return (std::isnan(a) || std::isnan(b)) ? NaN : (a < b ? a : b); }

// np.searchsorted on a sorted vector
inline int ss_right(const std::vector<double>& a, double v) {
  return int(std::upper_bound(a.begin(), a.end(), v) - a.begin());
}
inline int ss_left(const std::vector<double>& a, double v) {
  return int(std::lower_bound(a.begin(), a.end(), v) - a.begin());
}

// numpy's _lerp
inline double lerp(double a, double b, double t) {
  double d = b - a;
  return t >= 0.5 ? b - d * (1.0 - t) : a + d * t;
}

std::string fmt(const char* f, ...) __attribute__((format(printf, 1, 2)));
std::string fmt(const char* f, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, f);
  vsnprintf(buf, sizeof buf, f, ap);
  va_end(ap);
  return buf;
}

// ---- scipy.ndimage's exact Euclidean feature transform (ni_morphology.c, _VoronoiFT/_ComputeFT),
// 2-D, no sampling. in: nonzero = foreground (gets a feature), zero = feature. -> per cell the row
// and column of its nearest feature (ties as scipy breaks them); -1 where there is none.
void voronoi_ft(int len, int* fr, int* fc, long stride, int d, int coor0, int coor1, std::vector<int>& g,
                std::vector<std::array<int, 2>>& f) {
  // f[ii] = (row, col) of element ii along the line
  for (int ii = 0; ii < len; ++ii) f[ii] = {fr[ii * stride], fc[ii * stride]};
  const int coor[2] = {coor0, coor1};
  int l = -1;
  for (int ii = 0; ii < len; ++ii) {
    if (f[ii][0] >= 0) {
      double fd = f[ii][d];
      double wR = 0.0;
      for (int jj = 0; jj < 2; ++jj)
        if (jj != d) {
          double tw = f[ii][jj] - coor[jj];
          wR += tw * tw;
        }
      while (l >= 1) {
        double uR = 0.0, vR = 0.0;
        int idx1 = g[l], idx2 = g[l - 1];
        double f1 = f[idx1][d];
        double a = f1 - f[idx2][d];
        double b = fd - f1;
        double c = a + b;
        for (int jj = 0; jj < 2; ++jj)
          if (jj != d) {
            double cc = coor[jj];
            double tu = f[idx2][jj] - cc;
            double tv = f[idx1][jj] - cc;
            uR += tu * tu;
            vR += tv * tv;
          }
        if (c * vR - b * uR - a * wR - a * b * c <= 0.0) break;
        --l;
      }
      ++l;
      g[l] = ii;
    }
  }
  int maxl = l;
  if (maxl >= 0) {
    l = 0;
    for (int ii = 0; ii < len; ++ii) {
      double delta1 = 0.0, t;
      for (int jj = 0; jj < 2; ++jj) {
        t = jj == d ? f[g[l]][jj] - ii : f[g[l]][jj] - coor[jj];
        delta1 += t * t;
      }
      while (l < maxl) {
        double delta2 = 0.0;
        for (int jj = 0; jj < 2; ++jj) {
          t = jj == d ? f[g[l + 1]][jj] - ii : f[g[l + 1]][jj] - coor[jj];
          delta2 += t * t;
        }
        if (delta1 <= delta2) break;
        delta1 = delta2;
        ++l;
      }
      int idx1 = g[l];
      fr[ii * stride] = f[idx1][0];
      fc[ii * stride] = f[idx1][1];
    }
  }
}

void feature_transform(const std::vector<uint8_t>& in, int ny, int nx, std::vector<int>& fr, std::vector<int>& fc) {
  fr.assign(size_t(ny) * nx, -1);
  fc.assign(size_t(ny) * nx, -1);
  int L = std::max(ny, nx);
  std::vector<int> g(L);
  std::vector<std::array<int, 2>> f(L);
  // axis 0 (rows) per column
  for (int j = 0; j < nx; ++j) {
    for (int i = 0; i < ny; ++i) {
      if (in[size_t(i) * nx + j]) {
        fr[size_t(i) * nx + j] = -1;
      } else {
        fr[size_t(i) * nx + j] = i;
        fc[size_t(i) * nx + j] = j;
      }
    }
    voronoi_ft(ny, &fr[j], &fc[j], nx, 0, 0, j, g, f);
  }
  // axis 1 (columns) per row
  for (int i = 0; i < ny; ++i) voronoi_ft(nx, &fr[size_t(i) * nx], &fc[size_t(i) * nx], 1, 1, i, 0, g, f);
}

// binary_dilation with a (symmetric) structure given as offsets; border 0
std::vector<uint8_t> dilate(const std::vector<uint8_t>& a, int ny, int nx, const std::vector<Cell>& st) {
  std::vector<uint8_t> out(a.size(), 0);
  for (int i = 0; i < ny; ++i)
    for (int j = 0; j < nx; ++j)
      if (a[size_t(i) * nx + j])
        for (auto& o : st) {
          int ii = i + o.first, jj = j + o.second;
          if (ii >= 0 && jj >= 0 && ii < ny && jj < nx) out[size_t(ii) * nx + jj] = 1;
        }
  return out;
}

bool any(const std::vector<uint8_t>& a) {
  for (auto v : a)
    if (v) return true;
  return false;
}

// ---- scipy.sparse.csgraph's Fibonacci heap (_shortest_path.pyx), for the same tie-breaking
struct FNode {
  unsigned index = 0, rank = 0;
  int state = 1;   // 0 SCANNED, 1 NOT_IN_HEAP, 2 IN_HEAP
  double val = 0;
  FNode *parent = nullptr, *left = nullptr, *right = nullptr, *children = nullptr;
};

struct FHeap {
  FNode* min_node = nullptr;
  FNode* roots_by_rank[100];

  static FNode* leftmost(FNode* n) {
    while (n->left) n = n->left;
    return n;
  }
  static FNode* rightmost(FNode* n) {
    while (n->right) n = n->right;
    return n;
  }
  static void add_sibling(FNode* node, FNode* s) {
    FNode* t = rightmost(node);
    t->right = s;
    s->left = t;
    s->right = nullptr;
    s->parent = node->parent;
    if (s->parent) s->parent->rank += 1;
  }
  static void add_child(FNode* node, FNode* c) {
    c->parent = node;
    if (node->children) {
      add_sibling(node->children, c);
    } else {
      node->children = c;
      c->right = nullptr;
      c->left = nullptr;
      node->rank = 1;
    }
  }
  static void remove(FNode* node) {
    if (node->parent) {
      node->parent->rank -= 1;
      if (node->left)
        node->parent->children = node->left;
      else if (node->right)
        node->parent->children = node->right;
      else
        node->parent->children = nullptr;
    }
    if (node->left) node->left->right = node->right;
    if (node->right) node->right->left = node->left;
    node->left = nullptr;
    node->right = nullptr;
    node->parent = nullptr;
  }
  void insert(FNode* node) {
    if (min_node) {
      add_sibling(min_node, node);
      if (node->val < min_node->val) min_node = node;
    } else {
      min_node = node;
    }
  }
  void decrease(FNode* node, double v) {
    node->val = v;
    if (node->parent && node->parent->val >= v) {
      remove(node);
      insert(node);
    } else if (min_node->val > node->val) {
      min_node = node;
    }
  }
  void link(FNode* node) {
    while (true) {
      if (roots_by_rank[node->rank] == nullptr) {
        roots_by_rank[node->rank] = node;
        return;
      }
      FNode* ln = roots_by_rank[node->rank];
      roots_by_rank[node->rank] = nullptr;
      if (node->val < ln->val || node == min_node) {
        remove(ln);
        add_child(node, ln);
      } else {
        remove(node);
        add_child(ln, node);
        node = ln;
      }
    }
  }
  FNode* remove_min() {
    FNode* temp;
    if (min_node->children) {
      temp = leftmost(min_node->children);
      while (temp) {
        FNode* tr = temp->right;
        remove(temp);
        add_sibling(min_node, temp);
        temp = tr;
      }
      min_node->children = nullptr;
    }
    temp = leftmost(min_node);
    if (temp == min_node) {
      if (min_node->right) {
        temp = min_node->right;
      } else {
        FNode* out = min_node;
        min_node = nullptr;
        return out;
      }
    }
    FNode* out = min_node;
    remove(min_node);
    min_node = temp;
    for (auto& r : roots_by_rank) r = nullptr;
    while (temp) {
      if (temp->val < min_node->val) min_node = temp;
      FNode* tr = temp->right;
      link(temp);
      temp = tr;
    }
    return out;
  }
};

}  // namespace

const double FOOTPRINT_HALF_DIAGONAL = std::hypot(FOOTPRINT_LEN, FOOTPRINT_WID) / 2;
const char* const REVERSE_REASONS[2] = {"no room to pivot at the start",
                                        "the frame cannot drive off the pivot at the start"};

// ---------------------------------------------------------------- numpy / Python helpers
double pymod(double a, double b) {
  double m = std::fmod(a, b);
  if (m != 0.0) {
    if ((b < 0) != (m < 0)) m += b;
  } else {
    m = std::copysign(0.0, b);
  }
  return m;
}
double wrap(double a) { return pymod(a + PI, 2 * PI) - PI; }
double radians(double deg) { return deg * (PI / 180.0); }
double degrees(double rad) { return rad * (180.0 / PI); }
double pyround(double x, int nd) {
  if (!std::isfinite(x)) return x;
  char buf[64];
  snprintf(buf, sizeof buf, "%.*f", nd, x);
  return std::strtod(buf, nullptr);
}

std::vector<double> arange(double start, double stop, double step) {
  double n = std::ceil((stop - start) / step);
  std::vector<double> out;
  if (!(n > 0)) return out;
  size_t len = size_t(n);
  out.resize(len);
  out[0] = start;
  if (len > 1) {
    out[1] = start + step;
    double delta = out[1] - out[0];
    for (size_t i = 2; i < len; ++i) out[i] = start + double(i) * delta;
  }
  return out;
}

std::vector<double> linspace(double a, double b, int n) {
  std::vector<double> y(std::max(n, 0));
  if (n <= 0) return y;
  int div = n - 1;
  double delta = b - a;
  if (div > 0) {
    double step = delta / div;
    for (int i = 0; i < n; ++i) y[i] = (step == 0 ? (double(i) / div) * delta : double(i) * step) + a;
    y[n - 1] = b;
  } else {
    y[0] = a;
  }
  return y;
}

double percentile(std::vector<double> v, double q_pct) {
  if (v.empty()) return NaN;
  std::sort(v.begin(), v.end());
  double q = q_pct / 100.0;
  size_t n = v.size();
  double vi = double(n - 1) * q;
  if (vi >= double(n - 1)) return v[n - 1];
  if (vi < 0) return v[0];
  double prev = std::floor(vi);
  size_t pi = size_t(prev);
  return lerp(v[pi], v[pi + 1], vi - prev);
}

double median(std::vector<double> v) {
  if (v.empty()) return NaN;
  size_t n = v.size();
  std::nth_element(v.begin(), v.begin() + n / 2, v.end());
  double hi = v[n / 2];
  if (n % 2) return hi;
  double lo = *std::max_element(v.begin(), v.begin() + n / 2);
  return (lo + hi) / 2.0;   // numpy: mean of the two middle values
}

// ---------------------------------------------------------------- Config
void Config::update() {
  half_len = FOOTPRINT_LEN / 2 + margin;
  half_wid = FOOTPRINT_WID / 2 + margin;
  r_in = half_wid;
  r_hard = std::min(half_wid, half_len - off);
  r_out = std::hypot(half_len + off, half_wid);
  inflate = r_in;
  pivot_angle = radians(pivot_angle_deg);
  pivot_done = radians(pivot_done_deg);
  pivot_radius = FOOTPRINT_HALF_DIAGONAL + pivot_margin;
  // per heading bin: the cell offsets whose centres can fall inside the frame for a pose anywhere in
  // the centre cell at any heading in the bin (the frame grown by the sub-cell offset and half a bin)
  auto k = std::make_shared<Kernels>();
  double r = res;
  double grow = r * 0.75 + std::hypot(half_len + off, half_wid) * PI / N_HEADINGS;
  int R = int(std::ceil((r_out + grow) / r));
  k->R = R;
  for (int h = 0; h < N_HEADINGS; ++h) {
    double yaw = h * 2 * PI / N_HEADINGS;
    double cs = std::cos(yaw), sn = std::sin(yaw);
    for (int di = -R; di <= R; ++di)
      for (int dj = -R; dj <= R; ++dj) {
        double x = dj * r, y = di * r;
        double dx = x + off * cs, dy = y + off * sn;
        double lon = std::abs(dx * cs + dy * sn), lat = std::abs(-dx * sn + dy * cs);
        if (lon < half_len + grow && lat < half_wid + grow) k->ks[h].push_back({di, dj});
      }
  }
  kern = k;
}

Config live_config() {
  Config c;
  c.z_lo = 0.15;
  c.min_pts = 2;
  c.unknown_free = true;
  c.clear_misses = 1;
  c.clear_ratio = 0.99;
  c.min_tall_only = 2;
  c.mem_min_range = 1.5;
  c.update();
  return c;
}

// ---------------------------------------------------------------- voxel_down
CloudF voxel_down(const CloudF& p, double vox) {
  const float v = float(vox);
  std::vector<int64_t> key;
  key.reserve(p.size());
  const int64_t off = int64_t(1) << 20;
  for (auto& q : p) {
    if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z)) continue;
    int64_t kx = int64_t(std::floor(q.x / v)) + off, ky = int64_t(std::floor(q.y / v)) + off,
            kz = int64_t(std::floor(q.z / v)) + off;
    key.push_back((kx << 42) | (ky << 21) | kz);
  }
  std::sort(key.begin(), key.end());
  key.erase(std::unique(key.begin(), key.end()), key.end());
  CloudF out(key.size());
  const int64_t mask = (int64_t(1) << 21) - 1;
  for (size_t i = 0; i < key.size(); ++i) {
    float x = float((key[i] >> 42) & mask), y = float((key[i] >> 21) & mask), z = float(key[i] & mask);
    out[i] = {(x - 1048576.0f + 0.5f) * v, (y - 1048576.0f + 0.5f) * v, (z - 1048576.0f + 0.5f) * v};
  }
  return out;
}

// ---------------------------------------------------------------- HeightGrid
HeightGrid::HeightGrid(const Cloud& points, ConfigPtr cfg_, std::optional<V2> lo_, std::optional<V2> hi_,
                       std::optional<double> floor_z, bool ground_only)
    : cfg(std::move(cfg_)) {
  const Config& c = *cfg;
  const double r = c.res;
  V2 blo, bhi;
  if (!lo_) {
    if (!points.empty()) {
      blo = V2(INF, INF);
      bhi = V2(-INF, -INF);
      for (auto& p : points) {
        blo.x() = std::min(blo.x(), p.x);
        blo.y() = std::min(blo.y(), p.y);
        bhi.x() = std::max(bhi.x(), p.x);
        bhi.y() = std::max(bhi.y(), p.y);
      }
    } else {
      blo = V2(0, 0);
      bhi = V2(1, 1);
    }
  } else {
    blo = *lo_;
    bhi = *hi_;
  }
  int m = int(std::nearbyint(c.tile / r));
  double rm = r * m;
  lo = V2(std::floor(blo.x() / rm) * rm, std::floor(blo.y() / rm) * rm);
  ny = int(std::ceil((bhi.y() - lo.y()) / rm)) * m + m;
  nx = int(std::ceil((bhi.x() - lo.x()) / rm)) * m + m;
  const size_t n = size_t(ny) * nx;
  const int64_t off = int64_t(1) << 20, mask = (int64_t(1) << 21) - 1;
  if (ground_only && !floor_z) {
    // the ground alone needs each cell's lowest voxel, no voxel counts: one pass, no sort (the same cellmin)
    std::vector<int64_t> kmin(n, std::numeric_limits<int64_t>::max());
    for (auto& p : points) {
      int64_t jx = int64_t(std::floor((p.x - lo.x()) / r)), iy = int64_t(std::floor((p.y - lo.y()) / r));
      if (jx < 0 || iy < 0 || jx >= nx || iy >= ny) continue;
      int64_t kz = int64_t(std::floor(p.z / (r * 0.5)));
      int64_t& km = kmin[size_t(iy * nx + jx)];
      km = std::min(km, kz);
    }
    cellmin.assign(n, NaN);
    for (size_t q = 0; q < n; ++q)
      if (kmin[q] != std::numeric_limits<int64_t>::max()) cellmin[q] = (double(kmin[q]) + 0.5) * r * 0.5;
    ground_model(m);
    return;
  }
  // distinct voxels (res/2 high), so a map and a pile of scans count alike
  std::vector<int64_t> key;
  key.reserve(points.size());
  for (auto& p : points) {
    int64_t jx = int64_t(std::floor((p.x - lo.x()) / r)), iy = int64_t(std::floor((p.y - lo.y()) / r));
    if (jx < 0 || iy < 0 || jx >= nx || iy >= ny) continue;
    int64_t kz = int64_t(std::floor(p.z / (r * 0.5)));
    key.push_back((iy * nx + jx) * (int64_t(1) << 21) + (kz + off));
  }
  std::sort(key.begin(), key.end());
  key.erase(std::unique(key.begin(), key.end()), key.end());
  std::vector<int64_t> k(key.size());
  std::vector<double> z(key.size());
  cellmin.assign(n, NaN);
  for (size_t q = 0; q < key.size(); ++q) {
    k[q] = key[q] >> 21;
    z[q] = (double((key[q] & mask) - off) + 0.5) * r * 0.5;
    if (q == 0 || k[q] != k[q - 1]) cellmin[k[q]] = z[q];   // sorted: each cell's lowest voxel first
  }
  if (floor_z) {
    ground.assign(n, *floor_z);
    ground_seen.assign(n, 1);
  } else {
    ground_model(m);
  }
  if (ground_only) return;
  count.assign(n, 0);
  low.assign(n, 0);
  std::vector<int> tall(n, 0);
  std::vector<uint8_t> floor(n, 0);
  for (size_t q = 0; q < key.size(); ++q) {
    double h = z[q] - ground[k[q]];
    if (h > c.z_lo && h <= c.overhang_clear + r * 0.25) low[k[q]]++;   // the voxel's bottom at or under it
    if (h > c.z_lo && h < c.z_hi) count[k[q]]++;
    if (h >= c.z_hi && h < c.z_tall) tall[k[q]]++;
    if (h > -0.15 && h <= c.z_lo) floor[k[q]] = 1;
  }
  // a column above the band with its base hidden (the lidar sits ~0.08 m above the floor, furniture
  // occludes wall bases) is a wall too
  occ.assign(n, 0);
  seen_up.assign(n, 0);
  std::vector<uint8_t> seen(n, 0);
  for (size_t q = 0; q < n; ++q) {
    occ[q] = (count[q] >= c.min_pts) || (count[q] >= 1 && tall[q] >= c.min_tall) || (tall[q] >= c.min_tall_only);
    seen_up[q] = (count[q] + tall[q]) > 0;
    seen[q] = floor[q] | occ[q];
  }
  floor_seen = floor;
  // overhang candidates: no low voxel in the cell (or within overhang_ring cells of it)
  {
    std::vector<uint8_t> lw(n);
    for (size_t q = 0; q < n; ++q) lw[q] = low[q] > 0;
    std::vector<Cell> st8;
    for (int a = -c.overhang_ring; a <= c.overhang_ring; ++a)
      for (int b = -c.overhang_ring; b <= c.overhang_ring; ++b) st8.push_back({a, b});
    if (any(lw) && c.overhang_ring > 0) lw = dilate(lw, ny, nx, st8);
    overhang_ok.assign(n, 0);
    for (size_t q = 0; q < n; ++q) overhang_ok[q] = !lw[q];
  }
  int rr = int(std::ceil(c.known_radius / r));
  std::vector<Cell> st;
  for (int yy = -rr; yy <= rr; ++yy)
    for (int xx = -rr; xx <= rr; ++xx)
      if (xx * xx + yy * yy <= rr * rr) st.push_back({yy, xx});
  known = any(seen) ? dilate(seen, ny, nx, st) : seen;
  edt();
}

void HeightGrid::edt() {
  const size_t n = size_t(ny) * nx;
  if (!any(occ)) {
    clear.assign(n, 1e3);
    return;
  }
  std::vector<uint8_t> in(n);
  for (size_t q = 0; q < n; ++q) in[q] = !occ[q];
  std::vector<int> fr, fc;
  feature_transform(in, ny, nx, fr, fc);
  clear.resize(n);
  for (int i = 0; i < ny; ++i)
    for (int j = 0; j < nx; ++j) {
      size_t q = size_t(i) * nx + j;
      double di = fr[q] - i, dj = fc[q] - j;
      clear[q] = std::sqrt(di * di + dj * dj) * cfg->res;
    }
}

void HeightGrid::ground_model(int m) {
  const Config& c = *cfg;
  const int ty = ny / m, tx = nx / m;
  std::vector<double> g(size_t(ty) * tx, NaN);
  std::vector<double> buf;
  for (int a = 0; a < ty; ++a)
    for (int b = 0; b < tx; ++b) {
      buf.clear();
      for (int u = 0; u < m; ++u)
        for (int v = 0; v < m; ++v) {
          double val = cellmin[size_t(a * m + u) * nx + b * m + v];
          if (std::isfinite(val)) buf.push_back(val);
        }
      if (!buf.empty()) g[size_t(a) * tx + b] = percentile(buf, c.ground_pct);
    }
  buf.clear();
  for (double v : g)
    if (std::isfinite(v)) buf.push_back(v);
  if (!buf.empty()) {
    // only a ceiling or the top of a wall seen in this tile: not ground
    double lim = percentile(buf, 25) + c.max_ground_step;
    for (double& v : g)
      if (v > lim) v = NaN;
  }
  // a tile well above the median of its 3 x 3 neighbourhood: the median
  std::vector<double> nb(g.size(), NaN), w;
  for (int a = 0; a < ty; ++a)
    for (int b = 0; b < tx; ++b) {
      w.clear();
      for (int u = -1; u <= 1; ++u)
        for (int v = -1; v <= 1; ++v) {
          int aa = a + u, bb = b + v;
          if (aa < 0 || bb < 0 || aa >= ty || bb >= tx) continue;
          double val = g[size_t(aa) * tx + bb];
          if (std::isfinite(val)) w.push_back(val);
        }
      if (!w.empty()) nb[size_t(a) * tx + b] = median(w);
    }
  for (size_t q = 0; q < g.size(); ++q)
    if (std::isfinite(g[q]) && std::isfinite(nb[q]) && g[q] > nb[q] + c.furniture_step) g[q] = nb[q];
  ground_seen.resize(size_t(ny) * nx);
  for (int i = 0; i < ny; ++i)
    for (int j = 0; j < nx; ++j) ground_seen[size_t(i) * nx + j] = std::isfinite(g[size_t(i / m) * tx + j / m]);
  std::vector<uint8_t> miss(g.size());
  bool all = true, some = false;
  for (size_t q = 0; q < g.size(); ++q) {
    miss[q] = !std::isfinite(g[q]);
    all = all && miss[q];
    some = some || miss[q];
  }
  if (all) {
    std::fill(g.begin(), g.end(), 0.0);
  } else if (some) {
    std::vector<int> fr, fc;
    feature_transform(miss, ty, tx, fr, fc);
    std::vector<double> g2(g.size());
    for (size_t q = 0; q < g.size(); ++q) g2[q] = g[size_t(fr[q]) * tx + fc[q]];
    g.swap(g2);
  }
  ground.resize(size_t(ny) * nx);
  for (int i = 0; i < ny; ++i)
    for (int j = 0; j < nx; ++j) ground[size_t(i) * nx + j] = g[size_t(i / m) * tx + j / m];
}

int HeightGrid::apply_evidence(const std::vector<int>& hits, const std::vector<int>& misses) {
  const Config& c = *cfg;
  int dropped = 0;
  for (size_t q = 0; q < occ.size(); ++q)
    if (occ[q] && misses[q] >= c.clear_misses && misses[q] > c.clear_ratio * hits[q]) {
      occ[q] = 0;
      dropped++;
    }
  edt();
  return dropped;
}

int HeightGrid::apply_overhangs(const std::vector<int>& under, const std::vector<int>& low_hits) {
  const Config& c = *cfg;
  if (!(c.overhang_clear > c.z_lo) || overhang_ok.empty()) return 0;
  int dropped = 0;
  for (size_t q = 0; q < occ.size(); ++q)
    if (occ[q] && overhang_ok[q] && under[q] >= c.clear_misses && under[q] > c.clear_ratio * low_hits[q]) {
      occ[q] = 0;
      dropped++;
    }
  if (dropped) edt();
  return dropped;
}

Cell HeightGrid::cell(const V2& p) const {
  return {int(std::floor((p.y() - lo.y()) / cfg->res)), int(std::floor((p.x() - lo.x()) / cfg->res))};
}
V2 HeightGrid::xy(Cell c) const {
  return V2(lo.x() + (c.second + 0.5) * cfg->res, lo.y() + (c.first + 0.5) * cfg->res);
}
double HeightGrid::clearance_at(const V2& p) const {
  Cell c = cell(p);
  return inside(c) ? clear[idx(c.first, c.second)] : 0.0;
}
double HeightGrid::ground_at(const V2& p) const {
  Cell c = cell(p);
  return inside(c) ? ground[idx(c.first, c.second)] : NaN;
}

std::shared_ptr<Window> HeightGrid::crop(const V2& l, const V2& h) const {
  const double r = cfg->res;
  int i0 = std::max(0, int(std::floor((l.y() - lo.y()) / r)));
  int j0 = std::max(0, int(std::floor((l.x() - lo.x()) / r)));
  int i1 = std::min(ny, int(std::ceil((h.y() - lo.y()) / r)) + 1);
  int j1 = std::min(nx, int(std::ceil((h.x() - lo.x()) / r)) + 1);
  return std::make_shared<Window>(*this, i0, i1, j0, j1);
}

double Ground::at(double x, double y) const {
  int j = std::clamp(int(std::floor((x - lo.x()) / res)), 0, nx - 1);
  int i = std::clamp(int(std::floor((y - lo.y()) / res)), 0, ny - 1);
  return g[size_t(i) * nx + j];
}

Ground local_ground(const Cloud& points, const Config& cfg, std::optional<V2> lo, std::optional<V2> hi) {
  HeightGrid hg(points, std::make_shared<const Config>(cfg), lo, hi, std::nullopt, true);
  Ground out;
  out.g = std::move(hg.ground);
  out.ny = hg.ny;
  out.nx = hg.nx;
  out.lo = hg.lo;
  out.res = cfg.res;
  return out;
}

// ---------------------------------------------------------------- Window
Window::Window(const HeightGrid& hg, int i0, int i1, int j0, int j1) : cfg(hg.cfg) {
  const Config& c = *cfg;
  i1 = std::max(i1, i0);
  j1 = std::max(j1, j0);
  lo = V2(hg.lo.x() + j0 * c.res, hg.lo.y() + i0 * c.res);
  ny = std::max(0, std::min(i1, hg.ny) - i0);
  nx = std::max(0, std::min(j1, hg.nx) - j0);
  size_t n = size_t(ny) * nx;
  occ.resize(n);
  clear.resize(n);
  known.resize(n);
  free.resize(n);
  for (int i = 0; i < ny; ++i)
    for (int j = 0; j < nx; ++j) {
      size_t a = size_t(i) * nx + j, b = size_t(i + i0) * hg.nx + j + j0;
      occ[a] = hg.occ[b];
      clear[a] = hg.clear[b];
      known[a] = hg.known[b];
      free[a] = (clear[a] > c.r_in) && (known[a] || c.unknown_free);
    }
  // the obstacles around the box, padded, for the oriented checks
  pad = c.kern->R + 1;
  bny = ny + 2 * pad;
  bnx = nx + 2 * pad;
  occ_big.assign(size_t(bny) * bnx, 0);
  int bi0 = i0 - pad, bj0 = j0 - pad;
  int si0 = std::max(0, bi0), sj0 = std::max(0, bj0);
  int si1 = std::min(hg.ny, i1 + pad), sj1 = std::min(hg.nx, j1 + pad);
  for (int i = si0; i < si1; ++i)
    for (int j = sj0; j < sj1; ++j) occ_big[size_t(i - bi0) * bnx + (j - bj0)] = hg.occ[size_t(i) * hg.nx + j];
  skip = c.start_skip;
}

Cell Window::cell(const V2& p) const {
  return {int(std::floor((p.y() - lo.y()) / cfg->res)), int(std::floor((p.x() - lo.x()) / cfg->res))};
}
V2 Window::xy(Cell c) const { return V2(lo.x() + (c.second + 0.5) * cfg->res, lo.y() + (c.first + 0.5) * cfg->res); }
double Window::clearance_at(const V2& p) const {
  Cell c = cell(p);
  return inside(c) ? clear[size_t(c.first) * nx + c.second] : 0.0;
}

std::optional<Cell> Window::nearest_free(Cell c, double radius_m) const {
  if (inside(c) && is_free(c)) return c;
  int r = int(std::ceil(radius_m / cfg->res));
  int i0 = std::max(0, c.first - r), j0 = std::max(0, c.second - r);
  int i1 = std::min(ny, std::max(0, c.first + r + 1)), j1 = std::min(nx, std::max(0, c.second + r + 1));
  long best = -1;
  Cell bc{0, 0};
  for (int i = i0; i < i1; ++i)
    for (int j = j0; j < j1; ++j)
      if (free[size_t(i) * nx + j]) {
        long d = long(i - c.first) * (i - c.first) + long(j - c.second) * (j - c.second);
        if (best < 0 || d < best) {
          best = d;
          bc = {i, j};
        }
      }
  if (best < 0 || best > long(r) * r) return std::nullopt;
  return bc;
}

std::vector<uint8_t> Window::poses_free(const std::vector<V2>& xy, const std::vector<double>& yaw) const {
  const Config& c = *cfg;
  const double r = c.res;
  std::vector<uint8_t> out(xy.size(), 1);
  const auto& ks = c.kern->ks;
  for (size_t q = 0; q < xy.size(); ++q) {
    int ci = int(std::floor((xy[q].y() - lo.y()) / r)), cj = int(std::floor((xy[q].x() - lo.x()) / r));
    if (ci < 0 || cj < 0 || ci >= ny || cj >= nx) {
      out[q] = 0;
      continue;
    }
    if (!(clear[size_t(ci) * nx + cj] <= c.r_out + r * 0.75)) continue;
    int hb = int(int64_t(std::nearbyint(pymod(yaw[q], 2 * PI) / (2 * PI) * N_HEADINGS)) % N_HEADINGS);
    double cs = std::cos(yaw[q]), sn = std::sin(yaw[q]);
    double fx = xy[q].x() - c.off * cs, fy = xy[q].y() - c.off * sn;
    for (auto& o : ks[hb]) {
      int ii = ci + pad + o.first, jj = cj + pad + o.second;
      if (!occ_big[size_t(ii) * bnx + jj]) continue;
      // exact: obstacle cell centres against the frame at the true pose
      double ox = lo.x() + ((cj + o.second) + 0.5) * r, oy = lo.y() + ((ci + o.first) + 0.5) * r;
      double dx = ox - fx, dy = oy - fy;
      if (std::abs(dx * cs + dy * sn) < c.half_len && std::abs(-dx * sn + dy * cs) < c.half_wid) {
        out[q] = 0;
        break;
      }
    }
  }
  return out;
}

bool Window::pose_free(const V2& p, double yaw) const { return poses_free({p}, {yaw})[0] != 0; }

std::vector<int> Window::pose_hits(const V2& p, double yaw, double margin) const {
  const Config& c = *cfg;
  const double r = c.res;
  const double hl = margin < 0 ? c.half_len : FOOTPRINT_LEN / 2 + std::min(margin, c.margin);
  const double hw = margin < 0 ? c.half_wid : FOOTPRINT_WID / 2 + std::min(margin, c.margin);
  std::vector<int> out;
  int ci = int(std::floor((p.y() - lo.y()) / r)), cj = int(std::floor((p.x() - lo.x()) / r));
  if (ci < 0 || cj < 0 || ci >= ny || cj >= nx) return {-1};   // off the box: blocked
  int hb = int(int64_t(std::nearbyint(pymod(yaw, 2 * PI) / (2 * PI) * N_HEADINGS)) % N_HEADINGS);
  double cs = std::cos(yaw), sn = std::sin(yaw);
  double fx = p.x() - c.off * cs, fy = p.y() - c.off * sn;
  for (auto& o : c.kern->ks[hb]) {
    int ii = ci + pad + o.first, jj = cj + pad + o.second;
    if (!occ_big[size_t(ii) * bnx + jj]) continue;
    double ox = lo.x() + ((cj + o.second) + 0.5) * r, oy = lo.y() + ((ci + o.first) + 0.5) * r;
    double dx = ox - fx, dy = oy - fy;
    if (std::abs(dx * cs + dy * sn) < hl && std::abs(-dx * sn + dy * cs) < hw) out.push_back(ii * bnx + jj);
  }
  std::sort(out.begin(), out.end());
  return out;
}

std::vector<uint8_t> Window::discs_free(const std::vector<V2>& xy, double radius) const {
  const Config& c = *cfg;
  const double R = radius < 0 ? c.pivot_radius : radius;
  const double r = c.res;
  int k = int(std::ceil(R / r)) + 1;
  std::vector<uint8_t> out(xy.size(), 1);
  for (size_t q = 0; q < xy.size(); ++q) {
    int ci = int(std::floor((xy[q].y() - lo.y()) / r)), cj = int(std::floor((xy[q].x() - lo.x()) / r));
    bool ok = true;
    for (int di = -k; di <= k && ok; ++di)
      for (int dj = -k; dj <= k; ++dj) {
        int ii = ci + pad + di, jj = cj + pad + dj;
        if (ii < 0 || jj < 0 || ii >= bny || jj >= bnx) {
          ok = false;
          break;
        }
        if (!occ_big[size_t(ii) * bnx + jj]) continue;
        double ox = lo.x() + ((cj + dj) + 0.5) * r, oy = lo.y() + ((ci + di) + 0.5) * r;
        double ex = ox - xy[q].x(), ey = oy - xy[q].y();
        if (ex * ex + ey * ey < R * R) {
          ok = false;
          break;
        }
      }
    out[q] = ok;
  }
  return out;
}

bool Window::pivot_free(const V2& centre, double yaw_in, double tol, bool back) const {
  tol = tol < 0 ? cfg->pivot_tol : tol;
  double t0 = back ? -tol : 0.0;
  std::vector<double> t = tol > 0 ? linspace(t0, tol, int(std::ceil((tol - t0) / (cfg->res * 0.5))) + 1)
                                  : std::vector<double>{0.0};
  V2 u = unit(yaw_in);
  std::vector<V2> cs;
  for (double v : t) cs.push_back(V2(centre.x() + v * u.x(), centre.y() + v * u.y()));
  for (auto b : discs_free(cs))
    if (!b) return false;
  return true;
}

std::optional<int> Window::follow_free(const Path& P, int k0, const ClearRef* ref, int r0, int r1) {
  const Config& c = *cfg;
  const int n = int(P.size());
  if (n < 2) return std::nullopt;
  std::vector<double> L(n - 1), cum(n, 0.0);
  for (int s = 0; s < n - 1; ++s) {
    V2 d = P[s + 1] - P[s];
    L[s] = std::sqrt(d.x() * d.x() + d.y() * d.y());
    cum[s + 1] = cum[s] + L[s];
  }
  const double step = c.res * 0.5;
  std::vector<double> s = arange(cum[k0], cum[n - 1], step);
  s.push_back(cum[n - 1]);
  const size_t m = s.size();
  std::vector<int> seg(m);
  std::vector<V2> p(m);
  std::vector<double> a(m), yaw(m);
  for (size_t q = 0; q < m; ++q) {
    seg[q] = std::clamp(ss_right(cum, s[q]) - 1, 0, n - 2);
    double f = (s[q] - cum[seg[q]]) / std::max(L[seg[q]], 1e-9);
    p[q] = P[seg[q]] + (P[seg[q] + 1] - P[seg[q]]) * f;
    a[q] = std::min(s[q] + c.lookahead, cum[n - 1]);
  }
  std::vector<uint8_t> in_pivot(m, 0);
  std::vector<int> pv = pivot_vertices(P, c);
  if (!pv.empty()) {
    std::vector<double> cp;
    for (int k : pv) cp.push_back(cum[k]);
    const int nc = int(cp.size());
    for (size_t q = 0; q < m; ++q) {
      int nxp = ss_left(cp, s[q]);   // the first pivot at or ahead
      if (nxp < nc) a[q] = std::min(a[q], cp[std::min(nxp, nc - 1)]);
      int pr = ss_right(cp, s[q]) - 1;   // the last pivot at or behind
      in_pivot[q] = pr >= 0 && (s[q] - cp[std::max(pr, 0)] < 2 * c.off);
    }
  }
  for (size_t q = 0; q < m; ++q) {
    int sa = std::clamp(ss_right(cum, a[q]) - 1, 0, n - 2);
    double fa = (a[q] - cum[sa]) / std::max(L[sa], 1e-9);
    V2 qq = P[sa] + (P[sa + 1] - P[sa]) * fa;
    V2 d = qq - p[q];
    if (std::hypot(d.x(), d.y()) < 1e-6) d = P[seg[q] + 1] - P[seg[q]];
    yaw[q] = std::atan2(d.y(), d.x());
  }
  std::vector<uint8_t> ok = poses_free(p, yaw);
  for (size_t q = 0; q < m; ++q) ok[q] = ok[q] | in_pivot[q];
  // the preferred clearance (prune, sharpen): what each pose is held to, at least the margin
  const bool soft = ref && c.prefer_clearance > c.margin;
  std::vector<double> need(m, c.margin);
  if (soft)
    for (size_t q = 0; q < m; ++q)
      if (!in_pivot[q]) need[q] = std::max(c.margin, ref->need(p[q], r0, r1));
  std::vector<uint8_t> near_start(m, 0);
  if (start) {
    // near the start the point may not come nearer an obstacle than the start already is (nor nearer
    // than the bare half-width): no shortcut through a wall next to it. The frame (start_frame) is held
    // to the margin (and the preferred clearance) as everywhere where the start pose has that much;
    // start_soft: where it has less, to the start pose's own clearance (the first pose checked here,
    // at the follower's heading) -- a robot beside a wall drives on along it, but no nearer
    double lim = std::min(FOOTPRINT_WID / 2, clearance_at(*start)) - 1e-9;
    double m0 = NaN;
    for (size_t q = 0; q < m; ++q) {
      double d = std::hypot(p[q].x() - start->x(), p[q].y() - start->y());
      if (!(d < skip)) continue;
      near_start[q] = 1;
      int ci = int(std::floor((p[q].y() - lo.y()) / c.res)), cj = int(std::floor((p[q].x() - lo.x()) / c.res));
      double cl = (ci >= 0 && cj >= 0 && ci < ny && cj < nx) ? clear[size_t(ci) * nx + cj] : 0.0;
      bool fine = cl >= lim;
      if (fine && c.start_frame && !in_pivot[q]) {
        // with the robot's heading known, the pose turns from it to the path's over the stretch (the
        // robot's heading lags the path; Gate L2 w1: the path's heading at the robot, 12 deg off its own,
        // already sat at a door jamb, so the start's own clearance read low and the limit was lax); a
        // turn over pivot_angle is a start pivot, its circle checked by pivot_pass
        double yq = yaw[q];
        if (start_yaw && std::abs(wrap(yaw[q] - *start_yaw)) <= c.pivot_angle)
          yq = *start_yaw + wrap(yaw[q] - *start_yaw) * std::min(d / std::max(skip, 1e-9), 1.0);
        if (std::isnan(m0)) m0 = frame_margin(*this, p[q], yq, std::max(c.margin, c.prefer_clearance));
        double r = c.start_soft ? std::min(need[q], m0) : need[q];
        fine = frame_margin(*this, p[q], yq, r) >= r - 1e-9;
      }
      ok[q] = fine;
    }
  }
  if (soft)
    for (size_t q = 0; q < m; ++q)
      if (ok[q] && !near_start[q] && need[q] > c.margin + 1e-9)
        ok[q] = frame_margin(*this, p[q], yaw[q], need[q]) >= need[q] - 1e-9;
  size_t first = m;
  for (size_t q = 0; q < m; ++q)
    if (!ok[q]) {
      first = q;
      break;
    }
  // the driven bends (cut_check): the carrot chase cuts inside a bend under pivot_angle, the robot taking
  // about the chord from cut_lead before the vertex to cut_lead after it (along the path, so bends closer
  // than that share one chord, as the chase takes them); its frame along the chord, heading along it,
  // keeps the margin (near the start: the start check's limit). Not across a pivot (the carrot holds
  // there and the robot drives straight in, and leaves it heading along the path)
  if (c.cut_check && c.cut_lead > 0) {
    const double s_lo = cum[k0], s_hi = cum[n - 1];
    double m_start = NaN;
    for (int k = std::max(k0 + 1, 1); k < n - 1; ++k) {
      if (std::binary_search(pv.begin(), pv.end(), k)) continue;
      if (first < m && s[first] <= cum[k] - c.cut_lead) break;   // a failure before this bend's chord
      double turn = std::abs(wrap(hd(P[k + 1] - P[k]) - hd(P[k] - P[k - 1])));
      if (turn < radians(c.cut_min_deg)) continue;
      double sa = std::max(cum[k] - c.cut_lead, s_lo), sb = std::min(cum[k] + c.cut_lead, s_hi);
      for (int v : pv) {
        if (v < k) sa = std::max(sa, cum[v] + 2 * c.off);   // past a pivot's leaving point
        if (v > k) sb = std::min(sb, cum[v]);
      }
      if (sb - sa < 1e-6) continue;
      auto at = [&](double sv) {
        int sg = std::clamp(ss_right(cum, sv) - 1, 0, n - 2);
        return V2(P[sg] + (P[sg + 1] - P[sg]) * ((sv - cum[sg]) / std::max(L[sg], 1e-9)));
      };
      // the cut: the Bezier curve on the path's polyline from sa to sb (tangent to the path at both ends,
      // inside every bend between; two bends a few decimetres apart make one curve)
      std::vector<V2> ctl{at(sa)};
      for (int v = 1; v < n - 1; ++v)
        if (cum[v] > sa + 1e-9 && cum[v] < sb - 1e-9) ctl.push_back(P[v]);
      ctl.push_back(at(sb));
      const int nc = int(ctl.size());
      if (nc < 3) continue;
      auto bez = [&](double t) {
        std::vector<V2> w = ctl;
        for (int r = nc - 1; r > 0; --r)
          for (int i = 0; i < r; ++i) w[i] = w[i] * (1 - t) + w[i + 1] * t;
        return w[0];
      };
      auto bez_d = [&](double t) {   // the derivative: (nc - 1) x the Bezier of the control differences
        std::vector<V2> w(nc - 1);
        for (int i = 0; i + 1 < nc; ++i) w[i] = ctl[i + 1] - ctl[i];
        for (int r = nc - 2; r > 0; --r)
          for (int i = 0; i < r; ++i) w[i] = w[i] * (1 - t) + w[i + 1] * t;
        return w[0];
      };
      int nq = int(std::ceil((sb - sa) / step));
      for (int t = 0; t <= nq; ++t) {
        double tt = double(t) / nq;
        V2 x = bez(tt), dv = bez_d(tt);
        if (norm2(dv) < 1e-9) continue;
        double h = hd(dv);
        double r = c.margin;
        if (start && std::hypot(x.x() - start->x(), x.y() - start->y()) < skip) {
          if (!c.start_frame) continue;
          if (std::isnan(m_start)) m_start = frame_margin(*this, *start, start_yaw ? *start_yaw : h, c.margin);
          if (c.start_soft) r = std::min(r, m_start);
        }
        if (frame_margin(*this, x, h, r) < r - 1e-9) {
          // the bend's chord is blocked: fail at the bend (its incoming segment), unless a pose on the
          // path failed earlier
          double sv = sa + (sb - sa) * (double(t) / nq);
          if (first < m && s[first] <= sv) break;
          fail_at = x;
          return k - 1;
        }
      }
    }
  }
  if (first < m) {
    fail_at = p[first];   // the first blocked pose (odometry point)
    return seg[first];
  }
  return std::nullopt;
}

// ---------------------------------------------------------------- free-space evidence
Evidence scan_evidence(const HeightGrid& hg, const Cloud& pts, const Eigen::Vector3d& s, bool walls, bool low_sensor) {
  const Config& c = *hg.cfg;
  const double r = c.res;
  const int ny = hg.ny, nx = hg.nx;
  // the floor under the sensor. A tile with no floor seen has no level of its own (its ground is the
  // nearest seen tile's stand-in, which may be a lower floor beside the robot): the highest floor seen
  // within a tile of the sensor (the higher the floor, the later a ray enters the band: fewer
  // see-throughs); none seen there: no see-through evidence from this scan, its hits only
  auto seen = [&](size_t k) { return hg.ground_seen.empty() || hg.ground_seen[k] != 0; };
  Cell sc = hg.cell(V2(s.x(), s.y()));
  double g0 = NaN;
  bool rays = true;
  if (hg.inside(sc) && seen(hg.idx(sc.first, sc.second))) {
    g0 = hg.ground[hg.idx(sc.first, sc.second)];
  } else if (hg.inside(sc)) {
    const int m = int(std::nearbyint(c.tile / r));
    for (int i = std::max(0, sc.first - m); i <= std::min(ny - 1, sc.first + m); ++i)
      for (int j = std::max(0, sc.second - m); j <= std::min(nx - 1, sc.second + m); ++j)
        if (seen(hg.idx(i, j)) && !(hg.ground[hg.idx(i, j)] <= g0)) g0 = hg.ground[hg.idx(i, j)];
    rays = !std::isnan(g0);
  } else {
    std::vector<double> gk;
    for (auto& p : pts) {
      int64_t j = int64_t(std::floor((p.x - hg.lo.x()) / r)), i = int64_t(std::floor((p.y - hg.lo.y()) / r));
      double rng = std::hypot(p.x - s.x(), p.y - s.y());
      if (j >= 0 && i >= 0 && j < nx && i < ny && rng > 0.3 && rng < c.ray_range) gk.push_back(hg.ground[i * nx + j]);
    }
    g0 = gk.empty() ? 0.0 : median(gk);
  }
  struct Pt {
    double rng, h, dx, dy;
    int k;
  };
  std::vector<Pt> v;
  v.reserve(pts.size());
  for (auto& p : pts) {
    double dx = p.x - s.x(), dy = p.y - s.y();
    double rng = std::hypot(dx, dy);
    int64_t j = int64_t(std::floor((p.x - hg.lo.x()) / r)), i = int64_t(std::floor((p.y - hg.lo.y()) / r));
    if (!(j >= 0 && i >= 0 && j < nx && i < ny && rng > 0.3 && rng < c.ray_range)) continue;
    int k = int(i * nx + j);
    // each return against its own tile's ground (the per-scan grid's local model); in a tile with no
    // floor seen, against the sensor's floor (unknown: none there, the return is left out)
    double gk = seen(size_t(k)) ? hg.ground[k] : g0;
    if (std::isnan(gk)) continue;
    v.push_back({rng, p.z - gk, dx, dy, k});
  }
  Evidence ev;
  // a hit is anything from z_lo up to z_tall (a wall seen only above the band still counts); a cell
  // is cleared only where rays show the band itself free more often (an overhang, a person)
  const bool overhangs = low_sensor && c.overhang_clear > c.z_lo;
  // a return is low when its voxel's bottom (the scans are voxel centres, res/2 high) is at or under
  // overhang_clear, as HeightGrid::low
  const double oc_low = c.overhang_clear + r * 0.25;
  for (auto& q : v) {
    bool band = q.h > c.z_lo && q.h < c.z_hi;
    if (low_sensor ? (q.h > c.z_lo && q.h < c.z_tall) : band) ev.hit.push_back(q.k);
    if (walls && q.h > c.z_lo && q.h < c.z_tall && q.rng >= c.mem_min_range) ev.wall.push_back(q.k);
    if (overhangs && q.h > c.z_lo && q.h <= oc_low) ev.low.push_back(q.k);
  }
  auto uniq = [](std::vector<int>& a) {
    std::sort(a.begin(), a.end());
    a.erase(std::unique(a.begin(), a.end()), a.end());
  };
  uniq(ev.hit);
  uniq(ev.wall);
  uniq(ev.low);
  if (!rays) return ev;
  // the rays' band crossings below take the ground as a straight ramp from the sensor's floor to each
  // return's own (the same formula as flat ground, with heights above the local ground)
  const double hs = s.z() - g0;
  const int nb = c.ray_bins;
  std::vector<double> steps = arange(0.3, c.ray_range, r * 0.5);
  // the cells rays saw through in the slab (z_lo, top): per bin from the nearest ray's entry into the slab to
  // the farthest exit, cut 1.5 res before the nearest return in (z_lo, cut) (inclusive: (z_lo, cut]);
  // band_model: the whole-map model (from the sensor, to the farthest return)
  auto see_through = [&](double top, double cut, bool inclusive, bool band_model) {
    std::vector<double> start(nb, INF), reach(nb, 0.0), near(nb, INF), far(nb, 0.0);
    for (auto& q : v) {
      int b = int(int64_t((std::atan2(q.dy, q.dx) + PI) / (2 * PI) * nb) % nb);
      // each ray's stretch inside the slab (ranges where its height is in [z_lo, top])
      bool up = q.h > hs + 1e-3;
      double enter = hs >= c.z_lo ? 0.0 : q.rng * (c.z_lo - hs) / (q.h - hs);
      double leave = hs >= top ? 0.0 : np_min(q.rng, q.rng * (top - hs) / (q.h - hs));
      if (hs >= c.z_lo) {   // sensor in the slab (hovering low): level rays count too
        up = up || (q.h > c.z_lo) || (hs < top);
        if (q.h <= hs)
          leave = np_min(q.rng, q.h < c.z_lo ? q.rng * (hs - c.z_lo) / std::max(hs - q.h, 1e-9) : q.rng);
      }
      if (up && q.h > c.z_lo && leave > enter) {
        start[b] = std::min(start[b], enter);
        reach[b] = std::max(reach[b], leave);
      }
      if (q.h > c.z_lo && (inclusive ? q.h <= cut : q.h < cut)) near[b] = std::min(near[b], q.rng);
      far[b] = std::max(far[b], q.rng);
    }
    for (int b = 0; b < nb; ++b) {
      if (!band_model) {
        reach[b] = (std::isfinite(near[b]) ? near[b] : reach[b]) - 1.5 * r;
      } else {
        start[b] = 0.0;
        reach[b] = (std::isfinite(near[b]) ? near[b] : far[b]) - 1.5 * r;
      }
      start[b] = std::max(start[b], 0.3);
    }
    std::vector<int> fr;
    for (int b = 0; b < nb; ++b) {
      double ang = (b + 0.5) / nb * 2 * PI - PI;
      double ca = std::cos(ang), sa = std::sin(ang);
      for (double R : steps) {
        if (!(R < reach[b] && R >= start[b])) continue;
        int64_t ci = int64_t(std::floor((s.x() + R * ca - hg.lo.x()) / r));
        int64_t cj = int64_t(std::floor((s.y() + R * sa - hg.lo.y()) / r));
        if (ci >= 0 && cj >= 0 && ci < nx && cj < ny) fr.push_back(int(cj * nx + ci));
      }
    }
    uniq(fr);
    return fr;
  };
  std::vector<int> fr = see_through(c.z_hi, c.z_hi, false, !low_sensor);
  std::set_difference(fr.begin(), fr.end(), ev.hit.begin(), ev.hit.end(), std::back_inserter(ev.free));
  if (overhangs) {
    // beneath overhangs: the same rays in (z_lo, overhang_clear], cut only by returns in it (a table top above
    // does not cut them; its legs, clutter below it and a wall's base do); cells hit in it are left out
    std::vector<int> fu = see_through(c.overhang_clear, oc_low, true, false);
    std::set_difference(fu.begin(), fu.end(), ev.low.begin(), ev.low.end(), std::back_inserter(ev.under));
  }
  return ev;
}

void evidence(const HeightGrid& hg, const std::vector<std::pair<Cloud, Eigen::Vector3d>>& scans, std::vector<int>& hits,
              std::vector<int>& miss, bool low_sensor, std::vector<int>* under, std::vector<int>* low_hits) {
  size_t n = size_t(hg.ny) * hg.nx;
  hits.assign(n, 0);
  miss.assign(n, 0);
  if (under) under->assign(n, 0);
  if (low_hits) low_hits->assign(n, 0);
  for (auto& sc : scans) {
    Evidence e = scan_evidence(hg, sc.first, sc.second, false, low_sensor);
    for (int k : e.hit) hits[k]++;
    for (int k : e.free) miss[k]++;
    if (under)
      for (int k : e.under) (*under)[k]++;
    if (low_hits)
      for (int k : e.low) (*low_hits)[k]++;
  }
}

// ---------------------------------------------------------------- search
// Dijkstra over the free cells, 8-connected without corner cutting; step cost grows near obstacles
// (keep to the middle), where the frame could not turn round, and on unknown cells
std::optional<std::vector<Cell>> search(const Window& win, Cell start, Cell goal) {
  const Config& c = *win.cfg;
  const int ny = win.ny, nx = win.nx;
  const size_t N = size_t(ny) * nx;
  std::vector<double> pen(N);
  for (size_t q = 0; q < N; ++q) {
    double cl = win.clear[q];
    double p = 1.0 + c.clear_weight * std::clamp((c.prefer_clear - cl) / c.prefer_clear, 0.0, 1.0);
    p += c.pivot_weight * (cl <= c.r_out ? 1.0 : 0.0);
    if (c.unknown_free) p += c.unknown_cost * (win.known[q] ? 0.0 : 1.0);
    pen[q] = p;
  }
  const double h1 = std::hypot(1.0, 0.0) * 0.5, h2 = std::hypot(1.0, 1.0) * 0.5;
  auto F = [&](int i, int j) { return i >= 0 && j >= 0 && i < ny && j < nx && win.free[size_t(i) * nx + j]; };
  std::vector<FNode> nodes(N);
  for (size_t q = 0; q < N; ++q) nodes[q].index = unsigned(q);
  std::vector<int> pred(N, -9999);
  FHeap heap;
  for (auto& rr : heap.roots_by_rank) rr = nullptr;
  const int si = start.first * nx + start.second, gi = goal.first * nx + goal.second;
  nodes[si].state = 0;
  nodes[si].val = 0;
  heap.insert(&nodes[si]);
  bool found = false;
  // the CSR graph's neighbours in column order (scipy's sorted indices)
  static const int NB[8][2] = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};
  while (heap.min_node) {
    FNode* v = heap.remove_min();
    v->state = 0;
    if (int(v->index) == gi) {
      found = true;
      break;
    }
    int i = int(v->index) / nx, j = int(v->index) % nx;
    if (!win.free[v->index]) continue;   // the start cell is always free (nearest_free)
    for (auto& o : NB) {
      int ii = i + o[0], jj = j + o[1];
      if (!F(ii, jj)) continue;
      bool diag = o[0] && o[1];
      if (diag && !(F(i, jj) && F(ii, j))) continue;   // no corner cutting
      size_t u = size_t(ii) * nx + jj;
      FNode* cn = &nodes[u];
      if (cn->state == 0) continue;
      double w = (diag ? h2 : h1) * (pen[v->index] + pen[u]);
      double nv = v->val + w;
      if (cn->state == 1) {
        cn->state = 2;
        cn->val = nv;
        heap.insert(cn);
        pred[u] = int(v->index);
      } else if (cn->val > nv) {
        heap.decrease(cn, nv);
        pred[u] = int(v->index);
      }
    }
  }
  if (!found) return std::nullopt;
  std::vector<Cell> path;
  int k = gi;
  path.push_back({k / nx, k % nx});
  while (k != si) {
    k = pred[k];
    path.push_back({k / nx, k % nx});
  }
  std::reverse(path.begin(), path.end());
  return path;
}

// drop waypoints while the shortcut, followed by the carrot follower from the previous vertex, keeps
// the oriented frame clear -> (path, index of a blocked segment or none). From each vertex: the goal
// if it checks, else gallop out (+1, +2, +4 ...) to the first failure and bisect back. With ref the
// shortcut must also keep the preferred clearance where the vertices it replaces had it (each pose
// against the nearest of them): else the cell path's vertex stays, so the path keeps the room the
// search gave it instead of being pulled to the margin (the next vertex is always taken)
std::pair<Path, std::optional<int>> prune(Window& win, const Path& pts, const ClearRef* ref) {
  if (pts.size() <= 2) return {pts, win.follow_free(pts)};
  Path out{pts[0]};
  std::vector<int> oi{0};   // out's vertices in pts
  int i = 0;
  const int n = int(pts.size());
  auto ok = [&](int j) {
    Path q;
    if (out.size() >= 2) q.push_back(out[out.size() - 2]);
    q.push_back(out.back());
    q.push_back(pts[j]);
    int r0 = oi[out.size() >= 2 ? out.size() - 2 : 0];
    return !win.follow_free(q, 0, ref, r0, j).has_value();
  };
  while (i < n - 1) {
    int best;
    if (ok(n - 1)) {
      best = n - 1;
    } else {
      int good = i + 1, step = 1, bad = n - 1;
      while (good + step < bad) {
        if (ok(good + step)) {
          good += step;
          step *= 2;
        } else {
          bad = good + step;
          break;
        }
      }
      while (bad - good > 1) {
        int mid = (good + bad) / 2;
        if (ok(mid))
          good = mid;
        else
          bad = mid;
      }
      best = good;
    }
    out.push_back(pts[best]);
    oi.push_back(best);
    i = best;
  }
  auto bad = win.follow_free(out);
  return {out, bad};
}

V2 point_along(const Path& pts, double dist) {
  double left = dist;
  for (size_t k = 0; k + 1 < pts.size(); ++k) {
    const V2 &a = pts[k], &b = pts[k + 1];
    double L = norm2(b - a);
    if (L >= left) return L > 0 ? V2(a + (b - a) * (left / L)) : a;
    left -= L;
  }
  return pts.back();
}

namespace {
// the vertex the robot heads for after a pivot about C at vertex k: the first one past k farther from
// C than the odometry offset plus a cell (nearer ones lie in the pivot)
int depart(const Path& P, int k, const V2& C, const Config& cfg) {
  int j = k + 1;
  while (j < int(P.size()) - 1 && norm2(P[j] - C) < cfg.off + cfg.res) ++j;
  return j;
}

struct IsPivot {
  bool pivot;
  V2 C;
  int j;
};
IsPivot is_pivot(const Path& P, int k, double h_in, const V2& A, const Config& cfg, double thr) {
  V2 C = A - cfg.off * unit(h_in);
  int j = depart(P, k, C, cfg);
  Path tail(P.begin() + k, P.end());
  V2 q = point_along(tail, cfg.lookahead);
  if (norm2(q - A) <= 1e-6 || std::abs(wrap(hd(q - A) - h_in)) <= thr) return {false, C, j};
  return {norm2(P[j] - C) >= cfg.off + cfg.res && std::abs(wrap(hd(P[j] - C) - h_in)) > thr, C, j};
}

// A turn split into bends close together (owner request 2026-10-30, the hairpin drive chain: two 43-45 deg
// bends 0.42 m apart, each under pivot_angle, were driven as one 88 deg corner and the carrot chase cut
// inside it). The vertices within bend_merge of each other along the path, starting at a vertex (or at
// the start, with the robot's heading), turning by more than pivot_angle in all (from the heading into the
// first to the segment leaving the last) become one vertex where those two lines meet, if the frame can
// follow that path: a sharp corner, which pivot_pass makes a pivot (its hard check). Otherwise they stay
// driven bends (the cut check in follow_free keeps their inside cut clear).
Path merge_bends(Window& win, const Path& P0, std::optional<double> start_yaw) {
  const Config& cfg = *win.cfg;
  Path Q = P0;
  if (cfg.bend_merge <= 0) return Q;
  for (int guard = 0; guard < 8; ++guard) {
    const int n = int(Q.size());
    if (n < 3) return Q;
    std::vector<double> cum(n, 0.0);
    for (int k = 1; k < n; ++k) cum[k] = cum[k - 1] + norm2(Q[k] - Q[k - 1]);
    bool changed = false;
    for (int k = start_yaw ? 0 : 1; k + 1 < n && !changed; ++k) {
      if (k > 0 && cum[k] - cum[k - 1] < 1e-6) continue;
      double h_in = k == 0 ? *start_yaw : hd(Q[k] - Q[k - 1]);
      int m = k;
      while (m + 2 < n && cum[m + 1] - cum[k] <= cfg.bend_merge) ++m;
      if (m == k || cum[m + 1] - cum[m] < 1e-6) continue;
      // each bend in the cluster driven (under pivot_angle); a pivot there is one already
      bool driven = true;
      double prev = h_in;
      for (int v = k; v <= m && driven; ++v) {
        double ho = hd(Q[v + 1] - Q[v]);
        driven = std::abs(wrap(ho - prev)) <= cfg.pivot_angle;
        prev = ho;
      }
      double h_out = hd(Q[m + 1] - Q[m]);
      if (!driven || std::abs(wrap(h_out - h_in)) <= cfg.pivot_angle) continue;
      V2 u = unit(h_in), v = unit(h_out), w = Q[m] - Q[k];
      double den = u.x() * v.y() - u.y() * v.x();
      if (std::abs(den) < 0.2) continue;
      double t = (w.x() * v.y() - w.y() * v.x()) / den;    // along the incoming line from Q[k]
      double s2 = (w.x() * u.y() - w.y() * u.x()) / den;   // along the outgoing line from Q[m]
      if (t < 0 || t > cfg.bend_merge + 0.5 || s2 >= cum[m + 1] - cum[m] - 0.05) continue;
      Path cand(Q.begin(), Q.begin() + (k == 0 ? 1 : k));
      V2 X = Q[k] + t * u;
      if (norm2(X - cand.back()) > 1e-3) cand.push_back(X);
      cand.insert(cand.end(), Q.begin() + m + 1, Q.end());
      if (!win.follow_free(cand).has_value()) {
        Q = cand;
        changed = true;
      }
    }
    if (!changed) break;
  }
  return Q;
}

PivotResult pivot_fail(const std::string& reason, Block b) {
  PivotResult r;
  r.ok = false;
  r.reason = reason;
  r.block = std::move(b);
  return r;
}

// the frame can drive off from a pivot about C toward P[j] (the leg's first two segments)
bool departs(Window& win, const Path& P, int j, const V2& C) {
  V2 V = C + win.cfg->off * unit(hd(P[j] - C));
  Path q{V};
  for (int k = j; k < std::min(int(P.size()), j + 2); ++k) q.push_back(P[k]);
  return !win.follow_free(q).has_value();
}

struct Moved {
  V2 C, A;
  double h;
  int j;
};
// a pivot whose circle is not clear: the nearest centre within pivot_search where it is, reached from
// out[-1] by driving (no other pivot) with the frame clear
std::optional<Moved> move_pivot(Window& win, const Path& out, int leg0, std::optional<double> h_arr, const Path& P,
                                int k, const V2& C, bool start) {
  const Config& cfg = *win.cfg;
  const V2 prev = out.back();
  Cell c0 = win.cell(C);
  int m = int(std::ceil(cfg.pivot_search / cfg.res));
  int i0 = std::max(0, c0.first - m), j0 = std::max(0, c0.second - m);
  int i1 = std::min(win.ny, std::max(0, c0.first + m + 1)), j1 = std::min(win.nx, std::max(0, c0.second + m + 1));
  struct Cand {
    V2 xy, A;
    double h, d;
  };
  std::vector<Cand> cand;
  const double lim = cfg.pivot_radius - 0.75 * cfg.res;
  for (int i = i0; i < i1; ++i)
    for (int j = j0; j < j1; ++j) {
      if (!(win.clear[size_t(i) * win.nx + j] >= lim)) continue;
      V2 xy(win.lo.x() + (j + 0.5) * cfg.res, win.lo.y() + (i + 0.5) * cfg.res);
      double d = std::hypot(xy.x() - C.x(), xy.y() - C.y());
      if (!(d <= cfg.pivot_search)) continue;
      // the arrival heading (from out[-1], twice: the odometry point is footprint_offset ahead of the
      // centre) and arrival point
      V2 v = xy - prev;
      double h = std::atan2(v.y(), v.x());
      V2 A = xy + cfg.off * V2(std::cos(h), std::sin(h));
      v = A - prev;
      h = std::atan2(v.y(), v.x());
      A = xy + cfg.off * V2(std::cos(h), std::sin(h));
      bool keep = std::hypot((A - prev).x(), (A - prev).y()) >= cfg.res;
      // reached without another pivot: from the start, the bearing from the frame centre (what Rover
      // turns by); elsewhere the turn at the previous vertex
      if (start) {
        V2 fc = prev - cfg.off * unit(*h_arr);
        keep = keep && std::abs(wrap(std::atan2(A.y() - fc.y(), A.x() - fc.x()) - *h_arr)) <= cfg.pivot_angle;
      } else if (h_arr) {
        keep = keep && std::abs(wrap(h - *h_arr)) <= cfg.pivot_angle;
      }
      // the clearance (cell-centre transform) along the tolerance either side, before the exact check
      for (double t : {-cfg.pivot_tol, cfg.pivot_tol}) {
        V2 e = xy + t * V2(std::cos(h), std::sin(h));
        int ci = int(std::floor((e.y() - win.lo.y()) / cfg.res)), cj = int(std::floor((e.x() - win.lo.x()) / cfg.res));
        double cl = (ci >= 0 && cj >= 0 && ci < win.ny && cj < win.nx) ? win.clear[size_t(ci) * win.nx + cj] : 0.0;
        keep = keep && cl >= lim;
      }
      if (keep) cand.push_back({xy, A, h, d});
    }
  // nearest first, ties in row-major order (a stable sort). np.argsort broke ties by its own
  // introsort (numpy 1.17) or SIMD sort (numpy 2.x): the Python planner's order depended on its numpy
  std::stable_sort(cand.begin(), cand.end(), [](const Cand& a, const Cand& b) { return a.d < b.d; });
  int tries = 0;
  for (auto& o : cand) {
    if (tries++ >= cfg.pivot_tries) break;
    int jn = depart(P, k, o.xy, cfg);
    if (std::abs(wrap(hd(P[jn] - o.xy) - o.h)) <= cfg.pivot_angle) continue;   // no pivot there
    if (!win.pivot_free(o.xy, o.h, cfg.pivot_tol)) continue;
    Path leg(out.begin() + std::max(leg0, int(out.size()) - 2), out.end());
    leg.push_back(o.A);
    if (win.follow_free(leg).has_value()) continue;
    if (!departs(win, P, jn, o.xy)) continue;
    return Moved{o.xy, o.A, o.h, jn};
  }
  return std::nullopt;
}
}  // namespace

// The robot's arc onto a driving leg (owner request 2026-10-29, after the Gate L2 door-jamb clips in
// SITL c1 and c3: the robot landed 33-35 deg off the drive leg's first bearing, under pivot_angle, so
// no start pivot; it arced onto the path with its heading lagging 7-10 deg all the way to the 0.90 m
// door and clipped its jamb by 7.5 cm, which the follower model, heading at the carrot, does not see).
// The robot is simulated as the node drives it: its target is the carrot (follow_plan) run out by up
// to arc_reach (run_out, frame-checked), and Rover's fast-update path turns toward it with a curvature
// of 2 sin(error) / max(distance, arc_steer); arc_steer 3.0 m fits the heading rate on SITL's c1, c2,
// c3 and d18 drives (Rover turns about as lazily as pure pursuit on a point 2.5-3.2 m away, though its
// target is ~1.3 m away). The FC (Rover's turning point) sits at the frame centre, footprint_offset
// behind the odometry point. A bearing error over pivot_angle is a pivot in place (swept every 5 deg).
// The simulation runs every res/2 of FC travel until the target is within 0.3 m (Rover's WP_RADIUS) at
// the leg's end, or arc_max. A pose fails when its frame plus margin holds an obstacle cell; within
// start_skip of travel the cells the start pose already holds do not count (the robot is already
// there, as follow_free's start skip). The leg is taken as one driving leg (no pivots on it).
std::optional<ArcFail> arc_fail(const Window& win, const Path& leg, const V2& p0, double yaw0) {
  const Config& c = *win.cfg;
  const int n = int(leg.size());
  if (n < 2) return std::nullopt;
  const double ds = c.res * 0.5;
  const std::vector<int> h0 = win.pose_hits(p0, yaw0, c.arc_margin);
  double s = 0;
  auto seg_of = [&](const V2& p) {
    double best = 1e18;
    int bk = 0;
    for (int k = 0; k + 1 < n; ++k) {
      V2 ab = leg[k + 1] - leg[k];
      double L2 = ab.dot(ab), t = L2 == 0 ? 0.0 : std::clamp((p - leg[k]).dot(ab) / L2, 0.0, 1.0);
      double d = norm2(leg[k] + ab * t - p);
      if (d < best) {
        best = d;
        bk = k;
      }
    }
    return bk;
  };
  auto blocked = [&](const V2& p, double y) {
    for (int k : win.pose_hits(p, y, c.arc_margin))
      if (k < 0 || s >= c.start_skip || !std::binary_search(h0.begin(), h0.end(), k)) return true;
    return false;
  };
  V2 fc = p0 - c.off * unit(yaw0);
  double y = yaw0;
  for (int guard = 0; s < c.arc_max && guard < 4000; ++guard) {
    V2 odom = fc + c.off * unit(y);
    Follow f = follow_plan(leg, {}, odom, y, c);
    V2 tg = run_out(leg, {}, odom, y, f.c, f.phase, f.j, c, c.arc_reach, c.off, 0.15, &win).first;
    V2 d = tg - fc;
    double dist = norm2(d);
    if (dist < 0.3 && norm2(tg - leg.back()) < 1e-6) break;   // arrived: Rover stops within WP_RADIUS
    if (dist < 1e-6) break;
    double err = wrap(hd(d) - y);
    if (std::abs(err) > c.pivot_angle) {   // Rover pivots in place about the FC
      int m = int(std::ceil(std::abs(err) / radians(5.0)));
      for (int q = 1; q <= m; ++q) {
        double yy = y + err * q / m;
        V2 pp = fc + c.off * unit(yy);
        if (blocked(pp, yy)) return ArcFail{pp, seg_of(pp), s};
      }
      y = hd(d);
      continue;
    }
    y += 2.0 * std::sin(err) / std::max(dist, c.arc_steer) * ds;
    fc += ds * unit(y);
    s += ds;
    V2 p = fc + c.off * unit(y);
    if (blocked(p, y)) return ArcFail{p, seg_of(p), s};
  }
  return std::nullopt;
}

// An arc failure the leg's own bend causes (SITL o4, 2026-10-30): the leg driven from its own first heading
// (the robot on the path, aligned) fails too, on a later segment. The start heading has no part in it: a start
// pivot does not clear it and a stop does not change it. o4: a 56 deg driven bend round the crate past the
// door, 2-3 m ahead, whose lagging model cleared or touched the crate's corner cell by a few cm from replan to
// replan; each touch refused the whole route (NO_PATH in 45 of 70 status messages, a 3.5 s hesitation before
// the table) or made the start a pivot (the robot swung 50 deg at the leg's start).
static bool bend_fail(const Window& win, const Path& leg, const ArcFail& f) {
  if (leg.size() < 3 || f.seg < 1) return false;
  auto r = arc_fail(win, leg, leg[0], hd(leg[1] - leg[0]));
  return r && r->seg >= 1;
}

std::vector<int> pivot_vertices(const Path& P, const Config& cfg) {
  std::vector<int> out;
  for (int k = 1; k < int(P.size()) - 1; ++k)
    if (is_pivot(P, k, hd(P[k] - P[k - 1]), P[k], cfg, cfg.pivot_angle).pivot) out.push_back(k);
  return out;
}

// turns over pivot_angle as pivots, each its own step -> PivotResult(ok, path, pivots, reason, block).
//
// Vertex k turns when the chord to the point lookahead further along leaves the incoming heading by
// more than pivot_angle. The robot drives straight into the vertex (the carrot holds), so it stops
// with its odometry point at A = P[k] and its frame centre at C = A - footprint_offset along the
// incoming heading, pivots about C toward the next vertex past the pivot, and leaves with its
// odometry point at V = C + footprint_offset along the new heading: the path gets V after A, and
// pivots[n] = {i: index of A (V is i + 1), centre: C, yaw_in, yaw_out, turn, start}. A turn under
// pivot_angle seen from C is not a pivot (Rover only pivots past WP_PIVOT_ANGLE). The pivot circle
// about C (and up to pivot_tol before or past it) must be clear; else the nearest centre within
// pivot_search where it is, reached by driving; else the pivot fails with the cells to block (block)
// for another search.
//
// With start_yaw (the robot's heading) the start is a pivot too when the bearing from the frame
// centre to the carrot leaves it by more than pivot_angle (pivot_done when start_pivoting, a pivot
// under way); the start circle is checked where the robot stands and up to pivot_tol ahead.
//
// The arc check (Config::arc_check, with the heading known): the robot's own arc onto each driving
// leg must be clear (arc_fail). At the start a turn under pivot_angle whose arc is not clear becomes a
// start pivot. Any other arc failure (the heading lagging on a gentle turn) is a refusal (retry false:
// no other search); with Config::arc_vertices (off by default) a leg after a pivot first gets a pivot
// at the vertex of the largest such turn before the failing pose (driven straight in, pivoted, driven
// straight out; the pivot's hard checks apply) and the pass runs again, up to 3 such vertices.
namespace {
PivotResult pivot_pass_forced(Window& win, const Path& path, std::optional<double> start_yaw, bool start_pivoting,
                              const std::set<int>& forced, int* force_next);
}  // namespace

PivotResult pivot_pass(Window& win, const Path& path0, std::optional<double> start_yaw, bool start_pivoting) {
  // a turn split into driven bends close together: one corner, a pivot (merge_bends); not while a pivot
  // is under way (it finishes on its own route)
  const Path path = start_pivoting ? path0 : merge_bends(win, path0, start_yaw);
  std::set<int> forced;
  while (true) {
    int next = -1;
    PivotResult r = pivot_pass_forced(win, path, start_yaw, start_pivoting, forced, &next);
    if (r.ok || next < 0 || forced.count(next) || forced.size() >= 3) return r;
    forced.insert(next);
  }
}

namespace {
PivotResult pivot_pass_forced(Window& win, const Path& path, std::optional<double> start_yaw, bool start_pivoting,
                              const std::set<int>& forced, int* force_next) {
  const Config& cfg = *win.cfg;
  const Path& P = path;
  const int n = int(P.size());
  PivotResult res;
  Path& out = res.path;
  std::vector<int> src;   // per out point: its vertex of P, or -1 (a pivot's leaving point)
  // a pivot the arc check forced: failing from here on is a refusal, not another search (blocking cells
  // round an arc-forced pivot sent the search on detours: house_current keyframe 62)
  bool arc_involved = !forced.empty();
  out.push_back(P[0]);
  src.push_back(0);
  if (n < 2) return res;
  int leg0 = 0;
  std::optional<double> h_arr = start_yaw;
  int k = start_yaw ? 0 : 1;
  while (k < n - 1) {
    bool start = k == 0;
    double h_in, tol, thr;
    V2 A;
    if (start) {
      h_in = *start_yaw;
      A = P[0];
      // up to pivot_tol_start ahead: a replan may turn a robot that is still rolling; a pivot under
      // way turns in place, so only where it stands (ahead along a heading that sweeps with the pivot
      // swung the checked circles into the walls: the hairpin with the arc check's start pivots)
      tol = start_pivoting ? 0.0 : cfg.pivot_tol_start;
      thr = start_pivoting ? cfg.pivot_done : cfg.pivot_angle;
    } else {
      h_in = hd(P[k] - out.back());
      A = P[k];
      tol = cfg.pivot_tol;
      thr = cfg.pivot_angle;
    }
    bool turning;
    V2 C;
    int j;
    if (start) {
      // what Rover turns by from a standstill: the bearing from the frame centre to the carrot
      C = A - cfg.off * unit(h_in);
      j = depart(P, k, C, cfg);
      V2 q = point_along(P, cfg.lookahead);
      turning = std::abs(wrap(hd(q - C) - h_in)) > thr && norm2(P[j] - C) >= cfg.off + cfg.res &&
                std::abs(wrap(hd(P[j] - C) - h_in)) > thr;
      // a turn under pivot_angle is driven, the robot arcing onto the path with its heading lagging:
      // when that arc is not clear (arc_fail on the first leg), pivot here instead (the pivot's hard
      // check applies; the carrot leads the pivot so Rover pivots under WP_PIVOT_ANGLE: follow_plan)
      if (!turning && cfg.arc_check && std::abs(wrap(hd(q - C) - h_in)) > cfg.pivot_done &&
          norm2(P[j] - C) >= cfg.off + cfg.res) {
        auto pv = pivot_vertices(P, cfg);
        Path leg(P.begin(), P.begin() + (pv.empty() ? n : pv[0] + 1));
        auto af = arc_fail(win, leg, P[0], h_in);
        if (af && !bend_fail(win, leg, *af)) turning = arc_involved = true;   // a bend's: no start pivot
      }
    } else {
      IsPivot ip = is_pivot(P, k, h_in, A, cfg, thr);
      turning = ip.pivot;
      C = ip.C;
      j = ip.j;
      // a vertex the arc check forced (a gentle turn the lagging heading cannot follow): a pivot
      if (!turning && forced.count(k) && norm2(P[j] - C) >= cfg.off + cfg.res &&
          std::abs(wrap(hd(P[j] - C) - h_in)) > cfg.pivot_done)
        turning = true;
    }
    if (!turning) {
      if (!start) {
        out.push_back(A);
        src.push_back(k);
        h_arr = h_in;
      }
      ++k;
      continue;
    }
    // the start: where the robot stands and up to pivot_tol ahead (a replan can turn the robot on
    // the spot while it moves; Rover brakes, then pivots)
    bool room = win.pivot_free(C, h_in, tol, !start);
    if (!(room && departs(win, P, j, C))) {
      auto mv = move_pivot(win, out, leg0, h_arr, P, k, C, start);
      if (!mv) {
        std::string where = start ? std::string("at the start")
                                  : fmt("at (%.2f, %.2f), a %.0f deg turn", C.x(), C.y(),
                                        degrees(std::abs(wrap(hd(P[j] - C) - h_in))));
        std::string why = !room ? "no room to pivot " : "the frame cannot drive off the pivot ";
        Block b;
        if (start) {
          b.kind = Block::START;
          b.p = P[0];
          b.yaw = *start_yaw;
        } else {
          b.kind = Block::VERTEX;
          b.p = P[k];
        }
        PivotResult r = pivot_fail(why + where, b);
        r.retry = !arc_involved;
        return r;
      }
      C = mv->C;
      A = mv->A;
      h_in = mv->h;
      j = mv->j;
      out.push_back(A);   // driven to the moved pivot, then a pivot there
      src.push_back(k);
      start = false;
    } else if (!start) {
      out.push_back(A);
      src.push_back(k);
    }
    double h_out = hd(P[j] - C);
    Pivot pv;
    pv.i = int(out.size()) - 1;
    pv.centre = C;
    pv.yaw_in = h_in;
    pv.yaw_out = h_out;
    pv.turn = std::abs(wrap(h_out - h_in));
    pv.start = start;
    res.pivots.push_back(pv);
    out.push_back(C + cfg.off * unit(h_out));
    src.push_back(-1);
    leg0 = int(out.size()) - 1;
    h_arr = h_out;
    k = j;
  }
  out.push_back(P[n - 1]);
  src.push_back(n - 1);
  // each driving leg on its own: the carrot holds at a pivot, so the frame drives straight in; with
  // the robot's heading known, also the robot's own arc onto each leg (arc_fail: from the heading it
  // has there, the start's or a pivot's leaving one), which also covers the heading lagging the carrot
  // on the leg's gentle turns
  const bool arcs = cfg.arc_check && start_yaw.has_value();
  if (!res.pivots.empty() || arcs) {
    std::vector<int> starts{0}, ends;
    for (auto& p : res.pivots) {
      starts.push_back(p.i + 1);
      ends.push_back(p.i);
    }
    ends.push_back(int(out.size()) - 1);
    for (size_t q = 0; q < starts.size(); ++q) {
      int a = starts[q], b = ends[q];
      if (b - a + 1 < 2) continue;
      Path leg(out.begin() + a, out.begin() + b + 1);
      if (arcs) {
        auto f = arc_fail(win, leg, leg[0], q == 0 ? *start_yaw : res.pivots[q - 1].yaw_out);
        // the first leg without a start pivot: the robot heads along it already (a start off by more
        // than pivot_done with its arc not clear was made a start pivot); it is committed to its next
        // arc_commit metres (it cannot turn away there; earlier replans checked that stretch from
        // farther), so a failure there does not stop it (a stop would only flip-flop with the model)
        if (f && q == 0 && f->s < cfg.arc_commit) f.reset();
        if (f) {
          // the vertex of the largest turn over pivot_done on the leg up to the failing pose (not the
          // leg's end): a pivot there next time (pivot_pass)
          // (on the first leg always for a bend's failure, bend_fail: a pivot at the bend clears it, no stop does;
          // after a pivot only with arc_vertices, see Config::arc_vertices)
          double best = cfg.pivot_done;
          const bool vertices = cfg.arc_vertices || (q == 0 && bend_fail(win, leg, *f));
          for (int v = a + 1; vertices && v <= std::min(a + f->seg + 1, b - 1); ++v) {
            if (src[v] < 0) continue;
            double turn = std::abs(wrap(hd(out[v + 1] - out[v]) - hd(out[v] - out[v - 1])));
            if (turn > best) {
              best = turn;
              *force_next = src[v];
            }
          }
          Block bl;
          bl.kind = Block::SEGMENT;
          bl.seg = Path(leg.begin() + f->seg, leg.begin() + std::min(int(leg.size()), f->seg + 2));
          bl.at = f->at;
          win.fail_at = f->at;
          PivotResult r = pivot_fail("the frame cannot follow the arc onto the path", bl);
          r.retry = false;   // refused, not routed round: blocking cells there sent the search on detours
          return r;
        }
      }
      if (res.pivots.empty()) continue;   // (prune checked the whole route already)
      auto bad = win.follow_free(leg);
      if (bad) {
        Block bl;
        bl.kind = Block::SEGMENT;
        bl.seg = Path(leg.begin() + *bad, leg.begin() + std::min(int(leg.size()), *bad + 2));
        bl.at = win.fail_at;
        PivotResult r = pivot_fail("no path the oriented frame can follow", bl);
        r.retry = !arc_involved;
        return r;
      }
    }
  }
  return res;
}
}  // namespace

std::optional<Path> sharpen(Window& win, const Path& R, int bad, int max_skip, const ClearRef* ref) {
  const int n = int(R.size());
  // with ref: first the corners that keep the preferred clearance, then any the frame can follow
  for (int pass = ref ? 0 : 1; pass < 2; ++pass) {
    const ClearRef* rf = pass == 0 ? ref : nullptr;
    for (int b : {bad, bad - 1}) {
      if (b < 0 || b + 2 >= n) continue;
      V2 u = R[b + 1] - R[b];
      double Lu = norm2(u);
      if (Lu < 1e-9) continue;
      u = u / Lu;
      for (int m = b + 1; m < std::min(b + 1 + max_skip, n - 1); ++m) {
        V2 v = R[m + 1] - R[m];
        double Lv = norm2(v);
        if (Lv < 1e-9) continue;
        v = v / Lv;
        double den = u.x() * v.y() - u.y() * v.x();
        if (std::abs(den) < 0.2) continue;   // nearly parallel
        V2 w = R[m] - R[b];
        double t = (w.x() * v.y() - w.y() * v.x()) / den;   // along the incoming line from R[b]
        double s = (w.x() * u.y() - w.y() * u.x()) / den;   // along segment m from R[m]
        if (t <= 0.05 || t > Lu + 1.5 || s >= Lv - 0.05) continue;
        Path cand(R.begin(), R.begin() + b + 1);
        cand.push_back(R[b] + t * u);
        cand.insert(cand.end(), R.begin() + m + 1, R.end());
        if (!win.follow_free(cand, 0, rf).has_value()) return cand;
      }
    }
  }
  return std::nullopt;
}

namespace {
// take cells out of the search after a failed pivot or leg (never the start or goal cell)
void block(Window& win, const Block& blk, Cell sc, Cell gc) {
  const Config& cfg = *win.cfg;
  std::set<Cell> cells;
  if (blk.kind == Block::SEGMENT) {
    // the segment's cells within half a frame length of its first blocked pose (all of a long
    // segment's cells would close a corridor two cells wide)
    for (auto& p : densify(blk.seg, cfg.res * 0.5))
      if (!blk.at || std::hypot(p.x() - blk.at->x(), p.y() - blk.at->y()) <= cfg.half_len) cells.insert(win.cell(p));
  } else {
    V2 centre = blk.p;
    // no room to pivot where the robot stands: leave it ahead (within pivot_angle of its heading)
    double R = blk.kind == Block::VERTEX ? cfg.pivot_radius : cfg.off + cfg.lookahead + cfg.res;
    Cell c0 = win.cell(centre);
    int m = int(std::ceil(R / cfg.res));
    for (int di = -m; di <= m; ++di)
      for (int dj = -m; dj <= m; ++dj) {
        Cell c{c0.first + di, c0.second + dj};
        V2 p = win.xy(c);
        if (std::hypot(p.x() - centre.x(), p.y() - centre.y()) > R) continue;
        if (blk.kind == Block::START) {
          V2 fc = blk.p - cfg.off * unit(blk.yaw);
          if (std::hypot(p.x() - fc.x(), p.y() - fc.y()) > 1e-9 && std::abs(wrap(hd(p - fc) - blk.yaw)) <= cfg.pivot_angle)
            continue;
        }
        cells.insert(c);
      }
  }
  for (auto& c : cells)
    if (c != sc && c != gc && win.inside(c)) win.free[size_t(c.first) * win.nx + c.second] = 0;
}

bool starts_with_reverse_reason(const std::string& s) {
  for (auto r : REVERSE_REASONS)
    if (s.rfind(r, 0) == 0) return true;
  return false;
}
}  // namespace

PlanResult plan(std::shared_ptr<HeightGrid> hg, const V2& s, const V2& g, std::optional<double> start_yaw,
                bool start_pivoting) {
  const Config& cfg = *hg->cfg;
  PlanResult out = plan_grid(hg, s, g, start_yaw, start_pivoting);
  // a dead end only: no room for the start pivot where the robot stands, the goal behind (not any
  // failed plan with the goal off the nose: a flaky one backed the SITL rover up in the open, d8)
  if (out.ok || !start_yaw || start_pivoting || cfg.reverse_max <= 0 ||
      std::abs(wrap(hd(g - s) - *start_yaw)) <= cfg.pivot_angle || !starts_with_reverse_reason(out.reason))
    return out;
  PlanResult rev = plan_reverse(hg, s, g, *start_yaw);
  if (rev.ok) return rev;
  out.reverse_reason = rev.reason;
  return out;
}

PlanResult plan(const Cloud& cloud, std::optional<double> floor_z, const V2& s, const V2& g, const Config& cfg,
                std::optional<double> start_yaw, bool start_pivoting) {
  double m = cfg.r_out + cfg.res * 2 + cfg.pad;
  V2 lo = s.cwiseMin(g).array() - m, hi = s.cwiseMax(g).array() + m;
  Cloud pts;
  for (auto& p : cloud)
    if (p.x >= lo.x() && p.y >= lo.y() && p.x <= hi.x() && p.y <= hi.y()) pts.push_back(p);
  auto hg = std::make_shared<HeightGrid>(pts, std::make_shared<const Config>(cfg), lo, hi, floor_z);
  return plan(hg, s, g, start_yaw, start_pivoting);
}

PlanResult plan_grid(std::shared_ptr<HeightGrid> hg, const V2& s, const V2& gxy, std::optional<double> start_yaw,
                     bool start_pivoting) {
  const Config& cfg = *hg->cfg;
  PlanResult res;
  auto win = hg->crop(s.cwiseMin(gxy).array() - cfg.pad, s.cwiseMax(gxy).array() + cfg.pad);
  res.grid = win;
  win->start = s;
  win->start_yaw = start_yaw;
  win->skip = start_yaw ? cfg.start_skip_heading : cfg.start_skip;
  auto sc = win->nearest_free(win->cell(s), cfg.snap_radius);
  auto gc = win->nearest_free(win->cell(gxy), cfg.snap_radius);
  if (!sc) {
    res.reason = "start inside the footprint inflation";
    return res;
  }
  if (!gc) {
    res.reason = "goal blocked or unknown";
    return res;
  }
  int n_follow = 0, n_pivot = 0;
  std::string why = "no path";
  PivotResult pv;
  Path route;
  while (true) {
    res.attempts++;
    double t0 = now_s();
    auto cells = search(*win, *sc, *gc);
    res.t_search += now_s() - t0;
    if (!cells) {
      res.reason = why;
      return res;
    }
    Cell gcell = win->cell(gxy);
    V2 g_end = (win->inside(gcell) && win->is_free(gcell)) ? gxy : win->xy(*gc);
    Path pts{s};
    for (size_t q = 1; q + 1 < cells->size(); ++q) pts.push_back(win->xy((*cells)[q]));
    pts.push_back(g_end);
    t0 = now_s();
    std::optional<ClearRef> ref;
    if (cfg.prefer_clearance > cfg.margin) ref = clear_ref(*win, pts);
    auto pr = prune(*win, pts, ref ? &*ref : nullptr);
    route = pr.first;
    auto bad = pr.second;
    std::optional<V2> fail_at = bad ? win->fail_at : std::nullopt;
    if (bad) {
      auto sharp = sharpen(*win, route, *bad, 4, ref ? &*ref : nullptr);
      if (sharp) {
        route = *sharp;
        bad.reset();
      }
    }
    if (bad && ref) {
      // soft: the vertices the preferred clearance kept made a route the frame cannot follow (a corner
      // in a tight spot): the old straightening instead, before blocking cells for another search
      auto p2 = prune(*win, pts);
      std::optional<int> b2 = p2.second;
      Path r2 = p2.first;
      std::optional<V2> f2 = b2 ? win->fail_at : std::nullopt;
      if (b2) {
        auto sharp = sharpen(*win, r2, *b2);
        if (sharp) {
          r2 = *sharp;
          b2.reset();
        }
      }
      route = r2;   // both blocked: the old route's blocked segment is blocked, as before
      bad = b2;
      fail_at = f2;
    }
    res.t_prune += now_s() - t0;
    if (bad) {
      // the frame cannot follow this cell path (a corner in a tight spot): block the cells of the
      // blocked segment and search again
      why = "no path the oriented frame can follow";
      if (n_follow >= cfg.retries) {
        res.reason = why;
        return res;
      }
      n_follow++;
      Block b;
      b.kind = Block::SEGMENT;
      b.seg = Path(route.begin() + *bad, route.begin() + std::min(int(route.size()), *bad + 2));
      b.at = fail_at;
      block(*win, b, *sc, *gc);
      continue;
    }
    t0 = now_s();
    pv = pivot_pass(*win, route, start_yaw, start_pivoting);
    if (!pv.ok && ref) {
      // soft: the preferred route's pivots or arcs refused (it bends away from the straight line, so the
      // robot's heading is farther off it): the old straightened route, if the frame can follow it
      auto p2 = prune(*win, pts);
      Path r2 = p2.first;
      bool ok2 = !p2.second;
      if (!ok2) {
        auto sharp = sharpen(*win, r2, *p2.second);
        if (sharp) {
          r2 = *sharp;
          ok2 = true;
        }
      }
      if (ok2 && r2 != route) {
        PivotResult pv2 = pivot_pass(*win, r2, start_yaw, start_pivoting);
        if (pv2.ok) {
          route = r2;
          pv = pv2;
        }
      }
    }
    res.t_pivot += now_s() - t0;
    if (pv.ok) break;
    if (!pv.retry) {
      res.reason = pv.reason;
      return res;
    }
    // a pivot with no room (or a leg the frame cannot drive straight into it): block there, again
    why = pv.reason;
    if (n_pivot >= cfg.pivot_retries) {
      res.reason = why;
      return res;
    }
    n_pivot++;
    block(*win, pv.block, *sc, *gc);
  }
  res.ok = true;
  res.reason = "";
  res.path = pv.path;
  res.pivots = pv.pivots;
  res.route = route;
  double len = 0;
  for (size_t q = 0; q + 1 < res.path.size(); ++q) len += norm2(res.path[q + 1] - res.path[q]);
  res.length = len;
  Path dense = densify(res.path, cfg.res * 0.5);
  if (dense.size() > 2) {
    double mc = INF;
    for (size_t q = 1; q + 1 < dense.size(); ++q) mc = std::min(mc, win->clearance_at(dense[q]));
    res.min_clearance = mc;
  } else {
    res.min_clearance = 1e3;
  }
  return res;
}

// cells a reverse may sweep: floor seen in the cell itself or seen through more often than hit lately
// (the live evidence memory), grown by one cell; never unknown space
std::vector<uint8_t> observed_free(const HeightGrid& hg) {
  std::vector<uint8_t> o = hg.floor_seen;
  if (!hg.seen_free.empty())
    for (size_t q = 0; q < o.size(); ++q) o[q] = o[q] | hg.seen_free[q];
  if (!any(o)) return o;
  std::vector<Cell> st;
  for (int a = -1; a <= 1; ++a)
    for (int b = -1; b <= 1; ++b) st.push_back({a, b});
  return dilate(o, hg.ny, hg.nx, st);
}

namespace {
// the grid cells under the frame plus margin at odometry-point pose (xy, yaw)
std::set<Cell> frame_cells(const HeightGrid& hg, const V2& xy, double yaw) {
  const Config& cfg = *hg.cfg;
  double step = cfg.res * 0.5;
  V2 c = xy - cfg.off * unit(yaw);
  auto a = arange(-cfg.half_len, cfg.half_len + 1e-9, step);
  auto b = arange(-cfg.half_wid, cfg.half_wid + 1e-9, step);
  V2 u = unit(yaw), n = unit(yaw + PI / 2);
  std::set<Cell> out;
  for (double B : b)
    for (double A : a) {
      double px = c.x() + A * u.x() + B * n.x(), py = c.y() + A * u.y() + B * n.y();
      out.insert({int(std::floor((py - hg.lo.y()) / cfg.res)), int(std::floor((px - hg.lo.x()) / cfg.res))});
    }
  return out;
}

bool disc_observed(const HeightGrid& hg, const std::vector<uint8_t>& obs, const V2& centre, double radius) {
  double r = hg.cfg->res;
  int k = int(std::ceil(radius / r)) + 1;
  Cell c0 = hg.cell(centre);
  for (int di = -k; di <= k; ++di)
    for (int dj = -k; dj <= k; ++dj) {
      int i = c0.first + di, j = c0.second + dj;
      V2 p = hg.xy({i, j});
      double ex = p.x() - centre.x(), ey = p.y() - centre.y();
      if (ex * ex + ey * ey > radius * radius) continue;
      if (!(i >= 0 && i < hg.ny && j >= 0 && j < hg.nx) || !obs[size_t(i) * hg.nx + j]) return false;
    }
  return true;
}
}  // namespace

// a dead end (owner 2026-10-28): the goal is behind the robot (beyond pivot_angle) and the forward
// plan failed (no room to pivot where it stands). Back straight up along the heading, the way it came,
// to the nearest point (every res/2, at most reverse_max) from which the forward plan (a start pivot
// there, then on) succeeds. The robot backs up blind, so every frame pose of the reverse (the oriented
// frame plus margin) must be free of obstacle cells and every cell it sweeps outside the frame's own
// footprint now must be observed free (observed_free); the first pose that fails either ends the
// search (never into unknown space, never past an obstacle). The pivot spot it stops at needs its
// whole circle observed free too. -> plan()'s result for the forward part from the reverse's end,
// plus reverse {from, to (odometry points), dist, yaw}, or ok false with the reason.
PlanResult plan_reverse(std::shared_ptr<HeightGrid> hg, const V2& s, const V2& gxy, double yaw) {
  const Config& cfg = *hg->cfg;
  PlanResult fail;
  V2 back = -unit(yaw);
  V2 e = s + back * cfg.reverse_max;
  auto win = hg->crop(s.cwiseMin(e).array() - cfg.r_out - cfg.res * 2, s.cwiseMax(e).array() + cfg.r_out + cfg.res * 2);
  auto obs = observed_free(*hg);
  auto here = frame_cells(*hg, s, yaw);
  double step = cfg.res * 0.5;
  int tried = 0;
  std::string last = fmt("no room to pivot within %.1f m behind", cfg.reverse_max);
  int kmax = int(std::floor(cfg.reverse_max / step + 1e-9));
  for (int k = 1; k <= kmax; ++k) {
    double r = k * step;
    V2 q = s + back * r;
    if (!win->pose_free(q, yaw)) {
      fail.reason = fmt("reverse blocked %.2f m behind (%s)", r, last.c_str());
      return fail;
    }
    for (auto& c : frame_cells(*hg, q, yaw)) {
      if (here.count(c)) continue;
      if (!(c.first >= 0 && c.first < hg->ny && c.second >= 0 && c.second < hg->nx) ||
          !obs[size_t(c.first) * hg->nx + c.second]) {
        fail.reason = fmt("reverse into unobserved space %.2f m behind (%s)", r, last.c_str());
        return fail;
      }
    }
    V2 C = q - cfg.off * unit(yaw);
    // no start pivot here, or its circle not all observed free: back up further (d16)
    if (!win->pivot_free(C, yaw, cfg.pivot_tol_start, false) || !disc_observed(*hg, obs, C, cfg.pivot_radius)) continue;
    PlanResult out = plan_grid(hg, q, gxy, yaw, false);
    tried++;
    if (out.ok) {
      out.reverse = Reverse{s, q, r, yaw};
      return out;
    }
    last = out.reason;
    if (tried >= cfg.reverse_tries) {
      fail.reason = "no plan after reversing (" + last + ")";
      return fail;
    }
  }
  fail.reason = last;
  return fail;
}

double reverse_left(const Reverse& rev, const V2& pos) { return (pos - rev.to).dot(unit(rev.yaw)); }

PivotResult check_route(Window& win, const Path& route, std::optional<double> start_yaw, bool start_pivoting) {
  // (the kept route: the margin only, not the preferred clearance; the start eased as in plan_grid)
  if (route.size() >= 2 && win.follow_free(route).has_value()) {
    PivotResult r;
    r.ok = false;
    r.reason = "no path the oriented frame can follow";
    return r;
  }
  return pivot_pass(win, route, start_yaw, start_pivoting);
}

// ---------------------------------------------------------------- following
namespace {
struct Proj {
  double d;
  int k;
  double t;
};
Proj project(const Path& P, const V2& p) {
  Proj best{1e18, 0, 0.0};
  for (int k = 0; k + 1 < int(P.size()); ++k) {
    V2 a = P[k], ab = P[k + 1] - P[k];
    double L2 = ab.dot(ab);
    double t = L2 == 0 ? 0.0 : std::clamp((p - a).dot(ab) / L2, 0.0, 1.0);
    double d = norm2(a + ab * t - p);
    if (d < best.d) best = {d, k, t};
  }
  return best;
}
}  // namespace

Follow follow_plan(const Path& P, const std::vector<Pivot>& pivots, const V2& p, std::optional<double> yaw,
                   const Config& cfg, double lead) {
  if (pivots.empty() || !yaw) {
    auto cy = carrot(P, p, cfg.lookahead);
    return {cy.first, cy.second, "drive", std::nullopt};
  }
  V2 F = p - cfg.off * unit(*yaw);
  std::vector<int> starts{0}, ends;
  for (auto& pv : pivots) {
    starts.push_back(pv.i + 1);
    ends.push_back(pv.i);
  }
  ends.push_back(int(P.size()) - 1);
  // the leg of the nearest driving segment (pivot steps excluded): pivots before it are behind
  double best = 1e18;
  int leg = 0;
  for (int L = 0; L < int(starts.size()); ++L)
    for (int s = starts[L]; s < ends[L]; ++s) {
      V2 u = P[s + 1] - P[s];
      double L2 = u.dot(u);
      double t = L2 == 0 ? 0.0 : std::clamp((p - P[s]).dot(u) / L2, 0.0, 1.0);
      double d = norm2(P[s] + u * t - p);
      if (d < best) {
        best = d;
        leg = L;
      }
    }
  int j = 0;
  while (j < int(pivots.size())) {
    const Pivot& pv = pivots[j];
    V2 e = F - pv.centre;
    double along = e.dot(unit(pv.yaw_in));
    bool near = norm2(e) <= cfg.pivot_tol + 0.1;
    bool arrived = pv.start || (along >= -cfg.pivot_tol && near);
    if (j >= leg && !arrived) break;
    if (near && std::abs(wrap(*yaw - pv.yaw_out)) > cfg.pivot_done) {
      V2 c = pv.centre + (cfg.off + cfg.lookahead) * unit(pv.yaw_out);
      double e = wrap(pv.yaw_out - *yaw);
      if (pv.turn <= cfg.pivot_angle + 1e-9 && std::abs(e) <= cfg.pivot_angle) {
        // a pivot under pivot_angle (pivot_pass: the arc onto the path is not clear): Rover pivots
        // only past WP_PIVOT_ANGLE and then until within 5 deg of its target's bearing, so the carrot
        // leads the heading by pivot_angle + 15 deg (from the frame centre, the FC) until the heading
        // is within pivot_done of the leaving one; the carrot on the path then ends the pivot there
        c = pv.centre + (cfg.off + cfg.lookahead) * unit(*yaw + (e > 0 ? 1 : -1) * (cfg.pivot_angle + radians(15.0)));
      }
      return {c, hd(c - p), "pivot", j};
    }
    ++j;
  }
  Path pts(P.begin() + starts[j], P.begin() + ends[j] + 1);
  if (j < int(pivots.size())) {
    const Pivot& pv = pivots[j];
    V2 H = pv.centre + lead * unit(pv.yaw_in);
    if (pts.size() > 1) pts.back() = H;
    else pts.push_back(H);
    auto cy = carrot(pts, p, cfg.lookahead);
    return {cy.first, cy.second, "approach", j};
  }
  if (pts.size() > 1) {
    auto cy = carrot(pts, p, cfg.lookahead);
    return {cy.first, cy.second, "drive", std::nullopt};
  }
  return {pts.back(), hd(pts.back() - p), "drive", std::nullopt};
}

namespace {
// the polyline from the current leg's start to the point the robot stops at next
Path stop_leg(const Path& P, const std::vector<Pivot>& pivots, std::optional<double> yaw, const std::string& phase,
              std::optional<int> j, double lead) {
  if (phase == "approach" && j) {
    const Pivot& pv = pivots[*j];
    int a = *j == 0 ? 0 : pivots[*j - 1].i + 1;
    V2 H = pv.centre + lead * unit(pv.yaw_in);
    Path pts(P.begin() + a, P.begin() + pv.i + 1);
    if (pts.size() > 1) pts.back() = H;
    else pts.push_back(H);
    return pts;
  }
  int a = (!pivots.empty() && yaw) ? pivots.back().i + 1 : 0;
  return Path(P.begin() + std::min(a, int(P.size()) - 1), P.end());
}

double along_to_end(const Path& pts, const V2& xy) {
  if (pts.size() < 2) return pts.empty() ? 0.0 : norm2(pts.back() - xy);
  Proj b = project(pts, xy);
  double acc = norm2(pts[b.k + 1] - pts[b.k]) * (1 - b.t);
  for (size_t s = b.k + 1; s + 1 < pts.size(); ++s) acc += norm2(pts[s + 1] - pts[s]);
  return acc;
}

double to_turn(const Path& pts, const V2& xy, double turn) {
  if (pts.size() < 3) return along_to_end(pts, xy);
  Proj b = project(pts, xy);
  std::vector<double> seg(pts.size() - 1);
  for (size_t s = 0; s + 1 < pts.size(); ++s) seg[s] = norm2(pts[s + 1] - pts[s]);
  double acc = seg[b.k] * (1 - b.t);
  for (int v = b.k + 1; v < int(pts.size()) - 1; ++v) {
    if (seg[v - 1] > 1e-9 && seg[v] > 1e-9 &&
        std::abs(wrap(hd(pts[v + 1] - pts[v]) - hd(pts[v] - pts[v - 1]))) > turn)
      return acc;
    acc += seg[v];
  }
  return acc;
}
}  // namespace

// the driving target the planner publishes: the carrot (follow_plan) moved on along the bearing from
// the FC to it, so the FC's target is up to `reach` metres away -> (target xy, run-out m).
//
// Why: Rover Guided's fast-update path (AR_WPNav input shaping, the bridge's position-only targets)
// closes on its target with a speed set by the distance left: ~0.5 m/s per metre measured (SITL d10:
// 0.58 m/s at 1.15 m). A target at the carrot held the SITL rover to 0.15-0.25 m/s with WP_SPEED 1.0
// (d6); 1 m/s needs ~2 m (the node: reach = runout_s 2.1 x its drive_speed). The bearing is the
// carrot's, so Rover steers exactly as before; only the distance it sees grows. The run-out is
//   * none while pivoting (phase pivot: the carrot on the leaving ray is the bearing to pivot by);
//   * at most the path length from the carrot to the next stop point (the pivot's hold point in
//     approach, the goal in drive): it shrinks to nothing as the robot arrives (no overshoot);
//   * truncated before the first spot where the frame (the FC on the ray, heading along it) would
//     touch an obstacle cell of `win` (the last plan's window); none without a window. A dead planner
//     leaves Rover on the last target the bridge sent: a checked point;
//   * scaled down while the robot turns: in full with its heading within aligned_deg of the bearing to
//     the carrot, none beyond turning_deg (Rover's fast-update path steers like pure pursuit on the
//     distance to its target: a 2 m target turned it a third as hard as the carrot, d11);
//   * never past the next path vertex that turns by more than corner_deg (the chord past it leaves
//     the path), so the robot reaches a corner on the carrot alone.
// WP_SPEED still caps the speed. fc_behind: the FC behind the odometry point (owner 2026-10-29: the FC
// sits at the frame centre, footprint_offset 0.193 m).
std::pair<V2, double> run_out(const Path& path, const std::vector<Pivot>& pivots, const V2& p, std::optional<double> yaw,
                              const V2& c, const std::string& phase, std::optional<int> j, const Config& cfg, double reach,
                              double fc_behind, double lead, const Window* win, double aligned_deg, double turning_deg,
                              double corner_deg) {
  if (phase == "pivot" || win == nullptr || reach <= 0) return {c, 0.0};
  V2 fc = yaw ? V2(p - fc_behind * unit(*yaw)) : p;
  V2 d = c - fc;
  double n = norm2(d);
  if (n < 1e-6) return {c, 0.0};
  Path leg = stop_leg(path, pivots, yaw, phase, j, lead);
  double e = std::min({reach - n, along_to_end(leg, c), to_turn(leg, c, radians(corner_deg))});
  if (yaw) {
    double err = std::abs(degrees(wrap(hd(d) - *yaw)));
    e *= std::clamp((turning_deg - err) / std::max(turning_deg - aligned_deg, 1e-6), 0.0, 1.0);
  }
  if (e <= cfg.res * 0.5) return {c, 0.0};
  V2 u = d / n;
  auto t = arange(cfg.res * 0.5, e + 1e-9, cfg.res * 0.5);
  if (t.empty() || t.back() < e - 1e-9) t.push_back(e);
  // the FC at c + t u, heading u: the odometry point fc_behind ahead of it
  std::vector<V2> poses;
  for (double v : t) poses.push_back(c + (v + fc_behind) * u);
  auto ok = win->poses_free(poses, std::vector<double>(t.size(), hd(u)));
  for (size_t q = 0; q < ok.size(); ++q)
    if (!ok[q]) {
      e = q > 0 ? t[q - 1] : 0.0;
      break;
    }
  return {c + e * u, e};
}

// The drive slow-down near obstacles (owner request 2026-10-30). Gate L2 s8, leg 5: the robot passed the
// crate at 0.03 m (s6 0.11, s7 0.14) at 1.0-1.1 m/s. The paths of those runs pass the crate's corner at
// 0.04-0.10 m (exact geometry: the frame plus margin 0.05 is checked against obstacle cell centres; within
// start_skip of each 10 Hz replan only the bare-frame arc check applies); in s8 the replans switched
// route four times in 0.4 s beside it (a pivot of ~62 deg, at the pivot_angle threshold, moved and was
// dropped), and the last route turned 18-35 deg off the robot's heading. SITL's skid-steer turns at
// ~5-10 deg/s whatever its speed (s8: ~10 deg/s at 0.25 m/s and at 1 m/s), so its heading lags the path
// for a distance proportional to its speed: at 1 m/s the lagging frame swept 0.03-0.07 m nearer the crate
// than the plan. Slower near obstacles, the heading catches up in a shorter distance and each replan
// moves the robot less. Rover's fast-update path sets the speed from the distance to its target (~0.5 m/s
// per metre), so the node shortens the run-out: the FC-to-target reach goes from the full run-out at a
// clearance of clear_full down to reach_min at clear_slow, the clearance being the bare frame's to the
// obstacle cells where the robot stands and along the next `ahead` metres of the path (the robot slows
// before the gap, not in it). WP_SPEED still caps the speed; the arc check's model (arc_steer 3 m, at
// least the target's distance) does not change: a slower robot turns tighter than it assumes.
double frame_clearance(const Window& win, const V2& p, double yaw, double cap) {
  const Config& c = *win.cfg;
  const double r = c.res, hl = FOOTPRINT_LEN / 2, hw = FOOTPRINT_WID / 2;
  const double cs = std::cos(yaw), sn = std::sin(yaw);
  const double fx = p.x() - c.off * cs, fy = p.y() - c.off * sn;
  const int ci = int(std::floor((fy - win.lo.y()) / r)), cj = int(std::floor((fx - win.lo.x()) / r));
  const int k = int(std::ceil((std::hypot(hl, hw) + cap) / r)) + 1;
  double best = cap;
  for (int di = -k; di <= k; ++di)
    for (int dj = -k; dj <= k; ++dj) {
      long ii = long(ci) + win.pad + di, jj = long(cj) + win.pad + dj;
      if (ii < 0 || jj < 0 || ii >= win.bny || jj >= win.bnx) continue;
      if (!win.occ_big[size_t(ii) * win.bnx + jj]) continue;
      double dx = win.lo.x() + (cj + dj + 0.5) * r - fx, dy = win.lo.y() + (ci + di + 0.5) * r - fy;
      double lon = std::max(std::abs(dx * cs + dy * sn) - hl, 0.0), lat = std::max(std::abs(-dx * sn + dy * cs) - hw, 0.0);
      best = std::min(best, std::hypot(lon, lat));
    }
  return best;
}

double drive_clearance(const Window& win, const Path& path, const V2& pos, std::optional<double> yaw, double ahead,
                       double cap) {
  double best = cap;
  if (yaw) best = std::min(best, frame_clearance(win, pos, *yaw, cap));
  if (path.size() < 2 || ahead <= 0) return best;
  const Path rest = remaining(path, pos);
  const double step = win.cfg->res * 0.5;
  double s = 0;
  for (size_t k = 0; k + 1 < rest.size() && s < ahead; ++k) {
    V2 a = rest[k], ab = rest[k + 1] - rest[k];
    double L = norm2(ab);
    if (L < 1e-9) continue;
    double h = hd(ab);
    for (double t = 0; t <= L + 1e-9 && s + t <= ahead + 1e-9; t += step) best = std::min(best, frame_clearance(win, a + ab * (t / L), h, cap));
    s += L;
  }
  return best;
}

// The preferred clearance (owner decision 2026-10-30, option 1). The search prefers the middle of a
// passage (prefer_clear, clear_weight), but line-of-sight pruning then pulled the path straight, as near
// obstacles as the 0.05 margin allows: Gate L2 s8 planned 0.046-0.056 m from the crate past the 0.90 m
// door. frame_margin is the margin check's own metric (rectangle grown by m: max of the longitudinal and
// lateral excess), so a requirement of the margin itself is exactly poses_free's check.
double frame_margin(const Window& win, const V2& p, double yaw, double cap) {
  const Config& c = *win.cfg;
  const double r = c.res, hl = FOOTPRINT_LEN / 2, hw = FOOTPRINT_WID / 2;
  {
    // nothing within reach of the frame grown by cap (as poses_free's early out)
    int ci = int(std::floor((p.y() - win.lo.y()) / r)), cj = int(std::floor((p.x() - win.lo.x()) / r));
    if (ci >= 0 && cj >= 0 && ci < win.ny && cj < win.nx &&
        win.clear[size_t(ci) * win.nx + cj] > std::hypot(hl + c.off + std::max(cap, 0.0), hw + std::max(cap, 0.0)) + r * 0.75)
      return cap;
  }
  const double cs = std::cos(yaw), sn = std::sin(yaw);
  const double fx = p.x() - c.off * cs, fy = p.y() - c.off * sn;
  const int ci = int(std::floor((fy - win.lo.y()) / r)), cj = int(std::floor((fx - win.lo.x()) / r));
  const int k = int(std::ceil((std::hypot(hl, hw) + std::max(cap, 0.0) * M_SQRT2) / r)) + 1;
  double best = cap;
  for (int di = -k; di <= k; ++di)
    for (int dj = -k; dj <= k; ++dj) {
      long ii = long(ci) + win.pad + di, jj = long(cj) + win.pad + dj;
      if (ii < 0 || jj < 0 || ii >= win.bny || jj >= win.bnx) continue;   // off the padded box: free (poses_free blocks poses off it)
      if (!win.occ_big[size_t(ii) * win.bnx + jj]) continue;
      double dx = win.lo.x() + (cj + dj + 0.5) * r - fx, dy = win.lo.y() + (ci + di + 0.5) * r - fy;
      best = std::min(best, std::max(std::abs(dx * cs + dy * sn) - hl, std::abs(-dx * sn + dy * cs) - hw));
    }
  return best;
}

ClearRef clear_ref(const Window& win, const Path& pts) {
  const Config& c = *win.cfg;
  ClearRef out;
  out.pts = pts;
  const int n = int(pts.size());
  std::vector<double> cum(n, 0.0), raw(n, c.prefer_clearance);
  for (int k = 1; k < n; ++k) cum[k] = cum[k - 1] + norm2(pts[k] - pts[k - 1]);
  for (int k = 0; k < n && n >= 2; ++k) {
    // the follower's heading: to the point lookahead on (the last vertex: its incoming segment)
    double a = cum[k] + c.lookahead;
    int s = std::min(int(std::upper_bound(cum.begin(), cum.end(), a) - cum.begin()), n - 1);
    V2 q = pts[s];
    if (s > 0 && cum[s] > a) {
      double L = cum[s] - cum[s - 1];
      q = pts[s - 1] + (pts[s] - pts[s - 1]) * ((a - cum[s - 1]) / std::max(L, 1e-9));
    }
    V2 d = q - pts[k];
    if (norm2(d) < 1e-6) d = pts[std::max(k, 1)] - pts[std::max(k, 1) - 1];
    raw[k] = frame_margin(win, pts[k], hd(d), c.prefer_clearance + c.prefer_slack);
  }
  out.m.resize(n);
  for (int k = 0; k < n; ++k)
    out.m[k] = std::min({raw[k], raw[std::max(k - 1, 0)], raw[std::min(k + 1, n - 1)]}) - c.prefer_slack;
  return out;
}

double ClearRef::need(const V2& p, int r0, int r1) const {
  const int n = int(pts.size());
  if (n == 0) return -INF;
  r0 = std::clamp(r0, 0, n - 1);
  r1 = r1 < 0 ? n - 1 : std::clamp(r1, r0, n - 1);
  double best = INF, v = m[r0];
  for (int k = r0; k <= r1; ++k) {
    double d = (pts[k] - p).squaredNorm();
    if (d < best) {
      best = d;
      v = m[k];
    }
  }
  return v;
}

double slow_reach(double clear, double reach, const SlowDown& s) {
  if (!s.on || reach <= s.reach_min) return reach;
  double f = std::clamp((clear - s.clear_slow) / std::max(s.clear_full - s.clear_slow, 1e-6), 0.0, 1.0);
  return s.reach_min + (reach - s.reach_min) * f;
}

Path densify(const Path& path, double step) {
  Path out{path[0]};
  for (size_t q = 0; q + 1 < path.size(); ++q) {
    const V2 &a = path[q], &b = path[q + 1];
    int n = std::max(1, int(norm2(b - a) / step));
    for (int k = 1; k <= n; ++k) out.push_back(a + (b - a) * double(k) / double(n));
  }
  return out;
}

Path remaining(const Path& P, const V2& p) {
  Path out{p};
  if (P.size() < 2) {
    out.insert(out.end(), P.begin(), P.end());
    return out;
  }
  Proj b = project(P, p);
  out.insert(out.end(), P.begin() + b.k + 1, P.end());
  return out;
}

std::pair<V2, double> carrot(const Path& P, const V2& p, double lookahead) {
  if (P.size() < 2) {
    V2 c = P.empty() ? p : P.back();
    V2 d = c - p;
    return {c, norm2(d) > 1e-6 ? hd(d) : 0.0};
  }
  Proj b = project(P, p);
  int k = b.k;
  double left = lookahead;
  V2 a = P[k] + (P[k + 1] - P[k]) * b.t;
  V2 c = P.back();
  bool found = false;
  while (k < int(P.size()) - 1) {
    V2 seg = P[k + 1] - a;
    double L = norm2(seg);
    if (L >= left) {
      c = L > 0 ? V2(a + seg * (left / L)) : a;
      found = true;
      break;
    }
    left -= L;
    a = P[k + 1];
    ++k;
  }
  if (!found) c = P.back();
  V2 d = c - p;
  double yaw = norm2(d) > 1e-6 ? std::atan2(d.y(), d.x()) : hd(P.back() - P[P.size() - 2]);
  return {c, yaw};
}

// ---------------------------------------------------------------- EvidenceMemory
void EvidenceMemory::ensure(const V2& lo_, const V2& hi) {
  const double r = res;
  V2 l(std::floor(lo_.x() / r) * r, std::floor(lo_.y() / r) * r);
  if (have && l.x() >= lo.x() && l.y() >= lo.y() && hi.x() < lo.x() + nx * r && hi.y() < lo.y() + ny * r) return;
  V2 new_lo = have ? V2(std::min(lo.x(), l.x() - 10.0), std::min(lo.y(), l.y() - 10.0)) : V2(l.x() - 10.0, l.y() - 10.0);
  V2 top = have ? V2(std::max(lo.x() + nx * r, hi.x() + 10.0), std::max(lo.y() + ny * r, hi.y() + 10.0))
                : V2(hi.x() + 10.0, hi.y() + 10.0);
  int nny = int(std::ceil((top.y() - new_lo.y()) / r)), nnx = int(std::ceil((top.x() - new_lo.x()) / r));
  std::vector<float> h(size_t(nny) * nnx, 0.f), m(h.size(), 0.f), w(h.size(), 0.f), u(h.size(), 0.f), lw(h.size(), 0.f);
  std::vector<double> t(h.size(), 0.0);
  if (have) {
    int di = int(std::nearbyint((lo.y() - new_lo.y()) / r)), dj = int(std::nearbyint((lo.x() - new_lo.x()) / r));
    for (int i = 0; i < ny; ++i)
      for (int j = 0; j < nx; ++j) {
        size_t a = size_t(i + di) * nnx + j + dj, b = size_t(i) * nx + j;
        h[a] = h_[b];
        m[a] = m_[b];
        w[a] = w_[b];
        u[a] = u_[b];
        lw[a] = l_[b];
        t[a] = t_[b];
      }
  }
  lo = new_lo;
  ny = nny;
  nx = nnx;
  h_.swap(h);
  m_.swap(m);
  w_.swap(w);
  u_.swap(u);
  l_.swap(lw);
  t_.swap(t);
  have = true;
}

std::pair<int, int> EvidenceMemory::view(const HeightGrid& hg) {
  ensure(hg.lo, V2(hg.lo.x() + hg.nx * res, hg.lo.y() + hg.ny * res));
  return {int(std::nearbyint((hg.lo.y() - lo.y()) / res)), int(std::nearbyint((hg.lo.x() - lo.x()) / res))};
}

void EvidenceMemory::add(const HeightGrid& hg, const Evidence& ev, double now) {
  auto [di, dj] = view(hg);
  auto at = [&](int k) { return size_t(k / hg.nx + di) * nx + (k % hg.nx + dj); };
  std::vector<int> touched;
  std::set_union(ev.hit.begin(), ev.hit.end(), ev.free.begin(), ev.free.end(), std::back_inserter(touched));
  std::vector<int> t2, t3, t4;
  std::set_union(touched.begin(), touched.end(), ev.wall.begin(), ev.wall.end(), std::back_inserter(t2));
  std::set_union(t2.begin(), t2.end(), ev.under.begin(), ev.under.end(), std::back_inserter(t3));
  std::set_union(t3.begin(), t3.end(), ev.low.begin(), ev.low.end(), std::back_inserter(t4));
  for (int k : t4) {
    size_t a = at(k);
    double f = std::exp(-std::max(now - t_[a], 0.0) / tau);
    h_[a] = float(h_[a] * f);
    m_[a] = float(m_[a] * f);
    w_[a] = float(w_[a] * f);
    u_[a] = float(u_[a] * f);
    l_[a] = float(l_[a] * f);
    t_[a] = now;
  }
  for (int k : ev.hit) h_[at(k)] += 1.0f;
  for (int k : ev.free) m_[at(k)] += 1.0f;
  for (int k : ev.wall) w_[at(k)] += 1.0f;
  for (int k : ev.under) u_[at(k)] += 1.0f;
  for (int k : ev.low) l_[at(k)] += 1.0f;
}

void EvidenceMemory::read(const HeightGrid& hg, double now, std::vector<double>& h, std::vector<double>& m,
                          std::vector<double>& w) {
  std::vector<double> u, l;
  read(hg, now, h, m, w, u, l);
}

void EvidenceMemory::read(const HeightGrid& hg, double now, std::vector<double>& h, std::vector<double>& m,
                          std::vector<double>& w, std::vector<double>& u, std::vector<double>& l) {
  auto [di, dj] = view(hg);
  size_t n = size_t(hg.ny) * hg.nx;
  h.resize(n);
  m.resize(n);
  w.resize(n);
  u.resize(n);
  l.resize(n);
  for (int i = 0; i < hg.ny; ++i)
    for (int j = 0; j < hg.nx; ++j) {
      size_t a = size_t(i + di) * nx + j + dj, b = size_t(i) * hg.nx + j;
      double f = std::exp(-std::max(now - t_[a], 0.0) / tau);
      h[b] = double(h_[a]) * f;
      m[b] = double(m_[a]) * f;
      w[b] = double(w_[a]) * f;
      u[b] = double(u_[a]) * f;
      l[b] = double(l_[a]) * f;
    }
}

// ---------------------------------------------------------------- LiveWindow
LiveWindow::LiveWindow(const Config& c, double window_s, double chunk_s, int n_clear, std::optional<double> keep_radius,
                       double mem_tau, double mem_hits, double mem_clear_ratio, int max_new)
    : cfg(std::make_shared<const Config>(c)),
      window_s(window_s),
      chunk_s(chunk_s),
      n_clear(n_clear),
      keep_radius(keep_radius ? *keep_radius : c.ray_range + 2.0),
      mem_hits(mem_hits),
      mem_clear_ratio(mem_clear_ratio),
      max_new(max_new) {
  if (mem_tau > 0) mem = std::make_unique<EvidenceMemory>(c.res, mem_tau);
}

namespace {
CloudF vstack(const std::vector<std::pair<double, CloudF>>& v) {
  CloudF out;
  for (auto& x : v) out.insert(out.end(), x.second.begin(), x.second.end());
  return out;
}
Cloud to_double(const CloudF& p) {
  Cloud out(p.size());
  for (size_t q = 0; q < p.size(); ++q) out[q] = {double(p[q].x), double(p[q].y), double(p[q].z)};
  return out;
}
}  // namespace

void LiveWindow::add_scan(double t, const CloudF& points, const Eigen::Vector3d& s) {
  const double vox = cfg->res * 0.5;
  CloudF kept;
  kept.reserve(points.size());
  for (auto& q : points)
    if (std::hypot(double(q.x) - s.x(), double(q.y) - s.y()) < keep_radius) kept.push_back(q);
  CloudF p = voxel_down(kept, vox);
  seq++;
  if (!pending.empty() && t - pending.back().first > chunk_s) {
    // a gap: close the pending chunk at its own newest time, so it ages out on time
    chunks.push_back({pending.back().first, voxel_down(vstack(pending), vox)});
    pending.clear();
  }
  pending.push_back({t, p});
  recent.push_back({p, s, t, seq});
  size_t keep = size_t(std::max(n_clear, max_new));
  if (recent.size() > keep) recent.erase(recent.begin(), recent.end() - keep);
  if (t - pending.front().first >= chunk_s) {
    CloudF merged = pending.size() > 1 ? voxel_down(vstack(pending), vox) : p;
    chunks.push_back({t, merged});
    pending.clear();
  }
  std::vector<std::pair<double, CloudF>> keepc;
  for (auto& c : chunks)
    if (t - c.first <= window_s) keepc.push_back(std::move(c));
  chunks.swap(keepc);
}

CloudF LiveWindow::points() const {
  CloudF out = vstack(chunks);
  CloudF p = vstack(pending);
  out.insert(out.end(), p.begin(), p.end());
  return out;
}

void LiveWindow::correct(HeightGrid& hg) {
  const Config& c = *cfg;
  if (!mem) {
    if (n_clear) {
      std::vector<std::pair<Cloud, Eigen::Vector3d>> sc;
      for (size_t q = recent.size() > size_t(n_clear) ? recent.size() - n_clear : 0; q < recent.size(); ++q)
        sc.push_back({to_double(recent[q].p), recent[q].s});
      std::vector<int> hits, miss, under, low_hits;
      evidence(hg, sc, hits, miss, true, &under, &low_hits);
      hg.apply_evidence(hits, miss);
      hg.apply_overhangs(under, low_hits);
    }
    return;
  }
  double now = recent.back().t;
  size_t q0 = recent.size() > size_t(max_new) ? recent.size() - max_new : 0;
  CloudF all;
  bool have_all = false;
  for (size_t q = q0; q < recent.size(); ++q) {
    const Scan& sc = recent[q];
    if (sc.seq <= done) continue;
    // each scan on its own grid around the sensor (not clipped to this plan's box), every return judged
    // against its own tile's local ground (the plan grid's model, over the live window's points around
    // the sensor), not one flat level at the robot's floor: from a floor gap next to lower ground the
    // flat level read the container floor 0.36 m higher as walls (SITL k5/k6, 2026-10-29)
    if (!have_all) {
      all = points();
      have_all = true;
    }
    double R = c.ray_range;
    V2 s2(sc.s.x(), sc.s.y());
    V2 elo = s2.array() - R, ehi = s2.array() + R;
    Cloud gp;
    gp.reserve(all.size());
    for (auto& p : all)
      if (p.x >= elo.x() && p.y >= elo.y() && p.x <= ehi.x() && p.y <= ehi.y()) gp.push_back({p.x, p.y, p.z});
    HeightGrid eg(gp, cfg, elo, ehi, std::nullopt, true);
    Evidence ev = scan_evidence(eg, to_double(sc.p), sc.s, true);
    mem->add(eg, ev, sc.t);
  }
  done = recent.back().seq;
  std::vector<double> h, m, w, u, l;
  mem->read(hg, now, h, m, w, u, l);
  const bool oh = c.overhang_clear > c.z_lo;
  hg.seen_free.assign(h.size(), 0);
  for (size_t q = 0; q < h.size(); ++q) {
    bool moved = hg.occ[q] && m[q] >= 1.0 && m[q] > mem_clear_ratio * h[q];
    // an overhang: nothing in (z_lo, overhang_clear] here or next to it in the window's points (overhang_ok),
    // and the space beneath seen through lately more often than hit (mem_clear_ratio): passable, not held as
    // a wall from afar either
    bool beneath = oh && hg.overhang_ok[q] && u[q] >= c.overhang_seen && u[q] > mem_clear_ratio * l[q];
    // seen through more than hit lately (a reverse may use it); under an overhang, seen through beneath it
    hg.seen_free[q] = (m[q] >= 1.0 && m[q] > h[q]) || (beneath && (hg.seen_up[q] || h[q] >= 1.0));
    bool held = w[q] >= mem_hits && h[q] > m[q] && !beneath;
    hg.occ[q] = (hg.occ[q] && !moved && !beneath) || held;
  }
  hg.edt();
}

PlanResult LiveWindow::plan(const V2& s, const V2& g, std::optional<double> start_yaw, bool start_pivoting) {
  const Config& c = *cfg;
  if (recent.empty()) {
    PlanResult r;
    r.reason = "no scans yet";
    return r;
  }
  double m = c.r_out + c.res * 2 + c.pad;
  V2 lo = s.cwiseMin(g).array() - m, hi = s.cwiseMax(g).array() + m;
  Cloud pts;
  for (auto& x : chunks)
    for (auto& q : x.second)
      if (q.x >= lo.x() && q.y >= lo.y() && q.x <= hi.x() && q.y <= hi.y()) pts.push_back({q.x, q.y, q.z});
  for (auto& x : pending)
    for (auto& q : x.second)
      if (q.x >= lo.x() && q.y >= lo.y() && q.x <= hi.x() && q.y <= hi.y()) pts.push_back({q.x, q.y, q.z});
  double t0 = now_s();
  auto hg = std::make_shared<HeightGrid>(pts, cfg, lo, hi);
  double t1 = now_s();
  correct(*hg);
  double t2 = now_s();
  PlanResult out = l4_track::plan(hg, s, g, start_yaw, start_pivoting);
  out.hg = hg;
  out.t_grid = t1 - t0;
  out.t_evidence = t2 - t1;
  return out;
}

}  // namespace l4_track
