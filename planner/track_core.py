"""Track planner core (ROS-free): plan driving legs for the tracked QuadRover on a 2.5-D height grid.

Height grid (HeightGrid). The cloud is cut into res cells and deduplicated into res/2 voxels. The
ground is a robust local model, not a fixed height band: each cell's lowest voxel; a low percentile
of those per tile (tile, 0.5 m); a tile well above the median of its neighbours (a sofa seen only
from above) takes that median; a tile far above the cloud's floor level (only a ceiling or the top
of a wall seen there) or with no points takes the nearest good tile's value. A cell is an obstacle
when at least min_pts voxels sit between z_lo and z_hi above that ground, or one does under a column
of min_tall voxels up to z_tall (a pole or rail sparse in the band), or under a column of
2 x min_tall voxels with nothing in the band (a wall whose base is hidden). The obstacle band starts
above the floor smear: 0.30 m on whole maps (their floor is ~0.22-0.26 m thick from floor height
disagreeing between keyframes, z drift; one keyframe's floor is 0.03-0.05 m and the deskew time-unit
fix does not thin it), 0.15 m on the live window (a few seconds of scans hardly drift).

Moved objects. A merged map keeps everything that was ever there: on the house drive the cells
blocking the robot's own driven path were hit by a median of 3 of 524 keyframes, far in time from
when the robot drove there (people, doors). scan_evidence() casts each scan as a 2-D virtual scan
of the obstacle band and counts, per cell, the scans that hit it and the scans that saw through it;
apply_evidence() drops obstacle cells seen through more often than hit. With a prior map this uses
the mapping run's keyframes; on the live window the latest scans (the latest observation wins).

Footprint. The frame is 0.508 m long and 0.457 m wide (landing_core.FOOTPRINT) plus margin each
side, and the planned point is the odometry point (the IMU, footprint_offset 0.193 m ahead of the frame
centre; the bridge applies no lever arm). The search runs on cells whose clearance exceeds
the half-width plus margin, preferring the middle of a passage and room to turn. The path is then
shortened by line of sight with an oriented-rectangle check of the follower's poses: along each
segment the heading is the direction to the carrot lookahead ahead, as the carrot follower drives
(so corners are checked with the robot turning onto the next segment). A doorway narrower than the
frame's diagonal but wider than the frame passes when the robot goes straight through it. A carrot
a fixed distance ahead is the driving target (D4: position + yaw only, so a dead planner stops
within the lookahead).

Pivots (owner-approved 2026-10-28). The robot is a skid-steer: Rover Guided pivots in place when
the bearing to its target leaves the heading by more than WP_PIVOT_ANGLE (60 on the robot) and
pivots on until within 5 deg (AR_PivotTurn; the streamed Guided targets take the fast-update path,
which does not cancel a pivot). Tracks turn about the middle of their contact patches, the frame
centre; SITL's QuadRover turns about the vehicle origin, the FC, 0.015 m ahead of the frame centre
(odometry point 0.193 m ahead, FC 0.178 m behind it). A pivot sweeps the circle of the frame's
half-diagonal, 0.342 m, plus pivot_margin (its own parameter, owner 2026-10-28; the frame's margin
stays for driving): 0.392 m, 0.78 m across, at 0.05. pivot_pass() makes every turn of more
than pivot_angle (chord to the point lookahead further on, and the bearing from the frame centre)
its own step: the carrot holds just past the pivot (follow_plan) so the robot drives straight in
and stops with its frame centre there (within pivot_tol), the circle about that centre and up to
pivot_tol either side along the incoming heading must be clear, the robot pivots, and the path
goes on from where the odometry point is after the pivot. A circle that is not clear moves to the
nearest centre within pivot_search where it is (reached by driving, frame clear), or the cells
there leave the search and it runs again (pivot_retries). With the robot's heading the start is
checked too: a start pivot needs the circle where the robot stands and up to pivot_tol ahead (a
replan may turn a robot that is still rolling), or the robot drives ahead
(within pivot_angle of its heading) to a centre where it fits. Turns under pivot_angle are driven
and checked by the follower's oriented frame as before.

Dead ends (owner 2026-10-28, plan_reverse). With the robot's heading known, the goal behind it
(beyond pivot_angle) and the start pivot refused where it stands, plan() backs the robot straight
up along its heading, at most reverse_max, to the nearest spot the start pivot fits and plans on
from there (the result's 'reverse'). Every frame pose of the reverse must be clear and every cell
it sweeps observed free (observed_free: floor seen in the cell, or seen through more than hit in
the live evidence memory): the robot backs up blind. The node streams it as a velocity against the
heading, which the bridge turns into Rover Guided's velocity + heading target (reverse).

The driving target (run_out). Rover Guided's fast-update path asks ~0.5 m/s per metre to its
target and steers like pure pursuit on it; the node runs the carrot out along its bearing (up to
~2 s of WP_SPEED from the FC, frame-checked) on straights, and not at stop points or while the robot
turns.

LIMITS (recorded maps, 2026-10-28):
- obstacles lower than z_lo above the local ground are invisible: 0.30 m on whole maps, 0.15 m on
  the live window (kerbs, steps down, cables, low boxes, a pet); above z_hi (0.6 m), on whole maps
  only a tall column counts (overhangs are driven under), on the live window 2+ voxels up to 2 m do
  (a table top or shelf the robot could pass under blocks it);
- the lidar sits ~0.08 m above the floor: wall bases behind furniture go unseen. On the live window
  any column seen above the band (2+ voxels) is a wall, and the evidence memory holds walls hit
  from 1.5 m or more in earlier scans (decay 10 s); a wall seen by one voxel and never from afar is
  still missed (house drive tfix: 1 of 263 goals behind a seen wall);
- people standing still are obstacles until rays at band height see through them (1.5 x more often
  than hit, decayed): on the house drive about 13% of the live goals ahead are refused, about 6-8%
  of those whose driven stretch the scan of the moment shows free;
- unknown space is driven through on the live window (optimistic, replanned as it is seen);
- whole maps use the first, band-level evidence model (rays to the farthest return), which
  overstates see-throughs; it works over hundreds of keyframes, not for a few scans;
- doors seen open and closed equally often stay obstacles in a prior map;
- the ground model is one level per 0.5 m tile: no steps or ramps, one storey; glass is untested;
- pivots are checked where the plan puts them; Rover may also pivot elsewhere if its heading lags
  the carrot by more than 60 deg on a turn under pivot_angle (not seen in the drive-chain model);
  the follower's frame is not checked within start_skip (0.5 m) of the start, only a clearance (a
  start pivot's circle is). With replans at 10 Hz that region is always just ahead of the robot: a
  replan may cut a corner there, as a turn under pivot_angle, and the robot's lagging heading arcs
  wider than the follower model (synthetic hairpin with a 0.7 m way out: 6 cm into the wall).
  start_skip_heading 0.15 (used when the heading is known) checks it, but on the house drive's
  live replay it refused 29 more replans (338 instead of 367 of 490) and put the replan p95 over
  100 ms, so it stays 0.5 by default;
- the start pivot is a hard check where the robot stands: on the house drive (tight, the robot
  pivoted near walls) about 10% of live replans are refused for it (see the 2026-10-28 report);
- a pivot that cannot be moved within pivot_search blocks the cells within the pivot radius of its
  vertex for the next search, which also bars driving straight through them;
- margin 0.05 m: the house drive passes walls with ~5 cm to spare, so 0.10 m refuses it.
- the reverse only backs straight up the way the robot faces, over cells seen free in the last
  ~window_s (floor) or the evidence memory (see-throughs, decay mem_tau 10 s): a robot that waited long
  in a dead end may have forgotten the way out and is refused (never into unknown space).
"""
import math
import time
import warnings

import numpy as np
from scipy import ndimage
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

FOOTPRINT = (0.508, 0.457)   # length, width (m), as landing_core
FOOTPRINT_HALF_DIAGONAL = math.hypot(*FOOTPRINT) / 2   # 0.342 m
# The odometry point (FAST-LIO /Odometry = the JT128's built-in IMU, the body frame) ahead of the
# frame centre, along the nose: owner-measured 0.184 m from the lidar mounting face to the body
# centre, plus the IMU origin 8.68 mm above that face (JT128 manual J01 260710, Fig. 5 p.15) =
# 0.193 m. The lidar origin is 47.84 mm above the face (0.232 m); scan_evidence() casts rays from the
# odometry point it is given, ~4 cm behind the lidar origin, which is left as is.
ODOM_AHEAD = 0.193
LIDAR_AHEAD = ODOM_AHEAD     # (old name)
ROBOT_HEIGHT = 0.203


class Config:
    def __init__(self, res=0.1, margin=0.05, footprint_offset=ODOM_AHEAD, z_lo=0.30, z_hi=0.60,
                 min_pts=3, z_tall=2.0, min_tall=4, min_tall_only=8, mem_min_range=1.0, tile=0.5, furniture_step=0.15, max_ground_step=0.5, ground_pct=20.0,
                 prefer_clear=0.8, clear_weight=2.0, pivot_weight=1.0, unknown_free=False,
                 unknown_cost=1.0, known_radius=0.35, pad=3.0, lookahead=0.5, snap_radius=0.5,
                 ray_bins=720, ray_range=8.0, clear_misses=2, clear_ratio=1.0, retries=2,
                 start_skip=0.5, start_skip_heading=0.5, pivot_angle=60.0, pivot_done=10.0, pivot_tol=0.2, pivot_tol_start=0.2, pivot_search=0.6,
                 pivot_tries=12, pivot_retries=2, pivot_margin=0.05, reverse_max=2.0, reverse_tries=4):
        self.res = float(res)                       # grid cell, m
        self.margin = float(margin)                 # clearance kept each side of the frame, m
        self.off = float(footprint_offset)          # odometry point ahead of the frame centre, m
        self.half_len = FOOTPRINT[0] / 2 + self.margin
        self.half_wid = FOOTPRINT[1] / 2 + self.margin
        self.r_in = self.half_wid                   # search: cells nearer an obstacle than this are not used
        self.r_hard = min(self.half_wid, self.half_len - self.off)   # nearer: blocked at any heading
        self.r_out = math.hypot(self.half_len + self.off, self.half_wid)   # farther: free at any heading
        self.inflate = self.r_in                    # (name kept from the disc version)
        self.z_lo = float(z_lo)                     # obstacle band above the local ground, m
        self.z_hi = float(z_hi)
        self.min_pts = int(min_pts)                 # voxels in the band to call a cell an obstacle
        self.z_tall = float(z_tall)                 # or one voxel in the band under a column of
        self.min_tall = int(min_tall)               # min_tall voxels from z_hi to z_tall (a sparse pole or rail)
        self.min_tall_only = int(min_tall_only)     # or this many with nothing in the band (a wall, base unseen)
        self.mem_min_range = float(mem_min_range)   # live memory holds only cells hit from this far or more
        self.tile = float(tile)                     # ground model tile, m
        self.furniture_step = float(furniture_step) # tile this far above its neighbours: not ground
        self.max_ground_step = float(max_ground_step)   # tile this far above the floor level: not ground
        self.ground_pct = float(ground_pct)         # percentile of the cells' lowest voxels per tile
        self.prefer_clear = float(prefer_clear)     # clearance below which a cell costs extra, m
        self.clear_weight = float(clear_weight)
        self.pivot_weight = float(pivot_weight)     # extra cost where the frame could not turn round
        self.unknown_free = bool(unknown_free)      # plan through cells with no points (live window)
        self.unknown_cost = float(unknown_cost)
        self.known_radius = float(known_radius)     # floor seen within this: the cell is known
        self.pad = float(pad)                       # search box margin around start and goal, m
        self.lookahead = float(lookahead)
        self.snap_radius = float(snap_radius)       # a start or goal this close to a free cell snaps to it
        self.ray_bins = int(ray_bins)               # free-space evidence: azimuth bins per scan
        self.ray_range = float(ray_range)           # and its range, m
        self.clear_misses = int(clear_misses)       # a cell seen through this often (and more often
        self.clear_ratio = float(clear_ratio)       # than clear_ratio x its hits) is a moved object
        self.retries = int(retries)                 # searches again after an oriented-check failure
        self.start_skip = float(start_skip)         # follower poses this close to the start: not checked
        # the same with the robot's heading known (start_yaw); smaller checks the frame closer ahead
        # of the robot (replans at 10 Hz never check the first start_skip), see LIMITS
        self.start_skip_heading = float(start_skip_heading)
        # pivots (see the docstring): a turn over pivot_angle is a pivot in place about the frame
        # centre, which needs the circle of the frame's half-diagonal plus margin clear
        self.pivot_angle = math.radians(pivot_angle)   # Rover WP_PIVOT_ANGLE (60 on the robot)
        self.pivot_done = math.radians(pivot_done)     # a pivot under way goes on until within this
        # the pivot circle's own margin (owner 2026-10-28: separate from the driving margin above)
        self.pivot_margin = float(pivot_margin)
        self.pivot_radius = FOOTPRINT_HALF_DIAGONAL + self.pivot_margin   # 0.342 m + pivot_margin
        # dead ends (owner 2026-10-28): no room to pivot where the robot stands and the goal behind it:
        # back straight up at most reverse_max (0: never) over observed free cells, to the nearest spot
        # a pivot fits (plan_reverse); reverse_tries forward plans are tried from there on back
        self.reverse_max = float(reverse_max)
        self.reverse_tries = int(reverse_tries)
        self.pivot_tol = float(pivot_tol)           # the centre may sit this far before or past the planned one
        self.pivot_tol_start = float(pivot_tol_start)   # a start pivot: up to this far ahead (still rolling)
        self.pivot_search = float(pivot_search)     # a blocked pivot: other centres within this, m
        self.pivot_tries = int(pivot_tries)         # of which this many are checked exactly
        self.pivot_retries = int(pivot_retries)     # searches again after a pivot with no room


def live_config(**kw):
    """the live window (last few seconds of scans): little z drift, so a lower band; unknown space
    allowed; any column seen above the band is a wall (2 voxels: the window sees wall bases rarely)"""
    d = dict(z_lo=0.15, min_pts=2, unknown_free=True, clear_misses=1, clear_ratio=0.99, min_tall_only=2,
             mem_min_range=1.5)
    d.update(kw)
    return Config(**d)


def voxel_down(points, vox):
    """one point per vox cube (its centre): (N, 3) -> (M, 3) float32"""
    p = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    p = p[np.isfinite(p).all(1)]
    if not len(p):
        return p
    k = np.floor(p / vox).astype(np.int64) + 2**20
    key = np.unique((k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2])
    out = np.empty((len(key), 3), np.float32)
    out[:, 0] = (key >> 42) & (2**21 - 1)
    out[:, 1] = (key >> 21) & (2**21 - 1)
    out[:, 2] = key & (2**21 - 1)
    return (out - 2**20 + 0.5) * vox


def _nanmedian_3x3(a):
    """median of the finite values in each 3 x 3 neighbourhood (nan where none)"""
    p = np.pad(a, 1, constant_values=np.nan)
    st = np.stack([p[i:i + a.shape[0], j:j + a.shape[1]] for i in range(3) for j in range(3)])
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)   # all-nan neighbourhoods
        return np.nanmedian(st, axis=0)


def local_ground(points, cfg=None, lo=None, hi=None):
    """the local ground model alone (stable API, also used by the landing-spot check):
    points (N, 3) z up -> (ground, lo, res): ground[i, j] is the floor height (m) of the cell whose
    lower-left corner is lo + (j, i) * res (row = y, column = x); lo, hi bound the xy box (default:
    the points' extent, lo is snapped down to the tile grid)"""
    hg = HeightGrid(points, cfg or Config(), lo, hi)
    return hg.ground, hg.lo.copy(), hg.cfg.res


class HeightGrid:
    """ground, obstacles, clearance and known space on a res grid over [lo, hi] (xy);
    cell (i, j) = row y, column x"""

    def __init__(self, points, cfg, lo=None, hi=None, floor_z=None):
        self.cfg = cfg
        r = cfg.res
        p = np.asarray(points, dtype=float).reshape(-1, 3)
        if lo is None:
            lo, hi = (p[:, :2].min(0), p[:, :2].max(0)) if len(p) else (np.zeros(2), np.ones(2))
        lo, hi = np.asarray(lo, float), np.asarray(hi, float)
        m = int(round(cfg.tile / r))                     # cells per tile
        self.lo = np.floor(lo / (r * m)) * (r * m)       # tile-aligned, so tiles do not depend on the box
        ny = int(np.ceil((hi[1] - self.lo[1]) / (r * m))) * m + m
        nx = int(np.ceil((hi[0] - self.lo[0]) / (r * m))) * m + m
        self.shape = (ny, nx)
        n = ny * nx
        if len(p):
            ij = np.floor((p[:, :2] - self.lo) / r).astype(np.int64)
            ok = (ij[:, 0] >= 0) & (ij[:, 1] >= 0) & (ij[:, 0] < nx) & (ij[:, 1] < ny)
            p, ij = p[ok], ij[ok]
        if len(p):
            # distinct voxels (res/2 high), so a map and a pile of scans count alike
            kz = np.floor(p[:, 2] / (r * 0.5)).astype(np.int64)
            key = np.unique((ij[:, 1] * nx + ij[:, 0]) * 2**21 + (kz + 2**20))
            k = key >> 21
            z = ((key & (2**21 - 1)) - 2**20 + 0.5) * r * 0.5
            first = np.r_[0, np.flatnonzero(np.diff(k)) + 1]   # k is sorted: each cell's lowest voxel first
            cellmin = np.full(n, np.nan)
            cellmin[k[first]] = z[first]
        else:
            k, z = np.zeros(0, np.int64), np.zeros(0)
            cellmin = np.full(n, np.nan)
        self.cellmin = cellmin.reshape(ny, nx)
        self.ground = self._ground(m) if floor_z is None else np.full(self.shape, float(floor_z))
        h = z - self.ground.ravel()[k]
        band = (h > cfg.z_lo) & (h < cfg.z_hi)
        self.count = np.bincount(k[band], minlength=n).reshape(ny, nx)
        tall = np.bincount(k[(h >= cfg.z_hi) & (h < cfg.z_tall)], minlength=n).reshape(ny, nx)
        # a column above the band with its base hidden (the lidar sits ~0.08 m above the floor,
        # furniture occludes wall bases) is a wall too
        self.occ = (self.count >= cfg.min_pts) | ((self.count >= 1) & (tall >= cfg.min_tall)) | (tall >= cfg.min_tall_only)
        self.seen_up = (self.count + tall) > 0    # anything seen from z_lo to z_tall in the cell
        floor = np.bincount(k[(h > -0.15) & (h <= cfg.z_lo)], minlength=n).reshape(ny, nx) > 0
        self.floor_seen = floor                   # floor points in the cell itself (reverse: observed)
        seen = floor | self.occ
        rr = int(math.ceil(cfg.known_radius / r))
        yy, xx = np.mgrid[-rr:rr + 1, -rr:rr + 1]
        self.known = ndimage.binary_dilation(seen, structure=(xx * xx + yy * yy) <= rr * rr) if seen.any() else seen
        self._edt()

    def _edt(self):
        self.clear = (ndimage.distance_transform_edt(~self.occ) * self.cfg.res) if self.occ.any() else np.full(self.shape, 1e3)

    def _ground(self, m):
        ny, nx = self.shape
        cfg = self.cfg
        t = self.cellmin.reshape(ny // m, m, nx // m, m).transpose(0, 2, 1, 3).reshape(ny // m, nx // m, m * m)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            g = np.nanpercentile(t, cfg.ground_pct, axis=2)
            if np.isfinite(g).any():
                # only a ceiling or the top of a wall seen in this tile: not ground
                g[g > np.nanpercentile(g, 25) + cfg.max_ground_step] = np.nan
        nb = _nanmedian_3x3(g)
        high = np.isfinite(g) & np.isfinite(nb) & (g > nb + cfg.furniture_step)
        g[high] = nb[high]
        miss = ~np.isfinite(g)
        if miss.all():
            g[:] = 0.0
        elif miss.any():
            idx = ndimage.distance_transform_edt(miss, return_distances=False, return_indices=True)
            g = g[idx[0], idx[1]]
        return np.repeat(np.repeat(g, m, axis=0), m, axis=1)

    def apply_evidence(self, hits, misses):
        """drop obstacle cells that scans saw through more often than they hit -> number dropped"""
        cfg = self.cfg
        moved = self.occ & (misses >= cfg.clear_misses) & (misses > cfg.clear_ratio * hits)
        self.occ = self.occ & ~moved
        self._edt()
        return int(moved.sum())

    # geometry
    def cell(self, xy):
        return (int(math.floor((xy[1] - self.lo[1]) / self.cfg.res)), int(math.floor((xy[0] - self.lo[0]) / self.cfg.res)))

    def xy(self, c):
        return np.array([self.lo[0] + (c[1] + 0.5) * self.cfg.res, self.lo[1] + (c[0] + 0.5) * self.cfg.res])

    def inside(self, c):
        return 0 <= c[0] < self.shape[0] and 0 <= c[1] < self.shape[1]

    def clearance_at(self, xy):
        c = self.cell(xy)
        return float(self.clear[c]) if self.inside(c) else 0.0

    def ground_at(self, xy):
        c = self.cell(xy)
        return float(self.ground[c]) if self.inside(c) else float("nan")

    def crop(self, lo, hi):
        """the search box: the cells over [lo, hi] (clamped to the grid)"""
        r = self.cfg.res
        i0 = max(0, int(math.floor((lo[1] - self.lo[1]) / r)))
        j0 = max(0, int(math.floor((lo[0] - self.lo[0]) / r)))
        i1 = min(self.shape[0], int(math.ceil((hi[1] - self.lo[1]) / r)) + 1)
        j1 = min(self.shape[1], int(math.ceil((hi[0] - self.lo[0]) / r)) + 1)
        return Window(self, i0, i1, j0, j1)


N_HEADINGS = 64


def _kernels(cfg):
    """per heading bin: the cell offsets (di, dj) whose centres can fall inside the frame for a pose
    anywhere in the centre cell at any heading in the bin (the frame grown by the pose's sub-cell
    offset and half a bin); cached on cfg"""
    if getattr(cfg, "_kern", None) is None:
        r = cfg.res
        grow = r * 0.75 + math.hypot(cfg.half_len + cfg.off, cfg.half_wid) * math.pi / N_HEADINGS
        R = int(math.ceil((cfg.r_out + grow) / r))
        di, dj = np.mgrid[-R:R + 1, -R:R + 1]
        x, y = dj.ravel() * r, di.ravel() * r
        ks = []
        for h in range(N_HEADINGS):
            yaw = h * 2 * math.pi / N_HEADINGS
            cs, sn = math.cos(yaw), math.sin(yaw)
            dx, dy = x + cfg.off * cs, y + cfg.off * sn          # from the frame centre
            lon, lat = np.abs(dx * cs + dy * sn), np.abs(-dx * sn + dy * cs)
            m = (lon < cfg.half_len + grow) & (lat < cfg.half_wid + grow)
            ks.append((di.ravel()[m], dj.ravel()[m]))
        cfg._kern = (R, ks)
    return cfg._kern


class Window:
    """the search box, a slice of a HeightGrid (its clearance already sees obstacles outside it)"""

    def __init__(self, hg, i0, i1, j0, j1):
        self.cfg = cfg = hg.cfg
        sl = (slice(i0, i1), slice(j0, j1))
        self.lo = hg.lo + np.array([j0, i0]) * cfg.res
        self.occ, self.clear, self.known = hg.occ[sl], hg.clear[sl], hg.known[sl]
        self.shape = self.occ.shape
        self.free = (self.clear > cfg.r_in) & (self.known | cfg.unknown_free)
        # the obstacles around the box, padded, for the oriented checks
        R, _ = _kernels(cfg)
        self.pad = R + 1
        big = np.zeros((self.shape[0] + 2 * self.pad, self.shape[1] + 2 * self.pad), bool)
        bi0, bj0 = i0 - self.pad, j0 - self.pad
        si0, sj0 = max(0, bi0), max(0, bj0)
        si1, sj1 = min(hg.shape[0], i1 + self.pad), min(hg.shape[1], j1 + self.pad)
        big[si0 - bi0:si1 - bi0, sj0 - bj0:sj1 - bj0] = hg.occ[si0:si1, sj0:sj1]
        self.occ_big = big
        self.start = None
        self.skip = cfg.start_skip

    cell, xy, inside, clearance_at = HeightGrid.cell, HeightGrid.xy, HeightGrid.inside, HeightGrid.clearance_at

    def nearest_free(self, c, radius_m):
        if self.inside(c) and self.free[c]:
            return c
        r = int(math.ceil(radius_m / self.cfg.res))
        i0, j0 = max(0, c[0] - r), max(0, c[1] - r)
        ii, jj = np.nonzero(self.free[i0:max(0, c[0] + r + 1), j0:max(0, c[1] + r + 1)])
        if not len(ii):
            return None
        d = (ii + i0 - c[0]) ** 2 + (jj + j0 - c[1]) ** 2
        k = int(np.argmin(d))
        return None if d[k] > r * r else (int(ii[k] + i0), int(jj[k] + j0))

    def poses_free(self, xy, yaw):
        """(M, 2) poses (odometry point) and (M,) headings -> (M,) bool: the frame plus margin, its
        centre footprint_offset behind the point, holds no obstacle cell centre (the margin absorbs
        the half-cell rounding of obstacle cells). Poses off the padded box count as blocked."""
        cfg = self.cfg
        xy = np.asarray(xy, float).reshape(-1, 2)
        yaw = np.asarray(yaw, float).reshape(-1)
        out = np.ones(len(xy), bool)
        if not len(xy):
            return out
        r = cfg.res
        ci = np.floor((xy[:, 1] - self.lo[1]) / r).astype(np.int64)
        cj = np.floor((xy[:, 0] - self.lo[0]) / r).astype(np.int64)
        inside = (ci >= 0) & (cj >= 0) & (ci < self.shape[0]) & (cj < self.shape[1])
        out &= inside
        cl = np.where(inside, self.clear[np.clip(ci, 0, self.shape[0] - 1), np.clip(cj, 0, self.shape[1] - 1)], 0.0)
        todo = np.flatnonzero(inside & (cl <= cfg.r_out + r * 0.75))
        if not len(todo):
            return out
        _, ks = _kernels(cfg)
        hb = np.round(np.mod(yaw, 2 * math.pi) / (2 * math.pi) * N_HEADINGS).astype(np.int64) % N_HEADINGS
        for h in np.unique(hb[todo]):
            kk = todo[hb[todo] == h]
            di, dj = ks[h]
            ii = ci[kk, None] + self.pad + di[None, :]
            jj = cj[kk, None] + self.pad + dj[None, :]
            hit = self.occ_big[ii, jj]
            rows = np.flatnonzero(hit.any(1))
            if not len(rows):
                continue
            # exact: obstacle cell centres against the frame at the true pose
            kr = kk[rows]
            hm = hit[rows]
            ox = self.lo[0] + (jj[rows] - self.pad + 0.5) * r
            oy = self.lo[1] + (ii[rows] - self.pad + 0.5) * r
            cs, sn = np.cos(yaw[kr])[:, None], np.sin(yaw[kr])[:, None]
            dx = ox - (xy[kr, 0][:, None] - cfg.off * cs)
            dy = oy - (xy[kr, 1][:, None] - cfg.off * sn)
            inside_f = hm & (np.abs(dx * cs + dy * sn) < cfg.half_len) & (np.abs(-dx * sn + dy * cs) < cfg.half_wid)
            out[kr[inside_f.any(1)]] = False
        return out

    def pose_free(self, xy, yaw):
        return bool(self.poses_free(np.asarray(xy, float)[None], [yaw])[0])

    def discs_free(self, xy, radius=None):
        """(M, 2) centres -> (M,) bool: no obstacle cell centre within radius (default the pivot
        circle, the frame's half-diagonal plus margin) of the centre. Off the padded box: blocked."""
        cfg = self.cfg
        R = cfg.pivot_radius if radius is None else float(radius)
        xy = np.asarray(xy, float).reshape(-1, 2)
        if not len(xy):
            return np.ones(0, bool)
        r = cfg.res
        k = int(math.ceil(R / r)) + 1
        di, dj = np.mgrid[-k:k + 1, -k:k + 1]
        di, dj = di.ravel(), dj.ravel()
        ci = np.floor((xy[:, 1] - self.lo[1]) / r).astype(np.int64)
        cj = np.floor((xy[:, 0] - self.lo[0]) / r).astype(np.int64)
        ii = ci[:, None] + self.pad + di[None, :]
        jj = cj[:, None] + self.pad + dj[None, :]
        H, W = self.occ_big.shape
        inb = (ii >= 0) & (jj >= 0) & (ii < H) & (jj < W)
        hit = self.occ_big[np.clip(ii, 0, H - 1), np.clip(jj, 0, W - 1)] & inb
        ox = self.lo[0] + (jj - self.pad + 0.5) * r
        oy = self.lo[1] + (ii - self.pad + 0.5) * r
        d2 = (ox - xy[:, 0:1]) ** 2 + (oy - xy[:, 1:2]) ** 2
        return inb.all(1) & ~(hit & (d2 < R * R)).any(1)

    def pivot_free(self, centre, yaw_in, tol=None, back=True):
        """the pivot circle clear about centre and about every point up to tol before (back) or past
        it along yaw_in (where Rover may stop short of or run past the held carrot)"""
        tol = self.cfg.pivot_tol if tol is None else float(tol)
        t0 = -tol if back else 0.0
        t = np.linspace(t0, tol, int(math.ceil((tol - t0) / (self.cfg.res * 0.5))) + 1) if tol > 0 else np.zeros(1)
        c = np.asarray(centre, float)[:2] + t[:, None] * _unit(yaw_in)
        return bool(self.discs_free(c).all())

    def follow_free(self, pts, k0=0):
        """the carrot follower's poses along polyline pts from vertex k0 on: every res/2, heading
        toward the point lookahead further along -> index of the first blocked segment, or None.
        Poses within start_skip of self.start skip the frame check (the robot is already there, at
        a heading the plan does not know, and turns toward the carrot from it) but keep a clearance
        check. At a vertex pivot_pass makes a pivot (pivot_vertices) the carrot holds, so the frame
        drives straight in; the poses from the vertex to 2 x footprint_offset past it are in the
        pivot (its circle is checked by pivot_pass, which also checks the leg from where the
        odometry point is after the pivot)."""
        P = np.asarray(pts, float).reshape(-1, 2)
        if len(P) < 2:
            return None
        L = np.linalg.norm(np.diff(P, axis=0), axis=1)
        cum = np.r_[0.0, np.cumsum(L)]
        step = self.cfg.res * 0.5
        s = np.arange(cum[k0], cum[-1], step)
        s = np.r_[s, cum[-1]]
        seg = np.clip(np.searchsorted(cum, s, side="right") - 1, 0, len(P) - 2)
        f = (s - cum[seg]) / np.maximum(L[seg], 1e-9)
        p = P[seg] + (P[seg + 1] - P[seg]) * f[:, None]
        a = np.minimum(s + self.cfg.lookahead, cum[-1])
        in_pivot = np.zeros(len(s), bool)
        pv = pivot_vertices(P, self.cfg)
        if pv:
            cp = cum[pv]
            nx = np.searchsorted(cp, s, side="left")          # the first pivot at or ahead
            a = np.where(nx < len(cp), np.minimum(a, cp[np.minimum(nx, len(cp) - 1)]), a)
            pr = np.searchsorted(cp, s, side="right") - 1      # the last pivot at or behind
            in_pivot = (pr >= 0) & (s - cp[np.maximum(pr, 0)] < 2 * self.cfg.off)
        sa = np.clip(np.searchsorted(cum, a, side="right") - 1, 0, len(P) - 2)
        fa = (a - cum[sa]) / np.maximum(L[sa], 1e-9)
        q = P[sa] + (P[sa + 1] - P[sa]) * fa[:, None]
        d = q - p
        short = np.hypot(d[:, 0], d[:, 1]) < 1e-6
        d[short] = (P[seg + 1] - P[seg])[short]
        ok = self.poses_free(p, np.arctan2(d[:, 1], d[:, 0]))
        ok |= in_pivot
        if self.start is not None:
            # near the start the frame is not checked, but the point may not come nearer an
            # obstacle than the start already is (nor nearer than the bare half-width): no
            # shortcut through a wall next to the start
            near = np.hypot(p[:, 0] - self.start[0], p[:, 1] - self.start[1]) < self.skip
            if near.any():
                ci = np.floor((p[near, 1] - self.lo[1]) / self.cfg.res).astype(np.int64)
                cj = np.floor((p[near, 0] - self.lo[0]) / self.cfg.res).astype(np.int64)
                inb = (ci >= 0) & (cj >= 0) & (ci < self.shape[0]) & (cj < self.shape[1])
                cl = np.where(inb, self.clear[np.clip(ci, 0, self.shape[0] - 1), np.clip(cj, 0, self.shape[1] - 1)], 0.0)
                ok[near] = cl >= min(FOOTPRINT[1] / 2, self.clearance_at(self.start)) - 1e-9
        if ok.all():
            return None
        self.fail_at = p[np.argmin(ok)].copy()     # the first blocked pose (odometry point)
        return int(seg[np.argmin(ok)])


def scan_evidence(hg, pts, sensor_xyz, walls=False, low_sensor=True):
    """one scan's evidence on hg's cells: -> (hit, seen_free) flat cell indices, each unique.

    A 2-D virtual scan of the obstacle band, aware of height: the lidar sits ~0.08 m above the floor
    when driving, under the band, so a ray says the band is free only where its own height is inside
    the band. Per azimuth bin around the sensor: the free stretch starts where the steepest ray in
    the bin enters the band and ends 1.5 cells short of the nearest band return, or, with none, where
    the rays leave the band. A ray to the floor proves nothing about the band. walls=True also
    returns the cells hit from mem_min_range or farther (the moving points on the house drive,
    people beside the robot, lie within ~1 m of the lidar).

    low_sensor False: the first model, used for whole maps (evidence() with all the mapping run's
    keyframes): hits in the band only, and each bin seen free out to its nearest band return or, with
    none, its farthest return of any height. It overstates see-throughs for walls and people alike,
    but over hundreds of keyframes walls out-hit them (house drive: 80/80 wall checks); the height
    aware model clears too few people there (71-85 of ~103 driven goals instead of 98-105)."""
    cfg = hg.cfg
    r = cfg.res
    ny, nx = hg.shape
    p = np.asarray(pts, float).reshape(-1, 3)
    s = np.asarray(sensor_xyz, float)
    d = p[:, :2] - s[:2]
    rng = np.hypot(d[:, 0], d[:, 1])
    ij = np.floor((p[:, :2] - hg.lo) / r).astype(np.int64)
    ok = (ij[:, 0] >= 0) & (ij[:, 1] >= 0) & (ij[:, 0] < nx) & (ij[:, 1] < ny) & (rng > 0.3) & (rng < cfg.ray_range)
    p, ij, d, rng = p[ok], ij[ok], d[ok], rng[ok]
    k = ij[:, 1] * nx + ij[:, 0]
    gk = hg.ground.ravel()[k]
    h = p[:, 2] - gk
    band = (h > cfg.z_lo) & (h < cfg.z_hi)
    # a hit is anything from z_lo up to z_tall (a wall seen only above the band still counts); a
    # cell is cleared only where rays show the band itself free more often (an overhang, a person)
    hit = np.unique(k[(h > cfg.z_lo) & (h < cfg.z_tall)] if low_sensor else k[band])
    wall = np.unique(k[(h > cfg.z_lo) & (h < cfg.z_tall) & (rng >= cfg.mem_min_range)]) if walls else None
    sc = hg.cell(s[:2])
    hs = s[2] - (hg.ground[sc] if hg.inside(sc) else np.median(gk) if len(gk) else 0.0)
    nb = cfg.ray_bins
    b = ((np.arctan2(d[:, 1], d[:, 0]) + math.pi) / (2 * math.pi) * nb).astype(np.int64) % nb
    # each ray's stretch inside the band (ranges where its height is in [z_lo, z_hi])
    up = h > hs + 1e-3
    with np.errstate(divide="ignore", invalid="ignore"):
        enter = np.where(hs >= cfg.z_lo, 0.0, rng * (cfg.z_lo - hs) / (h - hs))
        leave = np.where(hs >= cfg.z_hi, 0.0, np.minimum(rng, rng * (cfg.z_hi - hs) / (h - hs)))
    if hs >= cfg.z_lo:          # sensor in the band (hovering low): level rays count too
        up = up | ((h > cfg.z_lo) | (hs < cfg.z_hi))
        leave = np.where(h <= hs, np.minimum(rng, np.where(h < cfg.z_lo, rng * (hs - cfg.z_lo) / np.maximum(hs - h, 1e-9), rng)), leave)
    use = up & (h > cfg.z_lo) & (leave > enter)
    start = np.full(nb, np.inf)
    np.minimum.at(start, b[use], enter[use])
    reach = np.zeros(nb)
    np.maximum.at(reach, b[use], leave[use])
    near = np.full(nb, np.inf)
    np.minimum.at(near, b[band], rng[band])              # nearest band return per bin
    reach = np.where(np.isfinite(near), near, reach) - 1.5 * r
    if not low_sensor:
        start = np.zeros(nb)
        reach = np.zeros(nb)
        np.maximum.at(reach, b, rng)
        reach = np.where(np.isfinite(near), near, reach) - 1.5 * r
    start = np.maximum(start, 0.3)
    steps = np.arange(0.3, cfg.ray_range, r * 0.5)
    ang = (np.arange(nb) + 0.5) / nb * 2 * math.pi - math.pi
    m = (steps[None, :] < reach[:, None]) & (steps[None, :] >= start[:, None])
    R = np.broadcast_to(steps[None, :], m.shape)[m]
    A = np.broadcast_to(ang[:, None], m.shape)[m]
    ci = np.floor((s[0] + R * np.cos(A) - hg.lo[0]) / r).astype(np.int64)
    cj = np.floor((s[1] + R * np.sin(A) - hg.lo[1]) / r).astype(np.int64)
    ok = (ci >= 0) & (cj >= 0) & (ci < nx) & (cj < ny)
    free = np.setdiff1d(np.unique(cj[ok] * nx + ci[ok]), hit, assume_unique=True)
    return (hit, free, wall) if walls else (hit, free)


def evidence(hg, scans, low_sensor=False):
    """hit and seen-free counts per cell over scans [(points, sensor_xyz), ...] (whole maps: the
    first, band-level model by default; see scan_evidence)"""
    n = hg.shape[0] * hg.shape[1]
    hits = np.zeros(n, np.int32)
    miss = np.zeros(n, np.int32)
    for pts, sensor in scans:
        h, f = scan_evidence(hg, pts, sensor, low_sensor=low_sensor)
        hits[h] += 1
        miss[f] += 1
    return hits.reshape(hg.shape), miss.reshape(hg.shape)


def search(win, start, goal):
    """Dijkstra over the free cells, 8-connected without corner cutting; step cost grows near
    obstacles (keep to the middle), where the frame could not turn round, and on unknown cells"""
    cfg = win.cfg
    ny, nx = win.shape
    free = win.free
    cl = win.clear
    pen = 1.0 + cfg.clear_weight * np.clip((cfg.prefer_clear - cl) / cfg.prefer_clear, 0.0, 1.0)
    pen += cfg.pivot_weight * (cl <= cfg.r_out)
    if cfg.unknown_free:
        pen += cfg.unknown_cost * ~win.known
    idx = np.arange(ny * nx).reshape(ny, nx)
    rows, cols, w = [], [], []
    for di, dj in ((0, 1), (1, 0), (1, 1), (1, -1)):
        a = (slice(0, ny - di), slice(max(0, -dj), nx - max(0, dj)))
        b = (slice(di, ny), slice(a[1].start + dj, a[1].stop + dj))
        ok = free[a] & free[b]
        if di and dj:   # no corner cutting
            ok &= free[a[0], b[1]] & free[b[0], a[1]]
        s = math.hypot(di, dj) * 0.5 * (pen[a] + pen[b])
        ia, ib, ws = idx[a][ok], idx[b][ok], s[ok]
        rows += [ia, ib]
        cols += [ib, ia]
        w += [ws, ws]
    g = csr_matrix((np.concatenate(w), (np.concatenate(rows), np.concatenate(cols))), shape=(ny * nx, ny * nx))
    si, gi = start[0] * nx + start[1], goal[0] * nx + goal[1]
    dist, pred = dijkstra(g, indices=si, return_predecessors=True)
    if not np.isfinite(dist[gi]):
        return None
    path = [gi]
    while path[-1] != si:
        path.append(pred[path[-1]])
    return [(int(k // nx), int(k % nx)) for k in path[::-1]]


def prune(win, pts):
    """drop waypoints while the shortcut, followed by the carrot follower from the previous vertex,
    keeps the oriented frame clear -> (path, index of a blocked segment or None). From each vertex:
    the goal if it checks, else gallop out (+1, +2, +4 ...) to the first failure and bisect back."""
    if len(pts) <= 2:
        return pts, win.follow_free(pts)
    out = [pts[0]]
    i = 0
    n = len(pts)
    while i < n - 1:
        ok = lambda j: win.follow_free(out[-2:] + [pts[j]], k0=max(0, len(out[-2:]) - 2)) is None  # noqa: E731
        if ok(n - 1):
            best = n - 1
        else:
            good, step, bad = i + 1, 1, n - 1
            while good + step < bad:
                if ok(good + step):
                    good += step
                    step *= 2
                else:
                    bad = good + step
                    break
            while bad - good > 1:
                mid = (good + bad) // 2
                if ok(mid):
                    good = mid
                else:
                    bad = mid
            best = good
        out.append(pts[best])
        i = best
    return out, win.follow_free(out)


def _unit(a):
    return np.array([math.cos(a), math.sin(a)])


def _hd(v):
    return math.atan2(v[1], v[0])


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def point_along(pts, dist):
    """the point dist metres along polyline pts from its first vertex (its last vertex if shorter)"""
    left = float(dist)
    for a, b in zip(pts[:-1], pts[1:]):
        a, b = np.asarray(a, float), np.asarray(b, float)
        L = float(np.linalg.norm(b - a))
        if L >= left:
            return a + (b - a) * (left / L) if L > 0 else a
        left -= L
    return np.asarray(pts[-1], float)


def _depart(P, k, C, cfg):
    """the vertex the robot heads for after a pivot about C at vertex k: the first one past k farther
    from C than the odometry offset plus a cell (nearer ones lie in the pivot) -> its index"""
    j = k + 1
    while j < len(P) - 1 and np.linalg.norm(P[j] - C) < cfg.off + cfg.res:
        j += 1
    return j


def _is_pivot(P, k, h_in, A, cfg, thr):
    """vertex k (arrived at A with heading h_in) is a pivot: the chord to the point lookahead further
    along leaves h_in by more than thr, and so does the bearing from the frame centre to the vertex
    the robot leaves for (_depart) -> (pivot, frame centre, that vertex's index)"""
    C = A - cfg.off * _unit(h_in)
    j = _depart(P, k, C, cfg)
    q = point_along(P[k:], cfg.lookahead)
    if np.linalg.norm(q - A) <= 1e-6 or abs(_wrap(_hd(q - A) - h_in)) <= thr:
        return False, C, j
    return bool(np.linalg.norm(P[j] - C) >= cfg.off + cfg.res and abs(_wrap(_hd(P[j] - C) - h_in)) > thr), C, j


def pivot_vertices(P, cfg):
    """the vertices of polyline P (not its ends) that pivot_pass makes pivots, from the polyline alone"""
    P = [np.asarray(p, float) for p in P]
    return [k for k in range(1, len(P) - 1) if _is_pivot(P, k, _hd(P[k] - P[k - 1]), P[k], cfg, cfg.pivot_angle)[0]]


def _pivot_fail(reason, block):
    return {"ok": False, "reason": reason, "block": block}


def _move_pivot(win, out, leg0, h_arr, P, k, C, h_in, start):
    """a pivot whose circle is not clear: the nearest centre within pivot_search where it is, reached
    from out[-1] by driving (no other pivot) with the frame clear -> (C, A, h_in, j) or None"""
    cfg = win.cfg
    prev = out[-1]
    c0 = win.cell(C)
    m = int(math.ceil(cfg.pivot_search / cfg.res))
    i0, j0 = max(0, c0[0] - m), max(0, c0[1] - m)
    sub = win.clear[i0:c0[0] + m + 1, j0:c0[1] + m + 1]
    ii, jj = np.nonzero(sub >= cfg.pivot_radius - 0.75 * cfg.res)
    if not len(ii):
        return None
    xy = np.c_[win.lo[0] + (jj + j0 + 0.5) * cfg.res, win.lo[1] + (ii + i0 + 0.5) * cfg.res]
    d = np.hypot(xy[:, 0] - C[0], xy[:, 1] - C[1])
    xy = xy[d <= cfg.pivot_search]
    d = d[d <= cfg.pivot_search]
    # the arrival heading (from out[-1], twice: the odometry point is footprint_offset ahead of
    # the centre) and arrival point of each
    v = xy - prev
    h = np.arctan2(v[:, 1], v[:, 0])
    A = xy + cfg.off * np.c_[np.cos(h), np.sin(h)]
    v = A - prev
    h = np.arctan2(v[:, 1], v[:, 0])
    A = xy + cfg.off * np.c_[np.cos(h), np.sin(h)]
    keep = np.hypot(*(A - prev).T) >= cfg.res
    # reached without another pivot: from the start, the bearing from the frame centre (what Rover
    # turns by); elsewhere the turn at the previous vertex
    if start:
        fc = prev - cfg.off * _unit(h_arr)
        keep &= np.abs(_wrap(np.arctan2(A[:, 1] - fc[1], A[:, 0] - fc[0]) - h_arr)) <= cfg.pivot_angle
    elif h_arr is not None:
        keep &= np.abs(_wrap(h - h_arr)) <= cfg.pivot_angle
    # the clearance (cell-centre transform) along the tolerance either side, before the exact check
    for t in (-cfg.pivot_tol, cfg.pivot_tol):
        e = xy + t * np.c_[np.cos(h), np.sin(h)]
        ci = np.floor((e[:, 1] - win.lo[1]) / cfg.res).astype(np.int64)
        cj = np.floor((e[:, 0] - win.lo[0]) / cfg.res).astype(np.int64)
        inb = (ci >= 0) & (cj >= 0) & (ci < win.shape[0]) & (cj < win.shape[1])
        cl = np.where(inb, win.clear[np.clip(ci, 0, win.shape[0] - 1), np.clip(cj, 0, win.shape[1] - 1)], 0.0)
        keep &= cl >= cfg.pivot_radius - 0.75 * cfg.res
    idx = np.flatnonzero(keep)
    for o in idx[np.argsort(d[idx])][:cfg.pivot_tries]:
        Cn, An, hn = xy[o], A[o], float(h[o])
        jn = _depart(P, k, Cn, cfg)
        if abs(_wrap(_hd(P[jn] - Cn) - hn)) <= cfg.pivot_angle:
            continue      # no pivot there (Rover would drive on, not as planned)
        if not win.pivot_free(Cn, hn, cfg.pivot_tol):
            continue
        if win.follow_free(out[max(leg0, len(out) - 2):] + [An]) is not None:
            continue
        if not _departs(win, P, jn, Cn):
            continue
        return Cn, An, hn, jn
    return None


def _departs(win, P, j, C):
    """the frame can drive off from a pivot about C toward P[j] (the leg's first two segments)"""
    V = C + win.cfg.off * _unit(_hd(P[j] - C))
    return win.follow_free([V] + list(P[j:j + 2])) is None


def pivot_pass(win, path, start_yaw=None, start_pivoting=False):
    """turns over pivot_angle as pivots, each its own step -> dict(ok, path, pivots, reason, block).

    Vertex k turns when the chord to the point lookahead further along leaves the incoming heading
    by more than pivot_angle. The robot drives straight into the vertex (the carrot holds), so it
    stops with its odometry point at A = P[k] and its frame centre at C = A - footprint_offset along
    the incoming heading, pivots about C toward the next vertex past the pivot, and leaves with its
    odometry point at V = C + footprint_offset along the new heading: the path gets V after A, and
    pivots[n] = {i: index of A (V is i + 1), centre: C, yaw_in, yaw_out, turn, start}. A turn under
    pivot_angle seen from C is not a pivot (Rover only pivots past WP_PIVOT_ANGLE). The pivot
    circle about C (and up to pivot_tol before or past it) must be clear; else the nearest centre
    within pivot_search where it is, reached by driving; else the pivot fails with the cells to
    block (block: list of cells) for another search.

    With start_yaw (the robot's heading) the start is a pivot too when the bearing from the frame
    centre to the carrot leaves it by more than pivot_angle (pivot_done when start_pivoting, a pivot
    under way); the start circle is checked where the robot stands and up to pivot_tol ahead."""
    cfg = win.cfg
    P = [np.asarray(p, float)[:2] for p in path]
    n = len(P)
    out = [P[0]]
    pivots = []
    if n < 2:
        return {"ok": True, "path": out, "pivots": pivots, "reason": ""}
    leg0 = 0              # index in out where the current driving leg starts
    h_arr = start_yaw     # heading arriving at out[-1]
    k = 0 if start_yaw is not None else 1
    while k < n - 1:
        start = k == 0
        prev = out[-1]
        if start:
            h_in, A, tol = float(start_yaw), P[0], cfg.pivot_tol_start
            thr = cfg.pivot_done if start_pivoting else cfg.pivot_angle
        else:
            h_in, A, tol, thr = _hd(P[k] - prev), P[k], cfg.pivot_tol, cfg.pivot_angle
        if start:
            # what Rover turns by from a standstill: the bearing from the frame centre to the carrot
            C = A - cfg.off * _unit(h_in)
            j = _depart(P, k, C, cfg)
            q = point_along(P, cfg.lookahead)
            turning = abs(_wrap(_hd(q - C) - h_in)) > thr and np.linalg.norm(P[j] - C) >= cfg.off + cfg.res \
                and abs(_wrap(_hd(P[j] - C) - h_in)) > thr
        else:
            turning, C, j = _is_pivot(P, k, h_in, A, cfg, thr)
        if not turning:
            if not start:
                out.append(A)
                h_arr = h_in
            k += 1
            continue
        # the start: where the robot stands and up to pivot_tol ahead (a replan can turn the robot
        # on the spot while it moves; Rover brakes, then pivots)
        room = win.pivot_free(C, h_in, tol, back=not start)
        if not (room and _departs(win, P, j, C)):
            mv = _move_pivot(win, out, leg0, h_arr, P, k, C, h_in, start)
            if mv is None:
                where = "at the start" if start else "at (%.2f, %.2f), a %.0f deg turn" % (
                    C[0], C[1], math.degrees(abs(_wrap(_hd(P[j] - C) - h_in))))
                why = "no room to pivot " if room is False else "the frame cannot drive off the pivot "
                return _pivot_fail(why + where, ("start", P[0], start_yaw) if start else ("vertex", P[k]))
            C, A, h_in, j = mv
            out.append(A)      # driven to the moved pivot, then a pivot there
            start = False
        elif not start:
            out.append(A)
        h_out = _hd(P[j] - C)
        pivots.append({"i": len(out) - 1, "centre": C, "yaw_in": h_in, "yaw_out": h_out,
                       "turn": abs(_wrap(h_out - h_in)), "start": start})
        out.append(C + cfg.off * _unit(h_out))
        leg0 = len(out) - 1
        h_arr = h_out
        k = j
    out.append(P[-1])
    if pivots:
        # each driving leg on its own: the carrot holds at a pivot, so the frame drives straight in
        starts = [0] + [p["i"] + 1 for p in pivots]
        ends = [p["i"] for p in pivots] + [len(out) - 1]
        for a, b in zip(starts, ends):
            leg = out[a:b + 1]
            if len(leg) < 2:
                continue
            bad = win.follow_free(leg)
            if bad is not None:
                return _pivot_fail("no path the oriented frame can follow", ("segment", leg[bad:bad + 2], win.fail_at))
    return {"ok": True, "path": out, "pivots": pivots, "reason": ""}


def sharpen(win, route, bad, max_skip=4):
    """a route the frame cannot follow at segment bad: the cells' staircase round a corner (45 deg
    steps the robot would arc through) as one sharp corner instead, where the incoming line (the
    segment bad or the one before) meets the line of one of the next max_skip segments; a corner
    over pivot_angle is a pivot (pivot_pass checks its circle) -> the first such route the frame
    can follow, or None"""
    R = [np.asarray(p, float)[:2] for p in route]
    n = len(R)
    for b in (bad, bad - 1):
        if b < 0 or b + 2 >= n:
            continue
        u = R[b + 1] - R[b]
        Lu = float(np.linalg.norm(u))
        if Lu < 1e-9:
            continue
        u = u / Lu
        for m in range(b + 1, min(b + 1 + max_skip, n - 1)):
            v = R[m + 1] - R[m]
            Lv = float(np.linalg.norm(v))
            if Lv < 1e-9:
                continue
            v = v / Lv
            den = u[0] * v[1] - u[1] * v[0]
            if abs(den) < 0.2:          # nearly parallel
                continue
            w = R[m] - R[b]
            t = (w[0] * v[1] - w[1] * v[0]) / den     # along the incoming line from R[b]
            s = (w[0] * u[1] - w[1] * u[0]) / den     # along segment m from R[m]
            if t <= 0.05 or t > Lu + 1.5 or s >= Lv - 0.05:
                continue
            cand = R[:b + 1] + [R[b] + t * u] + R[m + 1:]
            if win.follow_free(cand) is None:
                return cand
    return None


def _block(win, blk, sc, gc):
    """take cells out of the search after a failed pivot or leg (never the start or goal cell)"""
    cfg = win.cfg
    kind = blk[0]
    if kind == "segment":
        # the segment's cells within half a frame length of its first blocked pose (all of a long
        # segment's cells would close a corridor two cells wide)
        at = blk[2] if len(blk) > 2 and blk[2] is not None else None
        cells = {win.cell(p) for p in densify(blk[1], cfg.res * 0.5)
                 if at is None or np.hypot(*(np.asarray(p) - at)) <= cfg.half_len}
    else:
        if kind == "vertex":
            centre, R = blk[1], cfg.pivot_radius
        else:
            # no room to pivot where the robot stands: leave it ahead (within pivot_angle of its heading)
            centre, R = blk[1], cfg.off + cfg.lookahead + cfg.res
        c0 = win.cell(centre)
        m = int(math.ceil(R / cfg.res))
        cells = set()
        for di in range(-m, m + 1):
            for dj in range(-m, m + 1):
                c = (c0[0] + di, c0[1] + dj)
                p = win.xy(c)
                if np.hypot(*(p - centre)) > R:
                    continue
                if kind == "start":
                    fc = np.asarray(blk[1], float) - cfg.off * _unit(blk[2])
                    if np.hypot(*(p - fc)) > 1e-9 and abs(_wrap(_hd(p - fc) - blk[2])) <= cfg.pivot_angle:
                        continue
                cells.add(c)
    for c in cells:
        if c != sc and c != gc and win.inside(c):
            win.free[c] = False


REVERSE_REASONS = ("no room to pivot at the start", "the frame cannot drive off the pivot at the start")


def plan(source, floor_z, start_xy, goal_xy, cfg=None, obs_given=False, start_yaw=None, start_pivoting=False):
    """-> dict(ok, reason, path (list of xy), pivots, route, length, min_clearance, grid)

    source: a HeightGrid (a prior map, built once) or an (N, 3) cloud (a grid is built over the
    search box, the ground at floor_z, or estimated locally when floor_z is None). obs_given (an
    (N, 2) obstacle list, the first version's) is gone. start_yaw: the robot's heading, for the
    start pivot (pivot_pass); start_pivoting: a pivot is under way there. path has each pivot's
    leaving point after its arrival point (pivots); route is the path before that (pivot_pass's
    input, for checking what is left of it later: check_route)."""
    if obs_given:
        raise ValueError("plan() takes a cloud or a HeightGrid; obstacle xy lists are gone")
    s, gxy = np.asarray(start_xy, float)[:2], np.asarray(goal_xy, float)[:2]
    if isinstance(source, HeightGrid):
        hg = source
        cfg = hg.cfg
    else:
        cfg = cfg or Config()
        pts = np.asarray(source, float).reshape(-1, 3)
        m = cfg.r_out + cfg.res * 2 + cfg.pad
        lo, hi = np.minimum(s, gxy) - m, np.maximum(s, gxy) + m
        if len(pts):
            pts = pts[np.all((pts[:, :2] >= lo) & (pts[:, :2] <= hi), axis=1)]
        hg = HeightGrid(pts, cfg, lo, hi, floor_z)
    out = _plan_grid(hg, s, gxy, start_yaw, start_pivoting)
    # a dead end only: no room for the start pivot where the robot stands, the goal behind (not any
    # failed plan with the goal off the nose: a flaky one backed the SITL rover up in the open, d8)
    if out["ok"] or start_yaw is None or start_pivoting or cfg.reverse_max <= 0 \
            or abs(_wrap(_hd(gxy - s) - start_yaw)) <= cfg.pivot_angle \
            or not out["reason"].startswith(REVERSE_REASONS):
        return out
    rev = plan_reverse(hg, s, gxy, start_yaw)
    if rev["ok"]:
        return rev
    out["reverse_reason"] = rev["reason"]
    return out


def _plan_grid(hg, s, gxy, start_yaw=None, start_pivoting=False):
    """plan()'s search on a grid (see plan())"""
    cfg = hg.cfg
    win = hg.crop(np.minimum(s, gxy) - cfg.pad, np.maximum(s, gxy) + cfg.pad)
    win.start = s
    win.skip = cfg.start_skip if start_yaw is None else cfg.start_skip_heading
    sc = win.nearest_free(win.cell(s), cfg.snap_radius)
    gc = win.nearest_free(win.cell(gxy), cfg.snap_radius)
    if sc is None:
        return {"ok": False, "reason": "start inside the footprint inflation", "grid": win}
    if gc is None:
        return {"ok": False, "reason": "goal blocked or unknown", "grid": win}
    tm = {"search": 0.0, "prune": 0.0, "pivot": 0.0}
    n_follow = n_pivot = 0
    why = "no path"
    attempt = 0
    while True:
        attempt += 1
        t0 = time.perf_counter()
        cells = search(win, sc, gc)
        tm["search"] += time.perf_counter() - t0
        if cells is None:
            return {"ok": False, "reason": why, "grid": win, "t": tm, "attempts": attempt}
        g_end = gxy if (win.inside(win.cell(gxy)) and win.free[win.cell(gxy)]) else win.xy(gc)
        pts = [s] + [win.xy(c) for c in cells[1:-1]] + [g_end]
        t0 = time.perf_counter()
        route, bad = prune(win, pts)
        fail_at = getattr(win, "fail_at", None) if bad is not None else None
        if bad is not None:
            sharp = sharpen(win, route, bad)
            if sharp is not None:
                route, bad = sharp, None
        tm["prune"] += time.perf_counter() - t0
        if bad is not None:
            # the frame cannot follow this cell path (a corner in a tight spot): block the cells of
            # the blocked segment and search again
            why = "no path the oriented frame can follow"
            if n_follow >= cfg.retries:
                return {"ok": False, "reason": why, "grid": win, "t": tm, "attempts": attempt}
            n_follow += 1
            _block(win, ("segment", route[bad:bad + 2], fail_at), sc, gc)
            continue
        t0 = time.perf_counter()
        pv = pivot_pass(win, route, start_yaw, start_pivoting)
        tm["pivot"] += time.perf_counter() - t0
        if pv["ok"]:
            break
        # a pivot with no room (or a leg the frame cannot drive straight into it): block there and
        # search again
        why = pv["reason"]
        if n_pivot >= cfg.pivot_retries:
            return {"ok": False, "reason": why, "grid": win, "t": tm, "attempts": attempt}
        n_pivot += 1
        _block(win, pv["block"], sc, gc)
    path = pv["path"]
    pivots = [{"i": p["i"], "centre": [float(p["centre"][0]), float(p["centre"][1])], "yaw_in": float(p["yaw_in"]),
               "yaw_out": float(p["yaw_out"]), "turn": float(p["turn"]), "start": bool(p["start"])} for p in pv["pivots"]]
    dense = densify(path, cfg.res * 0.5)
    return {"ok": True, "reason": "", "path": path, "pivots": pivots, "route": route, "grid": win, "t": tm, "attempts": attempt,
            "length": float(sum(np.linalg.norm(np.diff(np.array(path), axis=0), axis=1))),
            "min_clearance": float(min(win.clearance_at(p) for p in dense[1:-1])) if len(dense) > 2 else 1e3}


def observed_free(hg):
    """cells a reverse may sweep: floor seen in the cell itself (the window's scans) or seen through
    more often than hit lately (the live evidence memory), grown by one cell (a 0.1 m cell between
    sparse floor returns); never unknown space: the JT128 dome sees nothing behind the robot"""
    o = hg.floor_seen | getattr(hg, "seen_free", np.zeros_like(hg.floor_seen))
    return ndimage.binary_dilation(o, structure=np.ones((3, 3), bool)) if o.any() else o


def _frame_cells(hg, xy, yaw, step=None):
    """the grid cells under the frame plus margin at odometry-point pose (xy, yaw) -> set of (i, j)"""
    cfg = hg.cfg
    step = step or cfg.res * 0.5
    c = np.asarray(xy, float)[:2] - cfg.off * _unit(yaw)
    a = np.arange(-cfg.half_len, cfg.half_len + 1e-9, step)
    b = np.arange(-cfg.half_wid, cfg.half_wid + 1e-9, step)
    A, B = np.meshgrid(a, b)
    u, n = _unit(yaw), _unit(yaw + math.pi / 2)
    P = c + A.reshape(-1, 1) * u + B.reshape(-1, 1) * n
    i = np.floor((P[:, 1] - hg.lo[1]) / cfg.res).astype(int)
    j = np.floor((P[:, 0] - hg.lo[0]) / cfg.res).astype(int)
    return set(zip(i.tolist(), j.tolist()))


def _disc_observed(hg, obs, centre, radius):
    """every grid cell whose centre lies within radius of centre is observed free (observed_free)"""
    r = hg.cfg.res
    k = int(math.ceil(radius / r)) + 1
    ci, cj = hg.cell(centre)
    for di in range(-k, k + 1):
        for dj in range(-k, k + 1):
            i, j = ci + di, cj + dj
            x, y = hg.xy((i, j))
            if (x - centre[0]) ** 2 + (y - centre[1]) ** 2 > radius * radius:
                continue
            if not (0 <= i < hg.shape[0] and 0 <= j < hg.shape[1]) or not obs[i, j]:
                return False
    return True


def plan_reverse(hg, s, gxy, yaw):
    """a dead end (owner 2026-10-28): the goal is behind the robot (beyond pivot_angle) and the
    forward plan failed (no room to pivot where it stands). Back straight up along the heading, the
    way it came, to the nearest point (every res/2, at most cfg.reverse_max) from which the forward
    plan (a start pivot there, then on) succeeds. The robot backs up blind, so every frame pose of
    the reverse (the oriented frame plus margin) must be free of obstacle cells and every cell it
    sweeps outside the frame's own footprint now must be observed free (observed_free); the first
    pose that fails either ends the search (never into unknown space, never past an obstacle). The
    pivot spot it stops at needs its whole circle observed free too.
    -> plan()'s dict for the forward part from the reverse's end, plus reverse: {from, to (odometry
    points), dist, yaw}, or ok False with the reason."""
    cfg = hg.cfg
    s = np.asarray(s, float)[:2]
    back = -_unit(yaw)
    win = hg.crop(np.minimum(s, s + back * cfg.reverse_max) - cfg.r_out - cfg.res * 2,
                  np.maximum(s, s + back * cfg.reverse_max) + cfg.r_out + cfg.res * 2)
    obs = observed_free(hg)
    here = _frame_cells(hg, s, yaw)
    step = cfg.res * 0.5
    tried = 0
    last = "no room to pivot within %.1f m behind" % cfg.reverse_max
    for k in range(1, int(math.floor(cfg.reverse_max / step + 1e-9)) + 1):
        r = k * step
        q = s + back * r
        if not win.pose_free(q, yaw):
            return {"ok": False, "reason": "reverse blocked %.2f m behind (%s)" % (r, last)}
        cells = _frame_cells(hg, q, yaw) - here
        if any(not (0 <= i < hg.shape[0] and 0 <= j < hg.shape[1]) or not obs[i, j] for i, j in cells):
            return {"ok": False, "reason": "reverse into unobserved space %.2f m behind (%s)" % (r, last)}
        C = q - cfg.off * _unit(yaw)
        if not win.pivot_free(C, yaw, tol=cfg.pivot_tol_start, back=False) or not _disc_observed(hg, obs, C, cfg.pivot_radius):
            continue        # no start pivot here (pivot_pass's check: the circle and up to
                            # pivot_tol_start ahead), or its circle not all observed free: in a
                            # corridor the walls beside the robot leave the live window (seen from
                            # under mem_min_range, not held) and a pivot there looked free (d16):
                            # back up further
        out = _plan_grid(hg, q, np.asarray(gxy, float)[:2], yaw, False)
        tried += 1
        if out["ok"]:
            out["reverse"] = {"from": [float(s[0]), float(s[1])], "to": [float(q[0]), float(q[1])], "dist": float(r),
                              "yaw": float(yaw)}
            return out
        last = out["reason"]
        if tried >= cfg.reverse_tries:
            return {"ok": False, "reason": "no plan after reversing (%s)" % last}
    return {"ok": False, "reason": last}


def reverse_left(rev, pos_xy):
    """how far the odometry point still has to back up for a reverse (plan_reverse's dict), m"""
    return float((np.asarray(pos_xy, float)[:2] - np.asarray(rev["to"], float)) @ _unit(rev["yaw"]))


def check_route(win, route, start_yaw=None, start_pivoting=False):
    """what is left of a route (remaining()) checked again in a new window: the frame along it and
    its pivots -> pivot_pass's dict (ok, path, pivots)"""
    route = [np.asarray(p, float)[:2] for p in route]
    if len(route) >= 2 and win.follow_free(route) is not None:
        return {"ok": False, "reason": "no path the oriented frame can follow", "path": None, "pivots": []}
    out = pivot_pass(win, route, start_yaw, start_pivoting)
    out["pivots"] = [{"i": p["i"], "centre": [float(p["centre"][0]), float(p["centre"][1])], "yaw_in": float(p["yaw_in"]),
                      "yaw_out": float(p["yaw_out"]), "turn": float(p["turn"]), "start": bool(p["start"])} for p in out.get("pivots", [])]
    return out


def follow_plan(path, pivots, pos_xy, yaw, cfg, lead=0.15):
    """the carrot with pivots as their own step -> (carrot xy, carrot yaw, phase, pivot index or None).

    phase 'drive': the carrot lookahead ahead on the path (carrot()); 'approach': the next pivot is
    ahead on this leg, and the carrot stops lead past its centre along the incoming heading (Rover
    Guided counts a target within WP_RADIUS, 0.3 m on the robot, reached and stops: the frame centre
    stops within lead - 0.3 .. lead of the centre, inside pivot_tol); 'pivot': the frame centre has
    arrived (pivot_tol before the centre or past it) and the heading is more than pivot_done off
    the leaving heading: the carrot is on the leaving ray from the centre, so the bearing Rover
    pivots by is the planned one. A start pivot has arrived already. Without a heading (yaw None)
    pivots are not tracked."""
    P = np.asarray(path, float)
    p = np.asarray(pos_xy, float)[:2]
    if not pivots or yaw is None:
        c, y = carrot(P, p, cfg.lookahead)
        return c, y, "drive", None
    F = p - cfg.off * _unit(yaw)
    starts = [0] + [pv["i"] + 1 for pv in pivots]
    ends = [pv["i"] for pv in pivots] + [len(P) - 1]
    # the leg of the nearest driving segment (pivot steps excluded): pivots before it are behind
    best, leg = 1e18, 0
    for L, (a, b) in enumerate(zip(starts, ends)):
        for s in range(a, b):
            u = P[s + 1] - P[s]
            L2 = float(u @ u)
            t = 0.0 if L2 == 0 else float(np.clip((p - P[s]) @ u / L2, 0, 1))
            d = float(np.linalg.norm(P[s] + u * t - p))
            if d < best:
                best, leg = d, L
    j = 0
    while j < len(pivots):
        pv = pivots[j]
        C = np.asarray(pv["centre"], float)
        e = F - C
        along = float(e @ _unit(pv["yaw_in"]))
        near = float(np.linalg.norm(e)) <= cfg.pivot_tol + 0.1
        arrived = pv["start"] or (along >= -cfg.pivot_tol and near)
        if j >= leg and not arrived:
            break
        if near and abs(_wrap(yaw - pv["yaw_out"])) > cfg.pivot_done:
            c = C + (cfg.off + cfg.lookahead) * _unit(pv["yaw_out"])
            return c, _hd(c - p), "pivot", j
        j += 1
    pts = P[starts[j]:ends[j] + 1]
    if j < len(pivots):
        pv = pivots[j]
        H = np.asarray(pv["centre"], float) + lead * _unit(pv["yaw_in"])
        pts = np.vstack([pts[:-1], H]) if len(pts) > 1 else np.vstack([pts, H])
        c, y = carrot(pts, p, cfg.lookahead)
        return c, y, "approach", j
    c, y = carrot(pts, p, cfg.lookahead) if len(pts) > 1 else (pts[-1], _hd(pts[-1] - p))
    return c, y, "drive", None


def _stop_leg(path, pivots, yaw, phase, j, lead):
    """the polyline from the current leg's start to the point the robot stops at next, as
    follow_plan drives it: the next pivot's hold point (approach) or the goal (drive)"""
    P = np.asarray(path, float)
    if phase == "approach" and j is not None:
        pv = pivots[j]
        a = 0 if j == 0 else pivots[j - 1]["i"] + 1
        H = np.asarray(pv["centre"], float) + lead * _unit(pv["yaw_in"])
        pts = P[a:pv["i"] + 1]
        return np.vstack([pts[:-1], H]) if len(pts) > 1 else np.vstack([pts, H])
    a = pivots[-1]["i"] + 1 if (pivots and yaw is not None) else 0
    return P[min(a, len(P) - 1):]


def _along_to_end(pts, xy):
    """path length from xy's projection on polyline pts to its last vertex"""
    pts = np.asarray(pts, float)
    if len(pts) < 2:
        return float(np.linalg.norm(pts[-1] - xy)) if len(pts) else 0.0
    best, k, t = 1e18, 0, 0.0
    for s in range(len(pts) - 1):
        u = pts[s + 1] - pts[s]
        L2 = float(u @ u)
        ts = 0.0 if L2 == 0 else float(np.clip((xy - pts[s]) @ u / L2, 0, 1))
        d = float(np.linalg.norm(pts[s] + u * ts - xy))
        if d < best:
            best, k, t = d, s, ts
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    return float(seg[k] * (1 - t) + seg[k + 1:].sum())


def _to_turn(pts, xy, turn):
    """path length from xy's projection on polyline pts to its first later vertex where the path
    turns by more than `turn` (rad), or to its end"""
    pts = np.asarray(pts, float)
    if len(pts) < 3:
        return _along_to_end(pts, xy)
    best, k, t = 1e18, 0, 0.0
    for s in range(len(pts) - 1):
        u = pts[s + 1] - pts[s]
        L2 = float(u @ u)
        ts = 0.0 if L2 == 0 else float(np.clip((xy - pts[s]) @ u / L2, 0, 1))
        d = float(np.linalg.norm(pts[s] + u * ts - xy))
        if d < best:
            best, k, t = d, s, ts
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    acc = seg[k] * (1 - t)
    for v in range(k + 1, len(pts) - 1):
        if seg[v - 1] > 1e-9 and seg[v] > 1e-9 and abs(_wrap(_hd(pts[v + 1] - pts[v]) - _hd(pts[v] - pts[v - 1]))) > turn:
            return float(acc)
        acc += seg[v]
    return float(acc)


def run_out(path, pivots, pos_xy, yaw, carrot_xy, phase, j, cfg, reach=1.2, fc_behind=0.178, lead=0.15, win=None,
            aligned_deg=10.0, turning_deg=30.0, corner_deg=20.0):
    """the driving target the planner publishes: the carrot (follow_plan) moved on along the bearing
    from the FC to it, so the FC's target is up to `reach` metres away -> (target xy, run-out m).

    Why: Rover Guided's fast-update path (AR_WPNav input shaping, the bridge's position-only targets)
    closes on its target with a speed set by the distance left: k_v = WP_JERK / accel = 1/s with
    the robot's WP_JERK 0, ~0.5 m/s per metre measured (SITL d10: 0.58 m/s at 1.15 m). A target at
    the carrot (0.5 m ahead of the odometry point, ~0.7 m from the FC) held the SITL rover to
    0.15-0.25 m/s with WP_SPEED 1.0 (d6); 1 m/s needs ~2 m (the node: reach = runout_s 2.1 x its
    drive_speed). The bearing is the carrot's, so Rover steers exactly as before; only the distance
    it sees grows. The run-out is
      * none while pivoting (phase 'pivot': the carrot on the leaving ray is the bearing to pivot by);
      * at most the path length from the carrot to the next stop point (the pivot's hold point in
        'approach', the goal in 'drive'): it shrinks to nothing as the robot arrives, so Rover's
        shaped target slows to a stop there (no overshoot past a stop point);
      * truncated before the first spot where the frame (the FC on the ray, heading along it) would
        touch an obstacle cell of `win` (the last plan's window); none without a window. A dead
        planner leaves Rover on the last target the bridge sent: a checked point.
      * scaled down while the robot turns: in full with its heading within aligned_deg of the
        bearing to the carrot, none beyond turning_deg. Rover's fast-update path steers like pure
        pursuit on the distance to its target (curvature ~ 2 sin(error) / distance): a 2 m target
        turned it a third as hard as the carrot, so on an angled approach it held its heading and met
        the Gate L2 door's jamb (d11: 47 deg off the bearing at 0.07 rad/s);
      * never past the next path vertex that turns by more than corner_deg (the chord past it
        leaves the path), so the robot reaches a corner on the carrot alone;
    WP_SPEED still caps the speed."""
    c = np.asarray(carrot_xy, float)[:2]
    p = np.asarray(pos_xy, float)[:2]
    if phase == "pivot" or win is None or reach <= 0:
        return c, 0.0
    fc = p - fc_behind * _unit(yaw) if yaw is not None else p
    d = c - fc
    n = float(np.linalg.norm(d))
    if n < 1e-6:
        return c, 0.0
    leg = _stop_leg(path, pivots, yaw, phase, j, lead)
    e = min(reach - n, _along_to_end(leg, c), _to_turn(leg, c, math.radians(corner_deg)))
    if yaw is not None:
        err = abs(math.degrees(_wrap(_hd(d) - yaw)))
        e *= float(np.clip((turning_deg - err) / max(turning_deg - aligned_deg, 1e-6), 0.0, 1.0))
    if e <= cfg.res * 0.5:
        return c, 0.0
    u = d / n
    t = np.arange(cfg.res * 0.5, e + 1e-9, cfg.res * 0.5)
    if not len(t) or t[-1] < e - 1e-9:
        t = np.append(t, e)
    # the FC at c + t u, heading u: the odometry point fc_behind ahead of it
    ok = win.poses_free(c + (t[:, None] + fc_behind) * u, np.full(len(t), _hd(u)))
    bad = np.flatnonzero(~ok)
    if len(bad):
        e = float(t[bad[0] - 1]) if bad[0] > 0 else 0.0
    return c + e * u, e


class EvidenceMemory:
    """per-cell hit and seen-through counts on a world-fixed grid that grows as needed, each decayed
    with time constant tau (s): a surface hit by several scans stays an obstacle after it leaves the
    window, and one grazing see-through does not erase it; a person who walked off is cleared once
    the scans that see through the spot outweigh the ones that hit it"""

    def __init__(self, res, tau):
        self.res, self.tau = float(res), float(tau)
        self.lo = None
        self.h = self.m = self.w = self.t = None   # decayed hits, see-throughs, hits from afar; last update

    def _ensure(self, lo, hi):
        r = self.res
        lo = np.floor(np.asarray(lo, float) / r) * r
        hi = np.asarray(hi, float)
        if self.lo is not None and np.all(lo >= self.lo) and np.all(hi < self.lo + np.array(self.h.shape[::-1]) * r):
            return
        new_lo = lo - 10.0 if self.lo is None else np.minimum(self.lo, lo - 10.0)
        top = hi + 10.0 if self.lo is None else np.maximum(self.lo + np.array(self.h.shape[::-1]) * r, hi + 10.0)
        shape = (int(np.ceil((top[1] - new_lo[1]) / r)), int(np.ceil((top[0] - new_lo[0]) / r)))
        h, m, w, t = np.zeros(shape, np.float32), np.zeros(shape, np.float32), np.zeros(shape, np.float32), np.zeros(shape)
        if self.lo is not None:
            di, dj = int(round((self.lo[1] - new_lo[1]) / r)), int(round((self.lo[0] - new_lo[0]) / r))
            sl = (slice(di, di + self.h.shape[0]), slice(dj, dj + self.h.shape[1]))
            h[sl], m[sl], w[sl], t[sl] = self.h, self.m, self.w, self.t
        self.lo, self.h, self.m, self.w, self.t = new_lo, h, m, w, t

    def _view(self, hg):
        self._ensure(hg.lo, hg.lo + np.array(hg.shape[::-1]) * self.res)
        di, dj = int(round((hg.lo[1] - self.lo[1]) / self.res)), int(round((hg.lo[0] - self.lo[0]) / self.res))
        return (slice(di, di + hg.shape[0]), slice(dj, dj + hg.shape[1]))

    def add(self, hg, hit, free, wall, now):
        sl = self._view(hg)
        h, m, w, t = self.h[sl], self.m[sl], self.w[sl], self.t[sl]   # views
        touched = np.union1d(np.union1d(hit, free), wall)
        i, j = np.divmod(touched, hg.shape[1])
        k = np.exp(-np.maximum(now - t[i, j], 0.0) / self.tau)
        h[i, j] *= k
        m[i, j] *= k
        w[i, j] *= k
        t[i, j] = now
        for idx, arr in ((hit, h), (free, m), (wall, w)):
            i, j = np.divmod(idx, hg.shape[1])
            arr[i, j] += 1.0

    def read(self, hg, now):
        sl = self._view(hg)
        k = np.exp(-np.maximum(now - self.t[sl], 0.0) / self.tau)
        return self.h[sl] * k, self.m[sl] * k, self.w[sl] * k


class LiveWindow:
    """the node's map: the last window_s seconds of registered scans. Each scan is deduplicated into
    res/2 voxels as it arrives and merged into a chunk every chunk_s, so a replan stacks a handful of
    chunks, not 60 scans. plan() builds a grid over the search box; every scan not yet used (up to
    max_new) adds its hits (z_lo to z_tall) and height-aware see-throughs to an EvidenceMemory
    (decay mem_tau s). The grid is then corrected by it: an obstacle seen through more than
    mem_clear_ratio x its hits is cleared (a person who left); a cell hit from mem_min_range or
    farther at least mem_hits times (decayed) and hit more than seen through is held as an obstacle
    even when the window lacks it (a wall seen earlier or only sparsely). Hits from closer than
    mem_min_range are not held: on the house drive the moving points (people beside the robot) lie
    within ~1 m of the lidar. mem_tau None: the latest n_clear scans clear what they see through."""

    def __init__(self, cfg=None, window_s=3.0, chunk_s=0.5, n_clear=2, keep_radius=None, mem_tau=10.0,
                 mem_hits=1.5, mem_clear_ratio=1.5, max_new=4):
        self.cfg = cfg or live_config()
        self.window_s = float(window_s)
        self.chunk_s = float(chunk_s)
        self.n_clear = int(n_clear)
        self.keep_radius = float(keep_radius if keep_radius is not None else self.cfg.ray_range + 2.0)
        self.mem = EvidenceMemory(self.cfg.res, mem_tau) if mem_tau else None
        self.mem_hits = float(mem_hits)
        self.mem_clear_ratio = float(mem_clear_ratio)   # cleared when seen through this many times more than hit
        self.max_new = int(max_new)
        self.chunks = []    # (t of the newest scan, points (M, 3) float32)
        self.pending = []   # scans not yet merged: (t, points)
        self.recent = []    # the latest scans: (points, sensor xyz, t, seq)
        self.seq = 0
        self.done = 0       # scans up to this seq are in the memory

    def add_scan(self, t, points, sensor_xyz):
        t = float(t)
        s = np.asarray(sensor_xyz, float)
        p = np.asarray(points, np.float32).reshape(-1, 3)
        p = voxel_down(p[np.hypot(p[:, 0] - s[0], p[:, 1] - s[1]) < self.keep_radius], self.cfg.res * 0.5)
        self.seq += 1
        if self.pending and t - self.pending[-1][0] > self.chunk_s:
            # a gap: close the pending chunk at its own newest time, so it ages out on time
            self.chunks.append((self.pending[-1][0], voxel_down(np.vstack([x[1] for x in self.pending]), self.cfg.res * 0.5)))
            self.pending = []
        self.pending.append((t, p))
        self.recent = (self.recent + [(p, s, t, self.seq)])[-max(self.n_clear, self.max_new):]
        if t - self.pending[0][0] >= self.chunk_s:
            merged = voxel_down(np.vstack([x[1] for x in self.pending]), self.cfg.res * 0.5) if len(self.pending) > 1 else p
            self.chunks.append((t, merged))
            self.pending = []
        self.chunks = [c for c in self.chunks if t - c[0] <= self.window_s]

    def points(self):
        return np.vstack([c[1] for c in self.chunks] + [x[1] for x in self.pending] or [np.zeros((0, 3), np.float32)])

    def _correct(self, hg):
        cfg = self.cfg
        if self.mem is None:
            if self.n_clear:
                hits, miss = evidence(hg, [(x[0], x[1]) for x in self.recent[-self.n_clear:]], low_sensor=True)
                hg.apply_evidence(hits, miss)
            return
        now = self.recent[-1][2]
        for p, s, t, q in self.recent[-self.max_new:]:
            if q > self.done:
                # each scan on its own grid around the sensor (not clipped to this plan's box), the
                # ground flat at the robot's floor (the plan grid's ground under it)
                sc = hg.cell(s[:2])
                g0 = float(hg.ground[sc]) if hg.inside(sc) else float(np.median(hg.ground))
                R = cfg.ray_range
                eg = HeightGrid(np.zeros((0, 3)), cfg, s[:2] - R, s[:2] + R, floor_z=g0)
                hit, free, wall = scan_evidence(eg, p, s, walls=True)
                self.mem.add(eg, hit, free, wall, t)
        self.done = self.recent[-1][3]
        h, m, w = self.mem.read(hg, now)
        moved = hg.occ & (m >= 1.0) & (m > self.mem_clear_ratio * h)
        hg.seen_free = (m >= 1.0) & (m > h)       # seen through more than hit lately (a reverse may use it)
        held = (w >= self.mem_hits) & (h > m)
        hg.occ = (hg.occ & ~moved) | held
        hg._edt()

    def plan(self, start_xy, goal_xy, start_yaw=None, start_pivoting=False):
        cfg = self.cfg
        s, g = np.asarray(start_xy, float)[:2], np.asarray(goal_xy, float)[:2]
        if not self.recent:
            return {"ok": False, "reason": "no scans yet"}
        m = cfg.r_out + cfg.res * 2 + cfg.pad
        lo, hi = np.minimum(s, g) - m, np.maximum(s, g) + m
        pts = self.points()
        pts = pts[np.all((pts[:, :2] >= lo) & (pts[:, :2] <= hi), axis=1)]
        t0 = time.perf_counter()
        hg = HeightGrid(pts, cfg, lo, hi)
        t1 = time.perf_counter()
        self._correct(hg)
        t2 = time.perf_counter()
        out = plan(hg, None, s, g, start_yaw=start_yaw, start_pivoting=start_pivoting)
        out["hg"] = hg
        out.setdefault("t", {}).update(grid=t1 - t0, evidence=t2 - t1)
        return out


def densify(path, step):
    out = [np.asarray(path[0], float)]
    for a, b in zip(path[:-1], path[1:]):
        a, b = np.asarray(a, float), np.asarray(b, float)
        n = max(1, int(np.linalg.norm(b - a) / step))
        out += [a + (b - a) * k / n for k in range(1, n + 1)]
    return out


def remaining(path, pos_xy):
    """pos_xy, then the path's vertices past its projection on the path"""
    P = np.asarray(path, float)
    p = np.asarray(pos_xy, float)[:2]
    if len(P) < 2:
        return [p] + [x for x in P]
    best = (1e18, 0)
    for k in range(len(P) - 1):
        a, ab = P[k], P[k + 1] - P[k]
        L2 = float(ab @ ab)
        t = 0.0 if L2 == 0 else float(np.clip((p - a) @ ab / L2, 0, 1))
        d = float(np.linalg.norm(a + ab * t - p))
        if d < best[0]:
            best = (d, k)
    return [p] + [x for x in P[best[1] + 1:]]


def carrot(path, pos_xy, lookahead):
    """the point lookahead metres along the path past the robot's projection on it, and its yaw"""
    P = np.array(path, float)
    p = np.asarray(pos_xy, float)[:2]
    best = (1e18, 0, 0.0)
    for k in range(len(P) - 1):
        a, b = P[k], P[k + 1]
        ab = b - a
        L2 = float(ab @ ab)
        t = 0.0 if L2 == 0 else float(np.clip((p - a) @ ab / L2, 0, 1))
        d = float(np.linalg.norm(a + ab * t - p))
        if d < best[0]:
            best = (d, k, t)
    _, k, t = best
    left = lookahead
    a = P[k] + (P[k + 1] - P[k]) * t
    while k < len(P) - 1:
        seg = P[k + 1] - a
        L = float(np.linalg.norm(seg))
        if L >= left:
            c = a + seg * (left / L) if L > 0 else a
            break
        left -= L
        a = P[k + 1]
        k += 1
    else:
        c = P[-1]
    d = c - p
    yaw = math.atan2(d[1], d[0]) if np.linalg.norm(d) > 1e-6 else math.atan2(P[-1][1] - P[-2][1], P[-1][0] - P[-2][0]) if len(P) > 1 else 0.0
    return c, yaw
