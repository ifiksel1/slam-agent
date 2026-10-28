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
- turning in place is not modelled (the follower's heading changes over the lookahead) and the frame
  is not checked within start_skip of the start, only a clearance;
- margin 0.05 m: the house drive passes walls with ~5 cm to spare, so 0.10 m refuses it.
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
                 start_skip=0.5):
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

    def follow_free(self, pts, k0=0):
        """the carrot follower's poses along polyline pts from vertex k0 on: every res/2, heading
        toward the point lookahead further along -> index of the first blocked segment, or None.
        Poses within start_skip of self.start skip the frame check (the robot is already there, at
        a heading the plan does not know, and turns toward the carrot from it) but keep a clearance
        check."""
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
        sa = np.clip(np.searchsorted(cum, a, side="right") - 1, 0, len(P) - 2)
        fa = (a - cum[sa]) / np.maximum(L[sa], 1e-9)
        q = P[sa] + (P[sa + 1] - P[sa]) * fa[:, None]
        d = q - p
        short = np.hypot(d[:, 0], d[:, 1]) < 1e-6
        d[short] = (P[seg + 1] - P[seg])[short]
        ok = self.poses_free(p, np.arctan2(d[:, 1], d[:, 0]))
        if self.start is not None:
            # near the start the frame is not checked, but the point may not come nearer an
            # obstacle than the start already is (nor nearer than the bare half-width): no
            # shortcut through a wall next to the start
            near = np.hypot(p[:, 0] - self.start[0], p[:, 1] - self.start[1]) < self.cfg.start_skip
            if near.any():
                ci = np.floor((p[near, 1] - self.lo[1]) / self.cfg.res).astype(np.int64)
                cj = np.floor((p[near, 0] - self.lo[0]) / self.cfg.res).astype(np.int64)
                inb = (ci >= 0) & (cj >= 0) & (ci < self.shape[0]) & (cj < self.shape[1])
                cl = np.where(inb, self.clear[np.clip(ci, 0, self.shape[0] - 1), np.clip(cj, 0, self.shape[1] - 1)], 0.0)
                ok[near] = cl >= min(FOOTPRINT[1] / 2, self.clearance_at(self.start)) - 1e-9
        return None if ok.all() else int(seg[np.argmin(ok)])


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


def plan(source, floor_z, start_xy, goal_xy, cfg=None, obs_given=False):
    """-> dict(ok, reason, path (list of xy), length, min_clearance, grid)

    source: a HeightGrid (a prior map, built once) or an (N, 3) cloud (a grid is built over the
    search box, the ground at floor_z, or estimated locally when floor_z is None). obs_given (an
    (N, 2) obstacle list, the first version's) is gone."""
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
    win = hg.crop(np.minimum(s, gxy) - cfg.pad, np.maximum(s, gxy) + cfg.pad)
    win.start = s
    sc = win.nearest_free(win.cell(s), cfg.snap_radius)
    gc = win.nearest_free(win.cell(gxy), cfg.snap_radius)
    if sc is None:
        return {"ok": False, "reason": "start inside the footprint inflation", "grid": win}
    if gc is None:
        return {"ok": False, "reason": "goal blocked or unknown", "grid": win}
    tm = {"search": 0.0, "prune": 0.0}
    for attempt in range(cfg.retries + 1):
        t0 = time.perf_counter()
        cells = search(win, sc, gc)
        tm["search"] += time.perf_counter() - t0
        if cells is None:
            why = "no path" if attempt == 0 else "no path the oriented frame can follow"
            return {"ok": False, "reason": why, "grid": win, "t": tm, "attempts": attempt + 1}
        g_end = gxy if (win.inside(win.cell(gxy)) and win.free[win.cell(gxy)]) else win.xy(gc)
        pts = [s] + [win.xy(c) for c in cells[1:-1]] + [g_end]
        t0 = time.perf_counter()
        path, bad = prune(win, pts)
        tm["prune"] += time.perf_counter() - t0
        if bad is None:
            break
        # the frame cannot follow this cell path (a corner in a tight spot): block the cells of the
        # blocked segment and search again
        for p in densify(path[bad:bad + 2], win.cfg.res):
            c = win.cell(p)
            if c != sc and c != gc and win.inside(c):
                win.free[c] = False
    else:
        return {"ok": False, "reason": "no path the oriented frame can follow", "grid": win, "t": tm, "attempts": attempt + 1}
    dense = densify(path, cfg.res * 0.5)
    return {"ok": True, "reason": "", "path": path, "grid": win, "t": tm, "attempts": attempt + 1,
            "length": float(sum(np.linalg.norm(np.diff(np.array(path), axis=0), axis=1))),
            "min_clearance": float(min(win.clearance_at(p) for p in dense[1:-1])) if len(dense) > 2 else 1e3}


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
        held = (w >= self.mem_hits) & (h > m)
        hg.occ = (hg.occ & ~moved) | held
        hg._edt()

    def plan(self, start_xy, goal_xy):
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
        out = plan(hg, None, s, g)
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
