"""Landing-spot check (D3), ROS-free: is a planned landing spot safe to land on?

The robot's footprint (0.508 x 0.457 m) plus a margin, taken as a yaw-independent disc of radius
half-diagonal + margin (0.542 m) around the frame centre, must be all of:
  (a) observed: enough of the disc has ground returns (unobserved = not clear),
  (b) flat: a ground plane fits with a small slope and residual,
  (c) free of obstacles: no points higher than the floor layer + obstacle_min_h above that plane, up
      to the height the robot descends from (which also covers overhead obstacles).
Anything uncertain is not clear. Points are in the planner frame (z up), metres.

The disc's centre (owner decision 2026-10-28). The spot is where the odometry point (FAST-LIO's IMU
frame) comes down, footprint_offset = track_core.ODOM_AHEAD = 0.193 m ahead of the frame centre along
the heading. Given the yaw (evaluate(yaw=...); the node takes it from /planner/odom: QuadRover descends
without yawing), the disc is centred on the frame: spot - footprint_offset * (cos yaw, sin yaw), so the
margin is 0.2 m all round the frame. Without a yaw the disc is centred on the spot itself (the node
answers "no heading" instead). The status gives both ("spot", "centre").

The floor. Recorded maps smear the floor to 0.2-0.26 m, but one scan's floor is only 0.03-0.05 m
thick: the smear is scans disagreeing on the floor height (slow z drift between visits) plus things
that moved (people, doors). So the check takes the scans one by one (evaluate(scans=...): the node's
window, or a map's keyframes) and:
  1. builds the track planner's local ground model (track_core.local_ground: per 0.5 m tile the 20th
     percentile of the cells' lowest voxels, a sofa-like tile replaced by its neighbours' median, a
     ceiling-only tile by the nearest good tile) over the merged scans around the spot;
  2. fits each scan's own floor offset from that model (the median, over the 10 cm cells within
     align_radius whose lowest point lies within align_band of the model, of lowest point - model)
     and shifts the scan by it, so every scan's floor lands on the model: the merged floor layer is
     one scan's thickness again (~0.01-0.05 m) and objects from ~0.06-0.10 m up are seen (a scan that
     sees too little floor near the spot keeps only its returns more than unaligned_min_h up);
  3. seeds the plane fit with the cells near the model (a box covering most of the disc is not taken
     for the floor);
  4. counts a 10 cm cell as occupied by the latest observations: of the last evidence_last scans that
     returned anything in the cell (below the descent height), at least evidence_frac have a point
     above the obstacle threshold there, or the last evidence_latest all do (and 2 points in all).
     An object that left is cleared by the scans that saw the floor since; one that arrived is seen
     by the latest ones.
A single merged cloud (evaluate(points)) is one scan: no alignment, the floor layer is the map's own
smear, and a layer thicker than floor_layer_max is not clear (uncertain).

Near range. On the 07/15 20:21:51 flight (hovering 0.5 m over the takeoff spot) floor returns nearer
than ~1.2 m to the lidar read 3-6 cm high (median; p95 10-13 cm), enough to make the floor under the
robot an "obstacle"; beyond 1.2 m they agree to ~1 cm. A scan given with its sensor position drops
returns nearer than near_range (1.2 m). So the spot must have been seen from at least that far: hover
at 1.2 m or more, or approach nose-first so the scans of the approach see it.

For the markers the last evaluation's geometry is kept in self.viz (plain numbers, no ROS): the disc
centre and radius, the floor height and plane, the occupied cells (centre xy, top height above the
floor) and the disc's unobserved cells.

LIMITS: the disc is centred on the frame for the heading at the time of the check: a yaw during the
descent (QuadRover does not yaw) or a footprint_offset that is not the real IMU offset moves the frame
off it (the 0.2 m margin absorbs ~0.2 m); objects lower than about the
floor layer + 5 cm (~0.06-0.13 m scan by scan, the map's smear + 5 cm on one merged cloud) are not
seen; glass, water and dark surfaces that return nothing read as unobserved (not clear), not as floor.
"""
import math

import numpy as np

import track_core as tc   # the local ground model (read-only use)

FOOTPRINT = (0.508, 0.457)   # length, width (m)


def _cell_keys(xy, origin, cell):
    k = np.floor((xy - origin) / cell).astype(np.int64)
    return (k[:, 0] + 100000) * 200001 + (k[:, 1] + 100000)


class LandingCheck:
    def __init__(self, margin=0.2, cell=0.10, observed_min=0.7, slope_max_deg=5.0, residual_max=0.03,
                 obstacle_min_h=0.05, obstacle_min_points=5, descent_top=3.0, ground_band=0.08,
                 floor_layer_max=0.20, obstacle_min_cells=2, align_radius=1.5, align_band=0.3,
                 align_min_cells=30, seed_band=0.15, evidence_last=10, evidence_frac=0.5, evidence_latest=3,
                 near_range=1.2, unaligned_min_h=0.3, footprint_offset=tc.ODOM_AHEAD):
        self.radius = math.hypot(*FOOTPRINT) / 2 + margin
        self.footprint_offset = footprint_offset   # the spot (odometry point) is this far ahead of the frame centre
        self.viz = {}
        self.cell = cell
        self.observed_min = observed_min
        self.slope_max = math.radians(slope_max_deg)
        self.residual_max = residual_max
        self.obstacle_min_h = obstacle_min_h
        self.obstacle_min_points = obstacle_min_points
        self.descent_top = descent_top
        self.ground_band = ground_band
        self.floor_layer_max = floor_layer_max
        self.obstacle_min_cells = obstacle_min_cells
        self.align_radius = max(align_radius, self.radius)   # per-scan floor offset fitted within this
        self.align_band = align_band          # a cell's lowest point this near the model counts as floor
        self.align_min_cells = align_min_cells
        self.seed_band = seed_band            # plane fit seeded with cells this near the model
        self.evidence_last = evidence_last
        self.evidence_frac = evidence_frac
        self.evidence_latest = evidence_latest
        self.near_range = near_range          # returns this near the sensor read high (see the docstring)
        self.unaligned_min_h = unaligned_min_h   # a scan without its own floor fit: only returns this high
        self.gcfg = tc.Config(res=cell)       # the track planner's ground model (tile 0.5 m, 20th pct)

    def ground_model(self, P, spot):
        """the local ground model over the box around the spot -> function xy (N, 2) -> ground z (N,)"""
        m = self.align_radius + self.gcfg.tile
        ground, lo, r = tc.local_ground(P, self.gcfg, spot - m, spot + m)

        def at(xy):
            j = np.clip(np.floor((xy[:, 0] - lo[0]) / r).astype(np.int64), 0, ground.shape[1] - 1)
            i = np.clip(np.floor((xy[:, 1] - lo[1]) / r).astype(np.int64), 0, ground.shape[0] - 1)
            return ground[i, j]
        return at

    def scan_offset(self, S, G, spot):
        """one scan's floor height relative to the ground model, or None when it sees too little floor"""
        S = S[np.hypot(S[:, 0] - spot[0], S[:, 1] - spot[1]) <= self.align_radius]
        if not len(S):
            return None
        h = S[:, 2] - G(S[:, :2])
        kk = _cell_keys(S[:, :2], spot, self.cell)
        order = np.lexsort((h, kk))
        _, first = np.unique(kk[order], return_index=True)
        low = h[order[first]]
        low = low[np.abs(low) < self.align_band]
        return float(np.median(low)) if len(low) >= self.align_min_cells else None

    def centre(self, spot_xy, yaw):
        """the disc centre (the frame centre) for the odometry point at spot_xy heading yaw"""
        sx, sy = float(spot_xy[0]), float(spot_xy[1])
        if yaw is None:
            return np.array([sx, sy])
        return np.array([sx - self.footprint_offset * math.cos(yaw), sy - self.footprint_offset * math.sin(yaw)])

    def evaluate(self, points, spot_xy, descend_from=None, scans=None, yaw=None):
        """points: Nx3 planner frame (one merged cloud), or scans: a list of Nx3 clouds, oldest first
        (points is then ignored), each optionally (cloud, sensor_xyz): returns nearer the sensor than
        near_range are dropped; spot_xy: (x, y) where the odometry point comes down; yaw: the heading
        (rad, planner frame): the disc is centred footprint_offset behind the spot (None: on the spot);
        descend_from: z the robot comes down from (defaults to ground + descent_top).
        -> (clear: bool, status: dict); the geometry for the markers in self.viz"""
        spot = self.centre(spot_xy, yaw)
        sx, sy = spot
        st = {"spot": [round(float(spot_xy[0]), 2), round(float(spot_xy[1]), 2)],
              "centre": [round(sx, 2), round(sy, 2)], "radius_m": round(self.radius, 3)}
        if yaw is not None:
            st["yaw_deg"] = round(math.degrees(yaw), 1)
            st["footprint_offset_m"] = round(self.footprint_offset, 3)
        viz = self.viz = {"centre": (float(sx), float(sy)), "yaw": yaw, "radius": self.radius, "floor_z": None,
                          "plane": None, "occupied": [], "unobserved": [], "cell": self.cell}
        box = self.align_radius + self.gcfg.tile
        raw = [points] if scans is None else list(scans)
        crop = []
        for S in raw:
            sensor = None
            if isinstance(S, tuple):
                S, sensor = S
            S = np.asarray(S, dtype=float).reshape(-1, 3)
            S = S[np.all(np.abs(S[:, :2] - spot) <= box, axis=1) & np.isfinite(S).all(1)]
            if sensor is not None and self.near_range > 0:
                S = S[np.linalg.norm(S - np.asarray(sensor, float)[:3], axis=1) >= self.near_range]
            crop.append(S)
        allp = np.concatenate(crop) if crop else np.zeros((0, 3))
        d_all = np.hypot(allp[:, 0] - sx, allp[:, 1] - sy)
        st["points"] = int((d_all <= self.radius).sum())
        if st["points"] < 20:
            st["reason"] = "not observed: %d points in the patch" % st["points"]
            return False, st

        # 1. the local ground model; 2. each scan's floor onto it
        G = self.ground_model(allp, spot)
        st["model_z"] = round(float(np.median(G(spot[None]))), 3)
        viz["floor_z"] = st["model_z"]
        offs = [self.scan_offset(S, G, spot) if len(crop) > 1 else None for S in crop]
        good = [o for o in offs if o is not None]
        fill = float(np.median(good)) if good else 0.0
        if len(crop) > 1:
            st["scans"] = len(crop)
            st["scans_aligned"] = len(good)
            if good:
                st["scan_offset_p5_p95_m"] = [round(float(np.percentile(good, 5)), 3), round(float(np.percentile(good, 95)), 3)]
        patch, sid = [], []
        for k, (S, o) in enumerate(zip(crop, offs)):
            S = S[np.hypot(S[:, 0] - sx, S[:, 1] - sy) <= self.radius]
            if o is None and len(crop) > 1:
                # too little floor to place this scan's floor: only its tall returns count (a drift
                # of a few cm cannot make them, and cannot hide them)
                S = S[S[:, 2] - G(S[:, :2]) - fill > self.unaligned_min_h]
            if len(S):
                patch.append(S - [0.0, 0.0, fill if o is None else o])
                sid.append(np.full(len(S), k))
        if sum(len(p) for p in patch) < 20:
            st["reason"] = "not observed: no scan places its floor here"
            return False, st
        patch = np.concatenate(patch)
        sid = np.concatenate(sid)

        # 3. ground: a robust plane through the lowest point of each 10 cm cell, seeded with the cells
        # near the ground model, then the ground points within ground_band of it
        ckk = _cell_keys(patch[:, :2], spot, self.cell)
        order = np.lexsort((patch[:, 2], ckk))
        _, first = np.unique(ckk[order], return_index=True)
        lows = patch[order[first]]
        keep = np.abs(lows[:, 2] - G(lows[:, :2])) < self.seed_band
        coef = None
        for _ in range(5):
            if keep.sum() < 10:
                coef = None
                break
            A = np.c_[lows[keep, 0] - sx, lows[keep, 1] - sy, np.ones(keep.sum())]
            coef, *_ = np.linalg.lstsq(A, lows[keep, 2], rcond=None)
            r = lows[:, 2] - np.c_[lows[:, 0] - sx, lows[:, 1] - sy, np.ones(len(lows))] @ coef
            new = np.abs(r) < max(0.03, 2.5 * np.std(r[keep]))
            if (new == keep).all():
                break
            keep = new
        if coef is None:
            st["reason"] = "no ground plane in the patch"
            return False, st
        r_all = patch[:, 2] - np.c_[patch[:, 0] - sx, patch[:, 1] - sy, np.ones(len(patch))] @ coef
        g = patch[np.abs(r_all) < self.ground_band]
        if len(g) < 10:
            st["reason"] = "no ground plane in the patch"
            return False, st
        A = np.c_[g[:, 0] - sx, g[:, 1] - sy, np.ones(len(g))]
        coef, *_ = np.linalg.lstsq(A, g[:, 2], rcond=None)   # refit on the band: the floor's middle, not its bottom
        resid = float(np.sqrt(np.mean((g[:, 2] - A @ coef) ** 2)))
        slope = math.atan(math.hypot(coef[0], coef[1]))
        ground_z = float(coef[2]) + (good[-1] if good else 0.0)   # back to the latest scan's level
        st.update(ground_z=round(ground_z, 3), slope_deg=round(math.degrees(slope), 2), residual_m=round(resid, 4))
        viz["floor_z"] = ground_z
        viz["plane"] = (float(coef[0]), float(coef[1]))   # floor z = floor_z + a (x - cx) + b (y - cy)

        # (a) observed: share of 10 cm cells of the disc that hold ground returns
        n = int(math.ceil(self.radius / self.cell))
        cx = np.arange(-n, n + 1) * self.cell
        gx, gy = np.meshgrid(cx, cx)
        inside = np.hypot(gx, gy) <= self.radius - self.cell / 2
        cells = {(int(round((x - sx) / self.cell)), int(round((y - sy) / self.cell))) for x, y in g[:, :2]}
        want = [(i - n, j - n) for j in range(2 * n + 1) for i in range(2 * n + 1) if inside[j, i]]
        miss = [c for c in want if c not in cells]
        observed = 1.0 - len(miss) / max(1, len(want))
        viz["unobserved"] = [(sx + i * self.cell, sy + j * self.cell) for i, j in miss]
        st["observed"] = round(observed, 3)
        if observed < self.observed_min:
            st["reason"] = "not observed: %.0f%% of the patch has ground returns" % (100 * observed)
            return False, st

        # (b) flat
        if slope > self.slope_max:
            st["reason"] = "not flat: slope %.1f deg" % math.degrees(slope)
            return False, st
        if resid > self.residual_max:
            st["reason"] = "not flat: ground residual %.3f m" % resid
            return False, st

        # (c) obstacles above the plane, up to the descent height (covers overhead obstacles too)
        h = patch[:, 2] - (np.c_[patch[:, 0] - sx, patch[:, 1] - sy, np.ones(len(patch))] @ coef)
        # the floor layer per 10 cm cell (90th percentile of the points below 0.3 m), its median over
        # the cells: a box covers a few cells and cannot raise it, an evenly thick floor does
        kk = ckk
        tops = []
        for c in np.unique(kk):
            hc = h[(kk == c) & (h < 0.3)]
            if len(hc) >= 3:
                tops.append(np.percentile(hc, 90))
        layer = float(np.median(tops)) if tops else 0.0
        st["floor_layer_m"] = round(layer, 3)
        if layer > self.floor_layer_max:
            st["reason"] = "uncertain floor: layer %.2f m thick" % layer
            return False, st
        thr = max(self.obstacle_min_h, layer + self.obstacle_min_h)
        st["obstacle_threshold_m"] = round(thr, 3)
        top = (descend_from - ground_z) if descend_from is not None else self.descent_top
        below = h < top
        hit = (h > thr) & below
        # 4. per cell, the latest evidence_last scans that returned anything there below the top
        occ_cells, tall = [], 0
        for c in np.unique(kk[hit]):
            inc = (kk == c) & below
            seen = np.unique(sid[inc])[-self.evidence_last:]
            hc = inc & hit & (sid >= seen[0])
            hit_s = np.unique(sid[hc])
            latest = len(seen) >= self.evidence_latest and np.isin(seen[-self.evidence_latest:], hit_s).all()
            if (len(hit_s) >= math.ceil(self.evidence_frac * len(seen)) or latest) and hc.sum() >= 2:
                occ_cells.append(c)
                tall += int((hc & (h >= 0.3)).sum())
        occ = hit & np.isin(kk, occ_cells)
        n_cells = len(occ_cells)
        viz["occupied"] = [(sx + ((c // 200001) - 100000 + 0.5) * self.cell, sy + ((c % 200001) - 100000 + 0.5) * self.cell,
                            float(h[occ & (kk == c)].max())) for c in occ_cells]
        st.update(obstacle_points=int(occ.sum()), obstacle_cells=n_cells, tall_points=tall,
                  hit_cells=int(len(np.unique(kk[hit]))),
                  max_obstacle_h=round(float(h[occ].max()), 3) if occ.any() else 0.0,
                  obstacle_xy=[[round(float(v), 2) for v in patch[occ & (kk == c), :2].mean(0)] +
                               [round(float(h[occ & (kk == c)].max()), 2)] for c in occ_cells[:12]])
        if n_cells >= self.obstacle_min_cells or tall >= self.obstacle_min_points:
            st["reason"] = "occupied: %d cells, %d points up to %.2f m above the floor" % (n_cells, occ.sum(), st["max_obstacle_h"])
            return False, st
        st["reason"] = "clear"
        return True, st
