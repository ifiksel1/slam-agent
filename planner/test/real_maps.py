"""Real-map checks for the track planner (used by test_track_planner.py --map ...).

A map is a cloud with the robot's driven path in the same gravity-up, ground-at-0 frame, and
optionally the mapping run's SC-PGO output (optimized_poses.txt, times.txt, Scans/*.pcd: the
keyframe scans, sensor frame), aligned to the path by a rigid fit of the keyframe positions.

  map mode   the whole map, built once (HeightGrid), with the keyframes' free-space evidence when
             given; goals sampled every step metres along the driven path, dist metres ahead of it
  live mode  the keyframes replayed in order through LiveWindow (the node's map), a replan from
             every live_step-th keyframe to a goal 3 and 6 m ahead along the driven path; timed
  walls      reference walls (points in each 0.3 m layer from 0.3 to 1.5 m above the ground and,
             with keyframes, hit by 3+ of them and never mostly seen through: people are tall
             too); driven-path pairs 1-4 m apart with a wall between them but far apart along the
             path, and goals 1 m behind the nearest wall: each must be refused, or routed with the
             whole path at least 0.2 m from every reference wall cell
"""
import json
import os
import time

import numpy as np
from scipy.spatial import cKDTree

import track_core as tc


def load_view(d):
    m = json.load(open(os.path.join(d, "map_meta.json")))
    raw = open(os.path.join(d, "map_points.bin"), "rb").read()
    return np.frombuffer(raw[:m["points"] * 6], dtype=np.int16).reshape(-1, 3) / 100.0, np.array(m["traj"])


def load_pcd(p):
    raw = open(p, "rb").read()
    k = raw.index(b"DATA binary\n") + len(b"DATA binary\n")
    hdr = raw[:k].decode()
    line = {l.split()[0]: l.split()[1:] for l in hdr.splitlines() if l and not l.startswith("#")}
    n, nf = int(line["POINTS"][0]), len(line["FIELDS"])
    return np.frombuffer(raw[k:k + n * 4 * nf], dtype="<f4").reshape(-1, nf)[:, :3].astype(float)


def load_spec(spec):
    """'view=DIR[,kf=OUT]' | 'pcd=FILE,path=CSV|meta=VIEWDIR[,kf=OUT]' -> (points, path xyz, kf dir)"""
    kv = dict(x.split("=", 1) for x in spec.split(","))
    if "view" in kv:
        pts, path = load_view(kv["view"])
    else:
        pts = load_pcd(kv["pcd"])
        if "path" in kv:
            path = np.loadtxt(kv["path"], delimiter=",", skiprows=1)[:, 1:4]
        else:
            path = load_view(kv["meta"])[1]
    return pts, path, kv.get("kf")


class Keyframes:
    """SC-PGO keyframes in the map frame: a rigid fit of the keyframe positions to the path"""

    def __init__(self, out, path):
        Tk = np.loadtxt(os.path.join(out, "optimized_poses.txt")).reshape(-1, 3, 4)
        t = np.loadtxt(os.path.join(out, "times.txt"))
        n = min(len(Tk), len(t), len(path))
        Tk, self.t, path = Tk[:n], t[:n], np.asarray(path)[:n]
        A = Tk[:, :, 3]
        ca, cb = A.mean(0), path.mean(0)
        U, _, Vt = np.linalg.svd((A - ca).T @ (path - cb))
        d = np.sign(np.linalg.det(Vt.T @ U.T))
        R = Vt.T @ np.diag([1, 1, d]) @ U.T
        off = cb - R @ ca
        self.rms = float(np.sqrt(((A @ R.T + off - path) ** 2).sum(1).mean()))
        self.Rk = np.einsum("ij,njk->nik", R, Tk[:, :, :3])
        self.pk = A @ R.T + off
        self.out = out

    def __len__(self):
        return len(self.t)

    def scan(self, k):
        b = load_pcd(os.path.join(self.out, "Scans", "%06d.pcd" % k))
        b = b[np.isfinite(b).all(1)]
        return b @ self.Rk[k].T + self.pk[k]


def past_start(path, cfg):
    """the path every 5 cm, less its first start_skip (the robot is already there: a start on a
    flight path can sit in an obstacle cell)"""
    P = np.array(tc.densify(path, 0.05))
    far = np.hypot(*(P - P[0]).T) >= cfg.start_skip
    return P[far] if far.any() else P[-1:]


def stats_ms(x):
    x = np.asarray(x) * 1000
    return {"median": float(np.median(x)), "p95": float(np.percentile(x, 95)), "max": float(x.max()), "n": int(len(x))} if len(x) else {}


class RealMap:
    def __init__(self, label, spec, log=print):
        self.label, self.log = label, log
        self.P, path, kfdir = load_spec(spec)
        self.kf = Keyframes(kfdir, path) if kfdir else None
        self.D = np.array(tc.densify(path[:, :2], 0.05))
        self.Dz = np.interp(np.linspace(0, len(path) - 1, len(self.D)), np.arange(len(path)), path[:, 2])
        self.S = np.r_[0, np.cumsum(np.linalg.norm(np.diff(self.D, axis=0), axis=1))]

    # map mode
    def map_mode(self, dists=(2, 5, 10, 20), step=2.0, low=9.0, cfg=None, evidence=True):
        cfg = cfg or tc.Config()
        t0 = time.time()
        hg = tc.HeightGrid(self.P, cfg)
        hits = miss = None
        dropped = 0
        if self.kf is not None and evidence:
            hits, miss = tc.evidence(hg, ((self.kf.scan(k), self.kf.pk[k]) for k in range(len(self.kf))))
            dropped = hg.apply_evidence(hits, miss)
        build = time.time() - t0
        D, S, Dz = self.D, self.S, self.Dz
        res = {"build_s": build, "moved_cells_dropped": dropped, "plans": {}, "fails": []}
        times = []
        for dist in dists:
            ok = n = 0
            for i in np.arange(0, len(D), int(step / 0.05)):
                j = int(np.searchsorted(S, S[i] + dist))
                if j >= len(D) or Dz[i] > low or Dz[j] > low:
                    continue
                t0 = time.time()
                out = tc.plan(hg, None, D[i], D[j])
                times.append(time.time() - t0)
                n += 1
                ok += out["ok"]
                if not out["ok"]:
                    res["fails"].append((float(S[i]), dist, out["reason"]))
            res["plans"][dist] = (ok, n)
        res["time"] = stats_ms(times)
        res["walls"] = self.walls(hg, hits, miss, low)
        ref = self.ref_walls(hg, hits, miss)
        wc = np.argwhere(ref)
        self.ref_xy = np.c_[hg.lo[0] + (wc[:, 1] + 0.5) * cfg.res, hg.lo[1] + (wc[:, 0] + 0.5) * cfg.res]
        return res

    def ref_walls(self, hg, hits, miss):
        r = hg.cfg.res
        ij = np.floor((self.P[:, :2] - hg.lo) / r).astype(int)
        ok = (ij[:, 0] >= 0) & (ij[:, 1] >= 0) & (ij[:, 0] < hg.shape[1]) & (ij[:, 1] < hg.shape[0])
        ij, z = ij[ok], self.P[ok, 2]
        h = z - hg.ground[ij[:, 1], ij[:, 0]]
        m = np.ones(hg.shape, bool)
        for lo in (0.3, 0.6, 0.9, 1.2):
            b = (h >= lo) & (h < lo + 0.3)
            g = np.zeros(hg.shape, bool)
            g[ij[b, 1], ij[b, 0]] = True
            m &= g
        if hits is not None:
            m &= (hits >= 3) & (miss <= hits)
        return m

    def walls(self, hg, hits, miss, low, n=40):
        D, S, Dz = self.D, self.S, self.Dz
        walls = self.ref_walls(hg, hits, miss)
        wc = np.argwhere(walls)
        if not len(wc):
            return {"pairs": (0, 0), "behind": (0, 0, 0), "fails": []}
        wxy = np.c_[hg.lo[0] + (wc[:, 1] + 0.5) * hg.cfg.res, hg.lo[1] + (wc[:, 0] + 0.5) * hg.cfg.res]
        tw = cKDTree(wxy)

        def judge(s, g):
            out = tc.plan(hg, None, s, g)
            if not out["ok"]:
                return True, False, "refused: " + out["reason"]
            d = tw.query(past_start(out["path"], hg.cfg))[0]
            return bool(d.min() >= 0.2), True, "routed %.1f m, %.2f m from a wall" % (out["length"], d.min())

        rng = np.random.RandomState(0)
        tree = cKDTree(D)
        pairs = []
        for i in rng.permutation(np.arange(0, len(D), 10)):
            for j in tree.query_ball_point(D[i], 4.0):
                e = np.linalg.norm(D[j] - D[i])
                if e < 1.0 or abs(S[j] - S[i]) < 3 * e + 2 or Dz[i] > low or Dz[j] > low:
                    continue
                seg = [D[i] + (D[j] - D[i]) * t for t in np.linspace(0, 1, int(e / 0.05) + 2)]
                if any(walls[hg.cell(p)] for p in seg if hg.inside(hg.cell(p))):
                    pairs.append((D[i], D[j]))
                    break
            if len(pairs) >= n:
                break
        behind = []
        for i in np.random.RandomState(1).permutation(np.arange(0, len(D), 10)):
            if Dz[i] > low:
                continue
            d, k = tw.query(D[i])
            if 0.4 <= d <= 2.5:
                behind.append((D[i], wxy[k] + (wxy[k] - D[i]) / d * 1.0))
            if len(behind) >= n:
                break
        fails = []
        good = 0
        for s, g in pairs:
            ok, _, why = judge(s, g)
            good += ok
            if not ok:
                fails.append(("pair", s.tolist(), g.tolist(), why))
        good2 = routed = 0
        for s, g in behind:
            ok, r, why = judge(s, g)
            good2 += ok
            routed += ok and r
            if not ok:
                fails.append(("behind", s.tolist(), g.tolist(), why))
        return {"pairs": (good, len(pairs)), "behind": (good2, len(behind), routed), "fails": fails}

    # live mode
    def live_mode(self, goals=(3.0, 6.0), live_step=2, window_s=3.0, low=9.0, cfg=None, **lw_kw):
        kf = self.kf
        if kf is None:
            return None
        cfg = cfg or tc.live_config()
        D, S, Dz = self.D, self.S, self.Dz
        tk = kf.t - kf.t[0]
        Dk = cKDTree(D)
        lw = tc.LiveWindow(cfg, window_s=window_s, **lw_kw)
        ing, times = [], []
        ref = getattr(self, "ref_xy", np.zeros((0, 2)))
        tw = cKDTree(ref) if len(ref) else None
        wall_n = wall_ok = wall_routed = wall_unseen = 0
        ok = {d: 0 for d in goals}
        n = {d: 0 for d in goals}
        okf = {d: 0 for d in goals}
        nf = {d: 0 for d in goals}
        fails = []
        for k in range(len(kf)):
            scan = kf.scan(k)
            t0 = time.time()
            lw.add_scan(tk[k], scan, kf.pk[k])
            ing.append(time.time() - t0)
            if k % live_step:
                continue
            i = Dk.query(kf.pk[k, :2])[1]
            for dist in goals:
                j = int(np.searchsorted(S, S[i] + dist))
                if j >= len(D) or Dz[i] > low or Dz[j] > low:
                    continue
                t0 = time.time()
                out = lw.plan(kf.pk[k, :2], D[j])
                times.append(time.time() - t0)
                n[dist] += 1
                ok[dist] += out["ok"]
                # free now: no band return of this very scan within the frame's half-width of the
                # driven stretch (the path is known free when the robot drove it, not before)
                if "hg" in out:
                    hg = out["hg"]
                    q = lw.recent[-1][0]
                    cidx = np.floor((q[:, :2] - hg.lo) / hg.cfg.res).astype(int)
                    inb = (cidx[:, 0] >= 0) & (cidx[:, 1] >= 0) & (cidx[:, 0] < hg.shape[1]) & (cidx[:, 1] < hg.shape[0])
                    q, cidx = q[inb], cidx[inb]
                    hq = q[:, 2] - hg.ground[cidx[:, 1], cidx[:, 0]]
                    bq = q[(hq > hg.cfg.z_lo) & (hq < hg.cfg.z_hi), :2]
                    seg = D[i:j + 1:2]
                    seg = seg[np.hypot(*(seg - kf.pk[k, :2]).T) > hg.cfg.start_skip]
                    busy = len(bq) and len(seg) and cKDTree(bq).query(seg)[0].min() < tc.FOOTPRINT[1] / 2
                    if not busy:
                        nf[dist] += 1
                        okf[dist] += out["ok"]
                if not out["ok"]:
                    fails.append((k, dist, out["reason"]))
            # a goal 1 m behind the nearest reference wall ahead of the nose (0.4-2.5 m away)
            if tw is not None:
                p = kf.pk[k, :2]
                nose = (kf.Rk[k] @ np.array([0.0, 0.0, 1.0]))[:2]
                idx = tw.query_ball_point(p, 2.5)
                cand = [c for c in idx if np.linalg.norm(ref[c] - p) >= 0.4 and
                        np.dot(ref[c] - p, nose) >= 0.5 * np.linalg.norm(ref[c] - p) * np.linalg.norm(nose)]
                if cand:
                    c = min(cand, key=lambda c: np.linalg.norm(ref[c] - p))
                    u = (ref[c] - p) / np.linalg.norm(ref[c] - p)
                    out = lw.plan(p, ref[c] + u)
                    wall_n += 1
                    if not out["ok"]:
                        wall_ok += 1
                    else:
                        # only walls the window has seen count: unknown space is driven through by
                        # design (optimistic, replanned as it is seen)
                        hl = out["hg"]
                        P = past_start(out["path"], cfg)
                        d, i = tw.query(P)
                        seen = np.array([hl.inside(hl.cell(ref[j])) and hl.seen_up[hl.cell(ref[j])] for j in i])
                        hit_seen = bool(np.any((d < 0.2) & seen))
                        wall_ok += not hit_seen
                        wall_routed += not hit_seen
                        wall_unseen += bool(np.any((d < 0.2) & ~seen)) and not hit_seen
                        if hit_seen:
                            fails.append((k, "wall", "path %.2f m from a seen wall" % d[seen].min()))
        return {"plans": {d: (ok[d], n[d]) for d in goals}, "plans_free_now": {d: (okf[d], nf[d]) for d in goals}, "time": stats_ms(times), "ingest": stats_ms(ing), "fails": fails,
                "walls": (wall_ok, wall_n, wall_routed, wall_unseen)}
