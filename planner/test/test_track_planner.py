#!/usr/bin/env python3
"""Track planner tests: synthetic cases, then real maps when given.

  python3 test_track_planner.py [--map LABEL:SPEC ...] [--live-step N] [--gate]

SPEC is view=DIR[,kf=OUT] (map_points.bin + map_meta.json, ground at 0) or
pcd=FILE,path=CSV|meta=VIEWDIR[,kf=OUT]; OUT is the mapping run's SC-PGO output (keyframe scans),
which adds free-space evidence to the map and the live-window replay (real_maps.py). Real maps
pass with >= 90% of the goals along the driven path planned on the whole map, >= 85% on the live
window, and every wall check passed; --gate also requires the live replan p95 <= 100 ms (Gate L1:
>= 10 Hz; run it pinned to 4 cores: taskset -c <4 cpus>).
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
import track_core as tc  # noqa: E402

results = []


def check(name, ok, detail=""):
    results.append(bool(ok))
    print("%s  %-62s %s" % ("PASS" if ok else "FAIL", name, detail))


RNG = np.random.RandomState(0)


def floor(x0, x1, y0, y1, thick=0.06, step=0.05):
    xx, yy = np.meshgrid(np.arange(x0, x1, step), np.arange(y0, y1, step))
    n = xx.size
    return np.c_[xx.ravel(), yy.ravel(), RNG.uniform(-0.02, thick - 0.02, n)]


def wall(x0, y0, x1, y1, z0=0.0, z1=1.0, step=0.03):
    n = max(2, int(np.hypot(x1 - x0, y1 - y0) / step))
    zs = np.arange(z0, z1, 0.05)
    xy = np.c_[np.linspace(x0, x1, n), np.linspace(y0, y1, n)]
    return np.vstack([np.c_[xy, np.full(n, z)] for z in zs])


def frame_clear(path, obs, cfg):
    """independent check: the bare frame (no margin), centre footprint_offset behind each follower
    pose, heading to the point lookahead ahead, holds no obstacle point; -> (ok, worst intrusion m)"""
    P = np.array(tc.densify(path, 0.02))
    cum = np.r_[0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
    worst = 0.0
    for k, p in enumerate(P):
        if np.linalg.norm(p - P[0]) < cfg.start_skip:
            continue
        q = P[min(int(np.searchsorted(cum, cum[k] + cfg.lookahead)), len(P) - 1)]
        d = q - p if np.linalg.norm(q - p) > 1e-6 else P[-1] - P[-2]
        yaw = math.atan2(d[1], d[0])
        c, s = math.cos(yaw), math.sin(yaw)
        o = obs - (p - cfg.off * np.array([c, s]))
        lon, lat = np.abs(o[:, 0] * c + o[:, 1] * s), np.abs(-o[:, 0] * s + o[:, 1] * c)
        inside = (lon < tc.FOOTPRINT[0] / 2) & (lat < tc.FOOTPRINT[1] / 2)
        if inside.any():
            worst = max(worst, float(np.max(np.minimum(tc.FOOTPRINT[0] / 2 - lon[inside], tc.FOOTPRINT[1] / 2 - lat[inside]))))
    return worst == 0.0, worst


def band_xy(pts, cfg, floor_z=0.0):
    h = pts[:, 2] - floor_z
    return pts[(h > cfg.z_lo) & (h < cfg.z_hi), :2]


def synthetic():
    cfg = tc.Config()
    # a wall across the middle with a 1.6 m gap at the top, on a floor
    walls = np.vstack([wall(0, -5, 0, 1.2), wall(-6, -5, 6, -5), wall(-6, 3.0, 6, 3.0)])
    room = np.vstack([floor(-6, 6, -5, 3), walls])
    out = tc.plan(room, None, (-3, 0), (3, 0), cfg)
    check("around a wall through the gap (local ground)", out["ok"] and out["length"] > 6.0, "length %.2f m" % out.get("length", 0))
    ok, worst = frame_clear(out["path"], band_xy(walls, cfg), cfg) if out["ok"] else (False, 0)
    check("the bare frame never touches the wall (independent check)", ok, "worst intrusion %.3f m" % worst)

    # a floor smeared 0.25 m thick and a high shelf are not obstacles on a whole map (z_lo 0.30)
    smear = floor(-3, 3, -2, 2, thick=0.25)
    shelf = wall(-3, 0.0, 3, 0.0, z0=1.4, z1=1.6)
    out = tc.plan(np.vstack([smear, shelf]), None, (-2, -1), (2, 1), cfg)
    check("0.25 m floor smear and a high shelf do not block", out["ok"] and out["length"] < 4.7, "length %.2f m" % out.get("length", 0))

    # boxed in: no path
    box = np.vstack([floor(-3, 6, -3, 3), wall(-1, -1, 1, -1), wall(1, -1, 1, 1), wall(1, 1, -1, 1), wall(-1, 1, -1, -1)])
    out = tc.plan(box, None, (0, 0), (4, 0), cfg)
    check("enclosed start reports no path", not out["ok"], out.get("reason", ""))

    # doorways: the frame is 0.457 m wide (+0.05 m margin each side), 0.68 m across the diagonal
    def door(w):
        h = w / 2
        return np.vstack([floor(-4, 4, -4, 4), wall(0, -4, 0, -h), wall(0, h, 0, 4), wall(-4, -4, 4, -4), wall(-4, 4, 4, 4)])
    out = tc.plan(door(0.50), None, (-2, 0), (2, 0), cfg)
    check("0.50 m doorway refused (frame + margins 0.56 m)", not out["ok"], out.get("reason", ""))
    for w in (0.70, 0.80):
        d = door(w)
        out = tc.plan(d, None, (-2, 0), (2, 0), cfg)
        ok, worst = frame_clear(out["path"], band_xy(d, cfg), cfg) if out["ok"] else (False, 0)
        check("%.2f m doorway passed straight through, frame clear" % w, out["ok"] and ok,
              "length %.2f m, worst intrusion %.3f m" % (out.get("length", 0), worst))

    # oriented check: in a 0.7 m corridor the frame fits along it, not across it
    corr = np.vstack([floor(-3, 3, -2, 2), wall(-3, -0.35, 3, -0.35), wall(-3, 0.35, 3, 0.35)])
    hg = tc.HeightGrid(corr, cfg)
    win = hg.crop((-2, -1), (2, 1))
    check("0.7 m corridor: frame free along it, blocked across it",
          win.pose_free((0.0, 0.0), 0.0) and win.pose_free((0.0, 0.0), math.pi) and not win.pose_free((0.0, 0.0), math.pi / 2), "")

    # a person on the path in an old scan, seen through by the latest scan: cleared
    lcfg = tc.live_config()
    back = wall(-1, 3, 1, 3)
    person = wall(-0.1, 1.5, 0.1, 1.5, z1=1.7)
    fl = floor(-3, 3, -1, 4)
    hg = tc.HeightGrid(np.vstack([fl, back, person]), lcfg, (-3, -1), (3, 4))
    blocked = tc.plan(hg, None, (0, 0), (0, 2.5))
    hits, miss = tc.evidence(hg, [(np.vstack([fl, back]), (0.0, 0.0, 0.25))])
    dropped = hg.apply_evidence(hits, miss)
    out = tc.plan(hg, None, (0, 0), (0, 2.5))
    back_kept = all(hg.occ[hg.cell((x, 3.0))] for x in (-0.5, 0.0, 0.5))
    check("a moved person is cleared by the latest scan, the wall behind is kept",
          dropped > 0 and out["ok"] and out["length"] < 2.6 and back_kept and (not blocked["ok"] or blocked["length"] > 2.6),
          "dropped %d cells, before: %s, after %.2f m" % (dropped, "%.2f m" % blocked["length"] if blocked["ok"] else "no path", out.get("length", 0)))

    # the lidar ~0.08 m above the floor: a person in front is cleared by rays rising to the wall
    # behind, but rays to the floor beyond a wall (seen through a gap below it) do not clear it
    hg = tc.HeightGrid(np.vstack([fl, back, person]), lcfg, (-3, -1), (3, 4))
    hits, miss = tc.evidence(hg, [(np.vstack([fl, back]), (0.0, 0.0, 0.08))], low_sensor=True)
    dropped_low = hg.apply_evidence(hits, miss)
    person_gone = not hg.occ[hg.cell((0.0, 1.5))]
    shelf = np.vstack([floor(-3, 3, -1, 5), wall(-1, 2.0, 1, 2.0, z0=0.2, z1=1.0)])      # a wall standing on legs
    hg = tc.HeightGrid(shelf, lcfg, (-3, -1), (3, 5))
    floor_only = floor(-3, 3, -1, 5)                                                     # a later scan sees only floor
    hits, miss = tc.evidence(hg, [(floor_only, (0.0, 0.0, 0.08))], low_sensor=True)
    hg.apply_evidence(hits, miss)
    kept = all(hg.occ[hg.cell((x, 2.0))] for x in (-0.5, 0.0, 0.5))
    check("low lidar (0.08 m): a person is cleared, floor rays do not clear a wall", person_gone and dropped_low > 0 and kept,
          "person cleared %s, wall kept %s" % (person_gone, kept))

    # ground model: a tile seeing only a ceiling is not ground; a sofa seen from above is an obstacle
    ceil = np.c_[RNG.uniform(1, 2, (400, 2)), np.full(400, 2.4)]
    fl = floor(-2, 4, -2, 4)
    fl = fl[~((fl[:, 0] > 1) & (fl[:, 0] < 2) & (fl[:, 1] > 1) & (fl[:, 1] < 2))]
    hg = tc.HeightGrid(np.vstack([fl, ceil]), cfg)
    check("a ceiling-only tile makes no obstacle (not taken as ground)", hg.occ.sum() == 0, "%d obstacle cells" % hg.occ.sum())
    g, lo, res = tc.local_ground(np.vstack([fl, ceil]))
    check("local_ground(): the floor everywhere, also under the ceiling-only tile", np.abs(g).max() < 0.05 and res == 0.1,
          "ground %.3f..%.3f m, lo %s" % (g.min(), g.max(), lo))
    xx, yy = np.meshgrid(np.arange(0, 1.2, 0.04), np.arange(0, 0.8, 0.04))
    sofa = np.vstack([np.c_[xx.ravel(), yy.ravel(), np.full(xx.size, 0.45)],
                      wall(0, 0, 1.2, 0, z1=0.45), wall(1.2, 0, 1.2, 0.8, z1=0.45), wall(1.2, 0.8, 0, 0.8, z1=0.45), wall(0, 0.8, 0, 0, z1=0.45)])
    fl = floor(-3, 4, -3, 4)
    fl = fl[~((fl[:, 0] > 0) & (fl[:, 0] < 1.2) & (fl[:, 1] > 0) & (fl[:, 1] < 0.8))]
    out = tc.plan(np.vstack([fl, sofa]), None, (-2, 0.4), (3, 0.4), cfg)
    near = min(np.hypot(*(p - np.array([0.6, 0.4]))) for p in tc.densify(out["path"], 0.05)) if out["ok"] else 0.0
    check("a sofa (0.45 m, no floor seen under it) is driven around", out["ok"] and near > 0.6,
          "length %.2f m, nearest %.2f m from its middle" % (out.get("length", 0), near))

    # the live window forgets: an obstacle last seen more than window_s ago is gone
    lw = tc.LiveWindow(tc.live_config(), window_s=3.0)
    blk = np.vstack([floor(-3, 3, -2, 4), wall(-2, 1.5, 2, 1.5)])
    lw.add_scan(0.0, blk, (0, 0, 0.25))
    a = lw.plan((0, 0), (0, 3))
    for t in (3.5, 4.0, 4.5):
        lw.add_scan(t, floor(-3, 3, -2, 4), (0, 0, 0.25))
    b = lw.plan((0, 0), (0, 3))
    check("live window: a wall seen 4 s ago is forgotten", (not a["ok"] or a["length"] > 3.5) and b["ok"] and b["length"] < 3.2,
          "before %s, after %.2f m" % ("%.2f m" % a["length"] if a["ok"] else a["reason"], b.get("length", 0)))

    # the evidence memory: a wall seen from 3 m and then out of view is still an obstacle after it
    # leaves the window; a person seen right beside the robot is not held once out of the window
    lw = tc.LiveWindow(tc.live_config(), window_s=3.0)
    fl = floor(-3, 3, -1, 5)
    seen = np.vstack([fl, wall(-1.5, 3.0, 1.5, 3.0), wall(0.5, 0.4, 0.7, 0.4, z1=1.7)])
    for t in np.arange(0.0, 2.5, 0.5):
        lw.add_scan(t, seen, (0, 0, 0.08))
        lw.plan((0, 0), (0, 4.5))
    for t in np.arange(4.0, 6.5, 0.5):
        lw.add_scan(t, fl[fl[:, 1] < 0.0], (0, 0, 0.08))      # looking away: no evidence either way
        out = lw.plan((0, 0), (0, 4.5))
    hg = out["hg"]
    wall_held = all(hg.occ[hg.cell((x, 3.0))] for x in (-0.5, 0.0, 0.5))
    person_gone = not hg.occ[hg.cell((0.6, 0.4))]
    check("memory: a wall seen from afar is held after the window, a person beside the robot is not",
          wall_held and person_gone and out["ok"] and out["length"] > 4.8,
          "wall held %s, person gone %s, path %.2f m" % (wall_held, person_gone, out.get("length", 0)))

    # carrot: 0.5 m ahead along the path, yaw toward it
    path = [np.array([0.0, 0.0]), np.array([2.0, 0.0]), np.array([2.0, 2.0])]
    c, yaw = tc.carrot(path, (1.0, 0.1), 0.5)
    check("carrot 0.5 m ahead on the path", abs(c[0] - 1.5) < 1e-6 and abs(c[1]) < 1e-6, "carrot %s yaw %.1f deg" % (np.round(c, 3), np.degrees(yaw)))
    c, yaw = tc.carrot(path, (1.9, 0.0), 0.5)
    check("carrot turns the corner", abs(c[0] - 2.0) < 1e-6 and abs(c[1] - 0.4) < 1e-6, "carrot %s" % np.round(c, 3))
    c, _ = tc.carrot(path, (2.0, 1.9), 0.5)
    check("carrot stops at the goal", np.allclose(c, [2.0, 2.0]), "carrot %s" % np.round(c, 3))
    rem = tc.remaining(path, (1.0, 0.1))
    check("remaining path from the robot", len(rem) == 3 and np.allclose(rem[1], [2, 0]), "")

    # the drive chain in the closed loop: the carrot, the planner bridge's drive target
    # (bridge_core.drive_target, ~drive_lookahead toward the carrot from the FC) and a skid-steer
    # driving to it (Rover Guided: pivot over WP_PIVOT_ANGLE 60 deg, else drive at WP_SPEED 0.5 m/s)
    # around the wall with the gap: the bare frame stays clear and the goal is reached
    ok, worst, reached, t = drive_chain(cfg, walls, room, (-3.0, 0.0), (3.0, 0.0))
    check("drive chain (carrot -> bridge target -> skid-steer) around the wall, frame clear", ok and reached,
          "reached %s in %.0f s, worst intrusion %.3f m" % (reached, t, worst))

    pivots(cfg)
    runouts(cfg)
    reverses(cfg)


def disc_clear(centre, obs, r=tc.FOOTPRINT_HALF_DIAGONAL):
    """independent check: no obstacle point within the bare frame's half-diagonal of a pivot centre
    -> (ok, nearest obstacle m)"""
    d = float(np.min(np.hypot(*(obs - np.asarray(centre)).T))) if len(obs) else 1e3
    return d >= r, d


def legs_clear(out, obs, cfg):
    """independent check of a plan with pivots: each driving leg on its own (frame_clear), each pivot
    circle (disc_clear) -> (ok, worst intrusion, nearest obstacle to a pivot centre)"""
    P = [np.asarray(p) for p in out["path"]]
    starts = [0] + [p["i"] + 1 for p in out["pivots"]]
    ends = [p["i"] for p in out["pivots"]] + [len(P) - 1]
    worst, near, ok = 0.0, 1e3, True
    for k, (a, b) in enumerate(zip(starts, ends)):
        if b - a < 1:
            continue
        leg = P[a:b + 1]
        if k > 0:
            # the start skip of frame_clear is for the plan's start only
            c = cfg.start_skip
            cfg.start_skip = 0.0
            o, w = frame_clear(leg, obs, cfg)
            cfg.start_skip = c
        else:
            o, w = frame_clear(leg, obs, cfg)
        ok &= o
        worst = max(worst, w)
    for p in out["pivots"]:
        o, d = disc_clear(p["centre"], obs)
        ok &= o
        near = min(near, d)
    return ok, worst, near


def pivots(cfg):
    """pivots in place (WP_PIVOT_ANGLE 60): the circle of the frame's half-diagonal (+ margin) about
    the frame centre must be clear where the plan turns by more than 60 deg"""
    fl = floor(-5, 5, -5, 5)
    # walls and points 0.025 m off the 0.1 m cell boundaries: a wall on a boundary falls in the
    # cell on either side depending on the grid's origin (float rounding)
    e = 0.025

    def w(x0, y0, x1, y1):
        return wall(x0 + e, y0 + e, x1 + e, y1 + e)

    def at(x, y):
        return (x + e, y + e)

    # a 90 deg corner of two 0.7 m corridors: the frame fits along each, the pivot circle (0.78 m
    # across) fits nowhere near the corner
    L = np.vstack([w(-4, -0.35, 0, -0.35), w(-4, 0.35, -0.7, 0.35), w(0, -0.35, 0, 4), w(-0.7, 0.35, -0.7, 4),
                   w(-4, -0.35, -4, 0.35), w(-0.7, 4, 0, 4)])
    out = tc.plan(np.vstack([fl, L]), None, at(-3, 0), at(-0.35, 3), cfg, start_yaw=0.0)
    check("pivot: a 90 deg corner of 0.7 m corridors is refused", not out["ok"], out.get("reason", ""))

    # in a 0.7 m corridor at the door of a 1.5 m x 2 m room, facing in, the goal behind: no room to
    # pivot where it stands, so the robot drives into the room, pivots there and comes back
    cor = np.vstack([w(-4, -0.35, 0.5, -0.35), w(-4, 0.35, 0.5, 0.35), w(-4, -0.35, -4, 0.35),
                     w(0.5, 0.35, 0.5, 1.0), w(0.5, -0.35, 0.5, -1.0), w(0.5, 1.0, 2.0, 1.0), w(0.5, -1.0, 2.0, -1.0),
                     w(2.0, -1.0, 2.0, 1.0)])
    cl = np.vstack([fl, cor])
    out = tc.plan(cl, None, at(0.7, 0), at(-2.5, 0), cfg, start_yaw=0.0)
    obs = band_xy(cor, cfg)
    ok, worst, near = legs_clear(out, obs, cfg) if out["ok"] else (False, 0, 0)
    pv = out.get("pivots", [])
    moved = len(pv) == 1 and not pv[0]["start"] and pv[0]["centre"][0] > 0.6
    check("pivot: no room where it stands -> moved ahead into the room, frame and circle clear", out["ok"] and moved and ok,
          "pivots %s, worst intrusion %.3f m, circle %.2f m from a wall" % (
              [(round(p["centre"][0], 2), round(p["centre"][1], 2), p["start"]) for p in pv], worst, near) if out["ok"] else out["reason"])

    # a U-turn in a dead-end 0.7 m corridor: refused (without the robot's heading it is planned,
    # as before)
    dead = np.vstack([w(-4, -0.35, 1.0, -0.35), w(-4, 0.35, 1.0, 0.35), w(1.0, -0.35, 1.0, 0.35), w(-4, -0.35, -4, 0.35)])
    dc = np.vstack([fl, dead])
    out = tc.plan(dc, None, at(0, 0), at(-2.5, 0), cfg, start_yaw=0.0)
    back = tc.plan(dc, None, at(0, 0), at(-2.5, 0), cfg, start_yaw=math.pi)
    blind = tc.plan(dc, None, at(0, 0), at(-2.5, 0), cfg)
    check("pivot: U-turn in a 0.7 m corridor refused; facing the goal it drives", not out["ok"] and back["ok"] and not back["pivots"] and blind["ok"],
          "facing away: %s; facing the goal: %s; no heading: %s" % (
              out.get("reason") or "planned", "planned, %d pivots" % len(back.get("pivots", [])) if back["ok"] else back["reason"],
              "planned" if blind["ok"] else blind["reason"]))

    # a hairpin through a 1.2 m x 2.5 m room, in by a 0.7 m corridor, out by a 0.9 m one: pivots in
    # the room (the way out is wider so that the drive chain's arcs, a turn under 60 deg after a
    # replan, do not touch it: those are the follower's, see LIMITS)
    hp = np.vstack([w(-4, -0.35, 0, -0.35), w(-4, 0.35, 0, 0.35), w(-4, 0.95, 0, 0.95), w(-4, 1.85, 0, 1.85),
                    w(0, -0.35, 0, -0.5), w(0, 1.85, 0, 2.0), w(0, -0.5, 1.2, -0.5), w(0, 2.0, 1.2, 2.0),
                    w(1.2, -0.5, 1.2, 2.0), w(0, 0.35, 0, 0.95), w(-4, -0.35, -4, 0.35), w(-4, 0.95, -4, 1.85)])
    hc = np.vstack([fl, hp])
    out = tc.plan(hc, None, at(-1.5, 0), at(-1.5, 1.4), cfg, start_yaw=0.0)
    obs = band_xy(hp, cfg)
    ok, worst, near = legs_clear(out, obs, cfg) if out["ok"] else (False, 0, 0)
    pv = out.get("pivots", [])
    check("pivot: hairpin through a 1.2 m room is planned with pivots there, frame and circles clear",
          out["ok"] and len(pv) >= 1 and all(p["centre"][0] > 0.0 for p in pv) and ok,
          "%d pivots %s, turns %s deg, worst intrusion %.3f m, circles >= %.2f m from a wall" % (
              len(pv), [(round(p["centre"][0], 2), round(p["centre"][1], 2)) for p in pv],
              [round(math.degrees(p["turn"])) for p in pv], worst, near) if out["ok"] else out["reason"])
    ok, worst, reached, t, seen = drive_chain_pivots(cfg, hp, hc, at(-1.5, 0.0), 0.0, np.array(at(-1.5, 1.4)))
    check("pivot: drive chain with pivots through the hairpin, frame clear, each pivot circle clear",
          ok and reached and len(seen) > 0 and all(d >= tc.FOOTPRINT_HALF_DIAGONAL for _, d in seen),
          "reached %s in %.0f s, worst intrusion %.3f m, %d pivots driven, circles >= %s m from a wall" % (
              reached, t, worst, len(seen), round(min(d for _, d in seen), 2) if seen else "-"))

    # the start pivot in the open: about the frame centre, footprint_offset behind the odometry point
    out = tc.plan(fl, None, (0, 0), (-3, 0), cfg, start_yaw=0.0)
    pv = out.get("pivots", [])
    ok = out["ok"] and len(pv) == 1 and pv[0]["start"] and np.allclose(pv[0]["centre"], [-cfg.off, 0], atol=1e-6) \
        and np.allclose(out["path"][1], [-2 * cfg.off, 0], atol=1e-6) and abs(abs(pv[0]["yaw_out"]) - math.pi) < 0.05
    same = tc.plan(fl, None, (0, 0), (-3, 0), cfg, start_yaw=math.pi)
    small = tc.plan(fl, None, (0, 0), (3, 1.5), cfg, start_yaw=0.0)     # 27 deg: driven, no pivot
    check("pivot: start pivot about the frame centre; none facing the goal or for a 27 deg turn",
          ok and same["ok"] and not same["pivots"] and small["ok"] and not small["pivots"],
          "centre %s, leaves at %s, yaw_out %.0f deg" % (np.round(pv[0]["centre"], 3), np.round(out["path"][1], 3), math.degrees(pv[0]["yaw_out"])) if pv else out.get("reason", ""))

    # the carrot as its own step: holds pivot_lead past the centre, then the leaving ray, then drives
    path = [np.array([0.0, 0.0]), np.array([2.0, 0.0]), np.array([2.0 - cfg.off, cfg.off]), np.array([2.0 - cfg.off, 2.0])]
    piv = [{"i": 1, "centre": [2.0 - cfg.off, 0.0], "yaw_in": 0.0, "yaw_out": math.pi / 2, "turn": math.pi / 2, "start": False}]
    c1, _, ph1, _ = tc.follow_plan(path, piv, (1.0, 0.0), 0.0, cfg, 0.15)
    c2, _, ph2, _ = tc.follow_plan(path, piv, (1.7, 0.0), 0.0, cfg, 0.15)
    c3, _, ph3, _ = tc.follow_plan(path, piv, (2.0, 0.0), 0.0, cfg, 0.15)
    c4, _, ph4, _ = tc.follow_plan(path, piv, (2.0 - cfg.off, cfg.off), math.pi / 2, cfg, 0.15)
    check("pivot: carrot drives, holds at the pivot, turns on the leaving ray, drives on",
          ph1 == "approach" and np.allclose(c1, [1.5, 0]) and ph2 == "approach" and np.allclose(c2, [2.0 - cfg.off + 0.15, 0])
          and ph3 == "pivot" and np.allclose(c3, [2.0 - cfg.off, cfg.off + cfg.lookahead]) and ph4 == "drive" and c4[1] > 0.6,
          "%s %s | %s %s | %s %s | %s %s" % (ph1, np.round(c1, 2), ph2, np.round(c2, 2), ph3, np.round(c3, 2), ph4, np.round(c4, 2)))


def drive_chain_pivots(cfg, obs_pts, cloud, start, yaw, goal, fc_behind=0.178, lookahead=0.5, speed=0.5, turn_rate=1.0,
                       dt=0.1, wp_radius=0.3, lead=0.15):
    """the drive leg with pivots: replan every second from the odometry point with its heading,
    follow_plan's carrot, the bridge's target from the FC, Rover Guided as a skid-steer about the FC:
    a pivot starts when the bearing to the target is over 60 deg and runs until within 5 deg, a
    target within wp_radius is reached (stop). -> (frame clear, worst intrusion, reached, time,
    [(FC at each pivot, nearest obstacle to the frame centre there)])"""
    import bridge_core as bc
    obs = band_xy(obs_pts, cfg)
    hg = tc.HeightGrid(cloud, cfg)
    odom = np.array(start, float)
    worst, t, out, phase, piv, seen = 0.0, 0.0, None, "drive", False, []
    while t < 180.0:
        if out is None or int(round(t / dt)) % 10 == 0:
            new = tc.plan(hg, None, odom, goal, start_yaw=yaw, start_pivoting=phase == "pivot")
            if not new["ok"]:
                return False, worst, False, t, seen
            if phase == "pivot" and out is not None:
                # as the node: a pivot under way finishes on its route while that still checks
                rest = tc.remaining(out["route"], odom)
                kept = tc.check_route(new["grid"], rest, yaw, True)
                if kept["ok"]:
                    new = {"path": kept["path"], "pivots": kept["pivots"], "route": rest}
            out = new
        if np.linalg.norm(odom - goal) < 0.3:
            return worst == 0.0, worst, True, t, seen
        c, _, phase, _ = tc.follow_plan(out["path"], out["pivots"], odom, yaw, cfg, lead)
        h = np.array([math.cos(yaw), math.sin(yaw)])
        fc = odom - fc_behind * h
        tgt = bc.drive_target([c[0], c[1], 0.0], [0, 0, 0], [fc[0], fc[1], 0.0], lookahead)[:2]
        d = tgt - fc
        err = bc.wrap_pi(math.atan2(d[1], d[0]) - yaw) if np.linalg.norm(d) > 1e-6 else 0.0
        if not piv and abs(err) > math.radians(60):
            piv = True
            centre = odom - cfg.off * h
            seen.append((fc.copy(), disc_clear(centre, obs)[1]))
        elif piv and abs(err) < math.radians(5):
            piv = False
        yaw += max(-turn_rate * dt, min(turn_rate * dt, err))
        if not piv and np.linalg.norm(d) >= wp_radius:
            v = min(speed, float(np.linalg.norm(d)) / dt)
            fc = fc + v * dt * np.array([math.cos(yaw), math.sin(yaw)])
        odom = fc + fc_behind * np.array([math.cos(yaw), math.sin(yaw)])
        cs, sn = math.cos(yaw), math.sin(yaw)
        o = obs - (odom - cfg.off * np.array([cs, sn]))
        lon, lat = np.abs(o[:, 0] * cs + o[:, 1] * sn), np.abs(-o[:, 0] * sn + o[:, 1] * cs)
        inside = (lon < tc.FOOTPRINT[0] / 2) & (lat < tc.FOOTPRINT[1] / 2)
        if inside.any():
            worst = max(worst, float(np.max(np.minimum(tc.FOOTPRINT[0] / 2 - lon[inside], tc.FOOTPRINT[1] / 2 - lat[inside]))))
        t += dt
    return worst == 0.0, worst, False, t, seen


def drive_chain_rover(cfg, obs_pts, cloud, start, yaw, goal, runout=2.1, wp_speed=1.0, kv=0.5, accel=1.0, fc_behind=0.178,
                      lookahead=2.5, turn_rate=1.0, dt=0.05, wp_radius=0.3, lead=0.15, settle=4.0):
    """the drive leg with Rover Guided's fast-update speed law: the track planner (replans every
    second, follow_plan, run_out with `runout`), the bridge (drive_target, ~drive_lookahead), and a
    skid-steer about the FC steering by pure pursuit on its target (curvature 2 sin(error) /
    distance, at most turn_rate) whose speed goes, at `accel`, to min(WP_SPEED, kv x distance to its
    target) (AR_WPNav input shaping: kv 0.5/s measured in SITL d10, WP_JERK 0), stops within
    wp_radius of the target, pivots over 60 deg (until within 5). After the planner's REACHED (odometry
    point within 0.3 m of the goal) it streams nothing and Rover runs on to the last target for
    `settle` s. -> dict(reached, worst intrusion, vmax, time, final odometry point, final speed)"""
    import bridge_core as bc
    obs = band_xy(obs_pts, cfg)
    hg = tc.HeightGrid(cloud, cfg)
    odom = np.array(start, float)
    goal = np.asarray(goal, float)
    worst, t, out, phase, piv, v, vmax, t_reached, tgt = 0.0, 0.0, None, "drive", False, 0.0, 0.0, None, None
    k = 0
    while t < 150.0:
        h = np.array([math.cos(yaw), math.sin(yaw)])
        fc = odom - fc_behind * h
        if t_reached is None:
            if np.linalg.norm(odom - goal) < 0.3:
                t_reached = t
            else:
                if out is None or k % int(round(1.0 / dt)) == 0:
                    new = tc.plan(hg, None, odom, goal, cfg, start_yaw=yaw, start_pivoting=phase == "pivot")
                    if not new["ok"]:
                        return {"reached": False, "worst": worst, "vmax": vmax, "t": t, "final": odom, "v": v, "why": new["reason"]}
                    if phase == "pivot" and out is not None:
                        rest = tc.remaining(out["route"], odom)
                        kept = tc.check_route(new["grid"], rest, yaw, True)
                        if kept["ok"]:
                            new = {"path": kept["path"], "pivots": kept["pivots"], "route": rest, "grid": new["grid"]}
                    out = new
                c, _, phase, j = tc.follow_plan(out["path"], out["pivots"], odom, yaw, cfg, lead)
                c, _ = tc.run_out(out["path"], out["pivots"], odom, yaw, c, phase, j, cfg, runout, fc_behind, lead, out["grid"])
                tgt = bc.drive_target([c[0], c[1], 0.0], [0, 0, 0], [fc[0], fc[1], 0.0], lookahead)[:2]
        elif t - t_reached > settle:
            break
        d = tgt - fc
        dist = float(np.linalg.norm(d))
        err = bc.wrap_pi(math.atan2(d[1], d[0]) - yaw) if dist > 1e-6 else 0.0
        if not piv and abs(err) > math.radians(60) and dist >= wp_radius:
            piv = True
        elif piv and abs(err) < math.radians(5):
            piv = False
        v_des = 0.0 if (piv or dist < wp_radius) else min(wp_speed, kv * dist)
        v += max(-accel * dt, min(accel * dt, v_des - v))
        if piv and abs(v) < 0.05:
            yaw += max(-turn_rate * dt, min(turn_rate * dt, err))
        elif not piv and dist >= wp_radius:
            # Rover's fast-update steering: pure pursuit on the target (curvature 2 sin(err) / dist)
            w_ = 2.0 * max(abs(v), 0.1) * math.sin(err) / dist
            yaw += max(-turn_rate * dt, min(turn_rate * dt, w_ * dt))
        fc = fc + v * dt * np.array([math.cos(yaw), math.sin(yaw)])
        odom = fc + fc_behind * np.array([math.cos(yaw), math.sin(yaw)])
        vmax = max(vmax, abs(v))
        cs, sn = math.cos(yaw), math.sin(yaw)
        o = obs - (odom - cfg.off * np.array([cs, sn]))
        lon, lat = np.abs(o[:, 0] * cs + o[:, 1] * sn), np.abs(-o[:, 0] * sn + o[:, 1] * cs)
        inside = (lon < tc.FOOTPRINT[0] / 2) & (lat < tc.FOOTPRINT[1] / 2)
        if inside.any():
            worst = max(worst, float(np.max(np.minimum(tc.FOOTPRINT[0] / 2 - lon[inside], tc.FOOTPRINT[1] / 2 - lat[inside]))))
        t += dt
        k += 1
    return {"reached": t_reached is not None, "worst": worst, "vmax": vmax, "t": t_reached if t_reached is not None else t,
            "final": odom, "v": v, "why": ""}


def runouts(cfg):
    """the run-out (track_core.run_out): the planner's target moved on along the carrot's bearing so
    Rover Guided's fast-update path (speed ~ 1/s x distance to the target) reaches WP_SPEED; none at
    stop points or while pivoting, never into an obstacle"""
    e = 0.025

    def w(x0, y0, x1, y1):
        return wall(x0 + e, y0 + e, x1 + e, y1 + e)

    fl = floor(-5, 5, -5, 5)
    win = tc.plan(fl, None, (0.0 + e, 0.0 + e), (4.0 + e, 0.0 + e), cfg)["grid"]
    path = [np.array([0.0, 0.0]), np.array([4.0, 0.0])]
    c, y, ph, j = tc.follow_plan(path, [], (1.0, 0.0), 0.0, cfg)
    t1, e1 = tc.run_out(path, [], (1.0, 0.0), 0.0, c, ph, j, cfg, 1.2, 0.178, 0.15, win)
    c2, _, ph2, j2 = tc.follow_plan(path, [], (3.5, 0.0), 0.0, cfg)
    t2, e2 = tc.run_out(path, [], (3.5, 0.0), 0.0, c2, ph2, j2, cfg, 1.2, 0.178, 0.15, win)
    t3, e3 = tc.run_out(path, [], (1.0, 0.0), 0.0, c, ph, j, cfg, 1.2, 0.178, 0.15, None)
    check("run-out: the target 1.2 m from the FC on the carrot's bearing; never past the goal; none without a window",
          abs(np.linalg.norm(t1 - [1.0 - 0.178, 0.0]) - 1.2) < 1e-6 and abs(t1[1]) < 1e-9 and e1 > 0.5
          and np.allclose(t2, [4.0, 0.0]) and e2 == 0.0 and np.allclose(t3, c) and e3 == 0.0,
          "open: %s (+%.2f m); 0.5 m from the goal: %s; no window: %s" % (np.round(t1, 3), e1, np.round(t2, 3), np.round(t3, 3)))

    # a pivot ahead: approach runs out at most to the hold point; pivoting: none
    path = [np.array([0.0, 0.0]), np.array([2.0, 0.0]), np.array([2.0 - cfg.off, cfg.off]), np.array([2.0 - cfg.off, 2.0])]
    piv = [{"i": 1, "centre": [2.0 - cfg.off, 0.0], "yaw_in": 0.0, "yaw_out": math.pi / 2, "turn": math.pi / 2, "start": False}]
    H = np.array([2.0 - cfg.off + 0.15, 0.0])
    out = []
    for x in (0.5, 1.0, 1.5):
        c, _, ph, j = tc.follow_plan(path, piv, (x, 0.0), 0.0, cfg, 0.15)
        tt, _ = tc.run_out(path, piv, (x, 0.0), 0.0, c, ph, j, cfg, 1.2, 0.178, 0.15, win)
        out.append((ph, tt))
    c, _, ph, j = tc.follow_plan(path, piv, (2.0, 0.0), 0.0, cfg, 0.15)
    tp, ep = tc.run_out(path, piv, (2.0, 0.0), 0.0, c, ph, j, cfg, 1.2, 0.178, 0.15, win)
    check("run-out: approaching a pivot never past its hold point; none while pivoting",
          all(p == "approach" and tt[0] <= H[0] + 1e-9 for p, tt in out) and ph == "pivot" and ep == 0.0 and np.allclose(tp, c),
          "approach targets x %s (hold %.3f); pivot carrot %s" % ([round(float(tt[0]), 3) for _, tt in out], H[0], np.round(tp, 3)))

    # a wall across the bearing past the carrot (a path that turns away before it): cut short of it
    wl = w(2.3, -2.0, 2.3, 1.0)
    win = tc.plan(np.vstack([fl, wl]), None, (0.0 + e, 0.0 + e), (-1.0 + e, 1.0 + e), cfg)["grid"]
    path = [np.array([0.0, 0.0]), np.array([4.0, 0.0])]       # (the run-out checks the frame, not the path)
    c, _, ph, j = tc.follow_plan(path, [], (1.2, 0.0), 0.0, cfg)
    tw, ew = tc.run_out(path, [], (1.2, 0.0), 0.0, c, ph, j, cfg, 1.2, 0.178, 0.15, win)
    front = tw[0] + 0.178 - cfg.off + tc.FOOTPRINT[0] / 2       # the bare frame's nose with the FC at the target
    # (the grid checks obstacle cell centres against the frame + margin: the bare frame is clear,
    # the margin absorbs the half-cell rounding, as poses_free)
    check("run-out: cut short of a wall across the bearing (bare frame clear, within a cell of it)",
          ew < 1.2 - 0.678 and 2.3 + e - cfg.margin - cfg.res <= front < 2.3 + e and tw[0] >= c[0],
          "target %s (+%.2f m), frame nose at %.3f m, wall at %.3f m" % (np.round(tw, 3), ew, front, 2.3 + e))

    # the drive chain with Rover's speed law through a 0.90 m door, approached at an angle (Gate L2)
    door = np.vstack([w(0.0, -5.0, 0.0, -0.45), w(0.0, 0.45, 0.0, 5.0)])
    cloud = np.vstack([fl, door])
    # (the bare carrot only for its speed: the model's low-speed steering is cruder than Rover's)
    bare = drive_chain_rover(cfg, door, cloud, (-2.5 + e, 1.2 + e), 0.0, (2.5 + e, -0.8 + e), runout=0.0)
    ro = drive_chain_rover(cfg, door, cloud, (-2.5 + e, 1.2 + e), 0.0, (2.5 + e, -0.8 + e), runout=2.1)
    past = float(np.linalg.norm(ro["final"] - [2.5 + e, -0.8 + e]))
    check("run-out: drive chain at WP_SPEED 1.0 through a 0.90 m door: ~1 m/s (bare carrot ~0.35), frame clear, stops at the goal",
          ro["reached"] and ro["worst"] == 0.0 and ro["vmax"] >= 0.9 and bare["vmax"] < 0.45
          and past < 0.4 and abs(ro["v"]) < 0.02,
          "run-out: %.1f s, vmax %.2f m/s, worst intrusion %.3f m, odometry point %.2f m from the goal at rest (v %.2f); "
          "bare carrot: reached %s %.1f s, vmax %.2f m/s %s %s" % (ro["t"], ro["vmax"], ro["worst"], past, ro["v"], bare["reached"], bare["t"], bare["vmax"], ro["why"], bare["why"]))


def reverses(cfg):
    """dead ends (owner 2026-10-28): the goal behind, no room to pivot where the robot stands -> back
    straight up the way it came (observed free cells only, the oriented frame clear, at most
    reverse_max) to the nearest spot the pivot circle fits, pivot there, drive on"""
    e = 0.025

    def w(x0, y0, x1, y1):
        return wall(x0 + e, y0 + e, x1 + e, y1 + e)

    # a 0.70 m dead-end corridor along +x (x -2..0, closed at 0) off an open room (x < -2)
    corr = np.vstack([w(-2.0, -0.35, 0.0, -0.35), w(-2.0, 0.35, 0.0, 0.35), w(0.0, -0.35, 0.0, 0.35),
                      w(-2.0, -3.0, -2.0, -0.35), w(-2.0, 0.35, -2.0, 3.0)])
    start, goal = (-0.6 + e, 0.0 + e), (-3.5 + e, 1.5 + e)
    fl = floor(-5, 0.5, -3, 3)
    out = tc.plan(np.vstack([fl, corr]), None, start, goal, cfg, start_yaw=0.0)
    rv = out.get("reverse")
    ok = out["ok"] and rv is not None and out["pivots"] and out["pivots"][0]["start"]
    worst = 0.0
    if ok:
        # independent: the bare frame along the reverse (heading kept) and the pivot circle at its end
        obs = band_xy(corr, cfg)
        for r in np.arange(0.0, rv["dist"] + 1e-9, 0.02):
            q = np.array(start) - r * np.array([1.0, 0.0])
            o = obs - (q - cfg.off * np.array([1.0, 0.0]))
            inside = (np.abs(o[:, 0]) < tc.FOOTPRINT[0] / 2) & (np.abs(o[:, 1]) < tc.FOOTPRINT[1] / 2)
            if inside.any():
                worst = max(worst, float(np.max(np.minimum(tc.FOOTPRINT[0] / 2 - np.abs(o[inside, 0]), tc.FOOTPRINT[1] / 2 - np.abs(o[inside, 1])))))
        circ, near = disc_clear(out["pivots"][0]["centre"], obs)
        lc, lworst, _ = legs_clear(out, obs, cfg)
        ok = ok and worst == 0.0 and circ and lc and rv["dist"] <= cfg.reverse_max
    check("reverse: dead-end 0.7 m corridor, goal behind -> back up to the opening, pivot, drive (frame and circle clear)",
          ok, "reverse %.2f m to %s, then pivots %s; worst intrusion %.3f m" % (
              rv["dist"], np.round(rv["to"], 2), [(round(p["centre"][0], 2), round(p["centre"][1], 2), round(math.degrees(p["turn"])))
                                                   for p in out["pivots"]], worst) if rv else out.get("reason", "") + " | " + out.get("reverse_reason", ""))

    # a box in the corridor behind the robot: refused, not backed into
    box = np.vstack([w(-1.6, -0.3, -1.6, 0.3), w(-1.4, -0.3, -1.4, 0.3)])
    cl = np.vstack([fl, corr, box])
    out = tc.plan(cl, None, start, goal, cfg, start_yaw=0.0)
    m = cfg.r_out + cfg.res * 2 + cfg.pad
    hg = tc.HeightGrid(cl, cfg, np.minimum(start, goal) - m, np.maximum(start, goal) + m)
    rb = tc.plan_reverse(hg, start, goal, 0.0)       # the reverse's own check (plan() fails the search first here)
    check("reverse: an obstacle behind in the corridor -> refused (plan, and the reverse sweep itself)",
          not out["ok"] and out.get("reverse") is None and not rb["ok"] and "blocked" in rb["reason"],
          out.get("reason", "") + " | plan_reverse: " + rb["reason"])

    # the corridor's floor behind the robot never seen (no returns): refused, never reversed into
    seen = fl[~((fl[:, 0] > -1.6) & (fl[:, 0] < -1.0) & (np.abs(fl[:, 1]) < 0.6))]
    out = tc.plan(np.vstack([seen, corr]), None, start, goal, cfg, start_yaw=0.0)
    check("reverse: unobserved floor behind -> refused (never into unknown space)",
          not out["ok"] and "unobserved" in out.get("reverse_reason", ""), out.get("reason", "") + " | " + out.get("reverse_reason", ""))

    # beyond reverse_max: refused
    c2 = tc.Config(reverse_max=0.8)
    out = tc.plan(np.vstack([fl, corr]), None, start, goal, c2, start_yaw=0.0)
    check("reverse: no room within reverse_max -> refused", not out["ok"] and out.get("reverse") is None,
          out.get("reason", "") + " | " + out.get("reverse_reason", ""))


def drive_chain(cfg, obs_pts, cloud, start, goal, fc_behind=0.178, lookahead=0.5, speed=0.5, turn_rate=1.0, dt=0.1):
    """simulate the drive leg: replan from the odometry point every second (the track planner, at
    10 Hz, keeps its path between), carrot, the bridge's target from the FC (fc_behind the odometry point), a skid-steer
    (unicycle) driving at it. -> (frame clear, worst intrusion, reached, time)"""
    import bridge_core as bc
    obs = band_xy(obs_pts, cfg)
    hg = tc.HeightGrid(cloud, cfg)
    yaw = 0.0
    odom = np.array(start, float)
    worst, t, path = 0.0, 0.0, None
    while t < 120.0:
        if path is None or int(round(t / dt)) % 10 == 0:
            out = tc.plan(hg, None, odom, goal)
            if not out["ok"]:
                return False, worst, False, t
            path = out["path"]
        if np.linalg.norm(odom - goal) < 0.3:
            return worst == 0.0, worst, True, t
        c, _ = tc.carrot(path, odom, cfg.lookahead)
        h = np.array([math.cos(yaw), math.sin(yaw)])
        fc = odom - fc_behind * h
        tgt = bc.drive_target([c[0], c[1], 0.0], [0, 0, 0], [fc[0], fc[1], 0.0], lookahead)[:2]
        d = tgt - fc
        err = bc.wrap_pi(math.atan2(d[1], d[0]) - yaw)
        yaw += max(-turn_rate * dt, min(turn_rate * dt, err))
        if abs(err) < math.radians(60):
            v = min(speed, float(np.linalg.norm(d)) / dt)
            fc = fc + v * dt * np.array([math.cos(yaw), math.sin(yaw)])
        odom = fc + fc_behind * np.array([math.cos(yaw), math.sin(yaw)])
        # the bare frame, its centre cfg.off behind the odometry point
        cs, sn = math.cos(yaw), math.sin(yaw)
        o = obs - (odom - cfg.off * np.array([cs, sn]))
        lon, lat = np.abs(o[:, 0] * cs + o[:, 1] * sn), np.abs(-o[:, 0] * sn + o[:, 1] * cs)
        inside = (lon < tc.FOOTPRINT[0] / 2) & (lat < tc.FOOTPRINT[1] / 2)
        if inside.any():
            worst = max(worst, float(np.max(np.minimum(tc.FOOTPRINT[0] / 2 - lon[inside], tc.FOOTPRINT[1] / 2 - lat[inside]))))
        t += dt
    return worst == 0.0, worst, False, t


def real(label, spec, live_step, gate):
    import real_maps
    m = real_maps.RealMap(label, spec)
    if m.kf is not None:
        print("%s: %d keyframes, fit to the path rms %.3f m" % (label, len(m.kf), m.kf.rms))
    r = m.map_mode()
    planned = sum(v[0] for v in r["plans"].values())
    total = sum(v[1] for v in r["plans"].values())
    per = ", ".join("%g m %d/%d" % (d, v[0], v[1]) for d, v in r["plans"].items())
    check("%s map: goals along the driven path planned" % label, total and planned >= 0.9 * total,
          "%d/%d (%s); %d moved cells dropped; plan median %.0f ms p95 %.0f ms" % (
              planned, total, per, r["moved_cells_dropped"], r["time"].get("median", 0), r["time"].get("p95", 0)))
    for f in r["fails"][:5]:
        print("        fail at %.0f m along, goal %g m ahead: %s" % f)
    w = r["walls"]
    check("%s map: walls refused or routed around" % label, w["pairs"][0] == w["pairs"][1] and w["behind"][0] == w["behind"][1],
          "pairs %d/%d; goals behind a wall %d/%d (%d routed around)" % (w["pairs"][0], w["pairs"][1], *w["behind"]))
    for f in w["fails"][:5]:
        print("        wall fail:", f)
    lv = m.live_mode(live_step=live_step)
    if lv is None:
        return
    planned = sum(v[0] for v in lv["plans"].values())
    total = sum(v[1] for v in lv["plans"].values())
    per = ", ".join("%g m %d/%d" % (d, v[0], v[1]) for d, v in lv["plans"].items())
    pf = sum(v[0] for v in lv["plans_free_now"].values())
    tf = sum(v[1] for v in lv["plans_free_now"].values())
    check("%s live window: goals along the driven path planned" % label, total and planned >= 0.85 * total,
          "%d/%d (%s); where the scan of the moment shows the driven stretch free: %d/%d; plans with a pivot %d (start pivots %d, pivots %d)" % (
              planned, total, per, pf, tf, *lv["pivots"]))
    for f in [f for f in lv["fails"] if f[1] != "wall"][:8]:
        print("        fail at keyframe %d, goal %g m ahead: %s" % f)
    w = lv["walls"]
    check("%s live window: goal behind a seen wall ahead refused or routed around" % label, w[0] == w[1],
          "%d/%d (%d routed around; %d of those through space the window had not seen, nothing seen there from z_lo to 2 m, where the map has a wall)" % w)
    for f in [f for f in lv["fails"] if f[1] == "wall"][:5]:
        print("        wall fail at keyframe %d: %s" % (f[0], f[2]))
    t, ing = lv["time"], lv["ingest"]
    detail = "replan median %.0f ms, p95 %.0f ms, max %.0f ms (n=%d); scan ingest median %.0f ms, p95 %.0f ms" % (
        t["median"], t["p95"], t["max"], t["n"], ing["median"], ing["p95"])
    if gate:
        check("%s live window: replan p95 <= 100 ms (Gate L1, 10 Hz)" % label, t["p95"] <= 100.0, detail)
    else:
        print("      %s" % detail)


if __name__ == "__main__":
    synthetic()
    args = sys.argv[1:]
    step = int(args[args.index("--live-step") + 1]) if "--live-step" in args else 2
    for i, a in enumerate(args):
        if a == "--map":
            label, spec = args[i + 1].split(":", 1)
            real(label, spec, step, "--gate" in args)
    print("%d of %d checks passed" % (sum(results), len(results)))
    sys.exit(0 if all(results) else 1)
