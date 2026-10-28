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
          "%d/%d (%s); where the scan of the moment shows the driven stretch free: %d/%d" % (planned, total, per, pf, tf))
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
