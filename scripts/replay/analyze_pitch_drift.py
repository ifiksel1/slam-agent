#!/usr/bin/env python3
"""
Diagnose LATERAL (left/right) drift during PITCH maneuvers.

Pairs the RAW input bag (has /lidar_imu) with the FAST-LIO replay output bag
(has /Odometry + /fastlio_health, produced by run_one.sh on isolated master :11312).

WHY THIS SCRIPT EXISTS -- the kinematic argument that rules out the calibration:
  For a pure pitch, body rate w = (0, wy, 0) and lever arm r, the induced velocity is
      w x r = (wy*rz, 0, -wy*rx)
  whose component ALONG THE PITCH AXIS is identically zero for ANY r. So no extrinsic_T
  error can move the vehicle laterally under pure pitch, and both a residual extrinsic_R
  error and a time-lag error act inside the pitch plane (forward/up). The LI-Init terms
  owned the ALTITUDE artifact (the wy*rx term) -- they cannot own this one.
  Remaining candidates, which this script separates:
    H1 DEGENERACY      pitching at floor/ceiling leaves the lateral direction weakly
                       observable  -> lateral drift tracks a DIP in hdh_eig_pos min.
    H2 CROSS-COUPLING  pitch rate leaking into the roll gyro -> false roll -> lateral
                       gravity projection  -> roll_rate regresses on pitch_rate with a
                       consistent nonzero slope.
    H3 IMPURE MOTION   the maneuver simply carries real roll/yaw -> drift tracks |roll_rate|
                       but WITHOUT a consistent pitch->roll slope.

FRAME HANDLING -- do not assume FLU. Raw /Odometry on this vehicle is
(+x=LEFT, +y=UP, +z=FORWARD) (config/fastlio_to_fc.launch:51). Rather than hardcode that,
the axes are derived from the data itself:
    up      = direction of specific force during the initial static hold
    pitch   = dominant horizontal gyro axis during the maneuvers (principal component)
    forward = up x pitch
"lateral" is then displacement ALONG the pitch axis -- the component a pitch cannot
produce. The detected frame is printed and cross-checked against the documented one.

Usage:  analyze_pitch_drift.py <raw_input_bag> <replay_output_bag> [--png] [--json]
        (run with ROS sourced, e.g. inside slam-hesai-fastlio)
"""
import sys, json, math
import numpy as np
import rosbag

GYRO_STATIC_MAX = 0.02   # rad/s -- below this the vehicle counts as still
STATIC_MIN_S    = 3.0    # shortest acceptable static hold
SMOOTH_S        = 0.30   # moving-average window for rate/velocity smoothing
MANEUVER_MIN_S  = 0.50   # ignore pitch blips shorter than this
MERGE_GAP_S     = 0.50   # join maneuver segments separated by less than this


# ----------------------------------------------------------------- loading

def load_imu(path):
    t, gyro, acc = [], [], []
    with rosbag.Bag(path) as b:
        for _, msg, _ in b.read_messages(topics=['/lidar_imu']):
            t.append(msg.header.stamp.to_sec())
            g, a = msg.angular_velocity, msg.linear_acceleration
            gyro.append((g.x, g.y, g.z))
            acc.append((a.x, a.y, a.z))
    return np.array(t), np.array(gyro, dtype=float), np.array(acc, dtype=float)


def load_replay(path):
    t, pos, quat, ht, health = [], [], [], [], []
    with rosbag.Bag(path) as b:
        for topic, msg, _ in b.read_messages(topics=['/Odometry', '/fastlio_health']):
            if topic == '/Odometry':
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                t.append(msg.header.stamp.to_sec())
                pos.append((p.x, p.y, p.z))
                quat.append((q.x, q.y, q.z, q.w))
            else:
                health.append(list(msg.data))
    # /fastlio_health carries no header stamp; it is published once per EKF update,
    # in lockstep with /Odometry, so borrow the odometry timebase.
    n = min(len(health), len(t))
    ht = np.array(t[:n]) if n else np.array([])
    return (np.array(t), np.array(pos, dtype=float), np.array(quat, dtype=float),
            ht, np.array(health[:n], dtype=float) if n else np.zeros((0, 7)))


# ----------------------------------------------------------------- helpers

def unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else v


def smooth(x, t, win_s):
    """Moving average over a time window, tolerant of irregular sampling."""
    if len(x) < 3:
        return x
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.01
    k = max(1, int(round(win_s / max(dt, 1e-6))))
    if k < 2:
        return x
    kern = np.ones(k) / k
    return np.convolve(x, kern, mode='same')


def spearman(a, b):
    """Rank correlation, no scipy dependency."""
    if len(a) < 4 or len(a) != len(b):
        return float('nan')
    if np.std(a) < 1e-12 or np.std(b) < 1e-12:
        return float('nan')
    ra = np.argsort(np.argsort(a)).astype(float)
    rb = np.argsort(np.argsort(b)).astype(float)
    return float(np.corrcoef(ra, rb)[0, 1])


def lstsq_slope(x, y):
    """y = m*x + c ; returns (m, r2)."""
    if len(x) < 4 or np.std(x) < 1e-9:
        return float('nan'), float('nan')
    A = np.vstack([x, np.ones(len(x))]).T
    (m, c), *_ = np.linalg.lstsq(A, y, rcond=None)
    resid = y - (m * x + c)
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    r2 = 1.0 - float(np.sum(resid ** 2)) / ss_tot if ss_tot > 1e-12 else float('nan')
    return float(m), float(r2)


def find_static(t, gyro, from_end=False):
    """Longest run of near-zero gyro at the start (or end) of the bag."""
    mag = np.linalg.norm(gyro, axis=1)
    still = mag < GYRO_STATIC_MAX
    idx = range(len(still) - 1, -1, -1) if from_end else range(len(still))
    run = []
    for i in idx:
        if still[i]:
            run.append(i)
        elif run:
            break
    if not run:
        return None
    run = sorted(run)
    if t[run[-1]] - t[run[0]] < STATIC_MIN_S:
        return None
    return run[0], run[-1]


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def axis_name(v):
    """Closest signed cardinal axis, for the frame-convention cross-check."""
    i = int(np.argmax(np.abs(v)))
    return f"{'+' if v[i] > 0 else '-'}{'xyz'[i]}"


# ----------------------------------------------------------------- frame

def derive_frame(t_i, gyro, acc):
    st = find_static(t_i, gyro)
    if st is None:
        print("  WARNING: no static hold >= %.0fs found at the start of the bag."
              % STATIC_MIN_S)
        print("           Falling back to the first 2 s for the gravity reference;")
        print("           the up axis (and therefore 'lateral') may be off.")
        sel = t_i <= t_i[0] + 2.0
    else:
        sel = np.zeros(len(t_i), dtype=bool)
        sel[st[0]:st[1] + 1] = True
        print("  static hold: %.1f s at bag start" % (t_i[st[1]] - t_i[st[0]]))

    g_meas = acc[sel].mean(axis=0)
    up = unit(g_meas)

    # Horizontal basis, then the dominant horizontal gyro direction = pitch axis.
    seed = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(seed, up)) > 0.9:
        seed = np.array([0.0, 0.0, 1.0])
    e1 = unit(seed - np.dot(seed, up) * up)
    e2 = unit(np.cross(up, e1))

    moving = np.linalg.norm(gyro, axis=1) > 5 * GYRO_STATIC_MAX
    if moving.sum() < 10:
        raise SystemExit("ERROR: almost no rotation in this bag -- nothing to analyze.")
    gh = np.vstack([gyro[moving] @ e1, gyro[moving] @ e2]).T
    cov = np.cov(gh.T)
    evals, evecs = np.linalg.eigh(cov)
    pv = evecs[:, int(np.argmax(evals))]
    pca_ax = unit(pv[0] * e1 + pv[1] * e2)

    # CRITICAL: do NOT adopt the PCA direction as the pitch axis. It aligns with the
    # TOTAL measured rotation, which already contains any pitch->roll cross-axis error,
    # so using it would rotate the frame to absorb exactly the error H2 is looking for
    # and roll_rate would come out ~0 by construction (verified: injecting 5% coupling
    # into a synthetic bag recovered a slope of 0.0000 under the PCA definition).
    # Instead use PCA only to IDENTIFY which body axis is the pitch axis, then snap to
    # that exact axis (de-tilted into the horizontal plane). Off-axis rotation then
    # survives as a measurable roll_rate.
    # Sign is pinned to the POSITIVE body axis, never to the measured rotation
    # direction: a symmetric back-and-forth pitch has a mean projection of ~0, so
    # keying the sign off the gyro lets noise flip it between runs (observed: +x on
    # one numpy, -x on another, same input). Pinning it keeps "+lateral" meaning the
    # same physical direction in every bag.
    i = int(np.argmax(np.abs(pca_ax)))
    card = np.zeros(3)
    card[i] = 1.0
    pitch_ax = unit(card - np.dot(card, up) * up)
    # cross(pitch, up), NOT cross(up, pitch): the documented raw frame
    # (+x=LEFT, +y=UP, +z=FORWARD) is right-handed with x x y = z, so this order makes
    # the derived "forward" actually point forward. The reversed order yields -z, which
    # analyses identically but prints a forward axis pointing backwards.
    fwd = unit(np.cross(pitch_ax, up))

    # Angle between measured rotation direction and the body axis -- this IS the
    # candidate cross-coupling, reported rather than absorbed.
    offaxis_deg = math.degrees(math.acos(max(-1.0, min(1.0, abs(np.dot(pca_ax, pitch_ax))))))
    dominance = float(max(evals) / max(min(evals), 1e-12))
    return up, pitch_ax, fwd, g_meas, dominance, offaxis_deg


# ----------------------------------------------------------------- main

def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    want_png = '--png' in sys.argv
    want_json = '--json' in sys.argv
    if len(args) != 2:
        print(__doc__)
        sys.exit(2)
    raw_bag, replay_bag = args

    print("=" * 78)
    print("PITCH -> LATERAL DRIFT ANALYSIS")
    print("=" * 78)
    print("  raw bag    : %s" % raw_bag)
    print("  replay bag : %s" % replay_bag)

    t_i, gyro, acc = load_imu(raw_bag)
    t_o, pos, quat, t_h, health = load_replay(replay_bag)
    if len(t_i) < 10:
        sys.exit("ERROR: no /lidar_imu in %s" % raw_bag)
    if len(t_o) < 10:
        sys.exit("ERROR: no /Odometry in %s -- replay it through run_one.sh first"
                 % replay_bag)
    print("  /lidar_imu %d msgs (%.1f s)   /Odometry %d msgs (%.1f s)   /fastlio_health %d"
          % (len(t_i), t_i[-1] - t_i[0], len(t_o), t_o[-1] - t_o[0], len(health)))

    # ---- frame -------------------------------------------------------------
    print("\n--- DERIVED FRAME (from data, not assumed) ---")
    up, pitch_ax, fwd, g_meas, dominance, offaxis_deg = derive_frame(t_i, gyro, acc)
    print("  |g| during static  : %.3f m/s^2   (expect ~9.81)" % np.linalg.norm(g_meas))
    print("  up      = [%+.3f %+.3f %+.3f]  ~ %s" % (*up, axis_name(up)))
    print("  pitch   = [%+.3f %+.3f %+.3f]  ~ %s   (lateral is ALONG this)"
          % (*pitch_ax, axis_name(pitch_ax)))
    print("  forward = [%+.3f %+.3f %+.3f]  ~ %s" % (*fwd, axis_name(fwd)))
    print("  horizontal gyro dominance (pitch vs other axis): %.1fx" % dominance)
    print("  measured rotation is %.2f deg OFF the body pitch axis" % offaxis_deg)
    print("    (deliberately NOT absorbed into the frame -- it is the H2 signal)")
    if dominance < 3.0:
        print("  WARNING: rotation is not strongly dominated by one horizontal axis.")
        print("           The maneuver was not isolated pitch -- H3 is likely, and the")
        print("           'lateral' axis below is a weaker separation of the motion.")
    # Cross-check against the documented convention (+x=LEFT,+y=UP,+z=FORWARD).
    if axis_name(up) == '+y' and axis_name(fwd) in ('+z', '-z'):
        print("  -> consistent with the documented raw /Odometry frame "
              "(+x=LEFT,+y=UP,+z=FORWARD)")
    else:
        print("  -> NOTE: differs from the documented raw frame "
              "(+x=LEFT,+y=UP,+z=FORWARD). Using the derived one.")

    # ---- projections -------------------------------------------------------
    pitch_rate = gyro @ pitch_ax
    roll_rate  = gyro @ fwd
    yaw_rate   = gyro @ up
    pr_s = smooth(pitch_rate, t_i, SMOOTH_S)

    p0 = pos[0]
    lat = (pos - p0) @ pitch_ax
    fw  = (pos - p0) @ fwd
    vert = (pos - p0) @ up

    # ---- maneuver segmentation --------------------------------------------
    thr = max(0.15, 0.25 * float(np.percentile(np.abs(pr_s), 95)))
    active = np.abs(pr_s) > thr
    segs = []
    i = 0
    while i < len(active):
        if active[i]:
            j = i
            while j + 1 < len(active) and active[j + 1]:
                j += 1
            segs.append([i, j])
            i = j + 1
        else:
            i += 1
    merged = []
    for s in segs:
        if merged and t_i[s[0]] - t_i[merged[-1][1]] < MERGE_GAP_S:
            merged[-1][1] = s[1]
        else:
            merged.append(s)
    segs = [s for s in merged if t_i[s[1]] - t_i[s[0]] >= MANEUVER_MIN_S]

    print("\n--- PITCH MANEUVERS (threshold %.2f rad/s) ---" % thr)
    if not segs:
        sys.exit("ERROR: no pitch maneuvers detected. Was pitch actually exercised?")

    def interp(tq, tsrc, v):
        return np.interp(tq, tsrc, v) if len(tsrc) else np.full_like(tq, np.nan)

    print("  %-3s %-8s %-7s %-9s %-10s %-10s %-10s" %
          ("#", "t0(s)", "dur", "pitch(deg)", "LAT(mm)", "FWD(mm)", "UP(mm)"))
    cycles = []
    t_rel = t_i - t_i[0]
    for n, (a, b) in enumerate(segs, 1):
        ta, tb = t_i[a], t_i[b]
        amp = math.degrees(abs(np.trapz(pitch_rate[a:b + 1], t_i[a:b + 1])))
        la = interp(np.array([ta, tb]), t_o, lat)
        fa = interp(np.array([ta, tb]), t_o, fw)
        va = interp(np.array([ta, tb]), t_o, vert)
        d_lat, d_fwd, d_up = la[1] - la[0], fa[1] - fa[0], va[1] - va[0]
        cycles.append(dict(n=n, t0=float(ta - t_i[0]), dur=float(tb - ta), amp_deg=amp,
                           d_lat=float(d_lat), d_fwd=float(d_fwd), d_up=float(d_up)))
        print("  %-3d %-8.1f %-7.1f %-9.1f %+10.1f %+10.1f %+10.1f" %
              (n, ta - t_i[0], tb - ta, amp, d_lat * 1e3, d_fwd * 1e3, d_up * 1e3))

    lat_steps = np.array([c['d_lat'] for c in cycles])
    up_steps  = np.array([c['d_up'] for c in cycles])
    same_sign = float(np.mean(np.sign(lat_steps) == np.sign(lat_steps[0]))) if len(lat_steps) else 0.0
    print("\n  per-cycle lateral step: mean %+.1f mm  sd %.1f mm  |sum| %.1f mm"
          % (lat_steps.mean() * 1e3, lat_steps.std() * 1e3, abs(lat_steps.sum()) * 1e3))
    print("  same-sign fraction    : %.0f%%   (high => RATCHET, accumulates per cycle;"
          " ~50%% => random walk)" % (100 * same_sign))
    print("  per-cycle vertical    : mean %+.1f mm  sd %.1f mm   (was the altitude artifact;"
          " expect small now)" % (up_steps.mean() * 1e3, up_steps.std() * 1e3))

    # ---- recovery between cycles -------------------------------------------
    rec = []
    for k in range(len(segs) - 1):
        tb = t_i[segs[k][1]]
        tn = t_i[segs[k + 1][0]]
        if tn - tb < 0.4:
            continue
        l0, l1 = interp(np.array([tb, tn]), t_o, lat)
        rec.append(l1 - l0)
    if rec:
        rec = np.array(rec)
        print("  drift during PAUSES   : mean %+.1f mm  max %+.1f mm  (large => not"
              " pitch-specific)" % (rec.mean() * 1e3, rec[np.argmax(np.abs(rec))] * 1e3))

    # ---- closure -----------------------------------------------------------
    st_end = find_static(t_i, gyro, from_end=True)
    if st_end is not None:
        ts, te = t_i[st_end[0]], t_i[st_end[1]]
        m = (t_o >= ts) & (t_o <= te)
        s0 = (t_o >= t_i[0]) & (t_o <= t_i[0] + 5.0)
        if m.sum() > 3 and s0.sum() > 3:
            c_lat = lat[m].mean() - lat[s0].mean()
            c_fwd = fw[m].mean() - fw[s0].mean()
            c_up = vert[m].mean() - vert[s0].mean()
            print("\n--- CLOSURE (final static hold vs initial) ---")
            print("  lateral %+.1f mm | forward %+.1f mm | vertical %+.1f mm | total %.1f mm"
                  % (c_lat * 1e3, c_fwd * 1e3, c_up * 1e3,
                     math.sqrt(c_lat ** 2 + c_fwd ** 2 + c_up ** 2) * 1e3))
            print("  (only meaningful if you physically returned to the start pose)")
    else:
        print("\n--- CLOSURE: no static hold found at the end of the bag; skipped ---")

    # ---- H1: degeneracy ----------------------------------------------------
    print("\n--- H1 DEGENERACY (lateral drift vs map observability) ---")
    eig_corr = float('nan')
    dt_o = np.gradient(t_o)
    lat_vel = np.abs(np.gradient(lat) / np.maximum(dt_o, 1e-3))
    lat_vel = smooth(lat_vel, t_o, SMOOTH_S)
    if len(health) and health.shape[1] >= 7:
        eig_min = health[:, 4]
        matched = health[:, 0]
        resid = health[:, 1]
        e_i = interp(t_o, t_h, eig_min)
        m_i = interp(t_o, t_h, matched)
        r_i = interp(t_o, t_h, resid)
        in_man = np.zeros(len(t_o), dtype=bool)
        for a, b in segs:
            in_man |= (t_o >= t_i[a]) & (t_o <= t_i[b])
        print("  hdh_eig_pos min : overall mean %.0f | during pitch %.0f | outside %.0f"
              % (eig_min.mean(), e_i[in_man].mean() if in_man.any() else float('nan'),
                 e_i[~in_man].mean() if (~in_man).any() else float('nan')))
        print("  matched_pts     : overall mean %.0f | during pitch %.0f"
              % (matched.mean(), m_i[in_man].mean() if in_man.any() else float('nan')))
        print("  mean_residual(m): overall %.4f | during pitch %.4f"
              % (resid.mean(), r_i[in_man].mean() if in_man.any() else float('nan')))
        eig_corr = spearman(lat_vel, -e_i)   # negative: drift UP as eig goes DOWN
        print("  Spearman(|lateral vel|, -eig_min) = %+.2f   "
              "(>= +0.4 supports H1)" % eig_corr)
        if in_man.any() and (~in_man).any():
            ratio = e_i[in_man].mean() / max(e_i[~in_man].mean(), 1e-9)
            print("  eig_min ratio during/outside pitch = %.2f   (<0.7 => pitching itself"
                  " degrades observability)" % ratio)
    else:
        print("  /fastlio_health absent or <7 fields -- H1 cannot be tested.")
        print("  (run_one.sh records it; check the replay actually produced it)")

    # ---- H2/H3: cross-coupling vs impure motion ----------------------------
    print("\n--- H2 CROSS-COUPLING vs H3 IMPURE MOTION ---")
    man_mask = np.zeros(len(t_i), dtype=bool)
    for a, b in segs:
        man_mask[a:b + 1] = True
    m_slope, m_r2 = lstsq_slope(pitch_rate[man_mask], roll_rate[man_mask])
    y_slope, y_r2 = lstsq_slope(pitch_rate[man_mask], yaw_rate[man_mask])
    print("  roll_rate = %+.4f * pitch_rate   (R^2 %.3f)" % (m_slope, m_r2))
    print("  yaw_rate  = %+.4f * pitch_rate   (R^2 %.3f)" % (y_slope, y_r2))
    print("  |roll| during pitch: mean %.4f rad/s  p95 %.4f"
          % (np.abs(roll_rate[man_mask]).mean(),
             np.percentile(np.abs(roll_rate[man_mask]), 95)))
    false_roll = math.degrees(abs(m_slope)) * 0.0 + math.degrees(math.atan(abs(m_slope))) \
        if not math.isnan(m_slope) else float('nan')
    if not math.isnan(m_slope):
        # A constant slope tilts the gravity projection by ~atan(slope) at full pitch.
        lat_acc = 9.81 * math.sin(math.radians(false_roll))
        print("  => implied false roll %.2f deg at full pitch -> %.3f m/s^2 lateral accel"
              % (false_roll, lat_acc))
    rr_i = interp(t_o, t_i, np.abs(smooth(roll_rate, t_i, SMOOTH_S)))
    roll_corr = spearman(lat_vel, rr_i)
    print("  Spearman(|lateral vel|, |roll_rate|) = %+.2f" % roll_corr)

    # ---- verdict -----------------------------------------------------------
    print("\n" + "=" * 78)
    print("READING")
    print("=" * 78)
    verdicts = []
    if not math.isnan(eig_corr) and eig_corr >= 0.4:
        verdicts.append("H1 DEGENERACY supported: lateral drift tracks the observability dip. "
                        "Fix is geometric/tuning (keep structure in FoV, revisit blind/det_range), "
                        "not calibration.")
    if not math.isnan(m_r2) and m_r2 >= 0.3 and abs(m_slope) >= 0.01:
        verdicts.append("H2 CROSS-COUPLING supported: roll rate scales consistently with pitch "
                        "rate (slope %+.3f, R^2 %.2f). That is an IMU axis-misalignment/"
                        "cross-axis term, not something extrinsic_R or the time lag can absorb."
                        % (m_slope, m_r2))
    if (math.isnan(m_r2) or m_r2 < 0.3) and not math.isnan(roll_corr) and roll_corr >= 0.4:
        verdicts.append("H3 IMPURE MOTION supported: drift tracks roll rate but with no "
                        "consistent pitch->roll slope, i.e. the maneuver carried real roll. "
                        "Re-record with more isolated pitch before chasing this further.")
    if same_sign >= 0.8 and abs(lat_steps.mean()) > 0.005:
        verdicts.append("RATCHET: the lateral step keeps the same sign every cycle "
                        "(%.0f%%, mean %+.1f mm). A systematic term, not random walk."
                        % (100 * same_sign, lat_steps.mean() * 1e3))
    if not verdicts:
        verdicts.append("No hypothesis clears its threshold. Either the drift is not present "
                        "in /Odometry at all -- which would point downstream at the FC EKF "
                        "(VISO_DELAY_MS / EK3 weighting), compare against "
                        "/mavros/local_position/pose -- or the maneuvers were too few/small. "
                        "Check the per-cycle table above for magnitude.")
    for v in verdicts:
        print("  * " + v)
    print("\n  Reminder: extrinsic_T/extrinsic_R/time_offset are RULED OUT by kinematics for")
    print("  pure pitch (w x r has no component along the pitch axis). More LI-Init runs")
    print("  will not move this symptom.")

    if want_json:
        out = dict(frame=dict(up=up.tolist(), pitch=pitch_ax.tolist(), fwd=fwd.tolist(),
                              dominance=dominance),
                   cycles=cycles,
                   lat_step_mean_mm=float(lat_steps.mean() * 1e3),
                   lat_step_sd_mm=float(lat_steps.std() * 1e3),
                   same_sign_frac=same_sign,
                   eig_corr=eig_corr, roll_corr=roll_corr,
                   pitch_to_roll_slope=m_slope, pitch_to_roll_r2=m_r2)
        print("\nJSON " + json.dumps(out))

    if want_png:
        make_plots(t_i, t_o, t_rel, pitch_rate, roll_rate, yaw_rate, lat, fw, vert,
                   segs, health, t_h, man_mask, m_slope, replay_bag)


def make_plots(t_i, t_o, t_rel, pitch_rate, roll_rate, yaw_rate, lat, fw, vert,
               segs, health, t_h, man_mask, m_slope, replay_bag):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except Exception:
        print("\n(matplotlib not available -> skipping plots)")
        return
    t0 = t_i[0]
    fig, ax = plt.subplots(2, 2, figsize=(15, 9))

    a = ax[0][0]
    a.plot(t_o - t0, lat * 1e3, label='lateral (along pitch axis)', lw=1.8)
    a.plot(t_o - t0, fw * 1e3, label='forward', lw=0.9, alpha=0.7)
    a.plot(t_o - t0, vert * 1e3, label='vertical', lw=0.9, alpha=0.7)
    for s, e in segs:
        a.axvspan(t_i[s] - t0, t_i[e] - t0, color='orange', alpha=0.15)
    a.set_title('Displacement (shaded = pitch maneuver)')
    a.set_xlabel('t (s)'); a.set_ylabel('mm'); a.legend(); a.grid(alpha=0.3)

    a = ax[0][1]
    a.plot(t_rel, pitch_rate, label='pitch rate', lw=1.2)
    a.plot(t_rel, roll_rate, label='roll rate', lw=1.0, alpha=0.8)
    a.plot(t_rel, yaw_rate, label='yaw rate', lw=0.8, alpha=0.6)
    a.set_title('Body rates'); a.set_xlabel('t (s)'); a.set_ylabel('rad/s')
    a.legend(); a.grid(alpha=0.3)

    a = ax[1][0]
    if len(health) and health.shape[1] >= 7:
        a.plot(t_h - t0, health[:, 4], label='hdh_eig_pos min', lw=1.2)
        a.set_ylabel('eig_min')
        a2 = a.twinx()
        a2.plot(t_h - t0, health[:, 0], color='green', alpha=0.5, lw=0.9,
                label='matched_pts')
        a2.set_ylabel('matched_pts')
        for s, e in segs:
            a.axvspan(t_i[s] - t0, t_i[e] - t0, color='orange', alpha=0.15)
        a.set_title('Observability (dips during shaded pitch => H1)')
        a.legend(loc='upper left')
    else:
        a.text(0.5, 0.5, 'no /fastlio_health', ha='center')
    a.set_xlabel('t (s)'); a.grid(alpha=0.3)

    a = ax[1][1]
    a.scatter(pitch_rate[man_mask], roll_rate[man_mask], s=3, alpha=0.3)
    if not math.isnan(m_slope):
        xs = np.linspace(pitch_rate[man_mask].min(), pitch_rate[man_mask].max(), 10)
        a.plot(xs, m_slope * xs, 'r-', lw=1.5, label='slope %+.4f' % m_slope)
        a.legend()
    a.set_title('Pitch->roll coupling (tilted line => H2)')
    a.set_xlabel('pitch rate (rad/s)'); a.set_ylabel('roll rate (rad/s)'); a.grid(alpha=0.3)

    out = replay_bag.rsplit('.', 1)[0] + '_pitch_drift.png'
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print("\nplots -> %s" % out)


if __name__ == '__main__':
    main()
