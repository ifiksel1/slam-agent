#!/usr/bin/env python3
"""
imu_allan_variance.py — Allan-variance IMU noise characterization from a static rosbag.

Self-contained (numpy + rosbag only; no ceres/catkin/allan_variance_ros build).
Computes the OVERLAPPING Allan deviation for each gyro/accel axis of a static
recording and extracts the four coefficients that LIO estimators consume:

  N  white-noise density  (VRW for accel, ARW for gyro)  -> acc_noise / gyr_noise
  K  bias random walk     (rate random walk coefficient)  -> acc_bias  / gyr_bias
  B  bias instability      (reported for reference)

Convention matches EllipseLIO / Kalibr:
  acc_noise = accelerometer_noise_density  [m/s^2/sqrt(Hz)]
  gyr_noise = gyroscope_noise_density      [rad/s/sqrt(Hz)]
  acc_bias  = accelerometer_random_walk    [m/s^3/sqrt(Hz)]  (bias RW)
  gyr_bias  = gyroscope_random_walk         [rad/s^2/sqrt(Hz)] (bias RW)

Usage:
  python3 imu_allan_variance.py <bag> [--topic /ouster/imu] [--out <dir>]
"""
import argparse, sys, os
import numpy as np

def load_imu(bag_paths, topic):
    import rosbag
    if isinstance(bag_paths, str):
        bag_paths = [bag_paths]
    t, gx, gy, gz, ax, ay, az = [], [], [], [], [], [], []
    for bag_path in bag_paths:
        with rosbag.Bag(bag_path) as bag:
            for _, msg, _ in bag.read_messages(topics=[topic]):
                # use the message header stamp (sensor time), not bag receive time
                t.append(msg.header.stamp.to_sec())
                gx.append(msg.angular_velocity.x); gy.append(msg.angular_velocity.y); gz.append(msg.angular_velocity.z)
                ax.append(msg.linear_acceleration.x); ay.append(msg.linear_acceleration.y); az.append(msg.linear_acceleration.z)
    if not t:
        sys.exit(f"ERROR: no messages on {topic} in {bag_paths}")
    t = np.asarray(t, float)
    g = np.column_stack([gx, gy, gz]).astype(float)
    a = np.column_stack([ax, ay, az]).astype(float)
    # segments may be read out of order — sort by sensor time and drop dup stamps
    order = np.argsort(t, kind="stable")
    t, g, a = t[order], g[order], a[order]
    keep = np.concatenate([[True], np.diff(t) > 0])
    return t[keep], g[keep], a[keep]

def overlapping_adev(x, fs, taus):
    """Overlapping Allan deviation of a *rate* series x (1D), sampled at fs Hz.
    Returns adev array aligned with taus (NaN where not enough data)."""
    x = np.asarray(x, float)
    N = x.size
    dt = 1.0 / fs
    theta = np.cumsum(x) * dt          # integrated angle/velocity
    theta = np.insert(theta, 0, 0.0)   # theta[0]=0, length N+1
    adev = np.full(taus.size, np.nan)
    for i, tau in enumerate(taus):
        m = int(round(tau * fs))
        if m < 1 or (N - 2 * m) < 1:
            continue
        # sigma^2 = 1/(2 tau^2 (N-2m)) * sum_k (theta[k+2m] - 2 theta[k+m] + theta[k])^2
        d = theta[2 * m:] - 2.0 * theta[m:-m] + theta[:-2 * m]
        s2 = np.sum(d * d) / (2.0 * (tau ** 2) * d.size)
        adev[i] = np.sqrt(s2)
    return adev

def fit_coeffs(taus, adev):
    """Extract N (tau=1, slope -1/2), K (tau=3 on slope +1/2), B (bias instability)."""
    good = np.isfinite(adev) & (adev > 0)
    tau, ad = taus[good], adev[good]
    if tau.size < 4:
        return dict(N=np.nan, K=np.nan, B=np.nan)
    logt, loga = np.log10(tau), np.log10(ad)
    slope = np.gradient(loga, logt)

    # N: white noise, sigma = N / sqrt(tau) -> read the -1/2 line at tau=1s.
    #    Fit the region where slope ~ -0.5, extrapolate N = sigma * sqrt(tau).
    mN = np.abs(slope + 0.5) < 0.2
    if mN.sum() >= 2:
        N = np.median(ad[mN] * np.sqrt(tau[mN]))
    else:  # fallback: value nearest tau=1s
        N = ad[np.argmin(np.abs(tau - 1.0))] * np.sqrt(tau[np.argmin(np.abs(tau - 1.0))])

    # K: rate random walk, sigma = K * sqrt(tau/3) -> read +1/2 line at tau=3s.
    mK = np.abs(slope - 0.5) < 0.25
    if mK.sum() >= 2:
        K = np.median(ad[mK] * np.sqrt(3.0 / tau[mK]))
    else:  # fallback: use the long-tau tail slope
        tail = tau > tau.max() / 3.0
        K = np.median(ad[tail] * np.sqrt(3.0 / tau[tail])) if tail.sum() else np.nan

    # B: bias instability = min(adev)/0.664
    B = ad.min() / 0.664
    return dict(N=float(N), K=float(K), B=float(B))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", nargs="+", help="one or more bag files (e.g. split segments)")
    ap.add_argument("--topic", default="/ouster/imu")
    ap.add_argument("--out", default=None)
    ap.add_argument("--npts", type=int, default=50, help="number of tau points")
    args = ap.parse_args()
    bags = sorted(args.bag)
    outdir = args.out or os.path.dirname(os.path.abspath(bags[0]))
    os.makedirs(outdir, exist_ok=True)

    print(f"[load] {len(bags)} bag(s) topic={args.topic}")
    t, gyro, acc = load_imu(bags, args.topic)
    dt = np.diff(t)
    dt = dt[(dt > 0) & (dt < 1.0)]
    fs = 1.0 / np.median(dt)
    dur = t[-1] - t[0]
    print(f"[data] {t.size} samples, {dur:.1f}s ({dur/3600:.2f}h), fs={fs:.2f} Hz")

    tau_max = dur / 9.0   # need >=9 clusters for a reliable estimate
    taus = np.logspace(np.log10(2.0 / fs), np.log10(tau_max), args.npts)

    axes = ["x", "y", "z"]
    results = {}
    curves = {"tau": taus}
    for name, data, kind in [("gyr", gyro, "gyro"), ("acc", acc, "accel")]:
        per_axis = []
        for a in range(3):
            ad = overlapping_adev(data[:, a], fs, taus)
            curves[f"{name}_{axes[a]}"] = ad
            c = fit_coeffs(taus, ad)
            per_axis.append(c)
            print(f"[{name}.{axes[a]}] N={c['N']:.3e}  K={c['K']:.3e}  B={c['B']:.3e}")
        # average across axes (Kalibr convention)
        results[name] = {k: float(np.nanmean([p[k] for p in per_axis])) for k in ("N", "K", "B")}
        results[name + "_per_axis"] = per_axis

    # map to EllipseLIO fields
    ell = {
        "acc_noise": results["acc"]["N"],
        "gyr_noise": results["gyr"]["N"],
        "acc_bias":  results["acc"]["K"],
        "gyr_bias":  results["gyr"]["K"],
    }

    # save curves CSV
    csv = os.path.join(outdir, "allan_curves.csv")
    hdr = ",".join(curves.keys())
    M = np.column_stack([curves[k] for k in curves.keys()])
    np.savetxt(csv, M, delimiter=",", header=hdr, comments="")
    print(f"[save] {csv}")

    # emit EllipseLIO-ready YAML block
    ycfg = os.path.join(outdir, "allan_ellipselio_imu.yaml")
    with open(ycfg, "w") as f:
        f.write("# Allan-variance calibration — Ouster OS1-64 built-in IMU (static)\n")
        f.write(f"# source: {len(bags)} bag(s), {os.path.basename(bags[0])} ...  ({dur/3600:.2f}h @ {fs:.1f}Hz, {t.size} samples)\n")
        f.write("# fields map 1:1 into ellipselio config imu: block\n")
        f.write("imu:\n")
        f.write(f"    acc_noise: {ell['acc_noise']:.6g}   # accel noise density  [m/s^2/sqrt(Hz)]\n")
        f.write(f"    gyr_noise: {ell['gyr_noise']:.6g}   # gyro noise density   [rad/s/sqrt(Hz)]\n")
        f.write(f"    acc_bias:  {ell['acc_bias']:.6g}   # accel bias rand walk [m/s^3/sqrt(Hz)]\n")
        f.write(f"    gyr_bias:  {ell['gyr_bias']:.6g}   # gyro bias rand walk  [rad/s^2/sqrt(Hz)]\n")
    print(f"[save] {ycfg}")

    print("\n===== EllipseLIO IMU params (Allan-variance) =====")
    for k, v in ell.items():
        print(f"  {k}: {v:.6g}")
    print("  (bias instability B: acc={:.3e} m/s^2, gyr={:.3e} rad/s)".format(
        results["acc"]["B"], results["gyr"]["B"]))

if __name__ == "__main__":
    main()
