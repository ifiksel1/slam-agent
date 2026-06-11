# Phase 7: Optimization & Tuning

After basic integration works, optimize for your environment and mission profile.

---

## SLAM Parameters

### Indoor / Small Spaces
```yaml
surroundingKeyframeSearchRadius: 25.0       # (default: 50) - reduce search radius
surroundingkeyframeAddingDistThreshold: 0.5  # (default: 1.0) - more frequent keyframes
odometrySurfLeafSize: 0.1                    # (default: 0.2) - higher resolution
mappingCornerLeafSize: 0.05                  # (default: 0.1)
```

### Outdoor / Large Spaces
```yaml
surroundingKeyframeSearchRadius: 100.0       # increase search radius
surroundingkeyframeAddingDistThreshold: 2.0  # less frequent keyframes
odometrySurfLeafSize: 0.4                    # lower resolution for speed
mappingCornerLeafSize: 0.2
```

---

## ArduPilot Flight Tuning

### Aggressive Flight
```
WPNAV_SPEED,300      # 3.0 m/s
WPNAV_ACCEL,500      # 5.0 m/s²
PSC_POSXY_P,2.0      # Higher P gain
```

### Smooth / Precision Flight
```
WPNAV_SPEED,150      # 1.5 m/s
WPNAV_ACCEL,200      # 2.0 m/s²
PSC_POSXY_P,1.0      # Lower P gain
```

---

## Resource Optimization

### High CPU Usage
```yaml
# SLAM config
downsampleRate: 2              # Skip every other scan
mappingProcessInterval: 0.2    # Slower mapping rate

# Reduce feature extraction
edgeFeatureMinValidNum: 10     # (default: 5) - filter weak edges
surfFeatureMinValidNum: 200    # (default: 100) - filter weak surfaces
```

### High RAM Usage
```yaml
# Reduce map size in memory
globalMapVisualizationSearchRadius: 50.0  # (default: 1000)
surroundingKeyframeSize: 25               # (default: 50)
```

---

---

## Benchmarking & Comparing LiDAR-Inertial Frameworks

When choosing or tuning between LIO stacks (EllipseLIO / SuperOdom / FAST-LIO), the
*methodology* determines whether your numbers mean anything. Hard-won rules:

### 1. Isolate compute before comparing — this is not optional
Run frameworks **strictly sequentially, one at a time, with every other LIO/mapping
node killed first.** They share the same `/ouster/*` topics and the Jetson's cores; a
co-running node both re-processes the replayed stream and steals CPU.
- A framework that looks **unstable in a shared-CPU benchmark may be perfectly stable
  when it owns the cores.** Real example: SuperOdom's "nondeterministic ±10 m transient"
  fired every run with a co-running node and **dropped to 0.23 m max-step the moment it
  had all 6 cores to itself.** The instability was the benchmark, not the algorithm.
- Before each run: `docker exec <other_ctr> pkill -9 -f '<their_node>'` and confirm 0 live
  nodes (zombies/`<defunct>` are fine — they hold no CPU).
- **Operational corollary for flight:** don't co-locate two heavy LIO nodes on one Jetson.

### 2. Validate the bag BEFORE trusting any result
A bag with a sensor blackout (simultaneous gap in BOTH `/ouster/points` AND `/ouster/imu`)
diverges *every* framework on resume — garbage in. Run `scripts/check_bag_gaps.py <bag>`
first; a `RE-RECORD` verdict means the numbers past the gap are meaningless.

### 3. Compare only FRAME-INVARIANT metrics
Different frameworks use different world conventions (SuperOdom = sensor-start orientation +
inverted yaw; FAST-LIO/EllipseLIO = gravity-aligned). **Raw x,y,z are NOT comparable.** Use:
- **peak excursion from start** (should agree across frameworks to <0.1 m on the same path),
- **path length** (inflated path with a normal endpoint = integrated jitter / rough trajectory),
- **loop closure** = final displacement from start (if you returned to origin),
- **max single-step** (>0.3 m ≈ a divergence/jump).

### 4. Boost the clock for compute-marginal frameworks — it's part of the config
EllipseLIO and SuperOdom are **marginal at the stock 1497 MHz** (EllipseLIO diverges ~50% of
runs at stock, 0% at boost). The boost to 1984 MHz costs only ~3 °C. Treat
`sudo ~/superodom_ws/set_cpu_clock.sh boost` as part of the validated runtime, not an
afterthought. (`nvpmodel -m 0`/MAXN is broken on the 6-core Orin NX 8GB — use the sysfs
per-core override.)

### Config failure modes that masquerade as "the algorithm diverged"
Before blaming a framework, check these — all caused catastrophic runaways this rig that were
pure config:
- **Wrong `lidar.rate` / `scan_rate`** (set to a dataset's Hz, not the live sensor's) → breaks
  IMU↔LiDAR association + deskew → runaway under motion.
- **Map/voxel resolution finer than the sparsest region's feature spacing** → feature
  starvation → runaway. Finer is not safer.
- **Missing `extrinsic_T` (lever-arm)** → end-of-run IMU runaway while stationary. And for
  FAST-LIO specifically: **`extrinsic_est_en: true` with a correct `extrinsic_T` is HARMFUL** —
  the free DOF drifts during feature-poor sections. Set the real lever-arm and turn online
  estimation OFF. (EllipseLIO has no such flag, so its extrinsic must be correct up front.)

---

## Output
Document final tuning parameters in your config files. Return to Phase 5 for re-testing with optimized settings.
