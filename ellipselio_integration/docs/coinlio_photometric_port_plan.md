# Port plan: COIN-LIO photometric residual → EllipseLIO

**Goal.** Add COIN-LIO's LiDAR-intensity *direct photometric* residual as a second, stacked
residual channel inside EllipseLIO's existing IEKF update, so geometry-degenerate viewpoints
(blank fuselage / nadir-over-wing in the 737 hangar) are constrained by reflectivity texture the
geometric residual can't see — while keeping EllipseLIO's `obs_score` degeneracy signal and
adaptive-ellipsoid geometric residual intact.

**Why this is a port, not a rewrite (verified in source 2026-06-12).**
- Identical state manifold: both `MTK_BUILD_MANIFOLD(state_ikfom, …)` = stock FAST-LIO2
  (pos,rot,offset_R_L_I,offset_T_L_I,vel,bg,ba,grav), `esekf<state_ikfom,12,input_ikfom>`.
  EllipseLIO `include/ikfom/use_ikfom.h` ≡ COIN-LIO `include/use_ikfom.h` (cosmetic diffs only).
- Same IKFoM `esekfom.hpp` toolkit; same callback type
  `void(state_ikfom&, esekfom::dyn_share_datastruct<double>&)`.
  EllipseLIO's `dyn_share_datastruct` is a **superset** (adds `finish`, `h_x_R`).
- COIN-LIO's photometric residual already fills **only h_x cols 0–5 (pos,rot)** — exactly
  EllipseLIO's 6-column `h_x` convention (EllipseLIO doesn't estimate extrinsics in the LiDAR
  update). No column remapping.
- Photometric source is the **Ouster reflectivity range-image**, not a camera → no camera, no
  cam-LiDAR extrinsic, works in any lighting, single OS1-64. EllipseLIO's existing
  `cam_processing` is camera-RGB *colorization only* and is **not** reusable here.
- EllipseLIO map point `PointXYZNRGBIT` already stores per-point `intensity` (common_pcl.h:33);
  but COIN-LIO matching uses a per-feature reference image, not the map, so this is a bonus not a
  dependency.

Estimated effort: **~1–2 weeks** for someone fluent in the FAST-LIO2/IKFoM codebase. The labor is
mechanical (port the Ouster intensity-image front-end, ROS1→ROS2); the only design decision with
teeth is the geo/photo weighting in `h_x_R` (§4).

---

## 1. File-by-file: which COIN-LIO sources land where

Source root: `/home/dev/coinlio_ws/src/coin-lio` → dest root: `/home/dev/ellipselio_ws/src/ellipselio`

| COIN-LIO source | → EllipseLIO destination | Action | ROS1→ROS2 / notes |
|---|---|---|---|
| `include/projector.h`, `src/projector.cpp` (281 L) | `include/photometric/projector.h`, `src/photometric/projector.cpp` | **Port near-verbatim** | Ouster spherical model + `projectionJacobian`. ROS-agnostic except param loading: replace `nh.param(...)` reads of `/data_format/*`, `beam_altitude_angles`, `pixel_shift_by_row`, `lidar_origin_to_beam_origin_mm` with EllipseLIO's `declare_parameter`/`get_parameter` or pass them in from the Ouster metadata already parsed in `lidar_processing`. |
| `src/image_processing.cpp`, `include/image_processing.h` (166 L) | `src/photometric/image_processing.cpp`, `include/photometric/image_processing.h` | **Port verbatim** | Pure OpenCV (line removal, adaptive brightness, blur, clamp). All online — **no calibration files**. Params from `config/line_removal.yaml` → fold into EllipseLIO config. |
| `src/feature_manager.cpp`, `include/feature_manager.h` (493 L) | `src/photometric/feature_manager.cpp`, `include/photometric/feature_manager.h` | **Port verbatim** | DSO-style feature detect/track lifecycle. ROS-agnostic. Holds `std::vector<Feature>` consumed by the residual. |
| `Feature` struct (feature_manager.h:23) | (comes with feature_manager.h) | — | `{life_time; center; intensities[]; p[](global-frame 3D); uv[]}`. |
| `LidarFrame` struct (common_lib.h:161) | `include/photometric/lidar_frame.h` (new) | **Extract** | Fields the residual needs: `img_intensity`(CV_32FC1), `img_mask`, `img_dx/dy`, `T_Li_Lk_vec`(per-step undistort), `vec_idx`, `proj_idx`, `img_idx`. Don't pull all of COIN's `common_lib.h` — extract just this struct to avoid clashing with EllipseLIO's `common_lib.h`. |
| `getSubPixelValue<T>()` (common_lib.h:286) | `include/photometric/photometric_util.h` (new) | **Copy** | Bilinear sampler used by the residual. |
| `h_share_model_photometric` body (laserMapping.cpp:664–801, 137 L) | **Inlined into** `MappingNode::TensorRegistration` (`src/map_processing.cpp`) as a helper `PhotometricResidual(s, …)` | **Adapt** | See §3 for the exact insertion. This is the only piece that is *adapted*, not copied. |
| `h_share_combined` stacking (laserMapping.cpp:803–840) | — | **Do NOT port** | EllipseLIO already assembles `h`/`h_x`/`h_x_R` dynamically; we append rows in-place instead of running two sub-functions and concatenating. |
| `calibration.cpp` | — | **Skip** | One-time `u_shift` integer tool, not radiometric calibration. Run it once offline against an OS1-64 bag to get `image/u_shift`; bake the int into config. |
| `src/imu_processing.cpp` (COIN's) | — | **Do NOT port** | EllipseLIO has its own IMU undistortion. We must instead *emit* `T_Li_Lk_vec`/`vec_idx` from EllipseLIO's `imu_processing.cpp::UndistortPointCloud` — see §2. |
| `vision_bridge.cpp`, `loop_closure.h`, `preprocess.cpp`, `ikd-Tree/` | — | **Skip** | Not part of the photometric residual. |

New EllipseLIO subtree: `src/photometric/` + `include/photometric/`. Keeps the lift isolated and
makes it toggleable (`enable_photometric` param → if false, `TensorRegistration` is byte-identical
to today).

---

## 2. Pipeline hooks (in `MappingNode::TimerCallback`, map_processing.cpp:1347)

The geometric path today is: `SyncPackages → UndistortPointCloud → UpdateStatesWithLidar(→IEKF→TensorRegistration) → MapIncremental`.

Add (only when `enable_photometric_`):

1. **In `imu_processing.cpp::UndistortPointCloud`:** while integrating IMU motion per sub-interval,
   record the per-step LiDAR→scan-end transform into a new output `current_frame_.T_Li_Lk_vec`
   and per-point `vec_idx`. EllipseLIO already computes this motion to deskew; we're persisting it.
   (COIN-LIO Jacobian step 3 needs `T_Li_Lk` per point — laserMapping.cpp:761.)
2. **After undistort, BEFORE `UpdateStatesWithLidar`:** call
   `image_processor_->createImages(current_frame_)` then
   `projector_->createImages(current_frame_)` to build `img_intensity`, `img_mask`, `img_dx/dy`,
   `proj_idx`. (COIN order: laserMapping.cpp:1102.)
3. **After the IEKF update:** call `feature_manager_->updateFeatures(current_frame_, kf_state_)`
   to detect/track features for the *next* frame. (COIN: laserMapping.cpp:1232 — features consumed
   by the next residual call, initialized from the live processed image.)

Members to add to `MappingNode`: `std::shared_ptr<Projector> projector_`,
`std::shared_ptr<ImageProcessor> image_processor_`,
`std::shared_ptr<FeatureManager> feature_manager_`, `LidarFrame current_frame_`,
`double photo_scale_`, `bool enable_photometric_`. Init in ctor / `Init*` from params.

---

## 3. THE EXACT INSERTION POINT in `TensorRegistration` (map_processing.cpp)

Today the geometric residual fills per-point working buffers in the `#pragma omp parallel for`
loop (lines 731–834), counts `feat_tot = feat_cnt.load()` (837), range-normalizes weights, applies
`pow(obs_min)` (882), and assembles the final matrices at **883–906**:

```cpp
// CURRENT (geometric only) — map_processing.cpp:883-906
ekfom_data_h_.block(0, 0, feat_tot, 1)   = ekfom_data_h_v_.head(feat_tot);
ekfom_data_w_x_.block(0, 0, feat_tot, 1) = ekfom_data_w_.head(feat_tot);
ekfom_data_h_x_.block(0, 0, feat_tot, 6) = ekfom_data_h_x_v_.topRows(feat_tot);
...
ekfom_data_h_x_r_.leftCols(feat_tot) =
    (ekfom_data_h_x_.topRows(feat_tot).array().colwise()
     * ekfom_data_w_x_.head(feat_tot)).transpose();
ekfom_data.h     = ekfom_data_h_.head(feat_tot);
ekfom_data.h_x   = ekfom_data_h_x_.topRows(feat_tot);
ekfom_data.h_x_R = ekfom_data_h_x_r_.leftCols(feat_tot);
```

**Insertion: between line 886 (geometric block fill) and the `h_x_R` build at 898.** Append the
photometric rows to the SAME working buffers starting at row `feat_tot`, then let the existing
`h_x_R` construction run over `feat_tot + n_photo` rows. Concretely:

```cpp
// ---- existing geometric fill (883-886) leaves rows [0, feat_tot) populated ----
ekfom_data_h_.block(0, 0, feat_tot, 1)   = ekfom_data_h_v_.head(feat_tot);
ekfom_data_w_x_.block(0, 0, feat_tot, 1) = ekfom_data_w_.head(feat_tot);
ekfom_data_h_x_.block(0, 0, feat_tot, 6) = ekfom_data_h_x_v_.topRows(feat_tot);

int n_tot = feat_tot;
// ======================= NEW: photometric rows [feat_tot, feat_tot+n_photo) =======================
if (enable_photometric_ && feature_manager_->features().size()) {
  const cv::Mat& img = current_frame_.img_intensity;
  int row = feat_tot;
  // (single-threaded first; parallelize after correctness — features() is shared)
  for (size_t j = 0; j < feature_manager_->features().size(); ++j) {
    const Feature& f = feature_manager_->features()[j];
    for (size_t l = 0; l < f.p.size(); ++l) {
      // ---- lifted from COIN-LIO h_share_model_photometric (laserMapping.cpp:746-776) ----
      V3D p_feat_G  = f.p[l];                                  // global-frame 3D
      V3D p_feat_I  = s.rot.conjugate() * (p_feat_G - s.pos);  // → IMU frame (R_IG·(p-t))
      V3D p_feat_Li = s.offset_R_L_I.conjugate()*(p_feat_I - s.offset_T_L_I); // → LiDAR frame
      V2D uv; if (!projector_->projectPoint(p_feat_Li, uv)) continue;
      if (!InImage(uv, img, kPatchBorder)) continue;

      double z_pho = getSubPixelValue<float>(img, uv(0), uv(1)) - f.intensities[l];

      Eigen::Matrix<double,1,2> dI_du;                         // image gradient (central diff)
      dI_du << 0.5*(getSubPixelValue<float>(img,uv(0)+1,uv(1)) - getSubPixelValue<float>(img,uv(0)-1,uv(1))),
               0.5*(getSubPixelValue<float>(img,uv(0),uv(1)+1) - getSubPixelValue<float>(img,uv(0),uv(1)-1));
      Eigen::Matrix<double,2,3> du_dp;  projector_->projectionJacobian(p_feat_Li, du_dp);
      M3D R_Li_I = (s.offset_R_L_I.conjugate()).toRotationMatrix();   // ∂p_Li/∂p_I
      M3D p_feat_I_x; p_feat_I_x << SKEW_SYM_MATRIX(p_feat_I);
      Eigen::Matrix<double,3,6> dp_dtf;
      dp_dtf << R_Li_I*s.rot.conjugate().toRotationMatrix(), -R_Li_I*p_feat_I_x;  // ∂p_Li/∂(pos,rot)
      Eigen::Matrix<double,1,6> h_x_row = dI_du * du_dp * dp_dtf;   // chain rule, cols 0-5

      // --- write into the SAME working buffers, at row `row` ---
      ekfom_data_h_.block(row,0,1,1)(0,0) = -z_pho;        // sign-convention: match geo (-residual)
      ekfom_data_h_x_.block(row,0,1,6)    = h_x_row;
      ekfom_data_w_x_(row)                = PhotoWeight(obs_min, dI_du.norm()); // §4
      ++row;
    }
  }
  n_tot = row;
}
// ================================================================================================

// ---- existing h_x_R build, now over n_tot rows (was feat_tot) ----
ekfom_data_h_x_r_.leftCols(n_tot) =
    (ekfom_data_h_x_.topRows(n_tot).array().colwise()
     * ekfom_data_w_x_.head(n_tot)).transpose();

ekfom_data.h     = ekfom_data_h_.head(n_tot);
ekfom_data.h_x   = ekfom_data_h_x_.topRows(n_tot);
ekfom_data.h_x_R = ekfom_data_h_x_r_.leftCols(n_tot);
```

Notes:
- **Sign convention:** geometric stores `-residual` (line 832). Photometric must match
  (`-z_pho`) so the IEKF innovation sign is consistent.
- **`obs_min` is already final** at the insertion point (computed 879–881) — so the photometric
  weight can be modulated by the *current* degeneracy estimate (§4). This is why we insert AFTER
  the obs computation, not inside the parallel loop.
- **Buffer sizing:** `ekfom_data_h_`, `ekfom_data_h_x_`, `ekfom_data_h_x_r_`, `ekfom_data_w_x_`
  are pre-allocated to max scan size. Grow their allocation to
  `max_scan_pts + max_features*patch_area` (one-time, in the ctor where they're sized).
- **Iteration count:** features' reference intensities are fixed per outer frame; re-projecting
  each IEKF iteration with the updated `s` is correct (the residual shrinks as pose converges),
  exactly as COIN-LIO does.
- Keep photometric single-threaded for v1 (shared `features()`); parallelize with a reduction once
  numerically validated.

---

## 4. THE `h_x_R` WEIGHTING CHANGE (the one design decision with teeth)

EllipseLIO's information-form gain uses `HTH = h_x_R * h_x` with per-point scalar weights baked
into `h_x_R` (= `(h_x.colwise * w_x)^T`), and a single scalar measurement noise
`R = kLidarPointCovariance`. So **a row's weight `w_x(i)` is its inverse-variance.** Geometric
weights are `time_recency · gravity_alignment`, range-normalized, then `^obs_min` (line 882).

The photometric rows must get a commensurate `w_x`. `PhotoWeight(obs_min, |∇I|)`:

**v1 — constant (ship this first).** Reproduce COIN-LIO's flat `photo_scale` (default 0.002) as a
variance ratio. Because EllipseLIO weights are inverse-variance and geometric weights are O(1)
after normalization, set:
```cpp
double PhotoWeight(double /*obs_min*/, double /*grad*/) { return photo_scale_; } // ~1e-2..1e-3, tune
```
Pick `photo_scale_` so median photometric `w_x` ≈ `photo_gain ×` median geometric `w_x`; expose
`photo_gain` in config. This already gets the benefit and is trivially A/B-toggled.

**v2 — `obs_score`-modulated (the real prize, do after v1 validates).** Make photometric *take
over exactly where geometry is degenerate*. `obs_min∈[0.1,1]` is high when well-constrained, low
when degenerate. Up-weight photometric as geometry degrades, and scale by gradient strength
(texture confidence):
```cpp
double PhotoWeight(double obs_min, double grad) {
  double deg = (1.0 - obs_min) / (1.0 - 0.1);          // 0 healthy → 1 fully degenerate
  double w   = photo_scale_ * (1.0 + photo_deg_gain_ * deg); // lean in when degenerate
  w *= std::min(grad / grad_ref_, 1.0);                 // trust only textured pixels
  return w;
}
```
This is the synergy that motivates the whole port: blank-skin/nadir → geometric `obs_min` collapses
→ photometric weight rises → reflectivity texture (rivets, panel lines, markings) pins the in-plane
DOF the planes can't. **Caveat:** `obs_min` is a *global* scalar here, not per-axis — v2 lifts the
whole photometric channel, not just the degenerate axis. A per-axis version (project the
photometric Jacobian onto the small-eigenvalue subspace of geometric `HᵀH`) is a v3 research step,
not required to ship.

Everything else in the gain path is unchanged: the existing `h_x_R = (h_x.colwise * w_x)^T` line
(now over `n_tot`) already produces the correct stacked information matrix `Σ wᵢ hᵢᵀhᵢ`.

---

## 5. Config, build, params

- **Config:** add an `photometric:` block to `config/os1_64_ouster.yaml`:
  `enable`, `photo_scale`/`photo_gain`, `photo_deg_gain`, `patch_size`, `n_features`,
  `min_range`/`max_range`, `intensity_scale`, `reflectivity`(bool), `window`, `highpass`/`lowpass`
  (from COIN's `line_removal.yaml`), `u_shift` (from the one-time calibration tool).
- **CMake:** add `src/photometric/*.cpp` to the component lib; `find_package(OpenCV REQUIRED)`
  (already pulled for `cam_processing`); link `${OpenCV_LIBS}`. **Honor the OpenCV 4.2/4.5 ABI
  fix** (LD_PRELOAD + link order — see `opencv_abi_conflict.md`) since this adds more cv:: surface.
- **package.xml:** no new ROS deps (OpenCV/Eigen/PCL already present); FeatureManager/Projector are
  not ROS nodes.

---

## 6. Risks, gates, sequencing

1. **GATE 0 (do before any code):** record an OS1-64 bag with (a) a slow pass parallel to flat,
   rivet/marking-textured skin and (b) a nadir-over-wing pass. Dump the reflectivity range-image
   (`projector` standalone) and confirm visible in-plane contrast where geometry is flat. **If the
   reflectivity image is contrast-less on real skin at your standoffs, STOP — no port is worth it.**
2. **Port front-end (Projector+ImageProcessor+FeatureManager) ROS1→ROS2, build, visualize the
   intensity image + tracked features** in Foxglove. No IEKF wiring yet.
3. **v1 constant-weight residual** behind `enable_photometric`. Validate on the apartment bag:
   trajectory must be ≈ unchanged in feature-rich sections (photometric shouldn't *hurt* where
   geometry is healthy). Watch `/analytics` and odom for NaNs/instability.
4. **v2 `obs_score`-modulated** weight. Re-run the degenerate bag from GATE 0; success = reduced
   in-plane drift through the flat-skin/nadir window vs geometric-only, with `obs_score` showing the
   dip and the photometric channel carrying it.
5. **Compute check:** photometric adds `n_features × patch²` rows × up-to-10 IEKF iterations on
   Orin NX. Budget with the CPU clock boost (`set_cpu_clock.sh boost`). Cap `n_features`.

Numerical-correctness risks to watch: the `p_feat_Li` transform chain (verify against COIN-LIO's
`R_IG`/`T_IL` conventions — EllipseLIO uses `s.rot`, `s.offset_R_L_I` MTK SO3, so use `.conjugate()`
for inverses as sketched), the photometric/geometric sign match, and the `h_x_R` weight scale
(get the variance ratio right or one channel dominates).
