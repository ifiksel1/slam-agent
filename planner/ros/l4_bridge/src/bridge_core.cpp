// Planner bridge core (see bridge_core.h). Port of the former planner/bridge_core.py, operation for
// operation, so the arithmetic matches numpy's to the last bit where the order of operations allows.
#include "l4_bridge/bridge_core.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>

namespace l4_bridge {

const char* const PLANNER_DEAD = "planner_dead";
const char* const SLAM_FROZEN = "slam_frozen";

std::string fmt(const char* f, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, f);
  std::vsnprintf(buf, sizeof buf, f, ap);
  va_end(ap);
  return buf;
}

const Mat3& position_matrix() {
  static const Mat3 m = (Mat3() << 0.0, 0.0, 1.0,
                                   1.0, 0.0, 0.0,
                                   0.0, 1.0, 0.0).finished();
  return m;
}

double wrap_pi(double a) { return std::atan2(std::sin(a), std::cos(a)); }

Quat quat_mul(const Quat& a, const Quat& b) {
  const double ax = a.x, ay = a.y, az = a.z, aw = a.w;
  const double bx = b.x, by = b.y, bz = b.z, bw = b.w;
  return {aw * bx + ax * bw + ay * bz - az * by,
          aw * by - ax * bz + ay * bw + az * bx,
          aw * bz + ax * by - ay * bx + az * bw,
          aw * bw - ax * bx - ay * by - az * bz};
}

Quat quat_conj(const Quat& q) { return {-q.x, -q.y, -q.z, q.w}; }

Quat quat_from_matrix(const Mat3& m) {
  const double tr = m(0, 0) + m(1, 1) + m(2, 2);
  if (tr > 0) {
    const double s = std::sqrt(tr + 1.0) * 2;
    return {(m(2, 1) - m(1, 2)) / s, (m(0, 2) - m(2, 0)) / s, (m(1, 0) - m(0, 1)) / s, 0.25 * s};
  }
  if (m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2)) {
    const double s = std::sqrt(1.0 + m(0, 0) - m(1, 1) - m(2, 2)) * 2;
    return {0.25 * s, (m(0, 1) + m(1, 0)) / s, (m(0, 2) + m(2, 0)) / s, (m(2, 1) - m(1, 2)) / s};
  }
  if (m(1, 1) > m(2, 2)) {
    const double s = std::sqrt(1.0 + m(1, 1) - m(0, 0) - m(2, 2)) * 2;
    return {(m(0, 1) + m(1, 0)) / s, 0.25 * s, (m(1, 2) + m(2, 1)) / s, (m(0, 2) - m(2, 0)) / s};
  }
  const double s = std::sqrt(1.0 + m(2, 2) - m(0, 0) - m(1, 1)) * 2;
  return {(m(0, 2) + m(2, 0)) / s, (m(1, 2) + m(2, 1)) / s, 0.25 * s, (m(1, 0) - m(0, 1)) / s};
}

const Quat& q_perm() {
  static const Quat q = quat_from_matrix(position_matrix());
  return q;
}

double yaw_of(const Quat& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

Quat quat_yaw(double psi) { return {0.0, 0.0, std::sin(psi / 2.0), std::cos(psi / 2.0)}; }

Vec3 raw_to_planner_pos(const Vec3& p) {
  // POSITION_MATRIX.dot(p): a permutation, row by row as numpy sums it
  const Mat3& m = position_matrix();
  Vec3 out;
  for (int i = 0; i < 3; ++i) out[i] = m(i, 0) * p[0] + m(i, 1) * p[1] + m(i, 2) * p[2];
  return out;
}

Quat raw_to_planner_quat(const Quat& q) { return quat_mul(quat_mul(q_perm(), q), quat_conj(q_perm())); }

// ---- Anchor -------------------------------------------------------------------------------------

Anchor Anchor::from_pair(const Vec3& p_planner, const Quat& q_planner, const Vec3& p_fc, const Quat& q_fc,
                         double mount_yaw) {
  // the bridge applies the mount body-side (q * Rz(-mount)) before the anchor's left Rz(psi); a left
  // yaw adds to yaw exactly, so this is exact even when the robot is tilted
  const Quat q_m = quat_mul(q_planner, quat_yaw(-mount_yaw));
  const double psi = wrap_pi(yaw_of(q_fc) - yaw_of(q_m));
  const double c = std::cos(psi), s = std::sin(psi);
  const Vec3& p = p_planner;
  const Vec3 rp(c * p[0] - s * p[1], s * p[0] + c * p[1], p[2]);
  return Anchor(psi, p_fc - rp);
}

Vec3 Anchor::pos(const Vec3& p) const {
  const double c = std::cos(psi), s = std::sin(psi);
  return Vec3(c * p[0] - s * p[1] + t[0], s * p[0] + c * p[1] + t[1], p[2] + t[2]);
}

Vec3 Anchor::vec(const Vec3& v) const {
  const double c = std::cos(psi), s = std::sin(psi);
  return Vec3(c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]);
}

double Anchor::yaw(double y) const { return wrap_pi(y + psi); }

Vec3 Anchor::inverse_pos(const Vec3& p) const {
  const double c = std::cos(-psi), s = std::sin(-psi);
  const Vec3 q = p - t;
  return Vec3(c * q[0] - s * q[1], s * q[0] + c * q[1], q[2]);
}

std::optional<std::string> Anchor::differs(const Anchor& other, double jump_m, double jump_rad) const {
  const double dpsi = std::abs(wrap_pi(psi - other.psi));
  const double dt = (t - other.t).norm();
  if (dpsi > jump_rad) return fmt("anchor yaw changed %.1f deg", dpsi * (180.0 / M_PI));
  if (dt > jump_m) return fmt("anchor moved %.2f m", dt);
  return std::nullopt;
}

AnchorTracker::AnchorTracker(double jump_m_, double jump_deg, int persist_, double mount_yaw_)
    : jump_m(jump_m_), jump_rad(jump_deg * (M_PI / 180.0)), persist(persist_), mount_yaw(mount_yaw_) {}

std::pair<Anchor, std::optional<std::string>> AnchorTracker::update(const Vec3& p_planner, const Quat& q_planner,
                                                                    const Vec3& p_fc, const Quat& q_fc) {
  const Anchor a = Anchor::from_pair(p_planner, q_planner, p_fc, q_fc, mount_yaw);
  if (!anchor) {
    anchor = a;
    return {a, std::nullopt};
  }
  auto why = a.differs(*anchor, jump_m, jump_rad);
  if (!why) {
    // small changes are noise: follow them slowly so the anchor stays exact
    anchor = Anchor(wrap_pi(anchor->psi + 0.1 * wrap_pi(a.psi - anchor->psi)), 0.9 * anchor->t + 0.1 * a.t);
    candidate_.clear();
    return {*anchor, std::nullopt};
  }
  candidate_.push_back(a);
  if (static_cast<int>(candidate_.size()) < persist) return {*anchor, std::nullopt};
  // it persisted: the frame really changed
  bool consistent = true;
  for (const auto& c : candidate_)
    if (c.differs(candidate_.back(), jump_m, jump_rad)) consistent = false;
  candidate_.clear();
  if (!consistent) return {*anchor, std::nullopt};
  anchor = a;
  jumps += 1;
  return {a, why};
}

// ---- output -------------------------------------------------------------------------------------

CapResult cap_command(Vec3 p, Vec3 v, Vec3 a, const Vec3& p_now, double speed_cap, double max_lead,
                      double fc_pos_p) {
  const double speed = v.norm();
  bool capped = false;
  if (speed > speed_cap && speed_cap > 0) {
    const double k = speed_cap / speed;
    v = v * k;
    a = a * k;
    capped = true;
  }
  Vec3 lead = p - p_now;
  double d = lead.norm();
  if (d > max_lead && max_lead > 0) {
    lead = lead * (max_lead / d);
    p = p_now + lead;
    d = max_lead;
    capped = true;
  }
  if (fc_pos_p > 0 && speed_cap > 0) {
    const Vec3 corr = fc_pos_p * lead;
    if (fc_pos_p * d >= speed_cap) {
      // the correction alone reaches the cap: no feed-forward, a shorter lead
      p = p_now + lead * (speed_cap / (fc_pos_p * d));
      v = Vec3::Zero();
      a = Vec3::Zero();
      capped = true;
    } else if ((v + corr).norm() > speed_cap) {
      // the largest k in [0, 1] with |k v + corr| = speed_cap
      const double qa = v.dot(v), qb = 2.0 * v.dot(corr), qc = corr.dot(corr) - speed_cap * speed_cap;
      const double k = (-qb + std::sqrt(qb * qb - 4 * qa * qc)) / (2 * qa);
      v = v * k;
      a = a * k;
      capped = true;
    }
  }
  return {p, v, a, capped};
}

Vec3 drive_target(const Vec3& p_cmd, const Vec3& v_cmd, const Vec3& p_now, double lookahead, double horizon_s) {
  const Vec3 aim = p_cmd + v_cmd * horizon_s;
  Vec3 d = aim - p_now;
  d[2] = 0.0;
  const double n = d.norm();
  if (n < 1e-6) return p_now;
  const double step = std::min(lookahead, n);
  Vec3 out = p_now + d / n * step;
  out[2] = p_now[2];
  return out;
}

std::optional<double> reverse_speed(const Vec3& p_cmd, const Vec3& v_cmd, double yaw, const Vec3& p_now, double gain,
                                    double stop_tol) {
  const Eigen::Vector2d v = v_cmd.head<2>();
  const double sp = v.norm();
  const Eigen::Vector2d h(std::cos(yaw), std::sin(yaw));
  if (sp < 1e-3 || v.dot(h) >= 0.0) return std::nullopt;
  const double left = (p_now.head<2>() - p_cmd.head<2>()).dot(h);
  if (left <= stop_tol) return 0.0;
  return std::min(sp, gain * left);
}

bool Geofence::inside(const Vec3& p) const {
  return (p.array() >= lo.array()).all() && (p.array() <= hi.array()).all();
}

Vec3 Geofence::clamp(const Vec3& p) const { return p.cwiseMax(lo).cwiseMin(hi); }

// ---- FcPoseGuard ----------------------------------------------------------------------------------

FcPoseGuard::FcPoseGuard(double timeout_, int resume_n_) : timeout(timeout_), resume_n(std::max(1, resume_n_)) {}

void FcPoseGuard::on_pose(double now, const FcPose& p) {
  if (t && now - *t > timeout) {
    stale = true;   // a gap the check did not see yet: the pose before it stays the good one
    fresh_n = 0;
  }
  pose = p;
  t = now;
  if (stale) {
    fresh_n += 1;
    if (fresh_n >= resume_n) stale = false;
  }
  if (!stale) good = p;
}

std::optional<double> FcPoseGuard::age(double now) const {
  if (!t) return std::nullopt;
  return now - *t;
}

std::string FcPoseGuard::check(double now) {
  if (!t) return "never";
  if (now - *t > timeout) {
    stale = true;
    fresh_n = 0;
  }
  return stale ? "stale" : "ok";
}

// ---- FrozenPoseCheck ------------------------------------------------------------------------------

FrozenPoseCheck::FrozenPoseCheck(double eps_m_, double eps_q_, double hold_s_, int min_n_, double cmd_speed_,
                                 double drive_dist_, double gyro_min_, double twist_min_, double evidence_age_,
                                 double max_age_, double drive_hold_s_)
    : eps_m(eps_m_), eps_q(eps_q_), hold_s(hold_s_), min_n(std::max(1, min_n_)), cmd_speed(cmd_speed_),
      drive_dist(drive_dist_), gyro_min(gyro_min_), twist_min(twist_min_), evidence_age(evidence_age_), max_age(max_age_),
      drive_hold_s(drive_hold_s_) {}

void FrozenPoseCheck::on_pose(double t, const Vec3& p, const Quat& q, const Vec3& twist) {
  bool same = false;
  if (last_) {
    const Quat& a = last_->second;
    same = (p - last_->first).norm() <= eps_m && std::abs(q.x - a.x) <= eps_q && std::abs(q.y - a.y) <= eps_q &&
           std::abs(q.z - a.z) <= eps_q && std::abs(q.w - a.w) <= eps_q;
  }
  if (same) {
    if (repeats == 0) run_start = t_pose;
    repeats += 1;
  } else {
    repeats = 0;
    run_start.reset();
    evidence_from.reset();
  }
  last_ = std::make_pair(p, q);
  twist_ = twist.norm();
  t_pose = t;
}

void FrozenPoseCheck::on_command(double t, double speed) {
  if (speed >= cmd_speed) t_cmd_move_ = t;
}

void FrozenPoseCheck::on_drive_target(double t, double dist) {
  if (dist >= drive_dist) t_cmd_move_ = t;
}

void FrozenPoseCheck::on_gyro(double t, double rate) {
  t_gyro_ = t;
  gyro_ = rate;
}

std::string FrozenPoseCheck::evidence(double now) const {
  std::string e;
  auto add = [&e](const char* k) { e += (e.empty() ? "" : "+") + std::string(k); };
  if (t_cmd_move_ && now - *t_cmd_move_ <= evidence_age) add("command");
  if (t_gyro_ && now - *t_gyro_ <= evidence_age && gyro_ >= gyro_min) add("gyro");
  if (repeats > 0 && twist_ >= twist_min) add("twist");
  return e;
}

std::optional<std::string> FrozenPoseCheck::check(double now, bool driving) {
  if (repeats == 0 || !t_pose || now - *t_pose > max_age) {
    evidence_from.reset();
    return std::nullopt;
  }
  const bool fc_alive = !t_gyro_ || now - *t_gyro_ <= evidence_age;
  const std::string e = fc_alive ? evidence(now) : std::string();
  if (e.empty()) {
    evidence_from.reset();   // the evidence must last without a break
    return std::nullopt;
  }
  if (!evidence_from) evidence_from = now;
  if (repeats >= min_n && now - *evidence_from >= (driving ? drive_hold_s : hold_s)) return e;
  return std::nullopt;
}

// ---- HoldLogic ------------------------------------------------------------------------------------

HoldLogic::HoldLogic(double planner_timeout_, double slam_timeout_)
    : planner_timeout(planner_timeout_), slam_timeout(slam_timeout_) {}

void HoldLogic::latch(const std::string& r, double now) {
  latched = r;
  hold(r, now);
}

void HoldLogic::new_leg() {
  dead = false;
  frozen = false;
  if (state == "HOLD" && (reason == PLANNER_DEAD || reason == SLAM_FROZEN)) reason = "waiting for a fresh planner command";
}

void HoldLogic::hold(const std::string& r, double now) {
  if (state != "HOLD") hold_since = now;
  state = "HOLD";
  reason = r;
}

std::string HoldLogic::step(double now, std::optional<double> t_cmd, std::optional<double> t_slam,
                            const std::vector<std::string>& faults, std::optional<bool> planner_alive,
                            bool slam_frozen) {
  if (slam_frozen) frozen = true;
  if (frozen) {
    hold(SLAM_FROZEN, now);
    return state;
  }
  if (planner_alive.has_value() && !*planner_alive && (!t_cmd || now - *t_cmd > planner_timeout)) dead = true;
  if (dead) {
    hold(PLANNER_DEAD, now);
    return state;
  }
  std::vector<std::string> causes(faults);
  if (!t_slam || now - *t_slam > slam_timeout)
    causes.push_back(t_slam ? fmt("SLAM stale %.2f s", now - *t_slam) : std::string("no SLAM odometry"));
  if (!t_cmd) {
    if (state == "WAIT" && causes.empty()) return state;
    causes.push_back("no planner command");
  } else if (now - *t_cmd > planner_timeout) {
    causes.push_back(fmt("planner stale %.2f s", now - *t_cmd));
  }
  if (latched) {
    if (t_cmd && hold_since && *t_cmd > *hold_since && causes.empty())
      latched.reset();
    else
      causes.insert(causes.begin(), *latched);   // a re-anchor is the reason to report first
  }
  if (!causes.empty()) {
    hold(causes[0], now);
    return state;
  }
  if (state == "HOLD" && (!t_cmd || !hold_since || *t_cmd <= *hold_since)) {
    reason = "waiting for a fresh planner command";
    return state;
  }
  state = "STREAM";
  reason = "";
  hold_since.reset();
  return state;
}

// ---- flight yaw ------------------------------------------------------------------------------------

std::optional<double> travel_yaw(const Vec3& v, double speed_min) {
  if (std::hypot(v[0], v[1]) <= speed_min) return std::nullopt;
  return std::atan2(v[1], v[0]);
}

double yaw_step(double current, double desired, double max_step) {
  const double d = wrap_pi(desired - current);
  return wrap_pi(current + std::max(-max_step, std::min(max_step, d)));
}

double yaw_max_step(double rate_max, double dt, double dt_max) {
  return rate_max * std::max(0.0, std::min(dt, dt_max));
}

std::optional<double> FloorFinder::below() const {
  if (static_cast<int>(d_.size()) < std::max(1, min_points_)) return std::nullopt;
  std::vector<double> d = d_;
  // numpy's linear percentile
  const double k = std::clamp(pct_, 0.0, 100.0) / 100.0 * (d.size() - 1);
  const size_t lo = static_cast<size_t>(std::floor(k)), hi = std::min(d.size() - 1, lo + 1);
  std::nth_element(d.begin(), d.begin() + lo, d.end());
  const double a = d[lo];
  const double b = hi == lo ? a : *std::min_element(d.begin() + lo + 1, d.end());
  return a + (b - a) * (k - lo);
}

Vec3 waypoint_hold_point(const std::optional<Vec3>& goal_fc, const Vec3& p_now, double goal_max) {
  if (goal_fc && (*goal_fc - p_now).norm() <= goal_max) return *goal_fc;
  return p_now;
}

HoldPoint waypoint_hold_point(const std::optional<Vec3>& goal_fc, const std::optional<Vec3>& planner_goal_fc,
                              const Vec3& p_now, double goal_max, const std::function<bool(const Vec3&)>& clear,
                              const std::optional<Vec3>& goal_short_fc) {
  auto ok = [&](const std::optional<Vec3>& c) { return c && (*c - p_now).norm() <= goal_max && (!clear || clear(*c)); };
  if (ok(goal_fc)) return {*goal_fc, "goal"};
  // the short point was checked with its own (larger) clearance
  if (clear && goal_short_fc && (*goal_short_fc - p_now).norm() <= goal_max) return {*goal_short_fc, "goal_short"};
  if (ok(planner_goal_fc)) return {*planner_goal_fc, "planner_goal"};
  return {p_now, "vehicle"};
}

std::optional<Vec3> first_clear_toward(const Vec3& from, const Vec3& to, double step,
                                       const std::function<bool(const Vec3&)>& clear) {
  const Vec3 d = to - from;
  const int n = std::max(1, int(std::ceil(d.norm() / std::max(step, 1e-3))));
  for (int k = 0; k <= n; ++k) {
    const Vec3 p = from + d * (double(k) / n);
    if (!clear || clear(p)) return p;
  }
  return std::nullopt;
}

// ---- obstacles on the path ahead ---------------------------------------------------------------------

ObstacleVoxels::ObstacleVoxels(double voxel_, int scans_n, double self_r_)
    : voxel(voxel_), self_r(self_r_), scans(std::max(1, scans_n)) {}

int64_t ObstacleVoxels::key(int64_t i, int64_t j, int64_t k) {
  // 21 bits per axis (+-1e6 voxels: +-100 km at 0.1 m)
  const int64_t m = (int64_t(1) << 21) - 1, o = int64_t(1) << 20;
  return (((i + o) & m) << 42) | (((j + o) & m) << 21) | ((k + o) & m);
}

void ObstacleVoxels::begin_scan(double t, const std::optional<Vec3>& vehicle) {
  cur_.clear();
  veh_ = vehicle;
  t_cur_ = t;
  open_ = true;
}

void ObstacleVoxels::add(double x, double y, double z) {
  if (!open_ || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return;
  if (veh_) {
    const double dx = x - (*veh_)[0], dy = y - (*veh_)[1], dz = z - (*veh_)[2];
    if (dx * dx + dy * dy + dz * dz <= self_r * self_r) return;
  }
  cur_.push_back(key(int64_t(std::floor(x / voxel)), int64_t(std::floor(y / voxel)), int64_t(std::floor(z / voxel))));
}

void ObstacleVoxels::end_scan() {
  if (!open_) return;
  open_ = false;
  std::sort(cur_.begin(), cur_.end());
  cur_.erase(std::unique(cur_.begin(), cur_.end()), cur_.end());
  for (int64_t k : cur_) occ_[k] += 1;
  scans_.emplace_back(t_cur_, std::move(cur_));
  cur_ = {};
  while (int(scans_.size()) > scans) {
    for (int64_t k : scans_.front().second) {
      auto it = occ_.find(k);
      if (it != occ_.end() && --it->second <= 0) occ_.erase(it);
    }
    scans_.pop_front();
  }
}

int ObstacleVoxels::count_within(const Vec3& p, double r) const {
  if (occ_.empty()) return 0;
  return count_voxels_near(p, r, r, voxel, [this](int64_t k) { return occ_.count(k) > 0; });
}

int count_voxels_near(const Vec3& p, double rh, double rz, double voxel, const std::function<bool(int64_t)>& occupied) {
  if (!(rh > 0) || !(rz > 0) || !(voxel > 0) || !occupied) return 0;
  const int64_t i0 = int64_t(std::floor((p[0] - rh) / voxel)), i1 = int64_t(std::floor((p[0] + rh) / voxel));
  const int64_t j0 = int64_t(std::floor((p[1] - rh) / voxel)), j1 = int64_t(std::floor((p[1] + rh) / voxel));
  const int64_t k0 = int64_t(std::floor((p[2] - rz) / voxel)), k1 = int64_t(std::floor((p[2] + rz) / voxel));
  const double h2 = rh * rh, z2 = rz * rz;
  int n = 0;
  for (int64_t i = i0; i <= i1; ++i) {
    const double dx = (i + 0.5) * voxel - p[0];
    for (int64_t j = j0; j <= j1; ++j) {
      const double dy = (j + 0.5) * voxel - p[1];
      const double eh = (dx * dx + dy * dy) / h2;
      if (eh > 1.0) continue;
      for (int64_t k = k0; k <= k1; ++k) {
        const double dz = (k + 0.5) * voxel - p[2];
        if (eh + dz * dz / z2 > 1.0) continue;
        if (occupied(ObstacleVoxels::key(i, j, k))) ++n;
      }
    }
  }
  return n;
}

LegMemory::LegMemory(double voxel_, double radius_, double self_r_, int min_hits_)
    : voxel(voxel_), radius(radius_), self_r(self_r_), min_hits(std::max(1, min_hits_)) {}

void LegMemory::reset(const std::optional<Vec3>& centre) {
  centre_ = centre;
  hits_.clear();
  cur_.clear();
  open_ = false;
  n_scans_ = 0;
  n_counted_ = 0;
}

void LegMemory::begin_scan(const std::optional<Vec3>& vehicle) {
  cur_.clear();
  veh_ = vehicle;
  open_ = centre_.has_value() && radius > 0;
}

void LegMemory::add(double x, double y, double z) {
  if (!open_ || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return;
  const double cx = x - (*centre_)[0], cy = y - (*centre_)[1], cz = z - (*centre_)[2];
  if (cx * cx + cy * cy + cz * cz > radius * radius) return;
  if (veh_) {
    const double dx = x - (*veh_)[0], dy = y - (*veh_)[1], dz = z - (*veh_)[2];
    if (dx * dx + dy * dy + dz * dz <= self_r * self_r) return;
  }
  cur_.push_back(ObstacleVoxels::key(int64_t(std::floor(x / voxel)), int64_t(std::floor(y / voxel)),
                                     int64_t(std::floor(z / voxel))));
}

void LegMemory::end_scan() {
  if (!open_) return;
  open_ = false;
  std::sort(cur_.begin(), cur_.end());
  cur_.erase(std::unique(cur_.begin(), cur_.end()), cur_.end());
  for (int64_t k : cur_)
    if (++hits_[k] == min_hits) ++n_counted_;
  cur_.clear();
  ++n_scans_;
}

bool LegMemory::has(int64_t k) const {
  auto it = hits_.find(k);
  return it != hits_.end() && it->second >= min_hits;
}

int LegMemory::count_within(const Vec3& p, double r) const {
  if (hits_.empty()) return 0;
  return count_voxels_near(p, r, r, voxel, [this](int64_t k) { return has(k); });
}

size_t LegMemory::size() const { return n_counted_; }

bool line_clear(const Vec3& a, const Vec3& b, double r, int min_voxels, double skip, double step, const VoxelCount& count) {
  const Vec3 e = b - a;
  const int n = std::max(1, int(std::ceil(e.norm() / std::max(step, 1e-3))));
  for (int k = 0; k <= n; ++k) {
    const Vec3 p = a + e * (double(k) / n);
    if ((p - a).norm() < skip) continue;
    if (count(p, r, r) >= min_voxels) return false;
  }
  return true;
}

GoalShort goal_short(const Vec3& goal, const Vec3& vehicle, double approach_dist, double short_max, double rh,
                     double short_r, double rz, int min_voxels, double line_r, double line_skip, double step,
                     const VoxelCount& count) {
  GoalShort out;
  if (!count) return out;
  if ((goal - vehicle).norm() > approach_dist) {
    out.why = "far";
    return out;
  }
  out.goal_clear = count(goal, rh, rz) < min_voxels;
  if (out.goal_clear) {
    out.why = "goal clear";
    return out;
  }
  // rings round the goal, nearest first: 16 directions and the one toward the vehicle
  std::vector<double> dirs;
  for (int k = 0; k < 16; ++k) dirs.push_back(k * (M_PI / 8.0));
  const Vec3 dv = vehicle - goal;
  if (dv.head<2>().norm() > 1e-6) dirs.push_back(std::atan2(dv[1], dv[0]));
  const double st = std::max(step, 1e-3);
  for (int ir = 1; ir * st <= short_max + 1e-9; ++ir) {
    const double r = ir * st;
    std::optional<Vec3> best;
    double best_d = 0.0;
    for (double th : dirs) {
      const Vec3 c = goal + Vec3(r * std::cos(th), r * std::sin(th), 0.0);
      if (count(c, short_r, rz) >= min_voxels) continue;
      const double d = (c - vehicle).norm();
      if (best && d >= best_d) continue;
      if (!line_clear(vehicle, c, line_r, min_voxels, line_skip, st, count)) continue;
      best = c;
      best_d = d;
    }
    if (best) {
      out.point = best;
      out.why = "short";
      return out;
    }
  }
  out.why = "no clear point within short_max";
  return out;
}

double PolyTraj::duration() const {
  double d = 0.0;
  for (double x : dur) d += x;
  return d;
}

Vec3 PolyTraj::pos(double t) const {
  if (dur.empty()) return Vec3::Zero();
  const int n1 = order + 1;
  double tt = std::max(0.0, t - t0);
  size_t i = 0;
  while (i + 1 < dur.size() && tt > dur[i]) {
    tt -= dur[i];
    ++i;
  }
  tt = std::min(tt, dur[i]);
  Vec3 out = Vec3::Zero();
  const size_t b = i * size_t(n1);
  if (b + n1 > cx.size() || b + n1 > cy.size() || b + n1 > cz.size()) return out;
  // Horner, highest power first
  for (int k = 0; k < n1; ++k) out = out * tt + Vec3(cx[b + k], cy[b + k], cz[b + k]);
  return out;
}

std::vector<Vec3> path_samples(const std::optional<PolyTraj>& traj, double t_from, double t_to, double dt,
                               const Vec3& vehicle, const Vec3& target, double step, double skip) {
  std::vector<Vec3> out;
  if (traj && !traj->empty() && dt > 0) {
    const double a = std::max(t_from, traj->t0), b = std::min(t_to, traj->t0 + traj->duration());
    if (b >= a) {
      for (double t = a; t < b; t += dt) out.push_back(traj->pos(t));
      out.push_back(traj->pos(b));
    } else {
      out.push_back(traj->pos(t_from));   // the trajectory is over (its end) or not begun (its start)
    }
  }
  const Vec3 d = target - vehicle;
  const int n = std::max(1, int(std::ceil(d.norm() / std::max(step, 1e-3))));
  for (int k = 0; k <= n; ++k) out.push_back(vehicle + d * (double(k) / n));
  if (skip > 0)
    out.erase(std::remove_if(out.begin(), out.end(), [&](const Vec3& p) { return (p - vehicle).norm() < skip; }), out.end());
  return out;
}

PathGuard::PathGuard(double r_, int min_voxels_, double release_s_)
    : r(r_), release_s(release_s_), min_voxels(std::max(1, min_voxels_)) {}

void PathGuard::reset() {
  blocked = false;
  since.reset();
  at.reset();
  voxels = 0;
  clear_since.reset();
}

bool PathGuard::update(double now, const ObstacleVoxels& obs, const std::vector<Vec3>& samples,
                       std::optional<double> traj_rcv) {
  return update(now, [&obs](const Vec3& p, double rr) { return obs.count_within(p, rr); }, samples, traj_rcv);
}

bool PathGuard::update(double now, const std::function<int(const Vec3&, double)>& count, const std::vector<Vec3>& samples,
                       std::optional<double> traj_rcv) {
  std::optional<Vec3> hit;
  int nv = 0;
  for (const Vec3& p : samples) {
    nv = count(p, r);
    if (nv >= min_voxels) {
      hit = p;
      break;
    }
  }
  if (hit) {
    at = hit;
    voxels = nv;
  }
  if (!blocked) {
    if (hit) {
      blocked = true;
      since = now;
      clear_since.reset();
    }
    return blocked;
  }
  const bool new_plan = !traj_rcv || (since && *traj_rcv > *since);
  if (hit || !new_plan) {
    clear_since.reset();
    return blocked;
  }
  if (!clear_since) clear_since = now;
  if (now - *clear_since >= release_s) {
    blocked = false;
    since.reset();
    clear_since.reset();
  }
  return blocked;
}

bool departed(const Vec3& point, const std::optional<Vec3>& goal_fc, bool goal_new, const Vec3& p_cmd, const Vec3& v_cmd,
              double speed_min, double behind_tol, double min_dist) {
  if (!goal_fc) return true;   // no goal known: nothing to wait for
  Eigen::Vector2d d = (*goal_fc - point).head<2>();
  const double n = d.norm();
  if (n <= min_dist) return goal_new;
  d /= n;
  return v_cmd.head<2>().dot(d) > speed_min && (p_cmd - point).head<2>().dot(d) >= -behind_tol;
}

// ---- the turn slowdown ------------------------------------------------------------------------------

double turn_speed_frac(double err, double start, double full, double min_frac) {
  if (err <= start) return 1.0;
  if (err >= full || full <= start) return min_frac;
  const double x = (err - start) / (full - start);
  const double s = x * x * (3.0 - 2.0 * x);
  return 1.0 - (1.0 - min_frac) * s;
}

std::optional<double> turn_heading_error(double yaw, const Vec3& v, const Vec3& a, double lookahead, double speed_min) {
  if (std::hypot(v[0], v[1]) <= speed_min) return std::nullopt;
  double err = std::abs(wrap_pi(yaw - std::atan2(v[1], v[0])));
  const Vec3 ahead = v + a * lookahead;
  if (lookahead > 0 && std::hypot(ahead[0], ahead[1]) > speed_min)
    err = std::max(err, std::abs(wrap_pi(yaw - std::atan2(ahead[1], ahead[0]))));
  return err;
}

double replay_rate(double allowed, double speed, double lag, double catchup, double max_lag) {
  // catching up only beyond REPLAY_LAG_MIN: SUPER's commands arrive in bursts against the bridge's tick,
  // and a replay sitting at the newest one would otherwise flick between catching up and SUPER's pace
  double rate = lag > REPLAY_LAG_MIN ? std::max(1.0, catchup) : 1.0;
  if (speed > 1e-6) rate = std::min(rate, std::max(0.0, allowed) / speed);
  // near the lag limit the rate's floor rises linearly to SUPER's pace (a hard switch at max_lag flicked
  // between the two with each burst of commands)
  const double band = std::min(REPLAY_LAG_BAND, max_lag);
  const double floor = band > 0 ? (lag - (max_lag - band)) / band : (lag >= max_lag ? 1.0 : 0.0);
  return std::max(rate, std::min(1.0, floor));
}

void RefBuffer::push(const Sample& s) {
  if (!s_.empty() && s.t < s_.back().t) return;
  s_.push_back(s);
}

RefBuffer::Sample RefBuffer::at(double t) const {
  if (t <= s_.front().t) return s_.front();
  if (t >= s_.back().t) return s_.back();
  // the first sample after t
  const auto it = std::upper_bound(s_.begin(), s_.end(), t, [](double x, const Sample& s) { return x < s.t; });
  const Sample& b = *it;
  const Sample& a = *(it - 1);
  const double span = b.t - a.t;
  const double w = span > 0 ? (t - a.t) / span : 1.0;
  return Sample{t, a.p + (b.p - a.p) * w, a.v + (b.v - a.v) * w, a.a + (b.a - a.a) * w, a.yaw};
}

void RefBuffer::trim(double before) {
  size_t k = 0;
  while (k + 1 < s_.size() && s_[k + 1].t <= before) ++k;
  if (k > 0) s_.erase(s_.begin(), s_.begin() + static_cast<long>(k));
}

// ---- HealthCheck ----------------------------------------------------------------------------------

HealthCheck::HealthCheck(double matched_min_, double eig_min_, double latency_max_ms_, double height_tol_,
                         double max_age_)
    : matched_min(matched_min_), eig_min(eig_min_), latency_max_ms(latency_max_ms_), height_tol(height_tol_),
      max_age(max_age_) {}

void HealthCheck::on_health(double t, const std::vector<double>& data) {
  if (data.size() > 7) health = Health{t, data[0], data[4], data[7]};
}

void HealthCheck::on_range(double t, double r, double fc_z) {
  if (!(0.05 < r && r < 20.0)) return;
  range = RangeSample{t, r, fc_z};
  if (!height_offset) height_offset = fc_z - r;
}

std::vector<std::string> HealthCheck::faults(double now) const {
  std::vector<std::string> out;
  if (health && now - health->t < max_age) {
    if (health->matched < matched_min) out.push_back(fmt("SLAM matched points %lld", static_cast<long long>(health->matched)));
    if (health->eig < eig_min) out.push_back(fmt("SLAM observability %.0f", health->eig));
    if (health->latency > latency_max_ms) out.push_back(fmt("SLAM latency %.0f ms", health->latency));
  }
  if (range && height_offset && now - range->t < max_age) {
    const double err = (range->fc_z - range->r) - *height_offset;
    if (std::abs(err) > height_tol) out.push_back(fmt("SLAM height disagrees with the rangefinder by %.2f m", err));
  }
  return out;
}

// ---- a SLAM frame reset (L2 3a / 3b) ----------------------------------------------------------------------

Anchor anchor_compose(const Anchor& a, const Anchor& b) { return Anchor(wrap_pi(a.psi + b.psi), a.vec(b.t) + a.t); }

Anchor anchor_inverse(const Anchor& a) {
  const Anchor r(-a.psi, Vec3::Zero());
  return Anchor(wrap_pi(-a.psi), -r.vec(a.t));
}

FrameChange classify_frame_change(const Anchor& old_anchor, const Vec3& p_raw_planner, const Quat& q_raw_planner,
                                  const Vec3& p_vision, const Quat& q_vision, double gap_s, double min_m, double rate,
                                  double min_deg, double mount_yaw) {
  const Anchor a_new = Anchor::from_pair(p_raw_planner, q_raw_planner, p_vision, q_vision, mount_yaw);
  FrameChange out;
  out.residual_m = (old_anchor.pos(p_raw_planner) - p_vision).norm();
  out.residual_deg = std::abs(wrap_pi(a_new.psi - old_anchor.psi)) * (180.0 / M_PI);
  out.reset = out.residual_m > std::max(min_m, rate * std::max(0.0, gap_s)) || out.residual_deg > min_deg;
  return out;
}

void FrameCarry::reset_to(const Anchor& a_old_raw, const Anchor& a_new_raw) {
  const Anchor a_pub = published(a_old_raw);
  c_ = anchor_compose(anchor_inverse(a_pub), a_new_raw);
  resets += 1;
}

Quat FrameCarry::quat(const Quat& q) const { return quat_mul(quat_yaw(c_.psi), q); }

bool plan_after(const std::optional<PolyTraj>& traj, double t_reanchor, const Vec3& vehicle, double start_tol,
                double slack) {
  if (!traj || traj->empty()) return false;
  if (traj->t0 < t_reanchor - slack) return false;
  return (traj->pos(traj->t0) - vehicle).norm() <= start_tol;
}

bool clamp_vertical(Vec3& p, Vec3& v, Vec3& a, const Vec3& p_now, const std::optional<double>& z_last, double dt,
                    double vz_max, double z_lead) {
  bool clipped = false;
  if (z_last && dt > 0 && vz_max > 0) {
    const double step = vz_max * dt;
    const double z = std::clamp(p[2], *z_last - step, *z_last + step);
    if (z != p[2]) {
      p[2] = z;
      clipped = true;
    }
  }
  if (z_lead > 0) {
    const double z = std::clamp(p[2], p_now[2] - z_lead, p_now[2] + z_lead);
    if (z != p[2]) {
      p[2] = z;
      clipped = true;
    }
  }
  if (vz_max > 0 && std::abs(v[2]) > vz_max) {
    v[2] = std::copysign(vz_max, v[2]);
    a[2] = 0.0;
    clipped = true;
  }
  return clipped;
}

}  // namespace l4_bridge
