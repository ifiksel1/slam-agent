// Core tests of the planner bridge: one TEST per case of the former planner/test/test_planner_bridge.py
// (classes Frames, AnchorTests, Output, Hold, FcPose, Health), same inputs and expectations.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>

#include <Eigen/LU>

#include <yaml-cpp/yaml.h>

#include "l4_bridge/bridge_core.h"

using namespace l4_bridge;

namespace {

void near3(const Vec3& a, const Vec3& b, double tol = 1e-7) {
  for (int i = 0; i < 3; ++i) EXPECT_NEAR(a[i], b[i], tol) << "component " << i << ": " << a.transpose() << " vs " << b.transpose();
}

Quat rand_quat(std::mt19937_64& rng) {
  std::normal_distribution<double> n;
  double q[4] = {n(rng), n(rng), n(rng), n(rng)};
  const double s = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  return {q[0] / s, q[1] / s, q[2] / s, q[3] / s};
}

Vec3 rand_vec(std::mt19937_64& rng, double scale = 1.0) {
  std::normal_distribution<double> n;
  return Vec3(n(rng), n(rng), n(rng)) * scale;
}

Vec3 quat_rotate(const Quat& q, const Vec3& v) {
  const Quat p = quat_mul(quat_mul(q, Quat{v[0], v[1], v[2], 0.0}), quat_conj(q));
  return Vec3(p.x, p.y, p.z);
}

FcPose tag(double k) { return FcPose{Vec3(k, 0, 0), 0.0}; }

}  // namespace

// ---- Frames ---------------------------------------------------------------------------------------

TEST(Frames, PositionMatrixIsTheBridges) {
  // raw +z forward -> ENU x, raw +x left -> ENU y, raw +y up -> ENU z
  near3(raw_to_planner_pos(Vec3(0, 0, 1)), Vec3(1, 0, 0));
  near3(raw_to_planner_pos(Vec3(1, 0, 0)), Vec3(0, 1, 0));
  near3(raw_to_planner_pos(Vec3(0, 1, 0)), Vec3(0, 0, 1));
  EXPECT_NEAR(position_matrix().determinant(), 1.0, 1e-12);
}

TEST(Frames, OrientationIsTheSimilarityTransform) {
  // rotating a vector with the converted quaternion = converting the rotated raw vector
  std::mt19937_64 rng(1);
  for (int i = 0; i < 50; ++i) {
    const Quat q = rand_quat(rng);
    const Vec3 v = rand_vec(rng);
    near3(quat_rotate(raw_to_planner_quat(q), raw_to_planner_pos(v)), raw_to_planner_pos(quat_rotate(q, v)), 1e-9);
  }
}

TEST(Frames, CloudMatchesPositions) {
  // the node rotates clouds as (x, y, z) -> (z, x, y) on float32: the same as raw_to_planner_pos
  std::mt19937_64 rng(2);
  for (int i = 0; i < 100; ++i) {
    const Vec3 p = rand_vec(rng);
    near3(Vec3(p[2], p[0], p[1]), raw_to_planner_pos(p), 0.0);
  }
}

// ---- Anchor ---------------------------------------------------------------------------------------

class AnchorTests : public ::testing::Test {
 protected:
  std::mt19937_64 rng{3};

  void check_anchor_recovered(double psi, const Vec3& t, double mount_yaw = 0.0) {
    const Vec3 p = rand_vec(rng, 5.0);
    const Quat q = rand_quat(rng);
    // what the SLAM bridge publishes: anchor(position); anchor(orientation * mount)
    const Quat q_m = quat_mul(q, quat_yaw(-mount_yaw));
    const Anchor A(psi, t);
    const Vec3 p_fc = A.pos(p);
    const Quat q_fc = quat_mul(quat_yaw(psi), q_m);
    const Anchor got = Anchor::from_pair(p, q, p_fc, q_fc, mount_yaw);
    EXPECT_NEAR(wrap_pi(got.psi - psi), 0.0, 1e-9);
    near3(got.t, t, 1e-9);
    // and it maps another point the same way as the bridge would
    const Vec3 p2 = rand_vec(rng, 5.0);
    near3(got.pos(p2), A.pos(p2), 1e-9);
    near3(got.inverse_pos(got.pos(p2)), p2, 1e-9);
  }
};

TEST_F(AnchorTests, RecoversTheBridgesAnchor) {
  std::uniform_real_distribution<double> u(-M_PI, M_PI);
  for (int i = 0; i < 20; ++i) {
    const double psi = u(rng);
    check_anchor_recovered(psi, rand_vec(rng, 3.0));
  }
}

TEST_F(AnchorTests, MountYawDoesNotBiasIt) { check_anchor_recovered(0.7, Vec3(1.0, -2.0, 0.3), -0.8 * M_PI / 180.0); }

TEST_F(AnchorTests, TrackerJumpNeedsToPersist) {
  AnchorTracker tr(0.1, 2.0, 3);
  const Vec3 p(1.0, 2.0, 1.0);
  const Quat q = quat_yaw(0.3);
  auto pair = [&](const Anchor& a) { return tr.update(p, q, a.pos(p), quat_mul(quat_yaw(a.psi), q)).second; };
  const Anchor A(0.5, Vec3(1.0, 0.0, 0.0));
  for (int i = 0; i < 5; ++i) EXPECT_FALSE(pair(A));
  const Anchor B(0.5, Vec3(1.5, 0.0, 0.0));
  EXPECT_FALSE(pair(B));   // one odd pair: a glitch
  EXPECT_FALSE(pair(A));
  std::vector<std::optional<std::string>> reasons;
  for (int i = 0; i < 3; ++i) reasons.push_back(pair(B));
  EXPECT_FALSE(reasons[0]);
  EXPECT_FALSE(reasons[1]);
  ASSERT_TRUE(reasons[2]);
  EXPECT_NE(reasons[2]->find("moved"), std::string::npos);
  near3(tr.anchor->t, B.t);
  const Anchor C(0.5 + 5.0 * M_PI / 180.0, B.t);
  reasons.clear();
  for (int i = 0; i < 3; ++i) reasons.push_back(pair(C));
  ASSERT_TRUE(reasons[2]);
  EXPECT_NE(reasons[2]->find("yaw"), std::string::npos);
}

// ---- Output ---------------------------------------------------------------------------------------

TEST(Output, SpeedCapScalesVelocityAndAcceleration) {
  auto r = cap_command(Vec3(1, 0, 1), Vec3(2, 0, 0), Vec3(1, 1, 0), Vec3(0.5, 0, 1), 0.5, 1.0);
  EXPECT_TRUE(r.capped);
  EXPECT_NEAR(r.v.norm(), 0.5, 1e-7);
  near3(r.a, Vec3(0.25, 0.25, 0));
  near3(r.p, Vec3(1, 0, 1));
}

TEST(Output, LeadIsClamped) {
  auto r = cap_command(Vec3(5, 0, 1), Vec3(0.3, 0, 0), Vec3(0, 0, 0), Vec3(0, 0, 1), 0.5, 1.0);
  EXPECT_TRUE(r.capped);
  near3(r.p, Vec3(1, 0, 1));
}

TEST(Output, UnderTheCapUnchanged) {
  auto r = cap_command(Vec3(0.3, 0, 1), Vec3(0.4, 0, 0), Vec3(0.1, 0, 0), Vec3(0, 0, 1), 0.5, 1.0);
  EXPECT_FALSE(r.capped);
  near3(r.v, Vec3(0.4, 0, 0));
}

TEST(Output, CapBoundsTheFcDemand) {
  // lagging 0.6 m behind a 1.0 m/s reference: the FC would fly 1.0 + 1.0 * 0.6 = 1.6 m/s
  auto r = cap_command(Vec3(0.6, 0, 1), Vec3(1.0, 0, 0), Vec3(0.5, 0, 0), Vec3(0, 0, 1), 1.0, 1.0, 1.0);
  EXPECT_TRUE(r.capped);
  near3(r.p, Vec3(0.6, 0, 1));   // the correction is kept
  EXPECT_NEAR((r.v + 1.0 * (r.p - Vec3(0, 0, 1))).norm(), 1.0, 1e-7);
  near3(r.a, Vec3(0.2, 0, 0), 1e-9);   // scaled with the velocity
  // a lateral error: the demand is still within the cap
  r = cap_command(Vec3(0.2, 0.5, 1), Vec3(1.0, 0, 0), Vec3(0, 0, 0), Vec3(0, 0, 1), 1.0, 1.0, 1.0);
  EXPECT_LE((r.v + (r.p - Vec3(0, 0, 1))).norm(), 1.0 + 1e-9);
  // the correction alone over the cap: no feed-forward, a lead of cap / gain
  r = cap_command(Vec3(2.0, 0, 1), Vec3(1.0, 0, 0), Vec3(0, 0, 0), Vec3(0, 0, 1), 1.0, 1.5, 1.0);
  near3(r.v, Vec3::Zero());
  near3(r.p, Vec3(1.0, 0, 1));
  // under the cap: unchanged
  r = cap_command(Vec3(0.2, 0, 1), Vec3(0.5, 0, 0), Vec3(0, 0, 0), Vec3(0, 0, 1), 1.0, 1.0, 1.0);
  EXPECT_FALSE(r.capped);
  near3(r.v, Vec3(0.5, 0, 0));
}

TEST(Output, DriveTargetWithinTheLookahead) {
  near3(drive_target(Vec3(3, 0, 0.2), Vec3(1, 0, 0), Vec3(0, 0, 0.2), 0.5), Vec3(0.5, 0, 0.2));
  const Vec3 n = drive_target(Vec3(0.1, 0.1, 0.2), Vec3(0, 0, 0), Vec3(0, 0, 0.2), 0.5);
  EXPECT_LE(n.head<2>().norm(), 0.5);
}

TEST(Output, DriveTargetOnATrackPlannerCarrot) {
  // the track planner's carrot: position + yaw, zero velocity; 0.5 m ahead of its odometry point, which
  // is ~0.18 m ahead of the FC: the target is the lookahead toward it, at the FC's z
  near3(drive_target(Vec3(0.68, 0.0, 0.25), Vec3(0, 0, 0), Vec3(0.0, 0.0, 0.05), 0.5), Vec3(0.5, 0.0, 0.05), 1e-9);
  near3(drive_target(Vec3(0.2, 0.2, 0.25), Vec3(0, 0, 0), Vec3(0.0, 0.0, 0.05), 0.5), Vec3(0.2, 0.2, 0.05), 1e-9);
}

TEST(Output, DriveTargetOnARunOut) {
  // the track planner's run-out puts its target up to 2.1 m from the FC on the carrot's bearing (at
  // 1 m/s): passed as is within ~drive_lookahead (2.5), clipped beyond it
  near3(drive_target(Vec3(2.1 * 0.6, 2.1 * 0.8, 0.25), Vec3(0, 0, 0), Vec3(0.0, 0.0, 0.05), 2.5), Vec3(1.26, 1.68, 0.05), 1e-9);
  near3(drive_target(Vec3(3.0, 0.0, 0.25), Vec3(0, 0, 0), Vec3(0.0, 0.0, 0.05), 2.5), Vec3(2.5, 0.0, 0.05), 1e-9);
}

TEST(Output, DriveLookaheadCoversTheRunOut) {
  // the config's drive_lookahead must not clip the track planner's run-out (2.1 s x 1.0 m/s from the FC
  // at the Gate's 1 m/s, plus the FC/EKF and odometry disagreeing by a few cm)
  YAML::Node cfg = YAML::LoadFile(BRIDGE_CONFIG);
  EXPECT_GE(cfg["drive_lookahead"].as<double>(), 2.1 * 1.0 + 0.1);
}

TEST(Output, ReverseSpeed) {
  // a velocity against the yaw is a reverse to p_cmd: |v| at most, tapering 1/s on the metres left
  // (along the heading, the FC past the end point), 0 within 2 cm or past it
  EXPECT_FALSE(reverse_speed(Vec3(1, 0, 0), Vec3(0.3, 0, 0), 0.0, Vec3(0, 0, 0)));   // forward
  EXPECT_FALSE(reverse_speed(Vec3(1, 0, 0), Vec3(0, 0, 0), 0.0, Vec3(0, 0, 0)));     // a carrot
  EXPECT_NEAR(*reverse_speed(Vec3(-1.0, 0, 0), Vec3(-0.25, 0, 0), 0.0, Vec3(0, 0, 0)), 0.25, 1e-7);
  EXPECT_NEAR(*reverse_speed(Vec3(-1.0, 0, 0), Vec3(-0.25, 0, 0), 0.0, Vec3(-0.9, 0.3, 0)), 0.1, 1e-7);
  EXPECT_EQ(*reverse_speed(Vec3(-1.0, 0, 0), Vec3(-0.25, 0, 0), 0.0, Vec3(-0.99, 0, 0)), 0.0);
  EXPECT_EQ(*reverse_speed(Vec3(-1.0, 0, 0), Vec3(-0.25, 0, 0), 0.0, Vec3(-1.2, 0, 0)), 0.0);
  // heading north, backing south
  EXPECT_NEAR(*reverse_speed(Vec3(0, -2.0, 0), Vec3(0, -0.3, 0), M_PI / 2, Vec3(0, 0, 0)), 0.3, 1e-7);
}

TEST(Output, CeilingOverTheVehicle) {
  // container fly-through k1: the roof 1.1 m over the vehicle at 1.05 m; walls 1.05 m to either side
  CeilingFinder c(Vec3(0, 0, 1.05), 0.6, 0.15);
  EXPECT_FALSE(c.above());
  c.add(0.0, 1.05, 1.5);    // a wall, outside the radius
  c.add(0.3, 0.0, 1.15);    // 0.10 m up: the vehicle's own level, not a ceiling
  c.add(0.1, 0.0, -0.05);   // the floor
  c.add(std::nan(""), 0.0, 1.5);
  EXPECT_FALSE(c.above());
  c.add(0.4, 0.3, 2.20);
  c.add(-0.2, 0.1, 2.15);   // the lowest point over it
  c.add(0.9, 0.0, 1.60);    // lower, but 0.9 m away
  ASSERT_TRUE(c.above());
  EXPECT_NEAR(*c.above(), 1.10, 1e-9);
}

TEST(Output, Geofence) {
  Geofence f(Vec3(-1, -1, 0), Vec3(1, 1, 2));
  EXPECT_TRUE(f.inside(Vec3(0, 0, 1)));
  EXPECT_FALSE(f.inside(Vec3(2, 0, 1)));
  near3(f.clamp(Vec3(2, -3, 5)), Vec3(1, -1, 2));
}

// ---- Hold -----------------------------------------------------------------------------------------

TEST(Hold, StreamsWhenFresh) {
  HoldLogic h(0.3, 0.2);
  EXPECT_EQ(h.step(10.0, 9.95, 9.95), "STREAM");
}

TEST(Hold, PlannerStale) {
  HoldLogic h(0.3, 0.2);
  h.step(10.0, 9.95, 9.95);
  EXPECT_EQ(h.step(10.5, 10.0, 10.45), "HOLD");
  EXPECT_NE(h.reason.find("planner stale"), std::string::npos);
}

TEST(Hold, SlamStale) {
  HoldLogic h(0.3, 0.2);
  h.step(10.0, 9.95, 9.95);
  EXPECT_EQ(h.step(10.3, 10.25, 10.0), "HOLD");
  EXPECT_NE(h.reason.find("SLAM stale"), std::string::npos);
}

TEST(Hold, ResumeNeedsAFreshCommand) {
  HoldLogic h(0.3, 0.2);
  h.step(10.0, 9.95, 9.95);
  h.step(10.3, 10.25, 10.0);                         // SLAM stale -> HOLD at 10.3
  EXPECT_EQ(h.step(10.4, 10.25, 10.38), "HOLD");      // SLAM back, but the command predates the hold
  EXPECT_EQ(h.step(10.5, 10.45, 10.48), "STREAM");
}

TEST(Hold, LatchedAnchorJump) {
  HoldLogic h(0.3, 0.2);
  h.step(10.0, 9.95, 9.95);
  h.latch("frame re-anchored", 10.0);
  EXPECT_EQ(h.step(10.1, 9.99, 10.08), "HOLD");       // command older than the jump
  EXPECT_EQ(h.step(10.2, 10.15, 10.18), "STREAM");
}

TEST(Hold, ExternalFault) {
  HoldLogic h(0.3, 0.2);
  EXPECT_EQ(h.step(10.0, 9.95, 9.95, {"target outside the geofence"}), "HOLD");
  EXPECT_EQ(h.reason, "target outside the geofence");
}

TEST(Hold, WaitBeforeAnyCommand) {
  HoldLogic h(0.3, 0.2);
  EXPECT_EQ(h.step(10.0, std::nullopt, 9.95), "WAIT");
}

TEST(Hold, ModeChangeHoldsUntilTheNewPlannerCommands) {
  // the node drops the last command on a mode change and latches: no command at all, then one from the
  // new planner, is what resumes
  HoldLogic h(0.3, 0.2);
  h.step(10.0, 9.95, 9.95);
  h.latch("mode changed to drive", 10.0);
  EXPECT_EQ(h.step(10.1, std::nullopt, 10.08), "HOLD");
  EXPECT_EQ(h.reason, "mode changed to drive");
  EXPECT_EQ(h.step(10.2, 10.15, 10.18), "STREAM");
}

TEST(Hold, PlannerDeadLatchesAndNeverResumes) {
  HoldLogic h(0.3, 0.2);
  EXPECT_EQ(h.step(10.0, 9.99, 9.99, {}, true), "STREAM");
  // the process is gone but its last command is only 0.1 s old: not yet
  EXPECT_EQ(h.step(10.1, 10.0, 10.09, {}, false), "STREAM");
  EXPECT_EQ(h.step(10.35, 10.0, 10.34, {}, false), "HOLD");
  EXPECT_EQ(h.reason, PLANNER_DEAD);
  EXPECT_TRUE(h.dead);
  // the planner comes back and plans again: the leg is over, still HOLD
  for (double t : {10.5, 11.0, 12.0}) {
    EXPECT_EQ(h.step(t, t - 0.01, t - 0.01, {}, true), "HOLD");
    EXPECT_EQ(h.reason, PLANNER_DEAD);
  }
}

TEST(Hold, PlannerDeadEndsWithANewLegOnly) {
  HoldLogic h(0.3, 0.2);
  h.step(10.0, 9.99, 9.99, {}, true);
  h.step(10.4, 10.0, 10.39, {}, false);
  h.new_leg();
  EXPECT_EQ(h.reason, "waiting for a fresh planner command");
  EXPECT_EQ(h.step(10.5, 10.0, 10.49, {}, false), "HOLD");   // still gone: again
  EXPECT_EQ(h.reason, PLANNER_DEAD);
  h.new_leg();
  EXPECT_EQ(h.step(10.6, 10.59, 10.59, {}, true), "STREAM");  // a restarted planner, new leg
}

TEST(Hold, AliveButSilentPlannerIsTheResumableStaleHold) {
  // SUPER with no plan, the track planner at NO_PATH or REACHED: not dead
  HoldLogic h(0.3, 0.2);
  h.step(10.0, 9.99, 9.99, {}, true);
  EXPECT_EQ(h.step(12.0, 10.0, 11.99, {}, true), "HOLD");
  EXPECT_NE(h.reason.find("planner stale"), std::string::npos);
  EXPECT_FALSE(h.dead);
  EXPECT_EQ(h.step(12.1, 12.05, 12.09, {}, true), "STREAM");
  // liveness unknown (no answer yet, master unreachable): not dead either
  EXPECT_EQ(h.step(13.0, 12.05, 12.99, {}, std::nullopt), "HOLD");
  EXPECT_NE(h.reason.find("planner stale"), std::string::npos);
}

TEST(Hold, APlannerThatNeverStartedKeepsTheWait) {
  HoldLogic h(0.3, 0.2);
  EXPECT_EQ(h.step(10.0, std::nullopt, 9.99, {}, std::nullopt), "WAIT");
  EXPECT_EQ(h.step(30.0, std::nullopt, 29.99, {}, std::nullopt), "WAIT");
}

TEST(Hold, AKnownPlannerDeadAfterAModeChange) {
  // the node drops the command on a mode change; that mode's planner, known from an earlier leg, is
  // gone: dead without waiting for a first command
  HoldLogic h(0.3, 0.2);
  h.step(10.0, 9.99, 9.99, {}, true);
  h.latch("mode changed to drive", 10.0);
  EXPECT_EQ(h.step(10.1, std::nullopt, 10.09, {}, false), "HOLD");
  EXPECT_EQ(h.reason, PLANNER_DEAD);
}

TEST(Hold, SlamFrozenLatchesAheadOfEverythingUntilANewLeg) {
  HoldLogic h(0.3, 0.2);
  EXPECT_EQ(h.step(10.0, 9.99, 9.99, {}, true), "STREAM");
  EXPECT_EQ(h.step(10.1, 10.09, 10.09, {}, true, true), "HOLD");
  EXPECT_EQ(h.reason, SLAM_FROZEN);
  EXPECT_TRUE(h.frozen);
  // fresh commands, the pose moving again, even a dead planner: still SLAM_FROZEN, the first reason
  for (double t : {10.5, 11.0, 12.0}) {
    EXPECT_EQ(h.step(t, t - 0.01, t - 0.01, {"vehicle outside the geofence"}, false), "HOLD");
    EXPECT_EQ(h.reason, SLAM_FROZEN);
  }
  h.new_leg();
  EXPECT_FALSE(h.frozen);
  EXPECT_EQ(h.step(12.1, 12.09, 12.09, {}, true, true), "HOLD");   // still frozen: again at once
  EXPECT_EQ(h.reason, SLAM_FROZEN);
  h.new_leg();
  EXPECT_EQ(h.reason, "waiting for a fresh planner command");
  EXPECT_EQ(h.step(12.2, 12.19, 12.19, {}, true), "STREAM");
}

// ---- a frozen SLAM pose (L2 faults 2b, 2d) -----------------------------------------------------------

namespace {
// FAST-LIO at 10 Hz from t0 for n frames: repeated (frozen) or with a 0.1 mm jitter (real FAST-LIO never
// repeats: the smallest step in 4 recordings was 0.075 mm)
void poses(FrozenPoseCheck& f, double t0, int n, bool repeat, const Vec3& twist = Vec3::Zero()) {
  for (int i = 0; i < n; ++i) {
    const Vec3 p = repeat ? Vec3(1, 2, 3) : Vec3(1 + 1e-4 * std::sin(0.7 * (t0 + i)), 2 + 1e-4 * std::cos(0.3 * i), 3);
    f.on_pose(t0 + 0.1 * i, p, Quat{0, 0, 0.2, std::sqrt(1 - 0.04)}, twist);
  }
}
// run the check at 50 Hz over [t0, t1) with an extra input per tick -> the first time it says frozen (the
// poses are fed ahead: the newest is never older than max_age while they last)
template <class F>
std::optional<double> ticks(FrozenPoseCheck& f, double t0, double t1, F each) {
  for (double t = t0; t < t1; t += 0.02) {
    each(t);
    if (f.check(t)) return t;
  }
  return std::nullopt;
}
}  // namespace

TEST(Frozen, AMovingPoseIsNeverFrozen) {
  FrozenPoseCheck f;
  // poses and commands interleaved at 10 Hz / 50 Hz for 5 s, the vehicle commanded at 1 m/s
  for (int i = 0; i < 50; ++i) {
    const double t = 0.1 * i;
    f.on_pose(t, Vec3(1e-4 * i, 0, 0), Quat{}, Vec3(1, 0, 0));
    for (int k = 0; k < 5; ++k) {
      f.on_command(t + 0.02 * k, 1.0);
      f.on_gyro(t + 0.02 * k, 1.0);
      EXPECT_FALSE(f.check(t + 0.02 * k));
    }
  }
  EXPECT_EQ(f.repeats, 0);
}

TEST(Frozen, ARepeatWithoutEvidenceIsNotFrozen) {
  // SITL's parked or hovering pose (f2: repeated runs up to 47 s): no command, a still IMU, no twist
  FrozenPoseCheck f;
  poses(f, 0.0, 100, true);
  EXPECT_EQ(f.repeats, 99);
  for (double t = 9.9; t < 9.99; t += 0.02) {
    f.on_command(t, 0.1);   // under freeze_cmd_speed
    f.on_gyro(t, 0.05);     // under freeze_gyro_min
    EXPECT_FALSE(f.check(t));
  }
  EXPECT_EQ(f.evidence(9.95), "");
}

TEST(Frozen, RepeatedWhileCommandedIsFrozenAfterHoldS) {
  FrozenPoseCheck f(1e-5, 1e-6, 0.8, 4);
  poses(f, 0.0, 10, false);   // moving until 0.9
  // frozen from 1.0 on: the pose of 0.9 repeated each 0.1 s, the bridge streaming 1 m/s at 50 Hz
  std::optional<double> at;
  for (int i = 0; i < 30 && !at; ++i) {
    const double t = 1.0 + 0.1 * i;
    f.on_pose(t, Vec3(1, 2, 3), Quat{}, Vec3::Zero());
    if (i == 0) f.on_pose(t + 1e-3, Vec3(1, 2, 3), Quat{}, Vec3::Zero());   // a duplicate to start the run
    for (int k = 0; k < 5 && !at; ++k) {
      const double tk = t + 0.002 + 0.02 * k;
      f.on_command(tk, 1.0);
      if (auto e = f.check(tk)) {
        at = tk;
        EXPECT_EQ(*e, "command");
      }
    }
  }
  ASSERT_TRUE(at);
  EXPECT_NEAR(*at, 1.0 + 0.8, 0.03);   // freeze_hold_s after the evidence began with the run
}

TEST(Frozen, DrivingWaitsDriveHoldS) {
  // flying: hold_s 0.4; driving: drive_hold_s 0.8 (SITL's rover from rest)
  FrozenPoseCheck f(1e-5, 1e-6, 0.4, 4, 0.2, 0.5, 0.2, 0.2, 0.25, 0.5, 0.8);
  poses(f, 0.0, 80, true);
  auto fly = ticks(f, 2.0, 4.0, [&](double t) { f.on_command(t, 1.0); });
  ASSERT_TRUE(fly);
  EXPECT_NEAR(*fly, 2.4, 0.03);
  FrozenPoseCheck g(1e-5, 1e-6, 0.4, 4, 0.2, 0.5, 0.2, 0.2, 0.25, 0.5, 0.8);
  poses(g, 0.0, 80, true);
  std::optional<double> drv;
  for (double t = 2.0; t < 4.0 && !drv; t += 0.02) {
    g.on_drive_target(t, 2.0);
    if (g.check(t, true)) drv = t;
  }
  ASSERT_TRUE(drv);
  EXPECT_NEAR(*drv, 2.8, 0.03);
}

TEST(Frozen, NeedsMinNRepeats) {
  FrozenPoseCheck f(1e-5, 1e-6, 0.1, 4);
  f.on_command(0.0, 1.0);
  f.on_pose(0.0, Vec3(1, 2, 3), Quat{}, Vec3::Zero());
  f.on_pose(0.1, Vec3(1, 2, 3), Quat{}, Vec3::Zero());   // 1 repeat
  f.on_command(0.2, 1.0);
  EXPECT_FALSE(f.check(0.2));
  EXPECT_FALSE(f.check(0.25));   // 0.1 s of evidence, but 1 repeat
  for (int i = 2; i <= 4; ++i) f.on_pose(0.1 * i, Vec3(1, 2, 3), Quat{}, Vec3::Zero());
  f.on_command(0.4, 1.0);
  EXPECT_TRUE(f.check(0.4));
}

TEST(Frozen, NearIdenticalWithinEpsRepeatsAndARealStepDoesNot) {
  FrozenPoseCheck f(1e-5, 1e-6, 0.8, 4);
  f.on_pose(0.0, Vec3(1, 2, 3), Quat{0, 0, 0, 1}, Vec3::Zero());
  f.on_pose(0.1, Vec3(1 + 5e-6, 2, 3), Quat{0, 0, 5e-7, 1}, Vec3::Zero());
  EXPECT_EQ(f.repeats, 1);
  f.on_pose(0.2, Vec3(1 + 5e-6 + 7.5e-5, 2, 3), Quat{0, 0, 5e-7, 1}, Vec3::Zero());   // the smallest real step
  EXPECT_EQ(f.repeats, 0);
  f.on_pose(0.3, Vec3(1 + 5e-6 + 7.5e-5, 2, 3), Quat{0, 0, 1.6e-5, 1}, Vec3::Zero());  // a real rotation step
  EXPECT_EQ(f.repeats, 0);
}

TEST(Frozen, EvidenceMustLastWithoutABreak) {
  // SITL's rover from rest: the carrot comes, the pose repeats until it moves; a short burst is not enough
  FrozenPoseCheck f(1e-5, 1e-6, 0.8, 4);
  poses(f, 0.0, 80, true);
  EXPECT_FALSE(ticks(f, 2.9, 3.4, [&](double t) { f.on_drive_target(t, 2.0); }));   // 0.5 s commanded
  EXPECT_FALSE(ticks(f, 3.4, 4.0, [&](double) {}));                                  // a break: no evidence
  EXPECT_FALSE(f.evidence_from);
  // a carrot too close to the vehicle (pivoting on the spot) is not evidence
  EXPECT_FALSE(ticks(f, 4.0, 5.0, [&](double t) { f.on_drive_target(t, 0.3); }));
  // from here a carrot 2 m ahead without a break: frozen 0.8 s later
  auto at = ticks(f, 5.0, 7.0, [&](double t) { f.on_drive_target(t, 2.0); });
  ASSERT_TRUE(at);
  EXPECT_NEAR(*at, 5.8, 0.03);
}

TEST(Frozen, GyroAndTwistAreEvidenceAndAnOldGyroIsNot) {
  FrozenPoseCheck f(1e-5, 1e-6, 0.8, 4, 0.2, 0.5, 0.2, 0.2, 0.25);
  poses(f, 0.0, 80, true);
  auto at = ticks(f, 2.9, 4.0, [&](double t) { f.on_gyro(t, 0.5); });
  ASSERT_TRUE(at);
  EXPECT_EQ(*f.check(*at), "gyro");
  // the IMU stopped arriving: its last sample is not evidence after evidence_age
  FrozenPoseCheck g(1e-5, 1e-6, 0.8, 4);
  poses(g, 0.0, 80, true);
  g.on_gyro(2.9, 1.0);
  EXPECT_EQ(g.evidence(3.0), "gyro");
  EXPECT_EQ(g.evidence(3.2), "");
  // a pose that says it moves (its own twist) but repeats
  FrozenPoseCheck h(1e-5, 1e-6, 0.8, 4);
  poses(h, 0.0, 80, true, Vec3(0.5, 0, 0));
  EXPECT_EQ(h.evidence(2.95), "twist");
  EXPECT_TRUE(ticks(h, 2.9, 4.0, [&](double) {}));
}

TEST(Frozen, AStalledFcIsNotEvidence) {
  // SITL stalled under load (fz13): MAVROS silent 0.6 s, the pose repeating with its last twist, the bridge
  // still streaming the last command. The FC's IMU stream stopped with it: no evidence while it is stale
  FrozenPoseCheck f(1e-5, 1e-6, 0.4, 4);
  poses(f, 0.0, 80, true, Vec3(0.31, 0, 0));
  f.on_gyro(2.0, 0.0);   // the last IMU sample before the stall
  EXPECT_FALSE(ticks(f, 2.0, 2.8, [&](double t) { f.on_command(t, 1.0); }));
  // the stream back (a burst), the pose still repeating: the evidence clock starts again from here
  auto at = ticks(f, 2.8, 4.0, [&](double t) {
    f.on_gyro(t, 0.0);
    f.on_command(t, 1.0);
  });
  ASSERT_TRUE(at);
  EXPECT_NEAR(*at, 3.2, 0.03);
}

TEST(Frozen, AStoppedFeedIsTheStaleHoldsNotThis) {
  FrozenPoseCheck f(1e-5, 1e-6, 0.8, 4);
  poses(f, 0.0, 30, true);   // the last pose at 2.9
  EXPECT_FALSE(ticks(f, 3.5, 6.0, [&](double t) { f.on_command(t, 1.0); }));   // the newest pose older than max_age 0.5: none
}

TEST(Frozen, AMovingPoseEndsTheRun) {
  FrozenPoseCheck f(1e-5, 1e-6, 0.8, 4);
  poses(f, 0.0, 30, true);
  ticks(f, 2.9, 3.3, [&](double t) { f.on_command(t, 1.0); });
  EXPECT_TRUE(f.evidence_from);
  f.on_pose(3.0, Vec3(1.001, 2, 3), Quat{0, 0, 0.2, std::sqrt(1 - 0.04)}, Vec3::Zero());
  EXPECT_EQ(f.repeats, 0);
  EXPECT_FALSE(f.run_start);
  EXPECT_FALSE(f.evidence_from);
  EXPECT_FALSE(f.check(3.01));
}

// ---- FcPose (the FC pose's freshness, owner decision 2026-10-28) ------------------------------------

TEST(FcPose, NeverThenOk) {
  FcPoseGuard g(0.3, 5);
  EXPECT_EQ(g.check(10.0), "never");
  EXPECT_FALSE(g.age(10.0));
  g.on_pose(10.0, tag(1));
  EXPECT_EQ(g.check(10.1), "ok");
  EXPECT_NEAR(*g.age(10.1), 0.1, 1e-7);
}

TEST(FcPose, StaleKeepsTheLastGoodPose) {
  FcPoseGuard g(0.3, 5);
  for (int k = 0; k < 10; ++k) g.on_pose(10.0 + 0.02 * k, tag(k));
  EXPECT_EQ(g.check(10.3), "ok");   // 0.12 s old
  EXPECT_EQ(g.check(10.6), "stale");
  EXPECT_EQ(g.good->p[0], 9);
}

TEST(FcPose, ResumeNeedsNFreshPoses) {
  FcPoseGuard g(0.3, 5);
  g.on_pose(10.0, tag(-1));
  EXPECT_EQ(g.check(11.0), "stale");
  for (int k = 0; k < 4; ++k) {
    g.on_pose(11.0 + 0.02 * k, tag(100 + k));
    EXPECT_EQ(g.check(11.0 + 0.02 * k), "stale");
    EXPECT_EQ(g.good->p[0], -1);   // still the pose from before the gap
  }
  g.on_pose(11.08, tag(104));
  EXPECT_EQ(g.check(11.08), "ok");
  EXPECT_EQ(g.good->p[0], 104);
}

TEST(FcPose, AGapSeenOnlyOnArrivalIsStale) {
  // no check during the gap (the tick was late): the arrival after it still starts the count
  FcPoseGuard g(0.3, 3);
  g.on_pose(10.0, tag(1));
  g.on_pose(10.5, tag(2));
  EXPECT_EQ(g.check(10.5), "stale");
  EXPECT_EQ(g.good->p[0], 1);
}

TEST(FcPose, ATrickleStaysStale) {
  FcPoseGuard g(0.3, 5);
  g.on_pose(10.0, tag(1));
  for (int k = 1; k < 20; ++k) {
    const double t = 10.0 + 0.4 * k;   // 2.5 Hz: each pose fresh on arrival
    EXPECT_EQ(g.check(t - 0.01), "stale");
    g.on_pose(t, tag(2));
    EXPECT_EQ(g.check(t), "stale");
  }
  EXPECT_EQ(g.good->p[0], 1);
}

// ---- Health ---------------------------------------------------------------------------------------

static bool any_has(const std::vector<std::string>& v, const char* s) {
  for (const auto& x : v)
    if (x.find(s) != std::string::npos) return true;
  return false;
}

TEST(Health, FastlioHealth) {
  HealthCheck hc;
  hc.on_health(10.0, {100, 0, 0, 0, 20000, 0, 0, 80});
  EXPECT_TRUE(any_has(hc.faults(10.1), "matched"));
  hc.on_health(10.2, {2000, 0, 0, 0, 20000, 0, 0, 300});
  EXPECT_TRUE(any_has(hc.faults(10.3), "latency"));
  EXPECT_TRUE(hc.faults(11.0).empty());   // too old: ignored
}

TEST(Health, HeightCrossCheck) {
  HealthCheck hc(150.0, 5000.0, 250.0, 0.5);
  hc.on_range(10.0, 0.20, 0.25);   // learns fc z - range = 0.05
  hc.on_range(10.5, 2.00, 2.06);
  EXPECT_TRUE(hc.faults(10.6).empty());
  hc.on_range(11.0, 4.50, 2.10);   // log 233: SLAM says 2.1 m, the rangefinder 4.5 m
  EXPECT_TRUE(any_has(hc.faults(11.1), "rangefinder"));
}

// ---- flight yaw (owner rule 2026-10-29) -------------------------------------------------------------

TEST(FlightYaw, TravelHeadingAboveTheSpeedThreshold) {
  EXPECT_FALSE(travel_yaw(Vec3(0.2, 0.2, 1.0), 0.3));          // 0.28 m/s horizontal: slow (z ignored)
  EXPECT_NEAR(*travel_yaw(Vec3(0.0, 0.5, -0.3), 0.3), M_PI / 2, 1e-12);
  EXPECT_NEAR(*travel_yaw(Vec3(-1.0, 0.0, 0.0), 0.3), M_PI, 1e-12);
  EXPECT_NEAR(*travel_yaw(Vec3(1.0, -1.0, 0.0), 0.3), -M_PI / 4, 1e-12);
}

TEST(FlightYaw, RateLimitedStepTheShortWayRound) {
  EXPECT_NEAR(yaw_step(0.0, 1.0, 0.1), 0.1, 1e-12);
  EXPECT_NEAR(yaw_step(0.0, -1.0, 0.1), -0.1, 1e-12);
  EXPECT_NEAR(yaw_step(0.0, 0.05, 0.1), 0.05, 1e-12);          // within a step: there
  EXPECT_NEAR(yaw_step(3.1, -3.1, 0.1), wrap_pi(3.1 + 0.0831853071795865), 1e-9);   // across +-pi
  double y = 0.0;
  for (int i = 0; i < 50; ++i) y = yaw_step(y, M_PI / 2, 60.0 * M_PI / 180.0 / 50.0);   // 1 s at 60 deg/s, 50 Hz
  EXPECT_NEAR(y, M_PI / 3, 1e-9);
}

TEST(FlightYaw, StepSizedOnTheElapsedTime) {
  // Gate L2 b2: the step follows the time since the last tick, so the commanded rate is the cap however
  // the ticks fall; at most two ticks' worth after a stall, nothing for no time
  const double r = 60.0 * M_PI / 180.0;
  EXPECT_NEAR(yaw_max_step(r, 0.02, 0.04), r * 0.02, 1e-15);
  EXPECT_NEAR(yaw_max_step(r, 0.0173, 0.04), r * 0.0173, 1e-15);   // early tick: a smaller step
  EXPECT_NEAR(yaw_max_step(r, 0.5, 0.04), r * 0.04, 1e-15);        // a stall: capped
  EXPECT_EQ(yaw_max_step(r, -0.01, 0.04), 0.0);
  // ticks at jittered times: the yaw over the elapsed time never beats the rate
  double y = 0.0, t = 0.0;
  const double dts[] = {0.02, 0.0173, 0.0227, 0.001, 0.039, 0.02};
  for (double dt : dts) {
    const double y1 = yaw_step(y, M_PI, yaw_max_step(r, dt, 0.04));
    EXPECT_LE(std::abs(y1 - y) / dt, r * (1 + 1e-12));
    y = y1;
    t += dt;
  }
  EXPECT_NEAR(y, r * t, 1e-12);
}

// ---- the waypoint hold (Gate L2 b2) -------------------------------------------------------------------

TEST(WaypointHold, AtTheReachedGoalElseAtTheVehicle) {
  const Vec3 p_now(1.0, 2.0, 1.3);
  // the reached goal (within goal_max): there, not where the vehicle is
  const Vec3 g(1.3, 2.4, 1.3);   // 0.5 m off, the executive's accept radius
  EXPECT_TRUE(waypoint_hold_point(g, p_now, 1.0).isApprox(g));
  // no goal known, or one farther than goal_max: the vehicle's position
  EXPECT_TRUE(waypoint_hold_point(std::nullopt, p_now, 1.0).isApprox(p_now));
  EXPECT_TRUE(waypoint_hold_point(Vec3(4.0, 2.0, 1.3), p_now, 1.0).isApprox(p_now));
  EXPECT_TRUE(waypoint_hold_point(Vec3(1.0, 2.0, 2.31), p_now, 1.0).isApprox(p_now));   // 1.01 m up
}

TEST(WaypointHold, TheCappedTargetEasesOntoTheGoal) {
  // the hold goes out through cap_command with zero velocity: the FC's demand (fc_pos_p * lead) stays
  // under the speed cap, the target never runs past the goal
  const Vec3 p_now(0.0, 0.0, 1.0), g(0.5, 0.0, 1.0), z = Vec3::Zero();
  auto c = cap_command(g, z, z, p_now, 0.5, 1.0, 1.0);
  EXPECT_TRUE(c.v.isZero());
  EXPECT_TRUE(c.a.isZero());
  EXPECT_LE(1.0 * (c.p - p_now).norm(), 0.5 + 1e-12);
  EXPECT_GE((c.p - p_now).dot(g - p_now), 0.0);
  auto c2 = cap_command(g, z, z, Vec3(0.3, 0.0, 1.0), 1.0, 1.0, 1.0);   // within reach: the goal itself
  EXPECT_TRUE(c2.p.isApprox(g));
  EXPECT_TRUE(c2.v.isZero());
}

TEST(WaypointHold, DepartureOnlyWhenTheCommandSetsOffTowardTheGoal) {
  // Gate L2 b5, leg 2: held at (1.73, 0.17), next goal (4.27, 0.13); SUPER first commands backwards
  const Vec3 pt(1.73, 0.17, 1.05), g(4.27, 0.13, 1.05);
  EXPECT_FALSE(departed(pt, g, true, Vec3(2.09, 0.13, 1.05), Vec3(-0.32, 0.0, 0.0), 0.3));   // backwards
  EXPECT_FALSE(departed(pt, g, true, Vec3(1.34, 0.12, 1.05), Vec3(0.18, 0.0, 0.0), 0.3));    // turning: slow
  EXPECT_FALSE(departed(pt, g, true, Vec3(1.40, 0.12, 1.05), Vec3(0.45, 0.0, 0.0), 0.3));    // forward, but behind
  EXPECT_TRUE(departed(pt, g, true, Vec3(1.70, 0.12, 1.05), Vec3(0.55, 0.0, 0.0), 0.3));     // at the point, off
  EXPECT_TRUE(departed(pt, g, true, Vec3(1.80, 0.40, 1.05), Vec3(0.45, 0.45, 0.0), 0.3));    // round an obstacle: 45 deg off
  EXPECT_FALSE(departed(pt, g, true, Vec3(1.80, 0.40, 1.05), Vec3(0.0, 0.6, 0.0), 0.3));     // across the leg
  EXPECT_FALSE(departed(pt, g, true, Vec3(1.73, 0.17, 1.05), Vec3(0.0, 0.0, 0.8), 0.3));     // climbing only
  // no direction (no goal, or the goal at the point: the old goal still): only a goal that came after the clear
  EXPECT_TRUE(departed(pt, std::nullopt, false, pt, Vec3::Zero(), 0.3));   // no goal known: nothing to wait for
  EXPECT_FALSE(departed(pt, Vec3(1.80, 0.17, 1.05), false, pt, Vec3(0.5, 0, 0), 0.3));
  EXPECT_TRUE(departed(pt, Vec3(1.80, 0.17, 1.05), true, pt, Vec3::Zero(), 0.3));
}

// ---- the turn slowdown (owner 2026-10-29) ---------------------------------------------------------------

TEST(TurnSlow, FractionFullThenSmoothThenTheFloor) {
  const double d = M_PI / 180.0;
  EXPECT_EQ(turn_speed_frac(0.0, 15 * d, 45 * d, 0.25), 1.0);
  EXPECT_EQ(turn_speed_frac(15 * d, 15 * d, 45 * d, 0.25), 1.0);
  EXPECT_NEAR(turn_speed_frac(30 * d, 15 * d, 45 * d, 0.25), 0.625, 1e-12);   // the midpoint: half way down
  EXPECT_EQ(turn_speed_frac(45 * d, 15 * d, 45 * d, 0.25), 0.25);
  EXPECT_EQ(turn_speed_frac(M_PI, 15 * d, 45 * d, 0.25), 0.25);
  // monotone, and smooth at both ends (no step: within 0.002 of the ends 0.1 deg inside them)
  double prev = 1.0;
  for (double e = 0; e <= 60; e += 0.5) {
    const double f = turn_speed_frac(e * d, 15 * d, 45 * d, 0.25);
    EXPECT_LE(f, prev + 1e-12);
    prev = f;
  }
  EXPECT_GT(turn_speed_frac(15.1 * d, 15 * d, 45 * d, 0.25), 0.998);
  EXPECT_LT(turn_speed_frac(44.9 * d, 15 * d, 45 * d, 0.25), 0.252);
}

TEST(TurnSlow, HeadingErrorNowAndAhead) {
  const double d = M_PI / 180.0;
  // flying +x with the nose at +x: no error; slower than speed_min: no travel direction
  EXPECT_NEAR(*turn_heading_error(0.0, Vec3(1, 0, 0), Vec3::Zero(), 0.5, 0.3), 0.0, 1e-12);
  EXPECT_FALSE(turn_heading_error(0.0, Vec3(0.2, 0.1, 0), Vec3(0, 2, 0), 0.5, 0.3));
  // the nose 30 deg off the velocity, either side, the short way round
  EXPECT_NEAR(*turn_heading_error(30 * d, Vec3(1, 0, 0), Vec3::Zero(), 0.5, 0.3), 30 * d, 1e-12);
  EXPECT_NEAR(*turn_heading_error(wrap_pi(M_PI - 10 * d), Vec3(-1, 0, 0), Vec3::Zero(), 0.5, 0.3), 10 * d, 1e-12);
  // a corner coming: +x at 1 m/s turning +y at 2 m/s^2 -> 0.5 s ahead (1, 1): 45 deg, before any error now
  EXPECT_NEAR(*turn_heading_error(0.0, Vec3(1, 0, 0), Vec3(0, 2, 0), 0.5, 0.3), 45 * d, 1e-12);
  EXPECT_NEAR(*turn_heading_error(0.0, Vec3(1, 0, 0), Vec3(0, 2, 0), 0.0, 0.3), 0.0, 1e-12);   // no lookahead
  // braking along the line (a against v) is no corner; braking to (nearly) a stop drops the ahead term
  EXPECT_NEAR(*turn_heading_error(0.0, Vec3(1, 0, 0), Vec3(-1, 0, 0), 0.5, 0.3), 0.0, 1e-12);
  EXPECT_NEAR(*turn_heading_error(0.0, Vec3(1, 0, 0), Vec3(-1.9, 0.1, 0), 0.5, 0.3), 0.0, 1e-12);
  // vertical velocity and acceleration play no part
  EXPECT_NEAR(*turn_heading_error(0.0, Vec3(1, 0, 3), Vec3(0, 0, 5), 0.5, 0.3), 0.0, 1e-12);
}

TEST(TurnSlow, ReplayRate) {
  // SUPER's pace when allowed; slower in proportion when not; never negative
  EXPECT_EQ(replay_rate(1.0, 1.0, 0.0, 1.5, 1.5), 1.0);
  EXPECT_NEAR(replay_rate(0.25, 1.0, 0.0, 1.5, 1.5), 0.25, 1e-12);
  EXPECT_NEAR(replay_rate(0.25, 0.5, 0.3, 1.5, 1.5), 0.5, 1e-12);
  EXPECT_EQ(replay_rate(-1.0, 1.0, 0.0, 1.5, 1.5), 0.0);
  // hovering (no speed): SUPER's pace, or catching up when behind
  EXPECT_EQ(replay_rate(0.25, 0.0, 0.0, 1.5, 1.5), 1.0);
  EXPECT_EQ(replay_rate(0.25, 0.0, 0.5, 1.5, 1.5), 1.5);
  // behind: catch up as far as the allowed speed permits
  EXPECT_NEAR(replay_rate(1.0, 0.8, 0.5, 1.5, 1.5), 1.25, 1e-12);
  EXPECT_EQ(replay_rate(1.0, 0.5, 0.5, 1.5, 1.5), 1.5);
  EXPECT_EQ(replay_rate(1.0, 0.5, 0.03, 1.5, 1.5), 1.0);   // within REPLAY_LAG_MIN: at SUPER's pace
  // at the lag limit: no slower than SUPER; within 0.5 s of it, the floor rising linearly to SUPER's pace
  EXPECT_EQ(replay_rate(0.25, 1.0, 1.5, 1.5, 1.5), 1.0);
  EXPECT_EQ(replay_rate(0.25, 1.0, 2.0, 1.5, 1.5), 1.0);
  EXPECT_NEAR(replay_rate(0.25, 1.0, 1.25, 1.5, 1.5), 0.5, 1e-12);
  EXPECT_NEAR(replay_rate(0.25, 1.0, 1.0, 1.5, 1.5), 0.25, 1e-12);
}

TEST(TurnSlow, RefBufferInterpolatesClampsAndTrims) {
  RefBuffer b;
  EXPECT_TRUE(b.empty());
  b.push({1.0, Vec3(0, 0, 1), Vec3(1, 0, 0), Vec3(0, 0, 0), 0.1});
  b.push({1.1, Vec3(0.1, 0, 1), Vec3(1, 0.2, 0), Vec3(0, 2, 0), 0.2});
  b.push({1.05, Vec3(9, 9, 9), Vec3(9, 9, 9), Vec3(9, 9, 9), 9.0});   // out of order: dropped
  b.push({1.2, Vec3(0.2, 0.01, 1), Vec3(1, 0.4, 0), Vec3(0, 2, 0), 0.3});
  EXPECT_EQ(b.size(), 3u);
  EXPECT_EQ(b.newest(), 1.2);
  auto s = b.at(1.05);
  EXPECT_NEAR(s.p[0], 0.05, 1e-12);
  EXPECT_NEAR(s.v[1], 0.1, 1e-12);
  EXPECT_NEAR(s.a[1], 1.0, 1e-12);
  EXPECT_EQ(s.yaw, 0.1);   // the earlier sample's
  EXPECT_NEAR(b.at(0.5).p[0], 0.0, 1e-12);   // before the oldest: the oldest
  EXPECT_NEAR(b.at(2.0).p[0], 0.2, 1e-12);   // after the newest: the newest
  b.trim(1.15);   // keeps the last sample at or before 1.15, so 1.15 still interpolates
  EXPECT_EQ(b.size(), 2u);
  EXPECT_EQ(b.oldest(), 1.1);
  EXPECT_NEAR(b.at(1.15).p[0], 0.15, 1e-12);
  b.trim(5.0);
  EXPECT_EQ(b.size(), 1u);
}

TEST(TurnSlow, ReplayedCornerStaysOnThePathCappedChaseCutsIt) {
  // SUPER flies a 90 deg corner at 1 m/s (+x to (2, 0), then +y), its command at 100 Hz. A vehicle allowed
  // 0.25 m/s through it: replaying SUPER's reference at replay_rate keeps every target on SUPER's path;
  // chasing SUPER's newest command with the cap alone (cap_command at 0.25 m/s, 1 m lead) aims across the
  // corner
  RefBuffer b;
  auto ref = [](double t) { return t <= 2.0 ? Vec3(t, 0, 1) : Vec3(2.0, t - 2.0, 1); };
  auto vel = [](double t) { return t <= 2.0 ? Vec3(1, 0, 0) : Vec3(0, 1, 0); };
  auto off_path = [](const Vec3& p) {   // distance from the corner path
    const double dx = p[0] <= 2.0 ? std::abs(p[1]) : 1e9, dy = p[1] >= 0.0 ? std::abs(p[0] - 2.0) : 1e9;
    return std::min({dx, dy, (p.head<2>() - Eigen::Vector2d(2, 0)).norm()});
  };
  double tau = 1.5;
  Vec3 veh = ref(1.5);   // the vehicle on the path at the corner's approach
  double worst_replay = 0, worst_chase = 0;
  for (double t = 1.5; t < 4.0; t += 0.01) {
    b.push({t, ref(t), vel(t), Vec3::Zero(), 0.0});
    const auto s0 = b.at(tau);
    const double rate = replay_rate(0.25, s0.v.norm(), b.newest() - tau, 1.5, 100.0);
    tau = std::min(tau + rate * 0.01, b.newest());
    const auto s = b.at(tau);
    worst_replay = std::max(worst_replay, off_path(s.p));
    veh = s.p;   // the vehicle tracks the replayed reference
    auto c = cap_command(ref(t), vel(t), Vec3::Zero(), veh, 0.25, 1.0, 1.0);
    worst_chase = std::max(worst_chase, off_path(c.p));
  }
  EXPECT_LT(worst_replay, 0.01);   // on the path (interpolation between two 10 ms samples at the corner)
  EXPECT_GT(worst_chase, 0.05);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
