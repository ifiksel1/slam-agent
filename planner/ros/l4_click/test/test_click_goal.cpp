// click -> goal conversions and the marker's state (l4_click/click_goal.h)
#include <gtest/gtest.h>

#include <cmath>

#include "l4_click/click_goal.h"

using l4_click::Anchor;
using l4_click::Vec3;

static Anchor yawed(double yaw, const Vec3& t) { return Anchor::from(0, 0, std::sin(yaw / 2), std::cos(yaw / 2), t); }

TEST(ClickGoal, DriveGoalIsTheClickInTheFcFrame) {
  const Anchor a = yawed(M_PI / 2, Vec3(1.0, 2.0, 0.5));
  const l4::ojson g = l4_click::drive_goal(Vec3(3.0, 0.0, -0.3), a);
  EXPECT_EQ(g["type"], "drive");
  ASSERT_EQ(g["goal"].size(), 2u);   // no height: the executive drives at the vehicle's
  EXPECT_NEAR(g["goal"][0].get<double>(), 1.0, 1e-9);   // Rz(90) (3, 0) = (0, 3), + (1, 2)
  EXPECT_NEAR(g["goal"][1].get<double>(), 5.0, 1e-9);
}

TEST(ClickGoal, FlyGoalIsHeightAboveTheFloorWithTheHeadingInFc) {
  const Anchor a = yawed(M_PI / 2, Vec3(1.0, 2.0, 0.5));
  const l4::ojson g = l4_click::fly_goal(Vec3(3.0, 0.0, 0.0), 0.0, a, 0.2, 1.0);
  EXPECT_EQ(g["type"], "fly");
  ASSERT_EQ(g["goal"].size(), 3u);
  EXPECT_NEAR(g["goal"][0].get<double>(), 1.0, 1e-9);
  EXPECT_NEAR(g["goal"][1].get<double>(), 5.0, 1e-9);
  EXPECT_NEAR(g["goal"][2].get<double>(), 1.2, 1e-9);   // floor 0.2 + 1.0, whatever the click's z
  EXPECT_NEAR(g["yaw"].get<double>(), 90.0, 1e-9);      // planner heading 0 -> fc 90 deg
  const l4::ojson h = l4_click::fly_goal(Vec3(0, 0, 0), M_PI * 0.75, a, 0.0, 1.0);
  EXPECT_NEAR(h["yaw"].get<double>(), -135.0, 1e-9);    // wrapped
  EXPECT_FALSE(l4_click::fly_goal(Vec3(0, 0, 0), 0.0, a, 0.0, 1.0, false).contains("yaw"));
}

TEST(ClickGoal, RoundTrip) {
  const Anchor a = Anchor::from(0.01, -0.02, 0.3, 0.95, Vec3(-1.0, 4.0, 0.2));
  Anchor n = a;
  // a non-unit quaternion is not what the bridge sends; normalise for the test
  const double q = std::sqrt(0.01 * 0.01 + 0.02 * 0.02 + 0.3 * 0.3 + 0.95 * 0.95);
  n = Anchor::from(0.01 / q, -0.02 / q, 0.3 / q, 0.95 / q, Vec3(-1.0, 4.0, 0.2));
  const Vec3 p(2.0, -3.0, 1.0);
  EXPECT_LT((n.to_planner(n.to_fc(p)) - p).norm(), 1e-9);
}

TEST(ClickGoal, MarkerStateFromTheExecutive) {
  using l4_click::goal_state;
  EXPECT_FALSE(goal_state({{"goal_received", 3}}, 3));
  EXPECT_EQ(*goal_state({{"phase", "GOAL"}, {"goal", 3}}, 3), "active");
  EXPECT_EQ(*goal_state({{"phase", "LEG"}, {"leg", 3}}, 3), "active");
  EXPECT_FALSE(goal_state({{"phase", "LEG"}, {"leg", 2}}, 3));
  EXPECT_EQ(*goal_state({{"phase", "REACHED"}, {"goal", 3}}, 3), "reached");
  EXPECT_EQ(*goal_state({{"phase", "UNREACHABLE"}, {"goal", 3}}, 3), "failed");
  EXPECT_EQ(*goal_state({{"phase", "REFUSED"}, {"goal", 3}}, 3), "failed");
  EXPECT_EQ(*goal_state({{"phase", "PREEMPTED"}, {"goal", 3}}, 3), "replaced");
  EXPECT_EQ(*goal_state({{"phase", "STOPPED"}, {"reason", "planner died"}}, 3), "stopped");
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
