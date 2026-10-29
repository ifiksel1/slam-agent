// The mission file (D6) and QuadRover's mode numbers, as the former mission_executive.py read them.
#include <gtest/gtest.h>

#include <dirent.h>

#include <cmath>

#include "l4_executive/mission.h"

using namespace l4_executive;

TEST(Mission, ModeNumbers) {
  EXPECT_EQ(mode_number("GUIDED"), 15);
  EXPECT_EQ(mode_number("CMODE(19)"), 19);
  EXPECT_EQ(mode_number("CMODE(23)"), 23);
  EXPECT_FALSE(mode_number("CMODE(x)"));
  EXPECT_FALSE(mode_number("STABILIZE"));
  EXPECT_FALSE(mode_number(""));
  EXPECT_TRUE(allowed_mode("GUIDED") && allowed_mode("QRLOITER") && allowed_mode("QRLAND") && allowed_mode("QRSMARTRTL"));
  EXPECT_FALSE(allowed_mode("QRSTABILIZE") || allowed_mode("MANUAL") || allowed_mode("AUTO") || allowed_mode("HOLD"));
}

TEST(Mission, DefaultsAndScalarTypes) {
  Mission m = load_mission_text(
      "name: t\nframe: fc\ntakeoff_height: 1\ndefaults: {speed: 1, accept_radius: 0.3, on_unreachable: stop}\n"
      "legs:\n  - {type: fly, goal: [1, 2, 1.5]}\n  - {type: land, at: [0.0, 1.0], speed: 0.4}\n"
      "  - {type: drive, goal: [3, 4]}\n  - {type: takeoff, height: 1.2}\n",
      "/x.yaml");
  EXPECT_EQ(m.name, "t");
  EXPECT_EQ(m.takeoff_height, 1.0);
  ASSERT_EQ(m.legs.size(), 4u);
  EXPECT_EQ(l4::dumps(m.legs[0]["speed"]), "1");         // PyYAML: an int, echoed as an int
  EXPECT_EQ(l4::dumps(m.legs[1]["speed"]), "0.4");
  EXPECT_EQ(l4::dumps(m.legs[0]["leg_timeout_s"]), "90.0");
  EXPECT_EQ(m.legs[0]["on_unreachable"], "stop");
  EXPECT_EQ(l4::dumps(m.legs[3]["height"]), "1.2");
  // key order: the defaults first, then the leg's own keys (Python dict(d); update(leg))
  EXPECT_EQ(l4::dumps(m.legs[2]),
            "{\"speed\": 1, \"accept_radius\": 0.3, \"leg_timeout_s\": 90.0, \"no_progress_s\": 20.0, \"hold_s\": 0.0, "
            "\"on_unreachable\": \"stop\", \"type\": \"drive\", \"goal\": [3, 4]}");
  Mission n = load_mission_text("legs: []\n", "/p/m.yaml");
  EXPECT_EQ(n.name, "/p/m.yaml");
  EXPECT_EQ(n.takeoff_height, 1.0);
}

TEST(Mission, Malformed) {
  EXPECT_THROW(load_mission_text("frame: map\nlegs: []\n", "x"), std::invalid_argument);
  EXPECT_THROW(load_mission_text("legs:\n  - {type: hover}\n", "x"), std::invalid_argument);
  EXPECT_THROW(load_mission_text("legs:\n  - {type: fly, goal: [1, 2]}\n", "x"), std::invalid_argument);
  EXPECT_THROW(load_mission_text("legs:\n  - {type: drive, goal: [1]}\n", "x"), std::invalid_argument);
  EXPECT_THROW(load_mission_text("legs:\n  - {type: land}\n", "x"), std::invalid_argument);
  EXPECT_THROW(load_mission_text("legs:\n  - {type: takeoff}\n", "x"), std::invalid_argument);
  EXPECT_THROW(load_mission_text("legs:\n  - {type: fly, goal: [1, 2, 3], on_unreachable: retry}\n", "x"),
               std::invalid_argument);
  EXPECT_THROW(load_mission_text("name: x\n", "x"), std::invalid_argument);
}

TEST(Mission, EveryMissionInTheRepoLoads) {
  DIR* d = opendir(MISSIONS_DIR);
  ASSERT_TRUE(d);
  int n = 0;
  while (dirent* e = readdir(d)) {
    std::string f = e->d_name;
    if (f.size() < 5 || f.substr(f.size() - 5) != ".yaml") continue;
    EXPECT_NO_THROW(load_mission(std::string(MISSIONS_DIR) + "/" + f)) << f;
    ++n;
  }
  closedir(d);
  EXPECT_GT(n, 5);
}

TEST(Mission, ScanLegAndLegYaw) {
  Mission m = load_mission_text("legs:\n  - {type: fly, goal: [1, 0, 1], yaw: -90}\n  - {type: scan, yaw_sweep_deg: 360}\n", "x");
  EXPECT_EQ(m.legs[1]["type"], "scan");
  EXPECT_EQ(num(m.legs[0]["yaw"]), -90.0);
  EXPECT_THROW(load_mission_text("legs:\n  - {type: scan}\n", "x"), std::invalid_argument);
  EXPECT_THROW(load_mission_text("legs:\n  - {type: fly, goal: [1, 0, 1], yaw: east}\n", "x"), std::invalid_argument);
}

TEST(Settle, NeedsAFullWindowStillAndOnTarget) {
  using V = Eigen::Vector3d;
  SettleWindow w(1.0);
  const double r5 = 5.0 * M_PI / 180.0;
  for (int i = 0; i <= 10; ++i) {
    w.add(i * 0.1, V(0, 0, 1), 0.5);
    EXPECT_EQ(w.settled(0.1, r5), i == 10) << i;   // settled only once the samples cover 1 s
  }
  EXPECT_TRUE(w.settled(0.1, r5, 0.5 + 0.1, 0.2));
  EXPECT_FALSE(w.settled(0.1, r5, 0.5 + 0.3, 0.2));   // off the waypoint yaw
  // turning at 10 deg/s: not settled; at 3 deg/s: settled
  SettleWindow turn(1.0), slow(1.0);
  for (int i = 0; i <= 20; ++i) {
    turn.add(i * 0.1, V(0, 0, 1), i * 0.1 * 10.0 * M_PI / 180.0);
    slow.add(i * 0.1, V(0, 0, 1), i * 0.1 * 3.0 * M_PI / 180.0);
  }
  EXPECT_FALSE(turn.settled(0.1, r5));
  EXPECT_TRUE(slow.settled(0.1, r5));
  // drifting at 0.2 m/s: not hovering
  SettleWindow drift(1.0);
  for (int i = 0; i <= 20; ++i) drift.add(i * 0.1, V(0.02 * i, 0, 1), 0.0);
  EXPECT_FALSE(drift.settled(0.1, r5));
  // across +-pi
  SettleWindow wrap(1.0);
  for (int i = 0; i <= 10; ++i) wrap.add(i * 0.1, V(0, 0, 1), (i % 2 ? 1 : -1) * (M_PI - 0.01));
  EXPECT_TRUE(wrap.settled(0.1, r5, M_PI, 0.1));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
