// l4_common: the JSON written is what Python's json.dumps writes for the same values.
#include <gtest/gtest.h>

#include <l4_common/pyjson.h>

using l4::ojson;

TEST(PyJson, FloatRepr) {
  // expected strings are Python 3's repr()
  EXPECT_EQ(l4::py_float_repr(1.0), "1.0");
  EXPECT_EQ(l4::py_float_repr(0.5), "0.5");
  EXPECT_EQ(l4::py_float_repr(0.1), "0.1");
  EXPECT_EQ(l4::py_float_repr(-0.45), "-0.45");
  EXPECT_EQ(l4::py_float_repr(0.1 + 0.2), "0.30000000000000004");
  EXPECT_EQ(l4::py_float_repr(1e-5), "1e-05");
  EXPECT_EQ(l4::py_float_repr(0.0001), "0.0001");
  EXPECT_EQ(l4::py_float_repr(1.5e-7), "1.5e-07");
  EXPECT_EQ(l4::py_float_repr(1e16), "1e+16");
  EXPECT_EQ(l4::py_float_repr(1e15), "1000000000000000.0");
  EXPECT_EQ(l4::py_float_repr(123456.789), "123456.789");
  EXPECT_EQ(l4::py_float_repr(1793289879.83), "1793289879.83");
  EXPECT_EQ(l4::py_float_repr(-0.0), "-0.0");
  EXPECT_EQ(l4::py_float_repr(0.0), "0.0");
  EXPECT_EQ(l4::py_float_repr(2.5e-300), "2.5e-300");
}

TEST(PyJson, Round) {
  EXPECT_EQ(l4::py_round(2.675, 2), 2.67);          // the binary value is below 2.675
  EXPECT_EQ(l4::py_round(0.125, 2), 0.12);          // an exact tie: to even
  EXPECT_EQ(l4::py_round(0.375, 2), 0.38);
  EXPECT_EQ(l4::py_round(1.23456, 3), 1.235);
  EXPECT_EQ(l4::py_round(-0.0004, 3), -0.0);
  EXPECT_EQ(l4::py_float_repr(l4::py_round(-0.0004, 3)), "-0.0");
}

TEST(PyJson, Dumps) {
  ojson j;
  j["state"] = "STREAM";
  j["reason"] = "";
  j["slam_hold"] = false;
  j["cmd_age"] = 0.012;
  j["planner_node"] = nullptr;
  j["anchor"] = {{"psi_deg", -0.45}, {"t", {0.0, 1.5, -2}}};
  j["targets_sent"] = 12;
  j["empty"] = ojson::array();
  j["obj"] = ojson::object();
  EXPECT_EQ(l4::dumps(j),
            "{\"state\": \"STREAM\", \"reason\": \"\", \"slam_hold\": false, \"cmd_age\": 0.012, \"planner_node\": null, "
            "\"anchor\": {\"psi_deg\": -0.45, \"t\": [0.0, 1.5, -2]}, \"targets_sent\": 12, \"empty\": [], \"obj\": {}}");
  EXPECT_EQ(l4::dumps(ojson("a\"b\\c\nd\x01\x7f")), "\"a\\\"b\\\\c\\nd\\u0001\\u007f\"");
  EXPECT_EQ(l4::dumps(ojson("\xc2\xb0")), "\"\\u00b0\"");    // ensure_ascii
}

TEST(PyJson, LoadsAndTruthiness) {
  ojson j = l4::loads("{\"a\": NaN, \"b\": [Infinity, -Infinity], \"c\": \"NaN\"}");
  ASSERT_TRUE(j.is_object());
  EXPECT_TRUE(std::isnan(j["a"].get<double>()));
  EXPECT_TRUE(std::isinf(j["b"][0].get<double>()));
  EXPECT_EQ(j["c"], "NaN");
  EXPECT_TRUE(l4::loads("{not json").is_discarded());
  EXPECT_FALSE(l4::truthy(ojson(0.0)));
  EXPECT_FALSE(l4::truthy(ojson("")));
  EXPECT_FALSE(l4::truthy(ojson(nullptr)));
  EXPECT_TRUE(l4::truthy(ojson(2)));
  EXPECT_EQ(l4::get(ojson::object(), "x", true), ojson(true));
  EXPECT_EQ(l4::get(ojson::array(), "x", 5), ojson(5));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
