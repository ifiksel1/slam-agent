#!/usr/bin/env python3
"""Tests for the planner bridge.

  python3 test_planner_bridge.py           core tests (numpy only, no ROS)
  python3 test_planner_bridge.py --ros     also the node tests: needs a roscore, rospy, mavros_msgs and
                                           quadrotor_msgs (the l4-sim image with SUPER's workspace sourced)
"""
import math
import os
import subprocess
import sys
import time
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import bridge_core as bc  # noqa: E402

# the real SLAM-to-FC bridge's frame anchor, for the cross-checks
sys.path.insert(0, os.path.join(HERE, "..", "..", "overrides", "FAST-LIO", "src"))
try:
    import frame_anchor as fa
except ImportError:
    fa = None


def rand_quat(rng):
    q = rng.normal(size=4)
    q /= np.linalg.norm(q)
    return tuple(q)


def quat_rotate(q, v):
    p = bc.quat_mul(bc.quat_mul(q, (v[0], v[1], v[2], 0.0)), bc.quat_conj(q))
    return np.array(p[:3])


class Frames(unittest.TestCase):
    def test_position_matrix_is_the_bridges(self):
        # raw +z forward -> ENU x, raw +x left -> ENU y, raw +y up -> ENU z
        np.testing.assert_allclose(bc.raw_to_planner_pos([0, 0, 1]), [1, 0, 0])
        np.testing.assert_allclose(bc.raw_to_planner_pos([1, 0, 0]), [0, 1, 0])
        np.testing.assert_allclose(bc.raw_to_planner_pos([0, 1, 0]), [0, 0, 1])
        self.assertAlmostEqual(np.linalg.det(bc.POSITION_MATRIX), 1.0)

    def test_orientation_is_the_similarity_transform(self):
        # rotating a vector with the converted quaternion = converting the rotated raw vector
        rng = np.random.default_rng(1)
        for _ in range(50):
            q = rand_quat(rng)
            v = rng.normal(size=3)
            lhs = quat_rotate(bc.raw_to_planner_quat(q), bc.raw_to_planner_pos(v))
            rhs = bc.raw_to_planner_pos(quat_rotate(q, v))
            np.testing.assert_allclose(lhs, rhs, atol=1e-9)

    def test_cloud_matches_positions(self):
        pts = np.random.default_rng(2).normal(size=(100, 3))
        np.testing.assert_allclose(bc.raw_to_planner_points(pts), [bc.raw_to_planner_pos(p) for p in pts])


class AnchorTests(unittest.TestCase):
    def setUp(self):
        self.rng = np.random.default_rng(3)

    def check_anchor_recovered(self, psi, t, mount_yaw=0.0):
        p = self.rng.normal(size=3) * 5
        q = rand_quat(self.rng)
        # what the SLAM bridge publishes: anchor(position); anchor(orientation * mount)
        q_m = bc.quat_mul(q, bc.quat_yaw(-mount_yaw))
        if fa is not None:
            A = fa.Anchor(psi, t)
            p_fc, q_fc = A.apply_pos(p), A.apply_quat(q_m)
        else:
            A = bc.Anchor(psi, t)
            p_fc, q_fc = A.pos(p), bc.quat_mul(bc.quat_yaw(psi), q_m)
        got = bc.Anchor.from_pair(p, q, p_fc, q_fc, mount_yaw)
        self.assertAlmostEqual(bc.wrap_pi(got.psi - psi), 0.0, places=9)
        np.testing.assert_allclose(got.t, t, atol=1e-9)
        # and it maps another point the same way as the bridge would
        p2 = self.rng.normal(size=3) * 5
        np.testing.assert_allclose(got.pos(p2), A.apply_pos(p2) if fa is not None else A.pos(p2), atol=1e-9)
        np.testing.assert_allclose(got.inverse_pos(got.pos(p2)), p2, atol=1e-9)

    def test_recovers_the_bridges_anchor(self):
        for _ in range(20):
            self.check_anchor_recovered(self.rng.uniform(-math.pi, math.pi), self.rng.normal(size=3) * 3)

    def test_mount_yaw_does_not_bias_it(self):
        self.check_anchor_recovered(0.7, [1.0, -2.0, 0.3], mount_yaw=math.radians(-0.8))

    def test_tracker_jump_needs_to_persist(self):
        tr = bc.AnchorTracker(jump_m=0.1, jump_deg=2.0, persist=3)
        p, q = np.array([1.0, 2.0, 1.0]), bc.quat_yaw(0.3)
        A = bc.Anchor(0.5, [1.0, 0.0, 0.0])
        def pair(anchor):
            return p, q, anchor.pos(p), bc.quat_mul(bc.quat_yaw(anchor.psi), q)
        for _ in range(5):
            self.assertIsNone(tr.update(*pair(A))[1])
        B = bc.Anchor(0.5, [1.5, 0.0, 0.0])
        self.assertIsNone(tr.update(*pair(B))[1])     # one odd pair: a glitch
        self.assertIsNone(tr.update(*pair(A))[1])
        reasons = [tr.update(*pair(B))[1] for _ in range(3)]
        self.assertIsNone(reasons[0]); self.assertIsNone(reasons[1])
        self.assertIn("moved", reasons[2])
        np.testing.assert_allclose(tr.anchor.t, B.t)
        C = bc.Anchor(0.5 + math.radians(5), B.t)
        reasons = [tr.update(*pair(C))[1] for _ in range(3)]
        self.assertIn("yaw", reasons[2])


class Output(unittest.TestCase):
    def test_speed_cap_scales_velocity_and_acceleration(self):
        p, v, a, capped = bc.cap_command([1, 0, 1], [2, 0, 0], [1, 1, 0], [0.5, 0, 1], 0.5, 1.0)
        self.assertTrue(capped)
        self.assertAlmostEqual(np.linalg.norm(v), 0.5)
        np.testing.assert_allclose(a, [0.25, 0.25, 0])
        np.testing.assert_allclose(p, [1, 0, 1])

    def test_lead_is_clamped(self):
        p, v, a, capped = bc.cap_command([5, 0, 1], [0.3, 0, 0], [0, 0, 0], [0, 0, 1], 0.5, 1.0)
        self.assertTrue(capped)
        np.testing.assert_allclose(p, [1, 0, 1])

    def test_under_the_cap_unchanged(self):
        p, v, a, capped = bc.cap_command([0.3, 0, 1], [0.4, 0, 0], [0.1, 0, 0], [0, 0, 1], 0.5, 1.0)
        self.assertFalse(capped)
        np.testing.assert_allclose(v, [0.4, 0, 0])

    def test_cap_bounds_the_fc_demand(self):
        # lagging 0.6 m behind a 1.0 m/s reference: the FC would fly 1.0 + 1.0 * 0.6 = 1.6 m/s
        p, v, a, capped = bc.cap_command([0.6, 0, 1], [1.0, 0, 0], [0.5, 0, 0], [0, 0, 1], 1.0, 1.0, 1.0)
        self.assertTrue(capped)
        np.testing.assert_allclose(p, [0.6, 0, 1])                       # the correction is kept
        self.assertAlmostEqual(np.linalg.norm(v + 1.0 * (p - [0, 0, 1])), 1.0)
        np.testing.assert_allclose(a, [0.2, 0, 0], atol=1e-9)            # scaled with the velocity
        # a lateral error: the demand is still within the cap
        p, v, a, _ = bc.cap_command([0.2, 0.5, 1], [1.0, 0, 0], [0, 0, 0], [0, 0, 1], 1.0, 1.0, 1.0)
        self.assertLessEqual(np.linalg.norm(v + (p - [0, 0, 1])), 1.0 + 1e-9)
        # the correction alone over the cap: no feed-forward, a lead of cap / gain
        p, v, a, _ = bc.cap_command([2.0, 0, 1], [1.0, 0, 0], [0, 0, 0], [0, 0, 1], 1.0, 1.5, 1.0)
        np.testing.assert_allclose(v, 0)
        np.testing.assert_allclose(p, [1.0, 0, 1])
        # under the cap: unchanged
        p, v, a, capped = bc.cap_command([0.2, 0, 1], [0.5, 0, 0], [0, 0, 0], [0, 0, 1], 1.0, 1.0, 1.0)
        self.assertFalse(capped)
        np.testing.assert_allclose(v, [0.5, 0, 0])

    def test_drive_target_within_the_lookahead(self):
        out = bc.drive_target([3, 0, 0.2], [1, 0, 0], [0, 0, 0.2], 0.5)
        np.testing.assert_allclose(out, [0.5, 0, 0.2])
        near = bc.drive_target([0.1, 0.1, 0.2], [0, 0, 0], [0, 0, 0.2], 0.5)
        self.assertLessEqual(np.linalg.norm(near[:2]), 0.5)

    def test_drive_target_on_a_track_planner_carrot(self):
        # the track planner's carrot: position + yaw, zero velocity; 0.5 m ahead of its odometry
        # point, which is ~0.18 m ahead of the FC: the target is the lookahead toward it, at the FC's z
        out = bc.drive_target([0.68, 0.0, 0.25], [0, 0, 0], [0.0, 0.0, 0.05], 0.5)
        np.testing.assert_allclose(out, [0.5, 0.0, 0.05], atol=1e-9)
        out = bc.drive_target([0.2, 0.2, 0.25], [0, 0, 0], [0.0, 0.0, 0.05], 0.5)
        np.testing.assert_allclose(out, [0.2, 0.2, 0.05], atol=1e-9)

    def test_geofence(self):
        f = bc.Geofence([-1, -1, 0], [1, 1, 2])
        self.assertTrue(f.inside([0, 0, 1]))
        self.assertFalse(f.inside([2, 0, 1]))
        np.testing.assert_allclose(f.clamp([2, -3, 5]), [1, -1, 2])


class Hold(unittest.TestCase):
    def test_streams_when_fresh(self):
        h = bc.HoldLogic(0.3, 0.2)
        self.assertEqual(h.step(10.0, 9.95, 9.95), "STREAM")

    def test_planner_stale(self):
        h = bc.HoldLogic(0.3, 0.2)
        h.step(10.0, 9.95, 9.95)
        self.assertEqual(h.step(10.5, 10.0, 10.45), "HOLD")
        self.assertIn("planner stale", h.reason)

    def test_slam_stale(self):
        h = bc.HoldLogic(0.3, 0.2)
        h.step(10.0, 9.95, 9.95)
        self.assertEqual(h.step(10.3, 10.25, 10.0), "HOLD")
        self.assertIn("SLAM stale", h.reason)

    def test_resume_needs_a_fresh_command(self):
        h = bc.HoldLogic(0.3, 0.2)
        h.step(10.0, 9.95, 9.95)
        h.step(10.3, 10.25, 10.0)                       # SLAM stale -> HOLD at 10.3
        self.assertEqual(h.step(10.4, 10.25, 10.38), "HOLD")   # SLAM back, but the command predates the hold
        self.assertEqual(h.step(10.5, 10.45, 10.48), "STREAM")

    def test_latched_anchor_jump(self):
        h = bc.HoldLogic(0.3, 0.2)
        h.step(10.0, 9.95, 9.95)
        h.latch("frame re-anchored", 10.0)
        self.assertEqual(h.step(10.1, 9.99, 10.08), "HOLD")    # command older than the jump
        self.assertEqual(h.step(10.2, 10.15, 10.18), "STREAM")

    def test_external_fault(self):
        h = bc.HoldLogic(0.3, 0.2)
        self.assertEqual(h.step(10.0, 9.95, 9.95, ["target outside the geofence"]), "HOLD")
        self.assertEqual(h.reason, "target outside the geofence")

    def test_wait_before_any_command(self):
        h = bc.HoldLogic(0.3, 0.2)
        self.assertEqual(h.step(10.0, None, 9.95), "WAIT")

    def test_mode_change_holds_until_the_new_planner_commands(self):
        # the node drops the last command on a mode change and latches: no command at all, then one
        # from the new planner, is what resumes
        h = bc.HoldLogic(0.3, 0.2)
        h.step(10.0, 9.95, 9.95)
        h.latch("mode changed to drive", 10.0)
        self.assertEqual(h.step(10.1, None, 10.08), "HOLD")
        self.assertEqual(h.reason, "mode changed to drive")
        self.assertEqual(h.step(10.2, 10.15, 10.18), "STREAM")


class Health(unittest.TestCase):
    def test_fastlio_health(self):
        hc = bc.HealthCheck()
        hc.on_health(10.0, [100, 0, 0, 0, 20000, 0, 0, 80])
        self.assertTrue(any("matched" in f for f in hc.faults(10.1)))
        hc.on_health(10.2, [2000, 0, 0, 0, 20000, 0, 0, 300])
        self.assertTrue(any("latency" in f for f in hc.faults(10.3)))
        self.assertEqual(hc.faults(11.0), [])            # too old: ignored

    def test_height_cross_check(self):
        hc = bc.HealthCheck(height_tol=0.5)
        hc.on_range(10.0, 0.20, 0.25)                    # learns fc z - range = 0.05
        hc.on_range(10.5, 2.00, 2.06)
        self.assertEqual(hc.faults(10.6), [])
        hc.on_range(11.0, 4.50, 2.10)                    # log 233: SLAM says 2.1 m, the rangefinder 4.5 m
        self.assertTrue(any("rangefinder" in f for f in hc.faults(11.1)))


# ---- node tests ---------------------------------------------------------------------------------

class Node(unittest.TestCase):
    """the bridge node against synthetic inputs: FAST-LIO odometry, the SLAM bridge's vision pose
    (with a known anchor), the FC pose and mode, and SUPER commands"""

    ANCHOR = bc.Anchor(math.radians(30.0), [2.0, -1.0, 0.1])

    @classmethod
    def setUpClass(cls):
        import rospy
        from geometry_msgs.msg import PoseStamped, TransformStamped  # noqa: F401
        from mavros_msgs.msg import PositionTarget, State
        from nav_msgs.msg import Odometry
        from quadrotor_msgs.msg import PositionCommand
        from std_msgs.msg import Bool, String
        cls.Bool = Bool
        cls.rospy, cls.PoseStamped, cls.PositionTarget, cls.State = rospy, PoseStamped, PositionTarget, State
        cls.Odometry, cls.PositionCommand, cls.String = Odometry, PositionCommand, String
        cls.proc = subprocess.Popen([sys.executable, os.path.join(HERE, "..", "planner_bridge.py"),
                                     "_speed_cap:=1.0", "_geofence_min:=[-20,-20,-1]", "_geofence_max:=[20,20,8]"],
                                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        rospy.init_node("test_planner_bridge", anonymous=True)
        cls.sp, cls.status = [], [None]
        rospy.Subscriber("/mavros/setpoint_raw/local", PositionTarget, lambda m: cls.sp.append((time.time(), m)))
        rospy.Subscriber("/planner_bridge/status", String, lambda m: cls.status.__setitem__(0, m.data))
        cls.pub_odom = rospy.Publisher("/Odometry", Odometry, queue_size=50)
        cls.pub_vis = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped, queue_size=50)
        cls.pub_fc = rospy.Publisher("/mavros/local_position/pose", PoseStamped, queue_size=50)
        cls.pub_state = rospy.Publisher("/mavros/state", State, queue_size=5, latch=True)
        cls.pub_cmd = rospy.Publisher("/planning/pos_cmd", PositionCommand, queue_size=50)
        cls.pub_drive = rospy.Publisher("/track_planner/pos_cmd", PositionCommand, queue_size=50)
        cls.pub_enable = rospy.Publisher("/planner_bridge/enable", Bool, queue_size=1, latch=True)
        cls.pub_mode = rospy.Publisher("/planner_bridge/set_mode", String, queue_size=1, latch=True)
        from std_msgs.msg import Int8
        cls.mode_code, cls.mode_out = [None], [None]
        rospy.Subscriber("/planner_bridge/mode_code", Int8, lambda m: cls.mode_code.__setitem__(0, m.data))
        rospy.Subscriber("/planner_bridge/mode", String, lambda m: cls.mode_out.__setitem__(0, m.data))
        time.sleep(3.0)
        cls.pub_enable.publish(Bool(data=True))

    @classmethod
    def tearDownClass(cls):
        cls.proc.terminate()
        cls.proc.wait(5)

    def feed(self, seconds, anchor=None, cmd=True, slam=True, mode="GUIDED", p_planner=(1.0, 0.5, 1.2),
             cmd_pos=None, cmd_vel=(0.4, 0.0, 0.0), cmd_pub=None):
        """publish every input at 20 Hz (FAST-LIO) / 50 Hz (commands) for `seconds`"""
        anchor = anchor or self.ANCHOR
        st = self.State(); st.mode = mode; st.armed = True
        self.pub_state.publish(st)
        p_planner = np.array(p_planner)
        raw = bc.POSITION_MATRIX.T.dot(p_planner)             # planner -> raw
        q_raw = (0.0, 0.0, 0.0, 1.0)
        q_pl = bc.raw_to_planner_quat(q_raw)
        t_end = time.time() + seconds
        k = 0
        while time.time() < t_end and not self.rospy.is_shutdown():
            if slam and k % 5 == 0:
                o = self.Odometry(); o.header.stamp = self.rospy.Time.now()
                o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = raw
                o.pose.pose.orientation.w = 1.0
                self.pub_odom.publish(o)
                time.sleep(0.002)
                for pub in (self.pub_vis, self.pub_fc):
                    v = self.PoseStamped(); v.header.stamp = self.rospy.Time.now()
                    v.pose.position.x, v.pose.position.y, v.pose.position.z = anchor.pos(p_planner)
                    qf = bc.quat_mul(bc.quat_yaw(anchor.psi), q_pl)
                    v.pose.orientation.x, v.pose.orientation.y, v.pose.orientation.z, v.pose.orientation.w = qf
                    pub.publish(v)
            if cmd and k % 2 == 0:
                c = self.PositionCommand(); c.header.stamp = self.rospy.Time.now()
                cp = p_planner + [0.3, 0.0, 0.0] if cmd_pos is None else np.array(cmd_pos)
                c.position.x, c.position.y, c.position.z = cp
                c.velocity.x, c.velocity.y, c.velocity.z = cmd_vel
                c.acceleration.x = 0.2
                c.yaw = 0.1
                (cmd_pub or self.pub_cmd).publish(c)
            k += 1
            time.sleep(0.01)

    def last(self, since):
        return [m for (t, m) in self.sp if t >= since]

    def test_1_streams_at_50_hz_in_fc_frame(self):
        self.feed(1.0)
        t0 = time.time()
        self.feed(2.0)
        out = self.last(t0)
        self.assertGreater(len(out) / 2.0, 45.0)
        m = out[-1]
        self.assertEqual(m.type_mask, 2048)                     # position + velocity + acceleration + yaw
        self.assertEqual(m.coordinate_frame, self.PositionTarget.FRAME_LOCAL_NED)
        want = self.ANCHOR.pos(np.array([1.3, 0.5, 1.2]))
        np.testing.assert_allclose([m.position.x, m.position.y, m.position.z], want, atol=1e-3)
        np.testing.assert_allclose([m.velocity.x, m.velocity.y], self.ANCHOR.vec([0.4, 0, 0])[:2], atol=1e-3)
        self.assertAlmostEqual(m.yaw, 0.1 + self.ANCHOR.psi, places=3)
        self.assertIn('"state": "STREAM"', self.status[0])

    def test_2_speed_cap(self):
        self.feed(0.5, cmd_vel=(3.0, 0.0, 0.0))
        t0 = time.time()
        self.feed(0.5, cmd_vel=(3.0, 0.0, 0.0))
        m = self.last(t0)[-1]
        # the FC's demand, feed-forward + Q_P_NE_POS_P (1.0) x the 0.3 m lead, is held to the 1.0 cap
        lead = np.array([m.position.x, m.position.y]) - self.ANCHOR.pos([1.0, 0.5, 1.2])[:2]
        self.assertAlmostEqual(np.linalg.norm(np.array([m.velocity.x, m.velocity.y]) + lead), 1.0, places=3)

    def test_3_planner_stale_holds_with_zero_velocity(self):
        self.feed(0.5)
        t0 = time.time()
        self.feed(1.0, cmd=False)
        out = self.last(t0 + 0.5)
        self.assertGreater(len(out), 20)                         # still streaming: a hold target
        m = out[-1]
        self.assertEqual((m.velocity.x, m.velocity.y, m.velocity.z), (0.0, 0.0, 0.0))
        np.testing.assert_allclose([m.position.x, m.position.y], self.ANCHOR.pos([1.0, 0.5, 1.2])[:2], atol=1e-3)
        self.assertIn("planner stale", self.status[0])

    def test_4_slam_stale_holds(self):
        self.feed(0.5)
        self.feed(0.8, slam=False)
        self.assertIn("SLAM stale", self.status[0])

    def test_5_anchor_jump_holds_until_a_fresh_plan(self):
        self.feed(0.5)
        moved = bc.Anchor(self.ANCHOR.psi, self.ANCHOR.t + [0.5, 0, 0])
        self.feed(0.6, anchor=moved, cmd=False)
        time.sleep(0.25)
        self.assertIn("re-anchored", self.status[0])
        self.feed(0.8, anchor=moved)                             # fresh commands after the jump
        self.assertIn('"state": "STREAM"', self.status[0])
        self.__class__.ANCHOR = moved

    def test_6_geofence(self):
        self.feed(0.5, cmd_pos=(50.0, 0.0, 1.2))
        time.sleep(0.25)
        self.assertIn("geofence", self.status[0])

    def test_7_nothing_outside_guided(self):
        self.feed(0.3)
        t0 = time.time()
        self.feed(0.8, mode="QRLOITER")
        self.assertEqual(len(self.last(t0 + 0.2)), 0)

    def test_8_nothing_unless_enabled(self):
        # the FC's own takeoff and landing: a target, even a hold, would override them
        self.pub_enable.publish(self.Bool(data=False))
        self.feed(0.3)
        t0 = time.time()
        self.feed(0.8)
        self.assertEqual(len(self.last(t0 + 0.2)), 0)
        self.assertIn('"enabled": false', self.status[0])
        self.pub_enable.publish(self.Bool(data=True))
        t1 = time.time()
        self.feed(0.8)
        self.assertGreater(len(self.last(t1 + 0.3)), 15)

    def test_9a_drive_mode_takes_only_the_track_planner(self):
        # the executive switches the bridge to drive for a drive leg; SUPER keeps publishing (toward
        # its last goal) and must be ignored
        ground = (1.0, 0.5, 0.2)
        self.pub_mode.publish(self.String(data="drive"))
        self.feed(0.6, p_planner=ground)                         # SUPER only
        time.sleep(0.25)
        self.assertIn('"mode": "drive"', self.status[0])
        self.assertIn("mode changed to drive", self.status[0])
        self.assertEqual(self.mode_out[0], "drive")
        t0 = time.time()
        self.feed(0.4, p_planner=ground)
        # a hold on the ground sends nothing: Rover Guided stops at the last target it had
        self.assertEqual(len(self.last(t0 + 0.1)), 0)
        # the track planner's carrot 2 m ahead: a target drive_lookahead (0.5 m) toward it
        t1 = time.time()
        self.feed(0.8, p_planner=ground, cmd_pos=np.array(ground) + [2.0, 0.0, 0.0], cmd_vel=(0, 0, 0), cmd_pub=self.pub_drive)
        out = self.last(t1 + 0.4)
        self.assertGreater(len(out), 10)
        m = out[-1]
        self.assertEqual(m.type_mask, 2552)
        self.assertEqual((m.acceleration_or_force.x, m.velocity.x), (0.0, 0.0))
        want = self.ANCHOR.pos(ground) + self.ANCHOR.vec([0.5, 0.0, 0.0])
        np.testing.assert_allclose([m.position.x, m.position.y, m.position.z], want, atol=1e-3)
        self.assertIn('"state": "STREAM"', self.status[0])      # checked while fed (SLAM goes stale in 0.2 s)
        self.assertEqual(self.mode_code[0], 2)

    def test_9b_back_to_flight_ignores_the_track_planner(self):
        self.pub_mode.publish(self.String(data="flight"))
        self.feed(0.6, cmd_pos=np.array([3.0, 0.5, 1.2]), cmd_vel=(0, 0, 0), cmd_pub=self.pub_drive)   # carrots only
        time.sleep(0.25)
        self.assertIn('"mode": "flight"', self.status[0])
        t0 = time.time()
        self.feed(0.4, cmd_pos=np.array([3.0, 0.5, 1.2]), cmd_vel=(0, 0, 0), cmd_pub=self.pub_drive)
        m = self.last(t0)[-1]
        self.assertEqual(m.type_mask, 2048)                      # a flight hold at the vehicle
        self.assertEqual((m.velocity.x, m.velocity.y, m.velocity.z), (0.0, 0.0, 0.0))
        np.testing.assert_allclose([m.position.x, m.position.y], self.ANCHOR.pos([1.0, 0.5, 1.2])[:2], atol=1e-3)
        self.feed(0.6)                                           # SUPER again
        self.assertIn('"state": "STREAM"', self.status[0])
        self.assertEqual(self.mode_code[0], 1)

    def test_9c_unknown_mode_ignored_and_code_zero_when_disabled(self):
        self.pub_mode.publish(self.String(data="hover"))
        self.feed(0.5)
        time.sleep(0.25)
        self.assertIn('"mode": "flight"', self.status[0])
        self.pub_enable.publish(self.Bool(data=False))
        self.feed(0.5)
        time.sleep(0.25)
        self.assertEqual(self.mode_code[0], 0)
        self.pub_enable.publish(self.Bool(data=True))


if __name__ == "__main__":
    ros = "--ros" in sys.argv
    sys.argv = [a for a in sys.argv if a != "--ros"]
    if not ros:
        del Node
    unittest.main(verbosity=2)
