"""Landing-spot check tests: synthetic patches and scan-by-scan windows, plus real data when present:
merged maps (LANDING_MAPS=dir with the map PCDs) and recorded keyframe scans (LANDING_SIM=Shannon's
~/sim: track/maps_tfix, flights/<run>/{out,view}, tfix/maps/<bag>/tfix/out).

  python3 test_landing_check.py
  LANDING_MAPS=~/sim/marsim/maps LANDING_SIM=~/sim python3 test_landing_check.py     (on Shannon, l4-sim-l2)
"""
import json
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from landing_core import LandingCheck  # noqa: E402

rng = np.random.default_rng(1)


def floor(cx=0.0, cy=0.0, r=1.0, z=0.0, slope_deg=0.0, noise=0.01, step=0.04):
    g = np.arange(-r, r + 1e-9, step)
    x, y = np.meshgrid(g, g)
    x, y = x.ravel() + cx, y.ravel() + cy
    zz = z + np.tan(np.radians(slope_deg)) * (x - cx) + rng.normal(0, noise, x.size)
    return np.c_[x, y, zz]


def box(cx, cy, h, size=0.3, z0=0.0, step=0.04):
    g = np.arange(-size / 2, size / 2 + 1e-9, step)
    zz = np.arange(0, h + 1e-9, step)
    x, y, z = np.meshgrid(g, g, zz)
    return np.c_[x.ravel() + cx, y.ravel() + cy, z.ravel() + z0]


def load_pcd(p):
    raw = open(p, "rb").read()
    k = raw.index(b"DATA binary\n") + 12
    return np.frombuffer(raw[k:], dtype="<f4").reshape(-1, 4)[:, :3].astype(float)


class Synthetic(unittest.TestCase):
    def setUp(self):
        self.lc = LandingCheck()

    def test_flat_floor_is_clear(self):
        ok, st = self.lc.evaluate(floor(), (0, 0))
        self.assertTrue(ok, st)

    def test_box_is_occupied(self):
        ok, st = self.lc.evaluate(np.r_[floor(), box(0.1, 0.0, 0.3)], (0, 0))
        self.assertFalse(ok)
        self.assertIn("occupied", st["reason"])

    def test_overhead_beam_is_occupied(self):
        beam = np.c_[np.linspace(-0.6, 0.6, 60), np.zeros(60), np.full(60, 1.5)]
        ok, st = self.lc.evaluate(np.r_[floor(), beam], (0, 0), descend_from=2.5)
        self.assertFalse(ok)
        self.assertIn("occupied", st["reason"])

    def test_beam_above_the_descent_height_is_ignored(self):
        beam = np.c_[np.linspace(-0.6, 0.6, 60), np.zeros(60), np.full(60, 3.0)]
        ok, st = self.lc.evaluate(np.r_[floor(), beam], (0, 0), descend_from=2.0)
        self.assertTrue(ok, st)

    def test_slope_is_not_flat(self):
        ok, st = self.lc.evaluate(floor(slope_deg=10), (0, 0))
        self.assertFalse(ok)
        self.assertIn("not flat", st["reason"])

    def test_nothing_seen_is_not_observed(self):
        ok, st = self.lc.evaluate(floor(cx=5.0), (0, 0))
        self.assertFalse(ok)
        self.assertIn("not observed", st["reason"])

    def test_half_seen_is_not_observed(self):
        P = floor()
        ok, st = self.lc.evaluate(P[P[:, 0] > 0], (0, 0))
        self.assertFalse(ok)
        self.assertIn("not observed", st["reason"])

    def test_thick_floor_layer_is_uncertain(self):
        P = floor()
        P[:, 2] += rng.uniform(0, 0.28, len(P))
        ok, st = self.lc.evaluate(P, (0, 0))
        self.assertFalse(ok)

    def test_obstacle_just_behind_the_frame_is_caught(self):
        """the odometry point is ODOM_AHEAD (0.193 m) ahead of the frame centre: the frame's rear edge is
        0.45 m behind the spot, and a post 0.57-0.67 m behind it lies outside a disc centred on the spot
        (r 0.542 m) but inside the 0.2 m margin of the frame (the disc centred on the frame)"""
        post = box(-0.62, 0.0, 0.5, size=0.1, step=0.02)
        ok, st = self.lc.evaluate(np.r_[floor(), post], (0, 0))   # no yaw: centred on the spot (the old way)
        self.assertTrue(ok, st)
        ok, st = self.lc.evaluate(np.r_[floor(), post], (0, 0), yaw=0.0)
        self.assertFalse(ok, st)
        self.assertIn("occupied", st["reason"])
        self.assertEqual(st["spot"], [0.0, 0.0])
        self.assertEqual(st["centre"], [-0.19, 0.0])
        # heading the other way, the post is in front, beyond the margin
        ok, st = self.lc.evaluate(np.r_[floor(), post], (0, 0), yaw=np.pi)
        self.assertTrue(ok, st)
        self.assertEqual(st["centre"], [0.19, 0.0])

    def test_centre_follows_the_heading_and_the_offset(self):
        c = self.lc.centre((1.0, 2.0), np.pi / 2)
        np.testing.assert_allclose(c, [1.0, 2.0 - 0.193], atol=1e-9)
        np.testing.assert_allclose(LandingCheck(footprint_offset=0.0).centre((1.0, 2.0), 0.3), [1.0, 2.0])
        np.testing.assert_allclose(self.lc.centre((1.0, 2.0), None), [1.0, 2.0])

    def test_viz_holds_the_occupied_cells_and_the_floor(self):
        ok, st = self.lc.evaluate(np.r_[floor(z=0.4), box(0.1, 0.0, 0.3, z0=0.4)], (0, 0), yaw=0.0)
        self.assertFalse(ok)
        v = self.lc.viz
        self.assertAlmostEqual(v["floor_z"], 0.4, delta=0.03)
        self.assertEqual(len(v["occupied"]), st["obstacle_cells"])
        for x, y, h in v["occupied"]:
            self.assertLess(abs(x - 0.1), 0.25)
            self.assertLess(abs(y), 0.25)
            self.assertGreater(h, 0.1)
        self.assertEqual(v["unobserved"], [])

    def test_viz_holds_the_unobserved_cells(self):
        P = floor()
        ok, st = self.lc.evaluate(P[P[:, 0] > 0], (0, 0), yaw=0.0)
        self.assertFalse(ok)
        self.assertGreater(len(self.lc.viz["unobserved"]), 20)
        self.assertTrue(all(x < 0.05 for x, _ in self.lc.viz["unobserved"]))

    def test_box_over_most_of_the_disc_is_not_taken_for_the_floor(self):
        P = floor(r=2.0)
        P = P[~((np.abs(P[:, 0]) < 0.45) & (np.abs(P[:, 1]) < 0.45))]
        ok, st = self.lc.evaluate(np.r_[P, box(0.0, 0.0, 0.4, size=0.9)], (0, 0))
        self.assertFalse(ok, st)


def drifting_scans(n=20, drift=0.25, extra=None, r=2.0):
    """n scans of one flat floor, each seeing it at its own height (z drift between scans, 0..drift),
    each 1 cm thick; extra(k) -> points added to scan k (in that scan's own frame)"""
    out = []
    for k in range(n):
        dz = drift * k / max(1, n - 1)
        S = floor(r=r, noise=0.01, step=0.05) + [0.0, 0.0, dz]
        if extra is not None:
            e = extra(k)
            if e is not None and len(e):
                if e[:, 2].min() < 0.05:   # stands on the floor: hides it
                    S = S[~((np.abs(S[:, 0] - e[:, 0].mean()) < 0.16) & (np.abs(S[:, 1] - e[:, 1].mean()) < 0.16))]
                S = np.r_[S, e + [0.0, 0.0, dz]]
        out.append(S)
    return out


class ScanByScan(unittest.TestCase):
    """the node's case: the window's scans one by one, each fitted to the local ground model"""

    def setUp(self):
        self.lc = LandingCheck()

    def test_drifting_floor_merged_is_uncertain_but_scan_by_scan_is_clear(self):
        scans = drifting_scans(drift=0.3)
        self.assertFalse(self.lc.evaluate(np.concatenate(scans), (0, 0))[0])
        ok, st = self.lc.evaluate(None, (0, 0), scans=scans)
        self.assertTrue(ok, st)
        self.assertLess(st["floor_layer_m"], 0.05)

    def test_low_box_on_a_drifting_floor_is_occupied(self):
        ok, st = self.lc.evaluate(None, (0, 0), scans=drifting_scans(extra=lambda k: box(0.1, 0.0, 0.10)))
        self.assertFalse(ok, st)
        self.assertIn("occupied", st["reason"])

    def test_object_that_left_is_cleared_by_the_latest_scans(self):
        ok, st = self.lc.evaluate(None, (0, 0), scans=drifting_scans(extra=lambda k: box(0.1, 0.0, 0.5) if k < 8 else None))
        self.assertTrue(ok, st)

    def test_object_that_arrived_is_seen_by_the_latest_scans(self):
        ok, st = self.lc.evaluate(None, (0, 0), scans=drifting_scans(extra=lambda k: box(0.1, 0.0, 0.5) if k >= 16 else None))
        self.assertFalse(ok, st)
        self.assertIn("occupied", st["reason"])

    def test_overhead_beam_seen_scan_by_scan_is_occupied(self):
        beam = np.c_[np.linspace(-0.6, 0.6, 60), np.zeros(60), np.full(60, 1.5)]
        ok, st = self.lc.evaluate(None, (0, 0), descend_from=2.5, scans=drifting_scans(extra=lambda k: beam, drift=0.1))
        self.assertFalse(ok, st)


try:
    import landing_check as node   # needs rospy and visualization_msgs (the l4-sim-l2 image)
except ImportError:
    node = None


@unittest.skipIf(node is None, "rospy / visualization_msgs not importable")
class Markers(unittest.TestCase):
    """/planner/landing_markers built from an evaluation (no ROS master needed)"""

    def ns(self, arr):
        return {m.ns: m for m in arr.markers}

    def test_occupied_markers(self):
        lc = LandingCheck()
        ok, st = lc.evaluate(np.r_[floor(z=0.4), box(0.1, 0.0, 0.3, z0=0.4)], (0, 0), yaw=0.5)
        arr = node.build_markers(ok, st, lc.viz, "world", node.rospy.Time(0), 0.0)
        m = self.ns(arr)
        self.assertEqual(set(m), {"landing_disc", "landing_footprint", "landing_occupied", "landing_unobserved", "landing_label"})
        self.assertTrue(all(x.header.frame_id == "world" for x in arr.markers))
        d = m["landing_disc"]
        self.assertEqual((d.color.r, d.color.g), node.RED[:2])
        self.assertLess(d.color.a, 1.0)
        self.assertAlmostEqual(d.pose.position.z, 0.4, delta=0.04)
        self.assertAlmostEqual(d.pose.position.x, -0.193 * np.cos(0.5), delta=0.01)
        self.assertAlmostEqual(d.scale.x, 2 * lc.radius)
        o = m["landing_occupied"]
        self.assertEqual(len(o.points), st["obstacle_cells"])
        self.assertTrue(all(0.5 < p.z < 0.75 for p in o.points))
        self.assertEqual(m["landing_unobserved"].action, node.Marker.DELETE)
        f = m["landing_footprint"]
        self.assertEqual(len(f.points), 7)
        self.assertTrue(m["landing_label"].text.startswith("OCCUPIED "), m["landing_label"].text)

    def test_clear_and_not_observed_and_no_heading(self):
        lc = LandingCheck()
        ok, st = lc.evaluate(floor(), (0, 0), yaw=0.0)
        m = self.ns(node.build_markers(ok, st, lc.viz, "world", node.rospy.Time(0), 0.0))
        self.assertEqual((m["landing_disc"].color.r, m["landing_disc"].color.g), node.GREEN[:2])
        self.assertTrue(m["landing_label"].text.startswith("CLEAR layer "), m["landing_label"].text)
        self.assertEqual(m["landing_occupied"].action, node.Marker.DELETE)
        P = floor()
        ok, st = lc.evaluate(P[P[:, 0] > 0], (0, 0), yaw=0.0)
        m = self.ns(node.build_markers(ok, st, lc.viz, "world", node.rospy.Time(0), 0.0))
        self.assertEqual((m["landing_disc"].color.r, m["landing_disc"].color.g), node.AMBER[:2])
        self.assertGreater(len(m["landing_unobserved"].points), 20)
        self.assertTrue(m["landing_label"].text.startswith("NOT OBSERVED"))
        st = {"spot": [1.0, 2.0], "centre": [1.0, 2.0], "radius_m": 0.542, "reason": "no heading: no /planner/odom yet"}
        m = self.ns(node.build_markers(False, st, {}, "world", node.rospy.Time(0), 0.3))
        self.assertEqual((m["landing_disc"].color.r, m["landing_disc"].color.g), node.AMBER[:2])
        self.assertAlmostEqual(m["landing_disc"].pose.position.z, 0.305)
        self.assertEqual(m["landing_footprint"].action, node.Marker.DELETE)
        self.assertTrue(m["landing_label"].text.startswith("NO HEADING"))


@unittest.skipUnless(os.environ.get("LANDING_MAPS"), "set LANDING_MAPS to the directory with the map PCDs and flight metas")
class RealMaps(unittest.TestCase):
    """Hensel Phelps 2026-10-15 and CAN USA 2026-07-15 20:34:50 (Shannon: ~/sim/marsim/maps)"""

    @classmethod
    def setUpClass(cls):
        d = os.environ["LANDING_MAPS"]
        cls.lc = LandingCheck()
        cls.hp = load_pcd(os.path.join(d, "hp_1015_carved.pcd"))
        cls.cu = load_pcd(os.path.join(d, "canusa_2034.pcd"))

    def test_open_floor_is_clear(self):
        self.assertTrue(self.lc.evaluate(self.hp, (-5.7, -8.4))[0])

    def test_where_the_robot_landed_is_clear(self):
        self.assertTrue(self.lc.evaluate(self.cu, (-0.42, 1.45))[0])

    def test_furniture_and_racks_are_not_clear(self):
        for P, xy in ((self.hp, (-3.75, -5.25)), (self.hp, (2.25, -1.25)), (self.cu, (2.25, -6.25)), (self.cu, (-0.75, -6.75))):
            self.assertFalse(self.lc.evaluate(P, xy)[0], xy)

    def test_box_on_open_floor_is_not_clear(self):
        z0 = np.percentile(self.hp[np.hypot(self.hp[:, 0] + 5.7, self.hp[:, 1] + 8.4) < 0.5, 2], 5)
        self.assertFalse(self.lc.evaluate(np.r_[self.hp, box(-5.7, -8.4, 0.3, z0=z0)], (-5.7, -8.4))[0])

    def test_unobserved_is_not_clear(self):
        self.assertFalse(self.lc.evaluate(self.hp, (30.0, 30.0))[0])


def load_pcd_any(p):
    raw = open(p, "rb").read()
    k = raw.index(b"DATA binary\n") + len(b"DATA binary\n")
    hdr = raw[:k].decode()
    n = int([l for l in hdr.splitlines() if l.startswith("POINTS")][0].split()[1])
    nf = len([l for l in hdr.splitlines() if l.startswith("FIELDS")][0].split()) - 1
    return np.frombuffer(raw[k:k + n * 4 * nf], dtype="<f4").reshape(-1, nf)[:, :3].astype(float)


class Keyframes:
    """an SC-PGO run's keyframe scans in a map's frame (rigid fit of the keyframe positions to the
    map's path), FAST-LIO's 0.4 m blind applied"""

    def __init__(self, out, traj):
        Tk = np.loadtxt(os.path.join(out, "optimized_poses.txt")).reshape(-1, 3, 4)
        self.t = np.loadtxt(os.path.join(out, "times.txt"))
        n = min(len(Tk), len(self.t), len(traj))
        Tk, self.t, traj = Tk[:n], self.t[:n], np.asarray(traj)[:n]
        A = Tk[:, :, 3]
        ca, cb = A.mean(0), traj.mean(0)
        U, _, Vt = np.linalg.svd((A - ca).T @ (traj - cb))
        R = Vt.T @ np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))]) @ U.T
        self.Rk = np.einsum("ij,njk->nik", R, Tk[:, :, :3])
        self.pk = A @ R.T + (cb - R @ ca)
        self.out = out

    def scans_near(self, xy, r=10.0, t_max=None):
        """[(scan, sensor xyz)] of the keyframes within r of xy, in time order"""
        out = []
        for k in range(len(self.t)):
            if np.hypot(*(self.pk[k, :2] - xy)) >= r or (t_max is not None and self.t[k] - self.t[0] > t_max):
                continue
            b = load_pcd_any(os.path.join(self.out, "Scans", "%06d.pcd" % k))
            b = b[np.isfinite(b).all(1)]
            b = b[np.linalg.norm(b, axis=1) >= 0.4]
            out.append((b @ self.Rk[k].T + self.pk[k], self.pk[k]))
        return out


@unittest.skipUnless(os.environ.get("LANDING_SIM"), "set LANDING_SIM to Shannon's ~/sim (maps_tfix, flights, tfix runs)")
class RealScans(unittest.TestCase):
    """Recorded keyframe scans, scan by scan, descending from 1.5 m: the takeoff and landing spots
    of the 2026-07-15 flights and Hensel Phelps 2026-10-15; furniture; a box put on a spot"""

    @classmethod
    def setUpClass(cls):
        cls.root = os.environ["LANDING_SIM"]
        cls.lc = LandingCheck()

    def run_(self, name):
        H = self.root
        if name == "20_21_51":
            tr = np.loadtxt(H + "/track/maps_tfix/20_21_51_tfix_path.csv", delimiter=",", skiprows=1)[:, 1:4]
            return tr, Keyframes(H + "/tfix/maps/20_21_51/tfix/out", tr)
        if name == "house":
            tr = np.loadtxt(H + "/track/maps_tfix/house_tfix_path.csv", delimiter=",", skiprows=1)[:, 1:4]
            return tr, Keyframes(H + "/tfix/maps/house/tfix/out", tr)
        tr = np.array(json.load(open("%s/flights/%s/view/map_meta.json" % (H, name)))["traj"])
        return tr, Keyframes("%s/flights/%s/out" % (H, name), tr)

    def judge(self, tr, kf, xy, **kw):
        return self.lc.evaluate(None, xy, descend_from=tr[0, 2] + 1.5, scans=kf.scans_near(np.asarray(xy, float), **kw))

    def test_flight_takeoff_and_landing_spots_are_clear(self):
        for name, ends in (("20_21_51", (0, -1)), ("20_34_50", (0, -1)), ("20_18_07", (0, -1)), ("hp_1015", (0,))):
            tr, kf = self.run_(name)
            for e in ends:
                ok, st = self.judge(tr, kf, tr[e, :2])
                self.assertTrue(ok, (name, e, st))

    def test_merged_20_21_51_scans_refuse_the_takeoff_spot_but_scan_by_scan_is_clear(self):
        tr, kf = self.run_("20_21_51")
        P = np.concatenate([s for s, _ in kf.scans_near(tr[0, :2], r=3.0)])
        ok, st = self.lc.evaluate(P, tr[0, :2], descend_from=tr[0, 2] + 1.5)
        self.assertFalse(ok, st)   # merged: floor drift reads as obstacles up to ~0.4 m (or a thick layer)
        self.assertTrue(self.judge(tr, kf, tr[0, :2], r=3.0)[0])

    def test_low_box_on_a_landing_spot_is_caught(self):
        tr, kf = self.run_("20_34_50")
        c = tr[-1, :2] + [0.15, 0.0]
        scans = []
        for S, s in kf.scans_near(tr[-1, :2]):
            near = S[np.hypot(S[:, 0] - c[0], S[:, 1] - c[1]) < 1.0, 2]
            if len(near) >= 20:        # a 0.3 m box 0.10 m high on this scan's own floor
                z0 = np.percentile(near, 30)
                under = (np.abs(S[:, 0] - c[0]) <= 0.15) & (np.abs(S[:, 1] - c[1]) <= 0.15) & (S[:, 2] < z0 + 0.12)
                S = np.r_[S[~under], box(c[0], c[1], 0.10, z0=z0, step=0.03)]
            scans.append((S, s))
        ok, st = self.lc.evaluate(None, tr[-1, :2], descend_from=tr[0, 2] + 1.5, scans=scans)
        self.assertFalse(ok, st)
        self.assertIn("occupied", st["reason"])

    def test_furniture_is_not_clear(self):
        tr, kf = self.run_("hp_1015")
        ok, st = self.judge(tr, kf, (-3.75, -5.25))
        self.assertFalse(ok, st)
        self.assertIn("occupied", st["reason"])

    def test_house_start_clear_until_someone_stands_there(self):
        """the house drive's start: clear in the first 160 s of scans; later scans (164 s, 329-373 s)
        see something 1.1-1.55 m tall over the robot's rear edge for a few seconds at a time (a person),
        and those were the last looks at it, so the whole drive reads occupied (the latest wins)"""
        tr, kf = self.run_("house")
        ok, st = self.judge(tr, kf, tr[0, :2], t_max=160.0)
        self.assertTrue(ok, st)
        ok, st = self.judge(tr, kf, tr[0, :2])
        self.assertFalse(ok, st)


if __name__ == "__main__":
    unittest.main(verbosity=2)
