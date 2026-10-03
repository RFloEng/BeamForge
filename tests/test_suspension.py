"""Suspension kinematics (beamforge/suspension.py): every layout, and corners read from an SVJ."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from beamforge import suspension as S  # noqa: E402
from beamforge.svj import to_sae  # noqa: E402


class TestLayouts(unittest.TestCase):
    def test_every_layout_sweeps(self):
        """Each built-in layout, from its default points, assembles and moves through +-60 mm."""
        for lay in [k for k in S.TOPOLOGY if not k.startswith("svj_")]:
            pts = S.default_points(lay, [0.70, 1.25, 0.30])
            res = S.analyse({"type": lay, "tyre_radius": 0.30}, pts, travel_mm=60)
            tr = res["curves"]["travel_mm"]
            self.assertLessEqual(tr[0], -60, lay)
            self.assertGreaterEqual(tr[-1], 60, lay)
            self.assertAlmostEqual(res["static"]["track_mm"], 1400, delta=20, msg=lay)


def svj_macpherson():
    """A left-front MacPherson corner in SAE (front axle at X 0, ground at Z 0), from BeamNG points."""
    P = lambda p: to_sae(p)  # noqa: E731
    return {"topology": {"system_type": "macpherson", "upright": {"hardpoints": {
        "wheel_center": P([0.70, 0.0, 0.30]), "lower_ball_joint": P([0.68, 0.0, 0.17]),
        "strut_top": P([0.60, 0.03, 0.75]), "strut_lower": P([0.63, 0.01, 0.48]),
        "steering_tie_rod_end": P([0.66, 0.13, 0.30])}},
        "links": [{"name": "lower_control_arm", "type": "arm", "inboard_points": [P([0.32, -0.30, 0.20]), P([0.32, 0.10, 0.20])],
                   "outboard_ref": "hardpoints.lower_ball_joint"},
                  {"name": "strut", "type": "strut", "inboard_points": [P([0.60, 0.03, 0.75])], "outboard_ref": "hardpoints.strut_top"},
                  {"name": "steering_tie_rod", "type": "rod", "inboard_points": [P([0.25, 0.16, 0.31])],
                   "outboard_ref": "hardpoints.steering_tie_rod_end"}]}}


class TestSvjCorners(unittest.TestCase):
    def test_macpherson_from_svj(self):
        spec, pts, labels, notes = S.svj_corner(svj_macpherson(), "svj_test")
        self.assertEqual(notes, [])
        self.assertEqual(S.TOPOLOGY["svj_test"]["toe"], "TRO")
        self.assertEqual([k for k, _, _ in S.TOPOLOGY["svj_test"]["links"]], ["arm", "strut", "rod"])
        self.assertAlmostEqual(spec["tyre_radius"], 0.30)
        res = S.analyse(spec, pts, travel_mm=60)
        st = res["static"]
        self.assertAlmostEqual(st["track_mm"], 1400, delta=1)
        self.assertGreater(st["kpi_deg"], 5)                     # strut top inboard of the ball joint
        self.assertLess(res["curves"]["travel_mm"][0], -50)

    def test_right_corner_is_mirrored(self):
        c = svj_macpherson()
        for p in list(c["topology"]["upright"]["hardpoints"].values()) + [q for l in c["topology"]["links"] for q in l["inboard_points"]]:
            p[1] = -p[1]                                         # SAE Y right: the same corner on the right
        _, pts, _, _ = S.svj_corner(c, "svj_test_r")
        self.assertAlmostEqual(pts["WC"][0], 0.70)

    def test_override_with_base_points(self):
        """The SVJ's layout solved on other points: tied points replaced, the others move with the wheel centre."""
        _, pts, _, _ = S.svj_corner(svj_macpherson(), "svj_ref")
        over = {"wheel_center": [0.72, 0.0, 0.30], "lower_ball_joint": [0.70, 0.0, 0.16]}
        _, p2, _, _ = S.svj_corner(svj_macpherson(), "svj_over", over)
        self.assertEqual(p2["WC"], [0.72, 0.0, 0.30])
        self.assertEqual(p2["lower_ball_joint"], [0.70, 0.0, 0.16])
        for i, shift in enumerate((0.02, 0.0, 0.0)):              # not tied: moved with the wheel centre (+0.02 in x)
            self.assertAlmostEqual(p2["TRI"][i], pts["TRI"][i] + shift, places=6)
        import json
        out = json.loads(S.study_svj_json(json.dumps({"suspension": {"FL": svj_macpherson()}}), 60, json.dumps({"FL": over})))
        self.assertIn("front", out["base"]["corners"])
        self.assertAlmostEqual(out["base"]["corners"]["front"]["static"]["track_mm"], 1440, delta=1)
        self.assertAlmostEqual(out["corners"]["front"]["static"]["track_mm"], 1400, delta=1)

    def test_problems_are_named(self):
        c = svj_macpherson()
        c["topology"]["links"].append({"name": "extra_rod", "type": "rod", "inboard_points": [to_sae([0.3, 0.0, 0.4])],
                                       "outboard_ref": "hardpoints.lower_ball_joint"})
        c["topology"]["links"].append({"name": "pushrod", "type": "rod", "inboard_points": [to_sae([0.3, 0.0, 0.6])],
                                       "outboard_ref": "hardpoints.lower_ball_joint"})
        _, _, _, notes = S.svj_corner(c, "svj_test_over")
        self.assertTrue(any("over-constrained" in n for n in notes))
        self.assertTrue(any("Pushrod" in n for n in notes))
        r = S.study_svj({"suspension": {"FL": c}})
        self.assertIn("front", r["corners"])                     # still shown at static height
        self.assertIsNone(r["corners"]["front"]["static"]["kpi_deg"])


if __name__ == "__main__":
    unittest.main()
