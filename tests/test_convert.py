"""A base corner converted into an SVJ corner by role (beamforge/roles.py, convert.py)."""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from beamforge import convert, roles, svj as svjmod  # noqa: E402


class TestConvert(unittest.TestCase):
    def setUp(self):
        # a strut corner: axle nodes, an upright (ball joint b, tie rod end t, strut bottom s), the strut top
        # k, the lower arm pivots p1 / p2 and the rack end r; left side, front axle at y -1.3
        self.nodes = {"wo": [0.85, -1.3, 0.3], "wi": [0.6, -1.3, 0.3], "bl": [0.7, -1.3, 0.2], "tl": [0.7, -1.43, 0.25],
                      "sl": [0.6, -1.3, 0.3], "kl": [0.55, -1.28, 0.8], "p1l": [0.35, -1.5, 0.2], "p2l": [0.35, -1.3, 0.2],
                      "rl": [0.3, -1.42, 0.26]}
        self.parts = {n: "car_suspension_F" for n in self.nodes}
        self.wheel = {"name": "FL", "node1": "wo", "node2": "wi"}
        roles.ROLES["car_suspension_F"] = {"upright": ["b", "t", "s"], "joints": {"lower_ball_joint": ["b"], "tie_rod_end": ["t"]},
                                           "pivots": {"lower_arm": ["p1", "p2"], "tie_rod": ["r"]},
                                           "strut": {"top": ["k"], "rail_start": "s"}}
        self.addCleanup(roles.ROLES.pop, "car_suspension_F")
        sae = lambda p: svjmod.to_sae(p, -1.3, 0.0)  # noqa: E731
        self.svj = {"suspension": {"FL": {"topology": {
            "upright": {"hardpoints": {"wheel_center": sae([0.75, -1.3, 0.3]), "lower_ball_joint": sae([0.65, -1.3, 0.2]),
                                       "steering_tie_rod_end": sae([0.65, -1.43, 0.25])}},
            "links": [{"name": "lower_control_arm", "inboard_points": [sae([0.3, -1.55, 0.2]), sae([0.3, -1.25, 0.2])],
                       "outboard_ref": "hardpoints.lower_ball_joint"},
                      {"name": "steering_tie_rod", "inboard_points": [sae([0.28, -1.42, 0.26])], "outboard_ref": "hardpoints.steering_tie_rod_end"},
                      {"name": "strut", "inboard_points": [sae([0.52, -1.28, 0.78])], "outboard_ref": "hardpoints.lower_ball_joint"}]}}}}

    def test_roles_found_and_corner_converted(self):
        rt = roles.for_corner(self.parts, self.nodes, self.wheel)
        self.assertEqual(rt["joints"]["lower_ball_joint"], ["bl"])
        hps = [h for h in svjmod.hardpoints(self.svj, -1.3, 0.0) if h["corner"] == "FL"]
        t, info = convert.corner(self.nodes, rt, self.wheel, self.svj, "FL", hps, [1.0, 0.0, 0.0])
        wc = [(t["wo"][i] + t["wi"][i]) / 2 for i in range(3)]
        for a, b in zip(wc, [0.75, -1.3, 0.3]):
            self.assertAlmostEqual(a, b, places=6)                            # the SVJ's wheel centre
        self.assertAlmostEqual(math.dist(t["bl"], t["tl"]), math.dist(self.nodes["bl"], self.nodes["tl"]), places=6)   # rigid
        shift = [t["bl"][i] - [0.65, -1.3, 0.2][i] for i in range(3)]     # the arm vector is the SVJ's, from where b landed
        for a, b in zip(t["p1l"], [0.3 + shift[0], -1.55 + shift[1], 0.2 + shift[2]]):
            self.assertAlmostEqual(a, b, places=6)
        self.assertAlmostEqual(t["kl"][2], 0.78, places=6)                   # the strut top at the SVJ's height
        d = [t["kl"][i] - t["sl"][i] for i in range(3)]                       # on the upright's rail line
        d0 = [self.nodes["kl"][i] - self.nodes["sl"][i] for i in range(3)]
        cos = sum(a * b for a, b in zip(d, d0)) / math.sqrt(sum(a * a for a in d) * sum(b * b for b in d0))
        self.assertGreater(cos, 0.999)


if __name__ == "__main__":
    unittest.main()
