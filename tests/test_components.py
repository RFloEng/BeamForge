"""Components, mass and centre of gravity (beamforge/components.py)."""

import json
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from beamforge import components as comp  # noqa: E402


class TestComponents(unittest.TestCase):
    def test_combine_and_ballast(self):
        m, cg = comp.combine([(100, [0, 0, -0.5]), (100, [-2, 0, -0.3])])
        self.assertEqual((m, cg), (200, [-1.0, 0.0, -0.4]))
        b = comp.ballast(200, cg, 300, [-1.5, 0, -0.35])
        self.assertEqual(b["mass"], 100)
        m2, cg2 = comp.combine([(200, cg), (b["mass"], b["position"])])
        self.assertEqual((m2, [round(x, 4) for x in cg2]), (300, [-1.5, 0.0, -0.35]))   # the target reached
        self.assertIsNone(comp.ballast(300, cg, 250))                                    # cannot take mass away

    def test_spread_keeps_the_centre_near(self):
        nodes = {"a": [0, 0, 0], "b": [1, 0, 0], "c": [0, 1, 0], "d": [5, 5, 5]}
        s = comp.spread(nodes, list(nodes), 60, [0.2, 0.2, 0])
        self.assertAlmostEqual(sum(s.values()), 60)
        self.assertLess(s["d"], s["c"])                                                   # the far node, least
        self.assertGreater(s["a"], s["b"])                                                # the nearer gets more

    def test_svj_and_estimate(self):
        svj = {"chassis": {"mass_total": 1100, "center_of_gravity": [-1.1, 0, -0.5],
                           "mass_bodies": [{"id": "driver", "category": "payload", "mass": 75, "position": [-1.3, -0.35, -0.45]}, {"id": "bad"}]}}
        self.assertEqual(comp.svj_components(svj), [{"id": "driver", "kind": "payload", "mass": 75.0, "position": [-1.3, -0.35, -0.45]}])
        self.assertEqual(comp.svj_target(svj), (1100.0, [-1.1, 0.0, -0.5]))
        wheels = {c: {"center": [x, y, -0.3]} for c, x, y in (("FL", 0, -0.7), ("FR", 0, 0.7), ("RL", -2.5, -0.7), ("RR", -2.5, 0.7))}
        e = comp.estimate(None, None, {"mass": 120, "position": [0.0, 0, -0.4]}, wheels, comp.svj_components(svj))
        self.assertEqual(e["mass"], 120 + 4 * comp.WHEEL_KG + 75)
        self.assertAlmostEqual(e["front_share"], 1 + e["cg"][0] / 2.5, places=4)


if __name__ == "__main__":
    unittest.main()
