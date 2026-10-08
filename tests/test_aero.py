"""Aerodynamics (beamforge/aero.py)."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from beamforge import aero  # noqa: E402


class TestAero(unittest.TestCase):
    def test_forces(self):
        a = {"area": 2.0, "Cd": 0.3, "Cl": -0.2, "Cl_front": None, "Cl_rear": None, "cop": [-1.0, 0, -0.5], "rho": 1.225}
        r = aero.forces(a, 2.5, (100,))[0]
        q = 0.5 * 1.225 * (100 / 3.6) ** 2
        self.assertAlmostEqual(r["drag"], q * 0.3 * 2.0, places=0)
        self.assertAlmostEqual(r["power"], q * 0.3 * 2.0 * 100 / 3.6 / 1000, places=2)
        self.assertAlmostEqual(r["down_front"] + r["down_rear"], q * 0.2 * 2.0, places=0)
        self.assertAlmostEqual(r["down_front"] / (r["down_front"] + r["down_rear"]), 0.6, places=3)   # cop 1.0 m behind the front axle of 2.5
        self.assertAlmostEqual(aero.front_share({"Cl_front": -0.1, "Cl_rear": -0.3}, 2.5), 0.25)

    def test_read_from_components(self):
        a = aero.read({"aerodynamics": {"reference": {"frontal_area": 1.9},
                                        "components": [{"id": "wing", "Cd_contribution": 0.05, "Cl_contribution": -0.3}, {"id": "body", "Cd_contribution": 0.28}]}})
        self.assertEqual((a["area"], round(a["Cd"], 3), a["Cl"], a["rho"]), (1.9, 0.33, -0.3, 1.225))

    def test_drag_plate(self):
        nodes = {"a": [0.5, -1.0, 0.2], "b": [-0.5, -1.0, 0.2], "c": [0.0, -1.0, 0.9], "d": [0.0, 1.0, 0.5]}
        ids, coef = aero.drag_plate(nodes, list(nodes), 0.7)
        self.assertEqual(sorted(ids), ["a", "b", "c"])                       # the triangle facing the flow
        self.assertAlmostEqual(coef / 100 * 0.35, 0.7, places=4)             # area 0.35 m2, square to the flow


if __name__ == "__main__":
    unittest.main()
