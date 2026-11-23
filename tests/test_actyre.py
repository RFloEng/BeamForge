"""actyre: Assetto Corsa's tyre model from a tyres.ini (made-up tyre, no game files)."""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from beamforge import actyre, tyrelab  # noqa: E402

INI = """
[HEADER]
VERSION=10
[BASIC]
WIDTH=0.245   ; m
RADIUS=0.33
[FRONT]
DY_REF=1.25
DX_REF=1.2
LS_EXPY=0.85
LS_EXPX=0.88
FRICTION_LIMIT_ANGLE=8
CAMBER_GAIN=0.06
FALLOFF_LEVEL=0.9
FALLOFF_SPEED=3
DY_CURVE=dy.lut
[REAR]
DY_REF=1.3
DX_REF=1.25
"""


def front(luts=None):
    return actyre.params(actyre.parse_ini(INI), "FRONT", luts)


class TestReading(unittest.TestCase):
    def test_ini_comments_and_sections(self):
        ini = actyre.parse_ini(INI)
        self.assertEqual(ini["BASIC"]["WIDTH"], 0.245)
        self.assertEqual(ini["FRONT"]["DY_CURVE"], "dy.lut")
        self.assertEqual(sorted(actyre.compounds(ini)), ["FRONT", "REAR"])

    def test_lut(self):
        self.assertEqual(actyre.lut("2000|1.3\n1000|1.4 ; c\n// x\n"), [[1000.0, 1.4], [2000.0, 1.3]])

    def test_params_size_and_defaults(self):
        p = front()
        self.assertEqual((p["WIDTH"], p["RADIUS"], p["FZ0"]), (0.245, 0.33, 2000.0))
        with self.assertRaises(KeyError):
            actyre.params(actyre.parse_ini(INI), "FRONT_9")

    def test_curve_table_replaces_power_law(self):
        p = front({"dy.lut": "1000|2.0\n4000|1.0"})
        self.assertAlmostEqual(actyre.peak_mu(p, 1000.0, "y"), 2.0)
        self.assertAlmostEqual(actyre.peak_mu(p, 9000.0, "y"), 1.0)          # flat past the table
        self.assertAlmostEqual(actyre.peak_mu(front(), 2000.0, "y"), 1.25)   # the power law is DY_REF at FZ0


class TestForces(unittest.TestCase):
    def test_nothing_without_load_or_slip(self):
        p = front()
        self.assertEqual(actyre.solve(p, 0.0, 0.1, 0.0)["fy"], 0.0)
        self.assertEqual(actyre.solve(p, 3000.0, 0.0, 0.0)["fy"], 0.0)

    def test_lateral_peaks_at_the_limit_angle_near_mu_times_load(self):
        p = front()
        fy = {a: abs(actyre.solve(p, 2000.0, math.radians(a), 0.0)["fy"]) for a in (2, 4, 6, 8, 10, 14)}
        self.assertGreater(fy[8], fy[4])
        self.assertGreater(fy[8], fy[14])                         # the force falls after the peak (FALLOFF_LEVEL 0.9)
        self.assertAlmostEqual(fy[8] / 2000.0, 1.25, delta=0.08)  # (the speed sensitivity takes a little off)

    def test_signs(self):
        p = front()
        self.assertGreater(actyre.solve(p, 3000.0, 0.05, 0.0)["fy"], 0)
        self.assertLess(actyre.solve(p, 3000.0, -0.05, 0.0)["fy"], 0)
        self.assertGreater(actyre.solve(p, 3000.0, 0.0, 0.05)["fx"], 0)
        self.assertLess(actyre.solve(p, 3000.0, 0.0, -0.05)["fx"], 0)

    def test_load_sensitivity_mu_falls_with_load(self):
        p = front()
        mu = dict(map(tuple, actyre.curves(p)["mu_y"]))
        self.assertGreater(mu[1000.0], mu[2000.0])
        self.assertGreater(mu[2000.0], mu[8000.0])

    def test_combined_slip_takes_lateral_grip(self):
        p = front()
        pure = abs(actyre.solve(p, 3000.0, math.radians(8), 0.0)["fy"])
        braking = abs(actyre.solve(p, 3000.0, math.radians(8), -0.15)["fy"])
        self.assertLess(braking, pure)

    def test_camber_changes_lateral_force(self):
        p = front()
        a = abs(actyre.solve(p, 3000.0, math.radians(6), 0.0, camber=0.0)["fy"])
        b = abs(actyre.solve(p, 3000.0, math.radians(6), 0.0, camber=math.radians(3))["fy"])
        self.assertNotAlmostEqual(a, b, places=1)


class TestProfile(unittest.TestCase):
    def test_profile_has_the_svj_profile_keys(self):
        p = actyre.profile(front(), 18)
        for k in ("name", "size", "rim_in", "width_mm", "radius", "fz0", "grip_y", "grip_x", "grip", "shape", "peak_deg", "stiffness", "class", "curves"):
            self.assertIn(k, p)
        self.assertEqual((p["width_mm"], p["rim_in"]), (245, 18))
        self.assertGreater(p["shape"][0], p["shape"][-1])
        self.assertAlmostEqual(p["peak_deg"], 8.0, delta=1.0)

    def test_it_ranks_against_the_games_tyres(self):
        sp = actyre.profile(front(), 18)
        sp["size"] = "AC"
        g = sp["grip"] / tyrelab.bng_profile({"props": {"frictionCoef": 1.0, "noLoadCoef": 1.04, "fullLoadCoef": 0.3, "loadSensitivitySlope": 0.0003,
                                                       "numRays": 16, "radius": 0.33}}, sp["fz0"])["grip"]

        def rec(part, width, friction):
            return {"part": part, "model": "x", "axle": None, "width_mm": width, "aspect": 45, "rim_in": 18, "radius": 0.33, "use": "sport",
                    "class": "sport", "props": {"frictionCoef": friction, "noLoadCoef": 1.04, "fullLoadCoef": 0.3, "loadSensitivitySlope": 0.0003,
                                                "slidingFrictionCoef": friction, "numRays": 16, "radius": 0.33}}
        best = tyrelab.rank(sp, [rec("near", 245, g), rec("narrow", 175, g)])
        self.assertEqual(best[0]["rec"]["part"], "near")


if __name__ == "__main__":
    unittest.main()
