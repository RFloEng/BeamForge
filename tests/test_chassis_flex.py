"""tools/chassis_flex.py: the torsion bench on a network of springs (needs numpy and scipy; skipped without)."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))

try:
    import chassis_flex as cf
except ImportError:                      # no numpy / scipy
    cf = None


@unittest.skipIf(cf is None, "numpy and scipy are needed")
class TestTorsionBench(unittest.TestCase):
    def two_posts(self, k=1e6, w=1.0, h=0.5, load=1000.0):
        pos = {"FLn": [w / 2, 1.0, 0.0], "FRn": [-w / 2, 1.0, 0.0], "gL": [w / 2, 1.0, -h], "gR": [-w / 2, 1.0, -h]}
        return cf.solve_torsion(pos, [("FLn", "gL", k), ("FRn", "gR", k)], ["gL", "gR"], ["FLn"], ["FRn"], load)

    def test_two_vertical_posts(self):
        r = self.two_posts(k=1e6, w=1.0)
        self.assertAlmostEqual(r["K_rad"], 1e6 * 1.0 ** 2 / 2, delta=500)      # a couple F W over a twist 2 F / (k W): K = k W^2 / 2
        self.assertLess(r["residual"], 1e-6)

    def test_scales_with_stiffness_and_width(self):
        a = self.two_posts(k=1e6, w=1.0)["K_rad"]
        self.assertAlmostEqual(self.two_posts(k=2e6, w=1.0)["K_rad"] / a, 2.0, places=3)
        self.assertAlmostEqual(self.two_posts(k=1e6, w=2.0)["K_rad"] / a, 4.0, places=3)   # W squared

    def test_load_does_not_change_the_stiffness(self):
        self.assertAlmostEqual(self.two_posts(load=100.0)["K_rad"], self.two_posts(load=5000.0)["K_rad"], delta=1)

    def test_degrees(self):
        r = self.two_posts()
        self.assertAlmostEqual(r["K"], r["K_rad"] * 3.141592653589793 / 180, places=3)

    def test_unconnected_mounts_are_reported(self):
        pos = {"FLn": [0.5, 1.0, 0.0], "FRn": [-0.5, 1.0, 0.0], "gL": [0.5, 1.0, -0.5], "gR": [-0.5, 1.0, -0.5]}
        r = cf.solve_torsion(pos, [("FLn", "FRn", 1e6), ("gL", "gR", 1e6)], ["gL", "gR"], ["FLn"], ["FRn"])
        self.assertTrue("error" in r or r["K_rad"] < 1e-3 * 1e6)


if __name__ == "__main__":
    unittest.main()
