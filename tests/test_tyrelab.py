"""tyrelab: the game's tyres against an SVJ's Pacejka tyre, ranked by size and characteristics (made-up tyres, no game files)."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from beamforge import tyrelab  # noqa: E402

LAT = {"pCy1": 1.5, "pDy1": 1.2, "pDy2": -0.2, "pEy1": -1.0, "pKy1": 15.0, "pKy2": 1.5}
LON = {"pCx1": 1.6, "pDx1": 1.1, "pDx2": -0.15, "pEx1": 0.5, "pKx1": 20.0}


def svj(thermal=""):
    return {"tires": {"sets": {"t": {"size_code": "225/45R17", "dimensions": {"section_width": 0.225, "aspect_ratio": 0.45, "rim_diameter_code": 17,
                                                                              "overall_diameter": 0.6}, "reference": {"load": 4000.0},
                                     "thermal": {"performance_curve": thermal}, "pacejka": {"model": "MF62", "lateral": LAT, "longitudinal": LON}}}},
            "suspension": {}}


def rec(part, width, radius, use, friction, no_load=1.04, full=0.3, slope=0.0003, rim=17):
    return {"part": part, "model": "x", "axle": None, "width_mm": width, "aspect": 45, "rim_in": rim, "radius": radius, "use": use,
            "class": tyrelab.bng_class(use), "props": {"frictionCoef": friction, "noLoadCoef": no_load, "fullLoadCoef": full,
                                                       "loadSensitivitySlope": slope, "slidingFrictionCoef": friction, "numRays": 16, "radius": radius}}


class TestProfile(unittest.TestCase):
    def test_svj_profile(self):
        p = tyrelab.svj_profile(svj())
        self.assertEqual((p["rim_in"], p["width_mm"]), (17, 225))
        self.assertTrue(0.9 < p["grip"] < 1.4)
        self.assertAlmostEqual(p["shape"][3], 1.0, places=3)          # the reference load, 4000 N, is the 1.0 of the shape
        self.assertGreater(p["shape"][0], p["shape"][-1])             # grip falls with load
        self.assertEqual(tyrelab.svj_profile({"tires": {}}), None)

    def test_slick_in_thermal_curve_is_race(self):
        self.assertEqual(tyrelab.svj_profile(svj("tcurve_slicksh.lut"))["class"], "race")

    def test_classes_of_the_games_words(self):
        self.assertEqual([tyrelab.bng_class(u) for u in ("race", "rally", "mud", "winter", "dually", None)],
                         ["race", "sport", "mud", "winter", "standard", "standard"])


class TestRank(unittest.TestCase):
    def setUp(self):
        self.sp = tyrelab.svj_profile(svj())
        g = self.sp["grip"] / tyrelab.bng_profile(rec("probe", 225, 0.3, "sport", 1.0), self.sp["fz0"])["grip"]    # the friction giving the SVJ's grip
        self.lib = [rec("near", 225, 0.300, "sport", g),
                    rec("narrow", 175, 0.300, "sport", g),
                    rec("sticky", 225, 0.300, "race", g * 1.6),
                    rec("other_rim", 225, 0.300, "sport", g, rim=15)]

    def test_nearest_in_size_and_grip_first(self):
        best = tyrelab.rank(self.sp, self.lib)
        self.assertEqual(best[0]["rec"]["part"], "near")

    def test_other_rims_left_out(self):
        self.assertNotIn("other_rim", [r["rec"]["part"] for r in tyrelab.rank(self.sp, self.lib)])

    def test_grip_counts_not_only_size(self):
        sticky_same_size = tyrelab.rank(self.sp, self.lib)
        names = [r["rec"]["part"] for r in sticky_same_size]
        self.assertLess(names.index("near"), names.index("sticky"))

    def test_identical_front_and_rear_parts_count_once(self):
        lib = self.lib + [dict(self.lib[0], part="near_R")]
        self.assertEqual(sum(1 for r in tyrelab.rank(self.sp, lib) if r["rec"]["part"].startswith("near")), 1)


if __name__ == "__main__":
    unittest.main()
