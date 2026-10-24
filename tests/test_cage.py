"""cage: a roll bar or safety cage from its tubes (FBeam's numbers for its 12-node cage, the fit to a cabin, the stiffness)."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from beamforge import cage  # noqa: E402


def run(option="fia_min", layout="cage", params=None):
    mem = cage.members_of(layout)
    nodes = cage.fit(layout, params)
    rows, nm = cage.design(nodes, mem, cage.OPTIONS[option])
    return nodes, rows, nm


class TestFBeamNumbers(unittest.TestCase):
    def test_fia_minimum_cage(self):               # FBeam's README: 41.3 kg of tube, the weakest member 38.5 kN
        nodes, rows, nm = run("fia_min")
        s = cage.summary(nodes, rows, nm)
        self.assertEqual(len(rows), 18)
        self.assertAlmostEqual(s["tube_kg"], 41.3, delta=0.1)
        self.assertAlmostEqual(s["weakest"]["force_kn"], 38.5, delta=0.1)
        self.assertEqual(s["weakest"]["governs"], "buckling")

    def test_light_and_recommended(self):
        n, r, m = run("light")
        self.assertAlmostEqual(cage.summary(n, r, m)["tube_kg"], 36.2, delta=0.1)
        n, r, m = run("recommended")
        self.assertAlmostEqual(cage.summary(n, r, m)["tube_kg"], 38.6, delta=0.1)

    def test_bar_is_part_of_the_cage(self):
        n, r, m = run("fia_min", "bar")
        self.assertEqual({x["group"] for x in r}, set(cage.BAR_GROUPS))
        self.assertLess(cage.summary(n, r, m)["tube_kg"], 20)


class TestSprings(unittest.TestCase):
    def test_spring_never_over_real_or_stable(self):
        _, rows, _ = run()
        for r in rows:
            self.assertLessEqual(r["k_set"], r["k_real"] * (1 + 1e-9))
            self.assertLessEqual(r["k_set"], r["k_stable"] * (1 + 1e-9))

    def test_beamng_keeps_a_share_of_the_real_stiffness(self):
        s = cage.summary(*run())
        for k in ("side", "fore_aft", "twist"):
            self.assertTrue(0.01 < s["share"][k] < 0.5, (k, s["share"][k]))


class TestFit(unittest.TestCase):
    def test_default_layout_untouched_without_params(self):
        self.assertEqual(cage.fit("cage")["MHt_L"], cage.NODES["MHt_L"])

    def test_cabin_numbers_move_the_nodes(self):
        p = {"hoop_y": 0.5, "apillar_y": -0.8, "backstay_y": 1.4, "half_width": 0.5, "floor_z": 0.3, "roof_z": 1.4}
        n = cage.fit("cage", p)
        self.assertAlmostEqual(n["MHf_L"][1], 0.5, places=3)             # the hoop at hoop_y
        self.assertAlmostEqual(n["APf_L"][1], -0.8, places=3)            # the A-pillar feet at apillar_y
        self.assertAlmostEqual(n["BSf_L"][1], 1.4, places=3)
        self.assertAlmostEqual(n["MHf_L"][2], 0.3, places=3)
        self.assertAlmostEqual(n["MHt_L"][0] / cage.NODES["MHt_L"][0], 0.5 / 0.6, places=3)

    def test_params_from_a_measure(self):
        p = cage.params_from_measure({"wheelbase": 2.6, "front_axle_y": -1.3, "ground_z": 0.0, "track_front": 1.5, "body_height": 1.4})
        self.assertTrue(-1.3 < p["apillar_y"] < p["hoop_y"] < p["backstay_y"] < 1.3 + 0.2)
        self.assertTrue(p["roof_z"] > p["floor_z"])


class TestPart(unittest.TestCase):
    def test_jbeam_part(self):
        n, r, m = run("recommended")
        p = cage.part("bf_cage", n, r, m)["bf_cage"]
        self.assertEqual(len(p["nodes"]) - 2, 12)
        self.assertEqual(len(p["beams"]) - 2, 18)
        self.assertTrue(all("fbeamSection" in b[-1] for b in p["beams"][2:]))


if __name__ == "__main__":
    unittest.main()
