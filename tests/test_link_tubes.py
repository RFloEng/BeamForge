"""The corner links as real tubes: the section a wheel gets (an SVJ link's own, else the build's default) and the stiffness limits."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from beamforge import export, tubes  # noqa: E402

WHEELS = {"wheels": [{"name": "FL"}, {"name": "FR"}]}


class TestLinkSections(unittest.TestCase):
    def test_none_without_a_section(self):
        self.assertEqual(export._link_sections({"svj": {}}, WHEELS), {})

    def test_default_for_every_wheel(self):
        r = export._link_sections({"link_tube": "tube 30x2 steel_4130n", "svj": {}}, WHEELS)
        self.assertEqual(set(r), {"FL", "FR"})
        self.assertEqual(r["FL"]["text"], "tube 30x2 steel_4130n")

    def test_svj_link_wins_over_default(self):
        svj = {"suspension": {"FL": {"topology": {"links": [{"x_tube": {"section": "tube 25x1.6 steel_4130n"}}]}}}}
        r = export._link_sections({"link_tube": "tube 30x2 steel_4130n", "svj": svj}, WHEELS)
        self.assertEqual(r["FL"]["text"], "tube 25x1.6 steel_4130n")
        self.assertEqual(r["FR"]["text"], "tube 30x2 steel_4130n")

    def test_bad_section_is_an_error(self):
        with self.assertRaises(tubes.SectionError):
            export._link_sections({"link_tube": "tube 30x16 steel_4130n"}, WHEELS)


class TestTubeStiffness(unittest.TestCase):
    def test_e_a_over_l(self):
        k = tubes.limits(tubes.section("tube 30x2 steel_4130n"), 0.4)["k"]
        self.assertAlmostEqual(k / 1e6, 90.2, delta=0.5)         # 205 GPa x 176 mm2 / 0.4 m


if __name__ == "__main__":
    unittest.main()
