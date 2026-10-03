"""Values taken from an SVJ (beamforge/values.py): springs, dampers, tyres."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from beamforge import values  # noqa: E402

SVJ = {"suspension": {
    "FL": {"spring": {"rate": 22000, "motion_ratio": 0.9},
           "damper": {"bump_curve": [[0, 0], [0.05, 550], [0.2, 1400]], "rebound_curve": [[0, 0], [0.05, 820], [0.2, 2100]]},
           "wheel": {"loaded_radius": 0.295}},
    "RR": {"spring": {"rate": 24000}, "tire": {"unloaded_radius": 0.31}}}}


class TestValues(unittest.TestCase):
    def test_svj_values(self):
        sv = values.svj_values(SVJ, {"corners": {"rear": {"static": {"motion_ratio": 0.6}}}})
        f, r = sv["front"], sv["rear"]
        self.assertEqual(f["coil_rate"], round(22000 / 0.81))               # wheel rate / MR^2 (the file's MR)
        self.assertEqual(r["coil_rate"], round(24000 / 0.36))               # the study's MR when the file has none
        self.assertEqual((f["damp_bump"], f["damp_bump_fast"], f["damp_split"]), (11000, 5667, 0.05))
        self.assertEqual(f["damp_rebound"], 16400)
        self.assertEqual(f["tyre_radius"], 0.305)                           # loaded + 10 mm
        self.assertEqual(r["tyre_radius"], 0.31)                            # the unloaded radius when given
        self.assertIsNone(r["damp_bump"])

    def test_apply_to_part(self):
        part = {"beams": [["id1:", "id2:"], {"beamType": "|NORMAL"}, ["a", "b"], ["c", "d", {"beamSpring": 30000}]],
                "pressureWheels": [["name"], {"radius": 0.33}],
                "flexbodies": [["mesh", "[group]:", "nonFlexMaterials"], ["tire_01a", ["tire_FL"], [], {"scale": {"x": 1, "y": 1, "z": 1}}],
                               ["rim_01a", ["wheel_FL"]]]}
        values.apply_to_part("p", part, {"p": {2: {"beamSpring": 23000}, 3: {"beamSpring": 23000}}},
                             {"p": {"radius": 0.305, "scale": [1.0, 0.92, 0.92]}})
        self.assertEqual(part["beams"][2], ["a", "b", {"beamSpring": 23000}])     # a row of its own gets inline values
        self.assertEqual(part["beams"][3][-1], {"beamSpring": 23000})            # an inline value is replaced
        self.assertEqual(part["pressureWheels"][1]["radius"], 0.305)
        self.assertEqual(part["flexbodies"][1][-1]["scale"], {"x": 1.0, "y": 0.92, "z": 0.92})
        self.assertEqual(part["flexbodies"][2], ["rim_01a", ["wheel_FL"]])        # only the tyre mesh is scaled

    def test_slopes(self):
        self.assertEqual(values._slopes([[0, 0], [0.1, 1000]]), (10000.0, 10000.0, 0.1))
        self.assertIsNone(values._slopes([[0, 0]]))


if __name__ == "__main__":
    unittest.main()
