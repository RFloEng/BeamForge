"""An SVJ in ISO 8855 axes (spec v0.109+: Y left, Z up) is read in SAE J670 (frame.py, unified.adopt)."""

import unittest

from beamforge import frame, unified


def doc(version="0.111", **md):
    return {"_metadata": dict({"version": version}, **md),
            "chassis": {"center_of_gravity": [-1.1, 0.01, 0.47], "inertia": {"Ixx": 1, "Ixy": 2, "Ixz": 3}},
            "wheel_stations": [], "constraints": [{"id": "h", "frame_a": {"position": [0.0, 0.7, 0.3], "orientation": [1, 0, 0, 0]}}]}


class TestFrame(unittest.TestCase):
    def test_declaration_and_version_rule(self):
        self.assertEqual(frame.frame_of(doc("0.103")), frame.SAE)
        self.assertEqual(frame.frame_of(doc("0.109")), frame.ISO)
        self.assertEqual(frame.frame_of(doc("0.111", physics_frame="SAE_J670")), frame.SAE)
        self.assertEqual(frame.frame_of({}), frame.SAE)

    def test_iso_file_is_converted_in_place(self):
        d = doc()
        note = frame.to_sae(d)
        self.assertIn("ISO 8855", note)
        self.assertEqual(d["chassis"]["center_of_gravity"], [-1.1, -0.01, -0.47])      # z up -> z down, y left -> y right
        self.assertEqual(d["chassis"]["inertia"]["Ixy"], -2)
        self.assertEqual(d["constraints"][0]["frame_a"]["position"], [0.0, -0.7, -0.3])
        self.assertEqual(frame.frame_of(d), frame.SAE)
        self.assertIsNone(frame.to_sae(d))                                             # (and only once)

    def test_adopt_converts_even_without_wheel_stations(self):
        d = doc()
        unified.adopt(d)
        self.assertLess(d["chassis"]["center_of_gravity"][2], 0)

    def test_sae_file_is_untouched(self):
        d = doc("0.103")
        before = repr(d)
        unified.adopt(d)
        self.assertEqual(repr(d), before)


if __name__ == "__main__":
    unittest.main()
