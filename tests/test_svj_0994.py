"""SVJ v0.99.4: the strut's lower end (strut_lower), damping given at the wheel (damper.reference), on made-up corners."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from beamforge import archetype, dampers, suspension, values  # noqa: E402

CURVE = [[0.0, 0.0], [0.1, 300.0], [0.5, 1500.0]]          # 3000 N s/m, one slope


def strut_corner(lower=True, zero=False):
    """A left front MacPherson corner (SAE, m): lower arm, tie rod, the strut from its top to its lower clamp."""
    top = [-0.06, -0.58, -0.73]
    hp = {"wheel_center": [0.0, -0.71, -0.30], "lower_ball_joint": [0.03, -0.66, -0.15],
          "steering_tie_rod_end": [0.15, -0.67, -0.23], "strut_top": top}
    if lower:
        hp["strut_lower"] = [0.01, -0.63, -0.30]
    ref = "hardpoints.strut_top" if zero else "hardpoints.strut_lower"
    return {"topology": {"system_type": "macpherson", "upright": {"hardpoints": hp}, "links": [
        {"name": "strut", "type": "strut", "inboard_points": [top], "outboard_ref": ref},
        {"name": "lower_control_arm", "type": "arm", "inboard_points": [[0.05, -0.38, -0.16], [-0.25, -0.38, -0.16]],
         "outboard_ref": "hardpoints.lower_ball_joint"},
        {"name": "steering_tie_rod", "type": "rod", "inboard_points": [[0.15, -0.38, -0.24]], "outboard_ref": "hardpoints.steering_tie_rod_end"}]},
        "spring": {"rate": 25000.0}, "damper": {"bump_curve": CURVE, "rebound_curve": CURVE}}


class TestV0994(unittest.TestCase):
    def test_strut_lower(self):
        r = suspension.study_svj({"suspension": {"FL": strut_corner()}})["corners"]["front"]
        self.assertTrue(0.85 < r["static"]["motion_ratio"] < 1.0)
        self.assertEqual(r["labels"]["SB"], "Strut lower mount (upright)")
        self.assertFalse(any("constraints" in n for n in r["notes"]))

    def test_zero_length_strut(self):
        # before v0.99.4 the spec gave strut_top as both ends: the lower end is estimated through the lower ball joint
        r = suspension.study_svj({"suspension": {"FL": strut_corner(lower=False, zero=True)}})["corners"]["front"]
        self.assertTrue(any("strut_lower" in n for n in r["notes"]))
        self.assertTrue(0.85 < r["static"]["motion_ratio"] < 1.0)

    def test_damping_at_the_wheel(self):
        c = strut_corner()
        c["damper"].update(motion_ratio=0.9)                 # at the damper: the wheel feels 3000 x 0.9^2
        self.assertEqual(dampers.wheel_values({"suspension": {"FL": c}})["front"]["bump"], round(3000 * 0.81))
        c["damper"]["reference"] = "wheel"                    # at the wheel: as given
        self.assertEqual(dampers.wheel_values({"suspension": {"FL": c}})["front"]["bump"], 3000)
        v = values.svj_values({"suspension": {"FL": c}}, None, {"front": {"spring": 0.9, "damper": 0.8}})["front"]
        self.assertAlmostEqual(v["damp_bump"], 3000 / 0.8 ** 2, delta=1)   # onto a base damper of ratio 0.8

    def test_archetype_reads_strut_lower(self):
        hp = {"lower_ball_joint": [0.66, 0.03, 0.15], "strut.0": [0.58, -0.06, 0.73], "strut_lower": [0.63, 0.01, 0.30],
              "steering_tie_rod_end": [0.67, 0.15, 0.23], "steering_tie_rod.0": [0.38, 0.15, 0.24],
              "lower_control_arm.0": [0.38, 0.05, 0.16], "lower_control_arm.1": [0.38, -0.25, 0.16], "wheel_center": [0.71, 0.0, 0.30]}
        self.assertEqual(archetype.strut_points(hp)["h2"], hp["strut_lower"])


if __name__ == "__main__":
    unittest.main()
