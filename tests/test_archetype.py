"""The strut archetype's pieces (beamforge/archetype.py)."""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from beamforge import archetype  # noqa: E402

HP = {"wheel_center": [0.75, -1.3, 0.3], "lower_ball_joint": [0.7, -1.31, 0.22], "steering_tie_rod_end": [0.7, -1.44, 0.25],
      "strut_outboard": [0.7, -1.31, 0.22], "strut.0": [0.6, -1.28, 0.8], "lower_control_arm.0": [0.33, -1.6, 0.22],
      "lower_control_arm.1": [0.35, -1.28, 0.23], "steering_tie_rod.0": [0.34, -1.4, 0.24]}


class TestStrut(unittest.TestCase):
    def test_points(self):
        p = archetype.strut_points(HP)
        self.assertNotIn("h2", p)                                   # strut bottom on the ball joint: one node
        self.assertTrue(p["steer"])
        a = [p["t"][i] - p["h1"][i] for i in range(3)]
        b = [p["h4"][i] - p["h1"][i] for i in range(3)]
        self.assertLess(math.dist([x / math.dist(p["t"], p["h1"]) for x in a],
                                  [x / math.dist(p["h4"], p["h1"]) for x in b]), 1e-9)   # h4 on the strut's axis
        self.assertAlmostEqual(p["h5"][1], 2 * HP["wheel_center"][1] - HP["steering_tie_rod_end"][1])
        self.assertIsNone(archetype.strut_points({k: v for k, v in HP.items() if k != "strut.0"}))

    def test_motion_ratio(self):
        mr = archetype.motion_ratio(archetype.strut_points(HP))
        self.assertTrue(0.95 < mr < 1.0)

    def test_corner_loads(self):
        f = archetype.corner_loads(1000, 1.0, 2.5)
        self.assertAlmostEqual(f["FL"] + f["RL"], 500 * 9.81)
        self.assertAlmostEqual(f["FL"], 300 * 9.81)


class TestSurgery(unittest.TestCase):
    def test_rows_using_removed_nodes_go(self):
        part = {"nodes": [["id", "posX", "posY", "posZ"], {"nodeWeight": 5}, ["a", 0, 0, 0], ["h", 1, 0, 0]],
                "beams": [["id1:", "id2:"], {"beamSpring": 1}, ["a", "h"], ["a", "b"]],
                "rails": {"r": {"links:": ["h", "b"]}, "s": {"links:": ["a", "b"]}},
                "slidenodes": [["id:", "railName"], ["x", "r"], ["y", "s"], ["h", "s"]],
                "flexbodies": [["mesh", "[group]:"], ["hubmesh", ["g_hub"]], ["armmesh", ["g_arm"]]]}
        dropped = archetype._drop_rows(part, {"h"}, {"g_hub": ["h"], "g_arm": ["a", "h"]})
        self.assertEqual(dropped, {"r"})
        self.assertEqual([r[0] for r in part["nodes"][1:] if isinstance(r, list)], ["a"])
        self.assertEqual([r for r in part["beams"][1:] if isinstance(r, list)], [["a", "b"]])
        self.assertEqual([r[0] for r in part["slidenodes"][1:]], ["y"])           # off the dropped rail, and h
        self.assertEqual([r[0] for r in part["flexbodies"][1:]], ["armmesh"])     # a mesh still on a kept node
        self.assertEqual(part["beams"][1], {"beamSpring": 1})                      # modifiers kept

    def test_wheel_remapped(self):
        part = {"pressureWheels": [["name", "hubGroup", "group", "node1:", "node2:", "nodeS", "nodeArm:", "wheelDir"],
                                   ["FL", "w", "t", "fwhl1ll", "fwhl1l", 9999, "fhub5l", -1,
                                    {"torqueCoupling:": "fhub1l", "torqueArm:": "fhub4l", "torqueArm2:": "fwhl1ll",
                                     "steerAxisUp:": "ftop1l", "steerAxisDown:": "fhub1l"}]]}
        names = {k: f"bfFL{k}" for k in ("h1", "h4", "h5", "t")}
        self.assertTrue(archetype._remap_wheels(part, "FL", names))
        row = part["pressureWheels"][1]
        self.assertEqual(row[6], "bfFLh5")
        self.assertEqual(row[-1], {"torqueCoupling:": "bfFLh1", "torqueArm:": "bfFLh4", "torqueArm2:": "fwhl1ll",
                                   "steerAxisUp:": "bfFLt", "steerAxisDown:": "bfFLh1"})

    def test_mount_weight_added_when_short(self):
        room = {"k": {"n": 1e6}, "c": {"n": 1000.0}, "lk": {"n": 2e6}, "lc": {"n": 1e3}, "kg": {}}
        archetype._take(room, "n", 3e6, 80)
        self.assertAlmostEqual(room["kg"]["n"], 1.0)                 # 2 MN/m short at 2 MN/m per kg
        self.assertAlmostEqual(room["k"]["n"], 0.0)


if __name__ == "__main__":
    unittest.main()
