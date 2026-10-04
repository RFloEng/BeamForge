"""The base vehicle's motion ratios measured on its beams (beamforge/kinematics.py)."""

import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from beamforge import beamng as bng  # noqa: E402
from beamforge import kinematics, values  # noqa: E402

# a trailing arm 0.5 m long turning about a lateral axis on the body (a1-a2), the wheel's axle nodes
# at its end, a spring seat halfway along it and a spring up to the body: a motion ratio of 0.5
LEVER = {
    "vehicles/lever/lever.jbeam": json.dumps({
        "lever_body": {
            "information": {"name": "Body"}, "slotType": "main",
            "slots": [["type", "default", "description"], ["lever_suspension_F", "lever_suspension_F", "Arm"]],
            "nodes": [["id", "posX", "posY", "posZ"], ["a1", 0.6, -1.0, 0.3], ["a2", 0.8, -1.0, 0.3],
                      ["t1", 0.7, -1.25, 0.7], ["b0", 0.0, -1.0, 0.3], ["b1", 0.0, 0.0, 0.3], ["b2", 0.0, 0.0, 0.7]],
            "beams": [["id1:", "id2:"], ["a1", "a2"], ["a1", "b0"], ["a2", "b0"], ["t1", "b0"], ["b0", "b1"], ["b1", "b2"]],
        },
        "lever_suspension_F": {
            "information": {"name": "Arm"}, "slotType": "lever_suspension_F",
            "nodes": [["id", "posX", "posY", "posZ"], ["w1", 0.6, -1.5, 0.3], ["w2", 0.8, -1.5, 0.3], ["s1", 0.7, -1.25, 0.3]],
            "beams": [["id1:", "id2:"], {"beamSpring": 5000000},
                      ["a1", "w1"], ["a2", "w1"], ["a1", "w2"], ["a2", "w2"], ["w1", "w2"],
                      ["s1", "a1"], ["s1", "a2"], ["s1", "w1"], ["s1", "w2"],
                      {"beamSpring": 30000, "beamType": "|NORMAL"},
                      ["s1", "t1", {"precompressionRange": 0.1}]],
        },
    }),
    "vehicles/lever/info.json": json.dumps({"Name": "Lever", "Type": "Car"}),
}


class TestKinematics(unittest.TestCase):
    def test_lever_motion_ratio(self):
        bng.reset()
        bng.add_files(json.dumps(LEVER))
        conf = json.loads(bng.configure("lever"))
        conf["wheels"] = [{"name": "FL", "node1": "w1", "node2": "w2", "centre": [0.7, -1.5, 0.3]}]
        sp = values.springs_and_dampers("lever", conf)
        self.assertEqual([(r["kind"], r["a"], r["b"]) for r in sp], [("spring", "s1", "t1")])
        mr = kinematics.corner_ratios("lever", conf, conf["wheels"])
        self.assertAlmostEqual(next(iter(mr.values())), 0.5, places=2)
        sv = values.svj_values({"suspension": {"FL": {"spring": {"rate": 20000}}}}, None, {"front": {"spring": 0.5}})
        self.assertEqual(sv["front"]["coil_rate"], 80000)                    # wheel rate / 0.5^2

    def test_corner_stiffness(self):
        """The benchmark: a corner's stiffness at the wheel, and twice as stiff with links twice as stiff."""
        bng.reset()
        bng.add_files(json.dumps(LEVER))
        conf = json.loads(bng.configure("lever"))
        w = {"name": "FL", "node1": "w1", "node2": "w2", "centre": [0.7, -1.5, 0.3]}
        from beamforge import rigidity
        bl = rigidity.beams("lever", conf)
        k = kinematics.corner_stiffness("lever", conf, w, bl=bl)
        self.assertGreater(k["longitudinal"], 0)
        k2 = kinematics.corner_stiffness("lever", conf, w, bl=bl, kscale={id(b): 2.0 for b in bl})
        self.assertAlmostEqual(k2["longitudinal"] / k["longitudinal"], 2.0, places=1)


if __name__ == "__main__":
    unittest.main()
