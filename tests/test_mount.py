"""A sketch mounted on a base vehicle (beamforge/mount.py), on the made-up toy car."""

import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from beamforge import beamng as bng  # noqa: E402
from beamforge import export, mount, svj as svjmod  # noqa: E402
from tests.test_beamng import CAR  # noqa: E402


class TestMount(unittest.TestCase):
    def setUp(self):
        bng.reset()
        bng.add_files(json.dumps(CAR))
        self.v = json.loads(bng.configure("toycar"))
        meas = self.v.get("measure") or {}
        self.yf, self.zg = meas.get("front_axle_y") or 0.0, meas.get("ground_z") or 0.0
        sae = lambda n: svjmod.to_sae(self.v["geometry"]["nodes"][n], self.yf, self.zg)   # noqa: E731
        b1, b2 = sae("b1"), sae("b2")
        up = [b1[0], b1[1], b1[2] - 0.4]
        self.parts = [{"name": "bracket", "lines": [[b1, up], [up, b2], [b2, b1]]},                # on b1 and b2
                      {"name": "loose", "lines": [[[3.0, 0.0, -1.0], [3.0, 0.4, -1.0]]]}]       # touches nothing

    def test_plan(self):
        p = mount.plan("toycar", self.v, {"parts": self.parts, "opts": {}})
        self.assertEqual(sorted(b for _, b, _ in p["shared"]), ["b1", "b2"])
        self.assertEqual(p["skipped"], 1)                                     # b1-b2: the car already has that beam
        new = {x["id"] for x in p["nodes"]}
        self.assertTrue(all(i.startswith("bfsk") for i in new))
        self.assertTrue(all(x["a"] in new | {"b1", "b2"} and x["b"] in new | {"b1", "b2"} for x in p["beams"]))
        self.assertEqual(p["loose"], ["loose"])
        self.assertTrue(any("fall off" in w for w in p["warnings"]))
        self.assertGreater(p["weights"]["toycar"][1], 25.0)                   # b1 carries the bracket's end
        tie = p["tie_suggestions"]["loose"]
        self.assertEqual(next(x["group"] for x in p["nodes"] if x["sketch"] == tie), "sk_loose")
        tied = mount.plan("toycar", self.v, {"parts": self.parts, "opts": {}}, ties=[tie])
        self.assertEqual(tied["loose"], [])
        self.assertEqual(sorted(tied["ties"][0][1]), ["b1", "b2"])          # the body's nodes (the toy has two), not the wheel's

    def test_export(self):
        mj = json.dumps({"sketch": {"parts": self.parts[:1], "opts": {}}})
        out = json.loads(export.build("toycar", "toymount", "Toy Mount", json.dumps(self.v), "{}", None, None, None, mj))
        body = json.loads(out["files"]["vehicles/toymount/toycar.jbeam"])["toycar"]
        new = [r for r in body["nodes"][1:] if isinstance(r, list) and str(r[0]).startswith("bfsk")]
        self.assertEqual(len(new), 1)                                          # the bracket's top: its ends are b1, b2
        self.assertEqual(new[0][-1]["group"], "sk_bracket")
        b1 = next(r for r in body["nodes"][1:] if isinstance(r, list) and r[0] == "b1")
        self.assertGreater(b1[-1]["nodeWeight"], 25.0)
        self.assertEqual(out["counts"]["sketch_nodes"], 1)
        self.assertTrue(any(r[:2] in (["b1", new[0][0]], [new[0][0], "b1"]) for r in body["beams"][1:] if isinstance(r, list)))
        self.assertTrue(any("sketch mounted" in n for n in out["notes"]))


if __name__ == "__main__":
    unittest.main()
