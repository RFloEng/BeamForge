"""The structure's rigidity when beams change length or nodes change weight (beamforge/rigidity.py)."""

import json
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from beamforge import beamng as bng  # noqa: E402
from beamforge import rigidity, values  # noqa: E402
from tests.test_beamng import CAR  # noqa: E402


def _index(model, conf, weights, new):
    """{node: (sum k / m)} with the new beam values and weights."""
    rows, _ = values.node_weights(model, conf)
    m = {}
    for part, row, nid, kg, _ in rows:
        m[nid] = m.get(nid, 0.0) + ((weights or {}).get(part) or {}).get(row, kg)
    k = {}
    for b in rigidity.beams(model, conf):
        x = (new.get(b["part"]) or {}).get(b["row"], {}).get("beamSpring", b["values"].get("beamSpring", 0.0))
        for n in (b["a"], b["b"]):
            k[n] = k.get(n, 0.0) + x
    return {n: k[n] / m[n] for n in k if m.get(n)}


class TestRigidity(unittest.TestCase):
    def setUp(self):
        bng.reset()
        bng.add_files(json.dumps(CAR))

    def test_length(self):
        """A longer beam is softer (k L0 / L1); a node's weight follows its beams' length."""
        base = json.loads(bng.configure("toycar"))
        n = next(iter(base["geometry"]["nodes"]))
        conf = json.loads(bng.configure("toycar", None, None, None, json.dumps({"nodes": {n: [0.0, 0.0, 0.3]}})))
        bl = rigidity.beams("toycar", conf)
        lf = rigidity.length_factors("toycar", conf, bl=bl)
        self.assertGreater(lf[n], 1.0)
        w = values.weight_changes("toycar", conf, length_factors=lf)
        new, stats = rigidity.changes("toycar", conf, w, bl=bl)
        self.assertGreater(stats["beams"], 0)
        rest, nodes = conf["geometry"]["rest"], conf["geometry"]["nodes"]
        for b in bl:
            if n in (b["a"], b["b"]) and "beamSpring" in b["values"]:
                l0, l1 = math.dist(rest[b["a"]], rest[b["b"]]), math.dist(nodes[b["a"]], nodes[b["b"]])
                got = new.get(b["part"], {}).get(b["row"], {}).get("beamSpring", b["values"]["beamSpring"])   # unchanged: not written
                self.assertLessEqual(got, b["values"]["beamSpring"] * l0 / l1 * 1.0001)

    def test_lighter_car_keeps_its_structure(self):
        """A much lighter target: the beams keep the base's values, the nodes stop at their floor (no node's
        k / m above the base's highest), and the mass is not reached rather than the structure softened."""
        conf = json.loads(bng.configure("toycar"))
        conf["measure"].update(front_axle_y=-1.2, ground_z=0.0)
        rows, wheel_kg = values.node_weights("toycar", conf)
        floors = rigidity.mass_floors("toycar", conf)
        tiny = sum(r[3] for r in rows) / 20 + wheel_kg
        w = values.weight_changes("toycar", conf, mass=tiny, floors=floors)
        new, _ = rigidity.changes("toycar", conf, w)
        self.assertEqual(new, {})                                            # no beam changed
        before, after = _index("toycar", conf, None, {}), _index("toycar", conf, w, new)
        self.assertLessEqual(max(after.values()), max(before.values()) * 1.002)
        got = sum(kg for part in w.values() for kg in part.values())
        self.assertGreater(got, sum(r[3] for r in rows) / 20)              # held up by the floors

if __name__ == "__main__":
    unittest.main()
