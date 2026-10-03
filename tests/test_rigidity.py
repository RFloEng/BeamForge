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

    def test_lighter_nodes_softer_beams(self):
        """Half the mass: the beams take half their k, and no node's k / m goes above the base's."""
        conf = json.loads(bng.configure("toycar"))
        conf["measure"].update(front_axle_y=-1.2, ground_z=0.0)
        rows, wheel_kg = values.node_weights("toycar", conf)
        half = sum(r[3] for r in rows) / 2 + wheel_kg
        w = values.weight_changes("toycar", conf, mass=half, cg_y=0.3)
        new, _ = rigidity.changes("toycar", conf, w)
        before, after = _index("toycar", conf, None, {}), _index("toycar", conf, w, new)
        for n in before:
            self.assertLessEqual(after[n], before[n] * 1.002, n)


if __name__ == "__main__":
    unittest.main()
