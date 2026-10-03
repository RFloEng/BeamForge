"""Fitting a vehicle to an SVJ (beamforge/fit.py), on a made-up box car and mesh."""

import os
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from beamforge import fit, gltf  # noqa: E402
from beamforge import svj as svjmod  # noqa: E402


def box_car():
    """A body of nodes on a grid (y -2.0..2.0, x +-0.8, z 0.2..1.4), wheels at y -1.25 / 1.25."""
    nodes, parts = {}, {}
    i = 0
    for y in [k / 10 for k in range(-20, 21, 2)]:
        for x in (-0.8, -0.4, 0.0, 0.4, 0.8):
            for z in (0.2, 0.8, 1.4):
                nodes[f"b{i}"] = [x, y, z]
                parts[f"b{i}"] = "car_body"
                i += 1
    wheels = []
    for name, y in (("F", -1.25), ("R", 1.25)):
        for side, s in (("L", 1), ("R", -1)):
            a, b = f"w{name}{side}o", f"w{name}{side}i"
            nodes[a], nodes[b] = [0.8 * s, y, 0.3], [0.6 * s, y, 0.3]
            parts[a] = parts[b] = "car_suspension_" + name
            wheels.append({"name": name + side, "node1": a, "node2": b, "radius": 0.3})
    return nodes, parts, wheels


class TestStages(unittest.TestCase):
    def test_piecewise(self):
        f = fit.piecewise([(0, 0), (2, 3)])
        self.assertEqual((f(-1), f(1), f(2), f(4)), (-1, 1.5, 3, 5))

    def test_wheelbase_keeps_front_axle(self):
        nodes, parts, wheels = box_car()
        r = fit.fit(nodes, parts, wheels, {"chassis": {"wheelbase": 2.7}}, stages=("wheelbase",))
        n = r["nodes"]
        self.assertAlmostEqual(n["wFLo"][1], -1.25)                          # front axle stays
        self.assertAlmostEqual(n["wRLo"][1], 1.45)                           # rear axle: front + 2.7
        self.assertAlmostEqual(n["b0"][1], -2.0)                             # front overhang shifts with its axle
        self.assertAlmostEqual(n[max(nodes, key=lambda k: nodes[k][1])][1], 2.2)   # rear end shifts by +0.2
        self.assertEqual(r["report"][0]["after"], 2.7)

    def test_body_to_mesh(self):
        """A mesh 10 % wider, 0.1 m lower on top and with a 0.2 m longer front overhang."""
        nodes, parts, wheels = box_car()
        mesh = [[x * 1.1, y - 0.2 if y < -1.25 else y, z if z < 1.0 else z - 0.1]
                for x in (-0.8, -0.4, 0.0, 0.4, 0.8) for y in [k / 20 for k in range(-40, 41)] for z in (0.2, 0.8, 1.4)]
        r = fit.fit(nodes, parts, wheels, {"chassis": {}}, mesh=mesh, stages=("body",))
        rep = {x["label"]: x for x in r["report"]}
        self.assertAlmostEqual(rep["Width"]["after"], rep["Width"]["target"], places=2)
        self.assertAlmostEqual(rep["Height"]["after"], rep["Height"]["target"], places=2)
        self.assertAlmostEqual(rep["Front overhang"]["after"], rep["Front overhang"]["target"], places=2)
        self.assertAlmostEqual(r["nodes"]["wFLo"][1], -1.25)                 # the axle lines stay
        self.assertAlmostEqual(r["nodes"]["wRLo"][1], 1.25)
        self.assertNotIn("wFLo", fit.body_nodes(nodes, parts))              # suspension measured out, moved along
        self.assertAlmostEqual(r["nodes"]["wFLo"][0], 0.88, places=2)

    def test_no_wheels_or_mesh(self):
        nodes, parts, _ = box_car()
        self.assertTrue(fit.fit(nodes, parts, [], {})["notes"])
        nodes, parts, wheels = box_car()
        r = fit.fit(nodes, parts, wheels, {"chassis": {}}, mesh=None, stages=("body",))
        self.assertEqual(r["moves"], {})
        self.assertIn("no SVJ body mesh", r["notes"][0])


class TestMeshPoints(unittest.TestCase):
    def test_glb_body_mesh_into_beamng(self):
        """A one-triangle glb bound as the chassis, read back in BeamNG coordinates."""
        sae = [[0.5, 0.0, -1.0], [-3.5, 0.0, -1.0], [0.5, 0.9, 0.0]]     # front tip, rear tip, right low
        data = gltf.triangles_glb([sae], "SVJ::chassis::body")
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "body.glb")
            Path(path).write_bytes(data)
            doc = {"assets": {"meshes": [{"id": "m", "uri": "body.glb"}]}, "chassis": {"visual": {"mesh_ref": "m", "node": "SVJ::chassis::body"}}}
            pts = fit.mesh_points(doc, {"m": path}, yf=-1.2, ground=0.05)
        want = [svjmod.from_sae(p, -1.2, 0.05) for p in sae]
        for p, w in zip(pts, want):
            for a, b in zip(p, w):
                self.assertAlmostEqual(a, b, places=5)


if __name__ == "__main__":
    unittest.main()
