"""Fitting a vehicle to an SVJ (beamforge/fit.py), on a made-up box car and mesh."""

import math
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


def corner_car():
    """The box car with a left-front hub (h1 low, h2 high) beamed to the axle nodes, an arm mount m1
    on the body beamed to h1, and a body node far away."""
    nodes, parts, wheels = box_car()
    beams, beam_parts = [], []
    add = lambda n, p, part: (nodes.__setitem__(n, p), parts.__setitem__(n, part))  # noqa: E731
    add("h1", [0.68, -1.25, 0.15], "car_suspension_F")
    add("h2", [0.62, -1.25, 0.55], "car_suspension_F")
    add("m1", [0.30, -1.25, 0.18], "car_body")
    for a, b in (("wFLo", "h1"), ("wFLi", "h1"), ("wFLo", "h2"), ("wFLi", "h2"), ("h1", "h2"), ("h1", "m1")):
        beams.append([a, b])
        beam_parts.append("car_suspension_F")
    beams.append(["m1", "b0"])
    beam_parts.append("car_body")
    return nodes, parts, wheels, beams, beam_parts


def corner_svj(wc=(0.72, -1.25, 0.31), lbj=(0.70, -1.22, 0.16), inner=(0.33, -1.20, 0.20)):
    """An SVJ whose FL corner has a wheel centre, a lower ball joint and one arm inner point (BeamNG
    points given, placed at the box car's front axle y -1.25 and ground 0)."""
    sae = lambda p: svjmod.to_sae(list(p), -1.25, 0.0)  # noqa: E731
    return {"chassis": {"wheelbase": 2.5}, "suspension": {"FL": {"topology": {
        "upright": {"hardpoints": {"wheel_center": sae(wc), "lower_ball_joint": sae(lbj)}},
        "links": [{"name": "lower_arm", "type": "rod", "inboard_points": [sae(inner)], "outboard_ref": "hardpoints.lower_ball_joint"}]}}}}


class TestPickups(unittest.TestCase):
    def test_roles_and_mapping(self):
        nodes, parts, wheels, beams, bp = corner_car()
        hub, side = fit.corner_roles(nodes, beams, bp, parts, next(w for w in wheels if w["name"] == "FL"))
        self.assertEqual(hub, {"h1", "h2"})
        self.assertIn("m1", side)
        hps = svjmod.hardpoints(corner_svj(), -1.25, 0.0)
        rows = {r["name"]: r for r in fit.map_hardpoints(nodes, beams, bp, parts, wheels, hps)}
        self.assertEqual(rows["wheel_center"]["nodes"], ["wFLo", "wFLi"])
        self.assertEqual(rows["lower_ball_joint"]["nodes"], ["h1"])         # upright -> hub node
        self.assertEqual(rows["lower_arm.0"]["nodes"], ["m1"])              # inner point -> chassis side
        over = fit.map_hardpoints(nodes, beams, bp, parts, wheels, hps, {"FL:lower_ball_joint": "h2"})
        self.assertEqual(next(r for r in over if r["name"] == "lower_ball_joint")["nodes"], ["h2"])

    def test_pickups_exact_and_local(self):
        nodes, parts, wheels, beams, bp = corner_car()
        r = fit.fit(nodes, parts, wheels, corner_svj(), stages=("pickups",), beams=beams, beam_parts=bp)
        n = r["nodes"]
        for a, b in zip(n["h1"], [0.70, -1.22, 0.16]):
            self.assertAlmostEqual(a, b, places=6)
        for a, b in zip(n["m1"], [0.33, -1.20, 0.20]):
            self.assertAlmostEqual(a, b, places=6)
        wc = [(n["wFLo"][i] + n["wFLi"][i]) / 2 for i in range(3)]
        for a, b in zip(wc, [0.72, -1.25, 0.31]):
            self.assertAlmostEqual(a, b, places=6)
        self.assertEqual(n["wRLo"], nodes["wRLo"])                         # far away: unchanged
        self.assertTrue(all(m["after"] < 1e-6 for m in r["mapping"] if m["nodes"]))

    def test_upright_check_and_wheel_axis(self):
        nodes, parts, wheels, beams, bp = corner_car()
        doc = corner_svj()
        doc["suspension"]["FL"]["alignment"] = {"camber": math.radians(-1.5), "toe": math.radians(0.2)}
        r = fit.fit(nodes, parts, wheels, doc, stages=("pickups",), beams=beams, beam_parts=bp)
        u = next(x for x in r["uprights"] if x["corner"] == "FL")
        self.assertEqual(u["points"], ["wheel_center", "lower_ball_joint"])
        self.assertIsNone(u["shape_rms_mm"])                                # two points: no shape to compare
        self.assertEqual(u["worst_pair"][:2], ["wheel_center", "lower_ball_joint"])
        self.assertAlmostEqual(u["camber_deg"], -1.5, places=2)              # the SVJ static alignment
        self.assertAlmostEqual(u["toe_deg"], 0.2, places=2)
        self.assertEqual((u["svj_camber_deg"], u["svj_toe_deg"]), (-1.5, 0.2))
        n = r["nodes"]
        wc = [(n["wFLo"][i] + n["wFLi"][i]) / 2 for i in range(3)]
        for a, b in zip(wc, [0.72, -1.25, 0.31]):                            # still exactly on the wheel centre
            self.assertAlmostEqual(a, b, places=6)
        r2 = fit.fit(nodes, parts, wheels, corner_svj(), stages=("pickups",), beams=beams, beam_parts=bp)
        u2 = next(x for x in r2["uprights"] if x["corner"] == "FL")
        self.assertEqual((u2["camber_deg"], u2["toe_deg"]), (0.0, 0.0))     # no alignment in the file: the base axis

    def test_rigid_fit_and_distortion(self):
        src = [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]]
        dst = [[2 - p[1], 3 + p[0], 4 + p[2]] for p in src]                 # 90 degrees about z, then a shift
        f = fit.rigid_fit(src, dst)
        for p, q in zip(src, dst):
            for a, b in zip(f(p), q):
                self.assertAlmostEqual(a, b, places=6)
        before = {"a": [0, 0, 0], "b": [1, 0, 0]}
        self.assertEqual(fit.distortion(before, {"a": [0, 0, 0], "b": [3, 0, 0]}, [["a", "b"]]), [(3.0, "a", "b")])
        self.assertEqual(fit.distortion(before, before, [["a", "b"]]), [])


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
