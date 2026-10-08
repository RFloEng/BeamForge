"""A vehicle made from scratch (beamforge/scratch.py): a sketch, made-up donor parts, the files read back."""

import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from beamforge import beamng as bng, rigidity, scratch  # noqa: E402

TRACK, WB, RW = 1.45, 2.5, 0.29

# donor parts, made up: an engine block, a gearbox with its chain, a rim with its axle nodes and tyre slot, a tyre
DONORS = {
    "vehicles/toy/toy_engine.jbeam": json.dumps({
        "toy_engine": {"slotType": "toy_engine", "powertrain": [["type", "name", "inputName", "inputIndex"], ["combustionEngine", "mainEngine", "dummy", 0]],
                       "mainEngine": {"torque": [["rpm", "torque"], [0, 0], [1000, 100], [6000, 140]], "idleRPM": 800, "maxRPM": 6500,
                                      "inertia": 0.08, "requiredEnergyType": "gasoline", "torqueReactionNodes:": ["eng1l", "eng2l", "eng1r"],
                                      "breakTriggerBeam": "engine", "radiator": {"[engineGroup]:": ["radiator"]}},
                       "soundConfig": {"sampleName": "I4_2_engine"},
                       "vehicleController": {"shiftDownRPMOffsetCoef": 1.1},
                       "nodes": [["id", "posX", "posY", "posZ"], {"nodeWeight": 12}, ["eng1l", 0.2, -1.6, 0.3], ["eng1r", -0.2, -1.6, 0.3],
                                 ["eng2l", 0.2, -1.3, 0.3], ["eng2r", -0.2, -1.3, 0.3], ["eng3l", 0.2, -1.6, 0.6], ["eng3r", -0.2, -1.6, 0.6]],
                       "beams": [["id1:", "id2:"], {"beamSpring": 15001000, "beamDamp": 400}, ["eng1l", "eng1r"], ["eng1l", "eng2l"], ["eng2l", "eng2r"],
                                 ["eng1r", "eng2r"], ["eng1l", "eng2r"], ["eng3l", "eng3r"], ["eng1l", "eng3l"], ["eng1r", "eng3r"], ["eng2l", "eng3l"], ["eng2r", "eng3r"],
                                 ["eng1l", "body_node"]]}}),
    "vehicles/toy/toy_transmission.jbeam": json.dumps({
        "toy_transmission_5M": {"slotType": "toy_transmission",
                                "powertrain": [["type", "name", "inputName", "inputIndex"], ["frictionClutch", "clutch", "mainEngine", 1], ["manualGearbox", "gearbox", "clutch", 1]],
                                "gearbox": {"gearRatios": [-3.5, 0, 3.6, 2.1, 1.4, 1.0, 0.8], "gearboxNode:": ["gbx1"]},
                                "clutch": {"clutchFreePlay": 0.75},
                                "nodes": [["id", "posX", "posY", "posZ"], {"nodeWeight": 6}, ["gbx1", -0.3, -1.4, 0.4]],
                                "beams": [["id1:", "id2:"], ["gbx1", "eng1r"], ["gbx1", "eng2r"], ["gbx1", "eng3r"]]}}),
    "vehicles/common/wheels/toy_wheels.jbeam": json.dumps({
        "steel_13x5_F": {"slotType": "wheel_F_4", "slots": [["type", "default", "description"], ["tire_F_13x5", "tire_F_176_68_13_standard", "Front Tires"]],
                         "nodes": [["id", "posX", "posY", "posZ"], {"nodeWeight": 4.5}, ["fwhl1r", -0.33, 0, 0], ["fwhl1rr", -0.55, 0, 0], ["fwhl1l", 0.33, 0, 0], ["fwhl1ll", 0.55, 0, 0]],
                         "pressureWheels": [["name"], {"hubRadius": 0.18}, {"hubWidth": 0.14}]},
        "steel_13x5_R": {"slotType": "wheel_R_4", "slots": [["type", "default", "description"], ["tire_R_13x5", "tire_R_176_68_13_standard", "Rear Tires"]],
                         "nodes": [["id", "posX", "posY", "posZ"], {"nodeWeight": 4.5}, ["rwhl1r", -0.33, 0, 0], ["rwhl1rr", -0.55, 0, 0], ["rwhl1l", 0.33, 0, 0], ["rwhl1ll", 0.55, 0, 0]],
                         "pressureWheels": [["name"], {"hubRadius": 0.18}, {"hubWidth": 0.14}]},
        "tire_F_176_68_13_standard": {"slotType": "tire_F_13x5", "pressureWheels": [["name"], {"hasTire": True}, {"radius": 0.288}, {"tireWidth": 0.135}]},
        "tire_F_186_68_13_sport": {"slotType": "tire_F_13x5", "pressureWheels": [["name"], {"hasTire": True}, {"radius": 0.29}, {"tireWidth": 0.15}]},
        "tire_R_176_68_13_standard": {"slotType": "tire_R_13x5", "pressureWheels": [["name"], {"hasTire": True}, {"radius": 0.288}, {"tireWidth": 0.135}]},
    }),
}


def sketch():
    """A ladder frame and four double wishbone corners with uprights (a point in each: the wheel centre), front tie rods."""
    def pts(X):
        return [X, -0.40, -0.17], [X, 0.40, -0.17], [X, -0.45, -0.45], [X, 0.45, -0.45]
    xs = [0.6, 0.15, -0.20, -1.2, -2.3, -2.7, -3.1]
    lines = []
    for X in xs:
        a, b, c, d = pts(X)
        lines += [[a, b], [c, d], [a, c], [b, d], [a, d]]
    for X0, X1 in zip(xs, xs[1:]):
        lines += [[pts(X0)[i], pts(X1)[i]] for i in range(4)] + [[pts(X0)[0], pts(X1)[1]], [pts(X0)[2], pts(X1)[3]]]
    parts, wheels = [{"name": "frame", "lines": lines}], {}
    for c, X, s in (("fl", 0.0, -1), ("fr", 0.0, 1), ("rl", -WB, -1), ("rr", -WB, 1)):
        pf, pr = (0.15, 0.20) if c[0] == "f" else (0.20, 0.20)
        lbj, ubj, ter = [X, s * 0.66, -0.15], [X, s * 0.62, -0.47], [X - 0.12, s * 0.65, -0.25]
        wc = [X, s * TRACK / 2, -RW]
        parts += [{"name": f"lower_wishbone_{c}", "lines": [[[X + pf, s * 0.40, -0.17], lbj], [[X - pr, s * 0.40, -0.17], lbj]]},
                  {"name": f"upper_wishbone_{c}", "lines": [[[X + pf, s * 0.45, -0.45], ubj], [[X - pr, s * 0.45, -0.45], ubj]]},
                  {"name": f"upright_{c}", "lines": [[lbj, ubj], [ubj, ter], [ter, lbj]], "points": [wc]}]
        if c[0] == "f":
            parts.append({"name": f"tie_rod_{c}", "lines": [[[X - 0.12, s * 0.35, -0.25], ter]]})
        wheels[c.upper()] = {"center": wc}
    return parts, wheels


class TestScratch(unittest.TestCase):
    def setUp(self):
        bng.reset()
        bng.add_files(json.dumps(DONORS))
        parts, wheels = sketch()
        self.car = {"id": "bf_test_scratch", "name": "Scratch", "sketch": {"parts": parts, "opts": {}}, "wheels": wheels, "layout": "RWD",
                    "engine": {"source": {"model": "toy", "part": "toy_engine"}, "torque": [[1000, 120], [5000, 180], [7000, 150]], "max_rpm": 7200},
                    "gearbox": {"source": {"model": "toy", "part": "toy_transmission_5M", "device": "gearbox"}, "ratios": [3.2, 2.0, 1.4, 1.0, 0.8]},
                    "final_drive": 4.1, "rim": {"front": {"model": "common", "part": "steel_13x5_F"}, "rear": {"model": "common", "part": "steel_13x5_R"}},
                    "tyre": {"front": {"model": "common", "part": "tire_F_186_68_13_sport"}, "rear": {"model": "common", "part": "tire_R_176_68_13_standard"}},
                    "steering": {"lock_deg": 33, "turns": 3}}

    def test_vehicle_reads_back(self):
        out = json.loads(scratch.build(json.dumps(self.car)))
        self.assertEqual(sorted(out["files"]), ["vehicles/bf_test_scratch/base.pc", "vehicles/bf_test_scratch/bf_test_scratch.jbeam",
                                                "vehicles/bf_test_scratch/info.json"])
        bng.add_files(json.dumps(out["files"]))
        c = json.loads(bng.configure("bf_test_scratch", "base"))
        self.assertEqual(c["missing"], [])
        N = c["geometry"]["nodes"]
        # the rims' axle nodes on the wheel centres (BeamNG: x left, y rear, z up; front axle at y 0, ground at 0)
        self.assertEqual([round(x, 3) for x in N["fwhl1l"]], [0.615, 0.0, 0.29])
        self.assertEqual([round(x, 3) for x in N["rwhl1rr"]], [-0.835, 2.5, 0.29])
        self.assertEqual(sorted(w["name"] for w in c["wheels"]), ["FL", "FR", "RL", "RR"])
        self.assertEqual(json.loads(out["files"]["vehicles/bf_test_scratch/base.pc"])["parts"],
                         {"tire_F_13x5": "tire_F_186_68_13_sport", "tire_R_13x5": "tire_R_176_68_13_standard"})
        self.assertEqual(sum(out["counts"]["bands"][b] for b in ("high", "extreme", "beyond")), 0)     # stable by construction

    def test_powertrain_and_steering(self):
        out = json.loads(scratch.build(json.dumps(self.car)))
        body = json.loads(out["files"]["vehicles/bf_test_scratch/bf_test_scratch.jbeam"])["bf_test_scratch_body"]
        pt = [(r[0], r[1], r[2]) for r in body["powertrain"][1:]]
        self.assertEqual(pt[:3], [("combustionEngine", "mainEngine", "dummy"), ("frictionClutch", "clutch", "mainEngine"), ("manualGearbox", "gearbox", "clutch")])
        self.assertIn(("differential", "differential_R", "driveshaft_R"), pt)
        self.assertIn(("shaft", "spindleRL", "wheelaxleRL"), pt)
        self.assertEqual(body["gearbox"]["gearRatios"], [-3.5, 0, 3.2, 2.0, 1.4, 1.0, 0.8])            # reverse kept
        me = body["mainEngine"]
        self.assertEqual((me["maxRPM"], me["torque"][1], "breakTriggerBeam" in me, "radiator" in me), (7200.0, [1000.0, 120.0], False, False))
        self.assertEqual(body["clutch"], {"clutchFreePlay": 0.75})
        beams = {tuple(sorted(b[:2])) for b in body["beams"][2:]}
        self.assertIn(("eng1l", "eng1r"), beams)                                  # the engine block copied whole
        self.assertNotIn(("body_node", "eng1l"), beams)                         # not its mount to the donor's body
        hy = body["hydros"][3:]
        self.assertEqual(len(hy), 2)
        self.assertEqual(hy[0][2]["factor"], -hy[1][2]["factor"])            # the two hydros cross
        self.assertGreater(hy[0][2]["factor"], 0)                             # tie rods behind the axle: positive moves the rack left
        self.assertEqual(sorted(body["rails"]), ["bf_steeringrack"])

    def test_engine_values_and_differentials(self):
        car = dict(self.car, layout="AWD", diffs={"rear": {"type": "lsd_clutch", "preload": 60, "lock_power": 0.3, "lock_coast": 0.1}, "split": 0.35})
        car["engine"] = dict(car["engine"], inertia=0.12, friction=14, engine_brake=45, mass=120)
        body = json.loads(json.loads(scratch.build(json.dumps(car)))["files"]["vehicles/bf_test_scratch/bf_test_scratch.jbeam"])["bf_test_scratch_body"]
        me = body["mainEngine"]
        self.assertEqual((me["inertia"], me["friction"], me["engineBrakeTorque"]), (0.12, 14.0, 45.0))
        rows = {r[1]: r for r in body["powertrain"][1:]}
        self.assertEqual({k: rows["differential_R"][4][k] for k in ("diffType", "lsdPreload", "lsdLockCoef", "lsdRevLockCoef")},
                         {"diffType": "lsd", "lsdPreload": 60, "lsdLockCoef": 0.3, "lsdRevLockCoef": 0.1})
        self.assertEqual((rows["differential_F"][4]["diffType"], rows["differential_C"][4]["diffTorqueSplit"]), ("open", 0.35))
        engine_kg = sum(r[4]["nodeWeight"] for r in body["nodes"][5:] if r[0].startswith("e"))
        self.assertAlmostEqual(engine_kg, 120, delta=0.1)                    # the block's weights scaled to the engine's mass

    def test_tyre_with_the_users_values(self):
        car = json.loads(json.dumps(self.car))
        car["tyre"]["front"]["overrides"] = {"frictionCoef": 1.15, "loadSensitivitySlope": 0.0001, "bogus": 3}
        out = json.loads(scratch.build(json.dumps(car)))
        jb = json.loads(out["files"]["vehicles/bf_test_scratch/bf_test_scratch.jbeam"])
        t = jb["bf_test_scratch_tire_F"]
        self.assertEqual((t["slotType"], t["pressureWheels"][-1]), ("tire_F_13x5", {"frictionCoef": 1.15, "loadSensitivitySlope": 0.0001}))
        self.assertEqual(json.loads(out["files"]["vehicles/bf_test_scratch/base.pc"])["parts"]["tire_F_13x5"], "bf_test_scratch_tire_F")
        bng.add_files(json.dumps(out["files"]))
        w = {x["name"]: x for x in json.loads(bng.configure("bf_test_scratch", "base"))["wheels"]}
        self.assertEqual((w["FL"]["radius"], w["FL"]["width"]), (0.29, 0.15))   # the donor's size, kept

    def test_drag_plate(self):
        from beamforge import values
        out = json.loads(scratch.build(json.dumps(dict(self.car, aero={"cda": 0.65}))))
        bng.add_files(json.dumps(out["files"]))
        c = json.loads(bng.configure("bf_test_scratch", "base"))
        tris = values.aero_triangles("bf_test_scratch", c)
        self.assertEqual(len(tris), 1)
        self.assertAlmostEqual(values.drag_area(tris), 0.65, places=3)          # the drag area asked for, by the estimate
        body = json.loads(out["files"]["vehicles/bf_test_scratch/bf_test_scratch.jbeam"])["bf_test_scratch_body"]
        self.assertEqual((body["triangles"][1]["triangleType"], body["triangles"][1]["liftCoef"]), ("NONCOLLIDABLE", 0))

    def test_components_weigh_on_the_frame(self):
        plain = json.loads(scratch.build(json.dumps(self.car)))["counts"]["mass"]
        car = dict(self.car, components=[{"id": "driver", "kind": "driver", "mass": 75, "position": [-1.2, -0.3, -0.4]},
                                          {"id": "ballast", "kind": "ballast", "mass": 40, "position": [-2.6, 0, -0.2]}])
        out = json.loads(scratch.build(json.dumps(car)))
        self.assertAlmostEqual(out["counts"]["mass"] - plain, 115, delta=0.5)
        self.assertEqual(sum(out["counts"]["bands"][b] for b in ("high", "extreme", "beyond")), 0)   # heavier nodes: still stable

    def test_svj_body(self):
        import os
        import tempfile
        from beamforge import gltf
        svj = {"assets": {"meshes": [{"id": "m", "uri": "body.glb"}]}, "chassis": {"visual": {"mesh_ref": "m", "node": "SVJ::body::chassis"}}}
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "body.glb")
            Path(path).write_bytes(gltf.triangles_glb([[[0.5, 0, -1.0], [-3.0, 0, -1.0], [0.5, 0.8, -0.3]]], "SVJ::body::chassis"))
            out = json.loads(scratch.build(json.dumps(dict(self.car, body={"svj": svj, "files": {"m": path}}))))
        self.assertIn("vehicles/bf_test_scratch/bf_test_scratch_svj.dae", out["files"])
        body = json.loads(out["files"]["vehicles/bf_test_scratch/bf_test_scratch.jbeam"])["bf_test_scratch_body"]
        self.assertEqual(body["flexbodies"][1][:2], ["bf_test_scratch_svj_chassis", ["sk_frame"]])   # the body follows the frame

    def test_refused(self):
        bad = dict(self.car, engine={})
        with self.assertRaises(ValueError):
            scratch.build(json.dumps(bad))
        nof = dict(self.car, sketch={"parts": [p for p in self.car["sketch"]["parts"] if p["name"] != "frame"], "opts": {}})
        with self.assertRaises(ValueError):
            scratch.build(json.dumps(nof))


if __name__ == "__main__":
    unittest.main()
