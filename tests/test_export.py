"""A new vehicle from the edited base vehicle (beamforge/export.py), on the made-up toy car."""

import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from beamforge import beamng as bng  # noqa: E402
from beamforge import export  # noqa: E402
from tests.test_beamng import CAR, find  # noqa: E402


class TestExport(unittest.TestCase):
    def setUp(self):
        bng.reset()
        bng.add_files(json.dumps(CAR))
        moves = {"nodes": {"b1": [0.0, 0.1, 0.0], "fwhl1l": [0.02, 0.0, 0.0]}}
        self.configured = bng.configure("toycar", None, None, None, json.dumps(moves))

    def test_plan(self):
        plan = {p["part"]: p for p in json.loads(export.plan("toycar", self.configured))}
        self.assertEqual((plan["toycar"]["origin"], plan["toycar"]["choice"], plan["toycar"]["moved_mm"]), ("vehicle", "fit", 100.0))
        self.assertEqual((plan["steel_wheel_F"]["origin"], plan["steel_wheel_F"]["choice"]), ("common", "fit"))
        self.assertEqual(plan["toycar_engine_small"]["choice"], "copy")              # not moved: copied as it is
        self.assertEqual(plan["steel_wheel_F"]["choices"], ["reuse", "fit", "copy"])

    def test_build(self):
        out = json.loads(export.build("toycar", "toyfit", "Toy Fit", self.configured, "{}", brand="BeamForge"))
        f = out["files"]
        body = json.loads(f["vehicles/toyfit/toycar.jbeam"])["toycar"]
        b1 = next(r for r in body["nodes"][1:] if r[0] == "b1")
        self.assertEqual(b1[1:4], [0.5, -0.9, 0.3])                                 # fitted: -1.0 + 0.1
        self.assertIn("toycar_spoiler_wing", json.loads(f["vehicles/toyfit/toycar_hub.jbeam"]))   # inactive parts kept
        self.assertEqual(out["renamed"], {"steel_wheel_F": "toyfit_steel_wheel_F"})
        shared = json.loads(f["vehicles/toyfit/toyfit_shared_parts.jbeam"])
        fw = next(r for r in shared["toyfit_steel_wheel_F"]["nodes"][1:] if r[0] == "fwhl1l")
        self.assertAlmostEqual(fw[1], 0.33)                                         # as written (0.31) + the move
        hub = json.loads(f["vehicles/toyfit/toycar_hub.jbeam"])["toycar_hub_F"]
        self.assertEqual(hub["slots"][1][1], "toyfit_steel_wheel_F")                # slot default follows the new name
        pc = json.loads(f["vehicles/toyfit/beamforge.pc"])
        self.assertEqual((pc["model"], pc["parts"]["wheel_F_4"]), ("toyfit", "toyfit_steel_wheel_F"))
        self.assertEqual(json.loads(f["vehicles/toyfit/sport.pc"])["model"], "toyfit")
        info = json.loads(f["vehicles/toyfit/info.json"])
        self.assertEqual((info["Name"], info["Brand"], info["default_pc"]), ("Toy Fit", "BeamForge", "beamforge"))
        self.assertNotIn("vehicles/toycar/toycar.jbeam", f)                        # nothing of the base is rewritten

    def test_added_beams(self):
        from beamforge import rigidity
        v = json.loads(self.configured)
        x = rigidity.added_beam("toycar", v, "b1", "fwhl1l")
        self.assertEqual((x["nodes"]["b1"]["after"], x["limited"]), ("ok", False))
        self.assertTrue(any("suspension" in w for w in x["warnings"]))               # a wheel node: it would hold the wheel
        self.assertGreater(x["beamSpring"], 0)
        typed = rigidity.added_beam("toycar", v, "b1", "b2", [], want_k=1e10)
        self.assertTrue(typed["limited"])
        self.assertEqual(typed["beamSpring"], typed["k_room"])                       # held at what the nodes can take
        after = rigidity.added_beam("toycar", v, "b1", "b2", [{"a": "b1", "b": "b2", **typed}])
        self.assertEqual(after["k_room"], 0.0)                                       # the first one used all the room
        self.assertRaises(ValueError, rigidity.added_beam, "toycar", v, "b1", "b1")
        added = [{"a": "b1", "b": "b2", "beamSpring": x["beamSpring"], "beamDamp": x["beamDamp"], "beamDeform": 1e5},
                 {"a": "b1", "b": "gone", "beamSpring": 1, "beamDamp": 1}]
        out = json.loads(export.build("toycar", "toyfit", "Toy Fit", self.configured, "{}", None, None, json.dumps(added)))
        rows = json.loads(out["files"]["vehicles/toyfit/toycar.jbeam"])["toycar"]["beams"]
        self.assertEqual(rows[-1][:2], ["b1", "b2"])
        self.assertEqual((rows[-1][2]["beamSpring"], rows[-1][2]["beamType"], rows[-1][2]["beamStrength"]),
                         (x["beamSpring"], "|NORMAL", "FLT_MAX"))
        self.assertEqual(out["counts"]["added_beams"], 1)
        self.assertTrue(any("left out" in n for n in out["notes"]))

    def test_reuse_and_copy_choices(self):
        out = json.loads(export.build("toycar", "toyfit", "Toy Fit", self.configured,
                                      json.dumps({"steel_wheel_F": "reuse", "toycar": "copy"})))
        self.assertEqual(out["renamed"], {})
        self.assertNotIn("vehicles/toyfit/toyfit_shared_parts.jbeam", out["files"])
        body = json.loads(out["files"]["vehicles/toyfit/toycar.jbeam"])["toycar"]
        self.assertEqual(next(r for r in body["nodes"][1:] if r[0] == "b1")[1:4], [0.5, -1.0, 0.3])   # copied as it is
        self.assertTrue(any("without their edits" in n for n in out["notes"]))

    def test_ids_and_assets(self):
        self.assertEqual(export.check_id("toyfit", "toycar"), [])
        self.assertTrue(export.check_id("toycar", "toycar"))
        self.assertTrue(export.check_id("Toy Fit", "toycar"))
        with self.assertRaises(ValueError):
            export.build("toycar", "common", "x", self.configured, "{}")
        a = json.loads(export.assets("toycar", "toyfit", json.dumps([
            "vehicles/toycar/toycar.dae", "vehicles/toycar/main.materials.json", "vehicles/toycar/tex/body_d.dds",
            "vehicles/toycar/toycar.jbeam", "vehicles/toycar/base.pc", "vehicles/toycar/info.json", "vehicles/toycar/base.jpg",
            "vehicles/common/x.dae"])))
        self.assertEqual(a, {"vehicles/toycar/toycar.dae": "vehicles/toyfit/toycar.dae",
                             "vehicles/toycar/main.materials.json": "vehicles/toyfit/main.materials.json",
                             "vehicles/toycar/base.jpg": "vehicles/toyfit/base.jpg"})

    def test_svj_meshes_as_collada(self):
        """An SVJ body (one triangle) written as COLLADA in the vehicle's frame and added as a flexbody."""
        import os
        import tempfile
        import xml.etree.ElementTree as ET
        from beamforge import gltf, svj as svjmod
        sae = [[0.5, 0.0, -1.0], [-3.5, 0.0, -1.0], [0.5, 0.9, 0.0]]
        doc = {"assets": {"meshes": [{"id": "m", "uri": "body.glb"}]},
               "chassis": {"visual": {"mesh_ref": "m", "node": "SVJ::body::chassis"}}}
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "body.glb")
            Path(path).write_bytes(gltf.triangles_glb([sae], "SVJ::body::chassis"))
            attach = json.loads(export.svj_attach(self.configured, json.dumps(doc)))
            self.assertEqual([(a["path"], a["node"]) for a in attach], [("chassis", "SVJ::body::chassis")])
            attach[0]["part"], attach[0]["groups"] = "toycar", ["toycar_body"]
            opt = {"svj": doc, "files": {"m": path}, "place": {"yf": -1.25, "ground": 0.0}, "attach": attach, "replace": True}
            out = json.loads(export.build("toycar", "toyfit", "Toy Fit", self.configured, "{}", None, json.dumps(opt)))
        f = out["files"]
        self.assertEqual(out["counts"]["svj_meshes"], 1)
        body = json.loads(f["vehicles/toyfit/toycar.jbeam"])["toycar"]
        self.assertEqual(body["flexbodies"][-1][:2], ["toyfit_svj_chassis", ["toycar_body"]])
        root = ET.fromstring(f["vehicles/toyfit/toyfit_svj.dae"])
        ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
        self.assertEqual(root.find(".//c:up_axis", ns).text, "Z_UP")
        self.assertEqual(root.find(".//c:node", ns).get("name"), "toyfit_svj_chassis")
        self.assertEqual(root.find(".//c:triangles", ns).get("count"), "1")
        xyz = [float(x) for x in root.find(".//c:float_array", ns).text.split()]
        want = [c for p in sae for c in svjmod.from_sae(p, -1.25, 0.0)]       # in the vehicle's frame
        for a, b in zip(xyz, want):
            self.assertAlmostEqual(a, b, places=4)
        mats = json.loads(f["vehicles/toyfit/toyfit_svj.materials.json"])
        self.assertEqual(mats["toyfit_svj0_panel_0"]["mapTo"], "toyfit_svj0_panel_0")    # the glTF material, by name
        self.assertEqual(mats["toyfit_svj0_panel_0"]["Stages"][0]["baseColorFactor"], [0.8, 0.82, 0.86, 1.0])
        self.assertTrue(mats["toyfit_svj0_panel_0"]["doubleSided"])

    def test_panel_names(self):
        """Body panels found by their mesh node names (as converters and modders write them)."""
        kind = lambda n: next((k for k, p, _ in export.PANELS if p.search(n)), None)  # noqa: E731
        cases = {"DOOR_L": "door", "Door_RIGHT": "door", "MOTORHOOD": "hood", "bonnet003v": "hood", "REARHOOD": "trunk",
                 "Trunk": "trunk", "FRONT_BUMPER": "bumper_F", "REARE_BUMPER": "bumper_R", "g_Bumper_R": "bumper_R",
                 "paraurti ant": "bumper_F", "paraurti_post": "bumper_R", "CHASSIS_mesh": None, "COCKPIT_LR": None}
        for name, want in cases.items():
            self.assertEqual(kind(name), want, name)

    def test_svj_named_panels(self):
        """A door named by the SVJ convention goes on the base's door on its side, and leaves the body."""
        import base64
        import os
        import struct
        import tempfile
        from beamforge import svj as svjmod

        def tri(sae):                                     # a triangle around an SAE point, in glTF axes
            return [svjmod.sae_to_gltf([sae[0] + dx, sae[1] + dy, sae[2] + dz]) for dx, dy, dz in
                    ((0, 0, 0), (0.5, 0, 0), (0, 0, -0.5))]
        body, door = tri([-1.0, 0.0, -0.8]), tri([-1.0, -0.8, -0.6])          # the door on the left (SAE Y < 0)
        raw = b"".join(struct.pack("<3f", *p) for p in body + door)
        doc = {"asset": {"version": "2.0"}, "scene": 0, "scenes": [{"nodes": [0, 1]}],
               "nodes": [{"name": "SVJ::body::chassis", "mesh": 0}, {"name": "SVJ::body::door_fl", "mesh": 1}],
               "meshes": [{"primitives": [{"attributes": {"POSITION": 0}}]}, {"primitives": [{"attributes": {"POSITION": 1}}]}],
               "buffers": [{"byteLength": len(raw), "uri": "data:application/octet-stream;base64," + base64.b64encode(raw).decode()}],
               "bufferViews": [{"buffer": 0, "byteOffset": 0, "byteLength": 36}, {"buffer": 0, "byteOffset": 36, "byteLength": 36}],
               "accessors": [{"bufferView": 0, "componentType": 5126, "count": 3, "type": "VEC3"},
                             {"bufferView": 1, "componentType": 5126, "count": 3, "type": "VEC3"}]}
        svj = {"assets": {"meshes": [{"id": "m", "uri": "car.gltf"}]}, "chassis": {"visual": {"mesh_ref": "m", "node": "SVJ::body::chassis"}}}
        v = {"geometry": {"nodes": {"a": [0.8, -0.2, 0.6], "b": [-0.8, -0.2, 0.6], "c": [0.0, 0.0, 0.5]},
                          "parts": {"a": "car_door_L", "b": "car_door_R", "c": "car_body"},
                          "groups": {"a": ["car_door_L"], "b": ["car_door_R"], "c": ["car_body"]}},
             "flexbodies": [{"part": "car_door_L", "mesh": "door_L", "groups": ["car_door_L"]},
                            {"part": "car_door_R", "mesh": "door_R", "groups": ["car_door_R"]}]}
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "car.gltf")
            Path(path).write_text(json.dumps(doc))
            rows = json.loads(export.svj_attach(json.dumps(v), json.dumps(svj), "[]", json.dumps({"m": path}),
                                                json.dumps({"yf": -1.25, "ground": 0.0})))
            door_row = next(r for r in rows if r["node"] == "SVJ::body::door_fl")
            self.assertEqual((door_row["part"], door_row["groups"]), ("car_door_L", ["car_door_L"]))
            body_row = next(r for r in rows if r["path"] == "chassis")
            self.assertEqual(body_row["exclude"], ["SVJ::body::door_fl"])
            for r in rows:
                r["part"] = r["part"] or "car_body"
            meshes = export.svj_meshes(svj, {"m": path}, {"yf": -1.25, "ground": 0.0}, rows, "x")
            self.assertEqual([len(m["indices"]) for m in meshes], [3, 3])                 # one triangle each, no overlap

    def test_base_meshes_dropped_but_wheels_and_lights(self):
        """The SVJ is the car's looks: of the base's meshes only the wheels and tyres stay, and the lights."""
        part = {"flexbodies": [["mesh", "[group]:", "nonFlexMaterials"], ["car_body", ["b"]], ["car_door_FL", ["d"]],
                               ["car_seat_FL", ["s"]], ["car_windshield_int", ["g"]], {"deformGroup": ""}, ["brake_disc", ["w"]],
                               ["car_exhaust_heatshield", ["b"]], ["car_steeringwheel", ["s"]], ["wheel_02a", ["w"]]],
                "props": [["func", "mesh", "idRef:"], ["steering", "car_steer", "a"], ["rpm", "car_needle_tacho", "a"],
                          ["$electric", "SPOTLIGHT", "a"], ["signal", "", "a"]]}
        self.assertEqual(export._drop_body_meshes("car_body", part), 7 + 2)
        self.assertEqual([r[0] for r in part["flexbodies"][1:] if isinstance(r, list)], ["wheel_02a"])
        self.assertEqual([r[0] for r in part["props"][1:]], ["$electric", "signal"])          # the lights stay, the needles go
        self.assertEqual(export._drop_body_meshes("car_wheel_F", {"flexbodies": [["mesh"], ["rim", []]]}), 0)

    def test_new_vehicle_builds(self):
        """The generated files, read back as the game would (new folder + common), build the edited shape."""
        out = json.loads(export.build("toycar", "toyfit", "Toy Fit", self.configured, "{}"))
        bng.reset()
        files = {p: t for p, t in CAR.items() if p.startswith("vehicles/common/")}
        files.update(out["files"])
        bng.add_files(json.dumps(files))
        v = json.loads(bng.configure("toyfit"))
        self.assertEqual(v["config"], "beamforge")
        g = v["geometry"]["nodes"]
        self.assertEqual(g["b1"], [0.5, -0.9, 0.3])
        self.assertEqual(g["fwhl1l"], [0.58, -1.2, 0.3])                              # 0.56 + 0.02, through the slot offset
        self.assertEqual(find(v["tree"], "wheel_F_4")["part"], "toyfit_steel_wheel_F")


@unittest.skipUnless(__import__("os").environ.get("BEAMNG_VEHICLES"), "set BEAMNG_VEHICLES to a BeamNG content/vehicles folder")
class TestLocalInstallExport(unittest.TestCase):
    """Every car and truck of a local install, edited, exported with every part regenerated, and read
    back as the game would (new folder + common): every node where the edit put it, no part lost."""

    def test_every_vehicle_exports(self):
        import math
        import os
        import zipfile
        lib = Path(os.environ["BEAMNG_VEHICLES"])
        bng.reset()
        with zipfile.ZipFile(lib / "common.zip") as z:
            bng.add_files(json.dumps({n: z.read(n).decode("utf-8-sig", "replace") for n in z.namelist() if n.endswith(".jbeam")}))
        for zp in sorted(lib.glob("*.zip")):
            if zp.name == "common.zip":
                continue
            with zipfile.ZipFile(zp) as z:
                texts = {n: z.read(n).decode("utf-8-sig", "replace") for n in z.namelist() if n.endswith((".jbeam", ".pc", ".json"))}
            info = next((bng.jbeam.parse(t) for n, t in texts.items() if n.count("/") == 2 and n.endswith("/info.json")), {})
            if not isinstance(info, dict) or info.get("Type") not in ("Car", "Truck"):
                continue
            model = next(iter({n.split("/")[1] for n in texts if n.startswith("vehicles/")}))
            with self.subTest(model=model):
                bng.add_files(json.dumps(texts))
                base = json.loads(bng.configure(model))
                moved = {n: [0.01, -0.02, 0.005] for n in list(base["geometry"]["nodes"])[::7]}
                configured = bng.configure(model, base["config"], None, None, json.dumps({"nodes": moved}))
                edited = json.loads(configured)["geometry"]["nodes"]
                plan = json.loads(export.plan(model, configured))
                new_id = f"bf_{model.lower()}"[:40]
                out = json.loads(export.build(model, new_id, "BF", configured, json.dumps({p["part"]: "fit" for p in plan})))
                bng.add_files(json.dumps(out["files"]))
                nv = json.loads(bng.configure(new_id))
                ng = nv["geometry"]["nodes"]
                self.assertEqual(set(ng), set(edited))
                self.assertLess(max(math.dist(ng[n], edited[n]) for n in edited), 2e-4)     # positions are kept to 0.1 mm
                self.assertEqual(nv["missing"], base["missing"])                          # nothing lost (some vanilla
                #                                                                           slots name parts that do not exist)


if __name__ == "__main__":
    unittest.main()
