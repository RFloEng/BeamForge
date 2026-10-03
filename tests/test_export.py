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


if __name__ == "__main__":
    unittest.main()
