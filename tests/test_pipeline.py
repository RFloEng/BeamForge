"""An SVJ onto a base vehicle in one call (beamforge/pipeline.py), on the made-up toy car, and the command-line tool's
arguments (tools/svj_to_beamng.py)."""

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from beamforge import beamng as bng, pipeline  # noqa: E402
from tests.test_beamng import CAR  # noqa: E402

SVJ = {"_metadata": {"version": "0.99.4"}, "chassis": {"mass_total": 900.0}, "assets": {"meshes": [{"id": "body", "uri": "body.glb"}]}}


class TestPipeline(unittest.TestCase):
    def setUp(self):
        bng.reset()
        bng.add_files(json.dumps(CAR))

    def test_run(self):
        r = pipeline.run("toycar", SVJ, "toynew", "Toy New", stages=(), base_paths=["vehicles/toycar/toycar.jbeam"])
        files = r["out"]["files"]
        self.assertIn("vehicles/toynew/info.json", files)
        self.assertEqual(json.loads(files["vehicles/toynew/info.json"])["Name"], "Toy New")
        self.assertEqual(r["fit"]["moves"], {})                       # no stages: no fit
        self.assertEqual(r["files"], {})                              # no folder given: no meshes
        bng.add_files(json.dumps(files))
        self.assertEqual(json.loads(bng.configure("toynew"))["missing"], [])

    def test_mesh_files(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(pipeline.svj_mesh_files(SVJ, d), {})       # not there: left out
            Path(d, "body.glb").write_bytes(b"glTF")
            self.assertEqual(pipeline.svj_mesh_files(SVJ, d), {"body": os.path.join(d, "body.glb")})

    def test_tool_needs_the_vehicles_folder(self):
        sys.path.insert(0, str(ROOT / "tools"))
        import svj_to_beamng
        env = os.environ.pop("BEAMNG_VEHICLES", None)
        try:
            with self.assertRaises(SystemExit):
                svj_to_beamng.main(["x.svj.json", "--base", "toycar", "--id", "toynew", "--vehicles", str(ROOT / "nowhere")])
        finally:
            if env is not None:
                os.environ["BEAMNG_VEHICLES"] = env


if __name__ == "__main__":
    unittest.main()
