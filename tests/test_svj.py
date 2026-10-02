"""SVJ reading: frames, bundles with glTF meshes, hardpoints, and the base-vs-SVJ comparison.

The SVJ below is written for the tests (a minimal SAE J670 file with one MacPherson corner).
"""

import json
import sys
import tempfile
import unittest
import zipfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from beamforge import gltf, svj  # noqa: E402

DOC = {
    "_metadata": {"specification": "SVJ", "version": "0.99.1", "coordinate_system": "SAE_J670", "units": "SI"},
    "vehicle_info": {"make": "Toy", "model": "Car"},
    "chassis": {"wheelbase": 2.5, "track_front": 1.45, "track_rear": 1.44, "mass_total": 1100,
                "visual": {"mesh_ref": "body", "node": "SVJ::body::chassis"}},
    "assets": {"meshes": [{"id": "body", "uri": "meshes/body.glb"}]},
    "suspension": {"FL": {"topology": {"system_type": "macpherson",
                                       "upright": {"hardpoints": {"wheel_center": [0.0, -0.725, -0.30]}},
                                       "links": [{"name": "lower_control_arm",
                                                  "inboard_points": [[0.2, -0.35, -0.20], [-0.1, -0.35, -0.20]]}]}}},
}


class TestFrames(unittest.TestCase):
    def test_round_trip_and_axes(self):
        p = [1.0, 0.5, -0.3]                         # 1 m ahead of the front axle, 0.5 m right, 0.3 m up
        b = svj.from_sae(p, yf=-1.2, zg=0.1)
        self.assertEqual(b, [-0.5, -2.2, 0.4])        # BeamNG: right is -X, forward is -Y, up is +Z
        self.assertEqual([round(c, 9) for c in svj.to_sae(b, -1.2, 0.1)], p)


class TestBundle(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())

    def _zip(self, doc, meshes):
        z = self.tmp / "car.zip"
        with zipfile.ZipFile(z, "w") as f:
            f.writestr("car.svj.json", json.dumps(doc))
            for name, data in meshes.items():
                f.writestr(name, data)
        return z

    def test_meshes_found_and_bindings_checked(self):
        glb = gltf.triangles_glb([[[0, 0, 0], [1, 0, 0], [0, 1, 0]]], "SVJ::body::chassis")
        r = json.loads(svj.load_bundle(str(self._zip(DOC, {"meshes/body.glb": glb})), str(self.tmp / "in")))
        self.assertEqual(r["notes"], [])
        self.assertEqual(r["meshes"][0]["nodes"], ["SVJ::body::chassis"])
        self.assertEqual(r["summary"]["vehicle"], "Toy Car")
        self.assertEqual(r["summary"]["corners"], {"FL": "macpherson"})
        self.assertEqual(r["axes"], {"up": "Y", "forward": "-Z"})

    def test_missing_mesh_and_wrong_node_are_reported(self):
        r = json.loads(svj.load_bundle(str(self._zip(DOC, {})), str(self.tmp / "a")))
        self.assertTrue(any("was not provided" in n for n in r["notes"]))
        glb = gltf.triangles_glb([[[0, 0, 0], [1, 0, 0], [0, 1, 0]]], "SVJ::body::other")
        r = json.loads(svj.load_bundle(str(self._zip(DOC, {"meshes/body.glb": glb})), str(self.tmp / "b")))
        self.assertTrue(any("not found in meshes/body.glb" in n for n in r["notes"]))

    def test_unsafe_zip_path_is_refused(self):
        z = self._zip(DOC, {"../evil.txt": b"x"})
        with self.assertRaises(ValueError):
            svj.load_bundle(str(z), str(self.tmp / "in"))


class TestHardpointsAndCompare(unittest.TestCase):
    def test_hardpoints_in_beamng_frame(self):
        hp = {h["name"]: h for h in svj.hardpoints(DOC, yf=-1.3, zg=0.0)}
        self.assertEqual(hp["wheel_center"]["pos"], [0.725, -1.3, 0.3])      # left front wheel: +X, at the axle, 0.3 up
        self.assertEqual(hp["lower_control_arm.0"]["kind"], "chassis")
        self.assertAlmostEqual(hp["lower_control_arm.0"]["pos"][1], -1.5)    # 0.2 m ahead of the axle

    def test_compare_rows(self):
        rows = {r["key"]: r for r in svj.compare(DOC, {"wheelbase": 2.47, "track_front": 1.45, "track_rear": None})}
        self.assertAlmostEqual(rows["wheelbase"]["delta"], 0.03)
        self.assertEqual(rows["track_front"]["delta"], 0.0)
        self.assertIsNone(rows["track_rear"]["delta"])
        self.assertIsNone(rows["mass_total"]["base"])


if __name__ == "__main__":
    unittest.main()
