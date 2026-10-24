"""gltf.guessed_pieces: the suspension's parts found by name, never a container of the whole car."""

import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from beamforge import gltf  # noqa: E402


def gltf_of(tree):
    """A .gltf (bytes) of nested nodes: {name: children dict or None (a mesh node)}; no buffers, only the structure."""
    nodes, meshes = [], []

    def add(name, kids):
        i = len(nodes)
        nodes.append({"name": name})
        if kids is None:
            nodes[i]["mesh"] = len(meshes)
            meshes.append({"primitives": []})
        else:
            nodes[i]["children"] = [add(n, k) for n, k in kids.items()]
        return i
    roots = [add(n, k) for n, k in tree.items()]
    return json.dumps({"asset": {"version": "2.0"}, "scene": 0, "scenes": [{"nodes": roots}], "nodes": nodes, "meshes": meshes}).encode()


class TestPieces(unittest.TestCase):
    def test_a_converters_root_node_is_no_axle(self):           # AC conversions hang the whole car under "_ac_front_axle_align"
        data = gltf_of({"_ac_z_rotation": {"_ac_front_axle_align": {"car": {"body_shell": None, "SVJ::wheel::wheel_fl": {"tyre_lf": None}}}}})
        self.assertEqual(gltf.guessed_pieces(data, False), [])

    def test_an_axle_housing_with_its_shocks_is_found(self):
        data = gltf_of({"root": {"REAR_AXLE": None, "suspensions": {"x0_suspension_br": None}}})
        found = dict(gltf.guessed_pieces(data, False))
        self.assertEqual(found.get("x0_suspension_br"), "strut")
        self.assertEqual(found.get("REAR_AXLE"), "axle")

    def test_arms_found_by_their_spanish_name_not_the_drivers_arms(self):
        data = gltf_of({"root": {"brazo_LF": None, "SVJ::body::brazos": {"copiloto": None}}})
        self.assertEqual([n for n, _ in gltf.guessed_pieces(data, False)], ["brazo_LF"])


if __name__ == "__main__":
    unittest.main()
