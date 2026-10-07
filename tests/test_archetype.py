"""The strut archetype's pieces (beamforge/archetype.py)."""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from beamforge import archetype  # noqa: E402

HP = {"wheel_center": [0.75, -1.3, 0.3], "lower_ball_joint": [0.7, -1.31, 0.22], "steering_tie_rod_end": [0.7, -1.44, 0.25],
      "strut_outboard": [0.7, -1.31, 0.22], "strut.0": [0.6, -1.28, 0.8], "lower_control_arm.0": [0.33, -1.6, 0.22],
      "lower_control_arm.1": [0.35, -1.28, 0.23], "steering_tie_rod.0": [0.34, -1.4, 0.24]}


class TestStrut(unittest.TestCase):
    def test_points(self):
        p = archetype.strut_points(HP)
        self.assertNotIn("h2", p)                                   # strut bottom on the ball joint: one node
        self.assertTrue(p["steer"])
        a = [p["t"][i] - p["h1"][i] for i in range(3)]
        b = [p["h4"][i] - p["h1"][i] for i in range(3)]
        self.assertLess(math.dist([x / math.dist(p["t"], p["h1"]) for x in a],
                                  [x / math.dist(p["h4"], p["h1"]) for x in b]), 1e-9)   # h4 on the strut's axis
        self.assertAlmostEqual(p["h5"][1], 2 * HP["wheel_center"][1] - HP["steering_tie_rod_end"][1])
        self.assertIsNone(archetype.strut_points({k: v for k, v in HP.items() if k != "strut.0"}))

    def test_motion_ratio(self):
        mr = archetype.motion_ratio(archetype.strut_points(HP))
        self.assertTrue(0.95 < mr < 1.0)

    def test_corner_loads(self):
        f = archetype.corner_loads(1000, 1.0, 2.5)
        self.assertAlmostEqual(f["FL"] + f["RL"], 500 * 9.81)
        self.assertAlmostEqual(f["FL"], 300 * 9.81)


class TestSurgery(unittest.TestCase):
    def test_rows_using_removed_nodes_go(self):
        part = {"nodes": [["id", "posX", "posY", "posZ"], {"nodeWeight": 5}, ["a", 0, 0, 0], ["h", 1, 0, 0]],
                "beams": [["id1:", "id2:"], {"beamSpring": 1}, ["a", "h"], ["a", "b"]],
                "rails": {"r": {"links:": ["h", "b"]}, "s": {"links:": ["a", "b"]}},
                "slidenodes": [["id:", "railName"], ["x", "r"], ["y", "s"], ["h", "s"]],
                "flexbodies": [["mesh", "[group]:"], ["hubmesh", ["g_hub"]], ["armmesh", ["g_arm"]]]}
        dropped = archetype._drop_rows(part, {"h"}, {"g_hub": ["h"], "g_arm": ["a", "h"]})
        self.assertEqual(dropped, {"r"})
        self.assertEqual([r[0] for r in part["nodes"][1:] if isinstance(r, list)], ["a"])
        self.assertEqual([r for r in part["beams"][1:] if isinstance(r, list)], [["a", "b"]])
        self.assertEqual([r[0] for r in part["slidenodes"][1:]], ["y"])           # off the dropped rail, and h
        self.assertEqual([r[0] for r in part["flexbodies"][1:]], [])              # a mesh on any removed node goes (it would
                                                                                   # hang in place bound to the kept one)
        self.assertEqual(part["beams"][1], {"beamSpring": 1})                      # modifiers kept

    def test_wheel_remapped(self):
        part = {"pressureWheels": [["name", "hubGroup", "group", "node1:", "node2:", "nodeS", "nodeArm:", "wheelDir"],
                                   ["FL", "w", "t", "fwhl1ll", "fwhl1l", 9999, "fhub5l", -1,
                                    {"torqueCoupling:": "fhub1l", "torqueArm:": "fhub4l", "torqueArm2:": "fwhl1ll",
                                     "steerAxisUp:": "ftop1l", "steerAxisDown:": "fhub1l"}]]}
        names = {k: f"bfFL{k}" for k in ("h1", "h4", "h5", "t")}
        self.assertTrue(archetype._remap_wheels(part, "FL", names))
        row = part["pressureWheels"][1]
        self.assertEqual(row[6], "bfFLh5")
        self.assertEqual(row[-1], {"torqueCoupling:": "bfFLh1", "torqueArm:": "bfFLh4", "torqueArm2:": "fwhl1ll",
                                   "steerAxisUp:": "bfFLt", "steerAxisDown:": "bfFLh1"})

    def test_mount_weight_added_when_short(self):
        room = {"k": {"n": 1e6}, "c": {"n": 1000.0}, "lk": {"n": 2e6}, "lc": {"n": 1e3}, "kg": {}}
        archetype._take(room, "n", 3e6, 80)
        self.assertAlmostEqual(room["kg"]["n"], 1.0)                 # 2 MN/m short at 2 MN/m per kg
        self.assertAlmostEqual(room["k"]["n"], 0.0)


    def test_node_weighed_by_all_its_damping(self):
        part = {"nodes": [["id", "posX", "posY", "posZ"], ["bfRLt", 0, 0, 0, {"nodeWeight": 2.5}], ["x", 1, 0, 0]],
                "beams": [["id1:", "id2:"], {"beamType": "|BOUNDED", "beamSpring": 0, "beamDamp": 4500},
                          ["bfRLt", "x", {"beamDampRebound": 8400, "beamDampFast": 1700}]]}
        out = archetype._size_nodes([part])
        self.assertAlmostEqual(out["bfRLt"], 8400 / 2000 / archetype.NODE_C_INDEX, places=2)   # rebound counted
        self.assertNotIn("x", out)                                                            # base nodes left



class TestSuspensionMeshes(unittest.TestCase):
    def test_pieces_found_by_name_and_role(self):
        from beamforge import gltf
        # a corner group (SUSP_LF) holding a hub, an arm and a damper, a brake caliper under it, a wiper arm
        # and a body panel: the finest suspension parts are the pieces, the rest is not
        nodes = [{"name": "SUSP_LF", "children": [1, 2, 3, 4]}, {"name": "g_SUSP_LF_Hub", "mesh": 0},
                 {"name": "g_SUSP_LF_Lever_A", "mesh": 1}, {"name": "g_SUSP_LF_Dump", "mesh": 2},
                 {"name": "brake_caliper_LF", "mesh": 3}, {"name": "wiper_arm_left", "mesh": 4}, {"name": "body", "mesh": 5}]
        pieces = gltf._pieces({"nodes": nodes, "scenes": [{"nodes": [0, 5, 6]}]})
        self.assertEqual(pieces, [("g_SUSP_LF_Hub", "hub"), ("g_SUSP_LF_Lever_A", "arm"), ("g_SUSP_LF_Dump", "strut")])

    def test_corner_group_alone_is_a_piece_and_calipers_are_not(self):
        from beamforge import gltf
        nodes = [{"name": "SUSP_RR", "children": [1]}, {"name": "x0_hub_caliper_br", "mesh": 0}]
        self.assertEqual(gltf._pieces({"nodes": nodes, "scenes": [{"nodes": [0]}]}), [])      # only a caliper below it

    def test_base_node_joins_a_group(self):
        part = {"nodes": [["id", "posX", "posY", "posZ"], {"group": "a"}, ["n1", 0, 0, 0], ["n2", 1, 0, 0, {"nodeWeight": 3}]]}
        self.assertTrue(archetype._tag_base(part, "n2", ["a", "bf_arm_FL"]))
        self.assertEqual(part["nodes"][3][-1], {"nodeWeight": 3, "group": ["a", "bf_arm_FL"]})
        self.assertFalse(archetype._tag_base(part, "missing", ["x"]))


class TestVanillaMeshes(unittest.TestCase):
    def test_only_svj_and_wheel_meshes_stay(self):
        part = {"flexbodies": [["mesh", "[group]:"], {"pos": {"x": 0}}, ["compact_subframe_F", ["g"]], ["brake_disc_slotted", ["g"]],
                               ["wheel_02a_16x8", ["w"]], ["tire_01f_16x8_25", ["t"]], ["car_svj_susp_fl_x0_wishbone_fl", ["bf_arm_FL"]]]}
        n = archetype._drop_vanilla_meshes([{"p": part}])
        self.assertEqual(n, 2)
        self.assertEqual([r[0] for r in part["flexbodies"][1:] if isinstance(r, list)],
                         ["wheel_02a_16x8", "tire_01f_16x8_25", "car_svj_susp_fl_x0_wishbone_fl"])


class TestBoundParts(unittest.TestCase):
    """The suspension parts an SVJ v0.99.2 binds node by node (svj.part_bindings), and link_between_points."""
    SVJ = {"assets": {"meshes": [{"id": "s", "uri": "susp.glb"}]},
           "suspension": {"FL": {"topology": {"upright": {"id": "upright_fl", "hardpoints": {"lower_ball_joint": [0.0, -0.7, 0.2]},
                                                          "visual": {"node": "SVJ::suspension::upright_fl"}},
                                              "links": [{"name": "lower_wishbone", "inboard_points": [[0.2, -0.3, 0.2], [-0.2, -0.3, 0.2]],
                                                         "outboard_ref": "hardpoints.lower_ball_joint",
                                                         "visual": {"node": "SVJ::suspension::lower_wishbone_fl",
                                                                    "placement": "link_between_points", "mesh_axis": "+y"}},
                                                        {"name": "tie_rod", "inboard_points": [[0.1, -0.3, 0.3]],
                                                         "visual": {"node": "SVJ::suspension::tie_rod_fl"}}]},
                          "spring": {"visual": {"node": "SVJ::suspension::spring_fl"}},
                          "arb": {"visual": {"node": "SVJ::suspension::arb_fl"}}}}}

    def test_roles_and_bindings(self):
        from beamforge import svj
        self.assertEqual([svj.part_role(n) for n in ("upright_fl", "upper_wishbone_fr", "tie_rod_rl", "toe_link_rr", "spring_fl",
                                                      "trailing_arm_rl", "arb_fl", "knuckle")],
                         ["hub", "arm", "tie", "tie", "strut", "arm", None, "hub"])
        b = {x["part"]: x for x in svj.part_bindings(self.SVJ)}
        self.assertEqual([b[k]["role"] for k in ("upright", "lower_wishbone", "tie_rod", "spring", "arb")],
                         ["hub", "arm", "tie", "strut", None])
        w = b["lower_wishbone"]
        self.assertEqual((w["placement"], w["mesh_axis"], w["outboard"], len(w["inboard"])), ("link_between_points", "+y", [0.0, -0.7, 0.2], 2))
        self.assertIsNone(b["tie_rod"]["inboard"])                   # rigid: no placement
        self.assertEqual(b["upright"]["mesh_ref"], "s")              # the only mesh

    def test_link_place(self):
        from beamforge import export
        pts = [[0.0, 0.0, 0.0], [0.0, 0.4, 0.0]]                      # a wishbone along its own +y, pivot at the origin
        inb, out = [[0.2, -0.3, 0.2], [-0.2, -0.3, 0.2]], [0.0, -0.7, 0.2]
        placed = export.link_place(pts, "+y", inb, out)
        self.assertEqual([round(c, 6) for c in placed[0]], [0.0, -0.3, 0.2])        # origin on the pivots' centre
        self.assertAlmostEqual(math.dist(placed[0], placed[1]), 0.4, places=6)       # kept its size
        d = [placed[1][i] - placed[0][i] for i in range(3)]
        self.assertAlmostEqual(d[1], -0.4, places=6)                                  # pointing outboard (-y)
        self.assertEqual([round(c, 6) for c in export.link_place(pts, "+y", inb, out, None, True)[1]], out)   # stretched onto it
        slant = export.link_place(pts, "+y", [[0.0, 0.0, 0.0]], [0.3, 0.4, 0.0])      # turned onto a slanted link
        self.assertEqual([round(c, 6) for c in slant[1]], [0.24, 0.32, 0.0])
        flip = export.link_place(pts, "-y", [[0.0, 0.0, 0.0]], [0.0, 1.0, 0.0])       # axis opposite to the link
        self.assertEqual([round(c, 6) for c in flip[1]], [0.0, -0.4, 0.0])           # the mesh's -y runs along the link: its +y end goes the other way

    def test_bound_pieces_from_the_file(self):
        import os
        import tempfile
        from beamforge import export, gltf
        tri = [[0.0, 0.0, 0.0], [0.0, 0.4, 0.0], [0.0, 0.0, 0.05]]
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "susp.glb")
            Path(path).write_bytes(gltf.triangles_glb([tri], "SVJ::suspension::lower_wishbone_fl"))   # the file has this one only
            v = {"wheels": [{"name": "FL"}, {"name": "FR"}]}
            got = export.susp_pieces(v, self.SVJ, path, {"yf": 0.0, "ground": 0.0}, {"s": path})
        self.assertEqual([(p["node"], p["role"], p["corner"]) for p in got], [("SVJ::suspension::lower_wishbone_fl", "arm", "FL")])
        self.assertEqual(got[0]["link"]["axis"], "+y")


if __name__ == "__main__":
    unittest.main()
