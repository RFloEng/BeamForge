"""Jbeam structures from STEP skeletons (beamforge/skeleton.py), on made-up assemblies."""

import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from beamforge import skeleton as sk  # noqa: E402

# a double wishbone corner (mm, SVJ frame): a flat frame, two A-arms hinged on it, an upright, a tie rod
CORNER = [
    {"name": "frame", "lines": [[[300, -300, -200], [-300, -300, -200]], [[300, -300, -450], [-300, -300, -450]],
                                [[300, -300, -200], [300, -300, -450]], [[-300, -300, -200], [-300, -300, -450]],
                                [[300, -300, -450], [0, -300, -650]], [[-300, -300, -450], [0, -300, -650]]]},
    {"name": "lower_wishbone_fl", "lines": [[[300, -300, -200], [0, -700, -180]], [[-300, -300, -200], [0, -700, -180]]]},
    {"name": "upper_wishbone_fl", "lines": [[[300, -300, -450], [0, -650, -470]], [[-300, -300, -450], [0, -650, -470]]]},
    {"name": "upright_fl", "lines": [[[0, -700, -180], [0, -650, -470]], [[0, -650, -470], [120, -680, -300]],
                                     [[120, -680, -300], [0, -700, -180]]]},
    {"name": "tie_rod_fl", "lines": [[[120, -300, -300], [120, -680, -300]]]},
]


class TestRead(unittest.TestCase):
    def test_round_trip_assembly(self):
        r = sk.read(sk.write(CORNER))
        self.assertEqual([(p["name"], len(p["lines"])) for p in r["parts"]],
                         [("frame", 6), ("lower_wishbone_fl", 2), ("upper_wishbone_fl", 2), ("upright_fl", 3), ("tie_rod_fl", 1)])
        self.assertEqual((r["unit"], r["unit_name"], r["skipped"]), (0.001, "mm", {}))

    def test_part_placed_in_the_assembly(self):
        """A part drawn in its own frame (a link along its +X) is placed by its assembly transform."""
        txt = sk.write([{"name": "link", "lines": [[[0, 0, 0], [1, 0, 0]]]}], unit="m", placements={"link": ([1, 2, 3], [0, 1, 0], [0, 0, 1])})
        r = sk.read(txt)
        self.assertEqual(r["unit"], 1.0)
        self.assertEqual([[round(c, 6) for c in p] for p in r["parts"][0]["lines"][0]], [[1, 2, 3], [1, 3, 3]])

    def test_bare_curve_set_and_unknown_curves(self):
        bare = ("ISO-10303-21;HEADER;ENDSEC;DATA;#1=CARTESIAN_POINT('',(0.,0.,0.));#2=CARTESIAN_POINT('',(100.,0.,0.));"
                "#3=POLYLINE('',(#1,#2));#5=CIRCLE('',#6,10.);#6=AXIS2_PLACEMENT_3D('',#1,$,$);"
                "#4=GEOMETRIC_CURVE_SET('Rocker_FL:1',(#3,#5));ENDSEC;END-ISO-10303-21;")
        r = sk.read(bare)
        self.assertEqual([(p["name"], len(p["lines"])) for p in r["parts"]], [("rocker_fl", 1)])
        self.assertEqual(r["skipped"], {"CIRCLE": 1})                 # reported, not read
        self.assertTrue(r["unit_name"].startswith("mm (assumed"))

    def test_trimmed_line(self):
        txt = ("ISO-10303-21;HEADER;ENDSEC;DATA;#1=CARTESIAN_POINT('',(0.,0.,0.));#2=DIRECTION('',(0.,1.,0.));"
               "#3=VECTOR('',#2,1.);#4=LINE('',#1,#3);#5=TRIMMED_CURVE('',#4,(PARAMETER_VALUE(10.)),(PARAMETER_VALUE(60.)),.T.,.PARAMETER.);"
               "#6=GEOMETRIC_CURVE_SET('arm',(#5));ENDSEC;END-ISO-10303-21;")
        self.assertEqual(sk.read(txt)["parts"][0]["lines"], [[[0.0, 10.0, 0.0], [0.0, 60.0, 0.0]]])

    def test_frame_rotation(self):
        f = sk.place_fn(0.001, (0, 180, 0), (0.1, 0, 0))             # CAD X rearward, Z up -> SVJ X forward, Z down
        self.assertEqual([round(c, 6) for c in f([1000, 0, 500])], [-0.9, 0.0, -0.5])


class TestBuild(unittest.TestCase):
    def test_counts_on_simple_parts(self):
        """One line: 1 beam. Two lines with three ends: 3. Four ends in space: 6 (3n - 6)."""
        for lines, beams in (([[[0, 0, 0], [1000, 0, 0]]], 1),
                             ([[[0, 0, 0], [1000, 0, 0]], [[1000, 0, 0], [1000, 800, 0]]], 3),
                             ([[[0, 0, 0], [1000, 0, 0]], [[1000, 0, 0], [1000, 800, 0]], [[1000, 800, 0], [1000, 800, 600]]], 6)):
            r = sk.build([{"name": "p", "lines": lines}])
            self.assertEqual((r["report"]["beams"], r["report"]["helper_nodes"], r["report"]["not_rigid"]), (beams, 0, []))

    def test_corner(self):
        r = sk.build(sk.read(sk.write(CORNER))["parts"])
        rep = r["report"]
        self.assertEqual((rep["nodes"], rep["helper_nodes"], rep["bodies"], rep["not_rigid"]), (10, 1, 5, []))
        self.assertEqual(rep["joints"], {"ball": 3, "hinge": 2, "weld": 0})
        # free: the body's 6, the wheel's travel, and the tie rod's inner end (3, no rack in this skeleton)
        self.assertEqual((rep["free_motions"], rep["mechanism"]), (10, 4))
        frame = next(b for b in r["bodies"] if b["parts"] == ["frame"])
        self.assertEqual((frame["shape"], frame["helpers"], frame["rank"], frame["need"]), ("flat", 1, 12, 12))
        # a shared node is the part's nearer the body: the arms' pivots are the frame's, the ball joints the arms'
        hinge = next(j for j in r["joints"] if j["parts"][1] == ["lower_wishbone_fl"])
        self.assertEqual({r["nodes"][n]["owner"] for n in hinge["nodes"]}, {"frame"})
        ball = next(j for j in r["joints"] if j["parts"] == [["lower_wishbone_fl"], ["upright_fl"]])
        self.assertEqual(r["nodes"][ball["nodes"][0]]["owner"], "lower_wishbone_fl")

    def test_split_weld_and_straight(self):
        t = sk.build([{"name": "a", "lines": [[[0, 0, 0], [1000, 0, 0]]]}, {"name": "b", "lines": [[[500, 0, 0], [500, 500, 0]]]}])
        a = next(b for b in t["bodies"] if b["parts"] == ["a"])
        self.assertEqual((t["report"]["splits"], a["shape"], a["helpers"], a["rank"], a["need"]), (1, "straight", 2, 9, 9))
        self.assertEqual([j["type"] for j in t["joints"]], ["ball"])
        w = sk.build([{"name": "p", "lines": [[[0, 0, 0], [1000, 0, 0]], [[1000, 0, 0], [0, 1000, 0]]]},
                      {"name": "q", "lines": [[[0, 0, 0], [0, 1000, 0]], [[0, 1000, 0], [0, 0, 1000]], [[1000, 0, 0], [0, 0, 1000]]]}])
        self.assertEqual((w["report"]["bodies"], w["bodies"][0]["parts"], w["bodies"][0]["rank"]), (1, ["p", "q"], 6))

    def test_jbeam(self):
        r = sk.build(sk.read(sk.write(CORNER))["parts"])
        jb = sk.to_jbeam(r, "sk", yf=-1.3, ground=0.0)
        self.assertEqual(sorted(jb), ["sk_frame", "sk_lower_wishbone_fl", "sk_tie_rod_fl", "sk_upper_wishbone_fl", "sk_upright_fl"])
        defined = [row[0] for p in jb.values() for row in p["nodes"][2:]]
        self.assertEqual(sorted(defined), sorted(r["nodes"]))                 # every node defined once
        used = {n for p in jb.values() for row in p["beams"][2:] for n in row[:2]}
        self.assertTrue(used <= set(defined))
        self.assertEqual(sum(len(p["beams"]) - 2 for p in jb.values()), r["report"]["beams"])
        self.assertTrue(all(len(row) == 5 and row[4]["nodeWeight"] >= sk.MIN_KG for p in jb.values() for row in p["nodes"][2:]))
        out = json.loads(sk.build_json(sk.write(CORNER), json.dumps({"yf": -1.3})))
        self.assertEqual(out["unit_name"], "mm")
        self.assertIn("bng", next(iter(out["nodes"].values())))
        self.assertEqual(out["report"]["bands"], {"ok": 10, "high": 0, "risky": 0, "unstable": 0})   # stable by construction
        self.assertTrue(all(b["beamSpring"] > 0 and b["beamDamp"] > 0 for b in out["beams"]))

    def test_stable_values(self):
        """A new beam's values: its own mode at the vanilla median; a node near its limit gets a softer beam."""
        from beamforge import rigidity
        v = rigidity.beam_values(2.0, 2.0)
        self.assertAlmostEqual(v["beamSpring"] * rigidity.DT ** 2 * (1 / 2 + 1 / 2), rigidity.VANILLA["beam_k"]["p50"], places=2)
        full = rigidity.VANILLA["node_k"]["p90"] * 2.0 / rigidity.DT ** 2 - 5e5      # a node 0.5 MN/m short of its cap
        w = rigidity.beam_values(2.0, 2.0, (full, 0.0))
        self.assertTrue(w["limited"])
        self.assertEqual(w["beamSpring"], 5e5)
        self.assertEqual(rigidity.node_index(2.0, 4e6, 400)[2], "ok")
        self.assertEqual(rigidity.node_index(2.0, 4e7, 400)[2], "unstable")


class TestTubes(unittest.TestCase):
    """Real tubes behind the beams (beamforge/tubes.py, ported from FBeam)."""

    def test_section_and_limits(self):
        from beamforge import tubes
        s = tubes.section("tube 45x2.5 steel_1018")
        self.assertAlmostEqual(s["A"] * 1e6, 333.79, places=1)                 # pi/4 (45^2 - 40^2) mm^2
        lim = tubes.limits(s, 1.0)
        self.assertAlmostEqual(lim["k"] / 1e6, 68.4, places=1)                 # E A / L
        self.assertAlmostEqual(lim["mass"], 2.62, places=2)                    # rho A L
        self.assertEqual(lim["governs"], "yield")
        self.assertEqual(tubes.limits(s, 3.0)["governs"], "buckling")          # a long tube buckles first
        with self.assertRaises(tubes.SectionError):
            tubes.section("tube 45x1 steel_1018")                              # under the minimum wall

    def test_bending_beam_budget(self):
        """A welded right-angle corner: k = 1 / (L/3EI + L/3EI) / h^2, F = Mp / h (FBeam 8.10.3)."""
        from beamforge import tubes
        import math
        pos = {"j": [0, 0, 0], "a": [1, 0, 0], "c": [0, 1, 0]}
        s = tubes.section("tube 40x2 steel_1018")
        e = tubes.bending_budgets(pos, [{"id": 0, "a": "j", "b": "a"}, {"id": 1, "a": "j", "b": "c"}], lambda m: s)[("a", "c")]
        h = math.sqrt(0.5)
        self.assertAlmostEqual(e["k"], 1 / (2 / (3 * s["E"] * s["I"])) / h ** 2, delta=1)
        self.assertAlmostEqual(e["F"], s["sy"] * tubes.plastic_modulus(s["D"], s["t"]) / h, delta=1)

    def test_buckling_run(self):
        from beamforge import tubes
        pos = {"a": [0, 0, 0], "b": [1, 0, 0], "c": [2, 0, 0], "d": [2, 1, 0]}
        run = tubes.buckling_lengths(pos, [{"id": 0, "a": "a", "b": "b"}, {"id": 1, "a": "b", "b": "c"}, {"id": 2, "a": "c", "b": "d"}])
        self.assertEqual((run[0], run[1], run[2]), (2.0, 2.0, 1.0))          # a straight split tube buckles as one

    def test_part_kinds_and_tubes(self):
        self.assertEqual((sk.kind_of("frame"), sk.kind_of("lower_wishbone_fl"), sk.kind_of("frame", {"frame": "link"})),
                         ("frame", "link", "link"))
        self.assertEqual(sk.tube_of("subframe_tube_45x2_5", "frame"), "tube 45x2.5 steel_1018")     # "_" for the decimal point
        self.assertEqual(sk.tube_of("rocker_sqtube_30x2_al_6061_t6", "link"), "sqtube 30x2 al_6061_t6")
        self.assertEqual(sk.tube_of("frame", "frame"), sk.TUBE["frame"])
        self.assertEqual(sk.tube_of("frame", "frame", {"frame": "tube 50x3 steel_4130n"}), "tube 50x3 steel_4130n")

    def test_frame_members_and_corners(self):
        """A frame's lines are tubes: stiffness at most E A / L, nodes carrying their mass; its corners get bending beams."""
        r = json.loads(sk.build_json(sk.write(CORNER)))
        frame = next(p for p in r["parts"] if p["name"] == "frame")
        self.assertEqual((frame["kind"], frame["tube"], frame["members"]), ("frame", sk.TUBE["frame"], 6))
        self.assertEqual(r["report"]["bending"], 4)
        for b in r["beams"]:
            if b["kind"] == "line":
                self.assertLessEqual(b["beamSpring"], b["real_k"])
            if b["kind"] == "bend":
                self.assertLess(b["beamSpring"], b["real_k"])                  # scaled like the members
        self.assertGreater(r["report"]["node_mass"], r["report"]["tube_mass"])  # joints and the minimum weight on top


if __name__ == "__main__":
    unittest.main()
