"""SVJ v0.100+ unified suspension read as corners, movable parts and half shafts (beamforge/unified.py).

The SVJ below is written for the tests (invented numbers): a MacPherson front corner and a double-wishbone rear corner as
bodies, joints, springs and dampers, a steering rack, a rear half shaft, a hood and two rear doors on hinges, a wiper.
Its link bodies sit where the SVJ repository's migration puts a generated link (midway between the inboard mean and the
outboard point), so a one-hinge arm's second bush can be recovered.
"""

import base64
import json
import math
import os
import struct
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from beamforge import components, dampers, export, suspension, svj, unified  # noqa: E402


def quat(axis):
    """Unit quaternion [x, y, z, w] turning +Z onto axis."""
    n = math.sqrt(sum(c * c for c in axis))
    a = [c / n for c in axis]
    w = 1.0 + a[2]
    if w < 1e-9:
        return [1.0, 0.0, 0.0, 0.0]
    q = [-a[1], a[0], 0.0, w]
    m = math.sqrt(sum(c * c for c in q))
    return [c / m for c in q]


def joint(jid, jt, a, b, pos, axis=None, **extra):
    f = {"position": list(pos), "orientation": quat(axis) if axis else [0.0, 0.0, 0.0, 1.0]}
    return dict({"id": jid, "type": "joint", "joint_type": jt, "body_a": a, "frame_a": dict(f), "body_b": b, "frame_b": dict(f)}, **extra)


def mid(*ps):
    return [sum(p[i] for p in ps) / len(ps) for i in range(3)]


def sub(a, b):
    return [x - y for x, y in zip(a, b)]


# front left, MacPherson (SAE: X forward, Y right, Z down)
WC_F, LBJ, TRE = [0.0, -0.75, -0.30], [0.01, -0.70, -0.15], [-0.14, -0.68, -0.30]
ST_LO, ST_TOP = [0.0, -0.68, -0.40], [-0.03, -0.64, -0.68]
LCA0, LCA1 = [0.05, -0.33, -0.15], [-0.30, -0.36, -0.15]
RACK, TRI = [-0.2, 0.0, -0.30], [-0.2, -0.36, -0.30]
# rear left, double wishbone
WC_R, UBJ, RLBJ, TOE_O, CV_O = [-2.6, -0.75, -0.30], [-2.6, -0.68, -0.52], [-2.6, -0.72, -0.14], [-2.75, -0.70, -0.18], [-2.6, -0.68, -0.30]
UW0, UW1, LW0, LW1, TOE_I, CV_I = ([-2.45, -0.40, -0.50], [-2.80, -0.40, -0.50], [-2.40, -0.35, -0.14], [-2.85, -0.37, -0.14],
                                   [-2.75, -0.35, -0.18], [-2.6, -0.15, -0.30])
SPR_TOP, SPR_LOW = [-2.6, -0.55, -0.62], [-2.6, -0.55, -0.14]
MR_F = 0.9

DOC = {
    "_metadata": {"specification": "SVJ", "version": "0.102", "coordinate_system": "SAE_J670", "units": "SI"},
    "vehicle_info": {"make": "Toy", "model": "Sedan"},
    "assets": {"meshes": [{"id": "car", "uri": "car.gltf"}]},
    "chassis": {
        "wheelbase": 2.6, "mass_total": 1300, "visual": {"mesh_ref": "car", "node": "SVJ::body::chassis"},
        "markers": {"hood_hinge": {"position": [-0.3, 0.0, -0.95], "orientation": quat([0, 1, 0])},
                    "door_rl_hinge": {"position": [-1.40, -0.78, -0.60], "orientation": quat([0, 0, 1])}},
        "mass_bodies": [
            {"id": "upright_fl", "mass": 0.0, "position": WC_F, "x_derived": True,
             "markers": {"wheel_center": {"position": WC_F}, "lower_ball_joint": {"position": LBJ},
                         "steering_tie_rod_end": {"position": TRE}, "strut_lower": {"position": ST_LO},
                         "damper_outboard": {"position": ST_LO}, "strut_top": {"position": ST_TOP}}},
            {"id": "wheel_fl", "mass": 20.0, "position": WC_F, "inertia": {"Ixx": 0.6, "Iyy": 1.1, "Izz": 0.6},
             "visual": {"mesh_ref": "car", "node": "SVJ::wheel::wheel_fl"}},
            {"id": "lower_control_arm_fl", "mass": 0.0, "position": mid(mid(LCA0, LCA1), LBJ), "x_derived": True},
            {"id": "rack", "mass": 0.0, "position": RACK, "x_derived": True},
            {"id": "steering_tie_rod_fl", "mass": 0.0, "position": mid(TRI, TRE), "x_derived": True},
            {"id": "strut_fl", "mass": 0.0, "position": mid(ST_TOP, ST_LO), "x_derived": True},
            {"id": "upright_rl", "mass": 4.0, "position": WC_R,
             "markers": {"wheel_center": {"position": WC_R}, "upper_ball_joint": {"position": UBJ},
                         "lower_ball_joint": {"position": RLBJ}, "toe_link_outer": {"position": TOE_O},
                         "cv_joint_outer": {"position": CV_O}}},
            {"id": "wheel_rl", "mass": 20.0, "position": WC_R, "visual": {"mesh_ref": "car", "node": "SVJ::wheel::wheel_rl"}},
            {"id": "upper_wishbone_rl", "mass": 1.5, "position": mid(mid(UW0, UW1), UBJ),
             "visual": {"mesh_ref": "car", "node": "SVJ::suspension::upper_wishbone_rl", "placement": "link_between_points",
                        "mesh_axis": "+y"}},
            {"id": "lower_wishbone_rl", "mass": 2.0, "position": mid(mid(LW0, LW1), RLBJ)},
            {"id": "toe_link_rl", "mass": 0.5, "position": mid(TOE_I, TOE_O)},
            {"id": "half_shaft_rl", "mass": 5.0, "position": mid(CV_I, CV_O),
             "visual": {"mesh_ref": "car", "node": "SVJ::powertrain::half_shaft_rl", "placement": "link_between_points",
                        "mesh_axis": "+y", "scale_to_length": True}},
            {"id": "hood", "category": "attachment", "mass": 12.0, "position": [-0.6, 0.0, -0.95],
             "markers": {"hinge": {"position": [-0.3, 0.0, -0.95], "orientation": quat([0, 1, 0])}},
             "visual": {"mesh_ref": "car", "node": "SVJ::body::hood"}},
            {"id": "door_rl", "category": "attachment", "mass": 18.0, "position": [-1.8, -0.78, -0.60],
             "visual": {"mesh_ref": "car", "node": "SVJ::body::door_rl"}},
            {"id": "door_rr", "category": "attachment", "mass": 18.0, "position": [-1.8, 0.78, -0.60],
             "visual": {"mesh_ref": "car", "node": "REAR_DOOR_RIGHT_MESH"}},
            {"id": "wiper_l", "category": "attachment", "mass": 0.4, "position": [-0.9, -0.3, -1.0],
             "visual": {"mesh_ref": "car", "node": "SVJ::body::wiper_l"}},
            {"id": "fuel_tank", "category": "fluid", "mass": 40.0, "position": [-2.2, 0.0, -0.35]},
        ]},
    "constraints": [
        joint("hinge_wheel_fl", "revolute", "upright_fl", "wheel_fl", WC_F, [0, 1, 0]),
        joint("hinge_lower_control_arm_fl_inboard", "revolute", "chassis", "lower_control_arm_fl", LCA0, sub(LCA1, LCA0)),
        joint("ball_lower_control_arm_fl_outboard", "spherical", "lower_control_arm_fl", "upright_fl", LBJ),
        joint("slide_rack", "translational", "chassis", "rack", RACK, [0, 1, 0]),
        joint("ball_steering_tie_rod_fl_inboard", "spherical", "rack", "steering_tie_rod_fl", TRI),
        joint("ball_steering_tie_rod_fl_outboard", "spherical", "steering_tie_rod_fl", "upright_fl", TRE),
        joint("fixed_strut_fl_inboard", "fixed", "chassis", "strut_fl", ST_TOP),
        joint("slide_strut_fl_outboard", "translational", "strut_fl", "upright_fl", ST_LO, sub(ST_LO, ST_TOP)),
        {"id": "spring_fl", "type": "spring", "spring_type": "coilover", "body_a": "chassis",
         "frame_a": {"position": ST_TOP}, "body_b": "strut_fl", "frame_b": {"position": ST_LO},
         "rate": 30000.0, "x_converted_from_wheel_rate": {"motion_ratio": MR_F, "estimated": False}},
        {"id": "damper_fl", "type": "damper", "body_a": "chassis", "frame_a": {"position": ST_TOP}, "body_b": "strut_fl",
         "frame_b": {"position": ST_LO}, "bump_curve": [[0.0, 0.0], [0.09, 900.0]], "rebound_curve": [[0.0, 0.0], [0.09, 1800.0]],
         "x_converted_from_wheel_rate": {"motion_ratio": MR_F, "estimated": False}},
        joint("hinge_wheel_rl", "revolute", "upright_rl", "wheel_rl", WC_R, [0, 1, 0]),
        joint("ball_upper_wishbone_rl_front", "spherical", "chassis", "upper_wishbone_rl", UW0),
        joint("ball_upper_wishbone_rl_rear", "spherical", "chassis", "upper_wishbone_rl", UW1),
        joint("ball_upper_wishbone_rl_outboard", "spherical", "upper_wishbone_rl", "upright_rl", UBJ),
        joint("hinge_lower_wishbone_rl_inboard", "revolute", "chassis", "lower_wishbone_rl", LW0, sub(LW1, LW0),
              bushing={"radial_stiffness": 2.0e6}),
        joint("ball_lower_wishbone_rl_outboard", "spherical", "lower_wishbone_rl", "upright_rl", RLBJ),
        joint("ball_toe_link_rl_inboard", "spherical", "chassis", "toe_link_rl", TOE_I),
        joint("ball_toe_link_rl_outboard", "spherical", "toe_link_rl", "upright_rl", TOE_O),
        joint("universal_half_shaft_rl_inboard", "universal", "chassis", "half_shaft_rl", CV_I, [0, 1, 0]),
        joint("universal_half_shaft_rl_outboard", "universal", "half_shaft_rl", "upright_rl", CV_O, [0, 1, 0]),
        {"id": "spring_rl", "type": "spring", "spring_type": "coil", "body_a": "chassis", "frame_a": {"position": SPR_TOP},
         "body_b": "lower_wishbone_rl", "frame_b": {"position": SPR_LOW}, "rate": 50000.0},
        {"id": "damper_rl", "type": "damper", "body_a": "chassis", "frame_a": {"position": [-2.62, -0.55, -0.62]},
         "body_b": "lower_wishbone_rl", "frame_b": {"position": [-2.62, -0.55, -0.14]}, "damping": 2000.0},
        {"id": "hinge_hood", "type": "joint", "joint_type": "revolute", "body_a": "chassis", "marker_a": "hood_hinge",
         "body_b": "hood", "marker_b": "hinge", "range": {"min": 0.0, "max": 1.0}},
        {"id": "hinge_door_rl", "type": "joint", "joint_type": "revolute", "body_a": "chassis", "marker_a": "door_rl_hinge",
         "body_b": "door_rl", "frame_b": {"position": [-1.40, -0.78, -0.60], "orientation": quat([0, 0, 1])},
         "range": {"min": 0.0, "max": 1.1}},
        joint("hinge_door_rr", "revolute", "chassis", "door_rr", [-1.40, 0.78, -0.60], [0, 0, -1], range={"min": 0.0, "max": 1.1}),
        joint("hinge_wiper_l", "revolute", "chassis", "wiper_l", [-0.9, -0.3, -1.0], [0.3, 0, -1], range={"min": 0.0, "max": 1.5}),
    ],
    "poses": [{"id": "closed"}, {"id": "rear_doors_open", "values": {"hinge_door_rl": 0.8, "hinge_door_rr": 0.8}},
              {"id": "hood_up", "values": {"hinge_hood": 0.9}}],
    "wheel_stations": [
        {"id": "FL", "wheel_body": "wheel_fl", "upright_body": "upright_fl", "wheel_center_marker": "wheel_center",
         "steering_joint": "slide_rack", "axle": "A1", "side": "L", "topology_hint": "macpherson",
         "wheel": {"rim_diameter": 0.4318}, "tire": {"set_ref": "front"}, "alignment": {"camber": -1.0},
         "visual": {"mesh_ref": "car", "node": "SVJ::suspension::upright_fl"}},
        {"id": "RL", "wheel_body": "wheel_rl", "upright_body": "upright_rl", "axle": "A2", "side": "L",
         "topology_hint": "double_wishbone", "x_lateral_location": "none"},
    ],
    "anti_roll_bars": [{"id": "arb_front", "stations": ["FL", "FR"], "bar_rate": 15000.0, "drop_link_mounts": {"FL": LBJ}}],
}


def view():
    d = json.loads(json.dumps(DOC))
    unified.adopt(d)
    return d


def links(corner):
    return {x["name"]: x for x in corner["topology"]["links"]}


class TestCornerView(unittest.TestCase):
    def test_corners_hardpoints_and_links(self):
        d = view()
        self.assertEqual(sorted(d["suspension"]), ["FL", "RL"])
        fl = d["suspension"]["FL"]
        self.assertEqual(fl["topology"]["system_type"], "macpherson")
        hp = fl["topology"]["upright"]["hardpoints"]
        self.assertEqual(hp["wheel_center"], WC_F)
        self.assertEqual(hp["strut_top"], ST_TOP)
        ln = links(fl)
        self.assertEqual(sorted(ln), ["lower_control_arm", "steering_tie_rod", "strut"])
        arm = ln["lower_control_arm"]
        self.assertEqual(arm["type"], "arm")
        self.assertEqual(arm["outboard_ref"], "hardpoints.lower_ball_joint")
        for got, want in zip(arm["inboard_points"], (LCA0, LCA1)):        # the second bush, recovered from the body's place
            self.assertAlmostEqual(math.dist(got, want), 0.0, places=6)
        self.assertEqual(ln["strut"]["type"], "strut")
        self.assertEqual(ln["strut"]["inboard_points"], [ST_TOP])
        self.assertEqual(ln["steering_tie_rod"]["inboard_points"], [TRI])  # its inboard end is on the rack
        self.assertEqual(ln["steering_tie_rod"]["type"], "rod")
        self.assertEqual((fl["alignment"], fl["tire"], fl["x_steering_joint"]), ({"camber": -1.0}, {"set_ref": "front"}, "slide_rack"))
        self.assertEqual((fl["wheel"]["mass"], fl["wheel"]["rotational_inertia"]), (20.0, 1.1))
        self.assertEqual(fl["arb"], {"bar_id": "arb_front", "bar_rate": 15000.0, "drop_link_mount": LBJ})

        rl = d["suspension"]["RL"]
        ln = links(rl)
        self.assertEqual(sorted(ln), ["lower_wishbone", "toe_link", "upper_wishbone"])   # the half shaft is no link
        self.assertEqual(ln["upper_wishbone"]["inboard_points"], [UW0, UW1])
        self.assertEqual(ln["lower_wishbone"]["bushings"], [{"radial_stiffness": 2.0e6}])
        self.assertEqual(rl["topology"]["upright"]["mass"], 4.0)
        self.assertEqual(rl["topology"]["lateral_location"], "none")
        self.assertTrue(d["x_beamforge_corner_view"])

    def test_springs_and_dampers_back_at_the_wheel(self):
        d = view()
        sp, dm = d["suspension"]["FL"]["spring"], d["suspension"]["FL"]["damper"]
        self.assertAlmostEqual(sp["rate"], 30000.0 * MR_F ** 2)
        self.assertEqual((sp["motion_ratio"], sp["type"], sp["inboard_mount"], sp["outboard_mount"]), (MR_F, "coilover", ST_TOP, ST_LO))
        self.assertEqual(dm["reference"], "wheel")
        self.assertEqual(dm["bump_curve"], [[0.0, 0.0], [0.1, 810.0]])                   # (v / MR, F * MR)
        w = dampers.wheel_values(d)["front"]
        self.assertAlmostEqual(w["wheel_rate"], 24300.0)
        rsp, rdm = d["suspension"]["RL"]["spring"], d["suspension"]["RL"]["damper"]
        self.assertEqual((rsp["rate"], rsp["x_rate_reference"]), (50000.0, "element"))  # no motion ratio given: as it is
        self.assertEqual(rdm["reference"], "damper")
        self.assertTrue(rdm["x_motion_ratio_estimated"])
        self.assertAlmostEqual(rdm["motion_ratio"], 0.49, delta=0.01)   # on the lower wishbone, about 0.2 m of its 0.4 m out

    def test_study_and_hardpoints(self):
        d = view()
        self.assertTrue(svj.hardpoints(d))
        study = suspension.study_svj(d, 50)
        self.assertEqual(sorted(study["corners"]), ["front", "rear"])
        for c in study["corners"].values():
            self.assertFalse([n for n in c["notes"] if "constraints where a corner needs 5" in n], c["notes"])
        self.assertEqual(svj.summary(d)["corners"], {"FL": "macpherson", "RL": "double_wishbone"})

    def test_corner_file_left_alone(self):
        d = {"suspension": {"FL": {"topology": {}}}, "wheel_stations": [{"id": "FL"}]}
        self.assertEqual(unified.adopt(d), [])
        self.assertEqual(d["suspension"], {"FL": {"topology": {}}})

    def test_second_point_along_the_axis_when_not_recoverable(self):
        d = json.loads(json.dumps(DOC))
        body = next(b for b in d["chassis"]["mass_bodies"] if b["id"] == "lower_control_arm_fl")
        body["position"] = [0.0, -0.5, -0.15]                                             # an authored CG: no midpoint
        notes = unified.adopt(d)
        p0, p1 = links(d["suspension"]["FL"])["lower_control_arm"]["inboard_points"]
        u = [x / math.dist(LCA0, LCA1) for x in sub(LCA1, LCA0)]
        self.assertAlmostEqual(math.dist(p1, [LCA0[i] + unified.SECOND_POINT * u[i] for i in range(3)]), 0.0, places=5)
        self.assertTrue(any("second point" in n for n in notes))

    def test_strut_with_a_spherical_top_mount(self):         # the spec's strut top pivots (v0.102 rule: spherical, not fixed)
        d = json.loads(json.dumps(DOC))
        top = next(j for j in d["constraints"] if j["id"] == "fixed_strut_fl_inboard")
        top["joint_type"] = "spherical"
        notes = unified.adopt(d)
        self.assertFalse(any("strut" in n and ("left out" in n or "not read" in n) for n in notes))   # the strut is read as before
        strut = links(d["suspension"]["FL"])["strut"]
        self.assertEqual((strut["type"], strut["inboard_points"], strut["inboard_joint_type"]), ("strut", [ST_TOP], "spherical"))

    def test_two_spherical_pickups_need_no_guessed_point(self):
        notes = unified.adopt(json.loads(json.dumps(DOC)))
        self.assertFalse(any("second point" in n for n in notes if n.startswith("RL")))    # the rear wishbone has both pickups as given

    def test_shared_axle_body(self):
        """A beam axle carries both stations: each takes its own markers and links."""
        wl, wr = [-2.6, -0.75, -0.30], [-2.6, 0.75, -0.30]
        d = {"chassis": {"mass_bodies": [
            {"id": "axle_rear", "mass": 60.0, "position": [-2.6, 0, -0.3],
             "markers": {"wheel_center_rl": {"position": wl}, "wheel_center_rr": {"position": wr},
                         "link_end_rl": {"position": [-2.6, -0.5, -0.15]}, "link_end_rr": {"position": [-2.6, 0.5, -0.15]},
                         "panhard_axle_end": {"position": [-2.6, -0.6, -0.35]}}},
            {"id": "wheel_rl", "mass": 20, "position": wl}, {"id": "wheel_rr", "mass": 20, "position": wr},
            {"id": "trailing_link_rl", "mass": 1, "position": [-2.3, -0.5, -0.15]},
            {"id": "trailing_link_rr", "mass": 1, "position": [-2.3, 0.5, -0.15]},
            {"id": "panhard_rod", "mass": 2, "position": [-2.6, 0.0, -0.35]}]},
            "constraints": [joint("hinge_wheel_rl", "revolute", "axle_rear", "wheel_rl", wl, [0, 1, 0]),
                            joint("hinge_wheel_rr", "revolute", "axle_rear", "wheel_rr", wr, [0, 1, 0]),
                            joint("a", "spherical", "chassis", "trailing_link_rl", [-2.0, -0.5, -0.15]),
                            joint("b", "spherical", "trailing_link_rl", "axle_rear", [-2.6, -0.5, -0.15]),
                            joint("c", "spherical", "chassis", "trailing_link_rr", [-2.0, 0.5, -0.15]),
                            joint("e", "spherical", "trailing_link_rr", "axle_rear", [-2.6, 0.5, -0.15]),
                            joint("f", "spherical", "chassis", "panhard_rod", [-2.6, 0.6, -0.35]),
                            joint("g", "spherical", "panhard_rod", "axle_rear", [-2.6, -0.6, -0.35])],
            "wheel_stations": [{"id": "RL", "wheel_body": "wheel_rl", "upright_body": "axle_rear", "wheel_center_marker": "wheel_center_rl"},
                               {"id": "RR", "wheel_body": "wheel_rr", "upright_body": "axle_rear", "wheel_center_marker": "wheel_center_rr"}]}
        unified.adopt(d)
        rl, rr = d["suspension"]["RL"], d["suspension"]["RR"]
        self.assertEqual(rl["topology"]["system_type"], "solid_axle")
        self.assertEqual(rl["topology"]["axle_body"]["id"], "axle_rear")
        self.assertEqual(rl["topology"]["upright"]["hardpoints"]["wheel_center"], wl)
        self.assertEqual(rr["topology"]["upright"]["hardpoints"]["link_end"], [-2.6, 0.5, -0.15])
        self.assertEqual(sorted(links(rl)), ["panhard_rod", "trailing_link"])     # the rod's axle end is on the left
        self.assertEqual(sorted(links(rr)), ["trailing_link"])
        self.assertEqual(links(rl)["trailing_link"]["outboard_ref"], "hardpoints.link_end")


class TestMovableAndShafts(unittest.TestCase):
    def test_movable_parts(self):
        parts = {p["id"]: p for p in unified.movable_parts(DOC)}
        self.assertEqual(sorted(parts), ["door_rl", "door_rr", "hood", "wiper_l"])     # not the suspension, not the tank
        door = parts["door_rl"]
        self.assertEqual((door["kind"], door["joint"], door["hinge"], door["range"]), ("door", "hinge_door_rl", [-1.40, -0.78, -0.60], [0.0, 1.1]))
        self.assertAlmostEqual(door["axis"][2], 1.0)
        self.assertEqual(door["poses"], ["rear_doors_open"])
        self.assertEqual((parts["hood"]["kind"], parts["hood"]["hinge"], parts["wiper_l"]["kind"]), ("hood", [-0.3, 0.0, -0.95], "wiper"))
        self.assertAlmostEqual(parts["hood"]["axis"][1], 1.0)

    def test_half_shaft_link_between_points(self):
        self.assertEqual(unified.link_ends(DOC, "half_shaft_rl"), ([CV_I], CV_O))
        b = next(x for x in svj.part_bindings(DOC) if x["role"] == "shaft")
        self.assertEqual((b["corner"], b["part"], b["placement"], b["inboard"], b["outboard"], b["scale_to_length"]),
                         ("RL", "half_shaft", "link_between_points", [CV_I], CV_O, True))
        old = {"powertrain": {"half_shafts": {"RR": {"cv_inner": {"position": [-2.6, 0.15, -0.3]},
                                                     "cv_outer": {"position": [-2.6, 0.68, -0.3]},
                                                     "visual": {"node": "SVJ::powertrain::half_shaft_rr"}}}}}
        self.assertEqual([(h["corner"], h["inboard"], h["outboard"]) for h in unified.half_shafts(old)],
                         [("RR", [[-2.6, 0.15, -0.3]], [-2.6, 0.68, -0.3])])

    def test_bindings_and_components(self):
        d = view()
        paths = {b["path"]: b["node"] for b in svj.visual_bindings(d)}
        self.assertEqual(paths["suspension.FL"], "SVJ::suspension::upright_fl")
        self.assertIn("mass_bodies.hood", paths)
        self.assertNotIn("mass_bodies.wheel_fl", paths)                  # the wheels are the base's
        self.assertNotIn("mass_bodies.upper_wishbone_rl", paths)          # a link goes through its corner (part_bindings)
        parts = {b["part"]: b for b in svj.part_bindings(d)}
        w = parts["upper_wishbone"]
        self.assertEqual((w["placement"], w["inboard"], w["outboard"]), ("link_between_points", [UW0, UW1], UBJ))
        self.assertEqual(sorted(c["id"] for c in components.svj_components(d)), ["door_rl", "door_rr", "fuel_tank", "hood", "wiper_l"])


def gltf_file(nodes):
    """A .gltf with one triangle per node: {name: SAE point}."""
    raw, names = b"", list(nodes)
    for n in names:
        p = nodes[n]
        raw += b"".join(struct.pack("<3f", *svj.sae_to_gltf([p[0] + dx, p[1] + dy, p[2] + dz]))
                        for dx, dy, dz in ((0, 0, 0), (0.5, 0, 0), (0, 0, -0.5)))
    return {"asset": {"version": "2.0"}, "scene": 0, "scenes": [{"nodes": list(range(len(names)))}],
            "nodes": [{"name": n, "mesh": i} for i, n in enumerate(names)],
            "meshes": [{"primitives": [{"attributes": {"POSITION": i}}]} for i in range(len(names))],
            "buffers": [{"byteLength": len(raw), "uri": "data:application/octet-stream;base64," + base64.b64encode(raw).decode()}],
            "bufferViews": [{"buffer": 0, "byteOffset": 36 * i, "byteLength": 36} for i in range(len(names))],
            "accessors": [{"bufferView": i, "componentType": 5126, "count": 3, "type": "VEC3"} for i in range(len(names))]}


class TestExport(unittest.TestCase):
    PLACE = {"yf": 0.0, "ground": 0.0}

    def rows(self):
        mesh = gltf_file({"SVJ::body::chassis": [-1.0, 0.0, -0.8], "SVJ::body::door_rl": [-1.8, -0.8, -0.6],
                          "REAR_DOOR_RIGHT_MESH": [-1.8, 0.8, -0.6], "SVJ::powertrain::half_shaft_rl": [-2.6, -0.4, -0.3]})
        d = view()
        # the base, in BeamNG axes (x = -Y, y = yf - X, z = ground - Z): rear doors and rear half shafts on each side
        v = {"geometry": {"nodes": {"dl": [0.8, 1.8, 0.6], "dr": [-0.8, 1.8, 0.6], "c": [0.0, 1.0, 0.5],
                                    "h1": [0.15, 2.6, 0.3], "h2": [0.68, 2.6, 0.3], "h3": [-0.15, 2.6, 0.3], "h4": [-0.68, 2.6, 0.3]},
                          "parts": {"dl": "car_door_RL", "dr": "car_door_RR", "c": "car_body",
                                    "h1": "car_halfshaft_R", "h2": "car_halfshaft_R", "h3": "car_halfshaft_R", "h4": "car_halfshaft_R"},
                          "groups": {"dl": ["car_door_RL"], "dr": ["car_door_RR"], "c": ["car_body"],
                                     "h1": ["hs_RL"], "h2": ["hs_RL"], "h3": ["hs_RR"], "h4": ["hs_RR"]}},
             "flexbodies": [{"part": "car_door_RL", "mesh": "door_RL", "groups": ["car_door_RL"]},
                            {"part": "car_door_RR", "mesh": "door_RR", "groups": ["car_door_RR"]},
                            {"part": "car_halfshaft_R", "mesh": "halfshaft_RL", "groups": ["hs_RL"]},
                            {"part": "car_halfshaft_R", "mesh": "halfshaft_RR", "groups": ["hs_RR"]}]}
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "car.gltf")
            Path(path).write_text(json.dumps(mesh))
            return json.loads(export.svj_attach(json.dumps(v), json.dumps(d), "[]", json.dumps({"car": path}), json.dumps(self.PLACE)))

    def test_rear_doors_with_their_hinges(self):
        rows = {r["node"]: r for r in self.rows()}
        left, right = rows["SVJ::body::door_rl"], rows["REAR_DOOR_RIGHT_MESH"]      # (the right one named as no convention has it)
        self.assertEqual((left["part"], right["part"]), ("car_door_RL", "car_door_RR"))
        self.assertEqual(left["hinge"]["joint"], "hinge_door_rl")
        self.assertEqual(left["hinge"]["position"], [0.78, 1.4, 0.6])                # BeamNG axes
        self.assertAlmostEqual(left["hinge"]["axis"][2], -1.0)
        self.assertEqual(left["hinge"]["range"], [0.0, 1.1])
        self.assertEqual(len([r for r in self.rows() if r["node"] == "SVJ::body::door_rl"]), 1)   # once (not also as a mass body)
        body = rows["SVJ::body::chassis"]
        self.assertTrue({"SVJ::body::door_rl", "REAR_DOOR_RIGHT_MESH", "SVJ::powertrain::half_shaft_rl"} <= set(body["exclude"]))

    def test_half_shaft_on_the_base_half_shaft(self):
        row = next(r for r in self.rows() if r["node"] == "SVJ::powertrain::half_shaft_rl")
        self.assertEqual((row["part"], row["groups"], row["replaces"]), ("car_halfshaft_R", ["hs_RL"], ["halfshaft_RL"]))
        self.assertEqual((row["link"]["inboard"], row["link"]["outboard"], row["link"]["scale"]), ([CV_I], CV_O, True))

    def test_replaced_base_mesh_dropped(self):
        part = {"flexbodies": [["mesh", "[group]:"], ["halfshaft_RL", ["hs_RL"]], ["halfshaft_RR", ["hs_RR"]]]}
        self.assertEqual(export._drop_replaced(part, {"halfshaft_RL"}), 1)
        self.assertEqual([r[0] for r in part["flexbodies"][1:]], ["halfshaft_RR"])


class TestBundle(unittest.TestCase):
    def test_load_bundle_reads_a_unified_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            f = Path(tmp) / "car.svj.json"
            f.write_text(json.dumps(DOC))
            out = json.loads(svj.load_bundle(str(f), str(Path(tmp) / "work")))
        self.assertEqual(sorted(out["svj"]["suspension"]), ["FL", "RL"])
        self.assertTrue(any("unified suspension" in n for n in out["notes"]))
        self.assertEqual(out["summary"]["corners"]["FL"], "macpherson")


if __name__ == "__main__":
    unittest.main()
