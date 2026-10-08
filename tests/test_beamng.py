"""BeamNG vehicle reading (beamforge/beamng.py) and jbeam expressions, on made-up files only.

No game files are used or shipped: the vehicles below are written for the tests and follow
the formats of the game's own (slots and slots2 tables, .pc configs, tuning variables).
An optional check against a local install runs when BEAMNG_VEHICLES points at its
content/vehicles folder.
"""

import json
import os
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from beamforge import beamng as bng  # noqa: E402
from beamforge import jbeam  # noqa: E402

CAR = {
    "vehicles/toycar/toycar.jbeam": json.dumps({
        "toycar": {"slotType": "main", "information": {"name": "Toy car"},
                   "slots2": [["name", "allowTypes", "denyTypes", "default", "description"],
                              ["toycar_engine", ["toycar_engine"], [], "toycar_engine_small", "Engine", {"coreSlot": True}],
                              ["toycar_hub_F", ["toycar_hub_F"], [], "toycar_hub_F", "Front hubs"],
                              ["toycar_spoiler", ["toycar_spoiler"], [], "", "Spoiler"]],
                   "nodes": [["id", "posX", "posY", "posZ"], ["b1", 0.5, -1.0, 0.3], ["b2", -0.5, -1.0, 0.3]],
                   "beams": [["id1:", "id2:"], ["b1", "b2"]]},
    }),
    "vehicles/toycar/toycar_engines.jbeam": json.dumps({
        "toycar_engine_small": {"slotType": "toycar_engine", "information": {"name": "Small engine"},
                                "variables": [["name", "type", "unit", "category", "default", "min", "max", "title", "description"],
                                              ["$revLimiter", "range", "rpm", "Engine", 6000, 3000, 7000, "Rev limiter", "", {"stepDis": 100}]]},
        "toycar_engine_big": {"slotType": "toycar_engine", "information": {"name": "Big engine"}},
    }),
    "vehicles/toycar/toycar_hub.jbeam": json.dumps({
        "toycar_hub_F": {"slotType": "toycar_hub_F",
                         "slots": [["type", "default", "description"],
                                   ["wheel_F_toy", "steel_wheel_F", "Front wheels",
                                    {"nodeOffset": {"x": "$=case($trackwidth == nil, $trackoffset + 0.25, $trackwidth)", "y": -1.2, "z": 0.3}}]],
                         "variables": [["name", "type", "unit", "category", "default", "min", "max", "title", "description"],
                                       ["$camber", "range", "", "Wheel Alignment", 1.0, 0.95, 1.05, "Camber", "", {"subCategory": "Front"}],
                                       ["$trackoffset", "range", "m", "Wheels", 0.0, -0.05, 0.05, "Track offset", "", {"subCategory": "Front"}]]},
        "toycar_spoiler_wing": {"slotType": "toycar_spoiler"},
    }),
    "vehicles/toycar/base.pc": json.dumps({"format": 2, "model": "toycar", "mainPartName": "toycar",
                                           "parts": {"toycar_engine": "toycar_engine_small"}, "vars": {"$revLimiter": 6500}}),
    "vehicles/toycar/sport.pc": json.dumps({"format": 2, "model": "toycar", "parts": {"toycar_engine": "toycar_engine_big",
                                                                                    "toycar_spoiler": "toycar_spoiler_wing"}}),
    "vehicles/toycar/info.json": json.dumps({"Name": "vehiclesData.toycar.Name", "Brand": "Toyco", "Type": "Car", "default_pc": "base"}),
    "vehicles/toycar/info_sport.json": json.dumps({"Configuration": "Sport", "Power": 150, "Drivetrain": "RWD"}),
    "vehicles/common/wheels.jbeam": json.dumps({
        "steel_wheel_F": {"slotType": "wheel_F_toy", "information": {"name": {"txt": "ui.vehicleconfig.Steel Wheels"}},
                          "nodes": [["id", "posX", "posY", "posZ"], ["wil", 0.31, 0, 0], ["wir", -0.31, 0, 0]]},
        "alloy_wheel_F": {"slotType": "wheel_F_toy", "information": {"name": "Alloy wheels"}},
    }),
}


def find(node, slot):
    if node["slot"] == slot:
        return node
    for c in node["children"]:
        hit = find(c, slot)
        if hit:
            return hit
    return None


class TestExpressions(unittest.TestCase):
    def test_lua_forms(self):
        self.assertAlmostEqual(jbeam.resolve("$=case($w == nil, 0.25, $w)", {}), 0.25)
        self.assertAlmostEqual(jbeam.resolve("$=case($w == nil, 0.25, $w)", {"$w": 0.7}), 0.7)
        self.assertAlmostEqual(jbeam.resolve("$=$o == nil and -0.5 or $o-0.5", {"$o": 1}), 0.5)
        self.assertAlmostEqual(jbeam.resolve("$=2^3 + sin(0)", {}), 8.0)
        self.assertAlmostEqual(jbeam.resolve("$=$a ~= nil and 1 or 2", {"$a": 3}), 1)

    def test_hostile_expressions_are_refused(self):
        for bad in ("$=9^9^9^9", "$=__import__('os')", "$=(1).__class__", "$=open('x')", "$=" + "1+" * 300 + "1"):
            with self.assertRaises(jbeam.UnresolvedValue, msg=bad):
                jbeam.resolve(bad, {})


class TestVehicle(unittest.TestCase):
    def setUp(self):
        bng._DOCS.clear()
        bng._TEXT.clear()
        bng.add_files(json.dumps(CAR))

    def cfg(self, config=None, parts=None, vars_=None):
        return json.loads(bng.configure("toycar", config, json.dumps(parts or {}), json.dumps(vars_ or {})))

    def test_default_config_tree_and_options(self):
        v = self.cfg()
        self.assertEqual((v["main"], v["config"], v["configs"]), ("toycar", "base", ["base", "sport"]))
        eng = find(v["tree"], "toycar_engine")
        self.assertEqual(eng["part"], "toycar_engine_small")
        self.assertTrue(eng["core"])                                     # coreSlot: cannot be left empty
        self.assertEqual([o[0] for o in eng["options"]], ["toycar_engine_big", "toycar_engine_small"])
        wheel = find(v["tree"], "wheel_F_toy")                            # an old "slots" table, part from common
        self.assertEqual(wheel["part"], "steel_wheel_F")
        self.assertEqual(wheel["title"], "Steel Wheels")                # localisation object -> its last segment
        self.assertEqual(find(v["tree"], "toycar_spoiler")["part"], "")  # empty by default
        self.assertEqual(v["missing"], [])

    def test_slot_offsets_with_expressions(self):
        g = self.cfg()["geometry"]["nodes"]
        self.assertEqual(g["wil"], [0.56, -1.2, 0.3])                  # 0.31 + 0.25 ($trackwidth is nil), mirrored
        self.assertEqual(g["wir"], [-0.56, -1.2, 0.3])
        g2 = self.cfg(vars_={"$trackoffset": 0.05})["geometry"]["nodes"]   # a tuning slider moves the wheels out
        self.assertEqual(g2["wil"][0], 0.61)
        g3 = self.cfg(vars_={"$trackwidth": 0.4})["geometry"]["nodes"]     # not declared by any part: ignored, as in game
        self.assertEqual(g3["wil"][0], 0.56)

    def test_moved_part_and_node(self):
        moves = {"parts": {"steel_wheel_F": [0.0, 0.1, 0.0]}, "nodes": {"wil": [0.02, 0.0, -0.05], "b1": "bad"}}
        g = json.loads(bng.configure("toycar", None, None, None, json.dumps(moves)))["geometry"]["nodes"]
        self.assertEqual(g["wil"], [0.58, -1.1, 0.25])                 # part move + node move
        self.assertEqual(g["wir"], [-0.56, -1.1, 0.3])                 # part move only
        self.assertEqual(g["b1"], [0.5, -1.0, 0.3])                     # a bad move is ignored
        g2 = json.loads(bng.configure("toycar", None, None, None, json.dumps({"parts": {"toycar_hub_F": [0.0, 0.0, 0.1]}})))["geometry"]
        self.assertEqual(g2["nodes"]["wil"], [0.56, -1.2, 0.4])          # the wheel in the hub's slot moves with it
        self.assertEqual(g2["nodes"]["wir"], [-0.56, -1.2, 0.4])         # a plain shift: x not mirrored
        self.assertEqual(g2["nodes"]["b1"], [0.5, -1.0, 0.3])
        self.assertEqual(g2["beam_parts"], ["toycar"])

    def test_tuning_and_saved_config(self):
        v = self.cfg()
        rev = next(x for x in v["variables"] if x["name"] == "$revLimiter")
        self.assertEqual((rev["value"], rev["default"], rev["step"], rev["category"]), (6500, 6000, 100, "Engine"))
        camber = next(x for x in v["variables"] if x["name"] == "$camber")
        self.assertEqual(camber["subCategory"], "Front")
        v2 = self.cfg(parts={"toycar_engine": "toycar_engine_big", "toycar_spoiler": "toycar_spoiler_wing"}, vars_={"$camber": 0.98})
        self.assertNotIn("$revLimiter", [x["name"] for x in v2["variables"]])   # the big engine has no limiter variable
        pc = json.loads(bng.write_pc(json.dumps(v2["pc"])))
        self.assertEqual(pc["parts"]["toycar_engine"], "toycar_engine_big")
        self.assertEqual(pc["vars"]["$camber"], 0.98)
        self.assertEqual((pc["format"], pc["model"], pc["mainPartName"]), (2, "toycar", "toycar"))

    def test_other_config_and_missing_part(self):
        v = self.cfg("sport")
        self.assertEqual(find(v["tree"], "toycar_spoiler")["part"], "toycar_spoiler_wing")
        v2 = self.cfg(parts={"wheel_F_toy": "no_such_wheel"})
        self.assertEqual(v2["missing"], [["wheel_F_toy", "no_such_wheel"]])

    def test_catalog(self):
        entries = {"toycar": {"info": CAR["vehicles/toycar/info.json"],
                              "configs": {"base": None, "sport": CAR["vehicles/toycar/info_sport.json"]}}}
        c = json.loads(bng.catalog(json.dumps(entries)))[0]
        self.assertEqual((c["name"], c["brand"], c["type"]), ("Toycar", "Toyco", "Car"))   # translation key -> readable id
        self.assertEqual([(x["name"], x["title"]) for x in c["configs"]], [("base", "Base"), ("sport", "Sport")])


class TestWheelsAndMeshes(unittest.TestCase):
    """pressureWheels read as one table over the active parts; flexbody rows with their transforms."""

    PARTS = {
        "tyre": {"part": {"pressureWheels": [["name", "hubGroup", "group", "node1:", "node2:"], {"radius": 0.3}, {"tireWidth": "$=$w"}]}},
        "rim": {"part": {"pressureWheels": [["name", "hubGroup", "group", "node1:", "node2:"], {"hubRadius": 0.2, "wheelOffset": -0.05}],
                         "flexbodies": [["mesh", "[group]:", "nonFlexMaterials"],
                                        ["rim_a", ["wheel_FL", "wheelhub_FL"], [], {"pos": {"x": 0.47, "y": 0, "z": 0}, "rot": {"x": 0, "y": 0, "z": 180}}],
                                        {"pos": {"x": 0, "y": 0, "z": 0.1}}, ["body_a", ["body"]]]}},
        "wheeldata": {"part": {"pressureWheels": [["name", "hubGroup", "group", "node1:", "node2:"],
                                                  ["FL", "wheel_FL", "tire_FL", "w1ll", "w1l"], ["FR", "wheel_FR", "tire_FR", "w1rr", "w1r"]]}},
    }
    GEO = {"nodes": {"w1ll": [0.8, -1.0, 0.3], "w1l": [0.6, -1.0, 0.3], "w1rr": [-0.8, -1.0, 0.3], "w1r": [-0.6, -1.0, 0.3]}}

    def test_wheel_properties_carry_over_parts(self):
        w = bng.wheels(self.PARTS, ["tyre", "rim", "wheeldata"], self.GEO, {"$w": 0.15})
        self.assertEqual([x["name"] for x in w], ["FL", "FR"])
        fl = w[0]
        self.assertEqual((fl["radius"], fl["width"], fl["hubRadius"], fl["group"], fl["hubGroup"]), (0.3, 0.15, 0.2, "tire_FL", "wheel_FL"))
        self.assertEqual(fl["axis"], [-1.0, 0.0, 0.0])                     # node1 -> node2
        self.assertEqual(fl["centre"], [0.75, -1.0, 0.3])                  # midpoint, then wheelOffset along the axis
        self.assertEqual(w[1]["centre"], [-0.75, -1.0, 0.3])

    def test_flexbody_rows(self):
        f = bng.flexbodies(self.PARTS, ["rim"], {})
        self.assertEqual([(x["mesh"], x["groups"]) for x in f], [("rim_a", ["wheel_FL", "wheelhub_FL"]), ("body_a", ["body"])])
        self.assertEqual((f[0]["pos"], f[0]["rot"], f[0]["scale"]), ([0.47, 0, 0], [0, 0, 180], [1.0, 1.0, 1.0]))
        self.assertEqual(f[1]["pos"], [0, 0, 0.1])                           # from the property row before it
        f2 = bng.flexbodies(self.PARTS, ["rim"], {}, {"rim": [[0.2, -1.0, 0.3], [0.0, 0.0, 0.05]]})
        self.assertEqual(f2[0]["pos"], [0.67, -1.0, 0.35])                    # slot nodeOffset (x by the side) + nodeMove

    def test_rest_positions_and_groups(self):
        bng.reset()
        bng.add_files(json.dumps(CAR))
        g = json.loads(bng.configure("toycar", None, None, None, json.dumps({"nodes": {"b1": [0, 0, 0.1]}})))["geometry"]
        self.assertEqual(g["rest"]["wil"], [0.56, -1.2, 0.3])             # with slot offsets
        self.assertEqual(g["ops"]["steel_wheel_F"][0], [0.25, -1.2, 0.3])
        self.assertEqual(g["rest"]["b1"], [0.5, -1.0, 0.3])                # before the user's move
        self.assertEqual(g["nodes"]["b1"], [0.5, -1.0, 0.4])
        self.assertEqual(g["groups"]["b1"], [])


class TestSources(unittest.TestCase):
    """The install, mods and the user folder as one file system (resolve, inactive_mods, ranks)."""

    def test_user_over_mods_over_install(self):
        r = json.loads(bng.resolve(json.dumps([
            {"name": "/vehicles", "kind": "user", "paths": ["vehicles/toycar/base.pc"]},
            {"name": "/mods/b.zip", "kind": "mod", "paths": ["vehicles/toycar/base.pc", "vehicles/common/extra.jbeam"]},
            {"name": "/mods/a.zip", "kind": "mod", "paths": ["vehicles/common/extra.jbeam"]},
            {"name": "game/toycar.zip", "kind": "vanilla", "paths": ["vehicles/toycar/base.pc", "Vehicles/Toycar/Info.json"]},
        ])))
        self.assertEqual(r["order"], ["game/toycar.zip", "/mods/a.zip", "/mods/b.zip", "/vehicles"])
        self.assertEqual(r["files"]["vehicles/toycar/base.pc"], "/vehicles")
        self.assertEqual(r["shadowed"]["vehicles/toycar/base.pc"], ["game/toycar.zip", "/mods/b.zip"])
        self.assertEqual(r["files"]["vehicles/common/extra.jbeam"], "/mods/b.zip")   # mods: by path
        self.assertEqual(r["files"]["Vehicles/Toycar/Info.json"], "game/toycar.zip")
        self.assertEqual(r["rank"]["/vehicles"], 3)

    def test_inactive_mods(self):
        db = json.dumps({"header": {}, "mods": {
            "a": {"active": True, "fullpath": "/mods/repo/a.zip"},
            "b": {"active": False, "fullpath": "/mods/repo/B.zip"},
            "c": {"active": False, "fullpath": "/mods/unpacked/c/"}}})
        self.assertEqual(json.loads(bng.inactive_mods(db)), ["/mods/repo/b.zip", "/mods/unpacked/c"])
        self.assertEqual(json.loads(bng.inactive_mods("not json {")), [])

    def test_common_part_from_a_mod(self):
        """A mod's vehicles/common part that redefines a common part wins over the install's."""
        bng.reset()
        mod_wheels = {"steel_wheel_F": {"slotType": "wheel_F_toy", "information": {"name": "Mod steel wheels"},
                                        "nodes": [["id", "posX", "posY", "posZ"], ["wil", 0.41, 0, 0], ["wir", -0.41, 0, 0]]}}
        files = dict(CAR, **{"vehicles/common/mod_wheels.jbeam": json.dumps(mod_wheels)})
        bng.add_files(json.dumps(files), json.dumps({"vehicles/common/mod_wheels.jbeam": 1}))
        v = json.loads(bng.configure("toycar"))
        self.assertEqual(find(v["tree"], "wheel_F_toy")["title"], "Mod steel wheels")
        self.assertEqual(v["geometry"]["nodes"]["wil"][0], 0.66)
        bng.reset()
        self.assertEqual(json.loads(bng.held_models()), [])


@unittest.skipUnless(os.environ.get("BEAMNG_VEHICLES"), "set BEAMNG_VEHICLES to a BeamNG content/vehicles folder")
class TestLocalInstall(unittest.TestCase):
    """Reads the user's own install (nothing is copied): every car's default config builds."""

    def test_every_car_builds(self):
        import zipfile
        lib = Path(os.environ["BEAMNG_VEHICLES"])
        bng._DOCS.clear()
        with zipfile.ZipFile(lib / "common.zip") as z:
            bng.add_files(json.dumps({n: z.read(n).decode("utf-8-sig", "replace") for n in z.namelist() if n.endswith(".jbeam")}))
        for zp in sorted(lib.glob("*.zip")):
            if zp.name == "common.zip":
                continue
            with zipfile.ZipFile(zp) as z:
                texts = {n: z.read(n).decode("utf-8-sig", "replace") for n in z.namelist() if n.endswith((".jbeam", ".pc", ".json"))}
            info = next((jbeam.parse(t) for n, t in texts.items() if n.endswith("/info.json")), {})
            if info.get("Type") not in ("Car", "Truck"):
                continue
            bng.add_files(json.dumps(texts))
            model = next(iter({n.split("/")[1] for n in texts}))
            v = json.loads(bng.configure(model))
            self.assertGreater(len(v["geometry"]["beams"]), 100, model)


if __name__ == "__main__":
    unittest.main()
