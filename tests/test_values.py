"""Values taken from an SVJ (beamforge/values.py): springs, dampers, tyres."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from beamforge import values  # noqa: E402

SVJ = {"suspension": {
    "FL": {"spring": {"rate": 22000, "motion_ratio": 0.9},
           "damper": {"bump_curve": [[0, 0], [0.05, 550], [0.2, 1400]], "rebound_curve": [[0, 0], [0.05, 820], [0.2, 2100]]},
           "wheel": {"loaded_radius": 0.295}},
    "RR": {"spring": {"rate": 24000}, "tire": {"unloaded_radius": 0.31}}}}


class TestValues(unittest.TestCase):
    def test_svj_values(self):
        sv = values.svj_values(SVJ, {"corners": {"rear": {"static": {"motion_ratio": 0.6}}}})
        f, r = sv["front"], sv["rear"]
        self.assertEqual(f["coil_rate"], round(22000 / 0.81))               # wheel rate / MR^2 (the file's MR)
        self.assertEqual(r["coil_rate"], round(24000 / 0.36))               # the study's MR when the file has none
        self.assertEqual((f["damp_bump"], f["damp_bump_fast"], f["damp_split"]), (11000, 5667, 0.05))
        self.assertEqual(f["damp_rebound"], 16400)
        self.assertEqual(f["tyre_radius"], 0.305)                           # loaded + 10 mm
        self.assertEqual(r["tyre_radius"], 0.31)                            # the unloaded radius when given
        self.assertIsNone(r["damp_bump"])

    def test_apply_to_part(self):
        part = {"beams": [["id1:", "id2:"], {"beamType": "|NORMAL"}, ["a", "b"], ["c", "d", {"beamSpring": 30000}]],
                "pressureWheels": [["name"], {"radius": 0.33}],
                "flexbodies": [["mesh", "[group]:", "nonFlexMaterials"], ["tire_01a", ["tire_FL"], [], {"scale": {"x": 1, "y": 1, "z": 1}}],
                               ["rim_01a", ["wheel_FL"]]]}
        values.apply_to_part("p", part, {"p": {2: {"beamSpring": 23000}, 3: {"beamSpring": 23000}}},
                             {"p": {"radius": 0.305, "scale": [1.0, 0.92, 0.92]}})
        self.assertEqual(part["beams"][2], ["a", "b", {"beamSpring": 23000}])     # a row of its own gets inline values
        self.assertEqual(part["beams"][3][-1], {"beamSpring": 23000})            # an inline value is replaced
        self.assertEqual(part["pressureWheels"][1]["radius"], 0.305)
        self.assertEqual(part["flexbodies"][1][-1]["scale"], {"x": 1.0, "y": 0.92, "z": 0.92})
        self.assertEqual(part["flexbodies"][2], ["rim_01a", ["wheel_FL"]])        # only the tyre mesh is scaled

    def test_mass_and_cg(self):
        """Node weights rescaled to a target mass and CG, as the game would read them back."""
        import json
        from beamforge import beamng as bng
        from tests.test_beamng import CAR
        bng.reset()
        bng.add_files(json.dumps(CAR))
        v = json.loads(bng.configure("toycar"))
        v["measure"].update(front_axle_y=-1.2, ground_z=0.0)                 # the toy car has no wheels
        rows, wheel_kg = values.node_weights("toycar", v)
        self.assertEqual(sum(r[3] for r in rows), 25.0 * len(rows))          # no nodeWeight: the game's 25 kg
        pos = {(r[0], r[1]): r[4] for r in rows}

        def result(w):
            m = sum(kg for part in w.values() for kg in part.values())
            cy = sum(kg * pos[(p, i)][1] for p, part in w.items() for i, kg in part.items()) / m
            return m, cy
        m, _ = result(values.weight_changes("toycar", v, mass=200.0))
        self.assertAlmostEqual(m + wheel_kg, 200.0, places=3)
        mean = sum(r[3] * r[4][1] for r in rows) / sum(r[3] for r in rows)   # -0.9 (nodes at y -1.0 and -0.6)
        m, cy = result(values.weight_changes("toycar", v, mass=200.0, cg_y=0.25))   # 0.25 behind the front axle
        self.assertAlmostEqual(m + wheel_kg, 200.0, places=3)                # the mass is kept
        self.assertLess(abs(cy - (-0.95)), abs(mean - (-0.95)))              # moved towards the target
        w = values.weight_changes("toycar", v, cg_y=0.25)                    # the mass as it is: no rescale
        self.assertTrue(all(kg >= 25.0 * values.MIN_FACTOR - 1e-6 for part in w.values() for kg in part.values()))

    def test_powertrain(self):
        svj = {"powertrain": {"layout": "FR", "engine": {"idle_rpm": 750, "max_rpm": 6350,
                                                         "torque_curve": [[1000, 140], [4000, 226], [6350, 190]]},
                              "gearbox": {"type": "manual", "ratios": [3.83, 2.2, 1.4, 1.0, 0.81]},
                              "differentials": [{"location": "rear", "type": "open", "final_drive": 3.73}]}}
        sp = values.svj_powertrain(svj)
        self.assertEqual((sp["driven"], sp["turbo"], sp["final_drive"]), ("rear", False, 3.73))
        self.assertEqual(values._peak(sp["torque"]), (226, 126.3))          # peak power: 190 Nm at 6350 rpm
        self.assertEqual(values._interp([[0, 0], [1000, -10]], 500), -5)
        part = {"mainEngine": {"torque": [["rpm", "torque"], [0, 0]], "idleRPM": 650},
                "gearbox": {"gearRatios": [-3.3, 0, 4.7, 3.1]},
                "powertrain": [["type", "name"], ["differential", "differential_R", {"gearRatio": 3.07}]]}
        values.apply_powertrain("e", part, {"e": {"mainEngine.idleRPM": 750, "gearbox.gearRatios": [-3.3, 0, 3.83],
                                                  ("powertrain", 1): {"gearRatio": 3.73}}})
        self.assertEqual((part["mainEngine"]["idleRPM"], part["gearbox"]["gearRatios"]), (750, [-3.3, 0, 3.83]))
        self.assertEqual(part["powertrain"][1][-1]["gearRatio"], 3.73)

    def test_steering_turns(self):
        self.assertEqual(values.svj_steering({"steering": {"lock_to_lock_turns": 3.5, "overall_ratio": 20.5}}), (3.5, 20.5))
        self.assertEqual(values.svj_steering({"steering": {"lock_to_lock": 21.991}})[0], 3.5)      # radians of wheel
        self.assertEqual(values.svj_steering({}), (None, None))

    def test_aero(self):
        self.assertEqual(values.svj_aero({"aerodynamics": {"reference": {"frontal_area": 1.72}, "coefficients": {"Cd": 0.31}}}),
                         (0.5332, 0.31, 1.72))
        comps = {"aerodynamics": {"reference": {"frontal_area": 1.5},
                                  "components": [{"Cd_contribution": 0.28}, {"Cd_contribution": 0.45}]}}
        self.assertEqual(values.svj_aero(comps)[1], 0.73)                   # summed contributions
        self.assertEqual(values.svj_aero({}), (None, None, None))
        # a 1 m2 square facing the flow (two triangles) at 100 %: 1 m2; turned 60 degrees away: a quarter
        self.assertAlmostEqual(values.drag_area([("p", 1, 100, 100, 0.5, 1.0), ("p", 2, 100, 100, 0.5, 1.0)]), 1.0)
        self.assertAlmostEqual(values.drag_area([("p", 1, 100, 100, 1.0, 0.5)]), 0.25)

    def test_tyre_size_and_wheel_parts(self):
        """The tyre set a corner refers to, and the base's wheel and tyre parts closest to it."""
        svj = {"tires": {"sets": {"t": {"dimensions": {"section_width": 0.195, "aspect_ratio": 0.564, "rim_diameter_code": 15,
                                                     "overall_diameter": 0.601}}}},
               "suspension": {"FL": {"tire": {"set_ref": "t"}}, "RL": {"tire": {"set_ref": "t"}}}}
        self.assertEqual(values.tyre_size(svj, svj["suspension"]["FL"]), {"width": 0.195, "aspect": 0.564, "rim_in": 15, "diameter": 0.601})
        self.assertEqual(values.svj_values(svj)["front"]["tyre_radius"], 0.3005)
        pick, _ = values._closest_wheel("wheel_F_4", "steel_13x5_F", ["steel_13x5_F", "alloy_15x6_F", "alloy_15x7_F", "alloy_17x7_F"], svj)
        self.assertEqual(pick, "alloy_15x7_F")                               # R15, near the tyre width less an inch
        self.assertEqual(values._closest_wheel("wheel_F_4", "alloy_15x6_F", ["alloy_15x7_F"], svj), (None, ""))   # R15 already
        pick, _ = values._closest_wheel("tire_F_15x7", "tire_F_206_51_15_sport", ["tire_F_206_51_15_sport", "tire_F_196_61_15_sport"], svj)
        self.assertEqual(pick, "tire_F_196_61_15_sport")

    def test_final_drive_through_the_driveline(self):
        """The final drive is every reduction between the gearbox and the wheels; a final drive part sets it
        with a section named after a device; taking the SVJ's changes that one, not the differential."""
        import json
        from beamforge import beamng as bng
        files = {"vehicles/dl/dl.jbeam": json.dumps({
            "dl_body": {"information": {"name": "Body"}, "slotType": "main",
                        "slots": [["type", "default", "description"], ["dl_final", "dl_final_425", "Final drive"]],
                        "powertrain": [["type", "name", "inputName", "inputIndex"],
                                       ["manualGearbox", "gearbox", "mainEngine", 1],
                                       ["torsionReactor", "torsionReactorF", "gearbox", 1],
                                       ["differential", "differential_F", "torsionReactorF", 1, {"gearRatio": 1}],
                                       ["shaft", "wheelaxleFL", "differential_F", 1]]},
            "dl_final_425": {"information": {"name": "4.25"}, "slotType": "dl_final", "torsionReactorF": {"gearRatio": 4.25}}}),
            "vehicles/dl/info.json": json.dumps({"Name": "DL", "Type": "Car"})}
        bng.reset()
        bng.add_files(json.dumps(files))
        v = json.loads(bng.configure("dl"))
        prod, chain = values.driveline("dl", v, "front")
        self.assertEqual(prod, 4.25)
        svj = {"powertrain": {"layout": "FF", "differentials": [{"location": "front", "final_drive": 4.4}]}}
        self.assertEqual(values.powertrain_changes("dl", v, svj, {"final_drive": True}),
                         {"dl_final_425": {"torsionReactorF.gearRatio": 4.4}})

    def test_slopes(self):
        self.assertEqual(values._slopes([[0, 0], [0.1, 1000]]), (10000.0, 10000.0, 0.1))
        self.assertIsNone(values._slopes([[0, 0]]))


if __name__ == "__main__":
    unittest.main()
