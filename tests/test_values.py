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

    def test_slopes(self):
        self.assertEqual(values._slopes([[0, 0], [0.1, 1000]]), (10000.0, 10000.0, 0.1))
        self.assertIsNone(values._slopes([[0, 0]]))


if __name__ == "__main__":
    unittest.main()
