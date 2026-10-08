"""The Powertrain workspace's values (beamforge/powertrain.py): an SVJ-shaped block from the SVJ or by hand."""

import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from beamforge import powertrain as pt, values  # noqa: E402

SVJ = {"powertrain": {"layout": "FF", "engine": {"idle_rpm": 850, "max_rpm": 8000, "torque_curve": [[1000, 120], [6000, 160], [8000, 140]]},
                      "gearbox": {"type": "manual", "ratios": [3.2, 2.1, 1.4, 1.1, 0.85]},
                      "differentials": [{"location": "front", "final_drive": 4.4}]},
       "steering": {"lock_to_lock_turns": 3.2}}


class TestPowertrain(unittest.TestCase):
    def test_forward_gears(self):
        self.assertEqual(pt.forward([-3.3, 0, 3.6, 2.1, 1.4]), [3.6, 2.1, 1.4])
        self.assertEqual(pt.forward(["$=-$gear_R", 0, "$g1", "$g2"], {"$g1": 3.4, "$g2": 2.0}), [3.4, 2.0])   # tuning variables

    def test_from_svj(self):
        e = json.loads(pt.from_svj_json(json.dumps(SVJ)))
        self.assertEqual((e["gears"], e["final_drive"], e["steering_turns"], e["engine"]["max_rpm"]), ([3.2, 2.1, 1.4, 1.1, 0.85], 4.4, 3.2, 8000))
        s = json.loads(pt.summary_json(None, None, json.dumps(SVJ)))
        self.assertEqual((s["base"], s["svj"]["driven"], s["svj"]["layout"]), (None, "front", "FF"))

    def test_export_by_hand_and_from_the_svj(self):
        # by hand, without any SVJ: only what is set is taken
        x = json.loads(pt.export_json(json.dumps({"gears": [3.0, 1.9, 1.3], "final_drive": 4.1}), None))
        self.assertEqual(x["take"], {"engine": False, "gears": True, "final_drive": True, "steering": False, "diffs": False})
        sp = values.svj_powertrain(x["svj"])
        self.assertEqual((sp["ratios"], sp["final_drive"]), ([3.0, 1.9, 1.3], 4.1))
        # an edit over the SVJ's block replaces its values, the rest of the SVJ stays
        e = {"engine": {"torque": [[1000, 100], [7000, 200]], "idle_rpm": 900, "max_rpm": 7200}, "steering_turns": 2.5}
        y = json.loads(pt.export_json(json.dumps(e), json.dumps(SVJ)))
        sp = values.svj_powertrain(y["svj"])
        self.assertEqual((sp["torque"], sp["idle_rpm"], sp["max_rpm"], sp["final_drive"]), ([[1000.0, 100.0], [7000.0, 200.0]], 900.0, 7200.0, 4.4))
        self.assertEqual(values.svj_steering(y["svj"])[0], 2.5)
        self.assertEqual(y["take"], {"engine": True, "gears": False, "final_drive": False, "steering": True, "diffs": False})
        self.assertEqual(SVJ["steering"]["lock_to_lock_turns"], 3.2)                    # the SVJ itself untouched


    def test_differentials_inertia_and_hand_values(self):
        svj = json.loads(json.dumps(SVJ))
        svj["powertrain"]["engine"]["inertia"] = 0.11
        svj["powertrain"]["differentials"][0].update(type="lsd_clutch", preload=60, lock_power=0.35, lock_coast=0.15)
        svj["powertrain"]["transfer_case"] = {"torque_split": [0.4, 0.6]}
        e = json.loads(pt.from_svj_json(json.dumps(svj)))
        self.assertEqual((e["engine"]["inertia"], e["diffs"]["front"]["type"], e["diffs"]["split"]), (0.11, "lsd_clutch", 0.4))
        self.assertEqual(values.bng_diff(e["diffs"]["front"]), {"diffType": "lsd", "lsdPreload": 60, "lsdLockCoef": 0.35, "lsdRevLockCoef": 0.15})
        self.assertEqual(values.bng_diff({"type": "lsd_viscous"}), {"diffType": "viscous"})
        self.assertEqual(values.bng_diff({"type": "spool"}), {"diffType": "locked"})
        self.assertEqual(values.bng_diff({"type": "unknown"}), {})
        e["engine"].update(friction=12, engine_brake=40)
        y = json.loads(pt.export_json(json.dumps(e), None))
        sp = values.svj_powertrain(y["svj"])
        self.assertEqual((sp["inertia"], sp["x_beamng"], sp["diffs"]["front"]["type"], sp["split"], y["take"]["diffs"]),
                         (0.11, {"friction": 12.0, "engineBrakeTorque": 40.0}, "lsd_clutch", 0.4, True))


if __name__ == "__main__":
    unittest.main()
