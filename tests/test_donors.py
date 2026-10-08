"""What the vanilla cars are made of (beamforge/donors.py), on made-up parts."""

import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from beamforge import beamng as bng, donors  # noqa: E402

PARTS = {"vehicles/toy/toy_engine.jbeam": json.dumps({
    "toy_engine_i4": {"information": {"name": "1.6 I4"}, "slotType": "toy_engine",
                      "variables": [["name", "type", "unit", "category", "default", "min", "max"], ["$revs", "range", "rpm", "Engine", 6800, 5000, 8000]],
                      "powertrain": [["type", "name", "inputName", "inputIndex"], ["combustionEngine", "mainEngine", "dummy", 0]],
                      "mainEngine": {"torque": [["rpm", "torque"], [0, 0], [1000, 100], [4000, 150], [6000, 140], [7000, 120]],
                                     "idleRPM": 800, "maxRPM": "$revs", "inertia": 0.08, "requiredEnergyType": "gasoline"},
                      "soundConfig": {"sampleName": "I4_2_engine"}},
    "toy_motor": {"slotType": "toy_motor", "powertrain": [["type", "name", "inputName", "inputIndex"], ["electricMotor", "rearMotor", "dummy", 0]],
                  "rearMotor": {"torque": [["rpm", "torque"], [0, 300], [5000, 300], [12000, 120]], "maxRPM": 12000}},
    "toy_transmission_5M": {"slotType": "toy_transmission",
                            "powertrain": [["type", "name", "inputName", "inputIndex"], ["frictionClutch", "clutch", "mainEngine", 1], ["manualGearbox", "gearbox", "clutch", 1]],
                            "gearbox": {"gearRatios": [-3.5, 0, 3.6, 2.1, 1.4, 1.0, 0.8]}},
    "tire_F_206_54_16_sport": {"slotType": "tire_F_16x7t", "pressureWheels": [["name"], {"hasTire": True}, {"radius": 0.315}, {"tireWidth": 0.185}, {"frictionCoef": 1.1}]},
    "tire_RR_30_11_15_alt_drag": {"slotType": "tire_R_15x10t", "pressureWheels": [["name"], {"hasTire": True}, {"radius": 0.37}, {"tireWidth": 0.25}]},
    "wheel_02a_16x7_F": {"slotType": "wheel_F_5", "pressureWheels": [["name"], {"hubRadius": 0.22}, {"hubWidth": 0.16}]},
})}


class TestDonors(unittest.TestCase):
    def setUp(self):
        bng.reset()
        bng.add_files(json.dumps(PARTS))

    def test_catalogue(self):
        cat = donors.catalogue(["toy"])
        eng = {e["part"]: e for e in cat["engines"]}
        i4 = eng["toy_engine_i4"]
        self.assertEqual((i4["cylinders"], i4["max_rpm"], i4["peak_torque"], i4["kind"]), ("I4", 6800.0, 150.0, "combustion"))   # a variable resolved
        self.assertAlmostEqual(i4["peak_power"], 120 * 7000 * 2 * 3.14159265 / 60 / 1000, delta=0.1)
        self.assertEqual((eng["toy_motor"]["kind"], eng["toy_motor"]["device"]), ("electric", "rearMotor"))
        g = cat["gearboxes"][0]
        self.assertEqual((g["type"], g["gears"], g["ratios"], g["reverse"]), ("manual", 5, [3.6, 2.1, 1.4, 1.0, 0.8], -3.5))
        t = {x["part"]: x for x in cat["tyres"]}
        self.assertEqual((t["tire_F_206_54_16_sport"]["rim_in"], t["tire_F_206_54_16_sport"]["use"], t["tire_F_206_54_16_sport"]["frictionCoef"]), (16, "sport", 1.1))
        drag = t["tire_RR_30_11_15_alt_drag"]
        self.assertEqual((drag["rim_in"], drag["use"], drag["width_mm"], drag["diameter_in"]), (15, "drag", 279, 30.0))   # inch sizes
        self.assertEqual((cat["rims"][0]["diameter_in"], cat["rims"][0]["width_in"]), (16, 7.0))

    def test_archetypes(self):
        a = donors.archetypes(donors.catalogue(["toy"]))
        self.assertEqual([x["name"] for x in a["engines"]], ["Electric motor, strong (150-250 kW)", "I4 petrol, medium (75-150 kW)"])
        self.assertEqual([x["name"] for x in a["gearboxes"]], ["5-speed manual"])
        self.assertEqual(sorted(x["name"] for x in a["tyres"]), ["15 in drag", "16 in sport"])
        self.assertEqual(a["engines"][1]["pick"]["part"], "toy_engine_i4")


if __name__ == "__main__":
    unittest.main()
