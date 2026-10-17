"""A suspension corner copied from another vehicle (beamforge/susp_donor.py), with a made-up donor."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from beamforge import scratch, susp_donor  # noqa: E402


def donor():
    """A made-up front axle: a hub and a lower arm a side, a spring-damper, the body's two nodes it hangs on, a rim of two nodes."""
    nodes, beams, hydros = {}, [], []
    for s, c in ((1, "l"), (-1, "r")):
        nodes[f"h1{c}"] = {"pos": [s * 0.62, 0.0, 0.15], "kg": 5.0, "props": {}}
        nodes[f"h2{c}"] = {"pos": [s * 0.60, 0.0, 0.45], "kg": 5.0, "props": {}}
        nodes[f"h3{c}"] = {"pos": [s * 0.62, -0.12, 0.28], "kg": 5.0, "props": {}}
        for a, b in ((f"h1{c}", f"h2{c}"), (f"h2{c}", f"h3{c}"), (f"h3{c}", f"h1{c}")):
            beams.append([a, b, {"beamSpring": 3000000, "beamDamp": 100, "beamDeform": 60000, "beamStrength": "FLT_MAX"}])
        for rim in (f"w1{c}", f"w2{c}"):
            for h in (f"h1{c}", f"h2{c}", f"h3{c}"):
                beams.append([rim, h, {"beamSpring": 2000000, "beamDamp": 80, "beamDeform": 90000, "beamStrength": "FLT_MAX"}])
        beams.append([f"h1{c}", f"b1{c}", {"beamSpring": 1500000, "beamDamp": 100}])             # the lower arm, to the body
        beams.append([f"h2{c}", f"b2{c}", {"beamSpring": 60000, "beamDamp": 0, "name": f"spring_F{c}", "precompressionRange": 0.03}])
        beams.append([f"h1{c}", f"b2{c}", {"beamSpring": 0, "beamDamp": 4000, "beamDampRebound": 8000, "name": f"damper_F{c}"}])
        hydros.append([f"h3{c}", f"b1{c}", {"factor": 0.2, "steeringWheelLock": 400}])
    wheels = {"FL": {"node1": "w2l", "node2": "w1l", "centre": [0.725, 0.0, 0.3]},
              "FR": {"node1": "w2r", "node2": "w1r", "centre": [-0.725, 0.0, 0.3]}}
    ext = {"b1l": [0.35, 0.1, 0.2], "b2l": [0.4, 0.0, 0.8], "b1r": [-0.35, 0.1, 0.2], "b2r": [-0.4, 0.0, 0.8]}
    return {"model": "toy", "axle": "F", "parts": ["toy_suspension_F"], "nodes": nodes, "beams": beams, "slides": [], "rails": {},
            "hydros": hydros, "torsion": [], "ext": ext, "wheels": wheels, "notes": [], "mass": 1000.0, "share": 0.55,
            "springs": [i for i, b in enumerate(beams) if str(b[2].get("name", "")).startswith("spring_")],
            "dampers": [i for i, b in enumerate(beams) if str(b[2].get("name", "")).startswith("damper_")],
            "wheel_rows": {"FL": {"torqueCoupling": "h1l", "torqueArm": "h2l", "nodeArm": "h1l", "wheelDir": -1},
                           "FR": {"torqueCoupling": "h1r", "torqueArm": "h2r", "nodeArm": "h1r", "wheelDir": 1}}}


class TestPlace(unittest.TestCase):
    def car(self):
        car = scratch._Car()
        frame = []
        for i, (x, y, z) in enumerate([(-0.4, 0.2, 0.2), (0.4, 0.2, 0.2), (-0.4, -0.4, 0.2), (0.4, -0.4, 0.2), (-0.4, 0.0, 0.8), (0.4, 0.0, 0.8)]):
            car.node(f"f{i}", [x, y, z], 6.0, group="sk_frame")
            frame.append(f"f{i}")
        for k in ("o1", "o2", "i1", "i2"):                    # the new rim's axle nodes (not in the car's nodes: the rim part's)
            car.kg[k], car.ks[k], car.cs[k] = 4.5, 0.0, 0.0
        return car, frame

    def test_corner_moves_to_its_wheel_and_hangs_on_the_frame(self):
        car, frame = self.car()
        wheels = {"FL": {"centre": [0.70, 0.5, 0.29]}, "FR": {"centre": [-0.70, 0.5, 0.29]}}
        placed = susp_donor.place(car, donor(), wheels, frame, {"FL": ("o1", "i1"), "FR": ("o2", "i2")})
        m = placed["map"]
        self.assertEqual(m["w2l"], "o1")                                   # the donor's rim nodes are the new rim's
        self.assertEqual(m["w1r"], "i2")
        self.assertEqual([round(x, 3) for x in car.nodes["h1l"]], [0.595, 0.5, 0.14])      # the corner moved with its wheel
        self.assertTrue(m["b1l"].startswith("bfp_"))                      # a pickup of its own, tied to the frame
        self.assertTrue(any(b[0] == m["b1l"] and b[1] in frame for b in car.beams))
        self.assertEqual(len(placed["springs"]), 2)
        self.assertEqual(len(placed["dampers"]), 2)
        self.assertEqual(placed["hydros"][0][2]["factor"], 0.2)           # the donor's numbers as they were
        beam = next(b for b in car.beams if b[0] == "h1l" and b[1] == "h2l")
        self.assertEqual(beam[2]["beamSpring"], 3000000)

    def test_springs_scaled_to_the_car_weight(self):
        car, frame = self.car()
        wheels = {"FL": {"centre": [0.70, 0.5, 0.29]}, "FR": {"centre": [-0.70, 0.5, 0.29]}}
        placed = susp_donor.place(car, donor(), wheels, frame, {"FL": ("o1", "i1"), "FR": ("o2", "i2")})
        k0 = car.beams[placed["springs"][0]][2]["beamSpring"]
        w = {"FL": {"centre": [0.7, 0.0, 0.3]}, "FR": {"centre": [-0.7, 0.0, 0.3]}, "RL": {"centre": [0.7, 2.5, 0.3]}, "RR": {"centre": [-0.7, 2.5, 0.3]}}
        scratch._donor_parts(car, {"F": donor()}, {"F": placed}, w, {})
        k1 = car.beams[placed["springs"][0]][2]["beamSpring"]
        self.assertNotEqual(k0, k1)                                       # times this car's corner weight over the donor's


if __name__ == "__main__":
    unittest.main()
