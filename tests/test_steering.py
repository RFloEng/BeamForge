"""Which way a wheel turns when the rack moves (beamforge/steering.py)."""

import json
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from beamforge import steering  # noqa: E402


def turn(axis_down, axis_up, tie_rod_end, rack_end, rack_other, factor):
    """dtheta per metre for one wheel, by the formula direction() uses (a configured vehicle with one wheel)."""
    nodes = {"d": axis_down, "u": axis_up, "h": tie_rod_end, "e": rack_end, "s": rack_other, "a": [0.6, 0, 0.3], "b": [0.85, 0, 0.3]}
    u = steering._unit(steering._sub(nodes["u"], nodes["d"]))
    de = steering._unit(steering._sub(nodes["e"], nodes["s"]))
    de = [x * (1 if factor > 0 else -1) for x in de]
    he = steering._sub(nodes["h"], nodes["e"])
    r = steering._sub(nodes["h"], nodes["d"])
    return steering._dot(he, de) / steering._dot(he, steering._cross(u, r))


class TestDirection(unittest.TestCase):
    def test_tie_rod_end_ahead_or_behind_the_axis_turns_the_wheel_the_other_way(self):
        # left front wheel, steering axis nearly vertical at x 0.68, y -1.3; the rack end inboard at x 0.3
        ahead = turn([0.68, -1.3, 0.2], [0.55, -1.28, 0.8], [0.68, -1.45, 0.24], [0.3, -1.4, 0.24], [-0.25, -1.4, 0.24], 0.135)
        behind = turn([0.68, -1.3, 0.2], [0.55, -1.28, 0.8], [0.68, -1.15, 0.24], [0.3, -1.2, 0.24], [-0.25, -1.2, 0.24], 0.135)
        self.assertLess(ahead * behind, 0)                    # the Z3 on the RWD coupe: the base's rack, the SVJ's rod

    def test_reversed_factor_turns_it_back(self):
        a = turn([0.68, -1.3, 0.2], [0.55, -1.28, 0.8], [0.68, -1.45, 0.24], [0.3, -1.4, 0.24], [-0.25, -1.4, 0.24], 0.135)
        b = turn([0.68, -1.3, 0.2], [0.55, -1.28, 0.8], [0.68, -1.45, 0.24], [0.3, -1.4, 0.24], [-0.25, -1.4, 0.24], -0.135)
        self.assertAlmostEqual(a, -b)


class TestFix(unittest.TestCase):
    def test_reversed_wheel_has_its_hydro_negated(self):
        part = {"hydros": [["id1:", "id2:"], {"beamSpring": 1}, ["fhub6r", "fsub5l", {"factor": 0.135}], ["fhub6l", "fsub5r", {"factor": -0.135}]]}
        files = {"vehicles/x/s.jbeam": json.dumps({"steering": part})}
        base = {"FL": {"turn": -7.0, "rack_end": "fhub6l", "tie_rod_end": "a"}, "FR": {"turn": -7.0, "rack_end": "fhub6r", "tie_rod_end": "b"}}
        built = {"FL": {"turn": 7.5, "rack_end": "fhub6l", "tie_rod_end": "a"}, "FR": {"turn": -7.2, "rack_end": "fhub6r", "tie_rod_end": "b"}}
        notes, flipped = steering.fix(files, "x", base, built)
        self.assertEqual(flipped, ["FL"])
        rows = json.loads(files["vehicles/x/s.jbeam"])["steering"]["hydros"]
        self.assertEqual([r[-1]["factor"] for r in rows if isinstance(r, list) and isinstance(r[-1], dict)], [0.135, 0.135])
        self.assertEqual(len(notes), 1)


if __name__ == "__main__":
    unittest.main()
