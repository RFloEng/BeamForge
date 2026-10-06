"""A node held by fewer than three independent beams is a mechanism (beamforge/structure.py)."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from beamforge import structure  # noqa: E402


class TestRank(unittest.TestCase):
    def test_three_independent_directions_hold_a_node(self):
        self.assertEqual(structure._rank([[1, 0, 0], [0, 1, 0], [0, 0, 1]]), 3)

    def test_beams_in_a_plane_or_a_line_leave_a_direction_free(self):
        self.assertEqual(structure._rank([[1, 0, 0], [0, 1, 0], [0.7071, 0.7071, 0], [-1, 0, 0]]), 2)   # a plane
        self.assertEqual(structure._rank([[1, 0, 0], [-1, 0, 0], [1, 0.05, 0]]), 1)                       # a line
        self.assertEqual(structure._rank([[1, 0, 0], [0, 1, 0]]), 2)                                      # two beams: free

    def test_held_counts_beams_and_slides(self):
        geo = {"nodes": {"a": [0, 0, 0], "b": [1, 0, 0], "c": [0, 1, 0], "d": [0, 0, 1], "e": [0.5, 0.5, 0.5], "r1": [0, 0, 2], "r2": [0, 0, 3]},
               "parts": {}, "slides": []}
        bl = [{"a": "a", "b": x, "type": "|NORMAL", "values": {"beamSpring": 1e6}} for x in ("b", "c", "d")]
        bl += [{"a": "e", "b": "b", "type": "|NORMAL", "values": {"beamSpring": 1e6}},
               {"a": "e", "b": "c", "type": "|NORMAL", "values": {"beamSpring": 1e6}},
               {"a": "e", "b": "d", "type": "|SUPPORT", "values": {"beamSpring": 1e6}}]       # a support beam holds nothing at rest
        h = structure.held("m", {"geometry": geo}, bl)
        self.assertEqual(h["a"]["rank"], 3)
        self.assertEqual(h["e"]["rank"], 2)
        geo["slides"] = [["e", "r1", "r2", 1e6]]                                                # held across its rail: 2 more
        self.assertEqual(structure.held("m", {"geometry": geo}, bl)["e"]["rank"], 3)


if __name__ == "__main__":
    unittest.main()
