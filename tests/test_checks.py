"""Static checks of a built vehicle (checks.py), on made-up nodes."""
import unittest

from beamforge import checks


class TestChecks(unittest.TestCase):
    def test_torsion_bar_in_a_line_is_found(self):
        N = {"a": [0, 0, 0], "b": [0, 0, 1], "c": [0.0, 0.5, 1.0], "d": [0.0, 1.0, 1.01]}
        docs = {"part": {"torsionbars": [["id1:", "id2:", "id3:", "id4:"], ["a", "b", "c", "d"]]}}
        self.assertEqual(len(checks.torsion_bars(N, docs)), 1)

    def test_torsion_bar_with_a_frame_passes(self):
        N = {"a": [0, 0, 0], "b": [0, 0, 1], "c": [0.0, 0.5, 1.0], "d": [0.5, 0.5, 1.5]}
        docs = {"part": {"torsionbars": [["id1:", "id2:", "id3:", "id4:"], ["a", "b", "c", "d"]]}}
        self.assertEqual(checks.torsion_bars(N, docs), [])

    def test_overlap_only_where_the_base_had_them_apart(self):
        N = {"a": [0, 0, 0], "b": [0.001, 0, 0], "c": [1, 0, 0], "d": [1.001, 0, 0]}
        parts = {"a": "p", "b": "q", "c": "p", "d": "q"}
        base_N = {"a": [0, 0, 0], "b": [0.3, 0, 0], "c": [1, 0, 0], "d": [1.001, 0, 0]}
        out = checks.overlaps(N, parts, base_N, parts)
        self.assertEqual(len(out), 1)
        self.assertIn("1 node pairs", out[0])

    def test_torque_rating_under_the_peak_is_found(self):
        docs = {"eng": {"mainEngine": {"torque": [["rpm", "torque"], [1000, 100], [3000, 300]],
                                       "maxTorqueRating": 310, "maxOverTorqueDamage": 400}}}
        self.assertEqual(len(checks.engine(docs)), 1)
        docs["eng"]["mainEngine"]["maxTorqueRating"] = 400
        docs["eng"]["mainEngine"]["maxOverTorqueDamage"] = 500
        self.assertEqual(checks.engine(docs), [])


if __name__ == "__main__":
    unittest.main()
