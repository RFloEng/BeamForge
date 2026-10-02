"""The lenient jbeam reader and its safe expression evaluator."""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from beamforge import jbeam  # noqa: E402


class TestJBeamParser(unittest.TestCase):
    def test_comments_and_optional_commas(self):
        doc = jbeam.parse('{"a": [1 2, 3,], /* c */ "b": {"x": "y",} // end\n "c": -1.5e3}')
        self.assertEqual(doc, {"a": [1, 2, 3], "b": {"x": "y"}, "c": -1500.0})

    def test_leading_plus_sign(self):
        self.assertEqual(jbeam.parse('[+0.5, +2, -1, 1e+3]'), [0.5, 2, -1, 1000.0])

    def test_hostile_and_non_numeric_values_are_unresolved(self):
        with self.assertRaises(jbeam.UnresolvedValue):
            jbeam.resolve("$=9**9**9**9", {})                       # would hang eval()
        with self.assertRaises(jbeam.UnresolvedValue):
            jbeam.to_float("abc", {})
        with self.assertRaises(jbeam.UnresolvedValue):
            jbeam.resolve("$=$k*2", {"$k": "soft"})

    def test_table_properties_and_inline_override(self):
        rows = [["id", "posX"], {"nodeWeight": 2}, ["a", 0], ["b", 1, {"nodeWeight": 5}], ["c", 2]]
        recs = jbeam.expand_table(rows)
        self.assertEqual([r["nodeWeight"] for r in recs], [2, 5, 2])

    def test_variables(self):
        self.assertEqual(jbeam.resolve("$k", {"$k": 3}), 3)
        self.assertAlmostEqual(jbeam.resolve("$=$k*2+1", {"$k": 3}), 7.0)
        self.assertTrue(math.isinf(jbeam.to_float("FLT_MAX", {})))
        with self.assertRaises(jbeam.UnresolvedValue):
            jbeam.resolve("$=os.system('x')", {})


if __name__ == "__main__":
    unittest.main()
