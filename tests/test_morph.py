"""morph: the outer skin of a box pulled onto a larger mesh, the beams kept inside their length bounds, the slide nodes kept on their rails."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from beamforge import morph  # noqa: E402

if morph.available():
    import numpy as np


def box_tris(lo, hi):
    """The 12 triangles of an axis-aligned box as an (12, 3, 3) array."""
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    c = [(x, y, z) for z in (z0, z1) for y in (y0, y1) for x in (x0, x1)]
    quads = [(0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4), (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5)]
    t = []
    for a, b, d, e in quads:
        t += [[c[a], c[b], c[d]], [c[a], c[d], c[e]]]
    return np.array(t, float)


@unittest.skipUnless(morph.available(), "needs numpy and scipy")
class SamplesAndRelax(unittest.TestCase):
    def test_samples_lie_on_the_triangles_with_unit_normals(self):
        tri = box_tris((0, 0, 0), (1, 1, 1))
        pts, nrm = morph.surface_samples(tri, 0.1)
        self.assertGreater(len(pts), 100)
        self.assertTrue(np.allclose(np.linalg.norm(nrm, axis=1), 1))
        on = np.isclose(pts, 0) | np.isclose(pts, 1)
        self.assertTrue(on.any(axis=1).all())

    def test_outer_nodes_leave_out_the_inner_one(self):
        pos = {f"g{x}{y}{z}": [x, y, z] for x in range(3) for y in range(3) for z in range(3)}
        outer = morph.outer_nodes(pos, list(pos), 0.1)
        self.assertNotIn("g111", outer)
        self.assertIn("g222", outer)
        self.assertEqual(len(outer), 26)

    def test_rbf_goes_through_its_control_points(self):
        rng = np.random.default_rng(1)
        ctrl = rng.random((20, 3))
        disp = rng.random((20, 3)) * 0.1
        f = morph.rbf_map(ctrl, disp, 1e-9)
        self.assertTrue(np.allclose(f(ctrl), disp, atol=1e-6))

    def test_relax_keeps_beams_in_bounds_and_fixed_nodes_still(self):
        X = np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0.0]])
        ia, ib = np.array([0, 1]), np.array([1, 2])
        L0 = np.array([1.0, 1.0])
        target = np.array([[1.0, 0.0, 0.0]])
        Y = morph.relax(X + [0, 0, 0], L0, ia, ib, [0], [2], np.array([[0.2, 0, 0]]), 0.8, 1.2, 100, 0.0)
        self.assertTrue(np.allclose(Y[0], X[0]))
        r = np.linalg.norm(Y[ib] - Y[ia], axis=1)
        self.assertTrue(((r >= 0.8 - 1e-3) & (r <= 1.2 + 1e-3)).all())
        del target

    def test_per_beam_bounds(self):
        X = np.array([[0, 0, 0], [0.1, 0, 0.0]])
        Y = morph.relax(X, np.array([0.2]), np.array([0]), np.array([1]), [0], [], np.zeros((0, 3)), np.array([0.9]), np.array([50.0]), 50, 0.0)
        self.assertGreaterEqual(np.linalg.norm(Y[1] - Y[0]), 0.18 - 1e-3)


class Slides(unittest.TestCase):
    def test_slide_node_keeps_its_fraction_of_the_rail(self):
        base = {"a": [0, 0, 0], "b": [0, 0, 1], "n": [0, 0, 0.25]}
        out = {"a": [1, 0, 0], "b": [1, 0, 2], "n": [5, 5, 5]}
        morph.place_slides(out, base, [["n", "a", "b", 1e5]])
        self.assertEqual(out["n"], [1.0, 0.0, 0.5])


@unittest.skipUnless(morph.available(), "needs numpy and scipy")
class Morph(unittest.TestCase):
    def test_box_skin_grows_onto_a_bigger_mesh(self):
        base, parts = {}, {}
        for i, (x, y, z) in enumerate([(x, y, z) for x in range(5) for y in range(5) for z in range(5)]):
            n = f"n{i}"
            base[n] = [x * 0.25, y * 0.25, z * 0.25]
            parts[n] = "body"
        beams, bp = [], []
        names = list(base)
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                if np.linalg.norm(np.array(base[a]) - base[b]) < 0.26:
                    beams.append([a, b])
                    bp.append("body")
        out = {n: list(p) for n, p in base.items()}
        skin = {n: "body" for n in base}
        tri = np.tile(box_tris((-0.1, -0.1, -0.1), (1.1, 1.1, 1.1)), (10, 1, 1))          # (morph wants at least 100 triangles)
        res = morph.morph(out, base, parts, skin, beams, bp, tri)
        self.assertNotIn("error", res)
        self.assertLess(res["stats"]["after"], res["stats"]["before"])
        self.assertGreaterEqual(res["stats"]["beam_min"], morph.HARD_LO - 0.02)
        self.assertLessEqual(res["stats"]["beam_max"], morph.HARD_HI + 0.02)
        self.assertLess(out["n0"][0], 0.0)


if __name__ == "__main__":
    unittest.main()
