"""Tyres (beamforge/tyres.py): the Magic Formula, BeamNG's load coefficient, the fit."""

import json
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from beamforge import tyres  # noqa: E402

LAT = {"pCy1": 1.4, "pDy1": 1.1, "pDy2": -0.15, "pEy1": -0.5, "pKy1": -18.0, "pKy2": 2.0, "pKy4": 2.0}
LON = {"pCx1": 1.65, "pDx1": 1.2, "pDx2": -0.08, "pEx1": 0.4, "pKx1": 25.0}
SVJ = {"tires": {"sets": {"t1": {"size_code": "205/55R16", "dimensions": {"rim_diameter_code": 16, "section_width": 0.205, "overall_diameter": 0.632},
                                 "reference": {"load": 4000}, "pacejka": {"model": "MF52", "lateral": LAT, "longitudinal": LON}}}},
       "suspension": {"FL": {"tire": {"set_ref": "t1"}}}}


class TestTyres(unittest.TestCase):
    def test_magic_formula(self):
        # at the reference load the peak is D = pDy1 Fz (no shifts); zero slip, zero force; the cornering stiffness
        peak = max(abs(tyres.mf_fy(LAT, 4000, math.radians(a / 4), 4000)) for a in range(60))
        self.assertAlmostEqual(peak, 1.1 * 4000, delta=4400 * 0.01)
        self.assertEqual(tyres.mf_fy(LAT, 4000, 0.0, 4000), 0.0)
        k = abs(tyres.mf_fy(LAT, 4000, 1e-5, 4000)) / 1e-5
        self.assertAlmostEqual(k, 18.0 * 4000 * math.sin(2 * math.atan(4000 / (2.0 * 4000))), delta=k * 0.01)
        self.assertAlmostEqual(max(abs(tyres.mf_fx(LON, 4000, i / 200, 4000)) for i in range(100)), 1.2 * 4000, delta=48)
        # load sensitivity: the friction coefficient falls with load (pDy2 < 0)
        c = tyres.pacejka_curves(tyres.svj_tyre(SVJ, "FL"))
        self.assertGreater(c["mu_y"][0][1], c["mu_y"][-1][1])

    def test_beamng_load_coefficient(self):
        p = {"frictionCoef": 1.0, "noLoadCoef": 1.5, "fullLoadCoef": 0.6, "loadSensitivitySlope": 0.0002, "numRays": 16, "radius": 0.3}
        n = tyres.contact_nodes(p)
        self.assertAlmostEqual(tyres.bng_coef(p, 3000, n), 1.5 - 0.0002 * 3000 / n)
        self.assertEqual(tyres.bng_coef(p, 1e6, n), 0.6)                         # down to the full-load floor

    def test_fit_keeps_the_reference_and_matches_the_shape(self):
        p = {"frictionCoef": 1.0, "noLoadCoef": 2.0, "fullLoadCoef": 0.6, "loadSensitivitySlope": 0.0003, "numRays": 16, "radius": 0.3}
        b = json.loads(tyres.benchmark_json(json.dumps(SVJ), json.dumps(p), "FL"))
        vals, err = b["fit"]["values"], b["fit"]["rms"]
        self.assertLess(err, 0.01)
        q = dict(p, **vals)
        self.assertAlmostEqual(tyres.bng_coef(q, 4000), tyres.bng_coef(p, 4000), places=3)   # the grip at the reference load kept
        self.assertEqual(b["svj"]["rim_in"], 16)


if __name__ == "__main__":
    unittest.main()
