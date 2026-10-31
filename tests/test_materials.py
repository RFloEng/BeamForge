"""export.svj_materials: how glTF alpha modes become BeamNG materials (the game reads opacity from an opacityMap, not the colour alpha)."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from beamforge import export  # noqa: E402
from test_png import rgba_png  # noqa: E402


def mat(name, alpha, pixels, cutoff=0.5):
    img = (rgba_png([[px for px in row] for row in pixels]), "image/png")
    return {"name": name, "colour": [1, 1, 1, 1], "metallic": 0.0, "roughness": 0.5, "image": img, "image_id": (0, 1), "normal_image": None,
            "normal_id": (0, None), "alpha": alpha, "cutoff": cutoff, "double_sided": False, "texture": 1}


def build(m):
    svjm = [{"name": "m", "material": m["name"], "materials": {m["name"]: m}, "submeshes": [{"material": m["name"], "indices": []}]}]
    mats, images = export.svj_materials(svjm, "bf_x")
    return mats[m["name"]], images


CLEAR = [[(10, 10, 10, 0), (10, 10, 10, 0)]]                 # a texture with alpha 0 all over (an AC window)
GRILLE = [[(10, 10, 10, 255), (10, 10, 10, 0)]]               # half cut out
TINT = [[(10, 10, 10, 112), (10, 10, 10, 112)]]               # alpha 0.44 all over


class TestAlphaModes(unittest.TestCase):
    def test_clear_cutout_glass_becomes_a_light_tint(self):
        entry, images = build(mat("clearglass", "MASK", CLEAR))
        self.assertTrue(entry["translucent"])
        self.assertNotIn("alphaTest", entry)
        self.assertEqual(entry["Stages"][0]["opacityFactor"], export.GLASS_TINT)

    def test_alpha_tested_material_gets_an_opacity_map(self):
        entry, images = build(mat("grille", "MASK", GRILLE))
        self.assertTrue(entry["alphaTest"])
        self.assertTrue(entry["Stages"][0]["opacityMap"].endswith("_op.png"))
        self.assertIn("vehicles/bf_x/svj_textures/t0_1_op.png", images)

    def test_glass_marked_opaque_but_with_a_see_through_texture_is_glass(self):
        entry, _ = build(mat("orangeglass", "OPAQUE", TINT))
        self.assertTrue(entry["translucent"])
        self.assertTrue(entry["Stages"][0]["opacityMap"].endswith("_op.png"))

    def test_a_solid_material_is_left_solid(self):
        entry, _ = build(mat("paint", "OPAQUE", TINT))
        self.assertNotIn("translucent", entry)
        self.assertNotIn("opacityMap", entry["Stages"][0])

    def test_lights_are_not_glass(self):
        entry, _ = build(mat("headlight_lens", "OPAQUE", TINT))
        self.assertNotIn("translucent", entry)


if __name__ == "__main__":
    unittest.main()
