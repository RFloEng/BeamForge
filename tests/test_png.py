"""png.alpha: the alpha channel of a colour texture as a grey opacity map."""

import struct
import sys
import unittest
import zlib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from beamforge import png  # noqa: E402


def rgba_png(rows, ctype=6, filt=0):
    """A PNG from rows of pixels ((r, g, b, a) tuples, or (grey, a) for ctype 4), written with one filter type for every row."""
    h, w, bpp = len(rows), len(rows[0]), 4 if ctype == 6 else 2
    raw, prev = b"", bytes(w * bpp)
    for r in rows:
        line = bytes(c for px in r for c in px)
        if filt == 1:
            out = bytes((line[i] - (line[i - bpp] if i >= bpp else 0)) & 255 for i in range(len(line)))
        elif filt == 2:
            out = bytes((line[i] - prev[i]) & 255 for i in range(len(line)))
        elif filt == 4:
            out = bytes((line[i] - png._paeth(line[i - bpp] if i >= bpp else 0, prev[i], prev[i - bpp] if i >= bpp else 0)) & 255 for i in range(len(line)))
        else:
            out = line
        raw += bytes([filt]) + out
        prev = line
    return (png.SIGNATURE + png._chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, ctype, 0, 0, 0)) + png._chunk(b"IDAT", zlib.compress(raw))
            + png._chunk(b"IEND", b""))


ROWS = [[(10, 20, 30, 255), (40, 50, 60, 128)], [(70, 80, 90, 0), (100, 110, 120, 64)]]


class TestAlpha(unittest.TestCase):
    def test_alpha_of_an_rgba_png_for_every_filter(self):
        for filt in (0, 1, 2, 4):
            grey, mean, least = png.alpha(rgba_png(ROWS, 6, filt))
            self.assertAlmostEqual(mean, (255 + 128 + 0 + 64) / (4 * 255), places=4, msg=f"filter {filt}")
            self.assertEqual(least, 0.0)
            self.assertTrue(grey.startswith(png.SIGNATURE))

    def test_the_grey_png_is_the_alpha(self):
        grey, _, _ = png.alpha(rgba_png(ROWS))
        for kind, body in png._chunks(grey):
            if kind == b"IHDR":
                self.assertEqual(struct.unpack(">IIBB", body[:10]), (2, 2, 8, 0))     # 2 x 2, 8-bit grey
            if kind == b"IDAT":
                raw = zlib.decompress(body)
        self.assertEqual(list(raw), [0, 255, 128, 0, 0, 64])                          # (filter, grey...) per row: the alpha values

    def test_grey_with_alpha(self):
        r = png.alpha(rgba_png([[(200, 255), (200, 100)]], ctype=4))
        self.assertAlmostEqual(r[1], (255 + 100) / (2 * 255), places=4)

    def test_no_alpha_to_take(self):
        self.assertIsNone(png.alpha(b"not a png"))
        rgb = png.SIGNATURE + png._chunk(b"IHDR", struct.pack(">IIBBBBB", 1, 1, 8, 2, 0, 0, 0)) + png._chunk(b"IDAT", zlib.compress(b"\x00\x01\x02\x03")) + png._chunk(b"IEND", b"")
        self.assertIsNone(png.alpha(rgb))                     # no alpha channel
        self.assertIsNone(png.alpha(b"\xff\xd8\xff\xe0jpeg"))   # a JPEG


if __name__ == "__main__":
    unittest.main()
