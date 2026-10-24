"""The alpha channel of a PNG as a grey PNG: the game's translucent materials take their opacity from a separate map (opacityMap).

Pure Python, standard library only (zlib). A glTF glass material keeps its transparency in the alpha of its colour texture; BeamNG's
version 1.5 materials read opacity from an opacityMap (or an opacityFactor), so the alpha is written out as a grey image.
Only 8-bit, non-interlaced RGBA and grey+alpha PNGs carry alpha this way; any other PNG (or a JPEG) has no alpha to take.
"""

import struct
import zlib

SIGNATURE = b"\x89PNG\r\n\x1a\n"


def _chunks(data):
    pos = len(SIGNATURE)
    while pos + 8 <= len(data):
        n, kind = struct.unpack(">I4s", data[pos:pos + 8])
        yield kind, data[pos + 8:pos + 8 + n]
        pos += 12 + n


def _chunk(kind, body):
    return struct.pack(">I", len(body)) + kind + body + struct.pack(">I", zlib.crc32(kind + body) & 0xFFFFFFFF)


def grey_png(width, height, rows):
    """An 8-bit grey PNG from `rows` (a bytes-like per row)."""
    raw = b"".join(b"\x00" + bytes(r) for r in rows)
    return (SIGNATURE + _chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)) + _chunk(b"IDAT", zlib.compress(raw, 6))
            + _chunk(b"IEND", b""))


def _paeth(a, b, c):
    p = a + b - c
    pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
    return a if pa <= pb and pa <= pc else b if pb <= pc else c


def alpha(data):
    """(grey PNG of the alpha channel, mean alpha 0..1, least alpha 0..1) of an 8-bit non-interlaced RGBA or grey+alpha PNG, or None when
    the image has no alpha to take (another colour type, a palette, 16 bits, interlaced, not a PNG)."""
    if not data.startswith(SIGNATURE):
        return None
    width = height = depth = ctype = interlace = None
    idat = []
    try:
        for kind, body in _chunks(data):
            if kind == b"IHDR":
                width, height, depth, ctype, _, _, interlace = struct.unpack(">IIBBBBB", body)
            elif kind == b"IDAT":
                idat.append(body)
            elif kind == b"IEND":
                break
        if depth != 8 or ctype not in (4, 6) or interlace or not width or not height:
            return None
        bpp = 2 if ctype == 4 else 4
        stride = width * bpp
        raw = zlib.decompress(b"".join(idat))
        if len(raw) < height * (stride + 1):
            return None
    except (struct.error, zlib.error):
        return None
    prev = bytearray(stride)
    rows, total, least = [], 0, 255
    for y in range(height):
        o = y * (stride + 1)
        ft = raw[o]
        line = bytearray(raw[o + 1:o + 1 + stride])
        if ft == 1:
            for i in range(bpp, stride):
                line[i] = (line[i] + line[i - bpp]) & 255
        elif ft == 2:
            line = bytearray((a + b) & 255 for a, b in zip(line, prev))
        elif ft == 3:
            for i in range(stride):
                left = line[i - bpp] if i >= bpp else 0
                line[i] = (line[i] + ((left + prev[i]) >> 1)) & 255
        elif ft == 4:
            for i in range(stride):
                left = line[i - bpp] if i >= bpp else 0
                upleft = prev[i - bpp] if i >= bpp else 0
                line[i] = (line[i] + _paeth(left, prev[i], upleft)) & 255
        elif ft != 0:
            return None
        prev = line
        a = bytes(line[bpp - 1::bpp])
        rows.append(a)
        total += sum(a)
        least = min(least, min(a))
    return grey_png(width, height, rows), total / (255.0 * width * height), least / 255.0
