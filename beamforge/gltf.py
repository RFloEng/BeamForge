"""Minimal glTF 2.0 binary (.glb) writer and reader for SVJ meshes (SVJ §22).

Pure Python, standard library only (runs in Pyodide). Used by beamforge.svj to
list the node names of imported meshes so their SVJ visual bindings can be checked,
and to write simple meshes (tubes, triangles) into SVJ bundles.

Frames. Callers pass points in SAE J670 (X forward, Y right, Z down, metres; convert
from BeamNG X left / Y rear / Z up with beamforge.svj.to_sae). The glTF asset uses the
convention SVJ documents for Blender exports (§22.4): Y up, -Z forward, right-handed,
metres. From SAE J670:
    glTF x = SAE Y (right),  glTF y = -SAE Z (up),  glTF z = -SAE X (backward)

Each file has one scene, one node (named for its SVJ binding, e.g. SVJ::body::chassis),
one mesh and one material. The numeric constants are fixed by the glTF 2.0 spec:
0x46546C67 "glTF" magic, chunk types 0x4E4F534A "JSON" and 0x004E4942 "BIN",
bufferView targets 34962 ARRAY_BUFFER / 34963 ELEMENT_ARRAY_BUFFER, accessor component
types 5126 FLOAT / 5125 UNSIGNED_INT. Chunks are padded to 4 bytes (JSON with spaces).
"""

import json
import math
import re
import struct


def sae_to_gltf(p):
    """SAE J670 point [X, Y, Z] (m) -> glTF [x, y, z] (m), Y up, -Z forward."""
    return [p[1], -p[2], -p[0]]


def _tube(a, b, r, seg=10):
    """Vertices, normals and triangle indices of an open cylinder from a to b.

    a, b: end points (glTF m); r: radius (m); seg: sides. No end caps.
    """
    d = [b[i] - a[i] for i in range(3)]
    L = math.sqrt(sum(c * c for c in d)) or 1.0
    e = [c / L for c in d]
    ref = [0.0, 1.0, 0.0] if abs(e[1]) < 0.9 else [1.0, 0.0, 0.0]
    u = [e[1] * ref[2] - e[2] * ref[1], e[2] * ref[0] - e[0] * ref[2], e[0] * ref[1] - e[1] * ref[0]]
    un = math.sqrt(sum(c * c for c in u))
    u = [c / un for c in u]
    v = [e[1] * u[2] - e[2] * u[1], e[2] * u[0] - e[0] * u[2], e[0] * u[1] - e[1] * u[0]]
    pos, nrm, idx = [], [], []
    for k in range(seg):
        t = 2 * math.pi * k / seg
        n = [math.cos(t) * u[i] + math.sin(t) * v[i] for i in range(3)]
        for end in (a, b):
            pos.append([end[i] + r * n[i] for i in range(3)])
            nrm.append(n)
    for k in range(seg):
        i0, i1 = 2 * k, 2 * ((k + 1) % seg)
        idx += [i0, i1, i0 + 1, i1, i1 + 1, i0 + 1]
    return pos, nrm, idx


def tubes_glb(tubes, node_name="SVJ::body::chassis", colour=(0.45, 0.5, 0.56)):
    """One glTF node holding every tube. tubes: [(a_sae, b_sae, radius_m), ...].

    node_name: glTF node (and mesh) name, which the SVJ visual binding refers to;
    colour: linear RGB 0-1 of the steel material. Returns the .glb bytes.
    """
    P, N, I = [], [], []
    for a, b, r in tubes:
        pos, nrm, idx = _tube(sae_to_gltf(a), sae_to_gltf(b), r)
        base = len(P)
        P += pos
        N += nrm
        I += [base + i for i in idx]
    pb = b"".join(struct.pack("<3f", *p) for p in P)
    nb = b"".join(struct.pack("<3f", *n) for n in N)
    ib = b"".join(struct.pack("<I", i) for i in I)
    binary = pb + nb + ib
    binary += b"\0" * ((4 - len(binary) % 4) % 4)
    mins = [min(p[i] for p in P) for i in range(3)] if P else [0, 0, 0]
    maxs = [max(p[i] for p in P) for i in range(3)] if P else [0, 0, 0]
    doc = {
        "asset": {"version": "2.0", "generator": "BeamForge"},
        "scene": 0, "scenes": [{"nodes": [0]}],
        "nodes": [{"name": node_name, "mesh": 0}],
        "meshes": [{"name": node_name, "primitives": [{"attributes": {"POSITION": 0, "NORMAL": 1}, "indices": 2, "material": 0}]}],
        "materials": [{"name": "steel", "pbrMetallicRoughness": {"baseColorFactor": [*colour, 1.0],
                                                                        "metallicFactor": 0.6, "roughnessFactor": 0.45},
                       "doubleSided": True}],
        "buffers": [{"byteLength": len(binary)}],
        "bufferViews": [{"buffer": 0, "byteOffset": 0, "byteLength": len(pb), "target": 34962},
                        {"buffer": 0, "byteOffset": len(pb), "byteLength": len(nb), "target": 34962},
                        {"buffer": 0, "byteOffset": len(pb) + len(nb), "byteLength": len(ib), "target": 34963}],
        "accessors": [{"bufferView": 0, "componentType": 5126, "count": len(P), "type": "VEC3", "min": mins, "max": maxs},
                      {"bufferView": 1, "componentType": 5126, "count": len(N), "type": "VEC3"},
                      {"bufferView": 2, "componentType": 5125, "count": len(I), "type": "SCALAR"}],
    }
    js = json.dumps(doc, separators=(",", ":")).encode()
    js += b" " * ((4 - len(js) % 4) % 4)
    total = 12 + 8 + len(js) + 8 + len(binary)
    return (struct.pack("<III", 0x46546C67, 2, total) + struct.pack("<II", len(js), 0x4E4F534A) + js
            + struct.pack("<II", len(binary), 0x004E4942) + binary)


def triangles_glb(tris, node_name, colour=(0.8, 0.82, 0.86), opacity=1.0):
    """One glTF node with flat-shaded triangles. tris: [(p1, p2, p3), ...] in SAE J670 metres.

    Non-indexed, one normal per face; opacity < 1 turns on alpha blending.
    Returns the .glb bytes (used for the body shell).
    """
    P, N = [], []
    for t in tris:
        a, b, c = (sae_to_gltf(p) for p in t)
        u = [b[i] - a[i] for i in range(3)]
        v = [c[i] - a[i] for i in range(3)]
        n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
        L = math.sqrt(sum(x * x for x in n)) or 1.0
        n = [x / L for x in n]
        P += [a, b, c]
        N += [n, n, n]
    pb = b"".join(struct.pack("<3f", *p) for p in P)
    nb = b"".join(struct.pack("<3f", *n) for n in N)
    binary = pb + nb                     # 12 bytes per vertex: already 4-byte aligned
    mins = [min(p[i] for p in P) for i in range(3)] if P else [0, 0, 0]
    maxs = [max(p[i] for p in P) for i in range(3)] if P else [0, 0, 0]
    doc = {
        "asset": {"version": "2.0", "generator": "BeamForge"},
        "scene": 0, "scenes": [{"nodes": [0]}],
        "nodes": [{"name": node_name, "mesh": 0}],
        "meshes": [{"name": node_name, "primitives": [{"attributes": {"POSITION": 0, "NORMAL": 1}, "material": 0}]}],
        "materials": [{"name": "panel", "doubleSided": True,
                       "alphaMode": "BLEND" if opacity < 1 else "OPAQUE",
                       "pbrMetallicRoughness": {"baseColorFactor": [*colour, opacity], "metallicFactor": 0.2, "roughnessFactor": 0.6}}],
        "buffers": [{"byteLength": len(binary)}],
        "bufferViews": [{"buffer": 0, "byteOffset": 0, "byteLength": len(pb), "target": 34962},
                        {"buffer": 0, "byteOffset": len(pb), "byteLength": len(nb), "target": 34962}],
        "accessors": [{"bufferView": 0, "componentType": 5126, "count": len(P), "type": "VEC3", "min": mins, "max": maxs},
                      {"bufferView": 1, "componentType": 5126, "count": len(N), "type": "VEC3"}],
    }
    js = json.dumps(doc, separators=(",", ":")).encode()
    js += b" " * ((4 - len(js) % 4) % 4)
    total = 12 + 8 + len(js) + 8 + len(binary)
    return (struct.pack("<III", 0x46546C67, 2, total) + struct.pack("<II", len(js), 0x4E4F534A) + js
            + struct.pack("<II", len(binary), 0x004E4942) + binary)


def glb_json(data):
    """The JSON chunk of a .glb (node names, meshes); raises ValueError if it is not a glb."""
    magic, version, _ = struct.unpack_from("<III", data, 0)
    if magic != 0x46546C67 or version != 2:
        raise ValueError("not a glTF 2.0 binary")
    length, kind = struct.unpack_from("<II", data, 12)
    if kind != 0x4E4F534A:
        raise ValueError("first glb chunk is not JSON")
    return json.loads(data[20:20 + length].decode("utf-8"))


def node_names(data, is_glb=True):
    """Node names of a .glb (is_glb) or a .gltf JSON file, given as bytes."""
    doc = glb_json(data) if is_glb else json.loads(data.decode("utf-8"))
    return [n.get("name", "") for n in doc.get("nodes", [])]


# ---------------------------------------------------------------- vertex positions (reading)

def _matrix(node):
    """A glTF node's local transform as a column-major 4x4 list (matrix, or translation / rotation / scale)."""
    if "matrix" in node:
        return [float(x) for x in node["matrix"]]
    t = node.get("translation", [0, 0, 0])
    x, y, z, w = node.get("rotation", [0, 0, 0, 1])
    s = node.get("scale", [1, 1, 1])
    r = [1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w),
         2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w),
         2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)]
    return [r[0] * s[0], r[1] * s[0], r[2] * s[0], 0, r[3] * s[1], r[4] * s[1], r[5] * s[1], 0,
            r[6] * s[2], r[7] * s[2], r[8] * s[2], 0, t[0], t[1], t[2], 1]


def _mul(a, b):
    """Column-major 4x4 product a * b."""
    return [sum(a[k * 4 + r] * b[c * 4 + k] for k in range(4)) for c in range(4) for r in range(4)]


def _document(data, is_glb, buffers):
    """(the glTF JSON, [bytes of each buffer]) of a .glb or .gltf; buffers: {uri: bytes} for separate files."""
    import base64
    if is_glb:
        doc = glb_json(data)
        jlen = struct.unpack_from("<I", data, 12)[0]
        off = 20 + jlen
        blen = struct.unpack_from("<I", data, off)[0] if len(data) > off + 8 else 0
        glb_bin = data[off + 8:off + 8 + blen]
    else:
        doc, glb_bin = json.loads(data.decode("utf-8")), b""
    bufs = []
    for b in doc.get("buffers", []):
        uri = b.get("uri")
        if uri is None:
            bufs.append(glb_bin)
        elif uri.startswith("data:"):
            bufs.append(base64.b64decode(uri.split(",", 1)[1]))
        else:
            bufs.append((buffers or {}).get(uri, b""))
    return doc, bufs


_COMPONENT = {5121: ("B", 1), 5123: ("H", 2), 5125: ("I", 4), 5126: ("f", 4)}


def _read(doc, bufs, ai, width):
    """An accessor's values as tuples of `width` numbers (unsigned byte / short / int, or float)."""
    acc = doc["accessors"][ai]
    if "bufferView" not in acc or acc.get("componentType") not in _COMPONENT:
        return []
    code, size = _COMPONENT[acc["componentType"]]
    bv = doc["bufferViews"][acc["bufferView"]]
    buf = bufs[bv["buffer"]]
    start = bv.get("byteOffset", 0) + acc.get("byteOffset", 0)
    stride = bv.get("byteStride") or size * width
    fmt = "<" + code * width
    return [struct.unpack_from(fmt, buf, start + i * stride) for i in range(acc["count"])]


# what is not the body when a file has no body node (Assetto Corsa's names, as converters keep them):
# the running gear (the base's stays), the high-detail cockpit (the low one stays), broken glass,
# cameras and other helper nodes
NOT_BODY = re.compile(r"^(wheel|tyre|tire|rim|disc|disk|brake|caliper|susp|flycam|damage_|cockpit_hr|steer_hr|"
                      r"shift_hr|driver|helmet|ext_|camera)|_hr$", re.I)


def _meshes(doc, under, skip=None):
    """(mesh index, world matrix) of every mesh node at or below the node named `under` (None: all; a
    name no node has: none). skip: a regex of node names whose subtrees are left out."""
    nodes = doc.get("nodes", [])
    out = []

    def walk(ni, parent, on):
        node = nodes[ni]
        if skip is not None and skip.search(node.get("name") or ""):
            return
        m = _mul(parent, _matrix(node))
        on = on or under is None or node.get("name") == under
        if on and "mesh" in node:
            out.append((node["mesh"], m))
        for c in node.get("children", []):
            walk(c, m, on)
    scenes = doc.get("scenes") or [{"nodes": list(range(len(nodes)))}]
    ident = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
    for ni in scenes[doc.get("scene", 0)].get("nodes", []):
        walk(ni, ident, False)
    return out


def _apply(m, p):
    x, y, z = p
    return [m[0] * x + m[4] * y + m[8] * z + m[12], m[1] * x + m[5] * y + m[9] * z + m[13],
            m[2] * x + m[6] * y + m[10] * z + m[14]]


def has_node(data, name, is_glb=True):
    """Whether a node of the file has this name."""
    doc, _ = _document(data, is_glb, None)
    return any(n.get("name") == name for n in doc.get("nodes", []))


def positions(data, is_glb=True, under=None, buffers=None, limit=60000, skip=None):
    """World-space vertex positions of the meshes in a .glb (or .gltf JSON) file, in the file's own axes.

    under: a node name; only meshes at or below that node are read (None: every node of the default
    scene). buffers: {uri: bytes} for a .gltf whose buffers are separate files (data: URIs are read
    directly). At most `limit` points are returned (evenly thinned), enough for bounds and slices.
    Float VEC3 POSITION accessors only (the glTF 2.0 rule); sparse accessors are not read.
    """
    doc, bufs = _document(data, is_glb, buffers)
    out = []
    for mi, m in _meshes(doc, under, skip):
        for prim in doc["meshes"][mi].get("primitives", []):
            ai = prim.get("attributes", {}).get("POSITION")
            if ai is not None and doc["accessors"][ai].get("type") == "VEC3":
                out += [_apply(m, p) for p in _read(doc, bufs, ai, 3)]
    step = max(1, len(out) // limit)
    return out[::step]


def _read_norm(doc, bufs, ai, width):
    """_read, with normalized integer accessors (UVs stored as bytes or shorts) scaled to 0-1."""
    acc = doc["accessors"][ai]
    vals = _read(doc, bufs, ai, width)
    if acc.get("normalized") and acc.get("componentType") in (5121, 5123):
        k = 255.0 if acc["componentType"] == 5121 else 65535.0
        vals = [tuple(x / k for x in v) for v in vals]
    return vals


def textured(data, is_glb=True, under=None, buffers=None, skip=None):
    """The meshes at or below `under` with their materials, in world space and the file's own axes:
    (positions [[x, y, z]], uvs [[u, v]] (glTF's: v down; [0, 0] where a primitive has none),
    [(glTF material index or None, indices [i0, i1, i2, ...])], one entry per primitive)."""
    doc, bufs = _document(data, is_glb, buffers)
    pos, uvs, groups = [], [], []
    for mi, m in _meshes(doc, under, skip):
        for prim in doc["meshes"][mi].get("primitives", []):
            att = prim.get("attributes", {})
            ai = att.get("POSITION")
            if ai is None or prim.get("mode", 4) != 4 or doc["accessors"][ai].get("type") != "VEC3":
                continue
            base = len(pos)
            pts = _read(doc, bufs, ai, 3)
            pos += [_apply(m, p) for p in pts]
            uv = _read_norm(doc, bufs, att["TEXCOORD_0"], 2) if "TEXCOORD_0" in att else []
            uvs += [list(t) for t in uv] if len(uv) == len(pts) else [[0.0, 0.0]] * len(pts)
            if "indices" in prim:
                idx = [base + v[0] for v in _read(doc, bufs, prim["indices"], 1)]
            else:
                idx = [base + i for i in range(len(pts) - len(pts) % 3)]
            groups.append((prim.get("material"), idx))
    return pos, uvs, groups


def materials(data, is_glb=True, buffers=None):
    """The file's materials and images: ([{"name", "colour" [r, g, b, a], "texture" (image index or
    None), "normal" (image index or None), "alpha" ("OPAQUE" | "MASK" | "BLEND"), "cutoff",
    "double_sided", "metallic", "roughness"}], [(bytes, mime type)])."""
    doc, bufs = _document(data, is_glb, buffers)
    tex = doc.get("textures", [])

    def image(ref):
        if not isinstance(ref, dict) or ref.get("index") is None or ref["index"] >= len(tex):
            return None
        return tex[ref["index"]].get("source")
    mats = []
    for i, m in enumerate(doc.get("materials", [])):
        pbr = m.get("pbrMetallicRoughness") or {}
        mats.append({"name": m.get("name") or f"material{i}", "colour": pbr.get("baseColorFactor", [1, 1, 1, 1]),
                     "texture": image(pbr.get("baseColorTexture")), "normal": image(m.get("normalTexture")),
                     "alpha": m.get("alphaMode", "OPAQUE"), "cutoff": m.get("alphaCutoff", 0.5),
                     "double_sided": bool(m.get("doubleSided")), "metallic": pbr.get("metallicFactor", 1.0),
                     "roughness": pbr.get("roughnessFactor", 1.0)})
    imgs = []
    for im in doc.get("images", []):
        raw = b""
        if "bufferView" in im:
            bv = doc["bufferViews"][im["bufferView"]]
            raw = bufs[bv["buffer"]][bv.get("byteOffset", 0):bv.get("byteOffset", 0) + bv["byteLength"]]
        elif str(im.get("uri", "")).startswith("data:"):
            import base64
            raw = base64.b64decode(im["uri"].split(",", 1)[1])
        elif im.get("uri"):
            raw = (buffers or {}).get(im["uri"], b"")
        mime = im.get("mimeType") or ("image/png" if raw[1:4] == b"PNG" else "image/jpeg" if raw[:2] == bytes([0xFF, 0xD8]) else "")
        imgs.append((bytes(raw), mime))
    return mats, imgs


def triangles(data, is_glb=True, under=None, buffers=None, skip=None):
    """The triangles of the meshes at or below `under`, in world space and the file's own axes, as one
    mesh: (positions [[x, y, z]], indices [i0, i1, i2, ...]). Triangle primitives only (mode 4, the
    default); a primitive without indices takes its vertices in order."""
    doc, bufs = _document(data, is_glb, buffers)
    pos, idx = [], []
    for mi, m in _meshes(doc, under, skip):
        for prim in doc["meshes"][mi].get("primitives", []):
            ai = prim.get("attributes", {}).get("POSITION")
            if ai is None or prim.get("mode", 4) != 4 or doc["accessors"][ai].get("type") != "VEC3":
                continue
            base = len(pos)
            pts = _read(doc, bufs, ai, 3)
            pos += [_apply(m, p) for p in pts]
            if "indices" in prim:
                idx += [base + v[0] for v in _read(doc, bufs, prim["indices"], 1)]
            else:
                idx += [base + i for i in range(len(pts) - len(pts) % 3)]
    return pos, idx
