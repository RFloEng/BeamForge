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
