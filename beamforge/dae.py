"""Minimal COLLADA (.dae) writer for meshes the game will use as flexbodies.

Pure Python, standard library only (runs in Pyodide). BeamNG vehicles take their meshes from COLLADA
files (every vanilla vehicle ships .dae, with a compiled .cdae the game makes itself), so the SVJ's
glTF meshes are written as COLLADA for the game. The layout follows the game's own files: COLLADA
1.4.1, metres, Z up, positions already in the vehicle's frame (BeamNG axes: x left, y rear, z up),
one <node> per mesh with an identity matrix and the mesh's name (the name a flexbody row asks for),
and its materials, named so a materials.json entry can map onto each (one per triangle group, with
texture coordinates when the mesh has them).
"""

import math
from xml.sax.saxutils import escape


def _normals(pos, idx):
    """Per-vertex normals, the area-weighted mean of the faces around each vertex."""
    n = [[0.0, 0.0, 0.0] for _ in pos]
    for t in range(0, len(idx) - 2, 3):
        a, b, c = idx[t], idx[t + 1], idx[t + 2]
        pa, pb, pc = pos[a], pos[b], pos[c]
        u = [pb[i] - pa[i] for i in range(3)]
        v = [pc[i] - pa[i] for i in range(3)]
        f = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
        for k in (a, b, c):
            n[k] = [n[k][i] + f[i] for i in range(3)]
    out = []
    for v in n:
        length = math.sqrt(sum(x * x for x in v)) or 1.0
        out.append([x / length for x in v])
    return out


def _floats(rows, digits=5):
    return " ".join(f"{x:.{digits}f}".rstrip("0").rstrip(".") if x else "0" for r in rows for x in r)


def write(meshes):
    """COLLADA text for meshes: [{"name", "material", "positions": [[x, y, z]], "indices": [...],
    "colour"?: [r, g, b], "uvs"?: [[u, v]] (glTF's, v down), "submeshes"?: [{"material", "indices"}]}].
    Names are the node, geometry and flexbody names; materials are named as given (colour: the
    diffuse colour of the material's effect, 0-1). submeshes: the triangles split by material (else one
    group, "material" with "indices")."""
    effects, materials, geometries, nodes = [], [], [], []
    seen = set()
    for i, m in enumerate(meshes):
        name = m["name"]
        groups = m.get("submeshes") or [{"material": m.get("material") or f"{name}_mat", "indices": m["indices"]}]
        gid, nid = f"g{i}", escape(name, {'"': "&quot;"})
        pos = m["positions"]
        idx = [v for g in groups for v in g["indices"]]
        nor = _normals(pos, idx)
        uvs = m.get("uvs")
        for g in groups:
            mat = g["material"]
            if mat in seen:
                continue
            seen.add(mat)
            r, gr, b = (m.get("colour") or (0.75, 0.77, 0.8))[:3]
            effects.append(f'<effect id="{mat}-fx"><profile_COMMON><technique sid="common"><lambert>'
                           f'<diffuse><color sid="diffuse">{r} {gr} {b} 1</color></diffuse></lambert></technique>'
                           f'</profile_COMMON></effect>')
            materials.append(f'<material id="{mat}" name="{mat}"><instance_effect url="#{mat}-fx"/></material>')
        uv_src = (f'<source id="{gid}-uv"><float_array id="{gid}-uv-a" count="{2 * len(uvs)}">'
                  f'{_floats([[u, 1.0 - v] for u, v in uvs], 5)}</float_array>'
                  f'<technique_common><accessor source="#{gid}-uv-a" count="{len(uvs)}" stride="2">'
                  f'<param name="S" type="float"/><param name="T" type="float"/></accessor></technique_common></source>'
                  if uvs else "")
        uv_in = f'<input semantic="TEXCOORD" source="#{gid}-uv" offset="0" set="0"/>' if uvs else ""
        tris = "".join(
            f'<triangles material="{g["material"]}" count="{len(g["indices"]) // 3}">'
            f'<input semantic="VERTEX" source="#{gid}-v" offset="0"/><input semantic="NORMAL" source="#{gid}-nor" offset="0"/>{uv_in}'
            f'<p>{" ".join(str(v) for v in g["indices"][:len(g["indices"]) - len(g["indices"]) % 3])}</p></triangles>'
            for g in groups if len(g["indices"]) >= 3)
        binds = "".join(f'<instance_material symbol="{g["material"]}" target="#{g["material"]}">'
                        + ('<bind_vertex_input semantic="UVSET0" input_semantic="TEXCOORD" input_set="0"/>' if uvs else "")
                        + '</instance_material>' for g in groups if len(g["indices"]) >= 3)
        geometries.append(
            f'<geometry id="{gid}" name="{nid}"><mesh>'
            f'<source id="{gid}-pos"><float_array id="{gid}-pos-a" count="{3 * len(pos)}">{_floats(pos)}</float_array>'
            f'<technique_common><accessor source="#{gid}-pos-a" count="{len(pos)}" stride="3">'
            f'<param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>'
            f'<source id="{gid}-nor"><float_array id="{gid}-nor-a" count="{3 * len(nor)}">{_floats(nor, 4)}</float_array>'
            f'<technique_common><accessor source="#{gid}-nor-a" count="{len(nor)}" stride="3">'
            f'<param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>'
            f'{uv_src}'
            f'<vertices id="{gid}-v"><input semantic="POSITION" source="#{gid}-pos"/></vertices>'
            f'{tris}</mesh></geometry>')
        nodes.append(f'<node id="n{i}" name="{nid}" type="NODE"><matrix sid="transform">1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1</matrix>'
                     f'<instance_geometry url="#{gid}" name="{nid}"><bind_material><technique_common>'
                     f'{binds}</technique_common></bind_material>'
                     f'</instance_geometry></node>')
    nl = "\n"
    return (f'<?xml version="1.0" encoding="utf-8"?>{nl}'
            f'<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">{nl}'
            f'<asset><contributor><authoring_tool>BeamForge</authoring_tool></contributor>'
            f'<unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>{nl}'
            f'<library_effects>{"".join(effects)}</library_effects>{nl}'
            f'<library_materials>{"".join(materials)}</library_materials>{nl}'
            f'<library_geometries>{nl.join(geometries)}</library_geometries>{nl}'
            f'<library_visual_scenes><visual_scene id="scene" name="scene">{nl.join(nodes)}</visual_scene></library_visual_scenes>{nl}'
            f'<scene><instance_visual_scene url="#scene"/></scene>{nl}</COLLADA>{nl}')
