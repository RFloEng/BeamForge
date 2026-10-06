"""SVJ (Standard Vehicle JSON) reading for BeamForge: bundles, meshes, hardpoints, comparison.

SVJ: https://github.com/RFloEng/SVJ-standard-vehicle-json (spec v0.99.1).
Pure Python, standard library only (runs in Pyodide).

Frames
  BeamNG: X left, Y rear, Z up, metres; the origin is wherever the vehicle's jbeam puts it.
  SVJ (SAE J670): X forward, Y right, Z down; origin at the front-axle centre on the ground.
  An SVJ is placed on a BeamNG vehicle by its front axle line y_f and its ground height z_g
  (beamng.measure gives both as front_axle_y and ground_z):
      x = -Y_svj,   y = y_f - X_svj,   z = z_g - Z_svj
  glTF meshes (SVJ §22) use a third frame, Y up / -Z forward by default; see gltf.py and
  gltfToThree in editor/app.js.

What this module does now: read an .svj.json or a .zip bundle, find its glTF meshes and
check the visual bindings, list the suspension hardpoints in BeamNG coordinates, and compare
the file's headline values with a BeamNG base vehicle (the start of the per-parameter choice,
docs/roadmap.md step 3). It does not change the BeamNG vehicle yet.
"""
import json
import math
import re
import shutil
import zipfile
from pathlib import Path

from beamforge import gltf

SVJ_VERSION = "0.99.1"
DEFAULT_GLTF_AXES = {"up": "Y", "forward": "-Z"}      # SVJ §22.4, Blender export convention


# ---------------------------------------------------------------- frames

def from_sae(p, yf=0.0, zg=0.0):
    """SAE J670 point [X, Y, Z] (m) -> BeamNG [x, y, z] (m), front axle at y = yf, ground at z = zg."""
    return [-p[1], yf - p[0], zg - p[2]]


def to_sae(p, yf=0.0, zg=0.0):
    """BeamNG point [x, y, z] (m) -> SAE J670 [X, Y, Z] (m); the inverse of from_sae."""
    return [yf - p[1], -p[0], zg - p[2]]


def gltf_to_sae(p, axes=None):
    """glTF point -> SAE J670 [X, Y, Z] for the asset axes {"up", "forward"} (§22.4; see gltf_axes):
    X = p . forward, Y = p . (forward x up), Z = -p . up. The same frame as gltfToThree in the editor."""
    axes = axes or DEFAULT_GLTF_AXES

    def vec(t):
        s, a = (-1.0, t[1:]) if t.startswith("-") else (1.0, t)
        return [s if a == "X" else 0.0, s if a == "Y" else 0.0, s if a == "Z" else 0.0]
    u, f = vec(axes["up"]), vec(axes["forward"])
    r = [f[1] * u[2] - f[2] * u[1], f[2] * u[0] - f[0] * u[2], f[0] * u[1] - f[1] * u[0]]
    dot = lambda a, b: a[0] * b[0] + a[1] * b[1] + a[2] * b[2]  # noqa: E731
    return [dot(p, f), dot(p, r), -dot(p, u)]


def sae_to_gltf(p, axes=None):
    """SAE J670 vector -> glTF for the asset axes; the inverse of gltf_to_sae."""
    axes = axes or DEFAULT_GLTF_AXES

    def vec(t):
        s, a = (-1.0, t[1:]) if t.startswith("-") else (1.0, t)
        return [s if a == "X" else 0.0, s if a == "Y" else 0.0, s if a == "Z" else 0.0]
    u, f = vec(axes["up"]), vec(axes["forward"])
    r = [f[1] * u[2] - f[2] * u[1], f[2] * u[0] - f[0] * u[2], f[0] * u[1] - f[1] * u[0]]
    return [p[0] * f[i] + p[1] * r[i] - p[2] * u[i] for i in range(3)]


# a mesh's wheel node: WHEEL_LF, tyre_FL, TIRE RR... (side and axle in either order)
WHEEL_NODE = re.compile(r"^(?:wheel|tyre|tire)[ _]?(?:([LR])([FR])|([FR])([LR]))$", re.I)
OFFSET_MIN = 0.005      # m: a mesh closer than this to its wheels is left where it is
OFFSET_SPREAD = 0.05    # m: the wheels must agree on the offset within this, else it is not applied


def mesh_offset(svj, data, is_glb=True):
    """The SAE [dX, dY, dZ] that puts a mesh's wheels (its nodes named like WHEEL_LF) on the SVJ's
    wheel centres, and how many wheels agreed; (None, n) when the mesh has fewer than two wheel nodes,
    they disagree, or it already sits within OFFSET_MIN. Converters do not always put the mesh's origin
    where the SVJ's is (some leave it at the source game's origin)."""
    axes = gltf_axes(svj)
    centres = {h["corner"]: to_sae(h["pos"]) for h in hardpoints(svj) if h["name"] == "wheel_center"}
    seen, diffs = set(), []
    for name in gltf.node_names(data, is_glb=is_glb):
        m = WHEEL_NODE.match(name or "")
        if not m:
            continue
        side, axle = (m.group(1), m.group(2)) if m.group(1) else (m.group(4), m.group(3))
        corner = (axle + side).upper()
        if corner in seen or corner not in centres:
            continue
        pts = [gltf_to_sae(q, axes) for q in gltf.positions(data, is_glb=is_glb, under=name, limit=4000)]
        if not pts:
            continue
        seen.add(corner)
        mid = [(min(q[i] for q in pts) + max(q[i] for q in pts)) / 2 for i in range(3)]
        diffs.append([centres[corner][i] - mid[i] for i in range(3)])
    if len(diffs) < 2:
        return None, len(diffs)
    mean = [sum(d[i] for d in diffs) / len(diffs) for i in range(3)]
    if max(math.dist(d, mean) for d in diffs) > OFFSET_SPREAD or math.hypot(*mean) < OFFSET_MIN:
        return None, len(diffs)
    return [round(x, 4) for x in mean], len(diffs)


# ---------------------------------------------------------------- reading

def visual_bindings(svj):
    """Every visual binding in an SVJ document: [{"path", "mesh_ref", "node", "uri"}].

    Looks at chassis, suspension corners, mass_bodies and aerodynamics components. path is the
    SVJ location, node the glTF node name (SVJ::<category>::<id>); a binding without mesh_ref
    uses the only mesh asset if there is one. uri is None when mesh_ref is not in assets.meshes.
    An SVJ with mesh assets but no chassis binding gets one implied ("implied": True, node None): its
    first mesh is the body (the whole mesh but the running gear; gltf.NOT_BODY).
    """
    uris = {a["id"]: a["uri"] for a in (svj.get("assets") or {}).get("meshes", [])}
    out = []

    def add(path, vis):
        if isinstance(vis, dict) and "node" in vis:
            ref = vis.get("mesh_ref") or (next(iter(uris)) if len(uris) == 1 else None)
            out.append({"path": path, "mesh_ref": ref, "node": vis["node"], "uri": uris.get(ref)})

    add("chassis", (svj.get("chassis") or {}).get("visual"))
    for name, c in (svj.get("suspension") or {}).items():
        if isinstance(c, dict):
            add(f"suspension.{name}", c.get("visual"))
    for b in (svj.get("chassis") or {}).get("mass_bodies", []) or []:
        add(f"mass_bodies.{b.get('id')}", b.get("visual"))
    comps = (svj.get("aerodynamics") or {}).get("components")
    if isinstance(comps, dict):
        for k, a in comps.items():
            add(f"aerodynamics.{k}", a.get("visual") if isinstance(a, dict) else None)
    elif isinstance(comps, list):
        for a in comps:
            if isinstance(a, dict):
                add(f"aerodynamics.{a.get('id')}", a.get("visual"))
    if uris and not any(b["path"] == "chassis" for b in out):
        first = next(iter(uris))
        out.insert(0, {"path": "chassis", "mesh_ref": first, "node": None, "uri": uris[first], "implied": True})
    return out


def gltf_axes(svj):
    """Axis convention of the glTF assets: _metadata.coordinate_system if it is an object (§22.4).

    Returns {"up": axis, "forward": axis} with axes like "Y" or "-Z"; the default is Y up, -Z forward.
    """
    cs = (svj.get("_metadata") or {}).get("coordinate_system")
    return {"up": cs["up"], "forward": cs["forward"]} if isinstance(cs, dict) else dict(DEFAULT_GLTF_AXES)


def summary(svj):
    """Headline values of an SVJ: {"vehicle", "version", "wheelbase", "track_front", "track_rear",
    "mass_total", "corners": {key: system_type}} (None where the file has no value)."""
    ch = svj.get("chassis") or {}
    vi = svj.get("vehicle_info") or {}
    corners = {}
    for k, c in (svj.get("suspension") or {}).items():
        if isinstance(c, dict) and "topology" in c:
            corners[k] = (c["topology"] or {}).get("system_type")
    return {"vehicle": " ".join(str(vi.get(k, "")) for k in ("make", "model", "variant")).strip(),
            "version": (svj.get("_metadata") or {}).get("version"),
            "wheelbase": ch.get("wheelbase"), "track_front": ch.get("track_front"), "track_rear": ch.get("track_rear"),
            "mass_total": ch.get("mass_total"), "corners": corners}


def hardpoints(svj, yf=0.0, zg=0.0):
    """Suspension points of every corner in BeamNG coordinates.

    Returns [{"corner", "name", "kind" ("upright" or "chassis"), "pos" [x, y, z]}]: the upright
    hardpoints, and the inboard points of every link (named <link>.<index>), placed with from_sae.
    """
    out = []
    for corner, c in (svj.get("suspension") or {}).items():
        topo = c.get("topology") if isinstance(c, dict) else None
        if not isinstance(topo, dict):
            continue
        for name, p in ((topo.get("upright") or {}).get("hardpoints") or {}).items():
            if isinstance(p, list) and len(p) == 3:
                out.append({"corner": corner, "name": name, "kind": "upright", "pos": from_sae(p, yf, zg)})
        for link in topo.get("links") or []:
            for i, p in enumerate(link.get("inboard_points") or []):
                if isinstance(p, list) and len(p) == 3:
                    out.append({"corner": corner, "name": f"{link.get('name', 'link')}.{i}", "kind": "chassis",
                                "pos": from_sae(p, yf, zg)})
    return out


# what compare() looks at: (key, label, unit); values in m or kg
COMPARED = [("wheelbase", "Wheelbase", "m"), ("track_front", "Front track", "m"),
            ("track_rear", "Rear track", "m"), ("mass_total", "Total mass", "kg")]


def compare(svj, base_measure, base_mass=None):
    """Base vehicle against the SVJ, one row per parameter the user can choose to take.

    base_measure: beamng.measure() of the configured vehicle; base_mass: its total node mass in kg
    if known. Returns [{"key", "label", "unit", "base", "svj", "delta"}] (None where either side has
    no value). Applying a chosen row to the vehicle is roadmap step 3; this only lists them.
    """
    s = summary(svj)
    base = dict(base_measure or {})
    base["mass_total"] = base_mass
    rows = []
    for key, label, unit in COMPARED:
        b, v = base.get(key), s.get(key)
        d = round(v - b, 4) if isinstance(b, (int, float)) and isinstance(v, (int, float)) else None
        rows.append({"key": key, "label": label, "unit": unit, "base": b, "svj": v, "delta": d})
    return rows


# ---------------------------------------------------------------- bundles

def _safe_extract(zpath, base):
    """Empty `base` (deleting it) and extract the zip into it, refusing paths that leave it."""
    if base.exists():
        shutil.rmtree(base)
    base.mkdir(parents=True)
    root = base.resolve()
    with zipfile.ZipFile(zpath) as z:
        for info in z.infolist():
            target = (base / info.filename).resolve()
            if root not in target.parents:
                raise ValueError(f"unsafe path in zip: {info.filename}")
            if not info.is_dir():
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(z.read(info))


def load_bundle(path, work_dir):
    """Read an SVJ from a .zip bundle, or from a .svj.json whose meshes sit beside it (JSON).

    A zip is extracted into work_dir, which is deleted first. Mesh files are matched by their
    uri, else by file name anywhere under the SVJ's folder; their glTF node names are read so
    each visual binding can be checked.

    Returns {"svj" (the document), "summary", "notes", "meshes": [{"id", "uri", "file", "nodes"}],
    "bindings", "axes", "base_dir"}.
    """
    p, base = Path(path), Path(work_dir)
    if zipfile.is_zipfile(p):
        _safe_extract(p, base)
        cands = sorted(base.rglob("*.svj.json")) or [
            f for f in sorted(base.rglob("*.json")) if '"_metadata"' in f.read_text("utf-8", "ignore")[:4000]]
        if not cands:
            raise ValueError("no .svj.json file in the zip")
        svj_file = cands[0]
    else:
        svj_file = p
    doc = json.loads(svj_file.read_text(encoding="utf-8"))
    meta = doc.get("_metadata") or {}
    notes, meshes = [], []
    if meta.get("coordinate_system", "SAE_J670") not in ("SAE_J670",) and not isinstance(meta.get("coordinate_system"), dict):
        notes.append(f"coordinate_system {meta.get('coordinate_system')!r}: only SAE_J670 is supported; points may be misplaced")
    root = svj_file.parent
    for a in (doc.get("assets") or {}).get("meshes", []):
        f = root / a["uri"]
        if not f.is_file():                           # loose files: match by file name
            hits = sorted(root.rglob(Path(a["uri"]).name))
            f = hits[0] if hits else None
        entry = {"id": a["id"], "uri": a["uri"], "file": str(f) if f else None, "nodes": [], "offset": None, "offset_gltf": None}
        if f:
            try:
                data = f.read_bytes()
                entry["nodes"] = gltf.node_names(data, is_glb=f.suffix.lower() == ".glb")
                off, n = mesh_offset(doc, data, f.suffix.lower() == ".glb")
                if off:
                    entry["offset"], entry["offset_gltf"] = off, sae_to_gltf(off, gltf_axes(doc))
                    notes.append(f"mesh {a['uri']}: its {n} wheels are {math.hypot(*off) * 1000:.0f} mm from the SVJ wheel "
                                 f"centres (SAE {off[0]:+.3f} {off[1]:+.3f} {off[2]:+.3f} m): moved onto them")
            except (ValueError, UnicodeDecodeError) as exc:
                notes.append(f"mesh {a['uri']}: {exc}")
        else:
            notes.append(f"mesh {a['uri']} (id {a['id']}) is listed in the SVJ but was not provided: the SVJ names it by a "
                         "path beside the SVJ file, which the editor cannot open itself; pick its folder with "
                         "'Find the meshes folder' in the SVJ panel")
        meshes.append(entry)
    bindings = visual_bindings(doc)
    by_id = {m["id"]: m for m in meshes}
    for b in bindings:
        m = by_id.get(b["mesh_ref"])
        if m is None:
            notes.append(f"{b['path']}: visual mesh_ref {b['mesh_ref']!r} is not in assets.meshes")
        elif m["file"] and b["node"] is not None and b["node"] not in m["nodes"]:
            notes.append(f"{b['path']}: node {b['node']} not found in {m['uri']}")
    return json.dumps({"svj": doc, "summary": summary(doc), "notes": notes, "meshes": meshes,
                       "bindings": bindings, "axes": gltf_axes(doc), "base_dir": str(root)})


def hardpoints_json(svj_json, yf=0.0, zg=0.0):
    """hardpoints() for the editor: SVJ text in, JSON list out."""
    return json.dumps(hardpoints(json.loads(svj_json), yf, zg))


def compare_json(svj_json, measure_json, base_mass=None):
    """compare() for the editor: SVJ and beamng.measure() as JSON text, JSON rows out."""
    return json.dumps(compare(json.loads(svj_json), json.loads(measure_json), base_mass))
