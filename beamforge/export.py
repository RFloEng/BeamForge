"""A new BeamNG vehicle from the edited base vehicle, as a mod: nothing of the game is modified.

Pure Python, standard library only (runs in Pyodide). The game builds a vehicle from its own folder
(vehicles/<model>/) and vehicles/common, so the new vehicle gets a folder of its own:

  base vehicle parts   (vehicles/<base>/...) cannot be seen from the new folder, so their files are
                       copied, part by part either "fit" (the edited node positions written in) or
                       "copy" (as they are). Parts of those files that are not active are copied as
                       they are, so the parts menu keeps its choices.
  shared parts         (vehicles/common/...) are "reuse"d where they are (nothing copied), or
                       regenerated ("copy" / "fit") under a new name, <id>_<part>, so they can never
                       clash with the game's own; slot defaults and configurations follow the new name.
  configurations       every configuration of the base vehicle, pointing at the new model, plus
                       "beamforge" (the edited configuration), the new default.
  info.json            the base's, with the new name.
  SVJ values           (optional) springs, dampers, tyres, mass, CG and powertrain taken from the SVJ (values.py),
                       written into the copied parts; a tyre part reused from the game is regenerated
                       to take its new radius; a value driven by a tuning variable is set in the
                       configuration instead.
  SVJ meshes           (optional) the SVJ's glTF meshes written as COLLADA (<id>_svj.dae, the format
                       the game's vehicles use) with a materials file, each added as a flexbody of the
                       part it is attached to (svj_attach), following that part's node groups. With
                       "replace", the base vehicle's body meshes are left out (wheels, tyres, brakes,
                       interior, engine bay and the rest of the running gear are kept).

Node positions written for "fit" are numbers: the jbeam's own value (an expression is evaluated with
the configuration's tuning values) plus the edit (the fitted / moved position minus the configured
one). Slot offsets are added by the game on top, as before. A flexbody "pos" of a fitted part moves
with the node nearest to it, so wheels, brakes and hubs stay on their nodes.

The editor copies the rest of the base folder (meshes, materials, Lua) beside these files; textures
stay in the game, where the materials find them by their absolute paths.
"""

import json
import math
import re

from beamforge import beamng, dae, gltf, jbeam, rigidity, values
from beamforge import svj as svjmod

ID = re.compile(r"^[a-z][a-z0-9_]{1,40}$")
TEXTURE = re.compile(r"\.dds$", re.I)       # textures stay in the game; config thumbnails (.jpg, .png) are copied


def check_id(new_id, base):
    """Problems with a new vehicle id (folder name), as a list of messages ([] when it is fine)."""
    out = []
    if not ID.match(new_id or ""):
        out.append("the id must be 2 to 41 characters: lower-case letters, digits and _, starting with a letter")
    if new_id == base:
        out.append("the id must differ from the base vehicle's, or the base would be replaced")
    if new_id == "common":
        out.append("'common' is the shared parts folder")
    return out


def check_id_json(new_id, base):
    """check_id() for the editor, as JSON."""
    return json.dumps(check_id(new_id, base))


def _walk(node, out):
    if node.get("part"):
        out.append((node["part"], node["slot"]))
    for c in node.get("children", []):
        _walk(c, out)
    return out


def plan(model, configured_json):
    """The active parts of the configured vehicle (beamng.configure() result) and what can be done with
    each: [{"part", "slot", "file", "origin": "vehicle" | "common", "nodes", "moved_mm", "choice",
    "choices"}]. The proposed choice: "fit" for a part whose nodes were moved, else "copy" for the
    base vehicle's parts and "reuse" for shared ones."""
    v = json.loads(configured_json)
    parts = beamng._parts_held(model)
    geo = v["geometry"]
    moved = {}
    for n, p in geo["nodes"].items():
        r = geo["rest"].get(n)
        if r:
            d = math.dist(p, r)
            part = geo["parts"].get(n)
            moved[part] = max(moved.get(part, 0.0), d)
    count = {}
    for n, part in geo["parts"].items():
        count[part] = count.get(part, 0) + 1
    out = []
    for name, slot in _walk(v["tree"], []):
        rec = parts.get(name)
        if not rec:
            continue
        origin = "common" if rec["model"] == "common" else "vehicle"
        mm = round(moved.get(name, 0.0) * 1000, 1)
        choice = "fit" if mm >= 0.1 else ("copy" if origin == "vehicle" else "reuse")
        out.append({"part": name, "slot": slot, "file": rec["file"], "origin": origin, "nodes": count.get(name, 0),
                    "moved_mm": mm, "choice": choice,
                    "choices": ["fit", "copy"] if origin == "vehicle" else ["reuse", "fit", "copy"]})
    return json.dumps(out)


def _deltas(geo):
    """{node: [dx, dy, dz]}: the edits (fit and hand moves) of every moved node."""
    out = {}
    for n, p in geo["nodes"].items():
        r = geo["rest"].get(n)
        if r:
            d = [p[i] - r[i] for i in range(3)]
            if any(abs(x) > 1e-6 for x in d):
                out[n] = d
    return out


def _num(v, vars_):
    try:
        return jbeam.to_float(v, vars_, 0.0)
    except (jbeam.UnresolvedValue, TypeError, ValueError):
        return 0.0


def _fit_nodes(part, deltas, vars_):
    """Write the edits into a part's node table (in place). Returns the number of nodes changed."""
    rows = part.get("nodes")
    if not (isinstance(rows, list) and rows and isinstance(rows[0], list)):
        return 0
    head = [str(h).rstrip(":") for h in rows[0]]
    if not all(k in head for k in ("id", "posX", "posY", "posZ")):
        return 0
    i_id, ix, iy, iz = (head.index(k) for k in ("id", "posX", "posY", "posZ"))
    changed = 0
    for row in rows[1:]:
        if not isinstance(row, list) or len(row) <= max(i_id, ix, iy, iz):
            continue
        d = deltas.get(str(row[i_id]))
        if not d:
            continue
        for k, i in ((0, ix), (1, iy), (2, iz)):
            row[i] = round(_num(row[i], vars_) + d[k], 5)
        changed += 1
    return changed


def _fit_flexbodies(part, name, shifts, vars_):
    """Move each flexbody "pos" of a part by its shift ({index of the row among the part's
    flexbodies: [dx, dy, dz]}), written as the row's own pos (in place)."""
    rows = part.get("flexbodies")
    if not (isinstance(rows, list) and rows and isinstance(rows[0], list)):
        return
    props, k = {}, 0
    for i, row in enumerate(rows[1:], 1):
        if isinstance(row, dict):
            props.update(row)
            continue
        if not isinstance(row, list) or not row or not isinstance(row[0], str) or row[0].startswith("$"):
            continue
        d = shifts.get(k)
        k += 1
        if not d or not any(abs(x) > 1e-6 for x in d):
            continue
        inline = row[-1] if isinstance(row[-1], dict) else None
        pos = (inline or {}).get("pos", props.get("pos")) or {}
        new = {a: round(_num(pos.get(a, 0), vars_) + d[j], 5) for j, a in enumerate("xyz")}
        if inline is None:
            row.append({"pos": new})
        else:
            inline["pos"] = new


def _rename_refs(part, renames):
    """Slot defaults that name a renamed part follow the new name (in place)."""
    for key, col in (("slots", "default"), ("slots2", "default")):
        rows = part.get(key)
        if not (isinstance(rows, list) and rows and isinstance(rows[0], list)):
            continue
        head = [str(h).rstrip(":") for h in rows[0]]
        if col not in head:
            continue
        i = head.index(col)
        for row in rows[1:]:
            if isinstance(row, list) and len(row) > i and row[i] in renames:
                row[i] = renames[row[i]]


# what of the base's own meshes a car with the SVJ's meshes still draws: the wheels, their tyres and hubcaps (the
# wheel parts are the base's own, chosen for the SVJ's size). Everything else of the base's, its body, interior,
# engine, running gear and what its props animate (pedals, gauge needles, the steering wheel), would show beside
# the SVJ's meshes or through them; the SVJ is the car's looks. Lights (props with the SPOTLIGHT mesh) stay: they
# are not meshes.
KEEP = re.compile(r"(?<!steer)(?<!steer_)(?<!steering)(?<!steering_)wheel|tire|tyre|hubcap", re.I)
LIGHT_MESH = re.compile(r"^(spotlight|pointlight|)$", re.I)


def _safe(name):
    return re.sub(r"[^A-Za-z0-9_]+", "_", name).strip("_").lower() or "mesh"


# the panels of a body mesh that follow their own part (a door opens, a hood flies off): the kind, the
# mesh node names it takes, and the base parts of that kind
PANELS = [("trunk", re.compile(r"trunk|boot(?!h)|tailgate|hatch(?!back)|rear_?lid|decklid|rear_?hood|baule|bagagliaio", re.I),
           re.compile(r"_trunk(_|$)|_tailgate(_|$)|_hatch(_|$)", re.I)),
          ("door", re.compile(r"^doors?$|^door_|door_?[lr]\b|door_?(left|right)|door_?[fr][lr]|portiera", re.I),
           re.compile(r"_door(_|$)", re.I)),
          ("hood", re.compile(r"hood|bonnet|cofano", re.I), re.compile(r"_hood(_|$)|_bonnet", re.I)),
          ("bumper_F", re.compile(r"(front|^f)_?bumper|bumper_?(front|f\b|fa)|paraurti[ _]?ant", re.I),
           re.compile(r"_bumper_F(_|$)", re.I)),
          ("bumper_R", re.compile(r"rear\w*_?bumper|^r_?bumper|bumper_?(rear|r\b|ra)|paraurti[ _]?post", re.I),
           re.compile(r"_bumper_R(_|$)", re.I))]
# panels named by the SVJ convention (SVJ::body::<id>, docs/naming_convention.md): read before the names above
SVJ_PANEL = re.compile(r"^SVJ::body::(door_(fl|fr|rl|rr|l|r)|hood|bonnet|trunk|tailgate|hatch|bumper_(f|r|front|rear))$", re.I)
SVJ_KIND = {"door": "door", "hood": "hood", "bonnet": "hood", "trunk": "trunk", "tailgate": "trunk", "hatch": "trunk",
            "bumper_f": "bumper_F", "bumper_front": "bumper_F", "bumper_r": "bumper_R", "bumper_rear": "bumper_R"}
PANEL_MIN = 0.4     # m: the bounding-box diagonal of the smallest thing taken as a panel
PANEL_ANY = re.compile("|".join(f"(?:{p.pattern})" for _, p, _ in PANELS), re.I)
# where panels are looked for: everywhere but the running gear and helpers (some cars keep their doors
# under the high-detail cockpit, which the body itself leaves out)
PANEL_SKIP = re.compile(r"^(wheel|tyre|tire|rim|disc|disk|brake|caliper|susp|flycam|damage_|camera|bullone|bolt)", re.I)


def _panels(v, svj, files, place, chassis):
    """The body mesh's panels as attach rows of their own: [{"path", "node", "mesh_ref", "part",
    "groups"}], each on the base flexbody of its kind whose node group is nearest to it. Nodes named
    by the SVJ convention (SVJ::body::door_fl, hood, trunk, bumper_f...) are taken when the file has
    them; else the panels are found by the names modders use (PANELS)."""
    path = files.get(chassis["mesh_ref"]) or (next(iter(files.values())) if len(files) == 1 else None)
    if not path or not place:
        return []
    with open(path, "rb") as fh:
        data = fh.read()
    glb = path.lower().endswith(".glb")
    axes = svjmod.gltf_axes(svj)
    off = svjmod.mesh_offset(svj, data, glb)[0] or [0.0, 0.0, 0.0]
    geo = v["geometry"]
    members = {}
    for n, gs in (geo.get("groups") or {}).items():
        for g in gs:
            members.setdefault(g, []).append(geo["nodes"][n])
    centre = lambda ps: [sum(p[i] for p in ps) / len(ps) for i in range(3)]  # noqa: E731
    out = []
    names = gltf.named(data, SVJ_PANEL, glb, skip=PANEL_SKIP)
    by_name = not names                                    # no SVJ panel nodes: the modders' names
    for name in names or gltf.named(data, PANEL_ANY, glb, skip=PANEL_SKIP):
        if by_name:
            kind, _, base_re = next(k for k in PANELS if k[1].search(name))
        else:
            ident = name.split("::")[-1].lower()
            kind = SVJ_KIND.get("door" if ident.startswith("door") else ident)
            base_re = next(b for k, _, b in PANELS if k == kind)
        pts = gltf.positions(data, is_glb=glb, under=name, limit=4000)
        if not pts:
            continue
        pts = [svjmod.from_sae([x + o for x, o in zip(svjmod.gltf_to_sae(p, axes), off)], place["yf"], place["ground"])
               for p in pts]
        if math.dist([min(p[i] for p in pts) for i in range(3)], [max(p[i] for p in pts) for i in range(3)]) < PANEL_MIN:
            continue                                       # a switch or a label named "doors", not a panel
        cands = [f for f in v.get("flexbodies", []) if base_re.search(f["part"]) and "_body" not in f["part"]
                 and f.get("groups") and all(g in members for g in f["groups"])]
        if not cands:
            continue
        # one node for both doors (or both of anything): a row per side, each side's triangles only
        both = kind == "door" and min(p[0] for p in pts) < -0.3 and max(p[0] for p in pts) > 0.3
        for side in ((1, -1) if both else (0,)):
            mine = [p for p in pts if p[0] * side > 0] if side else pts
            if not mine:
                continue
            c = centre(mine)
            best = min(cands, key=lambda f: math.dist(c, centre([p for g in f["groups"] for p in members[g]])))
            row = {"path": f"chassis.{kind}.{name}" + ({1: ".left", -1: ".right"}.get(side, "")), "node": name,
                   "mesh_ref": chassis["mesh_ref"], "part": best["part"], "groups": list(best["groups"])}
            if side:
                row["side"] = side
            out.append(row)
    return out


GLASS = re.compile(r"glass|window|windscreen|windshield|vetro|cristal|finestr|lunotto|parabrezza|screen", re.I)
NOT_GLASS = re.compile(r"light|lamp|lens|fari|faro|indicator|signal|led|gauge|mirror_?glass|dmg|damage", re.I)
BASE_GLASS = re.compile(r"glass|windshield|windscreen|window", re.I)


def _has(path, node):
    with open(path, "rb") as fh:
        return gltf.has_node(fh.read(), node, path.lower().endswith(".glb"))


def _glass(v, svj, path, place, body):
    """The body mesh's window glass as attach rows of its own: [{"path", "node": None, "mesh_ref",
    "part", "groups", "prims": [primitive index in the body], "exclude", "glass_of": base mesh}], each
    pane on the base's glass flexbody nearest to it, so it breaks as glass (the base's deform group
    and damaged material) instead of bending with the body. Glass: a transparent (BLEND) material, or
    one named like glass, and not a light's."""
    with open(path, "rb") as fh:
        data = fh.read()
    glb = path.lower().endswith(".glb")
    names = body.get("exclude") or []
    pattern = f"(?:{gltf.NOT_BODY.pattern})"
    if names:
        pattern += "|^(?:" + "|".join(re.escape(x) for x in names) + ")$"
    skip = re.compile(pattern, re.I)
    mats, _ = gltf.materials(data, glb)
    pos, _, prims = gltf.textured(data, is_glb=glb, under=None, skip=skip)
    axes = svjmod.gltf_axes(svj)
    off = svjmod.mesh_offset(svj, data, glb)[0] or [0.0, 0.0, 0.0]
    geo = v["geometry"]
    members = {}
    for n, gs in (geo.get("groups") or {}).items():
        for g in gs:
            members.setdefault(g, []).append(geo["nodes"][n])
    centre = lambda ps: [sum(p[i] for p in ps) / len(ps) for i in range(3)]  # noqa: E731
    cands = [f for f in v.get("flexbodies", []) if BASE_GLASS.search(f["part"]) and not f["mesh"].lower().endswith("_int")
             and f.get("groups") and all(g in members for g in f["groups"])]
    if not cands:
        return []
    rows = {}
    for i, (k, idx) in enumerate(prims):
        m = mats[k] if k is not None and k < len(mats) else None
        if not m or NOT_GLASS.search(m["name"]) or not (m["alpha"] == "BLEND" or GLASS.search(m["name"])) or len(idx) < 3:
            continue
        c = centre([svjmod.from_sae([x + o for x, o in zip(svjmod.gltf_to_sae(pos[j], axes), off)], place["yf"], place["ground"])
                    for j in set(idx)])
        best = min(cands, key=lambda f: math.dist(c, centre([p for g in f["groups"] for p in members[g]])))
        r = rows.setdefault((best["part"], best["mesh"]), {
            "path": f"chassis.glass.{best['part']}", "node": None, "mesh_ref": body["mesh_ref"], "part": best["part"],
            "groups": list(best["groups"]), "prims": [], "exclude": list(names), "glass_of": best["mesh"]})
        r["prims"].append(i)
    return list(rows.values())


PIECE_REACH = 0.9      # m: a suspension piece belongs to a wheel when its middle is this close to its centre
PIECE_MID = 0.1        # m: a piece across the centre line (a front beam, both springs in one mesh) is no wheel's


_AXES = {"x": (1.0, 0.0, 0.0), "y": (0.0, 1.0, 0.0), "z": (0.0, 0.0, 1.0)}


def link_place(pts, axis, inboard, outboard, from_point=None, scale=False):
    """Mesh points (SAE, the mesh's own frame) placed on a link, SVJ section 22.6 (placement link_between_points):
    `axis` ("+x", "-y"...), the mesh's axis from its inboard end to its outboard end, is turned onto the vector from
    the link's inboard end (inboard[from_point], else the centroid of the inboard points) to the outboard point, the
    mesh's origin put on the inboard end; with `scale` the mesh is stretched along the axis only, to span the
    distance. Roll about the axis is the mesh's own. Returns the points in the vehicle's SAE frame."""
    o = list(inboard[from_point]) if isinstance(from_point, int) and 0 <= from_point < len(inboard) \
        else [sum(p[i] for p in inboard) / len(inboard) for i in range(3)]
    d = [outboard[i] - o[i] for i in range(3)]
    length = math.sqrt(sum(x * x for x in d))
    if length < 1e-9:
        return [[o[i] + p[i] for i in range(3)] for p in pts]
    u = [x / length for x in d]
    sign = -1.0 if str(axis).startswith("-") else 1.0
    a = [sign * c for c in _AXES.get(str(axis).lstrip("+-").lower(), _AXES["x"])]
    if scale:
        span = max(sum(p[i] * a[i] for i in range(3)) for p in pts)
        k = length / span if span > 1e-9 else 1.0
        pts = [[p[i] + (k - 1.0) * sum(p[j] * a[j] for j in range(3)) * a[i] for i in range(3)] for p in pts]
    c = sum(a[i] * u[i] for i in range(3))
    if c < -1 + 1e-9:                                    # opposite: a half turn about any axis across a
        t = next(e for e in _AXES.values() if abs(sum(e[i] * a[i] for i in range(3))) < 0.9)
        v = [a[1] * t[2] - a[2] * t[1], a[2] * t[0] - a[0] * t[2], a[0] * t[1] - a[1] * t[0]]
        n = math.sqrt(sum(x * x for x in v))
        v = [x / n for x in v]
        R = [[2 * v[i] * v[j] - (1.0 if i == j else 0.0) for j in range(3)] for i in range(3)]
    else:                                                # Rodrigues: I + [v] + [v]^2 / (1 + c), v = a x u
        v = [a[1] * u[2] - a[2] * u[1], a[2] * u[0] - a[0] * u[2], a[0] * u[1] - a[1] * u[0]]
        K = [[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]]
        K2 = [[sum(K[i][m] * K[m][j] for m in range(3)) for j in range(3)] for i in range(3)]
        R = [[(1.0 if i == j else 0.0) + K[i][j] + K2[i][j] / (1 + c) for j in range(3)] for i in range(3)]
    return [[o[i] + sum(R[i][j] * p[j] for j in range(3)) for i in range(3)] for p in pts]


def _bound_pieces(v, svj, files, path):
    """The suspension parts the SVJ binds to glTF nodes one by one (svj.part_bindings, v0.99.2) whose node is in its
    mesh file: [{"node", "role", "corner", "mesh_ref", "link"}], link the placement of a link_between_points part
    ({"inboard", "outboard", "axis", "from_point", "scale"}) else None. files: {mesh id: path}."""
    wheels = {w["name"] for w in v.get("wheels") or []}
    out = []
    for b in svjmod.part_bindings(svj):
        if not b["role"] or b["corner"] not in wheels or b["part"] == "upright":
            continue                                        # (the upright goes with its corner: svj_attach)
        f = (files or {}).get(b["mesh_ref"]) or path
        if not f or not _has(f, b["node"]):
            continue
        link = None
        if b["placement"] == "link_between_points" and b["inboard"] and b["outboard"]:
            link = {"inboard": b["inboard"], "outboard": b["outboard"], "axis": b["mesh_axis"],
                    "from_point": b["from_point"], "scale": b["scale_to_length"]}
        out.append({"node": b["node"], "role": b["role"], "corner": b["corner"], "mesh_ref": b["mesh_ref"], "link": link})
    return out


def susp_pieces(v, svj, path, place, files=None):
    """The suspension's visible parts in an SVJ mesh file that belong to one wheel: [{"node", "role", "corner"}]
    ("mesh_ref" and "link" too, for parts the SVJ binds itself). Those the SVJ binds node by node (v0.99.2,
    _bound_pieces) are taken as declared, from whichever of its mesh files holds them; without such bindings the
    parts are found by their names (gltf.susp_pieces), corner being the wheel nearest to the part's middle, in the
    placed vehicle's frame; a part across the centre line or far from every wheel is left out (it stays in the body)."""
    bound = _bound_pieces(v, svj, files, path)
    if bound:
        return bound
    glb = path.lower().endswith(".glb")
    with open(path, "rb") as fh:
        data = fh.read()
    axes = svjmod.gltf_axes(svj)
    off = svjmod.mesh_offset(svj, data, glb)[0] or [0.0, 0.0, 0.0]
    wheels = [(w["name"], w["centre"]) for w in v.get("wheels") or [] if w.get("centre")]
    out = []
    for name, role in gltf.susp_pieces(data, glb):
        pos = gltf.positions(data, glb, under=name)
        if not pos or not wheels:
            continue
        pts = [svjmod.from_sae([x + o for x, o in zip(svjmod.gltf_to_sae(p, axes), off)], place["yf"], place["ground"]) for p in pos]
        lo, hi = ([f(p[i] for p in pts) for i in range(3)] for f in (min, max))
        if lo[0] < -PIECE_MID and hi[0] > PIECE_MID:
            continue
        mid = [(a + b) / 2 for a, b in zip(lo, hi)]
        corner, c = min(wheels, key=lambda w: math.dist(w[1], mid))
        if math.dist(c, mid) <= PIECE_REACH:
            out.append({"node": name, "role": role, "corner": corner})
    return out


def svj_attach(configured_json, svj_json, mapping_json=None, files_json=None, place_json=None):
    """Where each SVJ mesh binding goes on the new vehicle by default: [{"path", "node", "mesh_ref",
    "part", "groups"}]. A suspension corner goes to the part and node groups of its tied hub nodes
    (fit mapping), else to the wheel's axle nodes; everything else to the body part (the part with
    the most nodes) and its most common node group. With the mesh files and the SVJ's place on the
    vehicle (files: {mesh id: path}; place: {"yf", "ground"}), the body is split into its panels:
    its doors, hood, trunk and bumpers (SVJ::body::door_fl..., else by the names modders use) go to the
    base's part of that kind nearest to them (_panels), and the body row lists them in "exclude"."""
    v, svj = json.loads(configured_json), json.loads(svj_json)
    mapping = json.loads(mapping_json) if mapping_json else []
    geo = v["geometry"]
    count = {}
    for n, part in geo["parts"].items():
        count[part] = count.get(part, 0) + 1
    body = max(count, key=count.get) if count else None

    def groups_of(nodes):
        c = {}
        for n in nodes:
            for g in geo.get("groups", {}).get(n, []):
                c[g] = c.get(g, 0) + 1
        return [max(c, key=c.get)] if c else []

    out = []
    for b in svjmod.visual_bindings(svj):
        part, groups = body, groups_of([n for n, p in geo["parts"].items() if p == body])
        if b["path"].startswith("suspension."):
            corner = b["path"].split(".", 1)[1]
            hub = [r["nodes"][0] for r in mapping if r["corner"] == corner and r["kind"] == "upright" and r["nodes"]]
            wheel = next((r["nodes"] for r in mapping if r["corner"] == corner and r["kind"] == "wheel"), [])
            nodes = hub or wheel
            if nodes:
                parts_ = [geo["parts"][n] for n in nodes if n in geo["parts"]]
                part = max(set(parts_), key=parts_.count) if parts_ else body
                groups = groups_of(nodes) or groups
        out.append({"path": b["path"], "node": b["node"], "mesh_ref": b["mesh_ref"], "part": part, "groups": groups})
    files = json.loads(files_json) if files_json else {}
    place = json.loads(place_json) if place_json else None
    body = next((r for r in out if r["path"] == "chassis"), None)
    if body and files and place:
        path = files.get(body["mesh_ref"]) or (next(iter(files.values())) if len(files) == 1 else None)
        if path:                                           # (a body with its own node: its SVJ-named panels,
            panels = _panels(v, svj, files, place, body)   # or the modders' names inside it)
            # the suspension's parts baked into the body (wishbones, struts) stay out of it: they sit at the
            # static pose and would hang there while the suspension moves; the base's own parts show, or
            # the archetype's (archetype.piece_rows) follow their nodes
            baked = susp_pieces(v, svj, path, place, files)
            body["exclude"] = sorted({p["node"] for p in panels} | {b["node"] for b in baked})
            glass = _glass(v, svj, path, place, body) if not body["node"] or not _has(path, body["node"]) else []
            if glass:
                body["exclude_prims"] = sorted({i for g in glass for i in g["prims"]})
            out[out.index(body) + 1:out.index(body) + 1] = panels + glass
    return json.dumps(out)


def svj_meshes(svj, files, place, attach, new_id):
    """The SVJ meshes in the new vehicle's frame: [{"name", "material", "part", "groups", "positions",
    "indices", "uvs", "submeshes": [{"material", "indices"}], "materials": {name: glTF material, with
    "image" / "normal_image": (bytes, mime) or None}}]. files: {mesh asset id: path}; place: {"yf",
    "ground"}, where the SVJ sits on the vehicle (the fit's placement); attach: svj_attach() rows
    (possibly changed by the user). One material per glTF material, named <id>_svj<file>_<material>."""
    axes = svjmod.gltf_axes(svj)
    cache, out = {}, []
    for a in attach:
        path = files.get(a.get("mesh_ref")) or (next(iter(files.values())) if len(files) == 1 else None)
        if not path or not a.get("part"):
            continue
        glb = path.lower().endswith(".glb")
        if path not in cache:
            with open(path, "rb") as fh:
                data = fh.read()
            cache[path] = (data, svjmod.mesh_offset(svj, data, glb)[0] or [0.0, 0.0, 0.0],   # the mesh on its wheels
                           gltf.materials(data, glb), len(cache))
        data, off, (gmats, imgs), fi = cache[path]
        under, skip = a["node"], None
        if not under or not gltf.has_node(data, under, glb):
            if a["path"] != "chassis" and not a.get("prims"):
                continue                                   # the file has no such node: nothing to add
            under, skip = None, gltf.NOT_BODY              # the body: the whole mesh but the wheels and such
        if a.get("exclude"):                               # and but the panels that go on their own parts
            names = "|".join(re.escape(x) for x in a["exclude"])
            skip = re.compile(f"(?:{skip.pattern})|^(?:{names})$" if skip else f"^(?:{names})$", re.I)
        pos, uvs, prims = gltf.textured(data, is_glb=glb, under=under, skip=skip)
        link = a.get("link")
        if a.get("prims") is not None:                     # glass panes taken out of the body
            keep = set(a["prims"])
            prims = [p for i, p in enumerate(prims) if i in keep]
        elif a.get("exclude_prims"):
            drop = set(a["exclude_prims"])
            prims = [p for i, p in enumerate(prims) if i not in drop]
        if not any(len(i) >= 3 for _, i in prims):
            continue
        if link:                                           # a link: the mesh in its own frame, put on its hardpoints
            sae = link_place([svjmod.gltf_to_sae(p, axes) for p in pos], link["axis"], link["inboard"], link["outboard"],
                             link.get("from_point"), link.get("scale"))
            pos = [[round(c, 5) for c in svjmod.from_sae(p, place["yf"], place["ground"])] for p in sae]
        else:
            pos = [[round(c, 5) for c in svjmod.from_sae([x + o for x, o in zip(svjmod.gltf_to_sae(p, axes), off)],
                                                         place["yf"], place["ground"])] for p in pos]
        if a.get("side"):                                  # one side of a node that holds both (both doors)
            s = a["side"]
            prims = [(k, [v for t in range(0, len(i) - 2, 3) for v in i[t:t + 3]
                          if (pos[i[t]][0] + pos[i[t + 1]][0] + pos[i[t + 2]][0]) * s > 0]) for k, i in prims]
        name = f"{new_id}_svj_{_safe(a['path'])}"
        subs, mats = {}, {}
        for k, idx in prims:
            g = gmats[k] if k is not None and k < len(gmats) else None
            mname = f"{new_id}_svj{fi}_{_safe(g['name'])}_{k}" if g else f"{name}_mat"
            subs.setdefault(mname, []).extend(idx)
            if g and mname not in mats:
                mats[mname] = dict(g, image=imgs[g["texture"]] if g["texture"] is not None and g["texture"] < len(imgs) else None,
                                   normal_image=imgs[g["normal"]] if g["normal"] is not None and g["normal"] < len(imgs) else None,
                                   image_id=(fi, g["texture"]), normal_id=(fi, g["normal"]))
        all_idx = [v for i in subs.values() for v in i]
        out.append({"name": name, "material": f"{name}_mat", "part": a["part"], "groups": a.get("groups") or [],
                    "glass_of": a.get("glass_of"),
                    "positions": pos, "indices": all_idx, "uvs": uvs, "path": a["path"],
                    "submeshes": [{"material": m, "indices": i} for m, i in subs.items()], "materials": mats})
    return out


def svj_materials(svjm, new_id):
    """The materials.json of the SVJ meshes and their texture files: ({name: BeamNG material (v1.5)},
    {path in the vehicle folder: bytes}). glTF base colour (map and factor), normal map, metal and
    roughness, alpha mode and double-sidedness carry over; a material without a glTF one is grey."""
    mats, images = {}, {}

    def tex(img, ident):
        if not img or not img[0]:
            return None
        ext = ".png" if "png" in (img[1] or "") else ".jpg" if "jpeg" in (img[1] or "") or "jpg" in (img[1] or "") else ".png"
        path = f"vehicles/{new_id}/svj_textures/t{ident[0]}_{ident[1]}{ext}"
        images[path] = img[0]
        return "/" + path
    for m in svjm:
        for sub in m.get("submeshes") or [{"material": m["material"]}]:
            name = sub["material"]
            if name in mats:
                continue
            g = (m.get("materials") or {}).get(name)
            stage = {"baseColorFactor": [0.75, 0.77, 0.8, 1], "roughnessFactor": 0.55, "metallicFactor": 0.1}
            entry = {"name": name, "mapTo": name, "class": "Material", "version": 1.5,
                     "materialTag0": "beamng", "materialTag1": "vehicle"}
            if g:
                stage = {"baseColorFactor": [round(float(x), 4) for x in (list(g["colour"]) + [1])[:4]],
                         "metallicFactor": round(float(g["metallic"]), 3), "roughnessFactor": round(float(g["roughness"]), 3)}
                cm = tex(g["image"], g["image_id"])
                if cm:
                    stage["baseColorMap"] = cm
                nm = tex(g["normal_image"], g["normal_id"])
                if nm:
                    stage["normalMap"] = nm
                if g["alpha"] == "BLEND":
                    entry.update(translucent=True, translucentBlendOp="PreMulAlpha", translucentZWrite=False)
                elif g["alpha"] == "MASK":
                    entry.update(alphaTest=True, alphaRef=int(255 * float(g.get("cutoff", 0.5))))
                if g["double_sided"]:
                    entry["doubleSided"] = True
            entry["Stages"] = [stage, {}, {}, {}]
            mats[name] = entry
    return mats, images


def _add_flexbody(part, mesh, groups, extra=None):
    """Add a flexbody row for `mesh` following `groups` to a part (in place); extra: more properties."""
    rows = part.get("flexbodies")
    if not (isinstance(rows, list) and rows and isinstance(rows[0], list)):
        rows = [["mesh", "[group]:", "nonFlexMaterials"]]
        part["flexbodies"] = rows
    props = {"pos": {"x": 0, "y": 0, "z": 0}, "rot": {"x": 0, "y": 0, "z": 0}, "scale": {"x": 1, "y": 1, "z": 1}}
    props.update(extra or {})
    rows.append([mesh, list(groups), [], props])


def _deform_props(part, mesh):
    """The deform* properties (glass breaking: deformGroup, deformMaterialBase, deformMaterialDamaged...)
    of a part's flexbody row for `mesh`, from its property rows and its own."""
    rows = part.get("flexbodies") if isinstance(part, dict) else None
    if not (isinstance(rows, list) and rows and isinstance(rows[0], list)):
        return {}
    props = {}
    for row in rows[1:]:
        if isinstance(row, dict):
            props.update(row)
        elif isinstance(row, list) and row and row[0] == mesh:
            p = dict(props)
            if isinstance(row[-1], dict):
                p.update(row[-1])
            return {k: v for k, v in p.items() if k.startswith("deform") and v not in (None, "")}
    return {}


def _drop_body_meshes(n, part):
    """Leave out a part's base meshes, its flexbodies and the props that animate a mesh (all but KEEP: the wheels
    and tyres, and the lights); returns how many rows were dropped."""
    dropped = 0
    props = part.get("props")
    if isinstance(props, list) and props and isinstance(props[0], list):       # the animated meshes (not the lights)
        head = [str(h).rstrip(":") for h in props[0]]
        mi = head.index("mesh") if "mesh" in head else 1
        kept = [props[0]] + [r for r in props[1:] if not isinstance(r, list) or len(r) <= mi or LIGHT_MESH.search(str(r[mi]))]
        dropped += sum(1 for r in props[1:] if isinstance(r, list)) - sum(1 for r in kept[1:] if isinstance(r, list))
        part["props"] = kept
    rows = part.get("flexbodies")
    if KEEP.search(n) or not (isinstance(rows, list) and rows and isinstance(rows[0], list)):
        return dropped
    kept = [rows[0]] + [r for r in rows[1:] if not isinstance(r, list) or (r and isinstance(r[0], str) and KEEP.search(r[0]))]
    dropped += sum(1 for r in rows[1:] if isinstance(r, list)) - sum(1 for r in kept[1:] if isinstance(r, list))
    part["flexbodies"] = kept
    return dropped


def strip_meshes(files, keep=r"_svj_susp_"):
    """A diagnostic: the written parts' flexbodies and props out but the meshes whose name matches `keep`
    (default the SVJ's suspension parts), so only those are drawn. In place on files {path: text}; returns
    the number of rows left out."""
    keep = re.compile(keep)
    n = 0
    for path, text in list(files.items()):
        if not path.endswith(".jbeam"):
            continue
        doc = json.loads(text)
        for part in doc.values():
            if not isinstance(part, dict):
                continue
            for sec in ("flexbodies", "props"):
                t = part.get(sec)
                if isinstance(t, list) and t and isinstance(t[0], list):
                    rows = [t[0]] + [r for r in t[1:] if isinstance(r, dict) or (isinstance(r, list) and r and keep.search(str(r[0])))]
                    n += len(t) - len(rows)
                    part[sec] = rows
        files[path] = json.dumps(doc, indent=1)
    return n


def axle_preload(model, configured, bl, axles):
    """{(part, row): {"beamPrecompression", "beamPrecompressionTime"}} for the |NORMAL beams from each
    wheel's axle nodes to the nodes that follow that wheel (its upright): each preloaded to the length it
    has with the axle at its target (fit()'s "axles": {corner: {"node1", "node2", "target": {node: pos}}}),
    in place of the precompression it had (a vanilla camber setting: the SVJ's geometry is the whole
    alignment). configured: with the axle as spawned."""
    from beamforge import kinematics
    nodes = configured["geometry"]["nodes"]
    out = {}
    for w in configured.get("wheels") or []:
        ax = axles.get(w["name"])
        if not ax:
            continue
        ends = {ax["node1"], ax["node2"]}
        hub = kinematics.follows_wheel(model, configured, w, bl)
        for b in bl:
            if "BOUNDED" in b["type"] or "SUPPORT" in b["type"] or "PRESSURED" in b["type"]:
                continue
            for e, o in ((b["a"], b["b"]), (b["b"], b["a"])):
                if e in ends and o in hub and o not in ends:
                    l0 = math.dist(nodes[o], nodes[e])
                    l1 = math.dist(nodes[o], ax["target"][e])
                    if l0 > 1e-6 and abs(l1 / l0 - 1) > 1e-5:
                        out[(b["part"], b["row"])] = {"beamPrecompression": round(l1 / l0, 6), "beamPrecompressionTime": 0.5}
    return out


def build(model, new_id, name, configured_json, choices_json, brand=None, svj_json=None):
    """The text files of the new vehicle: {"files": {path: text}, "renamed": {old: new}, "notes": [...],
    "counts": {...}}. configured: beamng.configure() of the edited vehicle (with its moves); choices:
    {part: "reuse" | "copy" | "fit"} (missing parts take plan()'s proposal). svj_json (optional):
    {"svj", "files", "place", "attach", "replace", "take", "study"}: the SVJ meshes to add (see
    svj_meshes; replace: leave out the base vehicle's body meshes; no "attach": no meshes) and the
    values to take ({row key: True}, values.table keys; study: the SVJ suspension study)."""
    problems = check_id(new_id, model)
    if problems:
        raise ValueError("; ".join(problems))
    v = json.loads(configured_json)
    choices = json.loads(choices_json) if choices_json else {}
    for p in json.loads(plan(model, configured_json)):
        choices.setdefault(p["part"], p["choice"])
    opt = json.loads(svj_json) if svj_json else None
    taken_beams, taken_tyres, taken_vars, taken_weights, taken_pt = ({}, {}, {}, {}, {})
    # rigidity (rigidity.py): node weights follow their beams' lengths, beams their length and weights
    springs = {(r["part"], r["row"]) for r in values.springs_and_dampers(model, v)}
    bl = rigidity.beams(model, v)
    fitted = {n for n, c in choices.items() if c == "fit"}
    lf = rigidity.length_factors(model, v, springs, bl, fitted)
    floors = rigidity.mass_floors(model, v, springs, bl, fitted)
    if opt and opt.get("take"):
        taken_beams, taken_tyres, taken_vars, taken_weights = values.apply(model, v, opt["svj"], opt["take"], opt.get("study"),
                                                                           lf, floors)
        taken_pt = values.powertrain_changes(model, v, opt["svj"], opt["take"])
        for part, ch in list(values.steering_changes(model, v, opt["svj"], opt["take"]).items()) +                 list(values.aero_changes(model, v, opt["svj"], opt["take"]).items()):
            taken_pt.setdefault(part, {}).update(ch)
    if lf and not taken_weights:
        taken_weights = values.weight_changes(model, v, length_factors=lf, floors=floors)
    mass_note = None
    if opt and opt.get("take", {}).get("mass") and taken_weights:
        target = values.svj_mass(opt["svj"])[0]
        wheel_kg = values.node_weights(model, v)[1]
        got = sum(kg for part in taken_weights.values() for kg in part.values()) + wheel_kg
        if target and got > target + 1:
            mass_note = (f"mass {got:.0f} kg, not the SVJ's {target:.0f}: lighter, the base's structure would be too stiff "
                         "for its nodes' weight (BeamNG's physics step); the structure is kept as the base's")
    rig, rig_stats = rigidity.changes(model, v, taken_weights, springs, bl, fitted) if (lf or taken_weights) else ({}, None)
    for part, rows in rig.items():
        for row, vals in rows.items():
            taken_beams.setdefault(part, {}).setdefault(row, {}).update(vals)
    # the benchmark: each corner as stiff at the wheel as the base's in its own geometry (kinematics.stiffen)
    from beamforge import kinematics
    pure = bool(opt and opt.get("pure"))           # a diagnostic build: nodes moved, no stiffness or mass changed
    stiff, bench = kinematics.stiffen(model, v, bl, values.springs_and_dampers(model, v), taken_weights) \
        if not pure and any(abs(x) > 1e-6 for d in _deltas(v["geometry"]).values() for x in d) else ({}, [])
    by_key = {(b["part"], b["row"]): b for b in bl}
    for (part, row), f in stiff.items():
        if f > 1.0 + 1e-3 and "beamSpring" in by_key[(part, row)]["values"]:
            taken_beams.setdefault(part, {}).setdefault(row, {})["beamSpring"] = round(by_key[(part, row)]["values"]["beamSpring"] * f)
    # the wheels' camber and toe (fit()'s "axles"): spawned square, the upright's beams to the wheel are
    # preloaded so it settles at the SVJ's alignment (as vanilla cars set theirs)
    preloaded = axle_preload(model, v, bl, (opt or {}).get("axles") or {})
    for (part, row), vals in preloaded.items():
        taken_beams.setdefault(part, {}).setdefault(row, {}).update(vals)
    for n in list(taken_beams) + list(taken_tyres) + list(taken_weights) + list(taken_pt):   # a part that takes values must be written
        if choices.get(n) == "reuse":
            choices[n] = "copy"
    geo = v["geometry"]
    deltas = _deltas(geo)
    vars_ = {x["name"]: x["value"] for x in v["variables"] if isinstance(x["value"], (int, float))}
    parts = beamng._parts_held(model)
    active = {n for n, _ in _walk(v["tree"], [])}

    # flexbody shifts: a flexbody follows the nodes it is skinned to (its node groups: the mean move of
    # their nodes; a wheel's groups, whose nodes the game builds, the move of its axle nodes); without
    # groups, the node nearest to it. A wheel's meshes (rim, tyre, hubcap) must follow their wheel, so their
    # parts are written with the shift whatever was chosen for them.
    rest = geo["rest"]
    members = {}
    for n, gs in (geo.get("groups") or {}).items():
        for g in gs:
            members.setdefault(g, []).append(n)
    zero = [0.0, 0.0, 0.0]
    wheel_move = {}
    for w in v.get("wheels", []):
        d = [(deltas.get(w["node1"], zero)[i] + deltas.get(w["node2"], zero)[i]) / 2 for i in range(3)]
        for g in (w.get("group"), w.get("hubGroup")):
            if g:
                wheel_move[g] = d
    shifts, seen, follow = {}, {}, set()
    for f in v.get("flexbodies", []):
        k = seen.get(f["part"], 0)
        seen[f["part"]] = k + 1
        if not rest:
            continue
        moves, on_wheel = [], False
        for g in f.get("groups") or []:
            if g in wheel_move:
                moves.append(wheel_move[g])
                on_wheel = True
            elif members.get(g):
                ms = members[g]
                moves.append([sum(deltas.get(n, zero)[i] for n in ms) / len(ms) for i in range(3)])
        if moves:
            d = [sum(m[i] for m in moves) / len(moves) for i in range(3)]
        else:
            near = min(rest, key=lambda n: math.dist(rest[n], f["pos"]))
            d = deltas.get(near) if math.dist(rest[near], f["pos"]) < 0.5 else None
        if not d or not any(abs(x) > 1e-6 for x in d):
            continue
        if on_wheel and choices.get(f["part"]) != "fit":
            follow.add(f["part"])
            choices[f["part"]] = "fit"
        if choices.get(f["part"]) == "fit":
            shifts.setdefault(f["part"], {})[k] = d

    renames = {n: f"{new_id}_{n}" for n in active
               if parts.get(n, {}).get("model") == "common" and choices.get(n) in ("copy", "fit")}
    files, counts, notes = {}, {"fitted": 0, "copied": 0, "reused": 0, "regenerated": 0, "nodes": 0, "svj_meshes": 0,
                                "base_meshes_dropped": 0, "values": 0, "rigidity_beams": 0}, []
    if mass_note:
        notes.append(mass_note)
    for r in bench:
        soft = max(r["base"][k] / r["fitted"][k] for k in r["base"])
        if soft > 1.05:
            what = f"its links stiffened x{r['factor']}" if r["factor"] > 1.0 else "its links could not be stiffened"
            notes.append(f"{r['wheel']}: the fitted suspension holds the wheel {100 * (1 - 1 / soft):.0f} % softer than the base's; "
                         + what + (" (its nodes are at the physics step's limit: still softer)" if r["capped"] else ""))
    counts["corners_stiffened"] = sum(1 for r in bench if r["factor"] > 1.0)
    if rig_stats:
        counts["rigidity_beams"] = rig_stats["beams"]
        if rig_stats["softened"]:
            notes.append(f"{rig_stats['softened']} beams softened so no node is stiffer for its weight than in the base (much shorter beams)")
        if rig_stats["kept_variables"]:
            notes.append(f"{rig_stats['kept_variables']} beam values written as tuning variables kept as they are (not rescaled)")
    base_dir = f"vehicles/{model}/"
    svjm = svj_meshes(opt["svj"], opt["files"], opt["place"], opt["attach"], new_id) if opt and opt.get("attach") else []
    # a mesh attached to a shared part that is reused (not copied) goes to the body part instead
    written = {n for n in parts if parts[n]["model"] != "common"} | set(renames)
    for m in svjm:
        if m["part"] not in written:
            notes.append(f"{m['path']}: attached to {m['part']}, a shared part reused from the game; added to {v['main']} instead")
            m["part"] = v["main"]

    def finish(n, part):
        if choices.get(n) == "fit" and n in active:
            counts["nodes"] += _fit_nodes(part, deltas, vars_)
            _fit_flexbodies(part, n, shifts.get(n, {}), vars_)
        _rename_refs(part, renames)
        if n in active and (n in taken_beams or n in taken_tyres or n in taken_weights or n in taken_pt):
            values.apply_to_part(n, part, taken_beams, taken_tyres, taken_weights)
            values.apply_powertrain(n, part, taken_pt)
            counts["values"] += 1
        if opt and opt.get("replace") and svjm and n in active:
            counts["base_meshes_dropped"] += _drop_body_meshes(n, part)
        for m in svjm:
            if m["part"] == n:
                extra = {}
                if m.get("glass_of"):                      # glass: breaks as the base's glass did
                    extra = _deform_props((parts.get(n) or {}).get("part"), m["glass_of"])
                    if "deformMaterialBase" in extra and m.get("submeshes"):   # the see-through one breaks
                        mats_ = m.get("materials") or {}
                        clear = [x["material"] for x in m["submeshes"] if (mats_.get(x["material"]) or {}).get("alpha") == "BLEND"]
                        extra["deformMaterialBase"] = (clear or [m["submeshes"][0]["material"]])[0]
                _add_flexbody(part, m["name"], m["groups"], extra)
                counts["svj_meshes"] += 1

    # the base vehicle's jbeam files, all their parts (inactive ones as they are)
    for path in sorted(p for p in beamng._DOCS if p.startswith(base_dir) and p.lower().endswith(".jbeam")):
        doc = beamng._DOCS[path]
        if not isinstance(doc, dict):
            notes.append(f"{path}: does not parse, copied as it is")
            files[f"vehicles/{new_id}/" + path[len(base_dir):]] = beamng._TEXT.get(path, "")
            continue
        doc = json.loads(json.dumps(doc))                   # a copy to edit
        for n, part in doc.items():
            if isinstance(part, dict):
                if n in active:
                    counts["fitted" if choices.get(n) == "fit" else "copied"] += 1
                finish(n, part)
        files[f"vehicles/{new_id}/" + path[len(base_dir):]] = json.dumps(doc, indent=1)

    # shared parts regenerated under their new names
    common = {}
    for old, new in sorted(renames.items()):
        part = json.loads(json.dumps(parts[old]["part"]))
        finish(old, part)
        common[new] = part
        counts["regenerated"] += 1
    if common:
        files[f"vehicles/{new_id}/{new_id}_shared_parts.jbeam"] = json.dumps(common, indent=1)
    counts["reused"] = sum(1 for n in active if parts.get(n, {}).get("model") == "common" and n not in renames)

    # configurations: the base's, pointing at the new model, and the edited one as the default
    def pc_for(pc):
        pc = dict(pc)
        pc["model"] = new_id
        pc["parts"] = {s: renames.get(p, p) for s, p in (pc.get("parts") or {}).items()}
        return json.dumps(pc, indent=2)
    for path in sorted(p for p in beamng._DOCS if p.startswith(base_dir) and p.lower().endswith(".pc")):
        doc = beamng._DOCS[path]
        if isinstance(doc, dict):
            files[f"vehicles/{new_id}/" + path[len(base_dir):]] = pc_for(doc)
    edited = dict(v["pc"])
    edited["parts"] = dict(edited.get("parts") or {})
    if taken_vars:                                          # values driven by tuning variables
        edited["vars"] = dict(edited.get("vars") or {}, **taken_vars)
    for n, slot in _walk(v["tree"], []):
        if n in renames:                                    # the regenerated part is chosen in its slot
            edited["parts"][slot] = renames[n]
    files[f"vehicles/{new_id}/beamforge.pc"] = pc_for(edited)

    # info files
    info = beamng._DOCS.get(f"{base_dir}info.json")
    info = dict(info) if isinstance(info, dict) else {}
    info.update({"Name": name, "default_pc": "beamforge"})
    if brand:
        info["Brand"] = brand
    files[f"vehicles/{new_id}/info.json"] = json.dumps(info, indent=2)
    for path in sorted(p for p in beamng._DOCS if p.startswith(base_dir) and re.search(r"/info_[^/]+\.json$", p)):
        files[f"vehicles/{new_id}/" + path[len(base_dir):]] = beamng._TEXT.get(path, "")
    files[f"vehicles/{new_id}/info_beamforge.json"] = json.dumps(
        {"Configuration": "BeamForge", "Description": f"{name}: made with BeamForge from {model}"}, indent=2)

    binary = {}
    if svjm:                                                # the meshes, their materials and textures
        files[f"vehicles/{new_id}/{new_id}_svj.dae"] = dae.write(svjm)
        mats, images = svj_materials(svjm, new_id)
        files[f"vehicles/{new_id}/{new_id}_svj.materials.json"] = json.dumps(mats, indent=2)
        import base64
        binary = {p: base64.b64encode(b).decode("ascii") for p, b in images.items()}
        counts["svj_textures"] = len(images)

    unfitted = [p["part"] for p in json.loads(plan(model, configured_json)) if p["moved_mm"] >= 0.1 and choices.get(p["part"]) != "fit"]
    if unfitted:
        notes.append("edited parts written without their edits (by your choice): " + ", ".join(unfitted))
    return json.dumps({"files": files, "binary": binary, "renamed": renames, "notes": notes, "counts": counts})


def assets(model, new_id, paths_json):
    """The other files of the base folder to copy beside the new ones: {source path: new path} for
    meshes, materials, Lua and other files under vehicles/<base>/, not the jbeam, configuration and
    info files (build() writes those) nor the textures (they stay in the game)."""
    base_dir = f"vehicles/{model}/"
    out = {}
    for p in json.loads(paths_json):
        if not p.startswith(base_dir) or TEXTURE.search(p):
            continue
        rel = p[len(base_dir):]
        if re.search(r"\.(jbeam|pc)$", rel, re.I) or re.match(r"^info[^/]*\.json$", rel, re.I):
            continue
        out[p] = f"vehicles/{new_id}/{rel}"
    return json.dumps(out)
