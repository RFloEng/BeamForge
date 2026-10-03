"""BeamNG vehicles as the game builds and modifies them: parts tree, tuning, configurations.

Pure Python, standard library only (runs in Pyodide). Nothing from the game ships with BeamForge:
the editor reads the user's own install (its content/vehicles zips, or unpacked mods), and
JavaScript hands this module the text files only ({path: text} for .jbeam, .pc and .json).

How BeamNG puts a vehicle together (what the game's Vehicle Config menu edits):
  parts       every .jbeam file holds parts: {"part_name": {"slotType": ..., "nodes": ...}}. A part
              fits a slot whose type matches its slotType (a string, or a list of types).
  main part   the part with slotType "main" is the root of the tree.
  slots       a part offers slots for children, in one of two tables:
                "slots"  [type, default, description, {options}]   the slot is named after its type
                "slots2" [name, allowTypes, denyTypes, default, description, {options}]
              options: coreSlot (cannot be left empty), nodeOffset / nodeMove (shift the child's
              nodes; nodeOffset x is mirrored by the node's side), nodeRotate, variables (values
              the slot passes down).
  config      a .pc file: {"format": 2, "model", "mainPartName", "parts": {slot: part or ""},
              "vars": {"$name": value}, "paints"?, "licenseName"?}. A slot missing from "parts"
              takes its default; "" (or "none", in older configs) leaves it empty.
  tuning      parts declare "variables": [name, type "range", unit, category, default, min, max,
              title, description, {subCategory, stepDis, minDis, maxDis, hideInUI}]. The
              Tuning menu shows them grouped by category and subCategory; the .pc stores values.
  info        info.json (Name, Brand, Type, Body Style, default_pc, paints ...) and one
              info_<config>.json per configuration (Configuration, Power, Torque, Weight, Value,
              Drivetrain, Transmission ...).
  shared      parts used by many vehicles (wheels, tyres, brakes, seats ...) live in
              vehicles/common (common.zip in the install, and in any mod that ships some).
  sources     the game reads one virtual file system made of the install (content/vehicles/*.zip),
              the mods of the user folder (mods/**/*.zip and mods/unpacked/<mod>/, active ones per
              mods/db.json) and the user folder's own vehicles/. The same path in several sources:
              the user folder wins over mods, mods over the install (see resolve()).

Paths are as inside the zips: "vehicles/<model>/<file>". Positions: BeamNG axes, X left,
Y rear, Z up, metres.
"""

import json
import math
import re

from beamforge import jbeam


# ---------------------------------------------------------------- reading files

def _parse(text, path):
    """jbeam or (lenient) JSON text -> value; None when the file does not parse."""
    try:
        return jbeam.parse(text.lstrip("﻿"), path)
    except jbeam.JBeamError:
        return None


def model_of(path):
    """Vehicle folder name of a path inside vehicles/ ("vehicles/hatchback/x.jbeam" -> "hatchback")."""
    parts = path.replace("\\", "/").split("/")
    return parts[1] if len(parts) > 2 and parts[0] == "vehicles" else None


def load_parts(files):
    """All parts in {path: text}: {part name: {"part": dict, "file": path, "model": folder}}.

    A part defined twice keeps the vehicle's own definition over vehicles/common, as the game
    lets a vehicle override a common part. Returns (parts, problems).
    """
    parts, problems = {}, []
    for path in sorted(files, key=lambda p: (model_of(p) == "common", p)):
        if not path.lower().endswith(".jbeam"):
            continue
        doc = _parse(files[path], path)
        if not isinstance(doc, dict):
            problems.append(f"{path}: does not parse")
            continue
        for name, part in doc.items():
            if isinstance(part, dict) and name not in parts:
                parts[name] = {"part": part, "file": path, "model": model_of(path)}
    return parts, problems


def slot_types_of(part):
    """The slot types a part fits: slotType as a list."""
    st = part.get("slotType")
    if isinstance(st, list):
        return [str(s) for s in st]
    return [str(st)] if st is not None else []


def slot_rows(part):
    """Slots a part offers, both tables unified:
    [{"name", "allow", "deny", "default", "description", "core", "options"}]."""
    out = []
    for r in jbeam.expand_table(part.get("slots2") or []):
        allow = r.get("allowTypes") or []
        deny = r.get("denyTypes") or []
        out.append({"name": str(r.get("name")), "allow": allow if isinstance(allow, list) else [allow],
                    "deny": deny if isinstance(deny, list) else [deny], "default": r.get("default") or "",
                    "description": _text(r.get("description")), "core": bool(r.get("coreSlot")),
                    "options": {k: r[k] for k in ("nodeOffset", "nodeMove", "nodeMove2", "nodeRotate", "variables") if k in r}})
    for r in jbeam.expand_table(part.get("slots") or []):
        t = str(r.get("type"))
        out.append({"name": t, "allow": [t], "deny": [], "default": r.get("default") or "",
                    "description": _text(r.get("description")), "core": bool(r.get("coreSlot")),
                    "options": {k: r[k] for k in ("nodeOffset", "nodeMove", "nodeMove2", "nodeRotate", "variables") if k in r}})
    return out


def _text(v):
    """Display text of a name or description that may be a localisation object {"txt", "ctx"}.

    Vehicle and configuration names are often keys into the game's translation files
    ("vehiclesData.hatchback.Name"), which are not in the vehicle files: those give "" so the caller
    falls back to a readable id (pretty()).
    """
    if isinstance(v, dict):
        v = v.get("txt") or ""
        return _pretty_key(str(v))
    if isinstance(v, str) and v.startswith("vehiclesData."):
        return ""
    return "" if v is None else str(v)


def pretty(ident):
    """A readable name from an id: "rally_gravel" -> "Rally gravel", "hatchback" -> "small hatchback"."""
    s = str(ident).replace("_", " ").strip()
    return s[:1].upper() + s[1:] if s else s


def _pretty_key(key):
    """'ui.vehicleconfig.information.type.Rally Front Tires' -> 'Rally Front Tires' (last segment)."""
    return key.rsplit(".", 1)[-1] if key.startswith(("ui.", "vehicles")) else key


def part_title(name, part):
    """What the parts menu shows for a part: information.name, else a readable part name.

    Names that are translation keys with placeholders (the game fills "BRAND", "wheelName" ...
    from its language files) fall back to the part name, made readable.
    """
    info = part.get("information") or {}
    title = _text(info.get("name")) if isinstance(info, dict) else ""
    if not title or "BRAND" in title or (" " not in title and re.search(r"[a-z][A-Z]", title)):
        return pretty(name)
    return title


def candidates(slot, by_type):
    """Part names that fit a slot: their slotType is allowed and not denied, sorted."""
    names = set()
    for t in slot["allow"]:
        names.update(by_type.get(t, ()))
    deny = set(slot["deny"])
    return sorted(n for n in names if not deny.intersection(by_type["_types"][n]))


def index_by_type(parts):
    """{slot type: [part names]}, plus "_types": {part: [its slot types]}."""
    by, types = {}, {}
    for n, rec in parts.items():
        ts = slot_types_of(rec["part"])
        types[n] = ts
        for t in ts:
            by.setdefault(t, []).append(n)
    by["_types"] = types
    return by


# ---------------------------------------------------------------- the configured vehicle

def main_part(parts, model, pc=None):
    """The root part: the .pc mainPartName, else a slotType "main" part of the vehicle's own folder."""
    if pc and pc.get("mainPartName") in parts:
        return pc["mainPartName"]
    own = [n for n, r in parts.items() if r["model"] == model and "main" in slot_types_of(r["part"])]
    if not own:
        raise ValueError(f"no main part for {model}")
    return model if model in own else sorted(own)[0]


def build_tree(parts, main, selection, by_type=None):
    """The active parts tree for a selection {slot name: part or ""} (a .pc "parts" table).

    Walks from the main part; every slot takes its selection, else its default. Returns
    {"tree": node, "active": [part names in order], "missing": [(slot, part)]}; a node is
    {"slot", "part", "title", "description", "core", "options": [[name, title]...], "children": [...]}.
    A part reached twice (a cycle) is not walked again.
    """
    by_type = by_type or index_by_type(parts)
    active, missing, seen = [], [], set()

    def walk(name, slot):
        node = {"slot": slot["name"] if slot else "main", "part": name, "children": [],
                "title": part_title(name, parts[name]["part"]) if name in parts else name,
                "description": slot["description"] if slot else "main", "core": bool(slot and slot["core"]) or not slot,
                "options": []}
        if slot:
            node["options"] = [[c, part_title(c, parts[c]["part"])] for c in candidates(slot, by_type)]
        if name not in parts or name in seen:
            return node
        seen.add(name)
        active.append(name)
        for s in slot_rows(parts[name]["part"]):
            choice = selection.get(s["name"], s["default"])
            if choice == "none":                         # older configs write "none" for an empty slot
                choice = ""
            if not choice:
                child = {"slot": s["name"], "part": "", "title": "(empty)", "description": s["description"],
                         "core": s["core"], "children": [],
                         "options": [[c, part_title(c, parts[c]["part"])] for c in candidates(s, by_type)]}
            elif choice not in parts:
                missing.append((s["name"], choice))
                child = {"slot": s["name"], "part": choice, "title": f"{choice} (not found)", "description": s["description"],
                         "core": s["core"], "children": [], "options": []}
            else:
                child = walk(choice, s)
            node["children"].append(child)
        return node

    tree = walk(main, None)
    return {"tree": tree, "active": active, "missing": missing}


def variables(parts, active, values=None):
    """Tuning variables of the active parts with their menu metadata and current value.

    Returns [{"name", "title", "description", "unit", "category", "subCategory", "min", "max",
    "step", "default", "value", "part", "hidden"}], in part order; values (a .pc "vars")
    override the defaults. Later parts redefine a variable as the game does.
    """
    values = values or {}
    out = {}
    for n in active:
        for r in jbeam.expand_table(parts[n]["part"].get("variables") or []):
            name = r.get("name")
            if not name:
                continue
            out[name] = {"name": name, "title": _text(r.get("title")) or name, "description": _text(r.get("description")),
                         "unit": r.get("unit") or "", "category": r.get("category") or "Other",
                         "subCategory": r.get("subCategory") or "", "min": r.get("min"), "max": r.get("max"),
                         "step": r.get("stepDis"), "default": r.get("default"), "part": n,
                         "hidden": bool(r.get("hideInUI"))}
    for v in out.values():
        v["value"] = values.get(v["name"], v["default"])
    return list(out.values())


def _num(v, vars_, default=0.0):
    try:
        return jbeam.to_float(v, vars_, default)
    except (jbeam.UnresolvedValue, TypeError, ValueError):
        return default


def _delta(v):
    """[dx, dy, dz] from a user move (missing or bad values count as 0)."""
    try:
        return [float(v[i]) for i in range(3)]
    except (TypeError, ValueError, IndexError):
        return [0.0, 0.0, 0.0]


def geometry(parts, tree, vars_, moves=None):
    """Nodes and beams of the configured vehicle, for drawing: {"nodes": {id: [x, y, z]}, "beams": [[a, b]],
    "parts": {node id: part}, "beam_parts": [part of each beam], "rest": {id: [x, y, z] without the
    user's moves}, "groups": {id: [node groups]}, "ops": {part: [nodeOffset, nodeMove]} (the slot
    shifts the part gets, summed down the tree)}. Slot nodeOffset / nodeMove are applied to the part in
    the slot and its children (nodeOffset x mirrored by each node's side, as the game does).
    Expressions that cannot be evaluated offline count as 0. "rest", "groups" and "ops" place the
    flexbody meshes (see flexbodies()): a mesh follows its nodes' moves from "rest".

    moves: the user's edits, {"parts": {part: [dx, dy, dz]}, "nodes": {node id: [dx, dy, dz]}} in m,
    added after everything else: a part move shifts that part's nodes and the parts in its slots, as a
    slot nodeMove does in the game (a plain shift, x not mirrored); a node move shifts one node. A beam
    is moved by moving its two nodes.
    """
    moves = moves or {}
    part_moves, node_moves = moves.get("parts") or {}, moves.get("nodes") or {}
    nodes, owner, beams, beam_parts, rest, groups, ops = {}, {}, [], [], {}, {}, {}

    def walk(node, off, move, user):
        name = node["part"]
        if name not in parts or not name:
            return
        part = parts[name]["part"]
        user = [a + b for a, b in zip(user, _delta(part_moves.get(name)))]
        ops[name] = [list(off), list(move)]
        for r in jbeam.expand_table(part.get("nodes") or []):
            try:
                x, y, z = (_num(r.get(k), vars_) for k in ("posX", "posY", "posZ"))
            except (TypeError, ValueError):
                continue
            if r.get("id") is None:
                continue
            o = r.get("nodeOffset") if isinstance(r.get("nodeOffset"), dict) else {}
            ox = off[0] + _num(o.get("x", 0), vars_)
            x += math.copysign(ox, x) if x else ox
            y += off[1] + _num(o.get("y", 0), vars_) + move[1]
            z += off[2] + _num(o.get("z", 0), vars_) + move[2]
            x += move[0]
            nid = str(r["id"])
            d = [a + b for a, b in zip(user, _delta(node_moves.get(nid)))]
            nodes[nid] = [round(x + d[0], 4), round(y + d[1], 4), round(z + d[2], 4)]
            rest[nid] = [round(x, 4), round(y, 4), round(z, 4)]
            g = r.get("group")
            groups[nid] = [str(x_) for x_ in g if x_] if isinstance(g, list) else ([str(g)] if g else [])
            owner[nid] = name
        rows = part.get("beams") or []
        if rows and isinstance(rows[0], list):
            for r in jbeam.expand_table(rows):
                a, b = r.get("id1"), r.get("id2")
                if isinstance(a, str) and isinstance(b, str):
                    beams.append([a, b])
                    beam_parts.append(name)
        slots = {s["name"]: s for s in slot_rows(part)}
        for child in node["children"]:
            s = slots.get(child["slot"], {"options": {}})
            no = s["options"].get("nodeOffset") or {}
            nm = s["options"].get("nodeMove") or {}
            walk(child, [off[0] + _num(no.get("x", 0), vars_), off[1] + _num(no.get("y", 0), vars_), off[2] + _num(no.get("z", 0), vars_)],
                 [move[0] + _num(nm.get("x", 0), vars_), move[1] + _num(nm.get("y", 0), vars_), move[2] + _num(nm.get("z", 0), vars_)],
                 user)

    walk(tree, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    keep = [i for i, b in enumerate(beams) if b[0] in nodes and b[1] in nodes]
    return {"nodes": nodes, "beams": [beams[i] for i in keep], "parts": owner, "beam_parts": [beam_parts[i] for i in keep],
            "rest": rest, "groups": groups, "ops": ops}


def _vec(d, vars_, default=0.0):
    """{x, y, z} of a flexbody pos / rot / scale -> [x, y, z] (missing values: default)."""
    d = d if isinstance(d, dict) else {}
    return [_num(d.get(k, default), vars_, default) for k in ("x", "y", "z")]


def wheels(parts, active, geo, vars_):
    """The wheels the game builds from the pressureWheels tables of the active parts.

    The tables of all active parts read as one, in part order: a property set in one part (radius
    and tireWidth by the tyre, hubRadius and hubWidth by the rim, wheelOffset) applies to the wheel
    rows that follow in later parts (the wheel data part), as in the game. Returns [{"name", "node1",
    "node2", "group", "hubGroup", "centre", "axis", "radius", "width", "hubRadius", "hubWidth",
    "hasTire"}], placed as the game's pressure wheels: axis the unit vector from node1 to node2,
    centre their midpoint (or, with offsetFromNode 1 / 2, half the hubWidth from that node), then
    moved by wheelOffset along the axis.
    """
    nodes = geo["nodes"]
    table = []
    for n in active:
        pw = parts[n]["part"].get("pressureWheels")
        if isinstance(pw, list) and pw and isinstance(pw[0], list):
            table += pw if not table else pw[1:]
    out = []
    for r in jbeam.expand_table(table):
        a, b = r.get("node1"), r.get("node2")
        if a not in nodes or b not in nodes:
            continue
        pa, pb = nodes[a], nodes[b]
        d = [pb[i] - pa[i] for i in range(3)]
        length = math.sqrt(sum(x * x for x in d)) or 1.0
        axis = [x / length for x in d]
        centre = [(pa[i] + pb[i]) / 2 for i in range(3)]
        hub_w = _num(r.get("hubWidth"), vars_, length)
        if r.get("offsetFromNode") == 1:
            centre = [pa[i] + axis[i] * hub_w / 2 for i in range(3)]
        elif r.get("offsetFromNode") == 2:
            centre = [pb[i] - axis[i] * hub_w / 2 for i in range(3)]
        off = _num(r.get("wheelOffset", 0), vars_)
        out.append({"name": str(r.get("name")), "node1": a, "node2": b,
                    "group": str(r.get("group") or ""), "hubGroup": str(r.get("hubGroup") or ""),
                    "centre": [round(centre[i] + axis[i] * off, 4) for i in range(3)], "axis": [round(x, 5) for x in axis],
                    "radius": _num(r.get("radius"), vars_, None), "width": _num(r.get("tireWidth"), vars_, None),
                    "hubRadius": _num(r.get("hubRadius"), vars_, None), "hubWidth": _num(r.get("hubWidth"), vars_, None),
                    "hasTire": r.get("hasTire") is not False})
    return out


def flexbodies(parts, active, vars_, ops=None):
    """The meshes of the active parts: [{"part", "mesh", "groups", "pos", "rot", "scale"}].

    A flexbody row names a mesh (a node of a .dae file in the vehicle's folder or in
    vehicles/common) and the node groups it follows. pos / rot (degrees; the game turns them in the
    order z, x, y) / scale come from the row or a property row before it. As in the game, pos then
    takes the slot shifts of its part (ops from geometry()): nodeOffset with x mirrored by the sign of
    pos x, then nodeMove. That is how a wheel mesh written beside the origin reaches its axle.
    """
    ops = ops or {}
    out = []
    for n in active:
        rows = parts[n]["part"].get("flexbodies")
        if not (isinstance(rows, list) and rows and isinstance(rows[0], list)):
            continue
        for r in jbeam.expand_table(rows):
            mesh, groups = r.get("mesh"), r.get("[group]")
            if not isinstance(mesh, str) or not mesh or mesh.startswith("$"):     # "$=..." names need game state
                continue
            groups = [str(g) for g in groups] if isinstance(groups, list) else ([str(groups)] if groups else [])
            pos = _vec(r.get("pos"), vars_)
            if n in ops:
                (ox, oy, oz), (mx, my, mz) = ops[n]
                pos = [round(pos[0] + math.copysign(ox, pos[0] or 1.0) + mx, 4), round(pos[1] + oy + my, 4), round(pos[2] + oz + mz, 4)]
            out.append({"part": n, "mesh": mesh, "groups": groups, "pos": pos,
                        "rot": _vec(r.get("rot"), vars_), "scale": _vec(r.get("scale"), vars_, 1.0)})
    return out


def measure(parts, active, geo):
    """Wheels and size of the configured vehicle, for using it as a base.

    Wheels come from the pressureWheels rows of the active parts (name, node1, node2): the wheel
    centre is taken midway between its two axle nodes (to confirm
    in-game), the tyre radius from the active tyre parts. Returns {"wheels": [{"name", "centre",
    "side" L/R, "axle" front/rear, "radius"}], "wheelbase", "track_front", "track_rear",
    "ground_z", "length", "width", "height", "front_overhang", "rear_overhang"} in m over every
    node (None where the vehicle has no wheels), plus "body_part", "body_width", "body_height",
    "body_bottom_clearance" over the main body part only (without mirrors, antennas, racks).
    """
    nodes = geo["nodes"]
    radius = None
    rows = []
    for n in active:
        pw = parts[n]["part"].get("pressureWheels") or []
        for r in pw[1:]:
            if isinstance(r, dict) and isinstance(r.get("radius"), (int, float)):
                radius = r["radius"]
        rows += [r for r in jbeam.expand_table(pw) if r.get("node1") in nodes and r.get("node2") in nodes]
    wheels = []
    for r in rows:
        a, c = nodes[r["node1"]], nodes[r["node2"]]
        centre = [round((a[i] + c[i]) / 2, 4) for i in range(3)]
        wheels.append({"name": str(r.get("name")), "centre": centre, "side": "L" if centre[0] > 0 else "R"})
    out = {"wheels": wheels, "wheelbase": None, "track_front": None, "track_rear": None, "ground_z": None}
    if nodes:
        xs, ys, zs = zip(*nodes.values())
        out.update(length=round(max(ys) - min(ys), 4), width=round(max(xs) - min(xs), 4))
    if wheels:
        ymid = sum(w["centre"][1] for w in wheels) / len(wheels)
        for w in wheels:
            w["axle"] = "front" if w["centre"][1] < ymid else "rear"
            w["radius"] = radius
        axle = {k: [w for w in wheels if w["axle"] == k] for k in ("front", "rear")}
        if axle["front"] and axle["rear"]:
            yf = sum(w["centre"][1] for w in axle["front"]) / len(axle["front"])
            yr = sum(w["centre"][1] for w in axle["rear"]) / len(axle["rear"])
            out["wheelbase"] = round(yr - yf, 4)
            out["front_axle_y"], out["rear_axle_y"] = round(yf, 4), round(yr, 4)
            for k in ("front", "rear"):
                xs_ = [w["centre"][0] for w in axle[k]]
                out[f"track_{k}"] = round(max(xs_) - min(xs_), 4) if len(xs_) > 1 else None
            out["front_overhang"] = round(yf - min(ys), 4)
            out["rear_overhang"] = round(max(ys) - yr, 4)
        if radius:
            out["ground_z"] = round(sum(w["centre"][2] for w in wheels) / len(wheels) - radius, 4)
            out["height"] = round(max(zs) - out["ground_z"], 4)
    # the body shell: the part with the most nodes (mirrors, antennas and racks are parts of their own)
    count = {}
    for nid, part in geo.get("parts", {}).items():
        count[part] = count.get(part, 0) + 1
    if count:
        body = max(count, key=count.get)
        bx, by, bz = zip(*(nodes[n] for n, p in geo["parts"].items() if p == body))
        out["body_part"] = body
        out["body_width"] = round(max(bx) - min(bx), 4)
        out["body_top_z"] = round(max(bz), 4)
        if out["ground_z"] is not None:
            out["body_height"] = round(max(bz) - out["ground_z"], 4)
            out["body_bottom_clearance"] = round(min(bz) - out["ground_z"], 4)
    return out


# ---------------------------------------------------------------- sources: install, mods, user folder

KINDS = ("vanilla", "mod", "user")   # lowest priority first


def source_order(sources):
    """Sources from lowest to highest priority: the install, then mods by path, then the user folder.

    The order among mods that ship the same file is not documented by the game: by path here
    (to confirm in-game).
    """
    return sorted(sources, key=lambda s: (KINDS.index(s.get("kind", "mod")), str(s.get("name", "")).lower()))


def resolve(sources_json):
    """Which source serves each file, as the game's virtual file system does.

    sources_json: [{"name", "kind": "vanilla" | "mod" | "user", "paths": ["vehicles/..."]}]. Paths
    compare without case (the game runs on Windows). Returns JSON {"order": [names, lowest
    priority first], "files": {path: source name}, "shadowed": {path: [names it hides]}, "rank":
    {source name: rank}}, the path being the winner's own spelling.
    """
    order = source_order(json.loads(sources_json))
    files, shadowed = {}, {}
    for s in order:
        for p in s.get("paths") or []:
            key = p.replace("\\", "/").lower()
            if key in files:
                shadowed.setdefault(key, []).append(files[key][1])
            files[key] = (p.replace("\\", "/"), s["name"])
    return json.dumps({"order": [s["name"] for s in order],
                       "files": {p: n for p, n in files.values()},
                       "shadowed": {files[k][0]: v for k, v in shadowed.items()},
                       "rank": {s["name"]: i for i, s in enumerate(order)}})


def inactive_mods(db_text):
    """Mods switched off in the game's mods/db.json, as JSON [paths], lower case without trailing
    slash ("/mods/repo/x.zip", "/mods/unpacked/x"). A file that does not parse gives []."""
    db = _parse(db_text or "", "mods/db.json")
    mods = db.get("mods") if isinstance(db, dict) else None
    out = []
    for m in (mods or {}).values():
        if isinstance(m, dict) and m.get("active") is False and m.get("fullpath"):
            out.append(str(m["fullpath"]).replace("\\", "/").rstrip("/").lower())
    return json.dumps(sorted(out))


# ---------------------------------------------------------------- the API the editor calls

def catalog(entries_json):
    """Vehicles of a library: entries {model: {"info": text or None, "configs": {name: info text or None},
    "source"?: kind of the source of its info.json}} (JSON).

    Returns [{"model", "name", "brand", "type", "body", "default_pc", "source", "configs": [{"name",
    "title", "power", "torque", "weight", "drivetrain", "value"}]}], cars first, sorted by name.
    """
    out = []
    for model, e in json.loads(entries_json).items():
        info = _parse(e.get("info") or "{}", f"{model}/info.json") or {}
        cfgs = []
        for cname, text in sorted((e.get("configs") or {}).items()):
            ci = _parse(text or "{}", f"{model}/info_{cname}.json") or {}
            cfgs.append({"name": cname, "title": _text(ci.get("Configuration")) or pretty(cname), "power": ci.get("Power"),
                         "torque": ci.get("Torque"), "weight": ci.get("Weight"), "drivetrain": ci.get("Drivetrain"),
                         "value": ci.get("Value"), "type": ci.get("Config Type")})
        out.append({"model": model, "name": _text(info.get("Name")) or pretty(model), "brand": _text(info.get("Brand")),
                    "type": info.get("Type") or "", "body": info.get("Body Style") or "",
                    "default_pc": info.get("default_pc") or "", "source": e.get("source") or "", "configs": cfgs})
    order = {"Car": 0, "Truck": 1}
    return json.dumps(sorted(out, key=lambda v: (order.get(v["type"], 2), v["name"].lower())))


# Parsed files stay in memory between calls (common is large: about 900 jbeam files), so a part or
# tuning change only rebuilds the tree. {path: parsed document or None}.
_DOCS = {}
_TEXT = {}
_RANK = {}     # {path: rank of its source} (resolve()); a part defined in two files keeps the higher rank


def reset():
    """Forget every held file (a new library was picked)."""
    _DOCS.clear()
    _TEXT.clear()
    _RANK.clear()


def add_files(files_json, ranks_json=None):
    """Parse and keep the files in {path: text} (paths already held are replaced). ranks_json:
    {path: source rank} from resolve(), for parts defined twice (missing paths rank 0).
    Returns {"files": total held, "added": n, "problems": [...]} (problems: files that do not parse)."""
    files = json.loads(files_json)
    ranks = json.loads(ranks_json) if ranks_json else {}
    problems = []
    for path, text in files.items():
        _TEXT[path] = text
        _RANK[path] = ranks.get(path, 0)
        _DOCS[path] = _parse(text, path) if path.lower().endswith((".jbeam", ".pc", ".json")) else None
        if _DOCS[path] is None and path.lower().endswith(".jbeam"):
            problems.append(f"{path}: does not parse")
    return json.dumps({"files": len(_DOCS), "added": len(files), "problems": problems[:20]})


def held_models():
    """Vehicle folders whose files are held, as a JSON list."""
    return json.dumps(sorted({m for m in (model_of(p) for p in _DOCS) if m}))


def _parts_held(model):
    """Parts of one vehicle plus vehicles/common from the held files. A part defined twice: the
    vehicle's own over common, then the higher source rank (a mod over the install)."""
    parts = {}
    for path in sorted(_DOCS, key=lambda p: (model_of(p) == "common", -_RANK.get(p, 0), p)):
        if model_of(path) not in (model, "common") or not path.lower().endswith(".jbeam"):
            continue
        doc = _DOCS[path]
        if isinstance(doc, dict):
            for name, part in doc.items():
                if isinstance(part, dict) and name not in parts:
                    parts[name] = {"part": part, "file": path, "model": model_of(path)}
    return parts


def configure(model, config=None, selection_json=None, values_json=None, moves_json=None):
    """The configured vehicle from the held files (see add_files).

    config: a .pc name of the vehicle (its info.json default_pc when None). selection_json /
    values_json override the .pc's "parts" and "vars" (the user's edits in the parts tree and the
    tuning sliders). moves_json: parts and nodes moved by the user (see geometry()). Returns {"model", "main", "config", "configs": [names], "tree", "variables",
    "geometry", "measure" (wheels and size, see measure()), "wheels" (wheels()), "flexbodies"
    (flexbodies()), "missing", "pc"}, "pc" being the .pc a
    save would write.
    """
    parts = _parts_held(model)
    configs = {re.sub(r"\.pc$", "", p.rsplit("/", 1)[-1]): p for p in _DOCS
               if p.lower().endswith(".pc") and model_of(p) == model}
    info = _DOCS.get(f"vehicles/{model}/info.json") or {}
    if not isinstance(info, dict):
        info = {}
    config = config if config in configs else (info.get("default_pc") if info.get("default_pc") in configs
                                               else (sorted(configs)[0] if configs else None))
    pc = _DOCS.get(configs[config]) if config else None
    pc = pc if isinstance(pc, dict) else {}
    selection = dict(pc.get("parts") or {})
    selection.update(json.loads(selection_json) if selection_json else {})
    values = dict(pc.get("vars") or {})
    values.update(json.loads(values_json) if values_json else {})
    main = main_part(parts, model, pc)
    by_type = index_by_type(parts)
    t = build_tree(parts, main, selection, by_type)
    var_list = variables(parts, t["active"], values)
    vars_ = {v["name"]: v["value"] for v in var_list if isinstance(v["value"], (int, float))}
    geo = geometry(parts, t["tree"], vars_, json.loads(moves_json) if moves_json else None)
    out_pc = {"format": 2, "model": model, "mainPartName": main, "parts": selection,
              "vars": {v["name"]: v["value"] for v in var_list if v["value"] != v["default"]}}
    for k in ("paints", "licenseName"):
        if k in pc:
            out_pc[k] = pc[k]
    return json.dumps({"model": model, "main": main, "config": config, "configs": sorted(configs),
                       "tree": t["tree"], "variables": var_list, "geometry": geo,
                       "measure": measure(parts, t["active"], geo),
                       "wheels": wheels(parts, t["active"], geo, vars_),
                       "flexbodies": flexbodies(parts, t["active"], vars_, geo["ops"]),
                       "missing": [list(m) for m in t["missing"]], "pc": out_pc})


def open_vehicle(files_json, model, config=None, selection_json=None, values_json=None, moves_json=None):
    """add_files + configure in one call (tests, scripts)."""
    add_files(files_json)
    return configure(model, config, selection_json, values_json, moves_json)


def write_pc(pc_json):
    """The text of a .pc file (BeamNG reads plain JSON), indented like the game's own."""
    return json.dumps(json.loads(pc_json), indent=2)
