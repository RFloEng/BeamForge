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

from beamforge import beamng, jbeam

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


def build(model, new_id, name, configured_json, choices_json, brand=None):
    """The text files of the new vehicle: {"files": {path: text}, "renamed": {old: new}, "notes": [...],
    "counts": {...}}. configured: beamng.configure() of the edited vehicle (with its moves); choices:
    {part: "reuse" | "copy" | "fit"} (missing parts take plan()'s proposal)."""
    problems = check_id(new_id, model)
    if problems:
        raise ValueError("; ".join(problems))
    v = json.loads(configured_json)
    choices = json.loads(choices_json) if choices_json else {}
    for p in json.loads(plan(model, configured_json)):
        choices.setdefault(p["part"], p["choice"])
    geo = v["geometry"]
    deltas = _deltas(geo)
    vars_ = {x["name"]: x["value"] for x in v["variables"] if isinstance(x["value"], (int, float))}
    parts = beamng._parts_held(model)
    active = {n for n, _ in _walk(v["tree"], [])}

    # flexbody shifts: each fitted part's flexbody follows the node nearest to it (configured positions)
    rest = geo["rest"]
    shifts, seen = {}, {}
    for f in v.get("flexbodies", []):
        k = seen.get(f["part"], 0)
        seen[f["part"]] = k + 1
        if choices.get(f["part"]) != "fit" or not rest:
            continue
        near = min(rest, key=lambda n: math.dist(rest[n], f["pos"]))
        if math.dist(rest[near], f["pos"]) < 0.5 and near in deltas:
            shifts.setdefault(f["part"], {})[k] = deltas[near]

    renames = {n: f"{new_id}_{n}" for n in active
               if parts.get(n, {}).get("model") == "common" and choices.get(n) in ("copy", "fit")}
    files, counts, notes = {}, {"fitted": 0, "copied": 0, "reused": 0, "regenerated": 0, "nodes": 0}, []
    base_dir = f"vehicles/{model}/"

    def finish(n, part):
        if choices.get(n) == "fit" and n in active:
            counts["nodes"] += _fit_nodes(part, deltas, vars_)
            _fit_flexbodies(part, n, shifts.get(n, {}), vars_)
        _rename_refs(part, renames)

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

    unfitted = [p["part"] for p in json.loads(plan(model, configured_json)) if p["moved_mm"] >= 0.1 and choices.get(p["part"]) != "fit"]
    if unfitted:
        notes.append("edited parts written without their edits (by your choice): " + ", ".join(unfitted))
    return json.dumps({"files": files, "renamed": renames, "notes": notes, "counts": counts})


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
