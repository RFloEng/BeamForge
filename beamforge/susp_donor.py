"""A suspension corner copied from a vanilla car: the hub, the arms, the springs and dampers, the steering rack, with the
numbers the game's developers tuned, put on a new car's wheel centres.

Pure Python, standard library only (runs in Pyodide). Nothing of the game is kept in BeamForge: the corner is read from the
user's own install (beamng.add_files) each time.

extract(model, axle, config)   the donor's corner: the nodes, beams, rails, slide nodes, hydros and torsion bars of its
                               suspension part and of its spring-damper and steering children, the nodes of the car's body
                               they hang on (the pickups), the wheel rows, each node's place as the donor car has it
place(car, data, ...)          the corner on the new car: each side moved to its wheel centre as it was (the donor's
                               geometry kept whole), its pickups on the sketch's frame (a frame node within a few cm is the
                               pickup; else a node tied to the nearest frame nodes), its rim nodes the new car's rim ids

The corner stays the donor's: a hub, a lower arm and a strut keep their shapes, so what the game's developers made stable
here is stable. What the new car brings is its frame (the sketch), its mass (the springs and dampers are scaled to it, scale())
and its wheel positions.
"""

import copy
import json
import math
import re

from . import beamng, jbeam

SKIP_SLOTS = re.compile(r"differential|brake|brakepad|^wheel_|wheeldata|power_steering|swaybar|tire|tyre|hubcap|abs|esc", re.I)
HERE = 0.08               # m: a frame node this close to a pickup is the pickup
MOUNTS = 4                # frame nodes a pickup of its own is tied to
MOUNT_REACH = 0.9         # m: no farther frame nodes
PICKUP_KG = 8.0           # kg: a pickup's node
WHEEL_ZONE = 0.3          # m: a node this close to the donor wheel centre that the corner does not define is the wheel's, not the body's
SIDE = 0.03               # m: |x| under this is on the centre line


def _vars_of(configured):
    return {x["name"]: x["value"] for x in configured.get("variables") or [] if isinstance(x, dict) and "name" in x}


def _resolve(v, vars_):
    """A donor value with its tuning variables put in (an expression "$=..." evaluated); other strings as they are."""
    if isinstance(v, str) and v.startswith("$"):
        try:
            x = vars_.get(v, v) if not v.startswith("$=") else v
            return jbeam.to_float(x, vars_, None)
        except (jbeam.UnresolvedValue, TypeError, ValueError, ZeroDivisionError):
            return None
    if isinstance(v, dict):
        return {k: _resolve(x, vars_) for k, x in v.items()}
    if isinstance(v, list):
        return [_resolve(x, vars_) for x in v]
    return v


def _rows(t):
    """[rec dict with the property rows in force and the inline dict merged] of a jbeam table, in order."""
    if not (isinstance(t, list) and t and isinstance(t[0], list)):
        return []
    head = [str(h).rstrip(":") for h in t[0]]
    props, out = {}, []
    for r in t[1:]:
        if isinstance(r, dict):
            props.update(r)
        elif isinstance(r, list) and r:
            rec = dict(props)
            body = r[:-1] if isinstance(r[-1], dict) else r
            rec.update(dict(zip(head, body)))
            if isinstance(r[-1], dict):
                rec.update(r[-1])
            out.append(rec)
    return out


WHEEL_KEYS = ("nodeArm", "torqueCoupling", "torqueArm", "torqueArm2", "steerAxisUp", "steerAxisDown", "wheelDir")


def _wheel_row(r):
    """The wheel row's node references and direction (its inline keys written with a colon, as jbeam does)."""
    return {k.rstrip(":"): v for k, v in r.items() if k.rstrip(":") in WHEEL_KEYS}


def subtree(configured, axle):
    """The suspension part of an axle in the donor's configuration and the parts of its slots (those that make the corner:
    not brakes, wheels, differentials, anti-roll bars). [(slot, part)]."""
    found = []

    def walk(n, inside):
        slot, part = n.get("slot") or "", n.get("part") or ""
        here = inside
        if not inside and re.search(rf"suspension_{axle}$", slot, re.I) and part:
            here = True
            found.append((slot, part))
        elif inside and part and not SKIP_SLOTS.search(slot) and not re.search(r"differential", part, re.I):
            found.append((slot, part))
        for c in n.get("children") or []:
            walk(c, here and not (inside and SKIP_SLOTS.search(slot)))
    walk(configured["tree"], False)
    return found


def extract(model, axle, config=None, held=None):
    """The donor's corner, or raises ValueError. model: the vehicle (its files read); axle "F" or "R"."""
    configured = json.loads(beamng.configure(model, config)) if held is None else held
    geo = configured["geometry"]
    pos = geo.get("rest") or geo["nodes"]
    parts_held = beamng._parts_held(model)
    vars_ = _vars_of(configured)
    chosen = subtree(configured, axle)
    if not chosen:
        raise ValueError(f"{model} has no {axle} suspension part in its configuration {configured['config']}")
    out = {"model": model, "axle": axle, "parts": [p for _, p in chosen], "nodes": {}, "beams": [], "slides": [], "rails": {},
           "hydros": [], "torsion": [], "wheel_rows": {}, "ext": {}, "springs": [], "dampers": [], "notes": []}
    defined = set()
    for _, name in chosen:
        for r in _rows((parts_held.get(name) or {}).get("part", {}).get("nodes")):
            if isinstance(r.get("id"), str):
                defined.add(r["id"])
    wheels = {w["name"]: w for w in configured["wheels"] if w["name"][0] == axle}
    rim_ids = {n for w in wheels.values() for n in (w["node1"], w["node2"])}
    out["wheels"] = {c: {"node1": w["node1"], "node2": w["node2"],
                         "centre": [(pos[w["node1"]][i] + pos[w["node2"]][i]) / 2 for i in range(3)]} for c, w in wheels.items()}
    seen_beam = 0
    for slot, name in chosen:
        part = (parts_held.get(name) or {}).get("part") or {}
        pv = dict(vars_)
        for r in _rows(part.get("variables")):                  # a part's own variable not set by the configuration
            if isinstance(r.get("name"), str) and r["name"] not in pv:
                pv[r["name"]] = r.get("default")
        for r in _rows(part.get("nodes")):
            nid = r.get("id")
            if not isinstance(nid, str) or nid not in pos:
                continue
            props = {k: _resolve(v, pv) for k, v in r.items() if k not in ("id", "posX", "posY", "posZ", "nodeWeight")}
            out["nodes"][nid] = {"pos": list(pos[nid]), "kg": float(_resolve(r.get("nodeWeight", 25), pv) or 25), "props": props}
        for r in _rows(part.get("beams")):
            a, b = r.get("id1"), r.get("id2")
            if not (isinstance(a, str) and isinstance(b, str)):
                continue
            props = {k: _resolve(v, pv) for k, v in r.items() if k not in ("id1", "id2", "optional")}
            props = {k: v for k, v in props.items() if v is not None and v != ""}
            if r.get("optional") is True and (a not in pos or b not in pos):
                continue
            out["beams"].append([a, b, props])
            if str(props.get("name", "")).startswith("spring_"):
                out["springs"].append(len(out["beams"]) - 1)
            elif str(props.get("name", "")).startswith("damper_"):
                out["dampers"].append(len(out["beams"]) - 1)
            seen_beam += 1
        for r in _rows(part.get("slidenodes")):
            out["slides"].append({k: _resolve(v, pv) for k, v in r.items()})
        for k, v in (part.get("rails") or {}).items():
            out["rails"][k] = copy.deepcopy(v)
        for r in _rows(part.get("hydros")):
            a, b = r.get("id1"), r.get("id2")
            props = {k: _resolve(v, pv) for k, v in r.items() if k not in ("id1", "id2")}
            out["hydros"].append([a, b, props])
        for r in _rows(part.get("torsionbars")):
            ids = [r.get(f"id{i}") for i in (1, 2, 3, 4)]
            props = {k: _resolve(v, pv) for k, v in r.items() if k not in ("id1", "id2", "id3", "id4")}
            out["torsion"].append(ids + [props])
        for r in _rows(part.get("pressureWheels")):
            c = r.get("name")
            if c in wheels:
                out["wheel_rows"][c] = _wheel_row(r)
    if len(out["wheel_rows"]) < len(wheels):                    # the wheel rows are in the wheeldata part, which is not a corner's
        for name in [p for s, p in subtree_all(configured, axle)]:
            for r in _rows((parts_held.get(name) or {}).get("part", {}).get("pressureWheels")):
                c = r.get("name")
                if c in wheels and c not in out["wheel_rows"]:
                    out["wheel_rows"][c] = _wheel_row(r)
    # the nodes the corner's beams, slide nodes, rails, hydros and wheel rows name that it does not define: the rim's
    # (mapped to the new car's) and the donor body's pickups
    def refs():
        for a, b, _ in out["beams"]:
            yield a
            yield b
        for a, b, _ in out["hydros"]:
            yield a
            yield b
        for t in out["torsion"]:
            yield from t[:4]
        for r in out["slides"]:
            yield r.get("id")
        for rail in out["rails"].values():
            yield from rail.get("links:") or []
        for row in out["wheel_rows"].values():
            for k in ("nodeArm", "torqueCoupling", "torqueArm", "torqueArm2", "steerAxisUp", "steerAxisDown"):
                yield row.get(k)
    for n in set(x for x in refs() if isinstance(x, str)):
        if n in defined or n in rim_ids:
            continue
        if n in pos:
            if any(math.dist(pos[n], w["centre"]) < WHEEL_ZONE for w in out["wheels"].values()):
                out["notes"].append(f"{n}: a node of the donor's wheel or brake, left out with the beams that hang on it")
                continue
            out["ext"][n] = list(pos[n])
    out["rim_ids"] = sorted(rim_ids)
    from . import values
    mc = values.mass_and_cg(model, configured)
    wb = (configured.get("measure") or {}).get("wheelbase") or 2.5
    out["mass"] = float(mc.get("mass") or 1000.0)
    out["share"] = min(0.9, max(0.1, 1 - float(mc.get("cg_behind_front_axle") or wb / 2) / wb))   # the front's share of its weight
    return out


def subtree_all(configured, axle):
    """Every part under the axle's suspension part, wheel data and all."""
    found = []

    def walk(n, inside):
        slot, part = n.get("slot") or "", n.get("part") or ""
        here = inside
        if not inside and re.search(rf"suspension_{axle}$", slot, re.I) and part:
            here = True
        if here and part:
            found.append((slot, part))
        for c in n.get("children") or []:
            walk(c, here)
    walk(configured["tree"], False)
    return found


def _side(x):
    return 1 if x > SIDE else -1 if x < -SIDE else 0


def place(car, data, wheels, frame, rim, notes=None):
    """The donor's corner on the new car's wheels. wheels: {corner: {"centre": [x, y, z]}} of this axle; frame: the sketch's
    frame node ids; rim: {corner: (outer id, inner id)}. Adds nodes and beams to car; returns {"rails", "slides", "hydros",
    "torsion", "wheel_rows": {corner: inline dict}, "springs": [beam index], "dampers": [beam index], "map": {donor id: car id}}."""
    from . import scratch as sc
    notes = notes if notes is not None else car.notes
    corners = [c for c in data["wheels"] if c in wheels]
    if len(corners) < len(data["wheels"]):
        raise ValueError(f"the donor's {data['axle']} suspension has {len(data['wheels'])} wheels, the new car's axle {len(wheels)}")
    off = {c: [wheels[c]["centre"][i] - data["wheels"][c]["centre"][i] for i in range(3)] for c in corners}
    mean = [sum(off[c][i] for c in corners) / len(corners) for i in range(3)]
    left = next((c for c in corners if data["wheels"][c]["centre"][0] > 0), None)
    right = next((c for c in corners if data["wheels"][c]["centre"][0] < 0), None)

    def offset(p):
        s = _side(p[0])
        return off[left] if s > 0 and left else off[right] if s < 0 and right else mean

    ids = {}                                                  # donor id -> car id
    for c in corners:                                         # the rim: the donor's outer node is the new rim's outer
        outer, inner = rim[c]
        ids[data["wheels"][c]["node1"]] = outer
        ids[data["wheels"][c]["node2"]] = inner
    for nid, n in data["nodes"].items():                      # the corner's own nodes, at their wheel
        new = nid if nid not in car.nodes else nid + "_" + data["axle"].lower()
        d = offset(n["pos"])
        car.node(new, [n["pos"][i] + d[i] for i in range(3)], n["kg"], **{k: v for k, v in n["props"].items() if v not in (None, "")})
        ids[nid] = new
    pickups, tied = 0, 0
    for nid, p in sorted(data["ext"].items()):               # the donor body's nodes the corner hangs on
        if nid in ids:
            continue
        d = offset(p)
        q = [p[i] + d[i] for i in range(3)]
        near = min(frame, key=lambda f: math.dist(car.nodes[f], q))
        if math.dist(car.nodes[near], q) <= HERE:
            ids[nid] = near
            tied += 1
            continue
        new = "bfp_" + nid
        car.node(new, q, PICKUP_KG, group="bf_pickup")
        cands = [f for f in car.nearest(q, frame, MOUNTS) if math.dist(car.nodes[f], q) <= MOUNT_REACH] or car.nearest(q, frame, 2)
        for f in cands:
            car.beam(new, f, aim="p90")
        ids[nid] = new
        pickups += 1
    notes.append(f"{data['model']} {data['axle']} suspension copied: {len(data['nodes'])} nodes, {len(data['beams'])} beams; "
                 f"{tied} pickups on frame nodes, {pickups} on pickups of their own tied to the frame")
    skipped = 0
    base_beam = len(car.beams)
    for a, b, props in data["beams"]:
        if a not in ids or b not in ids:
            skipped += 1
            continue
        vals = dict(props)
        for k in ("beamSpring", "beamDamp"):
            if not isinstance(vals.get(k), (int, float)):
                vals[k] = 0.0
        car.ks[ids[a]] = car.ks.get(ids[a], 0.0)
        car.beam(ids[a], ids[b], vals)
    # the springs and dampers' beam numbers in car.beams (the order kept)
    kept = [i for i, (a, b, _) in enumerate(data["beams"]) if a in ids and b in ids]
    where = {i: base_beam + j for j, i in enumerate(kept)}
    springs = [where[i] for i in data["springs"] if i in where]
    dampers = [where[i] for i in data["dampers"] if i in where]
    if skipped:
        notes.append(f"{skipped} beams of the donor's corner leave out: they hang on nodes it does not have here")
    out = {"rails": {}, "slides": [], "hydros": [], "torsion": [], "wheel_rows": {}, "springs": springs, "dampers": dampers, "map": ids}
    for name, rail in data["rails"].items():
        links = [ids.get(x) for x in rail.get("links:") or []]
        if all(links):
            out["rails"][name] = dict(rail, **{"links:": links})
    for r in data["slides"]:
        if r.get("id") in ids and r.get("railName") in out["rails"]:
            out["slides"].append(dict(r, id=ids[r["id"]]))
    for a, b, props in data["hydros"]:
        if a in ids and b in ids:
            out["hydros"].append([ids[a], ids[b], props])
    for t in data["torsion"]:
        if all(x in ids for x in t[:4]):
            out["torsion"].append([ids[x] for x in t[:4]] + [t[4]])
    for c in corners:
        row = data["wheel_rows"].get(c) or {}
        out["wheel_rows"][c] = {k: (ids.get(v, v) if isinstance(v, str) and k != "wheelDir" else v) for k, v in row.items() if v is not None}
    return out
