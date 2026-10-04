"""The structure's stiffness when beams change length or nodes change weight.

Pure Python, standard library only (runs in Pyodide). The base vehicle's structure is the reference:
BeamNG's own cars are tuned for its physics, and an SVJ describes geometry, springs and dampers, not
how stiff a body shell is. So the beams keep the base's material:

  length      a beam the fit (or a hand move) makes longer or shorter keeps its material: beamSpring and
              beamDamp x L0 / L1 (k = E A / L); beamDeform and beamStrength stay. A node's weight follows
              the length of its beams (m ~ A L).
  mass        the target mass and CG move the body's node weights (values.weight_changes), never the
              beams. A node may get only as light as the base's own most demanding node allows: its sum
              of k over its mass (and of damping over its mass) at most the base vehicle's highest
              (mass_floors), the step BeamNG's physics already takes on that car. Lighter than that, the
              target mass is not reached (and the export says so) rather than the structure softened.

A final check per node: where a node would still go over the base's highest k / m or c / m (a beam much
shorter than before), the body's beams on it are softened just enough. The suspension and running gear
(values.UNSPRUNG) keep their beams as they are, and the springs and dampers their SVJ values.
"""

import json
import math
import re

from . import beamng, jbeam, values

DEFAULTS = {"beamSpring": 4300000.0, "beamDamp": 580.0, "beamDeform": 220000.0}   # the game's defaults
SKIP_TYPE = re.compile(r"BOUNDED|PRESSURED|BROKEN", re.I)
CLAMP = (0.25, 4.0)            # L0 / L1 beyond this is a beam the fit folded: held at the limit
STEP = 1e-3                    # relative changes below this are not written
MIN_SOFT = 0.2                 # the check softens a beam to no less than this share of its value


def _clamp(x):
    return max(CLAMP[0], min(CLAMP[1], x))


def beams(model, configured):
    """Every beam of the active parts: [{"part", "row", "a", "b", "type", "values": {key: number},
    "vars": {keys written as a tuning variable}}], row being the index in the part's beams table."""
    parts = beamng._parts_held(model)
    nodes = configured["geometry"]["nodes"]
    vars_ = {x["name"]: x["value"] for x in configured["variables"] if isinstance(x["value"], (int, float))}
    out = []
    for name in values._active(configured):
        rows = (parts.get(name) or {}).get("part", {}).get("beams")
        if not (isinstance(rows, list) and rows and isinstance(rows[0], list)):
            continue
        head = [str(h).rstrip(":") for h in rows[0]]
        props = {}
        for i, row in enumerate(rows[1:], 1):
            if isinstance(row, dict):
                props.update(row)
                continue
            if not isinstance(row, list):
                continue
            inline = row[-1] if row and isinstance(row[-1], dict) else {}
            rec = dict(props)
            rec.update(inline)
            rec.update(zip(head, row[:-1] if inline else row))
            a, b = rec.get("id1"), rec.get("id2")
            if a not in nodes or b not in nodes:
                continue
            vals, var = {}, set()
            for k in ("beamSpring", "beamDamp", "beamDeform", "beamStrength"):
                raw = rec.get(k, DEFAULTS.get(k))
                if isinstance(raw, str) and raw.startswith("$") and not raw.startswith("$="):
                    var.add(k)
                try:
                    x = jbeam.to_float(raw, vars_, None)
                except (jbeam.UnresolvedValue, TypeError, ValueError):
                    continue
                if x is not None and math.isfinite(x):
                    vals[k] = x
            out.append({"part": name, "row": i, "a": a, "b": b, "type": str(rec.get("beamType", "")),
                        "values": vals, "vars": var})
    return out


def _scalable(b, exclude):
    return not SKIP_TYPE.search(b["type"]) and (b["part"], b["row"]) not in exclude


def length_factors(model, configured, exclude=(), bl=None, fitted=None):
    """{node: sum of its beams' new length / sum of their old length}, for the nodes whose beams
    changed length (old: the positions before the edits; new: as configured). fitted: the parts
    written with their edits (None: all); the others keep their beams as they were."""
    geo = configured["geometry"]
    rest, nodes = geo.get("rest") or {}, geo["nodes"]
    s0, s1 = {}, {}
    for b in bl if bl is not None else beams(model, configured):
        a, c = b["a"], b["b"]
        if not _scalable(b, exclude) or a not in rest or c not in rest or (fitted is not None and b["part"] not in fitted):
            continue
        l0, l1 = math.dist(rest[a], rest[c]), math.dist(nodes[a], nodes[c])
        if l0 < 1e-6 or l1 < 1e-6:
            continue
        for n in (a, c):
            s0[n] = s0.get(n, 0.0) + l0
            s1[n] = s1.get(n, 0.0) + l1
    out = {}
    for n in s0:
        f = _clamp(s1[n] / s0[n])
        if abs(f - 1) > STEP:
            out[n] = round(f, 5)
    return out


def _length_scaled(bl, configured, exclude, fitted):
    """Each beam's stiffness and damping after its length change: [{key: value}] (empty: unchanged)."""
    geo = configured["geometry"]
    rest, nodes = geo.get("rest") or {}, geo["nodes"]
    out = []
    for b in bl:
        v, nv = b["values"], {}
        if _scalable(b, exclude) and b["a"] in rest and b["b"] in rest and (fitted is None or b["part"] in fitted):
            l0, l1 = math.dist(rest[b["a"]], rest[b["b"]]), math.dist(nodes[b["a"]], nodes[b["b"]])
            g = _clamp(l0 / l1) if l0 > 1e-6 and l1 > 1e-6 else 1.0
            if abs(g - 1) > STEP:
                for k in ("beamSpring", "beamDamp"):
                    if k in v:
                        nv[k] = v[k] * g
        out.append(nv)
    return out


def _sums(bl, new):
    k, c = {}, {}
    for b, nv in zip(bl, new):
        for n in (b["a"], b["b"]):
            k[n] = k.get(n, 0.0) + nv.get("beamSpring", b["values"].get("beamSpring", 0.0))
            c[n] = c.get(n, 0.0) + nv.get("beamDamp", b["values"].get("beamDamp", 0.0))
    return k, c


def _limits(model, configured, bl):
    """The base vehicle's highest sum of k over mass and of damping over mass at a node: (k, c)."""
    rows, _ = values.node_weights(model, configured)
    m = {}
    for _, _, nid, kg, _ in rows:
        m[nid] = m.get(nid, 0.0) + kg
    k, c = _sums(bl, [{} for _ in bl])
    return (max((k[n] / m[n] for n in k if m.get(n, 0) > 0), default=0.0),
            max((c[n] / m[n] for n in c if m.get(n, 0) > 0), default=0.0))


def mass_floors(model, configured, exclude=(), bl=None, fitted=None):
    """{node: the least weight (kg) it may take}: its sum of k (after the length changes) over the base's
    highest k / m, and the same for the damping; values.weight_changes keeps every node at or above."""
    bl = bl if bl is not None else beams(model, configured)
    kmax, cmax = _limits(model, configured, bl)
    k, c = _sums(bl, _length_scaled(bl, configured, exclude, fitted))
    return {n: round(max(k.get(n, 0.0) / kmax if kmax else 0.0, c.get(n, 0.0) / cmax if cmax else 0.0), 4)
            for n in set(k) | set(c)}


def changes(model, configured, weights=None, exclude=(), bl=None, fitted=None):
    """The beams' new values: ({part: {row: {key: value}}}, stats). weights: the new node weights
    ({part: {row: kg}}, values.weight_changes); exclude: {(part, row)} left as they are; fitted: the
    parts written with their edits (None: all), the only ones whose beams follow their length."""
    bl = bl if bl is not None else beams(model, configured)
    rows, _ = values.node_weights(model, configured)
    m1 = {}
    for part, row, nid, kg, _ in rows:
        m1[nid] = m1.get(nid, 0.0) + ((weights or {}).get(part) or {}).get(row, kg)
    kmax, cmax = _limits(model, configured, bl)
    new = _length_scaled(bl, configured, exclude, fitted)
    k1, c1 = _sums(bl, new)

    # the check: no node over the base's highest k / m or c / m; where one would be, the body's beams
    # on it are softened just enough (their share of the sum cut by what is too much, never below MIN_SOFT)
    soft = [(b["part"], b["row"]) not in exclude and not re.search(r"PRESSURED|BROKEN", b["type"], re.I)
            and not values.UNSPRUNG.search(b["part"]) for b in bl]      # the suspension keeps its stiffness

    def factors(key, tot1, limit):
        part_ = {}
        for b, nv, ok in zip(bl, new, soft):
            if ok:
                x = nv.get(key, b["values"].get(key, 0.0))
                for n in (b["a"], b["b"]):
                    part_[n] = part_.get(n, 0.0) + x
        out_ = {}
        for n, t1 in tot1.items():
            if m1.get(n, 0) <= 0 or not limit:
                continue
            excess = t1 - limit * m1[n] * (1 + STEP)
            if excess > 0 and part_.get(n, 0) > 0:
                out_[n] = max(MIN_SOFT, 1 - excess / part_[n])
        return out_
    fk, fc = factors("beamSpring", k1, kmax), factors("beamDamp", c1, cmax)
    softened = 0
    for b, nv, ok in zip(bl, new, soft):
        if not ok:
            continue
        gk = min(fk.get(b["a"], 1.0), fk.get(b["b"], 1.0))
        gc = min(fc.get(b["a"], 1.0), fc.get(b["b"], 1.0))
        if gk < 1 and "beamSpring" in (nv.keys() | b["values"].keys()):
            nv["beamSpring"] = nv.get("beamSpring", b["values"].get("beamSpring", 0.0)) * gk
            softened += gk < 0.99
        if gc < 1 and "beamDamp" in (nv.keys() | b["values"].keys()):
            nv["beamDamp"] = nv.get("beamDamp", b["values"].get("beamDamp", 0.0)) * gc

    out, stats = {}, {"beams": 0, "softened": softened, "kept_variables": 0}
    for b, nv in zip(bl, new):
        write = {}
        for k, x in nv.items():
            old = b["values"][k]
            if old and abs(x / old - 1) <= STEP:
                continue
            if k in b["vars"]:
                stats["kept_variables"] += 1
                continue
            write[k] = round(x) if abs(x) >= 100 else round(x, 3)
        if write:
            out.setdefault(b["part"], {})[b["row"]] = write
            stats["beams"] += 1
    return out, stats


def stability(model, configured, bl=None, dt=1 / 2000):
    """The worst nodes for the physics step: [(node, sum k / m * dt^2, sum c / m * dt, kg)], worst
    first by the stiffness index (a check for a written vehicle against its base)."""
    rows, _ = values.node_weights(model, configured)
    m = {}
    for _, _, nid, kg, _ in rows:
        m[nid] = m.get(nid, 0.0) + kg
    k, c = {}, {}
    for b in bl if bl is not None else beams(model, configured):
        for n in (b["a"], b["b"]):
            k[n] = k.get(n, 0.0) + b["values"].get("beamSpring", 0.0)
            c[n] = c.get(n, 0.0) + b["values"].get("beamDamp", 0.0)
    out = [(n, k[n] / m[n] * dt * dt, c.get(n, 0.0) / m[n] * dt, m[n]) for n in k if m.get(n, 0) > 0]
    return sorted(out, key=lambda x: -x[1])


def stability_json(model, configured_json):
    return json.dumps([[n, round(a, 3), round(b, 3), round(kg, 3)] for n, a, b, kg in stability(model, json.loads(configured_json))[:20]])
