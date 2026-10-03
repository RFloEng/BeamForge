"""The structure's rigidity when beams change length or nodes change weight.

Pure Python, standard library only (runs in Pyodide). A beam stands for a strip of material with a
cross-section A, so when the fit (or a hand move) changes its length from L0 to L1, keeping the
material gives:

  stiffness   k = E A / L     ->  beamSpring and beamDamp x L0 / L1
  mass        m ~ A L         ->  a node's weight x (sum of its beams' L1) / (sum of their L0)
  yield       F = sigma A     ->  beamDeform and beamStrength unchanged by the length

A node made lighter or heavier by the mass and CG taken from the SVJ (factor f = new weight / (old
weight x its length factor)) stands for thinner or thicker material: the beams on it take k, damping, beamDeform and beamStrength
x f (the smaller f of its two nodes). Then k / m, which sets the step the physics can take (BeamNG
steps at 2000 Hz), stays where the base had it; lowering weights without this makes the car explode.

A final check per node: the sum of k on the node over its mass, and the same for the damping, may not
go above the base's value; where it would (a beam much shorter than before), the scaled beams on that
node are softened until it does not.

Left as they are: the suspension's springs and dampers (their values come from the SVJ wheel rates),
|BOUNDED, |PRESSURED and |BROKEN beams, and values written as tuning variables ("$spring_F").
"""

import json
import math
import re

from . import beamng, jbeam, values

DEFAULTS = {"beamSpring": 4300000.0, "beamDamp": 580.0, "beamDeform": 220000.0}   # the game's defaults
SKIP_TYPE = re.compile(r"BOUNDED|PRESSURED|BROKEN", re.I)
CLAMP = (0.25, 4.0)            # L0 / L1 beyond this is a beam the fit folded: held at the limit
STEP = 1e-3                    # relative changes below this are not written


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


def changes(model, configured, weights=None, exclude=(), bl=None, fitted=None):
    """The beams' new values: ({part: {row: {key: value}}}, stats). weights: the new node weights
    ({part: {row: kg}}, values.weight_changes); exclude: {(part, row)} left as they are; fitted: the
    parts written with their edits (None: all), the only ones whose beams follow their length."""
    geo = configured["geometry"]
    rest, nodes = geo.get("rest") or {}, geo["nodes"]
    bl = bl if bl is not None else beams(model, configured)
    rows, _ = values.node_weights(model, configured)
    m0, m1 = {}, {}
    for part, row, nid, kg, _ in rows:
        m0[nid] = m0.get(nid, 0.0) + kg
        m1[nid] = m1.get(nid, 0.0) + ((weights or {}).get(part) or {}).get(row, kg)
    # the material's thickness: the weight change beyond what the beams' length explains
    lf = length_factors(model, configured, exclude, bl, fitted)
    f = {n: m1[n] / (m0[n] * lf.get(n, 1.0)) for n in m0 if m0[n] > 0}

    new, k0, k1, c0, c1 = [], {}, {}, {}, {}
    for b in bl:
        v, nv = b["values"], {}
        if _scalable(b, exclude) and b["a"] in rest and b["b"] in rest:
            l0, l1 = math.dist(rest[b["a"]], rest[b["b"]]), math.dist(nodes[b["a"]], nodes[b["b"]])
            g = _clamp(l0 / l1) if l0 > 1e-6 and l1 > 1e-6 and (fitted is None or b["part"] in fitted) else 1.0
            fm = min(f.get(b["a"], 1.0), f.get(b["b"], 1.0))
            for k in ("beamSpring", "beamDamp"):
                if k in v:
                    nv[k] = v[k] * g * fm
            for k in ("beamDeform", "beamStrength"):
                if k in v:
                    nv[k] = v[k] * fm
        new.append(nv)
        for n in (b["a"], b["b"]):
            k0[n] = k0.get(n, 0.0) + v.get("beamSpring", 0.0)
            k1[n] = k1.get(n, 0.0) + nv.get("beamSpring", v.get("beamSpring", 0.0))
            c0[n] = c0.get(n, 0.0) + v.get("beamDamp", 0.0)
            c1[n] = c1.get(n, 0.0) + nv.get("beamDamp", v.get("beamDamp", 0.0))

    # the check: k / m and c / m at each node no higher than the base's
    mass = {n: m1[n] / m0[n] for n in m0 if m0[n] > 0}

    def over(s0, s1):
        return {n: (s1[n] / s0[n]) / mass[n] for n in s0 if s0[n] > 0 and n in mass and s1[n] / s0[n] > mass[n] * (1 + STEP)}
    rk, rc = over(k0, k1), over(c0, c1)
    softened = 0
    for b, nv in zip(bl, new):
        dk = max(rk.get(b["a"], 1.0), rk.get(b["b"], 1.0))
        dc = max(rc.get(b["a"], 1.0), rc.get(b["b"], 1.0))
        if "beamSpring" in nv and dk > 1:
            nv["beamSpring"] /= dk
            softened += dk > 1.01
        if "beamDamp" in nv and dc > 1:
            nv["beamDamp"] /= dc

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
