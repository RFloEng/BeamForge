"""The structure's stiffness when beams change length or nodes change weight.

Pure Python, standard library only (runs in Pyodide). The base vehicle's structure is the reference:
BeamNG's own cars are tuned for its physics, and an SVJ describes geometry, springs and dampers, not
how stiff a body shell is. So the beams keep the base's material:

  length      (LENGTH, off by default) a beam the fit makes longer or shorter keeps its material:
              beamSpring and beamDamp x L0 / L1 (k = E A / L), a node's weight following the length of its
              beams. Off, the fit only moves nodes: the shortened beams it made stiffer had to be softened
              again around them, engine mounts included (a driveshaft snapped under torque).
  mass        the target mass and CG move the body's node weights (values.weight_changes), never the
              beams. A node may get only as light as keeps its sum of k over its mass (and of damping
              over its mass) at its own value in the base, or the base's median over its nodes if that
              is higher (mass_floors): values the car's own nodes already run at. Lighter than that, the
              target mass is not reached (and the export says so) rather than the structure softened.

A final check per node: where a node would still go over that limit (a beam much shorter than before), the body's beams on it are softened just enough. The suspension and running gear
(values.UNSPRUNG) keep their beams as they are, whatever their length (their links only need to be rigid), and the springs and dampers their SVJ values.
"""

import json
import math
import re

from . import beamng, jbeam, values

DEFAULTS = {"beamSpring": 4300000.0, "beamDamp": 580.0, "beamDeform": 220000.0}   # the game's defaults
SKIP_TYPE = re.compile(r"BOUNDED|PRESSURED|BROKEN", re.I)
CLAMP = (0.25, 4.0)            # L0 / L1 beyond this is a beam the fit folded: held at the limit
LENGTH = False                 # beams follow their length change (same material)? Off: the base's structure is
#                                kept as it is (BeamNG's cars are tuned for its physics; the fit only moves nodes)
STEP = 1e-3                    # relative changes below this are not written
MIN_SOFT = 0.2                 # the check softens a beam to no less than this share of its value

# Where vanilla cars run, at the 2000 Hz step: 5 240 nodes of ten vanilla cars (hatchbacks, saloons, coupes, a
# muscle car, a full-size and a pickup), measured 2026-10-08. Per node: sum k / m * dt^2 and sum c / m * dt; per
# beam: k * dt^2 * (1/ma + 1/mb) and c * dt * (1/ma + 1/mb) (the beam's own two-mass mode). Values beyond the 99th
# percentile are where cars start to ring or explode; new beams aim at the median and keep nodes under the 90th.
VANILLA = {"node_k": {"p50": 2.08, "p90": 3.68, "p99": 4.90, "max": 7.49},
           "node_c": {"p50": 0.39, "p90": 0.88, "p99": 1.62, "max": 3.43},
           "beam_k": {"p50": 0.30, "p90": 0.60, "p99": 0.92, "max": 1.81},
           "beam_c": {"p50": 0.05, "p90": 0.12, "p99": 0.35, "max": 1.59}}
DT = 1 / 2000


def beam_values(ma, mb, k_at=(0.0, 0.0), c_at=(0.0, 0.0), aim="p50", cap="p90", dt=DT, want=None):
    """Stable values for a new beam between nodes of ma and mb kg that already carry k_at (sum of beamSpring, N/m)
    and c_at (sum of beamDamp, N s/m): {"beamSpring", "beamDamp", "k_room", "c_room", "limited"}. The beam's own
    mode aims at the vanilla `aim` (VANILLA["beam_k"]), and is lowered if either node would pass the vanilla `cap`
    for nodes; damping follows at the vanilla ratio of damping to stiffness. k_room / c_room: the most either node
    can still take (N/m, N s/m), what a hand-set value may go up to. want: the stiffness asked for (a real tube's
    E A / L, a hand-typed value), taken when it is under the aim."""
    inv = 1.0 / ma + 1.0 / mb
    k = VANILLA["beam_k"][aim] / (dt * dt * inv)
    if want is not None:
        k = min(k, want)
    room_k = min(VANILLA["node_k"][cap] * m / (dt * dt) - s for m, s in ((ma, k_at[0]), (mb, k_at[1])))
    room_c = min(VANILLA["node_c"][cap] * m / dt - s for m, s in ((ma, c_at[0]), (mb, c_at[1])))
    limited = k > room_k
    k = max(0.0, min(k, room_k))
    c = min(k * dt * VANILLA["beam_c"]["p50"] / VANILLA["beam_k"]["p50"], max(0.0, room_c))
    return {"beamSpring": round(k, -3), "beamDamp": round(c, 1), "k_room": round(max(0.0, room_k), -3),
            "c_room": round(max(0.0, room_c), 1), "limited": limited}


def node_index(m, k_sum, c_sum, dt=DT):
    """(stiffness index, damping index, band) of a node (band())."""
    ki, ci = k_sum / m * dt * dt, c_sum / m * dt
    return ki, ci, band(ki, ci)


def band(ki, ci):
    """The band of a node's stiffness and damping indices against vanilla cars' nodes: "ok" up to their 90th percentile,
    "high" to the 99th, "extreme" to the highest any of them runs at, "beyond" past it (where no vanilla car goes: the
    likeliest place for a structure to ring or explode). A vanilla car has about 10 % of its nodes high and 1 % extreme."""
    def under(p):
        return ki <= VANILLA["node_k"][p] and ci <= VANILLA["node_c"][p]
    return "ok" if under("p90") else "high" if under("p99") else "extreme" if under("max") else "beyond"


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
            for k in ("beamSpring", "beamDamp", "beamDeform", "beamStrength", "beamPrecompression"):
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
    written with their edits (None: all); the others keep their beams as they were. {} when LENGTH
    is off."""
    if not LENGTH:
        return {}
    geo = configured["geometry"]
    rest, nodes = geo.get("rest") or {}, geo["nodes"]
    s0, s1 = {}, {}
    for b in bl if bl is not None else beams(model, configured):
        a, c = b["a"], b["b"]
        if (not _scalable(b, exclude) or a not in rest or c not in rest or (fitted is not None and b["part"] not in fitted)
                or values.UNSPRUNG.search(b["part"])):
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
    """Each beam's stiffness and damping after its length change: [{key: value}] (empty: unchanged;
    all unchanged when LENGTH is off)."""
    if not LENGTH:
        return [{} for _ in bl]
    geo = configured["geometry"]
    rest, nodes = geo.get("rest") or {}, geo["nodes"]
    out = []
    for b in bl:
        v, nv = b["values"], {}
        if (_scalable(b, exclude) and b["a"] in rest and b["b"] in rest and (fitted is None or b["part"] in fitted)
                and not values.UNSPRUNG.search(b["part"])):        # suspension links: only rigid, kept as they are
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
    """Per node, the most k / m and damping / m it may reach: ({node: k limit}, {node: c limit}), its own
    in the base vehicle or the base's median over all its nodes, whichever is higher. The median, not
    the highest: a node is stable at a value its own beams' directions allow, and the base's extreme
    nodes owe theirs to beams spread every way; an ordinary node pushed that far went unstable."""
    rows, _ = values.node_weights(model, configured)
    m = {}
    for _, _, nid, kg, _ in rows:
        m[nid] = m.get(nid, 0.0) + kg
    k, c = _sums(bl, [{} for _ in bl])

    def per(tot):
        own = {n: tot[n] / m[n] for n in tot if m.get(n, 0) > 0}
        vals = sorted(own.values())
        med = vals[len(vals) // 2] if vals else 0.0
        return {n: max(x, med) for n, x in own.items()}
    return per(k), per(c)


def mass_floors(model, configured, exclude=(), bl=None, fitted=None):
    """{node: the least weight (kg) it may take}: its sum of k (after the length changes) over its limit
    (_limits), and the same for the damping; values.weight_changes keeps every node at or above."""
    bl = bl if bl is not None else beams(model, configured)
    lk, lc = _limits(model, configured, bl)
    k, c = _sums(bl, _length_scaled(bl, configured, exclude, fitted))
    return {n: round(max(k.get(n, 0.0) / lk[n] if lk.get(n) else 0.0, c.get(n, 0.0) / lc[n] if lc.get(n) else 0.0), 4)
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
    lk, lc = _limits(model, configured, bl)
    new = _length_scaled(bl, configured, exclude, fitted)
    k1, c1 = _sums(bl, new)

    # the check: no node over its limit of k / m or c / m (_limits); where one would be, the body's beams
    # on it are softened just enough (their share of the sum cut by what is too much, never below MIN_SOFT)
    soft = [(b["part"], b["row"]) not in exclude and not re.search(r"PRESSURED|BROKEN", b["type"], re.I)
            and not values.UNSPRUNG.search(b["part"]) for b in bl]      # the suspension keeps its stiffness

    def factors(key, tot1, limits):
        part_ = {}
        for b, nv, ok in zip(bl, new, soft):
            if ok:
                x = nv.get(key, b["values"].get(key, 0.0))
                for n in (b["a"], b["b"]):
                    part_[n] = part_.get(n, 0.0) + x
        out_ = {}
        for n, t1 in tot1.items():
            if m1.get(n, 0) <= 0 or not limits.get(n):
                continue
            excess = t1 - limits[n] * m1[n] * (1 + STEP)
            if excess > 0 and part_.get(n, 0) > 0:
                out_[n] = max(MIN_SOFT, 1 - excess / part_[n])
        return out_
    fk, fc = factors("beamSpring", k1, lk), factors("beamDamp", c1, lc)
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


def added_beam(model, configured, a, b, added=(), want_k=None, want_c=None, bl=None):
    """A beam added by hand between nodes a and b of a base vehicle: {"beamSpring", "beamDamp", "beamDeform",
    "beamStrength", "k_room", "c_room", "limited", "length", "nodes": {id: {"kg", "part", "before", "after"}},
    "warnings"}. The values aim at the vanilla median for the beam and stay under the vanilla 90th percentile at both
    nodes, counting their beams and the ones added before (added: [{"a", "b", "beamSpring", "beamDamp"}]). want_k /
    want_c: typed values, taken up to the room the nodes have. Deform and strength: the median of the beams already
    on the two nodes (the structure they belong to), else the game's default and unbreakable."""
    geo = configured["geometry"]
    if a == b or a not in geo["nodes"] or b not in geo["nodes"]:
        raise ValueError("pick two different nodes of the vehicle")
    bl = bl if bl is not None else beams(model, configured)
    rows, _ = values.node_weights(model, configured)
    m = {}
    for _, _, nid, kg, _ in rows:
        m[nid] = m.get(nid, 0.0) + kg
    if not m.get(a) or not m.get(b):
        raise ValueError(f"node {a if not m.get(a) else b} has no weight: it is not a node a beam can hold")
    k, c = _sums(bl, [{} for _ in bl])
    for x in added or ():
        for n in (x["a"], x["b"]):
            k[n] = k.get(n, 0.0) + float(x.get("beamSpring") or 0)
            c[n] = c.get(n, 0.0) + float(x.get("beamDamp") or 0)
    ka, kb, ca, cb = k.get(a, 0.0), k.get(b, 0.0), c.get(a, 0.0), c.get(b, 0.0)
    v = beam_values(m[a], m[b], (ka, kb), (ca, cb))
    limited = v["limited"]
    if want_k is not None:
        limited = want_k > v["k_room"]
        v["beamSpring"] = round(max(0.0, min(float(want_k), v["k_room"])), -3)
    if want_c is not None:
        limited = limited or want_c > v["c_room"]
        v["beamDamp"] = round(max(0.0, min(float(want_c), v["c_room"])), 1)
    near = [x for x in bl if x["a"] in (a, b) or x["b"] in (a, b)]

    def median(key, default):
        xs = sorted(x["values"][key] for x in near if key in x["values"] and math.isfinite(x["values"][key]))
        return xs[len(xs) // 2] if xs else default
    out = {"beamSpring": v["beamSpring"], "beamDamp": v["beamDamp"],
           "beamDeform": round(median("beamDeform", DEFAULTS["beamDeform"]), -2),
           "beamStrength": median("beamStrength", None), "k_room": v["k_room"], "c_room": v["c_room"], "limited": limited,
           "length": round(math.dist(geo["nodes"][a], geo["nodes"][b]), 4), "nodes": {}, "warnings": []}
    if out["beamStrength"] is not None:
        out["beamStrength"] = round(out["beamStrength"], -2)
    for n, ks, cs in ((a, ka, ca), (b, kb, cb)):
        out["nodes"][n] = {"kg": round(m[n], 3), "part": geo["parts"].get(n),
                           "before": node_index(m[n], ks, cs)[2],
                           "after": node_index(m[n], ks + out["beamSpring"], cs + out["beamDamp"])[2]}
    pa, pb = geo["parts"].get(a) or "", geo["parts"].get(b) or ""
    if bool(values.UNSPRUNG.search(pa)) != bool(values.UNSPRUNG.search(pb)):
        out["warnings"].append(f"it joins the suspension ({pa if values.UNSPRUNG.search(pa) else pb}) to the body: "
                               "the suspension will not move freely")
    if any({x["a"], x["b"]} == {a, b} for x in bl):
        out["warnings"].append("the vehicle already has a beam between these nodes")
    if out["length"] < 0.01:
        out["warnings"].append("the nodes are less than 1 cm apart")
    return out


def added_beam_json(model, configured_json, a, b, added_json=None, want_k=None, want_c=None):
    return json.dumps(added_beam(model, json.loads(configured_json), a, b, json.loads(added_json) if added_json else (),
                                 want_k, want_c))


def stability_json(model, configured_json):
    return json.dumps([[n, round(a, 3), round(b, 3), round(kg, 3)] for n, a, b, kg in stability(model, json.loads(configured_json))[:20]])
