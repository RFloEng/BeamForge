"""The base vehicle's own motion ratios, measured on its beams (a small static solve).

Pure Python, standard library only (runs in Pyodide). The SVJ gives wheel rates (spring.rate is
the force at the wheel per metre of wheel travel) and damper curves at the damper. BeamNG's spring
and damper are beams somewhere in the base's suspension, so what they need is the base's motion
ratio, not the SVJ's: coil rate = wheel rate / MR^2.

The ratio is measured the way the game would move, in heave: the body and the steering rack are
held, both wheels of an axle are pushed up by a small step together (so an anti-roll bar turns
without twisting; the solve has no torsion bars), and the axle's suspension nodes settle where the
beams around them balance (linear statics, each beam a spring along its axis; the springs and
dampers being measured, and bounded or support beams, carry nothing). The change of each spring and
damper beam's length over the step is its motion ratio.
"""

import json
import math
import re

from . import beamng, values
from .rigidity import beams as beam_list

STEP = 0.01          # m of wheel travel (the solve is linear: any small step gives the same ratio)
FORCE = 1000.0       # N at the wheel centre for the stiffness test (the solve is linear)
GLUE = 1e-4         # of the stiffest beam: the glue between close free nodes
GLUE_REACH = 0.1     # m: the glue of a beam this long (shorter ones glue more, as 1/length^2)
REACH = 0.9          # m from the wheel centre: suspension nodes beyond this are held with the body
SUSP = re.compile(r"suspension|strut|coilover|shock|damper|spring|hub|arm|link|subframe|knuckle|upright|wheeldata|brake|swaybar|sway_bar", re.I)
NO_LOAD = re.compile(r"BOUNDED|SUPPORT|PRESSURED|BROKEN", re.I)


def _solve(A, b):
    """Dense Gaussian elimination with partial pivoting (A is changed)."""
    n = len(b)
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(A[r][c]))
        if abs(A[p][c]) < 1e-30:
            continue
        A[c], A[p] = A[p], A[c]
        b[c], b[p] = b[p], b[c]
        piv = A[c][c]
        rowc = A[c]
        for r in range(c + 1, n):
            f = A[r][c] / piv
            if f:
                rowr = A[r]
                for k in range(c, n):
                    rowr[k] -= f * rowc[k]
                b[r] -= f * b[c]
    x = [0.0] * n
    for c in range(n - 1, -1, -1):
        s = b[c] - sum(A[c][k] * x[k] for k in range(c + 1, n))
        x[c] = s / A[c][c] if abs(A[c][c]) > 1e-30 else 0.0
    return x


def _system(nodes, owner, load, axle, centres, kscale=None, slides=None):
    """The stiffness matrix of the suspension around some axle nodes: (A, {node: index}, stiffest k).
    The free nodes: the suspension reached from the axle nodes through suspension beams, within REACH
    of the wheel centres; everything else is held. load: the beams that carry ({"a", "b", "part",
    "values"}); kscale: {id(beam): factor} on their k; slides: [[node, rail start, rail end, spring]],
    a slide node held across its rail (free along it). A weak glue along the beams fills the
    directions no beam holds."""
    near = {}
    for b in load:
        near.setdefault(b["a"], []).append(b)
        near.setdefault(b["b"], []).append(b)
    free, todo = set(axle), list(axle)
    while todo:
        n = todo.pop()
        for b in near.get(n, ()):
            m = b["b"] if b["a"] == n else b["a"]
            if m in free or m not in nodes or not SUSP.search(b["part"]) or not SUSP.search(owner.get(m, "")):
                continue
            if min(math.dist(nodes[m], c) for c in centres) > REACH:
                continue                                  # far from the wheels: held with the body
            free.add(m)
            todo.append(m)
    idx = {n: i for i, n in enumerate(sorted(free))}
    size = 3 * len(idx)
    A = [[0.0] * size for _ in range(size)]
    kmax = 0.0
    for b in load:
        a, c = b["a"], b["b"]
        if a not in idx and c not in idx:
            continue
        d = [nodes[c][i] - nodes[a][i] for i in range(3)]
        L = math.sqrt(sum(x * x for x in d))
        if L < 1e-6:
            continue
        u = [x / L for x in d]
        k = b["values"]["beamSpring"] * ((kscale or {}).get(id(b), 1.0))
        kmax = max(kmax, k)
        for p, q in ((a, c), (c, a)):
            if p not in idx:
                continue
            ip = 3 * idx[p]
            for i in range(3):
                for j in range(3):
                    A[ip + i][ip + j] += k * u[i] * u[j]
                    if q in idx:
                        A[ip + i][3 * idx[q] + j] -= k * u[i] * u[j]
    for n, ra, rb, k in slides or []:                     # a slide node across its rail: k (I - u u^T)
        if not k or n not in nodes or ra not in nodes or rb not in nodes or not ({n, ra, rb} & set(idx)):
            continue
        d = [nodes[rb][i] - nodes[ra][i] for i in range(3)]
        L2 = sum(x * x for x in d)
        if L2 < 1e-9:
            continue
        t = max(0.0, min(1.0, sum((nodes[n][i] - nodes[ra][i]) * d[i] for i in range(3)) / L2))
        u = [x / math.sqrt(L2) for x in d]
        P = [[(1.0 if i == j else 0.0) - u[i] * u[j] for j in range(3)] for i in range(3)]
        wts = ((n, 1.0), (ra, -(1 - t)), (rb, -t))
        for p, wp in wts:
            if p not in idx:
                continue
            for q, wq in wts:
                if q not in idx:
                    continue
                for i in range(3):
                    for j in range(3):
                        A[3 * idx[p] + i][3 * idx[q] + j] += k * wp * wq * P[i][j]
        kmax = max(kmax, k)
    if not kmax:
        return A, idx, 0.0
    glue = kmax * GLUE
    for b in load:
        a, c = b["a"], b["b"]
        d = math.dist(nodes[a], nodes[c])
        if d < 1e-6 or (a not in idx and c not in idx):
            continue
        g = glue * (GLUE_REACH / d) ** 2
        for j in range(3):
            if a in idx:
                A[3 * idx[a] + j][3 * idx[a] + j] += g
            if c in idx:
                A[3 * idx[c] + j][3 * idx[c] + j] += g
            if a in idx and c in idx:
                A[3 * idx[a] + j][3 * idx[c] + j] -= g
                A[3 * idx[c] + j][3 * idx[a] + j] -= g
    for i in range(size):
        A[i][i] += kmax * 1e-9                            # and a last hold, so nothing is singular
    return A, idx, kmax


def corner_stiffness(model, configured, wheel, nodes=None, bl=None, springs=None, kscale=None):
    """How stiff a corner holds its wheel: {"lateral", "longitudinal"} in N/m, the force at the wheel
    centre over its movement, and {"camber"} in N m/rad, a couple tilting the wheel over its tilt, with the body and steering rack held and the springs in (the dampers
    carry nothing at rest). nodes: positions to test (default: as configured; the base's "rest" for
    the original geometry); kscale: {id(beam): factor} to try stiffer links."""
    geo = configured["geometry"]
    nodes, owner = nodes or geo["nodes"], geo.get("parts") or {}
    bl = bl if bl is not None else beam_list(model, configured)
    springs = springs if springs is not None else values.springs_and_dampers(model, configured)
    dampers = {(r["part"], r["row"]) for r in springs if r["kind"] == "damper"}
    spring_rows = {(r["part"], r["row"]) for r in springs if r["kind"] == "spring"}
    if wheel["node1"] not in nodes or wheel["node2"] not in nodes:
        return {}
    axle = [wheel["node1"], wheel["node2"]]
    centre = [(nodes[axle[0]][i] + nodes[axle[1]][i]) / 2 for i in range(3)]
    load = [b for b in bl if (b["part"], b["row"]) not in dampers and b["values"].get("beamSpring", 0) > 0
            and ((b["part"], b["row"]) in spring_rows or not NO_LOAD.search(b["type"]))]
    out = {}
    A0, idx, kmax = _system(nodes, owner, load, axle, [centre], kscale, geo.get("slides"))
    if not kmax:
        return {}
    for name, axis in (("lateral", 0), ("longitudinal", 1)):
        f = [0.0] * len(A0)
        for n in axle:
            f[3 * idx[n] + axis] = FORCE / 2
        x = _solve([row[:] for row in A0], f)
        move = sum(abs(x[3 * idx[n] + axis]) for n in axle) / 2
        out[name] = round(FORCE / move) if move > 1e-12 else None
    # camber: a vertical couple on the axle nodes (up outboard, down inboard), the wheel's tilt per N m
    a, b = axle if abs(nodes[axle[0]][0]) >= abs(nodes[axle[1]][0]) else axle[::-1]
    d = math.dist(nodes[a], nodes[b])
    f = [0.0] * len(A0)
    f[3 * idx[a] + 2], f[3 * idx[b] + 2] = FORCE, -FORCE
    x = _solve([row[:] for row in A0], f)
    tilt = abs(x[3 * idx[a] + 2] - x[3 * idx[b] + 2]) / d if d > 1e-6 else 0.0
    out["camber"] = round(FORCE * d / tilt) if tilt > 1e-12 else None          # N m / rad
    return out


def stiffen(model, configured, bl=None, springs=None, weights=None, tolerance=0.05):
    """The benchmark: each corner's stiffness at the wheel (corner_stiffness) in the base's own geometry
    ("rest") against the fitted one; where the fitted corner is softer by more than `tolerance`, its
    suspension links (the corner's beams in suspension parts, not the springs and dampers) take one
    factor until it holds the wheel as the base did, never so far that a node goes over its limit of
    k / m (rigidity._limits, with the new node weights). Returns ({(part, row): factor}, [{"wheel",
    "base", "fitted", "after", "factor", "capped"}])."""
    from beamforge import rigidity
    geo = configured["geometry"]
    rest, nodes = geo.get("rest") or geo["nodes"], geo["nodes"]
    bl = bl if bl is not None else beam_list(model, configured)
    springs = springs if springs is not None else values.springs_and_dampers(model, configured)
    measured = {(r["part"], r["row"]) for r in springs}
    lk, _ = rigidity._limits(model, configured, bl)
    rows, _ = values.node_weights(model, configured)
    m = {}
    for part, row, nid, kg, _ in rows:
        m[nid] = m.get(nid, 0.0) + ((weights or {}).get(part) or {}).get(row, kg)
    ksum = {}
    for b in bl:
        for n in (b["a"], b["b"]):
            ksum[n] = ksum.get(n, 0.0) + b["values"].get("beamSpring", 0.0)
    out, report = {}, []
    for w in configured.get("wheels") or []:
        base = corner_stiffness(model, configured, w, rest, bl, springs)
        new = corner_stiffness(model, configured, w, nodes, bl, springs)
        if not base or not new or not all(base.values()) or not all(new.values()):
            continue
        need = max(base[k] / new[k] for k in base)
        row = {"wheel": w["name"], "base": base, "fitted": new, "after": new, "factor": 1.0, "capped": False}
        report.append(row)
        if need <= 1 + tolerance:
            continue
        axle = [w["node1"], w["node2"]]
        centre = [(nodes[axle[0]][i] + nodes[axle[1]][i]) / 2 for i in range(3)]
        _, idx, _ = _system(nodes, geo.get("parts") or {}, [b for b in bl if b["values"].get("beamSpring", 0) > 0],
                            axle, [centre])
        links = [b for b in bl if values.UNSPRUNG.search(b["part"]) and (b["part"], b["row"]) not in measured
                 and (b["a"] in idx or b["b"] in idx) and b["values"].get("beamSpring", 0) > 0]
        if not links:
            continue
        # the most the links may take: every node they touch stays within its k / m limit
        lsum = {}
        for b in links:
            for n in (b["a"], b["b"]):
                lsum[n] = lsum.get(n, 0.0) + b["values"]["beamSpring"]
        cap = min(((lk.get(n, 0) * m.get(n, 0) - (ksum[n] - lsum[n])) / lsum[n] for n in lsum
                   if lsum[n] > 0 and m.get(n)), default=1.0)
        cap = max(1.0, cap)
        f = 1.0
        after = new
        for _ in range(6):
            f = min(cap, f * max(base[k] / after[k] for k in base))
            after = corner_stiffness(model, configured, w, nodes, bl, springs, {id(b): f for b in links})
            if not after or max(base[k] / after[k] for k in base) <= 1 + tolerance / 2 or f >= cap:
                break
        row.update(after=after, factor=round(f, 3), capped=f >= cap and max(base[k] / after[k] for k in base) > 1 + tolerance)
        for b in links:
            key = (b["part"], b["row"])
            out[key] = max(out.get(key, 1.0), f)
    return out, report


def follows_wheel(model, configured, wheel, bl=None, springs=None, share=0.2):
    """The suspension nodes that move with a wheel (hub, strut bottom, the anti-roll bar's link): pushed
    up in the static solve, those that move at least this share of its travel. The wheel's own nodes
    are left out; the body side (subframe, pivots, strut top) stays still."""
    geo = configured["geometry"]
    nodes, owner = geo["nodes"], geo.get("parts") or {}
    if wheel["node1"] not in nodes or wheel["node2"] not in nodes:
        return set()
    bl = bl if bl is not None else beam_list(model, configured)
    springs = springs if springs is not None else values.springs_and_dampers(model, configured)
    measured = {(r["part"], r["row"]) for r in springs}
    load = [b for b in bl if (b["part"], b["row"]) not in measured and not NO_LOAD.search(b["type"])
            and b["values"].get("beamSpring", 0) > 0]
    axle = [wheel["node1"], wheel["node2"]]
    centre = [(nodes[axle[0]][i] + nodes[axle[1]][i]) / 2 for i in range(3)]
    A, idx, kmax = _system(nodes, owner, load, axle, [centre], slides=geo.get("slides"))
    if not kmax:
        return set()
    f = [0.0] * len(A)
    big = kmax * 1e3
    for n in axle:
        i = 3 * idx[n] + 2
        A[i][i] += big
        f[i] += big * STEP
    x = _solve(A, f)
    return {n for n in idx if n not in axle
            and math.sqrt(sum(x[3 * idx[n] + i] ** 2 for i in range(3))) >= share * STEP}


def corner_ratios(model, configured, wheels, bl=None, springs=None):
    """{(part, row): motion ratio} for the spring and damper beams at the wheels of one axle (pushed
    up together): the beam's length change over the wheel travel (positive: shortens as it goes up)."""
    geo = configured["geometry"]
    nodes, owner = geo["nodes"], geo.get("parts") or {}
    bl = bl if bl is not None else beam_list(model, configured)
    springs = springs if springs is not None else values.springs_and_dampers(model, configured)
    measured = {(r["part"], r["row"]): (r["a"], r["b"]) for r in springs}
    wheels = [w for w in wheels if w["node1"] in nodes and w["node2"] in nodes]
    if not wheels:
        return {}
    axle = [n for w in wheels for n in (w["node1"], w["node2"])]
    centres = [[(nodes[w["node1"]][i] + nodes[w["node2"]][i]) / 2 for i in range(3)] for w in wheels]

    load = [b for b in bl if (b["part"], b["row"]) not in measured and not NO_LOAD.search(b["type"])
            and b["values"].get("beamSpring", 0) > 0]
    A, idx, kmax = _system(nodes, owner, load, axle, centres)
    if not kmax:
        return {}
    f = [0.0] * len(A)
    big = kmax * 1e3                                      # the axle nodes: pushed up by STEP
    for n in axle:
        i = 3 * idx[n] + 2
        A[i][i] += big
        f[i] += big * STEP
    x = _solve(A, f)
    moved = {n: [nodes[n][i] + x[3 * idx[n] + i] for i in range(3)] for n in idx}
    out = {}
    for key, (a, c) in measured.items():
        if a not in idx and c not in idx:
            continue
        pa, pc = moved.get(a, nodes[a]), moved.get(c, nodes[c])
        out[key] = round((math.dist(nodes[a], nodes[c]) - math.dist(pa, pc)) / STEP, 4)
    return out


def motion_ratios(model, configured):
    """Per axle ("front", "rear"): {"spring": MR, "damper": MR}, the mean over the axle's spring and
    damper beams, and per beam {(part, row): MR}."""
    bl = beam_list(model, configured)
    springs = values.springs_and_dampers(model, configured)
    kinds = {(r["part"], r["row"]): (r["kind"], r["axle"]) for r in springs}
    per_beam, axles = {}, {}
    groups = {}
    for w in configured.get("wheels") or []:
        groups.setdefault(w["name"][:1], []).append(w)            # F, R (and M for a middle axle)
    for ws in groups.values():
        r = corner_ratios(model, configured, ws, bl, springs)
        per_beam.update(r)
        for key, mr in r.items():
            kind, axle = kinds[key]
            if abs(mr) > 1e-3:
                axles.setdefault(axle, {}).setdefault(kind, []).append(abs(mr))
    out = {axle: {k: round(sum(v) / len(v), 4) for k, v in d.items()} for axle, d in axles.items()}
    return out, per_beam


def motion_ratios_json(model, configured_json):
    return json.dumps(motion_ratios(model, json.loads(configured_json))[0])
