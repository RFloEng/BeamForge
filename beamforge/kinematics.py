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
    near = {}
    for b in load:
        near.setdefault(b["a"], []).append(b)
        near.setdefault(b["b"], []).append(b)
    # the free nodes: the corner's suspension, reached from the axle through suspension beams
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
    f = [0.0] * size
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
        k = b["values"]["beamSpring"]
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
    if not kmax:
        return {}
    # a weak glue along the beams (to free and held neighbours, closer ones more): in a direction no beam
    # holds (a spring seat on the line of its arm, a flat plate), a node takes the motion of the nodes
    # it is beamed to, as it would on the part they make
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
