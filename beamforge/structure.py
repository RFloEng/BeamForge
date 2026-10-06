"""Which nodes of a vehicle are held, and which are free to move: a check on its structure.

Pure Python, standard library only (runs in Pyodide). A node that must stay where it is (a subframe corner, a
strut top, an arm pivot) has to be held in every direction: by at least three beams whose directions are
independent (not all in one plane, not all along one line). A node with fewer is free in one direction or more:
it is a mechanism (a hinge, a slide, a joint of a linkage), not a structure node, and where it was meant to be
structure it floats: the Z3's strut top, held by two beams, moved 71 mm per kN sideways.

held() gives each node's rank: the dimension of the directions of the beams that hold it (a beam counts when it
carries load: spring above zero, not a support or a limit that works one way only; a slide node across its rail
counts for the two directions across the rail). rank 3 is held; below it the node has 3 - rank free directions.
"""

import math

from . import beamng, kinematics, rigidity, values


def _rank(dirs, tol=0.17):
    """The number of independent directions in a list of unit vectors: Gram-Schmidt, a direction counting when
    what is left of it after the others is longer than tol (about 10 degrees off their span)."""
    basis = []
    for d in dirs:
        v = list(d)
        for b in basis:
            p = sum(x * y for x, y in zip(v, b))
            v = [x - p * y for x, y in zip(v, b)]
        n = math.sqrt(sum(x * x for x in v))
        if n > tol:
            basis.append([x / n for x in v])
        if len(basis) == 3:
            break
    return len(basis)


def held(model, configured, bl=None):
    """{node: {"rank", "beams", "part"}} for every node some beam holds."""
    geo = configured["geometry"]
    nodes, owner = geo["nodes"], geo.get("parts") or {}
    bl = bl if bl is not None else rigidity.beams(model, configured)
    dirs = {}
    for b in bl:
        if kinematics.NO_LOAD.search(b["type"]) and "BOUNDED" not in b["type"]:
            continue
        if not (b["values"].get("beamSpring", 0) > 0):
            continue
        a, c = b["a"], b["b"]
        if a not in nodes or c not in nodes:
            continue
        d = [nodes[c][i] - nodes[a][i] for i in range(3)]
        n = math.sqrt(sum(x * x for x in d))
        if n < 1e-6:
            continue
        u = [x / n for x in d]
        dirs.setdefault(a, []).append(u)
        dirs.setdefault(c, []).append([-x for x in u])
    # a slide node, held across its rail: two directions perpendicular to it (and its rail's ends hold the
    # rail's own directions through their beams)
    for sn, ra, rb, k in geo.get("slides") or []:
        if not k or sn not in nodes or ra not in nodes or rb not in nodes:
            continue
        t = [nodes[rb][i] - nodes[ra][i] for i in range(3)]
        n = math.sqrt(sum(x * x for x in t))
        if n < 1e-6:
            continue
        t = [x / n for x in t]
        ref = [1.0, 0.0, 0.0] if abs(t[0]) < 0.9 else [0.0, 1.0, 0.0]
        p = sum(x * y for x, y in zip(ref, t))
        e1 = [x - p * y for x, y in zip(ref, t)]
        m = math.sqrt(sum(x * x for x in e1))
        e1 = [x / m for x in e1]
        e2 = [t[1] * e1[2] - t[2] * e1[1], t[2] * e1[0] - t[0] * e1[2], t[0] * e1[1] - t[1] * e1[0]]
        dirs.setdefault(sn, []).extend([e1, e2])
    return {n: {"rank": _rank(d), "beams": len(d), "part": owner.get(n)} for n, d in dirs.items()}


def weak(model, configured, base=None, names=None, bl=None):
    """The nodes held in fewer than three directions: [(node, rank, beams, part)], worst first. With `base` (held()
    of the base vehicle) only those that the base held in three; names: a regex a node's name must match."""
    import re
    rx = re.compile(names) if names else None
    out = []
    for n, h in held(model, configured, bl).items():
        if h["rank"] >= 3 or (rx and not rx.search(n)):
            continue
        if base is not None and (base.get(n) or {}).get("rank", 0) < 3:
            continue
        out.append((n, h["rank"], h["beams"], h["part"]))
    return sorted(out, key=lambda t: (t[1], t[0]))
