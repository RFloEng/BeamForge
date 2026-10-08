"""Jbeam structures from STEP skeletons: an assembly of wireframe parts, each part a rigid body.

Pure Python, standard library only (runs in Pyodide). See docs/study-step-and-full-editor.md, section 2.

The rule (agreed with the user, 2026-10-08)
  * A STEP file is an assembly. Each part is given by its lines (and points), and is a rigid body.
  * The end points of the lines are the nodes. A part gets the beams that make its nodes move as one: one line is one
    beam; two lines with three ends are three beams (the triangle); in general 3n - 6 independent beams for n nodes
    (the rank of the rigidity matrix), its own lines first, then the shortest braces that add stiffness.
  * Two parts with an end at the same place share that node: one node joining both parts. The number of nodes two
    parts share says the joint: 1 a ball joint, 2 a hinge about the line through them, 3 or more not on one line a
    weld (the parts are one rigid body, braced together).
  * A line on which another part's end lands is split there (the node joins that line's part).
  * A flat part (4 or more ends in one plane, within a tolerance) can fold out of its plane whatever its beams: it
    gets a helper node off the plane. A straight part with 3 or more ends bends at its middle: two helper nodes off
    the line. Helper nodes are not in the CAD; they are light and hidden.
  * Parts are named with the SVJ's canonical part names (upper_wishbone_fl, tie_rod_fr, spring_rl, upright_fl...,
    svj.part_role); any other name is structure (a frame, a subframe).

Frames
  The STEP is in its CAD's frame and unit. place() turns it into the SVJ frame (SAE J670: X forward, Y right, Z down,
  origin on the ground below the front axle centre) by 90 degree rotations about X, Y and Z and an offset; the
  structure is then placed on a BeamNG vehicle like an SVJ (svj.from_sae with the front axle line and ground).

What it reads of STEP (ISO 10303-21): CARTESIAN_POINT, POLYLINE, LINE with TRIMMED_CURVE, EDGE_CURVE between
VERTEX_POINTs on a line, COMPOSITE_CURVE, GEOMETRIC_(CURVE_)SET, edge loops; the products, their shape
representations and the assembly (NEXT_ASSEMBLY_USAGE_OCCURRENCE with its ITEM_DEFINED_TRANSFORMATION); the length
unit. Curves that are not straight (circles, splines) and solids are counted and reported, not read (yet).
"""

import json
import math
import re

from . import rigidity, svj as svjmod, tubes

TOL = 0.002              # m: ends closer than this are one node, a node closer than this to a line is on it
FLAT = 0.05              # a part thinner than this share of its size is flat (or straight): it gets helper nodes
HELPER = 0.5             # helper nodes sit this share of the part's size off its plane or line
QUALITY = (0.3, 0.05)    # a brace is taken in a first pass only if it adds this much new direction, then any that adds
NEAREST = 10             # in a body of more than 40 nodes, braces are looked for among each node's nearest
MIN_KG = 1.0             # a node weighs its share of the tubes, but no less than this (helper nodes: this)
BEAM = {"beamDeform": 80000, "beamStrength": "FLT_MAX"}   # every beam writes its own values (values())
TUBE = {"frame": "tube 40x2 steel_1018", "link": "tube 25x2 steel_1018"}   # a part's tube when nothing names one
_TUBE_NAME = re.compile(r"(sq)?tube_?(\d+(?:_\d+)?)x(\d+(?:_\d+)?)(?:_(steel_1018|steel_4130n|al_6061_t6))?")
ROLE_ORDER = {None: 0, "arm": 1, "hub": 2, "tie": 3, "strut": 4}   # the owner of a shared node: the part nearer the body


# ---------------------------------------------------------------- STEP text

_TOKEN = re.compile(r"\s*(?:(?P<str>'(?:[^']|'')*')|(?P<ref>#\d+)|(?P<enum>\.[A-Z0-9_]+\.)|"
                    r"(?P<num>[+-]?(?:\d+\.?\d*(?:[eE][+-]?\d+)?|\.\d+(?:[eE][+-]?\d+)?))|(?P<kw>[A-Z_][A-Z0-9_]*)|"
                    r"(?P<sym>[(),$*=;]))", re.S)


class _Typed:
    """A typed parameter inside a list (LENGTH_MEASURE(25.4)) or one entity of a complex entity."""
    __slots__ = ("name", "args")

    def __init__(self, name, args):
        self.name, self.args = name, args

    def __repr__(self):
        return f"{self.name}{self.args!r}"


def _tokens(text):
    pos, n = 0, len(text)
    while pos < n:
        m = _TOKEN.match(text, pos)
        if not m:
            if text[pos:].strip():
                raise ValueError(f"STEP: cannot read near {text[pos:pos + 40]!r}")
            return
        pos = m.end()
        kind = m.lastgroup
        yield kind, m.group(kind)


def _value(toks, i):
    kind, v = toks[i]
    if kind == "str":
        return v[1:-1].replace("''", "'"), i + 1
    if kind == "ref":
        return int(v[1:]), i + 1                              # a reference: an int; numbers are floats
    if kind == "enum":
        return v, i + 1
    if kind == "num":
        return float(v), i + 1
    if kind == "sym" and v in "$*":
        return None, i + 1
    if kind == "sym" and v == "(":
        return _list(toks, i + 1)
    if kind == "kw":                                         # a typed parameter: NAME(args)
        args, j = _list(toks, i + 2)
        return _Typed(v, args), j
    raise ValueError(f"STEP: unexpected {v!r}")


def _list(toks, i):
    out = []
    if toks[i] == ("sym", ")"):
        return out, i + 1
    while True:
        v, i = _value(toks, i)
        out.append(v)
        if toks[i] == ("sym", ","):
            i += 1
            continue
        if toks[i] == ("sym", ")"):
            return out, i + 1
        raise ValueError(f"STEP: expected , or ) not {toks[i][1]!r}")


def parse(text):
    """{id: (NAME, [args])} of the DATA section; a complex entity (several types at once) is ("(complex)", [_Typed...]).
    Strings, enums, references (ints), numbers (floats), None for $ and *, lists, typed parameters."""
    m = re.search(r"\bDATA\s*;(.*?)\bENDSEC\s*;", text, re.S)
    if not m:
        raise ValueError("STEP: no DATA section (is this an ISO 10303-21 .step / .stp file?)")
    body = re.sub(r"/\*.*?\*/", " ", m.group(1), flags=re.S)
    toks = list(_tokens(body))
    out, i = {}, 0
    while i < len(toks):
        kind, v = toks[i]
        if kind != "ref" or toks[i + 1] != ("sym", "="):
            raise ValueError(f"STEP: expected #id= not {v!r}")
        eid = int(v[1:])
        i += 2
        if toks[i] == ("sym", "("):                          # complex: (A(..) B(..) ...)
            parts, i = [], i + 1
            while toks[i] != ("sym", ")"):
                name = toks[i][1]
                args, i = _list(toks, i + 2)
                parts.append(_Typed(name, args))
            out[eid] = ("(complex)", parts)
            i += 1
        else:
            name = v = toks[i][1]
            args, i = _list(toks, i + 2)
            out[eid] = (name, args)
        if toks[i] != ("sym", ";"):
            raise ValueError("STEP: expected ;")
        i += 1
    return out


# ---------------------------------------------------------------- what the entities mean

def _types(e):
    """{TYPE: args} of an entity (a complex one gives several)."""
    name, args = e
    return {p.name: p.args for p in args} if name == "(complex)" else {name: args}


def _unit(ents):
    """Metres per unit of the file's length unit, and its name."""
    prefix = {".MILLI.": 0.001, ".CENTI.": 0.01, ".DECI.": 0.1, ".KILO.": 1000.0, ".MICRO.": 1e-6}
    for e in ents.values():
        t = _types(e)
        if "LENGTH_UNIT" not in t:
            continue
        if "SI_UNIT" in t:
            p, unit = t["SI_UNIT"][0], t["SI_UNIT"][1]
            if unit == ".METRE.":
                return prefix.get(p, 1.0), {0.001: "mm", 0.01: "cm", 1.0: "m"}.get(prefix.get(p, 1.0), "m")
        if "CONVERSION_BASED_UNIT" in t:
            name = str(t["CONVERSION_BASED_UNIT"][0]).lower()
            if "inch" in name:
                return 0.0254, "inch"
            if "foot" in name or "feet" in name:
                return 0.3048, "foot"
    return 0.001, "mm (assumed: the file names no length unit)"


def _point(ents, ref):
    e = ents.get(ref)
    if e and e[0] == "CARTESIAN_POINT":
        c = list(e[1][1]) + [0.0, 0.0]
        return [float(c[0]), float(c[1]), float(c[2])]
    if e and e[0] == "VERTEX_POINT":
        return _point(ents, e[1][1])
    return None


def _direction(ents, ref):
    e = ents.get(ref)
    if e and e[0] == "DIRECTION":
        c = list(e[1][1]) + [0.0, 0.0]
        n = math.sqrt(sum(x * x for x in c[:3])) or 1.0
        return [c[0] / n, c[1] / n, c[2] / n]
    return None


def _line_at(ents, ref, t):
    """The point at parameter t on a LINE(name, point, VECTOR(name, direction, magnitude))."""
    e = ents.get(ref)
    p = _point(ents, e[1][1])
    vec = ents.get(e[1][2])
    d = _direction(ents, vec[1][1])
    mag = vec[1][2] or 1.0
    return [p[i] + t * mag * d[i] for i in range(3)]


def _trim(ents, basis, sel):
    """A trim select of a TRIMMED_CURVE: a point (a reference) or a parameter (PARAMETER_VALUE(t))."""
    for s in sel:
        if isinstance(s, int) and _point(ents, s):
            return _point(ents, s)
    for s in sel:
        if isinstance(s, _Typed) and s.name == "PARAMETER_VALUE":
            return _line_at(ents, basis, s.args[0])
    return None


def _geometry(ents, ref, segs, points, skipped, seen):
    """Straight segments ([a, b], file units) and loose points of a representation item, recursively."""
    if ref in seen or ref not in ents:
        return
    seen.add(ref)
    name, args = ents[ref]
    if name == "POLYLINE":
        pts = [_point(ents, r) for r in args[1]]
        segs.extend([a, b] for a, b in zip(pts, pts[1:]) if a and b)
    elif name == "TRIMMED_CURVE":
        basis = args[1]
        if ents.get(basis, ("",))[0] == "LINE":
            a, b = _trim(ents, basis, args[2]), _trim(ents, basis, args[3])
            if a and b:
                segs.append([a, b])
        elif ents.get(basis, ("",))[0] == "POLYLINE":
            _geometry(ents, basis, segs, points, skipped, seen)
        else:
            skipped[ents.get(basis, ("?",))[0]] = skipped.get(ents.get(basis, ("?",))[0], 0) + 1
    elif name == "EDGE_CURVE":
        curve = ents.get(args[3], ("?",))[0]
        if curve in ("LINE", "POLYLINE"):
            a, b = _point(ents, args[1]), _point(ents, args[2])
            if a and b:
                segs.append([a, b])
        else:
            skipped[curve] = skipped.get(curve, 0) + 1
    elif name == "ORIENTED_EDGE":
        _geometry(ents, args[3], segs, points, skipped, seen)
    elif name in ("EDGE_LOOP", "PATH"):
        for r in args[1]:
            _geometry(ents, r, segs, points, skipped, seen)
    elif name == "COMPOSITE_CURVE":
        for r in args[1]:
            _geometry(ents, r, segs, points, skipped, seen)
    elif name == "COMPOSITE_CURVE_SEGMENT":
        _geometry(ents, args[2], segs, points, skipped, seen)
    elif name in ("GEOMETRIC_CURVE_SET", "GEOMETRIC_SET"):
        for r in args[1]:
            _geometry(ents, r, segs, points, skipped, seen)
    elif name == "CARTESIAN_POINT":
        points.append(_point(ents, ref))
    elif name == "VERTEX_POINT":
        points.append(_point(ents, ref))
    elif name in ("AXIS2_PLACEMENT_3D", "AXIS1_PLACEMENT"):
        pass
    elif name in ("LINE",):
        skipped["LINE (unbounded)"] = skipped.get("LINE (unbounded)", 0) + 1
    else:
        skipped[name] = skipped.get(name, 0) + 1


def _placement(ents, ref):
    """(origin, x, y, z) of an AXIS2_PLACEMENT_3D."""
    e = ents.get(ref)
    o = _point(ents, e[1][1]) or [0.0, 0.0, 0.0]
    z = _direction(ents, e[1][2]) if len(e[1]) > 2 and e[1][2] else [0.0, 0.0, 1.0]
    x = _direction(ents, e[1][3]) if len(e[1]) > 3 and e[1][3] else [1.0, 0.0, 0.0]
    d = sum(x[i] * z[i] for i in range(3))
    x = [x[i] - d * z[i] for i in range(3)]
    n = math.sqrt(sum(v * v for v in x)) or 1.0
    x = [v / n for v in x]
    y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]]
    return o, x, y, z


def _matrix(ents, idt):
    """The 3x4 transform of an ITEM_DEFINED_TRANSFORMATION: the child's frame (item 1) onto the parent's (item 2)."""
    args = ents[idt][1]
    o1, x1, y1, z1 = _placement(ents, args[2])
    o2, x2, y2, z2 = _placement(ents, args[3])
    a = [x1, y1, z1]                  # rows: the child frame's axes
    b = [x2, y2, z2]
    R = [[sum(b[k][i] * a[k][j] for k in range(3)) for j in range(3)] for i in range(3)]   # B^T A
    t = [o2[i] - sum(R[i][j] * o1[j] for j in range(3)) for i in range(3)]
    return [R[0] + [t[0]], R[1] + [t[1]], R[2] + [t[2]]]


def _mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) + (A[i][3] if j == 3 else 0.0) for j in range(4)] for i in range(3)]


def _apply(M, p):
    return [M[i][0] * p[0] + M[i][1] * p[1] + M[i][2] * p[2] + M[i][3] for i in range(3)]


IDENTITY = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]]
REPS = ("SHAPE_REPRESENTATION", "GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION", "EDGE_BASED_WIREFRAME_SHAPE_REPRESENTATION",
        "GEOMETRICALLY_BOUNDED_SURFACE_SHAPE_REPRESENTATION", "ADVANCED_BREP_SHAPE_REPRESENTATION",
        "MANIFOLD_SURFACE_SHAPE_REPRESENTATION")


def _clean(name):
    """A part name as the SVJ names it: lower case, an instance number from the CAD (":1", ".2", "<1>") dropped."""
    s = re.sub(r"(:\d+|<\d+>|\.\d+)$", "", str(name or "").strip())
    return re.sub(r"[^a-z0-9_]+", "_", s.lower()).strip("_")


def read(text):
    """The parts of a STEP skeleton: {"parts": [{"name", "lines": [[a, b]], "points": [p]}] (file units, in the
    assembly's top frame), "unit": metres per unit, "unit_name", "skipped": {entity: count}, "notes"}."""
    ents = parse(text)
    unit, unit_name = _unit(ents)
    by = {}
    for eid, (name, args) in ents.items():
        by.setdefault(name, []).append(eid)
    # products and their definitions
    product = {e: ents[e][1] for e in by.get("PRODUCT", [])}
    form = {e: ents[e][1][2] for n in ("PRODUCT_DEFINITION_FORMATION", "PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE")
            for e in by.get(n, [])}
    pd_product = {e: form.get(ents[e][1][2]) for n in ("PRODUCT_DEFINITION", "PRODUCT_DEFINITION_WITH_ASSOCIATED_DOCUMENTS")
                  for e in by.get(n, [])}
    pds = {e: ents[e][1][2] for e in by.get("PRODUCT_DEFINITION_SHAPE", [])}             # pds -> pd or nauo
    reps_of = {}                                                                          # pd -> [rep]
    for e in by.get("SHAPE_DEFINITION_REPRESENTATION", []):
        d, rep = ents[e][1][0], ents[e][1][1]
        reps_of.setdefault(pds.get(d), []).append(rep)
    linked = {}                                                                           # rep <-> rep (SRR)
    for eid, e in ents.items():
        t = _types(e)
        if "SHAPE_REPRESENTATION_RELATIONSHIP" in t and "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION" not in t:
            r = t.get("REPRESENTATION_RELATIONSHIP") or t["SHAPE_REPRESENTATION_RELATIONSHIP"]
            if len(r) >= 4:
                linked.setdefault(r[2], []).append(r[3])
                linked.setdefault(r[3], []).append(r[2])
    # the assembly: NAUO (parent pd, child pd) and its placement
    nauo = {e: ents[e][1] for e in by.get("NEXT_ASSEMBLY_USAGE_OCCURRENCE", [])}
    place = {}
    for e in by.get("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION", []):
        rel, d = ents[e][1][0], ents[e][1][1]
        t = _types(ents.get(rel, ("", [])))
        idt = (t.get("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION") or [None])[0]
        if pds.get(d) in nauo and idt in ents:
            place[pds[d]] = _matrix(ents, idt)
    children = {}
    for n, a in nauo.items():
        children.setdefault(a[3], []).append(n)

    skipped, parts, notes = {}, [], []

    def geometry(pd):
        segs, points, seen = [], [], set()
        todo = list(reps_of.get(pd, []))
        reps = set()
        while todo:
            r = todo.pop()
            if r in reps:
                continue
            reps.add(r)
            todo.extend(linked.get(r, []))
        for r in reps:
            e = ents.get(r)
            if e and e[0] in REPS:
                for item in e[1][1]:
                    _geometry(ents, item, segs, points, skipped, seen)
                if e[0] in ("ADVANCED_BREP_SHAPE_REPRESENTATION", "MANIFOLD_SURFACE_SHAPE_REPRESENTATION"):
                    skipped["solids and surfaces"] = skipped.get("solids and surfaces", 0) + 1
        return segs, points

    def walk(pd, M, name, depth):
        segs, points = geometry(pd)
        if segs or points:
            parts.append({"name": name, "lines": [[_apply(M, a), _apply(M, b)] for a, b in segs],
                          "points": [_apply(M, p) for p in points]})
        for n in children.get(pd, []):
            child = nauo[n][4]
            prod = product.get(pd_product.get(child)) or ["", ""]
            inst = nauo[n][1] or nauo[n][0]
            cname = _clean(inst) if _clean(inst) and not re.fullmatch(r"(nauo)?\d*", _clean(inst)) else _clean(prod[1] or prod[0])
            if depth < 32:
                walk(child, _mul(M, place.get(n, IDENTITY)), cname, depth + 1)

    roots = [pd for pd in pd_product if pd not in {a[4] for a in nauo.values()}]
    for pd in roots:
        prod = product.get(pd_product.get(pd)) or ["", ""]
        walk(pd, IDENTITY, _clean(prod[1] or prod[0]), 0)
    if not parts:                       # no products: each curve set or wireframe representation is a part
        for n in ("GEOMETRIC_CURVE_SET", "GEOMETRIC_SET") + REPS:
            for e in by.get(n, []):
                segs, points = [], []
                for item in ents[e][1][1]:
                    _geometry(ents, item, segs, points, skipped, set())
                if segs or points:
                    parts.append({"name": _clean(ents[e][1][0]) or f"part_{e}", "lines": segs, "points": points})
        if parts:
            notes.append("no products in the file: each curve set is a part")
    # names unique
    seen = {}
    for p in parts:
        base = p["name"] or "part"
        seen[base] = seen.get(base, 0) + 1
        if seen[base] > 1:
            p["name"] = f"{base}_{seen[base]}"
    return {"parts": parts, "unit": unit, "unit_name": unit_name, "skipped": skipped, "notes": notes}


# ---------------------------------------------------------------- CAD frame -> SVJ frame

def _rot(axis, quarter):
    c, s = [1, 0, -1, 0][quarter % 4], [0, 1, 0, -1][quarter % 4]
    if axis == 0:
        return [[1, 0, 0], [0, c, -s], [0, s, c]]
    if axis == 1:
        return [[c, 0, s], [0, 1, 0], [-s, 0, c]]
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def place_fn(unit, rot=(0, 0, 0), offset=(0.0, 0.0, 0.0)):
    """A function taking a CAD point (file units) to the SVJ frame (metres): scaled by the unit, turned by rot
    (degrees about X, then Y, then Z, multiples of 90) and moved by offset (m, SVJ frame)."""
    R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    for ax in range(3):
        q = int(round((rot[ax] or 0) / 90.0)) % 4
        if q:
            S = _rot(ax, q)
            R = [[sum(S[i][k] * R[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

    def f(p):
        q = [p[i] * unit for i in range(3)]
        return [sum(R[i][j] * q[j] for j in range(3)) + offset[i] for i in range(3)]
    return f


# ---------------------------------------------------------------- small linear algebra

def _shape(pts):
    """(centroid, [(sd, axis)] largest first): the principal spreads of a point set (Jacobi on the covariance)."""
    n = len(pts)
    c = [sum(p[i] for p in pts) / n for i in range(3)]
    A = [[sum((p[i] - c[i]) * (p[j] - c[j]) for p in pts) / n for j in range(3)] for i in range(3)]
    V = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    for _ in range(50):
        p, q = max(((0, 1), (0, 2), (1, 2)), key=lambda t: abs(A[t[0]][t[1]]))
        if abs(A[p][q]) < 1e-15:
            break
        th = 0.5 * math.atan2(2 * A[p][q], A[q][q] - A[p][p])
        cs, sn = math.cos(th), math.sin(th)
        for k in range(3):
            akp, akq = A[k][p], A[k][q]
            A[k][p], A[k][q] = cs * akp - sn * akq, sn * akp + cs * akq
        for k in range(3):
            apk, aqk = A[p][k], A[q][k]
            A[p][k], A[q][k] = cs * apk - sn * aqk, sn * apk + cs * aqk
        for k in range(3):
            vkp, vkq = V[k][p], V[k][q]
            V[k][p], V[k][q] = cs * vkp - sn * vkq, sn * vkp + cs * vkq
    out = [(math.sqrt(max(0.0, A[i][i])), [V[0][i], V[1][i], V[2][i]]) for i in range(3)]
    return c, sorted(out, key=lambda t: -t[0])


class _Basis:
    """Rows of a rigidity matrix kept orthonormal (modified Gram-Schmidt), sparse: {column: value}."""

    def __init__(self):
        self.rows = []

    def residual(self, row):
        v = dict(row)
        for b in self.rows:
            d = sum(v.get(k, 0.0) * x for k, x in b.items())
            if d:
                for k, x in b.items():
                    v[k] = v.get(k, 0.0) - d * x
        return v

    def add(self, row, tol):
        """Add a row if what is new in it is longer than tol (rows are unit); returns that length."""
        v = self.residual(row)
        n = math.sqrt(sum(x * x for x in v.values()))
        if n > tol:
            self.rows.append({k: x / n for k, x in v.items() if abs(x) > 1e-14})
        return n


def _row(pos, ia, ib, a, b):
    d = [pos[b][i] - pos[a][i] for i in range(3)]
    n = math.sqrt(sum(x * x for x in d))
    if n < 1e-12:
        return None
    u = [x / n for x in d]
    r = {}
    for i in range(3):
        r[3 * ia + i] = -u[i] / math.sqrt(2)
        r[3 * ib + i] = u[i] / math.sqrt(2)
    return r


# ---------------------------------------------------------------- build

def kind_of(name, kinds=None):
    """"frame" (welded tubes: members and bending beams) or "link" (a rigid body: its lines braced stiff): as the user
    set it, else a link for a part with a suspension role (svj.part_role), a frame for anything else."""
    return (kinds or {}).get(name) or ("link" if svjmod.part_role(name) else "frame")


def tube_of(name, kind, given=None):
    """The section text of a part's tubes: the user's, else one written in the part's name (frame_tube_45x2_5,
    arm_sqtube_30x2_al_6061_t6), else TUBE by kind."""
    if (given or {}).get(name):
        return given[name]
    m = _TUBE_NAME.search(name)
    if m:
        return f"{'sqtube' if m.group(1) else 'tube'} {m.group(2).replace('_', '.')}x{m.group(3).replace('_', '.')} {m.group(4) or 'steel_1018'}"
    return TUBE[kind]


def build(parts, unit=0.001, rot=(0, 0, 0), offset=(0.0, 0.0, 0.0), tol=TOL, flat=FLAT, kinds=None):
    """The jbeam structure of a skeleton (read()'s parts, file units). Returns {"nodes": {id: {"pos" (SVJ, m), "parts",
    "owner", "helper"}}, "beams": [{"a", "b", "part", "kind": "line" | "bend" | "brace" | "helper"}], "parts": [{"name",
    "role", "kind", "body", "nodes", "lines"}], "bodies": [{"parts", "kind", "nodes", "beams", "rank", "need", "helpers",
    "shape"}], "joints": [{"parts", "nodes", "type"}], "report": {...}}. kinds: {part: "frame" | "link"} (kind_of).
    A frame's welded corners get bending beams (tubes.corners, FBeam 8.10) before any brace for rigidity."""
    f = place_fn(unit, rot, offset)
    pos, ids = [], []                                   # merged nodes, by index

    def node(p):
        for i, q in enumerate(pos):
            if math.dist(p, q) <= tol:
                return i
        pos.append(p)
        return len(pos) - 1
    plist = []
    merged = 0
    for p in parts:
        segs = []
        for a, b in p["lines"]:
            ia, ib = node(f(a)), node(f(b))
            if ia != ib:
                segs.append((ia, ib))
        pts = {node(f(q)) for q in p.get("points") or []}
        plist.append({"name": p["name"], "role": svjmod.part_role(p["name"]), "kind": kind_of(p["name"], kinds),
                      "segs": segs, "refs": pts})
    ends = sum(2 * len(p["lines"]) for p in parts)
    merged = ends - len({n for p in plist for s in p["segs"] for n in s})
    # split lines where a node of the structure lands on them
    used = {n for p in plist for s in p["segs"] for n in s}
    splits = 0
    for p in plist:
        out = []
        for a, b in p["segs"]:
            A, B = pos[a], pos[b]
            d = [B[i] - A[i] for i in range(3)]
            L2 = sum(x * x for x in d)
            on = []
            for n in used:
                if n in (a, b):
                    continue
                t = sum((pos[n][i] - A[i]) * d[i] for i in range(3)) / L2
                if 0 < t < 1 and math.dist(pos[n], [A[i] + t * d[i] for i in range(3)]) <= tol:
                    on.append((t, n))
            chain = [a] + [n for _, n in sorted(on)] + [b]
            splits += len(on)
            out.extend(zip(chain, chain[1:]))
        p["segs"] = out
        p["nodes"] = sorted({n for s in out for n in s})
    # welded parts: sharing 3 or more nodes not on one line make one rigid body
    parent = list(range(len(plist)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    for i in range(len(plist)):
        for j in range(i + 1, len(plist)):
            shared = sorted(set(plist[i]["nodes"]) & set(plist[j]["nodes"]))
            if len(shared) >= 3 and _shape([pos[n] for n in shared])[1][1][0] > 1e-3:
                parent[find(i)] = find(j)
    groups = {}
    for i in range(len(plist)):
        groups.setdefault(find(i), []).append(i)
    beams, bodies = [], []
    for members in groups.values():
        nodes = sorted({n for i in members for n in plist[i]["nodes"]})
        main = max(members, key=lambda i: len(plist[i]["nodes"]))
        body = {"parts": [plist[i]["name"] for i in members], "nodes": list(nodes), "helpers": 0, "beams": 0, "shape": "solid",
                "kind": "frame" if any(plist[i]["kind"] == "frame" for i in members) else "link"}
        for i in members:
            plist[i]["body"] = len(bodies)
            for a, b in plist[i]["segs"]:
                beams.append({"a": a, "b": b, "part": plist[i]["name"], "kind": "line"})
        lines = {tuple(sorted((x["a"], x["b"]))) for x in beams if x["part"] in body["parts"]}
        if body["kind"] == "frame":                   # welded corners: a bending beam between the far ends of two tubes
            mem = [{"id": k, "a": x["a"], "b": x["b"]} for k, x in enumerate(beams)
                   if x["kind"] == "line" and x["part"] in body["parts"] and plist[next(i for i in members if plist[i]["name"] == x["part"])]["kind"] == "frame"]
            for j, ma, mc, a, c in tubes.corners(pos, mem):
                key = tuple(sorted((a, c)))
                if key not in lines and tubes._lever(pos[j], pos[a], pos[c]) >= tubes.MIN_LEVER:
                    lines.add(key)
                    beams.append({"a": key[0], "b": key[1], "part": plist[main]["name"], "kind": "bend"})
        if len(nodes) >= 3:
            c, axes = _shape([pos[n] for n in nodes])
            size = max(math.dist(pos[n], c) for n in nodes)
            extra = []
            if axes[1][0] < flat * axes[0][0]:                       # straight: two helpers off the line
                body["shape"] = "straight"
                extra = [[c[i] + HELPER * size * axes[1][1][i] for i in range(3)], [c[i] + HELPER * size * axes[2][1][i] for i in range(3)]]
            elif len(nodes) >= 4 and axes[2][0] < flat * axes[0][0]:   # flat: one helper off the plane
                body["shape"] = "flat"
                extra = [[c[i] + HELPER * size * axes[2][1][i] for i in range(3)]]
            for p in extra:
                pos.append(p)
                nodes.append(len(pos) - 1)
                ids.append(len(pos) - 1)
                body["helpers"] += 1
            body["nodes"] = list(nodes)
            # brace: its lines first, then candidate pairs, shortest first, taken when they add a new direction
            idx = {n: k for k, n in enumerate(nodes)}
            basis = _Basis()
            for a, b in sorted(lines):
                r = _row(pos, idx[a], idx[b], a, b)
                if r:
                    basis.add(r, 1e-9)
            need = 3 * len(nodes) - 6
            if len(nodes) > 40:
                cand = set()
                for a in nodes:
                    near = sorted((math.dist(pos[a], pos[b]), b) for b in nodes if b != a)[:NEAREST]
                    cand.update(tuple(sorted((a, b))) for _, b in near)
            else:
                cand = {(a, b) for k, a in enumerate(nodes) for b in nodes[k + 1:]}
            cand = sorted((math.dist(pos[a], pos[b]), a, b) for a, b in cand if (a, b) not in lines)
            helper_nodes = set(nodes[len(nodes) - body["helpers"]:]) if body["helpers"] else set()
            for q in QUALITY:
                for _, a, b in cand:
                    if len(basis.rows) >= need:
                        break
                    r = _row(pos, idx[a], idx[b], a, b)
                    if r and (a, b) not in lines and basis.add(r, q) > q:
                        lines.add((a, b))
                        kind = "helper" if a in helper_nodes or b in helper_nodes else "brace"
                        beams.append({"a": a, "b": b, "part": plist[main]["name"], "kind": kind})
            body["rank"], body["need"] = len(basis.rows), need
        else:
            body["rank"], body["need"] = len(lines), max(0, len(nodes) - 1)
        body["beams"] = sum(1 for x in beams if x["part"] in body["parts"])
        body["bending"] = sum(1 for x in beams if x["part"] in body["parts"] and x["kind"] == "bend")
        bodies.append(body)
    # joints between bodies: the nodes they share
    joints = []
    for i in range(len(bodies)):
        for j in range(i + 1, len(bodies)):
            shared = sorted(set(bodies[i]["nodes"]) & set(bodies[j]["nodes"]))
            if not shared:
                continue
            typ = "ball" if len(shared) == 1 else "hinge" if len(shared) == 2 or _shape([pos[n] for n in shared])[1][1][0] < 1e-3 else "weld"
            joints.append({"bodies": [i, j], "parts": [bodies[i]["parts"], bodies[j]["parts"]], "nodes": shared, "type": typ})
    # node names, owners (the part nearer the body), the whole assembly's free motions
    names = {}
    order = {p["name"]: (ROLE_ORDER.get(p["role"], 0), k) for k, p in enumerate(plist)}
    out_nodes = {}
    helpers = set(ids)
    k = 0
    for n in range(len(pos)):
        in_parts = [p["name"] for p in plist if n in p["nodes"]]
        is_ref = not in_parts and not any(n in b["nodes"] for b in bodies) and any(n in p["refs"] for p in plist)
        if not in_parts and n not in helpers and not is_ref:
            continue
        owner = min(in_parts, key=lambda s: order[s]) if in_parts else \
            next((bd["parts"][0] for bd in bodies if n in bd["nodes"]), None)
        if is_ref:
            owner = next(p["name"] for p in plist if n in p["refs"])
        names[n] = f"skh{k}" if n in helpers else f"skr{k}" if is_ref else f"sk{k}"
        k += 1
        out_nodes[names[n]] = {"pos": [round(x, 6) for x in pos[n]], "parts": in_parts, "owner": owner,
                               "helper": n in helpers, "reference": is_ref}
    full = _Basis()
    allidx = {n: i for i, n in enumerate(sorted(n for n in names if not out_nodes[names[n]]["reference"]))}
    for x in beams:
        r = _row(pos, allidx[x["a"]], allidx[x["b"]], x["a"], x["b"])
        if r:
            full.add(r, 1e-7)
    free = 3 * len(allidx) - len(full.rows)
    report = {"parts": len(plist), "lines": sum(len(p["lines"]) for p in parts), "ends_merged": merged, "splits": splits,
              "nodes": len(allidx), "helper_nodes": len(helpers), "beams": len(beams),
              "braces": sum(1 for x in beams if x["kind"] in ("brace", "helper")),
              "bending": sum(1 for x in beams if x["kind"] == "bend"), "bodies": len(bodies),
              "joints": {t: sum(1 for j in joints if j["type"] == t) for t in ("ball", "hinge", "weld")},
              "free_motions": free, "mechanism": free - 6 if len(allidx) >= 3 else None,
              "not_rigid": [b["parts"] for b in bodies if b["rank"] < b["need"]]}
    return {"nodes": out_nodes,
            "beams": [{"a": names[x["a"]], "b": names[x["b"]], "part": x["part"], "kind": x["kind"]} for x in beams],
            "parts": [{"name": p["name"], "role": p["role"], "kind": p["kind"], "body": p.get("body"), "nodes": [names[n] for n in p["nodes"]],
                       "lines": len(p["segs"])} for p in plist],
            "bodies": [dict(b, nodes=[names[n] for n in b["nodes"]]) for b in bodies],
            "joints": [dict(j, nodes=[names[n] for n in j["nodes"]]) for j in joints],
            "report": report}


def values(result, tubes_given=None, min_kg=MIN_KG):
    """Node weights and every beam's values, from the parts' tubes (tube_of; FBeam's member rule, tubes.py) and stable
    by construction (rigidity.beam_values):
      * a line is a member tube: stiffness the smaller of E A / L and the stable value (at most the vanilla 90th
        percentile per beam); beamDeform from yield or buckling over its unbraced run, beamStrength from su A;
      * its end nodes carry its mass (half each, plus tubes.JOINT), no node lighter than min_kg;
      * a bending beam (a welded corner) gets the corner's budget, scaled like the members around it, and the corner's
        collapse load; braces for rigidity and beams to helper nodes the stable value at the vanilla median;
      * damping follows the stiffness at the vanilla ratio; no node past the vanilla 90th percentile in total.
    Members go first, then bending beams, then braces. Returns ({node: kg}, [{"beamSpring", "beamDamp", "beamDeform",
    "beamStrength", "real_k", "limited"}] by beam, {node: (k index, c index, band)}, {part: {"tube", "kind", "mass",
    "members", "buckling"}})."""
    P = {n: v["pos"] for n, v in result["nodes"].items()}
    kind = {p["name"]: p.get("kind") or kind_of(p["name"]) for p in result["parts"]}
    sec = {}
    for p in result["parts"]:
        try:
            sec[p["name"]] = tubes.section(tube_of(p["name"], kind[p["name"]], tubes_given))
        except tubes.SectionError:
            sec[p["name"]] = tubes.section(TUBE[kind[p["name"]]])
    B = result["beams"]
    members = [{"id": i, "a": b["a"], "b": b["b"]} for i, b in enumerate(B) if b["kind"] == "line"]
    run = tubes.buckling_lengths(P, [m for m in members if kind[B[m["id"]]["part"]] == "frame"])
    lim = {m["id"]: tubes.limits(sec[B[m["id"]]["part"]], math.dist(P[m["a"]], P[m["b"]]), run.get(m["id"])) for m in members}
    kg = {n: 0.0 for n, v in result["nodes"].items() if not v["reference"]}
    for m in members:
        for n in (m["a"], m["b"]):
            kg[n] += lim[m["id"]]["mass"] / 2 * (1 + tubes.JOINT)
    kg = {n: round(max(min_kg, x), 3) for n, x in kg.items()}
    ks, cs, out = {n: 0.0 for n in kg}, {n: 0.0 for n in kg}, [None] * len(B)

    def put(i, v, deform, strength, real):
        a, b = B[i]["a"], B[i]["b"]
        ks[a] += v["beamSpring"]
        ks[b] += v["beamSpring"]
        cs[a] += v["beamDamp"]
        cs[b] += v["beamDamp"]
        out[i] = {"beamSpring": v["beamSpring"], "beamDamp": v["beamDamp"], "beamDeform": deform, "beamStrength": strength,
                  "real_k": real, "limited": v["limited"]}
    for m in members:                                   # member tubes
        a, b, L = m["a"], m["b"], lim[m["id"]]
        v = rigidity.beam_values(kg[a], kg[b], (ks[a], ks[b]), (cs[a], cs[b]), aim="p90", want=L["k"])
        put(m["id"], v, round(L["beamDeform"]), round(L["beamStrength"]), round(L["k"]))
    frame_members = [m for m in members if kind[B[m["id"]]["part"]] == "frame"]
    budgets = tubes.bending_budgets(P, frame_members, lambda m: sec[B[m["id"]]["part"]]) if frame_members else {}
    for i, x in enumerate(B):                           # bending beams: the corner's budget, scaled like its members
        if x["kind"] != "bend":
            continue
        e = budgets.get(tuple(sorted((x["a"], x["b"]))))
        a, b = x["a"], x["b"]
        if e:
            s = min(out[k]["beamSpring"] / lim[k]["k"] for k in e["members"] if out[k])
            v = rigidity.beam_values(kg[a], kg[b], (ks[a], ks[b]), (cs[a], cs[b]), aim="p90", want=e["k"] * s)
            put(i, v, round(e["F"]), round(tubes.STRENGTH_FACTOR * e["F"]), round(e["k"]))
        else:
            put(i, rigidity.beam_values(kg[a], kg[b], (ks[a], ks[b]), (cs[a], cs[b])), BEAM["beamDeform"], BEAM["beamStrength"], None)
    for i, x in enumerate(B):                           # braces for rigidity, beams to helper nodes
        if out[i] is None:
            a, b = x["a"], x["b"]
            put(i, rigidity.beam_values(kg[a], kg[b], (ks[a], ks[b]), (cs[a], cs[b])), BEAM["beamDeform"], BEAM["beamStrength"], None)
    parts = {}
    for p in result["parts"]:
        mine = [m for m in members if B[m["id"]]["part"] == p["name"]]
        parts[p["name"]] = {"tube": sec[p["name"]]["text"], "kind": kind[p["name"]],
                            "mass": round(sum(lim[m["id"]]["mass"] for m in mine), 3), "members": len(mine),
                            "buckling": sum(1 for m in mine if lim[m["id"]]["governs"] == "buckling")}
    return kg, out, {n: rigidity.node_index(kg[n], ks[n], cs[n]) for n in kg}, parts


def to_jbeam(result, prefix="skeleton", yf=0.0, ground=0.0, tubes_given=None, min_kg=MIN_KG):
    """The structure as jbeam parts, one per STEP part ({prefix}_{part}): the nodes it owns (placed on a vehicle like an
    SVJ: svj.from_sae with its front axle line yf and ground) with their weights, its beams (members, bending beams,
    braces) with their values (values()), a node group per part."""
    kg, bv, _, _ = values(result, tubes_given, min_kg)
    out = {}
    for p in result["parts"]:
        name = f"{prefix}_{p['name']}"
        own = [n for n, v in result["nodes"].items() if v["owner"] == p["name"]]
        rows = [["id", "posX", "posY", "posZ"], {"group": name}]
        for n in own:
            x, y, z = svjmod.from_sae(result["nodes"][n]["pos"], yf, ground)
            rows.append([n, round(x, 4), round(y, 4), round(z, 4), {"nodeWeight": kg[n]}])
        beams = [["id1:", "id2:"], dict(BEAM)]
        beams += [[b["a"], b["b"], {k: bv[i][k] for k in ("beamSpring", "beamDamp", "beamDeform", "beamStrength")}]
                  for i, b in enumerate(result["beams"]) if b["part"] == p["name"]]
        out[name] = {"information": {"authors": "BeamForge", "name": p["name"].replace("_", " ")}, "slotType": name,
                     "nodes": rows, "beams": beams}
    return out


def _built(text, opts):
    r = read(text)
    res = build(r["parts"], opts.get("unit") or r["unit"], opts.get("rot") or (0, 0, 0), opts.get("offset") or (0.0, 0.0, 0.0),
                opts.get("tol") or TOL, opts.get("flat") or FLAT, opts.get("kinds"))
    return r, res


def build_json(text, opts_json="{}"):
    """For the editor: STEP text and {"rot", "offset", "unit", "tol", "flat", "kinds": {part: "frame" | "link"},
    "tubes": {part: section text}, "min_kg", "yf", "ground"} in; the build out (JSON) with each node's BeamNG position
    ("bng"), weight and stability band, each beam's values, each part's tube and mass, the file's unit and what was not
    read."""
    opts = json.loads(opts_json or "{}")
    r, res = _built(text, opts)
    yf, ground = opts.get("yf") or 0.0, opts.get("ground") or 0.0
    for v in res["nodes"].values():
        v["bng"] = [round(x, 5) for x in svjmod.from_sae(v["pos"], yf, ground)]
    kg, bv, idx, parts = values(res, opts.get("tubes"), opts.get("min_kg") or MIN_KG)
    for i, b in enumerate(res["beams"]):
        b.update(bv[i])
    for n, (ki, ci, band) in idx.items():
        res["nodes"][n].update(kg=kg[n], k_index=round(ki, 3), c_index=round(ci, 3), band=band)
    for p in res["parts"]:
        p.update(parts[p["name"]])
    res["report"]["bands"] = {x: sum(1 for t in idx.values() if t[2] == x) for x in ("ok", "high", "extreme", "beyond")}
    res["report"]["tube_mass"] = round(sum(x["mass"] for x in parts.values()), 2)
    res["report"]["node_mass"] = round(sum(kg.values()), 2)
    res.update(unit=r["unit"], unit_name=r["unit_name"], skipped=r["skipped"], notes=r["notes"])
    return json.dumps(res)


def jbeam_json(text, opts_json="{}", prefix="skeleton"):
    """For the editor: the jbeam of a STEP skeleton (text), with the same options as build_json."""
    opts = json.loads(opts_json or "{}")
    _, res = _built(text, opts)
    return json.dumps(to_jbeam(res, prefix, opts.get("yf") or 0.0, opts.get("ground") or 0.0, opts.get("tubes"),
                               opts.get("min_kg") or MIN_KG), indent=1)


# ---------------------------------------------------------------- writing (tests, and skeletons made by BeamForge)

def write(parts, unit="mm", placements=None):
    """A STEP (AP214) text of a skeleton assembly: parts [{"name", "lines": [[a, b]], "points": [p]}] (in `unit`), each a
    product with a wireframe representation (polylines), all under one root assembly. placements: {part name:
    (origin, x axis, z axis)} where a part is drawn in its own frame and placed in the assembly (default: in place)."""
    ents, n = [], [0]

    def add(s):
        n[0] += 1
        ents.append(f"#{n[0]}={s};")
        return n[0]
    f = lambda v: f"{float(v):.6f}".rstrip("0").rstrip(".") + "." if float(v) == int(float(v)) else f"{float(v):.6f}"   # noqa: E731
    pt = lambda p: add(f"CARTESIAN_POINT('',({f(p[0])},{f(p[1])},{f(p[2])}))")                                            # noqa: E731
    dr = lambda d: add(f"DIRECTION('',({f(d[0])},{f(d[1])},{f(d[2])}))")                                                  # noqa: E731
    prefix = {"mm": ".MILLI.", "m": "$", "cm": ".CENTI."}[unit]
    u = add(f"(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT({prefix},.METRE.))")
    pa = add("(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.))")
    sa = add("(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT())")
    unc = add(f"UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-05),#{u},'distance_accuracy_value','')")
    ctx = add(f"(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{unc}))"
              f"GLOBAL_UNIT_ASSIGNED_CONTEXT((#{u},#{pa},#{sa}))REPRESENTATION_CONTEXT('',''))")
    app = add("APPLICATION_CONTEXT('automotive design')")
    pctx = add(f"PRODUCT_CONTEXT('',#{app},'mechanical')")
    dctx = add(f"PRODUCT_DEFINITION_CONTEXT('part definition',#{app},'design')")
    origin = add(f"AXIS2_PLACEMENT_3D('',#{pt([0, 0, 0])},#{dr([0, 0, 1])},#{dr([1, 0, 0])})")

    def product(name):
        p = add(f"PRODUCT('{name}','{name}','',(#{pctx}))")
        fm = add(f"PRODUCT_DEFINITION_FORMATION('','',#{p})")
        pd = add(f"PRODUCT_DEFINITION('design','',#{fm},#{dctx})")
        sh = add(f"PRODUCT_DEFINITION_SHAPE('','',#{pd})")
        return pd, sh
    root_pd, root_sh = product("assembly")
    root_rep = add(f"SHAPE_REPRESENTATION('assembly',(#{origin}),#{ctx})")
    add(f"SHAPE_DEFINITION_REPRESENTATION(#{root_sh},#{root_rep})")
    for k, part in enumerate(parts):
        pd, sh = product(part["name"])
        items = []
        for a, b in part.get("lines") or []:
            items.append(add(f"POLYLINE('',(#{pt(a)},#{pt(b)}))"))
        for p in part.get("points") or []:
            items.append(pt(p))
        cs = add(f"GEOMETRIC_CURVE_SET('{part['name']}',({','.join(f'#{i}' for i in items)}))")
        local = add(f"AXIS2_PLACEMENT_3D('',#{pt([0, 0, 0])},#{dr([0, 0, 1])},#{dr([1, 0, 0])})")
        rep = add(f"GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION('{part['name']}',(#{cs},#{local}),#{ctx})")
        add(f"SHAPE_DEFINITION_REPRESENTATION(#{sh},#{rep})")
        o, x, z = (placements or {}).get(part["name"], ([0, 0, 0], [1, 0, 0], [0, 0, 1]))
        target = add(f"AXIS2_PLACEMENT_3D('',#{pt(o)},#{dr(z)},#{dr(x)})")
        nauo = add(f"NEXT_ASSEMBLY_USAGE_OCCURRENCE('{part['name']}:1','{part['name']}','',#{root_pd},#{pd},$)")
        idt = add(f"ITEM_DEFINED_TRANSFORMATION('','',#{local},#{target})")
        rel = add(f"(REPRESENTATION_RELATIONSHIP('','',#{rep},#{root_rep})REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#{idt})"
                  f"SHAPE_REPRESENTATION_RELATIONSHIP())")
        nsh = add(f"PRODUCT_DEFINITION_SHAPE('','',#{nauo})")
        add(f"CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#{rel},#{nsh})")
    head = ("ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('BeamForge skeleton'),'2;1');\n"
            "FILE_NAME('skeleton.step','',(''),(''),'BeamForge','BeamForge','');\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n")
    return head + "\n".join(ents) + "\nENDSEC;\nEND-ISO-10303-21;\n"
