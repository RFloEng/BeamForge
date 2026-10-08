"""Real tubes behind jbeam beams: sections, their limits, bending beams for welded corners, buckling lengths.

Ported from FBeam (fbeam_checker/sections.py, bending.py, buckling.py; Copyright (c) 2026 FBeam contributors, MIT
License), where every structural beam declares the tube it stands for. Pure Python, standard library only.

A section is written like FBeam's "fbeamSection": "tube 45x2.5 steel_1018" (round tube, outer diameter and wall in mm),
"sqtube 40x2 al_6061_t6" (square tube). For a member tube of length L:

    k = E A / L     beamDeform = min(sy A, pi^2 E I / L_buckle^2)     beamStrength = su A     mass = rho A L

and its end nodes carry its mass (half each, plus JOINT for welds and gussets). BeamNG cannot run real steel stiffness
at real node masses (FBeam regulation 8.9.6): the stiffness used is the smaller of E A / L and the stable value
(rigidity.beam_values), so in BeamNG a stiffer structure needs heavier nodes, and a frame's rigidity comes from its
triangulation.

Bending beams (FBeam regulation 8.10): a jbeam node is a pin, so two tubes welded at a joint J would turn freely about
it. A weightless beam between their far ends A and C gives the corner back the bending stiffness of the real tubes:

    k_corner = 1 / (L_JA / 3 E I_A + L_JC / 3 E I_C)      k_bend = k_corner / h^2      F_bend = min(Mp_A, Mp_C) / h

with h the distance from J to the line A-C and Mp = sy (D^3 - d^3) / 6 the plastic moment of a round tube. It is
scaled like the members around it (k_set / (E A / L)), so it never takes a larger share of the load than in the real
car.

Units: SI inside (m, m^2, m^4, Pa, N, kg); section dimensions are written in mm.
"""

import math

MATERIALS = {                      # FBeam rules/fbeam_v1.json: E (Pa), yield and ultimate stress (Pa), density (kg/m^3)
    "steel_1018": {"E": 205e9, "sy": 365e6, "su": 440e6, "rho": 7850.0},
    "steel_4130n": {"E": 205e9, "sy": 460e6, "su": 560e6, "rho": 7850.0},
    "al_6061_t6": {"E": 69e9, "sy": 276e6, "su": 310e6, "rho": 2700.0},
}
MIN_WALL = 0.0015                  # m: thinner tube walls are not made (FBeam 8.4)
JOINT = 0.15                       # a node carries its share of the tubes' mass plus this, for welds and gussets (FBeam 8.3)
MIN_ANGLE_DEG = 20.0               # corners flatter or sharper than this get no bending beam
MIN_LEVER = 0.02                   # m: a bending beam this close to its joint gives no useful lever
STRENGTH_FACTOR = 1.2              # a bending beam's beamStrength over its beamDeform (FBeam 8.10.3)
MAX_TURN_DEG = 30.0                # a split tube's buckling run carries on through bends up to this


class SectionError(ValueError):
    """A section string that is malformed or outside the catalogue."""


def section(text):
    """{"text", "kind", "material", "A", "I", "D", "t", "E", "sy", "su", "rho"} of "tube 45x2.5 steel_1018" (SI)."""
    parts = str(text).split()
    if len(parts) == 2:
        parts.append("steel_1018")
    if len(parts) != 3:
        raise SectionError(f"section {text!r}: expected '<tube|sqtube> <D>x<t> [material]'")
    kind, dims, mat = parts
    if mat not in MATERIALS:
        raise SectionError(f"section {text!r}: unknown material {mat!r} ({', '.join(MATERIALS)})")
    try:
        a, t = (float(x) / 1000.0 for x in dims.lower().split("x"))
    except ValueError:
        raise SectionError(f"section {text!r}: dimensions must be like 45x2.5 (mm)")
    if t < MIN_WALL - 1e-9 or 2 * t >= a:
        raise SectionError(f"section {text!r}: wall must be at least {MIN_WALL * 1000:g} mm and under half the size")
    if kind == "tube":
        d = a - 2 * t
        A, I = math.pi / 4 * (a ** 2 - d ** 2), math.pi / 64 * (a ** 4 - d ** 4)
    elif kind == "sqtube":
        b = a - 2 * t
        A, I = a ** 2 - b ** 2, (a ** 4 - b ** 4) / 12
    else:
        raise SectionError(f"section {text!r}: kind must be tube or sqtube")
    m = MATERIALS[mat]
    return {"text": f"{kind} {dims} {mat}", "kind": kind, "material": mat, "A": A, "I": I, "D": a, "t": t,
            "E": m["E"], "sy": m["sy"], "su": m["su"], "rho": m["rho"]}


def limits(sec, L, L_buckle=None):
    """A member tube's real values for a beam of length L (m): {"k" (E A / L), "beamDeform" (yield or Euler buckling over
    L_buckle, whichever is lower), "beamStrength", "mass", "governs"}."""
    f_yield = sec["sy"] * sec["A"]
    f_buckle = math.pi ** 2 * sec["E"] * sec["I"] / (L_buckle or L) ** 2
    return {"k": sec["E"] * sec["A"] / L, "beamDeform": min(f_yield, f_buckle), "beamStrength": sec["su"] * sec["A"],
            "mass": sec["rho"] * sec["A"] * L, "governs": "buckling" if f_buckle < f_yield else "yield"}


def plastic_modulus(D, t):
    """Z = (D^3 - d^3) / 6 of a round tube (m^3); a square tube is taken as the round one of the same size (smaller)."""
    d = D - 2 * t
    return (D ** 3 - d ** 3) / 6


def _lever(j, a, c):
    ac = [c[i] - a[i] for i in range(3)]
    aj = [j[i] - a[i] for i in range(3)]
    L2 = sum(v * v for v in ac)
    if L2 <= 0:
        return 0.0
    t = sum(aj[i] * ac[i] for i in range(3)) / L2
    return math.dist(j, [a[i] + ac[i] * t for i in range(3)])


def corners(pos, members):
    """Every corner that can take a bending beam: [(joint, member a, member c, far a, far c)]. members: [{"id", "a", "b"}];
    two members at one joint whose far ends are not already joined by a member, at an angle within MIN_ANGLE_DEG of
    neither 0 nor 180 degrees."""
    at = {}
    for m in members:
        at.setdefault(m["a"], []).append(m)
        at.setdefault(m["b"], []).append(m)
    joined = {frozenset((m["a"], m["b"])) for m in members}
    out = []
    for j, ms in at.items():
        for i in range(len(ms)):
            for k in range(i + 1, len(ms)):
                ma, mc = ms[i], ms[k]
                a = ma["b"] if ma["a"] == j else ma["a"]
                c = mc["b"] if mc["a"] == j else mc["a"]
                if a == c or frozenset((a, c)) in joined:
                    continue
                u = [pos[a][q] - pos[j][q] for q in range(3)]
                v = [pos[c][q] - pos[j][q] for q in range(3)]
                cos = sum(u[q] * v[q] for q in range(3)) / (math.hypot(*u) * math.hypot(*v))
                ang = math.degrees(math.acos(max(-1.0, min(1.0, cos))))
                if MIN_ANGLE_DEG <= ang <= 180 - MIN_ANGLE_DEG:
                    out.append((j, ma, mc, a, c))
    return out


def bending_budgets(pos, members, section_of):
    """The bending beams of welded corners: {(a, c) sorted: {"k" (real, N/m), "F" (N), "corners", "members"}}. Several
    corners proposing the same beam add up. section_of(member) -> section()."""
    out = {}
    for j, ma, mc, a, c in corners(pos, members):
        h = _lever(pos[j], pos[a], pos[c])
        if h < MIN_LEVER:
            continue
        flex, Mp = 0.0, []
        for m, far in ((ma, a), (mc, c)):
            s = section_of(m)
            flex += math.dist(pos[j], pos[far]) / (3 * s["E"] * s["I"])
            Mp.append(s["sy"] * plastic_modulus(s["D"], s["t"]))
        e = out.setdefault(tuple(sorted((a, c))), {"k": 0.0, "F": 0.0, "corners": [], "members": set()})
        e["k"] += 1.0 / flex / (h * h)
        e["F"] += min(Mp) / h
        e["corners"].append(j)
        e["members"].update((ma["id"], mc["id"]))
    for e in out.values():
        e["members"] = sorted(e["members"])
    return out


def buckling_lengths(pos, members, max_turn_deg=MAX_TURN_DEG):
    """{member id: unbraced length (m)}: a tube split into segments buckles over the whole run, followed through every
    joint where exactly two members meet and the line turns by max_turn_deg or less (FBeam 8.9.7)."""
    at = {}
    for m in members:
        at.setdefault(m["a"], []).append(m)
        at.setdefault(m["b"], []).append(m)
    length = {m["id"]: math.dist(pos[m["a"]], pos[m["b"]]) for m in members}
    cos_max = math.cos(math.radians(max_turn_deg))

    def other(m, n):
        return m["b"] if m["a"] == n else m["a"]

    def carries_on(n, into, out_):
        u = [pos[n][i] - pos[other(into, n)][i] for i in range(3)]
        v = [pos[other(out_, n)][i] - pos[n][i] for i in range(3)]
        nu, nv = math.hypot(*u), math.hypot(*v)
        return nu > 0 and nv > 0 and sum(x * y for x, y in zip(u, v)) / (nu * nv) >= cos_max

    out = {}
    for m in members:
        total, seen = length[m["id"]], {m["id"]}
        for start in (m["a"], m["b"]):
            n, prev = start, m
            while len(at[n]) == 2:
                nxt = at[n][0] if at[n][1]["id"] == prev["id"] else at[n][1]
                if nxt["id"] in seen or not carries_on(n, prev, nxt):
                    break
                seen.add(nxt["id"])
                total += length[nxt["id"]]
                n, prev = other(nxt, n), nxt
        out[m["id"]] = total
    return out
