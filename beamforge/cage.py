"""A roll bar or safety cage from its tubes: every member's limits, the node weights, the springs BeamNG can run, the stiffness.

Pure Python, standard library only (runs in Pyodide). Ported from FBeam's examples/cage/cage_calc.py (Copyright (c) 2026 FBeam
contributors, MIT License) over beamforge/tubes.py, which carries FBeam's section and member rules:

  layout     the 12 nodes and 18 tubes of FBeam's FIA 253-8.3.3 cage (main hoop, A-pillar and roof-rail hoops, windscreen header,
             backstays, door bars, two diagonals), in BeamNG's axes (x left, y rear, z up, m); or the rear roll bar alone (main hoop,
             backstays and the hoop's diagonal). fit() moves the layout into a car's cabin from a few numbers.
  members    for each tube (tubes.section, steel_1018 by default): E A / L, yield and Euler buckling force over its length, the force it
             yields or buckles at (beamDeform), its strength, its mass. Its end nodes carry half its mass, plus a share for welds.
  springs    BeamNG cannot run real steel at those node weights (FBeam 8.9.6): the beam's spring is the smaller of E A / L and the
             explicit integrator's limit for the two end nodes, k <= SAFETY * mu * (2 / dt)^2 with mu their reduced mass.
  stiffness  stiffness(): the cage as a frame of pins and welded corners (tubes.bending_budgets), solved for a side load, a fore-aft load and
             a twist at the main hoop, with the real E A / L and with the springs BeamNG runs: the second as a share of the first is
             what the game's cage keeps of the real one.

Units: SI inside, tube sizes in mm as FBeam writes them ("45x2.5").
"""

import json
import math
from collections import defaultdict

from . import tubes

PHYSICS_HZ = 2000.0           # BeamNG's physics rate (FBeam 8.9.6)
SAFETY = 0.25                 # of the integrator's limit used
MATERIAL = "steel_1018"

# FBeam's cabin, in BeamNG axes (m): the main hoop's feet, its door-bar joints and its top corners; the A-pillar feet and tops; the backstay feet.
NODES = {
    "MHf_L": (0.60, 0.25, 0.20), "MHf_R": (-0.60, 0.25, 0.20),
    "MHm_L": (0.58, 0.25, 0.70), "MHm_R": (-0.58, 0.25, 0.70),
    "MHt_L": (0.55, 0.25, 1.20), "MHt_R": (-0.55, 0.25, 1.20),
    "APf_L": (0.60, -0.95, 0.35), "APf_R": (-0.60, -0.95, 0.35),
    "APt_L": (0.55, -0.40, 1.18), "APt_R": (-0.55, -0.40, 1.18),
    "BSf_L": (0.50, 1.15, 0.40), "BSf_R": (-0.50, 1.15, 0.40),
}

# Tube per member kind (D x t, mm). FIA 253-8.3.3 minimums are 45x2.5 or 50x2.0 for the main members, 38x2.5 or 40x2.0 for the others.
OPTIONS = {
    "recommended": {"main": "45x2.5", "other": "40x2", "diag": "50x2"},
    "fia_min": {"main": "45x2.5", "other": "38x2.5", "diag": "38x2.5"},
    "light": {"main": "50x2", "other": "40x2", "diag": "40x2"},
}

MEMBERS = [        # (group, node a, node b, kind)
    ("Main hoop, lower leg", "MHf_L", "MHm_L", "main"), ("Main hoop, lower leg", "MHf_R", "MHm_R", "main"),
    ("Main hoop, upper leg", "MHm_L", "MHt_L", "main"), ("Main hoop, upper leg", "MHm_R", "MHt_R", "main"),
    ("Main hoop, top", "MHt_L", "MHt_R", "main"),
    ("Lateral rollbar, A-pillar", "APf_L", "APt_L", "main"), ("Lateral rollbar, A-pillar", "APf_R", "APt_R", "main"),
    ("Lateral rollbar, roof rail", "APt_L", "MHt_L", "main"), ("Lateral rollbar, roof rail", "APt_R", "MHt_R", "main"),
    ("Windscreen header", "APt_L", "APt_R", "other"),
    ("Backstay", "MHt_L", "BSf_L", "other"), ("Backstay", "MHt_R", "BSf_R", "other"),
    ("Main hoop diagonal", "MHt_L", "MHf_R", "diag"),
    ("Roof diagonal", "APt_L", "MHt_R", "other"),
    ("Door bar, upper", "APf_L", "MHm_L", "other"), ("Door bar, upper", "APf_R", "MHm_R", "other"),
    ("Door bar, lower", "APf_L", "MHf_L", "other"), ("Door bar, lower", "APf_R", "MHf_R", "other"),
]
BAR_GROUPS = ("Main hoop, lower leg", "Main hoop, upper leg", "Main hoop, top", "Backstay", "Main hoop diagonal")   # the rear roll bar alone
LAYOUTS = {"cage": "Full cage (12 nodes, 18 tubes)", "bar": "Rear roll bar (main hoop, backstays, diagonal)"}


def members_of(layout="cage"):
    """The members of a layout: [(group, a, b, kind)]."""
    return [m for m in MEMBERS if layout != "bar" or m[0] in BAR_GROUPS]


def nodes_of(members):
    """The layout's nodes in use."""
    used = {n for _, a, b, _ in members for n in (a, b)}
    return {n: p for n, p in NODES.items() if n in used}


def _piece(x, xs, ys):
    """Piecewise-linear map of x through anchor points xs -> ys (extrapolated at both ends along the end segments)."""
    for i in range(len(xs) - 1):
        if x <= xs[i + 1] or i == len(xs) - 2:
            return ys[i] + (ys[i + 1] - ys[i]) * (x - xs[i]) / (xs[i + 1] - xs[i])


def params_from_measure(measure):
    """Starting cabin numbers for a vehicle (its beamng.measure(): wheelbase, axle lines, ground, tracks, body height) in BeamNG's axes:
    {"hoop_y", "apillar_y", "backstay_y", "half_width", "floor_z", "roof_z"}. About where a compact car's cabin is; the user edits them."""
    m = measure or {}
    wb = m.get("wheelbase") or 2.6
    yf = m.get("front_axle_y")
    yf = -wb * 0.55 if yf is None else yf
    ground = m.get("ground_z") or 0.0
    track = m.get("track_front") or 1.45
    height = m.get("body_height") or m.get("height") or 1.4
    return {"hoop_y": round(yf + 0.52 * wb, 3), "apillar_y": round(yf + 0.18 * wb, 3), "backstay_y": round(yf + 0.98 * wb, 3),
            "half_width": round(0.41 * track, 3), "floor_z": round(ground + 0.20, 3), "roof_z": round(ground + 0.97 * height, 3)}


def fit(layout="cage", params=None):
    """The layout's nodes moved into a cabin: {node: (x, y, z)}. params: params_from_measure's keys (missing ones keep FBeam's value).
    x scales with the half width, y is piecewise linear through the A-pillar feet, the main hoop and the backstay feet, z runs from the
    floor to the roof."""
    nodes = nodes_of(members_of(layout))
    p = params or {}
    if not p:
        return {n: tuple(v) for n, v in nodes.items()}
    ys = [-0.95, 0.25, 1.15]
    yt = [p.get("apillar_y", -0.95), p.get("hoop_y", 0.25), p.get("backstay_y", 1.15)]
    sx = p.get("half_width", 0.60) / 0.60
    z0, z1 = p.get("floor_z", 0.20), p.get("roof_z", 1.22)
    out = {}
    for n, (x, y, z) in nodes.items():
        out[n] = (round(x * sx, 4), round(_piece(y, ys, yt), 4), round(z0 + (z - 0.20) * (z1 - z0) / (1.22 - 0.20), 4))
    return out


def _forces(sec, L):
    """(yield force, Euler buckling force) of a member tube of length L (N)."""
    return sec["sy"] * sec["A"], math.pi ** 2 * sec["E"] * sec["I"] / (L * L)


def design(nodes, members, sizes, material=MATERIAL, joint=tubes.JOINT, hz=PHYSICS_HZ, safety=SAFETY):
    """Every member's limits, the node weights and the springs BeamNG can run. sizes: {kind: "<D>x<t>"} (an OPTIONS entry).
    Returns (rows, node_mass): a row per member {"group", "a", "b", "kind", "tube", "L", "mass", "k_real", "yield", "buckling", "deform",
    "strength", "governs", "k_stable", "k_set"}; node_mass {node: kg}, rounded up to 0.05 kg, joints included."""
    rows, share = [], defaultdict(float)
    for group, a, b, kind in members:
        tube = sizes[kind]
        sec = tubes.section(f"tube {tube} {material}")
        L = math.dist(nodes[a], nodes[b])
        lim = tubes.limits(sec, L)
        fy, fb = _forces(sec, L)
        rows.append({"group": group, "a": a, "b": b, "kind": kind, "tube": tube, "L": L, "mass": lim["mass"], "k_real": lim["k"],
                     "yield": fy, "buckling": fb, "deform": lim["beamDeform"], "strength": lim["beamStrength"],
                     "governs": "buckling" if fb < fy else "yield", "sec": sec})
        share[a] += lim["mass"] / 2
        share[b] += lim["mass"] / 2
    node_mass = {n: math.ceil(s * (1 + joint) * 20) / 20 for n, s in share.items()}
    dt = 1.0 / hz
    for r in rows:
        ma, mb = node_mass[r["a"]], node_mass[r["b"]]
        mu = ma * mb / (ma + mb)
        r["k_stable"] = safety * mu * (2 / dt) ** 2          # explicit integration is stable for k <= mu (2 / dt)^2
        r["k_set"] = min(r["k_real"], r["k_stable"])
    return rows, node_mass


# ---------------------------------------------------------------- stiffness

def _solve(A, b):
    """x of A x = b by Gauss elimination with partial pivoting (small systems); A is changed."""
    n = len(b)
    for i in range(n):
        p = max(range(i, n), key=lambda r: abs(A[r][i]))
        if abs(A[p][i]) < 1e-12:
            continue
        A[i], A[p] = A[p], A[i]
        b[i], b[p] = b[p], b[i]
        for r in range(i + 1, n):
            f = A[r][i] / A[i][i]
            if f:
                for c in range(i, n):
                    A[r][c] -= f * A[i][c]
                b[r] -= f * b[i]
    x = [0.0] * n
    for i in range(n - 1, -1, -1):
        if abs(A[i][i]) > 1e-12:
            x[i] = (b[i] - sum(A[i][c] * x[c] for c in range(i + 1, n))) / A[i][i]
    return x


def _springs(nodes, rows, use):
    """[(node a, node b, k)] of the frame: each member (E A / L real or the spring BeamNG runs), and the bending beams of its welded corners
    (tubes.bending_budgets) scaled like the members around them."""
    out = []
    ratio = {}
    for i, r in enumerate(rows):
        k = r["k_real"] if use == "real" else r["k_set"]
        out.append((r["a"], r["b"], k))
        ratio[i] = k / r["k_real"]
    mem = [{"id": i, "a": r["a"], "b": r["b"]} for i, r in enumerate(rows)]
    for (a, c), e in tubes.bending_budgets(nodes, mem, lambda m: rows[m["id"]]["sec"]).items():
        s = sum(ratio[i] for i in e["members"]) / len(e["members"])
        out.append((a, c, e["k"] * s))
    return out


def stiffness(nodes, rows, use="real", load=1000.0):
    """The frame's stiffness at the main hoop's top corners with its feet (and the backstay and A-pillar feet) held: {"side" (N/m: a lateral
    push on the top, force over its sway), "fore_aft" (N/m), "twist" (N m/rad: opposite vertical forces on the two top corners over the twist
    of the hoop's top)}. use "real": E A / L; "set": the springs BeamNG runs."""
    held = {n for n in nodes if n.endswith("f_L") or n.endswith("f_R")}
    names = [n for n in nodes if n not in held]
    idx = {n: i for i, n in enumerate(names)}
    N = 3 * len(names)
    K = [[0.0] * N for _ in range(N)]
    for a, b, k in _springs(nodes, rows, use):
        d = [nodes[b][i] - nodes[a][i] for i in range(3)]
        L = math.sqrt(sum(x * x for x in d))
        if L < 1e-9:
            continue
        u = [x / L for x in d]
        for n1, s1 in ((a, 1), (b, -1)):
            for n2, s2 in ((a, 1), (b, -1)):
                if n1 in idx and n2 in idx:
                    for p in range(3):
                        for q in range(3):
                            K[3 * idx[n1] + p][3 * idx[n2] + q] += k * s1 * s2 * u[p] * u[q]
    for i in range(N):                                  # a tiny anchor so a free node of a pin-jointed layout does not make the system singular
        K[i][i] += 1.0
    tl, tr = "MHt_L", "MHt_R"
    out = {}
    for name, f in (("side", ((tl, 0, load / 2), (tr, 0, load / 2))), ("fore_aft", ((tl, 1, load / 2), (tr, 1, load / 2)))):
        b = [0.0] * N
        for n, ax, v in f:
            b[3 * idx[n] + ax] += v
        x = _solve([r[:] for r in K], b)
        ax = 0 if name == "side" else 1
        move = (x[3 * idx[tl] + ax] + x[3 * idx[tr] + ax]) / 2
        out[name] = round(load / move) if abs(move) > 1e-15 else None
    b = [0.0] * N
    b[3 * idx[tl] + 2], b[3 * idx[tr] + 2] = load, -load
    x = _solve([r[:] for r in K], b)
    width = abs(nodes[tl][0] - nodes[tr][0])
    twist = abs(x[3 * idx[tl] + 2] - x[3 * idx[tr] + 2]) / width if width > 1e-6 else 0.0
    out["twist"] = round(load * width / twist) if twist > 1e-15 else None
    return out


def summary(nodes, rows, node_mass):
    """Totals for the panel: tube mass, node mass (joints included), the weakest member (its force, group and what governs), and the cage's
    stiffness real and as BeamNG runs it, with the share the game keeps."""
    weakest = min(rows, key=lambda r: r["deform"])
    real, run = stiffness(nodes, rows, "real"), stiffness(nodes, rows, "set")
    share = {k: (round(run[k] / real[k], 3) if real.get(k) and run.get(k) else None) for k in real}
    return {"tube_kg": round(sum(r["mass"] for r in rows), 2), "node_kg": round(sum(node_mass.values()), 2),
            "weakest": {"group": weakest["group"], "tube": weakest["tube"], "force_kn": round(weakest["deform"] / 1e3, 1), "governs": weakest["governs"]},
            "real": real, "beamng": run, "share": share}


def part(name, nodes, rows, node_mass, group="bf_cage"):
    """The cage as a jbeam part {name: {...}} to add to a vehicle: its nodes (weights set) and its beams (the springs BeamNG runs, the tubes'
    deform and strength 1 % under their limits, as FBeam writes them)."""
    nrows = [["id", "posX", "posY", "posZ"], {"group": group}]
    for n, (x, y, z) in nodes.items():
        nrows.append([n, x, y, z, {"nodeWeight": node_mass[n]}])
    brows = [["id1:", "id2:"], {"beamDamp": 150}]
    for r in rows:
        brows.append([r["a"], r["b"], {"fbeamSection": f"tube {r['tube']} {MATERIAL}", "beamSpring": round(r["k_set"]),
                                       "beamDeform": round(r["deform"] * 0.99), "beamStrength": round(r["strength"] * 0.99)}])
    return {name: {"information": {"authors": "BeamForge", "name": "BeamForge safety cage"}, "slotType": "main", "nodes": nrows, "beams": brows}}


# ---------------------------------------------------------------- for the editor (JSON in, JSON out)

def params_json(measure_json):
    """params_from_measure() of a vehicle's measure (JSON), as JSON."""
    return json.dumps(params_from_measure(json.loads(measure_json) if measure_json else {}))


def _run(layout, sizes_json, params_json_):
    sizes = json.loads(sizes_json)
    params = json.loads(params_json_) if params_json_ else None
    mem = members_of(layout)
    nodes = fit(layout, params)
    rows, node_mass = design(nodes, mem, sizes)
    return mem, nodes, rows, node_mass


def compute_json(layout, sizes_json, params_json_=None):
    """The cage for the Cage workspace: {"nodes": {id: [x, y, z]}, "node_mass", "rows": [member rows without the section], "summary"}, or
    {"error": text} for a tube that is not made (tubes.SectionError)."""
    try:
        mem, nodes, rows, node_mass = _run(layout, sizes_json, params_json_)
    except (tubes.SectionError, KeyError) as e:
        return json.dumps({"error": str(e)})
    return json.dumps({"nodes": {n: list(p) for n, p in nodes.items()}, "node_mass": node_mass,
                       "rows": [{k: v for k, v in r.items() if k != "sec"} for r in rows], "summary": summary(nodes, rows, node_mass),
                       "options": OPTIONS, "layouts": LAYOUTS})


def part_json(layout, sizes_json, params_json_=None, name="bf_cage"):
    """The cage as a jbeam part (JSON) to add to a vehicle."""
    mem, nodes, rows, node_mass = _run(layout, sizes_json, params_json_)
    return json.dumps(part(name, nodes, rows, node_mass), indent=1)
