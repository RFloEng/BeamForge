"""Static checks of a built vehicle, before it goes into the game: the causes of a spawn that explodes, found so far in
test cars, looked for in the written files. Each check returns plain sentences; nothing is changed here. The game's own
log (the diagnose option of the converter, export.DIAGNOSE_LUA) is still the final word: these catch what can be known
without running the physics.

  torsion bars   a torsion bar whose frame nodes are nearly in a line has no stable reference direction and spins up at
                 spawn (a mirror hung on its door, thrown off)
  overlaps       nodes of different parts that the base had apart and the build put on top of each other
  stiff nodes    nodes past the stiffest node of any stock vehicle at the physics step
  engine         a torque rating below what the curve and the turbo can reach (the engine damages its own oil)
  shrunk beams   beams that became a fraction of the base's length with a stiff spring: the SVJ's points that tie them are
                 (nearly) the same, the geometry is degenerate and the node is shaken apart at spawn
  bump steer     a steered wheel whose toe still changes with travel after the tie rods were tuned

compliance() is the other way round: the SVJ's own figures against the written vehicle, every one the SVJ gives and the
user asked to take (and the ones left out), so a value the SVJ defines that the vehicle does not reach is always said.
"""
import json
import math

from . import beamng, kinematics, rigidity, steering, values

LINE_SIN = 0.08          # sin of the angle between a torsion bar's frame edges below which they are nearly in a line
OVERLAP = 0.004          # m: nodes of different parts closer than this, where the base had them further apart
SHRINK = 0.45            # a stiff beam shorter than this share of its base length (and under SHORT_M) is flagged
SHORT_M = 0.07
BUMP_TOE = 0.15          # deg per 10 mm of bump: past it a steered wheel steers itself as the car dives
TORQUE_ROOM = 1.05       # the torque rating is at least this times the curve's peak


def _sub(a, b):
    return [x - y for x, y in zip(a, b)]


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def _norm(v):
    return math.sqrt(sum(x * x for x in v))


def _docs(files, active):
    """The parts of the written files that are in the configuration: {part name: part}."""
    out = {}
    for path, text in files.items():
        if not path.endswith(".jbeam"):
            continue
        try:
            doc = json.loads(text)
        except ValueError:
            continue
        for name, part in doc.items():
            if name in active and isinstance(part, dict):
                out[name] = part
    return out


def torsion_sin(N, row):
    """The smaller sin of the two angles a torsion bar's frame (rows' first four nodes) is made of; None for a row that is not one."""
    if not (isinstance(row, list) and len(row) >= 4 and all(isinstance(x, str) and x in N for x in row[:4])):
        return None
    a, b, c, d = (N[x] for x in row[:4])
    sins = []
    for u, v in ((_sub(c, b), _sub(d, c)), (_sub(b, a), _sub(c, b))):
        lu, lv = _norm(u), _norm(v)
        sins.append(_norm(_cross(u, v)) / (lu * lv) if lu > 1e-9 and lv > 1e-9 else 0.0)
    return min(sins)


def torsion_bars(N, docs):
    out = []
    for name, part in sorted(docs.items()):
        for row in part.get("torsionbars") or []:
            if not (isinstance(row, list) and len(row) >= 4 and all(isinstance(x, str) and x in N for x in row[:4])):
                continue
            a, b, c, d = (N[x] for x in row[:4])
            e1, e2 = _sub(c, b), _sub(d, c)
            ax = _sub(b, a)
            sins = []
            for u, v in ((e1, e2), (ax, e1)):
                lu, lv = _norm(u), _norm(v)
                sins.append(_norm(_cross(u, v)) / (lu * lv) if lu > 1e-9 and lv > 1e-9 else 0.0)
            if min(sins) < LINE_SIN:
                out.append(f"{name}: torsion bar {'-'.join(row[:4])} has nodes nearly in a line (sin {min(sins):.2f}); it can spin up at "
                           f"spawn and throw its part off")
    return out


def overlaps(N, parts, base_N, base_parts):
    cells = {}
    for n, p in N.items():
        cells.setdefault(tuple(int(math.floor(x / OVERLAP)) for x in p), []).append(n)
    bad = []
    seen = set()
    for n, p in N.items():
        k = tuple(int(math.floor(x / OVERLAP)) for x in p)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    for m in cells.get((k[0] + dx, k[1] + dy, k[2] + dz), ()):
                        if m <= n or (n, m) in seen or parts.get(m) == parts.get(n):
                            continue
                        if math.dist(p, N[m]) >= OVERLAP:
                            continue
                        seen.add((n, m))
                        if n in base_N and m in base_N and math.dist(base_N[n], base_N[m]) < OVERLAP * 3:
                            continue                                  # the base had them together
                        bad.append((n, m))
    if not bad:
        return []
    shown = ", ".join(f"{a}/{b}" for a, b in bad[:6])
    return [f"{len(bad)} node pairs of different parts within {OVERLAP * 1000:.0f} mm that the base had apart (e.g. {shown})"]


def stiff_nodes(model, configured):
    limit = rigidity.VANILLA["node_k"]["max"]
    bad = [(n, k) for n, k, _, _ in rigidity.stability(model, configured) if k > limit]
    if not bad:
        return []
    shown = ", ".join(f"{n} {k:.1f}" for n, k in bad[:6])
    return [f"{len(bad)} nodes stiffer than any stock vehicle's ({limit:.1f} at the physics step): {shown}"]


def engine(docs):
    out = []
    for name, part in sorted(docs.items()):
        me = part.get("mainEngine")
        if not (isinstance(me, dict) and isinstance(me.get("torque"), list)):
            continue
        peak = max((r[1] for r in me["torque"][1:] if isinstance(r, list) and len(r) > 1 and isinstance(r[1], (int, float))), default=0)
        rating = me.get("maxTorqueRating")
        if isinstance(rating, (int, float)) and peak and rating < peak * TORQUE_ROOM:
            out.append(f"{name}: torque rating {rating} is under {TORQUE_ROOM} times the curve's peak ({peak:.0f}); the engine will damage "
                       f"itself (oil) as soon as it makes that torque")
        over = me.get("maxOverTorqueDamage")
        if isinstance(over, (int, float)) and isinstance(rating, (int, float)) and over < rating:
            out.append(f"{name}: over-torque damage limit {over} is under the torque rating {rating}")
    return out


def bump_steer(model, configured):
    bl = rigidity.beams(model, configured)
    springs = values.springs_and_dampers(model, configured)
    dirs = steering.direction(model, configured)
    out = []
    for w in configured.get("wheels") or []:
        if w["name"] not in dirs or w["centre"][0] <= 0:
            continue
        t = kinematics.bump_toe(model, configured, w, None, bl, springs)
        if t is not None and abs(t) > BUMP_TOE:
            out.append(f"{w['name']}: toe changes {t:+.2f} deg per 10 mm of bump (a stock vehicle's is about 0.02): it will steer itself as it dives")
    return out


# how far a built value may be from the SVJ's before it is said: relative, or metres for the CG and the track
TOLERANCE = {"spring": 0.03, "damp": 0.08, "tyre": 0.01, "engine": 0.03, "gears": 0.01, "final_drive": 0.01, "aero": 0.10,
             "mass": 0.02, "cg_y": 0.03, "cg_z": 0.03, "wheelbase": 0.01, "track": 0.01}
WHY = {"mass": "the base's structure is kept (it would be too stiff for lighter nodes; the scale structure option lightens it, but cars built that way were thrown out by the game's instability check, but cars built that way were thrown out by the game's instability check)",
       "cg_y": "the body's node weights could not be moved further without going under what the physics step allows",
       "cg_z": "the body's node weights could not be moved further without going under what the physics step allows"}


def _tol(key):
    return next((t for k, t in TOLERANCE.items() if key.startswith(k)), 0.05)


def compliance(new_id, files, svj, taken, study=None):
    """The SVJ's figures the written vehicle does not reach, and the ones the user left out: sentences starting "SVJ". The
    vehicle's files are read back with beamng.add_files; taken: {values table key: bool}."""
    beamng.add_files(json.dumps(files))
    cj = beamng.configure(new_id)
    out = []
    for r in json.loads(values.table(new_id, cj, json.dumps(svj), json.dumps(study) if study else None)):
        want, got, key = r.get("svj"), r.get("base"), r["key"]
        if want is None or r.get("info"):
            continue
        unit = f" {r['unit']}" if r.get("unit") else ""
        if not taken.get(key):
            out.append(f"SVJ not taken: {r['label']}: the SVJ gives {want}{unit}, left as the base's ({got}{unit})")
            continue
        if isinstance(want, str) or isinstance(got, str):
            miss = str(want).split() != str(got).split()
        elif got is None:
            miss = True
        elif key.startswith("cg_") or key in ("wheelbase", "track"):
            miss = abs(got - want) > _tol(key)
        else:
            miss = abs(got - want) > _tol(key) * abs(want)
        if miss:
            why = WHY.get(key)
            out.append(f"SVJ not met: {r['label']}: the SVJ gives {want}{unit}, the vehicle has {got}{unit}" + (f" ({why})" if why else ""))
    out += suspension_points(new_id, json.loads(cj), svj)
    m = json.loads(cj).get("measure") or {}
    ch = (svj or {}).get("chassis") or {}
    for key, label in (("wheelbase", "wheelbase"), ("track_front", "front track"), ("track_rear", "rear track")):
        want, got = ch.get(key), m.get(key)
        if isinstance(want, (int, float)) and isinstance(got, (int, float)) and abs(got - want) > _tol(key) * want:
            out.append(f"SVJ not met: {label}: the SVJ gives {want} m, the vehicle has {got} m")
    return out


def shrunk_beams(new_id, configured, base_model, base_configured):
    if not base_configured:
        return []
    N, bN = configured["geometry"]["nodes"], base_configured["geometry"]["nodes"]
    base = {}
    for b in rigidity.beams(base_model, base_configured):
        if b["a"] in bN and b["b"] in bN:
            base[frozenset((b["a"], b["b"]))] = math.dist(bN[b["a"]], bN[b["b"]])
    bad = []
    for b in rigidity.beams(new_id, configured):
        k = frozenset((b["a"], b["b"]))
        if k in base and b["a"] in N and b["b"] in N and (b["values"].get("beamSpring") or 0) > 2e5:
            d = math.dist(N[b["a"]], N[b["b"]])
            if d < SHORT_M and base[k] > 1e-3 and d < SHRINK * base[k]:
                bad.append((d / base[k], b["a"], b["b"], d))
    if not bad:
        return []
    bad.sort()
    shown = ", ".join(f"{a}-{c} {d * 1000:.0f} mm ({r * 100:.0f}% of the base's)" for r, a, c, d in bad[:5])
    return [f"{len(bad)} stiff beams much shorter than the base's (the SVJ's points that tie them nearly coincide: bad geometry, it can "
            f"shake the node apart at spawn): {shown}"]


SUSP_TOL = 0.005       # m: a suspension point the vehicle puts further than this from the SVJ's is said


def suspension_points(new_id, configured, svj):
    """The SVJ's suspension points the written vehicle does not hold: each wheel's upright joints (the middle of the nodes that
    make a joint), the strut top and the links' chassis points, against the SVJ's, by the base's role table. Sentences."""
    from . import roles, svj as svjmod
    geo, me = configured["geometry"], configured.get("measure") or {}
    N, P = geo["nodes"], geo.get("parts") or {}
    if "front_axle_y" not in me or "ground_z" not in me:
        return []
    hp = {}
    for h in svjmod.hardpoints(svj, me["front_axle_y"], me["ground_z"]):
        hp.setdefault(h["corner"], {})[h["name"]] = h["pos"]
    mean = lambda ns: [sum(N[n][i] for n in ns) / len(ns) for i in range(3)]      # noqa: E731
    out = []
    for w in configured.get("wheels") or []:
        rt, pts = roles.for_corner(P, N, w), hp.get(w["name"])
        if not rt or not pts:
            continue
        miss = []
        for joint, names in (("lower_ball_joint", ("lower_ball_joint",)), ("upper_ball_joint", ("upper_ball_joint",)),
                             ("tie_rod_end", ("steering_tie_rod_end", "toe_link_outer"))):
            ns = [n for n in rt["joints"].get(joint, []) if n in N]
            want = next((pts[k] for k in names if k in pts), None)
            if ns and want and math.dist(mean(ns), want) > SUSP_TOL:
                miss.append(f"{joint.replace('_', ' ')} {math.dist(mean(ns), want) * 1000:.0f} mm")
        st = rt.get("strut")
        if st and st.get("top") and "strut.0" in pts and math.dist(mean([n for n in st["top"] if n in N]), pts["strut.0"]) > SUSP_TOL:
            miss.append(f"strut top {math.dist(mean([n for n in st['top'] if n in N]), pts['strut.0']) * 1000:.0f} mm")
        for role, prefixes in (("lower_arm", ("lower_control_arm.",)), ("upper_arm", ("upper_control_arm.", "upper_wishbone.")),
                               ("tie_rod", ("steering_tie_rod.", "toe_link."))):
            want = [p for k, p in pts.items() if k.startswith(prefixes)]
            ns = [n for n in rt["pivots"].get(role, []) if n in N]
            if want and ns:
                far = max(min(math.dist(N[n], p) for p in want) for n in ns)
                if far > SUSP_TOL:
                    miss.append(f"{role.replace('_', ' ')} pivot {far * 1000:.0f} mm")
        if miss:
            out.append(f"SVJ not met: {w['name']} suspension points off the SVJ's: {', '.join(miss)} (the upright keeps the base's "
                       "stiff shape; the other points are where the SVJ puts them)")
    return out


def run(new_id, files, base_model=None, base_configured=None):
    """The warnings for a built vehicle: its files {path: text} (read back with beamng.add_files) and, to compare with, the
    base's configuration. Returns a list of sentences (empty: nothing found)."""
    beamng.add_files(json.dumps(files))
    configured = json.loads(beamng.configure(new_id))
    geo = configured["geometry"]
    N, parts = geo["nodes"], geo.get("parts") or {}
    base_geo = (base_configured or {}).get("geometry") or {}
    docs = _docs(files, set(parts.values()))
    out = torsion_bars(N, docs)
    out += overlaps(N, parts, base_geo.get("nodes") or {}, base_geo.get("parts") or {})
    out += shrunk_beams(new_id, configured, base_model, base_configured)
    out += stiff_nodes(new_id, configured)
    out += engine(docs)
    out += bump_steer(new_id, configured)
    return out
