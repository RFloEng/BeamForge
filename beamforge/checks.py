"""Static checks of a built vehicle, before it goes into the game: the causes of a spawn that explodes, found so far in
test cars, looked for in the written files. Each check returns plain sentences; nothing is changed here. The game's own
log (the diagnose option of the converter, export.DIAGNOSE_LUA) is still the final word: these catch what can be known
without running the physics.

  torsion bars   a torsion bar whose frame nodes are nearly in a line has no stable reference direction and spins up at
                 spawn (a mirror hung on its door, thrown off)
  overlaps       nodes of different parts that the base had apart and the build put on top of each other
  stiff nodes    nodes past the stiffest node of any stock vehicle at the physics step
  engine         a torque rating below what the curve and the turbo can reach (the engine damages its own oil)
  bump steer     a steered wheel whose toe still changes with travel after the tie rods were tuned
"""
import json
import math

from . import beamng, kinematics, rigidity, steering, values

LINE_SIN = 0.08          # sin of the angle between a torsion bar's frame edges below which they are nearly in a line
OVERLAP = 0.004          # m: nodes of different parts closer than this, where the base had them further apart
BUMP_TOE = 0.15          # deg per 10 mm of bump: past it a steered wheel steers itself as the car dives
TORQUE_ROOM = 1.15       # the torque rating is at least this times the curve's peak


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
    out += stiff_nodes(new_id, configured)
    out += engine(docs)
    out += bump_steer(new_id, configured)
    return out
