"""Which way a front wheel turns when the rack moves, and fixing it when a build turns it the wrong way.

Pure Python, standard library only (runs in Pyodide). A wheel steers about its steering axis (the nodes
steerAxisDown and steerAxisUp of its pressureWheels row: the hub's ball joint and the strut top). The tie rod
(a beam from the hub's tie rod end to the rack end) turns it: moving the rack end by d turns the hub by

    dtheta = ((h - e) . d) / ((h - e) . (u x r))

with h the tie rod end, e the rack end, u the axis direction (down to up) and r = h - (a point of the axis):
the sign depends on whether the tie rod end is ahead of or behind the axis, and on which side of it the rod
runs. A base vehicle's rack drive (the hydros that move the rack ends) is built for its own geometry; an SVJ's
geometry on it can turn the wheels the other way (the SVJ's tie rod end ahead of the axle on a base whose is
behind it: a roadster SVJ on a vanilla compact coupe steered backwards). direction() gives the sign per wheel, and fix() negates the
hydros of a wheel whose sign differs from the base's.
"""

import math

from . import beamng, jbeam, kinematics, rigidity, values


def _sub(a, b):
    return [a[i] - b[i] for i in range(3)]


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def _unit(v):
    n = math.sqrt(_dot(v, v))
    return [x / n for x in v] if n > 1e-12 else None


def _tables(model, configured, section):
    """The rows of a section's table over the active parts, as dicts (jbeam.expand_table), with the part."""
    parts = beamng._parts_held(model)
    out = []
    for name in values._active(configured):
        t = (parts.get(name) or {}).get("part", {}).get(section)
        if isinstance(t, list) and t and isinstance(t[0], list):
            for r in jbeam.expand_table(t):
                out.append((name, r))
    return out


def wheel_axes(model, configured):
    """{wheel name: (steerAxisDown node, steerAxisUp node)} of the wheels that name them."""
    out = {}
    for _, r in _tables(model, configured, "pressureWheels"):
        r = {str(k).rstrip(":"): v for k, v in r.items()}        # an option's key may carry its colon (steerAxisUp:)
        if r.get("steerAxisDown") and r.get("steerAxisUp"):
            out[str(r.get("name"))] = (str(r["steerAxisDown"]), str(r["steerAxisUp"]))
    return out


def _axis_from_geometry(N, moving, wheel, springs):
    """(lower ball joint, strut top) of a wheel whose row names no steering axis: the lowest node that follows the
    wheel (not the axle's), and the far end of the spring beam from the hub (the strut top). None for a wheel
    with no spring beam at it (a rear wheel on a rail...)."""
    hub = [n for n in moving if n in N and n not in (wheel["node1"], wheel["node2"])]
    if not hub:
        return None
    top = next((r["b"] if r["a"] in moving else r["a"] for r in springs
                if r["kind"] == "spring" and ((r["a"] in moving) != (r["b"] in moving))), None)
    return (min(hub, key=lambda n: N[n][2]), top) if top else None


def rack_drives(model, configured):
    """[(rack end node, node it moves against, factor)] of the hydros that steer (rows with a factor)."""
    out = []
    for _, r in _tables(model, configured, "hydros"):
        f = r.get("factor")
        if isinstance(f, (int, float)) and r.get("id1") and r.get("id2"):
            out.append((str(r["id1"]), str(r["id2"]), float(f)))
    return out


def direction(model, configured, bl=None):
    """{wheel name: dtheta per unit rack move, in radians per metre} for each steered wheel: the angle the hub
    turns about its steering axis (down to up, right-handed) when the wheel's rack end moves one metre the way
    its hydros move it for a positive input (a positive factor lengthens the beam to the node it moves against)."""
    geo = configured["geometry"]
    N = geo["nodes"]
    axes = wheel_axes(model, configured)
    drives = rack_drives(model, configured)
    bl = bl if bl is not None else rigidity.beams(model, configured)
    out = {}
    springs = values.springs_and_dampers(model, configured)
    for w in configured.get("wheels") or []:
        moving = kinematics.follows_wheel(model, configured, w, bl) | {w["node1"], w["node2"]}
        ax = axes.get(w["name"]) or _axis_from_geometry(N, moving, w, springs)
        if not ax or ax[0] not in N or ax[1] not in N:
            continue
        u = _unit(_sub(N[ax[1]], N[ax[0]]))
        best = None
        for e, s, f in drives:
            if e not in N or s not in N or e in moving:
                continue
            for b in bl:
                h = b["b"] if b["a"] == e else b["a"] if b["b"] == e else None
                if h is None or h not in moving or h in ax:
                    continue
                d = math.dist(N[h], N[e])
                if best is None or d < best[0]:
                    best = (d, e, s, f, h)
        if not best or not u:
            continue
        _, e, s, f, h = best
        de = _unit(_sub(N[e], N[s]))
        if not de:
            continue
        de = [x * (1 if f > 0 else -1) for x in de]
        he = _sub(N[h], N[e])
        r = _sub(N[h], N[ax[0]])
        den = _dot(he, _cross(u, r))
        if abs(den) < 1e-9:
            continue
        out[w["name"]] = {"turn": _dot(he, de) / den, "rack_end": e, "tie_rod_end": h}
    return out


def tune_tie_rods(model, new_id, files, base=None, min_gain=0.3):
    """Bump steer out of the built vehicle: each front wheel's rack end (the node its tie rod ends on, direction())
    is moved, in height and fore-aft place, to where the wheel's toe changes least over its travel
    (kinematics.bump_toe), the rack end of the opposite wheel the mirror image, and the slide nodes on the rack's
    rail onto the new rail. A conversion keeps the SVJ's link vectors but not the instant centre they came from,
    and the toe changed 0.1-0.4 deg per 10 mm of bump (a vanilla car's 0.02): the cars steered themselves as the front
    dived. Applied only where it cuts the bump steer by min_gain or more. Returns (notes, wheels tuned); the files are
    changed in place. The vehicle's files are read back with beamng.add_files."""
    import json
    from . import export
    from .convert import _nelder_mead
    beamng.add_files(json.dumps(files))
    configured = json.loads(beamng.configure(new_id))
    N = configured["geometry"]["nodes"]
    dirs = direction(new_id, configured)
    bl = rigidity.beams(new_id, configured)
    springs = values.springs_and_dampers(new_id, configured)
    notes, tuned, deltas = [], [], {}
    owner = configured["geometry"].get("parts") or {}
    for w in configured.get("wheels") or []:
        if w["centre"][0] <= 0:                             # the left wheels are solved, the right ones mirror them
            continue
        twin = next((x for x in configured["wheels"] if x["name"][0] == w["name"][0] and x["centre"][0] < 0), None)
        e, e2 = _tie_inner(new_id, configured, w, dirs, owner), None
        if twin:
            e2 = _tie_inner(new_id, configured, twin, dirs, owner)
        if not e:
            continue
        base_toe = kinematics_bump(new_id, configured, w, None, bl, springs)
        if base_toe is None or abs(base_toe) < 0.03:
            continue
        e0 = N[e]

        def cost(v):
            n2 = dict(N)
            n2[e] = [e0[0], v[0], v[1]]
            t = kinematics_bump(new_id, configured, w, n2, bl, springs)
            return 1e3 if t is None else t * t + 2.0 * math.dist(n2[e], e0) ** 2
        best, _ = _nelder_mead(cost, [e0[1], e0[2]], 0.03)
        if math.dist([e0[0], best[0], best[1]], e0) > 0.15:
            continue
        n2 = dict(N)
        n2[e] = [e0[0], best[0], best[1]]
        after = kinematics_bump(new_id, configured, w, n2, bl, springs)
        if after is None or abs(after) > (1 - min_gain) * abs(base_toe):
            continue
        for node, pos in ((e, n2[e]), (e2, [N[e2][0], best[0], best[1]] if e2 else None)):
            if node and pos:
                deltas[node] = [pos[i] - N[node][i] for i in range(3)]
        notes.append(f"{w['name']}: rack end {e} moved {math.dist(e0, n2[e]) * 1000:.0f} mm for the least bump steer "
                     f"({base_toe:+.3f} deg per 10 mm of bump, {after:+.3f} now)")
        tuned.append(w["name"])
    if not deltas:
        return notes, tuned
    # the slide nodes on the rack's rail follow the new rail
    moved = {n: [N[n][i] + d[i] for i in range(3)] for n, d in deltas.items()}
    for sn, ra, rb, _ in configured["geometry"].get("slides") or []:
        if (ra in moved or rb in moved) and sn in N and ra in N and rb in N:
            a, b = moved.get(ra, N[ra]), moved.get(rb, N[rb])
            ab = [b[i] - a[i] for i in range(3)]
            l2 = sum(t * t for t in ab)
            if l2 > 1e-9:
                t = max(0.0, min(1.0, sum((N[sn][i] - a[i]) * ab[i] for i in range(3)) / l2))
                q = [a[i] + t * ab[i] for i in range(3)]
                deltas[sn] = [q[i] - N[sn][i] for i in range(3)]
    for path, text in list(files.items()):
        if not path.endswith(".jbeam"):
            continue
        doc = json.loads(text)
        hit = False
        for part in doc.values():
            if isinstance(part, dict) and export._fit_nodes(part, deltas, {}):
                hit = True
        if hit:
            files[path] = json.dumps(doc, indent=1)
    return notes, tuned


def _tie_inner(model, configured, wheel, dirs, owner):
    """The node a wheel's tie rod (a toe link at the rear) ends on, inboard: a front wheel's rack end (direction());
    else the inner node the role table names for its suspension (roles.for_corner), else the archetype's own."""
    d = dirs.get(wheel["name"])
    if d:
        return d["rack_end"]
    N = configured["geometry"]["nodes"]
    from . import roles
    rt = roles.for_corner(owner, N, wheel)
    if rt and rt["pivots"].get("tie_rod"):
        return rt["pivots"]["tie_rod"][0]
    own = f"bf{wheel['name']}tie"
    return own if own in N else None


def kinematics_bump(model, configured, wheel, nodes, bl, springs):
    return kinematics.bump_toe(model, configured, wheel, nodes, bl, springs)


def fix_files(model, base_configured, new_id, files):
    """Check the written vehicle's steering against its base's and reverse what turns the other way (fix). The
    vehicle's files are read back with beamng.add_files, so they must be what is written. Returns (notes, wheels
    flipped); on a flip the files are changed in place."""
    import json
    beamng.add_files(json.dumps(files))
    built = json.loads(beamng.configure(new_id))
    base_dir = direction(model, base_configured)
    notes, flipped = fix(files, new_id, base_dir, direction(new_id, built))
    if flipped:
        beamng.add_files(json.dumps(files))
        built = json.loads(beamng.configure(new_id))
    lnotes = lock(files, model, base_configured, base_dir, new_id, built, direction(new_id, built))
    return notes + lnotes, flipped


def lock(files, model, base_configured, base, new_id, built_configured, built):
    """The rack hydros' factors rescaled so every steered wheel turns as far at full lock as the base's: a hydro moves
    its rack end by its factor times the length of the beam it works on, and a rack wider than the base's (the SVJ's
    track, an archetype's rack ends) lengthens that beam: the Civic's lock went from 28 to 35 degrees, to where the tie
    rod is nearly over centre. Changes the files in place; returns notes."""
    import json
    notes = []
    bN, nN = base_configured["geometry"]["nodes"], built_configured["geometry"]["nodes"]
    bdr = {e: (s, f) for e, s, f in rack_drives(model, base_configured)}
    ndr = {e: (s, f) for e, s, f in rack_drives(new_id, built_configured)}
    scale = {}
    for name, b in built.items():
        a = base.get(name)
        if not a or not b["turn"] or a["rack_end"] not in bdr or b["rack_end"] not in ndr:
            continue
        (sb, fb), (sn, fn) = bdr[a["rack_end"]], ndr[b["rack_end"]]
        if not (a["rack_end"] in bN and sb in bN and b["rack_end"] in nN and sn in nN):
            continue
        lock_base = abs(a["turn"] * fb) * math.dist(bN[a["rack_end"]], bN[sb])
        k = lock_base / (abs(b["turn"]) * math.dist(nN[b["rack_end"]], nN[sn]) * abs(fn))
        if abs(k - 1) > 0.03 and 0.3 < k < 3:
            scale[b["rack_end"]] = k
            notes.append(f"{name}: lock {math.degrees(abs(b['turn'] * fn) * math.dist(nN[b['rack_end']], nN[sn])):.0f} deg of the "
                         f"wider rack brought back to the base's {math.degrees(lock_base):.0f} deg (rack drive of {b['rack_end']} x {k:.2f})")
    for path, text in list(files.items()):
        if not scale or not path.endswith(".jbeam"):
            continue
        doc = json.loads(text)
        hit = False
        for part in doc.values():
            t = part.get("hydros") if isinstance(part, dict) else None
            if not (isinstance(t, list) and t and isinstance(t[0], list)):
                continue
            for row in t[1:]:
                if isinstance(row, list) and row and row[0] in scale and isinstance(row[-1], dict)                         and isinstance(row[-1].get("factor"), (int, float)):
                    row[-1]["factor"] = round(row[-1]["factor"] * scale[row[0]], 5)
                    hit = True
        if hit:
            files[path] = json.dumps(doc, indent=1)
    return notes


def fix(files, new_id, base, built):
    """The built vehicle's files with the hydros of every steered wheel that turns the other way than the base's
    negated (in place); returns (notes, wheels flipped). base / built: direction() of the base and of the
    built vehicle, by wheel name."""
    import json
    notes, flipped = [], []
    for name, b in built.items():
        a = base.get(name)
        if not a or a["turn"] * b["turn"] >= 0:
            continue
        flipped.append(name)
        e = b["rack_end"]
        for path, text in list(files.items()):
            if not path.endswith(".jbeam"):
                continue
            doc = json.loads(text)
            hit = False
            for part in doc.values():
                t = part.get("hydros") if isinstance(part, dict) else None
                if not (isinstance(t, list) and t and isinstance(t[0], list)):
                    continue
                head = [str(h).rstrip(":") for h in t[0]]
                for row in t[1:]:
                    if isinstance(row, list) and row and row[0] == e and isinstance(row[-1], dict) \
                            and isinstance(row[-1].get("factor"), (int, float)):
                        row[-1]["factor"] = -row[-1]["factor"]
                        hit = True
            if hit:
                files[path] = json.dumps(doc, indent=1)
        notes.append(f"{name}: the SVJ's tie rod turns the wheel the other way than the base's (its tie rod end is on the other "
                     f"side of the steering axis): the rack drive of {e} reversed")
    return notes, flipped
