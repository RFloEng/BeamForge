"""Which way a front wheel turns when the rack moves, and fixing it when a build turns it the wrong way.

Pure Python, standard library only (runs in Pyodide). A wheel steers about its steering axis (the nodes
steerAxisDown and steerAxisUp of its pressureWheels row: the hub's ball joint and the strut top). The tie rod
(a beam from the hub's tie rod end to the rack end) turns it: moving the rack end by d turns the hub by

    dtheta = ((h - e) . d) / ((h - e) . (u x r))

with h the tie rod end, e the rack end, u the axis direction (down to up) and r = h - (a point of the axis):
the sign depends on whether the tie rod end is ahead of or behind the axis, and on which side of it the rod
runs. A base vehicle's rack drive (the hydros that move the rack ends) is built for its own geometry; an SVJ's
geometry on it can turn the wheels the other way (the SVJ's tie rod end ahead of the axle on a base whose is
behind it: a Z3 on the RWD coupe steered backwards). direction() gives the sign per wheel, and fix() negates the
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


def fix_files(model, base_configured, new_id, files):
    """Check the written vehicle's steering against its base's and reverse what turns the other way (fix). The
    vehicle's files are read back with beamng.add_files, so they must be what is written. Returns (notes, wheels
    flipped); on a flip the files are changed in place."""
    import json
    beamng.add_files(json.dumps(files))
    built = json.loads(beamng.configure(new_id))
    notes, flipped = fix(files, new_id, direction(model, base_configured), direction(new_id, built))
    return notes, flipped


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
