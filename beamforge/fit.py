"""Fitting a BeamNG vehicle to an SVJ, in stages (the method: docs/fitting.md).

Pure Python, standard library only (runs in Pyodide). Every stage maps node positions to new ones
and is applied to every node of the configured vehicle, so parts stay joined; the result is a move
per node, the same thing as a hand move in the editor.

  stage 1  wheelbase   y between the axle lines is scaled to the SVJ wheelbase; the front axle
                       stays, the overhangs shift with their axle (piecewise linear).
  stage 2  body        the body is fitted to the SVJ body mesh: the overhangs along y, the width
                       (about the centre line) slice by slice along y, and the heights piecewise:
                       the ground stays, the floor goes to the mesh's floor, the wheel centres to
                       the SVJ wheel centre height, the roof line to the mesh's per slice. Measured on the body nodes only (suspension, wheels,
                       drivetrain, mirrors and antennas are left out of the measurement but move
                       with the rest).
  stage 3  pickups     every SVJ hardpoint is tied to a node by its role (wheel centre: the axle
                       nodes; upright points: hub nodes; link inner points: the chassis side),
                       then those nodes go exactly onto the hardpoints and the nodes around
                       follow through a compactly supported displacement field.

Frames: BeamNG (x left, y rear, z up, m). The SVJ is placed on the vehicle as in the editor: its
origin (front-axle centre on the ground) on the base vehicle's front axle line and ground.
"""

import json
import math
import re

from beamforge import gltf
from beamforge import svj as svjmod

# parts measured out of stage 2 (they move with the body, but their nodes are not the body's shape)
NOT_BODY = re.compile(r"suspension|hub|strut|steer|swaybar|sway_bar|arb|axle|wheel|tire|tyre|brake|spring|shock|damper|"
                      r"coilover|subframe|link|trailingarm|driveshaft|halfshaft|differential|engine|transmission|"
                      r"exhaust|radiator|fueltank|seat|driver|antenna|mirror|license|plate", re.I)

SLICE = 0.10        # m, slice length along y for stage 2
SMOOTH = 2          # slices on each side in the moving average
LIMITS = (0.6, 1.6)  # scale factors outside this range are clamped (a wrong mesh must not explode the car)


def piecewise(pairs):
    """Monotone piecewise-linear map through (from, to) pairs; outside them, a shift (slope 1)."""
    pairs = sorted(pairs)

    def f(x):
        if x <= pairs[0][0]:
            return x + pairs[0][1] - pairs[0][0]
        if x >= pairs[-1][0]:
            return x + pairs[-1][1] - pairs[-1][0]
        for (a0, b0), (a1, b1) in zip(pairs, pairs[1:]):
            if x <= a1:
                return b0 + (x - a0) * (b1 - b0) / (a1 - a0) if a1 > a0 else b1
        return x
    return f


def pct(values, q):
    """The q-th percentile (0-100) of a list, by nearest rank; None if empty."""
    if not values:
        return None
    s = sorted(values)
    return s[min(len(s) - 1, max(0, int(round(q / 100 * (len(s) - 1)))))]


def axles(nodes, wheels):
    """Front and rear axle lines (y) and the ground (z) from the wheels' axle nodes and radius:
    {"yf", "yr", "ground"}; None when the vehicle has no front and rear wheels."""
    rows = []
    for w in wheels:
        a, b = nodes.get(w["node1"]), nodes.get(w["node2"])
        if a and b:
            rows.append(((a[1] + b[1]) / 2, (a[2] + b[2]) / 2, w.get("radius") or 0.0))
    if len(rows) < 2:
        return None
    ymid = sum(r[0] for r in rows) / len(rows)
    front = [r for r in rows if r[0] < ymid]
    rear = [r for r in rows if r[0] >= ymid]
    if not front or not rear:
        return None
    mean = lambda xs: sum(xs) / len(xs)  # noqa: E731
    return {"yf": mean([r[0] for r in front]), "yr": mean([r[0] for r in rear]),
            "ground": mean([r[1] - r[2] for r in rows])}


def body_nodes(nodes, parts):
    """Ids of the nodes that give the body's shape (see NOT_BODY)."""
    return [n for n in nodes if not NOT_BODY.search(parts.get(n, ""))]


def mesh_points(svj, files, yf, ground, under=None):
    """The SVJ body mesh's vertices in BeamNG coordinates, placed at the front axle line yf and ground.

    files: {mesh asset id: path of the .glb / .gltf}. The body is the chassis visual binding (its node
    and mesh); without one, every mesh. under overrides the node name.
    """
    axes = svjmod.gltf_axes(svj)
    chassis = next((b for b in svjmod.visual_bindings(svj) if b["path"] == "chassis"), None)
    pts = []
    for mid, path in files.items():
        if chassis and chassis["mesh_ref"] not in (None, mid):
            continue
        with open(path, "rb") as fh:
            data = fh.read()
        node = under or (chassis["node"] if chassis else None)
        raw = gltf.positions(data, is_glb=path.lower().endswith(".glb"), under=node)
        pts += [svjmod.from_sae(svjmod.gltf_to_sae(p, axes), yf, ground) for p in raw]
    return pts


# ---------------------------------------------------------------- stage 2 helpers

def _smooth(values):
    """Moving average over 2*SMOOTH+1 slices; None entries are filled from their neighbours first."""
    known = [i for i, v in enumerate(values) if v is not None]
    if not known:
        return values
    filled = []
    for i in range(len(values)):
        if values[i] is not None:
            filled.append(values[i])
            continue
        lo = max((k for k in known if k < i), default=None)
        hi = min((k for k in known if k > i), default=None)
        if lo is None or hi is None:
            filled.append(values[lo if hi is None else hi])
        else:
            filled.append(values[lo] + (values[hi] - values[lo]) * (i - lo) / (hi - lo))
    out = []
    for i in range(len(filled)):
        w = filled[max(0, i - SMOOTH):i + SMOOTH + 1]
        out.append(sum(w) / len(w))
    return out


def _interp(centres, values, y):
    """Linear interpolation of per-slice values at y (held flat beyond the ends)."""
    if y <= centres[0]:
        return values[0]
    if y >= centres[-1]:
        return values[-1]
    i = min(int((y - centres[0]) / SLICE), len(centres) - 2)
    t = (y - centres[i]) / SLICE
    return values[i] + (values[i + 1] - values[i]) * t


def _clamp(k):
    return min(LIMITS[1], max(LIMITS[0], k))


def _profile(points, centres, window):
    """Per slice: (half width, top) by robust percentiles of the points within `window` of its centre,
    or None with too few points."""
    out = []
    for c in centres:
        sl = [p for p in points if abs(p[1] - c) <= window]
        if len(sl) < 4:
            out.append(None)
            continue
        out.append((pct([abs(p[0]) for p in sl], 99), pct([p[2] for p in sl], 99)))
    return out


def size(points):
    """Length, width, height, front and rear y and bottom of a point cloud (robust percentiles)."""
    ys, xs, zs = [p[1] for p in points], [abs(p[0]) for p in points], [p[2] for p in points]
    return {"front": pct(ys, 0.5), "rear": pct(ys, 99.5), "length": pct(ys, 99.5) - pct(ys, 0.5),
            "width": 2 * pct(xs, 99), "bottom": pct(zs, 1), "top": pct(zs, 99.5), "height": pct(zs, 99.5) - pct(zs, 1)}


# ---------------------------------------------------------------- the fit

def fit(nodes, parts, wheels, svj, mesh=None, stages=("wheelbase", "body"), beams=None, beam_parts=None, overrides=None):
    """Fit the vehicle to the SVJ.

    nodes: {id: [x, y, z]} without moves (the configured vehicle); parts: {id: part}; wheels:
    beamng.wheels(); mesh: the body mesh points (mesh_points()), needed for "body"; beams and
    beam_parts (beamng geometry) and overrides ({"<corner>:<hardpoint>": node}), for "pickups".
    Returns {"nodes": new positions, "moves": {id: [dx, dy, dz]}, "report": [{"stage", "label",
    "base", "target", "after", "unit"}], "mapping": map_hardpoints() rows with "after" (m), "uprights":
    upright_check() rows,
    "place": {"yf", "ground"} where the SVJ was placed, "notes": [...]}.
    """
    out = {k: list(v) for k, v in nodes.items()}
    report, notes = [], []
    ax = axles(nodes, wheels)
    if ax is None:
        return {"nodes": out, "moves": {}, "report": [], "notes": ["the vehicle has no front and rear wheels to fit by"]}
    yf, yr = ax["yf"], ax["yr"]
    summary = svjmod.summary(svj)

    if "wheelbase" in stages:
        wb = summary.get("wheelbase")
        if isinstance(wb, (int, float)) and wb > 0:
            f = piecewise([(yf, yf), (yr, yf + wb)])
            for p in out.values():
                p[1] = f(p[1])
            report.append({"stage": "wheelbase", "label": "Wheelbase", "unit": "m", "base": round(yr - yf, 4),
                           "target": wb, "after": round(f(yr) - yf, 4)})
            yr = yf + wb
        else:
            notes.append("the SVJ has no chassis.wheelbase: stage 1 skipped")

    if "body" in stages:
        if not mesh:
            notes.append("no SVJ body mesh: stage 2 skipped")
        else:
            body = body_nodes(out, parts)
            before = size([out[n] for n in body])
            target = size(mesh)
            ratios = [target[k] / before[k] for k in ("length", "width", "height") if before[k] > 0]
            if not all(LIMITS[0] <= r <= LIMITS[1] for r in ratios):
                notes.append(f"the SVJ body mesh is {target['length']:.2f} x {target['width']:.2f} x {target['height']:.2f} m "
                             f"against the vehicle's {before['length']:.2f} x {before['width']:.2f} x {before['height']:.2f} m: "
                             "wrong units, axes or mesh? Stage 2 skipped")
                stages = [s_ for s_ in stages if s_ != "body"]
        if mesh and "body" in stages:
            # overhangs along y: the axle lines stay
            pairs = [(yf, yf), (yr, yr)]
            if before["front"] < yf and target["front"] < yf:
                pairs.append((before["front"], target["front"]))
            if before["rear"] > yr and target["rear"] > yr:
                pairs.append((before["rear"], target["rear"]))
            fy = piecewise(pairs)
            for p in out.values():
                p[1] = fy(p[1])
            # width and height slice by slice
            centres = []
            c = target["front"]
            while c <= target["rear"] + 1e-9:
                centres.append(c)
                c += SLICE
            if len(centres) >= 2:
                # width and roof line per slice (node clouds are sparse: a wider window on that side),
                # the floor once for the whole car (per-slice floors depend on where the nodes are)
                pb = _profile([out[n] for n in body], centres, 3 * SLICE)
                pm = _profile(mesh, centres, SLICE)
                fb, fm = size([out[n] for n in body])["bottom"], target["bottom"]
                # heights: the ground stays, the floor goes to the mesh's floor, the wheel centres to the
                # SVJ's wheel centre height (keeps the wheels on the ground), the roof line per slice
                g0 = ax["ground"]
                wc0 = sum((out[w["node1"]][2] + out[w["node2"]][2]) / 2 for w in wheels if w["node1"] in out and w["node2"] in out)
                wc0 /= max(1, sum(1 for w in wheels if w["node1"] in out and w["node2"] in out))
                hp = [h["pos"][2] for h in svjmod.hardpoints(svj, yf, g0) if h["name"] == "wheel_center"]
                wc1 = sum(hp) / len(hp) if hp else wc0
                both = [b is not None and m is not None for b, m in zip(pb, pm)]
                if any(both):
                    col = lambda prof, k: _smooth([p[k] if ok else None for p, ok in zip(prof, both)])  # noqa: E731
                    bw, bt, mw, mt = col(pb, 0), col(pb, 1), col(pm, 0), col(pm, 1)
                    for p in out.values():
                        y = p[1]
                        w0, t0, w1, t1 = (_interp(centres, v, y) for v in (bw, bt, mw, mt))
                        if w0 > 0.05:
                            p[0] *= _clamp(w1 / w0)
                        zpairs = [(g0, g0), (t0, t1)]
                        for z0, z1 in ((fb, fm), (wc0, wc1)):
                            if g0 < z0 < t0 - 0.05 and g0 < z1 < t1 - 0.05:
                                zpairs.append((z0, z1))
                        zpairs.sort()
                        if all(b0 < b1 and c0 < c1 for (b0, c0), (b1, c1) in zip(zpairs, zpairs[1:])):
                            p[2] = piecewise(zpairs)(p[2])
                else:
                    notes.append("the body nodes and the mesh do not overlap along the car: width and height not fitted")
            after = size([out[n] for n in body])
            for key, label in (("length", "Length"), ("width", "Width"), ("height", "Height"), ("bottom", "Lowest body point")):
                report.append({"stage": "body", "label": label, "unit": "m", "base": round(before[key], 4),
                               "target": round(target[key], 4), "after": round(after[key], 4)})
            report.append({"stage": "body", "label": "Front overhang", "unit": "m", "base": round(yf - before["front"], 4),
                           "target": round(yf - target["front"], 4), "after": round(yf - after["front"], 4)})
            report.append({"stage": "body", "label": "Rear overhang", "unit": "m", "base": round(before["rear"] - yr, 4),
                           "target": round(target["rear"] - yr, 4), "after": round(after["rear"] - yr, 4)})

    mapping, uprights = [], []
    if "pickups" in stages:
        hps = svjmod.hardpoints(svj, yf, ax["ground"])
        if not beams or not hps:
            notes.append("no suspension hardpoints in the SVJ" if not hps else "no beams: stage 3 skipped")
        else:
            bp = beam_parts or ["" for _ in beams]
            mapping = map_hardpoints(out, beams, bp, parts, wheels, hps, overrides)
            targets = {}
            # the rest of each hub moves rigidly with its tied points (a hub is one piece)
            mid = lambda w: [(out[w["node1"]][i] + out[w["node2"]][i]) / 2 for i in range(3)]  # noqa: E731
            for corner in {r["corner"] for r in mapping}:
                rows = [r for r in mapping if r["corner"] == corner and r["nodes"] and r["kind"] in ("wheel", "upright")]
                wheel_row = next((r for r in rows if r["kind"] == "wheel"), None)
                if not wheel_row:
                    continue
                wheel = next(w for w in wheels if [w["node1"], w["node2"]] == wheel_row["nodes"])
                src = [mid(wheel)] + [out[r["nodes"][0]] for r in rows if r["kind"] == "upright"]
                dst = [wheel_row["target"]] + [r["target"] for r in rows if r["kind"] == "upright"]
                move = rigid_fit(src, dst)
                if len(src) >= 3 and math.sqrt(sum(math.dist(move(p), q) ** 2 for p, q in zip(src, dst)) / len(src)) > HUB_TOLERANCE:
                    move = rigid_fit(src[:1], dst[:1])         # shapes too different to turn it: moved along only
                hub, _ = corner_roles(out, beams, bp, parts, wheel)
                for n in hub:
                    targets[n] = move(out[n])
                al = ((svj.get("suspension") or {}).get(corner) or {}).get("alignment") or {}
                for n, p in _place_wheel(out, wheel, wheel_row["target"], al).items():
                    targets[n] = p
            for row in mapping:
                if row["kind"] == "wheel":                 # the wheel centre: set with its hub above
                    continue
                else:
                    for n in row["nodes"]:
                        targets[n] = list(row["target"])
            before = out
            for row in mapping:                            # a wheel whose hub was not found: shifted only
                if row["kind"] == "wheel" and row["nodes"][0] not in targets:
                    a, b = (out[n] for n in row["nodes"])
                    shift = [row["target"][i] - (a[i] + b[i]) / 2 for i in range(3)]
                    for n in row["nodes"]:
                        targets[n] = [out[n][i] + shift[i] for i in range(3)]
            try:
                out = pickup_field(out, targets)
            except ValueError as exc:
                notes.append(str(exc))
            uprights = upright_check(before, out, mapping, wheels, svj, beams, bp, parts)
            off = [f"{u['corner']} ({u['shape_rms_mm']:.0f} mm)" for u in uprights
                   if u["shape_rms_mm"] is not None and u["shape_rms_mm"] > 1000 * HUB_TOLERANCE]
            if off:
                notes.append("hubs that do not match the SVJ uprights: " + ", ".join(off)
                             + "; check their ties, or pick a base vehicle with the same suspension type")
            bad = distortion(before, out, beams)
            if bad:
                worst = ", ".join(f"{a}-{b} x{r}" for r, a, b in bad[:3])
                notes.append(f"{len(bad)} beams stretched or squeezed more than 2 times (worst: {worst}): "
                             "check the hardpoint ties near them, or pick a base vehicle closer to the SVJ")
            for row in mapping:
                if row["nodes"]:
                    pts = [out[n] for n in row["nodes"]]
                    at = [sum(p[i] for p in pts) / len(pts) for i in range(3)]
                    row["after"] = round(math.dist(at, row["target"]), 4)
            far = [f"{r['corner']} {r['name'].replace('_', ' ')} ({r['nodes'][0]}, {r['distance'] * 1000:.0f} mm)"
                   for r in mapping if r["by"] == "user" and r["distance"] > 0.25]
            if far:
                notes.append("your ties far from their hardpoint: " + ", ".join(far) + "; is that the right node?")
            missing = [f"{r['corner']} {r['name']}" for r in mapping if not r["nodes"]]
            if missing:
                notes.append("no node for " + ", ".join(missing))
            done = [r for r in mapping if r["nodes"]]
            if done:
                report.append({"stage": "pickups", "label": "Pickups, largest gap", "unit": "m",
                               "base": max(r["distance"] for r in done), "target": 0.0, "after": max(r["after"] for r in done)})

    moves = {}
    for n, p in out.items():
        d = [round(p[i] - nodes[n][i], 4) for i in range(3)]
        if any(d):
            moves[n] = d
    return {"nodes": out, "moves": moves, "report": report, "mapping": mapping, "uprights": uprights, "notes": notes,
            "place": {"yf": round(yf, 4), "ground": round(ax["ground"], 4)}}


def fit_json(geometry_json, wheels_json, svj_json, files_json, stages_json, overrides_json=None):
    """fit() for the editor. geometry: beamng.configure()["geometry"] ("rest" positions are used, so
    hand moves stay out of the fit); files: {mesh asset id: path} of the SVJ meshes in the Python file
    system; stages: ["wheelbase", "body", "pickups"]; overrides: the user's hardpoint ties. Returns
    {"moves", "report", "mapping", "place", "notes"} as JSON."""
    geo, wheels, svj = json.loads(geometry_json), json.loads(wheels_json), json.loads(svj_json)
    stages = json.loads(stages_json)
    nodes = geo.get("rest") or geo["nodes"]
    ax = axles(nodes, wheels)
    mesh, notes = None, []
    if "body" in stages and ax:
        try:
            mesh = mesh_points(svj, json.loads(files_json), ax["yf"], ax["ground"]) or None
        except (OSError, ValueError, KeyError, IndexError) as exc:
            notes.append(f"could not read the SVJ body mesh: {exc}")
    r = fit(nodes, geo["parts"], wheels, svj, mesh, stages, geo.get("beams"), geo.get("beam_parts"),
            json.loads(overrides_json) if overrides_json else None)
    return json.dumps({"moves": r["moves"], "report": r["report"], "mapping": r["mapping"], "uprights": r["uprights"],
                       "place": r["place"],
                       "notes": notes + r["notes"]})


# ---------------------------------------------------------------- stage 3: pickup points

SUSPENSION = re.compile(r"suspension|strut|coilover|shock|damper|spring|steer|hub|arm|link|subframe|knuckle|upright", re.I)


def corner_roles(nodes, beams, beam_parts, parts, wheel):
    """The nodes that can carry a corner's hardpoints: (upright, chassis side).

    upright: the nodes of a suspension part beamed to the wheel's axle nodes by a suspension part's
    beam (the hub; body nodes tied to the axle by limiter beams are not). chassis side: the
    suspension parts' own nodes on that side of the car near the wheel, and the far ends of the
    suspension beams that touch them (arm mounts on the body or subframe).
    """
    axle = {wheel["node1"], wheel["node2"]}
    nb = {}
    for (a, b), bp in zip(beams, beam_parts):
        if SUSPENSION.search(bp):
            nb.setdefault(a, set()).add(b)
            nb.setdefault(b, set()).add(a)
    up = set()
    for a in axle:
        up |= {n for n in nb.get(a, ()) if SUSPENSION.search(parts.get(n, ""))}
    up -= axle
    wc = [(nodes[wheel["node1"]][i] + nodes[wheel["node2"]][i]) / 2 for i in range(3)]
    own_parts = {parts[n] for n in up}
    own = {n for n in nodes if parts.get(n) in own_parts and nodes[n][0] * wc[0] > 0 and abs(nodes[n][1] - wc[1]) < 0.9}
    side = set()
    for n in own | up:
        side |= nb.get(n, set())
    side = {n for n in (side | own) - up - axle if nodes[n][0] * wc[0] >= -0.05}
    return up, side


def map_hardpoints(nodes, beams, beam_parts, parts, wheels, hps, overrides=None):
    """Tie every SVJ hardpoint to a node of the base vehicle.

    hps: svj.hardpoints() placed on the vehicle. The wheel centre of a corner is its wheel's two axle
    nodes (the wheel nearest to it); upright hardpoints go to the hub nodes, link inner points to
    the chassis-side nodes (corner_roles), each node used once, nearest pairs first. A hardpoint at
    the same place as a link's inner point (a strut top listed twice) counts once, as a chassis
    point. overrides: {"<corner>:<name>": node id} set by the user. Returns [{"corner", "name",
    "kind", "nodes", "target", "distance", "by"}]; nodes is [] when nothing fits.
    """
    overrides = overrides or {}
    rows, used = [], set()
    for corner in sorted({h["corner"] for h in hps}):
        mine = [h for h in hps if h["corner"] == corner]
        wc = next((h for h in mine if h["name"] == "wheel_center"), None)
        if not wc or not wheels:
            continue
        mid = lambda w: [(nodes[w["node1"]][i] + nodes[w["node2"]][i]) / 2 for i in range(3)]  # noqa: E731
        wheel = min((w for w in wheels if w["node1"] in nodes and w["node2"] in nodes),
                    key=lambda w: math.dist(mid(w), wc["pos"]))
        rows.append({"corner": corner, "name": "wheel_center", "kind": "wheel", "nodes": [wheel["node1"], wheel["node2"]],
                     "target": wc["pos"], "distance": round(math.dist(mid(wheel), wc["pos"]), 4), "by": "wheel"})
        up, side = corner_roles(nodes, beams, beam_parts, parts, wheel)
        inner = [h["pos"] for h in mine if h["kind"] == "chassis"]
        todo = []
        for h in mine:
            if h["name"] == "wheel_center":
                continue
            if h["kind"] == "upright" and any(math.dist(h["pos"], p) < 0.002 for p in inner):
                continue                                   # the same point as a link's inner end
            todo.append(h)
        pairs = []
        free_up = []
        for h in todo:
            key = f"{corner}:{h['name']}"
            if key in overrides and overrides[key] in nodes:
                n = overrides[key]
                rows.append({"corner": corner, "name": h["name"], "kind": h["kind"], "nodes": [n], "target": h["pos"],
                             "distance": round(math.dist(nodes[n], h["pos"]), 4), "by": "user"})
                used.add(n)
                continue
            if h["kind"] == "upright":
                free_up.append(h)
            else:
                for n in side:
                    pairs.append((math.dist(nodes[n], h["pos"]), h["name"], n))
        # upright points: the assignment to hub nodes whose shape (with the wheel centre) best matches the
        # SVJ upright, nearness as a tie-break; nearest first when there are too many to try
        hub_free = sorted(n for n in up if n not in used)
        best = _shape_assignment(nodes, wheel, wc["pos"], free_up, hub_free)
        for h, n in best:
            rows.append({"corner": corner, "name": h["name"], "kind": h["kind"], "nodes": [n], "target": h["pos"],
                         "distance": round(math.dist(nodes[n], h["pos"]), 4), "by": "guess"})
            used.add(n)
        pairs.sort()
        done = {r["name"] for r in rows if r["corner"] == corner}
        for d, name, n in pairs:
            if name in done or n in used:
                continue
            h = next(x for x in todo if x["name"] == name)
            rows.append({"corner": corner, "name": name, "kind": h["kind"], "nodes": [n], "target": h["pos"],
                         "distance": round(d, 4), "by": "guess"})
            done.add(name)
            used.add(n)
        for h in todo:
            if h["name"] not in done:
                rows.append({"corner": corner, "name": h["name"], "kind": h["kind"], "nodes": [], "target": h["pos"],
                             "distance": None, "by": "none"})
    return rows


def _shape_assignment(nodes, wheel, wc, hps, hub, limit=20000):
    """[(hardpoint, node)] tying upright hardpoints to distinct hub nodes. With the wheel centre as a
    fixed pair, every assignment is tried and scored by the rigid-overlay gap (the shape) plus a
    quarter of the mean distance (nearness); beyond `limit` assignments, nearest pairs first."""
    from itertools import permutations
    if not hps or not hub:
        return []
    k = min(len(hps), len(hub))
    count = 1
    for i in range(k):
        count *= len(hub) - i
    mid = [(nodes[wheel["node1"]][i] + nodes[wheel["node2"]][i]) / 2 for i in range(3)]
    if len(hps) > len(hub) or count > limit or k < 2:
        out, used = [], set()
        for d, h, n in sorted(((math.dist(nodes[n], h["pos"]), id(h), n) for h in hps for n in hub)):
            hh = next(x for x in hps if id(x) == h)
            if n in used or any(x is hh for x, _ in out):
                continue
            out.append((hh, n))
            used.add(n)
        return out
    best, best_score = None, None
    dst = [wc] + [h["pos"] for h in hps]
    for perm in permutations(hub, k):
        src = [mid] + [nodes[n] for n in perm]
        f = rigid_fit(src, dst)
        shape = math.sqrt(sum(math.dist(f(p), q) ** 2 for p, q in zip(src, dst)) / len(src))
        near = sum(math.dist(nodes[n], h["pos"]) for h, n in zip(hps, perm)) / k
        score = shape + 0.25 * near
        if best_score is None or score < best_score:
            best, best_score = perm, score
    return list(zip(hps, best))


def _wendland(r, radius):
    q = r / radius
    return (1 - q) ** 4 * (4 * q + 1) if q < 1 else 0.0


def _solve(A, B):
    """Solve A X = B (B: rows of 3 values) by Gaussian elimination with partial pivoting."""
    n = len(A)
    M = [A[i][:] + list(B[i]) for i in range(n)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(M[r][c]))
        M[c], M[p] = M[p], M[c]
        if abs(M[c][c]) < 1e-12:
            raise ValueError("pickup points too close together to fit")
        for r in range(c + 1, n):
            f = M[r][c] / M[c][c]
            if f:
                M[r] = [a - f * b for a, b in zip(M[r], M[c])]
    X = [[0.0] * 3 for _ in range(n)]
    for r in range(n - 1, -1, -1):
        for k in range(3):
            X[r][k] = (M[r][n + k] - sum(M[r][c] * X[c][k] for c in range(r + 1, n))) / M[r][r]
    return X


def pickup_field(nodes, targets, radius=None):
    """Move nodes so each of `targets` ({node: [x, y, z]}) lands exactly on its target and the nodes
    around follow smoothly: a sum of compactly supported Wendland functions through every target.
    radius (m): how far the influence reaches; by default 0.4 m, or three times the largest move,
    at most 1 m (a short reach for a long move would fold the structure). Returns new positions."""
    ids = list(targets)
    if not ids:
        return {k: list(v) for k, v in nodes.items()}
    D = [[targets[n][i] - nodes[n][i] for i in range(3)] for n in ids]
    big = max(math.sqrt(sum(x * x for x in d)) for d in D)
    radius = radius or min(1.0, max(0.4, 3 * big))
    C = [nodes[n] for n in ids]
    A = [[_wendland(math.dist(a, b), radius) for b in C] for a in C]
    W = _solve(A, D)
    out = {}
    for n, p in nodes.items():
        d = [0.0, 0.0, 0.0]
        for c, w in zip(C, W):
            f = _wendland(math.dist(p, c), radius)
            if f:
                d = [d[i] + f * w[i] for i in range(3)]
        out[n] = [p[i] + d[i] for i in range(3)]
    for n in ids:                                          # exact, whatever the rounding
        out[n] = list(targets[n])
    return out


def rigid_fit(src, dst):
    """Best rotation R and translation t with R src_i + t ~ dst_i (least squares; Horn's quaternion
    method, the largest eigenvector found by power iteration). Fewer than 3 points: translation only.
    Returns a function p -> R p + t."""
    n = len(src)
    cs = [sum(p[i] for p in src) / n for i in range(3)]
    cd = [sum(p[i] for p in dst) / n for i in range(3)]
    if n < 3:
        return lambda p: [p[i] - cs[i] + cd[i] for i in range(3)]
    S = [[sum((a[i] - cs[i]) * (b[j] - cd[j]) for a, b in zip(src, dst)) for j in range(3)] for i in range(3)]
    (xx, xy, xz), (yx, yy, yz), (zx, zy, zz) = S
    Nm = [[xx + yy + zz, yz - zy, zx - xz, xy - yx],
          [yz - zy, xx - yy - zz, xy + yx, zx + xz],
          [zx - xz, xy + yx, -xx + yy - zz, yz + zy],
          [xy - yx, zx + xz, yz + zy, -xx - yy + zz]]
    shift = sum(abs(v) for row in Nm for v in row) + 1.0          # make it positive definite
    q = [1.0, 0.0, 0.0, 0.0]
    for _ in range(200):
        q2 = [sum(Nm[i][j] * q[j] for j in range(4)) + shift * q[i] for i in range(4)]
        norm_ = math.sqrt(sum(v * v for v in q2))
        q2 = [v / norm_ for v in q2]
        if sum(abs(a - b) for a, b in zip(q, q2)) < 1e-12:
            q = q2
            break
        q = q2
    w, x, y, z = q
    R = [[w * w + x * x - y * y - z * z, 2 * (x * y - w * z), 2 * (x * z + w * y)],
         [2 * (x * y + w * z), w * w - x * x + y * y - z * z, 2 * (y * z - w * x)],
         [2 * (x * z - w * y), 2 * (y * z + w * x), w * w - x * x - y * y + z * z]]

    def f(p):
        d = [p[i] - cs[i] for i in range(3)]
        return [sum(R[i][j] * d[j] for j in range(3)) + cd[i] for i in range(3)]
    return f


def distortion(before, after, beams, limit=2.0):
    """Beams whose length changed by more than `limit` times (either way):
    [(ratio, node a, node b)], worst first."""
    out = []
    for a, b in beams:
        if a in before and b in before:
            l0 = math.dist(before[a], before[b])
            if l0 > 0.01:
                r = math.dist(after[a], after[b]) / l0
                if r > limit or r < 1 / limit:
                    out.append((round(r, 2), a, b))
    return sorted(out, key=lambda x: -max(x[0], 1 / x[0]))


HUB_TOLERANCE = 0.025   # m: a hub laid over the SVJ upright with a larger rms gap does not match it


def _place_wheel(nodes, wheel, centre, alignment):
    """The two axle nodes with their midpoint on `centre`: the axis set to the SVJ static camber and
    toe (radians, alignment) where the file has both, else kept as the base vehicle's."""
    a, b = nodes[wheel["node1"]], nodes[wheel["node2"]]
    half = math.dist(a, b) / 2
    mid = [(a[i] + b[i]) / 2 for i in range(3)]
    c, t = alignment.get("camber"), alignment.get("toe")
    if not (isinstance(c, (int, float)) and isinstance(t, (int, float))):
        return {k: [nodes[k][i] - mid[i] + centre[i] for i in range(3)] for k in (wheel["node1"], wheel["node2"])}
    outer, inner = (wheel["node1"], wheel["node2"]) if abs(a[0]) >= abs(b[0]) else (wheel["node2"], wheel["node1"])
    side = 1.0 if nodes[outer][0] >= 0 else -1.0
    axis = [side * math.cos(c) * math.cos(t), -math.cos(c) * math.sin(t), -math.sin(c)]   # outboard, as _wheel_angles reads it
    return {outer: [centre[i] + axis[i] * half for i in range(3)], inner: [centre[i] - axis[i] * half for i in range(3)]}


def _wheel_angles(nodes, wheel):
    """Camber and toe (deg) of a wheel from its axle nodes, as the suspension module measures them:
    camber negative with the top leaning inboard, toe positive toe-in; right wheels mirrored."""
    a, b = nodes[wheel["node1"]], nodes[wheel["node2"]]
    out_, in_ = (a, b) if abs(a[0]) >= abs(b[0]) else (b, a)
    d = [out_[i] - in_[i] for i in range(3)]
    if out_[0] < 0:
        d[0] = -d[0]
    n = math.sqrt(sum(x * x for x in d)) or 1.0
    d = [x / n for x in d]
    return -math.degrees(math.atan2(d[2], math.hypot(d[0], d[1]))), -math.degrees(math.atan2(d[1], d[0]))


def upright_check(before, after, mapping, wheels, svj, beams, beam_parts, parts):
    """How well each corner's hub matches the SVJ upright.

    Per corner, from the upright points tied to nodes (the wheel centre and the upright hardpoints):
    "shape_rms_mm" / "shape_max_mm": what is left when the base hub (before the fit) is laid over the
    SVJ upright by the best rigid move, i.e. the difference in shape (0 for the same upright; None
    with fewer than 3 points). "worst_pair": the two points whose distance differs most, with the
    base and SVJ distances (mm). "hub_beams_pct": after the fit, the largest length change of a beam
    inside the hub. "camber_deg" / "toe_deg": the fitted wheel, from its axle nodes, with the SVJ
    static alignment beside it where the file has one ("svj_camber_deg", "svj_toe_deg").
    """
    out = []
    susp = svj.get("suspension") or {}
    for corner in sorted({r["corner"] for r in mapping}):
        rows = [r for r in mapping if r["corner"] == corner and r["nodes"] and r["kind"] in ("wheel", "upright")]
        wrow = next((r for r in rows if r["kind"] == "wheel"), None)
        if not wrow:
            continue
        wheel = next(w for w in wheels if [w["node1"], w["node2"]] == wrow["nodes"])
        mid = lambda N: [(N[wheel["node1"]][i] + N[wheel["node2"]][i]) / 2 for i in range(3)]  # noqa: E731
        ups = [r for r in rows if r["kind"] == "upright"]
        names = ["wheel_center"] + [r["name"] for r in ups]
        base = [mid(before)] + [before[r["nodes"][0]] for r in ups]
        svjp = [wrow["target"]] + [r["target"] for r in ups]
        row = {"corner": corner, "points": names, "shape_rms_mm": None, "shape_max_mm": None, "worst_pair": None}
        if len(base) >= 3:
            f = rigid_fit(base, svjp)
            res = [math.dist(f(p), q) for p, q in zip(base, svjp)]
            row["shape_rms_mm"] = round(1000 * math.sqrt(sum(r * r for r in res) / len(res)), 1)
            row["shape_max_mm"] = round(1000 * max(res), 1)
        if len(base) >= 2:
            pairs = [(i, j) for i in range(len(base)) for j in range(i + 1, len(base))]
            i, j = max(pairs, key=lambda ij: abs(math.dist(base[ij[0]], base[ij[1]]) - math.dist(svjp[ij[0]], svjp[ij[1]])))
            row["worst_pair"] = [names[i], names[j], round(1000 * math.dist(base[i], base[j])), round(1000 * math.dist(svjp[i], svjp[j]))]
        hub, _ = corner_roles(before, beams, beam_parts, parts, wheel)
        inside = hub | {wheel["node1"], wheel["node2"]}
        changes = [abs(math.dist(after[a], after[b]) / math.dist(before[a], before[b]) - 1)
                   for a, b in beams if a in inside and b in inside and math.dist(before[a], before[b]) > 0.01]
        row["hub_beams_pct"] = round(100 * max(changes), 1) if changes else None
        row["camber_deg"], row["toe_deg"] = (round(x, 2) for x in _wheel_angles(after, wheel))
        al = (susp.get(corner) or {}).get("alignment") or {}
        row["svj_camber_deg"] = round(math.degrees(al["camber"]), 2) if isinstance(al.get("camber"), (int, float)) else None
        row["svj_toe_deg"] = round(math.degrees(al["toe"]), 2) if isinstance(al.get("toe"), (int, float)) else None
        out.append(row)
    return out
