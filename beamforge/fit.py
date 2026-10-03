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
  stage 3  pickup points: next (exact hardpoints through a local displacement field).

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

def fit(nodes, parts, wheels, svj, mesh=None, stages=("wheelbase", "body")):
    """Fit the vehicle to the SVJ.

    nodes: {id: [x, y, z]} without moves (the configured vehicle); parts: {id: part}; wheels:
    beamng.wheels(); mesh: the body mesh points (mesh_points()), needed for "body". Returns
    {"nodes": new positions, "moves": {id: [dx, dy, dz]}, "report": [{"stage", "label", "base",
    "target", "after", "unit"}], "notes": [...]}.
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

    moves = {}
    for n, p in out.items():
        d = [round(p[i] - nodes[n][i], 4) for i in range(3)]
        if any(d):
            moves[n] = d
    return {"nodes": out, "moves": moves, "report": report, "notes": notes}


def fit_json(geometry_json, wheels_json, svj_json, files_json, stages_json):
    """fit() for the editor. geometry: beamng.configure()["geometry"] ("rest" positions are used, so
    hand moves stay out of the fit); files: {mesh asset id: path} of the SVJ meshes in the Python file
    system; stages: ["wheelbase", "body"]. Returns {"moves", "report", "notes"} as JSON."""
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
    r = fit(nodes, geo["parts"], wheels, svj, mesh, stages)
    return json.dumps({"moves": r["moves"], "report": r["report"], "notes": notes + r["notes"]})
