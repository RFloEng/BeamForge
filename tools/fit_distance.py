"""How far a fitted vehicle's skin is from the SVJ's body mesh: the measure a better body fit has to beat.

    python tools/fit_distance.py <car.svj.json> --base covet [--config rally_gravel] [--vehicles <BeamNG>/content/vehicles]
        [--stages wheelbase,body,pickups] [--png out.png] [--csv out.csv]

The skin is the base's collision skin: the nodes of the collision triangles of its body parts (the outside of the car, the part a
mesh must cover; parts inside it, seats, engine and the like, are left out). For each skin node the distance to the SVJ's body mesh
(the body's outer shell, without its wheels, interior and running gear) is measured along the mesh's normal there, in metres, positive
outside the mesh: a node should sit a little inside the mesh (the skin is under the paint). Three states are compared: the base as it is
placed on the SVJ (front axle line and ground), after the fit's wheelbase stage, and after all the stages asked for.

The numbers per state: the share of skin nodes within 2 cm / 5 cm / 10 cm of where they should be (OFFSET inside the mesh), the mean
and 90th percentile of the error, the worst nodes, and by region (front / middle / rear thirds, and roof / sides / under). --png draws
the error over the car from the side and from above.
"""

import argparse
import json
import math
import os
import re
import sys

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from beamforge import beamng, fit, gltf, jbeam, pipeline, svj as svjmod, unified  # noqa: E402

TEXT = (".jbeam", ".pc", ".json")
OFFSET = -0.02            # m: where a skin node sits relative to the visual mesh (inside it); the error is measured from here
INSIDE_PARTS = re.compile(r"interior|seat|occupant|engine|transmission|transaxle|fuel|radiator|exhaust|battery|shifter|cargo|oilpan|intake|"
                          r"driveshaft|differential|suspension|wheel|tire|brake|hub|steering|mirror|light|lamp", re.I)
MESH_INSIDE = re.compile(r"int_|interior|seat|cockpit|driver|helmet|steer|dash|engine|motor|roll_?cage|rollcage|pedal|gear|exhaust|undertray|"
                         r"^wheel|tyre|tire|rim|disc|brake|caliper|susp|hub_|camera|damage|wiper|mirror", re.I)


def load_zip(path):
    import zipfile
    with zipfile.ZipFile(path) as z:
        beamng.add_files(json.dumps({n: z.read(n).decode("utf-8-sig", "replace") for n in z.namelist() if n.lower().endswith(TEXT)}))


def skin_nodes(model, cv):
    """{node: part}: the nodes of the body parts' collision triangles."""
    parts = beamng._parts_held(model)
    out = {}
    for p in [v for v in cv["pc"]["parts"].values() if v and v in parts]:
        if INSIDE_PARTS.search(p):
            continue
        t = parts[p]["part"].get("triangles")
        if isinstance(t, list) and t:
            for r in jbeam.expand_table(t):
                for k in ("id1", "id2", "id3"):
                    if r.get(k):
                        out.setdefault(r[k], p)
    return out


def outer_nodes(nodes, ids, radius=0.06):
    """The skin nodes on the outside: no other skin node lies beyond them, within `radius` of the line through them, along one of the six
    axis directions (the sides, the roof, the underside, the nose and the tail). Inner structure (floor, bulkheads, pillars) is behind others."""
    xyz = np.array([nodes[i] for i in ids])
    out = np.zeros(len(ids), bool)
    for axis in range(3):
        others = [a for a in range(3) if a != axis]
        tree = cKDTree(xyz[:, others])
        for sign in (1, -1):
            for i in range(len(ids)):
                if out[i]:
                    continue
                near = tree.query_ball_point(xyz[i, others], radius)
                if all((xyz[j, axis] - xyz[i, axis]) * sign <= 0.01 for j in near):
                    out[i] = True
    return [i for i, o in zip(ids, out) if o]


def body_triangles(svj, path, place):
    """The SVJ body mesh's triangles in BeamNG coordinates: array (n, 3, 3), without the interior, wheels and running gear."""
    glb = path.lower().endswith(".glb")
    with open(path, "rb") as fh:
        data = fh.read()
    axes = svjmod.gltf_axes(svj)
    off = svjmod.mesh_offset(svj, data, glb)[0] or [0.0, 0.0, 0.0]
    inside = MESH_INSIDE
    skip = re.compile(gltf.not_body(data, glb).pattern + "|" + inside.pattern, re.I)
    pos, _, prims = gltf.textured(data, is_glb=glb, under=None, skip=skip)
    pts = np.array([svjmod.from_sae([a + b for a, b in zip(svjmod.gltf_to_sae(p, axes), off)], place["yf"], place["ground"]) for p in pos])
    idx = np.array([i for _, ix in prims for i in ix]).reshape(-1, 3)
    return pts[idx]


def surface_samples(tri, spacing=0.03):
    """Points over the triangles, about `spacing` apart, with each triangle's unit normal: (points (m, 3), normals (m, 3))."""
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    n = np.cross(b - a, c - a)
    area = np.linalg.norm(n, axis=1)
    keep = area > 1e-12
    a, b, c, n, area = a[keep], b[keep], c[keep], n[keep], area[keep]
    n = n / area[:, None]
    edge = np.maximum.reduce([np.linalg.norm(b - a, axis=1), np.linalg.norm(c - b, axis=1), np.linalg.norm(a - c, axis=1)])
    steps = np.minimum(np.ceil(edge / spacing).astype(int), 12)
    P, N = [(a + b + c) / 3, a, b, c], [n, n, n, n]
    for k in range(2, 13):                                   # a grid of barycentric points on the triangles that need it
        sel = steps >= k
        if not sel.any():
            break
        for i in range(1, k):
            for j in range(1, k - i):
                u, v = i / k, j / k
                P.append(a[sel] * (1 - u - v)[None] + b[sel] * u + c[sel] * v if np.ndim(u) else a[sel] * (1 - u - v) + b[sel] * u + c[sel] * v)
                N.append(n[sel])
    return np.vstack(P), np.vstack(N)


def errors(nodes, ids, tree, normals, pts):
    """Per node: the distance along the mesh's normal at the nearest surface point, positive outside, less OFFSET (so 0 is where it should be)."""
    xyz = np.array([nodes[i] for i in ids])
    d, k = tree.query(xyz)
    signed = np.einsum("ij,ij->i", xyz - pts[k], normals[k])
    return signed - OFFSET, d, xyz


def summary(e):
    a = np.abs(e)
    return {"n": len(e), "within_2cm": float((a <= 0.02).mean()), "within_5cm": float((a <= 0.05).mean()), "within_10cm": float((a <= 0.10).mean()),
            "mean_abs": float(a.mean()), "p90": float(np.percentile(a, 90)), "max": float(a.max()), "mean_signed": float(e.mean())}


def regions(e, xyz):
    y = xyz[:, 1]
    lo, hi = np.percentile(y, [0], axis=0)[0], np.percentile(y, [100])[0]
    third = lo + (hi - lo) * np.array([1 / 3, 2 / 3])
    z = xyz[:, 2]
    zr = np.percentile(z, [33, 80])
    out = {}
    for name, m in (("front third", y < third[0]), ("middle third", (y >= third[0]) & (y < third[1])), ("rear third", y >= third[1]),
                    ("low (under, sills)", z < zr[0]), ("belt (sides)", (z >= zr[0]) & (z < zr[1])), ("high (roof, screens)", z >= zr[1])):
        if m.any():
            out[name] = (int(m.sum()), float(np.abs(e[m]).mean()), float(e[m].mean()))
    return out


def plot(path, states, xyz_by):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(len(states), 2, figsize=(13, 3.6 * len(states)), squeeze=False)
    for r, (label, e) in enumerate(states):
        xyz = xyz_by[label]
        for c, (ix, iy, title) in enumerate(((1, 2, "side (y rear ->, z up)"), (1, 0, "from above (y rear ->, x left)"))):
            s = ax[r][c].scatter(xyz[:, ix], xyz[:, iy], c=np.clip(e, -0.15, 0.15), cmap="coolwarm", s=9, vmin=-0.15, vmax=0.15)
            ax[r][c].set_aspect("equal")
            ax[r][c].set_title(f"{label}: {title}", fontsize=9)
    fig.colorbar(s, ax=ax, shrink=0.6, label="error (m): red outside the mesh, blue inside")
    fig.savefig(path, dpi=100, bbox_inches="tight")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("svj")
    ap.add_argument("--base", required=True)
    ap.add_argument("--config")
    ap.add_argument("--vehicles", default=os.environ.get("BEAMNG_VEHICLES"))
    ap.add_argument("--stages", default=",".join(pipeline.STAGES))
    ap.add_argument("--png")
    ap.add_argument("--csv")
    ap.add_argument("--all-skin", action="store_true", help="measure every collision-skin node, the inner structure too")
    a = ap.parse_args(argv)
    if not a.vehicles or not os.path.isdir(a.vehicles):
        ap.error("--vehicles (or BEAMNG_VEHICLES) must be the game's vehicles folder")
    svj = json.load(open(a.svj, encoding="utf-8"))
    unified.adopt(svj)
    beamng.reset()
    load_zip(os.path.join(a.vehicles, "common.zip"))
    load_zip(os.path.join(a.vehicles, a.base + ".zip"))
    v = json.loads(beamng.configure(a.base, a.config))
    files = pipeline.svj_mesh_files(svj, os.path.dirname(os.path.abspath(a.svj)))
    if not files:
        print("the SVJ has no mesh file beside it")
        return 1
    path = next(iter(files.values()))
    skin = skin_nodes(a.base, v)
    all_ids = sorted(skin)
    ids = all_ids if a.all_skin else outer_nodes(v["geometry"]["nodes"], all_ids)
    print(f"{a.base}: {len(all_ids)} skin nodes (collision triangles of {len(set(skin.values()))} body parts), {len(ids)} of them on the outside")
    results = {}
    for label, stages in (("base placed", []), ("wheelbase only", ["wheelbase"]), ("fitted", [s for s in a.stages.split(",") if s])):
        if stages:
            r = json.loads(fit.fit_json(json.dumps(v["geometry"]), json.dumps(v["wheels"]), json.dumps(svj), json.dumps(files), json.dumps(stages)))
            place = r.get("place") or {"yf": v["measure"]["front_axle_y"], "ground": v["measure"]["ground_z"]}
            moved = {n: [p + d for p, d in zip(v["geometry"]["rest"].get(n) or v["geometry"]["nodes"][n], dl)] for n, dl in r["moves"].items()}
        else:
            place = {"yf": v["measure"]["front_axle_y"], "ground": v["measure"]["ground_z"]}
            moved = {}
        nodes = {n: moved.get(n, p) for n, p in v["geometry"]["nodes"].items()}
        if "tri" not in results:
            tri = body_triangles(svj, path, place)
        else:                                               # (the mesh stays where the first stage placed it only if the place is the same)
            tri = body_triangles(svj, path, place)
        pts, nrm = surface_samples(tri)
        tree = cKDTree(pts)
        e, d, xyz = errors(nodes, ids, tree, nrm, pts)
        results[label] = (e, xyz, summary(e), regions(e, xyz), len(tri), len(pts))
        results["tri"] = True
    print(f"SVJ body mesh: {results['fitted'][4]} triangles, {results['fitted'][5]} surface samples; ideal: nodes {abs(OFFSET) * 100:.0f} cm inside it\n")
    print(f"{'state':16s} {'<=2cm':>6s} {'<=5cm':>6s} {'<=10cm':>7s} {'mean':>7s} {'p90':>7s} {'max':>7s} {'mean signed':>12s}")
    for label in ("base placed", "wheelbase only", "fitted"):
        s = results[label][2]
        print(f"{label:16s} {s['within_2cm']:6.0%} {s['within_5cm']:6.0%} {s['within_10cm']:7.0%} {s['mean_abs'] * 100:6.1f}c {s['p90'] * 100:6.1f}c {s['max'] * 100:6.1f}c {s['mean_signed'] * 100:+11.1f}c")
    print("\nby region after the fit (nodes, mean |error|, mean signed error):")
    for k, (n, ma, ms) in results["fitted"][3].items():
        print(f"  {k:22s} {n:4d}  {ma * 100:5.1f} cm  {ms * 100:+6.1f} cm")
    e, xyz = results["fitted"][0], results["fitted"][1]
    worst = np.argsort(-np.abs(e))[:8]
    print("\nworst nodes after the fit:", ", ".join(f"{ids[i]} ({skin[ids[i]].replace(a.base + '_', '')}) {e[i] * 100:+.0f} cm" for i in worst))
    if a.csv:
        import csv
        with open(a.csv, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["node", "part", "x", "y", "z", "error_m_after_fit"])
            for i, n in enumerate(ids):
                w.writerow([n, skin[n], *[round(x, 4) for x in xyz[i]], round(float(e[i]), 4)])
    if a.png:
        plot(a.png, [(lab, results[lab][0]) for lab in ("base placed", "fitted")], {lab: results[lab][1] for lab in ("base placed", "fitted")})
        print("plot written to", a.png)
    return 0


if __name__ == "__main__":
    sys.exit(main())
