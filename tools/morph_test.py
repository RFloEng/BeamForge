"""A prototype of fitting the body by morphing: the outer skin nodes are pulled onto the SVJ's body mesh, every other node follows a smooth map.

    python tools/morph_test.py <car.svj.json> --base covet [--config rally_gravel] [--vehicles ...] [--smooth 0.001,0.01,0.05,0.2]

Steps (compared with the fit's body stage, tools/fit_distance.py measures its skin): start from the base with the wheelbase stage done;
for each outer skin node take the point on the mesh nearest it, `OFFSET` inside the surface, as its target; fit a biharmonic radial basis
function (phi(r) = r, with an affine part) through the skin nodes' displacements, with a smoothing term (so noise in the projections does not
become wrinkles) and the running gear held where it is; move every node by the map. Reported for each smoothing: how far the skin is from
the mesh afterwards (as fit_distance.py) and how much the body's beams changed length (a beam stretched or squeezed is a beam whose
stiffness and weight must follow). The fit's own result is printed for comparison.
"""

import argparse
import json
import os
import re
import sys

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fit_distance as fd  # noqa: E402
from beamforge import beamng, fit, pipeline, rigidity, unified  # noqa: E402

GEAR = re.compile(r"suspension|coilover|shock|damper|spring|hub|knuckle|upright|wheeldata|brake|swaybar|sway_bar|axle|wheel|tire|tyre|steering|rack|"
                  r"halfshaft|driveshaft|differential|transaxle|transmission|engine|enginemounts", re.I)


def rbf_map(ctrl, disp, smooth):
    """The biharmonic RBF through control points `ctrl` (m, 3) with displacements `disp` (m, 3), smoothed by `smooth` (m): a function of points."""
    m = len(ctrl)
    D = np.linalg.norm(ctrl[:, None, :] - ctrl[None, :, :], axis=2)
    P = np.hstack([np.ones((m, 1)), ctrl])
    A = np.zeros((m + 4, m + 4))
    A[:m, :m] = D + smooth * np.eye(m)
    A[:m, m:] = P
    A[m:, :m] = P.T
    rhs = np.zeros((m + 4, 3))
    rhs[:m] = disp
    sol = np.linalg.solve(A, rhs)
    w, c = sol[:m], sol[m:]

    def f(x):
        d = np.linalg.norm(x[:, None, :] - ctrl[None, :, :], axis=2)
        return d @ w + np.hstack([np.ones((len(x), 1)), x]) @ c
    return f


def relax(X, L0, ia, ib, fixed, skin, target, lo=0.75, hi=1.35, iters=80, pull=0.25):
    """Push the beams whose length ratio (to L0) is outside [lo, hi] back inside it, node by node (Jacobi: each node takes the mean of the corrections
    its beams ask for), while the skin nodes are pulled toward their targets by `pull` of the way each round; `fixed` nodes do not move.
    X (n, 3), L0 per beam (the lengths before the morph), ia / ib the beams' node indices, skin the indices of the skin nodes, target their positions."""
    X = X.copy()
    n = len(X)
    free = np.ones(n, bool)
    free[fixed] = False
    deg = np.maximum(np.bincount(np.concatenate([ia, ib]), minlength=n), 1)
    for _ in range(iters):
        d = X[ib] - X[ia]
        L = np.maximum(np.linalg.norm(d, axis=1), 1e-9)
        Lt = np.clip(L, lo * L0, hi * L0)
        corr = ((L - Lt) / L)[:, None] * d * 0.5
        move = np.zeros_like(X)
        np.add.at(move, ia, corr)
        np.add.at(move, ib, -corr)
        X += (move / deg[:, None]) * 1.5 * free[:, None]
        X[skin] += pull * (target - X[skin])
    return X


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("svj")
    ap.add_argument("--base", required=True)
    ap.add_argument("--config")
    ap.add_argument("--vehicles", default=os.environ.get("BEAMNG_VEHICLES"))
    ap.add_argument("--smooth", default="0.001,0.01,0.05,0.2")
    a = ap.parse_args(argv)
    svj = json.load(open(a.svj, encoding="utf-8"))
    unified.adopt(svj)
    beamng.reset()
    fd.load_zip(os.path.join(a.vehicles, "common.zip"))
    fd.load_zip(os.path.join(a.vehicles, a.base + ".zip"))
    v = json.loads(beamng.configure(a.base, a.config))
    files = pipeline.svj_mesh_files(svj, os.path.dirname(os.path.abspath(a.svj)))
    path = next(iter(files.values()))
    geo = v["geometry"]
    skin = fd.skin_nodes(a.base, v)
    all_ids = sorted(skin)
    outer = fd.outer_nodes(geo["nodes"], all_ids)
    # the starting point: the wheelbase stage (the body stage is what is being replaced) and the full fit, for comparison
    def moved(stages):
        r = json.loads(fit.fit_json(json.dumps(geo), json.dumps(v["wheels"]), json.dumps(svj), json.dumps(files), json.dumps(stages)))
        place = r.get("place") or {"yf": v["measure"]["front_axle_y"], "ground": v["measure"]["ground_z"]}
        base = geo.get("rest") or geo["nodes"]
        return {n: [p + d for p, d in zip(base[n], r["moves"].get(n, [0, 0, 0]))] for n in geo["nodes"]}, place
    start, place = moved(["wheelbase"])
    full, place_full = moved(list(pipeline.STAGES))
    tri = fd.body_triangles(svj, path, place)
    pts, nrm = fd.surface_samples(tri)
    tree = cKDTree(pts)
    ids = list(geo["nodes"])
    owner = geo.get("parts") or {}
    X0 = np.array([start[n] for n in ids])
    # targets for the outer nodes
    out_idx = [ids.index(n) for n in outer]
    P = X0[out_idx]
    d, k = tree.query(P)
    q = pts[k] + nrm[k] * fd.OFFSET                                  # on the surface, OFFSET inside it
    disp = np.clip(q - P, -0.25, 0.25)                               # (a projection that jumps further is a wrong surface)
    anchors = [i for i, n in enumerate(ids) if GEAR.search(owner.get(n, "")) and n not in set(outer)]
    ctrl = np.vstack([P, X0[anchors]])
    cd = np.vstack([disp, np.zeros((len(anchors), 3))])
    # beams of the body: their length before (the base placed, nothing moved) and after
    bl = [b for b in rigidity.beams(a.base, v) if not GEAR.search(b["part"]) and b["a"] in start and b["b"] in start and b["values"].get("beamSpring", 0) > 0]
    base_nodes = geo.get("rest") or geo["nodes"]
    L0 = np.array([np.linalg.norm(np.array(base_nodes[b["a"]]) - np.array(base_nodes[b["b"]])) for b in bl])

    def stretch(nodes):
        L1 = np.array([np.linalg.norm(np.array(nodes[b["a"]]) - np.array(nodes[b["b"]])) for b in bl])
        ok = L0 > 0.03
        r = L1[ok] / L0[ok]
        return {"p5": np.percentile(r, 5), "p50": np.percentile(r, 50), "p95": np.percentile(r, 95), "min": r.min(), "max": r.max(),
                "outside_0.7_1.4": float(((r < 0.7) | (r > 1.4)).mean())}

    def skin_err(nodes):
        e, _, _ = fd.errors(nodes, outer, tree, nrm, pts)
        s = fd.summary(e)
        return s

    def line(label, s, st):
        print(f"{label:22s} {s['within_2cm']:5.0%} {s['within_5cm']:5.0%} {s['mean_abs'] * 100:5.1f}c {s['p90'] * 100:5.1f}c {s['mean_signed'] * 100:+5.1f}c |"
              f" {st['p5']:5.2f} {st['p50']:5.2f} {st['p95']:5.2f}  {st['min']:5.2f} {st['max']:5.2f}  {st['outside_0.7_1.4']:6.1%}")
    print(f"{a.base}: {len(outer)} outer skin nodes, {len(anchors)} running-gear nodes held, {len(bl)} body beams; target {abs(fd.OFFSET) * 100:.0f} cm inside the mesh\n")
    print(f"{'':22s} {'<=2cm':>5s} {'<=5cm':>5s} {'mean':>6s} {'p90':>6s} {'signed':>6s} | beam length after / before: p5   p50   p95    min   max  outside 0.7-1.4")
    line("base placed", skin_err({n: base_nodes[n] for n in ids}), stretch({n: base_nodes[n] for n in ids}))
    line("the fit (all stages)", skin_err(full), stretch(full))
    idx = {n: i for i, n in enumerate(ids)}
    if os.environ.get("MORPH_WORST"):
        f = rbf_map(ctrl, cd, 0.01)
        X1 = X0 + f(X0)
        X2 = relax(X1, np.maximum(L0, 1e-9), np.array([idx[b["a"]] for b in bl]), np.array([idx[b["b"]] for b in bl]), anchors, out_idx, P + disp, 0.75, 1.35)
        L2 = np.array([np.linalg.norm(X2[idx[b["a"]]] - X2[idx[b["b"]]]) for b in bl])
        r = L2 / np.maximum(L0, 1e-9)
        import collections
        bad = [(r[i], bl[i]["part"], bl[i]["a"], bl[i]["b"], L0[i]) for i in range(len(bl)) if L0[i] > 0.03 and (r[i] < 0.7 or r[i] > 1.4)]
        print(len(bad), "beams outside 0.7-1.4 after morph + relax; by part:", dict(collections.Counter(b[1] for b in bad).most_common(8)))
        for b in sorted(bad, key=lambda b: -abs(np.log(b[0])))[:10]:
            print(f"  {b[1]:26s} {b[2]:7s}-{b[3]:7s} length {b[4]:.3f} -> x{b[0]:.2f}")
    ia = np.array([idx[b["a"]] for b in bl])
    ib = np.array([idx[b["b"]] for b in bl])
    for s in [float(x) for x in a.smooth.split(",")]:
        f = rbf_map(ctrl, cd, s)
        X1 = X0 + f(X0)
        nodes = {n: X1[i].tolist() for i, n in enumerate(ids)}
        line(f"morph, smooth {s:g}", skin_err(nodes), stretch(nodes))
        for lo, hi in ((0.75, 1.35), (0.85, 1.2)):
            X2 = relax(X1, np.maximum(L0, 1e-9), ia, ib, anchors, out_idx, P + disp, lo, hi)
            n2 = {n: X2[i].tolist() for i, n in enumerate(ids)}
            line(f"  + relax {lo}-{hi}", skin_err(n2), stretch(n2))
            X3 = relax(X2, np.maximum(L0, 1e-9), ia, ib, anchors, out_idx, P + disp, 0.7, 1.4, iters=60, pull=0.0)      # then the bounds held hard
            n3 = {n: X3[i].tolist() for i, n in enumerate(ids)}
            line(f"  + hard 0.7-1.4", skin_err(n3), stretch(n3))
    return 0


if __name__ == "__main__":
    sys.exit(main())
