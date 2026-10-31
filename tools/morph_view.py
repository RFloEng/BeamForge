"""Pictures of a fitted vehicle's parts one at a time, with the SVJ mesh around them: to see where the body fit puts each part.

    python tools/morph_view.py <car.svj.json> --base covet [--config rally_gravel] [--stages wheelbase,morph] --parts "door|hood" --out views/

For each part whose name matches --parts, one PNG with three views (side, front, top): the SVJ body mesh around the part as grey points
(the mesh piece the morph gave the part, when it has one, in green), the part's nodes and beams as the base has them placed (blue, thin) and
after the fit (red). A part that fits shows red lying on the green/grey surface, a little inside it.
"""

import argparse
import json
import os
import sys

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fit_distance as fd  # noqa: E402
from beamforge import beamng, fit, morph, pipeline, unified  # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("svj")
    ap.add_argument("--base", required=True)
    ap.add_argument("--config")
    ap.add_argument("--vehicles", default=os.environ.get("BEAMNG_VEHICLES"))
    ap.add_argument("--stages", default="wheelbase,morph,pickups")
    ap.add_argument("--parts", default="door|hood|tailgate|trunk|fender|bumper")
    ap.add_argument("--out", default="views")
    a = ap.parse_args(argv)
    import re
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    svj = json.load(open(a.svj, encoding="utf-8"))
    unified.adopt(svj)
    beamng.reset()
    fd.load_zip(os.path.join(a.vehicles, "common.zip"))
    fd.load_zip(os.path.join(a.vehicles, a.base + ".zip"))
    v = json.loads(beamng.configure(a.base, a.config))
    files = pipeline.svj_mesh_files(svj, os.path.dirname(os.path.abspath(a.svj)))
    geo = v["geometry"]
    r = json.loads(fit.fit_json(json.dumps(geo), json.dumps(v["wheels"]), json.dumps(svj), json.dumps(files), json.dumps(a.stages.split(","))))
    base = geo.get("rest") or geo["nodes"]
    after = {n: [p + d for p, d in zip(base[n], r["moves"].get(n, [0, 0, 0]))] for n in base}
    place = r.get("place")
    tri = fd.body_triangles(svj, next(iter(files.values())), place)
    pts, _ = fd.surface_samples(tri, 0.04)
    pieces = {}
    fit.mesh_surface(svj, files, place["yf"], place["ground"], pieces=pieces)
    tree = cKDTree(pts)
    parts = geo["parts"]
    beams = [(b[0], b[1], p) for b, p in zip(geo.get("beams") or [], geo.get("beam_parts") or [])]
    os.makedirs(a.out, exist_ok=True)
    rx = re.compile(a.parts, re.I)
    names = sorted({p for p in parts.values() if p and rx.search(p)})
    views = (("side (y up the page: front left)", 1, 2), ("front", 0, 2), ("top", 0, 1))
    for name in names:
        ids = [n for n, p in parts.items() if p == name and n in after]
        if len(ids) < 4:
            continue
        A = np.array([after[n] for n in ids])
        B = np.array([base[n] for n in ids])
        lo, hi = np.minimum(A.min(0), B.min(0)) - 0.25, np.maximum(A.max(0), B.max(0)) + 0.25
        near = pts[((pts >= lo) & (pts <= hi)).all(axis=1)]
        role = morph.role_of(name)
        piece = morph.part_pieces(A, role, pieces) if role else []
        pp = np.concatenate(piece).reshape(-1, 3) if piece else np.zeros((0, 3))
        pp = pp[((pp >= lo) & (pp <= hi)).all(axis=1)] if len(pp) else pp
        fig, axs = plt.subplots(1, 3, figsize=(18, 6))
        e, _ = tree.query(A)
        mine = set(ids)
        for ax, (title, i, j) in zip(axs, views):
            ax.scatter(near[:, i], near[:, j], s=1, c="#bbbbbb")
            if len(pp):
                ax.scatter(pp[:, i], pp[:, j], s=2, c="#2a9d3f")
            for x, y, p in beams:
                if x in mine and y in mine:
                    ax.plot([base[x][i], base[y][i]], [base[x][j], base[y][j]], c="#9db8e8", lw=0.4)
                    ax.plot([after[x][i], after[y][i]], [after[x][j], after[y][j]], c="#d62728", lw=0.7)
            ax.scatter(A[:, i], A[:, j], s=8, c="#d62728")
            ax.set_title(title)
            ax.set_aspect("equal")
            ax.grid(alpha=0.3)
        fig.suptitle(f"{name}: {len(ids)} nodes; mesh piece: {'yes (green)' if len(pp) else 'none (whole mesh)'}; nearest mesh point mean {e.mean() * 100:.1f} cm; "
                     "grey = SVJ body mesh, blue = base placed, red = after the fit")
        path = os.path.join(a.out, name + ".png")
        fig.savefig(path, dpi=70, bbox_inches="tight")
        plt.close(fig)
        print(path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
