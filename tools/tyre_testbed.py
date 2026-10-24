"""The game's tyres against an SVJ's Pacejka tyre: which vanilla tyre is nearest by size and by what it does.

    python tools/tyre_testbed.py [--vehicles <BeamNG>/content/vehicles] [--models sunburst2,vivace]
        [--svj car.svj.json [--corner FL] [--rim 17] [--top 10] [--png out.png]] [--csv library.csv] [--grip-scale 1.0]

Without --svj: the library by class (how many tyres, their grip range at 4000 N, their radii and rims), and --csv writes
every tyre with its characteristics. With --svj: the SVJ tyre's numbers (peak mu, load sensitivity, cornering stiffness,
the slip angle of its peak), then the nearest vanilla tyres of the same rim with the parts of their score, and --png the
grip against load of the SVJ's Pacejka tyre and the best three. BeamNG's grip numbers are multipliers (see tyres.py), so
the grip level is compared through --grip-scale; the load sensitivity and the size are what the model can be trusted on.
"""

import argparse
import csv
import glob
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from beamforge import beamng, tyrelab, tyres  # noqa: E402

TEXT = (".jbeam", ".pc", ".json")


def load_zip(path):
    import zipfile
    with zipfile.ZipFile(path) as z:
        beamng.add_files(json.dumps({n: z.read(n).decode("utf-8-sig", "replace") for n in z.namelist() if n.lower().endswith(TEXT)}))


def plot(path, svj_p, best):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    cur = svj_p["curves"]
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.4))
    for fz, col in zip(("2000", "4000", "6000"), ("#4c78a8", "#f58518", "#54a24b")):
        ax[0].plot(*zip(*cur["fy"][fz]), color=col, label=f"{fz} N")
    ax[0].set(title=f"SVJ lateral force ({svj_p['size']}, MF)", xlabel="slip angle (deg)", ylabel="Fy (N)")
    ax[0].legend()
    loads = [f for f, _ in cur["mu_y"]]
    ax[1].plot(loads, [m for _, m in cur["mu_y"]], "k-", lw=2.4, label="SVJ peak Fy/Fz")
    ax[1].plot(loads, [m for _, m in cur["mu_x"]], "k--", lw=1.6, label="SVJ peak Fx/Fz")
    for r in best[:3]:
        mu = tyres.bng_mu(r["rec"]["props"], tuple(loads))
        ax[1].plot(loads, [m * 1.0 for _, m in mu], label=f"{r['rec']['part'][:30]} ({r['score']['total']})")
    ax[1].set(title="peak grip against load (BeamNG: multiplier)", xlabel="load (N)", ylabel="mu")
    ax[1].legend(fontsize=7)
    ax[2].plot(loads, svj_p["shape"], "k-", lw=2.4, label="SVJ")
    for r in best[:3]:
        ax[2].plot(loads, r["profile"]["shape"], label=r["rec"]["part"][:30])
    ax[2].set(title="load sensitivity (relative to the reference load)", xlabel="load (N)", ylabel="grip / grip at reference")
    ax[2].legend(fontsize=7)
    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--vehicles", default=os.environ.get("BEAMNG_VEHICLES"), help="the game's content/vehicles folder")
    ap.add_argument("--models", help="only these vehicles' tyres (comma separated); default all of them (and common)")
    ap.add_argument("--svj", help="an .svj.json to rank the tyres against")
    ap.add_argument("--corner", help="the SVJ corner whose tyre set to use (default the first set)")
    ap.add_argument("--rim", type=int, help="rim diameter in inches (default the SVJ's)")
    ap.add_argument("--axle", choices=("front", "rear"))
    ap.add_argument("--top", type=int, default=10)
    ap.add_argument("--grip-scale", type=float, default=tyrelab.GRIP_SCALE, help="BeamNG multiplier per unit of real peak mu")
    ap.add_argument("--csv", help="write the whole library with its characteristics")
    ap.add_argument("--png", help="plot the SVJ tyre against the best three")
    a = ap.parse_args(argv)
    if not a.vehicles or not os.path.isdir(a.vehicles):
        ap.error("--vehicles (or BEAMNG_VEHICLES) must be the game's content/vehicles folder")
    beamng.reset()
    load_zip(os.path.join(a.vehicles, "common.zip"))
    wanted = set(a.models.split(",")) if a.models else None
    for z in sorted(glob.glob(os.path.join(a.vehicles, "*.zip"))):
        name = os.path.basename(z)[:-4]
        if name != "common" and (wanted is None or name in wanted):
            try:
                load_zip(z)
            except Exception:
                pass
    lib = tyrelab.library([m for m in wanted] if wanted else None)
    print(f"{len(lib)} tyres in the library")
    for c, s in sorted(tyrelab.summary(lib).items()):
        print(f"  {c:9s} {s['n']:4d}  grip {s['grip'][0]:.2f} / {s['grip'][1]:.2f} / {s['grip'][2]:.2f} (min / median / max)  "
              f"radius {s['radius'][0]:.3f}-{s['radius'][1]:.3f} m  rims {s['rims'][0]}-{s['rims'][-1]} in")
    fz0 = 4000.0
    if a.csv:
        with open(a.csv, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["part", "model", "axle", "class", "use", "width_mm", "aspect", "rim_in", "radius", "grip@4000N", "sliding_ratio",
                        "shape_2000", "shape_6000", "frictionCoef", "noLoadCoef", "fullLoadCoef", "loadSensitivitySlope", "softnessCoef", "treadCoef"])
            for r in lib:
                p = tyrelab.bng_profile(r, fz0)
                pr = r["props"]
                w.writerow([r["part"], r["model"], r["axle"], r["class"], r["use"], r["width_mm"], r["aspect"], r["rim_in"], r["radius"], p["grip"],
                            p["sliding"], p["shape"][1], p["shape"][5], pr.get("frictionCoef"), pr.get("noLoadCoef"), pr.get("fullLoadCoef"),
                            pr.get("loadSensitivitySlope"), pr.get("softnessCoef"), pr.get("treadCoef")])
        print("library written to", a.csv)
    if not a.svj:
        return 0
    svj = json.load(open(a.svj, encoding="utf-8"))
    sp = tyrelab.svj_profile(svj, a.corner)
    if not sp:
        print("the SVJ has no Magic Formula tyre set: nothing to compare against")
        return 1
    print(f"\nSVJ tyre {sp['name']} ({sp['size']}): width {sp['width_mm']} mm, radius {sp['radius']:.3f} m, rim {sp['rim_in']} in, class {sp['class']}")
    print(f"  at {sp['fz0']:.0f} N: peak mu lateral {sp['grip_y']}, longitudinal {sp['grip_x']}; cornering stiffness {sp['stiffness']} /rad; "
          f"peak at {sp['peak_deg']} deg; load sensitivity (2000 / 6000 N relative): {sp['shape'][1]} / {sp['shape'][5]}")
    best = tyrelab.rank(sp, lib, a.rim, a.axle, a.top, grip_scale=a.grip_scale)
    if not best:
        print("no tyre of that rim in the library")
        return 1
    print(f"\nnearest vanilla tyres, rim {a.rim or sp['rim_in']} in (score: lower is closer; width, radius, grip, shape, class are its parts):")
    for r in best:
        rec, s, p = r["rec"], r["score"], r["profile"]
        print(f"  {s['total']:6.3f}  {rec['part'][:40]:40s} {rec['width_mm']}mm r{rec['radius']:.3f} {rec['class']:8s} grip {p['grip']:.2f}  "
              f"[w {s['width']:.2f} r {s['radius']:.2f} g {s['grip']:.2f} s {s['shape']:.2f} c {s['class']:.2f}]")
    if a.png:
        plot(a.png, sp, best)
        print("plot written to", a.png)
    return 0


if __name__ == "__main__":
    sys.exit(main())
