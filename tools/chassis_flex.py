"""How stiff a vanilla car's body is in torsion, against the figures published for real cars.

    python tools/chassis_flex.py [--vehicles <BeamNG>/content/vehicles] [--models covet,pessima,etk800] [--parts shell|all|<regex>]
        [--load 1000] [--reference tools/flex_reference.json] [--csv out.csv]

The test is the one real bodies get (torsion bench): the body is supported at its rear suspension mounts, opposite vertical forces act at
the front mounts, and the stiffness is the torque over the twist of the front axle line, in N m per degree (the unit the published figures
use). Here the beams of the body are a network of linear springs (beamSpring, N/m, along each beam; the game's beams are springs in a pin-jointed
frame, so this is its stiffness for small loads): the nodes the suspension parts hang on are the mounts. One-way support beams, dampers and
springs, tyres and the running gear are left out; --parts chooses what counts as the body:

  shell   the body's own parts (names ending _body, _frame, _subframe, _header, ...): what a body-in-white test measures
  all     every part but the running gear, the powertrain, the exhaust, the occupants and the fuel: closures, panels and interior as well
  <regex> parts whose names match

BeamNG's body is a stability-limited model (beams are no stiffer than their nodes' weight allows at the 2000 Hz step), so it is expected to
sit well under a real body's stiffness: the point of the test is by how much, and which cars come closest.
"""

import argparse
import csv
import glob
import json
import math
import os
import re
import sys

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.linalg import spsolve

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from beamforge import beamng, rigidity  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
TEXT = (".jbeam", ".pc", ".json")
RUNNING_GEAR = re.compile(r"suspension|coilover|shock|damper|spring|hub|knuckle|upright|wheeldata|brake|swaybar|sway_bar|axle|wheel|tire|tyre|"
                          r"steering|rack|halfshaft|driveshaft", re.I)
NOT_STRUCTURE = re.compile(r"engine|transmission|transaxle|transfer|differential|intake|exhaust|radiator|oilpan|fueltank|fuel_tank|occupant|"
                           r"shifter|pedal|cargo|turbo|battery|license|antenna|mirror|light|lamp", re.I)
SHELL = re.compile(r"(_body(_|$)|_frame(_|$)|subframe|_header|_firewall|_floor|_roof|_pillar|chassis|unibody|_cage)", re.I)
LINEAR = ("|NORMAL", "|LBEAM", "|ANISOTROPIC")                  # (compared in upper case)
TOP = 0.12                    # m: the "top" mounts of a side are those within this of its highest
REACH = 0.45                  # m: a mount counts as the axle's when it is this close to the axle line along the car


def load_zip(path):
    import zipfile
    with zipfile.ZipFile(path) as z:
        beamng.add_files(json.dumps({n: z.read(n).decode("utf-8-sig", "replace") for n in z.namelist() if n.lower().endswith(TEXT)}))


def solve_torsion(pos, springs, rear, front_left, front_right, load=1000.0, tie=1e3):
    """The torsion bench on a network of linear springs. pos: {node: [x, y, z]}; springs: [(a, b, k N/m)]; rear: nodes held in all three
    directions; front_left / front_right: the mount nodes each side, tied by stiff springs to one fixture node per side (the bench's rigid
    bracket, so the result does not depend on how weakly one mount node is held) where `load` N acts, up on the left and down on the right.
    Returns {"K_rad" (N m/rad), "K" (N m/deg), "width", "twist", "residual" (the equilibrium error, share of the load), "components"} or {"error"}."""
    from scipy.sparse.csgraph import connected_components
    names = sorted({n for s in springs for n in s[:2]} | set(rear) | set(front_left) | set(front_right))
    idx = {n: i for i, n in enumerate(names)}
    FL, FR = len(names), len(names) + 1                      # the fixture nodes
    nn = len(names) + 2
    P = np.array([pos[n] for n in names] + [np.mean([pos[n] for n in front_left], axis=0), np.mean([pos[n] for n in front_right], axis=0)])
    kmax = max((k for *_, k in springs), default=1.0)
    allsp = [(idx[a], idx[b], k) for a, b, k in springs] + [(FL, idx[n], tie * kmax) for n in front_left] + [(FR, idx[n], tie * kmax) for n in front_right]
    rows, cols, vals = [], [], []
    for a, b, k in allsp:
        d = P[b] - P[a]
        L = float(np.linalg.norm(d))
        if L < 1e-6:
            u = np.eye(3)                                    # a fixture tie on a mount at the fixture's own place: an isotropic tie
            blk = k * u
        else:
            blk = k * np.outer(d / L, d / L)
        if k == tie * kmax:                                  # the fixture's ties hold all three directions (a ball joint, not a bar)
            blk = k * np.eye(3)
        for n1, s1 in ((a, 1.0), (b, -1.0)):
            for n2, s2 in ((a, 1.0), (b, -1.0)):
                for p in range(3):
                    for q in range(3):
                        rows.append(3 * n1 + p)
                        cols.append(3 * n2 + q)
                        vals.append(s1 * s2 * blk[p, q])
    N = 3 * nn
    K = coo_matrix((vals, (rows, cols)), shape=(N, N)).tocsr()
    nc, _ = connected_components(K != 0, directed=False)
    fixed = np.array(sorted({3 * idx[n] + p for n in rear for p in range(3)}))
    free = np.setdiff1d(np.arange(N), fixed)
    f = np.zeros(N)
    f[3 * FL + 2], f[3 * FR + 2] = load, -load
    reg = coo_matrix((np.full(len(free), 1e-9 * kmax), (np.arange(len(free)), np.arange(len(free)))), shape=(len(free),) * 2)
    u = np.zeros(N)
    u[free] = spsolve((K[free][:, free] + reg).tocsc(), f[free])
    react = K @ u - f                                         # at the free dofs: ~0; at the fixed ones: the supports' reactions
    resid = float(np.linalg.norm(react[free]) / (np.linalg.norm(f) or 1.0))
    width = abs(P[FL][0] - P[FR][0])
    twist = (u[3 * FL + 2] - u[3 * FR + 2]) / width if width > 1e-6 else 0.0
    if abs(twist) < 1e-15:
        return {"error": "no twist: the mounts are not connected through the body network"}
    k_rad = load * width / abs(twist)
    return {"K_rad": k_rad, "K": k_rad * math.pi / 180, "width": width, "twist": twist, "residual": resid, "components": int(nc)}


def structure_test(model, parts="shell", load=1000.0, mounts="top"):
    """The torsion test of a loaded vehicle (beamng.add_files done): {"nodes", "beams", "K" (N m/deg), "K_rad", "width" (m), "residual",
    "mounts": {"front_left", ...} (node counts), "components"}, or {"error": text}. The mounts are always the body shell's nodes that the running
    gear hangs on; `parts` chooses which parts carry the load between them."""
    cv = json.loads(beamng.configure(model))
    geo = cv["geometry"]
    pos, owner = geo["nodes"], geo.get("parts") or {}
    meas = cv.get("measure") or {}
    yf, yr = meas.get("front_axle_y"), meas.get("rear_axle_y")
    if yf is None or yr is None:
        return {"error": "no front and rear wheels"}
    rx = re.compile(parts, re.I) if parts not in ("shell", "all") else None

    def structure(p):
        if RUNNING_GEAR.search(p) or NOT_STRUCTURE.search(p):
            return False
        return bool(rx.search(p)) if rx else (parts == "all" or bool(SHELL.search(p)))
    bl = rigidity.beams(model, cv)
    struct = {n for n, p in owner.items() if structure(p)}
    shell = {n for n, p in owner.items() if not RUNNING_GEAR.search(p) and not NOT_STRUCTURE.search(p) and SHELL.search(p)} or struct
    gear = {n for n, p in owner.items() if RUNNING_GEAR.search(p)}
    springs, mounts = [], set()
    for b in bl:
        a, c, k = b["a"], b["b"], b["values"].get("beamSpring", 0.0)
        if k <= 0:
            continue
        if b["type"].upper() in LINEAR and a in struct and c in struct:        # (one-way supports, dampers, springs, tyres: not the body's stiffness)
            springs.append((a, c, k))
        if (a in shell and c in gear) or (c in shell and a in gear):
            mounts.add(a if a in shell else c)
    groups = {"front_left": [], "front_right": [], "rear_left": [], "rear_right": []}
    for n in mounts:
        x, y, _ = pos[n]
        for axle, yy in (("front", yf), ("rear", yr)):
            if abs(y - yy) <= REACH:
                groups[f"{axle}_{'left' if x > 0 else 'right'}"].append(n)
    if any(not g for g in groups.values()):
        return {"error": f"mounts not found: {', '.join(k for k, g in groups.items() if not g)} ({len(mounts)} mount nodes)"}
    inside = {n for s in springs for n in s[:2]}
    for g in groups.values():
        g[:] = [n for n in g if n in inside]
        if mounts == "top" and g:                                        # a real bench loads and supports at the strut towers / spring seats
            zmax = max(pos[n][2] for n in g)
            g[:] = [n for n in g if pos[n][2] >= zmax - TOP]
    if any(not g for g in groups.values()):
        return {"error": "the mount nodes are not in the body network"}
    r = solve_torsion(pos, springs, groups["rear_left"] + groups["rear_right"], groups["front_left"], groups["front_right"], load)
    if "error" in r:
        return r
    r.update(nodes=len(inside), beams=len(springs), mounts={k: len(v) for k, v in groups.items()})
    return r


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--vehicles", default=os.environ.get("BEAMNG_VEHICLES"), help="the game's content/vehicles folder")
    ap.add_argument("--models", help="comma separated vehicles (default every car the tool can measure)")
    ap.add_argument("--parts", default="shell", help="shell (default), all, or a regex of the part names that are the body")
    ap.add_argument("--mounts", choices=("top", "all"), default="top", help="load and support at the highest mounts of each side (strut towers, spring seats: default) or at every mount")
    ap.add_argument("--load", type=float, default=1000.0, help="N on each side at the front mounts (the test is linear)")
    ap.add_argument("--reference", default=os.path.join(HERE, "flex_reference.json"), help="the real cars' figures (JSON)")
    ap.add_argument("--csv")
    a = ap.parse_args(argv)
    if not a.vehicles or not os.path.isdir(a.vehicles):
        ap.error("--vehicles (or BEAMNG_VEHICLES) must be the game's content/vehicles folder")
    ref = json.load(open(a.reference, encoding="utf-8")) if os.path.exists(a.reference) else {"cars": []}
    names = a.models.split(",") if a.models else sorted(os.path.basename(z)[:-4] for z in glob.glob(os.path.join(a.vehicles, "*.zip"))
                                                      if os.path.basename(z)[:-4] != "common")
    rows = []
    for m in names:
        try:
            beamng.reset()
            load_zip(os.path.join(a.vehicles, "common.zip"))
            load_zip(os.path.join(a.vehicles, m + ".zip"))
            r = structure_test(m, a.parts, a.load, a.mounts)
        except Exception as e:                                  # (a vehicle that is no car, a part that does not parse)
            r = {"error": repr(e)[:80]}
        if "error" in r:
            if a.models:
                print(f"{m}: {r['error']}")
            continue
        if r["residual"] > 1e-3:                                 # the network does not hold the load: a mechanism on the way, no stiffness to read
            print(f"{m}: no valid result (the body network is a mechanism between the mounts: equilibrium error {r['residual']:.1g})")
            continue
        rows.append((m, r))
    rows.sort(key=lambda x: -x[1]["K"])
    print(f"torsional stiffness of the body ({a.parts}), rear mounts held, couple at the front mounts, N m per degree:")
    for m, r in rows:
        warn = "  [DISCONNECTED: %d separate pieces]" % r["components"] if r["components"] > 2 else ""
        print(f"  {m:14s} {r['K'] / 1000:7.2f} kN m/deg   ({r['nodes']} nodes, {r['beams']} beams, mounts {r['mounts']['front_left']}/{r['mounts']['rear_left']} per side, "
              f"front mounts {r['width']:.2f} m apart, equilibrium error {r['residual']:.0e}){warn}")
    if ref.get("cars"):
        print("\nreal cars, published (see tools/flex_reference.json for sources and what was measured):")
        for c in sorted(ref["cars"], key=lambda c: c["nm_per_deg"]):
            print(f"  {c['car']:42s} {c['nm_per_deg'] / 1000:7.2f} kN m/deg   {c.get('note', '')}")
        mid = ref.get("compact_road_car_nm_per_deg")
        if mid and rows:
            print(f"\nagainst a compact road car's {mid / 1000:.0f} kN m/deg: " + ", ".join(f"{m} {100 * r['K'] / mid:.0f} %" for m, r in rows[:12]))
    if a.csv:
        with open(a.csv, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["model", "parts", "kNm_per_deg", "nodes", "beams", "front_mount_width_m"])
            for m, r in rows:
                w.writerow([m, a.parts, round(r["K"] / 1000, 3), r["nodes"], r["beams"], round(r["width"], 3)])
        print("written to", a.csv)
    return 0


if __name__ == "__main__":
    sys.exit(main())
