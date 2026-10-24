"""A testbed of the game's own tyres against an SVJ's Pacejka tyre, so a tyre is chosen by size and by what it does.

Pure Python, standard library only (the plots are made by tools/tyre_testbed.py). The library is every tyre part of the
held vehicles (donors.catalogue: size from the part's name, BeamNG's values from its pressureWheels). Each tyre and the
SVJ's tyre are put in the same terms (profile):

  size       width (mm), radius (m), rim (in)
  grip       the peak grip at the reference load: the SVJ's peak Fy / Fz and Fx / Fz (Magic Formula, tyres.pacejka_curves);
             BeamNG's frictionCoef times its load coefficient there (tyres.bng_mu)
  shape      the grip against load relative to the reference load (load sensitivity): both sides
  class      road / sport / race / off-road / winter, from the part's name (BeamNG) and from the grip level and the
             thermal curve's name (the SVJ)

BeamNG's grip numbers are multipliers, not friction coefficients: the game's grip also comes out of the tyre's nodes and
beams sliding on the ground (tyres.py). So the grip level is compared through GRIP_SCALE (BeamNG multiplier per unit of
real friction coefficient, 1.0 until measured) and reported next to the SVJ's, and the load sensitivity, which the model
does carry, counts as much. score() puts size and characteristics in one number, lower is closer.
"""

import math

from . import donors, tyres

WEIGHTS = {"width": 1.0, "radius": 1.5, "grip": 2.0, "shape": 1.0, "class": 0.3}
GRIP_SCALE = 1.0               # BeamNG multiplier per unit of real peak friction coefficient (a road tyre's about 1.0 in both)
CLASS_ORDER = ("winter", "mud", "offroad", "standard", "sport", "race")
RACE_MU = 1.35                 # peak mu from which an SVJ tyre is a race tyre; a sport tyre from SPORT_MU
SPORT_MU = 1.15


CLASS_OF = {"race": "race", "slick": "race", "track": "race", "drag": "race",
            "sport": "sport", "rally": "sport", "drift": "sport", "stretched": "sport",
            "offroad": "offroad", "desert": "offroad", "crawler": "offroad", "allterrain": "offroad", "gravel": "offroad",
            "mud": "mud", "winter": "winter", "snow": "winter"}       # (standard, asphalt, eco, old, dually, biasply: standard)


def bng_class(use):
    """A BeamNG tyre's class from its name's use word (donors.TYRE_USES)."""
    return CLASS_OF.get((use or "standard").lower(), "standard")


def library(models=None):
    """The game's tyres: [{"part", "model", "axle", "width_mm", "aspect", "rim_in", "radius", "use", "class", "props": {BeamNG's values}}]."""
    return _records(donors.catalogue(models)["tyres"])


def model_library(model):
    """The same for the tyre parts one vehicle can use (its own and the game's common ones), as held now."""
    from . import beamng
    return _records(donors.tyres(model, beamng._parts_held(model)))


def _records(cat):
    out, seen = [], set()
    for t in cat:
        if t["part"] in seen or not t.get("rim_in") or not t.get("radius"):
            continue
        seen.add(t["part"])
        props = {k: t[k] for k in donors.TYRE_KEYS if isinstance(t.get(k), (int, float))}
        width = t.get("width_mm") or (round(props["tireWidth"] * 1000) if props.get("tireWidth") else None)
        out.append({"part": t["part"], "model": t["model"], "axle": t.get("axle"), "width_mm": width, "aspect": t.get("aspect"),
                    "rim_in": t["rim_in"], "radius": t["radius"], "use": t.get("use"), "class": bng_class(t.get("use")), "props": props})
    return out


def _norm(table, fz0):
    ref = tyres._interp(table, fz0) or 1.0
    return [round(m / ref, 4) for _, m in table]


def bng_profile(rec, fz0):
    """A library tyre's characteristics at the reference load fz0 (N): {"grip", "shape" [at tyres.LOADS], "sliding" (slidingFrictionCoef
    over frictionCoef), "contact_nodes"}."""
    p = rec["props"]
    mu = tyres.bng_mu(p)
    return {"grip": round(tyres._interp(mu, fz0), 4), "shape": _norm(mu, fz0),
            "sliding": round(p.get("slidingFrictionCoef", 1.0) / (p.get("frictionCoef", 1.0) or 1.0), 3),
            "contact_nodes": round(tyres.contact_nodes(p), 1)}


def svj_profile(svj, corner=None):
    """The SVJ tyre's characteristics, or None without a Magic Formula set: {"name", "size", "rim_in", "width_mm", "radius", "fz0",
    "grip_y", "grip_x", "grip", "shape", "peak_deg" (slip angle of the peak lateral force at fz0), "stiffness" (cornering stiffness,
    per rad over load, at fz0), "class", "curves"}."""
    t = tyres.svj_tyre(svj, corner)
    if not t or not t.get("pacejka") or not t["pacejka"].get("lateral"):
        return None
    cur = tyres.pacejka_curves(t)
    fz0 = t["fz0"]
    gy, gx = tyres._interp(cur["mu_y"], fz0), tyres._interp(cur["mu_x"], fz0)
    both = [[f, (a + b) / 2] for (f, a), (_, b) in zip(cur["mu_y"], cur["mu_x"])]
    pj = t["pacejka"]["lateral"]
    angles = [i * 0.1 for i in range(1, 160)]
    fy = [abs(tyres.mf_fy(pj, fz0, math.radians(a), fz0)) for a in angles]
    peak = angles[fy.index(max(fy))]
    stiff = abs(tyres.mf_fy(pj, fz0, math.radians(0.5), fz0)) / math.radians(0.5) / fz0
    thermal = str(((svj.get("tires") or {}).get("sets") or {}).get(t["name"], {}).get("thermal", {}).get("performance_curve") or "").lower()
    grip = (gy + gx) / 2
    cls = "race" if ("slick" in thermal or grip >= RACE_MU) else "sport" if grip >= SPORT_MU else "standard"
    return {"name": t["name"], "size": t["size"], "rim_in": t["rim_in"], "width_mm": round(t["width"] * 1000) if t["width"] else None,
            "radius": t["radius"], "fz0": fz0, "grip_y": round(gy, 3), "grip_x": round(gx, 3), "grip": round(grip, 3), "shape": _norm(both, fz0),
            "peak_deg": round(peak, 1), "stiffness": round(stiff, 2), "class": cls, "curves": cur}


def score(svj_p, rec, bng_p, weights=None, grip_scale=None):
    """How far a library tyre is from the SVJ's: ({"total", "width", "radius", "grip", "shape", "class"}), lower is closer. Each part is a
    relative gap (the grip's against the SVJ's peak mu times GRIP_SCALE, the shape's an RMS of the load-relative curves, the class an
    ordering distance over the classes)."""
    w = {**WEIGHTS, **(weights or {})}
    k = grip_scale if grip_scale is not None else GRIP_SCALE
    parts = {}
    parts["width"] = abs((rec["width_mm"] or svj_p["width_mm"]) - svj_p["width_mm"]) / svj_p["width_mm"] if svj_p.get("width_mm") else 0.0
    parts["radius"] = abs(rec["radius"] - svj_p["radius"]) / svj_p["radius"] if svj_p.get("radius") else 0.0
    want = svj_p["grip"] * k
    parts["grip"] = abs(bng_p["grip"] - want) / want
    n = min(len(svj_p["shape"]), len(bng_p["shape"]))
    parts["shape"] = math.sqrt(sum((a - b) ** 2 for a, b in zip(svj_p["shape"][:n], bng_p["shape"][:n])) / n)
    order = {c: i for i, c in enumerate(CLASS_ORDER)}
    parts["class"] = abs(order[rec["class"]] - order[svj_p["class"]]) / (len(CLASS_ORDER) - 1)
    parts = {k_: round(v, 4) for k_, v in parts.items()}
    parts["total"] = round(sum(w[k_] * parts[k_] for k_ in w), 4)
    return parts


def rank(svj_p, lib, rim_in=None, axle=None, top=10, weights=None, grip_scale=None):
    """The library's tyres nearest the SVJ's, same rim only (rim_in: default the SVJ's): [{"rec", "profile", "score"}] best first.
    axle: "front" / "rear" keeps that axle's parts and the ones with none."""
    rim = rim_in or svj_p.get("rim_in")
    out = []
    for rec in lib:
        if rim and rec["rim_in"] != rim:
            continue
        if axle and rec.get("axle") not in (axle, None):
            continue
        prof = bng_profile(rec, svj_p["fz0"])
        out.append({"rec": rec, "profile": prof, "score": score(svj_p, rec, prof, weights, grip_scale)})
    out.sort(key=lambda r: (r["score"]["total"], len(r["rec"]["part"])))
    kept, seen = [], set()
    for r in out:                                         # (a front and a rear part that are the same tyre count once)
        key = (r["rec"]["width_mm"], r["rec"]["radius"], r["rec"]["class"], r["profile"]["grip"], tuple(r["profile"]["shape"]))
        if key not in seen:
            seen.add(key)
            kept.append(r)
    return kept[:top]


def summary(lib, fz0=4000.0):
    """The library by class: {class: {"n", "grip" (min, median, max), "radius" (min, max), "rims"}} at the reference load."""
    out = {}
    for rec in lib:
        prof = bng_profile(rec, fz0)
        out.setdefault(rec["class"], []).append((prof["grip"], rec["radius"], rec["rim_in"]))
    res = {}
    for c, rows in out.items():
        g = sorted(r[0] for r in rows)
        res[c] = {"n": len(rows), "grip": (g[0], g[len(g) // 2], g[-1]), "radius": (min(r[1] for r in rows), max(r[1] for r in rows)),
                  "rims": sorted({r[2] for r in rows})}
    return res
