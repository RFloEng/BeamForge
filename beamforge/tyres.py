"""Tyres: the SVJ's Pacejka tyre against a BeamNG tyre, and a BeamNG tyre fitted to it.

Pure Python, standard library only (runs in Pyodide).

The SVJ side (§11): a tyre set's Magic Formula coefficients (MF 5.2 / 6.2, pure slip, at the reference pressure, no
camber; the scaling factors lambda at 1): the lateral force against slip angle and the longitudinal force against slip
ratio at several loads, and the peak friction coefficient (peak force over load) against load.

The BeamNG side (documentation.beamng.com, Pressure Wheels): a tyre's grip is frictionCoef (times the ground's), times a
load coefficient that starts at noLoadCoef, falls by loadSensitivitySlope per newton of normal force and stops at
fullLoadCoef; the load is each tread node's, not the wheel's. So a wheel's load is shared by the tread nodes in contact
(an estimate here: CONTACT_LENGTH of tread over the circumference, both sides of numRays rays). In the game the grip
also comes out of the tyre's nodes and beams sliding on the ground: these numbers are multipliers, not a friction
coefficient one can read off. So the benchmark compares the shape (grip against load, relative to the reference load)
and the fit matches BeamNG's load sensitivity to the Pacejka tyre's, keeping the BeamNG tyre's grip at the reference
load (the level the game's tyres are tuned at).
"""

import copy
import json
import math

CONTACT_LENGTH = 0.15         # m: the contact patch's length, for the tread nodes a wheel's load is shared by
LOADS = (1000.0, 2000.0, 3000.0, 4000.0, 5000.0, 6000.0, 7000.0, 8000.0)
OVERRIDABLE = ("radius", "tireWidth", "frictionCoef", "slidingFrictionCoef", "noLoadCoef", "fullLoadCoef", "loadSensitivitySlope",
               "softnessCoef", "treadCoef", "pressurePSI")


def _g(d, k, default=0.0):
    v = d.get(k)
    return float(v) if isinstance(v, (int, float)) else default


def mf_fy(lat, fz, alpha, fz0):
    """Pure lateral force (N) of an MF 5.2 / 6.2 set at load fz (N), slip angle alpha (rad), no camber."""
    dfz = (fz - fz0) / fz0
    shy = _g(lat, "pHy1") + _g(lat, "pHy2") * dfz
    ay = alpha + shy
    cy = _g(lat, "pCy1", 1.3)
    dy = (_g(lat, "pDy1", 1.0) + _g(lat, "pDy2") * dfz) * fz
    ey = min(1.0, (_g(lat, "pEy1") + _g(lat, "pEy2") * dfz) * (1 - _g(lat, "pEy3") * (1 if ay >= 0 else -1)))
    k4 = _g(lat, "pKy4", 2.0) or 2.0
    kya = _g(lat, "pKy1", -15.0) * fz0 * math.sin(k4 * math.atan(fz / ((_g(lat, "pKy2", 1.5) or 1.5) * fz0)))
    by = kya / (cy * dy) if cy * dy else 0.0
    svy = fz * (_g(lat, "pVy1") + _g(lat, "pVy2") * dfz)
    x = by * ay
    return dy * math.sin(cy * math.atan(x - ey * (x - math.atan(x)))) + svy


def mf_fx(lon, fz, kappa, fz0):
    """Pure longitudinal force (N) of an MF 5.2 / 6.2 set at load fz (N), slip ratio kappa, no camber."""
    dfz = (fz - fz0) / fz0
    kx = kappa + _g(lon, "pHx1") + _g(lon, "pHx2") * dfz
    cx = _g(lon, "pCx1", 1.65)
    dx = (_g(lon, "pDx1", 1.0) + _g(lon, "pDx2") * dfz) * fz
    ex = min(1.0, (_g(lon, "pEx1") + _g(lon, "pEx2") * dfz + _g(lon, "pEx3") * dfz * dfz) * (1 - _g(lon, "pEx4") * (1 if kx >= 0 else -1)))
    kxk = fz * (_g(lon, "pKx1", 20.0) + _g(lon, "pKx2") * dfz) * math.exp(_g(lon, "pKx3") * dfz)
    bx = kxk / (cx * dx) if cx * dx else 0.0
    svx = fz * (_g(lon, "pVx1") + _g(lon, "pVx2") * dfz)
    x = bx * kx
    return dx * math.sin(cx * math.atan(x - ex * (x - math.atan(x)))) + svx


def svj_tyre(svj, corner=None):
    """The SVJ's tyre set for a corner (its tire.set_ref / tire_ref, else the first set): {"name", "size", "rim_in",
    "width", "aspect", "radius", "fz0", "pacejka": {"model", "lateral", "longitudinal"} or None}, or None."""
    sets = (svj.get("tires") or {}).get("sets") or {}
    if not sets:
        return None
    c = ((svj.get("suspension") or {}).get(corner) or {}) if corner else {}
    t = c.get("tire") if isinstance(c.get("tire"), dict) else {}
    ref = t.get("set_ref") or t.get("tire_ref") or t.get("ref")
    name = ref if ref in sets else next(iter(sets))
    s = sets[name]
    dims, cons, refd = s.get("dimensions") or {}, s.get("construction") or {}, s.get("reference") or {}
    pj = s.get("pacejka") if isinstance(s.get("pacejka"), dict) else None
    od = dims.get("overall_diameter")
    return {"name": name, "size": s.get("size_code"), "rim_in": dims.get("rim_diameter_code"), "width": dims.get("section_width"),
            "aspect": dims.get("aspect_ratio"), "radius": (od / 2) if isinstance(od, (int, float)) else cons.get("rolling_radius"),
            "fz0": float(refd.get("load") or (pj or {}).get("FNOMIN") or 4000.0),
            "pacejka": {"model": pj.get("model"), "lateral": pj.get("lateral") or {}, "longitudinal": pj.get("longitudinal") or {}} if pj else None}


def pacejka_curves(tyre, loads=(2000.0, 4000.0, 6000.0)):
    """{"fy": {load: [[deg, N]]}, "fx": {load: [[slip %, N]]}, "mu_y": [[load, peak Fy / Fz]], "mu_x": [[load, mu]]}."""
    pj = tyre["pacejka"]
    fz0 = tyre["fz0"]
    out = {"fy": {}, "fx": {}, "mu_y": [], "mu_x": []}
    alphas = [i * 0.25 for i in range(0, 61)]                # 0-15 deg
    kappas = [i * 0.5 for i in range(0, 61)]                 # 0-30 %
    for fz in loads:
        out["fy"][str(int(fz))] = [[a, round(abs(mf_fy(pj["lateral"], fz, math.radians(a), fz0)), 1)] for a in alphas]
        out["fx"][str(int(fz))] = [[k, round(abs(mf_fx(pj["longitudinal"], fz, k / 100, fz0)), 1)] for k in kappas]
    for fz in LOADS:
        out["mu_y"].append([fz, round(max(abs(mf_fy(pj["lateral"], fz, math.radians(a), fz0)) for a in alphas) / fz, 4)])
        out["mu_x"].append([fz, round(max(abs(mf_fx(pj["longitudinal"], fz, k / 100, fz0)) for k in kappas) / fz, 4)])
    return out


def contact_nodes(props):
    """The tread nodes a wheel's load is shared by (an estimate): both sides of numRays rays over the contact length."""
    rays = props.get("numRays") or 16
    r = props.get("radius") or 0.3
    return max(2.0, 2 * rays * CONTACT_LENGTH / (2 * math.pi * r))


def bng_coef(props, fz, nc=None):
    """BeamNG's load coefficient at a wheel load: noLoadCoef less loadSensitivitySlope per newton on each tread node in
    contact, down to fullLoadCoef."""
    nc = nc or contact_nodes(props)
    no, full, s = _g(props, "noLoadCoef", 1.0), _g(props, "fullLoadCoef", 0.0), _g(props, "loadSensitivitySlope", 0.0)
    return max(full, no - s * fz / nc)


def bng_mu(props, loads=LOADS):
    """[[load, frictionCoef x the load coefficient]]: the tyre's grip multiplier against wheel load."""
    nc = contact_nodes(props)
    f = _g(props, "frictionCoef", 1.0)
    return [[fz, round(f * bng_coef(props, fz, nc), 4)] for fz in loads]


def fit(props, mu, fz0):
    """noLoadCoef, fullLoadCoef and loadSensitivitySlope that give the BeamNG tyre the Pacejka tyre's load sensitivity
    (its peak grip against load, relative to the reference load), keeping its load coefficient at the reference load.
    mu: [[load, mu]] of the Pacejka tyre. Returns ({"noLoadCoef", "fullLoadCoef", "loadSensitivitySlope"}, rms error)."""
    nc = contact_nodes(props)
    c0 = bng_coef(props, fz0, nc)
    ref = next((m for f, m in mu if abs(f - fz0) < 1), None) or _interp(mu, fz0)
    target = [(f, m / ref) for f, m in mu]
    best = None
    for i in range(0, 301):                                   # the slope per node: 0 to 0.0006 N^-1
        s = i * 2e-6
        no = c0 + s * fz0 / nc
        for j in range(0, 41):                                # the floor: 0.2 to 1.0 of the reference coefficient
            full = c0 * (0.2 + j * 0.02)
            if full > no:
                continue
            err = math.sqrt(sum((max(full, no - s * f / nc) / c0 - r) ** 2 for f, r in target) / len(target))
            if best is None or err < best[0]:
                best = (err, {"noLoadCoef": round(no, 4), "fullLoadCoef": round(full, 4), "loadSensitivitySlope": round(s, 7)})
    return best[1], round(best[0], 4)


def _interp(table, x):
    if x <= table[0][0]:
        return table[0][1]
    for (x0, y0), (x1, y1) in zip(table, table[1:]):
        if x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return table[-1][1]


def benchmark_json(svj_json, props_json, corner=None):
    """For the Wheels workspace: the SVJ tyre ("svj": svj_tyre), its curves ("curves": pacejka_curves), the BeamNG tyre's
    grip against load ("bng": bng_mu of props, the archetype's values with the user's changes), the shape of both
    relative to the reference load ("shape": {"svj_y", "svj_x", "bng"}), and the fitted load sensitivity ("fit")."""
    svj = json.loads(svj_json) if svj_json else {}
    props = json.loads(props_json or "{}")
    t = svj_tyre(svj, corner) if svj else None
    out = {"svj": t, "curves": None, "bng": bng_mu(props) if props else None, "shape": None, "fit": None,
           "contact_nodes": round(contact_nodes(props), 1) if props else None}
    if t and t["pacejka"]:
        cur = pacejka_curves(t)
        out["curves"] = cur
        fz0 = t["fz0"]
        norm = lambda tab: [[f, round(m / (_interp(tab, fz0) or 1), 4)] for f, m in tab]   # noqa: E731
        out["shape"] = {"svj_y": norm(cur["mu_y"]), "svj_x": norm(cur["mu_x"]), "bng": norm(out["bng"]) if out["bng"] else None}
        if props:
            mu = [[f, (my + mx) / 2] for (f, my), (_, mx) in zip(cur["mu_y"], cur["mu_x"])]
            vals, err = fit(props, mu, fz0)
            out["fit"] = {"values": vals, "rms": err}
    return json.dumps(out)


def custom_part(part, vars_, name, overrides):
    """A copy of a donor tyre part with the user's values (a property row at the end of its pressureWheels, so they win),
    under a new name and the same slot type, its meshes kept."""
    from . import scratch
    p = scratch._resolve(copy.deepcopy(part), vars_)
    t = p.get("pressureWheels")
    if isinstance(t, list) and t:
        t.append({k: v for k, v in overrides.items() if k in OVERRIDABLE and isinstance(v, (int, float))})
    p["information"] = {**(p.get("information") or {}), "name": f"{(p.get('information') or {}).get('name') or name} (BeamForge)"}
    p.pop("variables", None)
    return p
