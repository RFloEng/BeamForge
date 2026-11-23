"""Assetto Corsa's tyre model (the one a car's data/tyres.ini drives), put in the same terms as the SVJ's Pacejka tyre (tyres.py).

Pure Python, standard library only (runs in Pyodide).

The force model is the one rustyAC documents for AC 1.16 (github.com/thesupersupersigma/rustyAC, crates/rustyac-physics/src/tyre/sctm.rs),
written here from its equations (the repo's physics is GPL-3.0-or-later, so no code is copied). In short:

  peak grip       Dy0, Dx0 at the load: the DY_CURVE / DX_CURVE table when the file has one, otherwise a power law about the reference
                  load FZ0, D = DY_REF * (load / FZ0) ** (LS_EXPY - 1) (the same with DX_REF and LS_EXPX)
  camber          Dy divided by 1 + DCAMBER_0 * c - DCAMBER_1 * c**2 (c = camber, floored at -0.9 in the divisor's loss); the slip angle gains
                  sin(camber) * CAMBER_GAIN
  speed           Dy and Dx divided by 1 + SPEED_SENSITIVITY * the sliding speed
  stiffness       cf = 3 * 78.125 / max_slip; max_slip is the tangent of the peak slip angle (FRICTION_LIMIT_ANGLE here, see below)
  combined slip   s_x = kappa / (1 + kappa), s_y = tan(alpha) / (1 + kappa); slip = |(s_x, s_y)| (a p-norm for COMBINED_FACTOR other than 0 / 2)
  curve           up to the peak (n = slip / max_slip): (1 - n)**2 * K * slip + (3 - 2 n) * n**2, K = 2 * 0.0064 * cf; past it
                  1 / ((slip - peak) * FALLOFF_SPEED + 1) * (1 - FALLOFF_LEVEL) + FALLOFF_LEVEL
  forces          Fy = curve * Dy * (s_y / slip) * load; Fx = curve(cf * CX_MULT) * Dx * (s_x / slip) * load

Where this differs from the game, so the numbers are shown as an approximation: the game takes the peak slip angle (its maxSlip0 / maxSlip1)
from a brush table made from the tyre's data; here it is FRICTION_LIMIT_ANGLE of the file (degrees, both at FZ0 and 2 FZ0) when present, else 9 degrees.
Pressure, temperature, wear and the relaxation lag are not in it (a steady-state tyre at its ideal pressure and temperature). tyres.ini
keys read: [HEADER] VERSION, [BASIC] WIDTH RADIUS, and per compound ([FRONT] [REAR] [FRONT_n] [REAR_n]) the keys above plus FZ0, DY_CURVE
and DX_CURVE (a table [[load, D]], read from the .lut files by lut()). With no DY_REF the model uses 1.0, with no FZ0 2000 N.
"""

import math

FZ0_DEFAULT = 2000.0
LIMIT_ANGLE_DEFAULT = 9.0           # deg: the peak slip angle when the file gives no FRICTION_LIMIT_ANGLE
LOADS = (1000.0, 2000.0, 3000.0, 4000.0, 5000.0, 6000.0, 7000.0, 8000.0)      # the same grid as tyres.LOADS
SPEED = 20.0                        # m/s the curves are drawn at (the speed sensitivity needs one)
FALLBACK = {"DY_REF": 1.0, "DX_REF": 1.0, "LS_EXPY": 1.0, "LS_EXPX": 1.0, "SPEED_SENSITIVITY": 0.003, "CAMBER_GAIN": 0.0,
            "DCAMBER_0": 0.1, "DCAMBER_1": -0.8, "FALLOFF_LEVEL": 1.0, "FALLOFF_SPEED": 0.0, "CX_MULT": 1.0, "BRAKE_DX_MOD": 1.0,
            "COMBINED_FACTOR": 2.0, "FZ0": FZ0_DEFAULT, "FRICTION_LIMIT_ANGLE": LIMIT_ANGLE_DEFAULT}


# ---- reading the files ----------------------------------------------------------------------------------------------------------------------

def _num(s):
    try:
        return float(str(s).split(";")[0].split("//")[0].strip())
    except ValueError:
        return None


def parse_ini(text):
    """tyres.ini (or any AC ini) as {SECTION: {KEY: value}}: numbers become floats, other values stay strings, comments (; and //) go."""
    out, sec = {}, None
    for raw in (text or "").splitlines():
        line = raw.split(";")[0].split("//")[0].strip()
        if not line:
            continue
        if line.startswith("[") and line.endswith("]"):
            sec = line[1:-1].strip().upper()
            out.setdefault(sec, {})
        elif "=" in line and sec is not None:
            k, v = line.split("=", 1)
            n = _num(v)
            out[sec][k.strip().upper()] = n if n is not None else v.strip()
    return out


def lut(text):
    """An AC .lut file ('x|y' per line) as [[x, y]] sorted by x."""
    pts = []
    for raw in (text or "").splitlines():
        line = raw.split(";")[0].split("//")[0].strip()
        if "|" in line:
            a, b = (_num(p) for p in line.split("|", 1))
            if a is not None and b is not None:
                pts.append([a, b])
    return sorted(pts)


def compounds(ini):
    """The tyre compounds of a parsed tyres.ini: {"FRONT": ..., "REAR": ..., "FRONT_1": ...} (sections that have DY_REF or a curve)."""
    return {k: v for k, v in ini.items() if (k.startswith("FRONT") or k.startswith("REAR")) and ("DY_REF" in v or "DY_CURVE" in v)}


def params(ini, section="FRONT", luts=None):
    """The model's parameters for one compound: its keys over the FALLBACK, plus the size ([BASIC] / [section]: WIDTH, RADIUS in m) and the
    curve tables: luts is {file name: .lut text} for DY_CURVE / DX_CURVE (a name not in it leaves the power law)."""
    sec = ini.get(section.upper())
    if sec is None:
        raise KeyError(f"no [{section}] in the tyres.ini")
    p = dict(FALLBACK)
    p.update({k: v for k, v in sec.items() if isinstance(v, float)})
    p["section"] = section.upper()
    for key in ("WIDTH", "RADIUS"):
        v = sec.get(key, (ini.get("BASIC") or {}).get(key))
        if isinstance(v, float):
            p[key] = v
    for key in ("DY_CURVE", "DX_CURVE"):
        name = sec.get(key)
        if isinstance(name, str) and luts and name in luts:
            p[key] = lut(luts[name])
    return p


# ---- the model --------------------------------------------------------------------------------------------------------------------------------

def _spline(table, x):
    """Piecewise-linear interpolation (the game uses a cubic spline through the same points), flat outside the table."""
    if x <= table[0][0]:
        return table[0][1]
    for (x0, y0), (x1, y1) in zip(table, table[1:]):
        if x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0) if x1 > x0 else y1
    return table[-1][1]


def peak_mu(p, load, axis="y"):
    """The static peak friction coefficient at a load: DY / DX_CURVE if the file has one, else DY_REF * (load / FZ0) ** (LS_EXP - 1)."""
    if load <= 0:
        return 0.0
    curve = p.get("D" + axis.upper() + "_CURVE")
    if curve:
        return _spline(curve, load)
    return p["D" + axis.upper() + "_REF"] * (load / p["FZ0"]) ** (p["LS_EXP" + axis.upper()] - 1.0)


def _pure(p, slip, max_slip, stiff):
    k = stiff * 2 * 0.0064
    peak = 3.0 / k if k else max_slip
    if slip <= peak:
        n = slip / peak
        return (1 - n) ** 2 * k * slip + (3 - 2 * n) * n * n
    asy = p["FALLOFF_LEVEL"]
    return 1.0 / ((slip - peak) * p["FALLOFF_SPEED"] + 1.0) * (1 - asy) + asy


def solve(p, load, alpha, kappa, camber=0.0, speed=SPEED, grip=1.0):
    """The force of the tyre at a steady state: load (N), slip angle alpha (rad), slip ratio kappa, camber (rad), speed (m/s), grip (the surface
    and temperature multiplier u, 1.0). Returns {"fy", "fx", "mz", "dy", "dx", "nd_slip"} in newtons / newton-metres (Fy to the right with
    positive alpha, Fx the drive force with positive kappa)."""
    zero = {"fy": 0.0, "fx": 0.0, "mz": 0.0, "dy": 0.0, "dx": 0.0, "nd_slip": 0.0}
    if load <= 0 or (alpha == 0 and kappa == 0 and camber == 0):
        return zero
    dy, dx = grip * peak_mu(p, load, "y"), grip * peak_mu(p, load, "x")
    if kappa < 0:
        dx *= p["BRAKE_DX_MOD"]
    ca = alpha + math.sin(camber) * p["CAMBER_GAIN"]
    c = -abs(camber)
    loss = max(-0.9, c * p["DCAMBER_0"] - c * c * p["DCAMBER_1"])
    dy /= loss + 1.0
    slide = math.hypot(speed * math.sin(alpha), math.cos(alpha) * speed * kappa)
    sens = slide * p["SPEED_SENSITIVITY"] + 1.0
    dy, dx = dy / sens, dx / sens
    max_slip = math.tan(math.radians(p["FRICTION_LIMIT_ANGLE"])) * ((grip - 1) * 0.75 + 1)     # (the same at FZ0 and 2 FZ0 here)
    cf = 3 * 78.125 / max_slip
    kl = max(kappa, -0.99999)
    sx, sy = kappa / (1 + kl), math.tan(ca) / (1 + kl)
    f = p["COMBINED_FACTOR"]
    slip = math.hypot(sx, sy) if f in (0.0, 2.0) else (abs(sx) ** f + abs(sy) ** f) ** (1.0 / f)
    if slip == 0:
        return zero
    fy = _pure(p, slip, max_slip, cf) * dy * (sy / slip) * load
    fx = (sx / slip) * _pure(p, slip, max_slip, cf * p["CX_MULT"]) * dx * load
    peak = 3.0 / (cf * 2 * 0.0064)
    nd = slip / peak
    t = min(1.0, max(0.0, 1 - 0.8 * nd))
    mz = -((3 - 2 * t) * t * t * 1.1 - 0.1) * 0.12 * 0.1 * fy        # trail from an assumed 0.1 m contact patch; a rough guide only
    return {"fy": fy, "fx": fx, "mz": mz, "dy": dy, "dx": dx, "nd_slip": nd}


def curves(p, loads=(2000.0, 4000.0, 6000.0)):
    """The same dictionary as tyres.pacejka_curves: {"fy": {load: [[deg, N]]}, "fx": {load: [[slip %, N]]}, "mu_y": [[load, peak Fy / Fz]],
    "mu_x": [[load, peak Fx / Fz]]} (steady state, no camber, SPEED m/s)."""
    out = {"fy": {}, "fx": {}, "mu_y": [], "mu_x": []}
    alphas = [i * 0.25 for i in range(0, 61)]
    kappas = [i * 0.5 for i in range(0, 61)]
    for fz in loads:
        out["fy"][str(int(fz))] = [[a, round(abs(solve(p, fz, math.radians(a), 0.0)["fy"]), 1)] for a in alphas]
        out["fx"][str(int(fz))] = [[k, round(abs(solve(p, fz, 0.0, k / 100)["fx"]), 1)] for k in kappas]
    for fz in LOADS:
        out["mu_y"].append([fz, round(max(abs(solve(p, fz, math.radians(a), 0.0)["fy"]) for a in alphas) / fz, 4)])
        out["mu_x"].append([fz, round(max(abs(solve(p, fz, 0.0, k / 100)["fx"]) for k in kappas) / fz, 4)])
    return out


def profile(p, rim_in=None):
    """The AC tyre's characteristics in the terms of tyrelab.svj_profile, so tyrelab.score / rank can compare it with the game's tyres:
    {"name", "size", "rim_in", "width_mm", "radius", "fz0", "grip_y", "grip_x", "grip", "shape", "peak_deg", "stiffness", "class", "curves"}."""
    from . import tyrelab, tyres
    cur = curves(p)
    ref = max(p["FZ0"], LOADS[0])
    gy, gx = tyres._interp(cur["mu_y"], ref), tyres._interp(cur["mu_x"], ref)
    both = [[f, (a + b) / 2] for (f, a), (_, b) in zip(cur["mu_y"], cur["mu_x"])]
    angles = [i * 0.1 for i in range(1, 160)]
    fy = [abs(solve(p, ref, math.radians(a), 0.0)["fy"]) for a in angles]
    peak = angles[fy.index(max(fy))]
    stiff = abs(solve(p, ref, math.radians(0.5), 0.0)["fy"]) / math.radians(0.5) / ref
    grip = (gy + gx) / 2
    cls = "race" if grip >= tyrelab.RACE_MU else "sport" if grip >= tyrelab.SPORT_MU else "standard"
    return {"name": p.get("section", "AC"), "size": None, "rim_in": rim_in, "width_mm": round(p["WIDTH"] * 1000) if p.get("WIDTH") else None,
            "radius": p.get("RADIUS"), "fz0": ref, "grip_y": round(gy, 3), "grip_x": round(gx, 3), "grip": round(grip, 3),
            "shape": tyrelab._norm(both, ref), "peak_deg": round(peak, 1), "stiffness": round(stiff, 2), "class": cls, "curves": cur}
