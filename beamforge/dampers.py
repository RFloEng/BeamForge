"""Springs and dampers where they really are, with the wheel's stiffness and damping kept.

Pure Python, standard library only (runs in Pyodide). UNVERIFIED IN-GAME.

An SVJ gives the spring as a wheel rate (spring.rate, force at the wheel per metre of wheel travel; SVJ §9.3) and the
damper as curves at the damper with damper.motion_ratio (damper velocity / wheel velocity; §9.4). Files converted from
Assetto Corsa have no real spring or damper: AC works at the wheel, so the converter puts both on a vertical axis through
the wheel centre at a motion ratio of 1 (at_wheel()). Built that way in BeamNG the unit would stand on the wheel, which
no real car does. So the unit is placed where it really is (a damper part of the sketch: drawn by the user, or
suggest()ed: the SVJ's own strut or mounts where it has them, else on the lower arm), its motion ratio MR measured on
the structure (motion_ratios(): the wheels pushed up in a small static solve), and its beams given

  spring    k = wheel rate / MR^2, preloaded by the corner's sprung load / MR
  damper    c = damping at the wheel / MR^2, bump and rebound, slow and fast (values._slopes on the SVJ's curves)

so the wheel feels what the SVJ says. Without an SVJ: a ride frequency (FREQ) and damping ratios (ZETA_*) on the
corner's sprung mass.
"""

import json
import math
import re

from . import svj as svjmod, values

STEP = 0.01               # m of wheel travel in the solve (linear: any small step gives the same ratio)
DAMPER_LEN = 0.33         # m: a suggested damper's length
ON_ARM = 0.7              # a suggested damper's lower end: this share of the way along the arm, pivot to ball joint
TILT_IN = 0.15            # its top leans inboard by this share of its length
SLIDE_AT = 0.6            # a strut's guide node: this share of the way from its lower end to its top
MIN_MR = 0.15             # a unit that moves less than this with the wheel is placed wrong
FREQ = {"front": 1.5, "rear": 1.7}       # Hz: ride frequencies when the SVJ gives no spring
ZETA_BUMP, ZETA_REBOUND = 0.25, 0.45     # of critical damping, when it gives no damper
FAST, SPLIT = 0.5, 0.1    # fast slope / slow, and the velocity (m/s) where it changes, by default
CORNERS = ("FL", "FR", "RL", "RR")


def _vec(p):
    return [float(x) for x in p] if isinstance(p, list) and len(p) == 3 and all(isinstance(x, (int, float)) for x in p) else None


def mounts(unit):
    """(inboard, outboard) of an SVJ spring or damper (inboard_mount / outboard_mount; inboard_point / outboard_point as
    older converters wrote them), SAE, None where absent."""
    unit = unit or {}
    return (_vec(unit.get("inboard_mount") or unit.get("inboard_point")),
            _vec(unit.get("outboard_mount") or unit.get("outboard_point")))


def wheel_centre(corner):
    hp = (((corner or {}).get("topology") or {}).get("upright") or {}).get("hardpoints") or {}
    return _vec(hp.get("wheel_center")) or _vec((corner or {}).get("position"))


def at_wheel(corner):
    """True when the corner's spring or damper is the Assetto Corsa placeholder: on a vertical axis through the wheel
    centre (axis "vertical", or both mounts within 3 cm of the wheel centre seen from above)."""
    wc = wheel_centre(corner)
    for u in ((corner or {}).get("spring") or {}, (corner or {}).get("damper") or {}):
        if str(u.get("axis") or "").lower() == "vertical":
            return True
        a, b = mounts(u)
        if wc and a and b and all(math.hypot(p[0] - wc[0], p[1] - wc[1]) < 0.03 for p in (a, b)):
            return True
    return False


def wheel_values(svj):
    """Per axle ("front", "rear"): {"wheel_rate" (N/m), "bump", "bump_fast", "rebound", "rebound_fast" (N s/m at the
    wheel), "split" (m/s at the wheel), "at_wheel"} from the SVJ's FL / RL corners (FR / RR when missing); None where the
    file has no value."""
    out = {}
    susp = (svj or {}).get("suspension") or {}
    for axle, (L, R) in (("front", ("FL", "FR")), ("rear", ("RL", "RR"))):
        c = susp.get(L) if isinstance(susp.get(L), dict) else susp.get(R)
        if not isinstance(c, dict):
            continue
        sp, dm = c.get("spring") or {}, c.get("damper") or {}
        dmr = dm.get("motion_ratio") if isinstance(dm.get("motion_ratio"), (int, float)) and dm["motion_ratio"] > 0 else 1.0
        v = {"wheel_rate": float(sp["rate"]) if isinstance(sp.get("rate"), (int, float)) else None, "at_wheel": at_wheel(c),
             "split": None}
        for key, curve in (("bump", dm.get("bump_curve")), ("rebound", dm.get("rebound_curve"))):
            s = values._slopes(curve)                  # at the damper: F_wheel = F_d MR, v_d = MR v_wheel
            v[key] = round(s[0] * dmr * dmr) if s else None
            v[f"{key}_fast"] = round(s[1] * dmr * dmr) if s else None
            if s:
                v["split"] = round(s[2] / dmr, 3)
        out[axle] = v
    return out


def wheel_values_json(svj_json):
    return json.dumps(wheel_values(json.loads(svj_json)))


def defaults(sprung_kg, axle):
    """Wheel values for a corner of this sprung mass when the SVJ gives none: a ride frequency and damping ratios."""
    k = sprung_kg * (2 * math.pi * FREQ[axle]) ** 2
    cc = 2 * math.sqrt(k * sprung_kg)
    return {"wheel_rate": round(k), "bump": round(ZETA_BUMP * cc), "bump_fast": round(ZETA_BUMP * cc * FAST),
            "rebound": round(ZETA_REBOUND * cc), "rebound_fast": round(ZETA_REBOUND * cc * FAST), "split": SPLIT}


# ------------------------------------------------------------------ where the unit goes

def _corner_of(name):
    m = re.search(r"_(f[lr]|r[lr])$", str(name).lower())
    return m.group(1).upper() if m else None


def _lower_legs_sketch(parts, corner, tol=0.002):
    """The lower arm's legs of a corner in the sketch: ([(inner, outer)], wheel centre) in SAE, or (None, None)."""
    suf = "_" + corner.lower()
    mine = [p for p in parts if str(p["name"]).lower().endswith(suf)]
    up = next((p for p in mine if svjmod.part_role(p["name"]) == "hub"), None)
    arms = [p for p in mine if svjmod.part_role(p["name"]) == "arm"]
    arm = next((p for p in arms if "lower" in p["name"].lower()), None) or (arms[0] if len(arms) == 1 else None)
    if not up or not arm:
        return None, None
    wc = next((_vec(q) for q in up.get("points") or []), None)
    ends = [e for ln in up.get("lines") or [] for e in ln]
    legs = []
    for a, b in arm.get("lines") or []:
        a, b = _vec(a), _vec(b)
        if not a or not b:
            continue
        on_a = any(math.dist(a, e) <= tol for e in ends)
        on_b = any(math.dist(b, e) <= tol for e in ends)
        if on_a != on_b:
            legs.append((b, a) if on_a else (a, b))
    return (legs or None), wc


def _lower_legs_svj(svj, corner):
    c = ((svj or {}).get("suspension") or {}).get(corner) or {}
    topo = c.get("topology") or {}
    hp = (topo.get("upright") or {}).get("hardpoints") or {}
    links = [x for x in topo.get("links") or [] if isinstance(x, dict) and x.get("type") in ("arm", "link", None)]
    link = next((x for x in links if re.search(r"lower", str(x.get("name")))), None)
    if not link:
        return None, wheel_centre(c)
    ref = str(link.get("outboard_ref") or "").split(".")[-1]
    out = _vec(link.get("outboard_point")) or _vec(hp.get(ref))
    legs = [(_vec(p), out) for p in link.get("inboard_points") or [] if _vec(p) and out]
    return (legs or None), wheel_centre(c)


def _on_arm(legs, wc):
    """A damper on the lower arm: its lower end on the leg most across the car, ON_ARM of the way out; its top above,
    leaning inboard. (lower, top) in SAE (Z down)."""
    inner, outer = max(legs, key=lambda l: abs(l[1][1] - l[0][1]) / (math.dist(*l) or 1))
    bottom = [inner[i] + ON_ARM * (outer[i] - inner[i]) for i in range(3)]
    side = 1.0 if (wc or outer)[1] > 0 else -1.0                 # SAE Y: right +
    d = [0.0, -side * TILT_IN, -1.0]                              # up (Z down) and inboard
    n = math.sqrt(sum(x * x for x in d))
    top = [bottom[i] + DAMPER_LEN * d[i] / n for i in range(3)]
    return bottom, top


def suggest(parts, svj=None):
    """Damper parts for the corners that have none: {"parts": [{"name", "lines": [[top, lower]]}], "notes": [...]} in
    the SVJ frame (m). From the SVJ's strut (a MacPherson: its top and its lower end on the upright), else the SVJ's own
    mounts (when they are not the Assetto Corsa placeholder at the wheel), else on the corner's lower arm (the sketch's,
    else the SVJ's), to be moved where the real one is: the wheel's stiffness and damping are kept wherever it goes."""
    have = {_corner_of(p["name"]) for p in parts if _kind(p["name"]) == "damper"}
    susp = (svj or {}).get("suspension") or {}
    corners = [c for c in CORNERS if c not in have and (c in susp or any(str(p["name"]).lower().endswith("_" + c.lower()) for p in parts))]
    out, notes = [], []
    for c in corners:
        sc = susp.get(c) or {}
        topo = sc.get("topology") or {}
        hp = (topo.get("upright") or {}).get("hardpoints") or {}
        strut = next((x for x in topo.get("links") or [] if isinstance(x, dict) and (x.get("type") == "strut" or x.get("name") == "strut")), None)
        top = bottom = None
        if strut and (strut.get("inboard_points") or []):
            top = _vec(strut["inboard_points"][0])
            bottom = _vec(hp.get("strut_lower")) or _vec(hp.get("strut_outboard")) or _vec(hp.get("damper_outboard")) or _vec(hp.get("lower_ball_joint"))
            how = "from the SVJ's strut"
        if not (top and bottom) and sc and not at_wheel(sc):
            top, bottom = mounts(sc.get("damper") or sc.get("spring"))
            how = "from the SVJ's damper mounts"
        if not (top and bottom):
            legs, wc = _lower_legs_sketch(parts, c)
            src = "the sketch's"
            if not legs:
                legs, wc = _lower_legs_svj(svj, c)
                src = "the SVJ's"
            if not legs:
                notes.append(f"{c}: no lower arm to put a damper on: draw damper_{c.lower()} where it is")
                continue
            bottom, top = _on_arm(legs, wc)
            how = f"on {src} lower arm (suggested: move it where the real one is)"
            if sc and at_wheel(sc):
                how += "; the SVJ's is the Assetto Corsa placeholder at the wheel"
        out.append({"name": f"damper_{c.lower()}", "lines": [[[round(x, 4) for x in top], [round(x, 4) for x in bottom]]]})
        notes.append(f"{c}: damper_{c.lower()} {how}")
    return {"parts": out, "notes": notes}


def _kind(name):
    from . import skeleton
    return skeleton.kind_of(name)


def suggest_json(parts_json, svj_json=None):
    return json.dumps(suggest(json.loads(parts_json), json.loads(svj_json) if svj_json else None))


# ------------------------------------------------------------------ the motion ratio, on the structure

def _solve(A, b):
    from .kinematics import _solve as s
    return s(A, b)


def motion_ratios(nodes, beams, free, pushed, units, guides=()):
    """{unit name: MR} with the pushed nodes moved up (BeamNG z) by STEP together: each unit's shortening over the step.
    nodes: {id: [x, y, z]}; beams: [(a, b, k)] that carry (the units themselves not among them); free: the nodes that
    move (the rest are held); pushed: the axle nodes of one axle; units: {name: (a, b)}; guides: [(slider, lower, top,
    s)], a strut's guide: the slider (on the upright) stays on the line from the lower end (on the upright) to the top,
    at s of the way (as a BeamNG slide node on its rail)."""
    free = [n for n in free if n in nodes]
    idx = {n: i for i, n in enumerate(free)}
    N = 3 * len(free)
    if not N:
        return {}
    A = [[0.0] * N for _ in range(N)]
    f = [0.0] * N
    kmax = max([k for _, _, k in beams] or [1.0])

    def add(r):                                   # A += w r r^T, r sparse {dof: coef}
        for p, x in r.items():
            row = A[p]
            for q, y in r.items():
                row[q] += x * y
    for a, b, k in beams:
        if a not in idx and b not in idx:
            continue
        d = [nodes[b][i] - nodes[a][i] for i in range(3)]
        L = math.sqrt(sum(x * x for x in d))
        if L < 1e-9:
            continue
        u = [x / L for x in d]
        r = {}
        for n, s in ((a, -1.0), (b, 1.0)):
            if n in idx:
                for i in range(3):
                    r[3 * idx[n] + i] = r.get(3 * idx[n] + i, 0.0) + s * u[i] * math.sqrt(k)
        add(r)
    big = kmax * 1e3
    for sl, lo, top, s in guides:                 # the slider's motion across the strut follows the lower end's
        d = [nodes[top][i] - nodes[lo][i] for i in range(3)]
        L = math.sqrt(sum(x * x for x in d)) or 1.0
        u = [x / L for x in d]
        e1 = [u[1], -u[0], 0.0] if abs(u[2]) < 0.9 else [0.0, u[2], -u[1]]
        n1 = math.sqrt(sum(x * x for x in e1))
        e1 = [x / n1 for x in e1]
        e2 = [u[1] * e1[2] - u[2] * e1[1], u[2] * e1[0] - u[0] * e1[2], u[0] * e1[1] - u[1] * e1[0]]
        for e in (e1, e2):
            r = {}
            for n, c in ((sl, 1.0), (lo, -(1.0 - s))):
                if n in idx:
                    for i in range(3):
                        r[3 * idx[n] + i] = r.get(3 * idx[n] + i, 0.0) + c * e[i] * math.sqrt(big)
            add(r)
    for i in range(N):                            # a whisker of stiffness everywhere: a rod's spin has none
        A[i][i] += kmax * 1e-9
    for n in pushed:
        if n in idx:
            i = 3 * idx[n] + 2
            A[i][i] += big
            f[i] += big * STEP
    x = _solve(A, f)

    def at(n):
        return [nodes[n][i] + (x[3 * idx[n] + i] if n in idx else 0.0) for i in range(3)]
    return {name: round((math.dist(nodes[a], nodes[b]) - math.dist(at(a), at(b))) / STEP, 4) for name, (a, b) in units.items()}
