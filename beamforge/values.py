"""Values to take from an SVJ onto the base vehicle: springs, dampers, tyres (roadmap step 3).

Pure Python, standard library only (runs in Pyodide). Each value is read on both sides, the base
vehicle (its active parts, as configured) and the SVJ, per axle, and written into the new vehicle by
export.build when the user takes it.

  springs   SVJ spring.rate is the wheel rate (SVJ spec). BeamNG's coil spring is a beam with a
            precompressionRange between the hub side and the body; its beamSpring (N/m) is the wheel
            rate / MR^2, MR the motion ratio of that beam in the base vehicle as fitted, measured on its
            beams (kinematics.py). Without one: the SVJ's own coil rate (wheel rate / the SVJ spring's
            motion_ratio^2, else the study's static motion ratio, else 1).
  dampers   SVJ bump_curve / rebound_curve are [velocity m/s, force N] at the damper (SVJ spec), with
            damper.motion_ratio (damper / wheel, default 1). BeamNG's damper is a |BOUNDED beam:
            beamDamp / beamDampRebound take the slow slope, beamDampFast / beamDampReboundFast the fast
            one, beamDampVelocitySplit where it changes (_slopes). The damping at the wheel is slope x
            MR_svj^2, so the beam takes slope x (MR_svj / MR_base)^2, MR_base measured like the spring's.
  tyres     pressureWheels radius (unloaded; the SVJ loaded radius plus a 10 mm deflection when the
            file gives no unloaded radius) and tireWidth.
  aero      the SVJ drag area (Cd x frontal area): every drag triangle's dragCoef scaled so the base's
            drag area estimate (drag_area) matches; liftCoef written as it was.
  steering  the SVJ's turns lock to lock into the steering hydros' steeringWheelLock (degrees each way);
            the road-wheel lock (the hydros' factor and the steering arms) is kept.
  mass, CG  the base's mass is its node weights (default 25 kg) plus the wheels the game builds
            (2 numRays tyre nodes and 2 numRays hub nodes per wheel); its CG the weighted node
            position. SVJ chassis.mass_total and center_of_gravity (SAE, from the front-axle centre on
            the ground). Taking them rescales every non-wheel node weight: one factor for the mass,
            times a linear ramp along the car and one in height that put the weighted centre on
            the SVJ's CG (each weight kept at 30 % or more of its own).

A beam whose value is a tuning variable ("$spring_F") is not rewritten: the variable is set in the
new configuration instead, so the tuning menu keeps working.
"""

import json
import math
import re

from beamforge import beamng, jbeam

DEFLECTION = 0.010      # m: unloaded minus loaded tyre radius, when the SVJ gives only the loaded one
# parts that hold a corner's spring or damper (not the hood, trunk, steering or mirror dampers)
SUSPENSION_PART = re.compile(r"strut|shock|spring|coilover|damper|suspension", re.I)


def _axle_of(y, yf, yr):
    return "front" if abs(y - yf) <= abs(y - yr) else "rear"


def _slopes(curve):
    """(slow slope, fast slope, split velocity) of a [velocity, force] curve; None when it has no slope.

    BeamNG's damper is two lines through the origin: beamDamp up to beamDampVelocitySplit, then
    beamDampFast. Every curve point is tried as the split; the slow slope is the least-squares line
    through the origin over the points up to it, the fast one over the points after it (from the
    split's force), and the pair with the smallest squared force error wins (a two-point curve is one
    line; a curve that is two lines already is matched exactly)."""
    pts = sorted(p for p in curve or [] if isinstance(p, list) and len(p) == 2)
    pts = [p for p in pts if p[0] > 0] if pts and pts[0][0] <= 0 else pts
    if not pts:
        return None
    best = None
    for k in range(len(pts)):
        lo, hi = pts[:k + 1], pts[k + 1:]
        slow = sum(v * f for v, f in lo) / sum(v * v for v, _ in lo)
        vs, fs = pts[k][0], slow * pts[k][0]
        fast = (sum((v - vs) * (f - fs) for v, f in hi) / sum((v - vs) ** 2 for v, _ in hi)) if hi else slow
        err = sum((f - slow * v) ** 2 for v, f in lo) + sum((f - fs - fast * (v - vs)) ** 2 for v, f in hi)
        if best is None or err < best[0] - 1e-9:
            best = (err, slow, fast, vs)
    _, slow, fast, vs = best
    if slow <= 0:
        return None
    return round(slow, 3), round(max(fast, 0.0), 3), vs


def tyre_size(svj, corner):
    """A corner's tyre and rim size from the tyre set it refers to (tire.set_ref or wheel.set_ref in
    tires.sets): {"width" (m), "aspect" (0-1), "rim_in" (inches), "diameter" (m)}, keys missing where
    the file has no value."""
    ref = ((corner.get("tire") or {}).get("set_ref") or (corner.get("wheel") or {}).get("set_ref"))
    ts = (((svj.get("tires") or {}).get("sets") or {}).get(ref) or {}) if ref else {}
    d, rim = ts.get("dimensions") or {}, ts.get("rim") or {}
    out = {}
    if isinstance(d.get("section_width"), (int, float)):
        out["width"] = d["section_width"]
    if isinstance(d.get("aspect_ratio"), (int, float)):
        out["aspect"] = d["aspect_ratio"] if d["aspect_ratio"] < 1.5 else d["aspect_ratio"] / 100
    r = d.get("rim_diameter_code") or (rim.get("diameter") / 0.0254 if isinstance(rim.get("diameter"), (int, float)) else None)
    r = r or ((corner.get("wheel") or {}).get("rim_diameter") or 0) / 0.0254 or None
    if r:
        out["rim_in"] = round(r)
    if isinstance(d.get("overall_diameter"), (int, float)):
        out["diameter"] = d["overall_diameter"]
    elif out.get("width") and out.get("aspect") and out.get("rim_in"):
        out["diameter"] = out["rim_in"] * 0.0254 + 2 * out["width"] * out["aspect"]
    return out


WHEEL_SIZE = re.compile(r"(?<![\d.])(\d{2})x(\d{1,2}(?:\.\d+)?)(?![\d.])")      # 15x7: rim diameter x width, inches
TYRE_SIZE = re.compile(r"(?<!\d)(\d{3})_(\d{2})_(\d{2})(?!\d)")                 # 195_55_15: width mm, aspect %, rim in


def svj_values(svj, study=None, base_mr=None):
    """Per axle ("front", "rear"): {"wheel_rate", "spring_mr", "coil_rate", "damp_bump", "damp_bump_fast",
    "damp_rebound", "damp_rebound_fast", "damp_split", "tyre_radius", "tyre_width"} from the SVJ's FL / RL
    corners (FR / RR when the left one is missing); None where the file has no value. study: the
    suspension study (suspension.study_svj) for motion ratios the file does not give; base_mr: the base
    vehicle's measured ratios ({axle: {"spring", "damper"}}, kinematics.motion_ratios)."""
    out = {}
    susp = svj.get("suspension") or {}
    for axle, (L, R) in (("front", ("FL", "FR")), ("rear", ("RL", "RR"))):
        c = susp.get(L) if isinstance(susp.get(L), dict) else susp.get(R)
        if not isinstance(c, dict):
            continue
        spring, damper, wheel = c.get("spring") or {}, c.get("damper") or {}, c.get("wheel") or {}
        st = ((study or {}).get("corners") or {}).get(axle, {}).get("static") or {}
        mr = spring.get("motion_ratio") or st.get("motion_ratio") or 1.0
        rate = spring.get("rate") if isinstance(spring.get("rate"), (int, float)) else None
        bm = (base_mr or {}).get(axle) or {}
        smr = bm.get("spring") if (bm.get("spring") or 0) > 0.05 else mr      # the beam's ratio in the base
        v = {"wheel_rate": rate, "spring_mr": round(smr, 3), "spring_mr_from": "base" if smr is not mr else "svj",
             "coil_rate": round(rate / (smr * smr)) if rate else None}
        dmr = damper.get("motion_ratio") or 1.0
        bdm = bm.get("damper") if (bm.get("damper") or 0) > 0.05 else dmr
        k = (dmr / bdm) ** 2                              # slope at the damper -> the base's damper beam
        v["damper_mr"] = round(bdm, 3)
        for key, curve in (("bump", damper.get("bump_curve")), ("rebound", damper.get("rebound_curve"))):
            s = _slopes(curve)
            v[f"damp_{key}"] = round(s[0] * k) if s else None
            v[f"damp_{key}_fast"] = round(s[1] * k) if s else None
            v["damp_split"] = round(s[2], 3) if s else v.get("damp_split")
        tire = c.get("tire") or {}
        unloaded = tire.get("unloaded_radius") or wheel.get("unloaded_radius")
        loaded = tire.get("loaded_radius") or wheel.get("loaded_radius")
        size = tyre_size(svj, c)
        if not unloaded and not loaded and size.get("diameter"):
            unloaded = size["diameter"] / 2                # the tyre set the corner refers to (tires.sets)
        v["tyre_radius"] = round(unloaded, 4) if unloaded else (round(loaded + DEFLECTION, 4) if loaded else None)
        v["tyre_width"] = tire.get("width") or tire.get("section_width") or wheel.get("tire_width") or size.get("width")
        out[axle] = v
    return out


def _active(configured):
    out = []

    def walk(n):
        if n.get("part"):
            out.append(n["part"])
        for c in n.get("children", []):
            walk(c)
    walk(configured["tree"])
    return out


def _axles(configured):
    ws = configured.get("wheels") or []
    ys = sorted(w["centre"][1] for w in ws)
    return (ys[0], ys[-1]) if ys else (-1.3, 1.3)


def springs_and_dampers(model, configured):
    """The base vehicle's coil springs and dampers: [{"part", "row", "kind": "spring" | "damper",
    "axle", "a", "b", "values": {key: value as written}}], row being the index in the part's beams
    table, in the parts that hold a corner's spring or damper (SUSPENSION_PART). A spring is a |NORMAL
    beam with a precompressionRange; a damper a |BOUNDED beam with a rebound damping (a bump stop or
    limiter has none)."""
    parts = beamng._parts_held(model)
    nodes = configured["geometry"]["nodes"]
    yf, yr = _axles(configured)
    out = []
    for name in _active(configured):
        rows = (parts.get(name) or {}).get("part", {}).get("beams")
        if not SUSPENSION_PART.search(name) or not (isinstance(rows, list) and rows and isinstance(rows[0], list)):
            continue
        head = [str(h).rstrip(":") for h in rows[0]]
        props = {}
        for i, row in enumerate(rows[1:], 1):
            if isinstance(row, dict):
                props.update(row)
                continue
            if not isinstance(row, list):
                continue
            inline = row[-1] if row and isinstance(row[-1], dict) else {}
            rec = dict(props)
            rec.update(inline)
            rec.update(zip(head, row[:-1] if inline else row))
            a, b = rec.get("id1"), rec.get("id2")
            if a not in nodes or b not in nodes:
                continue
            bt = str(rec.get("beamType", ""))
            kind = None
            if "precompressionRange" in rec and rec.get("beamSpring") not in (0, None, "0"):
                kind = "spring"                           # |NORMAL, or |BOUNDED with its bump stop (small hatchback)
            elif "BOUNDED" in bt and rec.get("beamDampRebound") is not None and                     any(_num_or(rec.get(k), {}, 1.0) != 0 for k in ("beamDamp", "beamDampRebound")):
                kind = "damper"                           # (zero slow damping: a high-speed bump damper, left)
            if not kind:
                continue
            y = (nodes[a][1] + nodes[b][1]) / 2
            keys = ("beamSpring", "precompressionRange") if kind == "spring" else ("beamDamp", "beamDampRebound", "beamDampFast", "beamDampReboundFast", "beamDampVelocitySplit")
            out.append({"part": name, "row": i, "kind": kind, "axle": _axle_of(y, yf, yr), "a": a, "b": b,
                        "values": {k: rec.get(k) for k in keys if k in rec}})
    return out


def tyres(model, configured):
    """The base vehicle's tyre values per axle: {axle: {"part", "radius", "width"}}, from the parts whose
    pressureWheels set radius / tireWidth (the tyre parts)."""
    parts = beamng._parts_held(model)
    yf, yr = _axles(configured)
    by_group = {}
    for w in configured.get("wheels") or []:
        by_group[w["group"]] = _axle_of(w["centre"][1], yf, yr)
    out = {}
    for name in _active(configured):
        pw = (parts.get(name) or {}).get("part", {}).get("pressureWheels")
        if not (isinstance(pw, list) and pw):
            continue
        r = next((x["radius"] for x in pw[1:] if isinstance(x, dict) and "radius" in x), None)
        w = next((x["tireWidth"] for x in pw[1:] if isinstance(x, dict) and "tireWidth" in x), None)
        if r is None:
            continue
        axle = "front" if re.search(r"_F(_|$)", name) else "rear" if re.search(r"_R(_|$)", name) else None
        if axle is None:
            fb = (parts.get(name) or {}).get("part", {}).get("flexbodies") or []
            for row in fb[1:]:
                if isinstance(row, list) and len(row) > 1 and isinstance(row[1], list):
                    axle = next((by_group[g] for g in row[1] if g in by_group), None)
                    if axle:
                        break
        out.setdefault(axle or "front", {"part": name, "radius": r, "width": w})
    return out


def table(model, configured_json, svj_json, study_json=None):
    """The rows of the "values from the SVJ" table: [{"key", "label", "unit", "axle", "base", "svj"}]."""
    configured, svj = json.loads(configured_json), json.loads(svj_json)
    from beamforge import kinematics
    sv = svj_values(svj, json.loads(study_json) if study_json else None, kinematics.motion_ratios(model, configured)[0])
    sd = springs_and_dampers(model, configured)
    ty = tyres(model, configured)
    vars_ = {x["name"]: x["value"] for x in configured["variables"]}

    def base(kind, axle, key):
        for r in sd:
            if r["kind"] == kind and r["axle"] == axle and key in r["values"]:
                v = r["values"][key]
                v = vars_.get(v, v) if isinstance(v, str) and v.startswith("$") and not v.startswith("$=") else v
                try:
                    return jbeam.to_float(v, vars_, None)
                except (jbeam.UnresolvedValue, TypeError, ValueError):
                    return None
        return None
    rows = []
    for axle in ("front", "rear"):
        s = sv.get(axle, {})
        rows += [
            {"key": f"spring_{axle}", "label": f"{axle.capitalize()} spring (coil)", "unit": "N/m", "axle": axle,
             "base": base("spring", axle, "beamSpring"), "svj": s.get("coil_rate"),
             "note": f"wheel rate {s.get('wheel_rate')} N/m / MR {s.get('spring_mr')}^2" if s.get("coil_rate") else ""},
            {"key": f"damp_bump_{axle}", "label": f"{axle.capitalize()} damper, bump", "unit": "N/(m/s)", "axle": axle,
             "base": base("damper", axle, "beamDamp"), "svj": s.get("damp_bump")},
            {"key": f"damp_rebound_{axle}", "label": f"{axle.capitalize()} damper, rebound", "unit": "N/(m/s)", "axle": axle,
             "base": base("damper", axle, "beamDampRebound"), "svj": s.get("damp_rebound")},
            {"key": f"tyre_radius_{axle}", "label": f"{axle.capitalize()} tyre radius", "unit": "m", "axle": axle,
             "base": (ty.get(axle) or {}).get("radius"), "svj": s.get("tyre_radius")},
        ]
    sp = svj_powertrain(svj)
    pb = powertrain_base(model, configured, sp["driven"] if sp["driven"] in ("front", "rear") else None)
    (bt, bp_), (st, sp_) = _peak(pb.get("torque") and [[r, t + _interp(pb.get("exhaust_mod") or [], r)] for r, t in pb["torque"]]), _peak(sp["torque"])
    ratios = lambda r: " ".join(f"{x:g}" for x in r if isinstance(x, (int, float)) and x > 0) if r else None  # noqa: E731
    rows += [{"key": "engine", "label": "Peak torque (curve, idle and max rpm)", "unit": "Nm", "axle": None, "base": bt, "svj": st,
              "note": f"base {pb.get('idle_rpm')}-{pb.get('max_rpm')} rpm, SVJ {sp['idle_rpm']}-{sp['max_rpm']} rpm"
                      + (f"; the base's {pb['turbo']} adds boost on top" if pb.get("turbo") else "")},
             {"key": "engine_power", "label": "Peak power", "unit": "kW", "axle": None, "base": bp_, "svj": sp_, "info": True},
             {"key": "gears", "label": "Gear ratios", "unit": "", "axle": None, "base": ratios(pb.get("ratios")), "svj": ratios(sp["ratios"])},
             {"key": "final_drive", "label": f"Final drive ({sp['driven'] or 'driven'} axle)", "unit": "", "axle": None,
              "base": pb.get("final_drive"), "svj": sp["final_drive"]}]
    starget, scd, sarea = svj_aero(svj)
    rows.append({"key": "aero", "label": "Drag area CdA (estimate)", "unit": "m2", "axle": None,
                 "base": round(drag_area(aero_triangles(model, configured)), 3), "svj": starget,
                 "note": (f"SVJ Cd {scd} x frontal area {sarea} m2; " if starget else "")
                 + "the base's from its aero triangles (sum of coef x area x facing^2), an estimate"})
    _, bturns = steering_base(model, configured)
    sturns, sratio = svj_steering(svj)
    rows.append({"key": "steering", "label": "Steering wheel turns, lock to lock", "unit": "", "axle": None,
                 "base": bturns, "svj": sturns,
                 "note": (f"the SVJ's overall ratio is {sratio}:1; the base's road-wheel lock (its steering arms) is kept, "
                          "so the overall ratio follows the turns" if sratio else "")})
    mc, (sm, sy, sz) = mass_and_cg(model, configured), svj_mass(svj)
    rows += [{"key": "mass", "label": "Total mass", "unit": "kg", "axle": None, "base": mc["mass"], "svj": sm},
             {"key": "cg_y", "label": "CG behind the front axle", "unit": "m", "axle": None, "base": mc["cg_behind_front_axle"], "svj": sy},
             {"key": "cg_z", "label": "CG height", "unit": "m", "axle": None, "base": mc["cg_height"], "svj": sz}]
    return json.dumps([r for r in rows if r["base"] is not None or r["svj"] is not None])


DEFAULT_NODE_WEIGHT = 25.0     # kg, the game's node weight when a part gives none
MIN_FACTOR = 0.3               # a node keeps at least this share of its weight when the CG is moved


def node_weights(model, configured):
    """The base vehicle's node masses: ([(part, row index, node id, kg, [x, y, z])], wheel kg). Rows of
    the active parts' node tables; positions as configured (fit and moves included)."""
    parts = beamng._parts_held(model)
    nodes = configured["geometry"]["nodes"]
    vars_ = {x["name"]: x["value"] for x in configured["variables"] if isinstance(x["value"], (int, float))}
    out, wheel_kg = [], 0.0
    for name in _active(configured):
        part = (parts.get(name) or {}).get("part", {})
        rows = part.get("nodes")
        if isinstance(rows, list) and rows and isinstance(rows[0], list):
            head = [str(h).rstrip(":") for h in rows[0]]
            props = {}
            for i, row in enumerate(rows[1:], 1):
                if isinstance(row, dict):
                    props.update(row)
                    continue
                if not isinstance(row, list) or "id" not in head or len(row) <= head.index("id"):
                    continue
                nid = str(row[head.index("id")])
                if nid not in nodes:
                    continue
                inline = row[-1] if isinstance(row[-1], dict) else {}
                w = inline.get("nodeWeight", props.get("nodeWeight", DEFAULT_NODE_WEIGHT))
                try:
                    w = jbeam.to_float(w, vars_, DEFAULT_NODE_WEIGHT)
                except (jbeam.UnresolvedValue, TypeError, ValueError):
                    w = DEFAULT_NODE_WEIGHT
                out.append((name, i, nid, w, nodes[nid]))
    # the wheels the game builds: numRays rays, two tyre nodes and two hub nodes each
    table = []
    for name in _active(configured):
        pw = (parts.get(name) or {}).get("part", {}).get("pressureWheels")
        if isinstance(pw, list) and pw and isinstance(pw[0], list):
            table += pw if not table else pw[1:]
    for r in jbeam.expand_table(table):
        if r.get("node1") in nodes and r.get("node2") in nodes:
            rays = _num_or(r.get("numRays"), vars_, 0)
            wheel_kg += 2 * rays * (_num_or(r.get("nodeWeight"), vars_, 0) + _num_or(r.get("hubNodeWeight"), vars_, 0))
    return out, wheel_kg


def _num_or(v, vars_, default):
    try:
        return jbeam.to_float(v, vars_, default)
    except (jbeam.UnresolvedValue, TypeError, ValueError):
        return default


def mass_and_cg(model, configured):
    """{"mass" kg (nodes and wheels), "cg_behind_front_axle" m, "cg_height" m} of the base vehicle (the
    CG from the node masses only; the wheels sit near the axle height and change it little)."""
    rows, wheel_kg = node_weights(model, configured)
    m = sum(r[3] for r in rows)
    if not m:
        return {"mass": None, "cg_behind_front_axle": None, "cg_height": None}
    meas = configured.get("measure") or {}
    cy = sum(r[3] * r[4][1] for r in rows) / m
    cz = sum(r[3] * r[4][2] for r in rows) / m
    yf, zg = meas.get("front_axle_y"), meas.get("ground_z")
    return {"mass": round(m + wheel_kg, 1),
            "cg_behind_front_axle": round(cy - yf, 4) if yf is not None else None,
            "cg_height": round(cz - zg, 4) if zg is not None else None}


def svj_mass(svj):
    """(mass kg, CG behind the front axle m, CG height m) of the SVJ; None where it has no value."""
    ch = svj.get("chassis") or {}
    cg = ch.get("center_of_gravity")
    ok = isinstance(cg, list) and len(cg) == 3
    return (ch.get("mass_total") if isinstance(ch.get("mass_total"), (int, float)) else None,
            round(-cg[0], 4) if ok else None, round(-cg[2], 4) if ok else None)


# the unsprung side and its mounts: their node weights and beams stay the base's when the mass changes
# (their loads come from the wheels, not from the body's own weight)
UNSPRUNG = re.compile(r"suspension|strut|coilover|shock|damper|spring|hub|arm|link|subframe|knuckle|upright|"
                      r"steering|tierod|wheel|tire|tyre|brake|swaybar|halfshaft|axle|differential|finaldrive", re.I)


def weight_changes(model, configured, mass=None, cg_y=None, cg_z=None, length_factors=None, floors=None):
    """New node weights for a target mass (kg, wheels included) and CG (behind the front axle, height;
    m): {part: {row: kg}}. None leaves that value as it is. length_factors ({node: factor},
    rigidity.length_factors): each weight first follows the length of its node's beams. The nodes of
    the suspension and running gear (UNSPRUNG) keep their weight: the mass and CG are reached with the
    body's nodes alone (all of them when those would be left with too little). floors ({node: kg},
    rigidity.mass_floors): no node goes below its own (the structure keeps the base's stiffness, so a
    node may get only as light as the physics step allows); the others take what they cannot."""
    lf = length_factors or {}
    rows, wheel_kg = node_weights(model, configured)
    rows = [(p, i, n, kg * lf.get(n, 1.0), pos) for p, i, n, kg, pos in rows]
    m = sum(r[3] for r in rows)
    if not rows or not m:
        return {}
    meas = configured.get("measure") or {}
    yf, zg = meas.get("front_axle_y") or 0.0, meas.get("ground_z") or 0.0
    total = (mass - wheel_kg) if mass is not None else m             # the target, without the wheels
    # the suspension's nodes and the body nodes its beams hold on to (the mounts) keep their weight
    geo = configured.get("geometry") or {}
    mounts = set()
    for (a, b), bp in zip(geo.get("beams") or [], geo.get("beam_parts") or []):
        if UNSPRUNG.search(bp):
            mounts.update((a, b))
    # the suspension and its mounts; if that leaves the body too little, the suspension alone; else all scale
    for fixed in ([UNSPRUNG.search(r[0]) is not None or r[2] in mounts for r in rows],
                  [UNSPRUNG.search(r[0]) is not None for r in rows], [False] * len(rows)):
        m_fix = sum(r[3] for r, f in zip(rows, fixed) if f)
        if total - m_fix >= 0.3 * total:
            break
    free = [(r, i) for i, (r, f) in enumerate(zip(rows, fixed)) if not f]
    w = [r[3] for r in rows]
    m_free = total - m_fix
    # the two ramps on the body's nodes, a few rounds: the height ramp nudges the CG along the car a
    # little, and back. Their weighted mean goes where the whole car's CG lands on the target.
    for axis, target, origin in ((1, cg_y, yf), (2, cg_z, zg)) * 4:
        if target is None:
            continue
        fix_moment = sum(w[i] * r[4][axis] for i, r in enumerate(rows) if fixed[i])
        want = ((origin + target) * total - fix_moment) / m_free
        mw = sum(w[i] for _, i in free)
        mean = sum(w[i] * r[4][axis] for r, i in free) / mw
        var = sum(w[i] * (r[4][axis] - mean) ** 2 for r, i in free) / mw
        if var <= 0:
            continue
        k = (want - mean) / var
        # w' = w (1 + k (p - mean)) moves the weighted mean by k var; every node keeps MIN_FACTOR of
        # its own (original) weight: k is shortened where a node would go below that
        t = 1.0
        for r, i in free:
            d = k * (r[4][axis] - mean)
            if d < 0 and w[i] * (1 + d) < MIN_FACTOR * r[3]:
                t = min(t, max(0.0, (MIN_FACTOR * r[3] / w[i] - 1) / d))
        for r, i in free:
            w[i] = w[i] * (1 + t * k * (r[4][axis] - mean))
    s_free = sum(w[i] for _, i in free)
    scale = m_free / s_free if s_free > 0 else 1.0
    for _, i in free:
        w[i] *= scale
    # the floors: a node below its own is raised to it, and the free nodes above theirs give the
    # difference back (a few rounds); the mass is not reached when all are at their floor
    fl = floors or {}
    for _ in range(8):
        lo = {i: fl.get(r[2], 0.0) for r, i in free}
        for i, f in lo.items():
            w[i] = max(w[i], f)
        extra = sum(w[i] for _, i in free) - m_free
        room = sum(w[i] - lo[i] for _, i in free if w[i] > lo[i])
        if extra <= 1e-6 or room <= 0:
            break
        k = min(1.0, extra / room)
        for _, i in free:
            if w[i] > lo[i]:
                w[i] -= (w[i] - lo[i]) * k
    out = {}
    for i, (part, row, _, _, _) in enumerate(rows):
        out.setdefault(part, {})[row] = round(w[i], 4)
    return out


def _set_inline(row, values):
    """Set beam properties on one table row, as its own inline properties (in place)."""
    if row and isinstance(row[-1], dict):
        row[-1].update(values)
    else:
        row.append(dict(values))


def _axle_load_ratio(model, configured, svj, take):
    """{axle: new static load / base static load}, from the mass and CG taken from the SVJ (1 where not)."""
    sm, sy, _ = svj_mass(svj)
    if not ((take.get("mass") and sm) or (take.get("cg_y") and sy is not None)):
        return {}
    base = mass_and_cg(model, configured)
    wb = (configured.get("measure") or {}).get("wheelbase")
    if not base.get("mass") or not wb or base.get("cg_behind_front_axle") is None:
        return {}
    m0, y0 = base["mass"], base["cg_behind_front_axle"]
    m1 = sm if take.get("mass") and sm else m0
    y1 = sy if take.get("cg_y") and sy is not None else y0
    f0, f1 = m0 * (1 - y0 / wb), m1 * (1 - y1 / wb)
    r0, r1 = m0 * y0 / wb, m1 * y1 / wb
    return {"front": f1 / f0 if f0 > 0 else 1.0, "rear": r1 / r0 if r0 > 0 else 1.0}


def apply(model, configured, svj, take, study=None, length_factors=None, floors=None):
    """What taking values changes: ({part: {row index: {property: value}}} for beams, {part: {"radius",
    "tireWidth", "scale"}} for tyres, {"$var": value} for the configuration, {part: {row index: kg}} for
    node weights). take: {row key: True}."""
    from beamforge import kinematics
    sv = svj_values(svj, study, kinematics.motion_ratios(model, configured)[0])
    beams, tyre, pcvars = {}, {}, {}
    vars_ = {x["name"]: x["value"] for x in configured["variables"] if isinstance(x["value"], (int, float))}
    load = _axle_load_ratio(model, configured, svj, take)
    sd = springs_and_dampers(model, configured)
    spring_k = {r["axle"]: _num_or(r["values"].get("beamSpring"), vars_, None) for r in sd if r["kind"] == "spring"}
    damp_floor = set()
    for r in sd:
        s = sv.get(r["axle"]) or {}
        new = {}
        if r["kind"] == "spring" and take.get(f"spring_{r['axle']}") and s.get("coil_rate"):
            new["beamSpring"] = s["coil_rate"]
            # the spring's preload (precompressionRange, metres it starts compressed) holds the car up at its
            # ride height: the same force over the new rate, times the axle's load change
            k0 = _num_or(r["values"].get("beamSpring"), vars_, None)
            pr = _num_or(r["values"].get("precompressionRange"), vars_, None)
            if k0 and pr:
                new["precompressionRange"] = round(pr * k0 / s["coil_rate"] * load.get(r["axle"], 1.0), 4)
        if r["kind"] == "damper":
            if take.get(f"damp_bump_{r['axle']}") and s.get("damp_bump"):
                new.update(beamDamp=s["damp_bump"], beamDampFast=s["damp_bump_fast"], beamDampVelocitySplit=s["damp_split"])
            if take.get(f"damp_rebound_{r['axle']}") and s.get("damp_rebound"):
                new.update(beamDampRebound=s["damp_rebound"], beamDampReboundFast=s["damp_rebound_fast"],
                           beamDampVelocitySplit=s["damp_split"])
            # the base vehicle is the reference: its damping ratio (damping over the critical damping,
            # 2 sqrt(k m)) is kept as a floor, the base dampers scaled by sqrt(new spring / old x new load /
            # old load) (the motion ratios are the same beams'); below it, the base's dampers so scaled
            if new:
                k_old = spring_k.get(r["axle"])
                k_new = (sv.get(r["axle"]) or {}).get("coil_rate") if take.get(f"spring_{r['axle']}") else None
                f = math.sqrt((k_new / k_old if k_new and k_old else 1.0) * load.get(r["axle"], 1.0))
                base = {k: _num_or(r["values"].get(k), vars_, None) for k in
                        ("beamDamp", "beamDampRebound", "beamDampFast", "beamDampReboundFast")}
                floor = {k: v * f for k, v in base.items() if v}
                if any(new.get(k, 0) < floor.get(k, 0) * 0.999 for k in ("beamDamp", "beamDampRebound") if k in new):
                    new = {k: round(v) for k, v in floor.items()}
                    if r["values"].get("beamDampVelocitySplit") is not None:
                        new["beamDampVelocitySplit"] = r["values"]["beamDampVelocitySplit"]
                    damp_floor.add(r["axle"])
        for k in list(new):                               # a tuning variable: set it in the configuration
            old = r["values"].get(k)
            if isinstance(old, str) and re.fullmatch(r"\$[A-Za-z_]\w*", old):
                pcvars[old] = new.pop(k)
        if new:
            beams.setdefault(r["part"], {})[r["row"]] = new
    for axle, t in tyres(model, json.loads(json.dumps(configured))).items():
        s = sv.get(axle) or {}
        if take.get(f"tyre_radius_{axle}") and s.get("tyre_radius") and t.get("radius"):
            k = s["tyre_radius"] / t["radius"]
            tyre[t["part"]] = {"radius": s["tyre_radius"], "scale": [1.0, k, k]}
    sm, sy, sz = svj_mass(svj)
    weights = {}
    if (take.get("mass") and sm) or (take.get("cg_y") and sy is not None) or (take.get("cg_z") and sz is not None):
        weights = weight_changes(model, configured, sm if take.get("mass") else None,
                                 sy if take.get("cg_y") else None, sz if take.get("cg_z") else None, length_factors,
                                 floors)
    return beams, tyre, pcvars, weights


def apply_to_part(name, part, beams, tyre, weights=None):
    """Write the taken values into a part (in place)."""
    nrows = part.get("nodes")
    for i, kg in ((weights or {}).get(name) or {}).items():
        if isinstance(nrows, list) and 0 < i < len(nrows) and isinstance(nrows[i], list):
            _set_inline(nrows[i], {"nodeWeight": kg})
    rows = part.get("beams")
    for i, values in (beams.get(name) or {}).items():
        if isinstance(rows, list) and 0 < i < len(rows) and isinstance(rows[i], list):
            _set_inline(rows[i], values)
    t = tyre.get(name)
    if t:
        for row in part.get("pressureWheels") or []:
            if isinstance(row, dict) and "radius" in row:
                row["radius"] = t["radius"]
        fb = part.get("flexbodies")
        if isinstance(fb, list) and fb and isinstance(fb[0], list):
            props = {}
            for row in fb[1:]:
                if isinstance(row, dict):
                    props.update(row)
                    continue
                if not isinstance(row, list) or not row or not isinstance(row[0], str) or "tire" not in row[0].lower():
                    continue
                inline = row[-1] if isinstance(row[-1], dict) else None
                sc = dict((inline or {}).get("scale", props.get("scale")) or {"x": 1, "y": 1, "z": 1})
                new = {a: round(float(sc.get(a, 1)) * t["scale"][j], 4) for j, a in enumerate("xyz")}
                if inline is None:
                    row.append({"scale": new})
                else:
                    inline["scale"] = new


# ---------------------------------------------------------------- powertrain

def _interp(table, x):
    """Linear interpolation in [[x, y], ...] (held flat beyond the ends)."""
    if not table:
        return 0.0
    if x <= table[0][0]:
        return table[0][1]
    for (x0, y0), (x1, y1) in zip(table, table[1:]):
        if x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0) if x1 > x0 else y1
    return table[-1][1]


def _rows(t):
    """[[rpm, value], ...] from a jbeam [["rpm", "torque"], [0, 0], ...] table."""
    return [list(r[:2]) for r in (t or [])[1:] if isinstance(r, list) and len(r) >= 2
            and all(isinstance(x, (int, float)) for x in r[:2])]


def powertrain_base(model, configured, driven=None):
    """The base vehicle's powertrain values and where they live: {"engine": part, "torque": [[rpm, Nm]],
    "exhaust_mod": [[rpm, Nm]], "idle_rpm", "max_rpm", "ecu": part, "rev_limiter", "gearbox": part,
    "ratios": [...], "differential": (part, row index) of the driven axle ("front" / "rear", else the
    first), "final_drive", "turbo": part or None, "automatic": bool}."""
    parts = beamng._parts_held(model)
    out = {"turbo": None}
    for name in _active(configured):
        p = (parts.get(name) or {}).get("part", {})
        me = p.get("mainEngine") if isinstance(p.get("mainEngine"), dict) else {}
        if me.get("torque"):
            out.update(engine=name, torque=_rows(me["torque"]), idle_rpm=me.get("idleRPM"), max_rpm=me.get("maxRPM"))
        if me.get("torqueModExhaust"):
            out["exhaust_mod"] = _rows(me["torqueModExhaust"])
        if "revLimiterRPM" in me:
            out.update(ecu=name, rev_limiter=me["revLimiterRPM"])
        if me.get("turbocharger") or me.get("supercharger"):
            out["turbo"] = name
        gb = p.get("gearbox") if isinstance(p.get("gearbox"), dict) else {}
        if isinstance(gb.get("gearRatios"), list):
            out.update(gearbox=name, ratios=gb["gearRatios"])
        pt = p.get("powertrain")
        if isinstance(pt, list) and pt and isinstance(pt[0], list):
            head = [str(h) for h in pt[0]]
            for i, row in enumerate(pt[1:], 1):
                if not isinstance(row, list) or not row:
                    continue
                rec = dict(zip(head, row))
                inline = row[-1] if isinstance(row[-1], dict) else {}
                if "automaticGearbox" in str(rec.get("type")) or "dctGearbox" in str(rec.get("type")):
                    out["automatic"] = True
                if rec.get("type") == "differential" and "gearRatio" in inline:
                    axle = "rear" if re.search(r"_R(\b|$)|rear", str(rec.get("name")), re.I) else \
                        "front" if re.search(r"_F(\b|$)|front", str(rec.get("name")), re.I) else None
                    if "differential" not in out or (driven and axle == driven):
                        out.update(differential=(name, i), final_drive=inline["gearRatio"], diff_axle=axle)
    # the final drive: every reduction between the gearbox and the driven wheels (a final drive part may
    # set it on the differential or on a torsion reactor before it)
    for ax in ((driven,) if driven in ("front", "rear") else ("rear", "front")):
        prod, chain = driveline(model, configured, ax)
        if prod is not None:
            out.update(final_drive=prod, final_chain=chain, diff_axle=ax)
            break
    return out


GEARBOX = re.compile(r"gearbox|torqueConverter|frictionClutch|viscousClutch|centrifugalClutch", re.I)


def driveline(model, configured, axle):
    """The gear reduction between the gearbox and a driven wheel of one axle ("front" / "rear"):
    (product of the gearRatio of every device on the way, [(part, key, ratio, device)]) or (None, []).
    A device's ratio is its powertrain row's gearRatio, overridden by a section of the same name in a
    later part (BeamNG's final drive parts: "torsionReactorF": {"gearRatio": 4.25}); key is
    ("powertrain", row) for a row, "<device>.gearRatio" for a section."""
    parts = beamng._parts_held(model)
    vars_ = {x["name"]: x["value"] for x in configured["variables"] if isinstance(x["value"], (int, float))}
    active = _active(configured)
    dev = {}
    for name in active:
        pt = (parts.get(name) or {}).get("part", {}).get("powertrain")
        if not (isinstance(pt, list) and pt and isinstance(pt[0], list)):
            continue
        head, props = [str(h) for h in pt[0]], {}
        for i, row in enumerate(pt[1:], 1):
            if isinstance(row, dict):
                props.update(row)
                continue
            if not isinstance(row, list) or not row:
                continue
            inline = row[-1] if isinstance(row[-1], dict) else {}
            rec = dict(props)
            rec.update(inline)
            rec.update(zip(head, row[:-1] if inline else row))
            if rec.get("name"):
                dev[rec["name"]] = {"type": str(rec.get("type", "")), "input": rec.get("inputName"),
                                    "ratio": rec.get("gearRatio"), "src": (name, ("powertrain", i)) if "gearRatio" in inline else None}
    for name in active:                                   # sections named after a device: later parts win
        for key, val in (parts.get(name) or {}).get("part", {}).items():
            if key in dev and isinstance(val, dict) and "gearRatio" in val:
                dev[key].update(ratio=val["gearRatio"], src=(name, f"{key}.gearRatio"))
    tag = "F" if axle == "front" else "R"
    start = next((n for n in sorted(dev) if re.match(rf"(wheelaxle|spindle){tag}[LR]", n)), None)
    chain, seen, n = [], set(), start
    while n and n in dev and n not in seen:
        seen.add(n)
        d = dev[n]
        if GEARBOX.search(d["type"]) or n == "gearbox":
            break
        r = _num_or(d["ratio"], vars_, 1.0) if d["ratio"] is not None else 1.0
        chain.append((d["src"][0] if d["src"] else None, d["src"][1] if d["src"] else None, r, n))
        n = d["input"]
    if not chain or n not in dev:
        return None, []
    prod = 1.0
    for c in chain:
        prod *= c[2]
    return round(prod, 4), chain


def svj_powertrain(svj):
    """{"layout", "torque": [[rpm, Nm]], "idle_rpm", "max_rpm", "ratios": [forward], "final_drive",
    "gearbox_type", "driven": "front" | "rear" | "both", "turbo": bool, "diff_type"} of the SVJ."""
    pt = svj.get("powertrain") or {}
    eng, gb = pt.get("engine") or {}, pt.get("gearbox") or {}
    layout = str(pt.get("layout") or "").upper()
    driven = "rear" if layout in ("FR", "MR", "RR") else "front" if layout in ("FF",) else "both" if layout else None
    diffs = pt.get("differentials") or []
    diff = next((d for d in diffs if d.get("location") == driven), diffs[0] if diffs else {})
    text = json.dumps(eng).lower()
    return {"layout": layout or None, "torque": [list(x) for x in eng.get("torque_curve") or [] if isinstance(x, list) and len(x) == 2],
            "idle_rpm": eng.get("idle_rpm"), "max_rpm": eng.get("max_rpm"), "ratios": gb.get("ratios"),
            "final_drive": diff.get("final_drive"), "gearbox_type": gb.get("type"), "driven": driven,
            "turbo": "turbo" in text or "supercharg" in text, "diff_type": diff.get("type")}


def _peak(torque):
    """(peak torque Nm, peak power kW) of a [[rpm, Nm]] curve."""
    if not torque:
        return None, None
    return max(t for _, t in torque), round(max(r * t for r, t in torque) * 2 * math.pi / 60 / 1000, 1)


def suggest_parts(model, configured_json, svj_json):
    """Parts of the base vehicle closer to the SVJ's powertrain: [{"slot", "current", "suggested",
    "why"}], from the alternatives of the intake, gearbox, transfer case and differential slots."""
    configured, svj = json.loads(configured_json), json.loads(svj_json)
    sp = svj_powertrain(svj)
    out = []

    def walk(n):
        opts = [o[0] for o in n.get("options") or []]
        cur, slot = n.get("part") or "", n.get("slot") or ""
        pick, why = None, ""
        text = (slot + " " + cur).lower()
        if "intake" in text and opts:
            want_turbo = sp["turbo"]
            same = [o for o in opts if (("turbo" in o or "supercharg" in o) == want_turbo)
                    and ("diesel" in o) == ("diesel" in cur)]
            if same and (("turbo" in cur or "supercharg" in cur) != want_turbo):
                pick, why = min(same, key=len), "naturally aspirated, as the SVJ" if not want_turbo else "forced induction, as the SVJ"
        elif "transmission" in text and opts and sp["gearbox_type"]:
            kind = {"manual": r"\d+M", "automatic": r"\d+A", "dct": r"DCT"}.get(str(sp["gearbox_type"]).lower())
            if kind and not re.search(kind + r"(_|$)", cur):
                same = [o for o in opts if re.search(kind + r"(_|$)", o) and ("diesel" in o) == ("diesel" in cur)]
                if same:
                    pick, why = min(same, key=len), f"{sp['gearbox_type']} gearbox, as the SVJ"
        elif "transfer" in text and opts and sp["driven"]:
            want = {"rear": "RWD", "front": "FWD", "both": "AWD"}[sp["driven"]]
            if want not in cur:
                same = [o for o in opts if want in o]
                if same:
                    pick, why = min(same, key=len), f"{sp['layout']} layout: {want}"
        elif ("differential" in slot.lower() and cur and sp["driven"] in ("rear", "front")
              and re.search(r"_F$" if sp["driven"] == "rear" else r"_R$", slot)):
            pick, why = "", f"{sp['driven']} wheels driven only: no {'front' if sp['driven'] == 'rear' else 'rear'} differential"
        elif "differential" in text and opts and sp["diff_type"] == "open" and re.search(r"lsd|locker|welded|bias|active", cur, re.I):
            same = [o for o in opts if not re.search(r"lsd|locker|welded|bias|active|race", o, re.I)]
            if same:
                pick, why = min(same, key=len), "open differential, as the SVJ"
        elif opts and (WHEEL_SIZE.search(cur) or TYRE_SIZE.search(cur)) and re.search(r"(^|_)(wheel|tire|tyre)", slot + " " + cur, re.I):
            pick, why = _closest_wheel(slot, cur, opts, svj)
        if pick is not None and pick != cur and (pick or why):
            out.append({"slot": slot, "current": cur, "suggested": pick, "why": why})
        for c in n.get("children", []):
            walk(c)
    walk(configured["tree"])
    return json.dumps(out)


def _closest_wheel(slot, cur, opts, svj):
    """The wheel (rim) or tyre option closest to the SVJ's size on that axle: (part, why) or (None, "")."""
    axle = "R" if re.search(r"(^|_)R(_|\d|$)|rear", slot + " " + cur) else "F"
    susp = svj.get("suspension") or {}
    corner = next((susp[k] for k in (("RL", "RR") if axle == "R" else ("FL", "FR")) if isinstance(susp.get(k), dict)), None)
    size = tyre_size(svj, corner) if corner else {}
    if not size.get("rim_in"):
        return None, ""
    side = [o for o in opts if re.search(rf"(^|_){axle}(_|$)", o) or not re.search(r"(^|_)[FR](_|$)", o)] or opts
    if TYRE_SIZE.search(cur):
        cands = [(o, TYRE_SIZE.search(o)) for o in side]
        cands = [(o, m) for o, m in cands if m and int(m.group(3)) == size["rim_in"]]
        if not cands:
            return None, ""
        w, a = (size.get("width") or 0.2) * 1000, (size.get("aspect") or 0.55) * 100
        gap = lambda m: abs(int(m.group(1)) - w) / 10 + abs(int(m.group(2)) - a) / 5  # noqa: E731
        best = min(cands, key=lambda c: (gap(c[1]), len(c[0])))
        mc = TYRE_SIZE.search(cur)
        if int(mc.group(3)) == size["rim_in"] and gap(mc) <= gap(best[1]):
            return None, ""                                # the current tyre is as close: kept (its compound too)
        m = best[1]
        return best[0], f"tyre {m.group(1)}/{m.group(2)} R{m.group(3)}, nearest to the SVJ's {w:.0f}/{a:.0f} R{size['rim_in']}"
    if int(WHEEL_SIZE.search(cur).group(1)) == size["rim_in"]:
        return None, ""                                    # the rim diameter already matches: the wheel stays
    cands = [(o, WHEEL_SIZE.search(o)) for o in side]
    cands = [(o, m) for o, m in cands if m and int(m.group(1)) == size["rim_in"]]
    if not cands:
        return None, ""
    target = (size.get("width") or 0.2) / 0.0254 - 1.0     # rim width, inches: about the tyre's less an inch
    same_family = re.sub(WHEEL_SIZE, "", cur)
    best = min(cands, key=lambda c: (abs(float(c[1].group(2)) - target), re.sub(WHEEL_SIZE, "", c[0]) != same_family, len(c[0])))
    return best[0], f"{best[1].group(1)}x{best[1].group(2)} wheel for the SVJ's R{size['rim_in']} tyre"


def powertrain_changes(model, configured, svj, take):
    """{part: {"mainEngine.torque" | "mainEngine.idleRPM" | "mainEngine.maxRPM" | "mainEngine.revLimiterRPM"
    | "gearbox.gearRatios" | ("powertrain", row): value}} for the taken powertrain values."""
    sp = svj_powertrain(svj)
    base = powertrain_base(model, configured, sp["driven"] if sp["driven"] in ("front", "rear") else None)
    ch = {}
    if take.get("engine") and sp["torque"] and base.get("engine"):
        mod = base.get("exhaust_mod") or []
        curve = sorted(sp["torque"])
        rows = [["rpm", "torque"], [0, 0]]
        idle = sp["idle_rpm"] or curve[0][0]
        if curve[0][0] > idle:                            # below the first point: towards zero at standstill
            rows.append([idle, round(curve[0][1] * idle / curve[0][0], 1)])
        rows += [[r, round(t - _interp(mod, r), 1)] for r, t in curve]   # the exhaust's change added back
        e = ch.setdefault(base["engine"], {})
        e["mainEngine.torque"] = rows
        if sp["idle_rpm"]:
            e["mainEngine.idleRPM"] = sp["idle_rpm"]
        if sp["max_rpm"]:
            e["mainEngine.maxRPM"] = sp["max_rpm"]
            if base.get("ecu"):
                ch.setdefault(base["ecu"], {})["mainEngine.revLimiterRPM"] = sp["max_rpm"]
    if take.get("gears") and sp["ratios"] and base.get("gearbox"):
        old = base["ratios"]
        lead = [r for r in old[:2] if isinstance(r, (int, float)) and r <= 0]   # reverse and neutral, as they were
        ch.setdefault(base["gearbox"], {})["gearbox.gearRatios"] = lead + list(sp["ratios"])
    if take.get("final_drive") and sp["final_drive"]:
        axles = ("front", "rear") if sp["driven"] == "both" else (sp["driven"],) if sp["driven"] else (base.get("diff_axle"),)
        for ax in axles:
            prod, chain = driveline(model, configured, ax) if ax else (None, [])
            cands = [c for c in chain if c[0]]
            if not prod or not cands:
                continue
            # the one reduction to change: a final drive part's, else the largest, else the differential
            pick = next((c for c in cands if re.search(r"final", c[0], re.I)), None) or                 max(cands, key=lambda c: (abs(c[2] - 1) > 1e-6, c[2]))
            others = prod / pick[2] if pick[2] else 1.0
            value = round(sp["final_drive"] / others, 4)
            key = pick[1] if isinstance(pick[1], str) else pick[1]
            ch.setdefault(pick[0], {})[key] = value if isinstance(key, str) else {"gearRatio": value}
    return ch


def apply_powertrain(name, part, changes):
    """Write powertrain changes into a part (in place)."""
    for key, value in (changes.get(name) or {}).items():
        if isinstance(key, tuple):
            table, row = key
            rows = part.get(table)
            if isinstance(rows, list) and 0 < row < len(rows) and isinstance(rows[row], list):
                _set_inline(rows[row], value)
            continue
        section, prop = key.split(".", 1)
        if isinstance(part.get(section), dict):
            part[section][prop] = value


# ---------------------------------------------------------------- steering

def steering_base(model, configured):
    """The base vehicle's steering hydros with a steeringWheelLock: ([(part, row index, degrees)], turns
    lock to lock). steeringWheelLock is the steering wheel's angle at full lock, each way."""
    parts = beamng._parts_held(model)
    rows = []
    for name in _active(configured):
        hy = (parts.get(name) or {}).get("part", {}).get("hydros")
        if not (isinstance(hy, list) and hy and isinstance(hy[0], list)):
            continue
        for i, row in enumerate(hy[1:], 1):
            if isinstance(row, list) and row and isinstance(row[-1], dict) and isinstance(row[-1].get("steeringWheelLock"), (int, float)):
                rows.append((name, i, row[-1]["steeringWheelLock"]))
    return rows, (round(2 * rows[0][2] / 360, 2) if rows else None)


def svj_steering(svj):
    """(turns lock to lock, overall ratio) of the SVJ steering; None where it has no value."""
    st = svj.get("steering") or {}
    turns = st.get("lock_to_lock_turns")
    if turns is None and isinstance(st.get("lock_to_lock"), (int, float)):
        turns = st["lock_to_lock"] / (2 * math.pi)       # radians of steering wheel, lock to lock
    return (round(turns, 2) if isinstance(turns, (int, float)) else None,
            st.get("overall_ratio") if isinstance(st.get("overall_ratio"), (int, float)) else None)


def steering_changes(model, configured, svj, take):
    """{part: {("hydros", row): {"steeringWheelLock": degrees}}} for the SVJ's turns lock to lock."""
    turns, _ = svj_steering(svj)
    if not take.get("steering") or not turns:
        return {}
    rows, _ = steering_base(model, configured)
    out = {}
    for part, i, _ in rows:
        out.setdefault(part, {})[("hydros", i)] = {"steeringWheelLock": round(turns * 180, 1)}
    return out


# ---------------------------------------------------------------- aerodynamics

def aero_triangles(model, configured):
    """The base vehicle's aero triangles: [(part, row index, dragCoef %, liftCoef %, area m2, |n . y|)]
    (the game reads both coefficients in percent, default 100, liftCoef defaulting to dragCoef)."""
    parts = beamng._parts_held(model)
    nodes = configured["geometry"]["nodes"]
    out = []
    for name in _active(configured):
        rows = (parts.get(name) or {}).get("part", {}).get("triangles")
        if not (isinstance(rows, list) and rows and isinstance(rows[0], list)):
            continue
        head = [str(h).rstrip(":") for h in rows[0]]
        props = {}
        for i, row in enumerate(rows[1:], 1):
            if isinstance(row, dict):
                props.update(row)
                continue
            if not isinstance(row, list):
                continue
            inline = row[-1] if row and isinstance(row[-1], dict) else {}
            rec = dict(props)
            rec.update(inline)
            rec.update(zip(head, row[:-1] if inline else row))
            ids = [rec.get(k) for k in ("id1", "id2", "id3")]
            if not all(n in nodes for n in ids):
                continue
            dc = _num_or(rec.get("dragCoef", 100), {}, 100.0)
            lc = _num_or(rec.get("liftCoef", dc), {}, dc)
            a, b, c = (nodes[n] for n in ids)
            u = [b[k] - a[k] for k in range(3)]
            w = [c[k] - a[k] for k in range(3)]
            n = [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]]
            area2 = math.sqrt(sum(x * x for x in n))
            if area2:
                out.append((name, i, dc, lc, area2 / 2, abs(n[1]) / area2))
    return out


def drag_area(tris):
    """The drag area (m2) of aero triangles in head-on flow, estimated as sum(coef A (n . y)^2): each
    triangle pushing back with the square of how squarely it faces the flow. An estimate (the game's
    own aero model is not public); it gives about 0.67 m2 for the RWD saloon, a saloon of Cd ~0.3."""
    return sum(dc / 100 * a * ny * ny for _, _, dc, _, a, ny in tris)


def svj_aero(svj):
    """(drag area m2, Cd, frontal area m2) of the SVJ: Cd x frontal area, or the components' drag
    contributions over the frontal area; None where it has no value."""
    a = svj.get("aerodynamics") or {}
    area = (a.get("reference") or {}).get("frontal_area")
    cd = (a.get("coefficients") or {}).get("Cd")
    comps = a.get("components")
    if cd is None and isinstance(comps, list):
        parts = [c.get("Cd_contribution") for c in comps if isinstance(c, dict) and isinstance(c.get("Cd_contribution"), (int, float))]
        cd = sum(parts) if parts else None
    if not isinstance(area, (int, float)) or not isinstance(cd, (int, float)):
        return None, cd, area
    return round(cd * area, 4), cd, area


def aero_changes(model, configured, svj, take):
    """{part: {("triangles", row): {"dragCoef", "liftCoef"}}}: every drag triangle scaled so the drag area
    estimate is the SVJ's, its lift coefficient written as it was (it would otherwise follow dragCoef)."""
    target, _, _ = svj_aero(svj)
    if not take.get("aero") or not target:
        return {}
    tris = aero_triangles(model, configured)
    base = drag_area(tris)
    if base <= 0:
        return {}
    k = target / base
    out = {}
    for part, row, dc, lc, _, _ in tris:
        if dc > 0:
            out.setdefault(part, {})[("triangles", row)] = {"dragCoef": round(dc * k, 3), "liftCoef": lc}
    return out
