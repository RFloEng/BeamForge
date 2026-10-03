"""Values to take from an SVJ onto the base vehicle: springs, dampers, tyres (roadmap step 3).

Pure Python, standard library only (runs in Pyodide). Each value is read on both sides, the base
vehicle (its active parts, as configured) and the SVJ, per axle, and written into the new vehicle by
export.build when the user takes it.

  springs   SVJ spring.rate is the wheel rate (SVJ spec); the coil rate is wheel rate / MR^2, MR the
            spring's motion ratio (spring.motion_ratio, else the SVJ corner's static motion ratio from
            the suspension study, else 1). BeamNG's coil spring is a |NORMAL beam with a
            precompressionRange between the hub and the body; its beamSpring (N/m) takes the coil rate.
  dampers   SVJ bump_curve / rebound_curve are [velocity m/s, force N]. BeamNG's damper is a |BOUNDED
            beam: beamDamp / beamDampRebound take the slope of the first segment (N/(m/s)),
            beamDampFast / beamDampReboundFast the slope after it, beamDampVelocitySplit its end.
            Velocities are taken as at the damper (the SVJ damper motion_ratio, if given, converts
            wheel velocities to the damper: force slopes are divided by MR^2).
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


def svj_values(svj, study=None):
    """Per axle ("front", "rear"): {"wheel_rate", "spring_mr", "coil_rate", "damp_bump", "damp_bump_fast",
    "damp_rebound", "damp_rebound_fast", "damp_split", "tyre_radius", "tyre_width"} from the SVJ's FL / RL
    corners (FR / RR when the left one is missing); None where the file has no value. study: the
    suspension study (suspension.study_svj) for motion ratios the file does not give."""
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
        v = {"wheel_rate": rate, "spring_mr": round(mr, 3), "coil_rate": round(rate / (mr * mr)) if rate else None}
        dmr = damper.get("motion_ratio") or 1.0
        for key, curve in (("bump", damper.get("bump_curve")), ("rebound", damper.get("rebound_curve"))):
            s = _slopes(curve)
            v[f"damp_{key}"] = round(s[0] / (dmr * dmr)) if s else None
            v[f"damp_{key}_fast"] = round(s[1] / (dmr * dmr)) if s else None
            v["damp_split"] = round(s[2], 3) if s else v.get("damp_split")
        tire = c.get("tire") or {}
        unloaded = tire.get("unloaded_radius") or wheel.get("unloaded_radius")
        loaded = tire.get("loaded_radius") or wheel.get("loaded_radius")
        v["tyre_radius"] = round(unloaded, 4) if unloaded else (round(loaded + DEFLECTION, 4) if loaded else None)
        v["tyre_width"] = tire.get("width") or tire.get("section_width") or wheel.get("tire_width")
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
            if "NORMAL" in bt and "precompressionRange" in rec and rec.get("beamSpring") not in (0, None):
                kind = "spring"
            elif "BOUNDED" in bt and rec.get("beamDampRebound") is not None:
                kind = "damper"
            if not kind:
                continue
            y = (nodes[a][1] + nodes[b][1]) / 2
            keys = ("beamSpring",) if kind == "spring" else ("beamDamp", "beamDampRebound", "beamDampFast", "beamDampReboundFast", "beamDampVelocitySplit")
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
    sv = svj_values(svj, json.loads(study_json) if study_json else None)
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


def weight_changes(model, configured, mass=None, cg_y=None, cg_z=None, length_factors=None):
    """New node weights for a target mass (kg, wheels included) and CG (behind the front axle, height;
    m): {part: {row: kg}}. None leaves that value as it is. length_factors ({node: factor},
    rigidity.length_factors): each weight first follows the length of its node's beams."""
    lf = length_factors or {}
    rows, wheel_kg = node_weights(model, configured)
    rows = [(p, i, n, kg * lf.get(n, 1.0), pos) for p, i, n, kg, pos in rows]
    m = sum(r[3] for r in rows)
    if not rows or not m:
        return {}
    meas = configured.get("measure") or {}
    yf, zg = meas.get("front_axle_y") or 0.0, meas.get("ground_z") or 0.0
    w = [r[3] for r in rows]
    # the two ramps, a few rounds: the height ramp nudges the CG along the car a little, and back
    for axis, target, origin in ((1, cg_y, yf), (2, cg_z, zg)) * 4:
        if target is None:
            continue
        mw = sum(w)
        mean = sum(wi * r[4][axis] for wi, r in zip(w, rows)) / mw
        var = sum(wi * (r[4][axis] - mean) ** 2 for wi, r in zip(w, rows)) / mw
        if var <= 0:
            continue
        k = (origin + target - mean) / var
        # w' = w (1 + k (p - mean)) moves the weighted mean by k var; every node keeps MIN_FACTOR of
        # its own (original) weight: k is shortened where a node would go below that
        t = 1.0
        for wi, r in zip(w, rows):
            d = k * (r[4][axis] - mean)
            if d < 0 and wi * (1 + d) < MIN_FACTOR * r[3]:
                t = min(t, max(0.0, (MIN_FACTOR * r[3] / wi - 1) / d))
        w = [wi * (1 + t * k * (r[4][axis] - mean)) for wi, r in zip(w, rows)]
    scale = ((mass - wheel_kg) / sum(w)) if mass is not None and sum(w) > 0 else m / sum(w)
    out = {}
    for (part, row, _, _, _), wi in zip(rows, w):
        out.setdefault(part, {})[row] = round(wi * scale, 4)
    return out


def _set_inline(row, values):
    """Set beam properties on one table row, as its own inline properties (in place)."""
    if row and isinstance(row[-1], dict):
        row[-1].update(values)
    else:
        row.append(dict(values))


def apply(model, configured, svj, take, study=None, length_factors=None):
    """What taking values changes: ({part: {row index: {property: value}}} for beams, {part: {"radius",
    "tireWidth", "scale"}} for tyres, {"$var": value} for the configuration, {part: {row index: kg}} for
    node weights). take: {row key: True}."""
    sv = svj_values(svj, study)
    beams, tyre, pcvars = {}, {}, {}
    for r in springs_and_dampers(model, configured):
        s = sv.get(r["axle"]) or {}
        new = {}
        if r["kind"] == "spring" and take.get(f"spring_{r['axle']}") and s.get("coil_rate"):
            new["beamSpring"] = s["coil_rate"]
        if r["kind"] == "damper":
            if take.get(f"damp_bump_{r['axle']}") and s.get("damp_bump"):
                new.update(beamDamp=s["damp_bump"], beamDampFast=s["damp_bump_fast"], beamDampVelocitySplit=s["damp_split"])
            if take.get(f"damp_rebound_{r['axle']}") and s.get("damp_rebound"):
                new.update(beamDampRebound=s["damp_rebound"], beamDampReboundFast=s["damp_rebound_fast"],
                           beamDampVelocitySplit=s["damp_split"])
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
                                 sy if take.get("cg_y") else None, sz if take.get("cg_z") else None, length_factors)
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
    return out


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
        elif "differential" in text and opts and sp["diff_type"] == "open" and re.search(r"lsd|locker|welded|bias|active", cur, re.I):
            same = [o for o in opts if not re.search(r"lsd|locker|welded|bias|active|race", o, re.I)]
            if same:
                pick, why = min(same, key=len), "open differential, as the SVJ"
        if pick and pick != cur:
            out.append({"slot": slot, "current": cur, "suggested": pick, "why": why})
        for c in n.get("children", []):
            walk(c)
    walk(configured["tree"])
    return json.dumps(out)


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
    if take.get("final_drive") and sp["final_drive"] and base.get("differential"):
        part, row = base["differential"]
        ch.setdefault(part, {})[("powertrain", row)] = {"gearRatio": sp["final_drive"]}
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
