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

A beam whose value is a tuning variable ("$spring_F") is not rewritten: the variable is set in the
new configuration instead, so the tuning menu keeps working.
"""

import json
import re

from beamforge import beamng, jbeam

DEFLECTION = 0.010      # m: unloaded minus loaded tyre radius, when the SVJ gives only the loaded one
# parts that hold a corner's spring or damper (not the hood, trunk, steering or mirror dampers)
SUSPENSION_PART = re.compile(r"strut|shock|spring|coilover|damper|suspension", re.I)


def _axle_of(y, yf, yr):
    return "front" if abs(y - yf) <= abs(y - yr) else "rear"


def _slopes(curve):
    """(slow slope, fast slope, split velocity) of a [velocity, force] curve; None when it has no slope."""
    pts = [p for p in curve or [] if isinstance(p, list) and len(p) == 2]
    pts.sort()
    if len(pts) < 2 or pts[1][0] <= pts[0][0]:
        return None
    slow = (pts[1][1] - pts[0][1]) / (pts[1][0] - pts[0][0])
    fast = (pts[2][1] - pts[1][1]) / (pts[2][0] - pts[1][0]) if len(pts) > 2 and pts[2][0] > pts[1][0] else slow
    return slow, fast, pts[1][0]


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
    return json.dumps([r for r in rows if r["base"] is not None or r["svj"] is not None])


def _set_inline(row, values):
    """Set beam properties on one table row, as its own inline properties (in place)."""
    if row and isinstance(row[-1], dict):
        row[-1].update(values)
    else:
        row.append(dict(values))


def apply(model, configured, svj, take, study=None):
    """What taking values changes: ({part: {row index: {property: value}}} for beams, {part: {"radius",
    "tireWidth", "scale"}} for tyres, {"$var": value} for the configuration). take: {row key: True}."""
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
    return beams, tyre, pcvars


def apply_to_part(name, part, beams, tyre):
    """Write the taken values into a part (in place)."""
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
