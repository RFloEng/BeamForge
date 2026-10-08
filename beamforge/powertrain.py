"""The Powertrain workspace: engine, gearbox, final drive and steering, from the base vehicle, the SVJ or by hand.

Pure Python, standard library only (runs in Pyodide). The values live in an SVJ-shaped block (powertrain.engine with its
torque curve, idle and maximum rpm; powertrain.gearbox.ratios; the driven differential's final_drive;
steering.lock_to_lock_turns), so what the user sets goes into the new vehicle through the same code as an SVJ's values
(values.powertrain_changes, values.steering_changes): the torque curve into the engine part (the exhaust's own change
added back), the ratios into the gearbox (reverse and neutral kept), the final drive into the driven axle's reduction,
the turns into the steering hydros' steeringWheelLock.

The edit (the editor's ptEdit, saved in the project): {"engine": {"torque": [[rpm, Nm]], "idle_rpm", "max_rpm", "inertia",
"friction", "engine_brake"} | None, "gears": [forward ratios] | None, "final_drive": float | None, "steering_turns": float |
None, "diffs": {"front" | "rear": {"type" (SVJ: open, locked, lsd_clutch, lsd_viscous...), "preload", "lock_power",
"lock_coast"}, "split": front share of an AWD's torque} | None}; None keeps the base's.
"""

import json
import re

from . import beamng, jbeam, values

# the slots of a vehicle's powertrain and steering, by name: the parts a user swaps to change the transmission
SLOT = re.compile(r"engine|transmission|gearbox|transaxle|differential|finaldrive|final_drive|transfer|driveshaft|halfshaft|"
                  r"intake|turbo|supercharger|exhaust|ecu|clutch|converter|steering", re.I)


def _num(v, vars_):
    if isinstance(v, (int, float)):
        return float(v)
    try:
        return jbeam.to_float(vars_.get(v, v) if isinstance(v, str) and v.startswith("$") and not v.startswith("$=") else v, vars_, None)
    except (jbeam.UnresolvedValue, TypeError, ValueError):
        return None


def forward(ratios, vars_=None):
    """The forward gears of a gearbox's gearRatios list ([reverse, neutral, 1st, ...]: everything after the neutral 0),
    as numbers (tuning variables resolved)."""
    vals = [_num(r, vars_ or {}) for r in ratios or []]
    k = next((i for i, r in enumerate(vals[:3]) if r == 0), None)
    return [r for r in (vals[k + 1:] if k is not None else vals) if r]


def _slots(configured):
    out = []

    def walk(n, depth):
        if n.get("slot") and SLOT.search(str(n.get("slot"))) and (n.get("options") or n.get("part")):
            out.append({"slot": n["slot"], "description": n.get("description") or n["slot"], "part": n.get("part"),
                        "core": bool(n.get("core")), "options": n.get("options") or [[n.get("part"), n.get("title")]], "depth": depth})
        for c in n.get("children", []):
            walk(c, depth + 1)
    walk(configured["tree"], 0)
    return out


def summary_json(model, configured_json, svj_json=None):
    """For the panel: {"base": {"torque" (with the exhaust's change), "idle_rpm", "max_rpm", "ratios", "final_drive",
    "turns", "turbo", "automatic", "driven"} | None, "svj": {the same, "layout", "gearbox_type"} | None, "tyre_radius"
    (m, of the driven axle), "slots": [{"slot", "description", "part", "options", "core", "depth"}]}."""
    out = {"base": None, "svj": None, "tyre_radius": None, "slots": []}
    svj = json.loads(svj_json) if svj_json else None
    sp = values.svj_powertrain(svj) if svj else None
    if sp:
        turns, _ = values.svj_steering(svj)
        out["svj"] = {"torque": sp["torque"], "idle_rpm": sp["idle_rpm"], "max_rpm": sp["max_rpm"], "inertia": sp["inertia"],
                      "diffs": {k: v for k, v in sp["diffs"].items() if k in ("front", "rear")}, "split": sp["split"],
                      "ratios": [r for r in sp["ratios"] or [] if isinstance(r, (int, float)) and r > 0],
                      "final_drive": sp["final_drive"], "turns": turns, "layout": sp["layout"], "driven": sp["driven"],
                      "gearbox_type": sp["gearbox_type"], "turbo": sp["turbo"]}
    if model and configured_json:
        configured = json.loads(configured_json)
        vars_ = {x["name"]: x["value"] for x in configured.get("variables") or []}
        pb = values.powertrain_base(model, configured)
        mod = pb.get("exhaust_mod") or []
        _, turns = values.steering_base(model, configured)
        parts = beamng._parts_held(model)
        me = ((parts.get(pb.get("engine")) or {}).get("part") or {}).get("mainEngine") or {}
        diffs = {}
        for ax in ("front", "rear"):
            row = values.diff_row(model, configured, ax)
            if row:
                r = (parts[row[0]]["part"]["powertrain"])[row[1]]
                inl = r[-1] if isinstance(r[-1], dict) else {}
                diffs[ax] = {"type": inl.get("diffType"), "preload": inl.get("lsdPreload"), "lock_power": inl.get("lsdLockCoef"),
                             "lock_coast": inl.get("lsdRevLockCoef")}
        out["base"] = {"inertia": _num(me.get("inertia"), vars_), "friction": _num(me.get("friction"), vars_),
                       "engine_brake": _num(me.get("engineBrakeTorque"), vars_), "diffs": diffs,
                       "torque": [[r, round(t + values._interp(mod, r), 1)] for r, t in pb.get("torque") or []],
                       "idle_rpm": _num(pb.get("idle_rpm"), vars_), "max_rpm": _num(pb.get("rev_limiter") or pb.get("max_rpm"), vars_),
                       "ratios": forward(pb.get("ratios"), vars_), "final_drive": _num(pb.get("final_drive"), vars_), "turns": turns,
                       "turbo": bool(pb.get("turbo")), "automatic": bool(pb.get("automatic")), "driven": pb.get("diff_axle")}
        ty = values.tyres(model, configured)
        axle = (out["svj"] or {}).get("driven") if (out["svj"] or {}).get("driven") in ("front", "rear") else pb.get("diff_axle")
        out["tyre_radius"] = (ty.get(axle or "rear") or ty.get("front") or ty.get("rear") or {}).get("radius")
        out["slots"] = _slots(configured)
    return json.dumps(out)


def export_json(edit_json, svj_json=None):
    """The block for the export: {"svj": the SVJ (or a bare document) with the edited powertrain and steering, "take":
    {"engine", "gears", "final_drive", "steering": True where the edit sets it, False where it keeps the base's}}."""
    edit = json.loads(edit_json or "{}")
    svj = json.loads(svj_json) if svj_json else {}
    doc = json.loads(json.dumps(svj))
    pt = doc.setdefault("powertrain", {})
    take = {"engine": False, "gears": False, "final_drive": False, "steering": False, "diffs": False}
    if edit.get("engine") and edit["engine"].get("torque"):
        e = pt.setdefault("engine", {})
        e["torque_curve"] = [[float(r), float(t)] for r, t in edit["engine"]["torque"]]
        for k in ("idle_rpm", "max_rpm"):
            if edit["engine"].get(k):
                e[k] = float(edit["engine"][k])
        take["engine"] = True
    if edit.get("gears"):
        pt.setdefault("gearbox", {})["ratios"] = [float(r) for r in edit["gears"]]
        take["gears"] = True
    if edit.get("engine") and take["engine"]:
        e = pt["engine"]
        if edit["engine"].get("inertia"):
            e["inertia"] = float(edit["engine"]["inertia"])
        x = {k: float(edit["engine"][src]) for src, k in (("friction", "friction"), ("engine_brake", "engineBrakeTorque"))
             if isinstance(edit["engine"].get(src), (int, float))}
        if x:
            e["x_beamng"] = {**(e.get("x_beamng") or {}), **x}
    if edit.get("diffs"):
        diffs = pt.setdefault("differentials", [])
        for loc in ("front", "rear"):
            d = (edit["diffs"] or {}).get(loc)
            if not d or not d.get("type"):
                continue
            row = next((x for x in diffs if x.get("location") == loc), None)
            if row is None:
                row = {"id": f"diff_{loc}", "location": loc}
                diffs.append(row)
            row.update({k: d[k] for k in ("type", "preload", "lock_power", "lock_coast") if d.get(k) is not None})
        if isinstance(edit["diffs"].get("split"), (int, float)):
            pt.setdefault("transfer_case", {})["torque_split"] = [edit["diffs"]["split"], round(1 - edit["diffs"]["split"], 4)]
        take["diffs"] = True
    if edit.get("final_drive"):
        diffs = pt.setdefault("differentials", [])
        sp = values.svj_powertrain(doc)
        loc = sp["driven"] if sp["driven"] in ("front", "rear") else None
        d = next((x for x in diffs if x.get("location") == loc), None) if loc else (diffs[0] if diffs else None)
        if d is None:
            d = {"location": loc} if loc else {}
            diffs.append(d)
        d["final_drive"] = float(edit["final_drive"])
        take["final_drive"] = True
    if edit.get("steering_turns"):
        doc.setdefault("steering", {})["lock_to_lock_turns"] = float(edit["steering_turns"])
        doc["steering"].pop("lock_to_lock", None)
        take["steering"] = True
    return json.dumps({"svj": doc, "take": take})


def from_svj_json(svj_json):
    """The edit that takes every powertrain and steering value the SVJ has (what Import from the SVJ sets)."""
    s = json.loads(summary_json(None, None, svj_json))["svj"] or {}
    sp = values.svj_powertrain(json.loads(svj_json))
    diffs = {loc: {k: d.get(k) for k in ("type", "preload", "lock_power", "lock_coast") if d.get(k) is not None}
             for loc, d in sp["diffs"].items() if loc in ("front", "rear") and d.get("type")}
    if sp["split"] is not None:
        diffs["split"] = sp["split"]
    eng = {"torque": s["torque"], "idle_rpm": s["idle_rpm"], "max_rpm": s["max_rpm"]} if s.get("torque") else None
    if eng and sp["inertia"]:
        eng["inertia"] = sp["inertia"]
    return json.dumps({"engine": eng, "gears": s.get("ratios") or None, "final_drive": s.get("final_drive"),
                       "steering_turns": s.get("turns"), "diffs": diffs or None, "gearbox_type": s.get("gearbox_type"),
                       "layout": s.get("layout")})
