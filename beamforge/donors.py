"""What the vanilla cars are made of: every engine, gearbox, tyre and rim of the held vehicles, and archetypes of them.

Pure Python, standard library only (runs in Pyodide). Nothing from the game is kept in BeamForge: the catalogue is read
from the user's own install (beamng.add_files) each time, and an archetype is a pointer to a real part of it (model and
part name) with the numbers that describe it. A car made from scratch (scratch.py) takes the data blocks of the parts
its archetypes point at: an engine's mainEngine block and sounds, a gearbox's ratios and type, a tyre's pressureWheels
properties, so its engine, gearbox and wheels behave as the vanilla ones the game's developers tuned.

catalogue()   {"engines": [...], "gearboxes": [...], "tyres": [...], "rims": [...]}, every part of every held vehicle
archetypes()  the same, grouped: engines by kind, fuel and cylinders (from the sound sample) into power classes,
              gearboxes by type and number of gears, tyres by rim size and use, rims by diameter and width; each group
              one representative part (the median of its group)
"""

import json
import math
import re

from . import beamng, jbeam

GEARBOX_TYPES = {"manualGearbox": "manual", "sequentialGearbox": "sequential", "dctGearbox": "dct",
                 "automaticGearbox": "automatic", "cvtGearbox": "cvt"}
TYRE_NAME = re.compile(r"tire_[a-z]{1,2}_(\d+(?:\.\d+)?)_(\d+(?:\.\d+)?)_(\d+)((?:_[a-z0-9]+)*)", re.I)   # tire_F_176_68_13_standard, tire_RR_30_11_15_alt_drag
TYRE_NAME_LETTER = re.compile(r"tire_[a-z]{1,2}_[a-z]\d{2}_(\d{2})((?:_[a-z0-9]+)*)$", re.I)        # tire_F_E70_14: the old letter code
TYRE_NAME_SHORT = re.compile(r"tire_[a-z]{1,2}_(\d+)_(\d{2})((?:_[a-z0-9]+)*)$", re.I)          # tire_RR_165_13_standard, tire_RR_560_13 (no aspect)
TYRE_USES = ("standard", "sport", "race", "rally", "offroad", "mud", "drag", "eco", "old", "desert", "crawler", "dually",
             "stretched", "asphalt", "drift", "gravel", "snow", "winter", "allterrain", "biasply", "track", "slick")
RIM_NAME = re.compile(r"(?:^|_)(\d{2})x(\d+(?:\.\d+)?)(?:_|$)")                    # wheel_02a_15x7_F
CYLINDERS = re.compile(r"(?:^|[^a-z])(i3|i4|i5|i6|v6|v8|v10|v12|b4|b6|h4|h6|f4|f6|w12|rotary)(?=[^0-9]|$)", re.I)   # B4: boxer
TYRE_KEYS = ("radius", "tireWidth", "hubRadius", "frictionCoef", "slidingFrictionCoef", "noLoadCoef", "fullLoadCoef",
             "loadSensitivitySlope", "softnessCoef", "treadCoef", "pressurePSI", "numRays", "nodeWeight", "hubNodeWeight")


def _vars(part):
    """The defaults of a part's tuning variables: {"$name": value}."""
    out = {}
    t = part.get("variables")
    if isinstance(t, list) and t and isinstance(t[0], list):
        head = [str(h) for h in t[0]]
        for row in t[1:]:
            if isinstance(row, list) and len(row) == len(head):
                rec = dict(zip(head, row))
                if isinstance(rec.get("name"), str):
                    out[rec["name"]] = rec.get("default")
    return out


def _num(v, vars_):
    if isinstance(v, (int, float)):
        return float(v)
    try:
        return jbeam.to_float(vars_.get(v, v) if isinstance(v, str) and v.startswith("$") and not v.startswith("$=") else v, vars_, None)
    except (jbeam.UnresolvedValue, TypeError, ValueError, ZeroDivisionError):
        return None


def _rows(table, vars_):
    out = []
    for r in (table or [])[1:]:
        if isinstance(r, list) and len(r) >= 2:
            a, b = _num(r[0], vars_), _num(r[1], vars_)
            if a is not None and b is not None:
                out.append([a, b])
    return out


def _powertrain_rows(part):
    t = part.get("powertrain")
    if not (isinstance(t, list) and t and isinstance(t[0], list)):
        return []
    head = [str(h) for h in t[0]]
    return [dict(zip(head, r), _inline=r[-1] if isinstance(r[-1], dict) else {}) for r in t[1:] if isinstance(r, list) and r]


def _merged(t):
    """The property dicts of a jbeam table (pressureWheels) merged in order: the values in force after its rows."""
    out = {}
    for r in (t or [])[1:]:
        if isinstance(r, dict):
            out.update(r)
    return out


def _models():
    return [m for m in json.loads(beamng.held_models()) if m != "common"]


def _own_parts(model):
    return {k: v for k, v in beamng._parts_held(model).items() if v["model"] == model}


def _peak_power(curve, max_rpm):
    best = 0.0
    for r, t in curve:
        if max_rpm and r > max_rpm * 1.02:
            continue
        best = max(best, t * r * 2 * math.pi / 60 / 1000)
    return round(best, 1)


def engines(model, parts):
    out = []
    for name, rec in parts.items():
        p = rec["part"]
        me = p.get("mainEngine")
        rows = _powertrain_rows(p)
        vars_ = _vars(p)
        if isinstance(me, dict) and me.get("torque"):
            curve = _rows(me["torque"], vars_)
            if len(curve) < 3:
                continue
            idle, mx = _num(me.get("idleRPM"), vars_), _num(me.get("maxRPM"), vars_)
            sample = str((p.get("soundConfig") or {}).get("sampleName") or "") if isinstance(p.get("soundConfig"), dict) else ""
            cyl = CYLINDERS.search(sample + " " + name)
            fuel = str(me.get("requiredEnergyType") or "gasoline")
            out.append({"model": model, "part": name, "title": (p.get("information") or {}).get("name") or name, "kind": "combustion",
                        "fuel": fuel, "cylinders": cyl.group(1).upper() if cyl else "?", "sound": sample,
                        "peak_torque": round(max(t for _, t in curve), 1), "peak_power": _peak_power(curve, mx),
                        "idle_rpm": idle, "max_rpm": mx, "inertia": _num(me.get("inertia"), vars_),
                        "torque": [[round(r), round(t, 1)] for r, t in curve], "slot": p.get("slotType")})
        for r in rows:                                         # electric motors: a device with its own data block
            if r.get("type") == "electricMotor" and isinstance(p.get(r.get("name")), dict):
                d = p[r["name"]]
                curve = _rows(d.get("torque"), vars_)
                mx = _num(d.get("maxRPM"), vars_)
                if len(curve) < 3 or not mx or mx < 3000:          # a mixer drum or a prop's motor, not a car's
                    continue
                out.append({"model": model, "part": name, "device": r["name"], "title": (p.get("information") or {}).get("name") or name,
                            "kind": "electric", "fuel": "electricEnergy", "cylinders": "motor", "sound": "",
                            "peak_torque": round(max(t for _, t in curve), 1), "peak_power": _peak_power(curve, mx),
                            "idle_rpm": 0.0, "max_rpm": mx, "inertia": _num(d.get("inertia"), vars_),
                            "torque": [[round(x), round(t, 1)] for x, t in curve], "slot": p.get("slotType")})
    return out


def gearboxes(model, parts):
    out = []
    for name, rec in parts.items():
        p = rec["part"]
        vars_ = _vars(p)
        for r in _powertrain_rows(p):
            kind = GEARBOX_TYPES.get(r.get("type"))
            d = p.get(r.get("name"))
            if not kind or not isinstance(d, dict) or not isinstance(d.get("gearRatios"), list):
                continue
            ratios = [_num(x, vars_) for x in d["gearRatios"]]
            k = next((i for i, x in enumerate(ratios[:3]) if x == 0), None)
            fwd = [x for x in (ratios[k + 1:] if k is not None else ratios) if x]
            if not fwd:
                continue
            out.append({"model": model, "part": name, "device": r["name"], "title": (p.get("information") or {}).get("name") or name,
                        "type": kind, "gears": len(fwd), "ratios": [round(x, 3) for x in fwd],
                        "reverse": round(ratios[0], 3) if k == 1 and ratios[0] is not None else None,
                        "spread": round(fwd[0] / fwd[-1], 2) if fwd[-1] else None, "slot": p.get("slotType")})
    return out


def tyres(model, parts):
    out = []
    for name, rec in parts.items():
        p = rec["part"]
        pw = p.get("pressureWheels")
        if not (isinstance(pw, list) and pw):
            continue
        m = _merged(pw)
        if not m.get("hasTire") or "radius" not in m:
            continue
        vars_ = _vars(p)
        vals = {k: _num(m[k], vars_) for k in TYRE_KEYS if k in m}
        size, short = TYRE_NAME.search(name), None
        if size:
            a, b, rim, rest = float(size.group(1)), float(size.group(2)), int(size.group(3)), size.group(4)
        else:
            short = TYRE_NAME_SHORT.search(name)
            letter = TYRE_NAME_LETTER.search(name) if not short else None
            a, b, rim, rest = (float(short.group(1)), None, int(short.group(2)), short.group(3)) if short else                 (None, None, int(letter.group(1)), letter.group(2)) if letter else (None, None, None, "")
        inch = a is not None and a <= 45 and b is not None   # 30_11_15: 29 in tall, 10 in wide on a 15 in rim
        if short and a >= 400:                                # 560_13: the old 5.60-13 code, a width of 5.6 in
            a = round(a / 100 * 25.4)
        words = [w.lower() for w in rest.split("_") if w]
        use = next((w for w in words if w in TYRE_USES), "standard")
        rec_ = {"model": model, "part": name, "title": (p.get("information") or {}).get("name") or name,
                "axle": "front" if re.search(r"_F(_|$)", name) else "rear" if re.search(r"_R{1,2}(_|$)", name) else None,
                "width_mm": (round(b * 25.4) if inch else int(a)) if a is not None else None,
                "aspect": None if inch or b is None else int(b), "diameter_in": a if inch else None,
                "rim_in": rim, "use": use, "slot": p.get("slotType")}
        rec_.update({k: (round(v, 4) if isinstance(v, float) else v) for k, v in vals.items()})
        out.append(rec_)
    return out


def rims(model, parts):
    out = []
    for name, rec in parts.items():
        p = rec["part"]
        pw = p.get("pressureWheels")
        if not (isinstance(pw, list) and pw) or not re.search(r"wheel|rim|steelrim", name, re.I) or re.search(r"tire|tyre|hubcap|data", name, re.I):
            continue
        m = _merged(pw)
        if "hubRadius" not in m:
            continue
        vars_ = _vars(p)
        size = RIM_NAME.search(name)
        tslot = None
        t = p.get("slots") or p.get("slots2")
        if isinstance(t, list) and t and isinstance(t[0], list):
            head = [str(h) for h in t[0]]
            tslot = next((dict(zip(head, r)).get("type") or dict(zip(head, r)).get("name") for r in t[1:]
                          if isinstance(r, list) and str(dict(zip(head, r)).get("type") or dict(zip(head, r)).get("name") or "").startswith("tire")), None)
        out.append({"model": model, "part": name, "title": (p.get("information") or {}).get("name") or name, "tyre_slot": tslot,
                    "diameter_in": int(size.group(1)) if size else None, "width_in": float(size.group(2)) if size else None,
                    "hubRadius": _num(m.get("hubRadius"), vars_), "hubWidth": _num(m.get("hubWidth"), vars_),
                    "slot": p.get("slotType")})
    return out


def catalogue(models=None):
    """Every engine, electric motor, gearbox, tyre and rim of the held vehicles (and of vehicles/common, once); each
    entry with the file it is in ("file"), to read it again when a car is made from it."""
    out = {"engines": [], "gearboxes": [], "tyres": [], "rims": []}
    seen = set()
    for model in (models or _models()) + ["common"]:
        parts = _own_parts(model) if model != "common" else {k: v for k, v in beamng._parts_held("common").items() if v["model"] == "common"}
        parts = {k: v for k, v in parts.items() if (v["model"], k) not in seen}
        seen.update((v["model"], k) for k, v in parts.items())
        for kind, f in (("engines", engines), ("gearboxes", gearboxes), ("tyres", tyres), ("rims", rims)):
            for e in f(model, parts):
                e["file"] = parts[e["part"]]["file"]
                out[kind].append(e)
    return out


def _median(items, key):
    s = sorted(items, key=key)
    return s[len(s) // 2]


POWER_CLASSES = ((0, 75, "small"), (75, 150, "medium"), (150, 250, "strong"), (250, 400, "high"), (400, 1e9, "very high"))


def archetypes(cat=None):
    """Representative parts: {"engines": [{"name", "count", "members", "pick": catalogue entry}], "gearboxes": [...],
    "tyres": [...], "rims": [...]}, named as a person would say it ("I4 petrol, medium (75-150 kW)", "6-speed manual",
    "15 in sport", "15x7 rim")."""
    cat = cat or catalogue()
    out = {"engines": [], "gearboxes": [], "tyres": [], "rims": []}
    groups = {}
    for e in cat["engines"]:
        cls = next(c for c in POWER_CLASSES if c[0] <= e["peak_power"] < c[1])
        fuel = {"gasoline": "petrol", "diesel": "diesel", "electricEnergy": "electric"}.get(e["fuel"], e["fuel"])
        key = f"{e['cylinders'] if e['kind'] == 'combustion' else 'Electric motor'} {fuel if e['kind'] == 'combustion' else ''}".strip() + f", {cls[2]} ({cls[0]:g}-{cls[1]:g} kW)".replace("-1e+09", "+")
        groups.setdefault(("engines", key), []).append(e)
    for g in cat["gearboxes"]:
        groups.setdefault(("gearboxes", f"{g['gears']}-speed {g['type']}"), []).append(g)
    for t in cat["tyres"]:
        if t["rim_in"]:
            groups.setdefault(("tyres", f"{t['rim_in']} in {t['use']}"), []).append(t)
    for r in cat["rims"]:
        if r["diameter_in"]:
            groups.setdefault(("rims", f"{r['diameter_in']}x{r['width_in']:g} rim"), []).append(r)
    order = {"engines": lambda x: x["peak_power"], "gearboxes": lambda x: x["ratios"][0], "tyres": lambda x: x.get("radius") or 0,
             "rims": lambda x: x.get("hubRadius") or 0}
    for (kind, name), members in groups.items():
        pick = _median(members, order[kind])
        out[kind].append({"name": name, "count": len(members), "models": sorted({m["model"] for m in members}), "pick": pick})
    for kind in out:
        out[kind].sort(key=lambda a: (a["name"].split(",")[0], order[kind](a["pick"])))
    return out


def archetypes_json(models_json=None):
    """archetypes() for the editor (JSON); models_json: the vehicles to learn from (default: every held one)."""
    return json.dumps(archetypes(catalogue(json.loads(models_json) if models_json else None)))
