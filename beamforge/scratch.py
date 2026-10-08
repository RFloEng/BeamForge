"""A BeamNG vehicle made from scratch: a sketch's structure, wheels, an engine, a gearbox and a steering rack.

Pure Python, standard library only (runs in Pyodide). UNVERIFIED IN-GAME: the first version, to be tested in BeamNG.

What is generated
  structure  the sketch (skeleton.build: rigid parts, joints, tubes, stable values), in the vehicle's frame: the SVJ frame
             placed with the front axle at y = 0 and the ground at z = 0
  wheels     the rim and tyre parts of the user's install (archetypes, donors.py) as slots of a hub part per axle: the
             rim's own axle nodes, put on the wheel centres by the slot's nodeOffset (as the vanilla cars do), braced to
             the corner's upright; a wheel data part per axle with the brakes and the wheel rows (node1 outer, node2
             inner, the steering axis on the upright's ball joints), after the hub part so its rows take that axle's
             tyre; a base.pc choosing the tyre for the rim's tyre slot. The wheels show the rims' and tyres' meshes
  powertrain the engine block's nodes, beams and data (mainEngine, sounds, vehicleController) and the gearbox's chain
             and data from archetype parts, moved to the engine's place and mounted to the nearest frame nodes; the
             torque curve, ratios and final drive as set; a driveline for FWD, RWD or AWD; a fuel tank
  steering   a rack between the tie rods' inner ends: a rail with two slider nodes held to the frame and two hydros
             crosswise, as the vanilla cars do; it turns the way the tie rods ask (ahead of the axle or behind it)
             to the lock angle asked for
  the rest   refNodes, an external camera, the vehicle controller, info.json
Everything copied from a donor part comes from the user's own install at export time; nothing of it is in BeamForge.
"""

import copy
import json
import math
import re

from . import beamng, jbeam, rigidity, skeleton, svj as svjmod, values

RACK_KG, SLIDER_KG = 3.0, 2.0
MOUNTS = 3               # beams from each engine or gearbox node to the frame
SLIDER_IN = 0.1          # m: the rack's sliders this far inside its ends
DRIVETRAIN = ("FWD", "RWD", "AWD")
STRIP = ("breakTriggerBeam", "deformGroups", "axleBeams")   # names of beams of the donor car that are not here


def _vars(part):
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


def _resolve(v, vars_):
    """A donor value with its tuning variables put in (an expression "$=..." evaluated); other strings as they are."""
    if isinstance(v, str) and v.startswith("$"):
        try:
            return jbeam.to_float(vars_.get(v, v) if not v.startswith("$=") else v, vars_, None)
        except (jbeam.UnresolvedValue, TypeError, ValueError, ZeroDivisionError):
            return None
    if isinstance(v, dict):
        return {k: _resolve(x, vars_) for k, x in v.items() if k not in STRIP}
    if isinstance(v, list):
        return [_resolve(x, vars_) for x in v]
    return v


def _part(source):
    """The donor part a choice points at ({"model", "part"}) and its tuning variables' defaults."""
    if not source:
        return None, {}
    held = beamng._parts_held(source["model"] if source["model"] != "common" else "common")
    rec = held.get(source["part"])
    if not rec:
        raise ValueError(f"{source['part']} ({source['model']}) is not among the read files: learn from the install again")
    return rec["part"], _vars(rec["part"])


def _table_rows(t):
    """(header, [rows as dicts with the inline dict merged and the property rows in force]) of a jbeam table."""
    if not (isinstance(t, list) and t and isinstance(t[0], list)):
        return [], []
    head = [str(h).rstrip(":") for h in t[0]]
    props, out = {}, []
    for r in t[1:]:
        if isinstance(r, dict):
            props.update(r)
        elif isinstance(r, list) and r:
            rec = dict(props)
            rec.update(dict(zip(head, r)))
            if isinstance(r[-1], dict):
                rec.update(r[-1])
            out.append(rec)
    return head, out


def _merged_props(t):
    out = {}
    for r in (t or [])[1:]:
        if isinstance(r, dict):
            out.update(r)
    return out


def _unit(v):
    n = math.sqrt(sum(x * x for x in v))
    return [x / n for x in v] if n > 1e-12 else [0.0, 0.0, 0.0]


def _sub(a, b):
    return [a[i] - b[i] for i in range(3)]


def _dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


class _Car:
    """The vehicle as it is built: nodes {id: [x, y, z] BeamNG}, their kg and properties, beams with values."""

    def __init__(self):
        self.nodes, self.kg, self.props, self.beams, self.notes = {}, {}, {}, [], []
        self.ks, self.cs = {}, {}

    def node(self, nid, pos, kg, **props):
        self.nodes[nid] = [round(x, 4) for x in pos]
        self.kg[nid] = kg
        self.props[nid] = props
        self.ks.setdefault(nid, 0.0)
        self.cs.setdefault(nid, 0.0)

    def beam(self, a, b, values=None, aim="p90", want=None, **extra):
        """A beam; without values, stable ones (rigidity.beam_values) for its nodes as they are loaded so far."""
        if values is None:
            v = rigidity.beam_values(self.kg[a], self.kg[b], (self.ks[a], self.ks[b]), (self.cs[a], self.cs[b]), aim=aim, want=want)
            values = {"beamSpring": v["beamSpring"], "beamDamp": v["beamDamp"]}
        self.ks[a] = self.ks.get(a, 0.0) + float(values.get("beamSpring") or 0)
        self.ks[b] = self.ks.get(b, 0.0) + float(values.get("beamSpring") or 0)
        self.cs[a] = self.cs.get(a, 0.0) + float(values.get("beamDamp") or 0)
        self.cs[b] = self.cs.get(b, 0.0) + float(values.get("beamDamp") or 0)
        self.beams.append([a, b, dict(values, **extra)])

    def nearest(self, pos, among, k):
        return sorted(among, key=lambda n: math.dist(self.nodes[n], pos))[:k]


def _structure(car, sketch, yf, zg):
    """The sketch's nodes and beams (stable values), and its build (to find parts, joints and roles)."""
    opts = sketch.get("opts") or {}
    res = skeleton.build(sketch["parts"], 1.0, (0, 0, 0), (0.0, 0.0, 0.0), opts.get("tol") or skeleton.TOL,
                         opts.get("flat") or skeleton.FLAT, opts.get("kinds"))
    kg, bv, _, _ = skeleton.values(res, opts.get("tubes"), opts.get("min_kg") or skeleton.MIN_KG)
    for n, v in res["nodes"].items():
        if not v["reference"]:
            car.node(n, svjmod.from_sae(v["pos"], yf, zg), kg[n], group=f"sk_{v['owner']}" if v["owner"] else "")
    for i, b in enumerate(res["beams"]):
        car.beam(b["a"], b["b"], {k: bv[i][k] for k in ("beamSpring", "beamDamp", "beamDeform", "beamStrength")})
    return res


def _upright(res, corner, spec):
    """The part that carries a corner's wheel: the one named, else the corner's part with the hub role (upright_fl)."""
    name = spec.get("hub") or next((p["name"] for p in res["parts"] if p["role"] == "hub" and p["name"].endswith("_" + corner.lower())), None)
    up = next((p for p in res["parts"] if p["name"] == name), None)
    if not up or len(up["nodes"]) < 3:
        raise ValueError(f"{corner}: no upright part with three nodes or more ({name or 'none named'}) to carry the wheel")
    return up


def _steer_axis(car, res, upright):
    """The upright's ball joints (its nodes shared with the arms), upper first; else its highest and lowest nodes."""
    joints = [n for j in res["joints"] if any(upright in ps for ps in j["parts"]) for n in j["nodes"]]
    up = next(p for p in res["parts"] if p["name"] == upright)
    cand = [n for n in dict.fromkeys(joints) if n in up["nodes"]] or up["nodes"]
    cand = sorted(cand, key=lambda n: -car.nodes[n][2])
    return cand[0], cand[-1]


def _rim_axle(rim, vars_):
    """The rim part's own axle nodes, by side: {+1 (left): (inner id, outer id, inner x, outer x, kg), -1: ...}, local to
    its slot (the hub part's nodeOffset puts them on the wheel). The vanilla rims define them so (fwhl1l, fwhl1ll...)."""
    _, rows = _table_rows(rim.get("nodes"))
    by = {1: [], -1: []}
    for r in rows:
        x = _resolve(r.get("posX"), vars_)
        if isinstance(r.get("id"), str) and isinstance(x, (int, float)) and abs(x) > 1e-6:
            by[1 if x > 0 else -1].append((abs(x), r["id"], float(_resolve(r.get("nodeWeight", 4.5), vars_) or 4.5)))
    out = {}
    for side, ns in by.items():
        if len(ns) >= 2:
            ns.sort()
            out[side] = (ns[0][1], ns[-1][1], ns[0][0], ns[-1][0], ns[0][2])
    if len(out) < 2:
        raise ValueError("the rim part has no axle nodes on both sides")
    return out


def _wheel_slot(rim):
    """The slot type of the rim's tyre (a rim's child slot): its type and default part."""
    t = rim.get("slots") or rim.get("slots2")
    if isinstance(t, list) and t and isinstance(t[0], list):
        head = [str(h) for h in t[0]]
        for r in t[1:]:
            if isinstance(r, list):
                rec = dict(zip(head, r))
                kind = rec.get("type") or rec.get("name")
                if isinstance(kind, str) and kind.startswith("tire"):
                    return kind, rec.get("default")
    return None, None


def _copy_nodes(car, part, vars_, delta, prefix_group):
    """A donor part's nodes moved by delta (BeamNG): their ids, weights and properties kept. Returns the ids."""
    head, rows = _table_rows(part.get("nodes"))
    ids = []
    for r in rows:
        nid = r.get("id")
        if not isinstance(nid, str) or nid in car.nodes:
            continue
        pos = [_resolve(r.get(k), vars_) for k in ("posX", "posY", "posZ")]
        if any(p is None for p in pos):
            continue
        props = {k: _resolve(v, vars_) for k, v in r.items() if k not in ("id", "posX", "posY", "posZ", "nodeWeight")}
        car.node(nid, [pos[i] + delta[i] for i in range(3)], float(_resolve(r.get("nodeWeight", 25), vars_) or 25), **props)
        ids.append(nid)
    return ids


def _copy_beams(car, part, vars_, ids):
    """A donor part's beams between the given nodes (their values resolved), so a copied engine block stays one."""
    _, rows = _table_rows(part.get("beams"))
    keep = set(ids)
    n = 0
    for r in rows:
        a, b = r.get("id1"), r.get("id2")
        if a in keep and b in keep and "BOUNDED" not in str(r.get("beamType", "")):
            vals = {k: _resolve(r[k], vars_) for k in ("beamSpring", "beamDamp", "beamDeform", "beamStrength") if k in r}
            car.beam(a, b, vals)
            n += 1
    return n


def _mount(car, ids, frame, label):
    """Each of a block's nodes tied to its nearest frame nodes by stable beams."""
    for n in ids:
        for f in car.nearest(car.nodes[n], frame, MOUNTS):
            car.beam(n, f, aim="p50")
    car.notes.append(f"{label}: {len(ids)} nodes mounted to the frame by {MOUNTS} beams each")


def _rack(car, res, wheels, steering, frame):
    """The steering rack: the tie rods' inner ends (rack ends) on a rail, two sliders held to the frame, two hydros.
    Returns (hydros table, rails, slidenodes) or None."""
    ends = {}
    for corner in ("FL", "FR"):
        tie = next((p for p in res["parts"] if p["role"] == "tie" and p["name"].endswith("_" + corner.lower())), None)
        if not tie or corner not in wheels:
            continue
        hub = set(wheels[corner]["upright_nodes"])
        inner = [n for n in tie["nodes"] if n not in hub and n in car.nodes]
        outer = [n for n in tie["nodes"] if n in hub]
        if inner and outer:
            ends[corner] = (min(inner, key=lambda n: abs(car.nodes[n][0])), outer[0])
    if len(ends) < 2:
        car.notes.append("steering: no tie rods (tie_rod_fl and tie_rod_fr parts ending on the uprights): the front wheels do not steer")
        return None
    (el, ol), (er, orr) = ends["FL"], ends["FR"]
    for e in (el, er):
        car.kg[e] = max(car.kg[e], RACK_KG)
    car.beam(el, er, aim="p90")                                # the rack bar
    L, R = car.nodes[el], car.nodes[er]
    axis = _unit(_sub(L, R))
    sl, sr = "bfrackl", "bfrackr"
    car.node(sl, [L[i] - SLIDER_IN * axis[i] for i in range(3)], SLIDER_KG, group="bf_steering")
    car.node(sr, [R[i] + SLIDER_IN * axis[i] for i in range(3)], SLIDER_KG, group="bf_steering")
    for s in (sl, sr):
        for f in car.nearest(car.nodes[s], frame, 4):
            car.beam(s, f, aim="p50")
    # the lock: the rack travel that turns the wheel by the lock angle about its steering axis
    lock = math.radians(steering.get("lock_deg") or 35.0)
    w = wheels["FL"]
    up, down = car.nodes[w["steer_up"]], car.nodes[w["steer_down"]]
    u = _unit(_sub(up, down))
    o = car.nodes[ol]
    r = _sub(o, down)
    arm = math.sqrt(max(0.0, _dot(r, r) - _dot(r, u) ** 2))
    travel = arm * math.sin(lock)
    length = math.dist(car.nodes[el], car.nodes[sr])
    factor = travel / length if length > 1e-6 else 0.1
    # positive input turns right: with the tie rods behind the axle the rack moves left (+x), ahead of it right
    behind = o[1] > w["centre"][1]
    sign = 1.0 if behind else -1.0
    turns = steering.get("turns") or 3.0
    hyd = {"steeringWheelLock": round(turns * 180, 1), "inRate": 0.7, "outRate": 0.7}
    hydros = [["id1:", "id2:"], {"beamPrecompression": 1.0, "beamType": "|NORMAL", "beamLongBound": 1, "beamShortBound": 1},
              {"beamSpring": 7001000, "beamDamp": 40, "beamDeform": "FLT_MAX", "beamStrength": 124000},
              [el, sr, dict(hyd, factor=round(sign * factor, 4))], [er, sl, dict(hyd, factor=round(-sign * factor, 4))]]
    rails = {"bf_steeringrack": {"links:": [er, el], "broken:": [], "looped": False, "capped": True}}
    slides = [["id:", "railName", "attached", "fixToRail", "tolerance", "spring", "strength", "capStrength"],
              [sr, "bf_steeringrack", True, True, 0.0, 12001000, "FLT_MAX", "FLT_MAX"],
              [sl, "bf_steeringrack", True, True, 0.0, 12001000, "FLT_MAX", "FLT_MAX"]]
    car.notes.append(f"steering: rack {'behind' if behind else 'ahead of'} the axle, {math.degrees(lock):.0f} deg at full lock "
                     f"(travel {travel * 1000:.0f} mm), {turns:g} turns lock to lock")
    return hydros, rails, slides


def _ref_nodes(car, frame, yf):
    """ref, back, left, up: four frame nodes spanning the vehicle's axes (BeamNG reads its frame from them)."""
    ref = min(frame, key=lambda n: abs(car.nodes[n][0]) + abs(car.nodes[n][1] - yf) * 0.5)
    o = car.nodes[ref]
    score = lambda n, ax, s: s * (car.nodes[n][ax] - o[ax]) - sum(abs(car.nodes[n][i] - o[i]) for i in range(3) if i != ax)   # noqa: E731
    back = max((n for n in frame if n != ref), key=lambda n: score(n, 1, 1))
    left = max((n for n in frame if n not in (ref, back)), key=lambda n: score(n, 0, 1))
    up = max((n for n in frame if n not in (ref, back, left)), key=lambda n: score(n, 2, 1))
    return [["ref:", "back:", "left:", "up:"], [ref, back, left, up]]


def build(car_json):
    """The vehicle's files: {"files": {path: text}, "notes": [...], "counts": {...}}. car: {"id", "name", "brand",
    "sketch": {"parts", "opts"}, "wheels": {corner: {"center" (SVJ m), "hub"?, "steered"?, "driven"?}}, "layout",
    "engine": {"source", "torque", "idle_rpm", "max_rpm", "position" (SVJ m)?}, "gearbox": {"source", "ratios"},
    "final_drive", "tyre": {"front", "rear"} (sources), "rim": {"front", "rear"}, "steering": {"lock_deg", "turns"},
    "brakes": {"front", "rear"} (Nm), "fuel_l"}."""
    spec = json.loads(car_json)
    vid = spec["id"]
    if not re.fullmatch(r"[a-z0-9_]+", vid or ""):
        raise ValueError("the vehicle id must be lower case letters, digits and _")
    yf, zg = 0.0, 0.0                                       # front axle at y = 0, ground at z = 0
    car = _Car()
    res = _structure(car, spec["sketch"], yf, zg)
    frame = [n for n, v in res["nodes"].items() if not v["reference"] and not v["helper"]
             and any(p["kind"] == "frame" for p in res["parts"] if p["name"] in v["parts"])]
    if len(frame) < 4:
        raise ValueError("the sketch needs a frame part (a structure, not a suspension link) with four nodes or more")
    layout = (spec.get("layout") or "RWD").upper()
    if layout not in DRIVETRAIN:
        raise ValueError(f"layout {layout}: one of {', '.join(DRIVETRAIN)}")

    # wheels: the rim and tyre parts of the user's install as slots of a hub part per axle (their own axle nodes, put on
    # the wheel centres by the slot's nodeOffset, as the vanilla cars do), braced to the uprights; then a wheel data part
    # per axle with the brakes and the wheel rows, after the hub part so the rows take that axle's tyre
    wheels = {}
    for corner, w in sorted((spec.get("wheels") or {}).items()):
        up = _upright(res, corner, w)
        upn = [n for n in up["nodes"] if n in car.nodes]
        su, sd = _steer_axis(car, res, up["name"])
        wheels[corner] = {"centre": svjmod.from_sae(w["center"], yf, zg), "upright_nodes": upn, "steer_up": su, "steer_down": sd,
                          "upright": up["name"]}
    if len(wheels) < 3:
        raise ValueError("at least three wheels (FL, FR, RL, RR) with a centre each")
    layout_driven = {"FWD": ("F",), "RWD": ("R",), "AWD": ("F", "R")}
    driven = layout_driven[layout]
    brakes = spec.get("brakes") or {}
    extra_parts, pc_parts, slots = {}, {}, [["type", "default", "description"]]
    for ax, AX in (("front", "F"), ("rear", "R")):
        corners = [c for c in wheels if c[0] == AX]
        if not corners:
            continue
        rim, rvars = _part((spec.get("rim") or {}).get(ax))
        if not rim:
            raise ValueError(f"choose a rim for the {ax} axle (Wheels)")
        tyre_src = (spec.get("tyre") or {}).get(ax)
        tslot, tdefault = _wheel_slot(rim)
        tyre, _ = _part(tyre_src)
        if tyre and tslot and tyre.get("slotType") == tslot:
            pc_parts[tslot] = tyre_src["part"]
        elif tyre_src:
            car.notes.append(f"{ax}: the tyre {tyre_src['part']} does not fit the rim {spec['rim'][ax]['part']} "
                             f"(it takes {tslot}); its own tyre {tdefault} is used")
        axle = _rim_axle(rim, rvars)
        cx = sum(abs(wheels[c]["centre"][0]) for c in corners) / len(corners)
        cy = sum(wheels[c]["centre"][1] for c in corners) / len(corners)
        cz = sum(wheels[c]["centre"][2] for c in corners) / len(corners)
        mid = (axle[1][2] + axle[1][3]) / 2
        offset = {"x": round(cx - mid, 4), "y": round(cy, 4), "z": round(cz, 4)}
        hub = f"{vid}_hub_{AX}"
        hbeams = [["id1:", "id2:"], {"beamType": "|NORMAL", "beamPrecompression": 1, "beamDeform": 150000, "beamStrength": "FLT_MAX"}]
        rows = [["name", "hubGroup", "group", "node1:", "node2:", "nodeS", "nodeArm:", "wheelDir"],
                {"brakeTorque": float(brakes.get(ax) or (2000 if AX == "F" else 1200)), "parkingTorque": 0 if AX == "F" else 1500,
                 "enableBrakeThermals": False, "selfCollision": False, "collision": True, "enableHubcaps": False}]
        for c in corners:
            w = wheels[c]
            side = 1 if w["centre"][0] >= 0 else -1                      # BeamNG x: left +
            inner, outer, _, _, rim_kg = axle[side]
            w.update(node1=outer, node2=inner)
            for n in (inner, outer):                                    # the rim's axle nodes braced to the upright
                car.kg.setdefault(n, rim_kg)
                car.ks.setdefault(n, 0.0)
                car.cs.setdefault(n, 0.0)
                for u in w["upright_nodes"]:
                    v = rigidity.beam_values(car.kg[n], car.kg[u], (car.ks[n], car.ks[u]), (car.cs[n], car.cs[u]), aim="p90")
                    for a, b in ((n, u),):
                        car.ks[a] += v["beamSpring"]
                        car.ks[b] += v["beamSpring"]
                        car.cs[a] += v["beamDamp"]
                        car.cs[b] += v["beamDamp"]
                    hbeams.append([n, u, {"beamSpring": v["beamSpring"], "beamDamp": v["beamDamp"]}])
            inline = {"torqueCoupling:": w["upright_nodes"][0], "torqueArm:": w["upright_nodes"][1 % len(w["upright_nodes"])], "torqueArm2:": outer}
            if AX == "F":
                inline.update({"steerAxisUp:": w["steer_up"], "steerAxisDown:": w["steer_down"]})
            rows.append([c, f"wheel_{c}", f"tire_{c}", outer, inner, 9999, w["upright_nodes"][0], -1 if side > 0 else 1, inline])
        extra_parts[hub] = {"information": {"authors": "BeamForge", "name": f"{ax.capitalize()} hubs"}, "slotType": hub,
                            "slots": [["type", "default", "description"],
                                      [rim["slotType"], spec["rim"][ax]["part"], f"{ax.capitalize()} wheels", {"nodeOffset": offset}]],
                            "beams": hbeams}
        wd = f"{vid}_wheeldata_{AX}"
        extra_parts[wd] = {"information": {"authors": "BeamForge", "name": f"{ax.capitalize()} wheel data"}, "slotType": wd, "pressureWheels": rows}
        slots += [[hub, hub, f"{ax.capitalize()} hubs"], [wd, wd, f"{ax.capitalize()} wheel data"]]

    # powertrain
    eng = spec.get("engine") or {}
    epart, evars = _part(eng.get("source"))
    if not epart or not isinstance(epart.get("mainEngine"), dict):
        raise ValueError("choose an engine (Powertrain, from the vanilla cars)")
    gpart, gvars = _part((spec.get("gearbox") or {}).get("source"))
    if not gpart:
        raise ValueError("choose a gearbox (Powertrain, from the vanilla cars)")
    eids_donor = [r.get("id") for r in _table_rows(epart.get("nodes"))[1]]
    dpos = [[_resolve(r.get(k), evars) for k in ("posX", "posY", "posZ")] for r in _table_rows(epart.get("nodes"))[1]]
    dpos = [p for p in dpos if None not in p]
    if not dpos:
        raise ValueError(f"{eng['source']['part']} has no nodes to copy")
    dc = [sum(p[i] for p in dpos) / len(dpos) for i in range(3)]
    front = [wheels[c] for c in wheels if c[0] == "F"]
    fc = [sum(w["centre"][i] for w in front) / len(front) for i in range(3)]
    target = svjmod.from_sae(eng["position"], yf, zg) if eng.get("position") else [0.0, fc[1] + (0.05 if layout == "FWD" else 0.35), fc[2] + 0.05]
    delta = [target[i] - dc[i] for i in range(3)]
    eids = _copy_nodes(car, epart, evars, delta, "engine")
    n_eb = _copy_beams(car, epart, evars, eids)
    gids = _copy_nodes(car, gpart, gvars, delta, "gearbox")
    n_gb = _copy_beams(car, gpart, gvars, gids + eids)
    _mount(car, eids, frame, f"engine ({eng['source']['part']})")
    if gids:
        _mount(car, gids, frame, f"gearbox ({spec['gearbox']['source']['part']})")

    main_engine = _resolve(copy.deepcopy(epart["mainEngine"]), evars)
    main_engine.update({"torque": [["rpm", "torque"]] + [[float(r), float(t)] for r, t in (eng.get("torque") or [])] if eng.get("torque") else main_engine.get("torque"),
                        "energyStorage": ["mainTank"], "thermalsEnabled": False})
    if eng.get("idle_rpm"):
        main_engine["idleRPM"] = float(eng["idle_rpm"])
    for k, key in (("inertia", "inertia"), ("friction", "friction"), ("engine_brake", "engineBrakeTorque")):
        if isinstance(eng.get(k), (int, float)):
            main_engine[key] = float(eng[k])
    if isinstance(eng.get("mass"), (int, float)) and eids:     # the engine's mass: its block's node weights scaled to it
        k = eng["mass"] / sum(car.kg[n] for n in eids)
        for n in eids:
            car.kg[n] = round(car.kg[n] * k, 3)
    if eng.get("max_rpm"):
        main_engine.update(maxRPM=float(eng["max_rpm"]), revLimiterRPM=float(eng["max_rpm"]), hasRevLimiter=True)
    main_engine["torqueReactionNodes:"] = [n for n in (main_engine.get("torqueReactionNodes:") or []) if n in car.nodes] or eids[:3]
    for k in ("waterDamage", "radiator", "engineBlock", "particulates"):
        main_engine.pop(k, None)
    fuel = main_engine.get("requiredEnergyType") or "gasoline"

    grows = [r for r in (gpart.get("powertrain") or [])[1:] if isinstance(r, list)]
    gdev = (spec.get("gearbox") or {}).get("source", {}).get("device") or "gearbox"
    gearbox = _resolve(copy.deepcopy(gpart.get(gdev) or {}), gvars)
    ratios = (spec.get("gearbox") or {}).get("ratios")
    if ratios:
        old = gearbox.get("gearRatios") or [-3.5, 0]
        k = next((i for i, x in enumerate(old[:3]) if x == 0), 1)
        gearbox["gearRatios"] = list(old[:k + 1]) + [float(x) for x in ratios]
    if "gearboxNode:" in gearbox:
        gearbox["gearboxNode:"] = [n for n in gearbox["gearboxNode:"] if n in car.nodes] or gids[:1] or eids[:1]
    fd = float(spec.get("final_drive") or 4.0)
    pt = [["type", "name", "inputName", "inputIndex"], ["combustionEngine", "mainEngine", "dummy", 0]]
    out_dev = "gearbox"
    for r in grows:                                         # the gearbox part's own chain (clutch or converter, gearbox)
        rr = _resolve(copy.deepcopy(r), gvars)
        if rr[1] == gdev:
            out_dev = rr[1]
        pt.append(rr)
    sections = {}
    for r in grows:
        if isinstance(gpart.get(r[1]), dict) and r[1] != gdev:
            sections[r[1]] = _resolve(copy.deepcopy(gpart[r[1]]), gvars)
    halfshaft = lambda c, diff, i: ["shaft", f"wheelaxle{c}", diff, i, {"uiName": f"{c} halfshaft", "friction": 0.77, "dynamicFriction": 0.0019}]   # noqa: E731
    spindle = lambda c: ["shaft", f"spindle{c}", f"wheelaxle{c}", 1, {"connectedWheel": c, "friction": 0.8, "dynamicFriction": 0.0015}]   # noqa: E731
    diffs = spec.get("diffs") or {}
    if layout == "AWD":
        split = diffs.get("split") if isinstance(diffs.get("split"), (int, float)) else 0.4
        pt.append(["differential", "differential_C", out_dev, 1, {"diffType": "open", "diffTorqueSplit": round(split, 3), "gearRatio": 1, "uiName": "Centre Differential"}])
        feeds = {"F": ("differential_C", 1), "R": ("differential_C", 2)}
    else:
        feeds = {driven[0]: (out_dev, 1)}
    for ax in driven:
        src, idx = feeds[ax]
        pt.append(["shaft", f"driveshaft_{ax}", src, idx, {"friction": 0.5, "dynamicFriction": 0.0005, "uiName": f"Driveshaft {ax}"}])
        dset = {"diffType": "open", "gearRatio": fd, "uiName": f"{'Front' if ax == 'F' else 'Rear'} Differential", "defaultVirtualInertia": 0.25}
        dset.update(values.bng_diff(diffs.get("front" if ax == "F" else "rear")))
        pt.append(["differential", f"differential_{ax}", f"driveshaft_{ax}", 1, dset])
        sections[f"differential_{ax}"] = {"friction": 1.5, "dynamicFriction": 0.0007, "torqueLossCoef": 0.016}
        for i, c in enumerate((f"{ax}L", f"{ax}R"), 1):
            if c in wheels:
                pt.append(halfshaft(c, f"differential_{ax}", i))
                pt.append(spindle(c))

    vc = {}
    for part, vars_ in ((epart, evars), (gpart, gvars)):
        if isinstance(part.get("vehicleController"), dict):
            vc.update(_resolve(copy.deepcopy(part["vehicleController"]), vars_))
    steering = _rack(car, res, wheels, spec.get("steering") or {}, frame)

    # the main part
    p = {"information": {"authors": "BeamForge", "name": spec.get("name") or vid}, "slotType": "main",
         "refNodes": _ref_nodes(car, frame, yf),
         "cameraExternal": {"distance": 5.0, "distanceMin": 2, "offset": {"x": 0, "y": 0, "z": 0.45}, "fov": 65},
         "controller": [["fileName"], ["vehicleController", {}]],
         "powertrain": pt, "mainEngine": main_engine, "gearbox": gearbox, **sections,
         "energyStorage": [["type", "name"], ["fuelTank", "mainTank"]],
         "mainTank": {"energyType": fuel, "fuelCapacity": float(spec.get("fuel_l") or 50), "startingFuelCapacity": float(spec.get("fuel_l") or 50)},
         "vehicleController": vc, "slots": slots}
    for k in ("soundConfig", "soundConfigExhaust"):
        if isinstance(epart.get(k), dict):
            p[k] = _resolve(copy.deepcopy(epart[k]), evars)
    if steering:
        p["hydros"], p["rails"], p["slidenodes"] = steering
    nrows = [["id", "posX", "posY", "posZ"], {"selfCollision": False}, {"collision": True}, {"nodeMaterial": "|NM_METAL"}, {"frictionCoef": 0.5}]
    for n, pos in car.nodes.items():
        extra = {k: v for k, v in car.props[n].items() if v not in (None, "")}
        extra["nodeWeight"] = round(car.kg[n], 3)
        nrows.append([n, pos[0], pos[1], pos[2], extra])
    p["nodes"] = nrows
    p["beams"] = [["id1:", "id2:"], {"beamType": "|NORMAL", "beamPrecompression": 1, "beamDeform": 80000, "beamStrength": "FLT_MAX"}] + car.beams
    jb = {f"{vid}_body": p, **extra_parts}
    info = {"Name": spec.get("name") or vid, "Brand": spec.get("brand") or "BeamForge", "Author": "BeamForge", "Type": "Car",
            "Body Style": "Custom", "Description": "Made from scratch in BeamForge.", "Years": {"min": 2026, "max": 2026},
            "default_pc": "base"}
    pc = {"format": 2, "model": vid, "parts": dict(pc_parts), "vars": {}}
    mass = round(sum(car.kg.values()), 1)
    car.notes.append(f"{len(car.nodes)} nodes, {len(car.beams)} beams, {mass} kg of nodes (wheels and tyres come on top);"
                     f" engine block {len(eids)} nodes and {n_eb} beams copied, gearbox {len(gids)} nodes and {n_gb} beams")
    bands = {"ok": 0, "high": 0, "extreme": 0, "beyond": 0}
    for n in car.kg:                                         # the structure's nodes and the rims' axle nodes
        bands[rigidity.node_index(car.kg[n], car.ks[n], car.cs[n])[2]] += 1
    return json.dumps({"files": {f"vehicles/{vid}/info.json": json.dumps(info, indent=1),
                                 f"vehicles/{vid}/{vid}.jbeam": json.dumps(jb, indent=1),
                                 f"vehicles/{vid}/base.pc": json.dumps(pc, indent=1)},
                       "notes": car.notes, "counts": {"nodes": len(car.nodes), "beams": len(car.beams), "mass": mass,
                                                      "bands": bands, "wheels": len(wheels)}})
