"""Suspension archetypes: BeamForge's own suspension corners, built from the SVJ's hardpoints.

Pure Python, standard library only (runs in Pyodide). See docs/archetypes.md. The base keeps its body,
subframes, powertrain, wheels and brakes; what follows the wheel (kinematics.follows_wheel: hub, strut
bottom, anti-roll bar link) is taken out with every row that uses it, and the archetype's corner is
written in its place, its nodes the SVJ's hardpoints:

  hub        lower ball joint (h1), strut bottom (h2, when the SVJ's is not the ball joint), tie rod
             end (h3), a node on the strut's axis that slides up its rail (h4) and one across the
             wheel from the tie rod end (h5): a rigid body, beamed to the wheel's two nodes
  lower arm  its inner pivots (p0, p1), each its own node mounted to the body side
  strut      its top (t), mounted to the body; the rail from the strut bottom to it; spring (h4 to t),
             damper (strut bottom to t), bump stop and travel limit
  tie rod    to the steering rack (front: the rack's ends on a rail, steered by hydros as the base's) or
             to a toe link pivot (rear)

Mounts: each body-side node of the archetype is beamed to the nearest nodes of the body (the part
with the most nodes) and of the suspension parts' subframes that stay (MOUNTS of them, at least
MOUNT_KG heavy).
"""

import json
import math

from beamforge import kinematics, rigidity, svj as svjmod, values

# beams, N/m: as the vanilla struts' (front-drive compact)
WHEEL_SOFT_K = 6001000
HUB_K, WHEEL_K, ARM_K, TIE_K, MOUNT_K, RACK_K, SLIDE_K = 7501000, 9001000, 17001000, 15001000, 4501000, 10001000, 18001000
MOUNTS = 5            # body nodes each mounted node is beamed to
MOUNT_KG = 2.0        # kg: lighter nodes (flexbody helpers) are not mounted to
MOUNT_NEAR = 0.04     # m: closer body nodes are skipped (a beam needs a length)
MOUNT_C = 80          # N s/m: the mounts' damping (light: the body nodes' damping is near its limit)
ROOM = 1.0            # of a body node's stiffness limit that its mounts may fill
MOUNT_REACH = 0.45    # m: no farther body nodes
PIVOT_KG, TOP_KG, RACK_KG = 2.5, 2.5, 3.0
HUB_SHARE = {"h1": 0.3, "h2": 0.1, "h3": 0.2, "h4": 0.2, "h5": 0.2}   # of the corner's unsprung kg
STEER_C, STEER_C_FAST = 80, 800   # N s/m: steering dampers, slow and fast (the front-drive compact's)
SLIDE_AT = 0.6        # h4: this share of the way from the strut bottom to its top
RACK_INSET = 0.1      # m: the rack's slide nodes inboard of its ends

NODE_RESET = {"nodeMaterial": "|NM_METAL", "frictionCoef": 0.5, "collision": True, "selfCollision": False,
              "group": "", "engineGroup": "", "fixed": False}
BEAM_RESET = {"beamType": "|NORMAL", "beamPrecompression": 1, "beamPrecompressionTime": 0, "beamLongBound": 1,
              "beamShortBound": 1, "deformLimitExpansion": "", "breakGroup": "", "optional": False,
              "deformGroup": "", "beamDamp": 150, "beamDeform": 85000, "beamStrength": 350000}
BOUNDED = {"beamType": "|BOUNDED", "beamLongBound": 1, "beamShortBound": 1}


def _sub(a, b):
    return [a[i] - b[i] for i in range(3)]


def _add(a, b, s=1.0):
    return [a[i] + s * b[i] for i in range(3)]


def _unit(v):
    n = math.sqrt(sum(x * x for x in v))
    return [x / n for x in v] if n > 1e-12 else [0.0, 0.0, 0.0]


def _r(p):
    return [round(x, 4) for x in p]


def strut_points(hp):
    """The strut archetype's points of one corner from its SVJ hardpoints ({name: BeamNG position}), or
    None when the corner is not a strut (a lower arm of two pivots, a strut top, a tie rod or toe link)."""
    lbj, top = hp.get("lower_ball_joint"), hp.get("strut.0")
    tre = hp.get("steering_tie_rod_end") or hp.get("toe_link_outer")
    tri = hp.get("steering_tie_rod.0") or hp.get("toe_link.0")
    arm = [hp.get("lower_control_arm.0"), hp.get("lower_control_arm.1")]
    wc = hp.get("wheel_center")
    if not (lbj and top and tre and tri and wc and all(arm)):
        return None
    bottom = hp.get("strut_outboard") or hp.get("damper_outboard") or lbj
    if math.dist(bottom, lbj) < 0.02:
        bottom = lbj
    pts = {"wc": wc, "h1": lbj, "h3": tre, "t": top, "p0": arm[0], "p1": arm[1], "tie": tri,
           "steer": "steering_tie_rod.0" in hp}
    if bottom is not lbj:
        pts["h2"] = bottom
    base = pts.get("h2", lbj)
    pts["h4"] = _add(base, _sub(top, base), SLIDE_AT)
    side = 1.0 if wc[0] > 0 else -1.0
    pts["h5"] = [wc[0] - side * 0.04, 2 * wc[1] - tre[1], wc[2]]     # across the wheel from the tie rod end
    return pts


def motion_ratio(pts):
    """The spring's and damper's motion ratio of a strut corner: the strut rides on the hub, so it
    shortens by its axis' vertical share of the wheel's travel."""
    a = _unit(_sub(pts["t"], pts.get("h2", pts["h1"])))
    return round(abs(a[2]), 3)


def _take(room, node, k, c):
    """A beam of k N/m and c N s/m onto a base node: from the room it has under its limits for the
    physics step, and where that is short, weight added to the node so its k / m and c / m stay within
    them (room["kg"]: {node: kg added})."""
    short_k = k - room["k"].get(node, 0.0)
    short_c = c - room["c"].get(node, 0.0)
    kg = max(short_k / room["lk"][node] if short_k > 0 and room["lk"].get(node) else 0.0,
             short_c / room["lc"][node] if short_c > 0 and room["lc"].get(node) else 0.0)
    if kg > 0:
        room["kg"][node] = room["kg"].get(node, 0.0) + kg
        room["k"][node] = room["k"].get(node, 0.0) + kg * room["lk"].get(node, 0.0)
        room["c"][node] = room["c"].get(node, 0.0) + kg * room["lc"].get(node, 0.0)
    room["k"][node] = room["k"].get(node, 0.0) - k
    room["c"][node] = room["c"].get(node, 0.0) - c


def _mounts(at, nodes, kept, weights, side, room):
    """The body nodes a mounted node is beamed to: the MOUNTS nearest that stay, within MOUNT_REACH, on
    its side of the car (or near the middle), not lighter than MOUNT_KG, not closer than MOUNT_NEAR.
    Each takes its beam (_take: weight added where its limits are short)."""
    cand = [n for n in kept if weights.get(n, 0.0) >= MOUNT_KG and nodes[n][0] * side > -0.15
            and MOUNT_NEAR <= math.dist(nodes[n], at) <= MOUNT_REACH]
    out = sorted(cand, key=lambda n: math.dist(nodes[n], at))[:MOUNTS]
    for n in out:
        _take(room, n, MOUNT_K, MOUNT_C)
    return out


def _on_base(node, k, c, room):
    """A link's beam onto a base node (an archetype point merged with it), taken as _take does."""
    if node in room["lk"]:
        _take(room, node, k, c)
    return {"beamSpring": k, "beamDamp": c}


def _room(model, configured, built, removed):
    """{"k": {node: N/m of beam stiffness it can still take}, "c": {node: N s/m of damping}, "lk", "lc":
    its limits per kg (rigidity._limits, from the base), "kg": {node: kg to add}}: the limits times its
    weight in the built car, less the beams it keeps there."""
    lk, lc = rigidity._limits(model, configured, kinematics.beam_list(model, configured))
    bm, bc = built
    rows, _ = values.node_weights(bm, bc)
    m = {}
    for r in rows:
        m[r[2]] = m.get(r[2], 0.0) + r[3]
    k, c = {}, {}
    for b in rigidity.beams(bm, bc):
        if b["a"] in removed or b["b"] in removed:
            continue
        for n in (b["a"], b["b"]):
            k[n] = k.get(n, 0.0) + b["values"].get("beamSpring", 0.0)
            c[n] = c.get(n, 0.0) + b["values"].get("beamDamp", 0.0)
    return {"k": {n: lk[n] * m[n] * ROOM - k.get(n, 0.0) for n in lk if n in m},
            "c": {n: lc[n] * m[n] * ROOM - c.get(n, 0.0) for n in lc if n in m},
            "lk": {n: lk[n] * ROOM for n in lk}, "lc": {n: lc[n] * ROOM for n in lc}, "kg": {}}


def _table(part, sec, head):
    t = part.get(sec)
    if not isinstance(t, list) or not t or not isinstance(t[0], list):
        t = [list(head)]
        part[sec] = t
    return t


def _head(t):
    return [str(h).rstrip(":") for h in t[0]]


def _row(t, rec):
    """A row for a table in its own column order (rec by column name); the rest inline."""
    head = _head(t)
    row = [rec.get(h) for h in head]
    extra = {k: v for k, v in rec.items() if k not in head}
    return row + [extra] if extra else row


def _drop_rows(part, removed, members):
    """Every row of a part that uses a removed node, out. Returns the rails taken out."""
    dropped_rails = set()
    rails = part.get("rails")
    if isinstance(rails, dict):
        for name, r in list(rails.items()):
            links = (r.get("links:") or r.get("links")) if isinstance(r, dict) else None
            if isinstance(links, list) and any(str(n) in removed for n in links):
                del rails[name]
                dropped_rails.add(name)
    for sec, t in list(part.items()):
        if sec in ("pressureWheels", "refNodes", "variables", "slots", "slots2", "powertrain", "information") \
                or not isinstance(t, list) or not t or not isinstance(t[0], list):
            continue
        head = _head(t)
        keep = [t[0]]
        for row in t[1:]:
            if isinstance(row, list):
                if sec == "nodes":
                    if "id" in head and str(row[head.index("id")]) in removed:
                        continue
                elif sec == "flexbodies":
                    gi = next((i for i, h in enumerate(head) if h.startswith("[group]")), 1)
                    gs = row[gi] if gi < len(row) and isinstance(row[gi], list) else []
                    ns = [n for g in gs for n in members.get(g, [])]
                    if ns and all(n in removed for n in ns):
                        continue
                elif sec == "slidenodes":
                    if str(row[0]) in removed or (len(row) > 1 and str(row[1]) in dropped_rails):
                        continue
                elif any(isinstance(x, str) and x in removed for x in row):
                    continue
            keep.append(row)
        part[sec] = keep
    return dropped_rails


def _remap_wheels(part, corner, names):
    """The pressureWheels row of a corner pointed at the archetype's hub: arm, torque and steering axis
    nodes."""
    t = part.get("pressureWheels")
    if not isinstance(t, list) or not t or not isinstance(t[0], list):
        return False
    head = _head(t)
    done = False
    for row in t[1:]:
        if not isinstance(row, list) or not row or row[0] != corner:
            continue
        if "nodeArm" in head and head.index("nodeArm") < len(row):
            row[head.index("nodeArm")] = names["h5"]
        opts = row[-1] if isinstance(row[-1], dict) else None
        if opts is not None:
            for k in list(opts):
                key = k.rstrip(":")
                if key == "torqueCoupling" or key == "steerAxisDown":
                    opts[k] = names["h1"]
                elif key == "torqueArm":
                    opts[k] = names["h4"]
                elif key == "steerAxisUp":
                    opts[k] = names["t"]
        done = True
    return done


def apply(files, model, configured, svj, place, loads, built, axles=None):
    """The built vehicle's files (export.build) with its strut corners replaced by the archetype's:
    (files, notes, corners done). configured: the configure() the vehicle was built from (fit
    included); place: fit()'s {"yf", "ground"}; loads: {corner: static wheel load, N} of the built car
    (corner_loads); built: (model, configured) of the built car before this (its node weights); axles:
    fit()'s wheel targets (the wheels spawn square, their camber and toe from the hub's preloaded beams)."""
    notes, done = [], []
    geo = configured["geometry"]
    nodes, owner = geo["nodes"], geo.get("parts") or {}
    members = {}
    for n, gs in (geo.get("groups") or {}).items():
        for g in gs:
            members.setdefault(g, []).append(n)
    rows, _ = values.node_weights(model, configured)
    weights = {r[2]: r[3] for r in rows}
    hps = {}
    for h in svjmod.hardpoints(svj, place["yf"], place["ground"]):
        hps.setdefault(h["corner"], {})[h["name"]] = h["pos"]
    bl = kinematics.beam_list(model, configured)
    springs = values.springs_and_dampers(model, configured)

    # the corners to replace and what follows each wheel
    corners = {}
    for w in configured.get("wheels") or []:
        pts = strut_points(hps.get(w["name"]) or {})
        if not pts:
            continue
        moving = kinematics.follows_wheel(model, configured, w, bl, springs)
        if not moving:
            notes.append(f"{w['name']}: nothing follows the wheel in the static solve: kept as the base's")
            continue
        corners[w["name"]] = (w, pts, moving)
    if not corners:
        return files, notes, done
    removed = set().union(*(c[2] for c in corners.values()))
    wheel_nodes = {n for w, _, _ in corners.values() for n in (w["node1"], w["node2"])}

    # the exported parts, by name
    docs = {p: json.loads(t) for p, t in files.items() if p.endswith(".jbeam")}
    where = {}
    for p, doc in docs.items():
        for name, part in doc.items():
            if isinstance(part, dict):
                where.setdefault(name, (p, part))
    active = set(owner.values()) | {r["part"] for r in springs}
    for n, _ in _walk_tree(configured.get("tree") or {}):
        active.add(n)

    # the steering (front): a part with hydros that steers the replaced hubs. Its rack is kept (BeamNG's
    # own, as it works in the base); the archetype's tie rods go to its ends, the nodes of it that the
    # base's tie rods reached
    steer_parts, rack_ends = [], set()
    for name in active:
        part = (where.get(name) or (None, None))[1]
        if not part or not isinstance(part.get("hydros"), list):
            continue
        own = {n for n, o in owner.items() if o == name}
        uses = {x for b in bl if b["part"] == name for x in (b["a"], b["b"])}
        if own and uses & removed:
            steer_parts.append((name, part, own))
            rack_ends |= {n for b in bl for n, m in ((b["a"], b["b"]), (b["b"], b["a"])) if n in own and m in removed}
    rack = None
    for name, part, own in steer_parts:
        for row in part["hydros"][1:]:
            if isinstance(row, list) and isinstance(row[-1], dict) and "factor" in row[-1] \
                    and row[0] in nodes and row[1] in nodes:
                rack = {"factor": abs(row[-1]["factor"]), "length": math.dist(nodes[row[0]], nodes[row[1]]),
                        "lock": row[-1].get("steeringWheelLock", 450), "inRate": row[-1].get("inRate", 1.25),
                        "outRate": row[-1].get("outRate", 1.25)}
                break
    removed -= wheel_nodes
    # mounted to the body and the suspension's subframe only (not the engine, exhaust or differential,
    # which move on their own mounts)
    count = {}
    for n, o in owner.items():
        count[o] = count.get(o, 0) + 1
    body_part = max(count, key=count.get)
    frames = {body_part}
    for c, (w, pts, moving) in corners.items():
        hosts = {}
        for n in moving:
            hosts[owner.get(n)] = hosts.get(owner.get(n), 0) + 1
        frames.add(max(hosts, key=hosts.get))
    kept = [n for n in nodes if n not in removed and n not in wheel_nodes and n in weights and owner.get(n) in frames]
    room = _room(model, configured, built, removed)
    built_w = {r[2]: r[3] for r in values.node_weights(*built)[0]}

    # everything that used a removed node, out (the active parts)
    for name in active:
        if name in where:
            _drop_rows(where[name][1], removed, members)

    # per corner: the archetype's nodes and beams, in the base's suspension part of that corner
    unsprung = {}
    for c, (w, pts, moving) in corners.items():
        unsprung[c] = sum(weights.get(n, 0.0) for n in moving) or 20.0
    names_of, host_of = {}, {}
    front_rack = []
    for c, (w, pts, moving) in sorted(corners.items()):
        hosts = {}
        for n in moving:
            hosts[owner.get(n)] = hosts.get(owner.get(n), 0) + 1
        host = max((h for h in hosts if h in where), key=lambda h: hosts[h], default=None)
        if not host:
            notes.append(f"{c}: its suspension part is not in the written files: kept as the base's")
            continue
        part = where[host][1]
        side = 1.0 if pts["wc"][0] > 0 else -1.0
        nm = {k: f"bf{c}{k}" for k in ("h1", "h2", "h3", "h4", "h5", "p0", "p1", "t", "tie")}
        names_of[c] = nm
        host_of[c] = part
        if pts["steer"]:
            ends = [n for n in rack_ends if nodes[n][0] * side > 0]
            if ends:
                end = min(ends, key=lambda n: math.dist(nodes[n], pts["tie"]))
                nm["tie"] = end
                notes.append(f"{c}: tie rod to the base's rack end {end}, {math.dist(nodes[end], pts['tie']) * 1000:.0f} mm "
                             "from the SVJ's tie rod inner point")
            else:
                front_rack.append((c, part, pts, nm, side))
        inner, outer = sorted((w["node1"], w["node2"]), key=lambda n: abs(nodes[n][0]))
        bottom = "h2" if "h2" in pts else "h1"
        nt = _table(part, "nodes", ["id", "posX", "posY", "posZ"])
        bt = _table(part, "beams", ["id1:", "id2:"])
        kg = unsprung[c]
        nt.append(dict(NODE_RESET, group=f"bf_hub_{c}"))
        hub = [k for k in ("h1", "h2", "h3", "h4", "h5") if k in pts]
        share = sum(HUB_SHARE[k] for k in hub)
        for k in hub:
            nt.append(_row(nt, {"id": nm[k], "posX": pts[k][0], "posY": pts[k][1], "posZ": pts[k][2],
                                "nodeWeight": round(kg * HUB_SHARE[k] / share, 2)}))
        nt.append(dict(NODE_RESET, group=f"bf_mounts_{c}"))
        body = [("p0", PIVOT_KG), ("p1", PIVOT_KG), ("t", TOP_KG)]
        if not pts["steer"]:
            body.append(("tie", PIVOT_KG))
        # a body node that stays and is already on the point (the fit put the base's pivot or strut
        # top there) is the archetype's: no node of its own, no mounts
        merged = []
        for k, _ in body:
            near = min((n for n in kept if n not in nm.values()), key=lambda n: math.dist(nodes[n], pts[k]))
            if math.dist(nodes[near], pts[k]) < MOUNT_NEAR:
                nm[k] = near
                merged.append(f"{k} {near}")
        body = [(k, m) for k, m in body if not nm[k] in nodes]
        if merged:
            notes.append(f"{c}: on the base's own nodes (already at the SVJ's points): " + ", ".join(merged))
        for k, m in body:
            nt.append(_row(nt, {"id": nm[k], "posX": pts[k][0], "posY": pts[k][1], "posZ": pts[k][2], "nodeWeight": m}))
        nt.append({"group": ""})
        # the hub: rigid, and on the wheel
        bt.append(dict(BEAM_RESET, beamSpring=HUB_K, beamDamp=100))
        for i, a in enumerate(hub):
            for b in hub[i + 1:]:
                bt.append([nm[a], nm[b]])
        bt.append(dict(BEAM_RESET, beamSpring=WHEEL_K, beamDamp=100, beamStrength=275000, breakGroup=f"wheel_{c}",
                       optional=True))
        ax = (axles or {}).get(c)

        def to_wheel(k, e, extra=None):
            """A hub-to-wheel beam: preloaded to the wheel's camber and toe when it spawns square (axles)."""
            o = dict(extra or {})
            if ax and e in ax["target"]:
                l0, l1 = math.dist(pts[k], nodes[e]), math.dist(pts[k], ax["target"][e])
                if l0 > 1e-6 and abs(l1 / l0 - 1) > 1e-5:
                    o.update(beamPrecompression=round(l1 / l0, 6), beamPrecompressionTime=0.5)
            return [nm[k], e, o] if o else [nm[k], e]
        bt.append(to_wheel("h1", inner, {"name": f"axle_{c}"}))
        for k in hub:                                  # the node across the wheel more softly (as the vanilla's)
            soft = {"beamSpring": WHEEL_SOFT_K} if k == "h5" else None
            if k != "h1":
                bt.append(to_wheel(k, inner, soft))
            bt.append(to_wheel(k, outer, soft))
        # the lower arm, the tie rod (rear: to its pivot), the mounts
        bt.append(dict(BEAM_RESET, beamSpring=ARM_K, beamDamp=1500, beamDeform=75000, beamStrength=450000))
        for k in ("p0", "p1"):
            bt.append([nm[k], nm["h1"], dict(_on_base(nm[k], ARM_K, 1500, room), dampCutoffHz=500)])
        bt.append(dict(BEAM_RESET, beamSpring=TIE_K, beamDamp=150, beamDeform=75500, beamStrength=127500))
        bt.append([nm["h3"], nm["tie"], _on_base(nm["tie"], TIE_K, 150, room)])
        if pts["steer"]:
            # steering dampers, as the vanilla's (the front-drive compact's fhub3-fsub2, fhub5-fsub2): the steered hub's toe
            # damped against the lower arm's pivots; without them the front wheels shimmy under braking
            bt.append(dict(BEAM_RESET, **BOUNDED, beamSpring=0, beamDamp=STEER_C, beamDeform=20000, beamStrength=35000,
                           beamLimitSpring=0, beamLimitDamp=0))
            for k in ("h3", "h5"):
                piv = min(("p0", "p1"), key=lambda q: math.dist(pts[k], pts[q]))
                bt.append([nm[k], nm[piv], {"beamDampFast": STEER_C_FAST, "beamDampVelocitySplit": 0.1, "dampCutoffHz": 750}])
        bt.append(dict(BEAM_RESET, beamSpring=MOUNT_K, beamDamp=MOUNT_C, beamDeform=25000, beamStrength=170000))
        for k, _ in body:
            for m in _mounts(pts[k], nodes, kept, weights, side, room):
                bt.append([nm[k], m])
        # the strut: its rail, spring, damper, bump stop and travel limit
        rails = part.get("rails") if isinstance(part.get("rails"), dict) else {}
        part["rails"] = rails
        rails[f"bf_strut_{c}"] = {"links:": [nm[bottom], nm["t"]], "broken:": [], "looped": False, "capped": True}
        st = _table(part, "slidenodes", ["id:", "railName", "attached", "fixToRail", "tolerance", "spring", "strength", "capStrength"])
        st.append(_row(st, {"id": nm["h4"], "railName": f"bf_strut_{c}", "attached": True, "fixToRail": True, "tolerance": 0.0,
                            "spring": SLIDE_K, "strength": "FLT_MAX", "capStrength": "FLT_MAX"}))
        tb = _table(part, "torsionbars", ["id1:", "id2:", "id3:", "id4:"])
        tb.append({"spring": 200000, "damp": 0.5, "deform": 25000, "strength": 100000})
        tb.append([outer, nm["h4"], nm["h1"], nm["h3"]])
        done.append(c)

    # the springs and dampers: the SVJ's, through the archetype's motion ratio; the dampers no lighter
    # than the base's damping ratio (as values.apply)
    for c in done:
        w, pts, _ = corners[c]
        nm, part = names_of[c], host_of[c]
        axle = "front" if c.startswith("F") else "rear"
        mr = motion_ratio(pts)
        sv = (values.svj_values(svj, None, {axle: {"spring": mr, "damper": mr}}).get(axle)) or {}
        base_s = [r for r in springs if r["axle"] == axle and r["kind"] == "spring"]
        base_d = [r for r in springs if r["axle"] == axle and r["kind"] == "damper"]
        k = sv.get("coil_rate") or _num((base_s[0]["values"] if base_s else {}).get("beamSpring"), 30000)
        kd0 = _num((base_s[0]["values"] if base_s else {}).get("beamSpring"), k)
        floor = math.sqrt(k / kd0) if kd0 else 1.0
        dv = base_d[0]["values"] if base_d else {}
        def damp(key, svkey, default):
            b = _num(dv.get(key), default) * floor
            return round(max(sv.get(svkey) or 0, b))
        bump, rebound = damp("beamDamp", "damp_bump", 2500), damp("beamDampRebound", "damp_rebound", 5000)
        bump_f = damp("beamDampFast", "damp_bump_fast", bump * 0.4)
        rebound_f = damp("beamDampReboundFast", "damp_rebound_fast", rebound * 0.5)
        split = sv.get("damp_split") or _num(dv.get("beamDampVelocitySplit"), 0.1)
        # the preload: the corner's sprung weight on the spring at the SVJ's ride height
        load = (loads or {}).get(c, 0.0) - unsprung[c] * 9.81
        pre = round(max(0.002, load / mr / k), 4)
        bottom = "h2" if "h2" in pts else "h1"
        bt = part["beams"]
        bt.append(dict(BEAM_RESET, **BOUNDED, beamSpring=k, beamDamp=0, beamDeform=17000, beamStrength=180000,
                       beamLimitSpring=49000, beamLimitDamp=500))
        bt.append([nm["h4"], nm["t"], {"name": f"spring_{c}", "precompressionRange": pre, "longBoundRange": 1,
                                       "shortBoundRange": 0.1, "boundZone": 0.08, "beamLimitDampRebound": 0,
                                       "dampCutoffHz": 500}])
        bt.append(dict(BEAM_RESET, **BOUNDED, beamSpring=0, beamDamp=bump, beamDeform=17000, beamStrength=180000,
                       beamLimitSpring=0, beamLimitDamp=0))
        bt.append([nm[bottom], nm["t"], {"name": f"damper_{c}", "beamDampFast": bump_f, "beamDampRebound": rebound,
                                          "beamDampReboundFast": rebound_f, "beamDampVelocitySplit": split,
                                          "dampCutoffHz": 500}])
        bt.append(dict(BEAM_RESET, **BOUNDED, beamSpring=0, beamDamp=0, beamLimitSpring=181000, beamLimitDamp=5000))
        bt.append([nm["h4"], nm["t"], {"longBoundRange": 0.08, "shortBoundRange": 0.1, "boundZone": 0.04,
                                        "beamLimitDampRebound": 0, "dampCutoffHz": 500}])
        bt.append(dict(BEAM_RESET, **BOUNDED, beamSpring=0, beamDamp=0, beamLimitSpring=1001000, beamLimitDamp=1000,
                       beamDeform=15000, beamStrength=150000))
        bt.append([nm["h1"], nm["t"], {"longBoundRange": 0.12, "shortBoundRange": 0.12, "boundZone": 0.025,
                                        "beamLimitDampRebound": 0, "dampCutoffHz": 500}])
        bt.append(dict(BEAM_RESET))
        notes.append(f"{c}: strut archetype (spring {k:.0f} N/m, motion ratio {mr}, preload {pre * 1000:.0f} mm; "
                     f"damper bump {bump}, rebound {rebound})")

    # the steering rack (front): its ends on the SVJ's tie rod inner points, a rail between them, two
    # slide nodes mounted to the body, steered by hydros as the base's
    if len(front_rack) == 2:
        (cl, part, pl, nl, _), (cr, _, pr, nr, _) = sorted(front_rack, key=lambda x: -x[4])   # left (+x) first
        el, er = pl["tie"], pr["tie"]
        across = _unit(_sub(er, el))
        sl, sr = _add(el, across, RACK_INSET), _add(er, across, -RACK_INSET)
        names = {"el": nl["tie"], "er": nr["tie"], "sl": "bfrack_sl", "sr": "bfrack_sr"}
        nt, bt = part["nodes"], part["beams"]
        nt.append(dict(NODE_RESET, group="bf_rack"))
        for k, p in (("el", el), ("er", er), ("sl", sl), ("sr", sr)):
            nt.append(_row(nt, {"id": names[k], "posX": p[0], "posY": p[1], "posZ": p[2], "nodeWeight": RACK_KG}))
        nt.append({"group": ""})
        part.setdefault("rails", {})["bf_rack"] = {"links:": [names["er"], names["el"]], "broken:": [], "looped": False,
                                                    "capped": True}
        st = part["slidenodes"]
        for k in ("sl", "sr"):
            st.append(_row(st, {"id": names[k], "railName": "bf_rack", "attached": True, "fixToRail": True, "tolerance": 0.0,
                                "spring": 15001000, "strength": "FLT_MAX", "capStrength": "FLT_MAX"}))
        bt.append(dict(BEAM_RESET, beamSpring=RACK_K, beamDamp=150, beamDeform=79500, beamStrength=142500))
        bt.append([names["er"], names["el"]])
        bt.append(dict(BEAM_RESET, beamSpring=MOUNT_K, beamDamp=MOUNT_C, beamDeform=25000, beamStrength=170000))
        for k, p, s in (("sl", sl, 1.0), ("sr", sr, -1.0)):
            for m in _mounts(p, nodes, kept, weights, s, room):
                bt.append([names[k], m])
        ht = _table(part, "hydros", ["id1:", "id2:"])
        ln = math.dist(er, sl)
        f = round(rack["factor"] * rack["length"] / ln, 4) if rack else 0.135
        lock = rack["lock"] if rack else 450
        ht.append(dict(BEAM_RESET, beamSpring=RACK_K, beamDamp=50, beamDeform="FLT_MAX", beamStrength=92500))
        ht.append([names["er"], names["sl"], {"factor": f, "steeringWheelLock": lock, "inRate": 1.25, "outRate": 1.25}])
        ht.append([names["el"], names["sr"], {"factor": -f, "steeringWheelLock": lock, "inRate": 1.25, "outRate": 1.25}])
        ht.append(dict(BEAM_RESET))
        notes.append(f"steering rack on the SVJ's tie rod inner points (factor {f}, lock {lock})")
    elif front_rack:
        notes.append("only one front corner is a strut archetype: no steering rack written")

    # the weight the mounts need on the body nodes, in their rows
    if room["kg"]:
        for n, kg in room["kg"].items():
            part = (where.get(owner.get(n)) or (None, None))[1]
            if part is None or not _add_weight(part, n, kg, built_w.get(n, weights.get(n, 0.0))):
                notes.append(f"{n}: its row was not found to add {kg:.2f} kg (its mounts may be too stiff for it)")
        notes.append(f"{sum(room['kg'].values()):.1f} kg added to {len(room['kg'])} body nodes so their mounts keep "
                     "within the physics step's limits")

    # the wheels: arm, torque and steering axis nodes on the archetype's hubs
    for name in active:
        part = (where.get(name) or (None, None))[1]
        if part and isinstance(part.get("pressureWheels"), list):
            for c in done:
                _remap_wheels(part, c, names_of[c])

    if front_rack and steer_parts:
        notes.append("the base's steering (" + ", ".join(n for n, _, _ in steer_parts) + ") has no rack ends on one side: "
                     "the archetype's rack written")
    notes.append("anti-roll bars: the base's go with the hubs; none written yet")
    for p, doc in docs.items():
        files[p] = json.dumps(doc, indent=1)
    return files, notes, done


def _add_weight(part, node, kg, current):
    """The node's row in its part given kg more (its weight now: current)."""
    t = part.get("nodes")
    if not isinstance(t, list) or not t or not isinstance(t[0], list):
        return False
    head = _head(t)
    for row in t[1:]:
        if isinstance(row, list) and "id" in head and len(row) > head.index("id") and row[head.index("id")] == node:
            if isinstance(row[-1], dict):
                row[-1]["nodeWeight"] = round(current + kg, 3)
            else:
                row.append({"nodeWeight": round(current + kg, 3)})
            return True
    return False


def _num(v, default):
    return v if isinstance(v, (int, float)) else default


def _walk_tree(node, out=None):
    out = [] if out is None else out
    if node.get("part"):
        out.append((node["part"], node.get("slot")))
    for ch in node.get("children") or []:
        _walk_tree(ch, out)
    return out


def corner_loads(mass, cg_behind_front_axle, wheelbase):
    """{corner: static wheel load, N}: the car's mass split by its CG between the axles, then the sides."""
    front = mass * (1 - cg_behind_front_axle / wheelbase)
    f, r = front / 2 * 9.81, (mass - front) / 2 * 9.81
    return {"FL": f, "FR": f, "RL": r, "RR": r}
