"""SVJ v0.100+ unified suspension read as BeamForge's per-corner view; movable parts; half shafts.

Pure Python, standard library only (runs in Pyodide).

Since SVJ v0.100 (spec section 25) a suspension is bodies, joints and elements (`chassis.mass_bodies`, `constraints`)
plus a thin `wheel_stations[]` list; the corner-based `suspension` section is deprecated. BeamForge's readers (the
fit, the suspension study, the values, the dampers) were written for the corner form, so a unified file is given a
derived `suspension` section here, the inverse of the SVJ repository's tools/migrate_suspension.py:

  wheel station          -> a corner keyed by the station id (FL, A2R...)
  carrier's markers      -> the upright's hardpoints (on a shared axle body, the markers suffixed with the station)
  bodies jointed to the  -> links: inboard points are the frames of the link's other joints (chassis, rack...), the
  carrier                   outboard point the joint on the carrier, named by the carrier marker at that place
  one revolute inboard   -> two inboard points on the hinge axis: the second is recovered from the link body's
  joint                     position (a generated link body sits midway between its inboard mean and outboard point)
  spring_<st>, damper_<st> -> spring and damper, back at the wheel when the migration converted them by a motion ratio
  anti_roll_bars[]       -> the corner's arb
  topology_hint          -> topology.system_type

The view is read-only: nothing writes it back to a file. Bodies a station does not hold (doors, hood, trunk, wipers)
are movable parts (movable_parts); half shafts are parts with two ends (half_shafts), placed with
link_between_points (spec section 22.6) when the file binds them that way.
"""
import math
import re

STATION_SFX = re.compile(r"_(f[lr]|r[lr]|a\d+[lrc])$", re.I)
SHAFT = re.compile(r"^(half_?shaft|axle_shaft|cv_shaft|drive_?shaft_(?=f[lr]|r[lr]|a\d))", re.I)
ELEMENT_KEYS = ("id", "type", "body_a", "body_b", "frame_a", "frame_b", "marker_a", "marker_b", "x_derived",
                "x_outboard_body_inferred", "x_converted_from_wheel_rate", "reference", "kind", "spring_type", "x_damper_type")
STATION_KEYS = ("id", "name", "wheel_body", "upright_body", "topology_hint", "axle", "side", "wheel_center_marker",
                "steering_joint", "coupling_ref")
SAME = 1e-6            # m: two frames at the same place
SECOND_POINT = 0.25    # m: along the hinge axis, when a link's second inboard point cannot be recovered

# movable parts by their body id (docs/naming_convention.md and the names converters write)
MOVABLE_KINDS = (("door", re.compile(r"door", re.I)), ("hood", re.compile(r"hood|bonnet", re.I)),
                 ("trunk", re.compile(r"trunk|tailgate|hatch|boot|decklid|rear_?lid", re.I)),
                 ("mirror", re.compile(r"mirror", re.I)), ("wiper", re.compile(r"wiper", re.I)),
                 ("window", re.compile(r"window|glass", re.I)), ("spoiler", re.compile(r"spoiler|wing", re.I)),
                 ("light", re.compile(r"light|lamp", re.I)))


def is_unified(doc):
    """True for a file with wheel stations and no corner section of its own."""
    return bool(isinstance(doc, dict) and isinstance(doc.get("wheel_stations"), list) and doc["wheel_stations"]
                and not isinstance(doc.get("suspension"), dict))


def _slug(s):
    return re.sub(r"[^a-z0-9_]+", "_", str(s).strip().lower()).strip("_")


def _pos(v):
    if isinstance(v, dict):
        v = v.get("position")
    return [float(x) for x in v] if isinstance(v, (list, tuple)) and len(v) == 3 and all(isinstance(x, (int, float)) for x in v) else None


def _same(a, b, tol=SAME):
    return a is not None and b is not None and all(abs(x - y) <= tol for x, y in zip(a, b))


def quat_z(q):
    """The +Z axis of the frame with unit quaternion [x, y, z, w] (a joint's axis, spec section 24.3)."""
    if not (isinstance(q, (list, tuple)) and len(q) == 4):
        return [0.0, 0.0, 1.0]
    x, y, z, w = (float(c) for c in q)
    return [2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)]


class _Model:
    """The bodies, markers and joints of a file, with the frame of each joint end resolved."""

    def __init__(self, doc):
        ch = doc.get("chassis") if isinstance(doc.get("chassis"), dict) else {}
        self.chassis = ch
        self.bodies = {b["id"]: b for b in ch.get("mass_bodies") or [] if isinstance(b, dict) and b.get("id")}
        self.cons = [c for c in doc.get("constraints") or [] if isinstance(c, dict)]
        self.joints = [c for c in self.cons if c.get("type") == "joint" and c.get("body_a") and c.get("body_b")]

    def marker(self, body, name):
        src = self.chassis.get("markers") if body == "chassis" else (self.bodies.get(body) or {}).get("markers")
        return (src or {}).get(name) if isinstance(src, dict) else None

    def frame(self, c, side):
        """(position, orientation) of a constraint's end "a" or "b": its inline frame, else its marker."""
        f = c.get(f"frame_{side}")
        if not (isinstance(f, dict) and _pos(f)) and c.get(f"marker_{side}"):
            f = self.marker(c.get(f"body_{side}"), c[f"marker_{side}"])
        if not _pos(f):                                    # the two ends coincide at the reference pose
            other = "b" if side == "a" else "a"
            f = c.get(f"frame_{other}")
            if not (isinstance(f, dict) and _pos(f)) and c.get(f"marker_{other}"):
                f = self.marker(c.get(f"body_{other}"), c[f"marker_{other}"])
        return _pos(f), (f.get("orientation") if isinstance(f, dict) else None)

    def at(self, body):
        return [j for j in self.joints if body in (j["body_a"], j["body_b"])]


def link_ends(doc, body_id, model=None):
    """A link body's two ends, spec section 22.6 (v0.100): (inboard points, outboard point) in SAE, the inboard points
    the frames of the joints where it is body_b, the outboard point the frame of a joint where it is body_a;
    (None, None) when it has not both."""
    m = model or _Model(doc)
    ins = [m.frame(j, "b")[0] for j in m.joints if j["body_b"] == body_id]
    outs = [m.frame(j, "a")[0] for j in m.joints if j["body_a"] == body_id]
    ins = [p for p in ins if p]
    outs = [p for p in outs if p]
    return (ins, outs[0]) if ins and outs else (None, None)


def _second_point(body, p0, axis, outer):
    """The other inboard point of a two-bush arm the migration wrote as one revolute joint, or None. The generated
    body sits midway between its inboard mean and its outboard point (migrate_suspension), so the mean is
    2 cg - outer and the other bush 2 mean - p0; it is taken only when it lies on the hinge axis."""
    cg = _pos((body or {}).get("position"))
    if not cg or not outer:
        return None
    mean = [2 * c - o for c, o in zip(cg, outer)]
    p1 = [2 * m - p for m, p in zip(mean, p0)]
    d = [b - a for a, b in zip(p0, p1)]
    n = math.sqrt(sum(x * x for x in d))
    if n < 0.02:
        return None
    a = math.sqrt(sum(x * x for x in axis)) or 1.0
    cross = [d[1] * axis[2] - d[2] * axis[1], d[2] * axis[0] - d[0] * axis[2], d[0] * axis[1] - d[1] * axis[0]]
    return [round(x, 6) for x in p1] if math.sqrt(sum(x * x for x in cross)) / (n * a) < 2e-3 else None


def _estimate_mr(m, inner, outer, body, carrier, wc):
    """First-order motion ratio of an element from `inner` (chassis) to `outer` on `body`: its length change per unit
    of vertical wheel travel. A mount on the carrier moves with the wheel; one on a link turns about the link's hinge
    (its revolute joint to the chassis). None when it cannot be told."""
    d = [b - a for a, b in zip(inner, outer)]
    n = math.sqrt(sum(x * x for x in d))
    if n < 1e-9 or not wc:
        return None
    e = [x / n for x in d]
    if body == carrier:
        mr = abs(e[2])
    else:
        j = next((j for j in m.at(body) if j.get("joint_type") == "revolute" and carrier not in (j["body_a"], j["body_b"])), None)
        if j is None:
            return None
        p0, q = m.frame(j, "a")
        ax = quat_z(q)
        cross = lambda a, b: [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]  # noqa: E731
        vm = cross(ax, [o - p for o, p in zip(outer, p0)])
        vw = cross(ax, [w - p for w, p in zip(wc, p0)])
        if abs(vw[2]) < 1e-9:
            return None
        mr = abs(sum(a * b for a, b in zip(vm, e)) / vw[2])
    return round(mr, 4) if 0.05 <= mr <= 2.0 else None


def _curve_back(curve, mr):
    """A curve the migration took from the wheel to the element ((x mr, y / mr)) back at the wheel."""
    return [[round(p[0] / mr, 6), round(p[1] * mr, 6)] for p in curve if isinstance(p, (list, tuple)) and len(p) == 2]


def _element_view(m, c, kind, notes, sid):
    """A spring or damper constraint as a corner's spring / damper object (rates and curves at the wheel when the
    migration converted them by a motion ratio, which the object keeps as motion_ratio)."""
    out = {k: v for k, v in c.items() if k not in ELEMENT_KEYS}
    conv = c.get("x_converted_from_wheel_rate") if isinstance(c.get("x_converted_from_wheel_rate"), dict) else {}
    mr = conv.get("motion_ratio") if isinstance(conv.get("motion_ratio"), (int, float)) and conv["motion_ratio"] > 0 else None
    if kind == "spring":
        out["type"] = c.get("spring_type") or ""
        if c.get("kind") == "torsional" and not out["type"]:
            out["type"] = "torsion_bar"
    elif c.get("x_damper_type"):
        out["type"] = c["x_damper_type"]
    if c.get("reference") == "wheel":                       # a wheel-level element: values as quoted, no mounts
        if kind == "damper":
            out["reference"] = "wheel"
        return out
    out["inboard_mount"], out["outboard_mount"] = m.frame(c, "a")[0], m.frame(c, "b")[0]
    if kind == "spring":
        if mr:
            if isinstance(c.get("rate"), (int, float)):
                out["rate"] = round(c["rate"] * mr * mr, 4)
            if isinstance(c.get("rate_curve"), list):
                out["rate_curve"] = _curve_back(c["rate_curve"], mr)
            if isinstance(c.get("preload"), (int, float)):
                out["preload"] = round(c["preload"] * mr, 4)
            out["motion_ratio"] = mr
        elif isinstance(c.get("rate"), (int, float)) or c.get("rate_curve"):
            out["x_rate_reference"] = "element"
            notes.append(f"{sid}: the spring's rate is the element's own (no motion ratio given); "
                         "values read as wheel rates use it as it is")
    else:
        if mr:
            for k in ("bump_curve", "rebound_curve"):
                if isinstance(c.get(k), list):
                    out[k] = _curve_back(c[k], mr)
            if isinstance(c.get("damping"), (int, float)):
                out["damping"] = round(c["damping"] * mr * mr, 4)
            out["reference"], out["motion_ratio"] = "wheel", mr
        else:
            out["reference"] = "damper"
    return out


def corner_view(doc):
    """The per-corner `suspension` section of a unified file (spec section 25), as BeamForge's readers know it:
    ({station id: corner}, notes). Corners hold topology (system_type, upright with hardpoints, links, axle_body),
    spring, damper, arb, wheel, tire, brake, alignment and the station's other data. See the module docstring."""
    m = _Model(doc)
    stations = [s for s in doc.get("wheel_stations") or [] if isinstance(s, dict) and s.get("id")]
    out, notes = {}, []
    carriers = {}
    for st in stations:
        ub = st.get("upright_body")
        if not ub and st.get("wheel_body"):                  # the carrier the wheel is hinged to
            j = next((j for j in m.at(st["wheel_body"]) if j.get("joint_type") == "revolute"), None)
            ub = (j["body_a"] if j["body_b"] == st["wheel_body"] else j["body_b"]) if j else None
        carriers[st["id"]] = ub
    shared = {ub for ub in carriers.values() if ub and list(carriers.values()).count(ub) > 1}
    wheels = {s.get("wheel_body") for s in stations if s.get("wheel_body")}
    bars = [b for b in doc.get("anti_roll_bars") or [] if isinstance(b, dict)]
    used = set()                                             # spring / damper ids already given to a station
    for st in stations:
        sid, sfx, ub, wb = st["id"], _slug(st["id"]), carriers[st["id"]], st.get("wheel_body")
        if not ub or ub not in m.bodies:
            notes.append(f"{sid}: no wheel carrier body (upright_body); corner left out")
            continue
        up = m.bodies[ub]
        axle = ub in shared
        hps = {}
        for k, mk in (up.get("markers") or {}).items():
            p = _pos(mk)
            if not p:
                continue
            if k.lower().endswith("_" + sfx):
                hps[k[:-len(sfx) - 1]] = p
            elif not (axle and STATION_SFX.search(k)):       # (on a shared axle: not the other station's)
                hps.setdefault(k, p)
        wc = _pos(m.marker(ub, st.get("wheel_center_marker") or "wheel_center"))
        if not wc and wb in m.bodies:
            wc = _pos(m.bodies[wb].get("position"))
            if wc:
                notes.append(f"{sid}: no wheel centre marker on {ub}; the wheel body's position is used")
        if wc:
            hps["wheel_center"] = wc

        def ref_of(p, base):
            name = next((k for k, q in hps.items() if _same(q, p)), None)
            if name is None:
                name = f"{base}_outer"
                hps[name] = p
            return name

        links, held = [], set()
        for j in m.at(ub):
            other = j["body_a"] if j["body_b"] == ub else j["body_b"]
            if other in wheels or other == "ground" or other in held:
                continue
            if axle:                                         # a shared axle: this station's links only
                own = STATION_SFX.search(other)
                if own and own.group(1).lower() != sfx:
                    continue
                p = m.frame(j, "a")[0]
                if not own and wc and p and p[1] * wc[1] < 0:
                    continue                                 # (an unsuffixed link on the other side)
            if other == "chassis":                           # the carrier hinged straight to the chassis (a trailing arm)
                if j.get("joint_type") == "revolute":
                    p0, q = m.frame(j, "a")
                    ax = quat_z(q)
                    links.append({"name": STATION_SFX.sub("", ub), "type": "pivot", "joint_type": "revolute",
                                  "inboard_points": [p0, [p0[i] + SECOND_POINT * ax[i] for i in range(3)]],
                                  "outboard_ref": "hardpoints.wheel_center"})
                else:
                    notes.append(f"{sid}: {ub} is joined to the chassis by a {j.get('joint_type')} joint; not read as a link")
                continue
            body = m.bodies.get(other)
            if body is None:
                continue
            base = STATION_SFX.sub("", other)
            if SHAFT.search(base):
                continue                                     # a half shaft drives the wheel, it does not locate it
            held.add(other)
            outer = m.frame(j, "a")[0]
            if not outer:
                notes.append(f"{sid} {base}: its joint on the carrier has no frame; left out")
                continue
            inner = [k for k in m.at(other) if ub not in (k["body_a"], k["body_b"])]
            if not inner:
                notes.append(f"{sid} {base}: joined to the carrier only; left out")
                continue
            pts, types, bushings = [], [], []
            for k in inner:
                p, q = m.frame(k, "a")
                if not p:
                    continue
                pts.append(p)
                types.append(k.get("joint_type"))
                if isinstance(k.get("bushing"), dict):
                    bushings.append(k["bushing"])
            if len(pts) == 1 and types[0] == "revolute":    # one hinge: its second point on the axis
                q = m.frame(inner[0], "a")[1]
                ax = quat_z(q)
                p1 = _second_point(body, pts[0], ax, outer)
                if p1 is None:
                    p1 = [round(pts[0][i] + SECOND_POINT * ax[i], 6) for i in range(3)]
                    notes.append(f"{sid} {base}: one hinge inboard; its second point is put {SECOND_POINT * 1000:.0f} mm along the axis")
                pts.append(p1)
            jt = j.get("joint_type")
            kind = "strut" if jt in ("translational", "cylindrical") or re.fullmatch(r"(macpherson_)?strut", base) else \
                "arm" if len(pts) >= 2 else "rod"
            link = {"name": base, "type": kind, "inboard_points": pts, "outboard_ref": "hardpoints." + ref_of(outer, base),
                    "joint_type": jt, "inboard_joint_type": types[0] if types else None, "body_ref": other}
            for k in ("mass", "inertia", "visual"):
                if body.get(k) is not None:
                    link[k] = body[k]
            if _pos(body.get("position")):
                link["cg_position"] = _pos(body["position"])
            if bushings:
                link["bushings"] = bushings
            links.append(link)

        upright = {"id": ub, "body_ref": ub, "hardpoints": hps}
        topo = {"system_type": st.get("topology_hint") or ("solid_axle" if axle else ""), "upright": upright, "links": links}
        carrier = {k: up[k] for k in ("mass", "inertia", "visual") if up.get(k) is not None}
        if axle:
            topo["axle_body"] = dict(carrier, id=ub, position=_pos(up.get("position")) or wc,
                                     **(up.get("x_legacy_axle_body") if isinstance(up.get("x_legacy_axle_body"), dict) else {}))
        else:
            upright.update(carrier)
            if _pos(up.get("position")):
                upright["cg_position"] = _pos(up["position"])
        if st.get("x_lateral_location") is not None:
            topo["lateral_location"] = st["x_lateral_location"]
        corner = {k: v for k, v in st.items() if k not in STATION_KEYS and k != "x_lateral_location"}
        corner["topology"] = topo
        wheel = dict(st.get("wheel") or {})
        wbody = m.bodies.get(wb) or {}
        if wbody.get("mass") and not wheel.get("mass"):
            wheel["mass"] = wbody["mass"]
        if wbody.get("visual") and not wheel.get("visual"):
            wheel["visual"] = wbody["visual"]
        iy = (wbody.get("inertia") or {}).get("Iyy") if isinstance(wbody.get("inertia"), dict) else None
        if isinstance(iy, (int, float)) and "rotational_inertia" not in wheel:
            wheel["rotational_inertia"] = iy
        corner["wheel"] = wheel

        mine = {ub} | held
        for kind in ("spring", "damper"):
            c = next((c for c in m.cons if c.get("type") == kind and c.get("id") == f"{kind}_{sfx}"), None)
            if c is None:                                    # an element on the station's bodies, nearest its wheel
                cand = [c for c in m.cons if c.get("type") == kind and c.get("id") not in used and c.get("body_b") in mine
                        and not re.match(r"(bump|rebound)_stop", str(c.get("id"))) and c.get("kind") != "torsional"]
                if axle and wc:                              # a shared axle: the one on this side
                    cand = [c for c in cand if (m.frame(c, "b")[0] or [0, 0, 0])[1] * wc[1] > 0]
                c = min(cand, key=lambda c: math.dist(m.frame(c, "b")[0] or [1e9] * 3, wc or [0, 0, 0])) if cand else None
            if c is not None:
                used.add(c.get("id"))
                corner[kind] = _element_view(m, c, kind, notes, sid)
            else:
                corner[kind] = {}
        dm, sp = corner["damper"], corner["spring"]
        if dm.get("reference") == "damper" and not dm.get("motion_ratio") and dm.get("outboard_mount"):
            # the migration keeps no motion ratio for a damper quoted at the damper: a spring on the same mounts has it,
            # else it is estimated to first order (as the migration estimates one)
            same_mounts = _same(sp.get("inboard_mount"), dm.get("inboard_mount"), 1e-3) and \
                _same(sp.get("outboard_mount"), dm.get("outboard_mount"), 1e-3)
            if sp.get("motion_ratio") and same_mounts:
                dm["motion_ratio"] = sp["motion_ratio"]
            else:
                c = next((c for c in m.cons if c.get("id") in used and c.get("type") == "damper"
                          and _same(m.frame(c, "b")[0], dm["outboard_mount"])), None)
                mr = _estimate_mr(m, dm["inboard_mount"], dm["outboard_mount"], c.get("body_b") if c else ub, ub, wc)
                if mr:
                    dm["motion_ratio"], dm["x_motion_ratio_estimated"] = mr, True
                    notes.append(f"{sid}: the damper's motion ratio is estimated from the geometry ({mr:.2f})")
        bar = next((b for b in bars if sid in (b.get("stations") or [])), None)
        if bar:
            arb = {"bar_id": bar.get("id")}
            for k in ("bar_rate", "wheel_rate_equiv", "motion_ratio", "mass", "visual"):
                if bar.get(k) is not None:
                    arb[k] = bar[k]
            dl = (bar.get("drop_link_mounts") or {}).get(sid) if isinstance(bar.get("drop_link_mounts"), dict) else None
            if _pos(dl):
                arb["drop_link_mount"] = _pos(dl)
            corner["arb"] = arb
        if st.get("steering_joint"):
            corner["x_steering_joint"] = st["steering_joint"]
        out[sid] = corner
    return out, notes


def adopt(doc):
    """Give a unified file (is_unified) its derived corner section, in place; returns the notes. The section is marked
    x_beamforge_corner_view (a derived view, never written back). Any other file is left as it is."""
    if not is_unified(doc):
        return []
    view, notes = corner_view(doc)
    doc["suspension"] = view
    doc["x_beamforge_corner_view"] = {"from": "wheel_stations", "notes": notes}
    return notes


def suspension_bodies(doc):
    """The ids of the bodies a unified file's suspension is made of: wheels, carriers, the links on them, the bodies
    those hang from (a steering rack) and half shafts. Empty for a corner-based file."""
    if not (isinstance(doc, dict) and isinstance(doc.get("wheel_stations"), list)):
        return set()
    m = _Model(doc)
    ids = set()
    for st in doc["wheel_stations"]:
        if isinstance(st, dict):
            ids.update(x for x in (st.get("wheel_body"), st.get("upright_body")) if x)
    links = set()
    for ub in list(ids):
        for j in m.at(ub):
            other = j["body_a"] if j["body_b"] == ub else j["body_b"]
            if other in m.bodies and other not in ids:
                links.add(other)
    ids |= links
    for link in links:                                       # what the links hang from (a tie rod's rack)
        for j in m.at(link):
            other = j["body_a"] if j["body_b"] == link else j["body_b"]
            if other in m.bodies:
                ids.add(other)
    ids.update(b for b in m.bodies if SHAFT.search(STATION_SFX.sub("", b)) or re.fullmatch(r"rack(_\w+)?", b))
    return ids


def movable_parts(doc):
    """The parts a file hinges or slides on the vehicle that are not its suspension (doors, hood, trunk, wipers...,
    spec section 24): [{"id", "name", "kind", "joint", "joint_type", "parent", "hinge" (SAE point), "axis" (SAE unit
    vector, the joint's +Z), "range" [min, max] or None, "visual" (glTF node) or None, "mesh_ref", "poses" (the ids
    of the poses that move it)}]. kind: door, hood, trunk, mirror, wiper, window, spoiler, light, or "part"."""
    m = _Model(doc)
    skip = suspension_bodies(doc)
    poses = [p for p in doc.get("poses") or [] if isinstance(p, dict)]
    out = []
    for bid, b in m.bodies.items():
        if bid in skip:
            continue
        j = next((j for j in m.joints if j["body_b"] == bid and j.get("joint_type") != "fixed"), None)
        if j is None:
            continue
        p, q = m.frame(j, "a")
        if not p:
            continue
        rng = j.get("range") if isinstance(j.get("range"), dict) else {}
        lo, hi = rng.get("min"), rng.get("max")
        vis = b.get("visual") if isinstance(b.get("visual"), dict) else {}
        out.append({"id": bid, "name": b.get("name") or bid,
                    "kind": next((k for k, rx in MOVABLE_KINDS if rx.search(bid)), "part"),
                    "joint": j.get("id"), "joint_type": j.get("joint_type"), "parent": j["body_a"],
                    "hinge": p, "axis": quat_z(q),
                    "range": [lo, hi] if isinstance(lo, (int, float)) and isinstance(hi, (int, float)) else None,
                    "visual": vis.get("node"), "mesh_ref": vis.get("mesh_ref"),
                    "poses": [x["id"] for x in poses if j.get("id") in (x.get("values") or {})]})
    return out


def half_shafts(doc):
    """The half shafts a file describes, with their two ends: [{"corner", "id", "inboard" [SAE points],
    "outboard" (SAE point), "visual" (the binding object) or None}]. From half-shaft bodies (canonical name
    half_shaft_<station>, ends by link_ends), else powertrain.half_shafts (cv_inner, cv_outer)."""
    m = _Model(doc)
    by_sfx = {_slug(s["id"]): s["id"] for s in doc.get("wheel_stations") or [] if isinstance(s, dict) and s.get("id")}
    out, seen = [], set()
    for bid, b in m.bodies.items():
        if not SHAFT.search(STATION_SFX.sub("", bid)):
            continue
        sfx = STATION_SFX.search(bid)
        corner = by_sfx.get(sfx.group(1).lower(), sfx.group(1).upper()) if sfx else None
        ins, outer = link_ends(doc, bid, m)
        if not ins and corner:
            h = ((doc.get("powertrain") or {}).get("half_shafts") or {}).get(corner) or {}
            ci, co = _pos((h.get("cv_inner") or {}).get("position")), _pos((h.get("cv_outer") or {}).get("position"))
            ins, outer = ([ci], co) if ci and co else (None, None)
        if corner and ins:
            out.append({"corner": corner, "id": bid, "inboard": ins, "outboard": outer,
                        "visual": b.get("visual") if isinstance(b.get("visual"), dict) else None})
            seen.add(corner)
    for corner, h in ((doc.get("powertrain") or {}).get("half_shafts") or {}).items():
        if corner in seen or not isinstance(h, dict):
            continue
        ci, co = _pos((h.get("cv_inner") or {}).get("position")), _pos((h.get("cv_outer") or {}).get("position"))
        if ci and co:
            out.append({"corner": corner, "id": f"half_shaft_{_slug(corner)}", "inboard": [ci], "outboard": co,
                        "visual": h.get("visual") if isinstance(h.get("visual"), dict) else None})
    return out
