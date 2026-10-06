"""A base vehicle's suspension corner converted into an SVJ corner, by role (roles.py).

Pure Python, standard library only (runs in Pyodide). With the base's roles known, each corner is
converted part by part, from the wheel inwards:

  upright   kept rigid (its shape is the base's) and placed by the SVJ: turned so the wheel's axis is
            the SVJ's (static camber and toe), then about that axis so its steering axis (lower ball
            joint to strut top, or to the upper ball joint) comes as close to the SVJ's as it can
            (caster; the kingpin inclination stays the upright's own), then moved so the wheel centre
            is the SVJ's
  strut     its top on the upright's own rail line (so the slide node stays on it), at the height of
            the SVJ's strut top
  links     each link's inner pivots at the SVJ's pivots of the same link, measured from where the
            link's outer joint landed on the placed upright (the link vector is the SVJ's)

Pivots with no SVJ counterpart (a trailing arm against an SVJ without one) are left to the
displacement field, with the rest of the body.
"""

import copy
import math

from . import suspension, svj as svjmod

# SVJ link names and upright points -> roles
LINK_ROLE = (("semi_trailing", "semi_trailing_arm"), ("trailing", "trailing_arm"), ("upper", "upper_arm"),
             ("lower", "lower_arm"), ("tie_rod", "tie_rod"), ("toe", "tie_rod"), ("strut", "strut"))
JOINT_OF = {"lower_arm": "lower_ball_joint", "upper_arm": "upper_ball_joint", "tie_rod": "tie_rod_end"}


def _role(name):
    n = name.lower()
    return next((r for key, r in LINK_ROLE if key in n), None)


def _svj_points(svj, corner, hps):
    """The SVJ corner by role: ({upright point role: [x, y, z]}, {link role: [inner points]}), BeamNG
    coordinates (hps: svj.hardpoints() rows of this corner)."""
    by_name = {h["name"]: h["pos"] for h in hps}
    up = {}
    for name, pos in by_name.items():
        if "." in name:
            continue
        n = name.lower()
        if n == "wheel_center":
            up["wheel_center"] = pos
        elif "upper_ball" in n:
            up["upper_ball_joint"] = pos
        elif "lower_ball" in n:
            up["lower_ball_joint"] = pos
        elif "tie_rod_end" in n or "toe_link_outer" in n:
            up.setdefault("tie_rod_end", pos)
    links = {}
    topo = ((svj.get("suspension") or {}).get(corner) or {}).get("topology") or {}
    for link in topo.get("links") or []:
        role = _role(link.get("name", ""))
        pts = [by_name[f"{link.get('name')}.{i}"] for i in range(len(link.get("inboard_points") or []))
               if f"{link.get('name')}.{i}" in by_name]
        if role and pts:
            links.setdefault(role, []).extend(pts)
    return up, links


def _unit(v):
    n = math.sqrt(sum(x * x for x in v))
    return [x / n for x in v] if n > 1e-12 else None


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def _rot(axis, ang):
    """Rotation matrix about a unit axis (Rodrigues)."""
    x, y, z = axis
    c, s, t = math.cos(ang), math.sin(ang), 1 - math.cos(ang)
    return [[c + x * x * t, x * y * t - z * s, x * z * t + y * s],
            [y * x * t + z * s, c + y * y * t, y * z * t - x * s],
            [z * x * t - y * s, z * y * t + x * s, c + z * z * t]]


def _apply(R, v):
    return [sum(R[i][j] * v[j] for j in range(3)) for i in range(3)]


def _mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _align(a, b):
    """The smallest rotation taking direction a onto direction b."""
    a, b = _unit(a), _unit(b)
    if not a or not b:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    c = max(-1.0, min(1.0, sum(x * y for x, y in zip(a, b))))
    ax = _unit(_cross(a, b))
    if not ax:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    return _rot(ax, math.acos(c))


def _mean(ps):
    return [sum(p[i] for p in ps) / len(ps) for i in range(3)]


BUMP_TRAVEL = (-60, -40, -20, 20, 40, 60)     # mm of wheel travel the bump steer is judged over
BUMP_STAY = 40.0                              # weight of the tie rod point staying near the SVJ's (deg^2 per m^2)


def _nelder_mead(f, x0, step, iters=60):
    """A small Nelder-Mead minimiser for f over a few variables (pure python)."""
    n = len(x0)
    pts = [list(x0)] + [[x0[j] + (step if j == i else 0.0) for j in range(n)] for i in range(n)]
    vals = [f(p) for p in pts]
    for _ in range(iters):
        order = sorted(range(n + 1), key=lambda i: vals[i])
        pts, vals = [pts[i] for i in order], [vals[i] for i in order]
        c = [sum(p[j] for p in pts[:-1]) / n for j in range(n)]
        refl = [c[j] + (c[j] - pts[-1][j]) for j in range(n)]
        fr = f(refl)
        if fr < vals[0]:
            exp = [c[j] + 2 * (c[j] - pts[-1][j]) for j in range(n)]
            fe = f(exp)
            pts[-1], vals[-1] = (exp, fe) if fe < fr else (refl, fr)
        elif fr < vals[-2]:
            pts[-1], vals[-1] = refl, fr
        else:
            con = [c[j] + 0.5 * (pts[-1][j] - c[j]) for j in range(n)]
            fc = f(con)
            if fc < vals[-1]:
                pts[-1], vals[-1] = con, fc
            else:
                pts = [pts[0]] + [[pts[0][j] + 0.5 * (p[j] - pts[0][j]) for j in range(n)] for p in pts[1:]]
                vals = [vals[0]] + [f(p) for p in pts[1:]]
        if max(abs(pts[-1][j] - pts[0][j]) for j in range(n)) < 5e-4:
            break
    i = min(range(n + 1), key=lambda k: vals[k])
    return pts[i], vals[i]


def bump_steer_tie_rod(svj, corner_name, site, outer, tie_end, lower_piv, upper_piv, tie0):
    """The tie rod's inner point (BeamNG) that keeps the wheel's toe steadiest over its travel, for a double wishbone
    corner whose outer points and arm pivots are as the conversion put them, found with the project's own kinematics
    (suspension.study_svj on a copy of the SVJ with those points): (point, bump steer before, after) in degrees
    (the largest toe change over BUMP_TRAVEL), or None where it cannot be solved or does no better. The lateral
    place of the point (the rack's) stays; its height and fore-aft place are free, held near the SVJ's by a small
    penalty. outer: {"wheel_center", "lower_ball_joint", "upper_ball_joint"}; lower_piv / upper_piv: the arms' inner
    points, front to rear; all BeamNG, of the left-hand side (the right side is mirrored by the caller). site: the
    SVJ's {"yf", "ground"} on the vehicle."""
    tpl = (svj.get("suspension") or {}).get(corner_name)
    if not tpl or "topology" not in tpl:
        return None
    sae = lambda p: [round(v, 5) for v in svjmod.to_sae(p, site["yf"], site["ground"])]   # noqa: E731

    def doc(inner):
        d = copy.deepcopy(svj)
        d["suspension"] = {corner_name: copy.deepcopy(tpl)}
        topo = d["suspension"][corner_name]["topology"]
        up = topo["upright"]["hardpoints"]
        for k, p in outer.items():
            up[k] = sae(p)
        if "toe_link_outer" in up:
            up["toe_link_outer"] = sae(tie_end)
        up["steering_tie_rod_end"] = sae(tie_end)
        for link in topo["links"]:
            role = _role(link.get("name", ""))
            if role == "lower_arm":
                link["inboard_points"] = [sae(p) for p in lower_piv]
            elif role == "upper_arm":
                link["inboard_points"] = [sae(p) for p in upper_piv]
            elif role == "tie_rod":
                link["inboard_points"] = [sae(inner)]
        return d

    axle = "front" if corner_name.startswith("F") else "rear"

    def toes(inner):
        try:
            cv = suspension.study_svj(doc(inner), 70)["corners"][axle]["curves"]
        except Exception:                                                   # a pose the solver cannot reach
            return None
        out = []
        for t in BUMP_TRAVEL:
            i = min(range(len(cv["travel_mm"])), key=lambda k: abs(cv["travel_mm"][k] - t))
            out.append(cv["toe_deg"][i])
        return out

    def cost(v):
        inner = [tie0[0], v[0], v[1]]
        t = toes(inner)
        if t is None:
            return 1e6
        return sum(x * x for x in t) + BUMP_STAY * (math.dist(inner, tie0) ** 2)

    before = toes(list(tie0))
    if before is None:
        return None
    best, _ = _nelder_mead(cost, [tie0[1], tie0[2]], 0.04)
    inner = [tie0[0], best[0], best[1]]
    after = toes(inner)
    if after is None or max(abs(x) for x in after) >= max(abs(x) for x in before):
        return None
    return inner, max(abs(x) for x in before), max(abs(x) for x in after)


def corner(nodes, roles, wheel, svj, corner_name, hps, wheel_axis=None, site=None):
    """The converted corner: ({node: target position}, {"upright": [nodes kept rigid], "rows": mapping
    rows for the report, "notes": [...]}). nodes: the base's positions (the rest geometry); roles:
    roles.for_corner(); hps: this corner's svj.hardpoints() rows; wheel_axis: the wheel's outward axis
    the SVJ asks for (static camber and toe), else the base's kept relative to the upright; site: the SVJ's
    {"yf", "ground"} on the vehicle, which lets a double wishbone corner's tie rod be put where its bump steer is least."""
    up_svj, links_svj = _svj_points(svj, corner_name, hps)
    if "wheel_center" not in up_svj:
        return {}, {"upright": [], "rows": [], "notes": [f"{corner_name}: the SVJ has no wheel centre"]}
    a1, a2 = wheel["node1"], wheel["node2"]
    W0 = _mean([nodes[a1], nodes[a2]])
    W1 = up_svj["wheel_center"]
    notes, rows = [], []
    # the steering axis, base and SVJ: lower ball joint to strut top (or upper ball joint)
    jb = roles["joints"]
    lbj0 = _mean([nodes[n] for n in jb.get("lower_ball_joint", [])]) if jb.get("lower_ball_joint") else None
    top0 = (nodes[roles["strut"]["top"][0]] if roles.get("strut") and roles["strut"]["top"]
            else _mean([nodes[n] for n in jb["upper_ball_joint"]]) if jb.get("upper_ball_joint") else None)
    lbj1 = up_svj.get("lower_ball_joint")
    top1 = (links_svj["strut"][0] if links_svj.get("strut") else up_svj.get("upper_ball_joint"))
    # first the wheel: its axis onto the SVJ's (static camber and toe; without them, square to the car)
    outer, inner = (a1, a2) if abs(nodes[a1][0]) >= abs(nodes[a2][0]) else (a2, a1)
    side = 1.0 if nodes[outer][0] >= 0 else -1.0
    u0 = [nodes[outer][i] - nodes[inner][i] for i in range(3)]
    R = _align(u0, wheel_axis or [side, 0.0, 0.0])
    # then about the wheel's axis, so the steering axis comes as close to the SVJ's as the base upright
    # allows (caster; the kingpin inclination is the upright's own: a rigid upright cannot take both an
    # SVJ's camber and its kingpin axis)
    if roles.get("caster") is False:
        notes.append(f"{corner_name}: the upright is a trailing arm (no steering axis): turned about the wheel's axis "
                     "as the base's")
    elif lbj0 and top0 and lbj1 and top1:
        w = _unit(_apply(R, u0))
        a0 = _apply(R, [top0[i] - lbj0[i] for i in range(3)])
        a1_ = [top1[i] - lbj1[i] for i in range(3)]
        p0 = _unit([a0[i] - sum(a0[k] * w[k] for k in range(3)) * w[i] for i in range(3)])
        p1 = _unit([a1_[i] - sum(a1_[k] * w[k] for k in range(3)) * w[i] for i in range(3)])
        if p0 and p1:
            ang = math.atan2(sum(_cross(p0, p1)[i] * w[i] for i in range(3)), sum(p0[i] * p1[i] for i in range(3)))
            R = _mul(_rot(w, ang), R)
    else:
        notes.append(f"{corner_name}: no steering axis on one side (lower ball joint and strut top or upper ball "
                     "joint): the caster is the base's")
    # the upright and its axle nodes, rigid: placed so the wheel centre is the SVJ's
    place = lambda p: [W1[i] + v for i, v in enumerate(_apply(R, [p[k] - W0[k] for k in range(3)]))]  # noqa: E731
    targets = {}
    upright = [n for n in roles["upright"] if n in nodes] + [a1, a2]
    for n in upright:
        targets[n] = place(nodes[n])
    rows.append({"corner": corner_name, "name": "wheel_center", "kind": "wheel", "nodes": [a1, a2], "target": W1,
                 "distance": round(math.dist(W0, W1), 4), "by": "role"})
    # the strut top: on the placed upright's rail line, at the SVJ's strut top height
    st = roles.get("strut")
    if st and st["top"] and st["rail_start"] in nodes:
        s0, t0 = nodes[st["rail_start"]], nodes[st["top"][0]]
        s1 = place(s0)
        d = _unit(_apply(R, [t0[k] - s0[k] for k in range(3)]))
        lam = math.dist(s0, t0)
        if links_svj.get("strut") and d and abs(d[2]) > 0.2:
            lam = (links_svj["strut"][0][2] - s1[2]) / d[2]
        top = [s1[i] + d[i] * lam for i in range(3)]
        for n in st["top"]:
            targets[n] = list(top)
        rows.append({"corner": corner_name, "name": "strut_top", "kind": "chassis", "nodes": list(st["top"]),
                     "target": links_svj["strut"][0] if links_svj.get("strut") else top,
                     "distance": round(math.dist(t0, links_svj["strut"][0]), 4) if links_svj.get("strut") else 0.0,
                     "by": "role"})
    # the links' inner pivots: the SVJ's, measured from where the link's outer joint landed
    for role, base_nodes in roles["pivots"].items():
        pts = links_svj.get(role)
        if not base_nodes:
            continue
        if not pts:
            notes.append(f"{corner_name}: {role} ({', '.join(base_nodes)}) has no SVJ counterpart: left to the field")
            continue
        joint = JOINT_OF.get(role)
        if roles.get("keep_arms") and role in ("lower_arm", "upper_arm") and joint and jb.get(joint):
            # the base's arm as it is, moved with its ball joint (the SVJ's points are not used)
            landed = _mean([targets[n] for n in jb[joint] if n in targets] or [place(nodes[n]) for n in jb[joint]])
            was = _mean([nodes[n] for n in jb[joint]])
            for n in base_nodes:
                targets[n] = [nodes[n][i] + landed[i] - was[i] for i in range(3)]
                rows.append({"corner": corner_name, "name": role, "kind": "chassis", "nodes": [n], "target": targets[n],
                             "distance": round(math.dist(nodes[n], targets[n]), 4), "by": "role"})
            notes.append(f"{corner_name}: {role} kept as the base's, moved with its ball joint (the SVJ's arm points do not suit it)")
            continue
        shift = [0.0, 0.0, 0.0]
        if joint and jb.get(joint) and up_svj.get(joint):
            landed = _mean([targets[n] for n in jb[joint] if n in targets] or [place(nodes[n]) for n in jb[joint]])
            shift = [landed[i] - up_svj[joint][i] for i in range(3)]
        want = [[p[i] + shift[i] for i in range(3)] for p in pts]
        bn = sorted(base_nodes, key=lambda n: nodes[n][1])          # front to rear (BeamNG y grows rearwards)
        wp = sorted(want, key=lambda p: p[1])
        if len(bn) == len(wp):
            pairs = list(zip(bn, wp))
        else:                                                         # unequal: each base pivot to the nearest
            pairs = [(n, min(wp, key=lambda p: math.dist(p, nodes[n]))) for n in bn]
        for n, p in pairs:
            targets[n] = p
            rows.append({"corner": corner_name, "name": role, "kind": "chassis", "nodes": [n], "target": p,
                         "distance": round(math.dist(nodes[n], p), 4), "by": "role"})
    # a double wishbone corner's tie rod: the base upright's shape is kept, so the SVJ's link vectors, each from its own
    # joint, no longer share the SVJ's instant centre, and the toe changes over the travel (the Civic's, 0.2 deg in the
    # SVJ, came out 1 deg at 80 mm of bump: the car steered itself as the front dived). The inner point of the rod is
    # put where the project's kinematics give the steadiest toe.
    tie_nodes = roles["pivots"].get("tie_rod") or []
    if site and tie_nodes and roles["pivots"].get("lower_arm") and roles["pivots"].get("upper_arm") \
            and jb.get("upper_ball_joint") and jb.get("lower_ball_joint") and jb.get("tie_rod_end") \
            and all(n in targets for n in tie_nodes + roles["pivots"]["lower_arm"] + roles["pivots"]["upper_arm"]):
        side = 1.0 if W1[0] >= 0 else -1.0
        mirror = lambda p: [side * p[0], p[1], p[2]]                       # the left-hand side's frame   # noqa: E731
        tg = lambda ns: [mirror(targets[n]) for n in sorted(ns, key=lambda n: targets[n][1])]     # noqa: E731
        pos = lambda ns: mirror(_mean([targets[n] for n in ns]))                                 # noqa: E731
        res = bump_steer_tie_rod(svj, corner_name[0] + "L", site,
                                 {"wheel_center": mirror(W1), "lower_ball_joint": pos(jb["lower_ball_joint"]),
                                  "upper_ball_joint": pos(jb["upper_ball_joint"])},
                                 pos(jb["tie_rod_end"]), tg(roles["pivots"]["lower_arm"]), tg(roles["pivots"]["upper_arm"]),
                                 mirror(targets[tie_nodes[0]]))
        if res:
            inner, before, after = res
            targets[tie_nodes[0]] = mirror(inner)
            for r in rows:
                if r.get("nodes") == [tie_nodes[0]]:
                    r["target"], r["distance"] = targets[tie_nodes[0]], round(math.dist(nodes[tie_nodes[0]], targets[tie_nodes[0]]), 4)
            notes.append(f"{corner_name}: tie rod inner point placed for the least bump steer (toe change over +-60 mm of "
                         f"travel {before:.2f} deg as converted, {after:.2f} deg now)")
    return targets, {"upright": upright, "rows": rows, "notes": notes}
