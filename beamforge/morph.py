"""The body fitted by morphing: the outer skin pulled onto the SVJ's body mesh, every other node following a smooth map.

Needs numpy and scipy (imported when the morph runs; without them the stage says so and the body is left as the wheelbase stage has it).

The fit's body stage (fit.py) matches the car's length, width by slice and heights along each axis, which gets the proportions but not the shape:
a bonnet's slope, a tail, a roof's curve. This morphs instead (the technique of fitting a crash model to a new body shape):

  1. skin      the outer nodes of the base: the nodes of its body parts' collision triangles with no other skin node beyond them (the sides, roof,
               underside, nose and tail), so the inner structure (floor, bulkheads, pillars) is not chased onto the mesh
  2. targets   for each skin node the point of the SVJ's body mesh nearest it, OFFSET inside the surface (the skin is under the paint)
  3. the map   a biharmonic radial basis function (phi(r) = r, with an affine part) through the skin nodes' displacements, smoothed so noise in the
               projections does not become wrinkles, with the running gear (suspension, wheels, brakes, steering, drivetrain, engine) held where the
               earlier stages put it; every node moves by it, the interior ones included
  4. relaxed   a thin-wall pair of nodes (an inner and an outer fender, 10 cm apart) can be pulled onto the same patch of mesh and collapse: the
               body's beams that left [RELAX_LO, RELAX_HI] of their length are pushed back while the skin stays pulled to its targets, then
               [HARD_LO, HARD_HI] is held hard

The export then lets the stiffness of each body beam and the weight of each node follow the new beam lengths (rigidity.LENGTH), as a longer beam is a
softer one (k = E A / L).
"""

import math
import os
import re

OFFSET = -0.02              # m: a skin node sits this far inside the visual mesh
SMOOTH = 0.01               # m: the RBF's smoothing
MAX_PULL = 0.25             # m: a skin node is not pulled further than this to its target (a projection that jumps is a wrong surface)
RELAX_LO, RELAX_HI = 0.75, 1.35
HARD_LO, HARD_HI = 0.70, 1.40
SPACING = 0.03              # m: points over the mesh, about this far apart (0.05 for a mesh of more than 300 000 triangles)
RADIUS = 0.06               # m: the cylinder of the outer-node test

# parts whose nodes are not the outside of the car (they sit inside it) or are not skin to shape (the lights and mirrors are placed with their panels)
INSIDE_PARTS = re.compile(r"interior|seat|occupant|engine|transmission|transaxle|fuel|radiator|exhaust|battery|shifter|cargo|oilpan|intake|"
                          r"driveshaft|differential|suspension|wheel|tire|brake|hub|steering|mirror|light|lamp|dashboard", re.I)
# the running gear: held where it is
GEAR = re.compile(r"suspension|coilover|shock|damper|spring|hub|knuckle|upright|wheeldata|brake|swaybar|sway_bar|axle|wheel|tire|tyre|steering|rack|"
                  r"halfshaft|driveshaft|differential|transaxle|transmission|engine|enginemounts", re.I)
# the SVJ mesh's parts that are not the outer shell
MESH_INSIDE = re.compile(r"int_|interior|seat|cockpit|driver|helmet|steer|dash|engine|motor|roll_?cage|rollcage|pedal|gear|exhaust|undertray|"
                         r"^wheel|tyre|tire|rim|disc|brake|caliper|susp|hub_|camera|damage|wiper|mirror", re.I)


def available():
    """Whether numpy and scipy can be imported here."""
    try:
        import numpy  # noqa: F401
        import scipy.spatial  # noqa: F401
        return True
    except ImportError:
        return False


def skin_nodes(skin, parts):
    """The base's skin: the nodes of the collision triangles of its body parts ({node: part}; skin from beamng.geometry)."""
    return {n: p for n, p in (skin or {}).items() if not INSIDE_PARTS.search(p or "")}


def outer_nodes(pos, ids, radius=RADIUS):
    """The nodes of `ids` on the outside: no other of them lies beyond, within `radius` of the line through it, along one of the six axis
    directions."""
    import numpy as np
    from scipy.spatial import cKDTree
    xyz = np.array([pos[i] for i in ids])
    out = np.zeros(len(ids), bool)
    for axis in range(3):
        others = [a for a in range(3) if a != axis]
        tree = cKDTree(xyz[:, others])
        near = tree.query_ball_point(xyz[:, others], radius)
        for sign in (1, -1):
            for i in range(len(ids)):
                if not out[i] and all((xyz[j, axis] - xyz[i, axis]) * sign <= 0.01 for j in near[i]):
                    out[i] = True
    return [i for i, o in zip(ids, out) if o]


def surface_samples(tri, spacing=SPACING):
    """Points over triangles `tri` (n, 3, 3), about `spacing` apart, with each triangle's unit normal: (points (m, 3), normals (m, 3))."""
    import numpy as np
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    n = np.cross(b - a, c - a)
    area = np.linalg.norm(n, axis=1)
    keep = area > 1e-12
    a, b, c, n, area = a[keep], b[keep], c[keep], n[keep], area[keep]
    n = n / area[:, None]
    edge = np.maximum.reduce([np.linalg.norm(b - a, axis=1), np.linalg.norm(c - b, axis=1), np.linalg.norm(a - c, axis=1)])
    steps = np.minimum(np.ceil(edge / spacing).astype(int), 12)
    P, N = [(a + b + c) / 3, a, b, c], [n, n, n, n]
    for k in range(2, 13):
        sel = steps >= k
        if not sel.any():
            break
        for i in range(1, k):
            for j in range(1, k - i):
                u, v = i / k, j / k
                P.append(a[sel] * (1 - u - v) + b[sel] * u + c[sel] * v)
                N.append(n[sel])
    return np.vstack(P), np.vstack(N)


def rbf_map(ctrl, disp, smooth):
    """The biharmonic RBF through control points `ctrl` (m, 3) with displacements `disp` (m, 3), smoothed by `smooth`: a function of points."""
    import numpy as np
    m = len(ctrl)
    D = np.linalg.norm(ctrl[:, None, :] - ctrl[None, :, :], axis=2)
    P = np.hstack([np.ones((m, 1)), ctrl])
    A = np.zeros((m + 4, m + 4))
    A[:m, :m] = D + smooth * np.eye(m)
    A[:m, m:] = P
    A[m:, :m] = P.T
    rhs = np.zeros((m + 4, 3))
    rhs[:m] = disp
    try:
        sol = np.linalg.solve(A, rhs)
    except np.linalg.LinAlgError:                        # control points on one plane or line: the affine part is not determined; the least-squares one is
        sol = np.linalg.lstsq(A, rhs, rcond=None)[0]
    w, c = sol[:m], sol[m:]

    def f(x):
        d = np.linalg.norm(x[:, None, :] - ctrl[None, :, :], axis=2)
        return d @ w + np.hstack([np.ones((len(x), 1)), x]) @ c
    return f


def relax(X, L0, ia, ib, fixed, skin, target, lo, hi, iters, pull):
    """Push the beams whose length ratio (to L0) is outside [lo, hi] (numbers, or one per beam) back inside it, node by node (each node takes the mean of the
    corrections its beams ask for), while the skin nodes (indices `skin`) are pulled toward `target` by `pull` of the way each round; `fixed` nodes do not
    move."""
    import numpy as np
    X = X.copy()
    n = len(X)
    free = np.ones(n, bool)
    free[fixed] = False
    deg = np.maximum(np.bincount(np.concatenate([ia, ib]), minlength=n), 1)
    for _ in range(iters):
        d = X[ib] - X[ia]
        L = np.maximum(np.linalg.norm(d, axis=1), 1e-9)
        Lt = np.clip(L, lo * L0, hi * L0)
        corr = ((L - Lt) / L)[:, None] * d * 0.5
        move = np.zeros_like(X)
        np.add.at(move, ia, corr)
        np.add.at(move, ib, -corr)
        X += (move / deg[:, None]) * 1.5 * free[:, None]
        if pull:
            X[skin] += pull * (target - X[skin])
    return X


def skin_error(pos, ids, tree, pts, nrm):
    """Per node: the distance along the mesh's normal at its nearest surface point (positive outside) less OFFSET: 0 is where it should be."""
    import numpy as np
    xyz = np.array([pos[i] for i in ids])
    _, k = tree.query(xyz)
    return np.einsum("ij,ij->i", xyz - pts[k], nrm[k]) - OFFSET


ROLES = (("hood", re.compile(r"bonnet|hood", re.I)), ("trunk", re.compile(r"boot|trunk|tailgate|hatch|liftgate", re.I)),
         ("door", re.compile(r"door", re.I)), ("fender", re.compile(r"wing|fender|mudguard|quarter", re.I)),
         ("bumper", re.compile(r"bumper|fascia", re.I)), ("spoiler", re.compile(r"spoiler", re.I)))
ROLE_NOT = re.compile(r"mirror|glass|window|handle|light|lamp|latch|hinge|flap|pod|exhaust|rack|wiper|engine|under|mud|wheel|int_|interior", re.I)
PIECE_REACH = 0.5           # m: a part's piece is the nearest of its role within this of the part's centre
PIECE_PULL = 0.15           # m: a node whose own piece is further than this goes to the whole mesh instead


def role_of(name):
    """The closure or panel role a part or mesh node name stands for (hood, trunk, door, fender, bumper, spoiler), or None."""
    if not name or ROLE_NOT.search(name):
        return None
    return next((r for r, rx in ROLES if rx.search(name)), None)


def part_pieces(P, role, pieces):
    """The mesh pieces of `role` a part with nodes at P (n, 3) takes its targets from: the ones whose centre lies inside the part's box (grown
    0.1 m), or else the nearest one within PIECE_REACH of the part's centre; a list of triangle arrays, empty when none."""
    import numpy as np
    cand = [(n, np.asarray(t).reshape(-1, 3, 3)) for n, t in (pieces or {}).get(role, [])]
    if not cand:
        return []
    lo, hi = P.min(axis=0) - 0.1, P.max(axis=0) + 0.1
    cen = [t.mean(axis=(0, 1)) for _, t in cand]
    inside = [t for (_, t), c in zip(cand, cen) if (c >= lo).all() and (c <= hi).all()]
    if inside:
        return inside
    d = [float(np.linalg.norm(c - P.mean(axis=0))) for c in cen]
    k = int(np.argmin(d))
    return [cand[k][1]] if d[k] <= PIECE_REACH else []


NEAR = 0.12                 # m: nodes closer than this in the base may not be pulled closer than NEAR_LO of their distance (a thin wall's two skins)
NEAR_LO, NEAR_HI = 0.6, 50.0


def place_slides(out, base, slides):
    """Each slide node back on its rail at the same fraction along it as in the base (a node riding a rail would otherwise be put on the nearest point of
    the rail afterwards, and a rail that turned could carry it onto the rail's end, on top of its neighbour). slides: [[node, rail start, rail end, spring]]."""
    for n, a, b, _ in slides or []:
        if n in out and a in out and b in out and n in base and a in base and b in base:
            d = [base[b][i] - base[a][i] for i in range(3)]
            dd = sum(x * x for x in d)
            if dd < 1e-9:
                continue
            t = max(0.02, min(0.98, sum((base[n][i] - base[a][i]) * d[i] for i in range(3)) / dd))
            out[n] = [round(out[a][i] + t * (out[b][i] - out[a][i]), 4) for i in range(3)]


SHELL = re.compile(r"(?:^|_)body(?:_|$)|chassis|(?:^|_)frame(?:_|$)|unibody", re.I)
MAX_TURN = 25.0             # degrees: a part is turned no more than this to follow the nodes it hangs on (more: only moved)
SNAP_PIECE = 0.12          # m: a part's outer node goes to its own mesh piece when that is no further than this from where the part hangs
SNAP_MESH = 0.05            # m: ... to the nearest mesh surface, for a part with no piece of its own, when it is no further than this
ANCHOR_REACH = 0.6          # m: an SVJ fixing point belongs to the part of its role that has a node within this of it
ANCHOR_CLUSTER = 0.35       # m: the part's fixing nodes at the point: those within this of it
ANCHOR_MAX = 0.25           # m: the largest shift of a part to put its fixing nodes on the SVJ's point (more: the point is not trusted)
ANCHOR_ROLES = (("door", re.compile(r"door", re.I)), ("hood", re.compile(r"bonnet|hood", re.I)),
                ("trunk", re.compile(r"boot|trunk|tailgate|hatch|liftgate", re.I)), ("bumper", re.compile(r"bumper|fascia", re.I)),
                ("spoiler", re.compile(r"spoiler|wing", re.I)), ("fender", re.compile(r"fender|mudguard", re.I)))


def anchor_role(name):
    """The role a part or an SVJ body name stands for in a fixing point (door, hood, trunk, bumper, spoiler: a wing too, fender), or None."""
    if not name or ROLE_NOT.search(name):
        return None
    return next((r for r, rx in ANCHOR_ROLES if rx.search(name)), None)


def anchors_from_svj(svj, yf, ground):
    """The SVJ's fixing points between the chassis and its body pieces (a v0.103 RBR file's constraints: hinge_door_fl, hinge_hood, fixed_bumper_f...) as
    [{"id", "body", "role", "hinge", "point"}] in BeamNG coordinates for a front axle at yf and the ground at `ground`; the SVJ is read in SAE J670 (unified.adopt
    has converted an ISO 8855 file): x = -Y, y = yf - X, z = ground - Z. An SVJ without such constraints gives []."""
    out = []
    for c in (svj or {}).get("constraints") or []:
        if str(c.get("body_a") or "") != "chassis":
            continue
        body = str(c.get("body_b") or "")
        role = anchor_role(body)
        p = (c.get("frame_a") or {}).get("position")
        if role and isinstance(p, (list, tuple)) and len(p) == 3:
            out.append({"id": c.get("id"), "body": body, "role": role, "hinge": str(c.get("joint_type") or "") in ("revolute", "hinge"),
                        "point": [-p[1], yf - p[0], ground - p[2]]})
    return out


def shell_parts(by_part):
    """The parts that make the body shell, of {part: [outer nodes]}: the ones named body / chassis / frame, else the part with most outer nodes."""
    named = {p for p in by_part if SHELL.search(p)}
    if named:
        return named
    return {max(by_part, key=lambda p: len(by_part[p]))} if by_part else set()


def piecewise_map(ids, idx, X0, X1, outer, parts, ia, ib, shell, pieces, anchors, tree, pts, nrm):
    """Every part that is not the shell or the running gear is fitted on its own, in the order it can hang on parts already placed: it first moves as one
    rigid piece with the nodes it hangs on (the nodes with a beam to a placed part follow what those moved), then, if the SVJ has a fixing point for it
    (anchors), is shifted so its fixing nodes sit on that point, then its outer nodes snap to its own mesh piece (or, with none, to the nearest surface when
    that is close) through a smooth map over the part alone, its fixing nodes held. X1: the shell's map for every node (the start, and what is kept for a
    part with nothing to hang on). Returns (X1, {"rigid", "local", "anchored"}, notes)."""
    import numpy as np
    from scipy.spatial import cKDTree
    X1 = X1.copy()
    n = len(ids)
    pname = [parts.get(i) or "" for i in ids]
    by_outer = {}
    for o in outer:
        by_outer.setdefault(parts.get(o) or "", []).append(idx[o])
    todo = [p for p in by_outer if p not in shell and not GEAR.search(p)]
    done = np.array([pname[i] in shell or bool(GEAR.search(pname[i])) for i in range(n)])
    members = {p: [] for p in todo}
    for i in range(n):
        if pname[i] in members:
            members[pname[i]].append(i)
    nbr = [[] for _ in range(n)]
    for a, b in zip(ia.tolist(), ib.tolist()):
        if pname[a] != pname[b]:
            nbr[a].append(b)
            nbr[b].append(a)
    notes, stats = [], {"rigid": 0, "local": 0, "anchored": 0}
    # a fixing point belongs to the part of its role nearest it
    pick = {}
    for a in anchors or []:
        best, bd = None, ANCHOR_REACH
        for p in todo:
            if anchor_role(p) != a["role"] or p in pick:
                continue
            d = float(np.min(np.linalg.norm(X1[members[p]] - np.array(a["point"]), axis=1))) if members[p] else 9.0
            if d < bd:
                best, bd = p, d
        if best:
            pick[best] = a
        else:
            notes.append(f"SVJ fixing point {a['id']} ({a['role']}): no part of that role within {ANCHOR_REACH:.1f} m of it")
    pending = list(todo)
    for rnd in range(6):
        for q in list(pending):
            mem = members[q]
            att = [i for i in mem if any(done[j] for j in nbr[i])]
            if len(att) < 3 and rnd < 5 and len(mem) > 3:
                continue                                              # (wait for the parts it hangs on to be placed)
            pending.remove(q)
            if not att:
                done[mem] = True
                continue
            D = np.array([np.mean([X1[j] - X0[j] for j in nbr[i] if done[j]], axis=0) for i in att])
            A, B = X0[att], X0[att] + D
            Xm = X0[mem]
            ca, cb = A.mean(axis=0), B.mean(axis=0)
            R = None
            if len(att) >= 3:
                U, S, Vt = np.linalg.svd((A - ca).T @ (B - cb))
                if S[1] > 0.05 * S[0] and S[1] > 1e-4:                      # (nodes on a line leave the turn about it open: the part would spin)
                    R = Vt.T @ np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T)) or 1.0]) @ U.T
                    if math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2)))) > MAX_TURN:
                        R = None
            Xr = (Xm - ca) @ R.T + cb if R is not None else Xm + D.mean(axis=0)
            row = {m: k for k, m in enumerate(mem)}
            att_rows = [row[i] for i in att]
            stats["rigid"] += 1
            a = pick.get(q)
            hinged = False
            if a is not None and a.get("hinge"):                       # a hinge is a place the part really turns about: its nodes there go onto the SVJ's point
                near = [r for r in att_rows if np.linalg.norm(Xr[r] - np.array(a["point"])) <= ANCHOR_CLUSTER]
                if near:
                    t = np.array(a["point"]) - Xr[near].mean(axis=0)
                    if np.linalg.norm(t) <= ANCHOR_MAX:
                        Xr = Xr + t
                        stats["anchored"] += 1
                        hinged = True
                    else:
                        notes.append(f"{q}: its hinge nodes are {np.linalg.norm(t) * 100:.0f} cm from the SVJ's {a['id']}: the point is not used")
            js = by_outer.get(q, [])
            snap_rows, snaps = [], []
            if js:
                R_out = np.array([row[j] for j in js])
                role = role_of(q)
                tris_p = []
                if a is not None:                                    # the mesh piece the SVJ's fixing point is at (the names in a mesh may not be the constraints')
                    cand = [(n_, np.asarray(t_).reshape(-1, 3, 3)) for lst in (pieces or {}).values() for n_, t_ in lst]
                    dist = [float(np.linalg.norm(t_.reshape(-1, 3) - np.array(a["point"]), axis=1).min()) for _, t_ in cand]
                    if dist and min(dist) <= 0.15:
                        tris_p = [cand[int(np.argmin(dist))][1]]
                if not tris_p:
                    tris_p = part_pieces(Xr[R_out], role, pieces) if role else []
                if tris_p:
                    pp, pn = surface_samples(np.concatenate(tris_p), SPACING)
                    ptree = cKDTree(pp)
                    if not hinged:                                    # a part with a piece of its own is first put on it (a few passes of the mean offset)
                        for _ in range(3):
                            d_, k_ = ptree.query(Xr[R_out])
                            s_ = pp[k_] + pn[k_] * OFFSET - Xr[R_out]
                            ok_ = np.linalg.norm(s_, axis=1) <= 0.4
                            if ok_.sum() < 3:
                                break
                            t = s_[ok_].mean(axis=0)
                            if np.linalg.norm(t) > 0.35:
                                t = t * (0.35 / np.linalg.norm(t))
                            Xr = Xr + t
                    Q = Xr[R_out]
                    d_, k_ = ptree.query(Q)
                    tgt, lim = pp[k_] + pn[k_] * OFFSET, SNAP_PIECE
                else:
                    closure = bool(role) or a is not None             # a closure or a mounted part with no piece of its own (a hatch that is part of the shell): put on the surface by the
                    if closure and not hinged:                        # mean offset of its nodes to it (a few passes), as one with a piece is, then snapped
                        for _ in range(3):
                            d_, k_ = tree.query(Xr[R_out])
                            s_ = pts[k_] + nrm[k_] * OFFSET - Xr[R_out]
                            ok_ = np.linalg.norm(s_, axis=1) <= 0.4
                            if ok_.sum() < 3:
                                break
                            t = s_[ok_].mean(axis=0)
                            if np.linalg.norm(t) > 0.35:
                                t = t * (0.35 / np.linalg.norm(t))
                            Xr = Xr + t
                    Q = Xr[R_out]
                    d_, k_ = tree.query(Q)
                    tgt, lim = pts[k_] + nrm[k_] * OFFSET, (SNAP_PIECE if closure else SNAP_MESH)
                s = tgt - Q
                ok = np.linalg.norm(s, axis=1) <= lim
                snap_rows, snaps = R_out[ok], s[ok]
            if len(snap_rows) >= 4 or (len(snap_rows) >= 1 and len(att_rows) >= 3):
                ctrl = np.vstack([Xr[att_rows], Xr[snap_rows]])
                cd = np.vstack([np.zeros((len(att_rows), 3)), snaps])
                dq = rbf_map(ctrl, cd, SMOOTH)(Xr)
                nq = np.linalg.norm(dq, axis=1)
                Xr = Xr + dq * np.minimum(1.0, SNAP_PIECE * 1.5 / np.maximum(nq, 1e-9))[:, None]     # (few, nearly flat control points extrapolate wildly)
                stats["local"] += 1
            if os.environ.get("BEAMFORGE_PW_DEBUG"):
                ang = float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))) if R is not None else 0.0
                notes.append(f"PW {q}: att {len(att)} D {np.linalg.norm(D, axis=1).mean() * 100:.0f} cm rot {ang:.1f} deg anchor {a['id'] if a else '-'} "
                             f"snaps {len(snap_rows)} moved {np.linalg.norm(Xr - X0[mem], axis=1).mean() * 100:.0f} cm vs shell map {np.linalg.norm(Xr - X1[mem], axis=1).mean() * 100:.0f} cm")
            X1[mem] = Xr
            done[mem] = True
        if not pending:
            break
    for q in pending:                                                 # nothing to hang on: the shell's map has it
        notes.append(f"{q}: no node of it hangs on a placed part; moved by the body's map")
    return X1, stats, notes


def morph(out, base, parts, skin, beams, beam_parts, tri, smooth=SMOOTH, slides=None, pieces=None, anchors=None, piecewise=True):
    """Morph the nodes `out` ({id: [x, y, z]}, the wheelbase stage's result; changed in place) onto the body mesh `tri` (n, 3, 3 triangles in
    BeamNG coordinates). base: the nodes as the base has them (the beams' lengths to keep to); parts: {node: part}; skin: {node: part} (the collision
    skin); beams: [[a, b]] with beam_parts. Returns {"report": [...], "notes": [...], "changes": [...]}; {"error": text} when it cannot run."""
    if not available():
        return {"error": "the morph needs numpy and scipy (not found here): the body is left as the wheelbase stage has it"}
    import numpy as np
    from scipy.spatial import cKDTree
    ids_skin = sorted(skin_nodes(skin, parts))
    if len(ids_skin) < 30:
        return {"error": f"the base has only {len(ids_skin)} collision-skin nodes: the morph needs the collision triangles of its body parts"}
    if tri is None or len(tri) < 100:
        return {"error": "no SVJ body mesh to morph onto"}
    outer = outer_nodes(out, ids_skin)
    pts, nrm = surface_samples(tri, SPACING if len(tri) < 300000 else 0.05)
    tree = cKDTree(pts)
    ids = list(out)
    idx = {n: i for i, n in enumerate(ids)}
    X0 = np.array([out[n] for n in ids])
    out_idx = np.array([idx[n] for n in outer])
    P = X0[out_idx]
    err0 = skin_error(out, outer, tree, pts, nrm)
    _, k = tree.query(P)
    target = pts[k] + nrm[k] * OFFSET
    own, own_js = 0, set()
    if pieces:                           # a closure's or panel's nodes go to its own piece of the mesh, not to whatever surface is nearest
        by_part = {}
        for j, n in enumerate(outer):
            by_part.setdefault(parts.get(n) or "", []).append(j)
        for name, js in by_part.items():
            role = role_of(name)
            tris_p = part_pieces(P[js], role, pieces) if role else []
            if not tris_p:
                continue
            Q = P[js]
            pp, pn = surface_samples(np.concatenate(tris_p), SPACING)
            dd, kk = cKDTree(pp).query(Q)
            for j, d_, k_ in zip(js, dd, kk):
                if d_ <= PIECE_PULL:
                    target[j] = pp[k_] + pn[k_] * OFFSET
                    own += 1
                    own_js.add(j)
    disp = np.clip(target - P, -MAX_PULL, MAX_PULL)
    held = [i for i, n in enumerate(ids) if GEAR.search(parts.get(n) or "") and n not in set(outer)]
    notes_pw, pw = [], None
    by_outer = {}
    for j, n in enumerate(outer):
        by_outer.setdefault(parts.get(n) or "", []).append(j)
    shell = shell_parts(by_outer) if piecewise else set()
    if piecewise and shell:
        # the body shell and the pieces with a mesh piece of their own make the smooth map (what the shell's nodes follow); every other piece is fitted on its own
        js = sorted({j for p in shell for j in by_outer[p]} | own_js)
        ctrl = np.vstack([P[js], X0[held]])
        cd = np.vstack([disp[js], np.zeros((len(held), 3))])
    else:
        ctrl = np.vstack([P, X0[held]])
        cd = np.vstack([disp, np.zeros((len(held), 3))])
    f = rbf_map(ctrl, cd, smooth)
    X1 = X0 + f(X0)
    # the body's beams: the ones whose lengths must stay in bounds
    held_set = set(held)
    bl = [(a, b) for (a, b), p in zip(beams, beam_parts) if a in idx and b in idx and (not GEAR.search(p or "") or not (idx[a] in held_set and idx[b] in held_set))]
    ia = np.array([idx[a] for a, _ in bl])
    ib = np.array([idx[b] for _, b in bl])
    if piecewise and shell:
        X1, pw, notes_pw = piecewise_map(ids, idx, X0, X1, outer, parts, ia, ib, shell, pieces, anchors, tree, pts, nrm)
        disp = X1[out_idx] - P                                          # (the relax below pulls each skin node to where its piece put it)
    B0 = np.array([base[n] if n in base else out[n] for n in ids])
    L0 = np.linalg.norm(B0[ib] - B0[ia], axis=1)
    ok = L0 > 0.03
    L0 = np.maximum(L0, 1e-9)
    ia, ib, L0 = ia[ok], ib[ok], L0[ok]
    nb = len(ia)
    # nodes close together in the base (a thin wall's two skins, two parts side by side) that have no beam between them: not to be pulled closer than NEAR_LO
    have = {(min(a, b), max(a, b)) for a, b in zip(ia.tolist(), ib.tolist())}
    pairs = [(i, j) for i, j in cKDTree(B0).query_pairs(NEAR) if (i, j) not in have and np.linalg.norm(B0[i] - B0[j]) > 0.01
             and not (GEAR.search(parts.get(ids[i]) or "") and GEAR.search(parts.get(ids[j]) or ""))]
    if pairs:
        pa, pb = np.array([p[0] for p in pairs]), np.array([p[1] for p in pairs])
        ia2, ib2 = np.concatenate([ia, pa]), np.concatenate([ib, pb])
        L02 = np.concatenate([L0, np.linalg.norm(B0[pb] - B0[pa], axis=1)])
    else:
        ia2, ib2, L02 = ia, ib, L0
    lo = lambda v: np.concatenate([np.full(nb, v), np.full(len(ia2) - nb, NEAR_LO)])         # noqa: E731
    hi = lambda v: np.concatenate([np.full(nb, v), np.full(len(ia2) - nb, NEAR_HI)])         # noqa: E731
    X2 = relax(X1, L02, ia2, ib2, held, out_idx, P + disp, lo(RELAX_LO), hi(RELAX_HI), 80, 0.25)
    X3 = relax(X2, L02, ia2, ib2, held, out_idx, P + disp, lo(HARD_LO), hi(HARD_HI), 60, 0.0)
    for i, n in enumerate(ids):
        out[n] = [round(float(x), 4) for x in X3[i]]
    place_slides(out, base, slides)
    X3 = np.array([out[n] for n in ids])
    err1 = skin_error(out, outer, tree, pts, nrm)
    ratio = np.linalg.norm(X3[ib] - X3[ia], axis=1) / L0
    return {"report": [{"stage": "morph", "label": "Outer skin to the mesh (mean, m)", "unit": "m", "base": round(float(np.abs(err0).mean()), 4),
                        "target": 0.0, "after": round(float(np.abs(err1).mean()), 4)},
                       {"stage": "morph", "label": "Outer skin within 2 cm", "unit": "share", "base": round(float((np.abs(err0) <= 0.02).mean()), 3),
                        "target": 1.0, "after": round(float((np.abs(err1) <= 0.02).mean()), 3)}],
            "notes": notes_pw, "changes": [f"body morphed onto the mesh: {len(outer)} outer skin nodes, mean distance to the mesh {np.abs(err0).mean() * 100:.1f} cm -> "
                                     f"{np.abs(err1).mean() * 100:.1f} cm; {len(bl)} body beams now {ratio.min():.2f} to {ratio.max():.2f} of their length "
                                     f"(median {np.median(ratio):.2f}); the beams' stiffness and the nodes' weights follow"
                                     + (f"; {own} closure and panel nodes took the target from their own mesh piece" if own else "")
                                     + (f"; piece by piece: {pw['rigid']} parts moved with the nodes they hang on (rigid), {pw['local']} of them also fitted to their piece, "
                                        f"{pw['anchored']} held at an SVJ fixing point" if pw else "")],
            "stats": {"skin": len(outer), "before": float(np.abs(err0).mean()), "after": float(np.abs(err1).mean()),
                      "beam_min": float(ratio.min()), "beam_max": float(ratio.max())}}
