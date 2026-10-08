"""A sketch mounted on a base vehicle: new mechanisms on a vanilla car.

Pure Python, standard library only (runs in Pyodide). UNVERIFIED IN-GAME.

The sketch is drawn in the SVJ frame placed on the base (its front axle line and ground, as the editor shows it), so it
joins the vehicle by the same rule as its own parts join each other:

  shared     a sketch node at a base node's place (within MERGE) is that node: the sketch's beams end on it. Its weight
             (the sketch's tubes at that end) is added to the base node's, and the sketch's beams on it are softened, if
             needed, so the node stays under the vanilla cars' 90th percentile (rigidity.VANILLA)
  ties       a sketch node the user ties to the vehicle: beams to its TIE_N nearest body nodes (not the suspension's),
             with stable values (rigidity.beam_values), as the vanilla cars mount an engine or a subframe
  near       a sketch node closer than NEAR to a base node but not at its place: reported, it was probably meant to be
             shared (move it onto the node, or tie it)

The other sketch nodes are new nodes (ids bf<sketch id>, node group sk_<part>), written with the beams at the end of the
vehicle's main part. A sketch body not joined to the vehicle through shared nodes, ties or other bodies is reported:
it would fall off.
"""

import json
import math

from . import rigidity, skeleton, svj as svjmod, values

MERGE = 0.005            # m: a sketch node this close to a base node is that node
NEAR = 0.05              # m: closer than this but not at the node: reported
TIE_N = 3                # beams from a tied node to the vehicle
MIN_SOFT = 0.2           # a sketch beam is softened to no less than this share at a shared node


def plan(model, configured, sketch, yf=None, ground=None, ties=(), merge=MERGE):
    """The sketch on the vehicle: {"nodes": [{"id", "sketch", "pos" (BeamNG), "kg", "group"}] (the new ones),
    "beams": [{"a", "b", "beamSpring", "beamDamp", "beamDeform", "beamStrength"}], "weights": {part: {row: kg}} (the
    shared base nodes' new weights), "shared": [[sketch id, base id, mm]], "near": [[sketch id, base id, mm]],
    "ties": [[sketch id, [base ids]]], "softened", "skipped" (beams the base already has), "loose": [parts],
    "tie_suggestions": {loose part: its sketch node nearest to the body}, "warnings"}. sketch: {"parts", "opts"} (SVJ frame, m); yf, ground: where the SVJ frame sits on the vehicle (its
    measure by default)."""
    geo = configured["geometry"]
    meas = configured.get("measure") or {}
    yf = meas.get("front_axle_y") or 0.0 if yf is None else yf
    ground = meas.get("ground_z") or 0.0 if ground is None else ground
    opts = sketch.get("opts") or {}
    res = skeleton.build(sketch["parts"], 1.0, (0, 0, 0), (0.0, 0.0, 0.0), opts.get("tol") or skeleton.TOL,
                         opts.get("flat") or skeleton.FLAT, opts.get("kinds"))
    kg, bv, _, _ = skeleton.values(res, opts.get("tubes"), opts.get("min_kg") or skeleton.MIN_KG)
    base = geo["nodes"]
    rows, _ = values.node_weights(model, configured)
    m, where = {}, {}
    for part, row, nid, w, _ in rows:
        m[nid] = m.get(nid, 0.0) + w
        where.setdefault(nid, (part, row, w))
    bl = rigidity.beams(model, configured)
    ks, cs = {}, {}
    for b in bl:
        for n in (b["a"], b["b"]):
            ks[n] = ks.get(n, 0.0) + b["values"].get("beamSpring", 0.0)
            cs[n] = cs.get(n, 0.0) + b["values"].get("beamDamp", 0.0)
    has = {frozenset((b["a"], b["b"])) for b in bl}
    body = [n for n in base if n in m and not values.UNSPRUNG.search(geo["parts"].get(n) or "")]

    ids, pos, out_nodes, shared, near = {}, {}, [], [], []
    for n, v in res["nodes"].items():
        if v["reference"]:
            continue
        p = svjmod.from_sae(v["pos"], yf, ground)
        pos[n] = p
        best = min(((math.dist(p, base[b]), b) for b in base if b in m), default=None)
        if best and best[0] <= merge:
            ids[n] = best[1]
            shared.append([n, best[1], round(best[0] * 1000, 1)])
            continue
        if best and best[0] <= NEAR:
            near.append([n, best[1], round(best[0] * 1000, 1)])
        ids[n] = f"bf{n}"
        out_nodes.append({"id": ids[n], "sketch": n, "pos": [round(x, 4) for x in p], "kg": kg[n],
                          "group": f"sk_{v['owner']}" if v["owner"] else ""})

    # the sketch's beams, on the shared nodes where it has them
    beams, skipped = [], 0
    for i, b in enumerate(res["beams"]):
        a, c = ids[b["a"]], ids[b["b"]]
        if a == c:
            continue
        if frozenset((a, c)) in has:                     # the vehicle already holds these two nodes together
            skipped += 1
            continue
        beams.append(dict({"a": a, "b": c}, **{k: bv[i][k] for k in ("beamSpring", "beamDamp", "beamDeform", "beamStrength")}))

    # the shared nodes: the sketch's weight at that end added, its beams softened to keep the node in the vanilla range
    sh = {b: [] for _, b, _ in shared}
    for s, b, _ in shared:
        sh[b].append(s)
    weights, weight_add, newm = {}, {}, dict(m)
    for b, ss in sh.items():
        add = sum(kg[s] for s in ss)
        part, row, w = where[b]
        weights.setdefault(part, {})[row] = round(w + add, 3)
        weight_add.setdefault(part, {})[row] = round(add, 3)
        newm[b] = m[b] + add
    dt, cap_k, cap_c = rigidity.DT, rigidity.VANILLA["node_k"]["p90"], rigidity.VANILLA["node_c"]["p90"]
    fk, fc = {}, {}
    for b in sh:
        on = [x for x in beams if b in (x["a"], x["b"])]
        add_k, add_c = sum(x["beamSpring"] for x in on), sum(x["beamDamp"] for x in on)
        room_k = max(cap_k * newm[b] / (dt * dt), ks.get(b, 0.0)) - ks.get(b, 0.0)   # a node over the cap keeps its own
        room_c = max(cap_c * newm[b] / dt, cs.get(b, 0.0)) - cs.get(b, 0.0)
        fk[b] = 1.0 if add_k <= room_k else max(MIN_SOFT, room_k / add_k)
        fc[b] = 1.0 if add_c <= room_c else max(MIN_SOFT, room_c / add_c)
    softened, warnings = 0, []
    for x in beams:
        gk = min(fk.get(x["a"], 1.0), fk.get(x["b"], 1.0))
        gc = min(fc.get(x["a"], 1.0), fc.get(x["b"], 1.0))
        if gk < 1 or gc < 1:
            softened += 1
            x["beamSpring"] = round(x["beamSpring"] * gk, -3)
            x["beamDamp"] = round(x["beamDamp"] * gc, 1)
    if any(min(fk.get(b, 1.0), fc.get(b, 1.0)) <= MIN_SOFT for b in sh):
        warnings.append("a shared node is already as stiff as the vanilla cars go: the sketch's beams on it are at a fifth "
                        "of their value and it may still ring; tie the sketch near it instead")

    # ties: beams from a sketch node to the nearest body nodes, stable for both ends
    nk = {x["id"]: x["kg"] for x in out_nodes}
    sk_k, sk_c = {}, {}
    for x in beams:
        for n in (x["a"], x["b"]):
            sk_k[n] = sk_k.get(n, 0.0) + x["beamSpring"]
            sk_c[n] = sk_c.get(n, 0.0) + x["beamDamp"]
    tied = []
    for s in ties or ():
        if s not in ids:
            warnings.append(f"{s}: not a node of the sketch")
            continue
        a = ids[s]
        if a in base:
            warnings.append(f"{s}: shared with {a} already")
            continue
        to = sorted(body, key=lambda n: math.dist(base[n], pos[s]))[:TIE_N]
        for n in to:
            ma, mb = nk[a], newm[n]
            v = rigidity.beam_values(ma, mb, (sk_k.get(a, 0.0), ks.get(n, 0.0) + sk_k.get(n, 0.0)),
                                     (sk_c.get(a, 0.0), cs.get(n, 0.0) + sk_c.get(n, 0.0)))
            beams.append({"a": a, "b": n, "beamSpring": v["beamSpring"], "beamDamp": v["beamDamp"],
                          "beamDeform": rigidity.DEFAULTS["beamDeform"], "beamStrength": None})
            for q in (a, n):
                sk_k[q] = sk_k.get(q, 0.0) + v["beamSpring"]
                sk_c[q] = sk_c.get(q, 0.0) + v["beamDamp"]
        tied.append([s, to])

    # loose bodies: sketch nodes not joined to the vehicle through its beams, shared nodes and ties
    parent = {}

    def find(x):
        while parent.setdefault(x, x) != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x
    for x in beams:
        parent[find(x["a"])] = find(x["b"])
    for n in base:
        parent[find(n)] = find("__vehicle__")
    root = find("__vehicle__")
    loose_nodes = {x["sketch"] for x in out_nodes if find(x["id"]) != root}
    loose = sorted({res["nodes"][n]["owner"] for n in loose_nodes if res["nodes"][n]["owner"]})
    suggest = {}                                         # per loose part, its node nearest to the body: the one to tie
    for part in loose:
        cand = [n for n in loose_nodes if res["nodes"][n]["owner"] == part]
        if body and cand:
            suggest[part] = min(cand, key=lambda n: min(math.dist(pos[n], base[b]) for b in body))
    if loose:
        warnings.append("not joined to the vehicle (it would fall off): " + ", ".join(loose)
                        + ". Put a node of it on a node of the vehicle, or tie one")
    if near:
        warnings.append(f"{len(near)} sketch nodes are within {NEAR * 100:.0f} cm of a vehicle node but not on it: "
                        "move them onto it to share it, or tie them")
    return {"nodes": out_nodes, "beams": beams, "weights": weights, "weight_add": weight_add, "shared": shared, "near": near, "ties": tied,
            "softened": softened, "skipped": skipped, "loose": loose, "tie_suggestions": suggest, "warnings": warnings}


NODE_RESET = {"collision": True, "selfCollision": False, "fixed": False, "frictionCoef": 1.0,
              "nodeMaterial": "|NM_METAL", "engineGroup": "", "chemEnergy": False}


def write(part, p):
    """The mount's new nodes and beams at the end of a part's tables (the main part), each with its properties inline
    and the ones the rows above carry on reset."""
    from . import export
    nodes = part.get("nodes")
    if not isinstance(nodes, list) or not nodes:
        nodes = part["nodes"] = [["id", "posX", "posY", "posZ"]]
    for x in p["nodes"]:
        nodes.append([x["id"], *x["pos"], dict(NODE_RESET, nodeWeight=x["kg"], group=x["group"])])
    return export._add_beams(part, p["beams"])


def plan_json(model, configured_json, sketch_json, ties_json=None):
    p = plan(model, json.loads(configured_json), json.loads(sketch_json), ties=json.loads(ties_json) if ties_json else ())
    return json.dumps({k: v for k, v in p.items() if k not in ("beams", "weights", "weight_add")} | {"beam_count": len(p["beams"])})
