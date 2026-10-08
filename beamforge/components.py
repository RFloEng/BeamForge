"""Components: the masses a car carries (engine, fuel, driver, battery, panels, ballast), its mass and centre of gravity.

Pure Python, standard library only (runs in Pyodide). Positions are in the SVJ frame (SAE: X forward, Y right, Z down,
origin on the ground under the front axle centre), so the CG's X is minus its distance behind the front axle and its Z
minus its height. A component is {"id", "kind", "mass" (kg), "position" [x, y, z]}; kinds follow the SVJ's mass body
categories (§7.2.1) plus "driver".

estimate()    a car from scratch's mass and CG before it is made: its sketch's nodes (skeleton.values), the engine, the
              wheels and the components
ballast()     the one mass, and where, that brings a total to a target mass and CG
spread()      a component's mass on the nearest structure nodes (inverse distance), as a car from scratch carries it
"""

import json
import math

from . import skeleton

KINDS = ("structural", "powertrain", "fluid", "payload", "driver", "electrical", "ballast", "other")
WHEEL_KG = 20.0                # a rim and tyre, when the wheels are not chosen yet
NEAREST = 4                    # a component's mass goes to this many nodes


def svj_components(svj):
    """The SVJ's mass bodies as components (chassis.mass_bodies, with a mass and a position)."""
    out = []
    for b in ((svj or {}).get("chassis") or {}).get("mass_bodies") or []:
        if isinstance(b, dict) and isinstance(b.get("mass"), (int, float)) and isinstance(b.get("position"), list) and len(b["position"]) == 3:
            out.append({"id": str(b.get("id") or "body"), "kind": b.get("category") or "other", "mass": float(b["mass"]),
                        "position": [float(x) for x in b["position"]]})
    return out


def svj_target(svj):
    """(mass kg, CG [x, y, z] SAE) of the SVJ: chassis.mass_total and center_of_gravity, None where absent."""
    ch = (svj or {}).get("chassis") or {}
    cg = ch.get("center_of_gravity")
    return (float(ch["mass_total"]) if isinstance(ch.get("mass_total"), (int, float)) else None,
            [float(x) for x in cg] if isinstance(cg, list) and len(cg) == 3 else None)


def combine(items):
    """(total kg, CG) of [(kg, [x, y, z])]."""
    m = sum(k for k, _ in items)
    if m <= 0:
        return 0.0, None
    return m, [sum(k * p[i] for k, p in items) / m for i in range(3)]


def estimate(parts, opts, engine=None, wheels=None, components=None, wheelbase=None):
    """A car from scratch before it is made: {"mass", "cg" (SAE), "front_share" (of the weight on the front axle),
    "rows": [{"what", "mass", "position"}]}. engine: {"mass", "position"} (SAE), wheels: {corner: {"center"}}."""
    rows = []
    if parts:
        res = skeleton.build(parts, 1.0, (0, 0, 0), (0.0, 0.0, 0.0), (opts or {}).get("tol") or skeleton.TOL,
                             (opts or {}).get("flat") or skeleton.FLAT, (opts or {}).get("kinds"))
        kg, _, _, _ = skeleton.values(res, (opts or {}).get("tubes"), (opts or {}).get("min_kg") or skeleton.MIN_KG)
        m, cg = combine([(kg[n], v["pos"]) for n, v in res["nodes"].items() if n in kg])
        if m:
            rows.append({"what": "structure (the sketch's nodes)", "mass": round(m, 1), "position": [round(x, 3) for x in cg]})
    if engine and isinstance(engine.get("mass"), (int, float)) and engine.get("position"):
        rows.append({"what": "engine", "mass": float(engine["mass"]), "position": engine["position"]})
    for corner, w in sorted((wheels or {}).items()):
        if w.get("center"):
            rows.append({"what": f"wheel {corner}", "mass": float(w.get("mass") or WHEEL_KG), "position": w["center"]})
    for c in components or []:
        if isinstance(c.get("mass"), (int, float)) and c.get("position"):
            rows.append({"what": c.get("id") or c.get("kind") or "component", "mass": float(c["mass"]), "position": c["position"]})
    m, cg = combine([(r["mass"], r["position"]) for r in rows])
    wb = wheelbase or _wheelbase(wheels)
    fs = None
    if cg and wb:
        fs = round(min(1.0, max(0.0, 1 + cg[0] / wb)), 4)           # X 0 at the front axle, -wb at the rear one
    return {"mass": round(m, 1), "cg": [round(x, 4) for x in cg] if cg else None, "front_share": fs, "rows": rows}


def _wheelbase(wheels):
    xs = [w["center"][0] for w in (wheels or {}).values() if w.get("center")]
    return round(max(xs) - min(xs), 4) if len(xs) >= 2 and max(xs) - min(xs) > 0.5 else None


def ballast(now_mass, now_cg, target_mass, target_cg=None):
    """The ballast that brings a total of now_mass at now_cg to target_mass (and to target_cg when given):
    {"mass", "position"} or None when the car is already as heavy (ballast cannot take mass away)."""
    mb = (target_mass or 0) - now_mass
    if mb <= 0.5:
        return None
    if target_cg and now_cg:
        pos = [(target_mass * target_cg[i] - now_mass * now_cg[i]) / mb for i in range(3)]
    else:
        pos = list(now_cg) if now_cg else [0.0, 0.0, -0.3]
    return {"mass": round(mb, 1), "position": [round(x, 4) for x in pos]}


def spread(nodes, candidates, mass, pos, k=NEAREST):
    """{node: kg} of a component's mass on its k nearest candidate nodes, the nearer the more (1 / distance), so their
    weighted centre is close to the component's place. nodes: {id: [x, y, z]} in the same frame as pos."""
    near = sorted(candidates, key=lambda n: math.dist(nodes[n], pos))[:k]
    if not near:
        return {}
    w = {n: 1.0 / max(0.05, math.dist(nodes[n], pos)) for n in near}
    tot = sum(w.values())
    return {n: mass * w[n] / tot for n in near}


def estimate_json(spec_json):
    s = json.loads(spec_json)
    return json.dumps(estimate(s.get("parts"), s.get("opts"), s.get("engine"), s.get("wheels"), s.get("components"), s.get("wheelbase")))


def ballast_json(now_mass, now_cg_json, target_mass, target_cg_json=None):
    return json.dumps(ballast(now_mass, json.loads(now_cg_json) if now_cg_json else None, target_mass,
                              json.loads(target_cg_json) if target_cg_json else None))


def svj_json(svj_json_):
    svj = json.loads(svj_json_)
    m, cg = svj_target(svj)
    return json.dumps({"components": svj_components(svj), "mass": m, "cg": cg})


def base_json(model, configured_json):
    """A base vehicle's mass and CG (values.mass_and_cg): {"mass", "cg_behind_front_axle", "cg_height"}."""
    from . import values
    return json.dumps(values.mass_and_cg(model, json.loads(configured_json)))
