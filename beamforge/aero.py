"""Aerodynamics: the SVJ's (or the user's) drag and lift, their forces per axle at speed, and BeamNG's aero triangles.

Pure Python, standard library only (runs in Pyodide).

From the numbers (SVJ §13): drag D = q Cd A and lift L = q Cl A with q = rho v^2 / 2. The lift goes to the axles from
Cl_front and Cl_rear where the file gives them, else from the centre of pressure's place between the axles, else half
and half. Downforce is minus the lift.

In BeamNG every triangle of a vehicle is in the air: it pushes back with its dragCoef and sideways to itself with its
liftCoef (both in percent, liftCoef defaulting to dragCoef; lua/vehicle/jbeam/stage2.lua), against the flow it faces.
The forces' exact law is in the game's compiled physics, not public. BeamForge's estimate of a vehicle's drag area is
sum(dragCoef/100 x area x (n . y)^2) (values.drag_area; about 0.67 m2 for a vanilla saloon of Cd ~0.3). A car made
from scratch has no body triangles: it gets one drag plate, a non-colliding triangle on its front nodes facing the flow,
sized by that estimate to the drag area asked for, and no lift. Downforce devices (wings) are not generated yet.
"""

import json
import math

RHO = 1.225
SPEEDS = (80, 120, 160, 200)


def read(svj):
    """{"area", "Cd", "Cl", "Cl_front", "Cl_rear", "cop" ([x, y, z] SAE), "rho", "components": [{"id", "type",
    "position", "area", "Cd", "Cl"}]} of the SVJ's aerodynamics (None where it has no value)."""
    a = (svj or {}).get("aerodynamics") or {}
    ref, co = a.get("reference") or {}, a.get("coefficients") or {}
    comps = a.get("components") if isinstance(a.get("components"), list) else []
    num = lambda v: float(v) if isinstance(v, (int, float)) else None   # noqa: E731
    out = {"area": num(ref.get("frontal_area")), "Cd": num(co.get("Cd")), "Cl": num(co.get("Cl")),
           "Cl_front": num(co.get("Cl_front")), "Cl_rear": num(co.get("Cl_rear")),
           "cop": a.get("center_of_pressure") if isinstance(a.get("center_of_pressure"), list) else None,
           "rho": num(ref.get("air_density")) or RHO,
           "components": [{"id": c.get("id"), "type": c.get("type"), "position": c.get("position"), "area": num(c.get("area")),
                           "Cd": num(c.get("Cd_contribution")), "Cl": num(c.get("Cl_contribution"))} for c in comps if isinstance(c, dict)]}
    if out["Cd"] is None and out["components"]:
        cds = [c["Cd"] for c in out["components"] if c["Cd"] is not None]
        out["Cd"] = sum(cds) if cds else None
    if out["Cl"] is None and out["components"]:
        cls = [c["Cl"] for c in out["components"] if c["Cl"] is not None]
        out["Cl"] = sum(cls) if cls else None
    return out


def front_share(a, wheelbase):
    """The front axle's share of the lift: Cl_front over Cl where both are given, else from the centre of pressure
    (SAE x: 0 at the front axle, -wheelbase at the rear one), else a half."""
    if a.get("Cl_front") is not None and a.get("Cl_rear") is not None and (a["Cl_front"] + a["Cl_rear"]):
        return a["Cl_front"] / (a["Cl_front"] + a["Cl_rear"])
    if a.get("cop") and wheelbase:
        return min(1.0, max(0.0, 1 + float(a["cop"][0]) / wheelbase))
    return 0.5


def forces(a, wheelbase=2.5, speeds=SPEEDS):
    """[{"kmh", "drag" (N), "power" (kW to push through the air), "down_front", "down_rear" (N)}] at each speed."""
    out = []
    area, cd = a.get("area"), a.get("Cd")
    cl = a.get("Cl") if a.get("Cl") is not None else ((a.get("Cl_front") or 0) + (a.get("Cl_rear") or 0)
                                                       if a.get("Cl_front") is not None or a.get("Cl_rear") is not None else None)
    fs = front_share(a, wheelbase)
    for kmh in speeds:
        v = kmh / 3.6
        q = 0.5 * (a.get("rho") or RHO) * v * v
        drag = q * cd * area if cd is not None and area else None
        lift = q * cl * area if cl is not None and area else None
        out.append({"kmh": kmh, "drag": round(drag, 1) if drag is not None else None,
                    "power": round(drag * v / 1000, 2) if drag is not None else None,
                    "down_front": round(-lift * fs, 1) + 0.0 if lift is not None else None,        # + 0.0: no -0
                    "down_rear": round(-lift * (1 - fs), 1) + 0.0 if lift is not None else None})
    return out


def base_json(model, configured_json):
    """A base vehicle's aero by BeamForge's estimate: {"cda" (m2), "triangles"}."""
    from . import values
    tris = values.aero_triangles(model, json.loads(configured_json))
    return json.dumps({"cda": round(values.drag_area(tris), 4), "triangles": len(tris)})


def edit_from_svj_json(svj_json, wheelbase=2.5):
    """The Aero workspace's numbers from the SVJ ({"cd", "area", "cl_front", "cl_rear"}; a total Cl split between the
    axles by front_share), or null where it has no drag."""
    a = read(json.loads(svj_json))
    if not a["Cd"] or not a["area"]:
        return json.dumps(None)
    if a["Cl_front"] is not None or a["Cl_rear"] is not None:
        f, r = a["Cl_front"] or 0.0, a["Cl_rear"] or 0.0
    elif a["Cl"] is not None:
        fs = front_share(a, wheelbase)
        f, r = a["Cl"] * fs, a["Cl"] * (1 - fs)
    else:
        f = r = None
    return json.dumps({"cd": a["Cd"], "area": a["area"], "cl_front": None if f is None else round(f, 4),
                       "cl_rear": None if r is None else round(r, 4), "rho": a["rho"]})


def read_json(svj_json):
    return json.dumps(read(json.loads(svj_json)))


def forces_json(aero_json, wheelbase=2.5):
    return json.dumps(forces(json.loads(aero_json), wheelbase))


def drag_plate(nodes, candidates, cda, front_y=None):
    """The drag plate of a car from scratch: three of its nodes near its front (BeamNG: the smallest y) that make the
    triangle facing the flow most squarely, and the dragCoef (percent) that gives it the drag area cda by
    values.drag_area's estimate. Returns (ids, dragCoef) or None."""
    if not cda or len(candidates) < 3:
        return None
    ys = sorted(nodes[n][1] for n in candidates)
    lim = (front_y if front_y is not None else ys[0]) + 0.6
    front = [n for n in candidates if nodes[n][1] <= lim]
    if len(front) < 3:
        front = sorted(candidates, key=lambda n: nodes[n][1])[:8]
    best = None
    for i in range(len(front)):
        for j in range(i + 1, len(front)):
            for k in range(j + 1, len(front)):
                a, b, c = (nodes[n] for n in (front[i], front[j], front[k]))
                u = [b[m] - a[m] for m in range(3)]
                w = [c[m] - a[m] for m in range(3)]
                n = [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]]
                a2 = math.sqrt(sum(x * x for x in n))
                if a2 < 1e-6:
                    continue
                eff = a2 / 2 * (n[1] / a2) ** 2                 # area x (n . y)^2
                if best is None or eff > best[0]:
                    best = (eff, [front[i], front[j], front[k]])
    if not best or best[0] < 1e-3:
        return None
    return best[1], round(cda / best[0] * 100, 2)
