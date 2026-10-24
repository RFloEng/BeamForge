"""An SVJ onto a base vehicle, made into a new vehicle in one call: the editor's export path without the editor.

Pure Python, standard library only. The base vehicle's files must already be loaded (beamng.add_files); the steps are
the editor's, in its order:

  parts     the base's alternatives nearest the SVJ (values.suggest_parts: wheels, intake, gearbox, drive layout...)
  fit       the fit to the SVJ (fit.py: wheelbase, body to the mesh, pickup points; any of them)
  values    every value the values table has an SVJ figure for (springs, dampers, tyres, engine, gears, final drive,
            mass and CG, steering, drag), as with every "take" box ticked
  meshes    the SVJ's meshes on the base's parts (export.svj_attach), in place of the base's body or beside it
  export    the new vehicle's files (export.build) and the base's assets to copy (export.assets)

tools/svj_to_beamng.py runs it from the command line on the user's own install.
"""

import json
import os

from beamforge import beamng, export, fit, suspension, unified, values

STAGES = ("wheelbase", "body", "pickups")


def svj_mesh_files(svj, svj_dir):
    """{mesh id: path} of the SVJ's meshes found beside it (assets.meshes[].uri, relative to the SVJ's folder)."""
    out = {}
    for m in (svj.get("assets") or {}).get("meshes") or []:
        p = os.path.join(svj_dir, str(m.get("uri") or ""))
        if m.get("id") and m.get("uri") and os.path.exists(p):
            out[m["id"]] = p
    return out


def run(base, svj, new_id, name, brand="BeamForge", config=None, svj_dir=None, stages=STAGES, take=None,
        meshes=True, replace=True, base_paths=(), scale_structure=False, glass_break=True, tank_mode=None, diagnose=False, no_selfcol=False,
        link_tube=None):
    """The new vehicle from the base and the SVJ. take: the values table keys to take (None: every one the SVJ has a
    figure for); meshes: put the SVJ's meshes on it (replace: in place of the base's body); base_paths: every file of the
    base's folder, for the assets to copy; scale_structure: reach the SVJ's mass by scaling the body's beams with its
    nodes' weight (export.build). Returns {"out": export.build's result, "copies": {source path: new path},
    "parts", "fit", "table", "taken", "files" (the SVJ meshes)}. A v0.100+ SVJ (wheel stations, bodies, joints) is
    first given its corner view (unified.adopt), as the editor's import does."""
    unified.adopt(svj)
    v = json.loads(beamng.configure(base, config))
    cfg = v["config"]
    parts = {x["slot"]: x["suggested"] for x in json.loads(values.suggest_parts(base, json.dumps(v), json.dumps(svj)))
             if x.get("suggested") is not None}
    v = json.loads(beamng.configure(base, cfg, json.dumps(parts)))
    files = svj_mesh_files(svj, svj_dir) if (svj_dir and meshes) else {}
    r = {"moves": {}, "changes": [], "notes": [], "mapping": [], "axles": None, "place": None}
    if stages:
        r = json.loads(fit.fit_json(json.dumps(v["geometry"]), json.dumps(v["wheels"]), json.dumps(svj), json.dumps(files),
                                    json.dumps(list(stages))))
    configured = beamng.configure(base, cfg, json.dumps(parts), None, json.dumps({"nodes": r["moves"]}))
    cv = json.loads(configured)
    study = suspension.study_svj(svj, 100)
    table = json.loads(values.table(base, configured, json.dumps(svj), json.dumps({"corners": study["corners"]})))
    taken = {x["key"]: True for x in table if x["svj"] is not None and (take is None or x["key"] in take)}
    meas = cv.get("measure") or {}
    place = r.get("place") or {"yf": meas.get("front_axle_y") or 0.0, "ground": meas.get("ground_z") or 0.0}
    opt = {"svj": svj, "take": taken, "study": {"corners": study["corners"]}, "scale_structure": bool(scale_structure), "glass_break": bool(glass_break), "tank_mode": tank_mode, "diagnose": bool(diagnose), "no_selfcol": bool(no_selfcol),
           "link_tube": link_tube}
    if files:
        opt.update(files=files, place=place, replace=replace,
                   attach=json.loads(export.svj_attach(configured, json.dumps(svj), json.dumps(r.get("mapping") or []),
                                                       json.dumps(files), json.dumps(place))))
    if r.get("axles"):
        opt["axles"] = r["axles"]
    out = json.loads(export.build(base, new_id, name, configured, "{}", brand, json.dumps(opt)))
    copies = json.loads(export.assets(base, new_id, json.dumps(list(base_paths)))) if base_paths else {}
    return {"out": out, "copies": copies, "parts": parts, "fit": r, "table": table, "taken": taken, "files": files}
