"""Make a BeamNG vehicle from an SVJ and a base vehicle of your own install, as the editor's export does.

    python tools/svj_to_beamng.py path/to/car.svj.json --base <model> --id bf_mycar --name "My car (BF)"
        [--vehicles <BeamNG>/content/vehicles] [--config <pc name>] [--stages wheelbase,body,pickups]
        [--no-meshes] [--keep-body] [--scale-structure] [--out out/] [--install <user folder>/mods]

Steps (beamforge/pipeline.py): the base's parts nearest the SVJ, the fit, every SVJ value, the SVJ's meshes in place of
the base's body, the export, and the base's assets copied. The mod zip is read back the way the game reads it, and the
result is printed: missing parts, wheelbase, tracks, mass, the stiffest nodes.

--vehicles defaults to the BEAMNG_VEHICLES environment variable: the folder with the game's vehicle zips (the base's
<model>.zip and common.zip). Everything is read from your install; nothing of the game is kept here. --install copies
the zip into a mods folder: close the game first, and clear the vehicle's cache (temp/vehicles/<id>) if an older build
of it was loaded.
"""

import argparse
import base64
import json
import os
import shutil
import sys
import zipfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from beamforge import beamng, pipeline, rigidity, values  # noqa: E402

TEXT = (".jbeam", ".pc", ".json")


def load_zip(path):
    with zipfile.ZipFile(path) as z:
        beamng.add_files(json.dumps({n: z.read(n).decode("utf-8-sig", "replace") for n in z.namelist() if n.lower().endswith(TEXT)}))
        return [n for n in z.namelist() if not n.endswith("/")]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("svj", help="the .svj.json (its meshes are looked for beside it)")
    ap.add_argument("--base", required=True, help="the base vehicle's model folder name")
    ap.add_argument("--id", required=True, help="the new vehicle's folder name (lower case, digits, _)")
    ap.add_argument("--name", help="the name in the game (default: the id)")
    ap.add_argument("--brand", default="BeamForge")
    ap.add_argument("--vehicles", default=os.environ.get("BEAMNG_VEHICLES"), help="the game's content/vehicles folder")
    ap.add_argument("--config", help="the base configuration (default: its default .pc)")
    ap.add_argument("--stages", default=",".join(pipeline.STAGES), help="fit stages, comma separated; empty for no fit")
    ap.add_argument("--no-meshes", action="store_true", help="keep the base's meshes, add none of the SVJ's")
    ap.add_argument("--keep-body", action="store_true", help="add the SVJ's meshes beside the base's body, not in its place")
    ap.add_argument("--scale-structure", action="store_true",
                    help="reach the SVJ's mass: lighter nodes, the body's beams softened by the same factor")
    ap.add_argument("--out", default="out", help="folder for the mod zip")
    ap.add_argument("--install", help="a mods folder to copy the zip into")
    a = ap.parse_args(argv)
    if not a.vehicles or not os.path.isdir(a.vehicles):
        ap.error("--vehicles (or BEAMNG_VEHICLES) must be the game's content/vehicles folder")
    base_zip = os.path.join(a.vehicles, a.base + ".zip")
    if not os.path.exists(base_zip):
        ap.error(f"no {a.base}.zip in {a.vehicles}")

    svj = json.load(open(a.svj, encoding="utf-8"))
    load_zip(os.path.join(a.vehicles, "common.zip"))
    base_paths = load_zip(base_zip)
    stages = [s for s in a.stages.split(",") if s.strip()]
    r = pipeline.run(a.base, svj, a.id, a.name or a.id, a.brand, a.config, os.path.dirname(os.path.abspath(a.svj)), stages,
                     meshes=not a.no_meshes, replace=not a.keep_body, base_paths=base_paths,
                     scale_structure=a.scale_structure)
    out = r["out"]
    print("parts suggested:", r["parts"] or "none")
    print("fit:", len(r["fit"]["moves"]), "nodes moved")
    for n in r["fit"]["changes"] + r["fit"]["notes"]:
        print("   ", n)
    print("values taken:", ", ".join(sorted(r["taken"])) or "none")
    for x in r["table"]:
        if x["svj"] is None:
            print(f"    not in the SVJ: {x['key']}")
    print("export:", out["counts"])
    for n in out["notes"]:
        print("   ", n)

    os.makedirs(a.out, exist_ok=True)
    zpath = os.path.join(a.out, a.id + ".zip")
    with zipfile.ZipFile(zpath, "w", zipfile.ZIP_DEFLATED) as zo, zipfile.ZipFile(base_zip) as zb:
        for p, text in out["files"].items():
            zo.writestr(p, text)
        for p, b64 in (out.get("binary") or {}).items():
            zo.writestr(p, base64.b64decode(b64))
        for src, dst in r["copies"].items():
            zo.writestr(dst, zb.read(src))

    beamng.reset()                                       # read back as the game would: common and the new folder
    load_zip(os.path.join(a.vehicles, "common.zip"))
    load_zip(zpath)
    nv = json.loads(beamng.configure(a.id))
    m = nv.get("measure") or {}
    print("read back:", nv["config"], "| missing parts:", nv["missing"] or "none")
    print("   wheelbase", m.get("wheelbase"), "| tracks", m.get("track_front"), m.get("track_rear"),
          "| wheels", [(w["name"], round(w.get("radius") or 0, 3)) for w in nv["wheels"]])
    print("   mass and CG", values.mass_and_cg(a.id, nv))
    print("   stiffest nodes (k index, vanilla cars reach 7.5):", [(n, round(k, 2)) for n, k, _, _ in rigidity.stability(a.id, nv)[:5]])
    print(zpath, os.path.getsize(zpath) // 1024, "KB")
    if a.install:
        shutil.copy(zpath, os.path.join(a.install, a.id + ".zip"))
        print("installed in", a.install, "(close the game first; clear temp/vehicles/" + a.id, "if an older build was loaded)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
