# BeamForge

Start from **any BeamNG.drive vehicle**, vanilla or mod, and modify it from there. Lay an **SVJ** ([Standard Vehicle JSON](https://github.com/RFloEng/SVJ-standard-vehicle-json)) over it to compare and, step by step, take its geometry, setup and meshes.

> **Status: v0.2 prototype.** Base vehicle and SVJ overlay work. Applying SVJ values and exporting meshes to the game are next: see the [roadmap](docs/roadmap.md).

## What it does today

- **Base vehicle.** Add your BeamNG install and user folder once. Mods (zipped, from the repository or unpacked, active ones only) and your own `vehicles/` are read the game's way: the user folder over mods over the install, shared `vehicles/common` parts from mods included. You get the vehicle list, the parts tree with the alternatives for every slot, the tuning sliders, and the vehicle in 3D: its meshes (from the game's `.dae` files, untextured), wheels, nodes and beams, each part in its own colour. Parts can be hidden or locked. Save the result as a `.pc` configuration for the game. Only the jbeam, config and info text files are read, in your browser. Nothing is uploaded. Chrome and Edge remember the folders, except under AppData and Program Files (see [docs/beamng-vehicles.md](docs/beamng-vehicles.md#the-editors-base-vehicle)).
- **Move nodes, beams and parts.** Click a node or a beam, or pick a part in the tree, and type its x / y / z. The structure and the measurements follow. Moves stay in the editor for now.
- **Fit to the SVJ.** One click bends the base vehicle onto the SVJ in three stages: the wheelbase, the body to the SVJ's body mesh (overhangs, floor, width and roof line), then the suspension pickup points exactly onto the SVJ hardpoints. Hardpoints are tied to nodes by their role; the table shows each tie, and a click on a node re-ties it. Each stage can be taken or left; hand moves stay on top. See [docs/fitting.md](docs/fitting.md).
- **Suspension study.** The SVJ's corners are solved over ±100 mm of wheel travel (the kinematics from FBeam): camber, toe, track change, roll centre, motion ratio and wheel rate curves, kingpin, caster, scrub and trail, with the linkage moving in 3D. Corners that are over- or under-constrained, or files with an upward Z axis, are named.
- **SVJ overlay.** Import an `.svj.json` with its meshes, or a `.zip` bundle. Its glTF meshes and suspension hardpoints are placed on the base vehicle's front axle and ground. The visual bindings are checked, and wheelbase, tracks and mass are compared with the base vehicle.

## Run it

```bash
git clone https://github.com/RFloEng/BeamForge.git
cd BeamForge
python serve.py
```

Then open <http://localhost:8000/editor/> in Chrome or Edge. The page runs the Python package in the browser with Pyodide, so it must be served from the repo root. `serve.py` is `python -m http.server` that tells the browser not to cache, so an updated editor always loads in full.

Tests (standard library only):

```bash
python -m unittest discover -s tests -t .
```

Set `BEAMNG_VEHICLES` to your `content/vehicles` folder to also build every vehicle of your install in `tests/test_beamng.py`.

## Layout

```
beamforge/jbeam.py     lenient jbeam reader, safe expression evaluator
beamforge/beamng.py    vehicles as the game builds them: parts tree, tuning, configs, geometry, measurements
beamforge/svj.py       SVJ bundles, glTF bindings, hardpoints in BeamNG coordinates, base-vs-SVJ comparison
beamforge/fit.py       fitting the base vehicle to an SVJ, in stages (docs/fitting.md)
beamforge/suspension.py  suspension kinematics over wheel travel, for SVJ corners (from FBeam)
beamforge/gltf.py      minimal glTF 2.0 reader / writer
editor/                the browser editor (three.js + Pyodide); library.js finds the BeamNG folders
docs/beamng-vehicles.md  how BeamNG builds and modifies vehicles
docs/roadmap.md        what comes next
tests/                 unit tests on made-up vehicles and SVJ files
```

## Game files

The repository contains no BeamNG game files, and none may be committed. The editor reads your own install locally. BeamForge is not affiliated with or endorsed by BeamNG GmbH.

## License

[Apache License 2.0](LICENSE)
