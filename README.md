# BeamForge

Start from **any BeamNG.drive vehicle**, vanilla or mod, and modify it from there. Lay an **SVJ** ([Standard Vehicle JSON](https://github.com/RFloEng/SVJ-standard-vehicle-json)) over it to compare and, step by step, take its geometry, setup and meshes.

> **Status: v0.1 prototype.** Base vehicle and SVJ overlay work. Applying SVJ values and exporting meshes to the game are next: see the [roadmap](docs/roadmap.md).

## What it does today

- **Base vehicle.** Pick your BeamNG `content/vehicles` folder (or a mod's unpacked folder, or single zips). You get the vehicle list, the parts tree with the alternatives for every slot, the tuning sliders, and the node-and-beam structure in 3D. Save the result as a `.pc` configuration for the game. Only the jbeam, config and info text files are read, in your browser. Nothing is uploaded.
- **SVJ overlay.** Import an `.svj.json` with its meshes, or a `.zip` bundle. Its glTF meshes and suspension hardpoints are placed on the base vehicle's front axle and ground. The visual bindings are checked, and wheelbase, tracks and mass are compared with the base vehicle.

## Run it

```bash
git clone https://github.com/RFloEng/BeamForge.git
cd BeamForge
python -m http.server 8000
```

Then open <http://localhost:8000/editor/>. The page runs the Python package in the browser with Pyodide, so it must be served from the repo root.

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
beamforge/gltf.py      minimal glTF 2.0 reader / writer
editor/                the browser editor (three.js + Pyodide)
docs/beamng-vehicles.md  how BeamNG builds and modifies vehicles
docs/roadmap.md        what comes next
tests/                 unit tests on made-up vehicles and SVJ files
```

## Game files

The repository contains no BeamNG game files, and none may be committed. The editor reads your own install locally. BeamForge is not affiliated with or endorsed by BeamNG GmbH.

## License

[Apache License 2.0](LICENSE)
