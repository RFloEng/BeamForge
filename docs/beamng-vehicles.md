# BeamNG vehicles: how they are built and modified

BeamForge starts from any BeamNG vehicle (vanilla or mod) and modifies it the way the game's own Vehicle Config menu does. This page records how the game puts a vehicle together, as learned from the vanilla files. No game file is part of BeamForge: the editor reads the user's own install, in their browser, and only the facts below are written down here.

## Where the files are

| | Location |
| --- | --- |
| Vanilla vehicles | `BeamNG.drive/content/vehicles/<model>.zip`, one zip per vehicle or prop |
| Shared parts | `content/vehicles/common.zip`: wheels, tyres, brakes, seats, cargo and other parts many vehicles use |
| Mods and saved configs | the user folder, `vehicles/<model>/` (unpacked) or `mods/*.zip` |

A vehicle zip is mostly meshes and textures (`.dae` / `.cdae`, `.dds`, `.jpg`). The editor reads only its text files (`.jbeam`, `.pc`, `.json`): typically 0.5–6 MB per vehicle against 100–800 MB for the whole zip. It reads them entry by entry with zip.js, without loading the archive.

Survey of one 2026 install (122 zips): 28 cars, 10 trucks, 13 trailers, 2 heavy machines and 67 props, with 4,199 jbeam, 1,709 config and 2,548 json files. All of them parse with BeamForge's lenient jbeam reader.

## A vehicle

| Piece | What it is |
| --- | --- |
| Part | An entry of a `.jbeam` file, `{"part_name": {"slotType": ..., "nodes": ..., "beams": ...}}`. It fits a slot whose type matches its `slotType` (a string, or a list of types). |
| Main part | The part with `slotType: "main"`, the root of the tree (named in the config as `mainPartName`, newer configs also give `mainPartPath`). |
| Slots | The child slots a part offers, in one of two tables. `slots`: `[type, default, description, {options}]`, where the slot is named after its type (1,762 vanilla parts). `slots2`: `[name, allowTypes, denyTypes, default, description, {options}]` (319 parts, the newer form). |
| Slot options | `coreSlot` (must hold a part), `nodeOffset` and `nodeMove` (shift the nodes of the part in the slot and its children; `nodeOffset` x is applied away from the centre line, so one value places both sides), `nodeMove2`, `nodeRotate`, and `variables` (values passed down). |
| Config (`.pc`) | `{"format": 2, "model", "mainPartName", "parts": {slot name: part name or ""}, "vars": {"$name": value}}`, plus `licenseName`, `paints` or `coupledNodes` (trailers) when used. A slot missing from `parts` takes its default; `""` leaves it empty. |
| Info | `info.json` per vehicle (Name, Brand, Type, Body Style, `default_pc`, paints) and `info_<config>.json` per config (Configuration, Config Type, Power, Torque, Weight, Value, Drivetrain, Transmission, performance figures). Many names are keys into the game's translation files (`vehiclesData.<model>.Name`), which are not in the vehicle files: the editor shows a readable id instead. |
| Expressions | Values like `"$=case($trackwidth_F == nil, $trackoffset_F + 0.25, $trackwidth_F)"` or `"$=$x == nil and -0.5 or $x - 0.5"` are Lua: `case()`, `nil`, `and` / `or`, `~=`, `^`, `sin()`, `round()`. BeamForge's jbeam reader (`beamforge/jbeam.py`) evaluates them with a small, safe evaluator (no code execution, bounded powers). |

## How the game lets a player modify a vehicle

1. **Parts.** The Parts menu shows the slot tree. For every slot it lists the parts that fit (same type, not denied), plus "empty" unless the slot is a core slot. Choosing a part rebuilds the tree below it: the new part brings its own slots.
2. **Tuning.** Parts declare `variables`: `[name, "range", unit, category, default, min, max, title, description, {subCategory, stepDis, minDis, maxDis, hideInUI}]`. The Tuning menu shows the variables of the active parts as sliders, grouped by category (Wheels, Suspension, Engine, Transmission, Differentials, Brakes, Wheel Alignment, Chassis ...) and sub-category (Front, Rear). Many variables feed expressions, so a slider can move nodes (track width, ride height) as well as change rates.
3. **Paint.** Up to three paints per vehicle (`paints` in the config, palettes in `info.json`).
4. **Save.** The game saves the result as a `.pc` in the user folder, `vehicles/<model>/<name>.pc`.

## The editor's base vehicle

- **Choose vehicles folder:** pick `BeamNG.drive/content/vehicles` (or a mod folder, or single zips). The vehicle list comes from each zip's `info.json` and configs, cars first.
- **A vehicle:** its configurations, the parts tree with a menu per slot, tuning sliders by category, and its node-and-beam structure in 3D. Shared parts are read from `common.zip` once per session; after that a part or tuning change rebuilds the tree in a few hundredths of a second.
- **Save configuration (.pc):** downloads the config, to be placed in the user folder at `vehicles/<model>/`.

Code: `beamforge/beamng.py` (reading, the parts tree, tuning, geometry, `.pc` writing), the base vehicle section of `editor/app.js` (folder picking, zip reading, panels), tests in `tests/test_beamng.py` (made-up vehicles; set `BEAMNG_VEHICLES` to a `content/vehicles` folder to also build every car of a local install).

Not yet: flexbody meshes (only the node-and-beam structure is drawn), paints, mod zips from the user folder, and remembering the picked folders. See [roadmap.md](roadmap.md).
