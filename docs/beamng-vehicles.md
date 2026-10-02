# BeamNG vehicles: how they are built and modified

BeamForge starts from any BeamNG vehicle (vanilla or mod) and modifies it the way the game's own Vehicle Config menu does. This page records how the game puts a vehicle together, as learned from the vanilla files. No game file is part of BeamForge: the editor reads the user's own install, in their browser, and only the facts below are written down here.

## Where the files are

| | Location |
| --- | --- |
| Vanilla vehicles | `BeamNG.drive/content/vehicles/<model>.zip`, one zip per vehicle or prop |
| Shared parts | `content/vehicles/common.zip`: wheels, tyres, brakes, seats, cargo and other parts many vehicles use |
| User folder | `%LOCALAPPDATA%\BeamNG\BeamNG.drive\current\` (older versions: `%LOCALAPPDATA%\BeamNG.drive\<version>\`; the launcher can move it). `BeamNG.drive.ini` beside it names the install and a custom user folder |
| Mods | in the user folder: `mods/*.zip`, `mods/repo/*.zip` (downloaded from the repository) and `mods/unpacked/<mod>/`. `mods/db.json` lists every mod with `active` and `fullpath` (`/mods/repo/x.zip`) |
| Own files | the user folder's `vehicles/<model>/`: saved configs (`.pc` and their thumbnails), and anything else a player puts there |

The game reads all of them as one file system. When the same path is in several places, the user folder wins over mods and mods over the install. Mods often ship more than one vehicle: a mod zip can add configs or parts to a vanilla vehicle's folder (`vehicles/hatchback/…`), and many ship parts in `vehicles/common/`, which every vehicle can use. The order between two mods that ship the same file is not documented; BeamForge takes them by path (to confirm in-game). Mods switched off in `db.json` are skipped.

A vehicle zip is mostly meshes and textures (`.dae` / `.cdae`, `.dds`, `.jpg`). The editor reads its text files (`.jbeam`, `.pc`, `.json`, typically 0.5–6 MB per vehicle against 100–800 MB for the whole zip), and the `.dae` meshes a configuration uses, entry by entry with zip.js, without loading the archive. It does not read textures.

Survey of one 2026 install (122 zips): 28 cars, 10 trucks, 13 trailers, 2 heavy machines and 67 props, with 4,199 jbeam, 1,709 config and 2,548 json files. All of them parse with BeamForge's lenient jbeam reader. Mods are less tidy: some files end with an extra `}`, which the game accepts, and so does the reader.

## A vehicle

| Piece | What it is |
| --- | --- |
| Part | An entry of a `.jbeam` file, `{"part_name": {"slotType": ..., "nodes": ..., "beams": ...}}`. It fits a slot whose type matches its `slotType` (a string, or a list of types). |
| Main part | The part with `slotType: "main"`, the root of the tree (named in the config as `mainPartName`, newer configs also give `mainPartPath`). |
| Slots | The child slots a part offers, in one of two tables. `slots`: `[type, default, description, {options}]`, where the slot is named after its type (1,762 vanilla parts). `slots2`: `[name, allowTypes, denyTypes, default, description, {options}]` (319 parts, the newer form). |
| Slot options | `coreSlot` (must hold a part), `nodeOffset` and `nodeMove` (shift the nodes of the part in the slot and its children; `nodeOffset` x is applied away from the centre line, so one value places both sides), `nodeMove2`, `nodeRotate`, and `variables` (values passed down). |
| Config (`.pc`) | `{"format": 2, "model", "mainPartName", "parts": {slot name: part name or ""}, "vars": {"$name": value}}`, plus `licenseName`, `paints` or `coupledNodes` (trailers) when used. A slot missing from `parts` takes its default; `""` leaves it empty. |
| Info | `info.json` per vehicle (Name, Brand, Type, Body Style, `default_pc`, paints) and `info_<config>.json` per config (Configuration, Config Type, Power, Torque, Weight, Value, Drivetrain, Transmission, performance figures). Many names are keys into the game's translation files (`vehiclesData.<model>.Name`), which are not in the vehicle files: the editor shows a readable id instead. |
| Wheels | Not in the node list: the game builds each wheel's nodes when the vehicle spawns, from `pressureWheels`. The tables of all active parts read as one: a property set in one part (`radius`, `tireWidth` by the tyre; `hubRadius`, `hubWidth`, `wheelOffset` by the rim) applies to the wheel rows of later parts (the wheel data part: name, hubGroup, group, node1, node2 …). |
| Meshes (flexbodies) | `flexbodies: [mesh, [node groups], nonFlexMaterials, {pos, rot, scale}]`. The mesh is a named node of a COLLADA `.dae` in the vehicle's folder or `vehicles/common` (vanilla zips ship `.dae` beside the compiled `.cdae`). Body meshes are modelled in place, in the jbeam's axes and metres, and follow the nodes of their groups. Wheel meshes (their groups name a wheel's group or hubGroup) are modelled around the origin with the axle on x, and placed on the wheel. |
| Expressions | Values like `"$=case($trackwidth_F == nil, $trackoffset_F + 0.25, $trackwidth_F)"` or `"$=$x == nil and -0.5 or $x - 0.5"` are Lua: `case()`, `nil`, `and` / `or`, `~=`, `^`, `sin()`, `round()`. BeamForge's jbeam reader (`beamforge/jbeam.py`) evaluates them with a small, safe evaluator (no code execution, bounded powers). |

## How the game lets a player modify a vehicle

1. **Parts.** The Parts menu shows the slot tree. For every slot it lists the parts that fit (same type, not denied), plus "empty" unless the slot is a core slot. Choosing a part rebuilds the tree below it: the new part brings its own slots.
2. **Tuning.** Parts declare `variables`: `[name, "range", unit, category, default, min, max, title, description, {subCategory, stepDis, minDis, maxDis, hideInUI}]`. The Tuning menu shows the variables of the active parts as sliders, grouped by category (Wheels, Suspension, Engine, Transmission, Differentials, Brakes, Wheel Alignment, Chassis ...) and sub-category (Front, Rear). Many variables feed expressions, so a slider can move nodes (track width, ride height) as well as change rates.
3. **Paint.** Up to three paints per vehicle (`paints` in the config, palettes in `info.json`).
4. **Save.** The game saves the result as a `.pc` in the user folder, `vehicles/<model>/<name>.pc`.

## The editor's base vehicle

- **BeamNG folders:** add the install (the folder itself, `content/` or `content/vehicles/`) and the user folder (`BeamNG.drive/`, `current/` or a version folder). The editor tells which one was picked. A single unpacked mod folder (with `vehicles/` inside) or single zips can be added too. The vehicle list comes from every `info.json` and config across them, cars first; vehicles from a mod are tagged.
- **Remembered folders:** in Chrome and Edge, *Game folder* and *Mods folder* keep the folder handles in IndexedDB; the next session reopens them, or asks for one click to confirm access. A page cannot open a picker at a given path: each button has its own picker id, so the browser reopens it where it was last used (or in the folder already added). Chrome refuses folders under AppData and Program Files there, which is where the user folder is by default: *With file dialog* reads any folder, but only for the session, and opens wherever the browser last was. Moving the user folder in the BeamNG launcher, or a Steam library outside Program Files, makes them rememberable.
- **A vehicle:** its configurations, the parts tree with a menu per slot, tuning sliders by category, and its node-and-beam structure in 3D. Shared parts (`vehicles/common` of the install and of every mod) are read once per library; after that a part or tuning change rebuilds the tree in a few hundredths of a second.
- **Meshes and wheels:** the active parts' flexbodies are drawn untextured from the game's `.dae` files, each part tinted with its own colour (also on its nodes and beams). Every vertex of a body mesh follows its nearest node in the flexbody's groups, so slot offsets, tuning and the user's moves bend the mesh. Wheels are drawn from `pressureWheels`; wheel meshes sit on the wheel centre, which BeamForge takes as the outer axle node moved by `wheelOffset` (to confirm in-game). Mods that ship only `.cdae` show no meshes.
- **Hide and lock:** each part of the tree can be hidden (not drawn, not pickable) or locked (drawn, not pickable, cannot be moved), so it is not changed by accident.
- **Move:** click a node or a beam in the view (or *move* beside a part in the tree) and type its x / y / z in metres, BeamNG axes (x left, y rear, z up). A node takes a new position; a beam a new midpoint (both its nodes move by the same amount); a part an offset that also moves the parts in its slots, like a slot `nodeMove` in the game. Moves show in the 3D view and in the measurements (wheelbase, tracks) at once. They are not in the saved `.pc` yet.
- **Save configuration (.pc):** downloads the config, to be placed in the user folder at `vehicles/<model>/`.

Code: `beamforge/beamng.py` (which source serves each file, reading, the parts tree, tuning, geometry, `.pc` writing), `editor/library.js` (folders, zips, remembered handles), the base vehicle section of `editor/app.js` (panels), tests in `tests/test_beamng.py` (made-up vehicles; set `BEAMNG_VEHICLES` to a `content/vehicles` folder to also build every car of a local install).

Not yet: textures and paints, `.cdae` meshes, and saving a `.pc` straight into the user folder. See [roadmap.md](roadmap.md).
