# Roadmap

BeamForge starts from **any BeamNG vehicle** (vanilla or mod) and modifies it from there. An **SVJ** file can be laid over that vehicle, and the user chooses, parameter by parameter, what to take from it. Finally the SVJ's glTF meshes become the vehicle's visible mesh, mapped part by part onto the vehicle's part tree.

## Where it stands (v0.1)

| | State |
| --- | --- |
| Base vehicle | Reads `content/vehicles` zips and unpacked mod folders in the browser: parts tree, tuning, node-and-beam view, `.pc` save |
| SVJ | Reads an `.svj.json` with its meshes, or a `.zip` bundle. It checks the visual bindings and places the meshes and suspension hardpoints on the base vehicle's front axle and ground. It compares wheelbase, tracks and mass |
| Mesh | glTF meshes shown as translucent reference geometry |

## Steps

1. **Find the install and mods once.**
   - Pick the game's `content/vehicles` folder and the user folder (`%LOCALAPPDATA%\BeamNG.drive\<version>\` or `current`). Chrome and Edge can keep the folder handles in IndexedDB (File System Access API), so later sessions only confirm access.
   - Read `mods/*.zip`, `mods/unpacked/` and `vehicles/` as well. Resolve them in the game's order: user folder over mods over vanilla.
   - Read `vehicles/common` parts shipped inside mod zips. Today only `common.zip` and loose `vehicles/common` files are read.
2. **Map SVJ hardpoints to jbeam nodes.**
   - Vanilla suspensions have no meaningful node names (`fhub1l`, `fwhl1l`…), so each SVJ hardpoint needs to be tied to a node.
   - Automatic guess from the structure: the wheel centre from the `pressureWheels` node pair, arms as the beams between hub and body nodes, springs and dampers by beam type.
   - Manual fix: click a node, assign a hardpoint. Save the mapping as a small file per suspension part, so every vehicle sharing that part benefits.
3. **Choose each parameter.**
   - The comparison panel becomes a table with a "take" box per row (geometry per corner, wheelbase and track, springs and dampers, alignment, mass and CG, tyres, powertrain, aero), with "take all" per group.
   - A taken value is applied in the least invasive way that works: an existing tuning variable, then a slot `nodeOffset` / `nodeMove`, and only then a generated copy of the part with moved nodes, written to a local mod folder. Generated parts are made on the user's machine from their own install and are never committed here.
4. **glTF meshes on the part tree (editor).**
   - Match each glTF node (`SVJ::<category>::<id>`) to a part or slot of the tree: body to the main part, corners to the suspension parts, panels to their parts, with a manual override.
   - Attach each mesh to its part, so it follows part changes and wheel travel.
5. **Meshes into the game as flexbodies.**
   - Generate `flexbodies` entries that bind each mesh to its part's node groups.
   - **Check in-game first:** BeamNG vehicles use `.dae`, so glTF may need converting to DAE. Test this before building on it.

## Not planned

Rule checking. BeamForge is a general BeamNG and SVJ tool. Spec-series rules live in their own projects (for example FBeam).
