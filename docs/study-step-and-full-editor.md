# Study: STEP skeletons as references, and BeamForge as a full editor

Status: a study for discussion (2026-10-08, revised the same day: the STEP files are wireframes). Nothing here is built.

## 1. Where the editor stands

`editor/app.js` (1 544 lines, three.js + Pyodide running `beamforge/`) is a **viewer with a fitting pipeline**:

| Has | Lacks for a full editor |
| --- | --- |
| Reads the install and mods the game's way, parts tree, slots, tuning, `.pc` save | A document: edits live in one `vehEdit.moves` object (node and part offsets); no undo/redo, no save/load of the work |
| Click a node, beam or part, type x / y / z | Add or delete nodes and beams, mirror, multi-select, gizmo, snapping |
| Fit to an SVJ (wheelbase, body, pickups), values, archetype, conversion, steering and bump-steer tuning, all as Python run in the page | Those are one-shot batch steps; the user cannot see or edit the result of each, only the final export |
| Export a new vehicle as a mod zip (parts rewritten as JSON, meshes as COLLADA) | Writing into the game's folders (the browser cannot reach AppData), install, cache clear, version names, test notes: done by hand today |
| Checks in Python: structure (at least 3 beams), rigidity limits, static solves, bump steer, steering direction, wheel hold | They only run in test scripts; the editor shows none of them live |

So the distance to a full editor is mostly **three things**: an edit model with history, topology editing, and bringing the analyses into the editor. The Python side is already the strong part.

## 2. STEP files as a skeleton: points and lines a jbeam structure is built from

The STEP file is a **wireframe**, not solids. No tessellation and no geometry kernel are needed.

### What the file holds
A wireframe STEP is a list of entities in plain text, which a small reader of our own can parse (Python in Pyodide, a few hundred lines, no library):

| STEP entity | Becomes |
| --- | --- |
| `CARTESIAN_POINT`, `VERTEX_POINT` | a candidate **node** |
| `LINE` (point + vector) with `TRIMMED_CURVE`, `POLYLINE`, `EDGE_CURVE` between two vertices | a **beam** (a polyline is a chain of beams) |
| `CIRCLE`, `ELLIPSE`, B-spline curves (arcs, tube bends) | a chain of straight beams (chord tolerance set by the user) |
| `COMPOSITE_CURVE`, `GEOMETRIC_CURVE_SET`, `GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION` | **groups**: which lines belong together |
| `PRESENTATION_LAYER_ASSIGNMENT`, `STYLED_ITEM` colours, `PRODUCT` and assembly usage names | **names and layers**: what the CAD author called each set (frame, front lower arm, rack...) |
| `AXIS2_PLACEMENT_3D` of assemblies | the placement of sub-assemblies (a corner modelled once, placed four times) |

The unit comes from the file (`SI_UNIT` / `CONVERSION_BASED_UNIT`, usually mm). The importer reports the bounding box and offers an axis flip, since CAD systems disagree.

### The rule (agreed 2026-10-08)
The STEP is an **assembly**. **Each part is a rigid body**, given by its lines. The **end points of the lines are the nodes**, and the part gets the beams that make those nodes move as one rigid body. One line: one beam between its two ends. Two lines with three ends: three beams (the triangle). More ends: below.

Decided as well (2026-10-08): **helper nodes are allowed** for flat and straight parts (off the plane or the line, light,
hidden), and **the tool splits a line** where another part's end lands on it (a node there, the line's part braced with it).
No sample assembly yet: the reader is built against made-up files first.

**What the STEP is for** (the user, 2026-10-08): a reference of its own, not tied to any SVJ, and the way to add
**non-standard mechanisms** to BeamForge, the ones neither the SVJ conversion nor the archetypes describe (rockers and
pushrods, Watt's links, unusual steering, anything drawn as rigid parts and joints). SVJ part names are recognised when
a part uses them; any other name is plain structure. The SVJ *frame* is only the common vehicle frame it is placed in.

**Built (first step, standalone)**: `beamforge/skeleton.py` (reader, placement with 90 degree turns and an offset, the
build: merges, splits, welds, helper nodes, bracing to 3n - 6, joints, free motions, jbeam), a Skeleton panel and
3D layer in the editor (Import STEP), `tests/test_skeleton.py`. Not attached to a base vehicle yet.

### Stable stiffness and damping, for the skeleton and for beams added by hand
Measured on 5 240 nodes of ten vanilla cars at the 2000 Hz step (`rigidity.VANILLA`):

| Index | median | 90% | 99% | max |
| --- | --- | --- | --- | --- |
| node: sum k / m * dt^2 | 2.08 | 3.68 | 4.90 | 7.49 |
| node: sum c / m * dt | 0.39 | 0.88 | 1.62 | 3.43 |
| beam: k * dt^2 * (1/ma + 1/mb) | 0.30 | 0.60 | 0.92 | 1.81 |
| beam: c * dt * (1/ma + 1/mb) | 0.05 | 0.12 | 0.35 | 1.59 |

The rule (`rigidity.beam_values`): a new beam's stiffness puts its own two-mass mode at the vanilla median for the
masses of its two nodes; it is lowered if either node would pass the vanilla 90th percentile; damping follows at the
vanilla ratio (about k / 12 000). Every node gets a band (`rigidity.node_index`): ok up to the median, high to the
90th, risky to the 99th, unstable beyond. The skeleton's beams use it now, and the editor colours nodes by band.

For **hand-added beams** in the editor (not built yet): pick two nodes, the beam comes with these values; a stiffness
and damping field (or slider) goes up to the room left at both nodes (`k_room`, `c_room`), never past it; the two
nodes show their band live; a node out of room can instead be given more weight (the stable way vanilla cars stiffen
a spot), with the kg it takes shown. The same check runs on every save, so no edit can leave a node in the red.

### Research: how many beams a rigid part needs, and which
This is bar-and-joint rigidity in 3D (rigidity theory: Maxwell's counting rule, the rigidity matrix of Asimow and Roth, Henneberg constructions; standard results, quoted from memory, not re-checked against the papers). Checked numerically with a prototype (`rigidify.py` in the session scratchpad: rigidity matrix, rank, the smallest non-trivial eigenvalue as a measure of stiffness, greedy bracing).

- **The count.** n nodes have 3n coordinates. A rigid body keeps 6 free motions (3 moves, 3 turns). So a rigid part needs beams whose rigidity matrix has rank **3n − 6**: at least **3n − 6 beams** (Maxwell), exactly that many if none is redundant. n = 2: 1 beam (the bar can only spin about its own axis, which point nodes cannot show: one beam is right). n = 3: 3. n = 4: 6 (a tetrahedron). n = 5: 9. n = 8: 18.
- **The count is necessary, not sufficient.** In 2D, Laman's theorem says which graphs are rigid; in 3D no such combinatorial rule is known. The test is numerical: the **rank of the rigidity matrix** at the actual positions. It is cheap for a part (tens of nodes), and `structure.py` already does the same kind of rank test on directions.
- **Which beams.** Keep the CAD's lines (they are the real members). Add beams one at a time, each time the pair of nodes that raises the rank and most raises the stiffness of the weakest mode, until the rank is 3n − 6. Results of the prototype:

| Part | Ends | Lines | Beams added | Total (3n − 6) |
| --- | --- | --- | --- | --- |
| One line | 2 | 1 | 0 | 1 |
| Two lines, three ends (V, an A-arm) | 3 | 2 | 1 | 3 |
| Chain of 3 lines, 4 ends, not flat | 4 | 3 | 3 | 6 |
| Box frame | 8 | 12 | 6 (one diagonal per face) | 18 |
| 7 points in space, a chain | 7 | 6 | 9 | 15 |

  For the box, all 64 ways of putting one diagonal on each face are rigid, but their stiffness differs by about 2× (0.31 to 0.59): the choice matters for stiffness, not for rigidity. The greedy picks the stiff ones.
- **Flat parts are the hard case.** If all of a part's ends lie in one plane and there are 4 or more, **no set of beams between them is rigid**: the part can fold out of the plane (a flat quad with all six beams still has rank 5 of 6). The beams are perpendicular to that motion, so they resist it only to second order: in the game, a floppy part. It needs one node **off the plane** (an apex), joined to the others: n + 1 nodes, 3(n + 1) − 6 beams. Flat 4-end quad: +1 node, 9 beams. Flat hexagon: +1 node, 15 beams.
- **Nearly flat is as bad as flat.** A quad warped 5 mm out of 1 m is rigid on paper, but its weakest mode is about 10 000 times softer than a well-shaped part. So "flat" must be judged with a tolerance (the part's thickness over its size), not exactly, and the apex added when the part is thinner than that.
- **Straight parts.** Three or more ends on one line (a tube with a joint in its middle) can bend at the middle node whatever the beams: it needs **two** helper nodes off the line. A line crossed by another part's end (a T) is the same case; better: the CAD splits the line at the junction, or the junction is a joint (below).
- **Helper nodes** (apexes) are not in the CAD. They are light, hidden, placed at the part's centre plus half its size along the plane's normal, and they follow the `rigidity` rules for weight against stiffness like any node. The alternative is to accept the flex when the assembly holds it (below).
- **Joints come from shared ends.** Two parts whose ends coincide (within a tolerance) share a node there: one node joining both parts (agreed). In the jbeam a node is defined in one part and the other part's beams refer to it by name, as vanilla cars do (an arm's beams use the subframe's pivot nodes). The owner is the part nearer the body in the assembly: the frame owns its pivots, the arm owns the ball joint the hub hangs on. The rest of the rule, and the number of shared nodes says what the joint is: **1: a ball joint**, **2: a hinge** about the line through them, **3 or more not on one line: welded** (one rigid body). That is how the vanilla cars do it: a lower arm pivots on two body nodes (a hinge) and carries the hub on one (a ball joint). Welded parts are braced **together**, as one body, which needs fewer added beams than bracing each alone.
- **Check the assembly, not only the parts.** After bracing, the whole structure has as many free motions as the mechanism should have: 6 for a solid car body, plus one per wheel's travel and one for the steering. The same rank test on the whole assembly counts them, and any extra free motion is a part or joint that is not as intended. The editor reports them, and highlights the nodes that move in each.
- **Beyond rigid: strength.** A minimal (isostatic) part has no spare beam: one broken beam frees it, and loads take one path. BeamNG's vanilla structures are over-braced. A redundancy option (keep adding the next-best beams up to, say, 1.5 × 3n − 6, or until every node has the `structure.py` minimum of three independent beams) makes parts that bend and break more like the vanilla ones.

### From skeleton to jbeam
The file gives **topology and positions only**. What a jbeam needs beyond that comes from rules the user controls:

1. **Nodes.** Line ends joined within a tolerance are merged (CAD often leaves duplicates); within a part they become one node, across parts a joint (above). Isolated points stay *reference points* (hardpoints, wheel centres), not nodes.
2. **Beams.** The part's lines, plus the bracing above. A layer or the part's name decides its **kind**: a frame member, a suspension arm (the existing `roles` / archetype vocabulary), a link, a spring-damper (a line named by convention), a rack, a limiter.
3. **Properties from the kind.** Stiffness, damping, strength, node weight and bounds come from the project's rules, with the existing `rigidity` module keeping node weight, beam stiffness and the 2000 Hz step consistent (the reason cars exploded or rang). A tube size per layer (outer diameter, wall) gives beam spring and mass from length, so a long member is softer and heavier, as in the vanilla cars.
4. **Structure checks on the result**, the rule already in `structure.py`: a node held by fewer than three independent beams is a **mechanism** node, not structure. The editor flags an under-braced skeleton and **proposes bracing diagonals** that close each open quad.
5. **Mechanisms are named, not guessed**: a hinge axis, a rack rail with sliders, a steering hydro, a spring-damper. Names follow a small convention in layer or part names (`susp_FL_lower_arm`, `hp_FL_ball_joint_lower`, `rack`, `spring_FL`), shown in a mapping panel so the CAD author's own names can be mapped onto them.
6. **Parts, slots and groups.** Each layer becomes a jbeam part with its own slot (frame, front subframe, front and rear suspension, steering), node groups for the flexbody meshes to follow, and the placement in the vehicle frame: the output the exporter already writes.
7. **Wheels.** Wheel centres and axle directions come from pairs of reference points, feeding `pressureWheels`, with the camber and toe preload method already in place (wheels spawn square).

This is the **archetype** idea made user-driven. The archetype builds its hub, arms and strut from SVJ hardpoints by code; the skeleton route builds *any* structure from the points and lines the user draws. They share the back end (rigidity, structure checks, mounts, export), and the archetype can emit the same skeleton format, so a generated structure can be opened, edited by hand and re-exported.

### Editor consequences
- A **Skeleton** layer type in a Reference panel: visible lines and points, layers on and off, a transform (unit, axes, offset), and a *Build structure* action that turns the visible layers into a draft part with a **report** (nodes merged, intersections added, beams by kind, nodes held by fewer than three beams).
- Hand editing and the skeleton **meet**: after *Build*, the result is ordinary nodes and beams in the document, and the skeleton stays as the reference. When the CAD changes, re-running keeps the hand edits that did not touch the changed members (through the operation log).
- The SVJ supplies the same kind of data: its hardpoints are points and its links are lines, so an SVJ corner can become a skeleton and be edited. Conversely a skeleton with the SVJ naming gives a suspension description back, which `suspension.study_svj` can check.
- Optional later: solids or meshes (STEP B-rep, through `occt-import-js`, a 7.6 MB WASM, LGPL-2.1, loaded from a CDN) as a visual reference under the skeleton. Not needed for the skeleton route.

### Risks
- The rules (tube sizes, stiffness by kind) decide whether the car is stable and sensible. Defaults should be **measured from the vanilla cars** (the project already reads every beam of every vehicle), not invented.
- Lines that cross without sharing a node, near-duplicates and tiny segments: tolerance settings, with a preview of what merges.
- A skeleton does not say where springs, dampers and the rack are: the naming convention, or a click-to-assign in the editor, must.
- STEP wireframes come in several flavours (AP203 curve sets, AP214, bounded wireframe, edge-based B-rep). Start with the common ones and report unknown entities instead of failing silently.
- The files stay local (never uploaded, never committed), the same rule as game files.

### Effort
Reader (points, lines, polylines, layers, units): small. Build with merge, intersections, kinds and report: medium. Rules measured from vanilla cars: medium. Under-braced detection and bracing proposals: small (the check exists). Suspension and rack mechanisms from names: medium (reuses archetype code).

## 3. Becoming a full editor

### Principle
Keep what works: **a BeamNG vehicle as the base, edits on top, a new vehicle mod as the output, no game files in the repo.** The change is to make "the edits" a first-class, inspectable thing. A vehicle can also start from nothing but a skeleton.

### Architecture
1. **Document = base + operation log.** The project holds the base (model, config), the SVJ and reference layers, and an ordered list of operations: *move node*, *add/delete node/beam*, *set beam/node property*, *tie hardpoint*, *build from skeleton*, and macros such as *apply fit*, *convert suspension*, *apply archetype*, *tune tie rods*, *take values*. Each operation is data with an inverse or a replay. That gives undo/redo, saved projects and reproducible exports (build = base + operations), and lets the batch steps show what they did and be switched off one by one. It generalises today's `vehEdit.moves`.
2. **Editing tools** (JS, three.js): selection (click, box, by part or node group), translate/rotate gizmo, **mirror** (edit the left, the right follows), snap (nodes, grid, SVJ hardpoints, skeleton points), add node, add beam between two nodes, delete, copy/paste of a set, property editing for nodes (weight, group, collision) and beams (type, spring, damp, bounds, precompression), and for the other sections (hydros, torsionbars, slidenodes and rails, flexbodies, props).
3. **Parts and templates.** Author a new part with its own slot. **Archetypes** become *parametrised part generators* in a library (strut, double wishbone, multilink, semi-trailing, solid axle): instantiate from hardpoints (SVJ or skeleton), then edit by hand.
4. **Live checks.** The Python analyses run in the page (Pyodide has numpy for the nonlinear static solve written this week): nodes held by fewer than three beams, node weight and stiffness limits, ride height, bump steer, steering direction and lock, wheel hold, and an animated wheel-travel and steering sweep. A **Checks** panel highlights the failing nodes. This is what saved the most time in the test loop.
5. **JBeam fidelity.** Today parts are rewritten as JSON, which loses comments, layout and ordering. Options: (a) keep as is (works, unreadable diffs); (b) a **lossless jbeam writer** that edits the original text in place (more work, keeps files readable and diffable); (c) emit only **overrides** (new parts that slot over the base's). Recommend (a) now and (b) when users start reading the output.
6. **Test loop built in.** A small local helper (`serve.py` already exists) with an *Install* endpoint: writes the zip to the mods folder, refuses if BeamNG is running, clears the cached vehicle, keeps only the current builds; a version counter in the in-game name; **test notes** per build ("v20: rear high, toe-in") saved in the project. The browser cannot write under AppData, so this needs the helper. It removes the manual steps of every test round.
7. **Packaging.** A static page plus Pyodide (no install) with the optional helper for the installer; Electron or Tauri later if the helper becomes mandatory.

### Phasing (each phase usable on its own)
| Phase | Content | Why this order |
| --- | --- | --- |
| A | Operation log, undo/redo, save/load project; existing moves and batch steps expressed as operations | Everything else is built on it |
| B | Skeleton reader (STEP points and lines), layers, transform, *Build structure* with its report | The immediate request; needs A only for saving |
| C | Topology editing: select, gizmo, mirror, add/delete node/beam, property panels | The core of "editor" |
| D | Checks panel and sweeps (structure, rigidity, ride height, bump steer, lock) | Turns the test scripts into the tool |
| E | Part authoring and the archetype library (including the double-wishbone archetype still missing) | Makes SVJ-less builds possible |
| F | Local installer and test notes | Removes the manual test loop |

### Costs and open points
- `app.js` is one 1 500-line file. Phase A should split it into modules (document, tools, panels, three.js scene) first; the `FILES` list and its test already enforce module registration.
- Pyodide start-up and the pure-Python solvers are fine now; large vehicles or sweeps may need numpy (available in Pyodide) or a worker.
- JBeam is large and has edge cases (variables, `slots2`, expressions). The `jbeam.py` reader and the 38-vehicle export test are the safety net; keep running them every phase.

## 4. Questions for the discussion
1. **Where do your skeletons come from?** Which CAD program, and which export (3D sketch, wireframe, bounded curves)? One sample file would tell me which STEP vocabulary to read first.
2. **How are lines identified?** Layers, colours or assembly names per kind (frame, arm, spring...), or one flat set of lines the editor lets you tag after loading?
3. **Whole vehicle or parts?** A full structure (frame, suspension, steering) in one file, or one file per assembly (a front suspension, a subframe) that are placed and joined in the editor?
4. **Where do stiffness and weight come from?** Rules measured from the vanilla cars (a tube size per kind), or entered by hand per layer?
5. **How far does hand editing go** afterwards (add/delete nodes and beams, hydros, rails, flexbodies, new parts), and is a local helper (installer, cache clear) acceptable?
6. **Order**: the operation log with undo and the project file first, then the skeleton reader and *Build*, then the topology tools and the checks panel. Does that match what you want to see first?
