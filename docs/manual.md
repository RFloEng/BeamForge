# BeamForge manual

BeamForge makes BeamNG.drive vehicles. It works in two ways:

- **From a base vehicle.** Start from any vanilla or mod car. Change it by hand or lay an SVJ (Standard Vehicle JSON)
  over it. Then export a new vehicle.
- **From scratch.** Draw the structure and suspension as points and lines (or import them from CAD as STEP). Add an
  engine, a gearbox, rims and tyres from the vanilla cars. Export a complete new vehicle.

Everything runs in your browser. It reads your own BeamNG install and uploads nothing. The game itself is never
changed: every export is a new vehicle, in its own folder, inside a mod zip.

> **Status: v0.3 prototype.** The base vehicle, the SVJ overlay, taking values and the export work and are being tested
> in the game. The workspaces, cars from scratch, hand-added beams, mounted sketches and electric cars are new and
> **not yet verified in the game**. Test them and report what they do.

Contents

1. [Getting started](#1-getting-started)
2. [The screen](#2-the-screen)
3. [Projects](#3-projects)
4. [Workspaces](#4-workspaces)
5. [Recipe: a vanilla car turned into an SVJ car](#5-recipe-a-vanilla-car-turned-into-an-svj-car)
6. [Recipe: a car from scratch](#6-recipe-a-car-from-scratch)
7. [Recipe: a new mechanism on a vanilla car](#7-recipe-a-new-mechanism-on-a-vanilla-car)
8. [Testing in the game](#8-testing-in-the-game)
9. [Stability: what the bands mean](#9-stability-what-the-bands-mean)
10. [Coordinates](#10-coordinates)
11. [Troubleshooting](#11-troubleshooting)

---

## 1. Getting started

**You need:**
- Python 3.9 or newer
- Chrome or Edge, which can remember folders (other browsers ask for them every session)
- BeamNG.drive installed

**Start the editor** from the BeamForge folder:

```bash
python serve.py
```

Then open <http://localhost:8000/editor/>. The first load takes a few seconds while the Python engine starts
("Loading Python engine…"). If port 8000 is taken, run `python serve.py 8001` and use that port.

**Add your BeamNG folders once** (left panel, *BeamNG folders*):

- **Game folder…**: the install, for example `…/steamapps/common/BeamNG.drive`. The folder itself, `content` or
  `content/vehicles` all work.
- **Mods folder…**: your user folder, for example `AppData/Local/BeamNG/BeamNG.drive`. Mods switched off in the
  game are skipped.
- **With file dialog**: use this when Chrome refuses a folder, which it does under AppData and Program Files. You
  will have to pick it again each session.

The game's own rule decides which file wins: the user folder over mods, and mods over the install. The vehicle list
fills in, cars first. Use the filter box to find one.

---

## 2. The screen

| Area | What it holds |
| --- | --- |
| **Top bar** | The workspace tabs; **Open…** and **Save** (projects); **Import SVJ** and **Files…**; **Import STEP** |
| **Left** | Your BeamNG folders and the vehicle list |
| **Centre** | The 3D view. Drag to orbit, right-drag to pan, wheel to zoom. Click a node or a beam to pick it |
| **View switches** | Nodes + beams, Meshes, Part colours, and, when loaded, SVJ meshes, SVJ hardpoints, SVJ suspension, Skeleton. **Compare** overlaps the base's meshes with the SVJ's |
| **Right** | The panel of the current workspace |
| **Bottom** | Key numbers (parts, nodes and beams, wheelbase), then messages (PASS / INFO / WARN / ERROR), then timings |

---

## 3. Projects

A project holds **your work, not game files**:

- the base vehicle, by name
- its configuration, slot choices, tuning, moves, fit and hand-added beams
- the SVJ document
- the sketch
- the values taken, the powertrain, wheels, aero and components
- the export settings and the workspace you were in

| Action | How |
| --- | --- |
| Save | **Save**: downloads a `.beamforge.json`. Send it to someone or keep it to continue later |
| Open | **Open…**: pick a `.beamforge.json`. If the base vehicle isn't found yet, add your BeamNG folders and it opens with all its edits |
| Autosave | After every change the work is kept in the browser and comes back when you reopen the editor. A very large project may not fit; then use **Save** |

The SVJ's meshes are not stored in the project, because they can be large. After opening a project, point the SVJ
workspace at the meshes folder again.

---

## 4. Workspaces

The workspaces share one 3D view and one project. Each tab changes only the right-hand panel.

### Base vehicle

This is the vehicle you start from.

- **Pick a vehicle** in the left list. **Configuration** chooses one of its `.pc` configs.
- **Parts**: the game's Parts menu, with every slot and its alternatives.
  - **move** picks a part to move.
  - **lock** stops a part from being picked in the view.
  - **hide** hides it.
- **Tuning**: the vehicle's tuning sliders.
- **Save configuration (.pc)…** writes the current choices as a config for the game. Put it in
  `vehicles/<model>/` in your user folder.
- **Move**: click a node or a beam in the view, or **move** beside a part, then type x / y / z in metres.
  - A node moves itself.
  - A beam moves both its nodes; you type its midpoint.
  - A part moves as an offset, like a slot `nodeMove` in the game.
  - Moves are written into the vehicle when you export it.
- **Beams** adds beams by hand:
  1. Click a node, then **New beam from …**.
  2. Click the second node. The panel shows the suggested spring (k) and damping (c), how much each node can still
     take, and each node's stability band before → after (see [§9](#9-stability-what-the-bands-mean)).
  3. Type your own values if you like. They are held at what the nodes can take, and a message tells you when that
     happens. **suggested values** goes back to the suggestion.
  4. **Add beam**. Added beams are green in the view. **remove** deletes one.
  - Deform and strength follow the beams already on those nodes.
  - You get a warning if the beam would tie the suspension to the body (the wheel could no longer move) or if the car
    already has a beam between those nodes.
  - The beams are written at the end of the main part when you export.

### SVJ

This lays a Standard Vehicle JSON over the base.

- **Import SVJ** asks for the **folder** that holds the SVJ: its `.svj.json` (or a `.zip` bundle) and its glTF meshes,
  found in that folder or below it. **Files…** takes loose files instead.
- The SVJ's meshes and hardpoints are placed on the base's front axle and ground. Wheelbase, tracks and mass are
  compared.
- **Fit to the SVJ** bends the base onto the SVJ in three stages. Each stage can be ticked or left out.
  1. **Wheelbase**: stretches between the axles; the front axle stays put.
  2. **Body to the mesh**: overhangs, floor, width and roof line, fitted to the SVJ body mesh.
  3. **Pickup points**: each SVJ hardpoint is tied to a node by its role and moved exactly onto it.
- The **Hardpoints and their nodes** table shows each tie and its gap. To re-tie a hardpoint, click it, then click the
  right node in the view. Your ties are remembered per suspension part for every vehicle that uses it.
- **Remove fit** undoes the fit. Your hand moves always stay on top of it.
- **Values from the SVJ**: springs, dampers, tyre radius, mass and CG, torque curve, gears and final drive, steering,
  drag. Each row has a *take* box, and the taken values go into the new vehicle.

### Sketch

Here you draw mechanisms and structures as points and lines, or import them from CAD.

**The rules**
- Each **part** is one rigid body. Its line ends become nodes.
- Parts with an end at the **same place share that node**. That is a joint:
  - one shared node is a ball joint
  - two shared nodes make a hinge
  - three or more make a weld
- A line that another part's end lands on is split there.
- Points are reference points: wheel centres, or places to attach things.
- **Kind**:
  - **frame** parts are welded tubes. Their stiffness, strength and mass come from the tube (FBeam's rules), with
    bending beams at welded corners.
  - **link** parts are braced stiff, like suspension arms.
  - **damper** parts are a spring and damper unit (purple). They are not structure; their ends are joints like any
    other part's. Parts named `damper_fl`, `strut_fl`, `coilover_fl` or `shock_fl` are dampers by default.
- **Tube**: a section such as `tube 40x2 steel_4130n` or `sqtube 30x2 al_6061_t6`, in mm. A size written in the part
  name is used too (`frame_tube_45x2_5`).
- The tool adds **helper nodes** where a part is flat or straight, so that every part is truly rigid.

**Drawing**

| Tool | Key | Use |
| --- | --- | --- |
| Select | V | Click a node or a line to see it, move it or delete it |
| Line | L | Click two points; drawing goes on from the last end. Esc stops |
| Point | P | Add a reference point to the active part |
| Undo / redo | Ctrl+Z / Ctrl+Y | |
| Delete | Delete | Removes the selected node or line |

- **Snap to** sketch nodes, SVJ hardpoints or vehicle nodes.
- **Type a line** adds a line by numbers, in mm, X forward, Y right, Z down.
- **Parts** table:
  - the radio button makes a part active (new lines go into it)
  - you can rename a part, set its kind and tube
  - **mirror** makes the other side's copy
  - **×** deletes it
- Name parts with the SVJ names so they are understood: `lower_wishbone_fl`, `upright_rr`, `tie_rod_fl`, `frame`…
  The name list offers them.
- **Turn, move, scale**:
  - 90° turns about X, Y or Z
  - a move
  - a scale; **unit fix** repairs a STEP read in the wrong unit (×0.001, ×25.4…)
- **Settings**: merge tolerance (how close two ends must be to be one node) and the minimum node mass.

**Springs and dampers**

A car from scratch needs one damper part per corner: its top on the frame (or free, and then it is tied to the
nearest frame nodes), and its lower end on an arm or on the upright. A lower end on an arm's line splits the line
there, so it becomes a joint.

- **Add dampers** draws one for each corner that has none:
  - **from the SVJ's strut** when the SVJ has a MacPherson (its real top and lower end)
  - **from the SVJ's damper mounts** when it has real ones
  - otherwise **on the lower arm**, 70 % of the way out, leaning slightly inboard. Move it where the real one is.
- **Where it goes doesn't change what the wheel feels.** The car is built with the wheel's stiffness and damping. Each
  unit's motion ratio (how far it shortens per metre of wheel travel) is measured on the sketch, then:
  - spring = wheel rate ÷ motion ratio²
  - damping = wheel damping ÷ motion ratio²
  - preload = the corner's sprung weight ÷ motion ratio, on that spring
- **Assetto Corsa SVJs** give the spring and damper at the wheel, on a vertical axis through the wheel centre. That is
  not a real position. BeamForge recognises it, keeps those values at the wheel, and places the unit as above.
- A damper whose lower end is on the **upright** of a corner with **no upper arm** is a MacPherson strut. It also gets
  the strut's guide: a node on the upright that slides on a rail up the strut, as BeamNG's vanilla struts do.

**Import / export**
- **Import STEP**: a STEP assembly of wireframe parts (points and lines). Each product becomes a part.
- **save STEP** writes the sketch back as STEP for your CAD program.
- **jbeam** downloads the structure as jbeam parts.

**Structure report**: nodes, beams, rigid bodies, joints, free motions, mass, and stability bands.
- *Free motions*: 6 belong to the whole body moving in space. Any beyond those are the mechanism's own degrees of
  freedom.
- *Not rigid* lists parts that still flex.
- The line colours mean: orange tubes, green bending beams, blue braces, grey lines to helper nodes.

### Suspension

This solves the SVJ's corners over ±100 mm of wheel travel: camber, toe, track change, roll centre, motion ratio and
wheel rate, kingpin, caster, scrub and trail. The linkage moves in 3D. After a fit, the base's own points are solved
too and shown alongside.

A link named `strut` is solved as a MacPherson strut, even when the file calls it a rod. When the SVJ gives its spring
at the wheel (Assetto Corsa), the wheel rate shown is the file's. The notes give the coil rate that rate needs on the
corner's own damper.

### Wheels

You choose rims and tyres from the vanilla cars. First, **learn from the install** once, in Powertrain.

- Choose the same for both axles, or front and rear separately.
- **Rim**, then **Tyre** (only tyres that fit that rim are offered).
- **Pick the nearest to the SVJ tyre** picks by the SVJ tyre's rim size and width.
- The tyre's values (friction, load sensitivity, pressure and so on) can be overridden. Empty keeps the archetype's
  value. A tyre with its own values is written as a part of the car.
- **Benchmark**: the SVJ's Pacejka (Magic Formula) tyre, shown as lateral and longitudinal force curves. It is also
  compared with the BeamNG tyre for grip against load.
  - **Fit the load sensitivity to the SVJ** sets BeamNG's `noLoadCoef`, `fullLoadCoef` and `loadSensitivitySlope` so
    the tyre loses grip with load the way the SVJ tyre does.
  - BeamNG's friction values are multipliers, not a friction coefficient, so it is the *shape* that is compared.

### Components

These are the masses a car from scratch carries.

- **+ driver, + fuel, + battery, + ballast** add presets. **from the SVJ** adds the SVJ's mass bodies.
- Edit each one's mass and position, in mm in the SVJ frame (see [§10](#10-coordinates)).
- The panel shows the car's mass, its CG (behind the front axle, and its height) and the share on the front axle,
  against the SVJ's.
- **ballast to the SVJ** adds the one ballast mass, and its place, that brings the car to the SVJ's mass and CG.
- In the view, components are orange and the CG is red.
- A base vehicle keeps its own masses; its mass and CG are taken in SVJ instead.

### Aero

- The drag area (Cd × frontal area) and lift per axle from the base, the SVJ, or your own numbers.
- The forces at 80 to 200 km/h, as a table and charts.
- **The drag is written**:
  - a base car gets its aero triangles scaled to the drag area
  - a car from scratch gets a drag plate
- **The downforce is only shown.** BeamNG's aero law for its triangles isn't published, so a wing can't yet be sized to
  a downforce.

### Powertrain

- **Learn from the install** reads every vanilla car's engines, gearboxes, tyres and rims and groups them into
  archetypes, such as "I4 petrol, medium", "6-speed manual" or "15 in sport". Electric motors are included. Do it once,
  and again after a game update.
- **Engine**:
  - the base's, the SVJ's, an archetype, or **edit** (torque curve as rpm and Nm per line, idle and max rpm)
  - inertia, friction, engine braking, and the mass (for a car from scratch)
  - torque and power charts compare the base, the SVJ and the new engine
- **Gearbox and final drive**: an archetype's ratios, the SVJ's, or your own.
- **Differentials** per axle: open, limited slip, viscous or locked, with their values, plus the AWD front/rear split.
- **Steering**: turns lock to lock.
- **Suggest archetypes from the SVJ** picks the engine nearest the SVJ's (fuel, cylinders, power) and a gearbox of
  its type.
- **Transmission and engine parts**: the base's own alternatives, as in its Parts menu.

### Checks

- **Check the vehicle**: each node's stiffness and damping against the nodes of ten vanilla cars (bands, see
  [§9](#9-stability-what-the-bands-mean)), the 15 worst nodes, and nodes held in fewer than three directions.
- The sketch is checked the same way, with its free motions.

### Assembly

This is where the new vehicle is made.

- **Summary**: base, moves, SVJ, fit, values taken, sketch.
- **Sketch on the base** puts the sketch on the base vehicle (see [§7](#7-recipe-a-new-mechanism-on-a-vanilla-car)).
- **Make a car from scratch** (see [§6](#6-recipe-a-car-from-scratch)).
- **Make a new vehicle (mod)**, from the base:
  - **Id** (the folder name), **Name**, **Brand**
  - per part, choose how it is written:
    - *with the edits*
    - *as it is*
    - *reuse* (shared parts left in the game)
  - **Regenerate all** copies every part into the new vehicle, so the mod stands alone
  - with an SVJ loaded, its meshes can replace the base's body
  - **Export mod (.zip)…** downloads the mod

---

## 5. Recipe: a vanilla car turned into an SVJ car

1. **Base vehicle**: pick the car closest to the target (size, layout, suspension type) and its configuration.
2. **Import SVJ**: pick the SVJ's folder. Its meshes and hardpoints appear on the car.
3. **SVJ → Fit to the SVJ**: leave all three stages ticked and click **Fit**.
   - Check the hardpoint table. Re-tie any hardpoint with a large gap.
   - Look at the uprights table for camber and toe.
4. **SVJ → Values**: tick what to take (springs, dampers, tyres, mass, powertrain, aero).
5. **Powertrain / Wheels / Aero** if you want to go beyond the SVJ's values.
6. **Checks → Check the vehicle**: look for nodes beyond the vanilla range.
7. **Assembly → Make a new vehicle**: set id and name, decide whether the SVJ meshes replace the body, and export.
8. **Save** the project.

The base is a starting point. Expect differences from the SVJ where the base's design differs, such as another
suspension type.

---

## 6. Recipe: a car from scratch

1. **Powertrain → Learn from the install** (once).
2. **Sketch → New sketch**, or **Import STEP**. You need:
   - a **frame** part with at least four nodes
   - per corner, the suspension links (`lower_wishbone_fl`, `upper_wishbone_fl`…) and an **upright** part
     (`upright_fl`…) with **a point at the wheel centre**
   - at the front, **tie rods** (`tie_rod_fl`…) ending on the upright; the steering rack is built between their inner
     ends
   - the uprights sharing their ball joints with the arms
   - **a damper per corner**: **Add dampers**, then move each one where the real one is
   - **mirror** draws the other side
3. **Powertrain**:
   - an **engine** (an archetype, or the SVJ's curve on an archetype's block) and a **gearbox**
   - or an **electric motor**, which needs no gearbox: the motor drives the differentials, and the final drive is its
     reduction (8 when not set)
   - set the final drive, differentials and steering turns
4. **Wheels**: a rim and tyre per axle.
5. **Components**: driver, fuel or battery, ballast. **Aero**: the drag.
6. Optional: import an SVJ with its meshes, and its chassis mesh becomes the car's body.
7. **Assembly → Make a car from scratch**: the checklist must be all ✓ (frame, wheels, engine, gearbox or motor,
   library, rims, springs and dampers). The **At the wheel** table holds the wheel rate (N/mm) and the bump and
   rebound damping (N·s/m) per axle. Empty fields take the SVJ's values, or else a ride frequency on the corner's
   weight. Then set:
   - the layout (FWD, RWD, AWD)
   - the full-lock angle
   - the battery kWh for an electric car
   - id, name, brand

   Click **Make the car (mod zip)…**. The message shows nodes, beams, mass and stability, plus notes on what was
   built.

The car gets:
- the sketch's structure
- the rims and tyres as slots, with their meshes
- the engine and gearbox blocks mounted to the frame
- a spring, a damper and a travel limit at each damper part, with the wheel's values
- a driveline and a steering rack
- the components' masses
- a drag plate
- the body mesh, if given

---

## 7. Recipe: a new mechanism on a vanilla car

Use this to add a brace, a new linkage, or any non-standard mechanism to an existing car.

1. **Base vehicle**: open the car.
2. **Sketch**: draw the mechanism. Turn on **Snap to vehicle nodes**. Wherever the mechanism attaches, start or end a
   line **on a vehicle node**: that sketch node *becomes* the vehicle's node.
3. **Assembly → Sketch on the base**: tick **Mount the sketch on …**. The panel shows:
   - **Shared nodes**: the sketch nodes that are vehicle nodes. The sketch's weight there is added to the node, and
     its beams are softened if needed to keep the node in the vanilla range.
   - **New nodes · beams**: what will be added. Beams the car already has between the same nodes are left out.
   - **Not joined to the vehicle**: parts that would fall off. **tie** beams the suggested node to its three nearest
     body nodes (never to the suspension).
   - **Near a vehicle node but not on it**: sketch nodes within 5 cm of one. Move them onto it, or tie them.
4. **Make a new vehicle → Export**. The sketch's nodes (as `bfsk…`) and beams are written into the main part.

For a single extra beam, use **Base vehicle → Beams** instead.

---

## 8. Testing in the game

1. **Close the game.**
2. Put the zip in your user folder's `mods` folder, for example
   `AppData/Local/BeamNG/BeamNG.drive/current/mods/`.
3. If an older build of the same vehicle was loaded before, delete its cache:
   `current/temp/vehicles/<id>`.
4. Keep only the builds you are testing in `mods`. Old ones with the same parts confuse results.
5. Start the game and spawn the vehicle by its name.

**What to report:** whether it spawns; whether it holds its shape standing still; how it drives and steers; anything
that shakes, explodes, sags or breaks. Note when it happens: at spawn, standing, rolling, braking or at full lock.

When something is wrong and the obvious fix doesn't help, build test cars that change **one thing each** and name
them so they are easy to tell apart in the game.

---

## 9. Stability: what the bands mean

BeamNG steps its physics 2,000 times a second. A node that is too stiff or too damped for its weight vibrates or
explodes. BeamForge compares every node with **5,240 nodes of ten vanilla cars**:

| Band | Meaning | In a vanilla car |
| --- | --- | --- |
| **ok** | at or below the 90th percentile of vanilla nodes | about 90 % of nodes |
| **high** | up to the 99th percentile | about 10 % |
| **extreme** | up to the highest any vanilla node runs at | about 1 % |
| **beyond** | stiffer than any vanilla node | none: the likeliest place to ring or explode |

New beams (in sketches, hand-added beams, mounts, ties) aim at the vanilla median for a beam and keep both their nodes
in **ok**. A few *high* nodes are normal. *Beyond* is a reason to look.

---

## 10. Coordinates

| Frame | X | Y | Z | Origin | Used in |
| --- | --- | --- | --- | --- | --- |
| **SVJ (SAE)** | forward | right | down | on the ground under the front axle centre | sketches, components, SVJ files (mm in the editor) |
| **BeamNG** | left | rearward | up | the vehicle's own | the Base vehicle's Move panel (m) |

A sketch drawn over a base vehicle is placed at that vehicle's front axle and ground. A STEP from CAD may need
**Turn, move, scale** (90° turns, unit fix) to sit in the SVJ frame.

---

## 11. Troubleshooting

| Problem | What to do |
| --- | --- |
| "Port … is already in use" | An older server is still running. Stop it, or run `python serve.py 8001` |
| The editor looks old after an update | `serve.py` turns caching off. If you use another server, hard-reload (Ctrl+Shift+R) |
| Chrome refuses the BeamNG folder | Use **With file dialog** (folders under AppData and Program Files) |
| A project opens without its vehicle | Add your BeamNG folders. The vehicle opens with its edits once it is found |
| The SVJ's meshes are missing after opening a project | Point the SVJ workspace at the meshes folder again |
| "learn from the install again" when making a car | The library is from an older BeamForge. Powertrain → **learn again** |
| A sketch part is "Not rigid" | Add lines, or make it a *link*. BeamForge adds helpers, but a part needs at least one real triangle |
| A mounted sketch "would fall off" | Put one of its nodes on a vehicle node, or use **tie** |
| A beam is "held at what the nodes can take" | The nodes are already as stiff as vanilla nodes go. Use a softer value, or spread the load over more beams |
| A car from scratch spawns and explodes | Run **Checks** on the sketch. Report the vehicle and what happened (see [§8](#8-testing-in-the-game)) |

More detail: [fitting.md](fitting.md) (the fit), [beamng-vehicles.md](beamng-vehicles.md) (how BeamNG builds
vehicles), [study-step-and-full-editor.md](study-step-and-full-editor.md) (the rules behind sketches and stable
values), [roadmap.md](roadmap.md) (what is done and what comes next).
