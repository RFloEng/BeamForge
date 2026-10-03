# Fitting a BeamNG vehicle to an SVJ

A vanilla car has 400–600 nodes and 3,000–5,000 beams: too many to move by hand. This page is the method for bending a base vehicle onto an SVJ automatically, in a few stages the user can check and accept one by one.

The base is a starting point, not a copy: the RWD saloon is taken and modified until it has the BMW E30's values. Large differences between the two are the work to be done, not errors. The editor lists them as the changes made to the base, and keeps red for real problems (a stage that could not run, a hardpoint without a node, a tie far from its point).

**Built:** all three stages and the hardpoint mapping (`beamforge/fit.py`, the *Fit to the SVJ* section of the SVJ panel). **Next:** writing the fitted parts into the game.

## What there is to fit to

| SVJ gives | Used for |
| --- | --- |
| `chassis.wheelbase`, `track_front`, `track_rear` | the axle lines and wheel centres |
| per corner `topology.upright.hardpoints` (ball joints, strut top, tie-rod end, wheel centre…) and every link's `inboard_points` | the suspension pickup points |
| the glTF meshes (`assets.meshes`, body bound to `SVJ::chassis::…`) | the overall size and the body shape |
| `tires`, `corners.*.wheel` | tyre radius and width (pressureWheels values, not node positions) |

An SVJ has no overall dimensions of its own: length, width, height, overhangs and ground clearance come from the bounds of its body mesh.

## The idea: one deformation, built in stages

Every stage is a map from old to new positions. It is applied to **every node of the configured vehicle** and to the flexbody `pos` values (wheels, brakes, hubs), so beams, wheels and parts all move together. The result is a set of node moves, the same thing as a hand move in the editor: the view, the mesh bending and a later hand correction keep working.

The order matters. Global, approximate stages come first; exact constraints come last, so that a later stage cannot undo them.

### Stage 1: wheelbase (longitudinal, piecewise linear)

- Fixed points: the front and rear axle lines (from the wheel centres).
- Between the axles, y is scaled by `wheelbase_svj / wheelbase_base`. In front of the front axle and behind the rear axle, nodes only shift with their axle: the overhangs keep their length here (stage 2 sets them).
- Piecewise linear, so it is continuous, keeps the order of every node along the car, and leaves the axle lines exactly where they are.

### Stage 2: body to the visual mesh (per axis, smooth)

- **The mesh:** the SVJ's chassis visual binding (its glTF node), placed like the SVJ overlay: origin on the base vehicle's front axle line and ground.
- **Length:** two more piecewise-linear segments along y. The front overhang (front axle to the front of the body nodes) is scaled to the mesh's front overhang, and the rear likewise. The axle lines stay fixed.
- **Width by section:** the car is cut into slices along y (10 cm). In each slice the body nodes' half width is compared with the mesh's (robust percentiles; on the node side over a wider window, because a node cloud is sparse). The profiles are smoothed along y, then each node's x is scaled about the centre line.
- **Heights, piecewise:** the ground stays; the floor (the lowest body point) goes to the mesh's floor; the wheel centres go to the SVJ wheel centre height, which keeps the wheels on the ground; the roof line goes to the mesh's, slice by slice. A per-slice floor was tried first and dropped: where a slice has no floor nodes, it jumps.
- **Measured on the body only:** suspension, wheel, drivetrain, mirror and antenna nodes are left out of the measurement, but every node moves with the map, so the car stays joined. Stage 3 then puts the pickups where the SVJ says.
- **Safety:** a mesh more than 1.6 times larger or smaller than the vehicle (wrong units or axes) is refused with a note, and the scale factors are clamped to 0.6–1.6.

### Stage 3: suspension pickup points (exact, local)

- Every SVJ hardpoint is tied to one node of the base vehicle (see the mapping below). The tie gives a target for that node. A wheel centre moves its wheel's two axle nodes together.
- A hub of the same shape as the SVJ upright (within 25 mm rms) moves as one piece with its tied points (the best rotation and translation, Horn's method). A hub of a different shape is reshaped: only its tied nodes are set, and the rest of it follows them through the displacement field. On the RWD saloon against the E30 that halves the largest hub beam change against moving it stiffly (57 % to 34 %).
- **The wheel axis** takes the SVJ's static camber and toe (`alignment`, radians) when the file has them, and otherwise keeps the base vehicle's. It is never taken from the hub's best rotation: with hubs of different shapes that rotation tipped wheels by 15–40° in testing.
- A smooth displacement field passes through every target exactly and fades to zero within a radius (Wendland's compactly supported function). The radius is 0.4 m, or three times the largest move (at most 1 m), so a long move is spread out instead of folding the structure. Nodes near a pickup point move with it, so the subframe, strut tower and arm mounts around it follow. Nodes far away do not move.
- Track widths are not a separate step. They follow from the wheel centre and upright hardpoints.
- **Changes and problems:** beams that change length more than 2 times and reshaped hubs are listed as the largest changes made to the base (expected, and a place to look if a tie is wrong). Ties the user set more than 25 cm from their hardpoint, and hardpoints with no node, are problems.
- **Uprights against the SVJ:** per corner, the base hub's tied points (wheel centre and upright hardpoints) are laid over the SVJ upright points by the best rigid move. What is left is the shape gap (0 for the same upright; it needs 3 points). The editor shows it with the worst point pair, the largest hub beam change after the fit, and the fitted camber and toe beside the SVJ's. Hubs more than 25 mm off are listed among the changes as reshaped.

This is your order (wheelbase, pickup points, rest of the body) with the last two swapped. Fitting the body after the pickup points would move them again; fitting it first and finishing with the exact pickup points keeps them exact.

## The mapping: which node is which hardpoint

Vanilla suspensions have no meaningful node names (`fhub1l`, `fsub2l`, `rhub1l`). A test with the RWD saloon as base (BeamNG's 3 Series; wheelbase 2.589 m against the E30's 2.570 m) and the BMW E30 example as target, after stage 1 only:

| E30 hardpoint | Nearest RWD saloon suspension node | Distance |
| --- | --- | --- |
| FL strut top | fhub4l | 105 mm |
| FL lower ball joint | fhub1l | 99 mm |
| FL tie-rod end | fhub3l | 134 mm |
| FL wheel centre | fwhl2l | 79 mm |
| FL lower arm, front inner | fsub5l | 131 mm |
| FL lower arm, rear inner | fsub2l | 220 mm |
| FL tie-rod inner | fsub5l (again) | 88 mm |
| RL wheel centre | arbrl (anti-roll bar) | 68 mm |
| RL semi-trailing arm inner, both points | rsub2l (twice) | 202 and 270 mm |

The small hatchback as base gave the same picture (77–118 mm for the upright points, up to 296 mm for the inboard ones, one node taken twice and an anti-roll bar node for an arm mount). The RWD saloon's tracks are wider than the E30's (1.450 / 1.540 m against 1.407 / 1.415 m), so stage 3 moves its corners inwards.

The nearest node gets the upright points roughly right, but not the inboard points: a node is taken twice, and an anti-roll bar node is taken for a wheel centre. So the mapping uses the structure, not distance alone:

1. **Guess by role.** The wheel centre is the wheel's two `pressureWheels` axle nodes. The hub is the nodes of a suspension part beamed to them by a suspension part's beam (the axle nodes are also tied to the body by limiter beams; those body nodes are not the hub). The chassis side is the suspension parts' own nodes on that side near the wheel, and the far ends of the suspension beams that touch them (arm mounts on the body or subframe). Upright hardpoints go to hub nodes, link inner points to the chassis side. A hardpoint listed twice (a strut top that is also the strut link's inner point) counts once, as a chassis point.
2. **Then by shape and distance.** Upright hardpoints: every assignment to distinct hub nodes is tried, and the one whose shape (with the wheel centre) best matches the SVJ upright wins, nearness breaking ties (nearest first when there are too many to try). Link inner points: the nearest chassis-side node, each node used once.
3. **The user checks it:** the editor lists each hardpoint, its node, how it was tied and the gap before the fit. *re-tie* then a click on a node in the view ties it there; *auto* goes back to the guess.
4. **Later, saved per suspension part:** a small mapping file per part (for example `hatchback_suspension_F`), so every vehicle and configuration that uses that part gets it for free. Today the ties live with the open vehicle.

**Choosing the base:** the closer the base vehicle, the smaller every stage, so pick one of the same kind, size and era (the RWD saloon for a BMW 3 Series). Differences are expected and are what the fit changes. Where the suspension types differ (the RWD saloon's multi-link rear against the E30's semi-trailing arm), only the points that have a node of the same role are tied (the wheel centre, the damper, the arm's mounts); the base keeps its other links.

## The user's choice per stage

Each stage is a row in the SVJ panel, as in roadmap step 3. It shows the base value, the SVJ value and the residual after fitting (for example the largest hardpoint error in mm), with a take box. A stage that is not taken leaves the vehicle as it is. Hand moves made after the fit stay on top.

## What fitting does not do

- **Stiffness:** beam rest lengths come from node positions when the game spawns the vehicle, so moved nodes give correct beams. But springs, dampers and beam strengths keep their values. Springs and dampers are taken from the SVJ as separate parameters.
- **Mass:** node weights stay as they are, so the total mass and the CG change only through node positions. Matching `mass_total` and the CG is a separate parameter (scaling `nodeWeight` per part).
- **Vanilla meshes:** in the editor they bend with the nodes. In the game a flexbody binds to the node positions at spawn, so a vanilla mesh would not stretch with the fit. That does not matter, because the SVJ glTF meshes replace them (roadmap steps 4 and 5).
- **Expressions:** a node whose position is a `$=` expression or a tuning variable gets a plain number when it is written. The tuning variables that move it keep working on top.

## Writing it into the game

The fit gives each node a new final position. A generated copy of each changed part is written to a local mod folder. Its node positions are the new positions minus the slot offsets the part gets (`nodeOffset` with x mirrored by side, `nodeMove`), so the game rebuilds the same shape. The flexbody `pos` values are written the same way. These files are made on the user's machine from their own install and are never committed here (see the roadmap).

## Results so far

RWD saloon fitted to the BMW E30 example, with a real car body mesh as the target (the E30 example ships none):

| | Base | Target | After |
| --- | --- | --- | --- |
| Wheelbase | 2.589 | 2.570 | 2.570 |
| Length | 4.361 | 3.700 | 3.700 |
| Width | 1.920 | 1.681 | 1.672 |
| Height | 1.140 | 1.116 | 1.100 |
| Front / rear overhang | 0.885 / 0.906 | 0.572 / 0.557 | 0.572 / 0.557 |

Metres. Width and height land within 1–2 cm (the per-slice profiles are smoothed). A mesh in centimetres is refused.

Stage 3 on the same pair: the largest pickup gap goes from 218 mm to 0 (every tied node exactly on its hardpoint) in a few hundredths of a second. Four beams change length more than twice, around the rear damper mount: the RWD saloon's rear is not a semi-trailing arm, so its nodes are far from the E30's points, and the note says where to look.

Uprights: the RWD saloon's front hub is 32 mm (rms) off the E30 upright after stage 1 alone, 40 mm with the body stage. The worst pair is wheel centre to lower ball joint, 162 mm on the RWD saloon against 217 mm on the E30 (it runs 19-inch wheels, the E30 14-inch). The MX-5 template's double-wishbone front on the RWD saloon's MacPherson base: 132 mm with nearest-node ties, 59 mm with shape-matched ties, still flagged.

## Plan

1. ~~`beamforge/fit.py` with stages 1 and 2, and the editor's Fit section.~~ Done.
2. ~~Stage 3: the role-based hardpoint-to-node mapping, the hardpoint table with click-to-retie, and the local displacement field.~~ Done.
3. ~~Writing the fitted parts into the game~~: done as a new vehicle mod (see beamng-vehicles.md, *Make a new vehicle*). Still to do: the ties saved per suspension part.
4. The base vehicle's own suspension studied with the solver, from its tied nodes.
