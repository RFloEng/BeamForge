# Fitting a BeamNG vehicle to an SVJ

A vanilla car has 400–600 nodes and 3,000–5,000 beams: too many to move by hand. This page is the method for bending a base vehicle onto an SVJ automatically, in a few stages the user can check and accept one by one.

**Built:** stages 1 and 2 (`beamforge/fit.py`, the *Fit to the SVJ* section of the SVJ panel). **Next:** stage 3 and the hardpoint mapping.

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

- Every SVJ hardpoint is tied to one node of the base vehicle (see the mapping below). The tie gives a target displacement for that node.
- A smooth displacement field passes through every target exactly and fades to zero within a radius (about 0.3 m, a compactly supported radial basis function such as Wendland's). Nodes near a pickup point move with it, so the subframe, strut tower and arm mounts around it follow without kinks. Nodes far away do not move.
- Track widths are not a separate step. They follow from the wheel centre and upright hardpoints.

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

1. **Guess by role.** The wheel centre is the midpoint of the `pressureWheels` axle nodes. The upright is the group of nodes beamed rigidly to them. An arm is a beam (or pair of beams) from an upright node to a node of the body or subframe. Its inboard point is that other node. The strut top is the upright's highest beam end on the body. The tie rod is the arm whose inner end belongs to the steering part.
2. **Then by distance**, among the nodes with the right role, each node used once.
3. **The user checks it:** the editor shows each hardpoint, its node and the distance, and a click on another node re-ties it.
4. **Saved per suspension part:** a small mapping file per part (for example `hatchback_suspension_F`). Every vehicle and configuration that uses that part gets it for free.

**Choosing the base:** the closer the base vehicle, the smaller every stage. Pick one of the same kind, size and era (the RWD saloon for a BMW 3 Series), and above all with the same suspension type at each axle: a hardpoint can only be matched to a node that plays the same role. Where the types differ (a twist beam against a semi-trailing arm), only the wheel centre and the damper match; the editor should say so.

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

## Plan

1. ~~`beamforge/fit.py` with stages 1 and 2, and the editor's Fit section.~~ Done.
2. Stage 3: the role-based hardpoint-to-node mapping, the hardpoint table with click-to-retie, and the local displacement field.
3. Writing the fitted parts into a local mod (roadmap step 3).
