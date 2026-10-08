# Suspension archetypes

BeamForge's own suspension jbeam, built from the SVJ's hardpoints, in place of the base's suspension
converted node by node (roles.py, convert.py). The base keeps its body, subframes, powertrain, wheels
and brakes. Everything from the arms' inner pivots out to the wheel is BeamForge's.

Why: role tables have to be written for every vanilla suspension, and they cannot change its type. A
double wishbone SVJ on a MacPherson base stays a strut; a wishbone rear on a semi-trailing base loses
its upper arm. An archetype has the SVJ's type and its hardpoints as nodes, so the geometry is the
SVJ's by construction. Two archetypes cover the converter's output today:

| SVJ links                                               | archetype       |
|---------------------------------------------------------|-----------------|
| lower_control_arm (2), steering_tie_rod or toe_link, strut | strut        |
| upper_wishbone (2), lower_wishbone (2), steering_tie_rod or toe_link | double wishbone |

## Step 1: the front-drive compact's interface

Read from the configured front-drive compact (its sport wagon configuration) with the scratchpad's `iface2.py` and
`unsprung.py`.

### What follows the wheel

The wheel is pushed up in the static solve (kinematics._system). The nodes that move with it are the
running gear; those that stay still are the body side:

| corner | follows the wheel (replaced)                                  | stays (kept)                     |
|--------|---------------------------------------------------------------|----------------------------------|
| FL     | the hub nodes (anti-roll bar link)     | the subframe nodes         |
| RL     | the rear hub nodes                          | the rear subframe nodes   |

This needs no table: what to replace is found per base by the same test. On the front-drive compact it matches
the hand-written role table exactly.

### The suspension slots, part by part

`<base>_suspension_F` and `_R` hang from the body. Their children:

| part (front / rear)                         | what it holds                                   | archetype |
|---------------------------------------------|-------------------------------------------------|-----------|
| `<base>_suspension_F` / _R                 | subframe nodes (the subframe nodes), hub and arm nodes, their beams | split: subframe kept, hub and arms replaced |
| `<base>_strut_F` / _R                      | spring and damper beams, hub to strut top       | replaced (the SVJ's spring and damper) |
| `<base>_swaybar_F` / _R                    | anti-roll bar, torsion bars to the arms         | replaced |
| `<base>_steering` (front)                  | rack (the hub nodes rail ends, the subframe nodes slide nodes, hydros), tie rods to the hub nodes | replaced |
| `<base>_wheeldata_F` / _R                  | pressureWheels: node1/node2 the wheel nodes; arm, torque and steering axis nodes on the hub (the hub nodes); the diff's output nodes | replaced: same wheels, the archetype's hub nodes |
| <brand>_wheel_*, tire_*                     | the wheel nodes the wheel nodes (the rear wheel nodes)         | kept |
| `<base>_brake_*`, brakepad_*               | no nodes                                        | kept |
| `<base>_halfshafts_F` / _R                 | bounded beams diff output (the diff output nodes) to the wheel nodes | kept |
| `<base>_differential_R` (+ carrier, final drive, driveshaft) | the diff output nodes; beams to the rear subframe nodes | kept (the subframe stays) |
| `<base>_undertray`                         | beams to the strut top nodes                       | kept |

### Names that must stay

Other parts use these nodes, so the archetype keeps them (at the base's place, or its own with the
same name):

- **wheel nodes** the wheel nodes: defined by the wheel part; the fenders,
  doors and bumpers beam to them (wheel intrusion), the halfshafts and pressureWheels use them. The
  archetype's hub beams to these names, as the vanilla hub does.
- **subframe** the subframe nodes: the body, engine mounts, exhaust, undertray and the
  rear differential's carrier beam to them. They stay where they are. The archetype's inner pivots are
  its own nodes, mounted to them.
- **strut tops** the strut top nodes: the body, fenders, rear doors, rear seats, undertray and bumper
  use them. The archetype's strut top is its own node, mounted to these; they stay.
- **diff outputs** the front one (from the transaxle's differential, outside the suspension) and the rear one:
  kept, and the archetype's wheeldata points its torque at them as the vanilla one does.

### What the suspension hangs on

The vanilla suspension parts beam to these body nodes (besides their own subframe):

- front: f1 f2 f5 f6 f10-f12 f14 f15 f19 (both sides, ll/rr included)
- rear: f3 f4 f8 f9, q6 q7, r1 r3 r4 (ll/rr)

The archetype mounts its pivots and strut top to the nearest kept nodes: the subframe nodes first,
then these. Each pivot gets at least three mount beams to nodes not in a line, as the vanilla
subframe's are.

## The archetype part

One generated part replaces the hub and arms inside the suspension part (written as the base's
suspension part with the replaced nodes and their beams removed, plus the archetype's). The strut,
anti-roll bar, steering and wheeldata slots get BeamForge's parts.

Per corner, the nodes are the SVJ's hardpoints:

- hub: wheel centre (the wheel nodes' place), lower ball joint, upper ball joint or strut bottom, tie
  rod end, plus one or two stiffening nodes, so the hub is a rigid body
- inner pivots: one per arm point, mounted to the body side
- strut: its top, and a rail from the strut bottom to it with the hub's slide node (as BeamNG's
  MacPherson parts)
- steering: tie rod inner ends on a rack rail between the two sides, steered by hydros (as the
  vanilla rack)

The beam strengths and node weights are set once per archetype for stability at the physics step
(2000 Hz), then checked per car with kinematics.corner_stiffness. Spring and damper from the SVJ, as
now.

Meshes: the SVJ's suspension meshes as flexbodies on the archetype's node groups (hub, each arm). If
the SVJ has none, a simple generated mesh.

## Step 2: the strut archetype on the Subaru

`beamforge/archetype.py`, run after export (scratchpad `pair.py ... arch`). On the front-drive compact with the
Group A Subaru SVJ, all four corners:

- What follows the wheel found by kinematics.follows_wheel, as in step 1. Every row of the active parts
  that used those nodes is dropped: vanilla hub, arms, strut beams, anti-roll bar links, tie rods.
- Points the fit had already put on the SVJ's (the front strut tops the strut top nodes, the rear lower arm's rear
  pivot the rear subframe nodes) are used as they are. The other pivots and the rear strut tops are new nodes, each beamed
  to the five nearest body or subframe nodes within 0.45 m (never the engine, exhaust or differential:
  they move on their own mounts; the first build mounted to them and overloaded them).
- A body node's stiffness limit for the physics step (rigidity._limits) is kept by adding weight where
  its mounts need more room: 9.1 kg on 22 nodes. No node over its limit; stability as the base's.
  (Mounting only to nodes with room left found one node for the rear pivot: a pivot on one beam is
  loose. Weight is the way to make room.)
- Spring 45 075 / 46 380 N/m (SVJ wheel rate through the archetype's motion ratio, 0.99 / 0.97),
  preload for the built car's corner loads at the SVJ's ride height, dampers the SVJ's with the base's
  damping ratio as the floor.
- Steering: **the base's rack is kept** and the archetype's tie rods go to its ends (the nodes of the
  steering part the base's tie rods reached), 57 mm from the SVJ's tie rod inner points on the
  the front-drive compact (the rack stays on its subframe's slide nodes). The first build wrote its own rack (rail,
  slide nodes, hydros as the vanilla's, the game logged no error) and the steering was free in the
  game: keep BeamNG's own steering until a written rack is proven. The own rack is kept only as the
  fallback for a base whose rack ends are not found.
- Wheels spawn square, the hub's beams to the wheel preloaded to the SVJ's camber and toe (see
  fitting.md, *Wheels spawn square*).
- Steering dampers, as the vanilla's (the front-drive compact's: |BOUNDED, no spring,
  damping 80, fast 800): the tie rod end and the node across the hub to the nearest lower arm pivot.
  v4 had none (the vanilla's went with the hub nodes) and its front wheels steered back and forth
  under braking, while the converted front-drive compact, which keeps its own, did not. Not a compliance: the
  static solve gives the archetype less brake steer than the vanilla (0.78 against 1.96 deg/kN, 2.5
  against 28 mm/kN back). Every vanilla part the archetype drops is to be checked for what it did, not
  only for its nodes.
- Every node the archetype makes is weighed against all that loads it (`_size_nodes`): each beam's
  spring, or its limit spring if |BOUNDED and stiffer; its largest damping (slow, rebound, fast); slide
  node springs on the slide node and its rail's ends. Kept within k dt^2 / m 4.0 and c dt / m 1.0 (the
  vanilla the front-drive compact's nodes reach 6.8 and 2.3). rigidity's check counts only beamSpring and beamDamp,
  and missed it: v5's new rear strut tops (2.5 kg) carried the dampers' rebound at c dt / m 2.75, more
  than any vanilla node, and the car shook at high frequency braking to a stop and broke its fuel tank
  on load (the tank's trigger beam tank nodes breaks at 200 N on 20 N s/m of damping, next to the rear
  pivots). v6: 6.9 kg, as the vanilla's 7.

In the game: v2 steers (still the wheel wobble, since found: the axle tilted at spawn); v4 no wobble,
but the front wheels steer under braking; v5 (steering dampers) still shook braking to a stop, front and rear, and broke its fuel tank on load;
v6 (strut tops weighed for their dampers) to be driven.

Not yet: anti-roll bars (the base's go with the hubs), suspension meshes (none in this SVJ; the base's
hub and arm meshes are dropped with their nodes).

## The archetype's meshes

The glTFs have no skeleton, animation or skin (none of the seven AC conversions): their suspension parts are
separate rigid nodes at the static pose. `gltf.susp_pieces` finds them by name (hub, arm, strut, steering
rod; finest node, brakes and trim excluded: the Subaru's `x0_wishbone_*` and `x0_suspension_*`, the E30's
`g_SUSP_*_Hub / Lever_A / Dump`, the Miata's `GEO_HUB_* / GEO_SPRING_*`, the E92's and Z4's `SUSP_*_mesh`; the
Civic and Z3 name none). `export.susp_pieces` gives each to the wheel nearest its middle; one across the centre
line (the E30's front beam) stays in the body.

- Every piece is left out of the body mesh. Baked into it they hang at the static pose while the suspension
  moves (the Subaru's wishbones and struts, the Miata's `GEO_SUSP_*` were).
- On an archetype car each piece is a flexbody on a node group the archetype tags: `bf_hub_<corner>` (its
  hub nodes), `bf_arm_<corner>` (the ball joint and the two pivots), `bf_strut_<corner>` (the strut bottom and
  top), `bf_tie_<corner>` (the tie rod end and the rack end). A base node used as a point joins the group
  too (`_tag_base`). `archetype.piece_rows` makes the attach rows.
- A vanilla mesh bound to a removed node goes with it (a mesh bound to one removed and one kept node, such as
  the lower arm's, from a hub node to a pivot, hung in place from the kept one). The base's other meshes also go by
  default: only the SVJ's and the wheels' and tyres' are drawn (`_drop_vanilla_meshes`; switch
  `vanilla_meshes` keeps them). The base's brake calipers went with the hubs; the SVJ's
  (`x0_hub_caliper_*`) are not attached yet.
- On a converted car (the base's own suspension) the SVJ's pieces are not drawn, as nothing there is bound to
  them, and no base mesh is either: the suspension is invisible until they are attached (to the base's hub
  groups; not done).

No base mesh is drawn on any car (export `_drop_body_meshes`, whenever the SVJ's meshes go along): only the SVJ's,
the wheels' and tyres' (`KEEP`), and the lights (props whose mesh is SPOTLIGHT or empty). The props that animate a
mesh (pedals, stalks, gauge needles, the steering wheel, sun visors, the engine's pulleys and fan, the driveshaft)
go too. Checked on the seven cars with the audit of what each still draws (the active parts' flexbodies and props):
none left but the lights. The earlier rule kept the mechanical meshes (running gear, engine bay, tank).
- The diagnostic `strip_meshes` leaves only the SVJ's suspension parts drawn (everything else invisible, the
  wheels too): the clearest check that they move. Tested on the Subaru: the parts move right.

## The oil message

"Motor sin aceite" appeared on every car with the front archetype and on none without it. The engine, its mounts,
the oil pan and the weights are identical in all of them, so it is not the engine's files. The oil pan is a
node whose beams deform at 8000 N and then leak (`oilpan_damage`, lua/vehicle/powertrain/combustionEngine.lua);
the engine hangs on the subframe nodes through bounded mounts. The archetype reused and moved those nodes
as its arm pivots (46 mm and 99 mm at the front). Test O1 (those nodes never reused or moved) had no oil
message. Moving the engine-mount nodes is the cause, and not reusing or moving them is now the default
(`apply(switch={"move_mounts"})` restores the old way). Which nodes those are depends on the base: the
engine hangs on the subframe nodes on the front-drive compact, the subframe nodes on the RWD compact coupe, the subframe nodes on the two RWD saloons
(found from the engine-mount parts' beams), so the Z3, Z4 and E92 still reuse the subframe nodes.

The price: without them the front pivots are own nodes mounted to body nodes at least 10 cm away, and the
front's lowest mode is 13 Hz at 1 % (27 Hz with the reused nodes; the vanilla's is 40). Not yet settled which
is right; O1's wobble was not reported. The way to both: keep the subframe nodes where they are and give the pivot a
stiff bracket of beams to the nearest subframe nodes, the engine-mount nodes included, which are loaded but not
moved. The 10 cm minimum of `_mounts` stays out of that (it kept the first build's short stiff beams off the
fuel tank's trigger beam).

Where a pivot or strut top may be mounted (`_mounts`): never to the steering rack's slide nodes, nodes a hydro
or a torsion bar drives, or the engine's mount nodes (`avoid`), unless the base's own lower arm hangs on them
(the front-drive compact's is on the subframe nodes, its engine mounts too). v11 mounted pivots to the subframe nodes (the rack's slide node:
steering dragged the pivot, and the Subaru's front broke) and the Z3's to the subframe nodes (engine mounts; the oil
message). The reuse loop and the mount list used the same variable name for a while, which hid the filter: the
audit (pivots and tops by the nodes they are anchored to) is the check.

A steered corner needs the base's steering rack (a steering part with hydros) for its tie rod. A base
without one, the older RWD saloon's pitman arm and idler (the E30), keeps its own suspension at that corner: an archetype
rack of its own left the steering free in the game.

Mixed cars (an archetype front on a converted rear: the Z3, Z4 and E92) draw no vanilla meshes, so their
rear suspension is not drawn until the double wishbone archetype exists.

## Still to check on other bases

- Front-wheel drive: the diff output nodes comes from the transaxle part (the front-drive compact, the small hatchback). Rear-wheel drive bases
  (the RWD compact coupe, the RWD saloon, the older RWD saloon) have their differential under the rear suspension: same split.
- Differentials and halfshafts that beam to hub nodes instead of wheel nodes would need those names
  kept.
- Steering boxes that are not racks (the older RWD saloon's pitman arm and idler): their tie rod ends are found
  the same way (the steering part's nodes the base's tie rods reached); to check in the game.

## Next

- The editor: a choice of suspension per build, the base's converted (roles.py, convert.py) or the
  archetype, falling back to the base's where the SVJ's layout has no archetype. Both are developed.
- The double wishbone archetype (Miata, Civic, the BMWs' rears).
- Anti-roll bars and suspension meshes.

## SVJ v0.99.2: the file says which node is which part

Older files (the AC converter's, v0.97) bind only each corner's upright (`SVJ::body::upright_fl`), so the archetype finds
the arms, struts and steering rods inside the mesh by their names (`gltf.susp_pieces`). From v0.99.2 (§22) a file can bind
every part itself: `topology.links[].visual`, `spring.visual`, `damper.visual`, `topology.axle_body.visual`, with nodes named
`SVJ::suspension::<part>_<corner>` (`docs/naming_convention.md` of the SVJ repository).

- `svj.part_bindings` reads them; `svj.part_role` maps a canonical part name onto the role a BeamNG corner follows (hub,
  arm, strut, tie rod; an anti-roll bar follows no corner).
- `export.susp_pieces` takes the declared parts as they are (from whichever mesh file holds them, the corner from the file,
  not the wheel nearest to the part's middle) when the file binds any part besides the upright; otherwise it keeps guessing
  by names.
- `placement: link_between_points` (§22.6): the mesh is authored in its own frame, not at the design position.
  `export.link_place` turns its `mesh_axis` onto the vector from the link's inboard end to its outboard hardpoint and puts its
  origin on the inboard end (scaling along the axis if asked); the archetype's pieces then follow their node group as before.
  Unverified against a real file: the axes are read in the SAE frame of the mesh (after `gltf_to_sae`), which the spec
  does not say outright.

Not used yet: `static_setup` (ride height, corner weight, spring compression: the preload is computed for the car as built, which
is what sets its ride height), `compliance_summary` (toe and lateral stiffness), and the A-notation stations (`A1L`...) of
multi-axle vehicles (BeamForge's corners are FL, FR, RL, RR).
