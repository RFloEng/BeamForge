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

Read from the configured `compact` (sport_RS_wagon_DCT) with the scratchpad's `iface2.py` and
`unsprung.py`.

### What follows the wheel

The wheel is pushed up in the static solve (kinematics._system). The nodes that move with it are the
running gear; those that stay still are the body side:

| corner | follows the wheel (replaced)                                  | stays (kept)                     |
|--------|---------------------------------------------------------------|----------------------------------|
| FL     | fhub1 fhub3 fhub4 fhub5 fhub7 ftop2, barf1 barf2 (anti-roll bar link)     | fsub1 fsub2 fsub5 fsub6 fsub6l fsub7         |
| RL     | rhub1 rhub2 rhub3 rhub4 rhub5 rhub6, barr1 barr2                          | rsub1 rsub2 rsub3 rsub4 rsub5 rtopm1 barr3   |

This needs no table: what to replace is found per base by the same test. On the front-drive compact it matches
the hand-written role table exactly.

### The suspension slots, part by part

`compact_suspension_F` and `_R` hang from the body. Their children:

| part (front / rear)                         | what it holds                                   | archetype |
|---------------------------------------------|-------------------------------------------------|-----------|
| compact_suspension_F / _R                 | subframe nodes (fsub*, rsub*), hub and arm nodes, their beams | split: subframe kept, hub and arms replaced |
| compact_strut_F / _R                      | spring and damper beams, hub to strut top       | replaced (the SVJ's spring and damper) |
| compact_swaybar_F / _R                    | anti-roll bar, torsion bars to the arms         | replaced |
| compact_steering (front)                  | rack (fhub6 rail ends, fsub5 slide nodes, hydros), tie rods to fhub3 | replaced |
| compact_wheeldata_F / _R                  | pressureWheels: node1/node2 fwhl1, fwhl1ll; arm, torque and steering axis nodes on the hub (fhub1 fhub4 fhub5 ftop1; rhub4); the diff's output nodes | replaced: same wheels, the archetype's hub nodes |
| brand_wheel_*, tire_*                     | the wheel nodes fwhl1l fwhl1ll (rwhl1l rwhl1ll)         | kept |
| compact_brake_*, brakepad_*               | no nodes                                        | kept |
| compact_halfshafts_F / _R                 | bounded beams diff output (fdiffout1, rdiffout1) to fwhl1/rwhl1 | kept |
| compact_differential_R (+ carrier, final drive, driveshaft) | rdiffout1, rdiffout2, rdiffout3; beams to rsub1-rsub4 | kept (the subframe stays) |
| compact_undertray                         | beams to ftop1, fsub1 fsub2 fsub6                       | kept |

### Names that must stay

Other parts use these nodes, so the archetype keeps them (at the base's place, or its own with the
same name):

- **wheel nodes** fwhl1l fwhl1ll fwhl1r fwhl1rr, rwhl1l rwhl1ll rwhl1r rwhl1rr: defined by the wheel part; the fenders,
  doors and bumpers beam to them (wheel intrusion), the halfshafts and pressureWheels use them. The
  archetype's hub beams to these names, as the vanilla hub does.
- **subframe** fsub1 fsub2 fsub5 fsub6 fsub7 and rsub1-rsub5: the body, engine mounts, exhaust, undertray and the
  rear differential's carrier beam to them. They stay where they are. The archetype's inner pivots are
  its own nodes, mounted to them.
- **strut tops** ftop1 ftopm1, rtop1 rtopm1: the body, fenders, rear doors, rear seats, undertray and bumper
  use them. The archetype's strut top is its own node, mounted to these; they stay.
- **diff outputs** fdiffout1 (from the transaxle's differential, outside the suspension) and rdiffout1:
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
- Points the fit had already put on the SVJ's (the front strut tops ftopm1, the rear lower arm's rear
  pivot rsub2) are used as they are. The other pivots and the rear strut tops are new nodes, each beamed
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
  front-drive compact (the rack stays on its subframe's slide nodes). The first build wrote its own rack (rail,
  slide nodes, hydros as the vanilla's, the game logged no error) and the steering was free in the
  game: keep BeamNG's own steering until a written rack is proven. The own rack is kept only as the
  fallback for a base whose rack ends are not found.
- Wheels spawn square, the hub's beams to the wheel preloaded to the SVJ's camber and toe (see
  fitting.md, *Wheels spawn square*).
- Steering dampers, as the vanilla's (the front-drive compact's `fhub3`-`fsub2`, `fhub5`-`fsub2`: |BOUNDED, no spring,
  damping 80, fast 800): the tie rod end and the node across the hub to the nearest lower arm pivot.
  v4 had none (the vanilla's went with the hub nodes) and its front wheels steered back and forth
  under braking, while the converted front-drive compact, which keeps its own, did not. Not a compliance: the
  static solve gives the archetype less brake steer than the vanilla (0.78 against 1.96 deg/kN, 2.5
  against 28 mm/kN back). Every vanilla part the archetype drops is to be checked for what it did, not
  only for its nodes.
- Every node the archetype makes is weighed against all that loads it (`_size_nodes`): each beam's
  spring, or its limit spring if |BOUNDED and stiffer; its largest damping (slow, rebound, fast); slide
  node springs on the slide node and its rail's ends. Kept within k dt^2 / m 4.0 and c dt / m 1.0 (the
  vanilla front-drive compact's nodes reach 6.8 and 2.3). rigidity's check counts only beamSpring and beamDamp,
  and missed it: v5's new rear strut tops (2.5 kg) carried the dampers' rebound at c dt / m 2.75, more
  than any vanilla node, and the car shook at high frequency braking to a stop and broke its fuel tank
  on load (the tank's trigger beam tank nodes breaks at 200 N on 20 N s/m of damping, next to the rear
  pivots). v6: 6.9 kg, as the vanilla's 7.

In the game: v2 steers (still the wheel wobble, since found: the axle tilted at spawn); v4 no wobble,
but the front wheels steer under braking; v5 (steering dampers) still shook braking to a stop, front and rear, and broke its fuel tank on load;
v6 (strut tops weighed for their dampers) to be driven.

Not yet: anti-roll bars (the base's go with the hubs), suspension meshes (none in this SVJ; the base's
hub and arm meshes are dropped with their nodes).

## Still to check on other bases

- Front-wheel drive: fdiffout1 comes from the transaxle part (front-drive compact, small hatchback). Rear-wheel drive bases
  (RWD coupe, RWD saloon, older RWD saloon) have their differential under the rear suspension: same split.
- Differentials and halfshafts that beam to hub nodes instead of wheel nodes would need those names
  kept.
- Steering boxes that are not racks (the older RWD saloon's pitman arm and idler): their tie rod ends are found
  the same way (the steering part's nodes the base's tie rods reached); to check in the game.

## Next

- The editor: a choice of suspension per build, the base's converted (roles.py, convert.py) or the
  archetype, falling back to the base's where the SVJ's layout has no archetype. Both are developed.
- The double wishbone archetype (Miata, Civic, the BMWs' rears).
- Anti-roll bars and suspension meshes.
