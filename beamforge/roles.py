"""Suspension roles of vanilla BeamNG cars, by suspension part.

Pure Python, standard library only (runs in Pyodide). To convert a base vehicle's suspension into an
SVJ's (convert.py), BeamForge has to know what each of its nodes is: which make the upright, which are
its joints, and which are the inner pivots of each link. BeamNG does not say (a part is nodes and
beams; an arm may be two beams to subframe nodes, or a body of its own), and guessing by distance tied
the wrong nodes (a lower arm pivot taken for the tie rod's inner end). So the roles are written down
here once per suspension part, read from the car's own jbeam (its comments and node groups).

Node ids are given without their side letter: "l" or "r" is added per corner. Per part:

  upright      the nodes of the upright (hub, knuckle), the strut bottom included
  joints       upright points by role: lower_ball_joint, upper_ball_joint, tie_rod_end (several
               nodes: their middle, e.g. the outer ends of two lateral links)
  pivots       inner pivots of each link by role (lower_arm, upper_arm, tie_rod, trailing_arm),
               front to rear
  strut        a MacPherson strut: its top node(s) and the upright node its rail starts from
  keep_arms    True where the SVJ's arm points are no use to this base's arm (the SVJ's layout is another kind of
               arm, or its points are inconsistent): the arms' pivots keep the base's own place relative to the
               ball joint, and move with it
  caster       False for an upright that is a trailing arm (long, pivoting at its front): not turned to
               the SVJ's steering axis, which would swing its pivot up or down
"""

import math

NEAR = 0.8   # m: an upright's nodes are this close to its wheel centre

ROLES = {
    # front-drive compact: MacPherson front, A-arm of two beams from the hub to the subframe
    "compact_suspension_F": {
        "upright": ["fhub1", "fhub3", "fhub4", "fhub5", "fhub7", "ftop2"],
        "joints": {"lower_ball_joint": ["fhub1"], "tie_rod_end": ["fhub3"]},
        "pivots": {"lower_arm": ["fsub1", "fsub2"], "tie_rod": ["fhub6"]},
        "strut": {"top": ["ftop1", "ftopm1"], "rail_start": "ftop2"},
    },
    # front-drive compact: strut rear, a trailing arm and two lateral links. As an A-arm and a toe link
    # (an SVJ strut rear): the trailing arm and the rear link are the arm (its front and rear pivots,
    # its outer joint between theirs), the front link is the toe link
    "compact_suspension_R": {
        "upright": ["rhub1", "rhub2", "rhub3", "rhub4", "rhub5", "rhub6"],
        "joints": {"lower_ball_joint": ["rhub2", "rhub1"], "tie_rod_end": ["rhub3"]},
        "pivots": {"lower_arm": ["rsub5", "rsub1"], "tie_rod": ["rsub2"]},
        "strut": {"top": ["rtop1", "rtopm1"], "rail_start": "rhub5"},
    },
    # RWD compact coupe: MacPherson front, a lateral arm and a tension rod to the hub's lower joint
    "coupe_suspension_F": {
        "upright": ["fhub1", "fhub3", "fhub4", "fhub5", "ftop2"],
        "joints": {"lower_ball_joint": ["fhub1"], "tie_rod_end": ["fhub3"]},
        "pivots": {"lower_arm": ["fsub1", "fsub2"], "tie_rod": ["fhub6"]},
        "strut": {"top": ["ftop1"], "rail_start": "ftop2"},
    },
    # RWD compact coupe: multilink rear, a lower A-arm, two upper links (an upper arm) and a toe link;
    # coilover to the body
    "coupe_suspension_R": {
        "upright": ["rhub1", "rhub2", "rhub3", "rhub4", "rhub5", "rhub6"],
        "joints": {"lower_ball_joint": ["rhub1"], "upper_ball_joint": ["rhub3", "rhub4"], "tie_rod_end": ["rhub5"]},
        "pivots": {"lower_arm": ["rsub2", "rsub1"], "upper_arm": ["rsub3", "rsub4"], "tie_rod": ["rsub5"]},
    },
    # RWD saloon: MacPherson front, two lower links (a lateral arm and a tension rod, each to its own joint),
    # the strut's rail from the lower joint
    "saloon_suspension_F": {
        "upright": ["fhub1", "fhub3", "fhub4", "fhub5", "fwhl2", "ftop2"],
        "joints": {"lower_ball_joint": ["fhub1", "fwhl2"], "tie_rod_end": ["fhub3"]},
        "pivots": {"lower_arm": ["fsub1", "fsub2"], "tie_rod": ["fhub6"]},
        "strut": {"top": ["ftop1"], "rail_start": "fhub1"},
    },
    # RWD saloon: multilink rear, two lower links, two upper links and a toe link
    "saloon_suspension_R": {
        "upright": ["rhub1", "rhub2", "rhub3", "rhub4", "rhub5", "rhub6"],
        "joints": {"lower_ball_joint": ["rhub1", "rhub3"], "upper_ball_joint": ["rhub4", "rhub6"], "tie_rod_end": ["rhub2"]},
        "pivots": {"lower_arm": ["rsub2", "rsub1"], "upper_arm": ["rsub4", "rsub5"], "tie_rod": ["rsub3"]},
    },
    # older RWD saloon: MacPherson front, two lower links to their own joints, tie rod to the steering linkage
    "saloon2_suspension_F": {
        "upright": ["fhub1", "fhub3", "fhub4", "fhub5", "fwhl2"],
        "joints": {"lower_ball_joint": ["fhub1", "fwhl2"], "tie_rod_end": ["fhub3"]},
        # the tie rod's inner end (steer1) is left where it is: it hangs on the steering box's idler and pitman
        # linkage (a rail with torsion bars to the pitman), which does not move with it: moved onto the SVJ's
        # point the linkage binds and the car does not turn
        "pivots": {"lower_arm": ["fsub1", "fsub2"]},
        "strut": {"top": ["ftop1"], "rail_start": "fhub1"},
        # the SVJ's lower arm is not a wishbone's (the E30's real one is an L-arm located by the anti-roll bar): on
        # the SVJ's two points the base's upright and strut moved 0.39 deg of toe per 10 mm of bump (a vanilla car's
        # 0.02). The arm keeps the base's own geometry, moved with the corner
        "keep_arms": True,
    },
    # older RWD saloon: semi-trailing arm rear, the hub rigid on the arm. As a lower arm (an SVJ wishbone rear):
    # its two pivots on the SVJ's lower arm's, measured from the hub's bottom
    "saloon2_suspension_R": {
        "upright": ["rhub1", "rhub2", "rhub4", "rhub5", "rhub6"],
        "joints": {"lower_ball_joint": ["rhub1"]},
        "pivots": {"lower_arm": ["rsub1ll", "rsub1"]},
        # a semi-trailing arm on an SVJ wishbone's points: 0.77 deg of toe per 10 mm of bump (6 deg over 80 mm)
        "keep_arms": True,
    },
    # small hatchback: double wishbone front, a lateral arm and a torque rod (the lower arm), the upper
    # arm's pivots on the body
    "hatchback_suspension_F": {
        "upright": ["fhub1", "fhub2", "fhub3", "fhub6"],
        "joints": {"lower_ball_joint": ["fhub1"], "upper_ball_joint": ["fhub2"], "tie_rod_end": ["fhub3"]},
        "pivots": {"lower_arm": ["fsub1", "fsub2"], "upper_arm": ["fsub3", "fsub4"], "tie_rod": ["fhub8"]},
    },
    # small hatchback: trailing arm rear (the hub on it), a lower and an upper lateral link. The links to
    # the SVJ's arms; the trailing arm's pivots have no SVJ counterpart (left to the field)
    "hatchback_suspension_R": {
        "upright": ["rhub1", "rhub2", "rhub3", "rhub4", "rwhl2", "rwhl3"],
        "joints": {"lower_ball_joint": ["rhub1"], "upper_ball_joint": ["rhub3"]},
        "pivots": {"lower_arm": ["rsub1"], "upper_arm": ["rsub3"], "trailing_arm": ["rsub2"]},
        "caster": False,
    },
}
ROLES["saloon_suspension_F_wide"] = ROLES["saloon_suspension_F"]


def for_corner(parts, nodes, wheel):
    """The role table of the suspension at a wheel, with the side letter added: {"part", "upright",
    "joints", "pivots", "strut"} in node ids, or None when its part has no table (or a node is missing)."""
    side = "l" if (nodes.get(wheel["node1"], [0])[0] + nodes.get(wheel["node2"], [0])[0]) > 0 else "r"
    for part, t in ROLES.items():
        up = [n + side for n in t["upright"]]
        if not all(n in nodes and parts.get(n) == part for n in up):
            continue
        wc = [(nodes[wheel["node1"]][i] + nodes[wheel["node2"]][i]) / 2 for i in range(3)]
        if max(math.dist(nodes[n], wc) for n in up) > NEAR:
            continue                                       # another corner's (the front part at a rear wheel)
        sided = lambda ids: [n + side for n in ids if n + side in nodes]  # noqa: E731
        out = {"part": part, "upright": up,
               "joints": {k: sided(v) for k, v in t.get("joints", {}).items()},
               "pivots": {k: sided(v) for k, v in t.get("pivots", {}).items()}}
        if "caster" in t:
            out["caster"] = t["caster"]
        if t.get("keep_arms"):
            out["keep_arms"] = True
        if t.get("strut"):
            out["strut"] = {"top": sided(t["strut"]["top"]), "rail_start": t["strut"]["rail_start"] + side}
        return out
    return None
