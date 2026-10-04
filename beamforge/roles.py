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
}


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
        if t.get("strut"):
            out["strut"] = {"top": sided(t["strut"]["top"]), "rail_start": t["strut"]["rail_start"] + side}
        return out
    return None
