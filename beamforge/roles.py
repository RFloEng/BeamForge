"""Suspension roles of a base vehicle's suspension parts, from role tables the user keeps locally.

Pure Python, standard library only (runs in Pyodide). To convert a base vehicle's suspension into an
SVJ's (convert.py), BeamForge has to know what each of its nodes is: which make the upright, which are
its joints, and which are the inner pivots of each link. BeamNG does not say (a part is nodes and
beams; an arm may be two beams to subframe nodes, or a body of its own), and guessing by distance can tie
the wrong nodes (a lower arm pivot taken for the tie rod's inner end). So a suspension part can be given a
role table, read from the car's own jbeam (its comments and node groups).

BeamForge ships no tables: they name game parts and nodes. The user's own are read from local/roles.json
beside the package (gitignored; the editor fetches it when it is there), or given with load(). Without a
table a corner is fitted by the guess (fit.py).

Per suspension part name: node ids without their side letter ("l" or "r" is added per corner):

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

import json
import math
from pathlib import Path

NEAR = 0.8   # m: an upright's nodes are this close to its wheel centre
LOCAL = Path(__file__).resolve().parent.parent / "local" / "roles.json"

ROLES = {}


def load(tables):
    """Add role tables ({part: table}, as above); returns how many."""
    n = 0
    for part, t in (tables or {}).items():
        if isinstance(t, dict) and isinstance(t.get("upright"), list):
            ROLES[part] = t
            n += 1
    return n


def load_json(text):
    return load(json.loads(text))


try:                                                       # the user's own tables, never in the repository
    load(json.loads(LOCAL.read_text(encoding="utf-8")))
except (OSError, ValueError):
    pass


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
