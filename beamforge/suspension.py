"""Suspension kinematics: the motion of a corner over wheel travel (from the FBeam editor).

BeamForge uses it to study an SVJ's suspension (svj.suspension_corners reads the corners), and
later the base vehicle's once its nodes are tied to the hardpoints (docs/fitting.md).

Pure Python, standard library only (runs in Pyodide). BeamNG axes: X left, Y rear,
Z up, metres. Corners are defined for the left side (X > 0); the right side is the
mirror image (X -> -X). Angles are degrees in the results, radians inside.

One solver for every layout. The upright (hub carrier) is a rigid body with six
degrees of freedom, pose x = (t, w): p' = WC0 + t + R(w) (p - WC0), with t a
translation and w a rotation vector about the static wheel centre WC0. The
layout is a list of links that constrain it (constraint count in brackets):
  rod           [1] a link with a ball joint at each end: one point of the upright keeps
                its distance to one chassis point (tie rod, toe link, one link of a multi-link)
  arm           [2] an A-arm / wishbone: two rods from two inner pivots to one outer ball joint
  strut         [2] a MacPherson strut: the strut axis, fixed in the upright, passes through
                the chassis top mount
  pivot         [5] the upright is rigid with an arm that turns about a chassis axis through
                its two points (trailing arm, semi-trailing arm)
  pivot_mirror  [5] as pivot, about the line from one chassis point to its mirror image
                (both bushes of a twist beam)
  ball          [3] the upright is rigid with an arm on a ball bush at one chassis point
  watt          [1] Watt's linkage on a solid axle: its centre pivot keeps its static X
                (an ideal straight-line linkage; the rods WL/WR are drawn, not solved)
For a solid axle the "upright" is the whole axle with both hubs on it: every rod and
arm is used on both sides (mirrored) except the ones listed as single (Panhard rod),
the two wheel centres are driven together (mean height), and the roll centre is the
height of the lateral locator.
A corner has one degree of freedom left (five independent constraints); the sixth
equation drives the wheel-centre height to each travel step, and the pose is solved
by damped Gauss-Newton. Steering is held straight ahead (rack fixed) unless a rack
offset is given, which moves the point named TRI along X.

Everything that depends on the layout (pickups, moving points, the damper, the
steering axis, the roll centre) is derived from the links, so a new layout is
data, not code. To add one:
  1. Add a TOPOLOGY entry, keyed by the layout id:
       name        label shown in the editor
       about       one-paragraph help text
       links       [(kind, (chassis points...), upright point or None), ...]; kinds above.
                   rod/arm/watt/strut name their upright point; pivot/ball/pivot_mirror use
                   None (the chassis point itself is the point the upright turns about).
                   Constraints must add up to 5: rods and arms count twice on an axle
                   (unless single), and roll_lock counts as one.
       damper      (top on the chassis, bottom, carrier): carrier "upright" if the bottom
                   moves with the upright, or the index in links of the arm it sits on
       toe         the tie rod's outer point (normally "TRO", with "TRI" as its inner end)
                   if the layout steers, else None.
       draw        extra lines for the 3D view, (a, b, "upright"); "~P" is P mirrored
     optional:
       axle        True: one rigid body carrying both hubs (solid axle, de Dion)
       single      axle points used once, not mirrored: a rod is solved once when its
                   chassis and upright points are both listed, and the editor draws these
                   links and puts these pickups on one frame joint (Panhard, Watt's)
       roll_lock   True: add left height = right height, for an axle free in roll
       rc          roll centre from the lateral locator instead of the contact patch:
                   ("point", P) height of moving point P; ("line", A, B) height at X = 0
                   of the line from chassis point A to moving point B
       halfshafts  True: an axle with a chassis differential; DIFF becomes required and
                   the halfshaft angle and plunge are analysed
  2. Give every new point name a LABEL.
  3. Add a _DEFAULTS entry with every chassis and upright point (offsets from WC).
INNER, MOVING, REQUIRED and describe() follow automatically, and tests/test_suspension.py
runs every TOPOLOGY entry.

analyse() returns {"static": {...}, "curves": {...}, "frames": [...]}; see its docstring.
"""

import math

# Tyre used when a corner gives none (spec "tyre_radius"): a compact-car tyre, radius and width in m.
DEFAULT_TYRE = {"radius": 0.297, "width": 0.150}

# ---------------------------------------------------------------- vectors

def add(a, b): return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
def sub(a, b): return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
def mul(a, s): return [a[0] * s, a[1] * s, a[2] * s]
def dot(a, b): return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
def cross(a, b): return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
def norm(a): return math.sqrt(dot(a, a))
def unit(a): n = norm(a); return [a[0] / n, a[1] / n, a[2] / n]


def rot(axis, ang, v):
    """Rodrigues rotation of v about a unit axis through the origin."""
    c, s = math.cos(ang), math.sin(ang)
    return add(add(mul(v, c), mul(cross(axis, v), s)), mul(axis, dot(axis, v) * (1 - c)))


def rotvec(w, v):
    """Rotate v by the rotation vector w (axis w/|w|, angle |w| in radians)."""
    ang = norm(w)
    return v if ang < 1e-15 else rot(mul(w, 1 / ang), ang, v)


def compose(w2, w1):
    """Rotation vector of (rotate by w1, then w2), via the three basis vectors.

    Builds R = R(w2) R(w1) column by column, then takes it back to a rotation vector:
    angle = acos((trace R - 1) / 2), axis from the skew part (R - R^T) / (2 sin angle).
    """
    ex, ey, ez = (rotvec(w2, rotvec(w1, e)) for e in ([1, 0, 0], [0, 1, 0], [0, 0, 1]))
    R = [[ex[0], ey[0], ez[0]], [ex[1], ey[1], ez[1]], [ex[2], ey[2], ez[2]]]
    tr = R[0][0] + R[1][1] + R[2][2]
    ang = math.acos(max(-1.0, min(1.0, (tr - 1) / 2)))
    if ang < 1e-12:
        return [0.0, 0.0, 0.0]
    if ang > math.pi - 1e-6:                           # the skew-part formula breaks down near 180 deg
        raise ValueError("suspension rotated half a turn")
    k = ang / (2 * math.sin(ang))
    return [k * (R[2][1] - R[1][2]), k * (R[0][2] - R[2][0]), k * (R[1][0] - R[0][1])]


def _solve(A, b):
    """Solve A x = b by Gaussian elimination with partial pivoting (small dense systems).

    Raises ValueError when a pivot is (numerically) zero: a locked or underconstrained linkage.
    """
    n = len(b)
    M = [row[:] + [b[i]] for i, row in enumerate(A)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(M[r][c]))
        if abs(M[p][c]) < 1e-18:
            raise ValueError("suspension geometry is locked or singular")
        M[c], M[p] = M[p], M[c]
        for r in range(c + 1, n):
            f = M[r][c] / M[c][c]
            for k in range(c, n + 1):
                M[r][k] -= f * M[c][k]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (M[r][n] - sum(M[r][k] * x[k] for k in range(r + 1, n))) / M[r][r]
    return x


def mirror(p):
    """The point on the other side of the car (X -> -X)."""
    return [-p[0], p[1], p[2]]


# ---------------------------------------------------------------- layouts

# Display names of every point used in TOPOLOGY (and DIFF, WC); a new point needs one here.
LABEL = {
    "LIF": "Lower arm, inner front", "LIR": "Lower arm, inner rear", "LBJ": "Lower ball joint",
    "UIF": "Upper arm, inner front", "UIR": "Upper arm, inner rear", "UBJ": "Upper ball joint",
    "ST": "Strut top mount", "SB": "Strut lower mount (upright)", "TRI": "Tie rod / toe link, inner",
    "TRO": "Tie rod / toe link, outer", "DT": "Damper top", "DB": "Damper bottom",
    "WC": "Wheel centre", "DIFF": "Halfshaft inner joint",
    "UFI": "Upper front link, inner", "UFO": "Upper front link, outer",
    "URI": "Upper rear link, inner", "URO": "Upper rear link, outer",
    "LFI": "Lower front link, inner", "LFO": "Lower front link, outer",
    "LRI": "Lower rear link, inner", "LRO": "Lower rear link, outer",
    "PF": "Arm pivot, front (inner bush)", "PR": "Arm pivot, rear (outer bush)",
    "BSH": "Twist-beam bush",
    "LLI": "Lower lateral link, inner", "LLO": "Lower lateral link, outer",
    "ULI": "Upper lateral link, inner", "ULO": "Upper lateral link, outer",
    "LTI": "Lower trailing link, chassis", "LTO": "Lower trailing link, axle",
    "UTI": "Upper link, chassis", "UTO": "Upper link, axle",
    "PHC": "Panhard rod, chassis end", "PHA": "Panhard rod, axle end (other side)",
    "WP": "Watt's linkage, centre pivot (on the axle)",
    "WL": "Watt's linkage, upper rod chassis end", "WR": "Watt's linkage, lower rod chassis end (other side)",
}

# Layout catalogue; the keys are explained in the module docstring ("To add one").
# links: (kind, chassis points, upright point); "pivot" lists its two axis points;
# "pivot_mirror" turns about the line from BSH to its mirror image (both bushes of a twist beam).
# damper: (top on the chassis, bottom, carrier), carrier "upright" or the index of an arm link.
# draw: extra upright / damper lines for the 3D view; "~WC" is the right-hand wheel centre.
TOPOLOGY = {
    "macpherson": {
        "name": "MacPherson strut",
        "about": "Lower A-arm, strut fixed in the upright, tie rod. Light and compact; camber gain is small.",
        "links": [("arm", ("LIF", "LIR"), "LBJ"), ("strut", ("ST",), "SB"), ("rod", ("TRI",), "TRO")],
        "damper": ("ST", "SB", "upright"), "toe": "TRO",
        "draw": [("LBJ", "SB", "upright"), ("SB", "TRO", "upright"), ("LBJ", "TRO", "upright")],
    },
    "double_wishbone": {
        "name": "Double wishbone",
        "about": "Upper and lower A-arms and a tie rod. Camber gain and roll centre are set by the arm lengths and angles.",
        "links": [("arm", ("LIF", "LIR"), "LBJ"), ("arm", ("UIF", "UIR"), "UBJ"), ("rod", ("TRI",), "TRO")],
        "damper": ("DT", "DB", 0), "toe": "TRO",
        "draw": [("LBJ", "UBJ", "upright"), ("UBJ", "TRO", "upright"), ("LBJ", "TRO", "upright")],
    },
    "multilink": {
        "name": "Five-link (multi-link)",
        "about": "Five separate rods. The most freedom: the virtual steering axis, camber and toe curves come from where the links point.",
        "links": [("rod", ("UFI",), "UFO"), ("rod", ("URI",), "URO"), ("rod", ("LFI",), "LFO"),
                  ("rod", ("LRI",), "LRO"), ("rod", ("TRI",), "TRO")],
        "damper": ("DT", "DB", "upright"), "toe": "TRO",
        "draw": [("UFO", "URO", "upright"), ("LFO", "LRO", "upright"), ("UFO", "LFO", "upright"),
                 ("URO", "LRO", "upright"), ("LRO", "TRO", "upright")],
    },
    "trailing_arm": {
        "name": "Trailing arm",
        "about": "The hub is rigid with an arm that turns about a chassis axis. A lateral axis gives no camber or toe change in heave; angle it in plan view for a semi-trailing arm.",
        "links": [("pivot", ("PF", "PR"), None)],
        "damper": ("DT", "DB", "upright"), "toe": None,
        "draw": [("PF", "WC", "upright"), ("PR", "WC", "upright"), ("PF", "PR", "upright")],
    },
    "semi_trailing_arm": {
        "name": "Semi-trailing arm",
        "about": "A trailing arm whose pivot axis is swept in plan view (and may be tilted): camber and toe change with travel.",
        "links": [("pivot", ("PF", "PR"), None)],
        "damper": ("DT", "DB", "upright"), "toe": None,
        "draw": [("PF", "WC", "upright"), ("PR", "WC", "upright"), ("PF", "PR", "upright")],
    },
    "trailing_arm_lateral": {
        "name": "Trailing arm + lateral links",
        "about": "A trailing arm rigid with the hub, on a ball bush at its front end, located sideways by an upper and a lower lateral link (as a vanilla small hatchback's rear). Link lengths and angles set camber and toe.",
        "links": [("ball", ("PF",), None), ("rod", ("LLI",), "LLO"), ("rod", ("ULI",), "ULO")],
        "damper": ("DT", "DB", "upright"), "toe": None,
        "draw": [("PF", "WC", "upright"), ("LLO", "ULO", "upright"), ("PF", "LLO", "upright"), ("LLO", "WC", "upright")],
    },
    "solid_axle_panhard": {
        "name": "Solid axle: 4-link + Panhard rod",
        "about": "A rigid axle carrying both wheels, located fore and aft by two trailing links per side and sideways by a Panhard rod. Camber stays constant in heave; the roll centre is the Panhard rod's height at the centre line, and the axle moves sideways a little as the rod swings.",
        "axle": True, "single": ["PHC", "PHA"],
        "links": [("rod", ("LTI",), "LTO"), ("rod", ("UTI",), "UTO"), ("rod", ("PHC",), "PHA")],
        "damper": ("DT", "DB", "upright"), "toe": None, "rc": ("line", "PHC", "PHA"),
        "draw": [("LTO", "UTO", "upright"), ("LTO", "WC", "upright"), ("UTO", "WC", "upright"), ("WC", "~WC", "upright")],
    },
    "solid_axle_watt": {
        "name": "Solid axle: 4-link + Watt's linkage",
        "about": "A rigid axle on two trailing links per side, located sideways by a Watt's linkage: its centre pivot moves on an almost straight vertical line, so the axle does not shift sideways. The roll centre is the centre pivot's height.",
        "axle": True, "single": ["WP", "WL", "WR"],
        "links": [("rod", ("LTI",), "LTO"), ("rod", ("UTI",), "UTO"), ("watt", ("WL", "WR"), "WP")],
        "damper": ("DT", "DB", "upright"), "toe": None, "rc": ("point", "WP"),
        "draw": [("LTO", "UTO", "upright"), ("LTO", "WC", "upright"), ("UTO", "WC", "upright"), ("WC", "~WC", "upright")],
    },
    "solid_axle_triangulated": {
        "name": "Solid axle: triangulated 4-link",
        "about": "Two lower trailing links and two upper links angled towards the centre: the upper triangle locates the axle sideways, with no Panhard rod. The roll centre is about where the upper links meet.",
        "axle": True, "roll_lock": True,
        "links": [("rod", ("LTI",), "LTO"), ("rod", ("UTI",), "UTO")],
        "damper": ("DT", "DB", "upright"), "toe": None, "rc": ("line", "UTI", "UTO"),
        "draw": [("LTO", "UTO", "upright"), ("LTO", "WC", "upright"), ("UTO", "WC", "upright"), ("WC", "~WC", "upright")],
    },
    "de_dion_watt": {
        "name": "De Dion: trailing links + Watt's linkage",
        "about": "A rigid de Dion tube carrying both hubs, like a solid axle, but the differential is on the chassis (sprung) and drives the wheels through halfshafts, so the unsprung mass is lower. Two trailing links per side and a Watt's linkage, as the Alfa Romeo 75. The halfshaft angle and plunge are analysed from the chassis differential (DIFF).",
        "axle": True, "single": ["WP", "WL", "WR"], "halfshafts": True,
        "links": [("rod", ("LTI",), "LTO"), ("rod", ("UTI",), "UTO"), ("watt", ("WL", "WR"), "WP")],
        "damper": ("DT", "DB", "upright"), "toe": None, "rc": ("point", "WP"),
        "draw": [("LTO", "UTO", "upright"), ("LTO", "WC", "upright"), ("UTO", "WC", "upright"), ("WC", "~WC", "upright")],
    },
    "de_dion_panhard": {
        "name": "De Dion: trailing links + Panhard rod",
        "about": "A rigid de Dion tube with a chassis-mounted differential and halfshafts, located by two trailing links per side and a Panhard rod. The roll centre is the Panhard rod's height at the centre line; the tube moves sideways a little as the rod swings, which the halfshafts' plunge must absorb.",
        "axle": True, "single": ["PHC", "PHA"], "halfshafts": True,
        "links": [("rod", ("LTI",), "LTO"), ("rod", ("UTI",), "UTO"), ("rod", ("PHC",), "PHA")],
        "damper": ("DT", "DB", "upright"), "toe": None, "rc": ("line", "PHC", "PHA"),
        "draw": [("LTO", "UTO", "upright"), ("LTO", "WC", "upright"), ("UTO", "WC", "upright"), ("WC", "~WC", "upright")],
    },
    "twist_beam": {
        "name": "Twist beam (heave)",
        "about": "Two trailing arms joined by a beam that twists in roll. Heave is analysed (both arms turn about the line through the two bushes); the roll stiffness of the beam is not modelled.",
        "links": [("pivot_mirror", ("BSH",), None)],
        "damper": ("DT", "DB", "upright"), "toe": None,
        "draw": [("BSH", "WC", "upright")],
    },
}


def _points_of(t):
    """(chassis points, upright points) of a TOPOLOGY entry, in link order, damper included."""
    inner, outer = [], []
    for kind, ins, out in t["links"]:
        inner += [p for p in ins if p not in inner]
        if out and out not in outer:
            outer.append(out)
    top, bottom, carrier = t["damper"]
    if top not in inner:
        inner.append(top)
    if bottom not in outer and bottom not in inner:
        outer.append(bottom)
    return inner, outer


# derived per layout: chassis pickups (INNER, the points on the chassis), points
# that move with the upright (MOVING, WC always included), all required points (REQUIRED, plus
# DIFF for a de Dion)
INNER, MOVING, REQUIRED = {}, {}, {}
for _k, _t in TOPOLOGY.items():
    _i, _o = _points_of(_t)
    INNER[_k] = _i
    MOVING[_k] = _o + (["WC"] if "WC" not in _o else [])
    REQUIRED[_k] = _i + MOVING[_k] + (["DIFF"] if _t.get("halfshafts") else [])


def describe():
    """Layout catalogue for the editor: names, help text, pickups, moving points, lines to draw.

    Returns {layout: {"name", "about", "inner", "moving", "required", "lines", "steered", "axle",
    "single"}}. Each line is [a, b, style] or [a, b, style, once]: style "arm", "tie" (the toe
    link) or "strut" for links and the damper, "upright" for the draw lines. once is True when
    the viewer draws the line a single time as given instead of on both sides: a single link
    (Panhard rod, Watt's linkage) or a draw line to a mirrored "~" point (the axle tube).
    """
    out = {}
    for k, t in TOPOLOGY.items():
        lines = []
        single = set(t.get("single", []))
        for kind, ins, o in t["links"]:
            if kind in ("arm", "rod", "watt"):
                lines += [[i, o, "tie" if o == t["toe"] else "arm", i in single and o in single] for i in ins]
            elif kind == "strut":
                lines.append([ins[0], o, "strut"])
        top, bottom, _ = t["damper"]
        if not any(l[0] == top and l[1] == bottom for l in lines):
            lines.append([top, bottom, "strut"])
        lines += [list(d) + [d[1].startswith("~")] for d in t["draw"]]
        out[k] = {"name": t["name"], "about": t["about"], "inner": INNER[k], "moving": MOVING[k],
                  "required": REQUIRED[k], "lines": lines, "steered": t["toe"] is not None,
                  "axle": bool(t.get("axle")), "single": t.get("single", [])}
    return out


# ---------------------------------------------------------------- corner model

class Corner:
    """One corner (left side). Pose of the upright: p' = WC0 + t + R(w) (p - WC0).

    spec: the model's corner dict ("type", optional "tyre_radius" m, "spring_N_per_mm",
    "damper_stroke_mm"). points: {name: [x, y, z]} resolved static points in metres (left side).
    rack: steering rack offset, m along X, applied to the inner tie-rod point TRI.
    Each link becomes rows of self.cons, (kind, chassis point, upright static point, length):
      "rod"    |q - c| = L                        (1 row)
      "lat"    q_x = static x                     (1 row, Watt's centre pivot)
      "fix"    q = c                              (3 rows, a point that cannot move)
      "strut"  c on the line through q along R(w)(c - u)   (3 rows, rank 2)
    """

    def __init__(self, spec, points, rack=0.0):
        self.type = spec["type"]
        if self.type not in TOPOLOGY:
            raise ValueError(f"unknown suspension layout {self.type!r}")
        self.t = TOPOLOGY[self.type]
        self.p = dict(points)                          # resolved static points
        self.r_tyre = spec.get("tyre_radius", DEFAULT_TYRE["radius"])
        self.k_spring = spec.get("spring_N_per_mm", 50.0)            # default spring rate when none is set
        self.stroke = spec.get("damper_stroke_mm", 160.0) / 1000     # default damper stroke, m
        self.diff = points.get("DIFF")
        p = self.p
        self.wc0 = p["WC"]
        self.spin0 = [1.0, 0.0, 0.0]                   # static spin axis: zero camber and toe
        self.cp0 = [p["WC"][0], p["WC"][1], p["WC"][2] - self.r_tyre]   # contact point, fixed in the upright
        self.axle = bool(self.t.get("axle"))
        single = set(self.t.get("single", []))
        self.cons = []                                 # (kind, chassis point, upright static point, length)
        for kind, ins, out in self.t["links"]:
            if kind in ("arm", "rod"):
                for i in ins:
                    ci = list(p[i])
                    if i == "TRI" and rack:
                        ci = [ci[0] + rack, ci[1], ci[2]]
                    self.cons.append(("rod", ci, p[out], norm(sub(p[out], p[i]))))
                    if self.axle and not (i in single and out in single):     # the same link on the other side
                        self.cons.append(("rod", mirror(ci), mirror(p[out]), norm(sub(p[out], p[i]))))
            elif kind == "watt":                       # straight-line linkage: the centre pivot keeps its X
                self.cons.append(("lat", None, p[out], None))
            elif kind == "strut":
                self.cons.append(("strut", p[ins[0]], p[out], None))
            elif kind == "ball":                       # the upright point at the bush stays put
                self.cons.append(("fix", p[ins[0]], p[ins[0]], None))
            elif kind == "pivot":                      # two fixed points on the axis: only rotation about it is left
                self.cons.append(("fix", p[ins[0]], p[ins[0]], None))
                self.cons.append(("fix", p[ins[1]], p[ins[1]], None))
            elif kind == "pivot_mirror":
                self.cons.append(("fix", p[ins[0]], p[ins[0]], None))
                self.cons.append(("fix", mirror(p[ins[0]]), mirror(p[ins[0]]), None))

    def move(self, x, q):
        """Where the upright's static point q goes at pose x = (t, w)."""
        t, w = x
        return add(add(self.wc0, t), rotvec(w, sub(q, self.wc0)))

    def residual(self, x, z_target):
        """Constraint errors at pose x (m), all zero when the corner assembles with the wheel
        centre at height z_target (for an axle, the mean of both wheel centres)."""
        r = []
        for kind, c, u, L in self.cons:
            q = self.move(x, u)
            if kind == "rod":
                r.append(norm(sub(q, c)) - L)
            elif kind == "lat":
                r.append(q[0] - u[0])
            elif kind == "fix":
                r += sub(q, c)
            else:                                      # strut: axis through ST; v minus its part along d
                d = unit(rotvec(x[1], sub(c, u)))
                v = sub(c, q)
                r += sub(v, mul(d, dot(v, d)))
        if self.axle:                                  # both wheels driven together (heave)
            zl, zr = self.move(x, self.wc0)[2], self.move(x, mirror(self.wc0))[2]
            r.append((zl + zr) / 2 - z_target)
            if self.t.get("roll_lock"):                # a layout that is free in roll: hold it level
                r.append(zl - zr)
        else:
            r.append(self.move(x, self.wc0)[2] - z_target)
        return r

    def solve(self, z_target, x0):
        """Pose (t, w) with the wheel centre at height z_target (m), starting from pose x0.

        Damped Gauss-Newton (Levenberg) on the residual: (J^T J + lam I) dx = -J^T r, with a
        forward-difference Jacobian. The rotation columns perturb w on the left (compose(dw, w)),
        matching the update w <- compose(dw, w). Raises ValueError if it does not converge.
        """
        t, w = x0[0][:], x0[1][:]
        lam = 1e-9                                     # tiny damping: keeps J^T J invertible when rows are redundant
        for _ in range(60):
            r = self.residual((t, w), z_target)
            err = max(abs(v) for v in r)
            if err < 1e-10:                            # m: far below anything the editor shows
                return (t, w)
            h = 1e-7                                   # finite-difference step, m and rad
            J = []
            for i in range(6):
                if i < 3:
                    tt = t[:]
                    tt[i] += h
                    ri = self.residual((tt, w), z_target)
                else:
                    dw = [0.0, 0.0, 0.0]
                    dw[i - 3] = h
                    ri = self.residual((t, compose(dw, w)), z_target)
                J.append([(a - b) / h for a, b in zip(ri, r)])        # column i
            JTJ = [[sum(J[i][k] * J[j][k] for k in range(len(r))) + (lam if i == j else 0.0) for j in range(6)]
                   for i in range(6)]
            JTr = [sum(J[i][k] * r[k] for k in range(len(r))) for i in range(6)]
            dx = _solve(JTJ, [-v for v in JTr])
            step = max(abs(v) for v in dx)
            if step > 0.05:                            # trust limit: at most 50 mm / 0.05 rad per iteration
                dx = [v * 0.05 / step for v in dx]
            t = add(t, dx[:3])
            w = compose(dx[3:], w)
        raise ValueError("suspension does not assemble at this travel (check pickup points)")

    def state(self, x):
        """Positions and wheel angles at pose x.

        Returns {point: [x, y, z]} for every moving point, plus "spin" (unit wheel axis,
        outboard), "CP" (lowest point of the tyre circle), "CPu" (static contact point carried
        rigidly by the upright, used for the kinematic roll centre), "camber" and "toe" (deg)
        and "damper" (damper length, m). Camber is negative with the top leaning inboard; toe
        is positive toe-in (front of the wheel towards the centre line).
        """
        p = self.p
        q = {k: self.move(x, p[k]) for k in MOVING[self.type] if k in p}
        q["WC"] = self.move(x, self.wc0)
        a = unit(rotvec(x[1], self.spin0))
        q["spin"] = a
        down = unit(sub([0, 0, -1], mul(a, -a[2])))    # -Z projected into the wheel plane
        q["CP"] = add(q["WC"], mul(down, self.r_tyre))
        q["CPu"] = self.move(x, self.cp0)               # contact point carried by the upright
        q["camber"] = -math.degrees(math.atan2(a[2], math.hypot(a[0], a[1])))
        q["toe"] = -math.degrees(math.atan2(a[1], a[0]))
        top, bottom, carrier = self.t["damper"]
        if carrier == "upright":
            db = q.get(bottom) or self.move(x, p[bottom])
        else:                                          # carried by an arm: turn it with the arm
            kind, ins, out = self.t["links"][carrier]
            db = _arm_point(p[ins[0]], p[ins[1]], p[out], q[out], p[bottom])
        q["DB" if bottom == "DB" else bottom] = db
        q["damper"] = norm(sub(p[top], db))
        return q

    def toe_with_rack(self, d):
        """Toe at static wheel height with the inner tie-rod point moved d along X (steering)."""
        c = Corner({"type": self.type, "tyre_radius": self.r_tyre}, self.p, rack=d)
        return c.state(c.solve(self.wc0[2], ([0.0] * 3, [0.0] * 3)))["toe"]

    def steer_axis(self):
        """Instantaneous axis the upright turns about when the rack moves: (point, unit direction).

        Solves the pose at static height with the rack moved 0.1 mm; that small motion (t, w) is a
        screw about the axis through a = WC0 + (w x t) / |w|^2, direction w (turned to point up).
        Returns None for a layout without steering or when the rack does not turn the upright.
        """
        if not self.t["toe"]:
            return None
        h = 1e-4
        c = Corner({"type": self.type, "tyre_radius": self.r_tyre}, self.p, rack=h)
        try:
            t, w = c.solve(self.wc0[2], ([0.0] * 3, [0.0] * 3))
        except ValueError:                             # the rack cannot move it (an over-constrained corner)
            return None
        if norm(w) < 1e-12:
            return None
        d = unit(w)
        if d[2] < 0:
            d = mul(d, -1)
        pt = add(self.wc0, mul(cross(w, t), 1 / dot(w, w)))
        return pt, d


def _arm_point(i1, i2, o0, o1, pt):
    """Where pt (fixed to an arm turning about i1-i2) goes when the arm's outer point moves o0 -> o1.

    The arm angle is the signed angle between o0 and o1 projected onto the plane normal to the axis.
    """
    a = unit(sub(i2, i1))
    u0, u1 = sub(o0, i1), sub(o1, i1)
    u0, u1 = sub(u0, mul(a, dot(u0, a))), sub(u1, mul(a, dot(u1, a)))
    ang = math.atan2(dot(a, cross(u0, u1)), dot(u0, u1))
    return add(i1, rot(a, ang, sub(pt, i1)))


# ---------------------------------------------------------------- analysis

def analyse(spec, points, travel_mm=120, step_mm=2.5):
    """Kinematic sweep of one corner from -travel_mm (rebound) to +travel_mm (bump), step_mm apart.

    spec and points as for Corner. The sweep stops early in a direction where the corner no
    longer assembles. Returns:
      "static"  camber_deg, toe_deg, track_mm, wheel_centre_mm, kpi_deg, caster_deg,
                scrub_radius_mm, trail_mm (None when the layout does not steer), bump_mm,
                rebound_mm, usable_travel_mm (damper within +-stroke/2),
                damper_static_mm, motion_ratio, roll_centre_mm, wheel_rate_N_per_mm
      "curves"  lists over travel_mm: camber_deg, toe_deg, track_change_mm, roll_centre_mm,
                motion_ratio, wheel_rate_N_per_mm, wc (wheel centre, m); for an axle also
                axle_shift_mm; with a DIFF point also halfshaft_deg and plunge_mm
      "frames"  [{"travel_mm", "pts": {point: [x, y, z]}, "spin"}] for the 3D view
    Raises ValueError if the corner does not assemble at static height.
    """
    c = Corner(spec, points)
    x0 = c.solve(c.wc0[2], ([0.0] * 3, [0.0] * 3))
    q0 = c.state(x0)
    z0 = q0["WC"][2]
    sweeps = {}
    for sign in (1, -1):
        x, s, pts = x0, 0.0, []
        while True:
            s += sign * step_mm
            if abs(s) > travel_mm + 1e-9:
                break
            try:
                x = c.solve(z0 + s / 1000, x)
            except ValueError:
                break
            pts.append((s, c.state(x)))
        sweeps[sign] = pts
    seq = list(reversed(sweeps[-1])) + [(0.0, q0)] + sweeps[1]
    travel = [s for s, _ in seq]
    dmp = [q["damper"] for _, q in seq]

    def deriv(vals, i):                                # d(vals)/dz per metre of travel, central difference
        j0, j1 = max(i - 1, 0), min(i + 1, len(seq) - 1)
        dz = (travel[j1] - travel[j0]) / 1000
        return (vals[j1] - vals[j0]) / dz if dz else 0.0

    mr = [abs(deriv(dmp, i)) for i in range(len(seq))]          # motion ratio = |d damper / d wheel|
    cpx = [q["CPu"][0] for _, q in seq]
    cpz = [q["CPu"][2] for _, q in seq]
    rc = []
    for i, (_, q) in enumerate(seq):
        # kinematic RC: the line through the contact patch normal to its path (dx, dz) meets the
        # centre line X = 0 at h = z_cp + (dx/dz) x_cp
        dxdz = deriv(cpx, i) / (deriv(cpz, i) or 1e-12)
        rc.append((q["CPu"][2] + dxdz * q["CPu"][0]) * 1000)
    if c.t.get("rc"):                                  # solid axle: the height of its lateral locator
        # ("point", P): P's height; ("line", A, B): height of line A-B where it crosses X = 0
        how = c.t["rc"]
        rc = []
        for _, q in seq:
            if how[0] == "point":
                rc.append(q[how[1]][2] * 1000)
            else:
                a, b = c.p[how[1]], q[how[2]]
                f = (0.0 - a[0]) / (b[0] - a[0]) if abs(b[0] - a[0]) > 1e-9 else 0.0
                rc.append((a[2] + f * (b[2] - a[2])) * 1000)
    curves = {"travel_mm": travel,
              "camber_deg": [q["camber"] for _, q in seq],
              "toe_deg": [q["toe"] for _, q in seq],
              "track_change_mm": [2 * (q["CP"][0] - q0["CP"][0]) * 1000 for _, q in seq],
              "roll_centre_mm": rc, "motion_ratio": mr,
              "wheel_rate_N_per_mm": [c.k_spring * m * m for m in mr],      # k_wheel = k_spring * MR^2
              "wc": [[round(v, 6) for v in q["WC"]] for _, q in seq]}
    if c.axle:                                         # a rigid axle keeps its track; it can shift sideways
        curves["track_change_mm"] = [0.0 for _ in seq]
        curves["axle_shift_mm"] = [(q["CPu"][0] - q0["CPu"][0]) * 1000 for _, q in seq]
    if c.diff:                                         # halfshaft DIFF -> WC: angle from the X axis, length change
        ang, plunge = [], []
        L0 = norm(sub(q0["WC"], c.diff))
        for _, q in seq:
            v = sub(q["WC"], c.diff)
            ang.append(math.degrees(math.atan2(math.hypot(v[1], v[2]), abs(v[0]))))
            plunge.append((norm(v) - L0) * 1000)
        curves["halfshaft_deg"], curves["plunge_mm"] = ang, plunge
    # usable travel: damper within +-stroke/2 of its static length
    d0 = q0["damper"]
    ok = [t for t, d in zip(travel, dmp) if abs(d - d0) <= c.stroke / 2 + 1e-9]
    usable = (min(ok), max(ok)) if ok else (0.0, 0.0)
    i0 = travel.index(0.0)
    static = _static_geometry(c, q0)
    static.update({"bump_mm": usable[1], "rebound_mm": -usable[0], "usable_travel_mm": usable[1] - usable[0],
                   "damper_static_mm": d0 * 1000, "motion_ratio": mr[i0], "roll_centre_mm": rc[i0]})
    static["wheel_rate_N_per_mm"] = c.k_spring * static["motion_ratio"] ** 2 if static["motion_ratio"] else None
    frames = []
    keys = [k for k in MOVING[c.type]] + ["CP"]
    for s, q in seq:                                   # every 2.5 mm step, so the 3D view moves smoothly
        frames.append({"travel_mm": s, "pts": {k: q[k] for k in keys if k in q}, "spin": q["spin"]})
    return {"static": static, "curves": curves, "frames": frames}


def _static_geometry(c, q):
    """Static camber, toe, track, wheel-centre height and steering-axis values (deg, mm).

    Scrub radius and trail are measured where the steering axis meets the plane Z = 0, taken as
    the ground; the contact point itself is at WC_z - tyre radius (a few mm off Z = 0 when the
    tyre radius and the wheel-centre height differ), so the two differ slightly.
    """
    cp = q["CP"]
    out = {"camber_deg": q["camber"], "toe_deg": q["toe"], "track_mm": 2 * cp[0] * 1000,
           "wheel_centre_mm": q["WC"][2] * 1000, "kpi_deg": None, "caster_deg": None,
           "scrub_radius_mm": None, "trail_mm": None}
    ax = c.steer_axis()
    if ax:
        a, s = ax
        out["kpi_deg"] = math.degrees(math.atan2(-s[0], s[2]))       # top leaning inboard is positive (left corner)
        out["caster_deg"] = math.degrees(math.atan2(s[1], s[2]))      # top rearward is positive
        g = add(a, mul(s, (cp[2] - a[2]) / s[2]))                      # axis meets the ground at the contact point
        out["scrub_radius_mm"] = (cp[0] - g[0]) * 1000                 # positive: axis meets the ground inboard
        out["trail_mm"] = (cp[1] - g[1]) * 1000                        # positive: axis meets the ground ahead
    return out


# ---------------------------------------------------------------- starting geometry

# Offsets (dx, dy, dz) from the wheel centre (left corner; -Y is forward), metres. A sensible start
# when a corner is switched to another layout. Every layout needs every REQUIRED point except WC (which is the corner's own). An
# optional fourth entry places the point off the left side: "R" = mirrored to the right side
# (x = -(wc_x + dx)), "C" = on the centre line (x = 0). Generic values for a compact-car corner;
# a new layout's defaults should assemble over the full travel (the tests sweep every layout).
_DEFAULTS = {
    "macpherson": {"LIF": (-0.34, -0.40, -0.08), "LIR": (-0.34, 0.06, -0.07), "LBJ": (0.02, 0.0, -0.08),
                   "ST": (-0.05, 0.02, 0.41), "SB": (-0.02, 0.01, 0.21),
                   "TRI": (-0.43, 0.16, 0.01), "TRO": (-0.01, 0.13, 0.02)},
    "double_wishbone": {"LIF": (-0.38, -0.15, -0.07), "LIR": (-0.38, 0.20, -0.07), "LBJ": (-0.02, 0.0, -0.09),
                        "UIF": (-0.30, -0.12, 0.16), "UIR": (-0.30, 0.15, 0.16), "UBJ": (-0.08, 0.0, 0.16),
                        "TRI": (-0.32, 0.15, 0.0), "TRO": (-0.04, 0.14, -0.01),
                        "DT": (-0.20, 0.05, 0.40), "DB": (-0.12, 0.05, -0.08)},
    "multilink": {"UFI": (-0.30, -0.20, 0.18), "UFO": (-0.08, -0.02, 0.16), "URI": (-0.30, 0.15, 0.17),
                  "URO": (-0.08, 0.04, 0.16), "LFI": (-0.38, -0.25, -0.07), "LFO": (-0.05, -0.02, -0.09),
                  "LRI": (-0.38, 0.10, -0.07), "LRO": (-0.05, 0.02, -0.09),
                  "TRI": (-0.33, 0.23, 0.01), "TRO": (-0.04, 0.20, -0.01),
                  "DT": (-0.20, 0.05, 0.40), "DB": (-0.10, 0.03, -0.06)},
    "trailing_arm": {"PF": (-0.40, -0.42, 0.03), "PR": (-0.18, -0.42, 0.03),
                     "DT": (-0.15, -0.05, 0.38), "DB": (-0.12, -0.05, -0.05)},
    "semi_trailing_arm": {"PF": (-0.55, -0.33, 0.03), "PR": (-0.22, -0.42, 0.03),
                          "DT": (-0.15, -0.05, 0.38), "DB": (-0.12, -0.05, -0.05)},
    "trailing_arm_lateral": {"PF": (-0.26, -0.66, -0.05), "LLI": (-0.36, 0.05, -0.08), "LLO": (-0.02, 0.03, -0.09),
                             "ULI": (-0.25, -0.09, 0.145), "ULO": (-0.08, -0.09, 0.15),
                             "DT": (-0.19, 0.09, 0.39), "DB": (-0.11, 0.04, -0.09)},
    "twist_beam": {"BSH": (-0.12, -0.45, 0.03), "DT": (-0.15, -0.05, 0.38), "DB": (-0.12, -0.05, -0.05)},
    # solid axles; "R" = on the other side (x mirrored), "C" = on the centre line
    "solid_axle_panhard": {"LTI": (-0.10, -0.55, -0.07), "LTO": (-0.10, 0.0, -0.13), "UTI": (-0.15, -0.45, 0.14),
                           "UTO": (-0.15, 0.0, 0.13), "PHC": (-0.10, 0.12, 0.03), "PHA": (-0.10, 0.12, -0.01, "R"),
                           "DT": (-0.12, 0.06, 0.38), "DB": (-0.12, 0.06, -0.06)},
    "solid_axle_watt": {"LTI": (-0.10, -0.55, -0.07), "LTO": (-0.10, 0.0, -0.13), "UTI": (-0.15, -0.45, 0.14),
                        "UTO": (-0.15, 0.0, 0.13), "WP": (0.0, 0.15, 0.0, "C"), "WL": (-0.38, 0.15, 0.10),
                        "WR": (-0.38, 0.15, -0.10, "R"), "DT": (-0.12, 0.06, 0.38), "DB": (-0.12, 0.06, -0.06)},
    "de_dion_watt": {"LTI": (-0.10, -0.55, -0.07), "LTO": (-0.10, 0.0, -0.13), "UTI": (-0.15, -0.45, 0.14),
                     "UTO": (-0.15, 0.0, 0.13), "WP": (0.0, 0.15, 0.0, "C"), "WL": (-0.38, 0.15, 0.10),
                     "WR": (-0.38, 0.15, -0.10, "R"), "DT": (-0.12, 0.06, 0.38), "DB": (-0.12, 0.06, -0.06),
                     "DIFF": (-0.53, 0.0, 0.03)},
    "de_dion_panhard": {"LTI": (-0.10, -0.55, -0.07), "LTO": (-0.10, 0.0, -0.13), "UTI": (-0.15, -0.45, 0.14),
                        "UTO": (-0.15, 0.0, 0.13), "PHC": (-0.10, 0.12, 0.03), "PHA": (-0.10, 0.12, -0.01, "R"),
                        "DT": (-0.12, 0.06, 0.38), "DB": (-0.12, 0.06, -0.06), "DIFF": (-0.53, 0.0, 0.03)},
    "solid_axle_triangulated": {"LTI": (-0.10, -0.55, -0.07), "LTO": (-0.10, 0.0, -0.13),
                                "UTI": (-0.56, -0.40, 0.14), "UTO": (-0.40, 0.0, 0.13),
                                "DT": (-0.12, 0.06, 0.38), "DB": (-0.12, 0.06, -0.06)},
}


def default_points(layout, wc, corner_name="rear"):
    """Starting points {name: [x, y, z]} (m, rounded to 0.1 mm) for layout around wheel centre wc.

    corner_name is accepted for the editor's call but not used: front and rear get the same offsets.
    """
    def place(d):
        x = wc[0] + d[0]
        if len(d) > 3:
            x = -x if d[3] == "R" else 0.0
        return [round(x, 4), round(wc[1] + d[1], 4), round(wc[2] + d[2], 4)]
    pts = {k: place(d) for k, d in _DEFAULTS[layout].items()}
    pts["WC"] = list(wc)
    return pts


# ---------------------------------------------------------------- SVJ corners

def _register(key, t):
    """Add a layout to TOPOLOGY (and INNER, MOVING, REQUIRED), replacing one of the same key."""
    TOPOLOGY[key] = t
    i, o = _points_of(t)
    INNER[key] = i
    MOVING[key] = o + (["WC"] if "WC" not in o else [])
    REQUIRED[key] = i + MOVING[key]


def _nice(name):
    return name.replace("_", " ").capitalize()


def svj_corner(corner, key, override=None):
    """A solver corner built from an SVJ corner's own links (left side, BeamNG axes, front axle at
    y 0, ground at z 0). The layout is registered in TOPOLOGY as `key`.

    SVJ link -> constraint: an arm with two inner points and an outer hardpoint -> "arm"; a rod (one
    inner point) -> "rod"; a strut -> "strut" (its lower point from strut_lower, the damper's outer
    mount, or 40 % up from the lower ball joint); an arm ending at the wheel centre (trailing,
    semi-trailing, twist-beam arm) -> "pivot" about its two inner points. Links that do not locate
    the wheel (push / pull rods, rockers, anti-roll bars) are left out. The steering or toe link is
    TRI -> TRO. A solid axle or de Dion is one body carrying both hubs; Panhard and Watt links are
    used once. Returns (spec, points, labels, notes); spec is None when the corner cannot be built.

    override: {SVJ hardpoint name (as svj.hardpoints names them): [x, y, z]} in the same frame, to
    solve the SVJ's layout on other points (the base vehicle's nodes tied to those hardpoints).
    Points without an override (an assumed strut lower point, a damper top) move with the wheel
    centre, so the corner stays together.
    """
    import re
    from beamforge.svj import from_sae
    notes = []
    topo = corner.get("topology") or {}
    hps = (topo.get("upright") or {}).get("hardpoints") or {}
    if "wheel_center" not in hps:
        return None, None, None, ["no wheel_center hardpoint"]
    flip = from_sae(hps["wheel_center"])[0] < 0           # a right corner: mirror it to the left
    B = (lambda p: mirror(from_sae(p))) if flip else (lambda p: from_sae(p))  # noqa: E731
    pts, labels = {"WC": B(hps["wheel_center"])}, {"WC": "Wheel centre"}
    src = {"WC": "wheel_center"}                            # point key -> SVJ hardpoint name, for override
    if pts["WC"][2] <= 0.05:
        notes.append(f"the wheel centre is {pts['WC'][2] * 1000:.0f} mm above the ground: the file's Z axis may point up "
                     "(SAE J670 has Z down, origin on the ground); angles will be wrong")
    axle = topo.get("system_type") in ("solid_axle", "de_dion")
    links, single, toe, count = [], [], None, 0
    damper = corner.get("damper") or {}
    db = damper.get("outboard_mount") or hps.get("damper_outboard")
    for link in topo.get("links") or []:
        name, kind = str(link.get("name", "link")), link.get("type")
        if re.fullmatch(r"(macpherson_)?strut", name):         # a strut by name (converters write it as a rod)
            kind = "strut"
        ins = [p for p in link.get("inboard_points") or [] if isinstance(p, list) and len(p) == 3]
        ref = str(link.get("outboard_ref") or "").split(".")[-1]
        if re.search(r"push|pull|rocker|anti_roll|arb|sway", name):
            notes.append(f"{_nice(name)}: does not locate the wheel, left out")
            continue
        if not ins or (ref and ref not in hps):
            notes.append(f"{_nice(name)}: inner points or outer hardpoint missing, left out")
            continue
        steer = bool(re.search(r"tie|toe|steer", name)) and len(ins) == 1 and kind != "strut"
        keys = []
        for i, p in enumerate(ins):
            k = "TRI" if steer else "ST" if kind == "strut" else f"{name}.{i}"
            pts[k] = B(p)
            src[k] = f"{name}.{i}"
            labels[k] = f"{_nice(name)}, inner" + (f" {i + 1}" if len(ins) > 1 else "")
            keys.append(k)
        once = axle and bool(re.search(r"panhard|watt", name))
        if kind == "strut":
            sb = hps.get("strut_lower") or db
            if sb:
                pts["SB"] = B(sb)
                if hps.get("strut_lower"):
                    src["SB"] = "strut_lower"
                elif sb is hps.get("damper_outboard"):
                    src["SB"] = "damper_outboard"
            elif "lower_ball_joint" in hps:
                lbj, st = B(hps["lower_ball_joint"]), pts["ST"]
                pts["SB"] = [lbj[i] + (st[i] - lbj[i]) * 0.4 for i in range(3)]
                notes.append("no strut lower point: the strut axis is taken through the lower ball joint")
            else:
                notes.append(f"{_nice(name)}: no strut lower point, left out")
                continue
            labels["ST"], labels["SB"] = "Strut top mount", "Strut lower mount (upright)"
            links.append(("strut", ("ST",), "SB"))
            count += 2
        elif ref == "wheel_center" and len(keys) >= 2 and not axle:
            a, b = pts[keys[0]], pts[keys[1]]
            d = [b[i] - a[i] for i in range(3)]
            if abs(d[2]) > 0.95 * math.sqrt(sum(x * x for x in d)):
                notes.append(f"{_nice(name)}: its pivot axis is vertical, so it cannot move the wheel up and down")
            links.append(("pivot", tuple(keys[:2]), None))
            count += 5
        else:
            out = "TRO" if steer else ref
            pts[out] = B(hps[ref])
            src[out] = ref
            labels.setdefault(out, _nice(ref))
            if steer:
                toe = out
            sides = 1 if once or not axle else 2
            if len(keys) >= 2:
                links.append(("arm", tuple(keys[:2]), out))
                count += 2 * sides
            else:
                links.append(("rod", (keys[0],), out))
                count += sides
            if once:
                single += [keys[0], out]
    if not links:
        return None, None, None, notes + ["no link that locates the wheel"]
    if "ST" in pts and "SB" in pts:
        dmp = ("ST", "SB", "upright")
    else:
        pts["DB"] = B(db) if db else list(pts["WC"])
        if db is not None and db is hps.get("damper_outboard"):
            src["DB"] = "damper_outboard"
        if damper.get("inboard_mount"):
            pts["DT"] = B(damper["inboard_mount"])
        else:
            pts["DT"] = [pts["DB"][0], pts["DB"][1], pts["DB"][2] + 0.35]
            notes.append("no damper inboard mount: placed 350 mm above its lower mount")
        labels.update({"DT": "Damper top", "DB": "Damper bottom"})
        dmp = ("DT", "DB", "upright")
    if override:
        shift = [override[src["WC"]][i] - pts["WC"][i] for i in range(3)] if src["WC"] in override else [0.0] * 3
        for k in pts:
            h = src.get(k)
            pts[k] = list(override[h]) if h in override else [pts[k][i] + shift[i] for i in range(3)]
    if count != 5:
        notes.append(f"the links give {count} constraints where a corner needs 5: "
                     + ("it cannot move (over-constrained)" if count > 5 else "it is not located (under-constrained)"))
    outs = [o for _, _, o in links if o] + ["WC"]
    draw = [(a, b, "upright") for a, b in zip(outs, outs[1:]) if a != b]
    draw += [(k, "WC", "upright") for kind, ins, _ in links if kind == "pivot" for k in ins]
    if axle:
        draw.append(("WC", "~WC", "upright"))
    t = {"name": _nice(str(topo.get("system_type") or "corner")), "about": "Read from the SVJ file.",
         "links": links, "damper": dmp, "toe": toe, "draw": draw}
    if axle:
        t.update(axle=True, single=single)
    _register(key, t)
    spring = corner.get("spring") or {}
    mr = spring.get("motion_ratio") or damper.get("motion_ratio") or 1.0
    spec = {"type": key, "tyre_radius": pts["WC"][2] if pts["WC"][2] > 0.1 else DEFAULT_TYRE["radius"],
            "spring_N_per_mm": round(spring.get("rate", 50000) / (mr * mr) / 1000, 2),
            "damper_stroke_mm": round((damper.get("stroke") or 0.16) * 1000)}
    return spec, pts, labels, notes


def study_svj(svj, travel_mm=100, overrides=None, suffix=""):
    """Kinematics of an SVJ's front and rear corners (FL and RL, else FR and RR mirrored).

    Returns {"corners": {"front" | "rear": {"corner", "type", "name", "points", "labels", "lines",
    "single", "static", "curves", "frames", "notes"}}, "notes": [...]}; lines as describe(). Points
    are in BeamNG axes with the front axle at y 0 and the ground at z 0 (the editor places them).
    overrides: {corner ("FL"...): svj_corner override}: the SVJ layout solved on other points.
    """
    out, notes = {}, []
    susp = svj.get("suspension") or {}
    for axle, (L, R) in (("front", ("FL", "FR")), ("rear", ("RL", "RR"))):
        name = L if isinstance(susp.get(L), dict) else R if isinstance(susp.get(R), dict) else None
        if not name:
            continue
        key = f"svj_{axle}{suffix}"
        if overrides is not None and name not in overrides:
            continue
        spec, pts, labels, n = svj_corner(susp[name], key, (overrides or {}).get(name))
        if spec is None:
            notes += [f"{name}: {x}" for x in n]
            continue
        try:
            res = analyse(spec, pts, travel_mm=travel_mm)
        except ValueError as exc:
            notes += [f"{name}: {x}" for x in n] + [f"{name}: {exc}"]
            continue
        from . import dampers
        rate = (susp[name].get("spring") or {}).get("rate")
        mr0 = res["static"].get("motion_ratio")
        if dampers.at_wheel(susp[name]) and isinstance(rate, (int, float)) and mr0 and mr0 > 0.05:
            # Assetto Corsa: the rate is at the wheel whatever the geometry, so the coil on this corner's own damper is
            # rate / MR^2 (the file's MR of 1 is its placeholder at the wheel, not this damper's)
            k = rate / 1000 / (mr0 * mr0)
            res["curves"]["wheel_rate_N_per_mm"] = [k * m * m for m in res["curves"]["motion_ratio"]]
            res["static"]["wheel_rate_N_per_mm"] = k * mr0 * mr0
            n = n + [f"spring and damper given at the wheel (Assetto Corsa): {rate / 1000:.1f} N/mm at the wheel, "
                     f"{k:.1f} N/mm on this damper (motion ratio {mr0:.2f})"]
        lay = describe()[key]
        res.update(corner=name, type=key, name=lay["name"], points=pts, labels=labels, lines=lay["lines"],
                   single=lay["single"], notes=n)
        out[axle] = res
    return {"corners": out, "notes": notes}


def study_svj_json(svj_json, travel_mm=100, base_json=None):
    """study_svj() for the editor: SVJ text in, JSON out. base_json: overrides (fit.base_points) for a
    second study, the SVJ's layout on the base vehicle's tied points, returned as "base"."""
    import json
    svj = json.loads(svj_json)
    out = study_svj(svj, travel_mm)
    if base_json:
        out["base"] = study_svj(svj, travel_mm, json.loads(base_json), "_base")
    return json.dumps(out)
