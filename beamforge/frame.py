"""The SVJ's physics frame: SAE J670 (X forward, Y right, Z down) or ISO 8855 (X forward, Y left, Z up; from spec v0.109).

BeamForge reads SAE (svj.from_sae, the hardpoints, the corner view), so an ISO file is converted once, on import (unified.adopt), by the rules of the
SVJ spec (section 2.1, svj-py/svj/frame.py of the standard's repository). The two frames differ by one fixed rotation of 180 degrees about X, nothing
mirrors:

    position or direction [x, y, z]      -> [x, -y, -z]
    frame orientation quaternion [x, y, z, w] -> [w, -z, y, -x]  (left-multiplied by (1, 0, 0, 0))
    inertia tensor in vehicle axes       -> Ixy and Ixz change sign (a tensor in a body's own axes, with inertia_orientation, stays)
    scalars, and quantities defined in words (camber top inward, toe-in, bump) -> unchanged

Only the keys in the allow-lists are touched. Pure Python, standard library only.
"""

import copy

SAE, ISO = "SAE_J670", "ISO_8855"

VECTOR_KEYS = {
    "position", "center_of_gravity", "rack_position", "end_points", "end_seats", "pivot_position", "pendulum_pivot",
    "pendulum_axis", "attach_point", "cg_position", "beam_position", "center_of_pressure", "linkage_points",
    "input_flange", "output_flange", "output_front", "output_rear", "output_left", "output_right", "column_axis",
}
SIDE_KEYS = {"FL", "FR", "RL", "RR", "left", "right"}      # (named positions when the value is a bare 3-vector)
QUAT_KEYS = {"orientation", "inertia_orientation"}
PRODUCTS = ("Ixy", "Ixz")


def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _neg(v):
    return v if v == 0 else -v


def _is_vec(v, n):
    return isinstance(v, list) and len(v) == n and all(_num(x) for x in v)


def _vec_groups(v):
    return _is_vec(v, 3) or (isinstance(v, list) and bool(v) and all(_is_vec(x, 3) for x in v))


def _version(v):
    try:
        return tuple(int(p) for p in str(v).split("."))
    except ValueError:
        return (0, 0)


def _named(value):
    if not isinstance(value, str):
        return None
    name = value.upper().replace(" ", "_").replace("-", "_")
    if name.startswith("ISO") or name == "OPENDRIVE":
        return ISO
    return SAE if name.startswith("SAE") else None


def frame_of(doc):
    """The declared physics frame: _metadata.physics_frame, then a string _metadata.coordinate_system (an object there is the glTF convention);
    with no declaration the version decides: SAE below 0.109, ISO from it. A document without metadata is SAE."""
    md = (doc or {}).get("_metadata") or {}
    for key in ("physics_frame", "coordinate_system"):
        f = _named(md.get(key))
        if f:
            return f
    return ISO if _version(md.get("version")) >= (0, 109) else SAE


def _flip(v):
    return [v[0], _neg(v[1]), _neg(v[2])]


def _quat(q):
    x, y, z, w = q
    return [w, _neg(z), y, _neg(x)]


def to_sae(doc):
    """Convert `doc` in place to SAE J670 when it declares ISO 8855 (the rotation is its own inverse for vectors; the quaternion's inverse is its
    negative, as quaternions here are only used as rotations the sign does not matter). Returns the note, or None when nothing was done."""
    if frame_of(doc) != ISO:
        return None
    count = [0, 0, 0]

    def flip_value(v):
        if _is_vec(v, 3):
            count[0] += 1
            return _flip(v)
        count[0] += len(v)
        return [_flip(x) for x in v]

    def walk(node):
        if isinstance(node, dict):
            body_frame = "inertia_orientation" in node
            for k, v in list(node.items()):
                if k in QUAT_KEYS and _is_vec(v, 4):
                    node[k] = [_neg(c) for c in _quat(v)]
                    count[1] += 1
                elif k == "inertia" and isinstance(v, dict):
                    if not body_frame:
                        for p in PRODUCTS:
                            if _num(v.get(p)):
                                v[p] = _neg(v[p])
                                count[2] += 1
                elif (k in VECTOR_KEYS or (k in SIDE_KEYS and _is_vec(v, 3))) and _vec_groups(v):
                    node[k] = flip_value(v)
                else:
                    walk(v)
        elif isinstance(node, list):
            for v in node:
                walk(v)

    walk(doc)
    md = doc.setdefault("_metadata", {})
    if isinstance(md.get("coordinate_system"), dict) or md.get("physics_frame"):
        md["physics_frame"] = SAE                         # (the glTF asset convention stays in coordinate_system)
    else:
        md["coordinate_system"] = SAE
    return (f"the file is in ISO 8855 axes (Y left, Z up): read in SAE J670, as BeamForge does ({count[0]} vectors, {count[1]} quaternions, "
            f"{count[2]} inertia products flipped)")
