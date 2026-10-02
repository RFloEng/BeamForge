"""Lenient JBeam reader.

JBeam is JSON plus C-style comments, with every comma optional. This parser
treats commas as whitespace, which accepts missing, extra and trailing commas.

parse() turns jbeam text into plain Python dicts, lists, str, int, float, bool
and None. expand_table() turns a jbeam table section (nodes, beams, variables,
...) into one dict per row, and resolve() / to_float() evaluate "$variable" and
"$=" values offline, and report the ones they cannot evaluate.
"""

import ast
import math
import re


class JBeamError(ValueError):
    """Syntax error in a jbeam file; the message starts with "source:line:"."""


class UnresolvedValue(ValueError):
    """A $variable or $= expression that cannot be evaluated offline."""


_NUMBER = re.compile(r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?")   # BeamNG files also write "+0.5"


class _Parser:
    """Recursive-descent parser over one string; self.i is the read position."""

    def __init__(self, text, source="<jbeam>"):
        self.s = text
        self.i = 0
        self.source = source

    def error(self, msg):
        line = self.s.count("\n", 0, self.i) + 1
        raise JBeamError(f"{self.source}:{line}: {msg}")

    def skip(self):
        """Advance past whitespace, commas and // or /* */ comments."""
        s, n = self.s, len(self.s)
        while self.i < n:
            c = s[self.i]
            if c in " \t\r\n,":
                self.i += 1
            elif s.startswith("//", self.i):
                end = s.find("\n", self.i)
                self.i = n if end < 0 else end + 1
            elif s.startswith("/*", self.i):
                end = s.find("*/", self.i + 2)
                if end < 0:
                    self.error("unterminated /* comment")
                self.i = end + 2
            else:
                break

    def value(self):
        """Parse one value at the read position."""
        self.skip()
        if self.i >= len(self.s):
            self.error("unexpected end of file")
        c = self.s[self.i]
        if c == "{":
            return self.obj()
        if c == "[":
            return self.arr()
        if c == '"':
            return self.string()
        for word, val in (("true", True), ("false", False), ("null", None)):
            if self.s.startswith(word, self.i):
                self.i += len(word)
                return val
        m = _NUMBER.match(self.s, self.i)
        if m:
            self.i = m.end()
            text = m.group()
            # as in JSON: no '.' or exponent means int
            return float(text) if any(ch in text for ch in ".eE") else int(text)
        self.error(f"unexpected character {c!r}")

    def obj(self):
        self.i += 1
        out = {}
        while True:
            self.skip()
            if self.i >= len(self.s):
                self.error("unterminated object")
            if self.s[self.i] == "}":
                self.i += 1
                return out
            if self.s[self.i] != '"':
                self.error("object key must be a string")
            key = self.string()
            self.skip()
            if self.i >= len(self.s) or self.s[self.i] != ":":
                self.error(f"expected ':' after key {key!r}")
            self.i += 1
            out[key] = self.value()

    def arr(self):
        self.i += 1
        out = []
        while True:
            self.skip()
            if self.i >= len(self.s):
                self.error("unterminated array")
            if self.s[self.i] == "]":
                self.i += 1
                return out
            out.append(self.value())

    def string(self):
        """Parse a double-quoted string with JSON escapes (unknown escapes keep the character)."""
        self.i += 1
        buf = []
        s = self.s
        while self.i < len(s):
            c = s[self.i]
            if c == '"':
                self.i += 1
                return "".join(buf)
            if c == "\\":
                self.i += 1
                esc = s[self.i : self.i + 1]
                if esc == "u":
                    buf.append(chr(int(s[self.i + 1 : self.i + 5], 16)))
                    self.i += 5
                    continue
                buf.append({"n": "\n", "t": "\t", "r": "\r", "b": "\b", "f": "\f"}.get(esc, esc))
                self.i += 1
                continue
            buf.append(c)
            self.i += 1
        self.error("unterminated string")


def parse(text, source="<jbeam>"):
    """Parse jbeam text into Python values; source names the file in error messages.

    Raises JBeamError on a syntax error or on content after the top-level value. Stray closing
    braces, brackets and commas after it are ignored: mods ship files with an extra "}" at the
    end, and the game loads them.
    """
    p = _Parser(text, source)
    v = p.value()
    p.skip()
    while p.i < len(p.s) and p.s[p.i] in "}],":
        p.i += 1
        p.skip()
    if p.i != len(p.s):
        p.error("trailing content after top-level value")
    return v


def expand_table(rows):
    """Turn a JBeam table (header, property dicts, data rows) into records.

    A dict row sets properties for every following row. A dict at the end of a
    data row overrides properties for that row only.
    """
    if not rows or not isinstance(rows[0], list):
        return []
    header = [str(h).rstrip(":") for h in rows[0]]
    props = {}
    out = []
    for row in rows[1:]:
        if isinstance(row, dict):
            props.update(row)
            continue
        if not isinstance(row, list):
            continue
        inline = {}
        cells = row
        if cells and isinstance(cells[-1], dict):
            inline = cells[-1]
            cells = cells[:-1]
        rec = dict(props)
        rec.update(inline)
        rec.update(zip(header, cells))
        out.append(rec)
    return out


MAX_EXPR = 400                 # characters: longer "$=" expressions are refused


def _case(*args):
    """BeamNG's case(c1, v1, c2, v2, ..., default): the value of the first true condition."""
    for i in range(0, len(args) - 1, 2):
        if args[i]:
            return args[i + 1]
    return args[-1] if len(args) % 2 else None


_FUNCS = {"case": _case, "sin": math.sin, "cos": math.cos, "tan": math.tan, "abs": abs, "min": min, "max": max,
          "sqrt": math.sqrt, "floor": math.floor, "ceil": math.ceil, "round": round, "exp": math.exp,
          "log": math.log, "random": lambda *a: 0.5}      # random: the middle of its range (offline, repeatable)
_LUA = [(re.compile(r"~="), "!="), (re.compile(r"\^"), "**"), (re.compile(r"\bnil\b"), "None"),
        (re.compile(r"\btrue\b"), "True"), (re.compile(r"\bfalse\b"), "False")]
_VAR = re.compile(r"\$([A-Za-z_][A-Za-z0-9_]*)")


def evaluate(expr, variables):
    """Value of a BeamNG "$=" expression (Lua syntax), or UnresolvedValue.

    Supported: numbers, $variables (an undefined one is nil), + - * / % ^, comparisons
    (== ~= < <= > >=), and / or / not (Lua semantics: "c and a or b" picks a or b), and the
    functions in _FUNCS (case, sin, cos, round, min, max, sqrt ...). The expression is parsed into a
    Python syntax tree and walked node by node: no eval, no names or attributes beyond these, and
    powers are bounded, so a hostile file cannot run code or hang the editor.
    """
    if len(expr) > MAX_EXPR:
        raise UnresolvedValue(expr)
    src = expr
    for pat, rep in _LUA:
        src = pat.sub(rep, src)
    src = _VAR.sub(lambda m: "__v_" + m.group(1), src)
    try:
        tree = ast.parse(src.strip(), mode="eval")
    except SyntaxError as exc:
        raise UnresolvedValue(expr) from exc

    def var(name):
        v = variables.get("$" + name)
        if v is None or isinstance(v, bool) or isinstance(v, (int, float)):
            return v
        try:
            return float(v)
        except (TypeError, ValueError):
            raise UnresolvedValue(expr)          # a variable whose value is not a number

    def ev(n):
        if isinstance(n, ast.Expression):
            return ev(n.body)
        if isinstance(n, ast.Constant) and (n.value is None or isinstance(n.value, (int, float, bool))):
            return n.value
        if isinstance(n, ast.Name):
            if n.id.startswith("__v_"):
                return var(n.id[4:])
            if n.id == "pi":
                return math.pi
            raise UnresolvedValue(expr)
        if isinstance(n, ast.UnaryOp):
            v = ev(n.operand)
            if isinstance(n.op, ast.Not):
                return not v
            if isinstance(n.op, ast.USub):
                return -v
            if isinstance(n.op, ast.UAdd):
                return +v
        if isinstance(n, ast.BinOp):
            a, b = ev(n.left), ev(n.right)
            if isinstance(n.op, ast.Pow):
                if abs(b) > 64 or abs(a) > 1e6:  # 9**9**9**9 would hang
                    raise UnresolvedValue(expr)
                return a ** b
            ops = {ast.Add: lambda: a + b, ast.Sub: lambda: a - b, ast.Mult: lambda: a * b,
                   ast.Div: lambda: a / b, ast.Mod: lambda: a % b}
            if type(n.op) in ops:
                return ops[type(n.op)]()
        if isinstance(n, ast.BoolOp):            # Lua and / or return an operand, as Python does
            v = None
            for x in n.values:
                v = ev(x)
                if isinstance(n.op, ast.And) and not v:
                    return v
                if isinstance(n.op, ast.Or) and v:
                    return v
            return v
        if isinstance(n, ast.Compare):
            left = ev(n.left)
            for op, right in zip(n.ops, n.comparators):
                r = ev(right)
                ok = {ast.Eq: lambda: left == r, ast.NotEq: lambda: left != r, ast.Lt: lambda: left < r,
                      ast.LtE: lambda: left <= r, ast.Gt: lambda: left > r, ast.GtE: lambda: left >= r}.get(type(op))
                if ok is None or not ok():
                    return False
                left = r
            return True
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id in _FUNCS and not n.keywords:
            return _FUNCS[n.func.id](*[ev(a) for a in n.args])
        raise UnresolvedValue(expr)

    try:
        out = ev(tree)
    except UnresolvedValue:
        raise
    except Exception as exc:                     # a type error, a division by zero, nil in arithmetic ...
        raise UnresolvedValue(expr) from exc
    if isinstance(out, bool) or out is None:
        raise UnresolvedValue(expr)
    return out


def resolve(value, variables):
    """Resolve "$var" and "$=" expressions to numbers.

    variables maps "$name" to its value (part default or .pc value). Anything that is not a
    string starting with "$" is returned unchanged. A "$=" expression is evaluated by evaluate()
    (BeamNG's Lua syntax, with case() and nil); a plain "$name" must be defined.
    """
    if not isinstance(value, str) or not value.startswith("$"):
        return value
    if value.startswith("$="):
        return evaluate(value[2:], variables)
    if value in variables:
        return variables[value]
    raise UnresolvedValue(value)


def to_float(value, variables, default=None):
    """Numeric value of a JBeam field; 'FLT_MAX' becomes math.inf.

    None (field absent) gives default. Raises UnresolvedValue for a variable or
    expression that cannot be evaluated, and for a value that is not a number
    (so callers catching UnresolvedValue report it instead of crashing).
    """
    if value is None:
        return default
    if value == "FLT_MAX":
        return math.inf
    v = resolve(value, variables)
    if v == "FLT_MAX":
        return math.inf
    try:
        return float(v)
    except (TypeError, ValueError) as exc:
        raise UnresolvedValue(f"{value!r} is not a number") from exc
