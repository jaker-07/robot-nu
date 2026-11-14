"""Find the tunable constants in a robot program and substitute trial values.

The robot's file on disk is never modified. Overrides are applied to a copy of the
source text in memory, changing only the number on the right of `NAME = number`.
"""
from __future__ import annotations

import ast

# Search ranges for the constants the navigation code is known to use (lo, hi).
RANGES = {
    "BASE_SPEED": (30, 55), "MIN_SPEED": (22, 35), "MAX_SPEED": (45, 80),
    "KP": (0.5, 4.0), "DEADBAND": (0, 4), "MAX_ERR": (4, 12),
    "FRONT_TURN_CM": (5, 16), "STOP_CM": (8, 16), "END_CONFIRM": (2, 6),
    "GAP_RF_MS": (0, 15), "MAX_CM": (40, 100),
    "LEFT_TRIM": (0.85, 1.0), "RIGHT_TRIM": (0.85, 1.0),
    "SPIN_TIME": (0.6, 2.5), "SPIN_SPEED": (35, 80), "BACKUP_TIME": (0.2, 1.0),
    "MIN_HOLD": (1.0, 2.8), "LIGHT_FROM": (3.3, 4.7), "SPIN_FROM": (5.5, 9.5),
    "LIGHT_TIME": (0.5, 4.0),
}
DEFAULT_TRAIN = ["BASE_SPEED", "MAX_SPEED", "KP", "DEADBAND", "FRONT_TURN_CM", "STOP_CM", "SPIN_TIME"]


def tunables(source):
    """Top-level `UPPER_CASE = number` assignments, in file order."""
    out = []
    try:
        tree = ast.parse(source)
    except SyntaxError:
        return out
    lines = source.split("\n")
    for node in tree.body:
        if not (isinstance(node, ast.Assign) and len(node.targets) == 1
                and isinstance(node.targets[0], ast.Name)):
            continue
        name = node.targets[0].id
        if not name.isupper():
            continue
        val = node.value
        neg = False
        if isinstance(val, ast.UnaryOp) and isinstance(val.op, ast.USub):
            neg = True
            inner = val.operand
        else:
            inner = val
        if not (isinstance(inner, ast.Constant) and isinstance(inner.value, (int, float))
                and not isinstance(inner.value, bool)):
            continue
        if val.lineno != val.end_lineno:
            continue
        number = -inner.value if neg else inner.value
        line = lines[val.lineno - 1]
        comment = line.split("#", 1)[1].strip() if "#" in line else ""
        lo, hi = RANGES.get(name, (None, None))
        if lo is None:
            if number > 0:
                lo, hi = number * 0.5, number * 1.5
            elif number < 0:
                lo, hi = number * 1.5, number * 0.5
            else:
                lo, hi = 0.0, 1.0
        out.append({"name": name, "value": number, "is_int": isinstance(number, int),
                    "line": val.lineno, "col": val.col_offset, "end_col": val.end_col_offset,
                    "comment": comment, "lo": min(lo, number), "hi": max(hi, number)})
    return out


def fmt(value, is_int):
    if is_int:
        return str(int(round(value)))
    text = "%.3f" % value
    text = text.rstrip("0")
    return text + "0" if text.endswith(".") else text


def clean(overrides, table):
    """Round overrides to the constant's own type and drop ones equal to the file's value."""
    by = {t["name"]: t for t in table}
    out = {}
    for name, v in (overrides or {}).items():
        t = by.get(name)
        if t is None:
            continue
        v = float(fmt(float(v), t["is_int"]))
        if abs(v - t["value"]) > 1e-9:
            out[name] = int(v) if t["is_int"] else v
    return out


def repair(values):
    """Keep the speed limits in order: MIN_SPEED < BASE_SPEED <= MAX_SPEED."""
    v = dict(values)
    if "BASE_SPEED" in v and "MAX_SPEED" in v and v["MAX_SPEED"] < v["BASE_SPEED"]:
        v["MAX_SPEED"] = v["BASE_SPEED"]
    if "BASE_SPEED" in v and "MIN_SPEED" in v and v["MIN_SPEED"] > v["BASE_SPEED"] - 3:
        v["MIN_SPEED"] = v["BASE_SPEED"] - 3
    return v


def apply_overrides(source, overrides):
    """Return source with the given constants replaced. Unknown names are ignored."""
    if not overrides:
        return source
    table = {t["name"]: t for t in tunables(source)}
    lines = source.split("\n")
    edits = {}
    for name, v in overrides.items():
        t = table.get(name)
        if t is not None:
            edits.setdefault(t["line"], []).append((t["col"], t["end_col"], fmt(float(v), t["is_int"])))
    for lineno, items in edits.items():
        raw = lines[lineno - 1].encode("utf-8")
        for col, end, text in sorted(items, reverse=True):
            raw = raw[:col] + text.encode("utf-8") + raw[end:]
        lines[lineno - 1] = raw.decode("utf-8")
    return "\n".join(lines)
