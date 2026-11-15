"""Check the twin against what the real robot is recorded as doing.

Each entry is a program from the repository and the outcome written down for it in the
testing log, the changelog or the project site. The check runs each program on the
hardware build it was written for and reports whether the twin agrees. It is the guard
against a model that only suits the latest version.

    reaches   True  = got to the Task Decider      False = did not get round the course
    stop      "end" = stopped at the Decider       "none" = never stopped (no working end stop)
              "early" = stopped before the end     None = not recorded
"""
from __future__ import annotations

import os

from . import train

RECORD = [
    ("navigation_v8.py", True, "none", "Trial 9: 'much better overall'; gets through the maze (changelog). No end stop."),
    ("navigation_v9.py", False, None, "Trial 10: zig-zagged +/-45 degrees on straights and hit the walls. Rejected."),
    ("navigation_v8a.py", True, "none", "Trial 13: completed the course. No end stop."),
    ("navigation_v11.py", True, "none", "Changelog: tuned v8 confirmed to run the course; saved as baseline."),
    ("navigation_v11a.py", None, "early", "Trial 14: stopped at every corner where the track widens."),
    ("navigation_v11e.py", None, "early", "Trial 16: stopped on a straight."),
    ("navigation_v11g.py", None, "early", "Trial 17: stopped at 20 s during a right turn."),
    ("navigation_v11h.py", False, None, "Trial 18: drove differently and failed the course."),
    ("navigation_v12.py", True, "none", "Trial 19: completed the whole maze autonomously; did not stop at the Decider."),
    ("main_v21.py", True, "end", "Reported 14 Nov: completes, stops at the end, about 20 s."),
]


def classify(runs):
    n = float(len(runs))
    return {"reaches": sum(1 for r in runs if r["reached_end"]) / n,
            "stop_end": sum(1 for r in runs if r["stopped_at_end"]) / n,
            "stop_early": sum(1 for r in runs if r["stopped_early"]) / n,
            "contacts": sum(r["contacts"] for r in runs) / n,
            "hardware": sorted({r["hardware"] for r in runs})}


def check(find, cal, n=12, progress=None):
    """find(name) -> source text or None. Returns one row per recorded program."""
    rows = []
    pool = train.make_pool()
    try:
        for k, (name, reaches, stop, note) in enumerate(RECORD):
            src = find(name)
            if src is None:
                rows.append({"program": name, "record": note, "available": False})
            else:
                runs = train.trials(src, name, {}, n, True, cal, seed0=7000, pool=pool)["runs"]
                c = classify(runs)
                checks = []
                if reaches is not None:
                    checks.append((c["reaches"] >= 0.6) == reaches)
                if stop == "end":
                    checks.append(c["stop_end"] >= 0.6)
                elif stop == "none":
                    checks.append(c["stop_end"] + c["stop_early"] <= 0.4)
                elif stop == "early":
                    checks.append(c["stop_early"] >= 0.4)
                c.update({"program": name, "record": note, "available": True, "agrees": all(checks),
                          "expect_reaches": reaches, "expect_stop": stop, "n": n})
                rows.append(c)
            if progress:
                progress((k + 1) / float(len(RECORD)))
    finally:
        if pool is not None:
            pool.shutdown()
    return rows


def finder(repo):
    def find(name):
        for sub in ("", os.path.join("py code archive", "navigation"), "py code archive"):
            path = os.path.join(repo, sub, name)
            if os.path.exists(path):
                try:
                    with open(path, "r", encoding="utf-8") as f:
                        return f.read()
                except OSError:
                    return None
        return None
    return find
