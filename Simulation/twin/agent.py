"""Claude engineering agent for the twin.

Talks to the Anthropic Messages API over plain HTTPS (standard library only) and gives
Claude tools to read the robot program, run attempts and trials, train constants, and
propose changes. Every proposed change is tested against the original on the same set of
varied attempts and is written to Simulation/candidates/ only if it wins. The agent
cannot edit the robot's own files.

The API key is read from the ANTHROPIC_API_KEY environment variable or, failing that,
from the first line of Simulation/anthropic_key.txt.
"""
from __future__ import annotations

import json
import os
import time
import urllib.error
import urllib.request

from . import params, sim, train

API_URL = "https://api.anthropic.com/v1/messages"
API_VERSION = "2023-06-01"
MODELS = ["claude-sonnet-5-5", "claude-opus-5-5", "claude-fable-5-2", "claude-haiku-5-5"]
DEFAULT_MODEL = "claude-sonnet-5-5"
MAX_ROUNDS = 14
COMPARE_N = 24
COMPARE_SEED = 500000

SYSTEM = """You are the engineering agent inside a digital twin of a student robot for the GE1501 AE1 course project.

The robot: Raspberry Pi Pico running one MicroPython file, two TT gear motors through an L298N, three HC-SR04 ultrasonic sensors (front, and left/right angled 45 degrees outwards), an IR sensor that sees the Task Decider's flag, a buzzer and a 5-LED strip. It must drive a walled corridor whose five legs are 30.0, 30.0, 25.4, 23.1 and 35.8 cm wide with one 90 degree and three 60 degree bends, stop at the Task Decider, read how long the flag is shown (about 3 s buzzer, 5 s light, 10 s spin) and do that task.

How it is marked: no wall contact, one continuous run with no pauses, staying centred, then prompt and correct task execution. Speed earns nothing. Code quality is also marked, so keep the program one coherent, readable file.

The twin executes the program unmodified against a physical model. Your tools read the program, run single attempts and batches of varied attempts, search constants, and propose changes. A proposed change is tested against the original on the same varied attempts and is saved as a candidate file only if it wins; you cannot edit the robot's own files.

Method:
- Look before you change: read the relevant code and run attempts to see what actually happens.
- Form a specific hypothesis about a failure, then test one change at a time.
- Judge by batches of varied attempts, never by a single run.
- Prefer the smallest change that fixes the problem. Constants before code.

Honesty about the model: every result is a prediction of the twin, which was fitted to three reported facts (the robot completes, stops at the end, takes about 20 s) and otherwise rests on estimates. Use get_calibration to see which numbers are assumed. Say so when a conclusion depends on one, and name the physical measurement or test run that would confirm it. Never present a twin result as proven on the real robot.

Write plain prose with numbers. No Markdown headings, tables or bullet symbols; short paragraphs only. Be concise."""

_OVR = {"type": "object", "description": "Constant name to value, e.g. {\"KP\": 2.5}. Omitted constants keep the file's value.",
        "additionalProperties": {"type": "number"}}

TOOLS = [
    {"name": "read_program",
     "description": "Return the selected robot program's source with line numbers. Give a line range to read part of it.",
     "input_schema": {"type": "object", "properties": {
         "start_line": {"type": "integer"}, "end_line": {"type": "integer"}}}},
    {"name": "list_constants",
     "description": "List the program's tunable constants (top-level NAME = number): value, line, comment and a sensible search range.",
     "input_schema": {"type": "object", "properties": {}}},
    {"name": "run_attempt",
     "description": "Run one attempt at the course. Returns the outcome, timings, wall contacts (with the leg, 1 to 5, each happened in), "
                    "centring, the task performed, a rubric-style score out of 60, and a timeline sampled twice a second "
                    "(position along the course, sideways offset, heading, sensor readings, motor power).",
     "input_schema": {"type": "object", "properties": {
         "overrides": _OVR,
         "task": {"type": "string", "enum": ["buzzer", "light", "spin"]},
         "seed": {"type": "integer"},
         "vary": {"type": "boolean", "description": "Vary start placement, battery, motors and wall positions. Default false."}}}},
    {"name": "run_trials",
     "description": "Run a batch of varied attempts (start placement, battery, motor mismatch, sensor noise, wall tolerance) cycling the "
                    "three task signals. Returns success rate, contact-free rate, contacts per run and by leg, time, score, and each failure.",
     "input_schema": {"type": "object", "properties": {
         "overrides": _OVR, "n": {"type": "integer", "description": "Attempts, 6 to 60. Default 24."}}}},
    {"name": "train_constants",
     "description": "Search the named constants for values that score better across varied attempts (evolution strategy), then validate the "
                    "best set against the starting values on fresh attempts. Slow: a default run is about 350 attempts.",
     "input_schema": {"type": "object", "properties": {
         "names": {"type": "array", "items": {"type": "string"}},
         "bounds": {"type": "object", "description": "Optional {name: [low, high]}.",
                    "additionalProperties": {"type": "array", "items": {"type": "number"}}},
         "overrides": _OVR,
         "rounds": {"type": "integer", "description": "1 to 12. Default 6."},
         "candidates": {"type": "integer", "description": "4 to 16. Default 10."},
         "attempts_each": {"type": "integer", "description": "2 to 8. Default 5."}},
         "required": ["names"]}},
    {"name": "save_candidate",
     "description": "Test a set of constant values against the file's own values on the same varied attempts and, only if it wins, save a copy "
                    "of the program with those constants to Simulation/candidates/. Returns the comparison either way.",
     "input_schema": {"type": "object", "properties": {
         "overrides": _OVR, "note": {"type": "string", "description": "One line on what the change is for."}},
         "required": ["overrides", "note"]}},
    {"name": "propose_code_edit",
     "description": "Propose a change to the program's code. Each edit replaces old_text (which must occur exactly once in the program) with "
                    "new_text. The edited program is compiled and tested against the original on the same varied attempts; only if it wins "
                    "is it saved to Simulation/candidates/. The original file is never changed. Returns the comparison either way.",
     "input_schema": {"type": "object", "properties": {
         "edits": {"type": "array", "items": {"type": "object", "properties": {
             "old_text": {"type": "string"}, "new_text": {"type": "string"}}, "required": ["old_text", "new_text"]}},
         "note": {"type": "string", "description": "One line on what the change is for."}},
         "required": ["edits", "note"]}},
    {"name": "get_history",
     "description": "Recent trial batches and training runs recorded by the app.",
     "input_schema": {"type": "object", "properties": {"limit": {"type": "integer"}}}},
    {"name": "get_calibration",
     "description": "Every physical number in the twin with its value and source (brief, code, log, photo, datasheet, fitted, assumed, measured).",
     "input_schema": {"type": "object", "properties": {}}},
]


class AgentError(Exception):
    pass


def find_key(sim_dir):
    key = os.environ.get("ANTHROPIC_API_KEY", "").strip()
    if key:
        return key
    path = os.path.join(sim_dir, "anthropic_key.txt")
    if os.path.exists(path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                for line in f:
                    if line.strip():
                        return line.strip()
        except OSError:
            pass
    return None


def http_transport(body, key, timeout=240):
    req = urllib.request.Request(
        API_URL, data=json.dumps(body).encode("utf-8"), method="POST",
        headers={"x-api-key": key, "anthropic-version": API_VERSION, "content-type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        detail = ""
        try:
            detail = json.loads(exc.read().decode("utf-8")).get("error", {}).get("message", "")
        except Exception:
            pass
        hint = {401: " Check the API key.", 429: " Rate limited; wait and retry.",
                529: " The API is overloaded; retry shortly."}.get(exc.code, "")
        raise AgentError("Anthropic API error %d: %s%s" % (exc.code, detail or exc.reason, hint))
    except urllib.error.URLError as exc:
        raise AgentError("Could not reach api.anthropic.com: %s" % exc.reason)


def _slim(agg):
    return {k: agg[k] for k in ("n", "success", "clean", "contacts", "hard_contacts", "nav_time",
                                "score", "score_sd", "objective", "spin_err", "outcomes")}


class Agent:
    def __init__(self, app, transport=None):
        self.app = app
        self.transport = transport or http_transport
        self.convs = {}
        self.seq = 0

    # ------------------------------------------------------------ helpers
    def _ctx(self, fw, overrides):
        src, name = self.app.read_fw(fw)
        table = params.tunables(src)
        return {"fw": fw, "src": src, "name": name, "table": table,
                "over": params.clean(overrides, table), "cal": self.app.cal()}

    def _over(self, ctx, given):
        if given is None:
            return dict(ctx["over"])
        return params.clean(params.repair(given), ctx["table"])

    def _compare(self, ctx, src_new, name_new, over_new):
        base = train.trials(ctx["src"], ctx["name"], {}, COMPARE_N, True, ctx["cal"], seed0=COMPARE_SEED)
        new = train.trials(src_new, name_new, over_new, COMPARE_N, True, ctx["cal"], seed0=COMPARE_SEED)
        b, a = base["summary"], new["summary"]
        errors = sorted({r["error"] for r in new["runs"] if r.get("error")})
        gain = round(a["objective"] - b["objective"], 2)
        wins = gain > 0.5 and a["success"] >= b["success"] - 1e-9 and not errors
        return {"original": _slim(b), "proposed": _slim(a), "gain": gain, "wins": wins,
                "program_errors": errors, "attempts": COMPARE_N}

    def _write_candidate(self, stem, text, header):
        os.makedirs(self.app.cand_dir, exist_ok=True)
        out = "%s_agent_%s.py" % (stem, time.strftime("%Y%m%d_%H%M%S"))
        with open(os.path.join(self.app.cand_dir, out), "w", encoding="utf-8") as f:
            f.write(header + text)
        return "Simulation/candidates/" + out

    # -------------------------------------------------------------- tools
    def t_read_program(self, ctx, a):
        lines = ctx["src"].split("\n")
        lo = max(1, int(a.get("start_line") or 1))
        hi = min(len(lines), int(a.get("end_line") or len(lines)))
        hi = min(hi, lo + 399)
        return {"program": ctx["name"], "total_lines": len(lines), "from": lo, "to": hi,
                "text": "\n".join("%4d  %s" % (i, lines[i - 1]) for i in range(lo, hi + 1))}

    def t_list_constants(self, ctx, a):
        return {"program": ctx["name"], "in_app_overrides": ctx["over"],
                "constants": [{k: t[k] for k in ("name", "value", "line", "comment", "lo", "hi")} for t in ctx["table"]]}

    def t_run_attempt(self, ctx, a):
        over = self._over(ctx, a.get("overrides"))
        r = sim.simulate(ctx["src"], ctx["name"], over, seed=int(a.get("seed") or 1),
                         task=a.get("task") or "spin", vary=bool(a.get("vary")), cal=ctx["cal"], trace=True)
        tr = r.pop("trace")
        from .track import Track
        tk = Track(ctx["cal"])
        timeline = []
        for i in range(0, len(tr["t"]), 25):
            if tr["ph"][i] == 0:
                continue
            s, lat, leg = tk.project(tr["x"][i], tr["y"][i])
            timeline.append({"t": tr["t"][i], "leg": leg + 1, "along_cm": round(s), "offset_cm": round(lat, 1),
                             "L": tr["dl"][i], "F": tr["df"][i], "R": tr["dr"][i],
                             "motor_L": tr["ul"][i], "motor_R": tr["ur"][i], "touching": tr["c"][i]})
        r.pop("track", None)
        r["overrides"] = over
        r["timeline_note"] = "offset_cm: + is left of the corridor centre. Sensor value -1 means no echo."
        r["timeline"] = timeline[:90]
        return r

    def t_run_trials(self, ctx, a):
        over = self._over(ctx, a.get("overrides"))
        n = max(6, min(60, int(a.get("n") or 24)))
        out = train.trials(ctx["src"], ctx["name"], over, n, True, ctx["cal"], seed0=int(time.time()) % 100000)
        self.app.store.add("trials", ctx["name"], "agent", over, {"n": n}, out["summary"], out["runs"])
        legs = {}
        for r in out["runs"]:
            for leg in r.get("contact_legs", []):
                legs["leg %d" % leg] = legs.get("leg %d" % leg, 0) + 1
        fails = [{k: r[k] for k in ("seed", "task", "outcome", "progress", "contacts", "performed", "spin_deg")}
                 for r in out["runs"] if not r["success"]]
        return {"overrides": over, "summary": out["summary"], "contacts_by_leg": legs, "failures": fails[:20]}

    def t_train_constants(self, ctx, a):
        names = [str(n) for n in a.get("names", [])]
        bounds = {k: (float(v[0]), float(v[1])) for k, v in (a.get("bounds") or {}).items() if len(v) == 2}
        base = self._over(ctx, a.get("overrides"))
        gens = max(1, min(12, int(a.get("rounds") or 6)))
        pop = max(4, min(16, int(a.get("candidates") or 10)))
        seeds = max(2, min(8, int(a.get("attempts_each") or 5)))
        out = train.optimise(ctx["src"], ctx["name"], names, bounds, base, ctx["cal"], gens=gens, pop=pop,
                             seeds=seeds, vary=True, w_time=0.1, study_seed=int(time.time()) % 9973 + 1,
                             should_stop=self._stop)
        summ = {k: out[k] for k in ("before", "after", "gain", "accepted", "changed", "best", "validation_n")}
        self.app.store.add("training", ctx["name"], "agent", base,
                           {"names": out["names"], "gens": gens, "pop": pop, "seeds": seeds, "w_time": 0.1}, summ)
        return {"trained": out["names"], "recommended": out["accepted"], "gain": out["gain"],
                "changed": out["changed"], "best": out["best"], "validation_attempts": out["validation_n"],
                "start": _slim(out["before"]), "trained_result": _slim(out["after"]),
                "rounds": [{"round": h["gen"], "best": h["best"], "average": h["mean"]} for h in out["history"]]}

    def t_save_candidate(self, ctx, a):
        over = params.clean(params.repair(a.get("overrides") or {}), ctx["table"])
        if not over:
            return {"saved": False, "reason": "No constant differs from the file's value."}
        cmp = self._compare(ctx, ctx["src"], ctx["name"], over)
        if not cmp["wins"]:
            return {"saved": False, "reason": "It did not beat the original on the same attempts.", "comparison": cmp}
        by = {t["name"]: t for t in ctx["table"]}
        notes = ", ".join("%s %s -> %s" % (k, params.fmt(by[k]["value"], by[k]["is_int"]),
                                           params.fmt(v, by[k]["is_int"])) for k, v in sorted(over.items()))
        head = ("# Candidate proposed by the twin's agent from %s. Not yet tested on the robot.\n"
                "# %s\n# Changed constants: %s\n" % (ctx["name"], str(a.get("note", "")).replace("\n", " ")[:200], notes))
        path = self._write_candidate(os.path.splitext(ctx["name"])[0], params.apply_overrides(ctx["src"], over), head)
        return {"saved": True, "file": path, "overrides": over, "comparison": cmp}

    def t_propose_code_edit(self, ctx, a):
        text = ctx["src"]
        for k, e in enumerate(a.get("edits") or []):
            old, new = e.get("old_text", ""), e.get("new_text", "")
            count = text.count(old) if old else 0
            if count != 1:
                return {"saved": False, "reason": "Edit %d: old_text occurs %d times in the program; it must occur exactly once."
                                                  % (k + 1, count)}
            text = text.replace(old, new)
        if text == ctx["src"]:
            return {"saved": False, "reason": "The edits leave the program unchanged."}
        try:
            compile(text, ctx["name"], "exec")
        except SyntaxError as exc:
            return {"saved": False, "reason": "Syntax error at line %s: %s" % (exc.lineno, exc.msg)}
        cmp = self._compare(ctx, text, ctx["name"], {})
        if not cmp["wins"]:
            return {"saved": False, "reason": "It did not beat the original on the same attempts.", "comparison": cmp}
        head = ("# Candidate proposed by the twin's agent from %s. Not yet tested on the robot.\n# %s\n"
                % (ctx["name"], str(a.get("note", "")).replace("\n", " ")[:200]))
        path = self._write_candidate(os.path.splitext(ctx["name"])[0], text, head)
        return {"saved": True, "file": path, "comparison": cmp}

    def t_get_history(self, ctx, a):
        items = self.app.store.history(max(1, min(30, int(a.get("limit") or 12))))
        return {"items": [{"when": time.strftime("%d %b %H:%M", time.localtime(i["ts"])), "kind": i["kind"],
                           "program": i["firmware"], "by": i["label"] or "user", "overrides": i["overrides"],
                           "summary": i["summary"]} for i in items]}

    def t_get_calibration(self, ctx, a):
        from . import config
        values, sources = config.load(self.app.cal_path)
        return {"rows": [{"key": r["key"], "value": r["value"], "source": r["source"], "meaning": r["note"]}
                         for r in config.describe(values, sources)]}

    # --------------------------------------------------------------- turn
    def _stop(self):
        return False

    @staticmethod
    def _brief(name, out):
        if not isinstance(out, dict):
            return ""
        if "error" in out and len(out) == 1:
            return "failed: %s" % out["error"]
        if name == "run_attempt":
            return "%s, %d contact(s), score %s" % (out["outcome"], out["contacts"], out["score"])
        if name == "run_trials":
            s = out["summary"]
            return "%d attempts: %.0f%% success, %.0f%% clean, score %.1f" % (s["n"], 100 * s["success"], 100 * s["clean"], s["score"])
        if name == "train_constants":
            return "%s, gain %+.1f" % ("recommended" if out["recommended"] else "not recommended", out["gain"])
        if name in ("save_candidate", "propose_code_edit"):
            return ("saved %s" % out["file"]) if out.get("saved") else ("not saved: %s" % out.get("reason", ""))
        if name == "read_program":
            return "lines %d to %d" % (out["from"], out["to"])
        return ""

    def turn(self, conv_id, text, fw, overrides, model, on_step, should_stop):
        key = find_key(self.app.sim_dir)
        if not key:
            raise AgentError("No API key. Put it in Simulation/anthropic_key.txt or set ANTHROPIC_API_KEY, then try again.")
        if model not in MODELS:
            model = DEFAULT_MODEL
        ctx = self._ctx(fw, overrides)
        self._stop = should_stop
        conv = self.convs.setdefault(conv_id, [])
        mark = len(conv)
        conv.append({"role": "user", "content": text})
        system = SYSTEM + "\n\nSelected program: %s (%d lines). Constants currently overridden in the app: %s." % (
            ctx["name"], ctx["src"].count("\n") + 1, json.dumps(ctx["over"]) if ctx["over"] else "none")
        reply, proposed = [], None
        try:
            for _ in range(MAX_ROUNDS):
                resp = self.transport({"model": model, "max_tokens": 8000, "system": system,
                                       "tools": TOOLS, "messages": conv}, key)
                content = resp.get("content") or []
                conv.append({"role": "assistant", "content": content})     # kept verbatim, thinking blocks included
                for b in content:
                    if b.get("type") == "text" and b.get("text", "").strip():
                        reply.append(b["text"].strip())
                        on_step({"kind": "text", "text": b["text"].strip()})
                uses = [b for b in content if b.get("type") == "tool_use"]
                if not uses:
                    break
                results = []
                for u in uses:
                    args = u.get("input") or {}
                    on_step({"kind": "tool", "name": u["name"], "input": args})
                    fn = getattr(self, "t_" + u["name"], None)
                    try:
                        if fn is None:
                            raise AgentError("unknown tool %s" % u["name"])
                        out, err = fn(ctx, args), False
                    except Exception as exc:
                        out, err = {"error": "%s: %s" % (type(exc).__name__, exc)}, True
                    if isinstance(out, dict) and (out.get("best") or (out.get("saved") and out.get("overrides"))):
                        proposed = out.get("best") or out.get("overrides")
                    block = {"type": "tool_result", "tool_use_id": u["id"], "content": json.dumps(out)[:40000]}
                    if err:
                        block["is_error"] = True
                    results.append(block)
                    on_step({"kind": "result", "name": u["name"], "summary": self._brief(u["name"], out),
                             "file": out.get("file") if isinstance(out, dict) else None})
                conv.append({"role": "user", "content": results})
                if should_stop():
                    conv.append({"role": "assistant", "content": [{"type": "text", "text": "(Stopped by the user.)"}]})
                    break
            else:
                note = "(Stopped after %d rounds of tool use.)" % MAX_ROUNDS
                conv.append({"role": "assistant", "content": [{"type": "text", "text": note}]})
                reply.append(note)
        except Exception:
            del conv[mark:]                 # leave the conversation as it was before this turn
            raise
        return {"reply": "\n\n".join(reply), "proposed": proposed}
