"""Local web app: serves the front end and a small JSON API. Standard library only.

Everything it writes stays inside the Simulation folder:
  data/experiments.db   run history
  calibration.json      saved calibration values
  candidates/*.py       exported copies of the robot program with trained constants
The robot's own files are opened read-only.
"""
from __future__ import annotations

import glob
import json
import os
import re
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

from . import config, params, sim, train
from .agent import DEFAULT_MODEL, MODELS, Agent, find_key
from .robot import build_hull
from .store import Store
from .track import Track


def _natural(s):
    return [int(p) if p.isdigit() else p.lower() for p in re.split(r"(\d+)", s)]


class App:
    def __init__(self, repo, sim_dir):
        self.repo = os.path.abspath(repo)
        self.sim_dir = os.path.abspath(sim_dir)
        self.cal_path = os.path.join(self.sim_dir, "calibration.json")
        self.cand_dir = os.path.join(self.sim_dir, "candidates")
        self.store = Store(os.path.join(self.sim_dir, "data", "experiments.db"))
        self.jobs = {}
        self.job_seq = 0
        self.lock = threading.Lock()
        self.agent = Agent(self)

    # ------------------------------------------------------------ firmware
    def firmware(self):
        items = []

        def add(paths, group):
            for p in paths:
                rel = os.path.relpath(p, self.repo)
                items.append({"id": rel.replace(os.sep, "/"), "name": os.path.basename(p), "group": group})

        mains = sorted(glob.glob(os.path.join(self.repo, "main_v*.py")),
                       key=lambda p: _natural(os.path.basename(p)), reverse=True)
        add(mains, "Current")
        add(sorted(glob.glob(os.path.join(self.cand_dir, "*.py")), reverse=True), "Candidates")
        arch = glob.glob(os.path.join(self.repo, "py code archive", "navigation", "*.py"))
        arch += glob.glob(os.path.join(self.repo, "py code archive", "*.py"))
        add(sorted(arch, key=lambda p: _natural(os.path.basename(p)), reverse=True), "Archive")
        return items

    def read_fw(self, fid):
        for item in self.firmware():
            if item["id"] == fid:
                with open(os.path.join(self.repo, *fid.split("/")), "r", encoding="utf-8") as f:
                    return f.read(), item["name"]
        raise ValueError("unknown program: %s" % fid)

    # ---------------------------------------------------------------- info
    def info(self):
        cal, src = config.load(self.cal_path)
        fw = self.firmware()
        ang = cal["robot.side_sensor_angle_deg"]
        sx, sy = cal["robot.side_sensor_x_cm"], cal["robot.side_sensor_y_cm"]
        return {
            "firmware": fw,
            "default": fw[0]["id"] if fw else None,
            "track": Track(cal).to_json(),
            "robot": {
                "hull": [[round(x, 2), round(y, 2)] for x, y in build_hull(cal)],
                "wheel_track": cal["robot.track_cm"],
                "sensors": {"L": [sx, sy, ang], "F": [cal["robot.front_reach_cm"], 0, 0], "R": [sx, -sy, -ang]},
                "ir_x": cal["robot.ir_x_cm"],
            },
            "wall_r": cal["track.wall_radius_cm"],
            "calibration": config.describe(cal, src),
            "default_train": params.DEFAULT_TRAIN,
            "repo": self.repo,
        }

    def cal(self):
        return config.load(self.cal_path)[0]

    # ----------------------------------------------------------------- jobs
    def start_job(self, kind, fn):
        with self.lock:
            self.job_seq += 1
            jid = str(self.job_seq)
            job = {"id": jid, "kind": kind, "status": "running", "progress": 0.0,
                   "history": None, "result": None, "error": None, "cancel": False}
            self.jobs[jid] = job

        def work():
            try:
                job["result"] = fn(job)
                job["status"] = "stopped" if job["cancel"] else "done"
            except Exception as exc:
                job["error"] = str(exc) if type(exc).__name__ == "AgentError" else "%s: %s" % (type(exc).__name__, exc)
                job["status"] = "error"
                traceback.print_exc()
            job["progress"] = 1.0

        threading.Thread(target=work, daemon=True).start()
        return jid

    def run_trials(self, body):
        src, name = self.read_fw(body["fw"])
        over = params.clean(body.get("overrides"), params.tunables(src))
        n = max(1, min(200, int(body.get("n", 20))))
        cal = self.cal()

        def fn(job):
            out = train.trials(src, name, over, n, True, cal,
                               seed0=int(body.get("seed0", 1)),
                               progress=lambda p: job.__setitem__("progress", p))
            out["id"] = self.store.add("trials", name, body.get("label") or "", over,
                                       {"n": n}, out["summary"], out["runs"])
            out["overrides"] = over
            return out

        return self.start_job("trials", fn)

    def run_training(self, body):
        src, name = self.read_fw(body["fw"])
        table = params.tunables(src)
        base = params.clean(body.get("overrides"), table)
        names = [str(n) for n in body.get("names", [])]
        bounds = {k: (float(v[0]), float(v[1])) for k, v in (body.get("bounds") or {}).items()}
        gens = max(1, min(40, int(body.get("gens", 8))))
        pop = max(4, min(40, int(body.get("pop", 12))))
        seeds = max(2, min(30, int(body.get("seeds", 6))))
        w_time = max(0.0, min(2.0, float(body.get("w_time", 0.1))))
        cal = self.cal()

        def fn(job):
            def progress(p, history):
                job["progress"] = p
                if history is not None:
                    job["history"] = list(history)

            out = train.optimise(src, name, names, bounds, base, cal, gens=gens, pop=pop,
                                 seeds=seeds, vary=True, w_time=w_time,
                                 study_seed=int(time.time()) % 9973 + 1,
                                 progress=progress, should_stop=lambda: job["cancel"])
            cfg = {"names": out["names"], "gens": gens, "pop": pop, "seeds": seeds, "w_time": w_time}
            summ = {k: out[k] for k in ("before", "after", "gain", "accepted", "changed", "best", "validation_n")}
            out["id"] = self.store.add("training", name, body.get("label") or "", base, cfg, summ)
            return out

        return self.start_job("training", fn)

    def run_agent(self, body):
        text = str(body.get("text", "")).strip()
        if not text:
            raise ValueError("empty message")
        conv = str(body.get("conv") or "")
        if not conv:
            self.agent.seq += 1
            conv = "c%d" % self.agent.seq
        self.read_fw(body["fw"])                       # fail early on an unknown program

        def fn(job):
            job["history"] = []
            out = self.agent.turn(conv, text, body["fw"], body.get("overrides"), body.get("model"),
                                  job["history"].append, lambda: job["cancel"])
            out["conv"] = conv
            return out

        return self.start_job("agent", fn), conv

    # --------------------------------------------------------------- export
    def export(self, body):
        src, name = self.read_fw(body["fw"])
        table = params.tunables(src)
        over = params.clean(body.get("overrides"), table)
        if not over:
            raise ValueError("nothing to export: every constant equals the file's value")
        by = {t["name"]: t for t in table}
        notes = ", ".join("%s %s -> %s" % (k, params.fmt(by[k]["value"], by[k]["is_int"]),
                                           params.fmt(v, by[k]["is_int"])) for k, v in sorted(over.items()))
        stem = os.path.splitext(name)[0]
        out_name = "%s_trained_%s.py" % (stem, time.strftime("%Y%m%d_%H%M%S"))
        head = ("# Candidate produced by the Simulation twin from %s. Not yet tested on the robot.\n"
                "# Changed constants: %s\n" % (name, notes))
        os.makedirs(self.cand_dir, exist_ok=True)
        path = os.path.join(self.cand_dir, out_name)
        with open(path, "w", encoding="utf-8") as f:
            f.write(head + params.apply_overrides(src, over))
        return {"file": "Simulation/candidates/" + out_name, "changed": notes}


def make_handler(app, web_dir):
    class Handler(BaseHTTPRequestHandler):
        server_version = "RobotTwin/1"

        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype="application/json"):
            data = body if isinstance(body, bytes) else json.dumps(body).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def _body(self):
            n = int(self.headers.get("Content-Length") or 0)
            return json.loads(self.rfile.read(n).decode("utf-8")) if n else {}

        def do_GET(self):
            url = urlparse(self.path)
            q = {k: v[0] for k, v in parse_qs(url.query).items()}
            try:
                if url.path in ("/", "/index.html"):
                    with open(os.path.join(web_dir, "index.html"), "rb") as f:
                        return self._send(200, f.read(), "text/html; charset=utf-8")
                if url.path == "/favicon.ico":
                    return self._send(204, b"", "image/x-icon")
                if url.path == "/api/info":
                    return self._send(200, app.info())
                if url.path == "/api/firmware":
                    src, name = app.read_fw(q["id"])
                    return self._send(200, {"id": q["id"], "name": name, "tunables": params.tunables(src),
                                            "lines": src.count("\n") + 1})
                if url.path == "/api/job":
                    job = app.jobs.get(q.get("id"))
                    if not job:
                        return self._send(404, {"error": "no such job"})
                    return self._send(200, {k: job[k] for k in
                                            ("id", "kind", "status", "progress", "history", "result", "error")})
                if url.path == "/api/agent/status":
                    return self._send(200, {"key": bool(find_key(app.sim_dir)), "models": MODELS,
                                            "default": DEFAULT_MODEL})
                if url.path == "/api/history":
                    return self._send(200, {"items": app.store.history()})
                return self._send(404, {"error": "not found"})
            except Exception as exc:
                return self._send(400, {"error": "%s: %s" % (type(exc).__name__, exc)})

        def do_POST(self):
            url = urlparse(self.path)
            try:
                body = self._body()
                if url.path == "/api/run":
                    src, name = app.read_fw(body["fw"])
                    over = params.clean(body.get("overrides"), params.tunables(src))
                    task = body.get("task", "spin")
                    if task not in ("buzzer", "light", "spin", "random"):
                        task = "spin"
                    res = sim.simulate(src, name, over, seed=int(body.get("seed", 1)), task=task,
                                       vary=bool(body.get("vary")), cal=app.cal(), trace=True)
                    res["overrides"] = over
                    return self._send(200, res)
                if url.path == "/api/trials":
                    return self._send(200, {"job": app.run_trials(body)})
                if url.path == "/api/train":
                    return self._send(200, {"job": app.run_training(body)})
                if url.path == "/api/agent":
                    job, conv = app.run_agent(body)
                    return self._send(200, {"job": job, "conv": conv})
                if url.path == "/api/agent/reset":
                    app.agent.convs.pop(str(body.get("conv")), None)
                    return self._send(200, {"ok": True})
                if url.path == "/api/job/stop":
                    job = app.jobs.get(str(body.get("id")))
                    if job:
                        job["cancel"] = True
                    return self._send(200, {"ok": True})
                if url.path == "/api/export":
                    return self._send(200, app.export(body))
                if url.path == "/api/calibration":
                    if body.get("reset"):
                        config.reset(app.cal_path)
                    else:
                        config.save(app.cal_path, body.get("changes") or {})
                    return self._send(200, app.info())
                if url.path == "/api/history/clear":
                    app.store.clear()
                    return self._send(200, {"ok": True})
                return self._send(404, {"error": "not found"})
            except Exception as exc:
                return self._send(400, {"error": "%s: %s" % (type(exc).__name__, exc)})

    return Handler


def serve(repo, sim_dir, port=8765, host="127.0.0.1"):
    app = App(repo, sim_dir)
    handler = make_handler(app, os.path.join(sim_dir, "web"))
    last = None
    for p in range(port, port + 12):
        try:
            httpd = ThreadingHTTPServer((host, p), handler)
            return httpd, p, app
        except OSError as exc:
            last = exc
    raise last
