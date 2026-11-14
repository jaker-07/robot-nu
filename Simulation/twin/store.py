"""Experiment history in SQLite (Simulation/data/experiments.db).

If the file cannot be used (read-only or sandboxed folder, some synced drives), the app
keeps the history in memory for the session instead of failing, and says so.
"""
from __future__ import annotations

import json
import os
import sqlite3
import threading
import time
from contextlib import contextmanager

SCHEMA = """
    CREATE TABLE IF NOT EXISTS experiments (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        ts REAL NOT NULL, kind TEXT NOT NULL, firmware TEXT NOT NULL,
        label TEXT, overrides TEXT, config TEXT, summary TEXT);
    CREATE TABLE IF NOT EXISTS runs (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        experiment_id INTEGER NOT NULL, seed INTEGER, task TEXT, outcome TEXT,
        score REAL, nav_time REAL, contacts INTEGER, detail TEXT);
    CREATE INDEX IF NOT EXISTS runs_exp ON runs (experiment_id);
"""


class Store:
    def __init__(self, path):
        self.path = path
        self.note = None                 # set when history is not reaching the disk
        self._lock = threading.Lock()
        self._mem = None
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with self._db() as c:
                c.executescript(SCHEMA)
        except (sqlite3.Error, OSError) as exc:
            self._mem = sqlite3.connect(":memory:", check_same_thread=False)
            self.note = "History is kept for this session only: %s could not be written (%s)." % (
                os.path.join("Simulation", "data", os.path.basename(path)), exc)
            with self._db() as c:
                c.executescript(SCHEMA)

    @contextmanager
    def _db(self):
        with self._lock:
            if self._mem is not None:
                with self._mem:
                    yield self._mem
                return
            conn = sqlite3.connect(self.path, timeout=15)
            try:
                with conn:
                    yield conn
            finally:
                conn.close()

    def add(self, kind, firmware, label, overrides, config, summary, runs=()):
        with self._db() as c:
            cur = c.execute(
                "INSERT INTO experiments (ts, kind, firmware, label, overrides, config, summary) "
                "VALUES (?,?,?,?,?,?,?)",
                (time.time(), kind, firmware, label, json.dumps(overrides or {}),
                 json.dumps(config or {}), json.dumps(summary or {})))
            eid = cur.lastrowid
            c.executemany(
                "INSERT INTO runs (experiment_id, seed, task, outcome, score, nav_time, contacts, detail) "
                "VALUES (?,?,?,?,?,?,?,?)",
                [(eid, r.get("seed"), r.get("task"), r.get("outcome"), r.get("score"),
                  r.get("nav_time"), r.get("contacts"),
                  json.dumps({k: v for k, v in r.items() if k != "path"})) for r in runs])
        return eid

    def history(self, limit=60):
        with self._db() as c:
            rows = c.execute(
                "SELECT id, ts, kind, firmware, label, overrides, config, summary FROM experiments "
                "ORDER BY id DESC LIMIT ?", (limit,)).fetchall()
        return [{"id": r[0], "ts": r[1], "kind": r[2], "firmware": r[3], "label": r[4],
                 "overrides": json.loads(r[5] or "{}"), "config": json.loads(r[6] or "{}"),
                 "summary": json.loads(r[7] or "{}")} for r in rows]

    def clear(self):
        with self._db() as c:
            c.execute("DELETE FROM runs")
            c.execute("DELETE FROM experiments")
