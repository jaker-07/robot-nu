"""Repeated trials and parameter training.

trials()    runs one configuration many times, each attempt with a different start
            placement, battery level, motor mismatch, sensor noise and course tolerance.
optimise()  searches the program's own constants for values that score better across
            such varied attempts, then checks the winner against the original on a
            fresh set of attempts it has never seen before recommending it.
"""
from __future__ import annotations

import math
import os
import random
from concurrent.futures import ProcessPoolExecutor

from . import params, sim

TASKS = ("buzzer", "light", "spin")


def run_one(job):
    src, name, overrides, seed, task, vary, cal, want_path = job
    r = sim.simulate(src, name, overrides, seed, task, vary, cal, trace=want_path)
    if want_path:
        tr = r.pop("trace")
        r["path"] = [[tr["x"][i], tr["y"][i]] for i in range(0, len(tr["x"]), 5)]
        for k in ("events", "contact_list", "log", "track"):
            r.pop(k, None)
    return r


def make_pool(workers=None):
    n = workers or max(1, min(8, (os.cpu_count() or 2) - 1))
    if n <= 1:
        return None
    try:
        return ProcessPoolExecutor(max_workers=n)
    except Exception:
        return None


def run_jobs(jobs, pool, tick=None):
    out = []
    if pool is None:
        for j in jobs:
            out.append(run_one(j))
            if tick:
                tick(len(out))
        return out
    try:
        for r in pool.map(run_one, jobs, chunksize=max(1, len(jobs) // 32)):
            out.append(r)
            if tick:
                tick(len(out))
        return out
    except Exception:
        out = []
        for j in jobs:                          # pool failed: fall back to one process
            out.append(run_one(j))
            if tick:
                tick(len(out))
        return out


def aggregate(results, w_time=0.0):
    n = len(results)
    scores = [r["score"] for r in results]
    mean = sum(scores) / n
    sd = math.sqrt(sum((s - mean) ** 2 for s in scores) / n)
    reached = [r for r in results if r["reached"]]
    times = [r["nav_time"] for r in reached]
    outcomes = {}
    for r in results:
        outcomes[r["outcome"]] = outcomes.get(r["outcome"], 0) + 1
    spin = [abs(abs(r["spin_deg"]) - 360.0) for r in results
            if r["task"] == "spin" and "spin" in r["performed"]]
    mean_time = sum(times) / len(times) if times else None
    objective = mean - 0.5 * sd
    if w_time:
        objective -= w_time * (sum(r["nav_time"] if r["reached"] else 60.0 for r in results) / n)
    return {
        "n": n,
        "objective": round(objective, 2),
        "score": round(mean, 1),
        "score_sd": round(sd, 1),
        "success": round(sum(1 for r in results if r["success"]) / n, 3),
        "reached": round(len(reached) / n, 3),
        "clean": round(sum(1 for r in results if r["reached"] and r["contacts"] == 0) / n, 3),
        "nav_time": None if mean_time is None else round(mean_time, 1),
        "contacts": round(sum(r["contacts"] for r in results) / n, 2),
        "hard_contacts": round(sum(r["hard_contacts"] for r in results) / n, 2),
        "off_mean": round(sum(r["off_mean"] for r in results) / n, 3),
        "spin_err": None if not spin else round(sum(spin) / len(spin), 1),
        "outcomes": outcomes,
    }


def trials(src, name, overrides, n, vary, cal, seed0=1, progress=None, pool=None):
    own = pool is None
    if own:
        pool = make_pool()
    try:
        jobs = [(src, name, overrides, seed0 + i, TASKS[i % 3], vary, cal, True) for i in range(n)]
        results = run_jobs(jobs, pool, (lambda k: progress(k / n)) if progress else None)
    finally:
        if own and pool is not None:
            pool.shutdown()
    return {"summary": aggregate(results), "runs": results}


def optimise(src, name, names, bounds, base, cal, gens=8, pop=12, seeds=6, vary=True,
             w_time=0.0, study_seed=1, progress=None, should_stop=None):
    """Evolution-strategy search over the chosen constants.

    Every candidate in a generation faces the same set of attempts, so they are compared
    like for like; each generation uses new attempts, so a setting cannot win by suiting
    one lucky set. Returns the search history and a validation of the best setting
    against the starting one on fresh attempts.
    """
    table = {t["name"]: t for t in params.tunables(src)}
    names = [n for n in names if n in table]
    if not names:
        raise ValueError("no trainable constants selected")
    start = {n: table[n]["value"] for n in table}
    start.update(base or {})
    lo = [float(bounds.get(n, (table[n]["lo"], table[n]["hi"]))[0]) for n in names]
    hi = [float(bounds.get(n, (table[n]["lo"], table[n]["hi"]))[1]) for n in names]
    for i in range(len(names)):
        if hi[i] <= lo[i]:
            hi[i] = lo[i] + 1e-6

    def to_z(values):
        return [min(1.0, max(0.0, (float(values[n]) - lo[i]) / (hi[i] - lo[i]))) for i, n in enumerate(names)]

    def to_overrides(z):
        v = dict(start)
        for i, n in enumerate(names):
            v[n] = lo[i] + z[i] * (hi[i] - lo[i])
        v = params.repair(v)
        return params.clean({k: v[k] for k in set(names) | set(base or {})
                             | {"MIN_SPEED", "MAX_SPEED", "BASE_SPEED"} if k in v}, table.values())

    rng = random.Random(study_seed)
    d = len(names)
    z0 = to_z(start)
    centre = list(z0)
    sigma = [0.2] * d
    best_z = list(z0)
    history = []
    base_over = to_overrides(z0)
    total = gens * pop * seeds + 2 * max(24, 4 * seeds)
    done = [0]
    pool = make_pool()

    def tick(_k):
        done[0] += 1
        if progress:
            progress(min(0.999, done[0] / total), None)

    try:
        for g in range(gens):
            if should_stop and should_stop():
                break
            gen_seeds = [10000 * study_seed + 97 * g + i for i in range(seeds)]
            cands = [list(best_z), list(centre)] if g else [list(z0)]
            while len(cands) < pop:
                cands.append([min(1.0, max(0.0, centre[i] + sigma[i] * rng.gauss(0, 1))) for i in range(d)])
            overs = [to_overrides(z) for z in cands]
            jobs = [(src, name, o, s, TASKS[k % 3], vary, cal, False)
                    for o in overs for k, s in enumerate(gen_seeds)]
            res = run_jobs(jobs, pool, tick)
            aggs = [aggregate(res[i * seeds:(i + 1) * seeds], w_time) for i in range(pop)]
            order = sorted(range(pop), key=lambda i: -aggs[i]["objective"])
            elite = order[:max(2, pop // 4)]
            best_z = list(cands[order[0]])
            weights = [len(elite) - k for k in range(len(elite))]
            wsum = float(sum(weights))
            centre = [sum(weights[k] * cands[e][i] for k, e in enumerate(elite)) / wsum for i in range(d)]
            for i in range(d):
                spread = math.sqrt(sum((cands[e][i] - centre[i]) ** 2 for e in elite) / len(elite))
                sigma[i] = max(0.03, 0.6 * sigma[i] + 0.6 * spread)
            entry = {"gen": g + 1, "best": aggs[order[0]]["objective"],
                     "mean": round(sum(a["objective"] for a in aggs) / pop, 2),
                     "start": aggs[0]["objective"] if g == 0 else None,
                     "best_success": aggs[order[0]]["success"],
                     "best_values": overs[order[0]]}
            history.append(entry)
            if progress:
                progress(min(0.999, done[0] / total), history)

        # validation on attempts no candidate has seen
        n_val = max(24, 4 * seeds)
        val_seeds = [900000 + 31 * study_seed + i for i in range(n_val)]
        best_over = to_overrides(best_z)
        jobs = [(src, name, o, s, TASKS[k % 3], vary, cal, False)
                for o in (base_over, best_over) for k, s in enumerate(val_seeds)]
        res = run_jobs(jobs, pool, tick)
        before = aggregate(res[:n_val], w_time)
        after = aggregate(res[n_val:], w_time)
    finally:
        if pool is not None:
            pool.shutdown()

    gain = round(after["objective"] - before["objective"], 2)
    accepted = gain > 0.5 and after["success"] >= before["success"] - 1e-9
    changed = {}
    for n in sorted(set(best_over) | set(base_over)):
        a = base_over.get(n, table[n]["value"])
        b = best_over.get(n, table[n]["value"])
        if abs(a - b) > 1e-9:
            changed[n] = [a, b]
    return {"history": history, "names": names, "before": before, "after": after, "gain": gain,
            "accepted": accepted, "best": best_over, "base": base_over, "changed": changed,
            "validation_n": n_val}
