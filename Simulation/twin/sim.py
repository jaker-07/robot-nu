"""Run one attempt at the course and score it.

simulate() executes a robot program against the twin and returns what happened:
outcome, timings, wall contacts, centring, the task performed, a score that mirrors the
AE1 rubric (navigation 40 + task 20), and optionally the full trace for playback.
"""
from __future__ import annotations

import math
import traceback

from . import config, params
from .shim import SimStop, World, make_env

OUTCOME_TEXT = {
    "success": "Completed, stopped at the Task Decider, correct task",
    "task_flawed": "Completed and chose the right task, but performed it badly",
    "wrong_task": "Completed, but performed the wrong task",
    "no_task": "Completed and saw the signal, but performed no task",
    "no_flag": "Stopped at the end out of the Task Decider's reach",
    "nav_only": "Completed and stopped at the end; the program has no task stage",
    "stopped_early": "Stopped before the end of the course",
    "stuck": "Wedged against a wall",
    "timeout": "Never stopped: ran out of time",
    "firmware_error": "The robot program raised an error",
}


def simulate(source, name="firmware.py", overrides=None, seed=0, task="spin", vary=True,
             cal=None, t_max=70.0, trace=True):
    if cal is None:
        cal = config.load()[0]
    if overrides:
        source = params.apply_overrides(source, overrides)
    world = World(cal, seed=seed, task=task, vary=vary, t_max=t_max, want_trace=trace)
    error = None
    try:
        exec(compile(source, name, "exec"), make_env(world))
        world.end("program_end")
    except SimStop:
        pass
    except SystemExit:
        world.end("program_end")
    except Exception as exc:
        tb = traceback.extract_tb(exc.__traceback__)
        where = [f for f in tb if f.filename == name]
        line = (" (line %d)" % where[-1].lineno) if where else ""
        error = "%s: %s%s" % (type(exc).__name__, exc, line)
        world.end("firmware_error")
    for _ in range(150):                      # let the wheels run down: 0.6 s
        try:
            world._step()
        except SimStop:
            pass
    return summarise(world, error, seed, trace)


def summarise(w, error, seed, with_trace):
    rb = w.robot
    reached = w.t_stop is not None and w.stop_in_zone
    progress = max(0.0, min(1.0, w.max_progress / (w.track.length - 12.0)))
    nav_time = (w.t_stop - w.t_first_motion) if w.t_stop is not None else None
    cut = w.t_stop + 0.2 if w.t_stop is not None else 1e9
    nav_contacts = [c for c in w.contacts if c["t"] <= cut]
    hard = sum(1 for c in nav_contacts if c["speed"] > 8.0 or c["dur"] > 0.5)
    light = len(nav_contacts) - hard
    off_mean = w.off_sum / w.off_n if w.off_n else 0.0

    buzz_ok = w.buzz_edges >= 100 and w.buzz_t1 - w.buzz_t0 >= 0.3
    light_ok = w.light_time >= 0.5
    spin_deg = math.degrees(rb.th - w.th_stop) if w.th_stop is not None else 0.0
    spin_ok = abs(spin_deg) >= 200.0
    performed = [n for n, ok in (("buzzer", buzz_ok), ("light", light_ok), ("spin", spin_ok)) if ok]
    starts = [t for t, ok in ((w.buzz_t0, buzz_ok), (w.light_t0, light_ok), (w.spin_t0, spin_ok)) if ok and t]
    delay = None
    if w.t_flag_off is not None and starts:
        delay = min(starts) - w.t_flag_off
    spin_err = abs(abs(spin_deg) - 360.0) if "spin" in performed else None

    if error:
        outcome = "firmware_error"
    elif w.t_stop is None:
        outcome = "stuck" if w.stopped == "stuck" else "timeout"
    elif not reached:
        outcome = "stopped_early"
    elif w.t_flag_on is None:
        outcome = "nav_only" if w.stopped == "program_end" else "no_flag"
    elif not performed:
        outcome = "no_task"
    elif performed != [w.task]:
        outcome = "wrong_task"
    elif w.task == "spin" and spin_err > 45.0:
        outcome = "task_flawed"
    else:
        outcome = "success"

    # ---- score mirroring the rubric bands (an estimate, not the marker)
    if not reached:
        nav_pts = 9.0 * progress
    elif hard >= 1 or light >= 3 or off_mean > 0.8:
        nav_pts = 21.0 - min(11.0, 4.0 * max(0, hard - 1) + 2.0 * max(0, light - 3))
    elif light >= 1 or off_mean > 0.5:
        nav_pts = 29.0 - min(7.0, 3.5 * max(0, light - 1) + (2.0 if off_mean > 0.5 else 0.0))
    elif w.hesitations >= 1:
        nav_pts = 35.0 - min(5.0, 2.0 * (w.hesitations - 1))
    else:
        nav_pts = 40.0 - 4.0 * min(1.0, off_mean / 0.5)

    if not reached or w.t_flag_on is None or not performed:
        task_pts = 0.0
    elif performed != [w.task]:
        task_pts = 2.0
    else:
        task_pts = 20.0
        if delay is not None and delay > 3.0:
            task_pts = 12.0
        elif delay is not None and delay > 1.0:
            task_pts = 17.0
        if spin_err is not None:
            if spin_err > 120.0:
                task_pts = min(task_pts, 7.0)
            elif spin_err > 45.0:
                task_pts = min(task_pts, 13.0 - min(3.0, (spin_err - 45.0) / 25.0))
            elif spin_err > 15.0:
                task_pts = min(task_pts, 17.0 - min(3.0, (spin_err - 15.0) / 10.0))

    stop_gap = None
    if w.stop_pose is not None:
        fx = w.stop_pose[0] + rb.sensors["F"][0] * math.cos(w.stop_pose[2])
        fy = w.stop_pose[1] + rb.sensors["F"][0] * math.sin(w.stop_pose[2])
        stop_gap = w.track.s_face - (fx * w.track.end_dir[0] + fy * w.track.end_dir[1])

    r1 = lambda v: None if v is None else round(v, 1)
    out = {
        "outcome": outcome,
        "outcome_text": OUTCOME_TEXT[outcome],
        "success": outcome == "success",
        "score": round(nav_pts + task_pts, 1),
        "nav_pts": round(nav_pts, 1),
        "task_pts": round(task_pts, 1),
        "nav_time": None if nav_time is None else round(nav_time, 2),
        "total_time": round(w.phys_t / 1e6, 2),
        "progress": round(progress, 3),
        "reached": reached,
        "contacts": len(nav_contacts),
        "hard_contacts": hard,
        "contact_time": round(w.contact_time, 2),
        "contact_legs": [w.track.project(c["x"], c["y"])[2] + 1 for c in nav_contacts],
        "off_mean": round(off_mean, 3),
        "off_max": round(w.off_max, 3),
        "hesitations": w.hesitations,
        "pivot_time": round(w.pivot_time, 2),
        "task": w.task,
        "performed": performed,
        "spin_deg": r1(spin_deg) if w.th_stop is not None else None,
        "delay": None if delay is None else round(delay, 2),
        "stop_gap": r1(stop_gap),
        "pings": w.sonar.pings,
        "end": w.stopped,
        "error": error,
        "seed": seed,
        "vary": w.vary,
    }
    if with_trace:
        out["trace"] = w.trace
        out["events"] = w.events
        out["contact_list"] = w.contacts
        out["log"] = w.log[:120]
        out["track"] = w.track.to_json()
    return out
