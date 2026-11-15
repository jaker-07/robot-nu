#!/usr/bin/env python3
"""Start the robot twin.

    python3 run.py            open the app in your browser
    python3 run.py --check    run main_vNN.py once through the twin and print the result
    python3 run.py --history  compare the twin with every result in the testing log

Needs Python 3.8 or newer and nothing else.
"""
import argparse
import os
import sys
import webbrowser

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from twin import config, sim, train  # noqa: E402
from twin.server import App, serve   # noqa: E402


def check(repo):
    app = App(repo, HERE)
    fw = app.firmware()
    if not fw:
        print("No main_v*.py found in", repo)
        return 1
    src, name = app.read_fw(fw[0]["id"])
    cal = app.cal()
    if app.store.note:
        print("Note:", app.store.note)
    print("Program: %s   (repo: %s)" % (name, repo))
    for task in train.TASKS:
        r = sim.simulate(src, name, seed=1, task=task, vary=False, cal=cal, trace=False)
        print("  %-6s -> %-9s drive %5.1f s, %d contact(s), stopped %s cm from the Decider, "
              "did %s, score %.0f/60" % (task, r["outcome"], r["nav_time"] or 0, r["contacts"],
                                         r["stop_gap"], "+".join(r["performed"]) or "nothing", r["score"]))
    s = train.trials(src, name, {}, 30, True, cal)["summary"]
    print("  30 varied attempts: %.0f%% success, %.0f%% without wall contact, mean score %.1f"
          % (100 * s["success"], 100 * s["clean"], s["score"]))
    return 0


def history_check(repo):
    from twin import history
    app = App(repo, HERE)
    rows = history.check(history.finder(repo), app.cal(), 12)
    bad = 0
    for r in rows:
        if not r["available"]:
            print("  ----  %-20s file not found" % r["program"])
            continue
        bad += 0 if r["agrees"] else 1
        print("  %-5s %-20s twin: reaches the end %3.0f%%, stops there %3.0f%%, stops early %3.0f%%, "
              "%.1f contacts/run  [%s build]" % ("OK" if r["agrees"] else "DIFF", r["program"], 100 * r["reaches"],
                                                 100 * r["stop_end"], 100 * r["stop_early"], r["contacts"],
                                                 "/".join(r["hardware"])))
        print("        recorded: %s" % r["record"])
    print("%d of %d recorded results reproduced." % (sum(1 for r in rows if r.get("agrees")),
                                                    sum(1 for r in rows if r["available"])))
    return 0


def main():
    ap = argparse.ArgumentParser(description="GE1501 AE1 robot twin")
    ap.add_argument("--repo", default=os.path.dirname(HERE),
                    help="folder holding main_vNN.py (default: the folder above Simulation)")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--no-browser", action="store_true")
    ap.add_argument("--check", action="store_true", help="run a self-check and exit")
    ap.add_argument("--history", action="store_true",
                    help="compare the twin with every result recorded in the testing log and exit")
    args = ap.parse_args()
    if args.check:
        return check(args.repo)
    if args.history:
        return history_check(args.repo)
    httpd, port, app = serve(args.repo, HERE, args.port)
    if app.store.note:
        print("Note:", app.store.note)
    url = "http://127.0.0.1:%d/" % port
    print("Robot twin running at %s   (Ctrl+C to stop)" % url)
    if not args.no_browser:
        try:
            webbrowser.open(url)
        except Exception:
            pass
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
