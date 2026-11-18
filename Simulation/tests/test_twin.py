"""Checks for the twin. Run from the Simulation folder:

    python3 -m unittest discover -s tests -v
"""
import glob
import json
import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.dirname(HERE)
sys.path.insert(0, SIM)

from twin import config, params, sim              # noqa: E402
from twin.shim import World, make_modules         # noqa: E402
from twin.track import Track                      # noqa: E402

CAL = config.defaults()

STRAIGHT = """
from machine import Pin, PWM
import time
POWER = 40
left = (PWM(Pin(10)), Pin(12, Pin.OUT), Pin(11, Pin.OUT))
right = (PWM(Pin(15)), Pin(13, Pin.OUT), Pin(14, Pin.OUT))
for en, fwd, rev in (left, right):
    fwd.on(); rev.off()
left[0].duty_u16(int(POWER * 655))
right[0].duty_u16(int(POWER * 0.92 * 655))
time.sleep(2)
left[0].duty_u16(0)
right[0].duty_u16(0)
time.sleep(1)
"""


class TrackTest(unittest.TestCase):
    def test_default_course_is_30_cm_throughout(self):
        tk = Track(CAL)
        self.assertEqual([round(w, 1) for w in tk.widths], [30.0] * 5)
        self.assertEqual([(round(x), round(y)) for x, y in tk.outer[:3]], [(0, 0), (0, 80), (93, 80)])   # outer wall as in the brief

    def test_brief_lengths_give_a_varying_width(self):
        cal = dict(CAL)
        cal["track.uniform_width_cm"] = 0.0
        self.assertEqual([round(w, 1) for w in Track(cal).widths], [30.0, 30.0, 25.4, 23.1, 35.8])


class GeometryTest(unittest.TestCase):
    def test_plate_matches_the_cad_sketch(self):
        from twin.robot import geometry
        g = geometry(CAL)
        xs = [x for x, _ in g["plate"]]
        ys = [y for _, y in g["plate"]]
        self.assertAlmostEqual(max(xs) - min(xs), 17.2, places=6)
        self.assertAlmostEqual(max(ys) - min(ys), 15.0, places=6)
        self.assertAlmostEqual(g["chamfer_deg"], 46.6, delta=0.1)        # 133.4 degree front corners
        rear = g["plate"][2], g["plate"][3]
        rear_deg = math.degrees(math.atan2(rear[0][1] - rear[1][1], rear[0][0] - rear[1][0]))
        self.assertAlmostEqual(rear_deg, 30.0, delta=0.1)                  # 120 degree rear corners
        self.assertGreaterEqual(g["half_width"], 7.5)                       # outline contains the plate


class ParamsTest(unittest.TestCase):
    SRC = 'KP = 1.5             # gain\nEND_CONFIRM = 3\nname = "x"\nOFF = (0, 0, 0)\n'

    def test_finds_only_numeric_constants(self):
        self.assertEqual([t["name"] for t in params.tunables(self.SRC)], ["KP", "END_CONFIRM"])

    def test_override_changes_only_the_number(self):
        out = params.apply_overrides(self.SRC, {"KP": 2.25, "END_CONFIRM": 4.4})
        self.assertEqual(out, self.SRC.replace("1.5", "2.25").replace("= 3", "= 4"))

    def test_no_override_returns_source_untouched(self):
        self.assertIs(params.apply_overrides(self.SRC, {}), self.SRC)


class ShimTest(unittest.TestCase):
    def test_pulse_timing_matches_distance(self):
        w = World(CAL, seed=1, vary=False)
        m = make_modules(w)
        trig, echo = m["machine"].Pin(18, 1, value=0), m["machine"].Pin(19, 0)
        trig.on(); m["time"].sleep_us(10); trig.off()
        us = m["machine"].time_pulse_us(echo, 1, 30000)
        true_cm = w.sonar.last["F"]
        self.assertGreater(true_cm, 20)
        self.assertAlmostEqual(us / 58.31, true_cm, delta=0.1)

    def test_unwired_pin_times_out(self):
        w = World(CAL, seed=1, vary=False)
        m = make_modules(w)
        self.assertEqual(m["machine"].time_pulse_us(m["machine"].Pin(27, 0), 1, 6000), -2)

    def test_gp26_and_gp10_share_a_pwm_channel(self):
        w = World(CAL, seed=1, vary=False)
        m = make_modules(w)
        ena = m["machine"].PWM(m["machine"].Pin(10))
        m["machine"].Pin(12, 1).on()
        buzzer = m["machine"].PWM(m["machine"].Pin(26))
        buzzer.duty_u16(30000)
        self.assertEqual(ena.duty_u16(), 30000)          # the v20d buzzer fault
        self.assertGreater(w.robot.ul, 40)


class SimTest(unittest.TestCase):
    def test_straight_run_distance(self):
        r = sim.simulate(STRAIGHT, "straight.py", seed=3, vary=False, cal=CAL, trace=True)
        tr = r["trace"]
        dist = tr["y"][-1] - tr["y"][0]
        expect = CAL["motor.speed_gain"] * (40 - CAL["motor.dead_run_pct"]) * 2.0
        self.assertAlmostEqual(dist, expect, delta=0.15 * expect)
        self.assertAlmostEqual(tr["x"][-1], tr["x"][0], delta=1.0)     # trimmed: drives straight
        self.assertIsNone(r["error"])

    def test_old_programs_run_on_the_build_they_were_written_for(self):
        old = STRAIGHT.replace("Pin(13, Pin.OUT), Pin(14, Pin.OUT)", "Pin(14, Pin.OUT), Pin(13, Pin.OUT)")   # right forward = IN4
        for src, build in ((STRAIGHT, "now"), (old, "nov10")):
            r = sim.simulate(src, "s.py", seed=1, vary=False, cal=CAL, trace=True)
            self.assertEqual(r["hardware"], build)
            self.assertGreater(r["trace"]["y"][-1] - r["trace"]["y"][0], 30)        # drives forwards on its own build
        wrong = sim.simulate(old, "s.py", seed=1, vary=False, cal=CAL, trace=True, hardware="now")
        self.assertLess(abs(wrong["trace"]["y"][-1] - wrong["trace"]["y"][0]), 15)   # on today's robot it just turns

    def test_unmeasured_values_vary_between_attempts_only_when_asked(self):
        cal = config.with_uncertainty(config.defaults(), {k: s for k, _, s, _ in config.CAL_ENTRIES})
        keep = dict(cal)
        a = sim.simulate(STRAIGHT, "s.py", seed=1, vary=True, cal=cal, trace=True)
        b = sim.simulate(STRAIGHT, "s.py", seed=2, vary=True, cal=cal, trace=True)
        self.assertNotEqual(a["trace"]["y"][-1], b["trace"]["y"][-1])
        self.assertEqual(cal, keep)                                                  # the shared table is not altered
        c = sim.simulate(STRAIGHT, "s.py", seed=1, vary=False, cal=cal, trace=True)
        d = sim.simulate(STRAIGHT, "s.py", seed=2, vary=False, cal=cal, trace=True)
        self.assertEqual(c["trace"]["y"][-1], d["trace"]["y"][-1])

    def test_same_seed_same_result(self):
        a = sim.simulate(STRAIGHT, "s.py", seed=5, vary=True, cal=CAL, trace=False)
        b = sim.simulate(STRAIGHT, "s.py", seed=5, vary=True, cal=CAL, trace=False)
        self.assertEqual(a, b)

    def test_program_error_is_reported_not_raised(self):
        r = sim.simulate("import time\ntime.sleep(0.1)\nboom()\n", "bad.py", cal=CAL, trace=False)
        self.assertEqual(r["outcome"], "firmware_error")
        self.assertIn("line 3", r["error"])

    def test_latest_robot_program_runs(self):
        mains = glob.glob(os.path.join(os.environ.get("ROBOT_REPO", os.path.dirname(SIM)), "main_v*.py"))
        if not mains:
            self.skipTest("no main_v*.py beside the Simulation folder")
        path = max(mains, key=lambda p: int("".join(c for c in os.path.basename(p) if c.isdigit()) or 0))
        with open(path, encoding="utf-8") as f:
            src = f.read()
        r = sim.simulate(src, os.path.basename(path), seed=1, task="light", vary=False, cal=CAL, trace=False)
        self.assertIsNone(r["error"])
        self.assertGreater(r["progress"], 0.5)


class StoreTest(unittest.TestCase):
    def test_round_trip_and_fallback_when_disk_unusable(self):
        import tempfile
        from twin.store import Store
        with tempfile.TemporaryDirectory() as tmp:
            st = Store(os.path.join(tmp, "data", "x.db"))
            st.add("trials", "main.py", "", {"KP": 2}, {"n": 1}, {"n": 1}, [{"seed": 1, "outcome": "success"}])
            self.assertIsNone(st.note)
            self.assertEqual(Store(os.path.join(tmp, "data", "x.db")).history()[0]["overrides"], {"KP": 2})
            blocker = os.path.join(tmp, "file")
            open(blocker, "w").close()
            bad = Store(os.path.join(blocker, "data", "x.db"))        # a file where a folder is needed
            self.assertIsNotNone(bad.note)
            bad.add("trials", "main.py", "", {}, {}, {"n": 1})
            self.assertEqual(len(bad.history()), 1)
            bad._mem.close()


class AgentTest(unittest.TestCase):
    """The tool loop, with the HTTP call replaced by a scripted reply."""

    def _app(self, tmp):
        from twin.server import App
        repo = os.path.join(tmp, "repo")
        os.makedirs(os.path.join(repo, "Simulation"))
        with open(os.path.join(repo, "main_v1.py"), "w") as f:
            f.write("POWER = 40\n" + STRAIGHT.replace("POWER = 40\n", ""))
        return App(repo, os.path.join(repo, "Simulation"))

    def test_tool_round_trip_and_message_shape(self):
        import tempfile
        from twin.agent import Agent
        with tempfile.TemporaryDirectory() as tmp:
            app = self._app(tmp)
            os.environ["ANTHROPIC_API_KEY"] = "test-key"
            sent = []
            script = [
                {"content": [{"type": "thinking", "thinking": "plan", "signature": "s"},
                             {"type": "text", "text": "Running it."},
                             {"type": "tool_use", "id": "toolu_1", "name": "run_attempt", "input": {"overrides": {"POWER": 45}}},
                             {"type": "tool_use", "id": "toolu_2", "name": "no_such_tool", "input": {}}],
                 "stop_reason": "tool_use"},
                {"content": [{"type": "text", "text": "Done."}], "stop_reason": "end_turn"},
            ]

            def transport(body, key):
                sent.append(json.loads(json.dumps(body)))
                return script[len(sent) - 1]

            steps = []
            agent = Agent(app, transport)
            out = agent.turn("c1", "try 45", "main_v1.py", {}, "claude-sonnet-5-5", steps.append, lambda: False)
            self.assertEqual(out["reply"], "Running it.\n\nDone.")
            self.assertEqual(sent[0]["model"], "claude-sonnet-5-5")
            self.assertEqual(sent[0]["messages"], [{"role": "user", "content": "try 45"}])
            second = sent[1]["messages"]
            self.assertEqual(second[1]["content"][0]["type"], "thinking")        # assistant turn kept verbatim
            results = second[2]["content"]
            self.assertEqual([b["type"] for b in results], ["tool_result", "tool_result"])
            self.assertEqual([b["tool_use_id"] for b in results], ["toolu_1", "toolu_2"])
            self.assertEqual(json.loads(results[0]["content"])["overrides"], {"POWER": 45})
            self.assertTrue(results[1]["is_error"])
            self.assertEqual([s["kind"] for s in steps], ["text", "tool", "result", "tool", "result", "text"])

    def test_failed_request_leaves_conversation_unchanged(self):
        import tempfile
        from twin.agent import Agent, AgentError
        with tempfile.TemporaryDirectory() as tmp:
            app = self._app(tmp)
            os.environ["ANTHROPIC_API_KEY"] = "test-key"

            def transport(body, key):
                raise AgentError("Anthropic API error 401")

            agent = Agent(app, transport)
            with self.assertRaises(AgentError):
                agent.turn("c1", "hello", "main_v1.py", {}, None, lambda s: None, lambda: False)
            self.assertEqual(agent.convs["c1"], [])

    def test_code_edit_must_match_once_and_compile(self):
        import tempfile
        from twin.agent import Agent
        with tempfile.TemporaryDirectory() as tmp:
            app = self._app(tmp)
            agent = Agent(app, lambda b, k: {})
            ctx = agent._ctx("main_v1.py", {})
            r = agent.t_propose_code_edit(ctx, {"edits": [{"old_text": "time.sleep", "new_text": "x"}], "note": "n"})
            self.assertFalse(r["saved"])
            self.assertIn("occurs 2 times", r["reason"])
            r = agent.t_propose_code_edit(ctx, {"edits": [{"old_text": "time.sleep(2)", "new_text": "time.sleep(2"}], "note": "n"})
            self.assertIn("Syntax error", r["reason"])
            self.assertFalse(os.path.exists(app.cand_dir))


if __name__ == "__main__":
    unittest.main()
