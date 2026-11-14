"""Stand-ins for the Pico's `machine`, `time` and `neopixel` modules.

The robot's own .py file is executed unmodified. Every hardware call it makes lands here:
pin writes drive the simulated motors and sensors, and every sleep or timed read moves the
simulated clock forward, which is what steps the physics. Nothing runs in real time, so a
25 s run takes a fraction of a second.
"""
from __future__ import annotations

import builtins
import math
import random
import types

from .config import WIRING
from .robot import Robot, Sonar
from .track import Track

H_US = 4000                      # physics step, microseconds
TICK_STEPS = 5                   # trace / bookkeeping every 5 steps = 20 ms
HOLD_S = {"buzzer": 3.0, "light": 5.0, "spin": 10.0}    # Task Decider flag times (v21 docstring)


class SimStop(BaseException):
    """Ends a run from inside the firmware. Not an Exception, so the robot's own
    try/except blocks cannot swallow it."""
    def __init__(self, reason):
        BaseException.__init__(self, reason)
        self.reason = reason


class World:
    def __init__(self, cal, seed=0, task="spin", vary=True, t_max=70.0, want_trace=True):
        self.cal = cal
        env = random.Random(seed * 7919 + 11)
        if task == "random":
            task = env.choice(("buzzer", "light", "spin"))
        self.task = task
        self.vary = vary
        self.track = Track(cal, env, vary)
        self.robot = Robot(cal, self.track, env, vary, H_US / 1e6)
        self.sonar = Sonar(cal, self.track, self.robot, random.Random(seed * 104729 + 23))
        self.hold = HOLD_S[task] + (env.uniform(-0.2, 0.2) if vary else 0.0)
        self.delay = cal["decider.delay_s"] * (env.uniform(0.7, 1.6) if vary else 1.0)
        self.flag_reach = cal["decider.flag_reach_cm"]
        self.half_span = cal["robot.sensor_span_cm"] / 2.0

        self.t = 0.0                 # firmware clock, us
        self.phys_t = 0.0            # physics clock, us (multiple of H_US)
        self.steps = 0
        self.t_max = t_max
        self.stopped = None          # reason once the run is over

        self.pin_val = {}
        self.pwm_on = set()
        self.pwm_duty = {}
        self.pwm_freq = {}
        self.strip = [(0, 0, 0)] * 5
        self.trig_of = {tr: n for n, (tr, _) in WIRING["sonar"].items()}
        self.echo_of = {ec: n for n, (_, ec) in WIRING["sonar"].items()}
        self.motor_pins = set()
        for m in ("motor_left", "motor_right"):
            self.motor_pins.update(WIRING[m].values())

        # run bookkeeping
        self.t_first_motion = None
        self.zero_since = None
        self.t_stop = None
        self.stop_pose = None
        self.stop_in_zone = False
        self.rest_since = None
        self.flag = "idle"           # idle -> on -> done
        self.t_flag_on = None
        self.t_flag_off = None
        self.stuck_ref = None
        self.contacts = []           # {t, x, y, speed, dur}
        self.contact_time = 0.0
        self._contact_open = False
        self._last_contact_end = -10.0
        self.events = []
        self.log = []
        self.max_progress = 0.0
        self.off_sum = 0.0
        self.off_max = 0.0
        self.off_n = 0
        self.slow_time = 0.0
        self.slow_run = 0.0
        self.hesitations = 0
        self.pivot_time = 0.0
        self.buzz_edges = 0
        self.buzz_t0 = None
        self.buzz_t1 = None
        self.light_time = 0.0
        self.light_t0 = None
        self.spin_t0 = None
        self.th_stop = None
        self.want_trace = want_trace
        self.trace = {k: [] for k in
                      ("t", "x", "y", "th", "ul", "ur", "dl", "df", "dr", "c", "led", "bz", "fl", "ph")}

    # ------------------------------------------------------------------ clock
    def advance(self, us):
        if self.stopped:
            raise SimStop(self.stopped)
        self.t += us
        while self.phys_t + H_US <= self.t:
            self._step()

    def _step(self):
        self.phys_t += H_US
        self.steps += 1
        rb = self.robot
        touching = rb.step()
        if touching:
            self.contact_time += H_US / 1e6
            now = self.phys_t / 1e6
            if not self._contact_open:
                if now - self._last_contact_end > 0.3:
                    self.contacts.append({"t": round(now, 2), "x": round(rb.contact_point[0], 1),
                                          "y": round(rb.contact_point[1], 1),
                                          "speed": round(rb.contact_speed, 1), "dur": 0.0})
                self._contact_open = True
            if self.contacts:
                self.contacts[-1]["dur"] = round(self.contacts[-1]["dur"] + H_US / 1e6, 3)
        elif self._contact_open:
            self._contact_open = False
            self._last_contact_end = self.phys_t / 1e6
        if self.steps % TICK_STEPS == 0:
            self._tick()

    def end(self, reason):
        if not self.stopped:
            self.stopped = reason
            self.events.append({"t": round(self.phys_t / 1e6, 2), "type": "end", "info": reason})

    def _halt(self, reason):
        self.end(reason)
        raise SimStop(reason)

    # ----------------------------------------------------------- 20 ms tick
    def _tick(self):
        t = self.phys_t / 1e6
        rb = self.robot
        dt = H_US * TICK_STEPS / 1e6
        cmd = rb.ul != 0 or rb.ur != 0

        if cmd and self.t_first_motion is None:
            self.t_first_motion = t
            self.events.append({"t": round(t, 2), "type": "go"})
        if self.t_first_motion is not None and self.t_stop is None:
            if cmd:
                self.zero_since = None
            else:
                if self.zero_since is None:
                    self.zero_since = t
                elif t - self.zero_since >= 0.4:
                    self.t_stop = self.zero_since
                    self.stop_pose = (rb.x, rb.y, rb.th)
                    self.th_stop = rb.th
                    self.stop_in_zone = self.front_gap() <= 20.0 and self.in_last_leg()
                    self.events.append({"t": round(self.t_stop, 2), "type": "stop",
                                        "info": "at Task Decider" if self.stop_in_zone else "early"})

        # navigation quality, only while driving the course
        if self.t_first_motion is not None and self.t_stop is None:
            s, lat, leg = self.track.project(*rb.world_point(-1.5, 0.0))
            if s > self.max_progress:
                self.max_progress = s
            half = self.track.widths[min(leg, len(self.track.widths) - 1)] / 2.0
            ratio = min(1.0, abs(lat) / max(1.0, half - self.half_span))   # share of the free clearance used
            self.off_sum += ratio
            self.off_n += 1
            if ratio > self.off_max:
                self.off_max = ratio
            if cmd and abs(rb.speed) < 5.0 and t - self.t_first_motion > 0.6:
                self.slow_run += dt
                self.slow_time += dt
                if self.slow_run >= 0.3 and self.slow_run - dt < 0.3:
                    self.hesitations += 1
            else:
                self.slow_run = 0.0
            if (rb.ul == 0) != (rb.ur == 0):
                self.pivot_time += dt

        # wedged against something
        if cmd:
            ref = self.stuck_ref
            if ref is None or math.hypot(rb.x - ref[1], rb.y - ref[2]) > 1.5 or abs(rb.th - ref[3]) > 0.14:
                self.stuck_ref = (t, rb.x, rb.y, rb.th)
            elif t - ref[0] > 4.0:
                self.end("stuck")
        else:
            self.stuck_ref = None

        # Task Decider
        at_rest = abs(rb.vl) < 0.3 and abs(rb.vr) < 0.3
        if self.flag == "idle":
            if at_rest and self.t_first_motion is not None and self.ir_in_zone():
                if self.rest_since is None:
                    self.rest_since = t
                elif t - self.rest_since >= self.delay:
                    self.flag = "on"
                    self.t_flag_on = t
                    self.events.append({"t": round(t, 2), "type": "flag_on",
                                        "info": "%s signal, %.1f s" % (self.task, self.hold)})
            else:
                self.rest_since = None
        elif self.flag == "on" and t - self.t_flag_on >= self.hold:
            self.flag = "done"
            self.t_flag_off = t
            self.events.append({"t": round(t, 2), "type": "flag_off"})

        # task activity (anything after the stop counts)
        lit = any(c != (0, 0, 0) for c in self.strip)
        if self.t_stop is not None:
            if lit:
                if self.light_t0 is None:
                    self.light_t0 = t
                self.light_time += dt
            if self.spin_t0 is None and cmd:
                self.spin_t0 = t

        # end conditions
        if t >= self.t_max:
            self.end("timeout")
        elif self.flag == "done" and t - self.t_flag_off > 8.0:
            self.end("no_task")
        elif self.t_stop is not None and not self.stop_in_zone and t - self.t_stop > 2.0:
            self.end("stopped_early")
        elif (self.flag == "idle" and self.t_stop is not None and at_rest
              and not self.ir_in_zone() and t - self.t_stop > 6.0):
            self.end("no_flag")

        if self.want_trace:
            tr = self.trace
            tr["t"].append(round(t, 2))
            tr["x"].append(round(rb.x, 2))
            tr["y"].append(round(rb.y, 2))
            tr["th"].append(round(rb.th, 4))
            tr["ul"].append(round(rb.ul, 1))
            tr["ur"].append(round(rb.ur, 1))
            last = self.sonar.last
            tr["dl"].append(round(last["L"], 1))
            tr["df"].append(round(last["F"], 1))
            tr["dr"].append(round(last["R"], 1))
            tr["c"].append(1 if rb.in_contact else 0)
            g = max(self.strip, key=lambda c: c[0] + c[1] + c[2])
            tr["led"].append((g[0] << 16) | (g[1] << 8) | g[2])
            tr["bz"].append(1 if self.buzzing(t) else 0)
            tr["fl"].append(1 if self.flag == "on" else 0)
            tr["ph"].append(self.phase())

        if self.stopped:
            raise SimStop(self.stopped)

    # -------------------------------------------------------------- queries
    def phase(self):
        if self.t_first_motion is None:
            return 0
        if self.t_stop is None:
            return 1
        if self.flag != "done":
            return 2
        return 3

    def front_gap(self):
        """Distance from the front sensor face to the Task Decider's face plane."""
        fx, fy = self.robot.world_point(self.robot.sensors["F"][0], 0.0)
        d = self.track.end_dir
        return self.track.s_face - (fx * d[0] + fy * d[1])

    def in_last_leg(self):
        return self.track.project(self.robot.x, self.robot.y)[2] == len(self.track.widths) - 1

    def ir_in_zone(self):
        """Can the IR sensor on the mast see the Decider's flag from here?"""
        rb, tk = self.robot, self.track
        px, py = rb.world_point(rb.ir_x, 0.0)
        d, n = tk.end_dir, tk.end_normal
        ahead = tk.s_face - (px * d[0] + py * d[1])
        across = (px - tk.face_a[0]) * n[0] + (py - tk.face_a[1]) * n[1]
        facing = math.cos(rb.th) * d[0] + math.sin(rb.th) * d[1]
        return 0.0 <= ahead <= self.flag_reach and 0.0 <= across <= tk.face_width and facing > 0.77

    def buzzing(self, t):
        if self.buzz_t1 is not None and t - self.buzz_t1 < 0.05:
            return True
        pin = WIRING["buzzer"]
        return pin in self.pwm_on and self.pwm_duty.get(_pwm_key(pin), 0) > 0

    # ------------------------------------------------------------- hardware
    def pin_write(self, pid, v):
        v = 1 if v else 0
        old = self.pin_val.get(pid, 0)
        self.advance(15)
        self.pin_val[pid] = v
        if pid in self.trig_of:
            if old == 1 and v == 0:
                self.sonar.trigger(self.trig_of[pid], self.t)
        elif pid in self.motor_pins:
            self.update_motors()
        elif pid == WIRING["buzzer"] and v == 1 and old == 0 and self.t_stop is not None:
            now = self.t / 1e6
            self.buzz_edges += 1
            if self.buzz_t0 is None:
                self.buzz_t0 = now
            self.buzz_t1 = now

    def pin_read(self, pid):
        self.advance(15)
        if pid in self.echo_of:
            return self.sonar.echo_level(self.echo_of[pid], self.t)
        if pid == WIRING["ir"]:
            return 0 if (self.flag == "on" and self.ir_in_zone()) else 1
        return self.pin_val.get(pid, 0)

    def _motor_u(self, m):
        en = m["en"]
        if en in self.pwm_on:
            duty = self.pwm_duty.get(_pwm_key(en), 0) / 65535.0 * 100.0
        else:
            duty = 100.0 if self.pin_val.get(en, 0) else 0.0
        return (self.pin_val.get(m["fwd"], 0) - self.pin_val.get(m["rev"], 0)) * duty

    def update_motors(self):
        self.robot.ul = self._motor_u(WIRING["motor_left"])
        self.robot.ur = self._motor_u(WIRING["motor_right"])

    def pulse_us(self, pid, level, timeout):
        """machine.time_pulse_us: -2 = timed out waiting for the pulse, -1 = pulse too long."""
        name = self.echo_of.get(pid)
        win = self.sonar.window[name] if name else None
        if win is None or level != 1:
            self.advance(timeout)
            return -2
        rise, fall = win
        t0 = self.t
        if rise <= t0 < fall:
            start = t0
        elif rise > t0 and rise - t0 <= timeout:
            self.advance(rise - t0)
            start = rise
        else:
            self.advance(timeout)
            return -2
        dur = fall - start
        if dur > timeout:
            self.advance(timeout)
            return -1
        self.advance(dur)
        return int(dur)


def _pwm_key(pid):
    return ((pid >> 1) & 7, pid & 1)        # RP2040: (slice, channel); GP10 and GP26 share one


def make_modules(world):
    """Build the fake machine / time / neopixel modules bound to one World."""

    class Pin:
        IN, OUT, OPEN_DRAIN, ALT = 0, 1, 2, 3
        PULL_UP, PULL_DOWN = 1, 2
        IRQ_RISING, IRQ_FALLING = 1, 2

        def __init__(self, id, mode=-1, pull=-1, value=None, **kw):
            self.id = WIRING["led"] if id == "LED" else int(id)
            if value is not None:
                world.pin_write(self.id, value)

        def init(self, mode=-1, pull=-1, value=None, **kw):
            if value is not None:
                world.pin_write(self.id, value)

        def value(self, v=None):
            if v is None:
                return world.pin_read(self.id)
            world.pin_write(self.id, v)

        __call__ = value

        def on(self):
            world.pin_write(self.id, 1)

        def off(self):
            world.pin_write(self.id, 0)

        high, low = on, off

        def toggle(self):
            world.pin_write(self.id, 0 if world.pin_val.get(self.id, 0) else 1)

        def irq(self, *a, **kw):
            raise NotImplementedError("Pin.irq is not simulated")

    class PWM:
        def __init__(self, pin, freq=None, duty_u16=None, **kw):
            self.id = pin.id if hasattr(pin, "id") else int(pin)
            self.key = _pwm_key(self.id)
            world.pwm_on.add(self.id)
            if freq is not None:
                self.freq(freq)
            if duty_u16 is not None:
                self.duty_u16(duty_u16)

        def freq(self, f=None):
            if f is None:
                return world.pwm_freq.get(self.key[0], 0)
            world.pwm_freq[self.key[0]] = f

        def duty_u16(self, v=None):
            if v is None:
                return world.pwm_duty.get(self.key, 0)
            world.advance(20)
            world.pwm_duty[self.key] = max(0, min(65535, int(v)))
            world.update_motors()
            if self.id == WIRING["buzzer"] and v > 0 and world.t_stop is not None:
                now = world.t / 1e6
                f = world.pwm_freq.get(self.key[0], 0)
                if world.buzz_t0 is None:
                    world.buzz_t0 = now
                world.buzz_t1 = now
                world.buzz_edges += max(1, int(f * 0.02))

        def duty_ns(self, v=None):
            f = world.pwm_freq.get(self.key[0], 1000) or 1000
            if v is None:
                return int(self.duty_u16() / 65535.0 * 1e9 / f)
            self.duty_u16(int(v * f / 1e9 * 65535))

        def deinit(self):
            world.pwm_on.discard(self.id)
            world.update_motors()

    def time_pulse_us(pin, level, timeout_us=1000000):
        return world.pulse_us(pin.id, level, timeout_us)

    machine = types.ModuleType("machine")
    machine.Pin, machine.PWM, machine.time_pulse_us = Pin, PWM, time_pulse_us
    machine.freq = lambda *a: 125000000
    machine.reset = lambda: world._halt("program_end")
    machine.idle = lambda: world.advance(100)

    def sleep(s):
        world.advance(s * 1e6 + 20)

    def sleep_ms(ms):
        world.advance(ms * 1000 + 20)

    def sleep_us(us):
        world.advance(us + 20)

    def ticks_ms():
        world.advance(10)
        return int(world.t // 1000)

    def ticks_us():
        world.advance(10)
        return int(world.t)

    tm = types.ModuleType("time")
    tm.sleep, tm.sleep_ms, tm.sleep_us = sleep, sleep_ms, sleep_us
    tm.ticks_ms, tm.ticks_us, tm.ticks_cpu = ticks_ms, ticks_us, ticks_us
    tm.ticks_diff = lambda a, b: a - b
    tm.ticks_add = lambda a, b: a + b
    tm.time = lambda: world.t / 1e6
    tm.time_ns = lambda: int(world.t * 1000)

    class NeoPixel:
        def __init__(self, pin, n, bpp=3, timing=1):
            self.pin = pin.id if hasattr(pin, "id") else int(pin)
            self.n = n
            self.buf = [(0, 0, 0)] * n

        def __len__(self):
            return self.n

        def __setitem__(self, i, c):
            self.buf[i] = tuple(int(v) for v in c[:3])

        def __getitem__(self, i):
            return self.buf[i]

        def fill(self, c):
            self.buf = [tuple(int(v) for v in c[:3])] * self.n

        def write(self):
            world.advance(200)
            if self.pin == WIRING["strip"]:
                world.strip = list(self.buf)

    neo = types.ModuleType("neopixel")
    neo.NeoPixel = NeoPixel

    micropython = types.ModuleType("micropython")
    micropython.const = lambda x: x

    return {"machine": machine, "time": tm, "utime": tm, "neopixel": neo, "micropython": micropython}


class _NullFile:
    """Files the firmware opens (CSV logs on the Pico) go nowhere."""
    def write(self, s):
        return len(s)

    def read(self, *a):
        return ""

    def readline(self, *a):
        return ""

    def flush(self):
        pass

    def close(self):
        pass

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False

    def __iter__(self):
        return iter(())


_REAL_OK = {"math", "random", "gc", "sys", "struct", "array", "collections", "json", "os", "re", "ustruct", "ujson"}


def make_env(world):
    """Globals for exec(): the firmware's imports resolve to the fakes above."""
    mods = make_modules(world)
    real_import = builtins.__import__

    def fake_import(name, globals=None, locals=None, fromlist=(), level=0):
        if name in mods:
            return mods[name]
        if name in _REAL_OK:
            if name == "os":
                dummy = types.ModuleType("os")
                dummy.listdir = lambda *a: []
                dummy.remove = lambda *a: None
                return dummy
            return real_import(name.lstrip("u") if name in ("ustruct", "ujson") else name,
                               globals, locals, fromlist, level)
        raise ImportError("no module named '%s' in the simulated Pico" % name)

    def fake_print(*args, **kw):
        world.advance(150)
        if len(world.log) < 300:
            world.log.append(" ".join(str(a) for a in args))

    def fake_input(*a):
        raise EOFError("input() is not available in the simulator")

    b = dict(vars(builtins))
    b["__import__"] = fake_import
    b["print"] = fake_print
    b["input"] = fake_input
    b["open"] = lambda *a, **kw: _NullFile()
    return {"__name__": "__main__", "__builtins__": b}
