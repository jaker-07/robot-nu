"""
ultrasonic_test_v4.py   (sensor test protocol, matches navigation_v13b)

Run over USB in Thonny with the battery ON (sensors take 5 V from the regulator).
Pick a test from the menu; the script tells you how to set up, takes 30 readings,
prints PASS / CHECK, and appends the result to sensor_tests.csv on the Pico.

Readings use exactly the same method as navigation_v13b (6 ms timeout,
busy-skip, 55 cm cap), so the results show what the robot actually sees.
  999 = no echo at all (nothing within about 1 m)
   55 = something further than 55 cm, or sensor still busy

Pins (as wired): L TRIG GP16 ECHO GP17 · F TRIG GP18 ECHO GP19 · R TRIG GP20 ECHO GP21
"""

from machine import Pin, time_pulse_us
import time
import os

MAX_CM = 55
NO_ECHO = 999
N = 30                      # readings per test
END_CM = 10                 # same end rule as navigation_v13b
END_CONFIRM = 4

SENSORS = {"L": (16, 17), "F": (18, 19), "R": (20, 21)}
pins = {}
for k, (t, e) in SENSORS.items():
    pins[k] = (Pin(t, Pin.OUT, value=0), Pin(e, Pin.IN))


def read_cm(trig, echo):
    """Same as navigation_v13b."""
    if echo.value() == 1:
        return MAX_CM
    trig.on()
    time.sleep_us(10)
    trig.off()
    pulse = time_pulse_us(echo, 1, 6000)
    if pulse < 0:
        return NO_ECHO
    return min(pulse / 58, MAX_CM)


def stats(vals):
    """(valid %, median, min, max) over readings below the 55 cm cap."""
    good = sorted(v for v in vals if v < MAX_CM)
    pct = 100 * len(good) // len(vals)
    if not good:
        return pct, None, None, None
    return pct, good[len(good) // 2], good[0], good[-1]


def fmt(x):
    return "  -  " if x is None else f"{x:5.1f}"


def save(test, where, setting, vals, verdict):
    new = "sensor_tests.csv" not in os.listdir()
    pct, med, lo, hi = stats(vals)
    no_echo = 100 * sum(1 for v in vals if v == NO_ECHO) // len(vals)
    with open("sensor_tests.csv", "a") as f:
        if new:
            f.write("test,sensor_or_spot,setting,valid_pct,median,min,max,no_echo_pct,verdict\n")
        f.write(f"{test},{where},{setting},{pct},{fmt(med).strip()},{fmt(lo).strip()},{fmt(hi).strip()},{no_echo},{verdict}\n")


def save_note(test, where, setting, verdict):
    new = "sensor_tests.csv" not in os.listdir()
    with open("sensor_tests.csv", "a") as f:
        if new:
            f.write("test,sensor_or_spot,setting,valid_pct,median,min,max,no_echo_pct,verdict\n")
        f.write(f"{test},{where},{setting},,,,,,{verdict}\n")


def take(sensor):
    trig, echo = pins[sensor]
    vals = []
    for _ in range(N):
        vals.append(read_cm(trig, echo))
        time.sleep_ms(40)                   # about one navigation loop
    return vals


def ask_sensor():
    s = input("Sensor (L / F / R): ").strip().upper()
    return s if s in SENSORS else ask_sensor()


def test_distance():
    s = ask_sensor()
    for d in (10, 20, 30):
        input(f"Flat board square-on to {s}, {d} cm from the front of the transducers. Enter when ready")
        vals = take(s)
        pct, med, lo, hi = stats(vals)
        ok = med is not None and abs(med - d) <= 1.0 and hi - lo <= 1.5 and pct >= 95
        verdict = "PASS" if ok else "CHECK"
        print(f"  {s} at {d} cm: valid {pct}%  median {fmt(med)}  min {fmt(lo)}  max {fmt(hi)}  -> {verdict}")
        save("1 distance", s, f"{d} cm", vals, verdict)


def test_angle():
    s = ask_sensor()
    input(f"Board at 45 deg to {s}'s axis (like a corridor wall), 15 cm along the axis. Enter when ready")
    vals = take(s)
    pct, med, lo, hi = stats(vals)
    verdict = "PASS" if pct >= 80 else "CHECK"
    print(f"  {s} at 45 deg: valid {pct}%  median {fmt(med)}  min {fmt(lo)}  max {fmt(hi)}  -> {verdict}")
    print("  (low valid % = the pulse glances off; the robot then holds its last good value)")
    save("2 angle 45", s, "15 cm", vals, verdict)


def test_open():
    s = ask_sensor()
    input(f"Point {s} at open space, nothing within 1 m. Enter when ready")
    vals = take(s)
    short = sum(1 for v in vals if v < 50)
    no_echo = 100 * sum(1 for v in vals if v == NO_ECHO) // len(vals)
    verdict = "PASS" if short == 0 and no_echo >= 90 else "CHECK"
    print(f"  {s} open: no echo {no_echo}%  false short readings {short}  -> {verdict}")
    print("  (false short readings are what can fake a wall or the end)")
    save("3 open space", s, "nothing within 1 m", vals, verdict)


def test_track():
    spot = input("Spot name (e.g. straight, bend1, bend3, stop): ").strip() or "spot"
    input("Robot on the track at that spot, battery on. Enter to record")
    vals = {"L": [], "F": [], "R": []}
    run = best = 0
    for _ in range(N):                      # same order and timing as navigation
        l = read_cm(*pins["L"])
        r = read_cm(*pins["R"])
        f = read_cm(*pins["F"])
        vals["L"].append(l)
        vals["R"].append(r)
        vals["F"].append(f)
        run = run + 1 if f < END_CM and (l == NO_ECHO or r == NO_ECHO) else 0
        best = max(best, run)
        time.sleep_ms(20)
    for s in ("L", "F", "R"):
        pct, med, lo, hi = stats(vals[s])
        no_echo = 100 * sum(1 for v in vals[s] if v == NO_ECHO) // N
        print(f"  {s}: valid {pct}%  median {fmt(med)}  min {fmt(lo)}  max {fmt(hi)}  no echo {no_echo}%")
        save("4 track", f"{spot} {s}", "nav timing", vals[s], "-")
    fires = best >= END_CONFIRM
    print(f"  End rule would {'STOP here' if fires else 'NOT stop here'} (longest run {best} of {END_CONFIRM})")
    save_note("4 track", f"{spot} end-rule", f"longest run {best}", "STOP" if fires else "no stop")


MENU = {"1": test_distance, "2": test_angle, "3": test_open, "4": test_track}
while True:
    print("\n1 distance  2 angle 45  3 open space  4 on track  q quit")
    c = input("Test: ").strip().lower()
    if c == "q":
        break
    if c in MENU:
        MENU[c]()
print("Results saved in sensor_tests.csv")
