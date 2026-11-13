"""
navigation_v13b.py

Changes from v13a (v13a stopped at a corner, taking the wall for the end):
- Pivot direction now comes from the CURRENT raw readings, not the held
  values, and stays locked until the front is clear (the v8a fix). In v13a
  the spike filter could still be holding the old value of the side that had
  just opened, so the robot could pivot into the wall and stay close enough
  to trigger the end check.
- The end check now needs a side with NO echo at all (nothing within about
  1 m: open room past the end of the course), not just a reading past 55 cm.
  At a corner the open side still sees the far wall, so it no longer counts.
  read_cm now returns NO_ECHO (999) for a true miss and MAX_CM (55) for a
  far-but-seen reading; steering treats both the same as before.
- Each change has its own switch (PIVOT_FROM_RAW, END_NEEDS_NO_ECHO) so they
  can be tested one at a time.

Changes in v13a from v13_test:
- Left sensor pins changed to match the wiring diagram: TRIG GP16, ECHO GP17
  (were GP28/GP27, which nothing is wired to, so the left sensor was never read).

Changes in v13_test from working10_11_2026.py (navigation otherwise unchanged):
- End check starts further out: front < END_CM (10 cm, was 5) AND a side open,
  for END_CONFIRM (4) loops in a row. In the logged run, corners never kept the
  front under 10 cm for more than 2 loops; the end held it for 4.
- Side spike filter: a side reading that jumps more than JUMP_CM further away
  than the last good value must repeat JUMP_CONFIRM (3) loops in a row before
  steering uses it. One-off high readings no longer make it swerve.
  JUMP_CONFIRM = 1 turns the filter off (same as before).
- Run log: every loop is written to run_N.csv on the Pico, plus
  run_N_summary.txt when it stops itself. LOG = False turns logging off.

Wiring:
  L298N         ENA GP10, IN1 GP11, IN2 GP12, IN3 GP13, IN4 GP14, ENB GP15
  Ultrasonic L  TRIG GP16 (pin 21), ECHO GP17 (pin 22)
  Ultrasonic F  TRIG GP18 (pin 24), ECHO GP19 (pin 25)
  Ultrasonic R  TRIG GP20 (pin 26), ECHO GP21 (pin 27)
ENA/ENB jumper caps must be OFF. No echo dividers fitted (ECHO goes straight to the Pico).
"""

from machine import Pin, PWM, time_pulse_us
import time
import math
import os

VERSION = "v13b"

# PIN NUMBERS (GP) 
L_TRIG = 16
L_ECHO = 17
F_TRIG = 18
F_ECHO = 19
R_TRIG = 20
R_ECHO = 21

# TUNING  
BASE_SPEED = 45      # % power driving straight 
MIN_SPEED = 30       # % lowest power sent to a motor 
MAX_SPEED = 50      # % highest power sent to a motor
LEFT_TRIM = 1.0
RIGHT_TRIM = 0.9     # right motor is stronger (from trim_test_v1.py)
KP = 3              # % power change per cm of centring error
DEADBAND = 1         # cm: no correction while within +/-2 cm of centre
MAX_ERR = 8          # cm: largest error acted on
FRONT_TURN_CM = 15   # cm: wall closer than this ahead -> sharp turn
MAX_CM = 55          # readings at or above this count as a missed echo
END_CM = 10          # v13: cm: front closer than this = possible end (was STOP_CM = 5)
END_CONFIRM = 4      # v13: loops in a row front < END_CM AND a side open before stopping
JUMP_CM = 8          # v13: cm: side reading this much further than the last good one = suspicious
JUMP_CONFIRM = 3     # v13: suspicious reading must repeat this many loops before it is used (1 = off)
LOG = True           # v13: write run_N.csv to the Pico (set False for the demo)
PIVOT_FROM_RAW = True     # v13b: pivot towards the side open NOW (False = v13a behaviour)
END_NEEDS_NO_ECHO = True  # v13b: end needs a side with no echo at all (False = v13a behaviour)
NO_ECHO = 999        # v13b: value returned when no echo comes back at all
COS45 = math.cos(math.pi / 4)

# MOTORS  
# Left wheel = L298N channel A, right wheel = channel B
# Forward is IN2 (left) and IN4 (right)
left_en = PWM(Pin(10))          # ENA
left_fwd = Pin(12, Pin.OUT)     # IN2
left_rev = Pin(11, Pin.OUT)     # IN1
right_en = PWM(Pin(15))         # ENB
right_fwd = Pin(14, Pin.OUT)    # IN4
right_rev = Pin(13, Pin.OUT)    # IN3
left_en.freq(1000)
right_en.freq(1000)


def drive(en, fwd, rev, power):
    """Run one motor. power in % (negative = reverse, 0 = stop).
    Any non-zero power is kept between MIN_SPEED and MAX_SPEED."""
    if power == 0:
        en.duty_u16(0)
        return
    if power > 0:
        fwd.on()
        rev.off()
    else:
        fwd.off()
        rev.on()
    power = max(MIN_SPEED, min(MAX_SPEED, abs(power)))
    en.duty_u16(int(power * 655))


def set_motors(left, right):
    """Set both wheels in % (negative = reverse, 0 = stop), with trims."""
    drive(left_en, left_fwd, left_rev, left * LEFT_TRIM)
    drive(right_en, right_fwd, right_rev, right * RIGHT_TRIM)


# ULTRASONIC SENSORS 
trig_l = Pin(L_TRIG, Pin.OUT, value=0)
echo_l = Pin(L_ECHO, Pin.IN)
trig_f = Pin(F_TRIG, Pin.OUT, value=0)
echo_f = Pin(F_ECHO, Pin.IN)
trig_r = Pin(R_TRIG, Pin.OUT, value=0)
echo_r = Pin(R_ECHO, Pin.IN)
led = Pin(25, Pin.OUT)


def read_cm(trig, echo):
    """One distance reading in cm along the sensor's axis, capped at MAX_CM.
    Returns NO_ECHO (v13b) when no echo comes back at all (nothing within ~1 m)."""
    if echo.value() == 1:                  # sensor still busy from last ping
        return MAX_CM
    trig.on()
    time.sleep_us(10)
    trig.off()
    pulse = time_pulse_us(echo, 1, 6000)   # 6 ms timeout, about 100 cm
    if pulse < 0:                          # no echo at all
        return NO_ECHO
    return min(pulse / 58, MAX_CM)         # 58 us per cm, there and back


def filter_side(raw, held, count):
    """v13: update one side's wall distance.
    Returns (new held value, new jump count, note for the log).
    - missed echo: keep the last good value (as before)
    - reading that jumps more than JUMP_CM further away: only used once it
      has repeated JUMP_CONFIRM loops in a row; until then it is ignored"""
    if raw >= MAX_CM:
        return held, count, ""
    new = raw * COS45                      # perpendicular distance to the wall
    if new - held > JUMP_CM:
        count += 1
        if count < JUMP_CONFIRM:
            return held, count, "HELD"
        return new, 0, "TAKEN"
    return new, 0, ""


# START 
set_motors(0, 0)
time.sleep(3)                              # time to put the robot down
led.on()                                   # LED on = log clock starts (t = 0)
left = 10.0                                # equal starting values,
right = 10.0                               # so the robot starts straight
end_count = 0                              # loops in a row with both end conditions
jump_l = 0                                 # v13: suspicious-reading counters
jump_r = 0
turn_dir = 0                               # v13b: 0 = not turning, 1 = left, -1 = right

# LOGGING (v13) 
# run_1.csv, run_2.csv, ... one file per run, nothing overwritten
start_ms = time.ticks_ms()
stats = {"loops": 0, "miss_l": 0, "miss_r": 0, "miss_f": 0, "held_l": 0, "held_r": 0,
         "turns": 0, "min_front": MAX_CM}
if LOG:
    run_num = 1
    for name in os.listdir():
        if name.startswith("run_") and name.endswith(".csv"):
            run_num += 1
    log = open(f"run_{run_num}.csv", "w")
    log.write(f"# {VERSION} END_CM={END_CM} END_CONFIRM={END_CONFIRM} JUMP_CM={JUMP_CM} " +
              f"JUMP_CONFIRM={JUMP_CONFIRM} BASE={BASE_SPEED} KP={KP} FRONT_TURN={FRONT_TURN_CM}\n")
    log.write("t_ms,raw_left,raw_right,front,left,right,error,action,end_count,note\n")


def log_row(action, note):
    """One line per loop: time, raw readings, held values, error, action, notes."""
    if not LOG:
        return
    t = time.ticks_diff(time.ticks_ms(), start_ms)
    log.write(f"{t},{raw_left:.1f},{raw_right:.1f},{front:.1f},{left:.1f},{right:.1f}," +
              f"{left - right:.1f},{action},{end_count},{note}\n")
    if stats["loops"] % 5 == 0:            # save to flash about every 0.2 s
        log.flush()


def write_summary(reason):
    """Short summary of the run, written when the robot stops itself."""
    if not LOG:
        return
    secs = time.ticks_diff(time.ticks_ms(), start_ms) / 1000
    loops = stats["loops"]
    n = max(loops, 1)
    turns = stats["turns"]
    min_f = stats["min_front"]
    pl = 100 * stats["miss_l"] // n
    pf = 100 * stats["miss_f"] // n
    pr = 100 * stats["miss_r"] // n
    hl = stats["held_l"]
    hr = stats["held_r"]
    with open(f"run_{run_num}_summary.txt", "w") as s:
        s.write(f"{VERSION}  run {run_num}\n")
        s.write(f"stopped: {reason} after {secs:.1f} s, {loops} loops\n")
        s.write(f"turn episodes: {turns}\n")
        s.write(f"min front: {min_f:.1f} cm\n")
        s.write(f"no-echo loops: L {pl}%  F {pf}%  R {pr}%\n")
        s.write(f"jump readings ignored: L {hl}  R {hr}\n")


# NAVIGATION
action = "CENTRE"
while True:
    raw_left = read_cm(trig_l, echo_l)
    raw_right = read_cm(trig_r, echo_r)
    front = read_cm(trig_f, echo_f)        # a miss reads MAX_CM = clear ahead
    stats["loops"] += 1
    stats["miss_l"] += raw_left >= MAX_CM
    stats["miss_r"] += raw_right >= MAX_CM
    stats["miss_f"] += front >= MAX_CM
    stats["min_front"] = min(stats["min_front"], front)

    # End of course: BOTH must be true, for END_CONFIRM loops in a row:
    #   1. front closer than END_CM (Task Decider)
    #   2. v13b: a side with no echo at all (open room, not a corner's far wall)
    if END_NEEDS_NO_ECHO:
        side_open = raw_left == NO_ECHO or raw_right == NO_ECHO
    else:
        side_open = raw_left >= MAX_CM or raw_right >= MAX_CM
    if front < END_CM and side_open:
        end_count += 1
    else:
        end_count = 0
    if end_count >= END_CONFIRM:
        set_motors(0, 0)
        led.off()
        log_row("STOP", "END_CONFIRMED")
        write_summary("end confirmed")
        if LOG:
            log.close()
        break

    # Side distances (v13): misses keep the last good value, sudden jumps
    # further away must repeat before they are used
    left, jump_l, note_l = filter_side(raw_left, left, jump_l)
    right, jump_r, note_r = filter_side(raw_right, right, jump_r)
    stats["held_l"] += note_l == "HELD"
    stats["held_r"] += note_r == "HELD"
    # print(raw_left, raw_right, front, left, right)   

    if front < FRONT_TURN_CM:
        # Wall ahead: sharp turn towards the more open side
        if not action.startswith("TURN"):
            stats["turns"] += 1
        # v13b: choose the direction once, from what the sensors see NOW
        # (no echo 999 > far 55 > near), then hold it until the front clears
        if turn_dir == 0:
            if PIVOT_FROM_RAW and raw_left != raw_right:
                turn_dir = 1 if raw_left > raw_right else -1
            else:
                turn_dir = 1 if left > right else -1
        if turn_dir == 1:
            set_motors(0, MAX_SPEED)       # stop left wheel -> turn left
            action = "TURN_L"
        else:
            set_motors(MAX_SPEED, 0)       # stop right wheel -> turn right
            action = "TURN_R"
    else:
        turn_dir = 0                       # v13b: front clear, turn finished
        # Centring: positive error = closer to the right wall, so steer left
        error = left - right
        if abs(error) <= DEADBAND:
            error = 0
        error = max(-MAX_ERR, min(MAX_ERR, error))
        set_motors(BASE_SPEED - KP * error, BASE_SPEED + KP * error)
        action = "CENTRE"

    note = ""
    if note_l:
        note += "L_JUMP_" + note_l + " "
    if note_r:
        note += "R_JUMP_" + note_r
    log_row(action, note.strip())

    time.sleep_ms(20)

# need to add task detection and stuff here 
