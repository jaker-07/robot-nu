"""
main_v20a.py   GE1501 AE1 robot: corridor navigation, end detection, three tasks.

Changes from main_v20 (v20 never stopped at the end and was a bit jittery):
- End check back to the v19 rule: a side counts as open if it has no echo, a
  reading at or beyond MAX_CM, OR the sensor is still busy. Past the end the open
  side alternates between "no echo" and "busy" (an HC-SR04 holds its echo line high
  after a miss), so v20, which excluded busy, kept resetting its count and never stopped.
- No gap between the right and front pings (v19 timing). The extra 5 ms in v20
  slowed every loop; GAP_RF_MS is still there if front crosstalk shows up.
- Pin comments updated: LED on GP27 connected; IR and buzzer not connected yet.

Changes in v20 from main_v19 (code audit fixes):
- Task stage added: detect_task() plus task_buzzer(), task_light(), task_spin(),
  selected by one if/elif. Detection is a stub until the Task Decider's signal is
  known (set TASK_TEST to 1, 2 or 3 to run a task).
- Motors, buzzer and light are switched off however the program ends (try/finally).
- A busy sensor no longer reads as "no echo": sides hold their last value, the
  front keeps its last reading, and a busy side never counts as open.
- End check needed a side with NO echo at all (reverted in v20a).
- Pivot direction comes from the current readings and is locked until the front
  clears; ties fall back to held values, then the last turn, not always right.
- 5 ms gap between the right and front pings (crosstalk), like the 10 ms left/right gap.
- Limits applied before trim, so the trim holds even at the 25-55 % limits.
- Run timeout (RUN_TIMEOUT_S) so the robot stops if it never finds the end.
- Docstring and comments corrected; math import, duplicate side code and the
  commented-out print replaced (COS45 constant, wall_distance(), DEBUG flag).

Wiring (as built):
  L298N         ENA GP10, IN1 GP11, IN2 GP12, IN3 GP13, IN4 GP14, ENB GP15
                Forward = IN2 (left), IN3 (right; right motor mounted reversed)
  Ultrasonic L  TRIG GP16 (pin 21), ECHO GP17 (pin 22)
  Ultrasonic F  TRIG GP18 (pin 24), ECHO GP19 (pin 25)
  Ultrasonic R  TRIG GP20 (pin 26), ECHO GP21 (pin 27)
  Task parts    LED GP27 (connected); IR OUT GP22 and buzzer GP26 planned
ENA/ENB jumper caps OFF. No echo dividers fitted (ECHO goes straight to the Pico).
"""

from machine import Pin, PWM, time_pulse_us
import time

# ================= PINS (GP numbers) =================
L_TRIG, L_ECHO = 16, 17
F_TRIG, F_ECHO = 18, 19
R_TRIG, R_ECHO = 20, 21
IR_PIN = 22          # IR module OUT (task detection) - not connected yet
BUZZER_PIN = 26      # buzzer - not connected yet
LIGHT_PIN = 27       # LED - connected

# ================= NAVIGATION TUNING =================
BASE_SPEED = 40      # % power driving straight
MIN_SPEED = 25       # % lowest power sent to a moving motor
MAX_SPEED = 55       # % highest power sent to a motor
LEFT_TRIM = 1.0
RIGHT_TRIM = 0.92    # right motor is stronger (straight-line tests after the motor flip)
KP = 1.5             # % power change per cm of centring error
DEADBAND = 2         # cm of error (about +/-1 cm off centre): no correction inside this
MAX_ERR = 8          # cm: largest error acted on
FRONT_TURN_CM = 8    # cm: wall closer than this ahead -> pivot
MAX_CM = 65          # cm: readings are capped here; at or above = no usable wall reading
STOP_CM = 12         # cm: end = front closer than this AND a side open
END_CONFIRM = 3      # loops in a row both end conditions must hold
RUN_TIMEOUT_S = 60   # s: stop if the end is not found within this time
COS45 = 0.7071       # cos(45 deg): beam reading -> perpendicular wall distance

# ================= TIMING =================
GAP_LR_MS = 10       # ms between left and right pings (crosstalk)
GAP_RF_MS = 0        # ms between right and front pings (0 = v19 timing; try 5 if the front
                     # starts picking up the right sensor's echoes)
LOOP_PAUSE_MS = 5    # ms at the end of each loop
ECHO_TIMEOUT_US = 6000   # us: about 1 m; needed so "no echo" means open room

# ================= TASK TUNING =================
TASK_TEST = None     # None = use detect_task(); 1 buzzer, 2 light, 3 spin to test a task
BUZZ_FREQ = 2000     # Hz
BUZZ_TIME = 2.0      # s
LIGHT_TIME = 3.0     # s
BACKUP_TIME = 0.5    # s reversing away from the Task Decider before spinning
SPIN_SPEED = 45      # % power for the spin
SPIN_TIME = 2.0      # s for a full 360 deg - CALIBRATE on the real floor

DEBUG = False        # True = print readings each loop (USB only)

BUSY = -1            # read_cm(): sensor still busy, no new reading
NO_ECHO = 999        # read_cm(): no echo at all (nothing within about 1 m)

# ================= MOTORS (L298N) =================
left_en = PWM(Pin(10))          # ENA
left_fwd = Pin(12, Pin.OUT)     # IN2
left_rev = Pin(11, Pin.OUT)     # IN1
right_en = PWM(Pin(15))         # ENB
right_fwd = Pin(13, Pin.OUT)    # IN3 (right motor mounted reversed)
right_rev = Pin(14, Pin.OUT)    # IN4
left_en.freq(1000)
right_en.freq(1000)


def limit(power):
    """Keep a non-zero motor command between MIN_SPEED and MAX_SPEED, sign kept."""
    if power == 0:
        return 0
    size = max(MIN_SPEED, min(MAX_SPEED, abs(power)))
    return size if power > 0 else -size


def drive(en, fwd, rev, power):
    """Run one motor at power % (negative = reverse, 0 = stop)."""
    if power >= 0:
        fwd.on()
        rev.off()
    else:
        fwd.off()
        rev.on()
    en.duty_u16(int(abs(power) * 655.35))


def set_motors(left, right):
    """Set both wheels in %: limits first, then trims, so the trim always holds."""
    drive(left_en, left_fwd, left_rev, limit(left) * LEFT_TRIM)
    drive(right_en, right_fwd, right_rev, limit(right) * RIGHT_TRIM)


# ================= SENSORS =================
trig_l = Pin(L_TRIG, Pin.OUT, value=0)
echo_l = Pin(L_ECHO, Pin.IN)
trig_f = Pin(F_TRIG, Pin.OUT, value=0)
echo_f = Pin(F_ECHO, Pin.IN)
trig_r = Pin(R_TRIG, Pin.OUT, value=0)
echo_r = Pin(R_ECHO, Pin.IN)
ir_in = Pin(IR_PIN, Pin.IN)


def read_cm(trig, echo):
    """One ping along the sensor's axis. Returns cm (capped at MAX_CM),
    NO_ECHO if nothing came back, or BUSY if the sensor was still busy."""
    if echo.value() == 1:
        return BUSY
    trig.on()
    time.sleep_us(10)
    trig.off()
    pulse = time_pulse_us(echo, 1, ECHO_TIMEOUT_US)
    if pulse < 0:
        return NO_ECHO
    return min(pulse / 58, MAX_CM)          # 58 us per cm, there and back


def wall_distance(raw, held):
    """Perpendicular wall distance from a side reading; keeps the last good value
    when the reading is busy, missing or at the cap."""
    if 0 <= raw < MAX_CM:
        return raw * COS45
    return held


def side_open(raw):
    """True if this side shows open space (end check only): no echo, a reading at or
    beyond MAX_CM, or busy (after a miss the sensor stays busy, so busy means open too)."""
    return raw == BUSY or raw >= MAX_CM


def pivot_direction(raw_left, raw_right, left, right, last_dir):
    """+1 = turn left, -1 = turn right: towards the side that is more open now.
    Busy readings use the held value; ties fall back to held values, then last turn."""
    rl = raw_left if raw_left != BUSY else left / COS45
    rr = raw_right if raw_right != BUSY else right / COS45
    if rl != rr:
        return 1 if rl > rr else -1
    if left != right:
        return 1 if left > right else -1
    return last_dir


# ================= OUTPUTS =================
led = Pin(25, Pin.OUT)              # onboard LED: on while navigating
buzzer = PWM(Pin(BUZZER_PIN))
buzzer.duty_u16(0)
light = Pin(LIGHT_PIN, Pin.OUT, value=0)


def stop_all():
    """Motors, buzzer and light off."""
    set_motors(0, 0)
    buzzer.duty_u16(0)
    light.off()


# ================= NAVIGATION =================
def navigate():
    """Drive the corridor until the end is confirmed (returns True)
    or RUN_TIMEOUT_S passes (returns False)."""
    left = right = 10.0             # held wall distances; equal so it starts straight
    front = NO_ECHO
    end_count = 0
    turn_dir = 0                    # 0 = not pivoting, 1 = left, -1 = right
    last_dir = -1
    start = time.ticks_ms()

    while time.ticks_diff(time.ticks_ms(), start) < RUN_TIMEOUT_S * 1000:
        raw_left = read_cm(trig_l, echo_l)
        time.sleep_ms(GAP_LR_MS)
        raw_right = read_cm(trig_r, echo_r)
        time.sleep_ms(GAP_RF_MS)
        raw_front = read_cm(trig_f, echo_f)
        if raw_front != BUSY:           # busy front: keep the last reading
            front = raw_front

        # End of course: front close AND a side open, END_CONFIRM loops in a row
        if front < STOP_CM and (side_open(raw_left) or side_open(raw_right)):
            end_count += 1
        else:
            end_count = 0
        if end_count >= END_CONFIRM:
            set_motors(0, 0)
            return True

        left = wall_distance(raw_left, left)
        right = wall_distance(raw_right, right)
        if DEBUG:
            print(raw_left, raw_right, front, left, right)

        if front < FRONT_TURN_CM:
            # Wall ahead: pivot towards the open side, direction locked until clear
            if turn_dir == 0:
                turn_dir = pivot_direction(raw_left, raw_right, left, right, last_dir)
                last_dir = turn_dir
            if turn_dir == 1:
                set_motors(0, MAX_SPEED)    # stop left wheel -> turn left
            else:
                set_motors(MAX_SPEED, 0)    # stop right wheel -> turn right
        else:
            turn_dir = 0
            # Centring: positive error = closer to the right wall, so steer left
            error = left - right
            if abs(error) <= DEADBAND:
                error = 0
            error = max(-MAX_ERR, min(MAX_ERR, error))
            set_motors(BASE_SPEED - KP * error, BASE_SPEED + KP * error)

        time.sleep_ms(LOOP_PAUSE_MS)

    set_motors(0, 0)
    return False


# ================= TASK DETECTION =================
def detect_task():
    """Which task the Task Decider asks for: 1 buzzer, 2 light, 3 spin, None = unknown.
    STUB: the decider's signal format is not known yet. ir_in is set up on IR_PIN
    (IR modules read 0 when they detect); replace this body once the signal is known.
    Until then it returns TASK_TEST."""
    return TASK_TEST


# ================= TASKS =================
def task_buzzer():
    """Task 1: a clear tone for BUZZ_TIME seconds."""
    buzzer.freq(BUZZ_FREQ)
    buzzer.duty_u16(32768)              # 50 % duty
    time.sleep(BUZZ_TIME)
    buzzer.duty_u16(0)


def task_light():
    """Task 2: light on for LIGHT_TIME seconds."""
    light.on()
    time.sleep(LIGHT_TIME)
    light.off()


def task_spin():
    """Task 3: back away from the Task Decider, then one full turn on the spot."""
    set_motors(-BASE_SPEED, -BASE_SPEED)
    time.sleep(BACKUP_TIME)
    set_motors(0, 0)
    time.sleep(0.2)
    set_motors(SPIN_SPEED, -SPIN_SPEED)     # left forward, right reverse
    time.sleep(SPIN_TIME)
    set_motors(0, 0)


def no_task():
    """No task recognised: blink the onboard LED so it is visible."""
    for _ in range(5):
        led.on()
        time.sleep(0.2)
        led.off()
        time.sleep(0.2)


# ================= MAIN =================
try:
    stop_all()
    time.sleep(3)                       # time to put the robot down
    led.on()
    if navigate():
        task = detect_task()
        if task == 1:
            task_buzzer()
        elif task == 2:
            task_light()
        elif task == 3:
            task_spin()
        else:
            no_task()
finally:
    stop_all()                          # always stop, however the program ends
    led.off()
