"""
main_v20g.py   GE1501 AE1 robot: corridor navigation, end detection, three tasks.

Changes from main_v20f:
- Task signal is how LONG the one flag stays in front of the IR sensor:
  about 3 s = buzzer, 5 s = light, 10 s = spin. The robot times the hold and only
  chooses once the flag has gone, so a 10 s spin signal can't fire the buzzer at 3 s.
  Holds under MIN_HOLD are ignored; a flag never taken away counts as spin after MAX_HOLD.
  (Replaces v20f's flag counting.)

Changes in v20f from main_v20e:
- Driving: two pieces of the first v20 brought back to stop twitching:
  5 ms gap between the right and front pings (GAP_RF_MS; stray right-sensor
  echoes could fake a wall ahead and cause a one-loop pivot), and the pivot
  direction is held until the front is clear (no left/right flip-flop mid-turn).
  End check stays the v19 rule.

Changes in v20e from main_v20d (settings from ir_task_test_v2 results):
- Task detection: after stopping, the robot waits (up to TASK_WAIT_S) for the
  Task Decider's flag to stay in front of the IR sensor for HOLD_TIME (3 s);
  removing it early resets the timer. It then runs FLAG_TASK (placeholder until
  we know how the decider picks between the three tasks).
- Buzzer 1 s (placeholder for a song).
- Spin is one turn: back up 0.5 s at 40 %, pause 0.2 s, spin for SPIN_TIME (1.5 s,
  the slider) at SPIN_SPEED. SPIN_TASK_TIME removed.

Changes in v20d from main_v20c:
- Driving back to the v19 style: busy or no echo both read MAX_CM, v19 end check,
  pivot towards the larger held wall distance, v19 loop timing and motor limits.
  Kept from v20: run timeout, everything switched off however the program ends.
- Buzzer is now a plain on/off pin with the tone made in software. GP26 shares a
  PWM channel with ENA (GP10, slice 5A), so as a PWM pin it copied the left
  motor's signal and buzzed whenever the motors ran.

Wiring (as built):
  L298N         ENA GP10, IN1 GP11, IN2 GP12, IN3 GP13, IN4 GP14, ENB GP15
                Forward = IN2 (left), IN3 (right; right motor mounted reversed)
  Ultrasonic L  TRIG GP16 (pin 21), ECHO GP17 (pin 22)
  Ultrasonic F  TRIG GP18 (pin 24), ECHO GP19 (pin 25)
  Ultrasonic R  TRIG GP20 (pin 26), ECHO GP21 (pin 27)
  IR module     OUT GP22
  Buzzer        GP26 and GND
  LED strip     DIN GP27 (WS2812, 5 LEDs)
ENA/ENB jumper caps OFF. No echo dividers fitted (ECHO goes straight to the Pico).
"""

from machine import Pin, PWM, time_pulse_us
import time
import neopixel

# ================= PINS (GP numbers) =================
L_TRIG, L_ECHO = 16, 17
F_TRIG, F_ECHO = 18, 19
R_TRIG, R_ECHO = 20, 21
IR_PIN = 22          # IR module OUT (task detection)
BUZZER_PIN = 26      # buzzer (plain pin, see docstring)
LIGHT_PIN = 27       # LED strip data in (WS2812)
NUM_LEDS = 5         # strip cut after LED 5

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
MAX_CM = 65          # cm: readings capped here; busy or no echo also read MAX_CM
STOP_CM = 12         # cm: end = front closer than this AND a side open
END_CONFIRM = 3      # loops in a row both end conditions must hold
RUN_TIMEOUT_S = 60   # s: stop if the end is not found within this time
COS45 = 0.70710678   # cos(45 deg): beam reading -> perpendicular wall distance
GAP_RF_MS = 5        # ms between right and front pings (0 = v19 timing)

# ================= TASK TUNING =================
BUZZER_HOLD = 3.0    # s the Task Decider holds the flag for the buzzer
LIGHT_HOLD = 5.0     # s ... for the light
SPIN_HOLD = 10.0     # s ... for the spin  (task = nearest of these three)
MIN_HOLD = 2.0       # s: shorter holds are ignored (hand passing, glitch)
MAX_HOLD = 15.0      # s: flag still there after this = spin signal, act anyway
TASK_WAIT_S = 60     # s to wait for the flag after stopping before giving up
IR_ACTIVE_LOW = True # IR module reads 0 when it detects
BUZZER_ACTIVE = False    # True = active buzzer (sounds with plain power); False = needs a tone
BUZZ_FREQ = 2000     # Hz tone for a passive buzzer
BUZZ_TIME = 1.0      # s (placeholder for a song)
LIGHT_TIME = 2.0     # s
LIGHT_COLOUR = (0, 120, 0)   # (red, green, blue) 0-255 for the light task
DIM = (0, 25, 0)             # startup sequence trail
BRIGHT = (0, 120, 0)         # startup sequence head and flashes
BACKUP_TIME = 0.5    # s reversing away from the Task Decider before spinning
BACKUP_SPEED = 40    # % power for backing up
SPIN_PAUSE = 0.2     # s stopped between backing up and spinning
SPIN_SPEED = 85      # % power for the spin (drive() caps it at MAX_SPEED, so 55 % now)
SPIN_TIME = 1.5      # s  <-- SLIDER: tune until the robot turns exactly 360 deg

DEBUG = False        # True = print readings each loop (USB only)

# ================= MOTORS (L298N) =================
left_en = PWM(Pin(10))          # ENA
left_fwd = Pin(12, Pin.OUT)     # IN2
left_rev = Pin(11, Pin.OUT)     # IN1
right_en = PWM(Pin(15))         # ENB
right_fwd = Pin(13, Pin.OUT)    # IN3 (right motor mounted reversed)
right_rev = Pin(14, Pin.OUT)    # IN4
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


# ================= SENSORS =================
trig_l = Pin(L_TRIG, Pin.OUT, value=0)
echo_l = Pin(L_ECHO, Pin.IN)
trig_f = Pin(F_TRIG, Pin.OUT, value=0)
echo_f = Pin(F_ECHO, Pin.IN)
trig_r = Pin(R_TRIG, Pin.OUT, value=0)
echo_r = Pin(R_ECHO, Pin.IN)
ir_in = Pin(IR_PIN, Pin.IN)


def ir_detected():
    """True while the IR sensor sees something (the Task Decider's flag)."""
    return ir_in.value() == (0 if IR_ACTIVE_LOW else 1)


def read_cm(trig, echo):
    """One distance reading in cm along the sensor's axis.
    Returns MAX_CM when the sensor is busy or no echo comes back."""
    if echo.value() == 1:                  # sensor still busy from last ping
        return MAX_CM
    trig.on()
    time.sleep_us(10)
    trig.off()
    pulse = time_pulse_us(echo, 1, 6000)   # 6 ms timeout, about 100 cm
    if pulse < 0:                          # no echo
        return MAX_CM
    return min(pulse / 58, MAX_CM)         # 58 us per cm, there and back


# ================= OUTPUTS =================
led = Pin(25, Pin.OUT)                     # onboard LED: on while navigating
buzzer = Pin(BUZZER_PIN, Pin.OUT, value=0)
strip = neopixel.NeoPixel(Pin(LIGHT_PIN), NUM_LEDS)


def strip_fill(colour):
    """Set every LED on the strip to one colour."""
    for i in range(NUM_LEDS):
        strip[i] = colour
    strip.write()


def startup_lights():
    """Green loading sequence (about 2 s): sweep, two flashes, fade out."""
    strip_fill((0, 0, 0))
    for i in range(NUM_LEDS):              # loading bar: bright head, dim trail
        strip[i] = BRIGHT
        if i > 0:
            strip[i - 1] = DIM
        strip.write()
        time.sleep_ms(180)
    for _ in range(2):                     # two flashes = ready
        strip_fill(BRIGHT)
        time.sleep_ms(150)
        strip_fill((0, 0, 0))
        time.sleep_ms(150)
    strip_fill(BRIGHT)
    for level in range(120, -1, -8):       # fade out
        strip_fill((0, level, 0))
        time.sleep_ms(25)


def stop_all():
    """Motors, buzzer and LED strip off."""
    set_motors(0, 0)
    buzzer.off()
    strip_fill((0, 0, 0))


# ================= NAVIGATION =================
def navigate():
    """Drive the corridor until the end is confirmed (returns True)
    or RUN_TIMEOUT_S passes (returns False)."""
    left = 10.0                            # equal starting values,
    right = 10.0                           # so the robot starts straight
    end_count = 0                          # loops in a row with both end conditions
    turn_dir = 0                           # 0 = not pivoting, 1 = left, -1 = right
    start = time.ticks_ms()

    while time.ticks_diff(time.ticks_ms(), start) < RUN_TIMEOUT_S * 1000:
        raw_left = read_cm(trig_l, echo_l)
        time.sleep_ms(10)                  # let left's echoes die out
        raw_right = read_cm(trig_r, echo_r)
        time.sleep_ms(GAP_RF_MS)           # let right's echoes die out
        front = read_cm(trig_f, echo_f)    # a miss reads MAX_CM = clear ahead

        # End of course: BOTH must be true, for END_CONFIRM loops in a row:
        #   1. front closer than STOP_CM (Task Decider)
        #   2. at least one side reads open (MAX_CM)
        if front < STOP_CM and (raw_left >= MAX_CM or raw_right >= MAX_CM):
            end_count += 1
        else:
            end_count = 0
        if end_count >= END_CONFIRM:
            set_motors(0, 0)
            return True

        # Only update on a good echo; a miss keeps the last good value
        if raw_left < MAX_CM:
            left = raw_left * COS45        # perpendicular distance, left wall
        if raw_right < MAX_CM:
            right = raw_right * COS45      # perpendicular distance, right wall
        if DEBUG:
            print(raw_left, raw_right, front, left, right)

        if front < FRONT_TURN_CM:
            # Wall ahead: sharp turn towards the more open side,
            # keeping the same direction until the front is clear
            if turn_dir == 0:
                turn_dir = 1 if left > right else -1
            if turn_dir == 1:
                set_motors(0, MAX_SPEED)   # stop left wheel -> turn left
            else:
                set_motors(MAX_SPEED, 0)   # stop right wheel -> turn right
        else:
            turn_dir = 0                   # front clear: turn finished
            # Centring: positive error = closer to the right wall, so steer left
            error = left - right
            if abs(error) <= DEADBAND:
                error = 0
            error = max(-MAX_ERR, min(MAX_ERR, error))
            set_motors(BASE_SPEED - KP * error, BASE_SPEED + KP * error)

        time.sleep_ms(5)

    set_motors(0, 0)
    return False


# ================= TASK DETECTION =================
def hold_length():
    """Wait while the flag is in front of the IR sensor; return how long it stayed (s).
    Stops timing at MAX_HOLD so a flag that is never taken away can't hang the robot."""
    start = time.ticks_ms()
    while ir_detected():
        if time.ticks_diff(time.ticks_ms(), start) >= MAX_HOLD * 1000:
            break
        time.sleep_ms(20)
    return time.ticks_diff(time.ticks_ms(), start) / 1000


def detect_task():
    """Time how long the Task Decider's flag stays in front of the IR sensor and,
    once it has gone, return the task: nearest of BUZZER_HOLD (1 buzzer),
    LIGHT_HOLD (2 light), SPIN_HOLD (3 spin). Holds under MIN_HOLD are ignored.
    Returns None if no flag comes within TASK_WAIT_S."""
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < TASK_WAIT_S * 1000:
        if ir_detected():
            held = hold_length()
            if held >= MIN_HOLD:
                if held < (BUZZER_HOLD + LIGHT_HOLD) / 2:      # under 4 s
                    return 1
                if held < (LIGHT_HOLD + SPIN_HOLD) / 2:        # under 7.5 s
                    return 2
                return 3
        time.sleep_ms(20)
    return None


# ================= TASKS =================
def task_buzzer():
    """Task 1: buzzer for BUZZ_TIME seconds (software tone for a passive buzzer)."""
    end = time.ticks_add(time.ticks_ms(), int(BUZZ_TIME * 1000))
    if BUZZER_ACTIVE:
        buzzer.on()
        while time.ticks_diff(end, time.ticks_ms()) > 0:
            time.sleep_ms(10)
    else:
        half_us = 500000 // BUZZ_FREQ      # half a tone period in microseconds
        while time.ticks_diff(end, time.ticks_ms()) > 0:
            buzzer.on()
            time.sleep_us(half_us)
            buzzer.off()
            time.sleep_us(half_us)
    buzzer.off()


def task_light():
    """Task 2: LED strip on for LIGHT_TIME seconds."""
    strip_fill(LIGHT_COLOUR)
    time.sleep(LIGHT_TIME)
    strip_fill((0, 0, 0))


def task_spin():
    """Task 3: back away from the Task Decider, then one turn on the spot."""
    set_motors(-BACKUP_SPEED, -BACKUP_SPEED)
    time.sleep(BACKUP_TIME)
    set_motors(0, 0)
    time.sleep(SPIN_PAUSE)
    set_motors(SPIN_SPEED, -SPIN_SPEED)    # left forward, right reverse
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
    startup_lights()                       # about 2 s of the start delay
    time.sleep(1)                          # rest of the time to put the robot down
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
    stop_all()                             # always stop, however the program ends
    led.off()
