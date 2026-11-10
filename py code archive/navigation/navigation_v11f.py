"""
navigation_v8.py
Corridor navigation for the GE1501 AE1 robot.

Changes from v7b:
- Front sensor added (TRIG GP18, ECHO GP19). A wall closer than
  FRONT_TURN_CM ahead triggers a sharp turn towards the more open side:
  inside wheel stopped, outside wheel at MAX_SPEED.

Method (from design sketch):
- Left and right HC-SR04 sensors face 45 deg outward from the front corners.
- Each reading d is converted to a perpendicular wall distance: d * cos(45 deg).
- The robot steers to keep left and right distances equal (within +/-2 cm),
  correcting while always moving forward.
- Missed echoes keep the last good reading.
- The front sensor triggers a sharp turn if a wall is close ahead.

Wiring:
  L298N         ENA GP10, IN1 GP11, IN2 GP12, IN3 GP13, IN4 GP14, ENB GP15
  Ultrasonic L  TRIG GP28 (pin 34), ECHO GP27 (pin 32)
  Ultrasonic F  TRIG GP18 (pin 24), ECHO GP19 (pin 25)
  Ultrasonic R  TRIG GP20 (pin 26), ECHO GP21 (pin 27)
ENA/ENB jumper caps must be OFF. ECHO lines keep their 1k/2k dividers.
"""

from machine import Pin, PWM, time_pulse_us
import time
import math
import os

# ================= PINS =================
L_TRIG = 28
L_ECHO = 27
F_TRIG = 18
F_ECHO = 19
R_TRIG = 20
R_ECHO = 21

# ================= TUNING =================
BASE_SPEED = 45      # % power driving straight (was 52 in v7a)
MIN_SPEED = 30       # % lowest power sent to a motor (they stall below this)
MAX_SPEED = 50      # % highest power sent to a motor
LEFT_TRIM = 1.0
RIGHT_TRIM = 0.9     # right motor is stronger (from trim_test_v1.py)
KP = 3              # % power change per cm of centring error
DEADBAND = 1         # cm: no correction while within +/-2 cm of centre
MAX_ERR = 8          # cm: largest error acted on
FRONT_TURN_CM = 15   # cm: wall closer than this ahead -> sharp turn
MAX_CM = 55          # readings at or above this count as a missed echo
STOP_CM = 8          # cm: end = front closer than this AND a side open
COS45 = math.cos(math.pi / 4)

# ================= MOTORS (L298N) =================
# Left wheel = L298N channel A, right wheel = channel B
# Forward is IN2 (left) and IN4 (right), as found in v6
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


# ================= SENSORS (HC-SR04) =================
trig_l = Pin(L_TRIG, Pin.OUT, value=0)
echo_l = Pin(L_ECHO, Pin.IN)
trig_f = Pin(F_TRIG, Pin.OUT, value=0)
echo_f = Pin(F_ECHO, Pin.IN)
trig_r = Pin(R_TRIG, Pin.OUT, value=0)
echo_r = Pin(R_ECHO, Pin.IN)
led = Pin(25, Pin.OUT)


def read_cm(trig, echo):
    """One distance reading in cm along the sensor's axis.
    Returns MAX_CM when no echo comes back."""
    if echo.value() == 1:                  # sensor still busy from last ping
        return MAX_CM
    trig.on()
    time.sleep_us(10)
    trig.off()
    pulse = time_pulse_us(echo, 1, 6000)   # 6 ms timeout, about 100 cm
    if pulse < 0:                          # no echo
        return MAX_CM
    return min(pulse / 58, MAX_CM)         # 58 us per cm, there and back


# ================= START =================
set_motors(0, 0)
time.sleep(3)                              # time to put the robot down
led.on()
left = 10.0                                # equal starting values,
right = 10.0                               # so the robot starts straight

# ================= LOGGING =================
# Each run gets its own file (run_1.csv, run_2.csv, ...) so nothing is overwritten
run_num = 1
for name in os.listdir():
    if name.startswith("run_"):
        run_num += 1
log = open(f"run_{run_num}.csv", "w")
log.write("t_ms,raw_left,raw_right,front,left,right,error,action\n")
log_lines = 0
start_ms = time.ticks_ms()


def log_row(action):
    """One line per loop: time, raw readings, held values, error, what it did."""
    t = time.ticks_diff(time.ticks_ms(), start_ms)
    log.write(f"{t},{raw_left:.1f},{raw_right:.1f},{front:.1f},"
              f"{left:.1f},{right:.1f},{left - right:.1f},{action}\n")

# ================= NAVIGATION =================
while True:
    raw_left = read_cm(trig_l, echo_l)
    raw_right = read_cm(trig_r, echo_r)
    front = read_cm(trig_f, echo_f)        # a miss reads MAX_CM = clear ahead

    # End of course: Task Decider close ahead and at least one side open
    if front < STOP_CM and (raw_left >= MAX_CM or raw_right >= MAX_CM):
        set_motors(0, 0)
        led.off()
        log_row("STOP")
        log.close()
        break

    # Only update on a good echo; a miss keeps the last good value
    if raw_left < MAX_CM:
        left = raw_left * COS45            # perpendicular distance, left wall
    if raw_right < MAX_CM:
        right = raw_right * COS45          # perpendicular distance, right wall
    # print(raw_left, raw_right, front, left, right)   # uncomment when tuning over USB

    if front < FRONT_TURN_CM:
        # Wall ahead: sharp turn towards the more open side
        if left > right:
            set_motors(0, MAX_SPEED)       # stop left wheel -> turn left
            action = "TURN_L"
        else:
            set_motors(MAX_SPEED, 0)       # stop right wheel -> turn right
            action = "TURN_R"
    else:
        action = "CENTRE"
        # Centring: positive error = closer to the right wall, so steer left
        error = left - right
        if abs(error) <= DEADBAND:
            error = 0
        error = max(-MAX_ERR, min(MAX_ERR, error))
        set_motors(BASE_SPEED - KP * error, BASE_SPEED + KP * error)

    log_row(action)
    log_lines += 1
    if log_lines % 5 == 0:                 # save to flash about every 0.2 s
        log.flush()

    time.sleep_ms(20)

# Task detection to be added after navigation is tuned
