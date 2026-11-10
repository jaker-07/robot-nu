"""
navigation_v9.py
Corridor navigation for the GE1501 AE1 robot.

Changes from v8 (v8 tuned on track: BASE 45, MIN 30, MAX 50, KP 3, MAX_CM 55):
- Corners: a missed echo now only holds the last good value for short
  dropouts. A side that stays missing for MISS_LIMIT readings counts as
  open, so corners register instead of being hidden by the held value.
- Corner mode: an error bigger than CORNER_ERR (or a wall ahead) gives a
  hard turn: inside wheel stopped, outside wheel at MAX_SPEED.
- Straights: DEADBAND 1 -> 2 cm (back to the design value) so sensor noise
  doesn't make the robot hunt left and right.
- MAX_ERR removed: CORNER_ERR now caps the proportional range.

Method (from design sketch):
- Left and right HC-SR04 sensors face 45 deg outward from the front corners.
- Each reading d is converted to a perpendicular wall distance: d * cos(45 deg).
- Small errors: steer proportionally to keep left and right equal (+/-2 cm).
- Large errors or a wall ahead: hard turn towards the more open side.

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

# ================= PINS =================
L_TRIG = 28
L_ECHO = 27
F_TRIG = 18
F_ECHO = 19
R_TRIG = 20
R_ECHO = 21

# ================= TUNING =================
BASE_SPEED = 45      # % power driving straight
MIN_SPEED = 30       # % lowest power sent to a moving motor
MAX_SPEED = 50       # % highest power sent to a motor
LEFT_TRIM = 1.0
RIGHT_TRIM = 0.9     # right motor is stronger (from trim_test_v1.py)
KP = 3               # % power change per cm of centring error
DEADBAND = 2         # cm: no correction while within +/-2 cm of centre
CORNER_ERR = 6       # cm: error bigger than this -> hard turn (corner)
FRONT_TURN_CM = 15   # cm: wall closer than this ahead -> hard turn
MISS_LIMIT = 3       # misses in a row before a side counts as open
MAX_CM = 55          # readings at or above this count as a miss / open
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
miss_left = 0
miss_right = 0

# ================= NAVIGATION =================
while True:
    raw_left = read_cm(trig_l, echo_l)
    raw_right = read_cm(trig_r, echo_r)
    front = read_cm(trig_f, echo_f)        # a miss reads MAX_CM = clear ahead

    # Short dropouts keep the last good value; a side that stays missing
    # for MISS_LIMIT readings in a row counts as open (e.g. a corner)
    if raw_left < MAX_CM:
        left = raw_left * COS45            # perpendicular distance, left wall
        miss_left = 0
    else:
        miss_left += 1
        if miss_left >= MISS_LIMIT:
            left = MAX_CM * COS45
    if raw_right < MAX_CM:
        right = raw_right * COS45          # perpendicular distance, right wall
        miss_right = 0
    else:
        miss_right += 1
        if miss_right >= MISS_LIMIT:
            right = MAX_CM * COS45
    # print(raw_left, raw_right, front, left, right)   # uncomment when tuning over USB

    # Positive error = more room on the left (closer to the right wall)
    error = left - right

    if front < FRONT_TURN_CM or abs(error) > CORNER_ERR:
        # Wall ahead or a corner: hard turn towards the more open side
        if error > 0:
            set_motors(0, MAX_SPEED)       # stop left wheel -> turn left
        else:
            set_motors(MAX_SPEED, 0)       # stop right wheel -> turn right
    else:
        # Centring: small proportional corrections
        if abs(error) <= DEADBAND:
            error = 0
        set_motors(BASE_SPEED - KP * error, BASE_SPEED + KP * error)

    time.sleep_ms(20)

# Task detection to be added after navigation is tuned
