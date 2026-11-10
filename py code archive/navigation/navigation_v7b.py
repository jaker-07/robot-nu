"""
navigation_v7b.py
Corridor navigation for the GE1501 AE1 robot.

Changes from v7a:
- BASE_SPEED 52 -> 60. Straight driving now runs left 60 %, right 54 %
  (after trim), clear of the low-power stall region. Because BASE_SPEED
  equals MAX_SPEED, steering works by slowing the inside wheel (to 45 %).

Method (from design sketch):
- Left and right HC-SR04 sensors face 45 deg outward from the front corners.
- Each reading d is converted to a perpendicular wall distance: d * cos(45 deg).
- The robot steers to keep left and right distances equal (within +/-2 cm),
  correcting while always moving forward.
- Missed echoes keep the last good reading.

Wiring:
  L298N         ENA GP10, IN1 GP11, IN2 GP12, IN3 GP13, IN4 GP14, ENB GP15
  Ultrasonic L  TRIG GP28 (pin 34), ECHO GP27 (pin 32)
  Ultrasonic R  TRIG GP20 (pin 26), ECHO GP21 (pin 27)
ENA/ENB jumper caps must be OFF. ECHO lines keep their 1k/2k dividers.
"""

from machine import Pin, PWM, time_pulse_us
import time
import math

# ================= PINS =================
L_TRIG = 28
L_ECHO = 27
R_TRIG = 20
R_ECHO = 21

# ================= TUNING =================
BASE_SPEED = 60      # % power driving straight (was 52 in v7a)
MIN_SPEED = 45       # % lowest power sent to a motor (they stall below this)
MAX_SPEED = 60       # % highest power sent to a motor
LEFT_TRIM = 1.0
RIGHT_TRIM = 0.9     # right motor is stronger (from trim_test_v1.py)
KP = 4               # % power change per cm of centring error
DEADBAND = 2         # cm: no correction while within +/-2 cm of centre
MAX_ERR = 8          # cm: largest error acted on
MAX_CM = 80          # readings at or above this count as a missed echo
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

# ================= NAVIGATION =================
while True:
    raw_left = read_cm(trig_l, echo_l)
    raw_right = read_cm(trig_r, echo_r)

    # Only update on a good echo; a miss keeps the last good value
    if raw_left < MAX_CM:
        left = raw_left * COS45            # perpendicular distance, left wall
    if raw_right < MAX_CM:
        right = raw_right * COS45          # perpendicular distance, right wall
    # print(raw_left, raw_right, left, right)   # uncomment when tuning over USB

    # Centring: positive error = closer to the right wall, so steer left
    error = left - right
    if abs(error) <= DEADBAND:
        error = 0
    error = max(-MAX_ERR, min(MAX_ERR, error))

    set_motors(BASE_SPEED - KP * error, BASE_SPEED + KP * error)

    time.sleep_ms(20)

# Task detection to be added after navigation is tuned
