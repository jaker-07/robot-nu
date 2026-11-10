"""
navigation_v5.py
Corridor navigation for the GE1501 AE1 robot.

Changes from v4:
- Left and right motors swapped in software: the left wheel is now driven
  by L298N channel B and the right wheel by channel A. No rewiring needed.

Method (from design sketch):
- Left and right HC-SR04 sensors face 45 deg outward from the front corners.
- Each reading d is converted to a perpendicular wall distance: d * cos(45 deg).
- The robot steers to keep left and right distances equal (within +/-2 cm),
  correcting while always moving forward.

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
BASE_SPEED = 45      # % motor power driving straight (tune to under 0.25 m/s)
MIN_SPEED = 30       # % lowest power for either wheel; raise if a wheel stalls
KP = 4               # % power change per cm of centring error
DEADBAND = 2         # cm: no correction while within +/-2 cm of centre
MAX_ERR = 8          # cm: largest error acted on (limits wheel speed difference)
LEFT_TRIM = 1.0      # lower one of these if the robot drifts driving straight
RIGHT_TRIM = 1.0
MAX_CM = 80          # readings at or above this count as a missed echo
COS45 = math.cos(math.pi / 4)

# ================= MOTORS (L298N) =================
# SWAPPED vs v4: left wheel = L298N channel B, right wheel = channel A
left_en = PWM(Pin(15))          # ENB
left_fwd = Pin(13, Pin.OUT)     # IN3
left_rev = Pin(14, Pin.OUT)     # IN4
right_en = PWM(Pin(10))         # ENA
right_fwd = Pin(11, Pin.OUT)    # IN1
right_rev = Pin(12, Pin.OUT)    # IN2
left_en.freq(1000)
right_en.freq(1000)


def set_motors(left, right):
    """Set wheel power in % (-100 to 100). Negative = reverse."""
    left = max(-100, min(100, left * LEFT_TRIM))
    right = max(-100, min(100, right * RIGHT_TRIM))
    if left >= 0:
        left_fwd.on()
        left_rev.off()
    else:
        left_fwd.off()
        left_rev.on()
    if right >= 0:
        right_fwd.on()
        right_rev.off()
    else:
        right_fwd.off()
        right_rev.on()
    left_en.duty_u16(int(abs(left) * 655))
    right_en.duty_u16(int(abs(right) * 655))


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

    left_speed = max(MIN_SPEED, BASE_SPEED - KP * error)
    right_speed = max(MIN_SPEED, BASE_SPEED + KP * error)
    set_motors(left_speed, right_speed)

    time.sleep_ms(20)

# Task detection to be added after navigation is tuned
