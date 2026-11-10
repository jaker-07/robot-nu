"""
navigation_v2.py
Corridor navigation for the GE1501 AE1 robot.

Changes from v1:
- Front sensor removed (not yet connected). Centring uses L and R only.

Method (from design sketch):
- Left and right HC-SR04 sensors face 45 deg outward from the front corners.
- Each reading d is converted to a perpendicular wall distance: d * cos(45 deg).
- The robot steers to keep left and right distances equal (within +/-2 cm),
  correcting while always moving forward (no stopping to reorient).
- When both side walls disappear, the robot has reached the end and stops.

Wiring (see wiring diagram):
  L298N         ENA GP10, IN1 GP11, IN2 GP12, IN3 GP13, IN4 GP14, ENB GP15
  Ultrasonic L  TRIG GP16, ECHO GP17
  Ultrasonic R  TRIG GP20, ECHO GP21
ENA/ENB jumper caps must be OFF.
"""

from machine import Pin, PWM, time_pulse_us
import time
import math

# ================= TUNING =================
BASE_SPEED = 45      # % motor power driving straight (tune to under 0.25 m/s)
KP = 4               # % power change per cm of centring error
DEADBAND = 2         # cm: no correction while within +/-2 cm of centre
OPEN_CM = 35         # both sides further than this -> end of corridor
LEFT_TRIM = 1.0      # lower one of these if the robot drifts driving straight
RIGHT_TRIM = 1.0
MAX_CM = 80          # cap readings so a missed echo can't cause a huge swing
COS45 = math.cos(math.pi / 4)

# ================= MOTORS (L298N) =================
ena = PWM(Pin(10))
enb = PWM(Pin(15))
ena.freq(1000)
enb.freq(1000)
in1 = Pin(11, Pin.OUT)
in2 = Pin(12, Pin.OUT)
in3 = Pin(13, Pin.OUT)
in4 = Pin(14, Pin.OUT)


def set_motors(left, right):
    """Set wheel power in % (-100 to 100). Negative = reverse."""
    left = max(-100, min(100, left * LEFT_TRIM))
    right = max(-100, min(100, right * RIGHT_TRIM))
    if left >= 0:
        in1.on()
        in2.off()
    else:
        in1.off()
        in2.on()
    if right >= 0:
        in3.on()
        in4.off()
    else:
        in3.off()
        in4.on()
    ena.duty_u16(int(abs(left) * 655))
    enb.duty_u16(int(abs(right) * 655))


# ================= SENSORS (HC-SR04) =================
trig_l = Pin(16, Pin.OUT, value=0)
echo_l = Pin(17, Pin.IN)
trig_r = Pin(20, Pin.OUT, value=0)
echo_r = Pin(21, Pin.IN)
led = Pin(25, Pin.OUT)


def read_cm(trig, echo):
    """One distance reading in cm along the sensor's axis, capped at MAX_CM."""
    if echo.value() == 1:                  # sensor still busy from last ping
        return MAX_CM
    trig.on()
    time.sleep_us(10)
    trig.off()
    pulse = time_pulse_us(echo, 1, 6000)   # 6 ms timeout, about 100 cm
    if pulse < 0:                          # no echo received
        return MAX_CM
    return min(pulse / 58, MAX_CM)         # 58 us per cm, there and back


# ================= START =================
set_motors(0, 0)
time.sleep(3)                              # time to put the robot down
led.on()
open_count = 0

# ================= NAVIGATION =================
while True:
    left = read_cm(trig_l, echo_l) * COS45     # perpendicular distance, left wall
    right = read_cm(trig_r, echo_r) * COS45    # perpendicular distance, right wall
    # print(left, right)                       # uncomment when tuning over USB

    # End of corridor: both walls gone for 5 readings in a row
    if left > OPEN_CM and right > OPEN_CM:
        open_count += 1
    else:
        open_count = 0
    if open_count >= 5:
        break

    # Centring: positive error = closer to the right wall, so steer left
    error = left - right
    if abs(error) <= DEADBAND:
        error = 0
    set_motors(max(0, BASE_SPEED - KP * error),
               max(0, BASE_SPEED + KP * error))

    time.sleep_ms(20)

# ================= END OF COURSE =================
set_motors(0, 0)
led.off()
# Task detection and the three tasks go here
