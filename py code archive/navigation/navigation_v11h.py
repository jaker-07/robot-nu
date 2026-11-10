"""
navigation_v11h.py
Corridor navigation for the GE1501 AE1 robot.

Changes from v11g (tuning values unchanged):
- Sensors are read the same way as ultrasonic_test_v3: 30 ms timeout
  (was 6 ms), and the code waits for a busy sensor instead of skipping it.
- Log records a status letter per sensor: k = ok, n = no echo,
  b = echo line stuck high, so missing readings can be diagnosed.
- Big readings (>= BIG_CM, not normal in the corridor) are never used for
  steering, so a glitch or an opening can't make the robot lurch towards it.
- A side only counts as OPEN after BIG_CONFIRM big readings in a row
  (a one-off is a glitch).
- End of course: front < STOP_CM AND a side OPEN, for END_CONFIRM loops.
- Wall-ahead turns go towards the OPEN side (larger raw reading if neither).

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

# ================= TUNING (same as v11g) =================
BASE_SPEED = 45      # % power driving straight
MIN_SPEED = 30       # % lowest power sent to a motor
MAX_SPEED = 50       # % highest power sent to a motor
LEFT_TRIM = 1.0
RIGHT_TRIM = 0.9     # right motor is stronger (from trim_test_v1.py)
KP = 3               # % power change per cm of centring error
DEADBAND = 1         # cm: no correction while within +/-1 cm of centre
MAX_ERR = 8          # cm: largest error acted on
FRONT_TURN_CM = 15   # cm: wall closer than this ahead -> sharp turn
MAX_CM = 55          # readings are capped here; no echo also reads MAX_CM

# ================= END / GLITCH RULES (new) =================
BIG_CM = 40          # raw reading at or above this = "big" (not normal in corridor)
BIG_CONFIRM = 2      # big readings in a row before a side counts as OPEN
STOP_CM = 5          # cm: end = front closer than this AND a side OPEN
END_CONFIRM = 2      # loops in a row both end conditions must be true

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
    """One reading in cm along the sensor's axis, capped at MAX_CM, plus a
    status letter for the log: k = ok, n = no echo, b = echo stuck high."""
    t0 = time.ticks_ms()
    while echo.value() == 1:               # wait for the last ping to finish
        if time.ticks_diff(time.ticks_ms(), t0) > 25:
            return MAX_CM, "b"
    trig.on()
    time.sleep_us(10)
    trig.off()
    pulse = time_pulse_us(echo, 1, 30000)  # 30 ms timeout, as in ultrasonic_test_v3
    if pulse < 0:
        return MAX_CM, "n"
    return min(pulse / 58, MAX_CM), "k"


# ================= START =================
set_motors(0, 0)
time.sleep(3)                              # time to put the robot down
led.on()
left = 10.0                                # held wall distances (perpendicular),
right = 10.0                               # equal so the robot starts straight
big_left = 0                               # big readings in a row, per side
big_right = 0
end_count = 0                              # loops in a row with both end conditions

# ================= LOGGING =================
# Each run gets its own file (run_1.csv, run_2.csv, ...) so nothing is overwritten
run_num = 1
for name in os.listdir():
    if name.startswith("run_"):
        run_num += 1
log = open(f"run_{run_num}.csv", "w")
log.write("t_ms,raw_left,raw_right,front,st_l,st_r,st_f,left,right,error,action\n")
log_lines = 0
start_ms = time.ticks_ms()


def log_row(action):
    """One line per loop: time, raw readings, sensor status, held values, action."""
    t = time.ticks_diff(time.ticks_ms(), start_ms)
    log.write(f"{t},{raw_left:.1f},{raw_right:.1f},{front:.1f},{st_l},{st_r},{st_f},"
              f"{left:.1f},{right:.1f},{left - right:.1f},{action}\n")


# ================= NAVIGATION =================
while True:
    raw_left, st_l = read_cm(trig_l, echo_l)
    raw_right, st_r = read_cm(trig_r, echo_r)
    front, st_f = read_cm(trig_f, echo_f)

    # Big readings must repeat BIG_CONFIRM loops in a row to count as OPEN;
    # a one-off big reading is a glitch
    big_left = big_left + 1 if raw_left >= BIG_CM else 0
    big_right = big_right + 1 if raw_right >= BIG_CM else 0
    left_open = big_left >= BIG_CONFIRM
    right_open = big_right >= BIG_CONFIRM

    # End of course: front close AND a side OPEN, for END_CONFIRM loops in a row
    if front < STOP_CM and (left_open or right_open):
        end_count += 1
    else:
        end_count = 0
    if end_count >= END_CONFIRM:
        set_motors(0, 0)
        led.off()
        log_row("STOP")
        log.close()
        break

    # Steering only uses normal readings; big ones never pull the robot over
    if raw_left < BIG_CM:
        left = raw_left * COS45            # perpendicular distance, left wall
    if raw_right < BIG_CM:
        right = raw_right * COS45          # perpendicular distance, right wall

    if front < FRONT_TURN_CM:
        # Wall ahead: turn towards the OPEN side (larger raw reading if neither)
        if left_open != right_open:
            turn_left = left_open
        else:
            turn_left = raw_left > raw_right
        if turn_left:
            set_motors(0, MAX_SPEED)       # stop left wheel -> turn left
            action = "TURN_L"
        else:
            set_motors(MAX_SPEED, 0)       # stop right wheel -> turn right
            action = "TURN_R"
    else:
        # Centring: positive error = closer to the right wall, so steer left
        error = left - right
        if abs(error) <= DEADBAND:
            error = 0
        error = max(-MAX_ERR, min(MAX_ERR, error))
        set_motors(BASE_SPEED - KP * error, BASE_SPEED + KP * error)
        action = "CENTRE"

    log_row(action)
    log_lines += 1
    if log_lines % 5 == 0:                 # save to flash about every 0.2 s
        log.flush()

    time.sleep_ms(20)

# ================= END OF COURSE =================
# Robot is stopped here. Task detection and the three tasks go here.
