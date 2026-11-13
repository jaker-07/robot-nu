"""
motor_test_v3.py
Motor check after rotating the right motor 180 deg (right forward is now IN3).
Wheels OFF the ground, battery ON, run over USB in Thonny and watch the shell.

Pick a test with MODE:
  "direction"  each wheel forward then reverse, 1.5 s each.
               Both wheels should spin the robot's forward way when it says FORWARD.
  "sweep"      each wheel on its own from 25 % to 65 % in 5 % steps, 2 s per step.
               Note the lowest % where each wheel turns steadily (no humming or stutter).
               That is its stall point.
  "pair"       both wheels together at BASE_SPEED with the trims, 5 s.
               Compare the two wheels by eye and sound.

Motor wiring (same as navigation_v13d):
  Left  = channel A: ENA GP10, forward IN2 GP12, reverse IN1 GP11
  Right = channel B: ENB GP15, forward IN3 GP13, reverse IN4 GP14  (flipped)
"""

from machine import Pin, PWM
import time

# ================= SETTINGS =================
MODE = "sweep"       # "direction", "sweep" or "pair"
BASE_SPEED = 50      # % for "pair", same as navigation_v13d
LEFT_TRIM = 1.0
RIGHT_TRIM = 1.05

# ================= MOTORS =================
left_en = PWM(Pin(10))          # ENA
left_fwd = Pin(12, Pin.OUT)     # IN2
left_rev = Pin(11, Pin.OUT)     # IN1
right_en = PWM(Pin(15))         # ENB
right_fwd = Pin(13, Pin.OUT)    # IN3  (flipped: right motor rotated 180 deg)
right_rev = Pin(14, Pin.OUT)    # IN4
left_en.freq(1000)
right_en.freq(1000)
led = Pin(25, Pin.OUT)

MOTORS = {"LEFT": (left_en, left_fwd, left_rev),
          "RIGHT": (right_en, right_fwd, right_rev)}


def run(name, power, forward=True):
    """Drive one motor at power % (0 = stop)."""
    en, fwd, rev = MOTORS[name]
    if forward:
        fwd.on()
        rev.off()
    else:
        fwd.off()
        rev.on()
    en.duty_u16(int(power * 655))


def stop_all():
    for name in MOTORS:
        run(name, 0)


# ================= COUNTDOWN =================
stop_all()
print(f"MODE = {MODE}. Wheels off the ground. Starting in 3 s")
for count in range(3, 0, -1):
    led.on()
    time.sleep(0.5)
    led.off()
    time.sleep(0.5)
led.on()

# ================= TESTS =================
if MODE == "direction":
    for name in ("LEFT", "RIGHT"):
        print(f"{name} FORWARD")
        run(name, 50, True)
        time.sleep(1.5)
        run(name, 0)
        time.sleep(0.5)
        print(f"{name} REVERSE")
        run(name, 50, False)
        time.sleep(1.5)
        run(name, 0)
        time.sleep(1)

elif MODE == "sweep":
    for name in ("LEFT", "RIGHT"):
        print(f"--- {name} wheel ---")
        for power in range(25, 70, 5):
            print(f"{name} {power} %")
            run(name, power, True)
            time.sleep(2)
        run(name, 0)
        time.sleep(1.5)

elif MODE == "pair":
    print(f"Both forward: left {BASE_SPEED * LEFT_TRIM:.1f} %, right {BASE_SPEED * RIGHT_TRIM:.1f} %")
    run("LEFT", BASE_SPEED * LEFT_TRIM, True)
    run("RIGHT", BASE_SPEED * RIGHT_TRIM, True)
    time.sleep(5)

else:
    print("Unknown MODE; use direction, sweep or pair")

stop_all()
led.off()
print("Done")
