"""
trim_test_v3.py
Straight-line trim test with both motors running together. No sensors and no
steering, so any curve is motor mismatch.

Changes from trim_test_v2:
- Right motor flipped (rotated 180 deg): forward is now IN3 (GP13), reverse IN4 (GP14).
- BASE_SPEED 50 and RIGHT_TRIM 1.05, the current navigation_v13d values.

How to use:
1. Mark a start line and the robot's centre on the floor.
2. Switch on; the LED blinks for 3 s, then the robot drives for RUN_TIME s.
3. Measure sideways drift at the end.
4. Curves right -> raise RIGHT_TRIM by 0.02-0.05. Curves left -> lower it.
   Re-run until drift is under about 2 cm, then copy RIGHT_TRIM into the
   navigation code.
"""

from machine import Pin, PWM
import time

# ================= SETTINGS =================
BASE_SPEED = 50      # % power, same as navigation_v13d
LEFT_TRIM = 1.0
RIGHT_TRIM = 1.05    # current value; adjust in steps of 0.02-0.05
RUN_TIME = 3         # seconds of driving

# ================= MOTORS (same as navigation_v13d) =================
left_en = PWM(Pin(10))          # ENA
left_fwd = Pin(12, Pin.OUT)     # IN2
left_rev = Pin(11, Pin.OUT)     # IN1
right_en = PWM(Pin(15))         # ENB
right_fwd = Pin(13, Pin.OUT)    # IN3  (flipped: right motor rotated 180 deg)
right_rev = Pin(14, Pin.OUT)    # IN4
left_en.freq(1000)
right_en.freq(1000)
led = Pin(25, Pin.OUT)

left_fwd.on()
left_rev.off()
right_fwd.on()
right_rev.off()
left_en.duty_u16(0)
right_en.duty_u16(0)

# ================= COUNTDOWN =================
for count in range(3, 0, -1):
    print(count)
    led.on()
    time.sleep(0.5)
    led.off()
    time.sleep(0.5)

# ================= STRAIGHT RUN =================
print(f"Left {BASE_SPEED * LEFT_TRIM:.1f} %, right {BASE_SPEED * RIGHT_TRIM:.1f} %")
led.on()
left_en.duty_u16(int(BASE_SPEED * LEFT_TRIM * 655))
right_en.duty_u16(int(BASE_SPEED * RIGHT_TRIM * 655))
time.sleep(RUN_TIME)

# ================= STOP =================
left_en.duty_u16(0)
right_en.duty_u16(0)
led.off()
print("Done")
