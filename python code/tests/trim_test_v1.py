"""
trim_test_v1.py
Straight-line test for motor trim. No sensors and no steering: both wheels
get the same power, so any curve is motor mismatch.
Motor setup matches navigation_v7.py.

How to use:
1. Mark a start line and the robot's centre on the floor.
2. Switch on; the LED blinks for 3 s, then the robot drives for RUN_TIME s.
3. Measure distance travelled and sideways drift at the end.
4. Change one trim, re-run, log the result.
"""

from machine import Pin, PWM
import time

# ================= SETTINGS =================
BASE_SPEED = 35      # % power, same as navigation_v7
LEFT_TRIM = 1.0
RIGHT_TRIM = 1.0     # start at 1.0 to see the raw mismatch, then try 0.95
RUN_TIME = 4         # seconds of driving

# ================= MOTORS (same as navigation_v7) =================
left_en = PWM(Pin(10))          # ENA
left_fwd = Pin(12, Pin.OUT)     # IN2
left_rev = Pin(11, Pin.OUT)     # IN1
right_en = PWM(Pin(15))         # ENB
right_fwd = Pin(14, Pin.OUT)    # IN4
right_rev = Pin(13, Pin.OUT)    # IN3
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
