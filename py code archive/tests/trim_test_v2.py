"""
trim_test_v2.py
Straight-line trim test at the navigation speed. No sensors and no steering:
both wheels get the same power, so any curve is motor mismatch.

Changes from trim_test_v1:
- BASE_SPEED 35 -> 45, the speed navigation_v13b actually drives at
  (the 0.9 trim was measured at 35 %).
- Starts from the current trims (left 1.0, right 0.9).

How to use:
1. Mark a start line and the robot's centre on the floor.
2. Switch on; the LED blinks for 3 s, then the robot drives for RUN_TIME s.
3. Measure sideways drift at the end (left or right, in cm).
4. Curves right -> raise RIGHT_TRIM by 0.02-0.05. Curves left -> lower it.
   Re-run until drift is under about 2 cm, then copy RIGHT_TRIM into the
   navigation code.
"""

from machine import Pin, PWM
import time

# ================= SETTINGS =================
BASE_SPEED = 45      # % power, same as navigation_v13b
LEFT_TRIM = 1.0
RIGHT_TRIM = 0.9     # current value; adjust in steps of 0.02-0.05
RUN_TIME = 3         # seconds of driving (about 1 m at this speed)

# ================= MOTORS (same as navigation_v13b) =================
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
