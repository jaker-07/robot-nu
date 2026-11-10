"""
ultrasonic_test_v2.py
Prints left and right HC-SR04 readings. Run over USB in Thonny.

Changes from v1:
- Left sensor moved to TRIG GP28, ECHO GP27 (pin numbers set at the top).
- No echo now reads MAX_CM instead of -1.
"""

from machine import Pin, time_pulse_us
import time

# ---- Pins ----
L_TRIG = 28
L_ECHO = 27
R_TRIG = 20
R_ECHO = 21

MAX_CM = 400         # HC-SR04 max range; shown when nothing is in range

trig_l = Pin(L_TRIG, Pin.OUT, value=0)
echo_l = Pin(L_ECHO, Pin.IN)
trig_r = Pin(R_TRIG, Pin.OUT, value=0)
echo_r = Pin(R_ECHO, Pin.IN)


def read_cm(trig, echo):
    trig.on()
    time.sleep_us(10)
    trig.off()
    pulse = time_pulse_us(echo, 1, 30000)   # 30 ms timeout, about 5 m
    if pulse < 0:
        return MAX_CM                       # nothing in range
    return min(pulse / 58, MAX_CM)


while True:
    left = read_cm(trig_l, echo_l)
    time.sleep_ms(30)                       # let echoes die out before next sensor
    right = read_cm(trig_r, echo_r)
    print(f"L: {left:6.1f} cm   R: {right:6.1f} cm")
    time.sleep_ms(200)
