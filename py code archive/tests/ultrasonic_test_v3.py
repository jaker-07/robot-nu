"""
ultrasonic_test_v3.py
Prints left, front and right HC-SR04 readings. Run over USB in Thonny.

Changes from v2:
- Front sensor added (TRIG GP18, ECHO GP19).

Pins match navigation_v8.py. No echo reads MAX_CM.
"""

from machine import Pin, time_pulse_us
import time

# ---- Pins ----
L_TRIG = 28
L_ECHO = 27
F_TRIG = 18
F_ECHO = 19
R_TRIG = 20
R_ECHO = 21

MAX_CM = 400         # HC-SR04 max range; shown when nothing is in range

trig_l = Pin(L_TRIG, Pin.OUT, value=0)
echo_l = Pin(L_ECHO, Pin.IN)
trig_f = Pin(F_TRIG, Pin.OUT, value=0)
echo_f = Pin(F_ECHO, Pin.IN)
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
    front = read_cm(trig_f, echo_f)
    time.sleep_ms(30)
    right = read_cm(trig_r, echo_r)
    print(f"L: {left:6.1f} cm   F: {front:6.1f} cm   R: {right:6.1f} cm")
    time.sleep_ms(200)
