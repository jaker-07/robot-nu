"""
ultrasonic_test_v4a.py
Live stream of all three ultrasonic sensors. Run over USB in Thonny with the
battery on; press Stop to end. "---" = no echo.

Pins (as wired): L TRIG GP16 ECHO GP17 · F TRIG GP18 ECHO GP19 · R TRIG GP20 ECHO GP21
"""

from machine import Pin, time_pulse_us
import time

trig_l = Pin(16, Pin.OUT, value=0)
echo_l = Pin(17, Pin.IN)
trig_f = Pin(18, Pin.OUT, value=0)
echo_f = Pin(19, Pin.IN)
trig_r = Pin(20, Pin.OUT, value=0)
echo_r = Pin(21, Pin.IN)


def read_cm(trig, echo):
    trig.on()
    time.sleep_us(10)
    trig.off()
    pulse = time_pulse_us(echo, 1, 30000)   # 30 ms timeout, about 5 m
    if pulse < 0:
        return None                         # no echo
    return pulse / 58


def show(x):
    return "   ---" if x is None else f"{x:6.1f}"


while True:
    left = read_cm(trig_l, echo_l)
    time.sleep_ms(30)                       # let echoes die out before next sensor
    front = read_cm(trig_f, echo_f)
    time.sleep_ms(30)
    right = read_cm(trig_r, echo_r)
    print(f"L: {show(left)} cm   F: {show(front)} cm   R: {show(right)} cm")
    time.sleep_ms(200)
