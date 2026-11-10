"""
ultrasonic_test_v1.py
Prints left and right HC-SR04 readings. Run over USB in Thonny.
Wiring: L TRIG GP16, ECHO GP17 / R TRIG GP20, ECHO GP21
"""

from machine import Pin, time_pulse_us
import time

trig_l = Pin(16, Pin.OUT, value=0)
echo_l = Pin(17, Pin.IN)
trig_r = Pin(20, Pin.OUT, value=0)
echo_r = Pin(21, Pin.IN)


def read_cm(trig, echo):
    trig.on()
    time.sleep_us(10)
    trig.off()
    pulse = time_pulse_us(echo, 1, 30000)   # 30 ms timeout, about 5 m
    if pulse < 0:
        return -1                           # no echo
    return pulse / 58


while True:
    left = read_cm(trig_l, echo_l)
    time.sleep_ms(30)                       # let echoes die out before next sensor
    right = read_cm(trig_r, echo_r)
    print(f"L: {left:.1f} cm   R: {right:.1f} cm")
    time.sleep_ms(200)
