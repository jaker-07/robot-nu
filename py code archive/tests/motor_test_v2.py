"""
motor_test_v2.py
Motor test: L298N driven by Raspberry Pi Pico, run from battery as main.py.

Changes from v1:
- ENA/ENB driven from GP10/GP15 instead of jumper caps (caps must be OFF)
- Onboard LED shows status:
    blinking  = 3 s countdown
    solid on  = motors being driven
    off       = pause / finished

Wiring: ENA GP10, IN1 GP11, IN2 GP12, IN3 GP13, IN4 GP14, ENB GP15,
        L298N GND to Pico GND.
"""

from machine import Pin
import time

# ---- Pin setup ----
led = Pin(25, Pin.OUT)  # onboard LED
ena = Pin(10, Pin.OUT)  # left motor enable  (physical pin 14)
in1 = Pin(12, Pin.OUT)  # left motor         (physical pin 15)
in2 = Pin(11, Pin.OUT)  # left motor         (physical pin 16)
in3 = Pin(13, Pin.OUT)  # right motor        (physical pin 17)
in4 = Pin(14, Pin.OUT)  # right motor        (physical pin 19)
enb = Pin(15, Pin.OUT)  # right motor enable (physical pin 20)

# ---- Start with everything off ----
in1.off()
in2.off()
in3.off()
in4.off()
led.off()

# ---- Enable both motor channels (full speed) ----
ena.on()
enb.on()

# ---- Countdown: LED blinks once per second ----
print("Starting - wheels off the table")
for count in range(3, 0, -1):
    print(count)
    led.on()
    time.sleep(0.5)
    led.off()
    time.sleep(0.5)

# ---- Test 1: left motor forward ----
print("Left forward")
led.on()
in1.on()
in2.off()
time.sleep(2)
in1.off()
led.off()
time.sleep(1)

# ---- Test 2: right motor forward ----
print("Right forward")
led.on()
in3.on()
in4.off()
time.sleep(5)
in3.off()
led.off()
time.sleep(1)

# ---- Test 3: both forward ----
print("Both forward")
led.on()
in1.on()
in2.off()
in3.on()
in4.off()
time.sleep(2)
in1.off()
in3.off()
led.off()
time.sleep(1)

# ---- Test 4: both reverse ----
print("Both reverse")
led.on()
in2.on()
in4.on()
time.sleep(2)

# ---- Stop and disable ----
in1.off()
in2.off()
in3.off()
in4.off()
ena.off()
enb.off()
led.off()
print("Done")
