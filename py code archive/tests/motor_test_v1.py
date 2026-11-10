"""
motor_test_v1.py
Motor test: L298N driven by Raspberry Pi Pico.
ENA/ENB jumpers fitted, so motors run at full speed whenever driven.
Wiring: IN1 GP11, IN2 GP12, IN3 GP13, IN4 GP14, L298N GND to Pico GND.
"""

from machine import Pin
import time

# ---- Pin setup ----
in1 = Pin(11, Pin.OUT)  # left motor  (physical pin 15)
in2 = Pin(12, Pin.OUT)  # left motor  (physical pin 16)
in3 = Pin(13, Pin.OUT)  # right motor (physical pin 17)
in4 = Pin(14, Pin.OUT)  # right motor (physical pin 19)

# ---- Start with everything off ----
in1.off()
in2.off()
in3.off()
in4.off()
print("Starting in 3 s - wheels off the table")
time.sleep(3)

# ---- Test 1: left motor forward ----
print("Left forward")
in1.on()
in2.off()
time.sleep(2)
in1.off()
time.sleep(1)

# ---- Test 2: right motor forward ----
print("Right forward")
in3.on()
in4.off()
time.sleep(2)
in3.off()
time.sleep(1)

# ---- Test 3: both forward ----
print("Both forward")
in1.on()
in2.off()
in3.on()
in4.off()
time.sleep(2)

# ---- Stop before reversing ----
in1.off()
in3.off()
time.sleep(1)

# ---- Test 4: both reverse ----
print("Both reverse")
in2.on()
in4.on()
time.sleep(2)

# ---- Stop ----
in1.off()
in2.off()
in3.off()
in4.off()
print("Done")
