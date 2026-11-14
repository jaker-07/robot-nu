"""
main_v21.py   GE1501 AE1 robot (Raspberry Pi Pico, MicroPython)

Drives a walled corridor using three ultrasonic sensors, stops at the Task Decider,
times how long the Decider's flag stays in front of the IR sensor, then does that task:
about 3 s = buzzer song, about 5 s = light, about 10 s = one spin on the spot.

Wiring
  L298N motor driver   ENA GP10, IN1 GP11, IN2 GP12, IN3 GP13, IN4 GP14, ENB GP15
                       (forward: left = IN2, right = IN3, as the right motor is mounted reversed)
  Ultrasonic sensors   TRIG/ECHO: left GP16/GP17, front GP18/GP19, right GP20/GP21
                       (left and right sensors point 45 degrees outwards)
  IR sensor            OUT GP22 (reads 0 while the flag is in front of it)
  Buzzer               GP26 to GND
  LED strip            WS2812, 5 LEDs, data in GP27
"""

from machine import Pin, PWM, time_pulse_us
import time
import neopixel


# =====================================================================
# SETTINGS: DRIVING
# =====================================================================
BASE_SPEED = 40      # % motor power when driving straight
MIN_SPEED = 25       # % lowest power sent to a moving motor (below this it stalls)
MAX_SPEED = 55       # % highest power sent to a motor
LEFT_TRIM = 1.0      # power multiplier, left motor
RIGHT_TRIM = 0.92    # power multiplier, right motor (it is the stronger one)
KP = 1.5             # % power change per cm of centring error
DEADBAND = 2         # cm of centring error ignored (about +/-1 cm off centre)
FRONT_TURN_CM = 8    # cm: wall closer than this ahead -> pivot on the spot
MAX_CM = 65          # cm: readings are capped here; no echo or a busy sensor read MAX_CM
STOP_CM = 12         # cm: end of course = front closer than this AND a side open...
END_CONFIRM = 3      # ...for this many loops in a row
GAP_RF_MS = 5        # ms pause between the right and front pings

# =====================================================================
# SETTINGS: TASKS
# =====================================================================
MIN_HOLD = 2.0       # s: flag sightings shorter than this are ignored
LIGHT_FROM = 4.0     # s: flag held less than this = buzzer (3 s signal)
SPIN_FROM = 7.5      # s: flag held at least this = spin (10 s signal); between = light (5 s)
SONG = [(2000, 1.0)]     # buzzer: (frequency in Hz, length in s) per note, 0 Hz = rest
LIGHT_TIME = 2.0     # s the strip stays on for the light task
GREEN = (0, 120, 0)  # strip colour (red, green, blue), each 0-255
BACKUP_TIME = 0.5    # s reversing away from the Task Decider before the spin
SPIN_SPEED = 55      # % power for the spin (capped at MAX_SPEED)
SPIN_TIME = 1.5      # s spinning: tune until the robot turns exactly 360 degrees


# =====================================================================
# HARDWARE SETUP
# =====================================================================
# Motors: (speed pin, forward pin, reverse pin)
LEFT_MOTOR = (PWM(Pin(10)), Pin(12, Pin.OUT), Pin(11, Pin.OUT))     # ENA, IN2, IN1
RIGHT_MOTOR = (PWM(Pin(15)), Pin(13, Pin.OUT), Pin(14, Pin.OUT))    # ENB, IN3, IN4
LEFT_MOTOR[0].freq(1000)             # 1 kHz PWM (unchanged since v1)
RIGHT_MOTOR[0].freq(1000)

# Ultrasonic sensors: (TRIG pin, ECHO pin)
SONAR = {"L": (Pin(16, Pin.OUT, value=0), Pin(17, Pin.IN)),
         "F": (Pin(18, Pin.OUT, value=0), Pin(19, Pin.IN)),
         "R": (Pin(20, Pin.OUT, value=0), Pin(21, Pin.IN))}

IR = Pin(22, Pin.IN)                     # 0 = flag in front, 1 = clear
BUZZER = Pin(26, Pin.OUT, value=0)       # plain on/off pin: tones are made in software
STRIP = neopixel.NeoPixel(Pin(27), 5)    # 5 LEDs
OFF = (0, 0, 0)


# =====================================================================
# MOTOR CONTROL
# =====================================================================
def drive(motor, power):
    """Run one motor at power % (negative = reverse, 0 = stop).
    A moving motor always gets between MIN_SPEED and MAX_SPEED."""
    speed_pin, forward_pin, reverse_pin = motor
    forward_pin.value(power > 0)
    reverse_pin.value(power < 0)
    if power != 0:
        power = max(MIN_SPEED, min(MAX_SPEED, abs(power)))
    speed_pin.duty_u16(int(power * 655))     # 655 = 1 % of the 0-65535 duty range


def set_motors(left, right):
    """Set both wheels in % (negative = reverse, 0 = stop), with the trims applied."""
    drive(LEFT_MOTOR, left * LEFT_TRIM)
    drive(RIGHT_MOTOR, right * RIGHT_TRIM)


# =====================================================================
# SENSORS
# =====================================================================
def read_cm(side):
    """One ping from sensor "L", "F" or "R". Returns the distance in cm along its beam,
    or MAX_CM if no echo comes back or the sensor is still busy with its last ping."""
    trig, echo = SONAR[side]
    if echo.value() == 1:                    # still busy from the last ping
        return MAX_CM
    trig.on()                                # 10 microsecond trigger pulse
    time.sleep_us(10)
    trig.off()
    pulse = time_pulse_us(echo, 1, 6000)     # echo time in us; 6 ms limit = about 1 m
    if pulse < 0:                            # no echo
        return MAX_CM
    return min(pulse / 58, MAX_CM)           # sound takes 58 us per cm, there and back


# =====================================================================
# LIGHTS
# =====================================================================
def show(colour):
    """Set the whole LED strip to one colour."""
    STRIP.fill(colour)
    STRIP.write()


def startup_lights():
    """Green start-up sequence, about 2 s: sweep along the strip, two flashes, fade out."""
    show(OFF)
    for i in range(5):                       # bright LED moves along, leaving a dim trail
        STRIP[i] = GREEN
        if i > 0:
            STRIP[i - 1] = (0, 25, 0)        # dim green
        STRIP.write()
        time.sleep_ms(180)
    for colour in (OFF, GREEN, OFF, GREEN):  # two flashes
        show(colour)
        time.sleep_ms(150)
    for level in range(120, -1, -8):         # fade from full green to off
        show((0, level, 0))
        time.sleep_ms(25)


# =====================================================================
# NAVIGATION
# =====================================================================
def navigate():
    """Drive the corridor and stop at the Task Decider.
    Every loop: read the three sensors, check for the end of the course,
    then either pivot (wall ahead) or steer to stay centred between the walls."""
    left = right = 10.0      # last good distance to each wall in cm (equal = start straight)
    turn = 0                 # pivot direction: 0 none, 1 left, -1 right
    end_count = 0            # loops in a row that looked like the end of the course

    while True:
        # 1. Read the sensors. The pauses stop one sensor hearing another's echo.
        raw_l = read_cm("L")
        time.sleep_ms(10)
        raw_r = read_cm("R")
        time.sleep_ms(GAP_RF_MS)
        front = read_cm("F")

        # 2. End of course: wall close ahead AND a side open, END_CONFIRM loops in a row
        if front < STOP_CM and max(raw_l, raw_r) >= MAX_CM:
            end_count += 1
            if end_count >= END_CONFIRM:
                set_motors(0, 0)
                return
        else:
            end_count = 0

        # 3. Wall distances: only good echoes update them; a miss keeps the last value.
        #    0.7071 = cos 45 deg, turns the 45-degree beam length into distance to the wall.
        if raw_l < MAX_CM:
            left = raw_l * 0.7071
        if raw_r < MAX_CM:
            right = raw_r * 0.7071

        # 4. Drive
        if front < FRONT_TURN_CM:
            # Wall ahead: pivot towards the side with more room, and keep
            # that direction until the front is clear (no flip-flopping mid-turn)
            if turn == 0:
                turn = 1 if left > right else -1
            if turn == 1:
                set_motors(0, MAX_SPEED)     # left wheel stopped -> turn left
            else:
                set_motors(MAX_SPEED, 0)     # right wheel stopped -> turn right
        else:
            turn = 0
            # Centring: positive error = closer to the right wall, so steer left
            error = left - right
            if abs(error) <= DEADBAND:
                error = 0
            error = max(-8, min(8, error))   # act on at most 8 cm of error (unchanged since v3)
            set_motors(BASE_SPEED - KP * error, BASE_SPEED + KP * error)

        time.sleep_ms(5)                     # short pause before the next loop


# =====================================================================
# TASK DETECTION
# =====================================================================
def detect_task():
    """Wait for the Task Decider's flag, time how long it stays in front of the
    IR sensor, and once it has gone return the task it asks for."""
    while True:
        while IR.value() == 1:               # wait for the flag
            time.sleep_ms(20)
        start = time.ticks_ms()
        while IR.value() == 0:               # time how long it stays
            time.sleep_ms(20)
        held = time.ticks_diff(time.ticks_ms(), start) / 1000     # seconds
        if held >= MIN_HOLD:                 # shorter = hand or glitch: keep waiting
            if held < LIGHT_FROM:
                return task_buzzer
            if held < SPIN_FROM:
                return task_light
            return task_spin


# =====================================================================
# TASKS
# =====================================================================
def task_buzzer():
    """Task 1: play SONG. Each note is made by switching the buzzer on and off
    at the note's frequency."""
    for freq, seconds in SONG:
        if freq == 0:                        # rest
            time.sleep(seconds)
        else:
            half = 500000 // freq            # half a wave in microseconds
            for _ in range(int(freq * seconds)):
                BUZZER.on()
                time.sleep_us(half)
                BUZZER.off()
                time.sleep_us(half)


def task_light():
    """Task 2: LED strip on for LIGHT_TIME seconds."""
    show(GREEN)
    time.sleep(LIGHT_TIME)
    show(OFF)


def task_spin():
    """Task 3: back away from the Task Decider, then one turn on the spot."""
    set_motors(-BASE_SPEED, -BASE_SPEED)     # reverse
    time.sleep(BACKUP_TIME)
    set_motors(0, 0)
    time.sleep(0.2)                          # brief stop between reversing and spinning
    set_motors(SPIN_SPEED, -SPIN_SPEED)      # left forward, right back = spin clockwise
    time.sleep(SPIN_TIME)
    set_motors(0, 0)


# =====================================================================
# MAIN PROGRAM
# =====================================================================
startup_lights()         # about 2 s...
time.sleep(1)            # ...plus 1 s = 3 s to put the robot down before it moves
navigate()               # drive until the Task Decider
task = detect_task()     # read the flag
task()                   # do that task
