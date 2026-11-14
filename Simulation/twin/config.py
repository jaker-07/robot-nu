"""Calibration table for the digital twin.

Every physical number the twin uses lives here, with where it came from:

  brief      dimensioned in the GE1501 AE1 assessment brief
  figure     scaled off the brief's undimensioned figure
  code       stated in the robot's own code or its docstrings
  log        from the testing log / changelog / trim tests
  photo      estimated from project-site photos (830-point breadboard, 165 mm, as scale)
  datasheet  HC-SR04 / L298N / TT-motor typical values
  fitted     tuned so the twin reproduces a reported real-world result
  assumed    no evidence; a plausible placeholder, the first things to measure

Values changed through the app are stored in Simulation/calibration.json and
override these defaults. Units: cm, s, degrees, % motor power.
"""
from __future__ import annotations

import json
import os

CAL_ENTRIES = [
    # ---- robot geometry (origin = midpoint of the drive axle, x forward, y left)
    ("robot.track_cm", 12.6, "photo", "Wheel centre to wheel centre"),
    ("robot.body_half_width_cm", 7.6, "photo", "Axle line to outer tyre face"),
    ("robot.sensor_span_cm", 19.0, "photo", "Outer tip to outer tip of the two 45 degree sensors (widest point)"),
    ("robot.front_reach_cm", 8.5, "photo", "Axle to front sensor face"),
    ("robot.rear_reach_cm", 12.0, "photo", "Axle to rear edge of plate / breadboard"),
    ("robot.side_sensor_x_cm", 3.3, "photo", "Side sensor face centre, ahead of the axle"),
    ("robot.side_sensor_y_cm", 7.6, "photo", "Side sensor face centre, off the centreline"),
    ("robot.side_sensor_angle_deg", 45.0, "code", "Left/right sensors point 45 degrees outwards (v21 docstring)"),
    ("robot.ir_x_cm", 6.0, "photo", "IR module on the mast, ahead of the axle"),
    # ---- drive
    ("motor.speed_gain", 0.90, "fitted", "cm/s per % of power above the dead band; fitted to the ~20 s course time"),
    ("motor.dead_run_pct", 18.0, "assumed", "Power below which a turning wheel stops"),
    ("motor.dead_start_pct", 23.0, "log", "Power needed to start from rest (MIN_SPEED 25: 'below this it stalls')"),
    ("motor.right_strength", 1.087, "log", "Right motor is stronger: 1 / 0.92, the trim that drove straight"),
    ("motor.tau_s", 0.12, "assumed", "Speed-up time constant"),
    ("motor.tau_stop_s", 0.07, "assumed", "Stop time constant (gearbox friction)"),
    ("motor.turn_scrub", 0.85, "assumed", "Fraction of the ideal turn rate achieved when pivoting or spinning"),
    # ---- ultrasonic sensors
    ("sonar.beam_full_deg", 15.0, "datasheet", "Half-angle with full sensitivity"),
    ("sonar.beam_edge_deg", 30.0, "assumed", "Half-angle beyond which nothing is heard"),
    ("sonar.inc_full_deg", 30.0, "assumed", "Wall incidence angle up to which an echo is certain"),
    ("sonar.graze_range_cm", 70.0, "fitted", "Reach of an echo off a wall met at a grazing angle (matte board walls)"),
    ("sonar.edge_gain", 0.5, "assumed", "Reach of the outermost rays of the beam, as a fraction of the axis"),
    ("sonar.noise_cm", 0.3, "datasheet", "Reading noise (1 sigma)"),
    ("sonar.dropout", 0.01, "assumed", "Chance a ping returns nothing"),
    ("sonar.xtalk_p0", 0.30, "assumed", "Cross-talk strength: chance a sensor hears the previous sensor's ping"),
    ("sonar.xtalk_len_cm", 50.0, "assumed", "Cross-talk decay length (sound path travelled since the previous ping)"),
    ("sonar.no_echo_ms", 38.0, "datasheet", "ECHO stays high this long when nothing returns (sensor 'busy')"),
    ("sonar.rise_delay_us", 460.0, "datasheet", "TRIG to ECHO rising edge"),
    # ---- course
    ("track.wall_radius_cm", 0.3, "assumed", "Half-thickness of a wall for contact"),
    ("decider.gap_cm", 13.0, "brief", "End of outer wall to the Task Decider"),
    ("decider.offset_cm", 5.9, "fitted", "Task Decider face start, measured across from the outer wall line (5.9 = centred)"),
    ("decider.width_cm", 24.0, "figure", "Task Decider face width"),
    ("decider.depth_cm", 14.0, "figure", "Task Decider depth"),
    ("decider.delay_s", 1.5, "assumed", "Robot at rest to flag appearing"),
    ("decider.flag_reach_cm", 16.0, "assumed", "How far in front of the Decider the IR sensor still sees the flag"),
    ("start.axle_y_cm", 12.0, "assumed", "Start position: axle this far past the start line (tail on the line)"),
    # ---- run-to-run variation used by trials and training
    ("vary.start_lat_cm", 1.5, "assumed", "Start placement error, sideways (1 sigma)"),
    ("vary.start_heading_deg", 3.0, "assumed", "Start placement error, heading (1 sigma)"),
    ("vary.speed_pct", 4.0, "assumed", "Battery / floor speed variation (1 sigma)"),
    ("vary.mismatch_pct", 2.0, "log", "Left/right motor mismatch left after trimming (1 sigma)"),
    ("vary.wall_len_cm", 0.4, "assumed", "Wall length tolerance of the built course (1 sigma)"),
    ("vary.wall_ang_deg", 0.4, "assumed", "Wall angle tolerance of the built course (1 sigma)"),
]

# As-built wiring (main_v21.py docstring). Code that addresses other pins reads nothing,
# exactly as on the robot.
WIRING = {
    "motor_left": {"en": 10, "fwd": 12, "rev": 11},     # ENA, IN2, IN1
    "motor_right": {"en": 15, "fwd": 13, "rev": 14},    # ENB, IN3, IN4
    "sonar": {"L": (16, 17), "F": (18, 19), "R": (20, 21)},   # (TRIG, ECHO)
    "ir": 22,
    "buzzer": 26,
    "strip": 27,
    "led": 25,
}


def defaults():
    return {k: v for k, v, _, _ in CAL_ENTRIES}


def load(path=None):
    """Return (values, sources). Saved overrides replace the defaults."""
    values = defaults()
    sources = {k: s for k, _, s, _ in CAL_ENTRIES}
    if path and os.path.exists(path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                saved = json.load(f)
            for k, v in saved.get("values", {}).items():
                if k in values:
                    values[k] = float(v)
                    sources[k] = saved.get("sources", {}).get(k, "measured")
        except (ValueError, OSError):
            pass
    return values, sources


def save(path, changes, source="measured"):
    """Merge {key: value} into the saved overrides."""
    saved = {"values": {}, "sources": {}}
    if os.path.exists(path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                saved = json.load(f)
        except (ValueError, OSError):
            pass
    base = defaults()
    for k, v in changes.items():
        if k not in base:
            continue
        saved.setdefault("values", {})[k] = float(v)
        saved.setdefault("sources", {})[k] = source
    with open(path, "w", encoding="utf-8") as f:
        json.dump(saved, f, indent=2, sort_keys=True)


def reset(path):
    if os.path.exists(path):
        with open(path, "w", encoding="utf-8") as f:
            json.dump({"values": {}, "sources": {}}, f)


def describe(values, sources):
    """Rows for the calibration panel."""
    return [{"key": k, "value": values[k], "default": v, "source": sources[k], "note": n}
            for k, v, _, n in CAL_ENTRIES]
