"""Calibration table for the digital twin.

Every physical number the twin uses lives here, with where it came from:

  brief      dimensioned in the GE1501 AE1 assessment brief
  figure     scaled off the brief's undimensioned figure
  code       stated in the robot's own code or its docstrings
  log        from the testing log / changelog / trim tests
  sketch     your top-view CAD sketch of the plate (18 Nov)
  photo      estimated from project-site photos
  datasheet  HC-SR04 / L298N / TT-motor typical values
  fitted     tuned so the twin reproduces reported real-world results
  assumed    no evidence; a plausible placeholder, the first things to measure
  user       stated by you
  measured   entered in the app

Values changed through the app are stored in Simulation/calibration.json and
override these defaults. Units: cm, s, degrees, % motor power.
"""
from __future__ import annotations

import json
import os

CAL_ENTRIES = [
    # ---- robot geometry. Plate from the top-view CAD sketch (18 Nov); front = the 65 mm edge,
    # whose 46.6 degree chamfers carry the angled sensors. Robot frame: origin at the midpoint
    # of the drive axle, x forward, y left.
    ("robot.plate_width_cm", 15.0, "sketch", "Plate width across the straight sides"),
    ("robot.plate_length_cm", 17.2, "sketch", "Plate length, front edge to rear edge"),
    ("robot.plate_front_cm", 6.5, "sketch", "Front edge of the plate"),
    ("robot.plate_rear_cm", 7.5, "sketch", "Rear edge of the plate"),
    ("robot.plate_side_cm", 6.2, "sketch", "Straight sides of the plate"),
    ("robot.plate_rear_chamfer_cm", 6.5, "sketch", "Depth of the rear chamfers (120 degree corners); the front chamfers take the rest"),
    ("robot.axle_cm", 7.6, "assumed", "Axle line, back from the plate's front edge (now: middle of the straight sides)"),
    ("robot.track_cm", 12.4, "assumed", "Wheel centre to wheel centre (now: tyre outer faces in line with the plate sides)"),
    ("robot.tyre_width_cm", 2.6, "datasheet", "Tyre width (TT motor wheel)"),
    ("robot.wheel_dia_cm", 6.5, "datasheet", "Wheel diameter (TT motor wheel)"),
    ("robot.sensor_depth_cm", 1.4, "datasheet", "Plate edge to ultrasonic sensor face (HC-SR04 board plus transducers)"),
    ("robot.side_sensor_angle_deg", 0.0, "assumed", "Side sensor axis from straight ahead (0 = square to the chamfer face, 46.6 degrees on this plate)"),
    ("robot.ir_back_cm", 1.5, "assumed", "IR sensor on the mast, behind the plate's front edge"),
    ("robot.rear_overhang_cm", 0.0, "assumed", "Anything sticking out past the plate's rear edge (breadboard, battery)"),
    # ---- drive
    ("motor.speed_gain", 0.90, "fitted", "cm/s per % of power above the dead band; fitted to the ~20 s course time"),
    ("motor.dead_run_pct", 18.0, "assumed", "Power below which a turning wheel stops"),
    ("motor.dead_start_pct", 23.0, "log", "Power needed to start from rest (MIN_SPEED 25: 'below this it stalls')"),
    ("motor.right_strength", 1.087, "log", "Right motor is stronger: 1 / 0.92, the trim that drove straight"),
    ("motor.tau_s", 0.12, "assumed", "Speed-up time constant"),
    ("motor.tau_stop_s", 0.07, "assumed", "Stop time constant (gearbox friction)"),
    ("motor.turn_scrub", 0.85, "assumed", "Fraction of the ideal turn rate achieved when pivoting or spinning"),
    # ---- ultrasonic sensors
    ("sonar.beam_full_deg", 11.0, "fitted", "Beam half-angle with full sensitivity (fitted to the recorded results of ten program versions)"),
    ("sonar.beam_edge_deg", 13.0, "fitted", "Beam half-angle beyond which nothing is heard"),
    ("sonar.inc_full_deg", 30.0, "fitted", "Wall incidence angle up to which an echo is certain"),
    ("sonar.graze_range_cm", 60.0, "fitted", "Reach of an echo off a wall met at a grazing angle (matte board walls)"),
    ("sonar.edge_gain", 0.65, "fitted", "Reach of the outermost rays of the beam, as a fraction of the axis"),
    ("sonar.min_range_cm", 2.0, "datasheet", "Closest distance a sensor can read; nearer than this it reports nothing"),
    ("sonar.noise_cm", 0.3, "datasheet", "Reading noise (1 sigma)"),
    ("sonar.dropout", 0.01, "assumed", "Chance a ping returns nothing"),
    ("sonar.xtalk_p0", 0.30, "assumed", "Cross-talk strength: chance a sensor hears the previous sensor's ping"),
    ("sonar.xtalk_len_cm", 50.0, "assumed", "Cross-talk decay length (sound path travelled since the previous ping)"),
    ("sonar.no_echo_ms", 38.0, "datasheet", "ECHO stays high this long when nothing returns (sensor 'busy')"),
    ("sonar.rise_delay_us", 460.0, "datasheet", "TRIG to ECHO rising edge"),
    # ---- course
    ("track.uniform_width_cm", 30.0, "user", "Corridor width used for every leg (0 = inner wall from the brief's own lengths, which narrows and widens)"),
    ("track.wall_radius_cm", 0.3, "assumed", "Half-thickness of a wall for contact"),
    ("decider.gap_cm", 13.0, "brief", "End of outer wall to the Task Decider"),
    ("decider.offset_cm", -1.0, "assumed", "Task Decider face start, measured across from the outer wall line (-1 = centred in the corridor)"),
    ("decider.width_cm", 24.0, "figure", "Task Decider face width"),
    ("decider.depth_cm", 14.0, "figure", "Task Decider depth"),
    ("decider.delay_s", 1.5, "assumed", "Robot at rest to flag appearing"),
    ("decider.flag_reach_cm", 20.0, "assumed", "How far in front of the Decider the IR sensor still sees the flag"),
    ("start.tail_cm", 0.0, "assumed", "Start position: rearmost point of the robot this far past the start line"),
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
#
# One thing changed during the build: the right motor was turned round on 11 Nov
# (trim_test_v3), so "right wheel forward" moved from IN4 to IN3. Programs written before
# that drive IN4. HARDWARE "auto" works out which build a program was written for from
# its first drive command and runs it on that build; "now" and "nov10" force one.
HARDWARE = ("auto", "now", "nov10")
WIRING = {
    "motor_left": {"en": 10, "fwd": 12, "rev": 11},     # ENA, IN2, IN1
    "motor_right": {"en": 15, "fwd": 13, "rev": 14},    # ENB, IN3, IN4
    "sonar": {"L": (16, 17), "F": (18, 19), "R": (20, 21)},   # (TRIG, ECHO)
    "ir": 22,
    "buzzer": 26,
    "strip": 27,
    "led": 25,
}


# Model values nobody has measured are also varied from attempt to attempt in trials and
# training (relative 1-sigma below), so a setting has to work across the plausible range
# rather than for one guess. A value stops being varied once you enter a measurement.
UNCERTAIN = {
    "motor.dead_run_pct": 0.15, "motor.dead_start_pct": 0.06, "motor.tau_s": 0.30, "motor.tau_stop_s": 0.30,
    "motor.turn_scrub": 0.07, "sonar.beam_edge_deg": 0.12, "sonar.inc_full_deg": 0.15,
    "sonar.graze_range_cm": 0.30, "sonar.edge_gain": 0.30, "sonar.dropout": 0.50, "sonar.xtalk_p0": 0.50,
    "sonar.xtalk_len_cm": 0.30, "decider.delay_s": 0.30, "decider.flag_reach_cm": 0.10,
}


def with_uncertainty(values, sources):
    """Attach the list of still-unmeasured values to a calibration dict."""
    out = dict(values)
    out["_unc"] = {k: s for k, s in UNCERTAIN.items() if sources.get(k) in ("assumed", "fitted")}
    return out


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
