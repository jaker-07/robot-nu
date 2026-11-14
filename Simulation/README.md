# Simulation: a digital twin of the AE1 robot

Runs the robot's own program (`main_v21.py`, or any other version) unmodified against a
model of the robot and the course, shows the run from above, and searches the program's
constants for better values.

## Start

```
cd Simulation
python3 run.py
```

Python 3.8 or newer, nothing to install. The app opens in the browser at
`http://127.0.0.1:8765/`. `python3 run.py --check` prints a self-check without the browser.

## What is on the page

| Part | What it does |
|---|---|
| Top view | The course as dimensioned in the brief (legs 30.0 / 30.0 / 25.4 / 23.1 / 35.8 cm wide), the robot, its three sonar beams drawn to the distance the program was told, the path so far, and a red ring wherever it touched a wall. |
| Control strip | Play/pause, playback speed 0.25× to 16×, New run, which task signal the Decider gives, Vary (random start placement, battery, motors, wall positions), and which program to run. |
| Plot | Sensor readings and motor power over the run. Click or drag to move through it. |
| Constants | Every `NAME = number` in the program, as sliders. Changes apply to a copy in memory; the file is never edited. |
| Trials | Repeats the run 12 to 96 times with variation and reports success rate, contact-free rate, time and score. Click a square to replay that attempt. |
| Training | Searches the ticked constants for a better set, then checks it against the starting values on fresh attempts. Recommended only if it wins there. |
| Agent | A Claude conversation with tools over the twin; see below. |
| History | Every trial batch and training run, stored in `data/experiments.db`. |
| Calibration | Every physical number in the model, its value and its source. Edit one and the twin re-runs with it. |

"Export as candidate file" writes a copy of the program with the changed constants to
`Simulation/candidates/`. Nothing outside `Simulation/` is ever written.

## How it works

- `twin/shim.py` supplies fake `machine`, `time` and `neopixel` modules. The program's pin
  writes drive simulated motors; its sleeps and `time_pulse_us` calls advance a simulated
  clock, which steps the physics. A 35 s run takes about 0.15 s to compute.
- `twin/robot.py` is the drive (dead band, stall, lag, left/right mismatch), wall contact,
  and the HC-SR04 model (beam fan, weaker echoes off oblique walls, the 38 ms "busy" period
  after a missed echo, cross-talk between consecutive pings).
- `twin/track.py` builds the course from the brief's wall lengths and angles.
- `twin/sim.py` runs one attempt and scores it against the rubric bands (navigation 40,
  task 20). The score is an estimate for comparing settings, not a prediction of the mark.
- `twin/train.py` runs trials and the optimiser. `twin/store.py` is the SQLite history.
- `twin/agent.py` is the Claude tool loop over the Anthropic Messages API.
- The wiring in `twin/config.py` is the as-built wiring from the `main_v21.py` docstring.
  A program that addresses other pins reads nothing, as on the robot.

## How far to trust it

Fitted to three reported facts about v21: it completes, it stops at the Task Decider, and
a run takes about 20 s. The twin gives 17.7 s driving plus 2.9 s start-up lights.

Independent check: with `GAP_RF_MS = 0` the twin shows the one-loop false pivots that v20f
was written to remove.

Not validated, in order of importance:

1. **Robot dimensions** are scaled from site photos (the 165 mm breadboard as reference),
   good to about ±1 cm. The 19 cm sensor span matters most in the 23.1 cm leg.
2. **Sonar behaviour at oblique angles** is fitted so that v21 gets round the bends without
   false stops, as it does in reality. No sensor log was available to fit it properly.
3. **Task Decider** size and position are scaled from the brief's figure and centred on
   the corridor.
4. **Spin**: the twin predicts about 373° for `SPIN_TIME = 1.5`. The turn-scrub factor
   behind that is a guess; measure one real spin and correct it in Calibration.
5. **Low motor power**: below about 30 % the stall model is the least certain part.

Each of these is one row in the Calibration panel. Measuring the robot (five lengths), one
3 s straight-line distance and one spin angle would remove most of the guesswork.

## Tests

```
python3 -m unittest discover -s tests -v
```

## The Claude agent

The Agent tab gives Claude tools over the twin: read the program, run attempts and
trials, train constants, and propose constant or code changes. Each proposal is tested
against the original on the same 24 varied attempts and written to
`Simulation/candidates/` only if it wins. The agent cannot edit the robot's own files.

To enable it, put your Anthropic API key on one line in `Simulation/anthropic_key.txt`
(ignored by git), or set `ANTHROPIC_API_KEY` before starting. The default model is
`claude-sonnet-5-5`; the tab has a selector. Calls go straight from your computer to
`api.anthropic.com` and are billed to your key. A training request can use several
hundred simulated attempts but only a handful of API calls.

The agent has been tested with scripted replies only, not against the live API.
