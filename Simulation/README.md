# Simulation: a digital twin of the AE1 robot

Runs the robot's own program (`main_v21.py`, or any other version) unmodified against a
model of the robot and the course, shows the run from above, and searches the program's
constants for better values.

## Start

```
cd Simulation
python3 run.py
```

Python 3.8 or newer, nothing to install. The app opens at `http://127.0.0.1:8765/`.

| Command | What it does |
|---|---|
| `python3 run.py` | Open the app |
| `python3 run.py --check` | Run the latest `main_vNN.py` and print the result |
| `python3 run.py --history` | Compare the twin with every result recorded in the testing log |
| `python3 -m unittest discover -s tests` | Run the tests |

## What is on the page

| Part | What it does |
|---|---|
| Top bar | Program, task signal, hardware build, Vary conditions, New run. |
| Top view | The course on a cm grid, the robot, its three sonar beams drawn to the distance the program was told, the path, and a cross wherever it touched a wall. |
| Telemetry / Result | Live values at the playback position; outcome and score of the run. |
| Robot size | Width and length sliders. The rest of the robot scales with them and the run repeats. |
| Plot | Sensor readings and motor power. Click or drag to move through the run. |
| Constants | Every `NAME = number` in the program, as sliders. The file is never edited. |
| Trials | Repeats the run with varied conditions; click a square to replay an attempt. |
| Training | Searches the ticked constants, then checks the best set on fresh attempts. |
| Agent | A Claude conversation with tools over the twin. Needs an API key (below). |
| Log check | Runs every program with a recorded real result and compares. |
| History | Every trial batch and training run (`data/experiments.db`). |
| Calibration | Every physical number in the model, its value and source. |

Nothing outside `Simulation/` is ever written.

## The course

The outer wall is taken from the brief (80, 93, 84, 63, 70 cm; 90 and 120 degree
corners) and the inner wall is drawn 30 cm from it, so every leg is 30 cm wide. The
brief's own inner-wall lengths do not give a constant width (30.0 / 30.0 / 25.4 / 23.1 /
35.8 cm); setting `track.uniform_width_cm` to 0 in Calibration uses them instead.

## Hardware builds

The right motor was turned round on 11 Nov, which moved "right wheel forward" from IN4 to
IN3. A program written before that drives IN4. With Build on `auto` the twin reads the
build from the program's first drive command and runs it on the robot it was written
for; before v13b that robot also had a left sensor that returned nothing, because the
code addressed pins the sensor was not on.

## How far to trust it

`QUESTIONNAIRE.md` lists everything still guessed. Three safeguards against a model that
only suits one version:

1. **Log check.** The sonar model is fitted to the recorded outcomes of ten program
   versions at once, not to v21. Eight are reproduced. The two that are not both concern
   stopping at under 5 cm from a wall (v11g stopped early in reality, v12 did not stop at
   the Decider; the twin has it the other way round).
2. **Unmeasured values vary.** In trials and training, every model value still marked
   assumed or fitted is redrawn within a plausible range on each attempt. A result has to
   hold across that range. A value stops varying once you enter a measurement.
3. **Validation before recommendation.** A trained or agent-proposed setting is compared
   with the original on fresh attempts and recommended only if it wins.

What the twin predicts for v21 on the 30 cm course: completes and stops in every attempt
with no wall contact, 17.4 s driving. The uncertain part is the spin: the turn angle
depends on drive values nobody has measured, and comes out more than 45 degrees off in
about 1 attempt in 12.

## The Claude agent

Put your Anthropic API key on one line in `Simulation/anthropic_key.txt` (ignored by
git), or set `ANTHROPIC_API_KEY`. The agent can read the program, run attempts and
trials, train constants, and propose constant or code changes. A proposal is written to
`Simulation/candidates/` only if it beats the original on the same varied attempts. It
has been tested with scripted replies only, not against the live API.

## Files

| File | Role |
|---|---|
| `twin/shim.py` | Fake `machine`, `time`, `neopixel`; the simulated clock and world |
| `twin/robot.py` | Drive, wall contact, HC-SR04 model |
| `twin/track.py` | The course |
| `twin/sim.py` | One attempt and its score (navigation 40 + task 20, mirroring the rubric) |
| `twin/train.py` | Trials and the optimiser |
| `twin/history.py` | The recorded results and the log check |
| `twin/agent.py` | Claude tool loop |
| `twin/config.py` | Calibration table, wiring, uncertainty ranges |
| `twin/params.py` | Finds and substitutes the program's constants |
| `twin/store.py`, `twin/server.py`, `web/index.html` | History, local server, page |
