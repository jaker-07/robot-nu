# Project: AI-Powered Robot Maze Simulator & Training Environment

Build a Python-based 2D top-down digital twin of my existing maze-solving robot.

## Core objective
Create an interactive simulation that runs my existing robot code in a virtual version of the actual maze, allowing it to be tested, analysed and progressively improved.

## Technology
- Python as the core language.
- Pygame for the 2D visual simulation.
- Claude API (Anthropic) for the AI engineering agent.
- Optuna for automated parameter optimisation.
- SQLite for storing experiments and results.
- Streamlit for the training dashboard.
- Gymnasium / Stable-Baselines3 for potential future reinforcement learning.

## Key requirements

**1. Simulation**
- Accurately recreate the maze and robot.
- Simulate robot movement, collisions and sensor readings.
- Run the existing robot navigation code without rewriting it.
- Visualise movement, sensor readings and travelled paths in real time.

**2. Training and optimisation**
- Automatically run repeated maze attempts.
- Record sensor readings, decisions, failures and performance.
- Optimise existing code parameters.
- Measure completion rate, speed, distance and collisions.
- Compare experiments and retain the best-performing configurations.

**3. Claude AI engineering agent**
Integrate Claude through the Anthropic API.

Give the agent access to:
- Existing project source code.
- Simulation controls.
- Experiment results and historical data.
- Failure analysis.
- Parameter optimisation.
- Code inspection and proposed modifications.

The agent should independently analyse failures, identify weaknesses, propose improvements, run experiments and evaluate results.

Code modifications must be version-controlled and tested before being accepted.

**4. Dashboard**
Create an interface featuring:
- Live 2D maze simulation.
- Robot movement and sensor visualisation.
- Start, pause and reset controls.
- Training progress and performance graphs.
- Experiment history and comparisons.
- Claude AI conversational interface.

**5. Architecture**
Separate the robot control logic from the hardware interface, allowing the same navigation code to operate in both simulation and on the physical Raspberry Pi.

## Development priorities

Build in this order:

1. Working digital twin using the existing robot code.
2. Automated experiment and performance tracking.
3. Parameter optimisation.
4. Claude-powered AI engineering agent.
5. Advanced learning and reinforcement learning if beneficial.
6. Validation and transfer to the physical robot.

## Overall goal

Create an AI-assisted robotics development environment, not simply a maze simulator. The system should be able to repeatedly test the robot, understand why it fails, experiment with improvements and demonstrate measurable improvements in real-world-relevant performance.