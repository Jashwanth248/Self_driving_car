# AutoDrive Lab v2 — Engineering design

## Data flow and components

`src/main.cpp` parses command-line flags and reads the checked-in pretrained synthetic logistic model. `Simulator` contains a smoothed A* route, one-dimensional speed PID, pure-pursuit steering, bicycle-model vehicle state, and a finite-state supervisory controller. Each `step()` creates an observation, calls the ML predictor, evaluates the stop rule, calculates throttle/brake/steering, integrates vehicle physics and returns telemetry. A CSV logger writes the result for test analysis and visual rendering. No real sensors or hardware are controlled.

![Architecture diagram](assets/architecture.svg)

## State transitions

`CRUISE → BRAKING → STOPPED`; a measured clearance violation transitions to `FAULT` and the scenario executable reports a failure. Once braking has started, cruise cannot be resumed. The PID integral is reset on transition, and minimum brake is required while still moving.

## Model lifecycle

Run `autodrive_train config/risk_model.txt` **offline** if you want to retrain on deterministic synthetic examples. The driving executable **loads** those four numerical weights. It does not train during startup. The logistic model is a toy classifier, not neural computer vision or calibrated collision risk.

## Constraints and known limitations

- Simulation-only: simplified kinematic bicycle, nominal acceleration constants, no tire dynamics, no actuator delay, no asynchronous sensing.
- A* returns a grid route. An iterative smoothing operation rounds the corner; the pure-pursuit controller follows those continuous waypoints.
- Obstacle is virtual and stationary beyond the goal along its final heading. No real detection, multi-object tracking, lane perception, or vehicle-specific braking certification.
- `boundary_crossed` uses actual Euclidean vehicle-to-obstacle distance instead of forcing the car to remain at a particular position.
- Telemetry is deterministic and intended for visualizations, regression tests and learning.
- Safety checks here should never be used to command real motors, steering, or braking hardware.

## Testing strategy

The C++ tests exercise A* route adjacency, blocked/unrectangular grids, PID calculations/reset, ML risk ordering and NaN handling, four speed configurations, active braking, non-negative bounded speeds, real clearance checks, turning, maximum route tracking error and post-stop stability. CTest runs the complete demo too. GitHub Actions also tests a Windows build and builds with sanitizers on Linux.

## Project tree

```text
include/autodrive/      interfaces (types, planning, control, perception, simulation, telemetry)
src/                    C++ implementations and two executables
tests/                  deterministic regression and scenario tests
config/                 versioned model weights, loaded at runtime
scripts/render_demo.py  generates dashboard plots and GIF/MP4 from CSV
.github/workflows/      CI: Linux, Windows, sanitizer, render artifacts
docs/assets/            architecture SVG, plots, GIF and MP4 demo
```
