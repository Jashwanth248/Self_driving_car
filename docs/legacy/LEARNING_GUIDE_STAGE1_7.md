# AutoDrive Lab: Simple, human explanation

## What happens first?
Open `src/main.cpp`. The `main()` function starts the program and creates `Simulator simulator;`. The simulator constructs a car and trains a small ML model on **synthetic examples**. Then `planned_route()` independently demonstrates pathfinding. Every loop iteration calls `simulator.step(0.1)`: one tenth of a simulated second.

## Stage 1: Read sensors
The program pretends there is a stationary obstacle at position 36 meters. It subtracts the car position from 36 to estimate distance. Real cars would use camera, LiDAR and radar. We are NOT reading real sensors here.

## Stage 2: Machine learning
`RiskModel::train()` makes synthetic labeled examples. An example might be "distance 3 meters, closing speed 12 m/s, same lane" labeled dangerous. The logistic-regression model has four weights: bias, normalized distance, normalized closing speed and normalized lane offset. Gradient descent changes the weights to reduce classification errors. `predict()` returns a probability-like **score**, not a calibrated real collision probability.

## Stage 3: Determine desired speed
The simulator combines the ML risk with a simple physics-based stopping-distance rule. If too close, it requests target speed 0. The ML model is NOT the sole braking authority.

## Stage 4: PID controller
`speed_pid_.update(target-car_.speed, dt)` measures the error between desired and current speed. P means present error, I means accumulated error and D means rate of change. The output is converted into throttle (positive) or brake (negative).

## Stage 5: Update car physics
The simulation calculates acceleration from throttle and brake, updates speed and position and returns the new values. A final artificial boundary keeps the car from crossing the obstacle buffer. This is **not** a proven real-world safety feature.

## Stage 6: Print results
Back inside `main()`, telemetry appears once per simulated second: time, position, speed, obstacle distance, ML score, throttle and brake.

## A* pathfinding (a separate subsystem)
`Planner::route()` reads an ASCII map. Dots are open cells; # means blocked. A* searches neighboring cells using distance traveled plus Manhattan distance to the goal. It reconstructs a path. **The current car drives straight and does not follow this route.**

## Beginner tasks
1. Find `int main()` and identify where the loop begins.
2. Change `obstacle_x` from 36 to 50; the test expects 34 so update tests too.
3. Change target-speed maximum from 9 to 6.
4. Print a warning when ML risk is above 0.5.
5. Add a new unit test for the A* planner.

## Interview explanation
"I built an educational C++17 car simulator with deterministic physics, synthetic training data for logistic-regression risk classification, A* route planning, PID longitudinal control, unit tests, and GitHub CI. It is a simulator, not production autonomous driving software."

## Next milestones
Waypoints + bicycle-model steering, more sensor models, neural vision perception, ROS 2 integration, CARLA adapter, scenario testing and performance benchmarking.
