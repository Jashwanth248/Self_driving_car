# Stage 7 — Natural braking and safety regression tests

## What was wrong?
The original simulator forcibly assigned `position.x=34` and `speed=0` when a proposed movement crossed a boundary. The unit test accepted this forced stop as success. It did not prove the braking controller could stop the car.

## Execution order, in plain English
1. `main.cpp` calls `simulator.step(0.1)`.
2. The simulator calculates distance from the car to the stationary obstacle at x=36.
3. The synthetic-data ML classifier calculates a risk score.
4. The simulator calculates stopping distance: `v*v/10 + 0.3*v + 3`.
5. The stop-request rule triggers when distance is within stopping distance plus 2m, distance is under 5m, or ML risk exceeds 0.85.
6. At the *first* stop request, we latch `stop_requested_` and call `speed_pid_.reset()`, so previous acceleration errors cannot keep pushing the car.
7. The target is now 0 m/s. The PID calculates a command; throttle is explicitly zero while stopping, and at least 35% brake is applied as long as speed is positive.
8. Acceleration comes from the original physics equation `a=3*throttle-6*brake-0.02*speed`.
9. Speed changes through `v_new=clamp(v_old+a*dt,0,14)` and position changes through `x += v_new*dt`. **There is no position teleport or safety-boundary override.** At sufficiently low speed, the discrete braking step reaches zero because the no-reverse velocity clamp limits speed to 0.
10. `natural_stop` becomes true when braking has brought speed to zero. `boundary_crossed` reports if the 2m clearance was violated, instead of hiding it.
11. Back in `main.cpp`, the program fails if the boundary was crossed and only succeeds if `natural_stop` is true.

## Numeric example
At the first time step, with distance 36m, speed 0, and low risk, target speed is 9m/s. PID yields 3.1635, so throttle is clamped to 1. Acceleration = 3m/s². After 0.1 seconds speed = 0.3m/s and x = 0.03m.

Later, with speed 5m/s and a latched stop request, the minimum brake 0.35 ensures acceleration is no greater than -2.2m/s² (brake = 0.35, throttle = 0). Speed decreases under the physics update; position remains an integrated value.

## Why this is a better test
The tests reject *any* crossing of x=34; they require the car to stop before x=34, require some actual braking, verify the car remains stopped for 20 more steps, check invalid dt, and check the PID reset behavior. This is still a one-dimensional educational simulator, not a verified road-vehicle safety system.

## Run
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

## Important limits
No physical car control, noisy sensors, road friction modeling, multi-vehicle scenarios, or safety certification. The classification model was trained on synthetic observations. In production, vehicle dynamics, emergency-braking authority, fault handling, and verification require much more rigorous design.
