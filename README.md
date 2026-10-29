# AutoDrive Lab

C++17 educational autonomous vehicle simulator with synthetic-data machine learning, A* path planning, PID speed control, automated tests, and clear explanations.

**Simulation only. Not for use on public roads or real vehicle hardware.**

## Build

```bash
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Run `./build/autodrive_demo` on Linux/macOS or `./build/Release/autodrive_demo.exe` on Windows (Visual Studio generator).

The project explains every subsystem in `docs/LEARNING_GUIDE.md`. The ML model is a small logistic regression trained on synthetic samples; it is **not** neural-network vision. The grid planner is a separate demonstration and is not connected to steering.
