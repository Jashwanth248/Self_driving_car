#pragma once
#include <string>
#include <vector>

namespace autodrive {
struct Point { double x = 0.0; double y = 0.0; };
struct VehicleState { Point position; double heading = 0.0; double speed = 0.0; };
struct Observation { double distance = 100.0; double closing_speed = 0.0; double lateral_offset = 0.0; };
struct Command { double throttle = 0.0; double brake = 0.0; double steering = 0.0; };
enum class DriveMode { Cruise, Braking, Stopped, Fault };
inline const char* mode_name(DriveMode m) {
    switch (m) {
        case DriveMode::Cruise: return "CRUISE";
        case DriveMode::Braking: return "BRAKING";
        case DriveMode::Stopped: return "STOPPED";
        default: return "FAULT";
    }
}
struct Telemetry {
    double time = 0.0;
    VehicleState vehicle;
    double target_speed = 0.0;
    double remaining_path = 0.0;
    double distance_to_obstacle = 0.0;
    double risk = 0.0;
    Command command;
    DriveMode mode = DriveMode::Cruise;
    bool boundary_crossed = false;
};
struct Scenario {
    double dt = 0.1;
    double max_speed = 6.0;
    double desired_clearance = 2.0;
    double obstacle_after_goal = 4.0;
    double cell_size_m = 4.0;
    int max_steps = 1000;
};
}
