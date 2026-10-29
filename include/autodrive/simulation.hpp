#pragma once
#include "autodrive/control.hpp"
#include "autodrive/perception.hpp"
namespace autodrive {
class Simulator {
public:
    Simulator(Scenario scenario, RiskModel model);
    Telemetry step();
    const Route& route() const { return route_; }
    const VehicleState& vehicle() const { return vehicle_; }
    DriveMode mode() const { return mode_; }
private:
    Scenario scenario_;
    RiskModel model_;
    Route route_;
    PID speed_pid_{0.35, 0.015, 0.03};
    VehicleState vehicle_;
    DriveMode mode_ = DriveMode::Cruise;
    double progress_m_ = 0.0;
    double elapsed_ = 0.0;
};
}
