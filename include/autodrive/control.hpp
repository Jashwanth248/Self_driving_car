#pragma once
#include "autodrive/planning.hpp"
namespace autodrive {
class PID {
public:
    PID(double kp, double ki, double kd);
    double update(double error, double dt);
    void reset();
private:
    double kp_, ki_, kd_, integral_ = 0.0, previous_ = 0.0;
    bool first_ = true;
};
class PurePursuit {
public:
    static double steering(const VehicleState& vehicle, const Route& route, double progress);
};
}
