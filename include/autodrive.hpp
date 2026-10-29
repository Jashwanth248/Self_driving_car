#pragma once
#include <vector>
#include <string>
namespace autodrive {
struct Point { double x=0,y=0; };
struct Vehicle { Point position; double heading=0, speed=0; };
struct Observation { double distance=100, closing_speed=0, lateral_offset=0; };
struct Control { double throttle=0, brake=0, steering=0; };
struct StepResult { Vehicle vehicle; double distance_to_obstacle=0, risk=0; Control control; };
class RiskModel {
public:
 void train(unsigned epochs=2500,double rate=0.08);
 double predict(const Observation& observation) const;
private: double weights_[4]={0,0,0,0};
};
class Planner {
public: static std::vector<Point> route(const std::vector<std::string>& map,Point start,Point goal);
};
class PID {
public: PID(double kp,double ki,double kd); double update(double error,double dt);
private: double kp_,ki_,kd_,integral_=0,previous_=0; bool first_=true;
};
class Simulator {
public:
 Simulator(); StepResult step(double dt);
 const Vehicle& vehicle() const {return car_;}
 std::vector<Point> planned_route() const;
private:
 Vehicle car_; RiskModel risk_; PID speed_pid_; double clock_=0;
};
}