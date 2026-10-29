#include "autodrive/control.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace autodrive {
PID::PID(double p,double i,double d):kp_(p),ki_(i),kd_(d) {}
double PID::update(double error,double dt) {
    if (!(dt>0.0)) throw std::invalid_argument("PID dt must be positive");
    integral_=std::clamp(integral_+error*dt,-20.0,20.0);
    const double derivative=first_?0.0:(error-previous_)/dt;
    first_=false;previous_=error;
    return kp_*error+ki_*integral_+kd_*derivative;
}
void PID::reset() {integral_=0;previous_=0;first_=true;}
double PurePursuit::steering(const VehicleState& v,const Route& route,double progress) {
    constexpr double wheelbase=2.7;
    const double lookahead=std::clamp(3.5+0.5*v.speed,3.5,7.0);
    const Point target=route.lookahead(progress,lookahead);
    const double alpha=std::atan2(target.y-v.position.y,target.x-v.position.x)-v.heading;
    const double delta=std::atan(2.0*wheelbase*std::sin(alpha)/lookahead);
    return std::clamp(delta,-0.55,0.55);
}
}
