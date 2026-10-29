#include "autodrive.hpp"
#include <iostream>
#include <iomanip>
int main(){
 autodrive::Simulator simulator;
 auto path=simulator.planned_route();
 std::cout<<"AUTODRIVE LAB | C++17 | synthetic ML risk + A* + PID\n";
 std::cout<<"A* route: "<<path.size()<<" cells\n";
 std::cout<<"time(s),position(m),speed(m/s),obstacle(m),ML_risk,throttle,brake\n";
 std::cout<<std::fixed<<std::setprecision(2);
 for(int tick=0;tick<600;++tick){
 auto s=simulator.step(0.1);
 if(tick%10==0)std::cout<<tick*0.1<<','<<s.vehicle.position.x<<','<<s.vehicle.speed<<','<<s.distance_to_obstacle<<','<<s.risk<<','<<s.control.throttle<<','<<s.control.brake<<'\n';
 if(s.boundary_crossed){std::cerr<<"FAIL: crossed stopping boundary\n";return 1;}
 if(s.natural_stop){
 std::cout<<"PASS: natural braking stop before obstacle at x="<<s.vehicle.position.x<<"m\n";return 0;
 }
 }
 std::cerr<<"FAIL: vehicle did not reach designated stopping position\n";
 return 1;
}