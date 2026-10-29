#include "autodrive.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>
int main(){
 using namespace autodrive;
 auto path=Planner::route({".....",".###.","....."},{0,0},{4,2});
 assert(!path.empty()&&path.front().x==0&&path.back().x==4&&path.back().y==2);
 assert(Planner::route({".#.","###",".#."},{0,0},{2,2}).empty());
 bool threw=false;
 try{Planner::route({"..","."},{0,0},{1,0});}catch(const std::invalid_argument&){threw=true;}
 assert(threw);
 PID pid(1,0,0);
 assert(std::abs(pid.update(2,0.1)-2)<1e-9);
 RiskModel ml;ml.train();
 assert(ml.predict({3,12,0})>ml.predict({50,0,4}));
 Simulator sim;double maxx=0;bool stopped=false;bool ever_braked=false;
 for(int i=0;i<600;++i){
 auto s=sim.step(.1);
 assert(!s.boundary_crossed); // Detects a boundary crossing instead of concealing it.
 assert(s.vehicle.position.x<34.0); // Strictly before the 2m clearance limit.
 assert(s.vehicle.speed>=0);
 if(s.control.brake>0)ever_braked=true;
 maxx=std::max(maxx,s.vehicle.position.x);
 if(s.natural_stop){
   assert(s.vehicle.speed==0);stopped=true;break;
 }
 }
 assert(stopped&&ever_braked);
 assert(maxx>20.0&&maxx<34.0);
 // A latched stop must not allow the car to start moving again.
 double resting_x=sim.vehicle().position.x;
 for(int i=0;i<20;++i){
   auto s=sim.step(.1);
   assert(s.natural_stop&&!s.boundary_crossed);
   assert(s.vehicle.speed==0);
   assert(std::abs(s.vehicle.position.x-resting_x)<1e-9);
 }
 // Verify rejection of invalid simulation time steps.
 bool bad_dt=false;
 try{sim.step(0);}catch(const std::invalid_argument&){bad_dt=true;}
 assert(bad_dt);
 // Ensure PID reset actually clears its integral and derivative history.
 PID reset_pid(0,1,1);
 reset_pid.update(2,.1);
 reset_pid.reset();
 assert(std::abs(reset_pid.update(2,.1)-.2)<1e-9);
 std::cout<<"All simulator, planning, PID, and ML tests passed.\n";
}