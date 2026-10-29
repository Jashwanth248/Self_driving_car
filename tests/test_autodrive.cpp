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
 Simulator sim;double maxx=0;bool stopped=false;
 for(int i=0;i<600;++i){
 auto s=sim.step(.1);
 assert(s.vehicle.position.x<=34.000001);
 assert(s.vehicle.speed>=0);
 maxx=std::max(maxx,s.vehicle.position.x);
 if(maxx>=33.99&&s.vehicle.speed==0){stopped=true;break;}
 }
 assert(stopped);
 std::cout<<"All simulator, planning, PID, and ML tests passed.\n";
}