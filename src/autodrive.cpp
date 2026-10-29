#include "autodrive.hpp"
#include <algorithm>
#include <cmath>
#include <queue>
#include <limits>
#include <stdexcept>
#include <utility>
namespace autodrive {
namespace {
double clamp(double v,double lo,double hi){return std::max(lo,std::min(hi,v));}
double sigmoid(double x){x=clamp(x,-40,40);return 1.0/(1.0+std::exp(-x));}
std::vector<double> features(const Observation& o){return {1.0,clamp(o.distance/30.0,0,4),clamp(o.closing_speed/12.0,0,3),clamp(std::abs(o.lateral_offset)/3.0,0,3)};}
}
void RiskModel::train(unsigned epochs,double rate){
 struct Sample { Observation o; double label; };
 std::vector<Sample> samples;
 for(int d=2;d<=60;d+=2) for(int closing=0;closing<=18;closing+=3)
 for(int lateral=0;lateral<=4;lateral+=2){
 bool hazardous=lateral<2&&(d<7||(closing>0&&static_cast<double>(d)/closing<2.5));
 samples.push_back({{static_cast<double>(d),static_cast<double>(closing),static_cast<double>(lateral)},hazardous?1.0:0.0});
 }
 for(unsigned epoch=0;epoch<epochs;++epoch){
 double grads[4]={};
 for(const auto& sample:samples){
 auto f=features(sample.o);double z=0;
 for(int j=0;j<4;++j)z+=weights_[j]*f[j];
 double e=sigmoid(z)-sample.label;
 for(int j=0;j<4;++j)grads[j]+=e*f[j];
 }
 for(int j=0;j<4;++j)weights_[j]-=rate*grads[j]/samples.size();
 }
}
double RiskModel::predict(const Observation& o) const {
 auto f=features(o);double z=0;
 for(int j=0;j<4;++j)z+=weights_[j]*f[j];
 return sigmoid(z);
}
std::vector<Point> Planner::route(const std::vector<std::string>& map,Point start,Point goal){
 if(map.empty()||map.front().empty())throw std::invalid_argument("empty map");
 int h=static_cast<int>(map.size()),w=static_cast<int>(map.front().size());
 for(const auto& row:map)if(static_cast<int>(row.size())!=w)throw std::invalid_argument("nonrectangular map");
 auto cell=[&](Point p){int x=static_cast<int>(p.x),y=static_cast<int>(p.y);return x>=0&&x<w&&y>=0&&y<h&&map[y][x]!='#';};
 if(!cell(start)||!cell(goal))throw std::invalid_argument("start or goal blocked");
 auto id=[&](int x,int y){return y*w+x;};
 int s=id(static_cast<int>(start.x),static_cast<int>(start.y)),g=id(static_cast<int>(goal.x),static_cast<int>(goal.y));
 std::vector<int> parent(w*h,-1),cost(w*h,std::numeric_limits<int>::max());
 using Node=std::pair<int,int>;
 std::priority_queue<Node,std::vector<Node>,std::greater<Node>> open;
 cost[s]=0;
 open.push({std::abs(s%w-g%w)+std::abs(s/w-g/w),s});
 while(!open.empty()){
 auto [score,u]=open.top();open.pop();
 int x=u%w,y=u/w;
 int heuristic=std::abs(x-g%w)+std::abs(y-g/w);
 if(score!=cost[u]+heuristic)continue;
 if(u==g)break;
 for(auto [dx,dy]:{std::pair<int,int>{1,0},{-1,0},{0,1},{0,-1}}){
 int nx=x+dx,ny=y+dy;
 if(!cell({double(nx),double(ny)}))continue;
 int v=id(nx,ny);
 if(cost[u]+1<cost[v]){
 cost[v]=cost[u]+1;parent[v]=u;
 open.push({cost[v]+std::abs(nx-g%w)+std::abs(ny-g/w),v});
 }
 }
 }
 if(cost[g]==std::numeric_limits<int>::max())return {};
 std::vector<Point> path;
 for(int at=g;at!=-1;at=parent[at])path.push_back({double(at%w),double(at/w)});
 std::reverse(path.begin(),path.end());return path;
}
PID::PID(double kp,double ki,double kd):kp_(kp),ki_(ki),kd_(kd){}
void PID::reset(){integral_=0;previous_=0;first_=true;}
double PID::update(double error,double dt){
 if(dt<=0)throw std::invalid_argument("dt must be positive");
 integral_=clamp(integral_+error*dt,-20,20);
 double derivative=first_?0:(error-previous_)/dt;
 first_=false;previous_=error;
 return kp_*error+ki_*integral_+kd_*derivative;
}
Simulator::Simulator():speed_pid_(0.35,0.015,0.03){
 car_.position={0,0};car_.speed=0;risk_.train();
}
StepResult Simulator::step(double dt){
 if(dt<=0||dt>0.2)throw std::invalid_argument("dt must be in (0, 0.2]");
 constexpr double obstacle_x=36.0;
 constexpr double desired_clearance=2.0;
 double distance=std::max(0.0,obstacle_x-car_.position.x);
 Observation reading{distance,car_.speed,0};
 double risk=risk_.predict(reading);
 double stopping_distance=car_.speed*car_.speed/(2*5.0)+car_.speed*0.3+3.0;
 // Latch a stop request: after deciding to stop, never reaccelerate.
 bool must_stop=distance<=stopping_distance+2.0||distance<5.0||risk>0.85;
 if(must_stop&&!stop_requested_){
   stop_requested_=true;
   speed_pid_.reset(); // Clear accumulated acceleration error before braking.
 }
 double target=stop_requested_?0.0:std::min(9.0,std::sqrt(std::max(0.0,2*3.0*(distance-4.0))));
 double command=speed_pid_.update(target-car_.speed,dt);
 Control action{clamp(command,0,1),clamp(-command,0,1),0};
 // Minimum braking deceleration prevents low-speed PID integral creep.
 if(stop_requested_){
   action.throttle=0;
   action.brake=car_.speed>0?std::max(action.brake,0.35):0;
 }
 double acceleration=3.0*action.throttle-6.0*action.brake-0.02*car_.speed;
 car_.speed=clamp(car_.speed+acceleration*dt,0,14);
 double movement=car_.speed*dt;
 car_.position.x+=movement; // No teleporting or forcing position/speed at the boundary.
 if(stop_requested_&&car_.speed==0)naturally_stopped_=true;
 clock_+=dt;
 bool crossed=car_.position.x>obstacle_x-desired_clearance+1e-9;
 return {car_,std::max(0.0,obstacle_x-car_.position.x),risk,action,naturally_stopped_,crossed};
}
std::vector<Point> Simulator::planned_route() const {
 return Planner::route({"...........","..###......","...........","......###..","..........."},{0,0},{10,4});
}
}