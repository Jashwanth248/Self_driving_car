#include "autodrive/simulation.hpp"
#include "autodrive/telemetry.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
namespace {
void require(bool ok, const char* message){ if(!ok)throw std::runtime_error(message); }
void test_astar(){
    auto grid=std::vector<std::string>{".....",".###.","....."};
    auto path=autodrive::AStar::solve(grid,{0,0},{4,2});
    require(!path.empty()&&path.front().x==0&&path.back().x==4&&path.back().y==2,"A* endpoints");
    for(size_t i=1;i<path.size();++i) {
        const auto a=path[i-1],b=path[i];
        require(std::abs(b.x-a.x)+std::abs(b.y-a.y)==1,"A* path must be adjacent");
        require(grid[static_cast<size_t>(b.y)][static_cast<size_t>(b.x)]!='#',"A* cannot go through obstacles");
    }
    require(autodrive::AStar::solve({".#.","###",".#."},{0,0},{2,2}).empty(),"blocked path");
    bool threw=false;try{autodrive::AStar::solve({"..","."},{0,0},{1,0});}catch(const std::invalid_argument&){threw=true;}
    require(threw,"nonrectangular grid validation");
}
void test_pid(){
    autodrive::PID pid(1,0,0);require(std::abs(pid.update(2,.1)-2)<1e-10,"PID proportional");
    autodrive::PID reset(0,1,1);reset.update(2,.1);reset.reset();
    require(std::abs(reset.update(2,.1)-.2)<1e-10,"PID reset");
    bool threw=false;try{pid.update(2,0);}catch(const std::invalid_argument&){threw=true;}
    require(threw,"PID rejects zero dt");
}
void test_model(){
    auto model=autodrive::RiskModel::train();
    require(model.predict({3,12,0})>model.predict({50,0,4}),"ML risk ordering");
    bool threw=false;try{model.predict({NAN,0,0});}catch(const std::invalid_argument&){threw=true;}
    require(threw,"ML rejects NaN");
}
void test_scenario(double max_speed){
    autodrive::Scenario scenario;scenario.max_speed=max_speed;
    autodrive::Simulator sim(scenario,autodrive::RiskModel::train());
    bool braked=false, stopped=false, steered=false;double prev_time=0,max_error=0;
    for(int i=0;i<scenario.max_steps;++i){
        const auto s=sim.step();
        require(s.time>prev_time,"clock must advance");prev_time=s.time;
        require(s.vehicle.speed>=0&&s.vehicle.speed<=14.0,"speed bounds");
        require(!s.boundary_crossed,"obstacle clearance violated");
        require(s.mode!=autodrive::DriveMode::Fault,"unexpected fault");
        if(s.command.brake>0)braked=true;
        if(std::abs(s.command.steering)>0.01)steered=true;
        const double progress=sim.route().progress(s.vehicle.position);
        const auto on_path=sim.route().lookahead(progress,0);
        max_error=std::max(max_error,std::hypot(s.vehicle.position.x-on_path.x,s.vehicle.position.y-on_path.y));
        if(s.mode==autodrive::DriveMode::Stopped){
            require(s.vehicle.speed==0,"stopped state must have zero speed");
            require(s.distance_to_obstacle>=scenario.desired_clearance,"stop clearance");
            stopped=true;break;
        }
    }
    require(stopped&&braked&&steered,"must naturally stop after steering and braking");
    require(max_error<6.0,"route tracking error too large");
    const auto position=sim.vehicle().position;
    for(int i=0;i<30;++i){
        const auto s=sim.step();
        require(s.mode==autodrive::DriveMode::Stopped,"stop mode must latch");
        require(s.vehicle.speed==0,"car cannot restart after stop");
        require(std::hypot(s.vehicle.position.x-position.x,s.vehicle.position.y-position.y)<1e-8,"car position must not jump");
    }
}
}
int main(){
    try {
        test_astar();test_pid();test_model();
        for(double v:{3.0,4.5,6.0,7.5})test_scenario(v);
        std::cout<<"PASS: A*, PID, ML, 4 driving scenarios, natural stop, clearance and hold tests\n";
        return 0;
    }catch(const std::exception& e){std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}
}
