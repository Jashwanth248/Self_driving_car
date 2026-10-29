#include "autodrive/simulation.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace autodrive {
Simulator::Simulator(Scenario s,RiskModel model)
    :scenario_(s),model_(std::move(model)),route_(default_route(s.cell_size_m)) {
    if(!(s.dt>0&&s.dt<=0.2)||!(s.max_speed>0)||!(s.cell_size_m>0)||!(s.obstacle_after_goal>s.desired_clearance)||s.max_steps<=0)
        throw std::invalid_argument("invalid scenario parameters");
    vehicle_.position=route_.points().front();
}
Telemetry Simulator::step() {
    if (mode_==DriveMode::Fault) throw std::runtime_error("simulation is in FAULT state");
    const double dt=scenario_.dt;
    progress_m_=std::max(progress_m_,route_.progress(vehicle_.position));
    const double remaining=std::max(0.0,route_.length()-progress_m_);
    const double to_obstacle=remaining+scenario_.obstacle_after_goal;
    const Observation obs{to_obstacle,vehicle_.speed,0.0};
    const double risk=model_.predict(obs);
    const double stopping_distance=vehicle_.speed*vehicle_.speed/(2.0*4.5)+0.3*vehicle_.speed+2.0;
    if(mode_==DriveMode::Cruise&&(to_obstacle<=stopping_distance+scenario_.desired_clearance+1.0||risk>0.85)) {
        mode_=DriveMode::Braking;
        speed_pid_.reset(); // old acceleration integral must not fight the stop request
    }
    const double target_speed=mode_==DriveMode::Cruise ? scenario_.max_speed : 0.0;
    double pid_command=speed_pid_.update(target_speed-vehicle_.speed,dt);
    Command cmd{std::clamp(pid_command,0.0,1.0),std::clamp(-pid_command,0.0,1.0),
                PurePursuit::steering(vehicle_,route_,progress_m_)};
    if(mode_!=DriveMode::Cruise) {
        cmd.throttle=0;
        cmd.brake=vehicle_.speed>0?std::max(cmd.brake,0.45):0.0;
    }
    // Educational kinematic bicycle model; not real vehicle motor, tire or brake dynamics.
    const double acceleration=3.0*cmd.throttle-6.0*cmd.brake-0.02*vehicle_.speed;
    vehicle_.speed=std::clamp(vehicle_.speed+acceleration*dt,0.0,14.0);
    vehicle_.heading+=vehicle_.speed/2.7*std::tan(cmd.steering)*dt;
    vehicle_.position.x+=vehicle_.speed*std::cos(vehicle_.heading)*dt;
    vehicle_.position.y+=vehicle_.speed*std::sin(vehicle_.heading)*dt;
    elapsed_+=dt;
    progress_m_=std::max(progress_m_,route_.progress(vehicle_.position));
    const double remaining_now=std::max(0.0,route_.length()-progress_m_);
    const auto& points=route_.points();
    const Point end=points.back(), before_end=points[points.size()-2];
    const double dx=end.x-before_end.x,dy=end.y-before_end.y;
    const double norm=std::hypot(dx,dy);
    const Point obstacle{end.x+scenario_.obstacle_after_goal*dx/norm, end.y+scenario_.obstacle_after_goal*dy/norm};
    const double obstacle_now=std::hypot(vehicle_.position.x-obstacle.x,vehicle_.position.y-obstacle.y);
    const bool boundary_crossed=obstacle_now<scenario_.desired_clearance-1e-8;
    if(boundary_crossed)mode_=DriveMode::Fault;
    else if(mode_==DriveMode::Braking&&vehicle_.speed==0)mode_=DriveMode::Stopped;
    return {elapsed_,vehicle_,target_speed,remaining_now,obstacle_now,risk,cmd,mode_,boundary_crossed};
}
}
