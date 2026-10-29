#include "autodrive/simulation.hpp"
#include "autodrive/telemetry.hpp"
#include <iostream>
#include <stdexcept>
#include <string>
int main(int argc,char** argv) {
    try {
        std::string model="config/risk_model.txt", output="telemetry.csv";
        for(int i=1;i<argc;++i) {
            const std::string arg=argv[i];
            if(arg=="--model"&&i+1<argc)model=argv[++i];
            else if(arg=="--output"&&i+1<argc)output=argv[++i];
            else {std::cerr<<"Usage: autodrive_demo [--model config/risk_model.txt] [--output telemetry.csv]\n";return 2;}
        }
        autodrive::Scenario scenario;
        autodrive::Simulator sim(scenario,autodrive::RiskModel::load(model));
        autodrive::CsvTelemetry log(output);
        std::cout<<"AUTODRIVE LAB v2 | C++17 | A* + pure pursuit + PID + ML inference\n";
        std::cout<<"time(s)   x(m)   y(m)   speed(m/s)  remaining(m)  state\n";
        for(int i=0;i<scenario.max_steps;++i) {
            const auto data=sim.step();log.write(data);
            if(i%10==0)std::cout<<data.time<<"  "<<data.vehicle.position.x<<"  "<<data.vehicle.position.y
                <<"  "<<data.vehicle.speed<<"  "<<data.remaining_path<<"  "<<autodrive::mode_name(data.mode)<<'\n';
            if(data.mode==autodrive::DriveMode::Fault) {std::cerr<<"FAIL: safety boundary crossed\n";return 1;}
            if(data.mode==autodrive::DriveMode::Stopped) {
                std::cout<<"PASS: natural stop after "<<data.time<<"s, obstacle distance "<<data.distance_to_obstacle<<"m\n";
                std::cout<<"Telemetry saved: "<<output<<'\n';return 0;
            }
        }
        std::cerr<<"FAIL: timeout before natural stop\n";return 1;
    } catch(const std::exception& e) {std::cerr<<"ERROR: "<<e.what()<<'\n';return 2;}
}
