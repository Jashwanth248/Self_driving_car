#include "autodrive/telemetry.hpp"
#include <iomanip>
#include <stdexcept>
namespace autodrive {
CsvTelemetry::CsvTelemetry(const std::string& name):stream_(name) {
    if(!stream_)throw std::runtime_error("cannot write telemetry to "+name);
    stream_<<"time_s,x_m,y_m,heading_rad,speed_mps,target_mps,remaining_path_m,obstacle_m,risk,throttle,brake,steering_rad,mode,boundary_crossed\n";
    stream_<<std::fixed<<std::setprecision(6);
}
void CsvTelemetry::write(const Telemetry& t) {
    stream_<<t.time<<','<<t.vehicle.position.x<<','<<t.vehicle.position.y<<','<<t.vehicle.heading<<','
           <<t.vehicle.speed<<','<<t.target_speed<<','<<t.remaining_path<<','<<t.distance_to_obstacle<<','
           <<t.risk<<','<<t.command.throttle<<','<<t.command.brake<<','<<t.command.steering<<','
           <<mode_name(t.mode)<<','<<(t.boundary_crossed?1:0)<<'\n';
}
}
