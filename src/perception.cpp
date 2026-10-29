#include "autodrive/perception.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <vector>
namespace autodrive {
namespace {
double sigmoid(double z) {return 1.0/(1.0+std::exp(-std::clamp(z,-40.0,40.0)));}
std::array<double,4> features(Observation o) {return {1.0,std::clamp(o.distance/30,0.0,4.0),std::clamp(o.closing_speed/12,0.0,3.0),std::clamp(std::abs(o.lateral_offset)/3,0.0,3.0)};}
}
RiskModel RiskModel::train(unsigned epochs,double lr) {
    if (!(lr>0&&lr<=1)) throw std::invalid_argument("invalid learning rate");
    struct Sample { Observation input;double label; };
    std::vector<Sample> samples;
    for (int d=2;d<=60;d+=2) for(int speed=0;speed<=18;speed+=3) for(int side=0;side<=4;side+=2) {
        const bool unsafe=side<2&&(d<7||(speed>0&&double(d)/speed<2.5));
        samples.push_back({{double(d),double(speed),double(side)},unsafe?1.0:0.0});
    }
    RiskModel result;
    for(unsigned epoch=0;epoch<epochs;++epoch) {
        std::array<double,4> gradient{};
        for(const auto& sample:samples) {
            const auto f=features(sample.input);
            double z=0;for(int j=0;j<4;++j)z+=result.weights_[j]*f[j];
            const double error=sigmoid(z)-sample.label;
            for(int j=0;j<4;++j)gradient[j]+=error*f[j];
        }
        for(int j=0;j<4;++j)result.weights_[j]-=lr*gradient[j]/static_cast<double>(samples.size());
    }
    return result;
}
RiskModel RiskModel::load(const std::string& filename) {
    std::ifstream file(filename);if(!file)throw std::runtime_error("cannot open model: "+filename);
    RiskModel m;
    for(auto& weight:m.weights_) if(!(file>>weight)||!std::isfinite(weight))throw std::runtime_error("invalid model file: "+filename);
    double extra=0;if(file>>extra)throw std::runtime_error("model contains unexpected values");
    return m;
}
void RiskModel::save(const std::string& filename) const {
    std::ofstream out(filename);if(!out)throw std::runtime_error("cannot write model: "+filename);
    out.precision(17);for(auto weight:weights_)out<<weight<<'\n';
}
double RiskModel::predict(const Observation& o) const {
    if (!std::isfinite(o.distance)||!std::isfinite(o.closing_speed)||!std::isfinite(o.lateral_offset))
        throw std::invalid_argument("non-finite observation");
    const auto f=features(o);
    double z=0;for(int j=0;j<4;++j)z+=weights_[j]*f[j];
    return sigmoid(z);
}
}
