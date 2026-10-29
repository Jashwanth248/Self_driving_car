#pragma once
#include "autodrive/types.hpp"
#include <array>
#include <string>
namespace autodrive {
class RiskModel {
public:
    static RiskModel train(unsigned epochs = 2500, double learning_rate = 0.08);
    static RiskModel load(const std::string& filename);
    void save(const std::string& filename) const;
    double predict(const Observation& observation) const;
    const std::array<double, 4>& weights() const { return weights_; }
private:
    std::array<double, 4> weights_{};
};
}
