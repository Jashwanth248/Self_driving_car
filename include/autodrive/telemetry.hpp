#pragma once
#include "autodrive/types.hpp"
#include <fstream>
#include <string>
namespace autodrive {
class CsvTelemetry {
public:
    explicit CsvTelemetry(const std::string& filename);
    void write(const Telemetry& state);
private:
    std::ofstream stream_;
};
}
